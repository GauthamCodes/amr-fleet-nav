"""FleetMapNode: composite every robot's SLAM map into one map that drives planning.

WHAT MAKES THIS LOAD-BEARING RATHER THAN A VIEWER

    ``/fleet_map`` is the static layer of BOTH robots' global costmaps, and both
    global costmaps run in the ``fleet_map`` frame. So a corridor amr2 maps is a
    corridor amr1 can immediately plan through. A merged map that only existed for
    RViz would satisfy the wording of the requirement and none of its point.

THE THREE THINGS THAT MUST HAPPEN IN __init__, BEFORE NAV2 EXISTS

    1. Both static transforms. ``Costmap2DROS::on_activate`` BLOCKS waiting for
       ``fleet_map -> amrN/base_footprint``, and the lifecycle manager's
       ``change_state`` call has no timeout, so a missing transform does not produce
       an error - it produces a lifecycle manager that prints "Activating
       planner_server" and then nothing at all, until ``initial_transform_timeout``
       expires 60 s later and the whole bringup aborts with no line naming
       ``fleet_map``.

    2. A fixed-extent grid, published immediately, all-unknown. ``Costmap2DROS``
       declares its own ``width``/``height``/``origin_*`` defaulting to 5 x 5 m at
       (0, 0); both robots spawn near (-11, +/-1.5), OUTSIDE it. A costmap that sizes
       itself from whatever has been explored so far starts wrong, and then resizes
       every time either robot finds a new corridor, wiping the global obstacle layer
       each time.

    3. TRANSIENT_LOCAL, RELIABLE QoS on the publisher. Nav2's StaticLayer subscribes
       transient-local; a default VOLATILE publisher never matches it, and neither
       end logs anything. The same profile on the SUBSCRIBER side is not optional
       either: slam_toolbox gates map publication on ``get_subscription_count()``,
       which counts MATCHED subscriptions, so an incompatible subscriber gets zero
       maps, zero warnings, and a slam_toolbox that looks perfectly healthy.

THE INTER-MAP TRANSFORM IS FIXED, AND THAT IS A DOCUMENTED LIMITATION

    ``fleet_map -> amrN/map`` is computed once from each robot's spawn pose in
    fleet.yaml and never corrected. Two independently built maps will disagree by
    roughly the sum of their pose errors - Phase 1 measured mean 0.036 m, p95 0.075 m
    - so shared walls thicken by one to three cells under the max-wins merge. Against
    a 5.5 m aisle and a 0.70 m inflation radius that is harmless, and it is reported
    rather than assumed: see the drift figures in the decision report.

    Correlation-based correction was on PLAN.md section 5's cut list and is cut. Note
    for whoever picks it up: rclpy's StaticTransformBroadcaster is APPEND-ONLY - it
    skips a transform whose ``child_frame_id`` it has already sent, unlike the C++
    one, which replaces it. Republishing a corrected transform through it silently
    does nothing. A raw TRANSIENT_LOCAL publisher on /tf_static, resending the whole
    set, is the way round it.
"""

import math
import os
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster

from amr_description.fleet_config import load_fleet, spawn_pose
from amr_fleet_control.fleet_grid import (
    cell_offset,
    composite,
    coverage,
    FLEET_FRAME,
    fleet_grid_spec,
    FLEET_MAP_TOPIC,
    FLEET_MARGIN,
    FLEET_RESOLUTION,
    occupied_mask,
    overlap_slices,
    WAREHOUSE_EXTENT,
)
from amr_fleet_control.selective_policy import bump_visits, DEFAULTS, score_update


def map_qos():
    """Return the QoS every map endpoint in this system must use.

    RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1), matching Nav2's StaticLayer under
    ``map_subscribe_transient_local: true`` and slam_toolbox's own publisher. See the
    module docstring for what silently breaks otherwise.
    """
    profile = QoSProfile(depth=1)
    profile.reliability = ReliabilityPolicy.RELIABLE
    profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return profile


class FleetMapNode(Node):
    """Composite per-robot SLAM maps into ``/fleet_map`` under a scored update policy.

    Loops over the typed robot list and never names a robot, so a third entry in
    fleet.yaml is composited with no code change (ENGINEERING_NOTES rule 5).
    """

    def __init__(self):
        """Publish the static frames and an empty fleet map, then start listening."""
        super().__init__("fleet_map_node")

        x_min, x_max, y_min, y_max = WAREHOUSE_EXTENT
        self.declare_parameter("resolution", FLEET_RESOLUTION)
        self.declare_parameter("x_min", x_min)
        self.declare_parameter("x_max", x_max)
        self.declare_parameter("y_min", y_min)
        self.declare_parameter("y_max", y_max)
        self.declare_parameter("margin", FLEET_MARGIN)
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("decision_log", "")
        for key, value in DEFAULTS.items():
            self.declare_parameter(f"policy.{key}", value)

        self.policy = {
            key: float(self.get_parameter(f"policy.{key}").value) for key in DEFAULTS
        }
        self.decision_log = self.get_parameter("decision_log").value

        self.grid = fleet_grid_spec(
            self.get_parameter("x_min").value,
            self.get_parameter("x_max").value,
            self.get_parameter("y_min").value,
            self.get_parameter("y_max").value,
            self.get_parameter("resolution").value,
            margin=self.get_parameter("margin").value,
        )
        self.get_logger().info(f"fleet grid: {self.grid}")

        self.fleet = load_fleet()
        self.state = {
            robot["name"]: {
                "projected": self.grid.empty(),
                "visits": np.zeros(self.grid.shape, dtype=np.int32),
                "first": True,
                "last_accept_s": None,
                "accepted": 0,
                "deferred": 0,
            }
            for robot in self.fleet
        }

        # ---- 1. Static frames, before anything can wait on them. ----------------
        self.tf_static = StaticTransformBroadcaster(self)
        self._publish_fleet_transforms()

        # ---- 2. A valid, correctly sized map, before Nav2 asks for one. ---------
        self.publisher = self.create_publisher(
            OccupancyGrid, FLEET_MAP_TOPIC, map_qos()
        )
        self.diagnostics = self.create_publisher(
            DiagnosticArray, "fleet_map/diagnostics", 10
        )
        self.merged = self.grid.empty()
        self.dirty = True
        self.composites = 0
        self.composite_ms = []
        self.decisions = []
        self._publish_map()

        # ---- 3. Only now does anything need to arrive. --------------------------
        for robot in self.fleet:
            name = robot["name"]
            self.create_subscription(
                OccupancyGrid,
                f"/{name}/map",
                lambda msg, robot_name=name: self._on_map(robot_name, msg),
                map_qos(),
            )

        period = float(self.get_parameter("publish_period_s").value)
        self.create_timer(period, self._on_publish_tick)
        self.create_timer(1.0, self._publish_diagnostics)

    # ------------------------------------------------------------------ frames --

    def _publish_fleet_transforms(self):
        """Broadcast ``fleet_map -> amrN/map`` for every robot, from its spawn pose.

        slam_toolbox anchors a robot's map frame on wherever that robot started, so
        the spawn pose IS the inter-map transform. z is deliberately 0 rather than
        fleet.yaml's spawn z of 0.15: that value drops the robot onto the floor at
        spawn time, while ``amrN/odom`` sits at ground level. Carrying it into the
        static transform would lift every fleet-frame pose 0.15 m into the air -
        invisible in a 2D costmap, wrong in RViz and wrong in any drift metric.
        """
        transforms = []
        for robot in self.fleet:
            x, y, _, yaw = spawn_pose(robot)
            if abs(yaw) > 1e-9:
                raise ValueError(
                    f"{robot['name']} spawns at yaw {yaw}; compositing assumes a "
                    "shared lattice and cannot rotate a source grid"
                )
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = FLEET_FRAME
            transform.child_frame_id = f"{robot['name']}/map"
            transform.transform.translation.x = float(x)
            transform.transform.translation.y = float(y)
            transform.transform.translation.z = 0.0
            transform.transform.rotation.z = math.sin(yaw / 2.0)
            transform.transform.rotation.w = math.cos(yaw / 2.0)
            transforms.append(transform)

        # One call with the whole set: the rclpy broadcaster accumulates into a single
        # latched TFMessage, and sending them separately works only by accident.
        self.tf_static.sendTransform(transforms)
        self.get_logger().info(
            f"published {len(transforms)} static {FLEET_FRAME} -> <robot>/map frames"
        )

    # -------------------------------------------------------------------- maps --

    def _on_map(self, robot_name, msg):
        """Score one robot's candidate map and merge it if the policy accepts."""
        state = self.state[robot_name]
        robot = {r["name"]: r for r in self.fleet}[robot_name]
        spawn_x, spawn_y, _, _ = spawn_pose(robot)

        if msg.info.width == 0 or msg.info.height == 0:
            return

        source = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        try:
            col, row = cell_offset(
                self.grid,
                spawn_x + msg.info.origin.position.x,
                spawn_y + msg.info.origin.position.y,
                msg.info.resolution,
            )
        except ValueError as error:
            self.get_logger().error(
                f"{robot_name}: cannot place map in the fleet grid: {error}",
                throttle_duration_sec=10.0,
            )
            return

        windows = overlap_slices(self.grid, source.shape, col, row)
        if windows is None:
            self.get_logger().warn(
                f"{robot_name}: map lies entirely outside the fleet grid "
                f"({self.grid}); check the extent parameters",
                throttle_duration_sec=10.0,
            )
            return
        fleet_slices, source_slices = windows

        candidate = source[source_slices]
        previous = state["projected"][fleet_slices]
        visits = state["visits"][fleet_slices]

        now = self._now_s()
        elapsed = (
            0.0 if state["last_accept_s"] is None else now - state["last_accept_s"]
        )
        decision = score_update(
            previous,
            candidate,
            visits,
            elapsed_s=elapsed,
            config=self.policy,
            first=state["first"],
        )

        if decision.accepted:
            bump_visits(visits, previous, candidate)
            known = candidate >= 0
            np.copyto(previous, candidate, where=known)
            state["first"] = False
            state["last_accept_s"] = now
            state["accepted"] += 1
            self.dirty = True
        else:
            state["deferred"] += 1

        self._record(robot_name, now, decision, elapsed)

    def _on_publish_tick(self):
        """Recomposite and republish, but only when an update was actually accepted.

        This is where the selective policy's saving is realised: a deferred candidate
        costs one score and nothing else - no composite, no serialisation, no message
        on the wire to two global costmaps.
        """
        if not self.dirty:
            return
        started = time.perf_counter()
        self.merged = composite(
            self.grid, [state["projected"] for state in self.state.values()]
        )
        self.composite_ms.append((time.perf_counter() - started) * 1000.0)
        self.composites += 1
        self.dirty = False
        self._publish_map()

    def _publish_map(self):
        """Publish the current fleet map."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FLEET_FRAME
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.merged.reshape(-1).tolist()
        self.publisher.publish(msg)

    # --------------------------------------------------------------- evidence --

    def _now_s(self):
        """Return the current time in seconds on whichever clock is in use."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _record(self, robot_name, now, decision, elapsed):
        """Append one decision to the log and rewrite the report.

        The report is rewritten on every decision rather than at shutdown. These runs
        are terminated by a mission node emitting Shutdown, and evidence that only
        exists if the process exits cleanly is evidence that goes missing.
        """
        self.decisions.append(
            {
                "t": now,
                "robot": robot_name,
                "frontier": decision.frontier,
                "change": decision.change,
                "recency": decision.recency,
                "revisit": decision.revisit,
                "score": decision.score,
                "new_cells": decision.new_cells,
                "changed_cells": decision.changed_cells,
                "elapsed_s": elapsed,
                "accepted": decision.accepted,
            }
        )
        self.get_logger().info(f"{robot_name}: {decision}")
        if self.decision_log:
            self._write_report()

    def _write_report(self):
        """Write the accept/defer CSV and its markdown summary."""
        stem = self.decision_log
        os.makedirs(os.path.dirname(stem) or ".", exist_ok=True)

        with open(f"{stem}.csv", "w", encoding="utf-8") as handle:
            handle.write(
                "t,robot,frontier,change,recency,revisit,score,new_cells,"
                "changed_cells,elapsed_s,decision\n"
            )
            for row in self.decisions:
                handle.write(
                    f"{row['t']:.3f},{row['robot']},{row['frontier']:.4f},"
                    f"{row['change']:.4f},{row['recency']:.4f},{row['revisit']:.4f},"
                    f"{row['score']:.4f},{row['new_cells']},{row['changed_cells']},"
                    f"{row['elapsed_s']:.3f},"
                    f"{'ACCEPT' if row['accepted'] else 'DEFER'}\n"
                )

        lines = []
        add = lines.append
        rule = "=" * 78
        thin = "-" * 78

        total = len(self.decisions)
        accepted = sum(1 for row in self.decisions if row["accepted"])
        deferred = total - accepted
        mean_ms = float(np.mean(self.composite_ms)) if self.composite_ms else 0.0

        add(rule)
        add("PHASE 3 - selective fleet-map update policy")
        add(rule)
        add(f"fleet grid:               {self.grid}")
        add(f"robots:                   {', '.join(self.state)}")
        add(thin)
        add("[A] POLICY")
        add("    score = w_f*frontier + w_c*change + w_r*recency - w_v*revisit")
        for key in sorted(self.policy):
            add(f"    {key + ':':<28} {self.policy[key]:8.3f}")
        add(thin)
        add("[B] DECISIONS")
        add(f"    {'candidates scored:':<40} {total:8d}")
        add(f"    {'accepted (merged and published):':<40} {accepted:8d}")
        add(f"    {'deferred:':<40} {deferred:8d}")
        if total:
            add(f"    {'deferred share:':<40} {100.0 * deferred / total:7.1f} %")
        add("")
        add("      robot        accepted   deferred   deferred %")
        for name, state in self.state.items():
            robot_total = state["accepted"] + state["deferred"]
            share = 100.0 * state["deferred"] / robot_total if robot_total else 0.0
            add(
                f"      {name:<12} {state['accepted']:8d}   {state['deferred']:8d}"
                f"   {share:9.1f}"
            )
        add(thin)
        add("[C] WHAT DEFERRAL SAVES")
        add(f"    {'composites performed:':<40} {self.composites:8d}")
        add(f"    {'mean composite + publish (ms):':<40} {mean_ms:8.2f}")
        add(f"    {'estimated work avoided (ms):':<40} " f"{deferred * mean_ms:8.1f}")
        add("")
        add("    ESTIMATED, and stated as such: it is the deferred count times the")
        add("    measured mean cost of the composite and publish those deferrals")
        add("    skipped. Scoring still runs on every candidate; that cost is not")
        add("    saved and is not claimed.")
        add("")
        add("    The absolute figure is small, and it is worth saying why rather")
        add("    than leaving it to look like a result. Compositing is numpy slice")
        add("    arithmetic on a fixed grid, so it was never the expensive part.")
        add("    What deferral actually bounds is how often a 680x400 grid is")
        add("    serialised to TWO global costmaps that each reprocess it - work")
        add("    that happens in the Nav2 processes and is not measured here. The")
        add("    deferred SHARE above is the honest headline; the milliseconds are")
        add("    a lower bound on one end of the saving.")
        add(thin)
        add("[D] FLEET MAP")
        add(f"    {'known cells:':<40} {100.0 * coverage(self.merged):7.1f} %")
        add(
            f"    {'occupied cells:':<40} "
            f"{int(np.count_nonzero(occupied_mask(self.merged))):8d}"
        )
        add(rule)

        text = "\n".join(lines)
        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write("# Phase 3 - selective fleet-map update policy\n\n")
            handle.write("```\n")
            handle.write(text)
            handle.write("\n```\n")

    def _publish_diagnostics(self):
        """Publish counters so a probe can watch the policy without parsing logs."""
        status = DiagnosticStatus()
        status.name = "fleet_map"
        status.level = DiagnosticStatus.OK
        status.message = f"{self.composites} composites"
        values = {
            "composites": float(self.composites),
            "coverage": coverage(self.merged),
            "decisions": float(len(self.decisions)),
        }
        for name, state in self.state.items():
            values[f"{name}.accepted"] = float(state["accepted"])
            values[f"{name}.deferred"] = float(state["deferred"])
        status.values = [KeyValue(key=k, value=f"{v:.4f}") for k, v in values.items()]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics.publish(array)


def main(args=None):
    """Spin the fleet map node."""
    rclpy.init(args=args)
    node = FleetMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
