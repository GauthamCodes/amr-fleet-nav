#!/usr/bin/env python3
"""Send goals to every robot in the fleet AT THE SAME TIME, and record what happened.

Two robots navigating one after the other proves nothing that Phase 1 did not already
prove twice. The point of dispatching concurrently is that both stacks then compete
for the same simulator, the same /clock, the same TF tree and the same fleet map, and
each robot's obstacle layer sees the other one as an obstacle. What this run measures
is whether both still reach their goals under that, and how close they came to each
other on the way.

GOALS ARE IN THE FLEET FRAME, WHICH IS THE WORLD FRAME

    Phase 1 and Phase 2 mission scripts author goals in world coordinates and convert
    them into the robot's own map frame, because slam_toolbox anchors that frame on
    wherever the robot spawned. From Phase 3 the global costmap plans in ``fleet_map``,
    which is defined to coincide with the Gazebo world origin, so that conversion is
    gone: a world coordinate IS a fleet coordinate. One goal list, no per-robot
    arithmetic, which is also what makes the goals readable next to world.yaml.

WHAT IS NOT CLAIMED

    Inter-robot distance here is a CLEARANCE measurement between two footprint
    origins, not a collision check, and the two robots are on separate deconflicted
    routes rather than a forced conflict. The narrow-intersection conflict, mutual
    local deviation and the yield protocol are Phases 6 and 7; this run establishes
    that two independent stacks coexist, which is the prerequisite for them.
"""

import math
import os

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from amr_description.fleet_config import load_fleet
from amr_fleet_control.fleet_grid import FLEET_FRAME, RAMP_REGION

#: A plan counts as a genuine replan when it deviates from the previous one by more
#: than this. Nav2's tree recomputes at about 1 Hz whether or not anything moved, so
#: counting messages would measure the tree's tick rate. Same threshold as Phase 1's
#: nav_goal_run, deliberately, so the numbers are comparable.
REPLAN_DEVIATION = 0.30


def yaw_of(quaternion):
    """Return the yaw encoded in a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def quaternion_from_yaw(yaw):
    """Return ``(z, w)`` for a planar rotation."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def max_deviation(path_a, path_b):
    """Return the largest distance from any point of path_a to the nearest of path_b.

    Defined locally rather than imported from ``amr_navigation.clearance``: this
    package is a dependency OF amr_navigation, which imports the fleet frame from it,
    and reaching back the other way would make the two packages circular.
    """
    if not path_a or not path_b:
        return float("inf")
    worst = 0.0
    for ax, ay in path_a:
        nearest = min(math.hypot(ax - bx, ay - by) for bx, by in path_b)
        worst = max(worst, nearest)
    return worst


class RobotRun:
    """Everything recorded about one robot's goal within a concurrent fleet run."""

    def __init__(self, name, goal):
        """Start an empty record for one robot and the goal it was given."""
        self.name = name
        self.goal = goal
        self.status = "PENDING"
        self.dispatched_at = None
        self.finished_at = None
        self.truth = None
        self.start_truth = None
        self.travelled = 0.0
        self.plans = []
        self.replans = 0
        self.client = None

    @property
    def duration(self):
        """Return seconds from dispatch to result, or None if it never finished."""
        if self.dispatched_at is None or self.finished_at is None:
            return None
        return self.finished_at - self.dispatched_at

    @property
    def final_error(self):
        """Return the distance from the final ground-truth pose to the goal."""
        if self.truth is None:
            return None
        return math.hypot(self.truth[0] - self.goal[0], self.truth[1] - self.goal[1])


class FleetMission(Node):
    """Dispatch one goal per robot simultaneously and report the fleet's behaviour."""

    def __init__(self):
        """Build a record per fleet member, wire up its action client and inputs."""
        super().__init__("fleet_mission")

        self.declare_parameter("goals", [0.0])
        self.declare_parameter("results_dir", os.path.join(os.getcwd(), "results"))
        self.declare_parameter("tag", "concurrent_goals")
        self.declare_parameter("start_delay_s", 5.0)
        self.declare_parameter("timeout_s", 180.0)
        self.declare_parameter("linger_s", 3.0)

        self.results_dir = self.get_parameter("results_dir").value
        self.tag = self.get_parameter("tag").value
        self.start_delay = float(self.get_parameter("start_delay_s").value)
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.linger = float(self.get_parameter("linger_s").value)

        fleet = load_fleet()
        goals = self._goals_for(fleet)

        self.runs = {}
        for robot in fleet:
            name = robot["name"]
            run = RobotRun(name, goals[name])
            run.client = ActionClient(self, NavigateToPose, f"/{name}/navigate_to_pose")
            self.create_subscription(
                Path, f"/{name}/plan", lambda msg, n=name: self._on_plan(n, msg), 10
            )
            self.runs[name] = run

        # Both robots' bridges remap their model pose onto this one topic, so it
        # carries an interleaved stream and every consumer filters by child_frame_id.
        self.create_subscription(TFMessage, "/gz_ground_truth", self._on_truth, 40)

        # The global costmap, read back to check two things at once: that the
        # obstacle and static layers are still populating it, and that the ramp
        # filter's all-zero Phase 3 mask adds nothing. Nav2 publishes this
        # transient-local, same as the fleet map.
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmaps = {}
        for robot in fleet:
            name = robot["name"]
            self.create_subscription(
                OccupancyGrid,
                f"/{name}/global_costmap/costmap",
                lambda msg, n=name: self._on_costmap(n, msg),
                costmap_qos,
            )

        self.separations = []
        self.started = self.now_s()
        self.state = "waiting"
        self.finished_at = None
        self.create_timer(0.1, self._tick)

    def _goals_for(self, fleet):
        """Return ``{robot_name: (x, y, yaw)}`` from the flat goals parameter.

        The parameter is a flat list of triples rather than a per-robot structure
        because ROS parameters have no nested arrays. It is paired with fleet.yaml's
        declaration order, so this file still names no robot.
        """
        flat = list(self.get_parameter("goals").value)
        if len(flat) != 3 * len(fleet):
            raise ValueError(
                f"'goals' has {len(flat)} values; {len(fleet)} robots need "
                f"{3 * len(fleet)} (x, y, yaw per robot, in fleet.yaml order)"
            )
        return {
            robot["name"]: tuple(float(v) for v in flat[3 * i : 3 * i + 3])
            for i, robot in enumerate(fleet)
        }

    def now_s(self):
        """Return the current simulation time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------- inputs

    def _on_truth(self, msg):
        for transform in msg.transforms:
            run = self.runs.get(transform.child_frame_id)
            if run is None:
                continue
            t = transform.transform.translation
            pose = (t.x, t.y, yaw_of(transform.transform.rotation))
            if run.truth is not None:
                run.travelled += math.hypot(
                    pose[0] - run.truth[0], pose[1] - run.truth[1]
                )
            run.truth = pose

        known = [r.truth for r in self.runs.values() if r.truth is not None]
        if len(known) >= 2:
            closest = min(
                math.hypot(a[0] - b[0], a[1] - b[1])
                for i, a in enumerate(known)
                for b in known[i + 1 :]
            )
            self.separations.append((self.now_s(), closest))

    def _on_costmap(self, name, msg):
        """Record what the global costmap contains, and what the ramp filter added.

        THE MEASUREMENT THAT IS NOT CIRCULAR

        The obvious test - "cost inside the ramp footprint is zero" - does not work
        and was tried first. The ramp region is only partly explored, and the part
        the LiDAR does reach contains the real plateau face, so it reads LETHAL from
        the static layer whatever the filter does. Worse, the unexplored remainder
        reads unknown, which would let a filter that never loaded look identical to
        one contributing nothing.

        What makes this decidable is that Phase 3's mask is UNIFORM. A filter adding
        cost would add the same cost over every cell, so the minimum cost across all
        KNOWN cells would rise off zero - which is exactly the symptom of the
        background-pixel mistake the mask generator guards against, where an unknown
        grey of 205 instead of white silently costs the entire warehouse about 48.

        So the claim rests on one number: mapped free space still reads exactly 0.
        """
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        x_min, x_max, y_min, y_max = RAMP_REGION
        col_lo = max(0, int((x_min - origin_x) / resolution))
        col_hi = min(msg.info.width, int((x_max - origin_x) / resolution))
        row_lo = max(0, int((y_min - origin_y) / resolution))
        row_hi = min(msg.info.height, int((y_max - origin_y) / resolution))

        ramp = grid[row_lo:row_hi, col_lo:col_hi]
        known = grid[grid >= 0]

        self.costmaps[name] = {
            "width": msg.info.width,
            "height": msg.info.height,
            "frame": msg.header.frame_id,
            "known_cells": int(known.size),
            "free_cells": int(np.count_nonzero(known == 0)),
            "known_min": int(known.min()) if known.size else -1,
            "costed_cells": int(np.count_nonzero(known > 0)),
            "ramp_known": int(np.count_nonzero(ramp >= 0)) if ramp.size else 0,
            "ramp_max": int(ramp.max()) if ramp.size else -1,
        }

    def _on_plan(self, name, msg):
        run = self.runs[name]
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not points:
            return
        if run.plans and max_deviation(points, run.plans[-1]) > REPLAN_DEVIATION:
            run.replans += 1
        run.plans.append(points)

    # -------------------------------------------------------------------- state

    def _tick(self):
        elapsed = self.now_s() - self.started
        if self.state == "waiting":
            if elapsed < self.start_delay:
                return
            missing = [
                r.name
                for r in self.runs.values()
                if not r.client.wait_for_server(timeout_sec=0.05)
            ]
            if missing:
                self.get_logger().info(
                    f"waiting for navigate_to_pose: {missing}",
                    throttle_duration_sec=5.0,
                )
                return
            self._dispatch_all()
            return

        if self.state == "running":
            if all(r.status not in ("PENDING", "RUNNING") for r in self.runs.values()):
                self.state = "lingering"
                self.finished_at = self.now_s()
            elif elapsed > self.timeout:
                for run in self.runs.values():
                    if run.status in ("PENDING", "RUNNING"):
                        run.status = "TIMEOUT"
                        run.finished_at = self.now_s()
                self.get_logger().error("run timed out; reporting what was recorded")
                self.state = "lingering"
                self.finished_at = self.now_s()

        if self.state == "lingering" and self.now_s() - self.finished_at >= self.linger:
            self.state = "done"
            try:
                self._write_report()
            finally:
                rclpy.shutdown()

    def _dispatch_all(self):
        """Send every robot's goal without waiting for any of them.

        Sent in one pass rather than one per tick: the whole point of the run is that
        both stacks are planning and driving at the same time, and staggering the
        dispatch by even a second would let the first robot clear the aisle first.
        """
        self.state = "running"
        for run in self.runs.values():
            x, y, yaw = run.goal
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = FLEET_FRAME
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            z, w = quaternion_from_yaw(yaw)
            goal.pose.pose.orientation.z = z
            goal.pose.pose.orientation.w = w

            run.status = "RUNNING"
            run.dispatched_at = self.now_s()
            run.start_truth = run.truth
            self.get_logger().info(
                f"{run.name}: dispatching goal ({x:+.2f}, {y:+.2f}) in {FLEET_FRAME}"
            )
            run.client.send_goal_async(goal).add_done_callback(
                lambda future, r=run: self._on_goal_response(r, future)
            )

    def _on_goal_response(self, run, future):
        handle = future.result()
        if not handle.accepted:
            run.status = "REJECTED"
            run.finished_at = self.now_s()
            self.get_logger().error(f"{run.name}: goal REJECTED")
            return
        handle.get_result_async().add_done_callback(
            lambda result, r=run: self._on_result(r, result)
        )

    def _on_result(self, run, future):
        status = future.result().status
        run.status = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"STATUS_{status}")
        run.finished_at = self.now_s()
        self.get_logger().info(f"{run.name}: goal finished {run.status}")

    # ------------------------------------------------------------------- report

    def _write_report(self):
        os.makedirs(self.results_dir, exist_ok=True)
        stem = os.path.join(self.results_dir, f"phase3_{self.tag}")

        lines = []
        add = lines.append
        rule = "=" * 78
        thin = "-" * 78

        add(rule)
        add("PHASE 3 - concurrent goals to the whole fleet")
        add(rule)
        add(f"robots dispatched together:   {len(self.runs)}")
        add(f"goal frame:                   {FLEET_FRAME} (= the Gazebo world frame)")
        add(thin)
        add("[A] PER ROBOT")
        add("")
        add(
            "      robot        result        t_goal   planned   driven   final err"
            "   replans"
        )
        for run in self.runs.values():
            duration = run.duration
            planned = 0.0
            if run.plans:
                first = run.plans[0]
                planned = sum(
                    math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(first, first[1:])
                )
            error = run.final_error
            add(
                f"      {run.name:<12} {run.status:<12} "
                f"{(duration if duration is not None else float('nan')):7.1f}  "
                f"{planned:8.2f} {run.travelled:8.2f}  "
                f"{(error if error is not None else float('nan')):9.3f}  "
                f"{run.replans:8d}"
            )
        add("")
        add("      t_goal seconds, distances metres. 'driven' is ground truth, not")
        add("      odometry - Phase 0 measured wheel odometry reporting an 11 m")
        add("      journey the robot never made.")
        add(thin)
        add("[B] FLEET SEPARATION")
        if self.separations:
            values = [s for _, s in self.separations]
            closest = min(values)
            when = next(t for t, s in self.separations if s == closest)
            add(f"    {'samples:':<40} {len(values):8d}")
            add(f"    {'closest approach (m):':<40} {closest:8.3f}")
            add(f"    {'at t (s):':<40} {when:8.1f}")
            median = sorted(values)[len(values) // 2]
            add(f"    {'median separation (m):':<40} {median:8.3f}")
        else:
            add("    no ground-truth samples for two robots at once")
        add("")
        add("    Distance between FOOTPRINT ORIGINS, so it is a clearance measure and")
        add("    not a collision check. The two routes are deconflicted by design;")
        add("    the forced conflict, mutual local deviation and the yield protocol")
        add("    are Phases 6 and 7.")
        add(thin)
        add("[C] GLOBAL COSTMAP AND THE RAMP FILTER")
        if self.costmaps:
            add("")
            add(
                "      robot        frame        size        known    free"
                "   costed   min cost"
            )
            for name, costmap in self.costmaps.items():
                add(
                    f"      {name:<12} {costmap['frame']:<12} "
                    f"{costmap['width']}x{costmap['height']:<5} "
                    f"{costmap['known_cells']:8d} {costmap['free_cells']:7d}"
                    f" {costmap['costed_cells']:8d}   {costmap['known_min']:8d}"
                )
            add("")
            add("    Phase 3 ships the KeepoutFilter loaded and wired with an")
            add("    all-zero mask, and 'min cost' over known cells is the number")
            add("    that shows it contributes nothing. The mask is UNIFORM, so a")
            add("    filter adding cost would raise every cell together and no cell")
            add("    could read 0 - which is exactly what a background pixel of 205")
            add("    instead of white would do, costing the whole warehouse ~48.")
            add("")
            add("    Cost inside the ramp footprint is NOT the test and is reported")
            add("    only for reference: that region is partly unexplored and partly")
            add("    the real plateau face, so it reads lethal from the static layer")
            add("    whatever the filter does. Phase 4 replaces the null mask with a")
            add("    graded one and measures the routing change instead.")
            add("")
            for name, costmap in self.costmaps.items():
                add(
                    f"      {name}: ramp footprint {costmap['ramp_known']} known "
                    f"cells, max cost {costmap['ramp_max']}"
                )
        else:
            add("    no costmap received")
        add(thin)
        add("[D] VERDICT")
        checks = [
            (
                "every robot's goal was accepted",
                all(r.status != "REJECTED" for r in self.runs.values()),
            ),
            (
                "every robot reached its goal",
                all(r.status == "SUCCEEDED" for r in self.runs.values()),
            ),
            (
                "both robots were tracked simultaneously",
                bool(self.separations),
            ),
            (
                "the fleet never closed to within 0.5 m",
                bool(self.separations) and min(s for _, s in self.separations) > 0.5,
            ),
            (
                "both global costmaps planned in the fleet frame",
                bool(self.costmaps)
                and all(c["frame"] == FLEET_FRAME for c in self.costmaps.values()),
            ),
            (
                "the costmaps are populated",
                bool(self.costmaps)
                and all(c["costed_cells"] > 0 for c in self.costmaps.values()),
            ),
            (
                "mapped free space still reads zero cost",
                bool(self.costmaps)
                and all(
                    c["free_cells"] > 0 and c["known_min"] == 0
                    for c in self.costmaps.values()
                ),
            ),
        ]
        for label, passed in checks:
            add(f"    {label + ':':<50} {'YES' if passed else 'NO'}")
        ok = all(passed for _, passed in checks)
        add(f"    RESULT: {'PASS' if ok else 'FAIL'}")
        add(rule)

        text = "\n".join(lines)
        print(text, flush=True)
        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write("# Phase 3 - concurrent goals to the whole fleet\n\n")
            handle.write("```\n")
            handle.write(text)
            handle.write("\n```\n")

        with open(f"{stem}.csv", "w", encoding="utf-8") as handle:
            handle.write(
                "robot,status,dispatched_s,finished_s,duration_s,driven_m,"
                "final_error_m,replans,goal_x,goal_y\n"
            )
            for run in self.runs.values():
                duration = run.duration
                error = run.final_error
                handle.write(
                    f"{run.name},{run.status},"
                    f"{run.dispatched_at if run.dispatched_at is not None else ''},"
                    f"{run.finished_at if run.finished_at is not None else ''},"
                    f"{duration if duration is not None else ''},"
                    f"{run.travelled:.3f},"
                    f"{error if error is not None else ''},"
                    f"{run.replans},{run.goal[0]:.3f},{run.goal[1]:.3f}\n"
                )

        with open(f"{stem}_separation.csv", "w", encoding="utf-8") as handle:
            handle.write("t,separation_m\n")
            for t, separation in self.separations:
                handle.write(f"{t:.3f},{separation:.4f}\n")

        self.get_logger().info(f"wrote {stem}.md")


def main(args=None):
    """Run one concurrent fleet mission."""
    rclpy.init(args=args)
    node = FleetMission()
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
