"""Measure one robot's trajectory arriving in another robot's local costmap.

This is the instrument for the MAPF claim. The claim is not "a plugin is
loaded"; it is that AMR-1's predicted trajectory becomes cost in AMR-2's LOCAL
costmap, at the right cell, at the right time, with the right magnitude. So the
probe reads the costmap AMR-2's controller actually reads and reports the cost
at the cell AMR-1 is predicted to occupy.

WHY THE PROBE LOOKS AHEAD RATHER THAN AT THE PEER

    AMR-1 is a physical object. AMR-2's LiDAR sees it, the obstacle layer marks
    it LETHAL and the inflation layer surrounds it - all of which happens whether
    or not this plugin exists. Sampling the cell AMR-1 is standing in would
    therefore measure the obstacle layer and report it as a MAPF result.

    The probe instead samples where AMR-1 is predicted to be ``probe_dt`` seconds
    from now. At the default 2 s that cell is empty floor, far enough ahead of
    AMR-1 to be outside its inflation radius - and the probe records that
    distance every sample, so the report can show the separation rather than
    assert it. Any cost there is this layer's, and the layer-off arm confirms it.

WHAT MAKES THE NUMBER FALSIFIABLE

    The layer stamps ``max_cost * exp(-dt / decay_tau_s)``. With the shipped
    240 and 3.0 s, a sample 2 s ahead predicts cost 123. The report states the
    prediction next to the measurement, so the run can disagree with the model
    instead of merely producing a positive number.
"""

import math
import os

from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

from amr_bsp.topics import PREDICTED_TRAJECTORY
from amr_description.fleet_config import frame_prefix, load_fleet
from amr_fleet_control.fleet_grid import FLEET_FRAME


def _quat_yaw(q):
    """Return the yaw of a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class TrajectoryProbe(Node):
    """Sample the observer's local costmap where the peer is predicted to be."""

    def __init__(self):
        """Wire up the peer's trajectory, the observer's costmap, and ground truth."""
        super().__init__("trajectory_probe")

        self.declare_parameter("observer", "amr2")
        self.declare_parameter("peer", "amr1")
        self.declare_parameter("probe_dt_s", 2.0)
        self.declare_parameter("layer_enabled", True)
        # Mirrors of the shipped layer configuration, so the report can print the
        # cost the model predicts beside the cost the costmap actually carried.
        self.declare_parameter("layer_max_cost", 240.0)
        self.declare_parameter("layer_decay_tau_s", 3.0)
        self.declare_parameter("inflation_radius_m", 0.70)
        self.declare_parameter("results_dir", os.path.join(os.getcwd(), "results"))
        self.declare_parameter("tag", "layer_on")
        self.declare_parameter("duration_s", 90.0)

        self.observer = self.get_parameter("observer").value
        self.peer = self.get_parameter("peer").value
        self.probe_dt = float(self.get_parameter("probe_dt_s").value)
        self.layer_enabled = bool(self.get_parameter("layer_enabled").value)
        self.max_cost = float(self.get_parameter("layer_max_cost").value)
        self.tau = float(self.get_parameter("layer_decay_tau_s").value)
        self.inflation_radius = float(self.get_parameter("inflation_radius_m").value)
        self.results_dir = self.get_parameter("results_dir").value
        self.tag = self.get_parameter("tag").value
        self.duration_s = float(self.get_parameter("duration_s").value)

        fleet = {r["name"]: r for r in load_fleet()}
        for name in (self.observer, self.peer):
            if name not in fleet:
                raise ValueError(f"'{name}' is not in fleet.yaml: {sorted(fleet)}")
        self.observer_frame = f"{frame_prefix(fleet[self.observer])}base_footprint"
        self.peer_frame = f"{frame_prefix(fleet[self.peer])}base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.peer_trajectory = None
        self.truth = {}
        self.samples = []
        self.costmap_count = 0
        self.trajectory_count = 0
        self.started = None
        self.written = False

        self.create_subscription(
            Path, f"/{self.peer}/{PREDICTED_TRAJECTORY}", self._on_trajectory, 5
        )
        self.create_subscription(
            Costmap, f"/{self.observer}/local_costmap/costmap_raw", self._on_costmap, 5
        )
        self.create_subscription(TFMessage, "/gz_ground_truth", self._on_truth, 40)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"TrajectoryProbe: {self.peer} trajectory -> {self.observer} local "
            f"costmap, probing {self.probe_dt:.1f} s ahead, "
            f"layer_enabled={self.layer_enabled}"
        )

    # -- inputs --------------------------------------------------------------

    def _on_trajectory(self, msg):
        self.trajectory_count += 1
        self.peer_trajectory = msg

    def _on_truth(self, msg):
        """Record ground-truth positions, keyed by child frame."""
        for transform in msg.transforms:
            child = transform.child_frame_id
            self.truth[child] = (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )

    def _truth_xy(self, robot):
        """Return a robot's ground-truth position, matching on frame suffix."""
        for child, xy in self.truth.items():
            if child == robot or child.startswith(f"{robot}/") or child.endswith(robot):
                return xy
        return None

    # -- the measurement -----------------------------------------------------

    def _probe_pose(self, now):
        """Return the peer pose ``probe_dt`` ahead, in the fleet frame."""
        if self.peer_trajectory is None or not self.peer_trajectory.poses:
            return None
        best = None
        best_error = float("inf")
        for pose in self.peer_trajectory.poses:
            dt = (Time.from_msg(pose.header.stamp) - now).nanoseconds * 1e-9
            error = abs(dt - self.probe_dt)
            if error < best_error:
                best_error = error
                best = (pose.pose.position.x, pose.pose.position.y, dt)
        # A trajectory that never reaches probe_dt (a stopped peer publishes one
        # sample at dt = 0) is not a probe failure, but it is not a measurement of
        # a LOOK-AHEAD either, so it is dropped rather than quietly reported.
        if best is None or best_error > 0.75:
            return None
        return best

    def _on_costmap(self, msg):
        """Read the cost at the peer's predicted cell in the observer's costmap."""
        self.costmap_count += 1
        now = self.get_clock().now()
        probe = self._probe_pose(now)
        if probe is None:
            return
        px, py, dt = probe

        try:
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id, FLEET_FRAME, Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"no {FLEET_FRAME} -> {msg.header.frame_id} transform: {exc}",
                throttle_duration_sec=10.0,
            )
            return

        yaw = _quat_yaw(tf.transform.rotation)
        cx = tf.transform.translation.x + px * math.cos(yaw) - py * math.sin(yaw)
        cy = tf.transform.translation.y + px * math.sin(yaw) + py * math.cos(yaw)

        meta = msg.metadata
        col = int((cx - meta.origin.position.x) / meta.resolution)
        row = int((cy - meta.origin.position.y) / meta.resolution)
        if not (0 <= col < meta.size_x and 0 <= row < meta.size_y):
            return  # Predicted cell is outside the observer's rolling window.
        cost = int(msg.data[row * meta.size_x + col])

        peer_xy = self._truth_xy(self.peer)
        observer_xy = self._truth_xy(self.observer)
        # Distance from the peer's CURRENT position to the probed cell. The
        # measurement is only clean while this exceeds the inflation radius,
        # and the report prints the minimum rather than assuming it.
        lead = math.hypot(px - peer_xy[0], py - peer_xy[1]) if peer_xy else float("nan")
        separation = (
            math.hypot(peer_xy[0] - observer_xy[0], peer_xy[1] - observer_xy[1])
            if peer_xy and observer_xy
            else float("nan")
        )

        peak = max(int(v) for v in msg.data) if len(msg.data) else 0
        self.samples.append(
            {
                "t": now.nanoseconds * 1e-9,
                "dt": dt,
                "cost": cost,
                "predicted_cost": self.max_cost * math.exp(-max(dt, 0.0) / self.tau),
                "lead_m": lead,
                "separation_m": separation,
                "peak_cost": peak,
            }
        )

    # -- lifecycle -----------------------------------------------------------

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.started is None:
            self.started = now
            return
        if now - self.started >= self.duration_s:
            self._write_report()
            raise SystemExit(0)

    def _write_report(self):
        """Write the markdown report and the per-sample CSV, at most once.

        Called from the duration timer AND from main() on shutdown. The mission
        node next to this one ends the launch as soon as both goals resolve, which
        can happen before the probe's own window closes; a probe that only wrote
        on its timer would lose the whole measurement to a fast run.
        """
        if self.written:
            return
        self.written = True
        os.makedirs(self.results_dir, exist_ok=True)
        stem = os.path.join(self.results_dir, f"phase6_cost_injection_{self.tag}")

        costs = [s["cost"] for s in self.samples]
        positive = [c for c in costs if c > 0]
        leads = [s["lead_m"] for s in self.samples if not math.isnan(s["lead_m"])]
        separations = [
            s["separation_m"] for s in self.samples if not math.isnan(s["separation_m"])
        ]
        predicted = [s["predicted_cost"] for s in self.samples]

        lines = []

        def add(text=""):
            lines.append(text)

        add(f"# Phase 6 - trajectory cost injection ({self.tag})")
        add()
        add(f"Does `{self.peer}`'s predicted trajectory become cost in")
        add(f"`{self.observer}`'s **local** costmap? The probe reads")
        add(f"`/{self.observer}/local_costmap/costmap_raw` at the cell `{self.peer}`")
        add(f"is predicted to occupy {self.probe_dt:.1f} s ahead.")
        add()
        add(f"- `fleet_trajectory_layer.enabled`: **{self.layer_enabled}**")
        add(f"- costmaps received: {self.costmap_count}")
        add(f"- peer trajectories received: {self.trajectory_count}")
        add(f"- samples with the predicted cell in the window: **{len(costs)}**")
        add()

        if not costs:
            add("**No samples.** Either the peer published no look-ahead")
            add("trajectory, or its predicted cell never entered the observer's")
            add("8 x 8 m rolling window. Nothing is claimed from this run.")
        else:
            ordered = sorted(costs)
            median = ordered[len(ordered) // 2]
            share = 100.0 * len(positive) / len(costs)
            add("| quantity | value |")
            add("|---|---|")
            add(f"| cells sampled | {len(costs)} |")
            add(f"| **samples with cost > 0** | **{len(positive)}** ({share:.1f} %) |")
            add(f"| median cost at the predicted cell | **{median}** |")
            add(f"| max cost at the predicted cell | {max(costs)} |")
            add(f"| min cost at the predicted cell | {min(costs)} |")
            model = sum(predicted) / len(predicted)
            add(f"| cost the decay model predicts (mean) | {model:.1f} |")
            if leads:
                add(f"| **min lead, peer to probed cell** | **{min(leads):.3f} m** |")
                add(f"| inflation radius | {self.inflation_radius:.2f} m |")
            if separations:
                add(f"| closest true separation | {min(separations):.3f} m |")
            add()
            if leads and min(leads) <= self.inflation_radius:
                add("**Caveat, stated rather than buried:** the probed cell came")
                add(f"within {min(leads):.3f} m of the peer at least once, inside the")
                add(f"{self.inflation_radius:.2f} m inflation radius. Those samples")
                add("cannot distinguish this layer from the obstacle layer's")
                add("inflation. The layer-off arm is the discriminator, not this")
                add("table.")
            elif leads:
                add(f"Every probed cell was at least {min(leads):.3f} m ahead of the")
                add("peer's own position when it was sampled, beyond the")
                add(f"{self.inflation_radius:.2f} m inflation radius - so no sample")
                add("here can be explained by the obstacle layer marking the peer's")
                add("body.")
            add()
            add("Read this against `phase6_cost_injection_layer_off.md`. A positive")
            add("cost here only means something if the same cell, in the same")
            add("scenario, reads 0 with the layer disabled.")

        add()
        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")

        with open(f"{stem}.csv", "w", encoding="utf-8") as handle:
            handle.write(
                "t,probe_dt_s,cost,predicted_cost,lead_m,separation_m,peak_cost\n"
            )
            for s in self.samples:
                handle.write(
                    f"{s['t']:.3f},{s['dt']:.3f},{s['cost']},{s['predicted_cost']:.1f},"
                    f"{s['lead_m']:.3f},{s['separation_m']:.3f},{s['peak_cost']}\n"
                )
        self.get_logger().info(f"wrote {stem}.md and {stem}.csv")


def main(args=None):
    """Run the probe for its configured duration, then write the report."""
    rclpy.init(args=args)
    node = TrajectoryProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
