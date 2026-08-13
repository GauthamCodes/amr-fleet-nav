"""TrajectoryPredictor: publish where this robot will be, and when.

One per robot. Its output is the input to every OTHER robot's
``FleetTrajectoryLayer``, which is the pair that satisfies the assignment's MAPF
requirement (3.2): a robot's local planner consumes its peers' intentions
directly, with no central node in the loop.

THE MESSAGE CONTRACT, AND WHY IT IS nav_msgs/Path

    A predicted trajectory is a path plus a time per point. ``nav_msgs/Path``
    already carries a ``header.stamp`` on every ``PoseStamped``, and this node
    fills each one with the time that pose is predicted to be REACHED. A custom
    message would have cost an interface package, a build dependency for every
    consumer, and a second thing to keep in sync, to add a field the standard
    message already has.

    The convention is not free of risk and the risk is worth naming: a per-pose
    stamp in the future is not a valid TF query time. The consuming layer looks
    the frame up at ``TimePointZero`` for exactly this reason - see
    ``fleet_trajectory_layer.cpp``.

FRAME

    ``fleet_map``. Peers consume this, and the fleet frame is the only frame two
    robots share - which is the practical argument for Phase 3 having built it.

STATE, AND WHAT EACH ONE PREDICTS

    active plan     resample the part of the plan ahead of the robot and time it
                    at the measured speed
    no plan, moving straight-line projection at the current heading and speed
    no plan, still  a single sample at the current pose, dt = 0

    That last case is deliberate. A robot parked in an aisle is precisely what
    its peers must route around, and publishing an empty path there would make a
    stopped robot invisible to the fleet.
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from amr_bsp.topics import PREDICTED_TRAJECTORY
from amr_description.fleet_config import frame_prefix, load_fleet
from amr_fleet_control.fleet_grid import FLEET_FRAME
from amr_fleet_control.trajectory_predict import (
    DEFAULT_HORIZON_S,
    DEFAULT_SPACING_M,
    MIN_PREDICT_SPEED,
    predict_along_plan,
    predict_constant_velocity,
)


def _yaw_from_quaternion(q):
    """Return the yaw of a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class TrajectoryPredictor(Node):
    """Publish this robot's predicted trajectory for its peers to consume."""

    def __init__(self):
        """Wire up this robot's plan and odometry inputs and its peer-facing output."""
        super().__init__("trajectory_predictor")

        self.declare_parameter("robot", "")
        self.declare_parameter("horizon_s", DEFAULT_HORIZON_S)
        self.declare_parameter("spacing_m", DEFAULT_SPACING_M)
        self.declare_parameter("min_predict_speed", MIN_PREDICT_SPEED)
        self.declare_parameter("rate_hz", 5.0)
        # A plan older than this is not driving the robot any more - the goal was
        # reached, cancelled or aborted. Predicting along a dead plan reserves a
        # corridor the robot has no intention of entering.
        self.declare_parameter("plan_stale_s", 3.0)

        robot_name = self.get_parameter("robot").value
        fleet = {r["name"]: r for r in load_fleet()}
        if robot_name not in fleet:
            raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
        self.robot = fleet[robot_name]
        self.base_frame = f"{frame_prefix(self.robot)}base_footprint"

        self.horizon_s = float(self.get_parameter("horizon_s").value)
        self.spacing_m = float(self.get_parameter("spacing_m").value)
        self.min_speed = float(self.get_parameter("min_predict_speed").value)
        self.plan_stale_s = float(self.get_parameter("plan_stale_s").value)

        self.plan_points = []
        self.plan_time = None
        self.speed = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Path, PREDICTED_TRAJECTORY, 10)
        self.create_subscription(Path, "plan", self._on_plan, 1)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f"TrajectoryPredictor for {robot_name}: horizon {self.horizon_s:.1f} s, "
            f"spacing {self.spacing_m:.2f} m, frame {FLEET_FRAME}"
        )

    def _on_plan(self, msg):
        """Cache the global plan, rejecting one that is not in the fleet frame."""
        # Phase 3 moved /plan into the fleet frame and a Phase 1 consumer kept
        # applying a spawn-pose offset on top, which corrupted a CSV while leaving
        # every derived number correct. Checking the frame here rather than
        # assuming it is the cheap version of that lesson.
        if msg.header.frame_id != FLEET_FRAME:
            self.get_logger().warn(
                f"plan is in '{msg.header.frame_id}', want '{FLEET_FRAME}'; ignoring",
                throttle_duration_sec=10.0,
            )
            return
        self.plan_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.plan_time = self.get_clock().now()

    def _on_odom(self, msg):
        """Record MEASURED forward speed (docs/ENGINEERING_NOTES.md rule 3, same as SafetyGate)."""
        self.speed = float(msg.twist.twist.linear.x)

    def _pose_in_fleet_frame(self):
        """Return ``(x, y, yaw)`` in the fleet frame, or ``None`` if TF is not ready."""
        try:
            tf = self.tf_buffer.lookup_transform(
                FLEET_FRAME, self.base_frame, Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"no {FLEET_FRAME} -> {self.base_frame} transform: {exc}",
                throttle_duration_sec=10.0,
            )
            return None
        return (
            tf.transform.translation.x,
            tf.transform.translation.y,
            _yaw_from_quaternion(tf.transform.rotation),
        )

    def _plan_is_live(self, now):
        """Return whether the cached plan is recent enough to predict along."""
        if not self.plan_points or self.plan_time is None:
            return False
        return (now - self.plan_time).nanoseconds * 1e-9 <= self.plan_stale_s

    def _publish(self):
        """Build and publish the prediction for this cycle."""
        pose = self._pose_in_fleet_frame()
        if pose is None:
            return
        x, y, yaw = pose
        now = self.get_clock().now()

        if self._plan_is_live(now):
            samples = predict_along_plan(
                self.plan_points,
                x,
                y,
                self.speed,
                horizon_s=self.horizon_s,
                spacing_m=self.spacing_m,
                min_speed=self.min_speed,
            )
        else:
            samples = predict_constant_velocity(
                x,
                y,
                yaw,
                self.speed,
                horizon_s=self.horizon_s,
                spacing_m=self.spacing_m,
            )

        path = Path()
        path.header.frame_id = FLEET_FRAME
        path.header.stamp = now.to_msg()
        for sx, sy, dt in samples:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = FLEET_FRAME
            # THE ARRIVAL TIME, not the publication time. See the module docstring.
            pose_msg.header.stamp = (now + Duration(seconds=dt)).to_msg()
            pose_msg.pose.position.x = sx
            pose_msg.pose.position.y = sy
            pose_msg.pose.orientation.w = 1.0
            path.poses.append(pose_msg)
        self.publisher.publish(path)


def main(args=None):
    """Spin one robot's trajectory predictor."""
    rclpy.init(args=args)
    node = TrajectoryPredictor()
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
