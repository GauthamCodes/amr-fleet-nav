"""PayloadJerkAdapter: the one link the stock smoother does not provide.

Sits AFTER ``nav2_velocity_smoother`` in the command chain, not instead of it
(ENGINEERING_NOTES rule 6):

    Nav2 -> cmd_vel_nav -> nav2_velocity_smoother -> cmd_vel_smoothed
         -> PayloadJerkAdapter -> cmd_vel_shaped -> twist_mux -> SafetyGate

The stock node bounds velocity and acceleration. This one adds exactly two
things it does not have:

    1. A JERK bound - the second derivative. The stock smoother will happily
       invert acceleration from +a_max to -a_max inside one control period,
       which is what unseats a load.
    2. PAYLOAD SCALING - both the acceleration and the jerk bounds tighten as
       mass rises, on the fixed-traction-force model derived in
       :mod:`amr_motion.jerk_limiter`.

RUNTIME PAYLOAD, AND WHY IT DEFAULTS TO ZERO

    ``payload_kg`` in fleet.yaml is the robot's RATED CAPACITY. It is what
    ``amr_safety.safety_model`` sizes the braking envelope against, deliberately
    conservatively. It is not a claim about what the robot is carrying right now.

    The current load arrives on the ``payload_kg`` topic, latched, and defaults
    to 0.0 - unloaded. So an ordinary bringup behaves exactly as the stack did
    before this node existed, and "loaded" is an explicit runtime state a
    mission (or the evidence run) sets. That is what makes the loaded/unloaded
    comparison an A/B rather than two different builds.

FAIL-SAFE, NOT FAIL-OPEN

    On losing its input this node emits nothing rather than repeating the last
    command. It is not the safety element - SafetyGate is, and it is downstream
    with its own command timeout - but a shaping node that keeps republishing a
    stale velocity would defeat that timeout by keeping the stream alive.
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

from amr_bsp.topics import CMD_SHAPED, CMD_SMOOTHED
from amr_description.fleet_config import load_fleet
from amr_motion.jerk_limiter import limit_twist, limits_from_robot, scaled_limits


def latched_qos():
    """Return the QoS a payload state is published and consumed with.

    TRANSIENT_LOCAL so a node that starts after the payload was set still learns
    the current load, instead of silently shaping for an empty vehicle.
    """
    profile = QoSProfile(depth=1)
    profile.reliability = ReliabilityPolicy.RELIABLE
    profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return profile


class PayloadJerkAdapter(Node):
    """Jerk-limit and payload-scale the smoothed command stream."""

    def __init__(self):
        """Load this robot's limits from fleet.yaml and wire the chain link."""
        super().__init__("payload_jerk_adapter")

        self.declare_parameter("robot", "")
        self.declare_parameter("rate_hz", 20.0)
        # No command in this long -> stop shaping and go quiet. Matches
        # safety.yaml's cmd_timeout_s so the two links agree about what "the
        # upstream has stopped" means.
        self.declare_parameter("cmd_timeout_s", 0.5)

        robot_name = self.get_parameter("robot").value
        fleet = {r["name"]: r for r in load_fleet()}
        if robot_name not in fleet:
            raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
        self.robot = fleet[robot_name]

        self.base_mass = float(self.robot["base_mass_kg"])
        self.rated_payload = float(self.robot["payload_kg"])
        self.unloaded_limits = limits_from_robot(self.robot)
        self.payload_kg = 0.0
        self.limits = scaled_limits(self.unloaded_limits, self.base_mass, 0.0)

        self.target = (0.0, 0.0)
        self.velocity = (0.0, 0.0)
        self.accel = (0.0, 0.0)
        self.last_cmd_time = None
        self.timeout = float(self.get_parameter("cmd_timeout_s").value)

        self.publisher = self.create_publisher(Twist, CMD_SHAPED, 10)
        self.create_subscription(Twist, CMD_SMOOTHED, self._on_cmd, 10)
        self.create_subscription(Float32, "payload_kg", self._on_payload, latched_qos())

        self.period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(self.period, self._tick)

        self.get_logger().info(
            f"PayloadJerkAdapter for {robot_name}: unloaded "
            f"a_max {self.unloaded_limits['max_accel_x']:.3f} m/s^2, "
            f"j_max {self.unloaded_limits['max_jerk_x']:.3f} m/s^3; "
            f"rated payload {self.rated_payload:.1f} kg on a "
            f"{self.base_mass:.1f} kg chassis"
        )

    def _on_cmd(self, msg):
        """Record the latest smoothed command as the shaping target."""
        self.target = (msg.linear.x, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def _on_payload(self, msg):
        """Rescale every limit for a new payload and log the change."""
        payload = max(0.0, float(msg.data))
        if math.isclose(payload, self.payload_kg):
            return
        self.payload_kg = payload
        self.limits = scaled_limits(self.unloaded_limits, self.base_mass, payload)
        self.get_logger().info(
            f"payload {payload:.1f} kg -> a_max "
            f"{self.limits['max_accel_x']:.3f} m/s^2, j_max "
            f"{self.limits['max_jerk_x']:.3f} m/s^3"
        )

    def _stale(self, now):
        """Return whether the upstream has stopped commanding."""
        if self.last_cmd_time is None:
            return True
        return (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout

    def _tick(self):
        """Advance the shaped command by one period and publish it."""
        now = self.get_clock().now()
        if self._stale(now):
            # Go quiet rather than repeat. See the module docstring.
            self.target = (0.0, 0.0)
            self.velocity = (0.0, 0.0)
            self.accel = (0.0, 0.0)
            return

        self.velocity, self.accel = limit_twist(
            self.target, (self.velocity, self.accel), self.period, self.limits
        )

        msg = Twist()
        msg.linear.x = self.velocity[0]
        msg.angular.z = self.velocity[1]
        self.publisher.publish(msg)


def main(args=None):
    """Spin one robot's payload-adaptive jerk limiter."""
    rclpy.init(args=args)
    node = PayloadJerkAdapter()
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
