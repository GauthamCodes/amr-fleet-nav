#!/usr/bin/env python3
"""PHASE 2 - injects an implausible IMU angular velocity, from OUTSIDE the BSP.

WHY THE FAULT HOOK IS A SEPARATE PROCESS

    The obvious way to demonstrate a validator is a ``simulate_fault`` parameter on
    the node being tested. That is also the way to end up shipping a safety-relevant
    node with a code path whose only purpose is to corrupt its own input. Here the
    corruption lives in a different package's script, the BSP is launched byte-for-
    byte as it is in production, and the only thing the evidence run changes is which
    topic the BSP's ``raw_imu_topic`` parameter points at:

        gz -> /amrN/imu -> imu_fault_injector -> /amrN/imu_injected -> SensorBSP

    So what the report shows is the production validator rejecting a real message on
    a real graph, not a node reporting on a branch it took because it was asked to.

WHAT IS AND IS NOT CORRUPTED

    Only ``angular_velocity``. The header stamp, the frame, the orientation and the
    linear acceleration pass through untouched, so a rejection can only be attributed
    to the injected rate - the other checks in ``validate_imu`` have nothing to fire
    on. Injecting several faults at once would produce the same WARN count and prove
    much less.

THE MAGNITUDE

    50 rad/s, against a bound of 4 x amr1's ``max_vel_theta`` (1.0 rad/s) = 4 rad/s.
    Twelve times the bound, and about eight revolutions per second - a rate a
    warehouse AMR cannot physically reach, which is the point: this is a broken
    sensor, not an aggressive turn. The false-positive boundary (a legitimate hard
    turn) is pinned separately and cheaply in tests/test_sensor_validation.py.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFaultInjector(Node):
    """Relays the IMU, replacing angular velocity with an impossible rate."""

    def __init__(self):
        """Declare the injection window and wire the relay."""
        super().__init__("imu_fault_injector")
        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("inject_after_s", 15.0)
        self.declare_parameter("inject_duration_s", 5.0)
        self.declare_parameter("injected_rate", 50.0)
        self.declare_parameter("axis", "z")

        get = self.get_parameter
        robot = get("robot_name").value
        self.inject_after = float(get("inject_after_s").value)
        self.inject_duration = float(get("inject_duration_s").value)
        self.rate = float(get("injected_rate").value)
        self.axis = get("axis").value

        input_topic = get("input_topic").value or f"/{robot}/imu"
        output_topic = get("output_topic").value or f"/{robot}/imu_injected"

        self.started = None
        self.passed = 0
        self.injected = 0
        self.announced = False
        self.finished = False

        self.publisher = self.create_publisher(Imu, output_topic, 20)
        self.create_subscription(Imu, input_topic, self._on_imu, 20)
        self.get_logger().info(
            f"injector: {input_topic} -> {output_topic}; |w_{self.axis}| = "
            f"{self.rate:.1f} rad/s for {self.inject_duration:.1f} s starting "
            f"{self.inject_after:.1f} s after the first sample"
        )

    def _on_imu(self, msg):
        """Republish the sample, corrupting angular velocity inside the window."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.started is None:
            self.started = now
        elapsed = now - self.started

        inside = self.inject_after <= elapsed < self.inject_after + self.inject_duration
        if inside:
            # Only this field. See the module docstring.
            setattr(msg.angular_velocity, self.axis, self.rate)
            self.injected += 1
            if not self.announced:
                self.announced = True
                self.get_logger().warn(
                    f"INJECTING |w_{self.axis}| = {self.rate:.1f} rad/s at "
                    f"t+{elapsed:.2f} s"
                )
        else:
            self.passed += 1
            if self.announced and not self.finished:
                self.finished = True
                self.get_logger().info(
                    f"injection window closed after {self.injected} corrupted samples"
                )
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ImuFaultInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
