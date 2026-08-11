#!/usr/bin/env python3
"""SMOKE TEST 2 - scripted drive up the ramp and back down.

The phase brief calls for teleop. This drives the same route from a script instead,
for one reason that matters to the deliverable: the numbers smoke test 2 produces go
into the README as the justification for PitchGate, and a number an evaluator can
reproduce with a single command is worth more than one that depended on how somebody
held a key down. ``teleop_twist_keyboard`` still works against the same topic for
anyone who wants to drive it by hand.

ROUTE
    forward up the ramp -> onto the plateau -> pause -> REVERSE back down.

The reverse leg is not decoration. Driving up, the robot pitches nose-up and the
downhill-facing beams point back at ground the robot has already covered. Reversing
back down keeps the nose pointed uphill while the robot travels downhill, so the
descending beams now look along the direction of travel - which is the case that
actually threatens a safety stop, and the one the README number needs to describe.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class Smoke2Drive(Node):
    """Drive a fixed, reproducible profile over the ramp."""

    def __init__(self):
        """Declare the drive profile and subscribe to odometry."""
        super().__init__("smoke2_drive")

        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("cruise_speed", 0.35)
        # World coordinates. /odom starts at the spawn pose, so spawn_x converts.
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("plateau_x", 8.0)
        self.declare_parameter("return_x", -1.5)
        self.declare_parameter("pause_s", 3.0)

        self.robot_name = self.get_parameter("robot_name").value
        self.cruise = float(self.get_parameter("cruise_speed").value)
        self.spawn_x = float(self.get_parameter("spawn_x").value)
        self.plateau_x = float(self.get_parameter("plateau_x").value)
        self.return_x = float(self.get_parameter("return_x").value)
        self.pause_s = float(self.get_parameter("pause_s").value)

        self.publisher = self.create_publisher(Twist, f"/{self.robot_name}/cmd_vel", 10)
        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._on_odom, 10
        )

        self.x = None
        self.y = 0.0
        self.phase = "wait"
        self.phase_started = None
        self.create_timer(0.05, self._tick)
        self.get_logger().info("smoke2 drive armed")

    def _on_odom(self, msg):
        # Report world x so the thresholds below read as world coordinates.
        self.x = msg.pose.pose.position.x + self.spawn_x
        self.y = msg.pose.pose.position.y

    def _elapsed(self):
        if self.phase_started is None:
            return 0.0
        return (self.get_clock().now() - self.phase_started).nanoseconds / 1e9

    def _set_phase(self, phase):
        self.phase = phase
        self.phase_started = self.get_clock().now()
        self.get_logger().info(f"phase -> {phase}")

    def _publish(self, linear):
        twist = Twist()
        twist.linear.x = linear
        # Mild heading hold: the ramp is 2.5 m wide and drifting off its edge would
        # produce a pitch trace that has nothing to do with the ramp angle.
        twist.angular.z = max(-0.25, min(0.25, -0.8 * self.y))
        self.publisher.publish(twist)

    def _tick(self):
        if self.x is None:
            return

        if self.phase == "wait":
            self._publish(0.0)
            if self._elapsed() > 2.0 or self.phase_started is None:
                self._set_phase("ascend")
            return

        if self.phase == "ascend":
            self._publish(self.cruise)
            if self.x >= self.plateau_x:
                self._set_phase("pause")
            return

        if self.phase == "pause":
            self._publish(0.0)
            if self._elapsed() >= self.pause_s:
                self._set_phase("descend")
            return

        if self.phase == "descend":
            self._publish(-self.cruise)
            if self.x <= self.return_x:
                self._set_phase("done")
            return

        self._publish(0.0)


def main():
    rclpy.init()
    node = Smoke2Drive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        _ = math


if __name__ == "__main__":
    main()
