#!/usr/bin/env python3
"""PHASE 2 - a model of the motor controller's command watchdog. PLANT, not nav stack.

WHY THIS EXISTS

    ENGINEERING_NOTES rule 1 says that if SafetyGate dies, nothing moves. Making the
    gate the
    only publisher of ``/amrN/cmd_vel`` is necessary for that but not sufficient: it
    guarantees no NEW command reaches the wheels, and says nothing about the command
    already there. Gazebo's DiffDrive latches its last target velocity indefinitely -
    verified directly against gz-sim 8.11.0's plugin binary, whose only cmd-related
    string is the topic name, with no timeout parameter anywhere. So a SIGKILLed gate
    would leave a rolling robot, and the invariant would be aspirational.

    Real hardware does not behave that way. A motor controller that stops hearing from
    its host brakes, because a silent host is indistinguishable from a crashed one.
    That behaviour belongs to the DRIVE, not to the navigation software, and this is
    where the simulation grows it:

        SafetyGate -> /amrN/cmd_vel -> drive_watchdog
                   -> /amrN/cmd_vel_plant -> DiffDrive

WHY THIS DOES NOT MAKE SafetyGate "NOT THE LAST LINK"

    This node originates nothing and modifies nothing. It forwards, or it zeroes on
    silence. Every command still reaches the wheels only by passing the gate, and
    nothing downstream of the gate can introduce motion. It sits on the plant side of
    the boundary for the same reason the DiffDrive plugin's own acceleration limits
    do: it models the vehicle, not the software driving it.

    On real hardware this node does not exist - the motor controller is the watchdog.
    The evidence run records the SIGKILL case both with and without it, so what the
    watchdog buys is a measurement rather than an assertion.

WHY THE TIMEOUT IS 0.3 s

    Longer than any gap the gate produces in normal operation and shorter than the
    distance an error can carry the robot. SafetyGate publishes at 20 Hz whenever it
    is blocked or its input has gone stale, and passes commands through at Nav2's
    20 Hz otherwise, so the steady-state gap is 50 ms - six times inside this. At
    amr1's 0.6 m/s top speed, 0.3 s of latched command is 0.18 m.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class DriveWatchdog(Node):
    """Forwards velocity commands and zeroes the wheels when the stream goes quiet."""

    def __init__(self):
        """Declare the timeout, wire the relay, and start the silence check."""
        super().__init__("drive_watchdog")
        self.declare_parameter("timeout_s", 0.3)
        self.declare_parameter("check_rate_hz", 50.0)
        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "cmd_vel_plant")

        self.timeout_s = float(self.get_parameter("timeout_s").value)
        check_rate = float(self.get_parameter("check_rate_hz").value)
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.last_command_time = None
        self.stopped = False
        self.timeouts = 0

        self.publisher = self.create_publisher(Twist, output_topic, 1)
        self.create_subscription(Twist, input_topic, self._on_command, 1)
        self.create_timer(1.0 / check_rate, self._check_silence)
        self.get_logger().info(
            f"drive_watchdog: {input_topic} -> {output_topic}, "
            f"zeroing after {self.timeout_s:.3f} s of silence"
        )

    def _on_command(self, msg):
        """Forward a command unchanged and note that the stream is alive."""
        self.last_command_time = self.get_clock().now()
        self.stopped = False
        self.publisher.publish(msg)

    def _check_silence(self):
        """Zero the wheels once if nothing has been heard within the timeout."""
        if self.last_command_time is None:
            return
        quiet = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if quiet <= self.timeout_s or self.stopped:
            return
        # Once, not continuously: DiffDrive latches, so one zero is enough to stop
        # the wheels, and repeating it would bury the log line that says why.
        self.stopped = True
        self.timeouts += 1
        self.publisher.publish(Twist())
        self.get_logger().warn(
            f"command stream silent for {quiet:.3f} s - wheels zeroed "
            f"(timeout {self.timeouts})"
        )


def main():
    rclpy.init()
    node = DriveWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
