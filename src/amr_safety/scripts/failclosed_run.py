#!/usr/bin/env python3
"""PHASE 2 - what happens to a moving robot when the SafetyGate is SIGKILLed.

ENGINEERING_NOTES rule 1 says that if the gate dies, nothing moves. Being the only
publisher
of the robot's command topic is necessary for that and not sufficient: it guarantees
no NEW command reaches the wheels and says nothing about the one already there.
Gazebo's DiffDrive latches its last target velocity indefinitely - verified against
the plugin binary, which has no timeout parameter at all - so on the simulator alone
a killed gate leaves a rolling robot and the invariant would be aspirational.

This run measures that instead of asserting it. The robot is driven at a constant
speed straight through the gate, the gate process is SIGKILLed mid-motion, and the
distance it travels afterwards is recorded. Run twice:

    with_watchdog:=true    the plant-side drive_watchdog is present, as it is in
                           every ordinary run. It models a motor controller that
                           brakes when its command stream stops.
    with_watchdog:=false   the control. Nothing on the plant side notices, and the
                           simulator keeps integrating the latched command.

SIGKILL, not SIGTERM, deliberately. SIGTERM would let rclcpp's on_shutdown handler
publish its best-effort zero, which would measure an orderly shutdown - the case
that is not in question. SIGKILL is the crash.

The kill is issued by this node, which is honest about what it is: an instrument
that breaks the thing it is measuring. Nothing in the navigation stack does this.
"""

import math
import os
import subprocess

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from amr_bsp.topics import CMD_NAV

#: Matches the installed executable path, so no other process can be caught by it.
GATE_PROCESS_PATTERN = "lib/amr_safety/safety_gate"


class FailClosedRun(Node):
    """Kills the gate mid-motion and measures how far the robot rolls afterwards."""

    def __init__(self):
        """Declare parameters, wire the command path, and arm the state machine."""
        super().__init__("failclosed_run")
        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("cruise_speed", 0.35)
        self.declare_parameter("cruise_hold_s", 6.0)
        self.declare_parameter("observe_s", 10.0)
        self.declare_parameter("stopped_speed", 0.02)
        self.declare_parameter("with_watchdog", True)
        self.declare_parameter("results_dir", "results")
        self.declare_parameter("tag", "failclosed")

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        self.cruise = float(get("cruise_speed").value)
        self.cruise_hold = float(get("cruise_hold_s").value)
        self.observe = float(get("observe_s").value)
        self.stopped_speed = float(get("stopped_speed").value)
        self.with_watchdog = bool(get("with_watchdog").value)
        self.results_dir = get("results_dir").value
        self.tag = get("tag").value

        self.publisher = self.create_publisher(Twist, CMD_NAV, 1)
        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._on_odom, 20
        )

        self.position = None
        self.speed = 0.0
        self.state = "spinup"
        self.state_since = None
        self.kill_position = None
        self.kill_speed = 0.0
        self.kill_at = None
        self.stopped_at = None
        self.stop_distance = None
        self.peak_speed = 0.0
        self.killed_pids = []

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"fail-closed run armed: cruise {self.cruise:.2f} m/s, watchdog "
            f"{'PRESENT' if self.with_watchdog else 'ABSENT (control)'}"
        )

    def _on_odom(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.peak_speed = max(self.peak_speed, self.speed)

    def now_s(self):
        """Return the current simulation time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def _enter(self, state):
        self.state = state
        self.state_since = self.now_s()
        self.get_logger().info(f"state -> {state}")

    def _elapsed(self):
        return 0.0 if self.state_since is None else self.now_s() - self.state_since

    def _command(self, vx):
        twist = Twist()
        twist.linear.x = vx
        self.publisher.publish(twist)

    def _travelled(self):
        if self.position is None or self.kill_position is None:
            return 0.0
        return math.hypot(
            self.position[0] - self.kill_position[0],
            self.position[1] - self.kill_position[1],
        )

    def _tick(self):
        if self.position is None:
            return
        if self.state_since is None:
            self.state_since = self.now_s()

        if self.state == "spinup":
            self._command(self.cruise)
            if self.speed >= 0.9 * self.cruise:
                self._enter("cruise")
            elif self._elapsed() > 30.0:
                self.get_logger().error("never reached cruise speed; killing anyway")
                self._enter("cruise")
        elif self.state == "cruise":
            self._command(self.cruise)
            if self._elapsed() >= self.cruise_hold:
                self._kill_gate()
        elif self.state == "observe":
            # KEEP COMMANDING. The upstream stack has no idea the gate died, and
            # that is exactly the scenario: a live planner, a dead safety node.
            self._command(self.cruise)
            if self.speed <= self.stopped_speed and self.stopped_at is None:
                self.stopped_at = self.now_s()
                self.stop_distance = self._travelled()
                self.get_logger().warn(
                    f"robot stopped {self.stop_distance:.3f} m after the kill, "
                    f"{self.stopped_at - self.kill_at:.2f} s later"
                )
            if self._elapsed() >= self.observe:
                self._command(0.0)
                try:
                    self._write_report()
                finally:
                    rclpy.shutdown()

    def _kill_gate(self):
        """SIGKILL every running SafetyGate process and record which."""
        self.kill_position = self.position
        self.kill_speed = self.speed
        self.kill_at = self.now_s()
        try:
            found = subprocess.run(
                ["pgrep", "-f", GATE_PROCESS_PATTERN],
                capture_output=True,
                text=True,
                check=False,
            )
            self.killed_pids = [
                line for line in found.stdout.split() if line.strip().isdigit()
            ]
            subprocess.run(["pkill", "-9", "-f", GATE_PROCESS_PATTERN], check=False)
        except OSError as error:  # pragma: no cover - platform dependent
            self.get_logger().error(f"could not kill the gate: {error}")
        self.get_logger().warn(
            f"SIGKILL sent to SafetyGate (pids {self.killed_pids or 'none found'}) "
            f"at {self.kill_speed:.3f} m/s"
        )
        self._enter("observe")

    # ------------------------------------------------------------------ report

    def _write_report(self):
        os.makedirs(self.results_dir, exist_ok=True)
        stem = os.path.join(self.results_dir, f"phase2_{self.tag}")
        travelled = self._travelled()
        stopped = self.stopped_at is not None
        lines = []
        add = lines.append
        rule = "=" * 78

        add(rule)
        add("PHASE 2 - fail-closed: SafetyGate SIGKILLed while the robot was moving")
        add(rule)
        add(f"robot:                    {self.robot_name}")
        add(
            f"plant-side watchdog:      "
            f"{'PRESENT' if self.with_watchdog else 'ABSENT  <- CONTROL RUN'}"
        )
        add(f"gate pids killed:         {', '.join(self.killed_pids) or 'none found'}")
        add("signal:                   SIGKILL (no shutdown handler runs)")
        add("upstream:                 kept commanding throughout")
        add("-" * 78)
        add(f"speed at the kill:                        {self.kill_speed:8.3f} m/s")
        add(f"peak speed during the run:                {self.peak_speed:8.3f} m/s")
        add(f"observation window after the kill:        {self.observe:8.1f} s")
        add(f"speed at the end of the window:           {self.speed:8.3f} m/s")
        add(f"distance travelled after the kill:        " f"{travelled:8.3f} m")
        if stopped:
            add(
                f"came to rest after:                       "
                f"{self.stop_distance:8.3f} m / "
                f"{self.stopped_at - self.kill_at:.2f} s"
            )
        else:
            add("came to rest:                             NO - still rolling")
        add("-" * 78)
        add("READING THIS")
        if self.with_watchdog:
            add("    With the watchdog present, the plant stops itself when the")
            add("    command stream goes quiet, which is what a real motor controller")
            add("    does. The gate being the only publisher of cmd_vel is what makes")
            add("    the stream go quiet; the watchdog is what acts on it.")
        else:
            add("    Without it, gz-sim's DiffDrive keeps integrating the velocity it")
            add("    last latched - it has no command timeout - so the robot rolls on")
            add("    with no software alive to stop it. This is the control, and it is")
            add("    the reason the watchdog exists rather than being assumed.")
        add("")
        add("    Both runs share the property that matters: no NEW command can reach")
        add("    the wheels once the gate is gone, because nothing else publishes the")
        add("    topic the plant listens to. A twist_mux in the same position would")
        add("    fail OPEN and pass the planner's commands straight through.")
        add(rule)

        text = "\n".join(lines)
        print(text, flush=True)
        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write("# Phase 2 - fail-closed SIGKILL test\n\n")
            handle.write("```\n")
            handle.write(text)
            handle.write("\n```\n")
        self.get_logger().info(f"wrote {stem}.md")


def main():
    rclpy.init()
    node = FailClosedRun()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
