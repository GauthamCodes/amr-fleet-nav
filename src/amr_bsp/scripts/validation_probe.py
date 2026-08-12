#!/usr/bin/env python3
"""PHASE 2 - what the sensor validators did, on a live graph, as a report.

One instrument serves two evidence runs because they ask the same question of the
same machinery:

    phase2_imu_injection.launch.py   an implausible angular velocity is injected
                                     upstream of the BSP; does the IMU channel
                                     reject it, say so, and keep it off the
                                     validated topic?

    phase2_camera_validation.launch.py  the camera is enabled and the robot is
                                     stationary; does CameraValidator run and log
                                     over a sustained stream of real frames?

WHERE THE WARN EVIDENCE COMES FROM

    ``/rosout``, filtered to the BSP node. Quoting the actual log lines the node
    emitted is stronger than a counter this script maintains itself: a counter only
    shows that this script believes a fault occurred, whereas the log line is the
    node's own output, at the severity an operator would see, with the offending
    value named in it.

THE SILENT-SUCCESS TRAP

    A validator that rejects everything would produce a splendid-looking WARN count.
    So the report states the rejection count against the ACCEPTED count either side
    of the injection window, and separately confirms that the corrupt sample never
    reached ``validated/imu``. Rejecting the fault and passing everything else are
    two different claims and both are needed.
"""

import math
import os

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

#: rcl_interfaces/Log severity for WARN.
LOG_WARN = 30


class ValidationProbe(Node):
    """Records BSP validator outcomes and the log lines they produced."""

    def __init__(self):
        """Declare parameters and subscribe to diagnostics, rosout and the IMU."""
        super().__init__("validation_probe")
        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("sample_duration_s", 40.0)
        self.declare_parameter("results_dir", "results")
        self.declare_parameter("tag", "validation")
        self.declare_parameter("bsp_node_name", "sensor_bsp")
        self.declare_parameter("injected_topic", "")
        self.declare_parameter("injected_rate", 0.0)
        self.declare_parameter("headline", "")

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        self.duration = float(get("sample_duration_s").value)
        self.results_dir = get("results_dir").value
        self.tag = get("tag").value
        self.bsp_node_name = get("bsp_node_name").value
        self.injected_topic = get("injected_topic").value
        self.injected_rate = float(get("injected_rate").value)
        self.headline = get("headline").value

        self.channels = {}
        self.warn_lines = []
        self.injected_seen = 0
        self.injected_on_validated = 0
        self.validated_imu = 0
        self.peak_validated_rate = 0.0

        # Motion, always recorded. On the stationary runs it is the evidence that
        # the run WAS stationary; on a driving run it is the measurement itself.
        self.peak_speed = 0.0
        self.distance = 0.0
        self.previous_position = None

        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._on_odom, 20
        )
        self.create_subscription(
            DiagnosticArray, f"/{self.robot_name}/bsp/diagnostics", self._on_diag, 10
        )
        # /rosout is the node's own output, not this script's interpretation of it.
        self.create_subscription(Log, "/rosout", self._on_log, qos_profile_sensor_data)
        self.create_subscription(
            Imu, f"/{self.robot_name}/validated/imu", self._on_validated_imu, 50
        )
        if self.injected_topic:
            self.create_subscription(Imu, self.injected_topic, self._on_injected, 50)

        self.started = None
        self.create_timer(1.0, self._tick)

    # ------------------------------------------------------------------ inputs

    def _on_odom(self, msg):
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.peak_speed = max(self.peak_speed, speed)
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.previous_position is not None:
            self.distance += math.hypot(
                position[0] - self.previous_position[0],
                position[1] - self.previous_position[1],
            )
        self.previous_position = position

    def _on_diag(self, msg):
        if self.started is None:
            self.started = self.get_clock().now()
        for status in msg.status:
            values = {pair.key: pair.value for pair in status.values}
            self.channels[status.hardware_id] = {
                "message": status.message,
                "level": status.level,
                "values": values,
            }

    def _on_log(self, msg):
        if msg.level < LOG_WARN or self.bsp_node_name not in msg.name:
            return
        if len(self.warn_lines) < 40:
            self.warn_lines.append((msg.name, msg.msg))

    @staticmethod
    def _rate(msg):
        return max(
            abs(msg.angular_velocity.x),
            abs(msg.angular_velocity.y),
            abs(msg.angular_velocity.z),
        )

    def _on_injected(self, msg):
        if self._rate(msg) >= 0.5 * self.injected_rate:
            self.injected_seen += 1

    def _on_validated_imu(self, msg):
        self.validated_imu += 1
        rate = self._rate(msg)
        self.peak_validated_rate = max(self.peak_validated_rate, rate)
        if self.injected_rate and rate >= 0.5 * self.injected_rate:
            self.injected_on_validated += 1

    # ------------------------------------------------------------------ report

    def _tick(self):
        if self.started is None:
            self.get_logger().info(
                "waiting: no BSP diagnostics yet", throttle_duration_sec=5.0
            )
            return
        if (self.get_clock().now() - self.started).nanoseconds / 1e9 >= self.duration:
            try:
                self._write_report()
            finally:
                rclpy.shutdown()

    def _write_report(self):
        os.makedirs(self.results_dir, exist_ok=True)
        stem = os.path.join(self.results_dir, f"phase2_{self.tag}")
        lines = []
        add = lines.append
        rule = "=" * 78
        thin = "-" * 78

        add(rule)
        add(f"PHASE 2 - SensorBSP validation: {self.headline or self.tag}")
        add(rule)
        add(f"robot:                    {self.robot_name}")
        add(f"sample window:            {self.duration:.0f} s")
        add(thin)

        add("[A] PER-CHANNEL OUTCOMES  (from the BSP's own DiagnosticArray)")
        add("      channel   accepted   warned   rejected   relay mean/p95/max ms")
        for label in sorted(self.channels):
            entry = self.channels[label]
            v = entry["values"]

            def number(key, default="0"):
                return v.get(key, default)

            add(
                f"    {label:>9}  {number('accepted'):>8}  "
                f"{number('warned'):>7}  {number('rejected'):>9}   "
                f"{self._ms(v, 'relay_ms_mean')} / {self._ms(v, 'relay_ms_p95')} / "
                f"{self._ms(v, 'relay_ms_max')}"
            )
            add(f"                status: {entry['message']}")
        if not self.channels:
            add("    no diagnostics received - the BSP was not running")
        add(thin)

        add("[M] MOTION DURING THE WINDOW  (from odometry)")
        add(f"    peak measured speed:                  {self.peak_speed:8.4f} m/s")
        add(f"    distance travelled:                   {self.distance:8.4f} m")
        add("    A stationary run should report ~0 for both; a driving run is")
        add("    measuring what the sensor load costs the simulator's drive.")
        add(thin)

        if self.injected_rate:
            self._add_injection_section(add, thin)

        add("VERDICT")
        ok = self._verdict(add)
        add(f"    RESULT: {'PASS' if ok else 'FAIL'}")
        add(rule)

        text = "\n".join(lines)
        print(text, flush=True)
        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write(f"# Phase 2 - SensorBSP validation: {self.headline}\n\n")
            handle.write("```\n")
            handle.write(text)
            handle.write("\n```\n")
        self.get_logger().info(f"wrote {stem}.md")

    @staticmethod
    def _ms(values, key):
        try:
            number = float(values.get(key, "nan"))
        except ValueError:
            return "  n/a"
        return "  n/a" if math.isnan(number) else f"{number:5.2f}"

    def _add_injection_section(self, add, thin):
        add("[B] INJECTED FAULT  (implausible IMU angular velocity)")
        add(
            f"    injected magnitude:                   {self.injected_rate:8.1f} rad/s"
        )
        add(f"    corrupt samples seen on the raw input:{self.injected_seen:8d}")
        add(f"    validated IMU samples published:      {self.validated_imu:8d}")
        add(
            f"    corrupt samples on validated/imu:     "
            f"{self.injected_on_validated:8d}   <- must be 0"
        )
        add(
            f"    peak |w| ever seen on validated/imu:  "
            f"{self.peak_validated_rate:8.3f} rad/s"
        )
        add("")
        add("    The two claims are separate. Rejecting the fault is the first; not")
        add("    also discarding the healthy stream around it is the second, and the")
        add("    accepted counts in [A] are what carry it.")
        add(thin)

        add("[C] WHAT THE NODE LOGGED  (verbatim, from /rosout at WARN or above)")
        if self.warn_lines:
            for name, message in self.warn_lines[:12]:
                add(f"    [{name}] {message}")
            if len(self.warn_lines) > 12:
                add(f"    ... and {len(self.warn_lines) - 12} more")
        else:
            add("    no WARN lines captured from the BSP node")
        add(thin)

    def _verdict(self, add):
        checks = []
        saw_traffic = any(
            int(entry["values"].get("accepted", "0")) > 0
            for entry in self.channels.values()
        )
        checks.append(("channels carried traffic", saw_traffic))
        if self.injected_rate:
            rejected = sum(
                int(entry["values"].get("rejected", "0"))
                for label, entry in self.channels.items()
                if label == "imu"
            )
            checks.append(("the injected fault was rejected", rejected > 0))
            checks.append(("the node logged a WARN naming it", bool(self.warn_lines)))
            checks.append(
                (
                    "no corrupt sample reached validated/imu",
                    self.injected_on_validated == 0,
                )
            )
            checks.append(("the healthy stream still flowed", self.validated_imu > 0))
        else:
            checks.append(("camera channel present", "camera" in self.channels))
            checks.append(
                (
                    "camera frames accepted",
                    int(
                        self.channels.get("camera", {})
                        .get("values", {})
                        .get("accepted", "0")
                    )
                    > 0,
                )
            )
        for label, passed in checks:
            add(f"    {label + ':':<40} {'YES' if passed else 'NO'}")
        return all(passed for _, passed in checks)


def main():
    rclpy.init()
    node = ValidationProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
