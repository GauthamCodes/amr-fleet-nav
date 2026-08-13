"""The per-robot SensorBSP node: three validators and the PitchGate, in one relay.

Everything that differs between robots - the mast height, the plausible turn rate,
the camera resolution, the PitchGate floor - arrives as a parameter computed by the
launch file from fleet.yaml. There is no robot name in this file (ENGINEERING_NOTES rule
5).

HOW PITCHGATE GETS ITS ANGLE, AND AT WHAT RATE

    From the VALIDATED IMU, at 100 Hz, into a short ring buffer. Each 10 Hz scan then
    takes the buffered sample NEAREST ITS OWN header stamp - not the most recent one
    received. That distinction is the whole point of the buffer: a scan is stamped
    when it was acquired and arrives some milliseconds later, so "latest IMU" is
    systematically the wrong attitude, by more at higher pitch rates.

    If no IMU sample lies within ``imu_max_age`` of the scan stamp, the scan is
    republished UNTRUNCATED with a throttled warning. That direction is deliberate.
    Failing to truncate is degraded but safe: the phantom it lets through sits at
    ~2.8 m, far outside any d_safe this fleet produces. Truncating on a guessed
    attitude is neither - it would delete real returns, starve SLAM, and in the worst
    case reach inside the braking envelope. When the attitude is unknown, the honest
    move is to do less, and say so.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, LaserScan

from amr_bsp.pitch_gate import (
    deadband_pitch,
    DEFAULT_MARGIN,
    quaternion_to_roll_pitch,
    truncate_scan,
)
from amr_bsp.sensor_bsp import SensorBSP, SensorChannel
from amr_bsp.topics import (
    RAW_CAMERA,
    RAW_IMU,
    RAW_SCAN,
    VALIDATED_CAMERA,
    VALIDATED_IMU,
    VALIDATED_SCAN,
)
from amr_bsp.validators import validate_camera, validate_imu, validate_lidar


class BspNode:
    """Composes the validated sensor channels and the PitchGate onto one node.

    Written as a plain class taking the node rather than subclassing ``Node`` so the
    same composition can be dropped into a component container later without
    inheriting a node identity it does not own.
    """

    def __init__(self, node):
        """Declare parameters, build the pitch buffer, and wire every channel."""
        self.node = node
        declare = node.declare_parameter

        self.lidar_height = declare("lidar_height", 0.35).value
        self.gate_margin = declare("gate_margin", DEFAULT_MARGIN).value
        self.min_gate_range = declare("min_gate_range", 0.0).value
        self.imu_max_age = declare("imu_max_age", 0.15).value
        self.range_max_hint = declare("range_max_hint", 12.0).value

        self.max_angular_rate = declare("max_angular_rate", 4.0).value
        self.max_linear_accel = declare("max_linear_accel", 29.42).value
        self.imu_stale_s = declare("imu_stale_s", 0.5).value
        self.scan_stale_s = declare("scan_stale_s", 0.5).value
        self.camera_stale_s = declare("camera_stale_s", 1.0).value
        self.max_nan_fraction = declare("max_nan_fraction", 0.25).value

        self.camera_enabled = declare("camera_enabled", True).value
        self.camera_width = declare("camera_width", 160).value
        self.camera_height = declare("camera_height", 120).value

        raw_scan = declare("raw_scan_topic", RAW_SCAN).value
        raw_imu = declare("raw_imu_topic", RAW_IMU).value
        raw_camera = declare("raw_camera_topic", RAW_CAMERA).value

        # A short buffer: 0.5 s at 100 Hz. Long enough to bracket any scan stamp,
        # short enough that a stale entry can never be the nearest one.
        self.pitch_buffer = []
        self.pitch_buffer_span = 0.5

        self.last_pitch = 0.0
        self.last_gate_used = float("inf")
        self.truncated_total = 0
        self.truncated_last = 0
        self.scans_gated = 0
        self.scans_without_attitude = 0

        self.bsp = SensorBSP(node)
        self.bsp.add(
            SensorChannel(
                node,
                "imu",
                Imu,
                raw_imu,
                VALIDATED_IMU,
                self._validate_imu,
                observe=self._note_attitude,
                depth=20,
            )
        )
        self.bsp.add(
            SensorChannel(
                node,
                "lidar",
                LaserScan,
                raw_scan,
                VALIDATED_SCAN,
                self._validate_lidar,
                transform=self._apply_pitch_gate,
                extra_values=self._pitch_gate_values,
            )
        )
        if self.camera_enabled:
            self.bsp.add(
                SensorChannel(
                    node,
                    "camera",
                    Image,
                    raw_camera,
                    VALIDATED_CAMERA,
                    self._validate_camera,
                    depth=2,
                )
            )

        deadband = deadband_pitch(
            self.lidar_height, self.range_max_hint, self.gate_margin
        )
        node.get_logger().info(
            f"PitchGate: h={self.lidar_height:.3f} m, margin={self.gate_margin:.2f}, "
            f"floor={self.min_gate_range:.3f} m, deadband={math.degrees(deadband):.2f} "
            f"deg (below it the gate exceeds range_max and is a no-op)"
        )
        if self.min_gate_range <= 0.0:
            # Loud, because the failure is silent otherwise: truncation would still
            # work, and would still look correct, with its safety interlock absent.
            node.get_logger().warn(
                "min_gate_range is 0.0 - PitchGate is running WITHOUT its braking "
                "envelope interlock. amr_bringup supplies this from "
                "amr_safety.safety_model.min_gate_range; a bare bsp.launch.py does "
                "not."
            )

    # ------------------------------------------------------------- validation

    def _validate_imu(self, msg, age, since_previous):
        return validate_imu(
            msg,
            self.max_angular_rate,
            self.max_linear_accel,
            age=age,
            max_age=self.imu_stale_s,
            since_previous=since_previous,
        )

    def _validate_lidar(self, msg, age, since_previous):
        return validate_lidar(
            msg,
            max_nan_fraction=self.max_nan_fraction,
            age=age,
            max_age=self.scan_stale_s,
            since_previous=since_previous,
        )

    def _validate_camera(self, msg, age, since_previous):
        return validate_camera(
            msg,
            self.camera_width,
            self.camera_height,
            age=age,
            max_age=self.camera_stale_s,
            since_previous=since_previous,
        )

    # -------------------------------------------------------------- attitude

    def _note_attitude(self, msg):
        """Record (stamp, pitch) from a validated IMU sample."""
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        _, pitch = quaternion_to_roll_pitch(msg.orientation)
        self.pitch_buffer.append((stamp, pitch))
        cutoff = stamp - self.pitch_buffer_span
        while self.pitch_buffer and self.pitch_buffer[0][0] < cutoff:
            self.pitch_buffer.pop(0)

    def _pitch_at(self, stamp):
        """Return the buffered pitch nearest a stamp, or None if none is close."""
        if not self.pitch_buffer:
            return None
        best_stamp, best_pitch = min(
            self.pitch_buffer, key=lambda sample: abs(sample[0] - stamp)
        )
        if abs(best_stamp - stamp) > self.imu_max_age:
            return None
        return best_pitch

    # ------------------------------------------------------------- pitch gate

    def _apply_pitch_gate(self, msg):
        """Return the scan with ground returns truncated, header carried across."""
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        pitch = self._pitch_at(stamp)
        if pitch is None:
            self.scans_without_attitude += 1
            self.node.get_logger().warn(
                "no IMU attitude within "
                f"{self.imu_max_age:.3f} s of the scan stamp; republishing "
                "UNTRUNCATED (degraded, not unsafe - see bsp_node docstring)",
                throttle_duration_sec=5.0,
            )
            return msg

        gated, truncated = truncate_scan(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            self.lidar_height,
            pitch,
            margin=self.gate_margin,
            floor=self.min_gate_range,
        )
        self.last_pitch = pitch
        self.truncated_last = truncated
        self.truncated_total += truncated
        self.scans_gated += 1
        if truncated == 0:
            return msg

        out = LaserScan()
        # RULE 4. This assignment is the whole reason the scan is rebuilt by hand
        # instead of being constructed from scratch: the ORIGINAL acquisition stamp
        # and frame travel with the data.
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = gated
        out.intensities = msg.intensities
        return out

    def _pitch_gate_values(self):
        """Return the PitchGate state that rides along with the lidar diagnostics."""
        return {
            "pitch_deg": math.degrees(self.last_pitch),
            "truncated_last": self.truncated_last,
            "truncated_total": self.truncated_total,
            "scans_gated": self.scans_gated,
            "scans_without_attitude": self.scans_without_attitude,
            "gate_floor_m": self.min_gate_range,
        }

    def log_summary(self):
        """Log the per-channel and PitchGate totals. Called on shutdown."""
        self.bsp.log_summary()
        self.node.get_logger().info(
            f"pitchgate  scans {self.scans_gated}  beams truncated "
            f"{self.truncated_total}  scans without attitude "
            f"{self.scans_without_attitude}"
        )


class SensorBspNode(Node):
    """The node the launch file runs: one BSP composition, one lifetime."""

    def __init__(self):
        """Create the node and compose the validated sensor channels onto it."""
        super().__init__("sensor_bsp")
        self.composition = BspNode(self)


def main():
    """Spin the BSP, and report the relay's own accounting on the way out."""
    rclpy.init()
    node = SensorBspNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.composition.log_summary()


if __name__ == "__main__":
    main()
