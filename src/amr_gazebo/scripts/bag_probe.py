#!/usr/bin/env python3
"""Read a smoke-test bag offline and print attitude against position.

Exists so the smoke test 2 numbers can be re-derived from the recorded artifact without
re-running Gazebo, and so an attitude anomaly can be traced to raw IMU fields rather
than inferred from a summary.

Usage:
    ros2 run amr_gazebo bag_probe.py <bag_dir> [robot_name]
"""

import math
import sys

from nav_msgs.msg import Odometry
import rclpy.serialization
import rosbag2_py
from sensor_msgs.msg import Imu


def quaternion_to_roll_pitch(q):
    """Return (roll, pitch) in radians from a geometry_msgs quaternion."""
    sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
    cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return roll, math.asin(sin_pitch)


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else "rosbags/smoke2_ramp"
    robot = sys.argv[2] if len(sys.argv) > 2 else "amr1"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )

    imu_topic = f"/{robot}/imu"
    odom_topic = f"/{robot}/odom"

    rows = []
    last_odom = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == odom_topic:
            msg = rclpy.serialization.deserialize_message(data, Odometry)
            last_odom = msg.pose.pose.position.x
        elif topic == imu_topic:
            msg = rclpy.serialization.deserialize_message(data, Imu)
            roll, pitch = quaternion_to_roll_pitch(msg.orientation)
            rows.append(
                {
                    "t": stamp / 1e9,
                    "x": last_odom,
                    "pitch": pitch,
                    "roll": roll,
                    "q": msg.orientation,
                    "ax": msg.linear_acceleration.x,
                    "az": msg.linear_acceleration.z,
                }
            )

    if not rows:
        print(f"no IMU samples on {imu_topic} in {bag}")
        return

    t0 = rows[0]["t"]
    print(f"{len(rows)} IMU samples from {bag}")
    print(
        f"{'t':>7} {'odom_x':>8} {'pitch_q':>9} {'roll_q':>8} "
        f"{'acc_x':>8} {'acc_z':>8} {'pitch_acc':>10}"
    )
    for row in rows[:: max(1, len(rows) // 25)]:
        # Attitude implied by where gravity points, independent of the quaternion.
        pitch_acc = math.degrees(math.atan2(-row["ax"], row["az"]))
        x = row["x"] if row["x"] is not None else float("nan")
        print(
            f"{row['t'] - t0:7.2f} {x:8.3f} "
            f"{math.degrees(row['pitch']):9.3f} {math.degrees(row['roll']):8.3f} "
            f"{row['ax']:8.3f} {row['az']:8.3f} {pitch_acc:10.3f}"
        )

    peak_q = max(rows, key=lambda r: abs(r["pitch"]))
    peak_a = max(rows, key=lambda r: abs(math.atan2(-r["ax"], r["az"])))
    print()
    print(
        f"peak |pitch| from quaternion:        "
        f"{math.degrees(abs(peak_q['pitch'])):7.3f} deg at odom_x={peak_q['x']}"
    )
    print(
        f"peak |pitch| from gravity vector:    "
        f"{math.degrees(abs(math.atan2(-peak_a['ax'], peak_a['az']))):7.3f} deg "
        f"at odom_x={peak_a['x']}"
    )
    print(
        f"sample quaternion at peak: x={peak_q['q'].x:.5f} y={peak_q['q'].y:.5f} "
        f"z={peak_q['q'].z:.5f} w={peak_q['q'].w:.5f}"
    )


if __name__ == "__main__":
    main()
