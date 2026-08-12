#!/usr/bin/env python3
"""Measure whether wheel odometry and the robot's own frame agree under rotation.

THE QUESTION

A differential drive rotates about the midpoint of its wheel axle, and Gazebo's
DiffDrive computes odometry from wheel encoders alone - gz::math::DiffDriveOdometry
takes only wheel separation and radii - so the pose it integrates IS the axle midpoint.
It then labels that pose child_frame_id = <robot>/base_footprint. If base_footprint is
anywhere other than the axle, odometry and TF describe two different points.

Pure translation hides such an error completely, because a constant frame offset
cancels. That is why Phase 0's smoke test 2 measured -0.000 m agreement over a straight
ramp traverse and saw nothing. Rotation is where it shows.

WHAT THIS FOUND

Against the original Phase 0 geometry (body centred on the frame origin, wheels 0.14 m
aft), wheel odometry reported 0.0000 m of displacement while ground truth showed the
robot frame sweeping 0.2428 m - a 0.24 m disagreement against a 0.02 m tolerance. The
frame origin was moved onto the drive axle in response; the same test then measured
0.0385 m, of which only 0.0193 m is an apparent offset.

That residual is PHYSICAL, not a modelling error, and the two runs prove it: the
effective pivot sat ~0.019 m forward of the geometric axle BOTH times (0.1214 against a
true 0.14, then 0.0193 against a true 0.0). A frictionless caster and finite wheel
friction shift the effective rotation centre forward by a constant amount that no choice
of frame can remove. It is sub-cell against the 0.05 m map resolution.

DECISION RULE (fixed in advance so the result cannot be rationalised)
    |implied offset - offset the URDF intends| >  0.02 m -> frame error, fix the URDF.
    |implied offset - offset the URDF intends| <= 0.02 m -> no frame error; the residual
        is physical drift, to be judged against map resolution.
"""

import math
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    """Return yaw in radians from a quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class OdomRotationCheck(Node):
    """Spin the robot in place and compare odometry against simulator ground truth."""

    def __init__(self):
        """Declare parameters and subscribe to odometry and ground truth."""
        super().__init__("odom_rotation_check")

        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("yaw_rate", 0.6)
        self.declare_parameter("settle_s", 3.0)
        self.declare_parameter("spin_s", 22.0)
        self.declare_parameter("expected_offset", 0.14)
        self.declare_parameter("tolerance", 0.02)
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("report_path", "")

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        self.yaw_rate = float(get("yaw_rate").value)
        self.settle_s = float(get("settle_s").value)
        self.spin_s = float(get("spin_s").value)
        self.expected_offset = float(get("expected_offset").value)
        self.tolerance = float(get("tolerance").value)
        self.map_resolution = float(get("map_resolution").value)
        self.report_path = get("report_path").value

        self.publisher = self.create_publisher(Twist, f"/{self.robot_name}/cmd_vel", 10)
        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._on_odom, 20
        )
        self.create_subscription(TFMessage, "/gz_ground_truth", self._on_truth, 20)

        self.odom = None
        self.truth = None
        self.odom0 = None
        self.truth0 = None
        self.samples = []
        self.started = None
        self.done = False

        self.create_timer(0.05, self._tick)
        self.get_logger().info("odom rotation check armed")

    # ---------------------------------------------------------------- inputs

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def _on_truth(self, msg):
        for t in msg.transforms:
            if t.child_frame_id != self.robot_name:
                continue
            p = t.transform.translation
            self.truth = (p.x, p.y, yaw_of(t.transform.rotation))

    # ------------------------------------------------------------------ run

    def _elapsed(self):
        if self.started is None:
            return 0.0
        return (self.get_clock().now() - self.started).nanoseconds / 1e9

    def _tick(self):
        if self.odom is None or self.truth is None or self.done:
            return
        if self.started is None:
            self.started = self.get_clock().now()

        t = self._elapsed()
        twist = Twist()

        if t < self.settle_s:
            self.publisher.publish(twist)
            # Re-latch the origin each tick until the spin starts, so any settling
            # motion is excluded from the measurement.
            self.odom0, self.truth0 = self.odom, self.truth
            return

        if t < self.settle_s + self.spin_s:
            twist.angular.z = self.yaw_rate
            self.publisher.publish(twist)
            self.samples.append(
                {
                    "t": t - self.settle_s,
                    "odom_dx": self.odom[0] - self.odom0[0],
                    "odom_dy": self.odom[1] - self.odom0[1],
                    "truth_dx": self.truth[0] - self.truth0[0],
                    "truth_dy": self.truth[1] - self.truth0[1],
                    "odom_dyaw": self._unwrap(self.odom[2] - self.odom0[2]),
                    "truth_dyaw": self._unwrap(self.truth[2] - self.truth0[2]),
                }
            )
            return

        self.publisher.publish(twist)
        self.done = True
        self._report()
        rclpy.shutdown()

    @staticmethod
    def _unwrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    # --------------------------------------------------------------- report

    def _report(self):
        rows = self.samples
        lines = []
        add = lines.append

        if not rows:
            add("no samples captured")
            print("\n".join(lines), flush=True)
            return

        # Displacement of the robot's own frame, per source.
        odom_disp = [math.hypot(r["odom_dx"], r["odom_dy"]) for r in rows]
        truth_disp = [math.hypot(r["truth_dx"], r["truth_dy"]) for r in rows]
        # The quantity that matters: how far apart the two say the robot is.
        error = [
            math.hypot(r["truth_dx"] - r["odom_dx"], r["truth_dy"] - r["odom_dy"])
            for r in rows
        ]
        peak = max(error)
        peak_row = rows[error.index(peak)]

        turns = sum(
            1
            for a, b in zip(rows, rows[1:])
            if abs(self._unwrap(b["truth_dyaw"] - a["truth_dyaw"])) > math.pi / 2
        )

        add("")
        add("=" * 78)
        add("ODOMETRY INTEGRITY UNDER ROTATION")
        add("=" * 78)
        add(f"robot:                    {self.robot_name}")
        add(
            f"commanded yaw rate:       {self.yaw_rate:.2f} rad/s "
            f"for {self.spin_s:.0f} s"
        )
        add(f"samples:                  {len(rows)}")
        add(
            f"total rotation observed:  "
            f"{math.degrees(self.yaw_rate * self.spin_s):.0f} deg commanded "
            f"({turns} half-turn wraps seen in ground truth)"
        )
        add("-" * 78)
        add("DISPLACEMENT OF THE ROBOT FRAME DURING A PURE IN-PLACE ROTATION")
        add(f"  peak per wheel odometry:            {max(odom_disp):8.4f} m")
        add(f"  peak per Gazebo ground truth:       {max(truth_disp):8.4f} m")
        add(
            f"  predicted if axle is offset:        "
            f"{2.0 * self.expected_offset:8.4f} m  (2 x {self.expected_offset:.2f})"
        )
        add("-" * 78)
        add("DISAGREEMENT BETWEEN THE TWO")
        add(f"  peak |truth - odom|:                {peak:8.4f} m")
        add(
            f"  at t = {peak_row['t']:.2f} s, after "
            f"{math.degrees(abs(peak_row['truth_dyaw'])):.0f} deg of yaw"
        )
        add(
            f"  mean |truth - odom|:                "
            f"{sum(error) / len(error):8.4f} m"
        )
        add(f"  implied frame offset (peak / 2):    {peak / 2.0:8.4f} m")
        add(f"  expected offset from the URDF:      " f"{self.expected_offset:8.4f} m")
        add("-" * 78)

        # Two different things can produce a displacement here and they have very
        # different remedies, so they are judged separately.
        #
        #   SYSTEMATIC - base_footprint is not where the robot actually pivots. Shows up
        #   as an implied offset matching the URDF geometry. Fixed by moving the frame.
        #
        #   PHYSICAL - the robot really does drift while spinning, because the caster
        #   drags and the wheels slip. Shows up as a residual that no frame definition
        #   can remove. Judged against map resolution, since that is what it degrades.
        implied = peak / 2.0
        systematic = abs(implied - self.expected_offset)

        add("VERDICT")
        add(f"  implied rotation-centre offset:     {implied:8.4f} m")
        add(f"  offset the URDF intends:            " f"{self.expected_offset:8.4f} m")
        add(f"  unexplained systematic component:   {systematic:8.4f} m")
        add(f"  decision threshold:                 {self.tolerance:8.4f} m")
        add("")
        if systematic > self.tolerance:
            add(f"  The measured pivot is {systematic:.4f} m from where the URDF")
            add("  places base_footprint - beyond tolerance. Wheel odometry and TF")
            add("  describe different points, so sensors rotate about the wrong")
            add("  centre and the map smears on every in-place turn.")
            add("  -> ACTION: move base_link/base_footprint onto the drive axle")
            add("     (wheels to x=0; body, caster, laser and imu shifted forward),")
            add("     then re-run smoke test 1 whose lidar_x_offset changes with it.")
            verdict = "SYSTEMATIC FRAME ERROR - FIX THE URDF"
        else:
            add(f"  The measured pivot is within {self.tolerance:.4f} m of where the")
            add("  URDF places base_footprint, so there is no frame-definition error.")
            add(f"  The residual {peak:.4f} m peak excursion is PHYSICAL: a")
            add("  frictionless caster and finite wheel friction shift the rotation")
            add("  centre slightly forward of the axle. A real robot does this too.")
            add(
                f"  Against the {self.map_resolution:.2f} m map resolution it is "
                f"{'sub-cell' if peak < self.map_resolution else 'ABOVE one cell'}"
                " - scan matching absorbs it."
            )
            verdict = "NO FRAME ERROR - RESIDUAL IS PHYSICAL DRIFT"
        add(f"  RESULT: {verdict}")
        add("=" * 78)
        add("")

        text = "\n".join(lines)
        print(text, flush=True)
        self.get_logger().info(f"RESULT: {verdict}")

        if self.report_path:
            os.makedirs(
                os.path.dirname(os.path.abspath(self.report_path)), exist_ok=True
            )
            with open(self.report_path, "w", encoding="utf-8") as handle:
                handle.write("# Odometry integrity under rotation\n\n```\n")
                handle.write(text)
                handle.write("\n```\n")


def main():
    rclpy.init()
    node = OdomRotationCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
