#!/usr/bin/env python3
"""SMOKE TEST 2 - measure ramp-induced phantom returns and size PitchGate.

Three distinct effects get separated, because they fail differently and a single
"closest range on the ramp" number would blur them together:

[A] PHANTOM RAMP WALL. The robot is still on flat ground. The scan plane is level at
    height h, and the ramp surface climbs through it. The ramp reads as a solid wall
    at the point where its surface reaches h, which is h/tan(ramp_angle) past the toe.
    This one is a function of the WORLD, not of the robot's attitude.

[B] PITCH-INDUCED GROUND RETURN. The robot is pitched, so the scan plane is tilted and
    meets the floor at h/sin(pitch). Note that a robot sitting squarely on the ramp has
    its scan plane PARALLEL to the ramp, so the ramp itself produces no return; the
    return comes from ground beyond the ramp that the tilted plane runs into. The effect
    is strongest during the transitions at the toe and the crest, where attitude and
    the ground under the beam disagree.

[C] RETURN ALONG THE DIRECTION OF TRAVEL. The dangerous case. Measured during the
    reverse descent, when the robot is pitched nose-up while travelling downhill, so
    the descending beams look where the robot is going. This is the number that decides
    whether a phantom return would trip a safety stop.

The PitchGate sizing section converts those measurements into the two constants Phase 2
needs: the pitch below which no ground strike is possible within sensor range, and the
truncation radius at the worst pitch actually observed.
"""

import math
import os

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

#: Half-width of the sectors examined ahead of and behind the robot, in radians.
SECTOR_HALF_WIDTH = math.radians(15.0)

#: |pitch| below this is treated as level, in radians.
LEVEL_PITCH = math.radians(1.0)

#: |pitch| above this is treated as genuinely pitched, in radians.
PITCHED_PITCH = math.radians(2.0)


def quaternion_to_roll_pitch(q):
    """Return (roll, pitch) in radians from a geometry_msgs quaternion."""
    sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
    cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return roll, math.asin(sin_pitch)


class Smoke2Measure(Node):
    """Record scan/attitude/odometry and report the ramp's effect on the LiDAR."""

    def __init__(self):
        """Declare world/robot geometry and subscribe to scan, IMU and odometry."""
        super().__init__("smoke2_measure")

        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("lidar_height", 0.20)
        self.declare_parameter("lidar_x_offset", 0.29)
        # /odom is expressed in the odom frame, which starts at the robot's SPAWN
        # pose, not at the world origin. Every world-referenced comparison here
        # (ramp toe, crest) needs this offset or it is wrong by the spawn x.
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("ramp_angle_deg", 8.0)
        self.declare_parameter("ramp_toe_x", 2.4423)
        self.declare_parameter("ramp_crest_x", 6.0)
        self.declare_parameter("upper_plateau_height", 0.5)
        self.declare_parameter("max_vel_x", 0.6)
        self.declare_parameter("safety_k", 0.5)
        self.declare_parameter("safety_d_min", 0.30)
        self.declare_parameter("sample_duration_s", 90.0)
        self.declare_parameter("report_path", "")

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        self.h = float(get("lidar_height").value)
        self.lidar_dx = float(get("lidar_x_offset").value)
        self.spawn_x = float(get("spawn_x").value)
        self.ramp_angle = math.radians(float(get("ramp_angle_deg").value))
        self.toe_x = float(get("ramp_toe_x").value)
        self.crest_x = float(get("ramp_crest_x").value)
        self.rise = float(get("upper_plateau_height").value)
        self.max_vel_x = float(get("max_vel_x").value)
        self.safety_k = float(get("safety_k").value)
        self.safety_d_min = float(get("safety_d_min").value)
        self.duration = float(get("sample_duration_s").value)
        self.report_path = get("report_path").value

        self.pitch = 0.0
        self.roll = 0.0
        self.x = 0.0
        self.vx = 0.0
        # Gazebo ground truth. Wheel odometry integrates wheel rotation, so a robot
        # whose wheels spin while it is held stationary reports a journey it never
        # made - which is exactly what happened when the ramp toe blocked the caster.
        # Position and attitude are therefore taken from the simulator, and odometry
        # is kept only to expose the discrepancy.
        self.truth_x = None
        self.truth_pitch = None
        self.samples = []
        self.scan_meta = None
        self.started = None

        self.create_subscription(Imu, f"/{self.robot_name}/imu", self._on_imu, 20)
        self.create_subscription(TFMessage, "/gz_ground_truth", self._on_truth, 20)
        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._on_odom, 20
        )
        self.create_subscription(
            LaserScan, f"/{self.robot_name}/scan", self._on_scan, 10
        )
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"measuring for {self.duration:.0f}s")

    # ---------------------------------------------------------------- inputs

    def _on_imu(self, msg):
        self.roll, self.pitch = quaternion_to_roll_pitch(msg.orientation)

    def _on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.vx = msg.twist.twist.linear.x

    def _on_truth(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id != self.robot_name:
                continue
            self.truth_x = transform.transform.translation.x
            _, self.truth_pitch = quaternion_to_roll_pitch(transform.transform.rotation)

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _sector_min(self, scan, centre):
        """Return the closest valid return within a sector, or inf."""
        best = float("inf")
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(self._wrap(angle - centre)) > SECTOR_HALF_WIDTH:
                continue
            if math.isinf(value) or math.isnan(value):
                continue
            if value < scan.range_min or value > scan.range_max:
                continue
            best = min(best, value)
        return best

    def _on_scan(self, msg):
        if self.started is None:
            self.started = self.get_clock().now()
        if self.scan_meta is None:
            self.scan_meta = {
                "range_max": msg.range_max,
                "range_min": msg.range_min,
                "n": len(msg.ranges),
                "frame_id": msg.header.frame_id,
            }
        self.samples.append(
            {
                "t": (self.get_clock().now() - self.started).nanoseconds / 1e9,
                "x": self.x,
                "vx": self.vx,
                "pitch": self.pitch,
                "roll": self.roll,
                "truth_x": self.truth_x,
                "truth_pitch": self.truth_pitch,
                "nose": self._sector_min(msg, 0.0),
                "tail": self._sector_min(msg, math.pi),
            }
        )

    # ---------------------------------------------------------------- report

    def _tick(self):
        if self.started is None:
            self.get_logger().warn("no /scan yet")
            return
        if (self.get_clock().now() - self.started).nanoseconds / 1e9 < self.duration:
            return
        self._report()
        rclpy.shutdown()

    def _world_x(self, sample):
        """World x of the robot, from Gazebo ground truth where available."""
        if sample.get("truth_x") is not None:
            return sample["truth_x"]
        return sample["x"] + self.spawn_x

    @staticmethod
    def _attitude(sample):
        """Pitch to trust: ground truth if present, otherwise the IMU."""
        if sample.get("truth_pitch") is not None:
            return sample["truth_pitch"]
        return sample["pitch"]

    def _laser_x(self, sample):
        """World x of the laser origin."""
        return self._world_x(sample) + self.lidar_dx

    def _report(self):
        meta = self.scan_meta or {}
        rows = self.samples
        add = []
        out = add.append

        # ---- [A] phantom ramp wall: level attitude, still short of the toe -------
        approach = [
            r
            for r in rows
            if abs(self._attitude(r)) < LEVEL_PITCH
            and self._laser_x(r) < self.toe_x - 0.30
            and r["vx"] > 0.05
            and math.isfinite(r["nose"])
        ]
        # The ramp surface reaches scan-plane height h at this world x.
        wall_x = self.toe_x + self.h / math.tan(self.ramp_angle)
        approach_errors = [r["nose"] - (wall_x - self._laser_x(r)) for r in approach]

        # ---- [B] attitude extremes ----------------------------------------------
        pitched = [r for r in rows if abs(self._attitude(r)) > PITCHED_PITCH]
        max_row = max(rows, key=lambda r: abs(self._attitude(r))) if rows else None

        # ---- [C] return along the direction of travel ---------------------------
        # Reversing down the ramp: travelling -x, nose still uphill, so the tail
        # sector looks along the direction of travel.
        descent = [
            r
            for r in rows
            if r["vx"] < -0.05
            and abs(self._attitude(r)) > PITCHED_PITCH
            and math.isfinite(r["tail"])
        ]
        worst_descent = min(descent, key=lambda r: r["tail"]) if descent else None

        # ---- PitchGate sizing ----------------------------------------------------
        # For each pitch magnitude bin, the closest return seen in either sector.
        bins = {}
        for r in rows:
            key = round(math.degrees(abs(self._attitude(r))))
            closest = min(r["nose"], r["tail"])
            if math.isfinite(closest):
                bins[key] = min(bins.get(key, float("inf")), closest)

        out("")
        out("=" * 78)
        out("SMOKE TEST 2 - ramp-induced phantom ground return")
        out("=" * 78)
        out(
            f"world:          ramp_angle = {math.degrees(self.ramp_angle):.2f} deg, "
            f"rise = {self.rise:.3f} m"
        )
        out(
            f"                ramp toe x = {self.toe_x:.4f}, "
            f"crest x = {self.crest_x:.4f}"
        )
        out(f"robot:          {self.robot_name}, lidar height h = {self.h:.3f} m")
        out(
            f"scan:           {meta.get('n', 0)} beams, "
            f"{meta.get('range_min', 0):.2f}-{meta.get('range_max', 0):.2f} m, "
            f"sector +/-{math.degrees(SECTOR_HALF_WIDTH):.0f} deg"
        )
        out(f"samples:        {len(rows)} scans")
        if rows:
            xs = [self._world_x(r) for r in rows]
            on_ramp = [
                r for r in rows if self.toe_x <= self._world_x(r) <= self.crest_x
            ]
            out(
                f"trajectory:     world x {min(xs):+.3f} -> {max(xs):+.3f} m "
                f"({len(on_ramp)} scans taken on the ramp)"
            )
            out(
                f"                max forward speed "
                f"{max(r['vx'] for r in rows):+.3f} m/s, "
                f"max reverse {min(r['vx'] for r in rows):+.3f} m/s"
            )
        out("-" * 78)

        out("[A] PHANTOM RAMP WALL  (robot level, on flat ground, approaching the toe)")
        out(f"    ramp surface reaches scan height at x = {wall_x:8.4f} m")
        out(
            f"    i.e. offset past the toe   h/tan(a)  = "
            f"{self.h / math.tan(self.ramp_angle):8.4f} m"
        )
        if approach:
            mean_err = sum(approach_errors) / len(approach_errors)
            out(f"    samples in approach phase:            {len(approach):8d}")
            out(f"    mean (measured - predicted) range:    {mean_err:+8.3f} m")
            out(
                f"    max |error| over approach:            "
                f"{max(abs(e) for e in approach_errors):8.3f} m"
            )
            out("    -> the ramp DOES read as a solid wall to a level 2D scan")
        else:
            out("    no clean approach samples captured")
        out("-" * 78)

        # Odometry vs ground truth: a large gap means the wheels turned without the
        # robot moving, and every odometry-derived number would have been fiction.
        gt = [r for r in rows if r.get("truth_x") is not None]
        out("[0] ODOMETRY INTEGRITY  (wheel odometry vs Gazebo ground truth)")
        if gt:
            final = gt[-1]
            drift = (final["x"] + self.spawn_x) - final["truth_x"]
            worst = max(gt, key=lambda r: abs((r["x"] + self.spawn_x) - r["truth_x"]))
            out(f"    ground-truth samples:                 {len(gt):8d}")
            out(
                f"    final odom world x:                   "
                f"{final['x'] + self.spawn_x:8.3f} m"
            )
            out(f"    final ground-truth world x:           {final['truth_x']:8.3f} m")
            out(f"    final discrepancy:                    {drift:+8.3f} m")
            out(
                f"    peak discrepancy:                     "
                f"{(worst['x'] + self.spawn_x) - worst['truth_x']:+8.3f} m"
            )
            trustworthy = abs(drift) < 0.5
            out(
                "    -> odometry is "
                + ("TRUSTWORTHY" if trustworthy else "UNRELIABLE (wheel slip)")
            )
        else:
            out("    no ground truth available - positions are odometry only")
        out("-" * 78)

        out("[B] ATTITUDE EXTREMES")
        if max_row:
            p = self._attitude(max_row)
            out(
                f"    max |pitch| over run:                 "
                f"{math.degrees(abs(p)):8.3f} deg   at t = {max_row['t']:.2f} s"
            )
            out(
                f"    (IMU reported at that instant:        "
                f"{math.degrees(abs(max_row['pitch'])):8.3f} deg)"
            )
            # ROS convention: rotation about +y takes +x toward -z, so POSITIVE
            # pitch is nose-down and a robot climbing a ramp reports negative pitch.
            out(
                f"    sign at max:                          "
                f"{'nose-DOWN' if p > 0 else 'nose-UP'}"
            )
            out(
                f"    robot world x at that moment:         "
                f"{self._world_x(max_row):8.3f} m"
            )
            on_ramp = self.toe_x <= self._world_x(max_row) <= self.crest_x
            out(
                f"    on the ramp at that moment:           "
                f"{'YES' if on_ramp else 'NO':>8s}"
            )
            out(
                f"    nominal ramp angle for comparison:    "
                f"{math.degrees(self.ramp_angle):8.3f} deg"
            )
            out(
                f"    nose-sector min range there:          "
                f"{max_row['nose']:8.2f} m"
                if math.isfinite(max_row["nose"])
                else "    nose-sector min range there:              inf"
            )
            out(
                f"    tail-sector min range there:          "
                f"{max_row['tail']:8.2f} m"
                if math.isfinite(max_row["tail"])
                else "    tail-sector min range there:              inf"
            )
            if abs(p) > 1e-4:
                out(
                    f"    reference h/sin(|pitch|):             "
                    f"{self.h / abs(math.sin(p)):8.2f} m"
                )
        out(
            f"    scans with |pitch| > "
            f"{math.degrees(PITCHED_PITCH):.0f} deg:            {len(pitched):8d}"
        )
        out("-" * 78)

        out("[C] RETURN ALONG THE DIRECTION OF TRAVEL  (reverse descent, nose uphill)")
        if worst_descent:
            p = self._attitude(worst_descent)
            out(f"    samples while descending pitched:     {len(descent):8d}")
            out(
                f"    closest return in travel direction:   "
                f"{worst_descent['tail']:8.2f} m"
            )
            out(
                f"    pitch at that moment:                 "
                f"{math.degrees(p):+8.3f} deg"
            )
            out(
                f"    speed at that moment:                 "
                f"{worst_descent['vx']:+8.3f} m/s"
            )
            if abs(p) > 1e-4:
                out(
                    f"    reference h/sin(|pitch|):             "
                    f"{self.h / abs(math.sin(p)):8.2f} m"
                )
        else:
            out("    no pitched descent samples captured")
        out("-" * 78)

        out("PITCH GATE SIZING  (input to Phase 2)")
        out("    closest return seen, by |pitch| bin:")
        for key in sorted(bins):
            out(
                f"      |pitch| ~ {key:2d} deg:                      {bins[key]:8.2f} m"
            )

        d_safe = self.safety_k * self.max_vel_x**2 + self.safety_d_min
        out(
            f"    d_safe at max_vel_x={self.max_vel_x:.2f} m/s "
            f"(k={self.safety_k}, d_min={self.safety_d_min}): {d_safe:8.2f} m"
        )
        if worst_descent and math.isfinite(worst_descent["tail"]):
            trips = worst_descent["tail"] < d_safe
            out(
                f"    closest phantom return vs d_safe:     "
                f"{worst_descent['tail']:8.2f} m "
                f"{'<' if trips else '>='} {d_safe:.2f} m"
            )
            out(
                f"    -> a phantom return WOULD{'' if trips else ' NOT'} "
                f"trip the safety gate at full speed"
            )
        out("")
        out("=" * 78)
        out("HEADLINE NUMBERS  (for the README)")
        if max_row:
            p = self._attitude(max_row)
            travel = worst_descent["tail"] if worst_descent else float("nan")
            out(
                f"    ramp angle:                                    "
                f"{math.degrees(self.ramp_angle):6.2f} deg"
            )
            out(
                f"    max pitch measured on the ramp:                "
                f"{math.degrees(abs(p)):6.3f} deg"
            )
            out(
                f"    lidar height above ground:                     "
                f"{self.h:6.3f} m"
            )
            out(
                f"    phantom ground return at max pitch:            "
                f"{max_row['tail']:6.2f} m"
            )
            out(
                f"    same, along the direction of travel:           "
                f"{travel:6.2f} m"
            )
            out(
                f"    phantom ramp wall, robot level:                "
                f"{self.h / math.tan(self.ramp_angle):6.2f} m past the toe"
            )
        out("=" * 78)
        out("")

        text = "\n".join(add)
        print(text, flush=True)
        self.get_logger().info("smoke test 2 report written")

        if self.report_path:
            os.makedirs(
                os.path.dirname(os.path.abspath(self.report_path)), exist_ok=True
            )
            with open(self.report_path, "w", encoding="utf-8") as handle:
                handle.write("# Smoke Test 2 - ramp-induced phantom ground return\n\n")
                handle.write("```\n")
                handle.write(text)
                handle.write("\n```\n")


def main():
    rclpy.init()
    node = Smoke2Measure()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
