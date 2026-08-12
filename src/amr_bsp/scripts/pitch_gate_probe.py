#!/usr/bin/env python3
"""PHASE 2 - before/after evidence that PitchGate suppresses the ramp phantom return.

WHY THIS SUBSCRIBES TO A RAW SENSOR TOPIC

    docs/ENGINEERING_NOTES.md rule 8 says nothing subscribes to raw sensor topics. This node is the
    stated exemption, and the reason is structural rather than convenient: an
    instrument that measures what the BSP removed has to see both sides of it. Every
    node in the NAVIGATION STACK reads validated topics only; this one is measuring
    the BSP, not navigating with it. See amr_bsp/topics.py.

THE PAIRING IS ITSELF THE RULE-4 TEST

    Raw and validated scans are matched by EXACT header stamp. If the BSP restamped
    anything on republish - the silent failure that breaks TF and costs a day - no
    pair would match and this run would report zero matched scans instead of quietly
    producing plausible numbers against mismatched data. So the match count is not
    bookkeeping; it is the evidence for rule 4.

WHAT THIS RUN CANNOT SHOW

    The LEVEL-robot ramp return. At pitch ~0 the ramp genuinely rises through the
    scan plane 2.49 m past the toe, and that return is real - a real surface at a
    real range, which a 2D scan simply cannot distinguish from a wall. PitchGate does
    not fire there and must not. That case belongs to Phase 4's RampCostLayer, and
    conflating the two would be an overclaim.
"""

import math
import os

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan

from amr_bsp.pitch_gate import gate_range, quaternion_to_roll_pitch

#: Reported descriptively, not as a pass/fail: a beam 30 degrees off the axis of
#: travel still has 87 % of the forward depression and its gate is legitimately
#: active, so "did we cut anything near the sides" is not by itself a defect. The
#: pass/fail criteria are the two per-beam invariants in section [C].
NEAR_SIDE_HALF_WIDTH = math.radians(15.0)


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


class PitchGateProbe(Node):
    """Pairs raw and validated scans and reports what the PitchGate removed."""

    def __init__(self):
        """Declare parameters and subscribe to both sides of the BSP."""
        super().__init__("pitch_gate_probe")
        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("lidar_height", 0.35)
        self.declare_parameter("gate_margin", 0.9)
        self.declare_parameter("min_gate_range", 0.0)
        self.declare_parameter("sample_duration_s", 90.0)
        self.declare_parameter("results_dir", "results")
        self.declare_parameter("tag", "pitch_gate")

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        self.lidar_height = float(get("lidar_height").value)
        self.gate_margin = float(get("gate_margin").value)
        self.min_gate_range = float(get("min_gate_range").value)
        self.duration = float(get("sample_duration_s").value)
        self.results_dir = get("results_dir").value
        self.tag = get("tag").value

        self.raw_by_stamp = {}
        self.pitch_samples = []
        self.samples = []
        self.matched = 0
        self.unmatched = 0
        self.skipped_startup = 0
        self.max_stamp_error_ns = 0
        self.first_raw_ns = None
        self.last_raw_ns = None
        self.raw_non_advancing = 0
        self.relay_ms = []
        self.best = None

        self.create_subscription(
            LaserScan, f"/{self.robot_name}/scan", self._on_raw, 20
        )
        self.create_subscription(
            LaserScan, f"/{self.robot_name}/validated/scan", self._on_validated, 20
        )
        self.create_subscription(
            Imu, f"/{self.robot_name}/validated/imu", self._on_imu, 50
        )
        self.create_subscription(
            DiagnosticArray, f"/{self.robot_name}/bsp/diagnostics", self._on_diag, 10
        )

        self.started = None
        self.create_timer(1.0, self._tick)

    # ------------------------------------------------------------------ inputs

    def _on_imu(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        _, pitch = quaternion_to_roll_pitch(msg.orientation)
        self.pitch_samples.append((stamp, pitch))
        cutoff = stamp - 1.0
        while self.pitch_samples and self.pitch_samples[0][0] < cutoff:
            self.pitch_samples.pop(0)

    def _on_raw(self, msg):
        key = Time.from_msg(msg.header.stamp).nanoseconds
        if self.first_raw_ns is None:
            self.first_raw_ns = key
        # A raw scan whose stamp does not advance means two publishers are feeding
        # this topic - in practice, a stray gz sim from an earlier run that was never
        # killed. It corrupts every before/after comparison here, so it is counted
        # and reported rather than left to look like a PitchGate result.
        if self.last_raw_ns is not None and key <= self.last_raw_ns:
            self.raw_non_advancing += 1
        self.last_raw_ns = key
        self.raw_by_stamp[key] = msg
        if len(self.raw_by_stamp) > 200:
            for old in sorted(self.raw_by_stamp)[:100]:
                del self.raw_by_stamp[old]

    def _on_diag(self, msg):
        for status in msg.status:
            if status.hardware_id != "lidar":
                continue
            for pair in status.values:
                if pair.key == "relay_ms_mean":
                    value = float(pair.value)
                    if not math.isnan(value):
                        self.relay_ms.append(value)

    def _pitch_at(self, stamp):
        if not self.pitch_samples:
            return None
        best_stamp, best_pitch = min(
            self.pitch_samples, key=lambda sample: abs(sample[0] - stamp)
        )
        if abs(best_stamp - stamp) > 0.15:
            return None
        return best_pitch

    def _on_validated(self, msg):
        if self.started is None:
            self.started = self.get_clock().now()
        key = Time.from_msg(msg.header.stamp).nanoseconds
        raw = self.raw_by_stamp.get(key)
        if raw is None:
            if self.first_raw_ns is None or key < self.first_raw_ns:
                # This scan was acquired before the probe's raw subscription existed,
                # so there is nothing to compare it against. A subscription-ordering
                # artefact, not a restamp - counted separately so it cannot be
                # mistaken for one in either direction.
                self.skipped_startup += 1
            else:
                self.unmatched += 1
            return
        self.matched += 1
        self.max_stamp_error_ns = max(
            self.max_stamp_error_ns,
            abs(key - Time.from_msg(raw.header.stamp).nanoseconds),
        )

        stamp = key / 1e9
        pitch = self._pitch_at(stamp)
        if pitch is None:
            return

        removed = []
        near_side_removed = 0
        inside_floor_removed = 0
        uphill_removed = 0
        below_gate_removed = 0
        widest_azimuth = 0.0
        for index, (before, after) in enumerate(zip(raw.ranges, msg.ranges)):
            if math.isinf(after) and math.isfinite(before):
                azimuth = raw.angle_min + index * raw.angle_increment
                removed.append((azimuth, before))
                widest_azimuth = max(widest_azimuth, abs(azimuth))
                if abs(abs(azimuth) - math.pi / 2.0) < NEAR_SIDE_HALF_WIDTH:
                    near_side_removed += 1
                if before < self.min_gate_range:
                    inside_floor_removed += 1
                # The two invariants that actually define correct truncation: a beam
                # may only be cut if it points DOWNHILL (a gate radius exists at all)
                # and its range lies BEYOND that radius.
                gate = gate_range(
                    self.lidar_height,
                    pitch,
                    azimuth,
                    self.gate_margin,
                    self.min_gate_range,
                )
                if math.isinf(gate):
                    uphill_removed += 1
                elif before < gate:
                    below_gate_removed += 1

        finite_before = [r for r in raw.ranges if math.isfinite(r)]
        finite_after = [r for r in msg.ranges if math.isfinite(r)]
        record = {
            "stamp": stamp,
            "pitch": pitch,
            "beams": len(raw.ranges),
            "removed": len(removed),
            "near_side_removed": near_side_removed,
            "inside_floor_removed": inside_floor_removed,
            "uphill_removed": uphill_removed,
            "below_gate_removed": below_gate_removed,
            "widest_azimuth": widest_azimuth,
            "min_before": min(finite_before) if finite_before else float("inf"),
            "min_after": min(finite_after) if finite_after else float("inf"),
            "closest_removed": min((r for _, r in removed), default=float("inf")),
        }
        self.samples.append(record)

        if self.best is None or abs(pitch) > abs(self.best["pitch"]):
            self.best = dict(record)
            self.best["raw"] = list(raw.ranges)
            self.best["validated"] = list(msg.ranges)
            self.best["angle_min"] = raw.angle_min
            self.best["angle_increment"] = raw.angle_increment
            self.best["range_max"] = raw.range_max

    # ------------------------------------------------------------------ report

    def _tick(self):
        if self.started is None:
            self.get_logger().info(
                "waiting: no validated scans yet", throttle_duration_sec=5.0
            )
            return
        elapsed = (self.get_clock().now() - self.started).nanoseconds / 1e9
        if elapsed >= self.duration:
            try:
                self._write_report()
            finally:
                rclpy.shutdown()

    def _stem(self):
        os.makedirs(self.results_dir, exist_ok=True)
        return os.path.join(self.results_dir, f"phase2_{self.tag}")

    def _write_scan_csv(self, stem):
        """Save the max-pitch scan, raw against validated, for the plot."""
        with open(f"{stem}_scan.csv", "w", encoding="utf-8") as handle:
            handle.write("azimuth_rad,raw_range,validated_range,gate_range\n")
            best = self.best
            for index, (before, after) in enumerate(
                zip(best["raw"], best["validated"])
            ):
                azimuth = best["angle_min"] + index * best["angle_increment"]
                gate = gate_range(
                    self.lidar_height,
                    best["pitch"],
                    azimuth,
                    self.gate_margin,
                    self.min_gate_range,
                )
                handle.write(
                    f"{azimuth:.6f},{before:.4f},{after:.4f},"
                    f"{gate if math.isfinite(gate) else float('nan'):.4f}\n"
                )

    def _write_series_csv(self, stem):
        with open(f"{stem}_series.csv", "w", encoding="utf-8") as handle:
            handle.write("t,pitch_deg,removed,min_before,min_after,closest_removed\n")
            origin = self.samples[0]["stamp"]
            for s in self.samples:
                handle.write(
                    f"{s['stamp'] - origin:.3f},{math.degrees(s['pitch']):.4f},"
                    f"{s['removed']:d},{s['min_before']:.4f},{s['min_after']:.4f},"
                    f"{s['closest_removed']:.4f}\n"
                )

    def _write_report(self):
        stem = self._stem()
        lines = []
        add = lines.append
        rule = "=" * 78
        thin = "-" * 78

        add(rule)
        add("PHASE 2 - PitchGate: ramp phantom ground return, before and after")
        add(rule)
        add(f"robot:                    {self.robot_name}")
        add(f"lidar height h:           {self.lidar_height:.3f} m")
        add(f"gate margin:              {self.gate_margin:.2f} x h/sin|pitch|")
        add(f"gate floor (interlock):   {self.min_gate_range:.3f} m")
        add(f"validated scans:          {self.matched + self.unmatched}")
        add(thin)

        add("[0] HEADER STAMP PRESERVATION  (docs/ENGINEERING_NOTES.md rule 4)")
        add(f"    raw/validated pairs matched by EXACT stamp: {self.matched:8d}")
        add(f"    RESTAMPED (no matching raw stamp):          {self.unmatched:8d}")
        add(
            f"    max |stamp difference| within a pair:       "
            f"{self.max_stamp_error_ns:8d} ns"
        )
        add(
            f"    skipped, acquired before this probe started:{self.skipped_startup:8d}"
        )
        preserved = (
            self.matched > 0 and self.unmatched == 0 and self.max_stamp_error_ns == 0
        )
        add(
            f"    -> original acquisition stamps are "
            f"{'PRESERVED' if preserved else 'NOT PRESERVED'}"
        )
        add("")
        add(
            f"    raw scans whose stamp did NOT advance:      "
            f"{self.raw_non_advancing:8d}"
        )
        add("        must be 0. Nonzero means a second publisher is on the raw")
        add("        topic - almost always a stray gz sim left from an earlier run -")
        add("        and every number below it is measured on a corrupted stream.")
        add(thin)

        if not self.samples:
            add("NO SAMPLES WITH A PITCH ESTIMATE - nothing to report.")
            self._emit(lines, stem)
            return

        add("[A] TRUNCATION BY |PITCH| BIN")
        add("      bin    scans   mean beams cut   closest cut   min range after")
        bins = {}
        for s in self.samples:
            bins.setdefault(int(round(abs(math.degrees(s["pitch"])))), []).append(s)
        for degrees in sorted(bins):
            group = bins[degrees]
            cut = sum(g["removed"] for g in group) / len(group)
            closest = min(g["closest_removed"] for g in group)
            after = min(g["min_after"] for g in group)
            add(
                f"    {degrees:3d} deg  {len(group):6d}   {cut:12.1f}   "
                f"{closest:11.2f}   {after:15.2f}"
            )
        add(thin)

        best = self.best
        pitch_deg = math.degrees(best["pitch"])
        nominal = (
            self.lidar_height / abs(math.sin(best["pitch"]))
            if best["pitch"]
            else float("inf")
        )
        add("[B] THE MAX-PITCH SCAN  (the headline before/after)")
        add(f"    |pitch|:                                  {abs(pitch_deg):8.3f} deg")
        add(
            f"    attitude:                                 "
            f"{'nose-DOWN' if best['pitch'] > 0 else 'nose-UP':>8s}"
        )
        add(f"    predicted ground intersection h/sin|p|:   {nominal:8.3f} m")
        add(
            f"    gate radius at that pitch (0.9 x):        "
            f"{self.gate_margin * nominal:8.3f} m"
        )
        add(f"    beams in the scan:                        {best['beams']:8d}")
        add(f"    BEAMS TRUNCATED:                          {best['removed']:8d}")
        add(
            f"    closest return BEFORE:                    {best['min_before']:8.3f} m"
        )
        add(f"    closest return AFTER:                     {best['min_after']:8.3f} m")
        add(
            f"    closest return that was removed:          "
            f"{best['closest_removed']:8.3f} m"
        )
        add(thin)

        add("[C] WHAT WAS NOT TRUNCATED  (the half that matters for safety)")
        uphill = sum(s["uphill_removed"] for s in self.samples)
        below = sum(s["below_gate_removed"] for s in self.samples)
        inside = sum(s["inside_floor_removed"] for s in self.samples)
        near_side = sum(s["near_side_removed"] for s in self.samples)
        widest = max(s["widest_azimuth"] for s in self.samples)
        total_cut = sum(s["removed"] for s in self.samples)
        total_beams = sum(s["beams"] for s in self.samples)
        add(f"    beams truncated, all scans:               {total_cut:8d}")
        add(
            f"    as a fraction of all beams seen:          "
            f"{100.0 * total_cut / max(1, total_beams):8.2f} %"
        )
        add("")
        add("    INVARIANTS - each must be zero, and each is checked per beam:")
        add(f"      cut while pointing UPHILL:              {uphill:8d}")
        add("          a beam with no downward component cannot strike the ground,")
        add("          so it has no gate radius and must never be cut")
        add(f"      cut while INSIDE its own gate radius:   {below:8d}")
        add("          the truncation rule itself: only returns BEYOND the predicted")
        add("          ground intersection are ground")
        add(f"      cut INSIDE the braking envelope:        {inside:8d}")
        add(
            f"          the floor is {self.min_gate_range:.3f} m = d_safe(v_max) + "
            f"footprint"
        )
        add("          + margin; nonzero means truncation could delete an obstacle")
        add("          the SafetyGate would have had to stop for")
        add("")
        add("    DESCRIPTIVE (not pass/fail - the gate is legitimately active here):")
        add(
            f"      widest |azimuth| truncated:             "
            f"{math.degrees(widest):8.1f} deg"
        )
        add(f"      cut within 15 deg of +/-90:             {near_side:8d}")
        add("          a beam 30 deg off-axis still carries 87 % of the forward")
        add("          depression, so a nonzero count here is geometry, not a fault")
        add(thin)

        add("[D] BSP RELAY LATENCY  (stamp -> validated publish, lidar channel)")
        if self.relay_ms:
            add(
                f"    samples:                                  {len(self.relay_ms):8d}"
            )
            add(
                f"    mean of the BSP's own rolling mean:       "
                f"{sum(self.relay_ms) / len(self.relay_ms):8.2f} ms"
            )
            add(
                f"    p95:                                      "
                f"{percentile(self.relay_ms, 0.95):8.2f} ms"
            )
            add(
                f"    max:                                      "
                f"{max(self.relay_ms):8.2f} ms"
            )
            add("    PLAN.md section 4 keeps the BSP in Python until measurement")
            add("    says otherwise. This is that measurement.")
        else:
            add("    no diagnostics received")
        add(thin)

        add("VERDICT")
        clean_stream = self.raw_non_advancing == 0
        ok = (
            preserved
            and clean_stream
            and uphill == 0
            and below == 0
            and inside == 0
            and best["removed"] > 0
        )
        add(
            f"    single publisher on the raw topic: "
            f"{'YES' if clean_stream else 'NO'}"
        )
        add(f"    stamps preserved:                  {'YES' if preserved else 'NO'}")
        add(
            f"    phantom truncated at max pitch:    "
            f"{'YES' if best['removed'] > 0 else 'NO'}"
        )
        add(f"    nothing cut pointing uphill:       {'YES' if uphill == 0 else 'NO'}")
        add(f"    nothing cut inside its gate:       {'YES' if below == 0 else 'NO'}")
        add(f"    braking envelope untouched:        {'YES' if inside == 0 else 'NO'}")
        add(f"    RESULT: {'PASS' if ok else 'FAIL'}")
        add(rule)

        self._write_scan_csv(stem)
        self._write_series_csv(stem)
        self._emit(lines, stem)

    def _emit(self, lines, stem):
        text = "\n".join(lines)
        print(text, flush=True)
        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write("# Phase 2 - PitchGate: ramp phantom ground return\n\n")
            handle.write("```\n")
            handle.write(text)
            handle.write("\n```\n")
        self.get_logger().info(f"wrote {stem}.md")


def main():
    rclpy.init()
    node = PitchGateProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
