#!/usr/bin/env python3
"""SMOKE TEST 1 - measure whether a Gazebo actor reaches /scan and the obstacle layer.

WHY THIS IS SHAPED THE WAY IT IS

Gazebo publishes no pose for ``<actor>`` entities. They appear in neither
``/world/<w>/pose/info`` nor ``/world/<w>/dynamic_pose/info``, and ``gz model --list``
does not list them, so there is no ground-truth stream to compare the scan against.
The test therefore uses the geometry the world file already fixes: the robot spawns
at a known pose and the actor walks a known scripted line, so the bearing sector the
actor can possibly occupy is known in advance. Any return inside that sector, in an
otherwise empty world, came from the actor.

Three independent measurements are taken so that a negative result is conclusive:

* CONTROL BOX - ordinary collision geometry in its own bearing sector. It must be
  detected. If it is not, the rig is broken and nothing else means anything.
* ACTOR CORRIDOR - the bearing/range window the actor sweeps through.
* RENDER WITNESS - a camera watching the actor's path. gpu_lidar raycasts the rendering
  scene, so this separates "the actor never rendered" from "the actor renders but is
  excluded from ray queries". Without it, a silent mesh-load failure would masquerade as
  a definitive finding.
"""

import math
import os

from nav2_msgs.msg import Costmap
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan


def quaternion_to_roll_pitch(q):
    """Return (roll, pitch) in radians from a geometry_msgs quaternion."""
    sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
    cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    return roll, pitch


#: Half-width of the bearing sector searched around the control box, in radians.
CONTROL_SECTOR_HALF_WIDTH = math.radians(10.0)

#: Radial tolerance around an expected range when attributing a return, in metres.
RANGE_TOLERANCE = 0.75

#: Cost at or above which a costmap cell counts as an obstacle. nav2_msgs/Costmap
#: carries raw costmap_2d values, so this is the genuine LETHAL_OBSTACLE constant
#: rather than the 0-100 rescaling an OccupancyGrid would apply.
OBSTACLE_COST = 253
LETHAL_COST = 254

#: Radius around a position searched for lethal cells, in metres.
LETHAL_SEARCH_RADIUS = 0.35

#: Mean per-pixel intensity change above which the camera is judged to show motion.
MOTION_THRESHOLD = 0.05


def percentile(values, fraction):
    """Return a percentile of a list without requiring numpy on the caller's side."""
    if not values:
        return float("inf")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


class Smoke1Check(Node):
    """Characterise actor visibility in the LiDAR and in a Nav2 obstacle layer."""

    def __init__(self):
        """Declare parameters and subscribe to scan, costmap, camera and IMU."""
        super().__init__("smoke1_check")

        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("robot_x", 0.0)
        self.declare_parameter("robot_y", 0.0)
        self.declare_parameter("robot_yaw", 0.0)
        self.declare_parameter("lidar_height", 0.20)
        # The laser sits forward of base_link. Ignoring that offset biases every
        # expected range by ~0.3 m, which is the same order as the effects measured.
        self.declare_parameter("lidar_x_offset", 0.29)
        self.declare_parameter("actor_x", 3.0)
        self.declare_parameter("actor_y_min", -2.0)
        self.declare_parameter("actor_y_max", 2.0)
        self.declare_parameter("control_x", 3.0)
        self.declare_parameter("control_y", -3.5)
        self.declare_parameter("control_size", 0.4)
        self.declare_parameter("sample_duration_s", 30.0)
        self.declare_parameter("report_path", "")

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        self.robot_yaw = float(get("robot_yaw").value)
        self.lidar_height = float(get("lidar_height").value)
        offset = float(get("lidar_x_offset").value)
        # Every bearing and range in this test is measured from the LASER, not from
        # base_link, because that is where the beams originate.
        self.robot_xy = (
            float(get("robot_x").value) + offset * math.cos(self.robot_yaw),
            float(get("robot_y").value) + offset * math.sin(self.robot_yaw),
        )
        self.actor_x = float(get("actor_x").value)
        self.actor_y = (
            float(get("actor_y_min").value),
            float(get("actor_y_max").value),
        )
        self.control_xy = (float(get("control_x").value), float(get("control_y").value))
        self.control_size = float(get("control_size").value)
        self.duration = float(get("sample_duration_s").value)
        self.report_path = get("report_path").value

        self.control_bearing, self.control_range = self._polar(self.control_xy)
        self.actor_window = self._actor_window()

        self.scan_meta = None
        self.scan_count = 0
        self.started = None
        self.control_hits = []
        self.actor_hits = []
        self.actor_ranges = []
        self.costmap = None
        self.costmap_count = 0
        self.control_cost_max = 0
        self.control_lethal_frames = 0
        self.actor_cost_max = 0
        self.actor_lethal_frames = 0
        self.prev_image = None
        self.image_deltas = []
        self.image_count = 0
        # Resting attitude. A robot that does not sit level puts its scan plane into
        # the floor and manufactures phantom returns before it ever reaches a ramp,
        # which would contaminate every range measured here and in smoke test 2.
        self.rolls = []
        self.pitches = []

        self.create_subscription(
            LaserScan, f"/{self.robot_name}/scan", self._on_scan, 10
        )
        # costmap_raw rather than the OccupancyGrid: Nav2 only publishes the
        # OccupancyGrid when something is already subscribed, and it rescales costs to
        # 0-100. costmap_raw publishes unconditionally and carries true 0-255 values.
        self.create_subscription(Costmap, "/costmap/costmap_raw", self._on_costmap, 10)
        self.create_subscription(Image, "/smoke1/witness_camera", self._on_image, 10)
        self.create_subscription(Imu, f"/{self.robot_name}/imu", self._on_imu, 10)

        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"sampling for {self.duration:.0f}s")

    # ------------------------------------------------------------- geometry

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _polar(self, xy):
        """Return (bearing in the robot's frame, planar range) of a world position."""
        dx = xy[0] - self.robot_xy[0]
        dy = xy[1] - self.robot_xy[1]
        return self._wrap(math.atan2(dy, dx) - self.robot_yaw), math.hypot(dx, dy)

    def _actor_window(self):
        """Return the bearing and range window the scripted actor can occupy."""
        corners = [
            self._polar((self.actor_x, self.actor_y[0])),
            self._polar((self.actor_x, self.actor_y[1])),
            self._polar((self.actor_x, 0.5 * sum(self.actor_y))),
        ]
        bearings = [c[0] for c in corners]
        ranges = [c[1] for c in corners]
        return {
            "bearing_min": min(bearings),
            "bearing_max": max(bearings),
            "range_min": min(ranges) - RANGE_TOLERANCE,
            "range_max": max(ranges) + RANGE_TOLERANCE,
        }

    # ---------------------------------------------------------------- inputs

    def _on_image(self, msg):
        """Track frame-to-frame change, which reveals whether anything is animating."""
        frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).astype(np.float32)
        self.image_count += 1
        if self.prev_image is not None and self.prev_image.shape == frame.shape:
            self.image_deltas.append(float(np.mean(np.abs(frame - self.prev_image))))
        self.prev_image = frame

    def _on_imu(self, msg):
        roll, pitch = quaternion_to_roll_pitch(msg.orientation)
        self.rolls.append(roll)
        self.pitches.append(pitch)

    def _on_costmap(self, msg):
        """Accumulate costmap evidence.

        Sampling only the final costmap would be wrong: the actor is moving and the
        obstacle layer clears cells behind it as rays pass through, so whether the
        actor is a lethal obstacle has to be judged over the whole run.
        """
        self.costmap = msg
        self.costmap_count += 1

        control = self._max_cost_near(msg, self.control_xy, LETHAL_SEARCH_RADIUS)
        if control is not None:
            self.control_cost_max = max(self.control_cost_max, control)
            if control >= OBSTACLE_COST:
                self.control_lethal_frames += 1

        path_max = 0
        steps = 21
        lo, hi = self.actor_y
        for index in range(steps):
            y = lo + (hi - lo) * index / (steps - 1)
            cost = self._max_cost_near(msg, (self.actor_x, y), LETHAL_SEARCH_RADIUS)
            if cost is not None:
                path_max = max(path_max, cost)
        self.actor_cost_max = max(self.actor_cost_max, path_max)
        if path_max >= OBSTACLE_COST:
            self.actor_lethal_frames += 1

    def _max_cost_near(self, grid, world_xy, radius):
        """Return the highest cost within a radius of a world position."""
        info = grid.metadata
        width, height = info.size_x, info.size_y
        col = int((world_xy[0] - info.origin.position.x) / info.resolution)
        row = int((world_xy[1] - info.origin.position.y) / info.resolution)
        span = int(math.ceil(radius / info.resolution))
        best = None
        for dr in range(-span, span + 1):
            for dc in range(-span, span + 1):
                r, c = row + dr, col + dc
                if not (0 <= c < width and 0 <= r < height):
                    continue
                if math.hypot(dr, dc) * info.resolution > radius:
                    continue
                value = grid.data[r * width + c]
                best = value if best is None else max(best, value)
        return best

    def _on_scan(self, msg):
        if self.started is None:
            self.started = self.get_clock().now()
        self.scan_count += 1
        if self.scan_meta is None:
            self.scan_meta = {
                "range_max": msg.range_max,
                "range_min": msg.range_min,
                "n": len(msg.ranges),
                "frame_id": msg.header.frame_id,
            }

        # Control box: fixed sector, fixed expected range.
        control = self._sector(
            msg,
            self.control_bearing - CONTROL_SECTOR_HALF_WIDTH,
            self.control_bearing + CONTROL_SECTOR_HALF_WIDTH,
        )
        self.control_hits.append(control)

        # Actor corridor: any return inside the swept window, at plausible range.
        window = self._sector(
            msg, self.actor_window["bearing_min"], self.actor_window["bearing_max"]
        )
        in_range = [
            r
            for r in window["returns"]
            if self.actor_window["range_min"] <= r <= self.actor_window["range_max"]
        ]
        self.actor_hits.append(len(in_range))
        if in_range:
            self.actor_ranges.append(min(in_range))

    def _sector(self, scan, bearing_lo, bearing_hi):
        """Return every valid range inside a bearing sector, plus beam accounting."""
        centre = self._wrap(0.5 * (bearing_lo + bearing_hi))
        half = abs(self._wrap(bearing_hi - bearing_lo)) / 2.0
        returns = []
        total = 0
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(self._wrap(angle - centre)) > half:
                continue
            total += 1
            if math.isinf(value) or math.isnan(value):
                continue
            if value < scan.range_min or value > scan.range_max:
                continue
            returns.append(value)
        return {"returns": returns, "total": total}

    # ---------------------------------------------------------------- report

    def _tick(self):
        if self.started is None:
            self.get_logger().warn("no /scan received yet")
            return
        elapsed = (self.get_clock().now() - self.started).nanoseconds / 1e9
        if elapsed < self.duration:
            return
        self._report()
        rclpy.shutdown()

    def _report(self):
        meta = self.scan_meta or {}

        control_detected = [h for h in self.control_hits if h["returns"]]
        control_frac = (
            len(control_detected) / len(self.control_hits) if self.control_hits else 0.0
        )
        # Per-scan minima, summarised by median. A single global minimum would be set
        # by whichever scan the moving actor happened to clip the sector edge on, which
        # says nothing about where the static box is.
        control_minima = [min(h["returns"]) for h in control_detected]
        control_median = percentile(control_minima, 0.5)
        control_min = min(control_minima) if control_minima else float("inf")
        # Expected range to the near face of the box, not to its centre.
        control_expected = self.control_range - self.control_size / 2.0

        actor_frames = sum(1 for h in self.actor_hits if h > 0)
        actor_frac = actor_frames / len(self.actor_hits) if self.actor_hits else 0.0
        actor_min = min(self.actor_ranges) if self.actor_ranges else float("inf")

        control_frames_frac = (
            self.control_lethal_frames / self.costmap_count
            if self.costmap_count
            else 0.0
        )
        actor_frames_frac = (
            self.actor_lethal_frames / self.costmap_count if self.costmap_count else 0.0
        )

        motion = (
            sum(self.image_deltas) / len(self.image_deltas)
            if self.image_deltas
            else 0.0
        )
        peak_motion = max(self.image_deltas) if self.image_deltas else 0.0

        lines = []
        add = lines.append
        add("")
        add("=" * 78)
        add("SMOKE TEST 1 - Gazebo actor LiDAR visibility")
        add("=" * 78)
        add(
            f"laser origin:         ({self.robot_xy[0]:.2f}, {self.robot_xy[1]:.2f}), "
            f"yaw {math.degrees(self.robot_yaw):.1f} deg  [robot {self.robot_name}]"
        )
        add(f"lidar height:         {self.lidar_height:.3f} m above ground")
        add(f"scan frame_id:        {meta.get('frame_id', '<none>')}")
        add(
            f"scan geometry:        {meta.get('n', 0)} beams, "
            f"{meta.get('range_min', 0):.2f}-{meta.get('range_max', 0):.2f} m"
        )
        add(f"scans analysed:       {self.scan_count}")
        add("-" * 78)

        add("[CONTROL] box with ordinary collision geometry - MUST be detected")
        add(
            f"  world position:                     "
            f"({self.control_xy[0]:.2f}, {self.control_xy[1]:.2f})"
        )
        add(
            f"  expected bearing:                   "
            f"{math.degrees(self.control_bearing):8.2f} deg"
        )
        add(f"  expected range to near face:        {control_expected:8.2f} m")
        add(f"  scans with a return in sector:      {control_frac * 100:8.1f} %")
        if control_detected:
            add(f"  median per-scan closest return:     {control_median:8.2f} m")
            add(
                f"  median - expected:                  "
                f"{control_median - control_expected:+8.2f} m"
            )
            add(f"  absolute closest over run:          {control_min:8.2f} m")
        else:
            add("  closest measured return:            NO RETURN")
        add(
            f"  peak costmap cost at box:           {self.control_cost_max:8d}"
            f"{'  (LETHAL)' if self.control_cost_max >= LETHAL_COST else ''}"
        )
        add(
            f"  costmap frames marking it lethal:   "
            f"{control_frames_frac * 100:8.1f} %"
        )
        add("-" * 78)

        add("[SUBJECT] Gazebo <actor> sweeping the corridor ahead")
        add(
            f"  corridor bearing window:            "
            f"{math.degrees(self.actor_window['bearing_min']):+.1f} to "
            f"{math.degrees(self.actor_window['bearing_max']):+.1f} deg"
        )
        add(
            f"  corridor range window:              "
            f"{self.actor_window['range_min']:.2f} to "
            f"{self.actor_window['range_max']:.2f} m"
        )
        add(f"  scans with a return in corridor:    {actor_frac * 100:8.1f} %")
        if self.actor_ranges:
            add(
                f"  median per-scan closest return:     "
                f"{percentile(self.actor_ranges, 0.5):8.2f} m"
            )
            add(
                f"  5th percentile:                     "
                f"{percentile(self.actor_ranges, 0.05):8.2f} m"
            )
            add(
                f"  95th percentile:                    "
                f"{percentile(self.actor_ranges, 0.95):8.2f} m"
            )
            add(f"  absolute closest over run:          {actor_min:8.2f} m")
            add(
                f"  actor centre-line distance:         "
                f"{self._polar((self.actor_x, 0.0))[1]:8.2f} m"
            )
        else:
            add("  closest measured return:            NO RETURN")
        add(f"  costmap frames analysed:            {self.costmap_count:8d}")
        add(
            f"  peak costmap cost on actor path:    {self.actor_cost_max:8d}"
            f"{'  (LETHAL)' if self.actor_cost_max >= LETHAL_COST else ''}"
        )
        add(
            f"  costmap frames marking it lethal:   "
            f"{actor_frames_frac * 100:8.1f} %"
        )
        add("-" * 78)

        add("[WITNESS] camera watching the actor's path")
        add(f"  frames received:                    {self.image_count:8d}")
        add(f"  mean frame-to-frame change:         {motion:8.4f} / 255")
        add(f"  peak frame-to-frame change:         {peak_motion:8.4f} / 255")
        add(
            f"  something is animating in view:     "
            f"{'YES' if peak_motion > MOTION_THRESHOLD else 'NO'}"
        )
        add("-" * 78)

        add("[ATTITUDE] resting attitude on flat ground")
        if self.pitches:
            mean_pitch = sum(self.pitches) / len(self.pitches)
            mean_roll = sum(self.rolls) / len(self.rolls)
            max_pitch = max(abs(p) for p in self.pitches)
            add(f"  IMU samples:                        {len(self.pitches):8d}")
            add(
                f"  mean pitch:                         "
                f"{math.degrees(mean_pitch):+8.3f} deg"
            )
            add(
                f"  mean roll:                          "
                f"{math.degrees(mean_roll):+8.3f} deg"
            )
            add(
                f"  max |pitch|:                        "
                f"{math.degrees(max_pitch):8.3f} deg"
            )
            # A tilted scan plane meets the floor at h/sin(tilt).
            if abs(mean_pitch) > math.radians(0.05):
                add(
                    f"  implied ground return h/sin(pitch): "
                    f"{self.lidar_height / abs(math.sin(mean_pitch)):8.2f} m"
                )
            else:
                add("  implied ground return:              none (level)")
        else:
            add("  no IMU data received")
        add("-" * 78)

        add("[PHYSICS] Gazebo entity accounting")
        add("  actors are absent from /world/<w>/pose/info, dynamic_pose/info")
        add("  and 'gz model --list' - they are not physics entities, so they can")
        add("  never generate contacts regardless of LiDAR visibility.")
        add("-" * 78)

        rig_ok = control_frac > 0.5 and self.control_cost_max >= OBSTACLE_COST
        actor_seen = actor_frac > 0.05
        rendered = peak_motion > MOTION_THRESHOLD
        actor_in_costmap = self.actor_cost_max >= OBSTACLE_COST

        add("VERDICT")
        if not rig_ok:
            add("  RIG BROKEN - the control box was not detected in the LiDAR,")
            add("  or never became a lethal obstacle in the costmap.")
            add("  The actor result carries no information until this is fixed.")
            verdict = "INCONCLUSIVE - RIG BROKEN"
        elif actor_seen:
            add("  Control box detected AND actor detected in the LiDAR.")
            add(
                f"  Actor reaches the Nav2 obstacle layer: "
                f"{'YES' if actor_in_costmap else 'NO - scan only'}"
            )
            verdict = "ACTOR VISIBLE TO LIDAR"
        else:
            add("  Control box detected; actor NOT detected in the LiDAR.")
            if rendered:
                add(
                    "  The witness camera shows motion, so the actor IS being rendered."
                )
                add("  It is drawn but excluded from LiDAR ray queries.")
            else:
                add("  The witness camera shows no motion either.")
            add("  -> FALLBACK REQUIRED: replace <actor> with a collision-bearing")
            add("     pedestrian proxy driven by the VelocityControl system.")
            verdict = "ACTOR NOT VISIBLE - FALLBACK REQUIRED"
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
                handle.write("# Smoke Test 1 - Gazebo actor LiDAR visibility\n\n")
                handle.write("```\n")
                handle.write(text)
                handle.write("\n```\n")


def main():
    rclpy.init()
    node = Smoke1Check()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
