#!/usr/bin/env python3
"""PHASE 1 - one goal-to-goal navigation run, fully instrumented.

Sends a single NavigateToPose goal and records what the stack did with it:
whether it arrived, how far it drove against how far it planned, how long it took,
how often it replanned, whether it fell back on a recovery behaviour, how close it
came to anything, and how far the SLAM pose was from simulator ground truth
throughout.

The metrics are the Navigation row of PLAN.md section 6 - goal success rate, path
length, time to goal, replan count, collisions - plus the two this world makes
possible and Phase 2 will need: clearance to a moving obstacle, and SLAM pose
error against truth.

THREE MEASUREMENT DECISIONS WORTH STATING

1. REPLAN COUNT IS NOT PLAN COUNT. Nav2's default behaviour tree recomputes the
   global plan at 1 Hz regardless of whether anything changed, so counting /plan
   messages measures the tree's tick rate and nothing about the world. Both are
   reported: the raw count, and the number of plans that were a materially
   different route (Hausdorff distance from the previous plan above a threshold),
   which is the number that responds to a pedestrian walking into the aisle.

2. CLEARANCE IS MEASURED FROM THE FOOTPRINT, AND SPLIT STATIC/DYNAMIC. Every scan
   return is projected into world coordinates and tested against the world's real
   static geometry, read from the SDF the simulator loaded. A return that no rack,
   ramp or plateau explains is a dynamic obstacle - in this world, a pedestrian.
   Reporting the nearest static return alongside is what makes the dynamic number
   credible: if the classifier were broken, rack faces would show up as dynamic
   at aisle-crossing ranges and the two series would be indistinguishable.

3. THE ACTOR'S POSITION IS RECONSTRUCTED, SO ITS RESIDUAL IS REPORTED. Gazebo
   publishes no pose for <actor> entities (Phase 0). The reconstruction from the
   world file's trajectory is checked against the LiDAR returns attributed to the
   actor, and the disagreement is printed. The headline clearance number itself
   comes from the LiDAR, which needs no reconstruction at all.

WHAT THIS RUN CANNOT MEASURE
   Not collisions. Actors are not physics entities in Gazebo and generate no
   contacts (Phase 0, smoke test 1), so a "no collisions" claim against a
   pedestrian would be vacuous - the simulator cannot produce one. Clearance is
   the honest substitute and is what is reported.
"""

import math
import os

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros

from amr_description.fleet_config import (
    footprint_polygon,
    laser_x_offset,
    load_fleet,
)
from amr_gazebo.world_geometry import (
    actor_pose_at,
    actor_trajectories,
    actor_velocity_at,
    distance_to_boxes,
    static_boxes,
    world_root,
)
from amr_navigation.clearance import (
    distance_to_polygon,
    max_deviation,
    path_length,
    scan_to_world,
    transform_polygon,
)

#: A scan return further than this from every static box is treated as dynamic.
#: Sized by what it must survive: 0.05 m map resolution, 0.01 m of range noise and
#: the conservative plan-view projection of the pitched ramp slab. Actors in this
#: world walk the centre of a 5.5 m aisle, so the margin never decides a case.
STATIC_TOLERANCE = 0.30

#: Returns closer to the footprint than this are discarded as the vehicle seeing
#: its own chassis: the laser sits 0.06 m INSIDE the body box, so a self-return
#: lands on the footprint boundary and would otherwise be reported as a clearance
#: of exactly zero - the metric's worst possible failure, because it looks like a
#: collision. Discarded returns are counted and reported, so if the LiDAR does see
#: the chassis it shows up as a number rather than as a silently missing obstacle.
SELF_RETURN_MARGIN = 0.05

#: Hausdorff distance above which a new global plan counts as a different route.
REPLAN_DEVIATION = 0.30

#: Behaviour-tree nodes whose activation means the stack stopped making progress
#: by ordinary means. Any of these firing during a clean run is a finding.
RECOVERY_NODES = ("Spin", "BackUp", "Wait", "ClearEntireCostmap", "ClearCostmap")


def yaw_of(q):
    """Return the yaw angle of a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def quaternion_from_yaw(yaw):
    """Return (z, w) of the quaternion for a planar rotation."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def percentile(values, fraction):
    """Return a percentile of a list of numbers."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


class NavGoalRun(Node):
    """Dispatch one navigation goal and record everything the stack does with it."""

    def __init__(self):
        """Declare parameters, load world geometry, and wire up every recorder."""
        super().__init__("nav_goal_run")

        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("tag", "run")
        # Goals are given in WORLD coordinates and converted here. slam_toolbox
        # anchors its map on the robot's starting pose, so the map frame is the
        # world frame translated by the spawn pose - stating goals in world terms
        # keeps them comparable with the rack and ramp coordinates in world.yaml.
        self.declare_parameter("goal_x", -0.5)
        self.declare_parameter("goal_y", 1.5)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("world_sdf", "")
        self.declare_parameter("results_dir", "results")
        # Encounter timing. Waiting for the aisle pedestrian to turn back towards
        # the robot before dispatching turns "an actor was somewhere in the world"
        # into a head-on encounter that actually tests the local planner. It shifts
        # WHEN the run starts and nothing else; the stack is not told about it.
        self.declare_parameter("actor_gate", "pedestrian_1")
        self.declare_parameter("actor_gate_x_min", -3.0)
        self.declare_parameter("actor_gate_timeout_s", 40.0)
        self.declare_parameter("startup_settle_s", 6.0)
        self.declare_parameter("run_timeout_s", 240.0)

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        self.tag = get("tag").value
        self.goal_world = (
            float(get("goal_x").value),
            float(get("goal_y").value),
            float(get("goal_yaw").value),
        )
        self.spawn = (
            float(get("spawn_x").value),
            float(get("spawn_y").value),
            float(get("spawn_yaw").value),
        )
        self.results_dir = get("results_dir").value
        self.gate_actor = get("actor_gate").value
        self.gate_x_min = float(get("actor_gate_x_min").value)
        self.gate_timeout = float(get("actor_gate_timeout_s").value)
        self.settle_s = float(get("startup_settle_s").value)
        self.run_timeout = float(get("run_timeout_s").value)

        robot = {r["name"]: r for r in load_fleet()}[self.robot_name]
        self.footprint = footprint_polygon(robot)
        self.laser_offset = laser_x_offset(robot)

        world = world_root(self._world_sdf_path())
        self.static = static_boxes(world)
        self.actors = actor_trajectories(world)
        self.get_logger().info(
            f"world geometry: {len(self.static)} static boxes, "
            f"{len(self.actors)} actors ({', '.join(sorted(self.actors)) or 'none'})"
        )

        self.map_frame = f"{self.robot_name}/map"
        self.base_frame = f"{self.robot_name}/base_footprint"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(TFMessage, "/gz_ground_truth", self._on_truth, 20)
        self.create_subscription(
            LaserScan, f"/{self.robot_name}/scan", self._on_scan, 10
        )
        self.create_subscription(Path, f"/{self.robot_name}/plan", self._on_plan, 10)
        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._on_odom, 10
        )
        self._subscribe_behavior_tree_log()

        self.client = ActionClient(
            self, NavigateToPose, f"/{self.robot_name}/navigate_to_pose"
        )

        self.truth = None
        self.speed = 0.0
        self.samples = []
        self.plans = []
        self.first_plan_world = None
        self.recoveries = {}
        self.bt_nodes_seen = set()
        self.beam_return_fraction = []
        self.self_return_total = 0

        self.state = "waiting"
        self.node_started = None
        self.dispatched_at = None
        self.finished_at = None
        self.result_status = None
        self.goal_handle = None
        self.start_pose = None

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"nav run '{self.tag}' armed: goal world "
            f"({self.goal_world[0]:+.2f}, {self.goal_world[1]:+.2f}), map "
            f"({self.goal_map()[0]:+.2f}, {self.goal_map()[1]:+.2f})"
        )

    # ------------------------------------------------------------------ setup

    def _world_sdf_path(self):
        """Return the rendered world the running simulation was launched with."""
        configured = self.get_parameter("world_sdf").value
        if configured:
            return configured
        generated = os.environ.get("AMR_GENERATED_DIR") or os.path.join(
            "/tmp", "amr_fleet_nav_generated"
        )
        return os.path.join(generated, "warehouse_nav.sdf")

    def _subscribe_behavior_tree_log(self):
        """Subscribe to the BT log if this Nav2 build publishes one.

        Recovery accounting is evidence, not plumbing: if the message type is not
        available the run still produces every other metric, and says so, rather
        than failing to start.
        """
        try:
            from nav2_msgs.msg import BehaviorTreeLog
        except ImportError:  # pragma: no cover - depends on the Nav2 build
            self.bt_log_available = False
            self.get_logger().warn("nav2_msgs/BehaviorTreeLog unavailable")
            return
        self.bt_log_available = True
        self.create_subscription(
            BehaviorTreeLog,
            f"/{self.robot_name}/behavior_tree_log",
            self._on_bt_log,
            10,
        )

    # -------------------------------------------------------------- transforms

    def goal_map(self):
        """Return the goal in the SLAM map frame."""
        return self.world_to_map(*self.goal_world)

    def world_to_map(self, x, y, yaw=0.0):
        """Convert a world pose into the map frame.

        slam_toolbox starts its map at the robot's initial pose and Gazebo's
        odometry starts at zero there too, so map == world translated and rotated
        by the spawn pose. The run verifies this rather than trusting it: the SLAM
        pose is compared against ground truth on every scan, and a wrong offset
        here would show up immediately as a constant error of that size.
        """
        sx, sy, syaw = self.spawn
        dx, dy = x - sx, y - sy
        cos_yaw, sin_yaw = math.cos(-syaw), math.sin(-syaw)
        return (
            dx * cos_yaw - dy * sin_yaw,
            dx * sin_yaw + dy * cos_yaw,
            yaw - syaw,
        )

    def map_to_world(self, x, y):
        """Convert a map-frame position into world coordinates."""
        sx, sy, syaw = self.spawn
        cos_yaw, sin_yaw = math.cos(syaw), math.sin(syaw)
        return (sx + x * cos_yaw - y * sin_yaw, sy + x * sin_yaw + y * cos_yaw)

    def now_s(self):
        """Return the current simulation time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ inputs

    def _on_truth(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id != self.robot_name:
                continue
            t = transform.transform.translation
            self.truth = (t.x, t.y, yaw_of(transform.transform.rotation))

    def _on_odom(self, msg):
        # Measured, not commanded - the same rule Phase 2's SafetyGate follows.
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def _on_plan(self, msg):
        # /plan arrives in the GLOBAL COSTMAP's frame, and Phase 3 moved that from
        # this robot's amrN/map to the fleet frame - which coincides with the world
        # frame. So the spawn-offset conversion is right for one and a double count
        # for the other, and the frame_id is the only thing that says which.
        #
        # Worth branching rather than assuming, because the failure is quiet: every
        # DERIVED metric here is translation-invariant, so path length, deviation and
        # the replan count all stay correct while the plan CSV and every plot drawn
        # from it move by the spawn pose. Measured before this branch existed: a plan
        # that starts at the robot's true (-11.00, -1.50) was written as
        # (-22.00, -3.00), which puts the route through a rack while the numbers
        # beside it still read fine.
        if msg.header.frame_id in ("", self.map_frame):
            points = [
                self.map_to_world(p.pose.position.x, p.pose.position.y)
                for p in msg.poses
            ]
        else:
            points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not points:
            return
        entry = {
            "t": self.now_s(),
            "points": points,
            "length": path_length(points),
            "changed": True,
        }
        if self.plans:
            entry["changed"] = (
                max_deviation(points, self.plans[-1]["points"]) > REPLAN_DEVIATION
            )
        if self.first_plan_world is None:
            self.first_plan_world = points
        self.plans.append(entry)

    def _on_bt_log(self, msg):
        for event in msg.event_log:
            self.bt_nodes_seen.add(event.node_name)
            if event.current_status != "RUNNING" or event.previous_status == "RUNNING":
                continue
            if any(event.node_name.startswith(n) for n in RECOVERY_NODES):
                self.recoveries[event.node_name] = (
                    self.recoveries.get(event.node_name, 0) + 1
                )

    def _on_scan(self, msg):
        """Turn one scan into a clearance sample. This is the measurement."""
        if self.truth is None or self.state != "running":
            return

        x, y, yaw = self.truth
        laser = (
            x + self.laser_offset * math.cos(yaw),
            y + self.laser_offset * math.sin(yaw),
        )
        polygon = transform_polygon(self.footprint, x, y, yaw)

        returns = scan_to_world(msg, laser[0], laser[1], yaw)
        self.beam_return_fraction.append(len(returns) / max(1, len(msg.ranges)))

        nearest_any = float("inf")
        nearest_dynamic = float("inf")
        dynamic_point = None
        dynamic_count = 0
        self_returns = 0
        for px, py, _ in returns:
            clearance = distance_to_polygon(polygon, px, py)
            if clearance < SELF_RETURN_MARGIN:
                self_returns += 1
                continue
            nearest_any = min(nearest_any, clearance)
            if distance_to_boxes(self.static, px, py) > STATIC_TOLERANCE:
                dynamic_count += 1
                if clearance < nearest_dynamic:
                    nearest_dynamic = clearance
                    dynamic_point = (px, py)
        self.self_return_total += self_returns

        t = self.now_s()
        slam = self._slam_pose()
        self.samples.append(
            {
                "t": t,
                "gt": (x, y, yaw),
                "slam": slam,
                "speed": self.speed,
                "clearance_any": nearest_any,
                "clearance_dynamic": nearest_dynamic,
                "dynamic_count": dynamic_count,
                "dynamic_point": dynamic_point,
                "actors": {
                    name: actor_pose_at(waypoints, t)[:2]
                    for name, waypoints in self.actors.items()
                },
            }
        )

    def _slam_pose(self):
        """Return the robot's pose as SLAM believes it, in world coordinates."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        t = tf.transform.translation
        wx, wy = self.map_to_world(t.x, t.y)
        return (wx, wy, yaw_of(tf.transform.rotation) + self.spawn[2])

    # ------------------------------------------------------------- state machine

    def _tick(self):
        if self.node_started is None:
            self.node_started = self.now_s()

        if self.state == "waiting":
            self._try_dispatch()
        elif self.state == "running":
            if self.now_s() - self.dispatched_at > self.run_timeout:
                self.get_logger().error("run timed out; cancelling goal")
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.result_status = "TIMEOUT"
                self._complete()

    def _ready(self):
        """Return whether every input the measurement depends on is live."""
        if self.truth is None:
            return False, "no ground truth"
        if not self.client.server_is_ready():
            return False, "navigate_to_pose not advertised"
        if self._slam_pose() is None:
            return False, f"no transform {self.map_frame} -> {self.base_frame}"
        if self.now_s() - self.node_started < self.settle_s:
            return False, "settling"
        return True, ""

    def _actor_gate_open(self):
        """Return whether the aisle pedestrian is walking back towards the robot.

        Returns True immediately when the world has no actors, when no gate is
        configured, or when the gate has waited long enough - a gate that can hang
        a run forever is worse than an encounter that happens at an awkward phase.
        """
        waypoints = self.actors.get(self.gate_actor)
        if not waypoints or not self.gate_actor:
            return True, "no actor gate"
        if self.now_s() - self.node_started > self.gate_timeout:
            return True, "gate timed out"
        t = self.now_s()
        ax, ay = actor_pose_at(waypoints, t)[:2]
        vx, _ = actor_velocity_at(waypoints, t)
        if ax > self.gate_x_min and vx < 0.0:
            return True, f"actor at x={ax:+.2f} closing"
        return False, f"actor at x={ax:+.2f}, vx={vx:+.2f}"

    def _try_dispatch(self):
        ready, why = self._ready()
        if not ready:
            self.get_logger().info(f"waiting: {why}", throttle_duration_sec=5.0)
            return
        gate_open, gate_why = self._actor_gate_open()
        if not gate_open:
            self.get_logger().info(f"gating: {gate_why}", throttle_duration_sec=2.0)
            return

        gx, gy, gyaw = self.goal_map()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        z, w = quaternion_from_yaw(gyaw)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w

        self.state = "running"
        self.dispatched_at = self.now_s()
        self.start_pose = self.truth
        self.get_logger().info(f"dispatching goal ({gate_why})")
        self.client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("goal REJECTED")
            self.result_status = "REJECTED"
            self._complete()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        self.result_status = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"STATUS_{status}")
        self.get_logger().info(f"goal finished: {self.result_status}")
        self._complete()

    def _complete(self):
        if self.state == "done":
            return
        self.state = "done"
        self.finished_at = self.now_s()
        try:
            self._write_report()
        finally:
            rclpy.shutdown()

    # ------------------------------------------------------------------ report

    def _actor_residual(self):
        """Compare the reconstructed actor position against what the LiDAR saw.

        For every sample with a dynamic return, the distance from the nearest
        dynamic return to the nearest reconstructed actor position. A small
        residual means the two independent accounts of where the pedestrian was
        agree, and the reconstruction can be trusted where it is used.
        """
        residuals = []
        for sample in self.samples:
            point = sample["dynamic_point"]
            if point is None or not sample["actors"]:
                continue
            residuals.append(
                min(
                    math.hypot(point[0] - ax, point[1] - ay)
                    for ax, ay in sample["actors"].values()
                )
            )
        return residuals

    def _slam_errors(self):
        """Return per-sample SLAM-vs-truth position errors, in metres."""
        return [
            math.hypot(s["slam"][0] - s["gt"][0], s["slam"][1] - s["gt"][1])
            for s in self.samples
            if s["slam"] is not None
        ]

    def _write_csv(self, path):
        header = (
            "t,gt_x,gt_y,gt_yaw,slam_x,slam_y,speed,"
            "clearance_any,clearance_dynamic,dynamic_count,dyn_x,dyn_y\n"
        )
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(header)
            for s in self.samples:
                slam = s["slam"] or (float("nan"), float("nan"), float("nan"))
                point = s["dynamic_point"] or (float("nan"), float("nan"))
                handle.write(
                    f"{s['t']:.3f},{s['gt'][0]:.4f},{s['gt'][1]:.4f},"
                    f"{s['gt'][2]:.4f},{slam[0]:.4f},{slam[1]:.4f},"
                    f"{s['speed']:.4f},{s['clearance_any']:.4f},"
                    f"{s['clearance_dynamic']:.4f},{s['dynamic_count']},"
                    f"{point[0]:.4f},{point[1]:.4f}\n"
                )

    def _write_plan_csv(self, path):
        with open(path, "w", encoding="utf-8") as handle:
            handle.write("x,y\n")
            for x, y in self.first_plan_world or []:
                handle.write(f"{x:.4f},{y:.4f}\n")

    def _write_report(self):
        os.makedirs(self.results_dir, exist_ok=True)
        stem = os.path.join(self.results_dir, f"phase1_nav_{self.tag}")
        self._write_csv(f"{stem}_track.csv")
        self._write_plan_csv(f"{stem}_plan.csv")

        executed = [s["gt"][:2] for s in self.samples]
        driven = path_length(executed)
        planned = self.plans[0]["length"] if self.plans else float("nan")
        duration = (self.finished_at or self.now_s()) - (
            self.dispatched_at or self.now_s()
        )
        start = self.start_pose or (float("nan"), float("nan"), float("nan"))
        straight = math.hypot(
            self.goal_world[0] - start[0], self.goal_world[1] - start[1]
        )
        final = executed[-1] if executed else (float("nan"), float("nan"))
        goal_error = math.hypot(
            final[0] - self.goal_world[0], final[1] - self.goal_world[1]
        )

        clearances_any = [
            s["clearance_any"]
            for s in self.samples
            if math.isfinite(s["clearance_any"])
        ]
        clearances_dyn = [
            s["clearance_dynamic"]
            for s in self.samples
            if math.isfinite(s["clearance_dynamic"])
        ]
        residuals = self._actor_residual()
        slam_errors = self._slam_errors()
        speeds = [s["speed"] for s in self.samples]
        replans = sum(1 for p in self.plans[1:] if p["changed"])

        closest = None
        if clearances_dyn:
            closest = min(
                (s for s in self.samples if math.isfinite(s["clearance_dynamic"])),
                key=lambda s: s["clearance_dynamic"],
            )

        lines = []
        add = lines.append
        add("")
        add("=" * 78)
        add(f"PHASE 1 - goal-to-goal navigation run '{self.tag}'")
        add("=" * 78)
        add(f"robot:                    {self.robot_name}")
        add(f"start (world):            ({start[0]:+.2f}, {start[1]:+.2f})")
        add(
            f"goal (world / map):       "
            f"({self.goal_world[0]:+.2f}, {self.goal_world[1]:+.2f}) / "
            f"({self.goal_map()[0]:+.2f}, {self.goal_map()[1]:+.2f})"
        )
        add(f"actors in world:          {', '.join(sorted(self.actors)) or 'none'}")
        add("-" * 78)

        add("[NAVIGATION]")
        add(f"  goal result:                        {self.result_status}")
        add(f"  time to goal:                       {duration:8.1f} s  (sim)")
        add(f"  straight-line start to goal:        {straight:8.2f} m")
        add(f"  planned path length (first plan):   {planned:8.2f} m")
        add(f"  executed path length (ground truth):{driven:8.2f} m")
        if planned == planned and planned > 0:
            add(f"  executed / planned:                 {driven / planned:8.2f}")
        add(f"  final position error vs goal:       {goal_error:8.3f} m")
        mean_speed = sum(speeds) / max(1, len(speeds))
        add(f"  mean speed while navigating:        {mean_speed:8.2f} m/s")
        add(f"  peak speed:                         {max(speeds or [0.0]):8.2f} m/s")
        add("-" * 78)

        add("[PLANNING]")
        add(f"  global plans published:             {len(self.plans):8d}")
        add(f"  of which a different route:         {replans:8d}   <- replan count")
        plan_rate = len(self.plans) / max(1e-6, duration)
        add(f"  plan publication rate:              {plan_rate:8.2f} Hz")
        if self.bt_log_available:
            fired = sum(self.recoveries.values())
            add(f"  recovery behaviours fired:          {fired:8d}")
            for name, count in sorted(self.recoveries.items()):
                add(f"    {name:<34}{count:8d}")
            if not self.recoveries:
                add("    none - the stack never stopped making progress")
        else:
            add("  recovery behaviours:                unavailable (no BT log)")
        add("-" * 78)

        add("[CLEARANCE] measured from the footprint polygon, not the base frame")
        add(f"  scans analysed:                     {len(self.samples):8d}")
        beams = (
            100
            * sum(self.beam_return_fraction)
            / max(1, len(self.beam_return_fraction))
        )
        add(f"  beams returning a range:            {beams:8.1f} %")
        add(
            f"  returns discarded as self-hits:     {self.self_return_total:8d}"
            f"   ({self.self_return_total / max(1, len(self.samples)):.1f} per scan)"
        )
        if clearances_any:
            p05 = percentile(clearances_any, 0.05)
            p50 = percentile(clearances_any, 0.5)
            add(f"  min clearance to ANY obstacle:      {min(clearances_any):8.3f} m")
            add(f"  5th percentile:                     {p05:8.3f} m")
            add(f"  median:                             {p50:8.3f} m")
        if clearances_dyn:
            dyn_p05 = percentile(clearances_dyn, 0.05)
            dyn_p50 = percentile(clearances_dyn, 0.5)
            add(f"  scans with a dynamic return:        {len(clearances_dyn):8d}")
            add(f"  MIN CLEARANCE TO A DYNAMIC OBSTACLE:{min(clearances_dyn):8.3f} m")
            add(f"  5th percentile:                     {dyn_p05:8.3f} m")
            add(f"  median while one was visible:       {dyn_p50:8.3f} m")
            if closest is not None:
                add(f"  closest approach at t =             {closest['t']:8.1f} s")
                add(
                    f"    robot at                          "
                    f"({closest['gt'][0]:+.2f}, {closest['gt'][1]:+.2f})"
                )
                add(
                    f"    obstacle return at                "
                    f"({closest['dynamic_point'][0]:+.2f}, "
                    f"{closest['dynamic_point'][1]:+.2f})"
                )
                for name, (ax, ay) in sorted(closest["actors"].items()):
                    add(f"    {name} reconstructed at    ({ax:+.2f}, {ay:+.2f})")
        else:
            add("  no dynamic returns seen - no encounter in this run")
        if residuals:
            add(
                f"  actor reconstruction residual:      "
                f"median {percentile(residuals, 0.5):.2f} m, "
                f"p95 {percentile(residuals, 0.95):.2f} m, n={len(residuals)}"
            )
            add("    (LiDAR-attributed obstacle vs world-file trajectory; the")
            add("     clearance number above comes from the LiDAR alone)")
        add("  NOTE actors generate no contacts in Gazebo, so this is a clearance")
        add("  measurement, not a collision check. See the module docstring.")
        add("-" * 78)

        add("[LOCALIZATION] slam_toolbox pose vs simulator ground truth")
        if slam_errors:
            mean_error = sum(slam_errors) / len(slam_errors)
            p95_error = percentile(slam_errors, 0.95)
            add(f"  samples with a map transform:       {len(slam_errors):8d}")
            add(f"  mean error:                         {mean_error:8.3f} m")
            add(f"  p95 error:                          {p95_error:8.3f} m")
            add(f"  max error:                          {max(slam_errors):8.3f} m")
        else:
            add("  no map->base transform was ever available")
        add("=" * 78)
        add("")

        text = "\n".join(lines)
        print(text, flush=True)
        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write(f"# Phase 1 - goal-to-goal navigation run: {self.tag}\n\n")
            handle.write("```\n")
            handle.write(text)
            handle.write("\n```\n")
        self.get_logger().info(f"report written to {stem}.md")


def main():
    rclpy.init()
    node = NavGoalRun()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
