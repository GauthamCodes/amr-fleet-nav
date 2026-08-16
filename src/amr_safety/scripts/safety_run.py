#!/usr/bin/env python3
"""PHASE 2 - a safety halt on a pedestrian encounter, and what Nav2 did during it.

This run puts the whole stack together and sends one goal down an aisle a pedestrian
walks into. It records three things that only exist when everything is running at
once: whether the gate halted for a moving obstacle, how long the sensor-to-zero
path took, and whether Nav2 fired a recovery behaviour while the robot was held.

THE A/B IS THE EVIDENCE, NOT THE ZERO

    A recovery count of zero proves nothing on its own - Nav2 may simply not have
    been stuck long enough to try one. So this run is executed twice, identically,
    with ``suppress_recovery`` true and false, and the two reports are compared. The
    claim is the DIFFERENCE between them. If the control run also shows zero, the
    honest reading is that the experiment did not exercise the mechanism, and the
    report says so rather than banking the pass.

    Both halves matter for a second reason: ``behavior_server`` publishes cmd_vel
    too (nav2_behaviors/timed_behavior.hpp), so a recovery that fires during a halt
    is not merely cosmetic - it is a second writer trying to move a robot the gate
    has stopped. The Phase 2 wiring routes it through cmd_vel_nav for exactly this
    reason, and the control run is what shows the path is real.

THE PARAMETER IS READ BACK, NOT ASSUMED

    At every halt and every release this node asks controller_server what
    ``progress_checker.movement_time_allowance`` actually is. A mechanism that
    silently failed to write would otherwise be indistinguishable from one that
    worked and was never needed.

WHAT THIS RUN CANNOT MEASURE
    Not collisions. Gazebo actors are not physics entities and generate no contacts
    (Phase 0, smoke test 1), so clearance is the honest quantity and is what is
    reported.
"""

import math
import os

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from amr_bsp.nav2_readiness import Nav2Readiness

#: Behaviour-tree nodes whose activation means the stack stopped making progress by
#: ordinary means. Same tuple Phase 1's nav_goal_run.py counts.
RECOVERY_NODES = ("Spin", "BackUp", "Wait", "ClearEntireCostmap", "ClearCostmap")

#: The Nav2 parameter the gate raises while it holds the robot.
ALLOWANCE_PARAMETER = "progress_checker.movement_time_allowance"


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


class SafetyRun(Node):
    """Sends one goal into a pedestrian encounter and records every halt."""

    def __init__(self):
        """Declare parameters, wire the recorders, and arm the goal dispatch."""
        super().__init__("safety_run")
        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("goal_x", -0.5)
        self.declare_parameter("goal_y", 1.5)
        self.declare_parameter("spawn_x", -11.0)
        self.declare_parameter("spawn_y", -1.5)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("start_delay_s", 8.0)
        self.declare_parameter("timeout_s", 180.0)
        self.declare_parameter("linger_s", 6.0)
        self.declare_parameter("results_dir", "results")
        self.declare_parameter("tag", "safety_halt")
        # What the report calls itself. Defaulted to the Phase 2 pedestrian run so
        # that run's artifact is unchanged; the Phase 3 carry-over parks a static
        # barrier instead, and a report headed "pedestrian encounter" describing a
        # barrier is the kind of small inconsistency an evaluator notices.
        self.declare_parameter("title", "SafetyGate halt on a pedestrian encounter")
        self.declare_parameter("suppress_recovery", True)
        self.declare_parameter("max_goal_retries", 2)

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        self.goal_world = (float(get("goal_x").value), float(get("goal_y").value))
        self.spawn = (
            float(get("spawn_x").value),
            float(get("spawn_y").value),
            float(get("spawn_yaw").value),
        )
        self.start_delay = float(get("start_delay_s").value)
        self.timeout = float(get("timeout_s").value)
        self.linger = float(get("linger_s").value)
        self.results_dir = get("results_dir").value
        self.tag = get("tag").value
        self.title = get("title").value
        self.suppress_recovery = bool(get("suppress_recovery").value)

        self.create_subscription(
            DiagnosticArray,
            f"/{self.robot_name}/safety_gate/diagnostics",
            self._on_gate,
            20,
        )
        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._on_odom, 20
        )
        self.create_subscription(
            Twist, f"/{self.robot_name}/cmd_vel", self._on_gated_command, 20
        )
        self.create_subscription(
            Twist, f"/{self.robot_name}/cmd_vel_nav", self._on_nav_command, 20
        )
        self._subscribe_behavior_tree_log()

        self.client = ActionClient(
            self, NavigateToPose, f"/{self.robot_name}/navigate_to_pose"
        )
        self.parameters = self.create_client(
            GetParameters, f"/{self.robot_name}/controller_server/get_parameters"
        )
        # An action server that EXISTS is not an action server that ACCEPTS. See
        # amr_bsp.nav2_readiness: dispatching on wait_for_server alone produced a run
        # whose only symptom was "Action server is inactive. Rejecting the goal.",
        # a goal result of REJECTED, and a safety report reading "halts: 0 ... FAIL"
        # - because the robot never drove at the pedestrian in the first place.
        self.readiness = Nav2Readiness(self, [self.robot_name])

        self.gate = None
        self.blocked = False
        self.halts = []
        self.current = None
        self.latency_ms = []
        self.compute_us = []
        self.k = None
        self.d_min = None
        self.speed = 0.0
        self.moved_while_blocked = 0.0
        self.position = None
        self.blocked_position = None
        self.nonzero_gated_while_blocked = 0
        self.leaked_commands = 0
        self.nav_commands_while_blocked = 0

        self.recoveries = {}
        self.recovery_events = []

        self.started = self.now_s()
        self.next_dispatch_at = 0.0
        self.attempts = 0
        self.max_attempts = int(get("max_goal_retries").value)
        self.dispatched_at = None
        self.finished_at = None
        self.result_status = None
        self.state = "waiting"

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"safety run '{self.tag}' armed: suppression "
            f"{'ON' if self.suppress_recovery else 'OFF (control)'}"
        )

    # ------------------------------------------------------------------ setup

    def _subscribe_behavior_tree_log(self):
        """Subscribe to the BT log if this Nav2 build publishes one.

        ``bt_log_available`` is set HERE and must not be reassigned afterwards.
        It was, once, a few lines further down __init__, and the result was a
        report that said "behaviour-tree log unavailable on this Nav2 build"
        while happily subscribed to it - the recovery evidence silently absent
        from a run that had collected it.
        """
        self.bt_log_available = False
        try:
            from nav2_msgs.msg import BehaviorTreeLog
        except ImportError:  # pragma: no cover - depends on the Nav2 build
            self.get_logger().warn("nav2_msgs/BehaviorTreeLog unavailable")
            return
        self.bt_log_available = True
        self.create_subscription(
            BehaviorTreeLog,
            f"/{self.robot_name}/behavior_tree_log",
            self._on_bt_log,
            10,
        )

    def goal_map(self):
        """Convert the world-frame goal into the SLAM map frame.

        slam_toolbox starts its map at the robot's spawn pose, so map is world
        translated and rotated by that pose. Same conversion Phase 1's
        nav_goal_run.py uses, and stated here for the same reason: the goal is
        specified in world coordinates because that is the frame the aisle, the
        racks and the pedestrian lanes are described in.
        """
        sx, sy, syaw = self.spawn
        dx, dy = self.goal_world[0] - sx, self.goal_world[1] - sy
        cos_yaw, sin_yaw = math.cos(-syaw), math.sin(-syaw)
        return (dx * cos_yaw - dy * sin_yaw, dx * sin_yaw + dy * cos_yaw)

    def now_s(self):
        """Return the current simulation time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ inputs

    def _on_odom(self, msg):
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_gated_command(self, msg):
        # What actually left the gate. A nonzero twist while blocked would mean the
        # serial gate leaked, which is the one failure this whole design forbids.
        if self.blocked and abs(msg.linear.x) > 1e-3:
            self.nonzero_gated_while_blocked += 1

    def _on_nav_command(self, msg):
        if self.blocked and abs(msg.linear.x) > 1e-3:
            self.nav_commands_while_blocked += 1

    def _on_bt_log(self, msg):
        for event in msg.event_log:
            if event.current_status != "RUNNING" or event.previous_status == "RUNNING":
                continue
            if not any(event.node_name.startswith(n) for n in RECOVERY_NODES):
                continue
            self.recoveries[event.node_name] = (
                self.recoveries.get(event.node_name, 0) + 1
            )
            self.recovery_events.append((self.now_s(), event.node_name, self.blocked))
            if self.blocked:
                self.get_logger().warn(
                    f"RECOVERY '{event.node_name}' fired while the gate held the robot"
                )

    def _on_gate(self, msg):
        for status in msg.status:
            try:
                values = {p.key: float(p.value) for p in status.values}
            except ValueError:
                continue
            self.gate = values
            self.leaked_commands = int(values.get("leaked_commands", 0))
            self.k = values.get("k", self.k)
            self.d_min = values.get("d_min", self.d_min)
            latency = values.get("sensor_to_zero_ms", -1.0)
            if latency >= 0.0:
                self.latency_ms.append(latency)
            compute = values.get("compute_us", -1.0)
            if compute >= 0.0:
                self.compute_us.append(compute)

            now_blocked = values.get("blocked", 0.0) > 0.5
            if now_blocked and not self.blocked:
                self._enter_halt(values)
            elif self.blocked and not now_blocked:
                self._exit_halt(values)
            self.blocked = now_blocked
            if self.blocked and self.blocked_position and self.position:
                self.moved_while_blocked = max(
                    self.moved_while_blocked,
                    math.hypot(
                        self.position[0] - self.blocked_position[0],
                        self.position[1] - self.blocked_position[1],
                    ),
                )

    # -------------------------------------------------------------- halt record

    def _enter_halt(self, values):
        self.blocked_position = self.position
        self.current = {
            "entered": self.now_s(),
            "clearance": values.get("clearance", float("nan")),
            "speed": values.get("speed_measured", float("nan")),
            "d_stop": values.get("d_stop", float("nan")),
            "latency_ms": values.get("sensor_to_zero_ms", float("nan")),
            "stale": values.get("stale", 0.0) > 0.5,
            "recoveries_at_entry": sum(self.recoveries.values()),
            "allowance_at_entry": None,
            "allowance_at_exit": None,
        }
        self.get_logger().warn(
            f"HALT {len(self.halts) + 1}: clearance "
            f"{self.current['clearance']:.3f} m at "
            f"{self.current['speed']:.3f} m/s, sensor->zero "
            f"{self.current['latency_ms']:.1f} ms"
        )
        self._read_allowance("allowance_at_entry", self.current)

    def _exit_halt(self, values):
        if self.current is None:
            return
        entry = self.current
        entry["released"] = self.now_s()
        entry["duration"] = entry["released"] - entry["entered"]
        entry["recoveries_during"] = (
            sum(self.recoveries.values()) - entry["recoveries_at_entry"]
        )
        entry["clearance_at_release"] = values.get("clearance", float("nan"))
        self._read_allowance("allowance_at_exit", entry)
        self.halts.append(entry)
        self.current = None
        self.get_logger().info(
            f"RELEASE after {entry['duration']:.2f} s; recoveries during the halt: "
            f"{entry['recoveries_during']}"
        )

    def _read_allowance(self, key, entry):
        """Ask controller_server what the progress checker's allowance really is."""
        if not self.parameters.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = [ALLOWANCE_PARAMETER]
        future = self.parameters.call_async(request)

        def done(result):
            # Asked rather than caught: a failed service call leaves the entry as
            # None, which the report prints as "n/a". Reporting "not read" is a
            # different statement from reporting a value, and conflating them is
            # how a broken mechanism comes to look like a working one.
            if result.exception() is not None:  # pragma: no cover - service failure
                return
            values = result.result().values
            if values and values[0].type == ParameterType.PARAMETER_DOUBLE:
                entry[key] = values[0].double_value

        future.add_done_callback(done)

    # ------------------------------------------------------------ goal dispatch

    def _tick(self):
        elapsed = self.now_s() - self.started
        if self.state == "waiting":
            if self.now_s() < self.next_dispatch_at:
                return
            if elapsed < self.start_delay:
                return
            if not self.client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info(
                    "waiting for navigate_to_pose", throttle_duration_sec=5.0
                )
                return
            if not self.readiness.all_active():
                self.get_logger().info(
                    "navigate_to_pose is up; waiting for bt_navigator to activate",
                    throttle_duration_sec=5.0,
                )
                return
            self._dispatch()
            return
        if self.state == "running" and elapsed > self.timeout:
            self.get_logger().error("run timed out; reporting what was recorded")
            self.result_status = "TIMEOUT"
            self._complete()
        if self.state == "lingering" and self.now_s() - self.finished_at >= self.linger:
            self.state = "done"
            try:
                self._write_report()
            finally:
                rclpy.shutdown()

    def _dispatch(self):
        gx, gy = self.goal_map()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = f"{self.robot_name}/map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.w = 1.0
        self.state = "running"
        self.dispatched_at = self.now_s()
        self.get_logger().info(
            f"dispatching goal world ({self.goal_world[0]:+.2f}, "
            f"{self.goal_world[1]:+.2f}) = map ({gx:+.2f}, {gy:+.2f})"
        )
        self.client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.result_status = "REJECTED"
            self._complete()
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        self.result_status = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"STATUS_{status}")
        # NavFn intermittently returns "Failed to create a plan from potential"
        # while the map is still filling in ahead of the robot. That is a mapping
        # transient, not a safety result, and ending the run on it would throw
        # away the encounter this launch exists to measure. Retried a bounded
        # number of times and REPORTED, so the retry is visible rather than
        # smoothing over a stack that could not navigate.
        remaining = self.now_s() - self.started < self.timeout
        if self.result_status == "ABORTED" and self.attempts < self.max_attempts:
            if remaining:
                self.attempts += 1
                self.get_logger().warn(
                    f"goal ABORTED; re-dispatching (attempt {self.attempts + 1} "
                    f"of {self.max_attempts + 1})"
                )
                self.state = "waiting"
                # A separate clock from self.started, which owns the run timeout
                # and must keep counting across retries.
                self.next_dispatch_at = self.now_s() + 3.0
                return
        self._complete()

    def _complete(self):
        if self.state in ("lingering", "done"):
            return
        if self.current is not None:
            self._exit_halt(self.gate or {})
        self.finished_at = self.now_s()
        self.state = "lingering"
        self.get_logger().info(f"goal finished: {self.result_status}")

    # ------------------------------------------------------------------ report

    def _write_report(self):
        os.makedirs(self.results_dir, exist_ok=True)
        stem = os.path.join(self.results_dir, f"phase2_{self.tag}")
        lines = []
        add = lines.append
        rule = "=" * 78
        thin = "-" * 78

        add(rule)
        add(f"PHASE 2 - {self.title}")
        add(rule)
        add(f"robot:                    {self.robot_name}")
        add(
            f"recovery suppression:     "
            f"{'ON' if self.suppress_recovery else 'OFF  <- CONTROL RUN'}"
        )
        add(f"goal result:              {self.result_status}")
        add(
            f"goal dispatches:          {self.attempts + 1}"
            f"{'  (re-planned after a NavFn transient)' if self.attempts else ''}"
        )
        if self.k is not None:
            add(f"k / d_min:                {self.k:.4f} s^2/m / {self.d_min:.3f} m")
        add(thin)

        self._add_halt_section(add, thin)
        self._add_latency_section(add, thin)
        self._add_recovery_section(add, thin)

        add("VERDICT")
        checks = [
            ("the gate halted at least once", bool(self.halts)),
            (
                "no command left the gate while latched",
                self.leaked_commands == 0,
            ),
            ("sensor-to-zero latency was measured", bool(self.latency_ms)),
        ]
        if self.suppress_recovery:
            during = sum(h.get("recoveries_during", 0) for h in self.halts)
            checks.append(("no recovery fired during any halt", during == 0))
            checks.append(
                (
                    "the allowance was actually raised at a halt",
                    any(
                        h.get("allowance_at_entry") is not None
                        and h["allowance_at_entry"] > 100.0
                        for h in self.halts
                    ),
                )
            )
        for label, passed in checks:
            add(f"    {label + ':':<50} {'YES' if passed else 'NO'}")
        ok = all(passed for _, passed in checks)
        add(f"    RESULT: {'PASS' if ok else 'FAIL'}")
        add(rule)

        text = "\n".join(lines)
        print(text, flush=True)
        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write(f"# Phase 2 - {self.title}\n\n")
            handle.write("```\n")
            handle.write(text)
            handle.write("\n```\n")
        self._write_csv(f"{stem}.csv")
        self.get_logger().info(f"wrote {stem}.md")

    def _add_halt_section(self, add, thin):
        add("[A] HALTS")
        add(f"    halts:                                    {len(self.halts):8d}")
        add(
            f"    GATE-REPORTED leaked commands:            "
            f"{self.leaked_commands:8d}   <- must be 0"
        )
        add(
            f"    observed by this probe (lagged, see below):"
            f"{self.nonzero_gated_while_blocked:8d}"
        )
        add("        The gate counts a leak by comparing the twist it is about to")
        add("        publish against its latch at that instant. This probe can only")
        add("        see the latch through diagnostics, published once per scan, so")
        add("        a command passed legitimately in the ~100 ms after a release")
        add("        reads to it as a leak. The gate's own count is the invariant;")
        add("        the probe's is kept because a large gap between them would mean")
        add("        the gate is releasing far more often than the report suggests.")
        add(
            f"    upstream kept commanding motion:          "
            f"{self.nav_commands_while_blocked:8d}"
        )
        add(
            f"    furthest the robot moved while blocked:   "
            f"{self.moved_while_blocked:8.3f} m"
        )
        add("")
        if self.halts:
            add("      #   clearance   speed    d_safe   held s   latency ms   recov")
            for index, halt in enumerate(self.halts, start=1):
                add(
                    f"    {index:3d}   {halt['clearance']:9.3f}  "
                    f"{halt['speed']:6.3f}  {halt['d_stop']:8.3f}  "
                    f"{halt.get('duration', float('nan')):7.2f}  "
                    f"{halt['latency_ms']:11.1f}  "
                    f"{halt.get('recoveries_during', 0):5d}"
                )
        else:
            add("    no halt occurred - the encounter did not bring the gate in")
        add(thin)

    def _add_latency_section(self, add, thin):
        add("[B] SENSOR STAMP -> ZERO COMMAND PUBLISHED")
        if self.latency_ms:
            add(f"    blocking decisions:           {len(self.latency_ms):8d}")
            add(
                f"    mean:                                     "
                f"{sum(self.latency_ms) / len(self.latency_ms):8.2f} ms"
            )
            add(
                f"    p95:                                      "
                f"{percentile(self.latency_ms, 0.95):8.2f} ms"
            )
            add(f"    max:                          {max(self.latency_ms):8.2f} ms")
        else:
            add("    none recorded")
        if self.compute_us:
            add(
                f"    in-node compute, mean / p95 / max:        "
                f"{sum(self.compute_us) / len(self.compute_us):.1f} / "
                f"{percentile(self.compute_us, 0.95):.1f} / "
                f"{max(self.compute_us):.1f} us"
            )
        add("")
        add("    End-to-end covers the simulator's sensor pipeline, the bridge, the")
        add("    BSP relay and the gate, and is quantised by Gazebo's /clock step")
        add("    under use_sim_time; the compute figure is not. Low-latency safety")
        add("    override - not hard real-time: no RT kernel, no scheduling guarantee.")
        add(thin)

    def _add_recovery_section(self, add, thin):
        add("[C] NAV2 RECOVERY BEHAVIOUR DURING THE HALT  (ENGINEERING_NOTES rule 2)")
        if not self.bt_log_available:
            add("    behaviour-tree log unavailable on this Nav2 build")
            add(thin)
            return
        during = sum(h.get("recoveries_during", 0) for h in self.halts)
        add(f"    recovery behaviours, whole run: {sum(self.recoveries.values()):8d}")
        add(f"    recovery behaviours fired DURING a halt:  {during:8d}")
        for name, count in sorted(self.recoveries.items()):
            add(f"        {name:<36} {count:8d}")
        if not self.recoveries:
            add("        none")
        add("")
        add("    progress_checker.movement_time_allowance, read back from")
        add("    controller_server at each transition:")
        for index, halt in enumerate(self.halts, start=1):
            entry = halt.get("allowance_at_entry")
            exit_value = halt.get("allowance_at_exit")
            add(
                f"      halt {index}: at entry "
                f"{'n/a' if entry is None else f'{entry:.0f} s':>12}"
                f"   at release "
                f"{'n/a' if exit_value is None else f'{exit_value:.0f} s':>12}"
            )
        add("")
        if self.suppress_recovery:
            add("    This is the SUPPRESSED run. Read it against the control run")
            add("    (suppress_recovery:=false). Zero here means something only if the")
            add("    control shows a nonzero count on the same encounter.")
        else:
            add("    This is the CONTROL run. Nothing suppresses the progress checker,")
            add("    so any recovery counted here is what the suppressed run prevents.")
            add("    behavior_server publishes cmd_vel, so a recovery during a halt is")
            add("    a second writer contending for a robot the gate has stopped.")
        add(thin)

    def _write_csv(self, path):
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(
                "halt,clearance_m,speed_mps,d_safe_m,held_s,latency_ms,"
                "recoveries_during,allowance_entry_s,allowance_exit_s\n"
            )
            for index, halt in enumerate(self.halts, start=1):
                entry = halt.get("allowance_at_entry")
                exit_value = halt.get("allowance_at_exit")
                handle.write(
                    f"{index},{halt['clearance']:.4f},{halt['speed']:.4f},"
                    f"{halt['d_stop']:.4f},{halt.get('duration', float('nan')):.3f},"
                    f"{halt['latency_ms']:.2f},{halt.get('recoveries_during', 0)},"
                    f"{'' if entry is None else f'{entry:.1f}'},"
                    f"{'' if exit_value is None else f'{exit_value:.1f}'}\n"
                )


def main():
    rclpy.init()
    node = SafetyRun()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
