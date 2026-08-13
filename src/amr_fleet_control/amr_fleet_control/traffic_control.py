"""TrafficControlNode: the yield protocol, for conflicts the local layer cannot fix.

ASSIGNMENT 3.2, THE SECOND HALF. The first half - each robot's local planner
consuming its peers' predicted trajectories - is ``FleetTrajectoryLayer``, and it
runs with no central node in the loop. This node is the escalation path, and
ENGINEERING_NOTES rule 7 is that the two must not be collapsed into one another: a
central
arbiter that resolved every conflict would make the MAPF layer decorative, and a
local layer alone deadlocks at a constriction where neither robot has anywhere to
deviate to.

    predicted trajectories  ->  FleetTrajectoryLayer   (per robot, local, always)
                            ->  TrafficControlNode     (central, only on escalation)

WHAT ESCALATION MEANS, OPERATIONALLY

    A conflict is escalated only when it has persisted for ``escalate_after_s``
    AND the predicted closest approach has stopped improving. Both halves are
    load-bearing. Without the timer the arbiter would fire on the first predicted
    crossing, which the local layer resolves routinely. Without the improvement
    test it would fire on conflicts that are already being opened up, and a robot
    would be stopped for a problem that was solving itself.

    Everything the arbiter declines to act on is COUNTED, not discarded. The
    "resolved without escalation" number is what makes the ordering visible in
    the evidence rather than merely asserted in a docstring.

HOW A YIELD IS COMMANDED, AND HOW IT ENDS

    By publishing a zero twist on the yielding robot's ``cmd_vel_yield`` at the
    control rate. That is a priority-150 input on its mux against navigation's
    100, so while this node is talking the navigation stream is ignored entirely.

    The release is the ABSENCE of a message: stop publishing and the mux's own
    0.5 s timeout drops the channel and navigation resumes. There is no release
    message that can be lost, and an arbiter that crashes mid-yield releases the
    robot instead of pinning it (the mux is not a safety element and must fail
    open; SafetyGate downstream is the serial fail-closed one - rule 1).

GATING WITHOUT NOTIFICATION IS A BUG (rule 2)

    A yield zeroes ``cmd_vel`` for as long as the conflict lasts, which is far
    longer than ``progress_checker.movement_time_allowance``. Left alone, Nav2
    decides the yielding robot is stuck and dispatches recovery behaviours -
    spin-in-place, at the constriction, into the path of the robot being given
    way to. So the arbiter raises that parameter on the yielding robot on entry
    and restores it after the robot has actually moved again. Phase 3 measured
    what happens without it: 6 progress-checker failures and 2 recoveries fired
    into a robot the safety gate was deliberately holding.

    SafetyGate writes the same parameter for the same reason, and the two can
    overlap. Both capture the ORIGINAL value at startup rather than reading
    whatever is there at entry, so neither can latch the other's suppressed
    value permanently, and this node RE-ASSERTS the allowance every
    ``reassert_period_s`` while it holds - so if the gate releases first and
    restores 10 s mid-yield, the suppression is back within one period, long
    before a 10 s progress-checker window could expire.
"""

from itertools import combinations
import math
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from amr_bsp.topics import CMD_YIELD, PREDICTED_TRAJECTORY
from amr_description.fleet_config import load_fleet
from amr_fleet_control.traffic_policy import (
    conflict_radius,
    DEFAULT_ESCALATE_AFTER_S,
    DEFAULT_IMPROVEMENT_M,
    DEFAULT_MAX_HOLD_S,
    DEFAULT_MIN_HOLD_S,
    DEFAULT_RELEASE_FACTOR,
    escalate,
    footprint_radius,
    gross_mass_kg,
    priority_order,
    release,
    yielder,
)
from amr_fleet_control.traffic_report import (
    ConflictRecord,
    report,
    write_csv,
    YieldRecord,
)
from amr_fleet_control.trajectory_conflict import closest_approach

#: Behaviour-tree nodes whose activation means Nav2 gave up on ordinary progress.
#: Same tuple Phase 1's nav_goal_run and Phase 2's safety_run count, restated
#: rather than imported: those two live in other packages' ``scripts/``
#: directories, which are installed as executables and are not importable.
RECOVERY_NODES = ("Spin", "BackUp", "Wait", "ClearEntireCostmap", "ClearCostmap")

#: The Nav2 parameter raised while a robot is held (rule 2).
ALLOWANCE_PARAMETER = "progress_checker.movement_time_allowance"


class TrafficControlNode(Node):
    """Escalate unresolvable trajectory conflicts into a priority yield."""

    def __init__(self):
        """Load the fleet, derive the arbitration constants, wire every robot."""
        super().__init__("traffic_control")

        fleet = load_fleet()
        if len(fleet) < 2:
            raise ValueError("traffic control needs at least two robots in fleet.yaml")
        self.fleet = fleet
        self.names = [robot["name"] for robot in fleet]
        self.order = priority_order(fleet)

        # A radius of 0.0 means "derive it from the fleet", which is the default
        # and what ships. It is exposed as a parameter only so a run can widen or
        # tighten it deliberately; nothing in the repo does.
        self.declare_parameter("conflict_radius_m", 0.0)
        self.declare_parameter("time_window_s", 3.0)
        self.declare_parameter("escalate_after_s", DEFAULT_ESCALATE_AFTER_S)
        self.declare_parameter("improvement_m", DEFAULT_IMPROVEMENT_M)
        self.declare_parameter("release_factor", DEFAULT_RELEASE_FACTOR)
        self.declare_parameter("min_hold_s", DEFAULT_MIN_HOLD_S)
        self.declare_parameter("max_hold_s", DEFAULT_MAX_HOLD_S)
        self.declare_parameter("rate_hz", 20.0)
        # A prediction this old is not a statement about the future any more.
        self.declare_parameter("trajectory_stale_s", 1.5)
        self.declare_parameter("suppress_recovery", True)
        self.declare_parameter("recovery_allowance_s", 1.0e6)
        self.declare_parameter("reassert_period_s", 2.0)
        self.declare_parameter("restore_grace_s", 2.0)
        self.declare_parameter("restore_movement_radius", 0.5)
        # Evidence. Empty writes nothing, which is what the fleet bringup uses -
        # the same convention as FleetMapNode's decision_log.
        self.declare_parameter("results_stem", "")
        self.declare_parameter("duration_s", 0.0)
        self.declare_parameter("title", "PHASE 7 - yield protocol")

        get = self.get_parameter
        radius = float(get("conflict_radius_m").value)
        self.radius = radius if radius > 0.0 else conflict_radius(fleet)
        self.time_window = float(get("time_window_s").value)
        self.escalate_after = float(get("escalate_after_s").value)
        self.improvement = float(get("improvement_m").value)
        self.release_factor = float(get("release_factor").value)
        self.min_hold = float(get("min_hold_s").value)
        self.max_hold = float(get("max_hold_s").value)
        self.stale_s = float(get("trajectory_stale_s").value)
        self.suppress_recovery = bool(get("suppress_recovery").value)
        self.allowance_s = float(get("recovery_allowance_s").value)
        self.reassert_period = float(get("reassert_period_s").value)
        self.restore_grace = float(get("restore_grace_s").value)
        self.restore_radius = float(get("restore_movement_radius").value)
        self.results_stem = get("results_stem").value
        self.duration = float(get("duration_s").value)
        self.title = get("title").value

        self.trajectories = {}
        self.positions = {}
        self.yield_publishers = {}
        self.set_clients = {}
        self.get_clients = {}
        self.original_allowance = {}
        self.gate_blocked = {name: False for name in self.names}
        self.recoveries = {name: {} for name in self.names}

        for name in self.names:
            self.create_subscription(
                Path,
                f"/{name}/{PREDICTED_TRAJECTORY}",
                lambda msg, n=name: self._on_trajectory(n, msg),
                5,
            )
            self.yield_publishers[name] = self.create_publisher(
                Twist, f"/{name}/{CMD_YIELD}", 1
            )
            self.set_clients[name] = self.create_client(
                SetParameters, f"/{name}/controller_server/set_parameters"
            )
            self.get_clients[name] = self.create_client(
                GetParameters, f"/{name}/controller_server/get_parameters"
            )
        self._subscribe_behaviour_tree_logs()
        self._subscribe_gate_diagnostics()

        self.live = {}
        self.holds = {}
        self.pending_restore = {}
        self.conflicts_seen = []
        self.yields = []
        self.report_written = False

        self.started = self.now_s()
        # The last time a tick ran. The report is normally written from a
        # shutdown handler, where reading the clock again is not guaranteed to
        # work, so a hold still open at the end is closed against this instead.
        self.last_tick = self.started
        self.create_timer(1.0 / float(get("rate_hz").value), self._tick)

        self.get_logger().info(
            "TrafficControlNode up: priority "
            + " > ".join(
                f"{n}({gross_mass_kg(self._robot(n)):.0f} kg)" for n in self.order
            )
            + f", conflict radius {self.radius:.2f} m, window "
            f"{self.time_window:.1f} s, "
            f"escalate after {self.escalate_after:.1f} s, recovery suppression "
            f"{'ON' if self.suppress_recovery else 'OFF'}"
        )

    def _robot(self, name):
        """Return one fleet.yaml entry by name."""
        return next(robot for robot in self.fleet if robot["name"] == name)

    def now_s(self):
        """Return the current simulation time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------- inputs

    def _subscribe_behaviour_tree_logs(self):
        """Subscribe to each robot's BT log if this Nav2 build publishes one.

        ``bt_log_available`` is set once, here, and never reassigned - Phase 2
        shipped a report claiming the log was unavailable while subscribed to it,
        because a later line in ``__init__`` reset the flag.
        """
        self.bt_log_available = False
        try:
            from nav2_msgs.msg import BehaviorTreeLog
        except ImportError:  # pragma: no cover - depends on the Nav2 build
            self.get_logger().warn("nav2_msgs/BehaviorTreeLog unavailable")
            return
        self.bt_log_available = True
        for name in self.names:
            self.create_subscription(
                BehaviorTreeLog,
                f"/{name}/behavior_tree_log",
                lambda msg, n=name: self._on_bt_log(n, msg),
                10,
            )

    def _subscribe_gate_diagnostics(self):
        """Watch every SafetyGate, so a gate halt is never reported as a yield.

        The two produce the same observable - a robot at a standstill - and this
        run's whole claim is about which one did it. Recording the gate's own
        state alongside each hold is what keeps the report from having to assume.
        """
        from diagnostic_msgs.msg import DiagnosticArray

        for name in self.names:
            self.create_subscription(
                DiagnosticArray,
                f"/{name}/safety_gate/diagnostics",
                lambda msg, n=name: self._on_gate(n, msg),
                10,
            )

    def _on_trajectory(self, name, msg):
        """Cache one robot's predicted trajectory with ABSOLUTE arrival times."""
        samples = []
        for pose in msg.poses:
            arrival = Time.from_msg(pose.header.stamp).nanoseconds / 1e9
            samples.append((pose.pose.position.x, pose.pose.position.y, arrival))
        if not samples:
            return
        self.trajectories[name] = (self.now_s(), samples)
        # The first sample is where the robot is NOW, in the fleet frame. Taking
        # the position from here rather than from odometry keeps this node on one
        # input per robot, and odometry is in the robot's own frame anyway.
        self.positions[name] = (samples[0][0], samples[0][1])

    def _on_bt_log(self, name, msg):
        """Count recovery behaviours as they begin running."""
        for event in msg.event_log:
            if event.current_status != "RUNNING" or event.previous_status == "RUNNING":
                continue
            if not any(event.node_name.startswith(n) for n in RECOVERY_NODES):
                continue
            counts = self.recoveries[name]
            counts[event.node_name] = counts.get(event.node_name, 0) + 1
            if name in self.holds:
                self.get_logger().warn(
                    f"RECOVERY '{event.node_name}' fired on {name} while it was "
                    "being held by a yield"
                )

    def _on_gate(self, name, msg):
        """Record whether this robot's SafetyGate is currently blocking."""
        for status in msg.status:
            for pair in status.values:
                if pair.key == "blocked":
                    try:
                        self.gate_blocked[name] = float(pair.value) > 0.5
                    except ValueError:  # pragma: no cover - malformed diagnostic
                        pass

    def _fresh(self, name, now):
        """Return this robot's trajectory as ``(x, y, dt)``, or None if stale."""
        entry = self.trajectories.get(name)
        if entry is None:
            return None
        received, samples = entry
        if now - received > self.stale_s:
            return None
        # dt is recomputed against the CURRENT time on every tick, not cached at
        # arrival: a prediction 0.4 s old describes 0.4 s less of the future, and
        # a conflict test that ignored that would compare two robots' futures
        # from two different presents.
        return [(x, y, arrival - now) for x, y, arrival in samples]

    # -------------------------------------------------------------- arbitration

    def _tick(self):
        """One arbitration cycle: score every pair, then service the holds."""
        now = self.now_s()
        self.last_tick = now
        for name in self.names:
            self._capture_original(name)
        live = {name: self._fresh(name, now) for name in self.names}
        live = {name: traj for name, traj in live.items() if traj}

        for name_a, name_b in combinations(sorted(live), 2):
            self._score_pair(name_a, name_b, live[name_a], live[name_b], now)
        self._expire_unseen(set(live), now)

        for name in list(self.holds):
            self._service_hold(name, live, now)
        self._service_restores(now)

        if self.duration > 0.0 and now - self.started >= self.duration:
            self.write_report()
            rclpy.shutdown()

    def _score_pair(self, name_a, name_b, traj_a, traj_b, now):
        """Advance one pair's conflict state by a single observation."""
        separation, _ = closest_approach(traj_a, traj_b, self.time_window)
        key = (name_a, name_b)
        state = self.live.get(key)

        if separation > self.radius:
            if state is not None and not state["escalated"]:
                self._resolved_locally(key, state, now, separation)
            self.live.pop(key, None)
            return

        if state is None:
            state = {
                "first_seen": now,
                "history": [],
                "escalated": False,
                "minimum": separation,
            }
            self.live[key] = state
            self.get_logger().info(
                f"conflict predicted {name_a}/{name_b}: closest approach "
                f"{separation:.2f} m within {self.time_window:.1f} s "
                f"(radius {self.radius:.2f} m) - local layer has it"
            )
        state["history"].append(separation)
        state["minimum"] = min(state["minimum"], separation)
        state["last_seen"] = now

        if state["escalated"]:
            return
        age = now - state["first_seen"]
        if escalate(age, state["history"], self.escalate_after, self.improvement):
            state["escalated"] = True
            self._escalate(key, state, now, age, separation)

    def _resolved_locally(self, key, state, now, separation):
        """Record a conflict that stopped being one without central action."""
        age = now - state["first_seen"]
        gain = separation - state["history"][0] if state["history"] else 0.0
        self.conflicts_seen.append(
            ConflictRecord(
                pair=key,
                first_seen=state["first_seen"],
                decided_at=now,
                age_s=age,
                outcome="resolved locally",
                minimum_m=state["minimum"],
                gain_m=gain,
                yielder="",
            )
        )
        self.get_logger().info(
            f"conflict {key[0]}/{key[1]} resolved without escalation after "
            f"{age:.1f} s; separation opened to {separation:.2f} m"
        )

    def _escalate(self, key, state, now, age, separation):
        """Take a conflict over from the local layer and start a yield."""
        name_a, name_b = key
        giving_way = yielder(self.fleet, name_a, name_b)
        peer = name_b if giving_way == name_a else name_a
        gain = state["history"][-1] - state["history"][0]
        self.conflicts_seen.append(
            ConflictRecord(
                pair=key,
                first_seen=state["first_seen"],
                decided_at=now,
                age_s=age,
                outcome="escalated",
                minimum_m=state["minimum"],
                gain_m=gain,
                yielder=giving_way,
            )
        )
        if giving_way in self.holds:
            return
        self.get_logger().warn(
            f"ESCALATED {name_a}/{name_b}: unresolved for {age:.1f} s "
            f"(closest approach {separation:.2f} m, opened by {gain:+.2f} m) - "
            f"{giving_way} yields to {peer}"
        )
        record = YieldRecord(
            robot=giving_way,
            peer=peer,
            entered=now,
            separation_entry=separation,
            age_at_escalation=age,
            recoveries_at_entry=self._recovery_total(giving_way),
        )
        self.holds[giving_way] = record
        self.yields.append(record)
        self.pending_restore.pop(giving_way, None)
        self._suppress(giving_way, record)

    def _service_hold(self, name, live, now):
        """Keep one yield commanded, and decide whether it ends this cycle."""
        record = self.holds[name]
        self.yield_publishers[name].publish(Twist())
        record.commands += 1
        if self.gate_blocked.get(name):
            record.gate_blocked_ticks += 1
        record.recoveries_during = (
            self._recovery_total(name) - record.recoveries_at_entry
        )

        traj_a = live.get(name)
        traj_b = live.get(record.peer)
        if traj_a and traj_b:
            separation, _ = closest_approach(traj_a, traj_b, self.time_window)
        else:
            # A peer whose prediction has gone stale cannot be conflicted with.
            # Holding a robot against a trajectory nobody is publishing any more
            # is how a yield becomes a permanent parking brake.
            separation = float("inf")
        record.separation_min = min(record.separation_min, separation)

        held = now - record.entered
        conflict_present = separation <= self.radius * self.release_factor
        ended, reason = release(conflict_present, held, self.min_hold, self.max_hold)
        if not ended:
            if now - record.last_asserted >= self.reassert_period:
                self._suppress(name, record, reassert=True)
            return

        record.released = now
        record.held_s = held
        record.reason = reason
        record.separation_exit = separation
        self.holds.pop(name)
        self.live.pop((min(name, record.peer), max(name, record.peer)), None)
        self.get_logger().info(
            f"RELEASE {name} after {held:.1f} s: {reason} (separation "
            f"{separation:.2f} m, release radius "
            f"{self.radius * self.release_factor:.2f} m); recoveries during the "
            f"hold: {record.recoveries_during}"
        )
        self._begin_restore(name, record, now)

    def _expire_unseen(self, live_names, now):
        """Drop conflict state for pairs that no longer have two predictions."""
        for key in list(self.live):
            if key[0] in live_names and key[1] in live_names:
                continue
            state = self.live.pop(key)
            if not state["escalated"]:
                self._resolved_locally(key, state, now, float("inf"))

    # ------------------------------------------------- Nav2 recovery suppression

    def _recovery_total(self, name):
        """Return every recovery behaviour counted on one robot so far."""
        return sum(self.recoveries[name].values())

    def _capture_original(self, name):
        """Ask controller_server for the allowance it was configured with.

        Captured ONCE, before anything has been suppressed, and reused for every
        restore. Reading the live value at yield entry instead would latch
        SafetyGate's suppressed 1e6 as this node's "original" whenever the two
        overlapped, and the parameter would never come back.
        """
        if name in self.original_allowance:
            return
        client = self.get_clients[name]
        if not client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = [ALLOWANCE_PARAMETER]
        future = client.call_async(request)

        def done(result):
            if result.exception() is not None:  # pragma: no cover - service failure
                return
            values = result.result().values
            if not values or values[0].type != ParameterType.PARAMETER_DOUBLE:
                return
            value = values[0].double_value
            # A value already at the suppressed magnitude is somebody else's
            # write, not a configured default; taking it would make the restore
            # a no-op forever.
            if value >= self.allowance_s / 2.0:
                return
            self.original_allowance[name] = value
            self.get_logger().info(
                f"{name}: {ALLOWANCE_PARAMETER} captured at {value:.1f} s"
            )

        future.add_done_callback(done)

    def _write_allowance(self, name, value, record=None, key=None):
        """Write the allowance, then READ IT BACK into the evidence record."""
        client = self.set_clients[name]
        if not client.service_is_ready():
            return
        parameter = ParameterMsg()
        parameter.name = ALLOWANCE_PARAMETER
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
        )
        request = SetParameters.Request()
        request.parameters = [parameter]
        future = client.call_async(request)

        def done(result):
            if result.exception() is not None:  # pragma: no cover - service failure
                return
            if record is not None and key is not None:
                self._read_back(name, record, key)

        future.add_done_callback(done)

    def _read_back(self, name, record, key):
        """Store what controller_server says the allowance actually is now.

        A mechanism that silently failed to write is otherwise indistinguishable
        from one that worked and was never needed - Phase 2's lesson, and the
        reason both ends of every hold carry a read-back rather than the value
        this node believes it sent.
        """
        client = self.get_clients[name]
        if not client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = [ALLOWANCE_PARAMETER]
        future = client.call_async(request)

        def done(result):
            if result.exception() is not None:  # pragma: no cover - service failure
                return
            values = result.result().values
            if values and values[0].type == ParameterType.PARAMETER_DOUBLE:
                setattr(record, key, values[0].double_value)

        future.add_done_callback(done)

    def _suppress(self, name, record, reassert=False):
        """Raise the yielding robot's progress-checker allowance."""
        record.last_asserted = self.now_s()
        if not self.suppress_recovery:
            return
        if name not in self.original_allowance:
            return
        self._write_allowance(
            name,
            self.allowance_s,
            record=None if reassert else record,
            key=None if reassert else "allowance_entry",
        )
        if not reassert:
            self.get_logger().info(
                f"{name}: recovery suppressed, {ALLOWANCE_PARAMETER} -> "
                f"{self.allowance_s:.0f} s"
            )

    def _begin_restore(self, name, record, now):
        """Arm the restore; it completes once the robot has actually moved.

        Handing a 10 s allowance back to a robot that is still stationary fires
        the recovery the suppression exists to prevent - the robot is released
        into a standstill and takes a moment to accelerate. Same rule, and the
        same two exit conditions, as SafetyGate's own restore.
        """
        if not self.suppress_recovery or name not in self.original_allowance:
            return
        self.pending_restore[name] = {
            "record": record,
            "deadline": now + self.restore_grace,
            "origin": self.positions.get(name),
        }

    def _service_restores(self, now):
        """Complete any restore whose robot has moved or whose grace has run out."""
        for name in list(self.pending_restore):
            entry = self.pending_restore[name]
            origin = entry["origin"]
            position = self.positions.get(name)
            moved = 0.0
            if origin and position:
                moved = math.hypot(position[0] - origin[0], position[1] - origin[1])
            if moved < self.restore_radius and now < entry["deadline"]:
                continue
            self.pending_restore.pop(name)
            self._write_allowance(
                name,
                self.original_allowance[name],
                record=entry["record"],
                key="allowance_exit",
            )
            self.get_logger().info(
                f"{name}: recovery restored, {ALLOWANCE_PARAMETER} -> "
                f"{self.original_allowance[name]:.1f} s after {moved:.2f} m of motion"
            )

    # ------------------------------------------------------------------- report

    def write_report(self):
        """Write the yield evidence, at most once."""
        for name in list(self.holds):
            record = self.holds.pop(name)
            record.released = self.last_tick
            record.held_s = record.released - record.entered
            record.reason = "run ended while still held"
        if self.report_written or not self.results_stem:
            return
        self.report_written = True
        os.makedirs(os.path.dirname(self.results_stem) or ".", exist_ok=True)
        text = report(self)
        print(text, flush=True)
        with open(f"{self.results_stem}.md", "w", encoding="utf-8") as handle:
            handle.write(f"# {self.title}\n\n```\n{text}\n```\n")
        write_csv(self, f"{self.results_stem}.csv")
        self.get_logger().info(f"wrote {self.results_stem}.md")

    def priority_table(self):
        """Return ``[(name, gross mass, footprint radius)]``, highest priority first."""
        return [
            (
                name,
                gross_mass_kg(self._robot(name)),
                footprint_radius(self._robot(name)),
            )
            for name in self.order
        ]


def main(args=None):
    """Spin the fleet's traffic arbiter."""
    rclpy.init(args=args)
    node = TrafficControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Written on the way out as well as on the duration timer: this node is
        # normally shut down by the launch when the mission ends, and evidence
        # that only exists if the run stops the way it was expected to is
        # evidence that goes missing on the interesting runs.
        node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
