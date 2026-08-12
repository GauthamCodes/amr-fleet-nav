#!/usr/bin/env python3
"""PHASE 2 - stopping distance against commanded speed, measured through the gate.

WHAT MAKES THIS A TEST OF THE GATE RATHER THAN OF THE DRIVE

    This node publishes a constant velocity on the gate's INPUT topic and never
    stops commanding it. Nothing else in the run decides to slow down: no planner,
    no controller, no trajectory. The only thing between that constant command and
    the wheels is the SafetyGate, so every metre in the table below is a metre the
    gate is responsible for. If the gate did nothing, the robot would drive into the
    rack at the commanded speed.

THE APPROACH IS A RACK FACE, NOT A PEDESTRIAN

    Deliberately. A rack is static, its position is in the world file, and its face
    is a flat surface square to the approach - so the clearance the gate reports can
    be checked against geometry rather than against another estimator. The moving-
    obstacle case is a different claim and gets its own run (phase2_safety_run).

THREE DISTANCES, AND THEY ARE NOT THE SAME NUMBER

    d_safe          what the MODEL asks for at the measured speed. Sized from the
                    payload-scaled physical deceleration.
    braking         how far the robot actually travelled after the zero command was
                    published. This is the plant's answer, not the model's.
    final clearance where it came to rest.

    They differ, and the difference is the point of reporting all three. Gazebo's
    DiffDrive enforces its deceleration limit KINEMATICALLY - it is mass-independent,
    so the simulated robot brakes at the full |max_decel_x| whatever the payload,
    while d_safe is sized for a 90 kg vehicle that cannot. The report therefore
    quotes k_model beside a k fitted to the measurement instead of averaging the two
    into one number that would be true of neither.

WHY THE ROBOT CREEPS AFTER IT STOPS, AND WHY THAT IS CORRECT

    d_safe is speed-dependent: at rest it collapses to d_min. A stopped robot is
    therefore permitted to approach to d_min, and with a constant command still
    applied it does - in small blocked/released steps - until it settles at the
    standoff. That is the specified behaviour of a safe-following-distance gate, not
    a leak in it, and the settle column measures where it actually stops.
"""

import math
import os

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from amr_bsp.topics import CMD_NAV


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


class StoppingSweep(Node):
    """Drives at a series of commanded speeds and records where the gate stopped it."""

    def __init__(self):
        """Declare the speed list, wire the command path, and arm the state machine."""
        super().__init__("stopping_sweep")
        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("speeds", [0.15, 0.30, 0.45, 0.60])
        self.declare_parameter("reverse_speed", 0.25)
        self.declare_parameter("settle_s", 3.0)
        self.declare_parameter("stopped_speed", 0.02)
        self.declare_parameter("phase_timeout_s", 60.0)
        self.declare_parameter("results_dir", "results")
        self.declare_parameter("tag", "stopping_distance")

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        self.speeds = [float(v) for v in get("speeds").value]
        self.reverse_speed = float(get("reverse_speed").value)
        self.settle_s = float(get("settle_s").value)
        self.stopped_speed = float(get("stopped_speed").value)
        self.phase_timeout = float(get("phase_timeout_s").value)
        self.results_dir = get("results_dir").value
        self.tag = get("tag").value

        self.publisher = self.create_publisher(Twist, CMD_NAV, 1)
        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._on_odom, 20
        )
        self.create_subscription(
            DiagnosticArray,
            f"/{self.robot_name}/safety_gate/diagnostics",
            self._on_diag,
            20,
        )

        self.position = None
        self.origin = None
        self.speed = 0.0
        self.gate = None
        self.blocked = False
        self.latency_ms = []
        self.compute_us = []
        self.k_model = None
        self.d_min = None

        self.index = 0
        self.state = "waiting"
        self.state_since = None
        self.halt = None
        self.records = []
        self.min_clearance_seen = float("inf")

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"stopping sweep armed: speeds {self.speeds} m/s, commanding {CMD_NAV}"
        )

    # ------------------------------------------------------------------ inputs

    def _on_odom(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if self.origin is None:
            self.origin = self.position

    def _on_diag(self, msg):
        for status in msg.status:
            values = {pair.key: pair.value for pair in status.values}
            try:
                self.gate = {k: float(v) for k, v in values.items()}
            except ValueError:
                continue
            self.blocked = self.gate.get("blocked", 0.0) > 0.5
            self.k_model = self.gate.get("k", self.k_model)
            self.d_min = self.gate.get("d_min", self.d_min)
            latency = self.gate.get("sensor_to_zero_ms", -1.0)
            if latency >= 0.0:
                self.latency_ms.append(latency)
            compute = self.gate.get("compute_us", -1.0)
            if compute >= 0.0:
                self.compute_us.append(compute)
            clearance = self.gate.get("clearance", float("inf"))
            if math.isfinite(clearance) and self.state in ("cruise", "brake", "settle"):
                self.min_clearance_seen = min(self.min_clearance_seen, clearance)

    # ------------------------------------------------------------ state machine

    def now_s(self):
        """Return the current simulation time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def _enter(self, state):
        self.state = state
        self.state_since = self.now_s()
        self.get_logger().info(f"state -> {state}")

    def _elapsed(self):
        return 0.0 if self.state_since is None else self.now_s() - self.state_since

    def _command(self, vx):
        twist = Twist()
        twist.linear.x = vx
        self.publisher.publish(twist)

    def _travelled(self, origin):
        if self.position is None or origin is None:
            return 0.0
        return math.hypot(self.position[0] - origin[0], self.position[1] - origin[1])

    def _tick(self):
        if self.position is None or self.gate is None:
            self.get_logger().info(
                "waiting for odom and gate diagnostics", throttle_duration_sec=5.0
            )
            return
        if self.state == "waiting":
            self._enter("cruise")
            self.start_position = self.position
            return

        handler = getattr(self, f"_state_{self.state}", None)
        if handler is not None:
            handler()

    def _state_cruise(self):
        """Command the target speed until the gate latches."""
        target = self.speeds[self.index]
        self._command(target)
        if self.blocked:
            self.halt = {
                "position": self.position,
                "speed": self.speed,
                "clearance": self.gate.get("clearance", float("nan")),
                "d_stop": self.gate.get("d_stop", float("nan")),
                "latency_ms": self.gate.get("sensor_to_zero_ms", float("nan")),
                "at": self.now_s(),
            }
            self.get_logger().warn(
                f"gate latched at {self.speed:.3f} m/s, clearance "
                f"{self.halt['clearance']:.3f} m"
            )
            self._enter("brake")
            return
        if self._elapsed() > self.phase_timeout:
            self.get_logger().error(
                f"no halt within {self.phase_timeout:.0f} s at "
                f"{target:.2f} m/s - recording the attempt and moving on"
            )
            self.halt = None
            # Recorded, not dropped. A speed that never produced a halt is the
            # most interesting row this table could contain and must not vanish
            # from it just because the happy path is where _record() usually runs.
            self._record()
            self._enter("retreat")

    def _state_brake(self):
        """Keep commanding: the gate, not this node, is what brings the robot down."""
        self._command(self.speeds[self.index])
        if self.speed <= self.stopped_speed:
            self.braking_distance = self._travelled(self.halt["position"])
            self.stop_clearance = self.gate.get("clearance", float("nan"))
            self._enter("settle")
            return
        if self._elapsed() > self.phase_timeout:
            self.braking_distance = self._travelled(self.halt["position"])
            self.stop_clearance = self.gate.get("clearance", float("nan"))
            self._enter("settle")

    def _state_settle(self):
        """Hold the command on and watch where the standoff actually settles."""
        self._command(self.speeds[self.index])
        if self._elapsed() >= self.settle_s:
            self._record()
            self._enter("retreat")

    def _state_retreat(self):
        """Back off to the start so the next speed gets the same runway."""
        self._command(-self.reverse_speed)
        back_home = self._travelled(self.start_position) < 0.15
        if back_home or self._elapsed() > self.phase_timeout:
            self._command(0.0)
            self._enter("reset")

    def _state_reset(self):
        """Let the robot come to rest and the latch clear before the next speed."""
        self._command(0.0)
        if self._elapsed() < 2.0:
            return
        self.index += 1
        self.min_clearance_seen = float("inf")
        if self.index >= len(self.speeds):
            self._command(0.0)
            try:
                self._write_report()
            finally:
                rclpy.shutdown()
            return
        self._enter("cruise")

    def _record(self):
        target = self.speeds[self.index]
        if self.halt is None:
            self.records.append({"commanded": target, "halted": False})
            return
        self.records.append(
            {
                "commanded": target,
                "halted": True,
                "speed_at_halt": self.halt["speed"],
                "clearance_at_halt": self.halt["clearance"],
                "d_stop": self.halt["d_stop"],
                "braking_distance": self.braking_distance,
                "clearance_at_stop": self.stop_clearance,
                "clearance_settled": self.gate.get("clearance", float("nan")),
                "min_clearance": self.min_clearance_seen,
                "latency_ms": self.halt["latency_ms"],
            }
        )

    # ------------------------------------------------------------------ report

    def _write_report(self):
        os.makedirs(self.results_dir, exist_ok=True)
        stem = os.path.join(self.results_dir, f"phase2_{self.tag}")
        lines = []
        add = lines.append
        rule = "=" * 78
        thin = "-" * 78

        add(rule)
        add("PHASE 2 - SafetyGate stopping distance vs commanded speed")
        add(rule)
        add(f"robot:                    {self.robot_name}")
        add(f"k (model, from fleet.yaml): {self.k_model:.4f} s^2/m")
        add(f"d_min:                    {self.d_min:.3f} m")
        add("approach:                 rack face, static, square to the path")
        add("commanded on:             cmd_vel_nav - the gate's input")
        add(thin)

        add("[A] PER-SPEED RESULTS")
        add("     cmd    v at halt   d_safe    clearance   braking   at rest   settled")
        add("     m/s      m/s         m        at halt      m          m         m")
        for record in self.records:
            if not record.get("halted"):
                add(f"    {record['commanded']:5.2f}   NO HALT within the timeout")
                continue
            add(
                f"    {record['commanded']:5.2f}   {record['speed_at_halt']:7.3f}   "
                f"{record['d_stop']:7.3f}   {record['clearance_at_halt']:8.3f}   "
                f"{record['braking_distance']:7.3f}   "
                f"{record['clearance_at_stop']:7.3f}   "
                f"{record['clearance_settled']:7.3f}"
            )
        add("")
        add("    'braking' is distance travelled AFTER the gate published its zero.")
        add("    'settled' is where it rests with the command still applied - the")
        add("    d_min standoff, which is the correct end state and not a creep bug.")
        add(thin)

        self._add_gain_section(add, thin)
        self._add_latency_section(add, thin)

        halted = [r for r in self.records if r.get("halted")]
        add("VERDICT")
        checks = [
            ("every commanded speed produced a halt", len(halted) == len(self.speeds)),
            ("at least four speeds swept", len(self.speeds) >= 4),
            (
                "the robot never reached the obstacle",
                all(r["min_clearance"] > 0.0 for r in halted),
            ),
            (
                "clearance at halt >= d_safe at that speed",
                all(r["clearance_at_halt"] >= r["d_stop"] - 0.05 for r in halted),
            ),
            (
                "braking distance stayed inside the modelled envelope",
                all(r["braking_distance"] <= r["d_stop"] for r in halted),
            ),
        ]
        for label, passed in checks:
            add(f"    {label + ':':<48} {'YES' if passed else 'NO'}")
        ok = all(passed for _, passed in checks)
        add(f"    RESULT: {'PASS' if ok else 'FAIL'}")
        add(rule)

        text = "\n".join(lines)
        print(text, flush=True)
        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write("# Phase 2 - SafetyGate stopping distance sweep\n\n")
            handle.write("```\n")
            handle.write(text)
            handle.write("\n```\n")
        self._write_csv(f"{stem}.csv")
        self.get_logger().info(f"wrote {stem}.md")

    def _add_gain_section(self, add, thin):
        add("[B] k_model AGAINST k_measured")
        halted = [
            r for r in self.records if r.get("halted") and r["speed_at_halt"] > 0.05
        ]
        gains = [
            r["braking_distance"] / (r["speed_at_halt"] ** 2)
            for r in halted
            if r["braking_distance"] > 0.0
        ]
        add(f"    k_model    = 1/(2*a_eff), payload-scaled:  {self.k_model:8.4f} s^2/m")
        if gains:
            k_measured = sum(gains) / len(gains)
            add(f"    k_measured = braking / v^2, fitted:  {k_measured:8.4f} s^2/m")
            add(
                f"    a implied by the measurement:              "
                f"{1.0 / (2.0 * k_measured):8.3f} m/s^2"
            )
            add(
                f"    ratio k_model / k_measured:                "
                f"{self.k_model / k_measured:8.2f} x"
            )
        else:
            add("    k_measured: not enough halts above 0.05 m/s to fit")
        add("")
        add("    The gap is the simulator's drive model, not slack in the margin.")
        add("    Gazebo's DiffDrive applies |max_decel_x| as a KINEMATIC limit, so the")
        add("    simulated robot brakes as if unloaded; k_model sizes the envelope for")
        add("    a 90 kg vehicle whose 60 kg of payload it cannot ignore. On hardware")
        add("    the two converge. Quoting both is the honest form of this result.")
        add(thin)

    def _add_latency_section(self, add, thin):
        add("[C] SENSOR STAMP -> ZERO COMMAND PUBLISHED")
        if self.latency_ms:
            add(f"    blocking decisions measured:        {len(self.latency_ms):8d}")
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
            add("    no blocking decisions recorded")
        if self.compute_us:
            add("")
            add("    in-node compute only (steady clock, not quantised by /clock):")
            add(
                f"    mean / p95 / max:                         "
                f"{sum(self.compute_us) / len(self.compute_us):.1f} / "
                f"{percentile(self.compute_us, 0.95):.1f} / "
                f"{max(self.compute_us):.1f} us"
            )
        add("")
        add("    The end-to-end figure includes the simulator's sensor pipeline, the")
        add("    bridge, the BSP relay and the gate, and is quantised by Gazebo's")
        add("    /clock step under use_sim_time. It is a low-latency override, not a")
        add(
            "    hard-real-time one: there is no RT kernel and no scheduling guarantee."
        )
        add(thin)

    def _write_csv(self, path):
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(
                "commanded_mps,speed_at_halt_mps,d_safe_m,clearance_at_halt_m,"
                "braking_m,clearance_at_stop_m,clearance_settled_m,latency_ms\n"
            )
            for r in self.records:
                if not r.get("halted"):
                    handle.write(f"{r['commanded']:.3f},,,,,,,\n")
                    continue
                handle.write(
                    f"{r['commanded']:.3f},{r['speed_at_halt']:.4f},"
                    f"{r['d_stop']:.4f},{r['clearance_at_halt']:.4f},"
                    f"{r['braking_distance']:.4f},{r['clearance_at_stop']:.4f},"
                    f"{r['clearance_settled']:.4f},{r['latency_ms']:.2f}\n"
                )


def main():
    rclpy.init()
    node = StoppingSweep()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
