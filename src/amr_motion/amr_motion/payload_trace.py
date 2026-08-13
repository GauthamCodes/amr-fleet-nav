"""PHASE 5 EVIDENCE - the command chain, traced stage by stage.

Drives a velocity STEP into the chain's input and records what comes out of
every link, for both robots, unloaded and loaded:

    cmd_vel_nav       what the trace commands (a step - the worst case)
    cmd_vel_smoothed  after the stock nav2_velocity_smoother
    cmd_vel_shaped    after our PayloadJerkAdapter
    odom              what the plant actually did

WHY THIS RUN DOES NOT NAVIGATE

    A step command is the input that separates the links. Under a Nav2 goal the
    controller already produces a smooth reference, so the smoother has almost
    nothing to do and the jerk limiter still less - the trace would show three
    nearly identical curves and prove nothing about either. Commanding the chain
    input directly is the same technique amr_safety.stopping_sweep uses to make
    the gate the only thing that can stop the robot.

    Nav2 is still running, because the stock smoother is one of its lifecycle
    servers. It simply has no goal.

WHAT IS DIFFERENTIATED, AND FROM WHAT

    Jerk is reported from the SHAPED COMMAND, because that is the signal this
    node controls and bounds. Acceleration is reported from both the shaped
    command and from odometry, so the report shows the plant tracking the
    command rather than asserting it. Odometry is not differentiated twice:
    numerically differentiating a sampled velocity signal to third order
    produces a number dominated by quantisation, and quoting it as a measured
    jerk would be quoting noise.
"""

import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from amr_bsp.topics import CMD_NAV, CMD_SHAPED, CMD_SMOOTHED
from amr_description.fleet_config import load_fleet
from amr_motion.jerk_limiter import limits_from_robot, payload_scale
from amr_motion.payload_jerk_adapter import latched_qos


def resample(series, period):
    """Linear-interpolate ``[(t, value), ...]`` onto a uniform time grid.

    THIS IS NOT COSMETIC, AND THE FIRST VERSION OF THIS FILE GOT IT WRONG.

    These samples are timestamped when the trace node RECEIVES them, not when
    the chain published them. The adapter publishes on a strict 20 Hz timer, but
    transport jitter delivers pairs 0.03 s apart as readily as 0.05 s - and
    differentiating straight through that inflates every derivative by the ratio
    of the periods. The first run of this trace reported a peak acceleration of
    0.686 m/s^2 against a configured limit of 0.400, and a peak jerk of 2.900
    against 1.000: a uniform ~1.7x, which is the jitter, not the limiter.

    A measurement that reports a bound being violated when it is not is worse
    than no measurement, because it looks like a finding. Resampling onto the
    grid the signal was actually generated on removes the artefact without
    smoothing away anything real.
    """
    if len(series) < 2 or period <= 0.0:
        return list(series)
    start, end = series[0][0], series[-1][0]
    steps = int((end - start) / period)
    out = []
    index = 0
    for step in range(steps + 1):
        t = start + step * period
        while index + 2 < len(series) and series[index + 1][0] < t:
            index += 1
        (ta, va), (tb, vb) = series[index], series[index + 1]
        if tb <= ta:
            out.append((t, va))
            continue
        ratio = (t - ta) / (tb - ta)
        out.append((t, va + ratio * (vb - va)))
    return out


def _peak_rate(series):
    """Return the peak absolute first derivative of ``[(t, value), ...]``."""
    peak = 0.0
    for (t0, v0), (t1, v1) in zip(series, series[1:]):
        dt = t1 - t0
        if dt <= 0.0:
            continue
        peak = max(peak, abs((v1 - v0) / dt))
    return peak


def _derivative(series):
    """Return ``[(t, dvalue/dt), ...]`` from ``[(t, value), ...]``."""
    out = []
    for (t0, v0), (t1, v1) in zip(series, series[1:]):
        dt = t1 - t0
        if dt <= 0.0:
            continue
        out.append((0.5 * (t0 + t1), (v1 - v0) / dt))
    return out


class PayloadTrace(Node):
    """Step the command chain and record every stage, unloaded then loaded."""

    def __init__(self):
        """Wire a publisher and four subscriptions per robot in the fleet."""
        super().__init__("payload_trace")

        self.declare_parameter("results_dir", os.path.join(os.getcwd(), "results"))
        self.declare_parameter("tag", "payload_trace")
        self.declare_parameter("start_delay_s", 6.0)
        self.declare_parameter("target_v", 0.5)
        self.declare_parameter("hold_s", 7.0)
        self.declare_parameter("settle_s", 5.0)
        self.declare_parameter("sample_hz", 50.0)
        # The chain's own publish period. Derivatives are taken on this grid,
        # not on message arrival times.
        self.declare_parameter("chain_period_s", 0.05)

        self.results_dir = self.get_parameter("results_dir").value
        self.tag = self.get_parameter("tag").value
        self.start_delay = float(self.get_parameter("start_delay_s").value)
        self.target_v = float(self.get_parameter("target_v").value)
        self.hold_s = float(self.get_parameter("hold_s").value)
        self.settle_s = float(self.get_parameter("settle_s").value)
        self.period = float(self.get_parameter("chain_period_s").value)

        self.fleet = load_fleet()
        self.cmd_pub = {}
        self.payload_pub = {}
        self.stage = {}
        for robot in self.fleet:
            name = robot["name"]
            self.cmd_pub[name] = self.create_publisher(Twist, f"/{name}/{CMD_NAV}", 10)
            self.payload_pub[name] = self.create_publisher(
                Float32, f"/{name}/payload_kg", latched_qos()
            )
            self.stage[name] = {"nav": [], "smoothed": [], "shaped": [], "odom": []}
            self._subscribe(name, CMD_NAV, "nav")
            self._subscribe(name, CMD_SMOOTHED, "smoothed")
            self._subscribe(name, CMD_SHAPED, "shaped")
            self.create_subscription(
                Odometry,
                f"/{name}/odom",
                lambda msg, n=name: self.stage[n]["odom"].append(
                    (self.now_s(), msg.twist.twist.linear.x)
                ),
                20,
            )

        # (label, payload multiplier). 0.0 is an empty vehicle; 1.0 is the rated
        # capacity from fleet.yaml. Both arms are the same binary, the same
        # world and the same commanded step - only the payload state differs.
        self.cases = [("unloaded", 0.0), ("loaded", 1.0)]
        self.case_index = -1
        self.case_start = None
        self.windows = {}
        self.started = self.now_s()
        self.state = "waiting"
        self.create_timer(
            1.0 / float(self.get_parameter("sample_hz").value), self._tick
        )

    def _subscribe(self, name, topic, key):
        """Record one command stage's forward velocity."""
        self.create_subscription(
            Twist,
            f"/{name}/{topic}",
            lambda msg, n=name, k=key: self.stage[n][k].append(
                (self.now_s(), msg.linear.x)
            ),
            20,
        )

    def now_s(self):
        """Return the current simulation time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def _publish(self, velocity):
        """Command the same forward velocity into every robot's chain input."""
        msg = Twist()
        msg.linear.x = velocity
        for publisher in self.cmd_pub.values():
            publisher.publish(msg)

    def _begin_case(self, now):
        """Latch the payload for the next case and open its measurement window."""
        self.case_index += 1
        if self.case_index >= len(self.cases):
            self._write_report()
            raise SystemExit(0)
        label, multiplier = self.cases[self.case_index]
        for robot in self.fleet:
            msg = Float32()
            msg.data = float(robot["payload_kg"]) * multiplier
            self.payload_pub[robot["name"]].publish(msg)
            self.get_logger().info(f"{robot['name']}: payload {msg.data:.1f} kg")
        self.case_start = now
        self.windows[label] = [now, None]
        self.state = "driving"

    def _tick(self):
        """Run the state machine and keep the commanded stream alive."""
        now = self.now_s()

        if self.state == "waiting":
            if now - self.started >= self.start_delay:
                self._begin_case(now)
            return

        elapsed = now - self.case_start
        label = self.cases[self.case_index][0]

        if self.state == "driving":
            if elapsed < self.hold_s:
                self._publish(self.target_v)
                return
            self.state = "settling"

        # Settling commands zero rather than falling silent. A stream that stops
        # is stopped by SafetyGate's command timeout, which would make the
        # deceleration a measurement of the gate; commanding zero keeps the
        # deceleration inside the chain, where it is the thing being measured.
        self._publish(0.0)
        if elapsed >= self.hold_s + self.settle_s:
            self.windows[label][1] = now
            self._begin_case(now)

    # -- reporting -----------------------------------------------------------

    def _window(self, series, span):
        """Return the samples of ``series`` inside ``[start, end]``."""
        start, end = span
        end = end if end is not None else float("inf")
        return [(t, v) for t, v in series if start <= t <= end]

    def _case_metrics(self, robot, label):
        """Return the shaped-command and measured metrics for one case."""
        name = robot["name"]
        span = self.windows[label]
        # Resampled onto the chain's own publish period before ANY derivative is
        # taken. See resample()'s docstring for what this run measured before it.
        shaped = resample(self._window(self.stage[name]["shaped"], span), self.period)
        odom = resample(self._window(self.stage[name]["odom"], span), self.period)
        accel = _derivative(shaped)
        return {
            "peak_v_cmd": max((abs(v) for _, v in shaped), default=0.0),
            "peak_v_odom": max((abs(v) for _, v in odom), default=0.0),
            "peak_a_cmd": _peak_rate(shaped),
            "peak_a_odom": _peak_rate(odom),
            "peak_j_cmd": _peak_rate(accel),
            "samples": len(shaped),
        }

    def _write_report(self):
        """Write the markdown report, the per-sample CSV and the plot."""
        os.makedirs(self.results_dir, exist_ok=True)
        stem = os.path.join(self.results_dir, f"phase5_{self.tag}")

        lines = []

        def add(text=""):
            lines.append(text)

        add("# Phase 5 - payload-adaptive velocity and jerk")
        add()
        add("Chain under test, a velocity STEP into its input:")
        add()
        add("```")
        add("cmd_vel_nav -> nav2_velocity_smoother -> cmd_vel_smoothed")
        add("            -> PayloadJerkAdapter     -> cmd_vel_shaped")
        add("            -> twist_mux -> SafetyGate -> cmd_vel")
        add("```")
        add()
        add(f"Commanded step: **{self.target_v:.2f} m/s**, held {self.hold_s:.0f} s,")
        add(f"then commanded zero for {self.settle_s:.0f} s.")
        add()

        add("## Configured limits (fleet.yaml), and what payload does to them")
        add()
        add("| robot | payload | scale | a_max m/s^2 | j_max m/s^3 |")
        add("|---|---|---|---|---|")
        for robot in self.fleet:
            limits = limits_from_robot(robot)
            for label, multiplier in self.cases:
                payload = float(robot["payload_kg"]) * multiplier
                scale = payload_scale(float(robot["base_mass_kg"]), payload)
                add(
                    f"| {robot['name']} | {label} ({payload:.0f} kg) | "
                    f"{scale:.3f} | {limits['max_accel_x'] * scale:.3f} | "
                    f"{limits['max_jerk_x'] * scale:.3f} |"
                )
        add()
        add("Scale is `m_base / (m_base + payload)` - fixed traction force, so")
        add("acceleration available falls as mass rises. The adapter only ever")
        add("scales DOWNWARD from the plant's own kinematic ceiling.")
        add()

        add("## Measured")
        add()
        add(
            "| robot | payload | peak v cmd | peak v odom | peak a cmd | "
            "peak a odom | **peak jerk cmd** |"
        )
        add("|---|---|---|---|---|---|---|")
        metrics = {}
        for robot in self.fleet:
            for label, _ in self.cases:
                m = self._case_metrics(robot, label)
                metrics[(robot["name"], label)] = m
                add(
                    f"| {robot['name']} | {label} | {m['peak_v_cmd']:.3f} | "
                    f"{m['peak_v_odom']:.3f} | {m['peak_a_cmd']:.3f} | "
                    f"{m['peak_a_odom']:.3f} | **{m['peak_j_cmd']:.3f}** |"
                )
        add()
        add("Velocities m/s, accelerations m/s^2, jerk m/s^3. Jerk is taken from")
        add("the shaped command, which is the signal this node bounds.")
        add()
        grid_ms = self.period * 1000.0
        add(f"Every series is resampled onto the chain's own {grid_ms:.0f} ms")
        add("publish grid before it is differentiated. Differentiating by message")
        add("ARRIVAL time instead measures transport jitter: the first run of this")
        add("trace reported 0.686 m/s^2 against a 0.400 limit and 2.900 m/s^3")
        add("against 1.000 - a uniform ~1.7x, which is the ratio of a jittered")
        add("0.03 s gap to the true 0.05 s period, not a violated bound.")
        add("Odometry is differentiated once, not twice: a third-order numeric")
        add("derivative of a sampled velocity is dominated by quantisation, and")
        add("quoting it as measured jerk would be quoting noise.")
        add()

        add("## Does the payload state change the motion?")
        add()
        for robot in self.fleet:
            name = robot["name"]
            unloaded = metrics[(name, "unloaded")]
            loaded = metrics[(name, "loaded")]
            if unloaded["peak_a_cmd"] > 0.0:
                ratio = loaded["peak_a_cmd"] / unloaded["peak_a_cmd"]
                add(
                    f"- **{name}**: peak commanded acceleration "
                    f"{unloaded['peak_a_cmd']:.3f} unloaded -> "
                    f"{loaded['peak_a_cmd']:.3f} loaded "
                    f"(x{ratio:.2f}); rated payload {robot['payload_kg']:.0f} kg "
                    f"on a {robot['base_mass_kg']:.0f} kg chassis."
                )
        add()
        add("The two robots differ because fleet.yaml says they differ - a")
        add("60 kg payload on a 30 kg chassis is a far larger perturbation than")
        add("5 kg on 18 kg, and no code distinguishes them.")
        add()

        with open(f"{stem}.md", "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")

        with open(f"{stem}.csv", "w", encoding="utf-8") as handle:
            handle.write("robot,payload,stage,t,v\n")
            for robot in self.fleet:
                name = robot["name"]
                for label, _ in self.cases:
                    span = self.windows[label]
                    for key in ("nav", "smoothed", "shaped", "odom"):
                        for t, v in self._window(self.stage[name][key], span):
                            handle.write(f"{name},{label},{key},{t:.3f},{v:.4f}\n")

        self._plot(f"{stem}.png")
        self.get_logger().info(f"wrote {stem}.md, .csv and .png")

    def _plot(self, path):
        """Plot velocity and shaped acceleration per robot and payload state."""
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn("matplotlib absent; skipping the plot")
            return

        rows = len(self.fleet)
        fig, axes = plt.subplots(rows, 2, figsize=(12, 4 * rows), squeeze=False)
        for row, robot in enumerate(self.fleet):
            name = robot["name"]
            for label, _ in self.cases:
                span = self.windows[label]
                style = "-" if label == "unloaded" else "--"
                shaped = resample(
                    self._window(self.stage[name]["shaped"], span), self.period
                )
                odom = resample(
                    self._window(self.stage[name]["odom"], span), self.period
                )
                if not shaped:
                    continue
                base = shaped[0][0]
                axes[row][0].plot(
                    [t - base for t, _ in shaped],
                    [v for _, v in shaped],
                    style,
                    label=f"shaped cmd, {label}",
                )
                if odom:
                    axes[row][0].plot(
                        [t - odom[0][0] for t, _ in odom],
                        [v for _, v in odom],
                        style,
                        alpha=0.45,
                        label=f"odom, {label}",
                    )
                accel = _derivative(shaped)
                axes[row][1].plot(
                    [t - base for t, _ in accel],
                    [a for _, a in accel],
                    style,
                    label=f"shaped accel, {label}",
                )
            axes[row][0].set_title(f"{name}: velocity")
            axes[row][0].set_xlabel("s")
            axes[row][0].set_ylabel("m/s")
            axes[row][0].legend(fontsize=7)
            axes[row][0].grid(alpha=0.3)
            axes[row][1].set_title(f"{name}: acceleration of the shaped command")
            axes[row][1].set_xlabel("s")
            axes[row][1].set_ylabel("m/s^2")
            axes[row][1].legend(fontsize=7)
            axes[row][1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)


def main(args=None):
    """Run the payload trace and write its evidence."""
    rclpy.init(args=args)
    node = PayloadTrace()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
