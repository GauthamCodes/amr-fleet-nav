#!/usr/bin/env python3
"""A minimal priority mux, semantically a drop-in for stock ``twist_mux``.

WHY THIS EXISTS RATHER THAN THE STOCK PACKAGE

    PLAN.md section 2 puts stock ``twist_mux`` here, and it is installed - but
    the binary in this ROS Jazzy snapshot does not load:

        /opt/ros/jazzy/lib/twist_mux/twist_mux: symbol lookup error:
        undefined symbol: _ZN18diagnostic_updater7UpdaterC1E...NodeTopicsInterfaceEEdh

    Diagnosed rather than guessed at. The installed ``libdiagnostic_updater.so``
    (ros-jazzy-diagnostic-updater 4.2.6, built 2026-04-12) exports that
    constructor ending ``...NodeTopicsInterfaceEEd`` - one trailing double.
    ``ros-jazzy-twist-mux`` 4.5.0, built 2026-06-15, was compiled against a
    newer header whose signature ends ``EEdh``: double, unsigned char. The two
    packages in this apt snapshot are binary-incompatible with each other, and
    nothing in this workspace can repair that.

    So this node preserves the ARCHITECTURE - a priority mux between the motion
    chain and the fail-closed gate - rather than dropping the requirement or
    routing the yield channel somewhere it does not belong. It reads the SAME
    ``twist_mux.yaml``, with the same ``topics/<name>/{topic,timeout,priority}``
    schema, so restoring the stock node once the packaging is fixed is a change
    of two lines in motion_chain.launch.py and nothing else.

    Deliberately NOT implemented: twist_mux's lock topics and its diagnostics.
    Neither is used here, and the failure above is a warning about shipping code
    whose surface exceeds its use.

SEMANTICS, WHICH ARE THE POINT

    The highest-priority input that has published within its own timeout wins.
    An input that goes quiet stops being a candidate, which is what lets
    TrafficControlNode release a yield by simply ceasing to publish - there is
    no release message that can be lost.

    This node is NOT the safety element and must never be treated as one. It
    fails open by construction: if it dies, its last output is whatever the
    plant latched. SafetyGate is downstream, serial and fail-closed, and that
    ordering is docs/ENGINEERING_NOTES.md rule 1.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class PriorityMux(Node):
    """Forward the highest-priority command source that is still publishing."""

    def __init__(self):
        """Declare each configured input and subscribe to it."""
        super().__init__("twist_mux")

        self.declare_parameter("output_topic", "cmd_vel_mux")
        self.declare_parameter("rate_hz", 20.0)
        # Input names, paired with topics/<name>/{topic,timeout,priority}. ROS
        # parameters have no nested arrays, which is the same reason
        # fleet_mission takes a flat goal list.
        self.declare_parameter("inputs", ["nav", "yield"])

        self.inputs = {}
        for name in self.get_parameter("inputs").value:
            self.declare_parameter(f"topics.{name}.topic", "")
            self.declare_parameter(f"topics.{name}.timeout", 0.5)
            self.declare_parameter(f"topics.{name}.priority", 0)
            topic = self.get_parameter(f"topics.{name}.topic").value
            if not topic:
                raise ValueError(f"input '{name}' has no topic configured")
            self.inputs[name] = {
                "topic": topic,
                "timeout": float(self.get_parameter(f"topics.{name}.timeout").value),
                "priority": int(self.get_parameter(f"topics.{name}.priority").value),
                "last": None,
                "msg": None,
            }
            self.create_subscription(
                Twist, topic, lambda msg, n=name: self._on_input(n, msg), 10
            )

        self.publisher = self.create_publisher(
            Twist, self.get_parameter("output_topic").value, 10
        )
        self.selected = None
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

        table = ", ".join(
            f"{name} <- {cfg['topic']} @ {cfg['timeout']}s p{cfg['priority']}"
            for name, cfg in sorted(
                self.inputs.items(), key=lambda kv: -kv[1]["priority"]
            )
        )
        self.get_logger().info(f"PriorityMux -> {self.publisher.topic_name}: {table}")

    def _on_input(self, name, msg):
        """Record the newest command from one input."""
        self.inputs[name]["msg"] = msg
        self.inputs[name]["last"] = self.get_clock().now()

    def _live(self, name, now):
        """Return whether an input has published inside its own timeout."""
        cfg = self.inputs[name]
        if cfg["last"] is None:
            return False
        return (now - cfg["last"]).nanoseconds * 1e-9 <= cfg["timeout"]

    def _tick(self):
        """Publish the winning input, or nothing when every input is quiet."""
        now = self.get_clock().now()
        live = [name for name in self.inputs if self._live(name, now)]
        if not live:
            # Publish NOTHING rather than a zero. The gate downstream has its
            # own command timeout and will zero the plant itself; emitting a
            # synthetic zero here would keep that timeout permanently fed and
            # disguise an upstream that had stopped talking.
            if self.selected is not None:
                self.get_logger().info("no live input; going quiet")
                self.selected = None
            return

        winner = max(live, key=lambda name: self.inputs[name]["priority"])
        if winner != self.selected:
            self.get_logger().info(
                f"source -> {winner} (priority {self.inputs[winner]['priority']})"
            )
            self.selected = winner
        self.publisher.publish(self.inputs[winner]["msg"])


def main(args=None):
    """Spin one robot's command mux."""
    rclpy.init(args=args)
    node = PriorityMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
