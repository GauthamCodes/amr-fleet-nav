"""The relay machinery every validated sensor channel shares.

A channel is one raw topic in, one validated topic out, with a plausibility check
and optional transform in between. The interesting parts are the two rules it
enforces on every message and the accounting it keeps, because both are what let
Phase 2 make claims rather than assertions.

RULE 4: THE ORIGINAL HEADER STAMP IS PRESERVED, ALWAYS

    Restamping a republished sensor message with "now" is the single most expensive
    mistake available in this layer. It looks harmless - the data is unchanged, the
    delay is milliseconds - and it silently destroys TF: every transform lookup for
    that message resolves against the wrong instant, scan matching drifts, and the
    resulting map is subtly, unreproducibly wrong. It costs a day to find because
    nothing errors. Messages that are not modified are republished as received, and
    the one that IS modified (the scan, by PitchGate) assigns ``header`` across
    explicitly rather than letting a fresh message default it.

RULE 8: THE RELAY IS THE ONLY PATH

    Because everything downstream reads the validated names, this class is the
    single place where a sensor fault can be caught, counted and reported. Which is
    also why it must not be silently lossy - see the note on REJECT in validators.py.

LATENCY IS MEASURED, NOT ASSUMED

    PLAN.md section 4 says the BSP is Python until measurement says otherwise. That
    is only an honest position if the measurement exists, so every channel records
    stamp-to-republish for every message and reports the distribution. A later move
    to C++ should be justified by these numbers or not made.
"""

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from amr_bsp.validators import REJECT, WARN


def _percentile(values, fraction):
    """Return a percentile of a list of numbers, as the Phase 0/1 nodes do."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


class SensorChannel:
    """One raw-to-validated sensor relay, with validation, accounting and latency.

    Args:
        node: The owning rclpy node.
        label: Short name used in logs and diagnostics, e.g. ``"lidar"``.
        message_type: The ROS message class carried on both topics.
        raw_topic: Topic to subscribe to, relative to the node's namespace.
        validated_topic: Topic to republish on.
        validate: ``callable(msg, age, since_previous) -> Verdict``.
        transform: Optional ``callable(msg) -> msg`` applied to accepted messages.
            It is responsible for carrying the original header across.
        observe: Optional ``callable(msg)`` run on accepted messages for their side
            effect only. Kept distinct from ``transform`` so that "this channel
            feeds the pitch buffer" does not masquerade as a message rewrite.
        extra_values: Optional ``callable() -> dict`` merged into this channel's
            diagnostics, for state that belongs to the transform rather than the
            relay.
        depth: Queue depth. RELIABLE to match what the ros_gz_bridge already offers,
            so inserting this relay does not change delivery behaviour that Phase 1
            was verified against.
    """

    def __init__(
        self,
        node,
        label,
        message_type,
        raw_topic,
        validated_topic,
        validate,
        transform=None,
        observe=None,
        extra_values=None,
        depth=5,
    ):
        """Wire the subscription and publication and zero the counters."""
        self.node = node
        self.label = label
        self.validate = validate
        self.transform = transform
        self.observe = observe
        self.extra_values = extra_values

        self.accepted = 0
        self.warned = 0
        self.rejected = 0
        self.reasons = {}
        self.last_reason = ""
        self.relay_ms = []
        self.previous_stamp = None

        qos = QoSProfile(depth=depth)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = node.create_publisher(message_type, validated_topic, qos)
        self.subscription = node.create_subscription(
            message_type, raw_topic, self._on_message, qos
        )
        node.get_logger().info(f"{label}: {raw_topic} -> {validated_topic}")

    def _ages(self, stamp):
        """Return ``(age, since_previous)`` in seconds for a message stamp."""
        stamped = Time.from_msg(stamp)
        now = self.node.get_clock().now()
        # Clamp: under sim time a stamp can lead the clock by a tick while /clock
        # catches up, and a negative age is a clock artefact rather than a fault.
        age = max(0.0, (now - stamped).nanoseconds / 1e9)
        since_previous = None
        if self.previous_stamp is not None:
            since_previous = (stamped - self.previous_stamp).nanoseconds / 1e9
        return stamped, age, since_previous

    def _on_message(self, msg):
        """Validate, optionally transform, and republish preserving the stamp."""
        stamped, age, since_previous = self._ages(msg.header.stamp)
        verdict = self.validate(msg, age, since_previous)

        if verdict.level == REJECT:
            self.rejected += 1
            self.last_reason = verdict.reason
            self.reasons[verdict.reason.split(":")[0]] = (
                self.reasons.get(verdict.reason.split(":")[0], 0) + 1
            )
            self.node.get_logger().warn(
                f"{self.label} REJECT ({self.rejected}): {verdict.reason}",
                throttle_duration_sec=2.0,
            )
            return

        if verdict.level == WARN:
            self.warned += 1
            self.last_reason = verdict.reason
            self.node.get_logger().warn(
                f"{self.label} WARN ({self.warned}): {verdict.reason}",
                throttle_duration_sec=5.0,
            )

        if self.observe is not None:
            self.observe(msg)
        out = self.transform(msg) if self.transform is not None else msg
        self.publisher.publish(out)

        self.accepted += 1
        self.previous_stamp = stamped
        published = self.node.get_clock().now()
        self.relay_ms.append((published - stamped).nanoseconds / 1e6)
        if len(self.relay_ms) > 4000:
            del self.relay_ms[:2000]

    def status(self):
        """Return this channel's DiagnosticStatus for the periodic array."""
        status = DiagnosticStatus()
        status.name = f"SensorBSP {self.label}"
        status.hardware_id = self.label
        if self.rejected:
            status.level = DiagnosticStatus.WARN
            status.message = f"{self.rejected} rejected: {self.last_reason}"
        elif self.warned:
            status.level = DiagnosticStatus.WARN
            status.message = f"{self.warned} warned: {self.last_reason}"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "nominal"
        pairs = {
            "accepted": self.accepted,
            "warned": self.warned,
            "rejected": self.rejected,
            "relay_ms_mean": (
                sum(self.relay_ms) / len(self.relay_ms) if self.relay_ms else math.nan
            ),
            "relay_ms_p95": _percentile(self.relay_ms, 0.95),
            "relay_ms_max": max(self.relay_ms) if self.relay_ms else math.nan,
        }
        if self.extra_values is not None:
            pairs.update(self.extra_values())
        status.values = [KeyValue(key=k, value=str(v)) for k, v in pairs.items()]
        return status

    def summary_line(self):
        """Return a one-line human summary for the shutdown log."""
        mean = sum(self.relay_ms) / len(self.relay_ms) if self.relay_ms else math.nan
        return (
            f"{self.label:<7} accepted {self.accepted:6d}  warned {self.warned:5d}  "
            f"rejected {self.rejected:5d}  relay mean {mean:6.2f} ms  "
            f"p95 {_percentile(self.relay_ms, 0.95):6.2f} ms  "
            f"max {max(self.relay_ms) if self.relay_ms else math.nan:6.2f} ms"
        )


class SensorBSP:
    """Mixin holding a set of channels and publishing their combined diagnostics.

    Kept separate from the node class so the channel set is a plain object the
    concrete node composes, rather than something bolted onto a Node subclass.
    """

    def __init__(self, node, diagnostics_topic="bsp/diagnostics", period_s=1.0):
        """Create the diagnostics publisher and its timer."""
        self.node = node
        self.channels = []
        self.diagnostics = node.create_publisher(DiagnosticArray, diagnostics_topic, 10)
        self.timer = node.create_timer(period_s, self._publish_diagnostics)

    def add(self, channel):
        """Register a channel and return it."""
        self.channels.append(channel)
        return channel

    def _publish_diagnostics(self):
        """Publish one DiagnosticStatus per channel."""
        array = DiagnosticArray()
        array.header.stamp = self.node.get_clock().now().to_msg()
        array.status = [channel.status() for channel in self.channels]
        self.diagnostics.publish(array)

    def log_summary(self):
        """Log one line per channel. Called on shutdown."""
        for channel in self.channels:
            self.node.get_logger().info(channel.summary_line())
