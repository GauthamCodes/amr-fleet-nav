"""Plausibility checks for one IMU, LiDAR or camera message.

Pure functions, no ROS types. Every validator takes an object that merely *exposes*
the message's fields, so the whole validation layer is unit-testable without a
simulator, a graph, or even rclpy installed - the same property that makes
``amr_navigation.clearance`` testable, and for the same reason.

WHY TIME IS PASSED IN RATHER THAN READ

    A validator that calls ``node.get_clock().now()`` cannot be tested without a
    clock, and under ``use_sim_time`` it would silently read wall time in a unit
    test. Both age checks take the already-computed interval as an argument:
    ``age`` is how long ago the message was stamped, ``since_previous`` is the gap
    to the last ACCEPTED message of the same stream. Passing ``None`` skips that
    check, which is what lets a test exercise one dimension at a time.

WHAT "REJECT" COSTS

    A rejected sample is not republished. For the IMU at 100 Hz that is cheap - the
    PitchGate holds its last good pitch and the next sample arrives 10 ms later. For
    the LiDAR at 10 Hz it is not cheap, because a dropped scan is a gap in Nav2's
    only observation source, so the LiDAR checks are deliberately structural
    (geometry that cannot be interpreted) rather than statistical (a scan that looks
    unusual). Individual bad BEAMS are handled by PitchGate and by the obstacle
    layer's own range filtering, never by dropping the scan around them. See
    docs/ENGINEERING_NOTES.md rule 2.
"""

import math

#: Validation outcomes. OK and WARN are both republished; REJECT is not.
OK = "OK"
WARN = "WARN"
REJECT = "REJECT"

#: A quaternion whose norm is off by more than this is not a rotation.
QUATERNION_NORM_TOLERANCE = 1e-3

#: Encodings the camera validator will accept, mapped to their bytes per pixel.
CAMERA_ENCODINGS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


class Verdict:
    """The outcome of validating a single sensor message.

    Attributes:
        level: One of ``OK``, ``WARN`` or ``REJECT``.
        reason: Empty for ``OK``; otherwise a human-readable explanation that names
            the offending quantity and its value, because a log line saying only
            "implausible IMU" costs an engineer the same debugging session twice.
    """

    def __init__(self, level, reason=""):
        """Store the level and its explanation."""
        self.level = level
        self.reason = reason

    @property
    def accepted(self):
        """Return whether the message may be republished."""
        return self.level != REJECT

    def __bool__(self):
        """Return :attr:`accepted`, so a verdict reads naturally in an ``if``."""
        return self.accepted

    def __repr__(self):
        """Return a debug representation naming the level and reason."""
        return f"Verdict({self.level}, {self.reason!r})"


#: The single OK verdict, shared because it carries no per-message state.
_OK = Verdict(OK)


def _finite(*values):
    """Return whether every value is a finite number."""
    return all(not (math.isnan(v) or math.isinf(v)) for v in values)


def _check_timing(age, max_age, since_previous, label):
    """Return a REJECT verdict if a message is stale or out of order, else None.

    Shared by all three validators because staleness and stamp regression are
    properties of the transport, not of the sensor.
    """
    if age is not None and age > max_age:
        return Verdict(
            REJECT, f"stale {label}: stamped {age:.3f} s ago > {max_age:.3f}"
        )
    if since_previous is not None and since_previous <= 0.0:
        return Verdict(
            REJECT,
            f"{label} stamp did not advance: {since_previous:+.6f} s since the last "
            f"accepted message",
        )
    return None


def validate_imu(
    imu,
    max_angular_rate,
    max_linear_accel,
    age=None,
    max_age=0.5,
    since_previous=None,
):
    """Validate one IMU message.

    Args:
        imu: Any object exposing ``angular_velocity``, ``linear_acceleration`` and
            ``orientation``, each with the usual component attributes.
        max_angular_rate: Largest plausible |w| on any axis, rad/s. Derived per
            robot from ``max_vel_theta`` rather than hardcoded.
        max_linear_accel: Largest plausible acceleration magnitude, m/s^2. This is
            SPECIFIC FORCE and therefore includes gravity: a stationary, level IMU
            reads 9.81, so any bound below that rejects a healthy sensor at rest.
        age: Seconds since the message was stamped, or None to skip the check.
        max_age: Largest tolerable age, seconds.
        since_previous: Seconds between this stamp and the last accepted one, or
            None to skip the check.

    Returns:
        A :class:`Verdict`. The angular-rate check is the primary demonstrated
        check per PLAN.md section 3.
    """
    w = imu.angular_velocity
    a = imu.linear_acceleration
    q = imu.orientation

    if not _finite(w.x, w.y, w.z):
        return Verdict(REJECT, f"non-finite angular velocity ({w.x}, {w.y}, {w.z})")
    if not _finite(a.x, a.y, a.z):
        return Verdict(REJECT, f"non-finite linear acceleration ({a.x}, {a.y}, {a.z})")
    if not _finite(q.x, q.y, q.z, q.w):
        return Verdict(REJECT, f"non-finite orientation ({q.x}, {q.y}, {q.z}, {q.w})")

    worst_axis, worst_rate = max(
        (("x", abs(w.x)), ("y", abs(w.y)), ("z", abs(w.z))), key=lambda pair: pair[1]
    )
    if worst_rate > max_angular_rate:
        return Verdict(
            REJECT,
            f"implausible angular velocity: |w_{worst_axis}| = {worst_rate:.3f} rad/s "
            f"> {max_angular_rate:.3f} rad/s",
        )

    accel = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
    if accel > max_linear_accel:
        return Verdict(
            REJECT,
            f"implausible linear acceleration: |a| = {accel:.3f} m/s^2 "
            f"> {max_linear_accel:.3f} m/s^2",
        )

    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        return Verdict(
            REJECT, f"orientation is not a unit quaternion: |q| = {norm:.6f}"
        )

    timing = _check_timing(age, max_age, since_previous, "IMU")
    return timing if timing is not None else _OK


def validate_lidar(
    scan,
    max_nan_fraction=0.25,
    age=None,
    max_age=0.5,
    since_previous=None,
):
    """Validate one LaserScan.

    Args:
        scan: Any object exposing ``ranges``, ``angle_min``, ``angle_max``,
            ``angle_increment``, ``range_min`` and ``range_max``.
        max_nan_fraction: NaN beams above this fraction reject the scan; any NaN at
            all below it produces a WARN. ``inf`` is NOT counted - it is the
            sensor's correct way to report "nothing within range" and an open aisle
            legitimately produces a scan that is almost entirely infinite.
        age: Seconds since the message was stamped, or None to skip the check.
        max_age: Largest tolerable age, seconds.
        since_previous: Seconds between this stamp and the last accepted one, or
            None to skip the check.

    Returns:
        A :class:`Verdict`.
    """
    ranges = scan.ranges
    count = len(ranges)
    if count < 2:
        return Verdict(REJECT, f"scan carries {count} beams")

    if not _finite(scan.angle_min, scan.angle_max, scan.angle_increment):
        return Verdict(REJECT, "non-finite scan geometry")
    if scan.angle_increment == 0.0:
        return Verdict(REJECT, "angle_increment is zero, so no beam has a bearing")

    # Tolerant by one increment because a full-circle scan may be indexed either
    # (max - min) / n or (max - min) / (n - 1) depending on whether the last beam
    # coincides with the first. Both are legitimate; a geometry that disagrees by
    # more than a beam is not.
    span = abs(scan.angle_max - scan.angle_min)
    implied = abs(scan.angle_increment) * (count - 1)
    if abs(span - implied) > 2.0 * abs(scan.angle_increment):
        return Verdict(
            REJECT,
            f"scan geometry is inconsistent: {count} beams x "
            f"{scan.angle_increment:.6f} rad spans {implied:.4f} rad, but "
            f"angle_max - angle_min is {span:.4f} rad",
        )

    if not _finite(scan.range_min, scan.range_max):
        return Verdict(REJECT, "non-finite range band")
    if scan.range_min < 0.0 or scan.range_max <= scan.range_min:
        return Verdict(
            REJECT,
            f"invalid range band [{scan.range_min:.3f}, {scan.range_max:.3f}]",
        )

    nan_count = sum(1 for r in ranges if math.isnan(r))
    if nan_count == count:
        return Verdict(REJECT, f"every one of {count} beams is NaN")

    timing = _check_timing(age, max_age, since_previous, "scan")
    if timing is not None:
        return timing

    if nan_count:
        fraction = nan_count / count
        if fraction > max_nan_fraction:
            return Verdict(
                REJECT,
                f"{nan_count}/{count} beams ({fraction:.1%}) are NaN, above the "
                f"{max_nan_fraction:.0%} limit",
            )
        return Verdict(
            WARN, f"{nan_count}/{count} beams ({fraction:.1%}) are NaN, republished"
        )
    return _OK


def validate_camera(
    image,
    expected_width,
    expected_height,
    age=None,
    max_age=0.5,
    since_previous=None,
):
    """Validate one camera Image.

    Args:
        image: Any object exposing ``width``, ``height``, ``encoding``, ``step`` and
            ``data``.
        expected_width: Width the URDF configures the sensor for.
        expected_height: Height the URDF configures the sensor for.
        age: Seconds since the message was stamped, or None to skip the check.
        max_age: Largest tolerable age, seconds.
        since_previous: Seconds between this stamp and the last accepted one, or
            None to skip the check.

    Returns:
        A :class:`Verdict`. Resolution is checked against the configured value
        rather than merely against zero, because a silently renegotiated resolution
        is exactly the failure that turns a downstream pixel coordinate into a
        wrong metric one.
    """
    if image.width <= 0 or image.height <= 0:
        return Verdict(REJECT, f"degenerate image {image.width}x{image.height}")
    if image.width != expected_width or image.height != expected_height:
        return Verdict(
            REJECT,
            f"resolution {image.width}x{image.height} is not the configured "
            f"{expected_width}x{expected_height}",
        )

    if image.encoding not in CAMERA_ENCODINGS:
        return Verdict(
            REJECT,
            f"unsupported encoding {image.encoding!r}, expected one of "
            f"{sorted(CAMERA_ENCODINGS)}",
        )

    minimum_step = image.width * CAMERA_ENCODINGS[image.encoding]
    if image.step < minimum_step:
        return Verdict(
            REJECT,
            f"step {image.step} is below {minimum_step} for {image.width} px of "
            f"{image.encoding}",
        )
    if len(image.data) != image.step * image.height:
        return Verdict(
            REJECT,
            f"truncated image: {len(image.data)} bytes for a {image.step} x "
            f"{image.height} buffer",
        )

    timing = _check_timing(age, max_age, since_previous, "image")
    return timing if timing is not None else _OK
