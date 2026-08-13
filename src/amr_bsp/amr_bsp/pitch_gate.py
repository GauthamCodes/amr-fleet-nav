r"""Suppress the ramp's phantom ground return from a pitched 2D scan.

THE PROBLEM, AND WHICH HALF OF IT THIS SOLVES

    A level 2D LiDAR on a ramp produces two quite different false obstacles, and
    only one of them is a pitch effect:

    (A) LEVEL ROBOT, RAMP AHEAD. The ramp surface rises through the horizontal scan
        plane at h/tan(ramp_angle) past the toe - 2.49 m at h = 0.35 m and 8 deg.
        That return is REAL: a real surface really is at that range. It reads as a
        wall only because a 2D scan cannot tell a traversable slope from a barrier.
        Pitch is ~0, so nothing here fires, and nothing here should. That case
        belongs to Phase 4's RampCostLayer.

    (B) PITCHED ROBOT. Once the robot is on the ramp, its scan plane tilts with it
        and sweeps the ground. Beams on the downhill side strike the floor and are
        mapped as an arc of obstacles that does not exist. THIS is the phantom
        return, and this module removes it.

    Measured at h = 0.350 m on an 8.00 deg ramp, 3618 scans - see
    ``results/smoke2_ramp_phantom_return.md``: at max pitch (8.002 deg, nose-UP) the
    nose sector reads ``inf`` and the TAIL sector reads 2.79 m, against a predicted
    h/sin|pitch| of 2.51 m. Which sector sees the ground is set by the sign of the
    pitch, which is why the gate below is per-beam rather than a single radius.

THE GEOMETRY

    Pitch is a rotation about +y, and in ROS a POSITIVE pitch is nose-DOWN. A beam
    at azimuth ``phi`` in the scan plane has body-frame direction
    ``(cos phi, sin phi, 0)``; after the pitch rotation its vertical component is
    ``-sin(theta) cos(phi)``. So the beam is depressed exactly when

        sin(theta) * cos(phi) > 0

    and its depression angle ``delta`` satisfies ``sin(delta) = sin(theta) cos(phi)``.
    The ground therefore lies at SLANT range

        r_ground(phi) = h / (sin(theta) cos(phi))

    This is exact for a rotation about y - not a small-angle approximation.

    Note ``sin``, not ``tan``. ``LaserScan.ranges`` are slant ranges; ``h/tan|theta|``
    is the HORIZONTAL distance to the same intersection. They differ by only 1 % at
    8 deg (2.490 m against 2.515 m), but sin is the correct one and stays correct as
    the ramp angle grows.

    Nose-up and nose-down fall out of the same expression with no branching: at
    theta = -8 deg the tail beams (cos phi < 0) give a positive product and are
    gated, which is precisely what the measurement shows.

WHY A MARGIN, A FLOOR, AND A DEADBAND

    MARGIN (default 0.9) absorbs sensor noise, mast-height tolerance and the fact
    that the ground ahead of a robot on a ramp is not the infinite level plane the
    formula assumes - the measurement came in at 2.79 m against a predicted 2.51 m,
    i.e. FARTHER than nominal, so gating at 0.9 x nominal truncates it with room to
    spare while cutting real returns only if they sit beyond 90 % of the ground
    intersection, where the ground would occlude them anyway.

    FLOOR is the safety interlock. ``r_gate`` is never allowed below
    ``min_gate_range`` (``amr_safety.safety_model.min_gate_range``), which is sized
    as the braking envelope plus the footprint. Truncation therefore CANNOT remove
    an obstacle the SafetyGate would have had to stop for, whatever the pitch.
    Pinned by ``tests/test_safety_distance.py``.

    DEADBAND is not a tuned constant - it falls out. Below
    ``asin(margin * h / range_max)`` the gate radius exceeds the sensor's own
    maximum range, so the gate is a no-op by construction. At h = 0.35 m,
    range_max = 12 m and margin 0.9 that is 1.50 deg.

WHAT A TRUNCATED BEAM BECOMES

    ``inf``, meaning "no return along this ray" - never ``range_max``. The obstacle
    layer runs with ``inf_is_valid: false``, so an infinite beam neither marks nor
    clears. Writing ``range_max`` instead would clear cells out to 8 m on the
    strength of a ray that actually hit the floor, asserting free space we cannot
    see. Beam count, ordering and the header are untouched, so the scan stays a
    well-formed observation and Nav2's only observation source never goes silent
    (ENGINEERING_NOTES rule 2).
"""

import math

#: Fraction of the predicted ground intersection at which a beam is truncated.
DEFAULT_MARGIN = 0.9

#: Depressions below this are floating-point noise, not geometry.
#:
#: Not a tuning knob. ``cos(pi/2)`` evaluates to 6.1e-17 rather than 0, so a beam
#: pointing exactly sideways yields a tiny positive depression and a gate radius of
#: ~1e17 m - finite, therefore not obviously "no ground", and it would flow onward as
#: a real number into anything that plots or compares gate radii. Any depression this
#: small puts the ground intersection past 1e11 m, so treating it as no intersection
#: at all is exact for every purpose this module has.
DEPRESSION_EPSILON = 1e-12


def quaternion_to_roll_pitch(q):
    """Return ``(roll, pitch)`` in radians from a quaternion.

    Duck-typed on ``x``, ``y``, ``z``, ``w``, so it needs no ROS types. Identical
    to the copy the Phase 0 and Phase 1 measurement scripts each carry - those are
    standalone instruments whose recorded results must stay reproducible, so they
    are deliberately not refactored onto this one. Within the package, PitchGate's
    angle comes from here.

    POSITIVE pitch is nose-DOWN (REP-103: y points left, so a positive rotation
    about it tips the nose toward -z). A robot climbing the ramp reports a NEGATIVE
    pitch, and it is the tail beams that then strike the ground.
    """
    sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
    cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return roll, math.asin(sin_pitch)


def ground_range(lidar_height, pitch, azimuth=0.0):
    """Return the slant range at which this beam strikes level ground.

    Args:
        lidar_height: Height of the scan plane above ground, metres.
        pitch: Robot pitch in radians, POSITIVE nose-down (the ROS convention).
        azimuth: Beam bearing in the scan plane, radians, 0 straight ahead.

    Returns:
        The slant range to the ground intersection, or ``inf`` when the beam is
        level or points upward and so never meets the ground.
    """
    depression = math.sin(pitch) * math.cos(azimuth)
    if depression <= DEPRESSION_EPSILON:
        return float("inf")
    return lidar_height / depression


def gate_range(lidar_height, pitch, azimuth=0.0, margin=DEFAULT_MARGIN, floor=0.0):
    """Return the range beyond which a return in this beam is ground, not obstacle.

    Args:
        lidar_height: Height of the scan plane above ground, metres.
        pitch: Robot pitch in radians, positive nose-down.
        azimuth: Beam bearing in the scan plane, radians.
        margin: Fraction of the predicted ground intersection to gate at.
        floor: Lower clamp, metres. The gate is never tightened past this, so a
            steep pitch cannot blind the robot inside its own braking envelope.

    Returns:
        A slant range in metres, or ``inf`` when this beam cannot see the ground.
    """
    nominal = ground_range(lidar_height, pitch, azimuth)
    if math.isinf(nominal):
        return nominal
    return max(floor, margin * nominal)


def deadband_pitch(lidar_height, range_max, margin=DEFAULT_MARGIN):
    """Return the pitch below which the gate cannot truncate anything.

    Below this angle the gate radius exceeds the sensor's maximum range, so the
    gate is inactive by construction rather than by a threshold someone chose.
    """
    ratio = margin * lidar_height / range_max
    if ratio >= 1.0:
        return math.pi / 2.0
    return math.asin(ratio)


def gate_is_active(lidar_height, pitch, range_max, margin=DEFAULT_MARGIN):
    """Return whether any beam could be truncated at this pitch."""
    return abs(pitch) > deadband_pitch(lidar_height, range_max, margin)


def gate_profile(
    angle_min,
    angle_increment,
    count,
    lidar_height,
    pitch,
    margin=DEFAULT_MARGIN,
    floor=0.0,
):
    """Return the per-beam gate radius, for plotting and for the evidence report."""
    return [
        gate_range(
            lidar_height, pitch, angle_min + index * angle_increment, margin, floor
        )
        for index in range(count)
    ]


def truncate_scan(
    ranges,
    angle_min,
    angle_increment,
    lidar_height,
    pitch,
    margin=DEFAULT_MARGIN,
    floor=0.0,
):
    """Replace ground returns with ``inf``, leaving every other beam untouched.

    Args:
        ranges: The scan's range list. Not modified; a new list is returned.
        angle_min: Bearing of beam 0, radians.
        angle_increment: Bearing step between beams, radians.
        lidar_height: Height of the scan plane above ground, metres.
        pitch: Robot pitch in radians, positive nose-down.
        margin: Fraction of the predicted ground intersection to gate at.
        floor: Lower clamp on the gate radius, metres.

    Returns:
        A ``(ranges, truncated_count)`` pair. The list has the same length and
        ordering as the input, so the result is still a well-formed scan.
    """
    sin_pitch = math.sin(pitch)
    if sin_pitch == 0.0:
        return list(ranges), 0

    gated = list(ranges)
    truncated = 0
    for index, distance in enumerate(gated):
        if math.isinf(distance) or math.isnan(distance):
            continue
        depression = sin_pitch * math.cos(angle_min + index * angle_increment)
        if depression <= DEPRESSION_EPSILON:
            continue
        limit = max(floor, margin * lidar_height / depression)
        if distance > limit:
            gated[index] = float("inf")
            truncated += 1
    return gated, truncated
