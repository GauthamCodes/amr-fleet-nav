"""PitchGate's truncation geometry, checked against the measured ramp.

The sizing table at the bottom is the one that matters. It replays the per-pitch-bin
closest returns that smoke test 2 actually recorded at h = 0.350 m
(``results/smoke2_ramp_phantom_return.md``) and asserts the gate splits them the
right way: from 4 deg up the closest return in the pitched sector IS the ground and
gets truncated, while at 3 deg and below it is nearer than the ground could possibly
be - real racks - and survives untouched. A gate that truncated everything would
pass a "phantom suppressed" test and quietly blind the robot; this table is what
distinguishes the two.

Pitch sign follows ROS: POSITIVE is nose-down. A robot climbing the ramp reports
NEGATIVE pitch, and it is the TAIL beams that strike the ground - which is exactly
what the smoke test saw (nose sector inf, tail sector 2.79 m at max pitch).
"""

import math

import pytest

from amr_bsp.pitch_gate import (
    deadband_pitch,
    DEFAULT_MARGIN,
    gate_is_active,
    gate_profile,
    gate_range,
    ground_range,
    truncate_scan,
)

LIDAR_HEIGHT = 0.35
RANGE_MAX = 12.0
RAMP_PITCH = math.radians(8.002)

#: Four cardinal beams: forward, left, rear, right.
FORWARD, LEFT, REAR, RIGHT = 0, 1, 2, 3
CARDINAL = {"angle_min": 0.0, "angle_increment": math.pi / 2.0}

#: Closest return per |pitch| bin, measured over 3618 scans on the 8.00 deg ramp.
#: results/smoke2_ramp_phantom_return.md, "PITCH GATE SIZING (input to Phase 2)".
MEASURED_CLOSEST_BY_DEGREE = {
    0: 0.69,
    1: 1.48,
    2: 1.56,
    3: 5.72,
    4: 4.64,
    5: 3.75,
    6: 3.32,
    7: 2.95,
    8: 2.77,
}


def cardinal_scan(forward=10.0, left=10.0, rear=10.0, right=10.0):
    return [forward, left, rear, right]


# -------------------------------------------------------------------- geometry


def test_the_ground_intersection_uses_sine_not_tangent():
    """Ranges are SLANT ranges. h/tan is the horizontal distance to the same point."""
    slant = ground_range(LIDAR_HEIGHT, RAMP_PITCH)
    horizontal = LIDAR_HEIGHT / math.tan(RAMP_PITCH)
    assert slant == pytest.approx(2.514, abs=1e-3)
    assert horizontal == pytest.approx(2.490, abs=1e-3)
    assert slant > horizontal


def test_the_measured_phantom_wall_offset_is_reproduced():
    """2.49 m past the toe, which smoke test 2 measured independently."""
    assert LIDAR_HEIGHT / math.tan(math.radians(8.0)) == pytest.approx(2.490, abs=1e-3)


def test_a_level_scan_never_meets_the_ground():
    assert math.isinf(ground_range(LIDAR_HEIGHT, 0.0))


def test_beams_pointing_upward_never_meet_the_ground():
    """Nose-down pitches the REAR beams up, and vice versa."""
    assert math.isinf(ground_range(LIDAR_HEIGHT, RAMP_PITCH, azimuth=math.pi))
    assert math.isinf(ground_range(LIDAR_HEIGHT, -RAMP_PITCH, azimuth=0.0))


def test_side_beams_are_never_gated_at_any_pitch():
    """cos(90 deg) is zero, so a side beam has no depression and sees no ground."""
    for degrees in range(0, 46):
        radius = gate_range(LIDAR_HEIGHT, math.radians(degrees), azimuth=math.pi / 2.0)
        assert math.isinf(radius), f"side beam gated at {degrees} deg"


def test_nose_up_gates_the_tail_exactly_as_measured():
    """The measurement: at max pitch the nose reads inf and the tail reads 2.79 m."""
    nose = gate_range(LIDAR_HEIGHT, -RAMP_PITCH, azimuth=0.0)
    tail = gate_range(LIDAR_HEIGHT, -RAMP_PITCH, azimuth=math.pi)
    assert math.isinf(nose)
    assert tail == pytest.approx(2.263, abs=1e-3)


def test_the_gate_is_the_margin_times_the_ground_intersection():
    nominal = ground_range(LIDAR_HEIGHT, RAMP_PITCH)
    assert gate_range(LIDAR_HEIGHT, RAMP_PITCH) == pytest.approx(
        DEFAULT_MARGIN * nominal
    )


def test_the_gate_tightens_as_pitch_grows():
    radii = [gate_range(LIDAR_HEIGHT, math.radians(d)) for d in (2, 4, 6, 8, 10)]
    assert all(b < a for a, b in zip(radii, radii[1:]))


# -------------------------------------------------------------------- deadband


def test_the_deadband_falls_out_of_the_sensor_range_rather_than_being_chosen():
    deadband = deadband_pitch(LIDAR_HEIGHT, RANGE_MAX)
    assert math.degrees(deadband) == pytest.approx(1.504, abs=1e-3)
    # At exactly the deadband the gate sits on range_max, so nothing can truncate.
    assert gate_range(LIDAR_HEIGHT, deadband) == pytest.approx(RANGE_MAX)


def test_the_gate_is_inactive_below_the_deadband():
    assert not gate_is_active(LIDAR_HEIGHT, math.radians(1.0), RANGE_MAX)
    assert gate_is_active(LIDAR_HEIGHT, math.radians(2.0), RANGE_MAX)


def test_a_level_robot_truncates_nothing():
    ranges = cardinal_scan(forward=2.0, rear=2.0)
    gated, count = truncate_scan(
        ranges, lidar_height=LIDAR_HEIGHT, pitch=0.0, **CARDINAL
    )
    assert count == 0
    assert gated == ranges


# ------------------------------------------------------------------ truncation


def test_the_measured_phantom_return_is_truncated_climbing_the_ramp():
    """2.78 m in the tail sector at 8 deg nose-up - the headline case."""
    ranges = cardinal_scan(rear=2.78)
    gated, count = truncate_scan(
        ranges, lidar_height=LIDAR_HEIGHT, pitch=-RAMP_PITCH, **CARDINAL
    )
    assert count == 1
    assert math.isinf(gated[REAR])
    assert gated[FORWARD] == 10.0


def test_truncation_writes_inf_and_never_range_max():
    """range_max would clear cells out to 8 m on a ray that struck the floor."""
    gated, _ = truncate_scan(
        cardinal_scan(rear=2.78),
        lidar_height=LIDAR_HEIGHT,
        pitch=-RAMP_PITCH,
        **CARDINAL,
    )
    assert gated[REAR] != RANGE_MAX
    assert math.isinf(gated[REAR])


def test_nose_down_truncates_the_nose_not_the_tail():
    gated, count = truncate_scan(
        cardinal_scan(forward=2.78, rear=2.78),
        lidar_height=LIDAR_HEIGHT,
        pitch=RAMP_PITCH,
        **CARDINAL,
    )
    assert count == 1
    assert math.isinf(gated[FORWARD])
    assert gated[REAR] == 2.78


def test_a_return_inside_the_gate_survives():
    """A real obstacle on the ramp is nearer than the ground intersection."""
    gated, count = truncate_scan(
        cardinal_scan(rear=1.50),
        lidar_height=LIDAR_HEIGHT,
        pitch=-RAMP_PITCH,
        **CARDINAL,
    )
    assert count == 0
    assert gated[REAR] == 1.50


def test_the_floor_protects_the_braking_envelope_at_a_steep_pitch():
    """At 30 deg the raw gate would be 0.63 m; the floor holds it at 1.565 m."""
    steep = math.radians(30.0)
    assert gate_range(LIDAR_HEIGHT, steep) == pytest.approx(0.63, abs=1e-3)
    gated, count = truncate_scan(
        cardinal_scan(forward=1.20),
        lidar_height=LIDAR_HEIGHT,
        pitch=steep,
        floor=1.565,
        **CARDINAL,
    )
    assert count == 0, "the floor must keep an obstacle inside the braking envelope"
    assert gated[FORWARD] == 1.20


def test_infinite_and_nan_beams_pass_through_untouched():
    ranges = [float("inf"), float("nan"), 2.78, 5.0]
    gated, count = truncate_scan(
        ranges, lidar_height=LIDAR_HEIGHT, pitch=-RAMP_PITCH, **CARDINAL
    )
    assert count == 1
    assert math.isinf(gated[FORWARD])
    assert math.isnan(gated[LEFT])


def test_the_scan_keeps_its_length_and_ordering():
    """Dropping beams would desynchronise every bearing downstream."""
    ranges = [2.78] * 360
    gated, _ = truncate_scan(
        ranges,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / 359,
        lidar_height=LIDAR_HEIGHT,
        pitch=-RAMP_PITCH,
    )
    assert len(gated) == len(ranges)


def test_the_input_scan_is_not_mutated():
    ranges = cardinal_scan(rear=2.78)
    truncate_scan(ranges, lidar_height=LIDAR_HEIGHT, pitch=-RAMP_PITCH, **CARDINAL)
    assert ranges[REAR] == 2.78, "a pure function must not edit its argument"


def test_only_the_downhill_half_is_ever_touched():
    """Half a scan is the most that can be lost, and never the uphill half."""
    ranges = [2.0] * 360
    gated, count = truncate_scan(
        ranges,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / 359,
        lidar_height=LIDAR_HEIGHT,
        pitch=math.radians(20.0),
    )
    assert count < len(ranges) / 2 + 2
    assert sum(1 for r in gated if not math.isinf(r)) > len(ranges) / 2 - 2


def test_the_gate_profile_matches_beamwise_evaluation():
    profile = gate_profile(0.0, math.pi / 2.0, 4, LIDAR_HEIGHT, RAMP_PITCH)
    assert profile[FORWARD] == pytest.approx(2.263, abs=1e-3)
    assert math.isinf(profile[REAR])


# ----------------------------------------------------------------- sizing table


def test_the_gate_splits_the_measured_returns_the_right_way():
    """Replays smoke test 2's per-bin table. See this module's docstring."""
    truncated, kept = [], []
    for degrees, measured in sorted(MEASURED_CLOSEST_BY_DEGREE.items()):
        radius = gate_range(LIDAR_HEIGHT, math.radians(degrees), azimuth=0.0)
        (truncated if measured > radius else kept).append(degrees)

    assert truncated == [4, 5, 6, 7, 8], (
        "from 4 deg up the closest return in the pitched sector is the ground and "
        f"must be truncated; got {truncated}"
    )
    assert kept == [0, 1, 2, 3], (
        "at 3 deg and below the closest return is nearer than the ground could be, "
        f"so it is real geometry and must survive; got {kept}"
    )


def test_the_worst_case_bin_has_real_margin():
    """8 deg is the tightest bin: 2.77 m measured against a 2.26 m gate."""
    radius = gate_range(LIDAR_HEIGHT, math.radians(8.0), azimuth=0.0)
    assert MEASURED_CLOSEST_BY_DEGREE[8] - radius == pytest.approx(0.51, abs=0.02)
