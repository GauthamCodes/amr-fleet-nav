"""Payload-scaled acceleration and jerk limiting.

The arithmetic that shapes every command the robot executes, tested without a
simulator. What these prove that a trace cannot: the bounds hold for arbitrary
inputs, not just the step the evidence run happened to command.
"""

import math

from amr_description.fleet_config import load_fleet
from amr_motion.jerk_limiter import (
    limit_axis,
    limit_twist,
    limits_from_robot,
    payload_scale,
    scaled_limits,
)

DT = 0.05
LIMITS = {
    "max_accel_x": 1.0,
    "max_decel_x": -1.0,
    "max_accel_theta": 2.0,
    "max_jerk_x": 2.0,
    "max_jerk_theta": 4.0,
}


# ---------------------------------------------------------------------------
# Payload scaling
# ---------------------------------------------------------------------------


def test_an_empty_vehicle_is_unscaled():
    assert payload_scale(30.0, 0.0) == 1.0


def test_payload_scale_falls_as_mass_rises():
    """Fixed traction force means a = F/m, so more mass is less acceleration."""
    light = payload_scale(30.0, 10.0)
    heavy = payload_scale(30.0, 60.0)
    assert 0.0 < heavy < light < 1.0


def test_payload_scale_matches_the_stated_model():
    assert math.isclose(payload_scale(30.0, 60.0), 30.0 / 90.0)


def test_a_negative_payload_is_treated_as_empty():
    """A load cell reading slightly below zero must not RAISE a limit."""
    assert payload_scale(30.0, -5.0) == 1.0


def test_zero_base_mass_is_rejected():
    try:
        payload_scale(0.0, 10.0)
    except ValueError:
        return
    raise AssertionError("a zero base mass should have raised")


def test_scaling_preserves_the_sign_of_the_deceleration_limit():
    """max_decel_x is negative by construction; scaling must not flip it."""
    scaled = scaled_limits(LIMITS, 30.0, 60.0)
    assert scaled["max_decel_x"] < 0.0


def test_a_heavier_payload_never_raises_any_limit():
    light = scaled_limits(LIMITS, 30.0, 0.0)
    heavy = scaled_limits(LIMITS, 30.0, 60.0)
    assert heavy["max_accel_x"] < light["max_accel_x"]
    assert heavy["max_jerk_x"] < light["max_jerk_x"]
    assert heavy["max_accel_theta"] < light["max_accel_theta"]
    assert heavy["max_jerk_theta"] < light["max_jerk_theta"]
    assert abs(heavy["max_decel_x"]) < abs(light["max_decel_x"])


# ---------------------------------------------------------------------------
# The limiter itself
# ---------------------------------------------------------------------------


def test_zero_in_zero_out():
    v, a = limit_axis(0.0, 0.0, 0.0, DT, 1.0, -1.0, 2.0)
    assert v == 0.0
    assert a == 0.0


def test_jerk_is_bounded_from_a_standing_step():
    """The case the stock smoother does not cover: a step into the chain."""
    v, a, previous_a = 0.0, 0.0, 0.0
    for _ in range(40):
        v, a = limit_axis(10.0, v, a, DT, 1.0, -1.0, 2.0)
        assert abs(a - previous_a) <= 2.0 * DT + 1e-9
        previous_a = a


def test_acceleration_is_bounded_too():
    v, a = 0.0, 0.0
    for _ in range(80):
        v, a = limit_axis(10.0, v, a, DT, 1.0, -1.0, 2.0)
        assert a <= 1.0 + 1e-9


def test_the_command_converges_on_the_target():
    v, a = 0.0, 0.0
    for _ in range(400):
        v, a = limit_axis(0.5, v, a, DT, 1.0, -1.0, 2.0)
    assert math.isclose(v, 0.5, abs_tol=1e-3)


def test_deceleration_is_bounded_by_its_own_limit():
    """An asymmetric chassis brakes harder than it accelerates; both are bounds."""
    v, a = 1.0, 0.0
    floor = 0.0
    for _ in range(60):
        v, a = limit_axis(0.0, v, a, DT, 0.4, -0.8, 2.0)
        floor = min(floor, a)
    assert floor >= -0.8 - 1e-9


def test_reversing_the_command_does_not_invert_acceleration_in_one_step():
    """The failure jerk limiting exists to prevent: +a_max to -a_max instantly."""
    v, a = 0.0, 0.0
    peak = 0.0
    for _ in range(40):
        v, a = limit_axis(1.0, v, a, DT, 1.0, -1.0, 2.0)
        peak = max(peak, a)
    previous = a
    for _ in range(40):
        v, a = limit_axis(-1.0, v, a, DT, 1.0, -1.0, 2.0)
        assert abs(a - previous) <= 2.0 * DT + 1e-9
        previous = a
    assert peak > 0.0
    assert previous < 0.0


def test_the_command_never_overshoots_its_target():
    """Bounded jerk means acceleration cannot be removed instantly either.

    A limiter that only clamps - rather than beginning its ramp-down early
    enough - sails past the commanded speed by a^2/(2j) while shedding its
    acceleration. This node is DOWNSTREAM of the stock smoother's velocity
    clamp, so that overshoot would reach the wheels.
    """
    for target in (0.3, 0.5, 1.0):
        v, a = 0.0, 0.0
        for _ in range(600):
            v, a = limit_axis(target, v, a, DT, 1.0, -1.0, 2.0)
            assert v <= target + 1e-9, f"overshot {target}: {v}"


#: How far past jerk_max the emitted command may go, and only on the single
#: step where the velocity arrives at its target. The discrete arrival profile
#: brings this to ~1.12x; the continuous-time formula it replaced reached 3.5x.
#: Asserted rather than described so a future change to limit_axis cannot
#: quietly reintroduce the spike.
ARRIVAL_JERK_TOLERANCE = 1.25


def _peak_jerk(target_sequence, limits, dt=DT):
    """Return the peak |da/dt| the limiter emits over a command sequence."""
    v, a, previous = 0.0, 0.0, 0.0
    peak = 0.0
    for target in target_sequence:
        v, a = limit_axis(
            target,
            v,
            a,
            dt,
            limits["max_accel_x"],
            limits["max_decel_x"],
            limits["max_jerk_x"],
        )
        peak = max(peak, abs(a - previous) / dt)
        previous = a
    return peak


def test_emitted_jerk_stays_within_tolerance_for_every_shipped_robot():
    """The bound that matters, measured the way the trace measures it.

    A step up, held, then a step to zero - the profile the Phase 5 evidence
    run commands. Both payload states, because the loaded limits are the
    tighter ones and therefore the harder ones to hold.
    """
    sequence = [0.5] * 400 + [0.0] * 400
    for robot in load_fleet():
        base = float(robot["base_mass_kg"])
        for payload in (0.0, float(robot["payload_kg"])):
            limits = scaled_limits(limits_from_robot(robot), base, payload)
            peak = _peak_jerk(sequence, limits)
            allowed = limits["max_jerk_x"] * ARRIVAL_JERK_TOLERANCE
            assert peak <= allowed, (
                f"{robot['name']} at {payload} kg: peak jerk {peak:.3f} "
                f"exceeds {allowed:.3f} (limit {limits['max_jerk_x']:.3f})"
            )


def test_it_does_not_undershoot_a_deceleration_either():
    v, a = 1.0, 0.0
    for _ in range(600):
        v, a = limit_axis(0.2, v, a, DT, 1.0, -1.0, 2.0)
        assert v >= 0.2 - 1e-9
    assert math.isclose(v, 0.2, abs_tol=1e-3)


def test_a_heavier_payload_takes_longer_to_reach_the_same_speed():
    """The behavioural claim, stated as a test rather than left to the plot."""

    def steps_to_reach(payload):
        limits = scaled_limits(LIMITS, 30.0, payload)
        v, a = 0.0, 0.0
        for step in range(2000):
            v, a = limit_axis(
                0.5,
                v,
                a,
                DT,
                limits["max_accel_x"],
                limits["max_decel_x"],
                limits["max_jerk_x"],
            )
            if v >= 0.49:
                return step
        return None

    assert steps_to_reach(60.0) > steps_to_reach(0.0)


def test_a_non_positive_timestep_changes_nothing():
    v, a = limit_axis(1.0, 0.3, 0.2, 0.0, 1.0, -1.0, 2.0)
    assert (v, a) == (0.3, 0.2)


def test_limit_twist_shapes_both_axes():
    (v, w), (a, alpha) = limit_twist((1.0, 1.0), ((0.0, 0.0), (0.0, 0.0)), DT, LIMITS)
    assert 0.0 < v
    assert 0.0 < w
    assert abs(a) <= LIMITS["max_jerk_x"] * DT + 1e-9
    assert abs(alpha) <= LIMITS["max_jerk_theta"] * DT + 1e-9


def test_rotation_is_symmetric():
    """There is no reverse about the yaw axis, so both directions share a bound."""
    _, (_, positive) = limit_twist((0.0, 5.0), ((0.0, 0.0), (0.0, 0.0)), DT, LIMITS)
    _, (_, negative) = limit_twist((0.0, -5.0), ((0.0, 0.0), (0.0, 0.0)), DT, LIMITS)
    assert math.isclose(positive, -negative)


# ---------------------------------------------------------------------------
# The shipped fleet
# ---------------------------------------------------------------------------


def test_every_robot_declares_jerk_limits():
    for robot in load_fleet():
        limits = limits_from_robot(robot)
        assert limits["max_jerk_x"] > 0.0
        assert limits["max_jerk_theta"] > 0.0


def test_every_robot_can_reach_its_own_acceleration_limit():
    """A jerk bound must not make the acceleration limit unreachable.

    Tighter than a/1 s and the vehicle could never use its own acceleration
    envelope in any realistic manoeuvre - a configuration error rather than a
    conservative choice.
    """
    for robot in load_fleet():
        limits = limits_from_robot(robot)
        assert limits["max_jerk_x"] >= limits["max_accel_x"] / 1.0


def test_deceleration_limits_are_stored_negative():
    for robot in load_fleet():
        assert limits_from_robot(robot)["max_decel_x"] < 0.0


def test_the_two_robots_have_genuinely_different_envelopes():
    """The fleet YAML must actually express a difference.

    Robot differences come from YAML only (ENGINEERING_NOTES rule 5), which is only
    meaningful if the YAML says something different about each robot.
    """
    fleet = {r["name"]: limits_from_robot(r) for r in load_fleet()}
    assert fleet["amr2"]["max_accel_x"] > fleet["amr1"]["max_accel_x"]
    assert fleet["amr2"]["max_jerk_x"] > fleet["amr1"]["max_jerk_x"]


def test_the_loaded_penalty_is_larger_for_the_higher_payload_robot():
    """The mapper is far more affected by its load, from configuration alone.

    amr1 carries 60 kg on a 30 kg chassis; amr2 carries 5 kg on 18 kg.
    """
    fleet = {r["name"]: r for r in load_fleet()}
    scales = {
        name: payload_scale(float(r["base_mass_kg"]), float(r["payload_kg"]))
        for name, r in fleet.items()
    }
    assert scales["amr1"] < scales["amr2"]
