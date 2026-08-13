"""The yield protocol's arbitration rules, as pure functions.

Named in PLAN.md section 6 alongside the conflict predicate. What is worth
pinning here is not that a yield happens - an integration run shows that - but
the three properties that make it SAFE to happen:

    * the priority order is a strict total order, so two robots can never each
      decide the other should yield (deadlock) or each decide to yield (also
      deadlock, politely);
    * escalation is reluctant, so a conflict the local FleetTrajectoryLayer is
      already opening up is left to it (docs/ENGINEERING_NOTES.md rule 7);
    * a hold always ends, and the report can tell the two ways it ends apart.
"""

import math

from amr_description.fleet_config import load_fleet
from amr_fleet_control.traffic_policy import (
    conflict_radius,
    DEFAULT_ESCALATE_AFTER_S,
    DEFAULT_IMPROVEMENT_M,
    DEFAULT_MAX_HOLD_S,
    DEFAULT_MIN_HOLD_S,
    escalate,
    footprint_radius,
    gross_mass_kg,
    priority_order,
    release,
    separation_gain,
    yielder,
)
from amr_safety.safety_model import D_MIN, d_safe, stopping_gain


def fleet():
    return load_fleet()


# --------------------------------------------------------------------- priority


def test_priority_is_by_gross_mass_not_by_name():
    order = priority_order(fleet())
    masses = [
        gross_mass_kg(next(r for r in fleet() if r["name"] == name)) for name in order
    ]
    assert masses == sorted(masses, reverse=True)


def test_the_shipped_fleet_puts_the_laden_lead_first():
    # The assignment's AMR-1 is the heavier, mission-critical lead and AMR-2 the
    # scout that gives way. That ordering has to fall out of fleet.yaml rather
    # than out of a name in the code (rule 5), and this is the assertion that it
    # does: change the masses and the yield direction changes with them.
    assert priority_order(fleet()) == ["amr1", "amr2"]
    assert yielder(fleet(), "amr1", "amr2") == "amr2"
    assert yielder(fleet(), "amr2", "amr1") == "amr2"


def test_equal_mass_robots_still_get_a_strict_order():
    twins = [
        {"name": "a", "base_mass_kg": 10.0, "payload_kg": 5.0},
        {"name": "b", "base_mass_kg": 10.0, "payload_kg": 5.0},
        {"name": "c", "base_mass_kg": 10.0, "payload_kg": 5.0},
    ]
    assert priority_order(twins) == ["a", "b", "c"]
    # Asked both ways round: an arbiter that answered differently depending on
    # argument order would have both robots yielding to each other.
    assert yielder(twins, "b", "c") == yielder(twins, "c", "b") == "c"


def test_a_third_robot_slots_in_by_its_own_mass():
    scaled = fleet() + [
        {"name": "amr3", "base_mass_kg": 50.0, "payload_kg": 0.0},
    ]
    assert priority_order(scaled) == ["amr1", "amr3", "amr2"]


# ------------------------------------------------------------- conflict radius


def test_conflict_radius_is_derived_from_footprints_and_braking():
    robots = fleet()
    radii = sorted((footprint_radius(r) for r in robots), reverse=True)
    envelope = max(
        d_safe(stopping_gain(r), D_MIN, float(r["max_vel_x"])) for r in robots
    )
    assert conflict_radius(robots) == radii[0] + radii[1] + envelope


def test_conflict_radius_covers_both_robots_touching():
    # The floor the number must clear: two robots whose footprints are in contact
    # must always be inside it, whatever the braking envelope works out to.
    robots = fleet()
    touching = sum(sorted((footprint_radius(r) for r in robots), reverse=True)[:2])
    assert conflict_radius(robots) > touching


def test_footprint_radius_uses_the_asymmetric_polygon():
    # The origin is the drive axle and the body sits forward of it, so the
    # enclosing radius must exceed half the base length - the number a symmetric
    # box would have given.
    for robot in fleet():
        assert footprint_radius(robot) > float(robot["base_length"]) / 2.0


# ------------------------------------------------------------------ escalation


def test_a_young_conflict_is_never_escalated():
    assert not escalate(DEFAULT_ESCALATE_AFTER_S - 0.1, [2.0, 1.5, 1.0])


def test_an_old_conflict_that_is_not_improving_escalates():
    assert escalate(DEFAULT_ESCALATE_AFTER_S + 0.1, [1.4, 1.35, 1.3])


def test_an_old_conflict_the_local_layer_is_opening_is_left_alone():
    # This is rule 7 as a test. The separation is still inside the conflict
    # radius, so the pair is still "in conflict" - but it has grown by more than
    # the improvement threshold, which means something downstream of the
    # prediction is already resolving it. The arbiter must not take it over.
    opening = [1.0, 1.0 + DEFAULT_IMPROVEMENT_M + 0.05]
    assert not escalate(DEFAULT_ESCALATE_AFTER_S + 5.0, opening)


def test_noise_sized_improvement_does_not_count_as_resolution():
    barely = [1.0, 1.0 + DEFAULT_IMPROVEMENT_M - 0.01]
    assert escalate(DEFAULT_ESCALATE_AFTER_S + 0.1, barely)


def test_separation_gain_is_measured_end_to_end_not_step_to_step():
    # A prediction that dips and recovers has NOT been resolved, and a rule that
    # looked at the last step alone would say it had.
    assert separation_gain([2.0, 0.5, 2.0]) == 0.0
    assert separation_gain([]) == 0.0
    assert separation_gain([1.0]) == 0.0


# --------------------------------------------------------------------- release


def test_a_hold_is_never_released_before_its_floor():
    released, reason = release(False, DEFAULT_MIN_HOLD_S - 0.1)
    assert not released
    assert reason == ""


def test_a_cleared_conflict_releases_and_says_so():
    released, reason = release(False, DEFAULT_MIN_HOLD_S + 0.1)
    assert released
    assert reason == "conflict cleared"


def test_a_live_conflict_holds():
    assert release(True, DEFAULT_MIN_HOLD_S + 5.0)[0] is False


def test_the_ceiling_releases_a_live_conflict_and_is_named_differently():
    released, reason = release(True, DEFAULT_MAX_HOLD_S + 0.1)
    assert released
    # The distinction the report depends on: a deadlock broken by the fail-safe
    # must never be readable as a conflict that cleared on its own.
    assert "fail-safe" in reason
    assert reason != "conflict cleared"


def test_the_ceiling_outranks_the_floor():
    # Degenerate configuration, but the ordering must still terminate: a max
    # below the min has to release, not hold forever.
    released, _ = release(True, 5.0, min_hold_s=10.0, max_hold_s=1.0)
    assert released


def test_every_hold_terminates_within_the_ceiling():
    held = 0.0
    while held < DEFAULT_MAX_HOLD_S * 2.0:
        released, _ = release(True, held)
        if released:
            break
        held += 0.1
    assert released
    assert math.isclose(held, DEFAULT_MAX_HOLD_S, abs_tol=0.11)
