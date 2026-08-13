"""The safety distance and its hysteresis, pinned as arithmetic.

Two things here are worth more than the individual assertions.

The first is that ``k`` is DERIVED, not chosen. It falls out of fleet.yaml's mass,
payload and deceleration limit, so these tests check a relationship between robots
(the heavier, more heavily laden one gets the longer stopping distance) rather than
a constant someone typed. Append an amr3 and the property still has to hold.

The second is the interlock at the bottom of this file. PitchGate deletes returns
beyond a pitch-dependent radius and SafetyGate stops for returns inside a
speed-dependent one. Nothing in either module's own tests would notice if those two
radii crossed - PitchGate would be quietly deleting the obstacle SafetyGate was
about to stop for, at exactly the moment the robot is on a ramp. That is the bug
this file exists to make impossible.
"""

import math
import os

import pytest

from amr_bsp.pitch_gate import DEFAULT_MARGIN, gate_range
from amr_description.fleet_config import load_fleet
from amr_safety.safety_model import (
    d_safe,
    effective_decel,
    footprint_front_extent,
    load_safety_config,
    min_gate_range,
    release_distance,
    SafetyLatch,
    stopping_gain,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SAFETY_PATH = os.path.join(REPO_ROOT, "src", "amr_safety", "config", "safety.yaml")

#: The ramp the warehouse actually has, and the mast the LiDAR actually sits on.
RAMP_PITCH_RAD = math.radians(8.002)
LIDAR_HEIGHT = 0.35


@pytest.fixture(scope="module")
def config():
    """Read the checked-in safety constants, not the installed copy."""
    return load_safety_config(SAFETY_PATH)


@pytest.fixture(scope="module")
def fleet():
    """Every robot in the fleet, so these properties are not amr1-specific."""
    return load_fleet()


def test_d_safe_at_rest_is_exactly_d_min(config):
    assert d_safe(1.875, config["d_min"], 0.0) == pytest.approx(config["d_min"])


def test_d_safe_is_strictly_monotonic_in_speed(config):
    distances = [d_safe(1.875, config["d_min"], v) for v in (0.0, 0.2, 0.4, 0.6, 1.0)]
    assert all(
        b > a for a, b in zip(distances, distances[1:])
    ), "a faster robot must never be allowed nearer"


def test_d_safe_ignores_the_sign_of_velocity(config):
    forward = d_safe(1.875, config["d_min"], 0.45)
    reverse = d_safe(1.875, config["d_min"], -0.45)
    assert forward == pytest.approx(reverse)


def test_the_gain_is_one_over_twice_the_effective_deceleration(fleet):
    for robot in fleet:
        expected = 1.0 / (2.0 * effective_decel(robot))
        assert stopping_gain(robot) == pytest.approx(expected)


def test_payload_lengthens_the_stopping_distance(fleet):
    """The payload scaling rule 3 requires, as a property rather than a constant."""
    robot = dict({r["name"]: r for r in fleet}["amr1"])
    unloaded = dict(robot, payload_kg=0.0)
    heavier = dict(robot, payload_kg=2.0 * float(robot["payload_kg"]))
    gains = [stopping_gain(r) for r in (unloaded, robot, heavier)]
    assert all(b > a for a, b in zip(gains, gains[1:]))


def test_an_unloaded_robot_reduces_to_the_bare_deceleration_limit(fleet):
    robot = dict({r["name"]: r for r in fleet}["amr1"], payload_kg=0.0)
    assert effective_decel(robot) == pytest.approx(abs(float(robot["max_decel_x"])))


def test_the_heavier_laden_robot_gets_the_larger_gain(fleet):
    by_name = {r["name"]: r for r in fleet}
    assert stopping_gain(by_name["amr1"]) > stopping_gain(by_name["amr2"])


def test_the_gains_are_the_values_the_report_quotes(fleet):
    """If these move, docs/SESSION_LOG.md and the README move with them."""
    by_name = {r["name"]: r for r in fleet}
    assert stopping_gain(by_name["amr1"]) == pytest.approx(1.875, abs=1e-3)
    assert stopping_gain(by_name["amr2"]) == pytest.approx(0.4564, abs=1e-3)


def test_release_is_always_beyond_stop(config):
    stop = d_safe(1.875, config["d_min"], 0.6)
    assert release_distance(stop, config["release_margin"]) > stop
    assert config["release_margin"] > 0.0


def test_the_release_margin_dominates_lidar_noise(config):
    """0.01 m is the scan's configured Gaussian stddev; the band must swamp it."""
    assert config["release_margin"] >= 10.0 * 0.01


def test_the_latch_enters_on_contact_with_the_stop_distance(config):
    latch = SafetyLatch(config["release_margin"], config["min_hold_s"])
    assert not latch.update(2.0, 1.0, 0.0)
    assert latch.update(1.0, 1.0, 0.1), "clearance == d_stop must block, not tie"


def test_the_latch_holds_inside_the_hysteresis_band(config):
    latch = SafetyLatch(config["release_margin"], config["min_hold_s"])
    latch.update(0.9, 1.0, 0.0)
    inside_band = 1.0 + 0.5 * config["release_margin"]
    assert latch.update(inside_band, 1.0, 5.0), "recovering past d_stop is not enough"


def test_the_latch_will_not_release_before_the_minimum_hold(config):
    latch = SafetyLatch(config["release_margin"], config["min_hold_s"])
    latch.update(0.9, 1.0, 0.0)
    clear = 1.0 + 2.0 * config["release_margin"]
    assert latch.update(clear, 1.0, 0.5 * config["min_hold_s"])
    assert not latch.update(clear, 1.0, config["min_hold_s"])


def test_noise_around_the_threshold_cannot_chatter_the_latch(config):
    """The failure this hysteresis exists to prevent, driven at 10 Hz for 2 s."""
    latch = SafetyLatch(config["release_margin"], config["min_hold_s"])
    transitions = 0
    previous = False
    for step in range(20):
        # A stationary obstacle exactly at d_stop, seen through +/-1 sigma of noise.
        clearance = 1.0 + (0.01 if step % 2 else -0.01)
        blocked = latch.update(clearance, 1.0, 0.1 * step)
        transitions += blocked != previous
        previous = blocked
    assert transitions == 1, "the gate should latch once and stay latched"


def test_min_gate_range_covers_the_braking_envelope_and_the_footprint(fleet, config):
    failures = []
    for robot in fleet:
        gain = stopping_gain(robot, config["brake_safety_factor"])
        envelope = d_safe(gain, config["d_min"], float(robot["max_vel_x"]))
        needed = envelope + footprint_front_extent(robot)
        floor = min_gate_range(
            robot,
            k=gain,
            d_min=config["d_min"],
            gate_clearance_margin=config["gate_clearance_margin"],
        )
        if floor < needed:
            failures.append(
                f"{robot['name']}: PitchGate floor {floor:.3f} m is inside the "
                f"braking envelope + footprint {needed:.3f} m"
            )
    assert not failures, "PitchGate could delete an obstacle SafetyGate must stop " + (
        "for:\n  " + "\n  ".join(failures)
    )


def test_truncation_never_reaches_the_braking_envelope_at_any_pitch(fleet, config):
    """The interlock itself: sweep pitch far past anything the warehouse contains."""
    failures = []
    for robot in fleet:
        gain = stopping_gain(robot, config["brake_safety_factor"])
        envelope = d_safe(gain, config["d_min"], float(robot["max_vel_x"]))
        needed = envelope + footprint_front_extent(robot)
        floor = min_gate_range(
            robot,
            k=gain,
            d_min=config["d_min"],
            gate_clearance_margin=config["gate_clearance_margin"],
        )
        for degrees in range(0, 46):
            radius = gate_range(
                float(robot["lidar_height"]),
                math.radians(degrees),
                azimuth=0.0,
                margin=DEFAULT_MARGIN,
                floor=floor,
            )
            if radius < needed:
                failures.append(
                    f"{robot['name']} at {degrees} deg: gate {radius:.3f} m "
                    f"< envelope {needed:.3f} m"
                )
    assert not failures, "\n  ".join([""] + failures)


def test_at_the_real_ramp_angle_the_floor_is_not_even_doing_the_work(fleet, config):
    """The honest version of the claim for the README.

    On this warehouse's 8 deg ramp the unclamped gate already sits well outside the
    braking envelope, so the floor is a guarantee held in reserve rather than a
    clamp the system leans on. Worth pinning, because "the floor saves us" and "we
    never get near the floor" are very different engineering statements.
    """
    robot = {r["name"]: r for r in fleet}["amr1"]
    gain = stopping_gain(robot, config["brake_safety_factor"])
    envelope = d_safe(gain, config["d_min"], float(robot["max_vel_x"]))
    unclamped = gate_range(LIDAR_HEIGHT, RAMP_PITCH_RAD, azimuth=0.0, floor=0.0)
    assert unclamped == pytest.approx(2.263, abs=1e-3)
    assert unclamped > envelope + footprint_front_extent(robot)
