"""The arithmetic behind ``d_safe = k*v^2 + d_min``, and where ``k`` comes from.

The SafetyGate itself is C++ because it sits in the command path. This module is
the SIZING side of the same contract: it derives the per-robot constants the gate
runs on, the launch file passes them in as parameters, and ``tests/`` exercises them
without a simulator. The gate contains no robot knowledge and no robot names, which
is what keeps ENGINEERING_NOTES rule 5 true of a safety-critical node.

WHY k IS DERIVED RATHER THAN TUNED

    ``d_safe`` is a braking distance plus a standoff, and braking distance is
    ``v^2 / (2a)``. So ``k = 1 / (2 a_eff)`` and ``[k] = s^2/m``, which is the unit
    PLAN.md section 6 asks to be stated explicitly.

    Payload enters through ``a_eff``. Treating the drive's deceleration limit as a
    FORCE limit, a heavier robot decelerates proportionally less:

        a_eff = |max_decel_x| * base_mass / (base_mass + payload)

    Every term already exists in ``fleet.yaml``, so payload scaling costs no new
    per-robot knob and cannot drift out of physical consistency with the mass the
    physics engine is simulating. Adding an ``amr3`` block scales its safety
    distance automatically - that is what "configuration-driven" means here.

    Today that gives amr1 (30 kg + 60 kg payload, 0.80 m/s^2) k = 1.875 s^2/m and
    amr2 (18 + 5, 1.40) k = 0.456 s^2/m. The heavier, more heavily laden robot gets
    the longer stopping distance, which is the whole point.

THE SIMULATOR CAVEAT, STATED RATHER THAN BURIED

    Gazebo's DiffDrive enforces ``min_linear_acceleration`` as a KINEMATIC limit -
    mass-independent. The simulated robot therefore brakes at the full
    ``|max_decel_x|`` regardless of payload, and the measured stopping sweep comes
    in well inside the ``d_safe`` this model asks for. That gap is a property of the
    simulator's drive model, not slack in the safety margin, and the stopping-sweep
    report quotes both this ``k`` and a ``k`` fitted to the measurement so the
    difference is visible rather than averaged away.
"""

import os

from ament_index_python.packages import get_package_share_directory
import yaml

from amr_description.fleet_config import footprint_polygon

#: Standoff held even at a standstill, metres. d_safe(0) == d_min, by construction.
D_MIN = 0.30

#: Extra clearance required to LEAVE the blocked state, metres. At roughly 15x the
#: LiDAR's 0.01 m noise sigma, range noise alone cannot chatter the gate.
RELEASE_MARGIN = 0.15

#: Shortest time the gate stays blocked once entered, seconds. Guards against an
#: obstacle that flickers in and out of the sector faster than the release margin
#: can damp.
MIN_HOLD_S = 0.30

#: Multiplier on the derived stopping gain. 1.0 means "exactly the physical model".
BRAKE_SAFETY_FACTOR = 1.0

#: Slack between the braking envelope and the tightest PitchGate truncation, metres.
GATE_CLEARANCE_MARGIN = 0.10

#: No command in this long and the gate emits zero. Covers the upstream stack dying
#: while the plant is still rolling on a latched command.
CMD_TIMEOUT_S = 0.5

#: No scan in this long and the gate BLOCKS. Fail-closed: an unknown world is
#: treated as an occupied one, not as a clear one.
SCAN_TIMEOUT_S = 0.5

#: Half-angle of the sector searched for obstacles, radians. Pi/2 is the whole
#: half-plane in the commanded direction of travel.
SECTOR_HALF_ANGLE = 1.5707963267948966

#: Returns closer than this to the footprint are discarded as possible self-hits.
#: Phase 1 needed 0.05 m; the 0.35 m mast makes a self-return geometrically
#: impossible, so Phase 2 lowers it and measures the true minimum.
SELF_RETURN_MARGIN = 0.01

#: Value written to the progress checker while the gate is blocked, seconds.
RECOVERY_ALLOWANCE_S = 1.0e6

#: After releasing, wait this long before restoring the progress checker if the
#: robot has not yet moved. Restoring a 10 s allowance onto a robot that is still
#: stationary fires the very recovery the suppression exists to prevent.
RESTORE_GRACE_S = 2.0


def default_safety_path():
    """Return the installed path of the fleet-wide safety constants."""
    return os.path.join(
        get_package_share_directory("amr_safety"), "config", "safety.yaml"
    )


def load_safety_config(path=None):
    """Load the fleet-wide safety constants, falling back to this module's defaults.

    Constants absent from the file keep their module default, so the YAML expresses
    only what has been deliberately chosen rather than restating every value.
    """
    path = default_safety_path() if path is None else path
    with open(path, "r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    config = dict(document.get("safety", {}))
    defaults = {
        "d_min": D_MIN,
        "release_margin": RELEASE_MARGIN,
        "min_hold_s": MIN_HOLD_S,
        "brake_safety_factor": BRAKE_SAFETY_FACTOR,
        "gate_clearance_margin": GATE_CLEARANCE_MARGIN,
        "cmd_timeout_s": CMD_TIMEOUT_S,
        "scan_timeout_s": SCAN_TIMEOUT_S,
        "sector_half_angle": SECTOR_HALF_ANGLE,
        "self_return_margin": SELF_RETURN_MARGIN,
        "recovery_allowance_s": RECOVERY_ALLOWANCE_S,
        "restore_grace_s": RESTORE_GRACE_S,
    }
    for key, value in defaults.items():
        config.setdefault(key, value)
    return config


def effective_decel(robot):
    """Return the payload-scaled deceleration this robot can actually deliver.

    Args:
        robot: One ``fleet.yaml`` entry.

    Returns:
        Deceleration in m/s^2, positive. Derived from ``max_decel_x`` (negative by
        the file's own convention) scaled by the mass the payload adds.
    """
    base = float(robot["base_mass_kg"])
    payload = float(robot["payload_kg"])
    return abs(float(robot["max_decel_x"])) * base / (base + payload)


def stopping_gain(robot, brake_safety_factor=BRAKE_SAFETY_FACTOR):
    """Return this robot's ``k`` in ``d_safe = k*v^2 + d_min``, in s^2/m."""
    return brake_safety_factor / (2.0 * effective_decel(robot))


def d_safe(k, d_min, speed):
    """Return the safe following distance at a measured speed, in metres.

    ``speed`` is the MEASURED speed from odometry, never the commanded one
    (ENGINEERING_NOTES rule 3). Its sign is irrelevant - the square removes it - so a
    reversing robot gets the same envelope as a forward one.
    """
    return k * speed * speed + d_min


def release_distance(stop_distance, release_margin=RELEASE_MARGIN):
    """Return the clearance at which a blocked gate is allowed to release."""
    return stop_distance + release_margin


def footprint_front_extent(robot):
    """Return how far the footprint reaches ahead of the base frame, metres.

    Non-trivial for this chassis: the frame origin is the drive axle and the body
    is offset forward of it, so the footprint reaches 0.49 m ahead and 0.21 m
    behind (commit 79d01a5).
    """
    return max(px for px, _ in footprint_polygon(robot))


def min_gate_range(
    robot,
    k=None,
    d_min=D_MIN,
    brake_safety_factor=BRAKE_SAFETY_FACTOR,
    gate_clearance_margin=GATE_CLEARANCE_MARGIN,
):
    """Return the tightest radius PitchGate may ever truncate to, in metres.

    This is the interlock between the two halves of Phase 2. PitchGate removes
    returns beyond a pitch-dependent radius; clamping that radius below by the
    braking envelope plus the footprint means truncation can never delete an
    obstacle the SafetyGate would have had to stop for, at any pitch. Asserted for
    every robot in the fleet by ``tests/test_safety_distance.py``.
    """
    k = stopping_gain(robot, brake_safety_factor) if k is None else k
    stop = d_safe(k, d_min, float(robot["max_vel_x"]))
    return stop + footprint_front_extent(robot) + gate_clearance_margin


class SafetyLatch:
    """The blocked/clear state machine, with hysteresis on release.

    Mirrors ``amr_safety/safety_model.hpp``, which is what actually runs. This copy
    exists so the state machine is exercised by the repository's pytest suite
    rather than only by an integration run - a silent regression in a hysteresis
    band is a safety bug, and it is invisible in any single-frame test.

    Entry is on ``clearance <= stop``; release needs BOTH a larger clearance
    (``stop + release_margin``) and a minimum dwell. Either alone is insufficient:
    a bare threshold chatters on range noise, and a bare dwell chatters on anything
    slower than the dwell.
    """

    def __init__(self, release_margin=RELEASE_MARGIN, min_hold_s=MIN_HOLD_S):
        """Store the hysteresis parameters and start in the clear state."""
        self.release_margin = release_margin
        self.min_hold_s = min_hold_s
        self.blocked = False
        self.entered_at = None

    def update(self, clearance, stop_distance, now):
        """Advance the latch by one observation and return whether it is blocked.

        Args:
            clearance: Distance from the footprint to the nearest relevant return.
            stop_distance: ``d_safe`` at the currently measured speed.
            now: Monotonic time in seconds.

        Returns:
            True when the gate must zero the command.
        """
        if self.blocked:
            held = now - self.entered_at
            if (
                clearance > release_distance(stop_distance, self.release_margin)
                and held >= self.min_hold_s
            ):
                self.blocked = False
                self.entered_at = None
        elif clearance <= stop_distance:
            self.blocked = True
            self.entered_at = now
        return self.blocked


def gate_parameters(robot, config=None):
    """Return the full parameter set the SafetyGate node is launched with.

    One function so that the numbers the gate runs on, the numbers the tests check
    and the numbers the report quotes are the same numbers.
    """
    config = load_safety_config() if config is None else config
    k = stopping_gain(robot, config["brake_safety_factor"])
    polygon = footprint_polygon(robot)
    return {
        "k": k,
        "d_min": config["d_min"],
        "release_margin": config["release_margin"],
        "min_hold_s": config["min_hold_s"],
        "footprint_x": [float(px) for px, _ in polygon],
        "footprint_y": [float(py) for _, py in polygon],
        "max_vel_x": float(robot["max_vel_x"]),
        "d_safe_at_max_vel": d_safe(k, config["d_min"], float(robot["max_vel_x"])),
        "min_gate_range": min_gate_range(
            robot,
            k=k,
            d_min=config["d_min"],
            gate_clearance_margin=config["gate_clearance_margin"],
        ),
        "cmd_timeout_s": config["cmd_timeout_s"],
        "scan_timeout_s": config["scan_timeout_s"],
        "sector_half_angle": config["sector_half_angle"],
        "self_return_margin": config["self_return_margin"],
        "recovery_allowance_s": config["recovery_allowance_s"],
        "restore_grace_s": config["restore_grace_s"],
    }
