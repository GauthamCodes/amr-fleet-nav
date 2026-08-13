"""Who yields, when a conflict escalates, and what ends a hold.

Pure functions, no ROS imports - the same shape as :mod:`selective_policy` and
:mod:`trajectory_predict`, and for the same reason: an arbitration rule that can
only be exercised by launching two robots is a rule nobody can check.

WHERE THIS SITS IN THE CONFLICT PIPELINE (ENGINEERING_NOTES rule 7)

    Every robot's local planner already consumes its peers' predicted
    trajectories through ``FleetTrajectoryLayer``. That is the MAPF requirement
    and it resolves what it can on its own - it pushes cost into the local
    costmap and the controller paces and steers around it, with no central node
    in the loop.

    ``TrafficControlNode`` exists for what the local layer CANNOT resolve: a
    constriction where there is no lateral room to deviate into, so both robots
    keep their routes, keep closing, and each ends up inside the other's braking
    envelope - where both safety gates halt and neither can proceed. The rules
    below are what decide that a conflict has stopped being local business.

    :func:`escalate` is therefore deliberately reluctant. It requires the
    conflict to have persisted for a while AND the predicted closest approach to
    have stopped improving. A conflict the local layer is opening up is one this
    module must keep its hands off.

WHY PRIORITY IS DERIVED FROM MASS

    The assignment names AMR-1 the heavier, mission-critical lead and AMR-2 the
    scout that gives way. Hard-coding that ordering would be a robot name in the
    code (rule 5) and would not survive an ``amr3``. Gross mass gives the same
    answer from ``fleet.yaml`` alone, and it is the physically right quantity:
    yielding costs the vehicle its momentum, and the heavier vehicle both spends
    more to stop and needs more distance to do it. Ties break on declaration
    order, so the ordering is always a strict total order - two robots that each
    believed the other should yield would deadlock, and two that each yielded
    would deadlock politely.
"""

from amr_description.fleet_config import footprint_polygon
from amr_safety.safety_model import D_MIN, d_safe, stopping_gain

#: Metres the predicted closest approach must grow by before the conflict counts
#: as "the local layer is handling it". Below this it is prediction noise: the
#: peers' trajectories are rebuilt from scratch at 5 Hz off a replanned global
#: path, and successive predictions differ by centimetres for no reason at all.
DEFAULT_IMPROVEMENT_M = 0.15

#: Seconds a conflict must persist before it may be escalated. One prediction
#: cycle of overlap is not a conflict, it is two robots whose plans happen to
#: cross; the local layer gets this long to open the gap on its own.
DEFAULT_ESCALATE_AFTER_S = 2.0

#: Shortest and longest a yield may be held, seconds. The floor stops a yield
#: chattering on and off across the mux's own timeout. The ceiling is a
#: fail-safe, not a policy: if a hold ever outlasts it, something the arbiter
#: cannot see is wrong, and releasing a stopped robot is the recoverable
#: direction to be wrong in. Both ends are reported, so a run released by the
#: ceiling can never be read as a run released by the conflict clearing.
DEFAULT_MIN_HOLD_S = 1.0
DEFAULT_MAX_HOLD_S = 45.0

#: Multiplier applied to the conflict radius when deciding to RELEASE. Releasing
#: at the same radius that triggered the hold puts the decision on a knife edge
#: and the yield flickers; the peer has to be properly clear, not marginally.
DEFAULT_RELEASE_FACTOR = 1.25


def gross_mass_kg(robot):
    """Return a robot's laden mass: chassis plus rated payload, in kg."""
    return float(robot["base_mass_kg"]) + float(robot["payload_kg"])


def footprint_radius(robot):
    """Return the radius of the disc that encloses this robot's footprint.

    Measured from ``base_footprint``, which is NOT the centre of the body: the
    origin is the drive axle and the chassis sits forward of it, so the enclosing
    radius is set by the front corners. Using half the base length here would
    understate the vehicle by about 0.2 m, in the direction it is driving.
    """
    return max((px * px + py * py) ** 0.5 for px, py in footprint_polygon(robot))


def priority_order(fleet):
    """Return robot names ordered highest priority first.

    Sorted on ``(gross mass, earlier declaration)``, so the result is a strict
    total order for any fleet - including one whose robots are identical, where
    ``fleet.yaml`` order alone decides.
    """
    ranked = sorted(
        enumerate(fleet),
        key=lambda pair: (-gross_mass_kg(pair[1]), pair[0]),
    )
    return [robot["name"] for _, robot in ranked]


def yielder(fleet, name_a, name_b):
    """Return which of two robots must give way to the other."""
    order = priority_order(fleet)
    if name_a not in order or name_b not in order:
        raise ValueError(f"unknown robot in ({name_a}, {name_b}): fleet is {order}")
    return name_b if order.index(name_a) < order.index(name_b) else name_a


def conflict_radius(fleet, d_min=D_MIN):
    """Return the centre-to-centre separation below which two robots conflict.

    DERIVED, NOT TUNED, and this is the argument for the number:

        radius = r_a + r_b + max(d_safe(v_max))

    Two robots closer than the sum of their footprint radii are touching. Add
    the larger of the two braking envelopes at cruise and you have the distance
    at which one robot's SafetyGate would already be holding the other - which
    is the operational definition of "too close" for this fleet, because past it
    the vehicles stop being able to resolve anything by driving.

    It follows the fleet: a heavier ``amr3`` with a longer stopping distance
    widens the radius by existing, with no constant to remember to retune.
    """
    if len(fleet) < 2:
        raise ValueError("a conflict radius needs at least two robots")
    radii = sorted((footprint_radius(robot) for robot in fleet), reverse=True)
    envelope = max(
        d_safe(stopping_gain(robot), d_min, float(robot["max_vel_x"]))
        for robot in fleet
    )
    return radii[0] + radii[1] + envelope


def separation_gain(history):
    """Return how much the predicted closest approach has opened up, in metres.

    Oldest sample against newest, over the life of one continuously predicted
    conflict. Positive means the two robots are being routed apart; negative or
    flat means nothing downstream of the prediction is fixing this.
    """
    if len(history) < 2:
        return 0.0
    return history[-1] - history[0]


def escalate(
    age_s,
    history,
    escalate_after_s=DEFAULT_ESCALATE_AFTER_S,
    improvement_m=DEFAULT_IMPROVEMENT_M,
):
    """Return whether a live conflict should be escalated to a yield.

    Args:
        age_s: Seconds this conflict has been continuously predicted.
        history: Predicted closest approach, oldest first, over that time.
        escalate_after_s: Grace given to the local layer.
        improvement_m: Gain that counts as the local layer resolving it.

    Returns:
        True when the central arbiter should take the conflict over.
    """
    if age_s < escalate_after_s:
        return False
    return separation_gain(history) < improvement_m


def release(
    conflict_present,
    held_s,
    min_hold_s=DEFAULT_MIN_HOLD_S,
    max_hold_s=DEFAULT_MAX_HOLD_S,
):
    """Return ``(release, reason)`` for a yield that has been held ``held_s``.

    The reason is part of the answer, not decoration. "The conflict cleared" and
    "the fail-safe ceiling expired" are the same observable event - a robot
    starts moving again - and a report that could not tell them apart would let
    a deadlock read as a successful yield.
    """
    if held_s >= max_hold_s:
        return True, "max hold elapsed (fail-safe)"
    if held_s < min_hold_s:
        return False, ""
    if not conflict_present:
        return True, "conflict cleared"
    return False, ""
