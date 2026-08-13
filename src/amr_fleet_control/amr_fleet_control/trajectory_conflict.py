"""Space-time conflict detection between two predicted trajectories.

Pure functions, no ROS imports. One implementation, used twice:

    * ``TrafficControlNode`` calls it to decide whether a conflict needs
      escalating to a yield.
    * The evidence runs call it offline to say what the local layer was asked to
      resolve, so "resolved locally" and "escalated" are counted against the same
      definition the arbiter used rather than against a second one written for
      the report.

WHAT A CONFLICT IS HERE

    Two robots are in conflict when they are predicted to be within ``radius_m``
    of each other at times within ``time_window_s``. Both halves matter. Paths
    that cross are not a conflict if one robot is through the intersection before
    the other arrives - that is the ordinary case in a warehouse and treating it
    as a conflict would have every robot yielding to every other one. Conversely
    two robots creeping down the same aisle never "cross" and are in conflict the
    whole way.

    ``radius_m`` is a separation between robot CENTRES, so it must cover both
    footprints plus the margin you want. It is passed in rather than derived
    here because the footprints live in fleet.yaml and this module does not read
    configuration.
"""

from collections import namedtuple
import math

#: One predicted meeting. ``t_a`` and ``t_b`` are seconds from now on each
#: robot's own prediction; ``separation`` is metres between the two predicted
#: positions.
Conflict = namedtuple("Conflict", "x y t_a t_b separation")


def conflicts(traj_a, traj_b, radius_m, time_window_s):
    """Return every predicted meeting between two trajectories.

    Each trajectory is ``[(x, y, dt), ...]`` in a COMMON frame, with ``dt`` in
    seconds from a common origin. Results are ordered by the earlier of the two
    times, so ``[0]`` is the conflict that happens first.
    """
    found = []
    for xa, ya, ta in traj_a:
        for xb, yb, tb in traj_b:
            if abs(ta - tb) > time_window_s:
                continue
            separation = math.hypot(xa - xb, ya - yb)
            if separation <= radius_m:
                found.append(
                    Conflict(
                        x=0.5 * (xa + xb),
                        y=0.5 * (ya + yb),
                        t_a=ta,
                        t_b=tb,
                        separation=separation,
                    )
                )
    found.sort(key=lambda c: min(c.t_a, c.t_b))
    return found


def first_conflict(traj_a, traj_b, radius_m, time_window_s):
    """Return the earliest conflict, or ``None``.

    Kept separate from :func:`conflicts` because the arbiter only ever wants the
    first one and building the whole list to take its head is wasted work at
    10 Hz across a growing fleet.
    """
    best = None
    for xa, ya, ta in traj_a:
        for xb, yb, tb in traj_b:
            if abs(ta - tb) > time_window_s:
                continue
            separation = math.hypot(xa - xb, ya - yb)
            if separation > radius_m:
                continue
            if best is None or min(ta, tb) < min(best.t_a, best.t_b):
                best = Conflict(
                    x=0.5 * (xa + xb),
                    y=0.5 * (ya + yb),
                    t_a=ta,
                    t_b=tb,
                    separation=separation,
                )
    return best


def closest_approach(traj_a, traj_b, time_window_s):
    """Return ``(separation, t)`` at the tightest time-aligned approach.

    Separate from the conflict test on purpose. The conflict test answers "is
    this close enough to act on"; this answers "how close did the prediction say
    they would get", which is what tells an escalation policy whether a conflict
    is being resolved (separation growing) or is not (separation flat or
    shrinking). Returns ``(inf, None)`` if the two predictions never overlap in
    time.
    """
    best = float("inf")
    when = None
    for xa, ya, ta in traj_a:
        for xb, yb, tb in traj_b:
            if abs(ta - tb) > time_window_s:
                continue
            separation = math.hypot(xa - xb, ya - yb)
            if separation < best:
                best = separation
                when = 0.5 * (ta + tb)
    return best, when
