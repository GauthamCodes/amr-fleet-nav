"""Turn a global plan and a measured speed into a timed prediction.

Pure functions, no ROS imports - the same shape as :mod:`fleet_grid` and
:mod:`selective_policy`, and for the same reason: the arithmetic that decides
where a robot will be, and when, is worth testing without a simulator.

A "trajectory" here is a list of ``(x, y, dt)`` samples: a position in the fleet
frame and the number of seconds from now at which the robot is predicted to
reach it. That triple is exactly what ``FleetTrajectoryLayer`` consumes on the
other side of the wire, where ``dt`` drives the cost decay.

WHY SPEED IS MEASURED RATHER THAN COMMANDED

    The same reason SafetyGate uses odometry (ENGINEERING_NOTES rule 3). A prediction
    built from the command is a prediction of what the controller intends, and a
    robot that is stalled, gated, or yielding intends plenty while going nowhere.
    A peer that believes a halted robot is about to sweep through the
    intersection will defer to a vehicle that is not moving.

WHY THERE IS A SPEED FLOOR

    A robot that is momentarily at rest but holds an active plan will move again.
    Predicting from a measured 0.0 m/s would either divide by zero or claim the
    robot occupies its current cell forever. The floor makes a stopped-but-tasked
    robot predict conservatively fast, which over-reserves space ahead of it -
    the safe direction to be wrong in.
"""

import math

#: Metres between successive samples along a predicted trajectory. Half the
#: 0.05 m costmap resolution times ten: fine enough that the discs
#: FleetTrajectoryLayer stamps overlap into a continuous corridor at the default
#: 0.45 m radius, coarse enough that a 6 s horizon is tens of samples, not
#: hundreds.
DEFAULT_SPACING_M = 0.25

#: Seconds ahead a prediction extends. Beyond this the cost has decayed to
#: nothing anyway (see the layer's decay_tau_s), so the samples would be pure
#: message size.
DEFAULT_HORIZON_S = 6.0

#: Speed floor, m/s. See the module docstring.
MIN_PREDICT_SPEED = 0.15


def polyline_length(points):
    """Return the arc length of a polyline given as ``[(x, y), ...]``."""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def nearest_index(points, x, y):
    """Return the index of the polyline vertex closest to ``(x, y)``.

    Used to drop the part of a global plan the robot has already driven. A plan
    is published whole, from the robot's position at plan time; by the time the
    prediction is built the robot is some way along it, and predicting that the
    robot will re-drive the part behind it would reserve space it has vacated.
    """
    if not points:
        raise ValueError("nearest_index() needs at least one point")
    best = 0
    best_d2 = float("inf")
    for index, (px, py) in enumerate(points):
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = index
    return best


def resample(points, spacing):
    """Return points spaced ``spacing`` metres apart along the polyline.

    The first and last points are always kept; the last is dropped only when it
    is a duplicate of the sample before it. A plan's final pose is its GOAL, and
    a rule that discarded it whenever it fell less than half a spacing past the
    last regular sample would silently shorten every prediction by up to half a
    sample - hardest to notice exactly where it matters, at the end of a plan
    where a robot is about to stop and occupy a cell.
    """
    if spacing <= 0.0:
        raise ValueError(f"spacing must be > 0, got {spacing}")
    if len(points) < 2:
        return list(points)

    out = [points[0]]
    carried = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        segment = math.hypot(x1 - x0, y1 - y0)
        if segment <= 0.0:
            continue
        travelled = spacing - carried
        while travelled <= segment:
            ratio = travelled / segment
            out.append((x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0)))
            travelled += spacing
        carried = segment - (travelled - spacing)

    tail = points[-1]
    if math.hypot(tail[0] - out[-1][0], tail[1] - out[-1][1]) > 0.1 * spacing:
        out.append(tail)
    return out


def timed(points, speed, horizon_s=DEFAULT_HORIZON_S, min_speed=MIN_PREDICT_SPEED):
    """Attach arrival times to a polyline travelled at ``speed``.

    Returns ``[(x, y, dt), ...]`` truncated at ``horizon_s``. ``dt`` is seconds
    from now, so the first sample is always 0.0.
    """
    effective = max(abs(float(speed)), float(min_speed))
    out = []
    elapsed = 0.0
    previous = None
    for point in points:
        if previous is not None:
            elapsed += (
                math.hypot(point[0] - previous[0], point[1] - previous[1]) / effective
            )
            if elapsed > horizon_s:
                break
        out.append((point[0], point[1], elapsed))
        previous = point
    return out


def predict_along_plan(
    plan_points,
    x,
    y,
    speed,
    horizon_s=DEFAULT_HORIZON_S,
    spacing_m=DEFAULT_SPACING_M,
    min_speed=MIN_PREDICT_SPEED,
):
    """Predict where a robot on ``plan_points`` will be over the next horizon.

    ``plan_points`` and ``(x, y)`` must be in the same frame; the result is in
    that frame too.
    """
    if not plan_points:
        return []
    ahead = plan_points[nearest_index(plan_points, x, y) :]
    if len(ahead) < 2:
        return [(x, y, 0.0)]
    return timed(resample(ahead, spacing_m), speed, horizon_s, min_speed)


def predict_constant_velocity(
    x,
    y,
    yaw,
    speed,
    horizon_s=DEFAULT_HORIZON_S,
    spacing_m=DEFAULT_SPACING_M,
    idle_speed=0.02,
):
    """Predict a robot with no active plan.

    Below ``idle_speed`` the honest prediction is that the robot stays where it
    is, so a single sample at ``dt = 0`` is returned. That is not a degenerate
    case to be skipped: a robot parked in an aisle is exactly the thing its peers
    need to route around, and returning an empty trajectory there would make a
    stopped robot invisible.
    """
    if abs(speed) < idle_speed:
        return [(x, y, 0.0)]
    reach = abs(speed) * horizon_s
    steps = max(1, int(reach / spacing_m))
    direction = 1.0 if speed >= 0.0 else -1.0
    return [
        (
            x + direction * (i * spacing_m) * math.cos(yaw),
            y + direction * (i * spacing_m) * math.sin(yaw),
            (i * spacing_m) / abs(speed),
        )
        for i in range(steps + 1)
    ]
