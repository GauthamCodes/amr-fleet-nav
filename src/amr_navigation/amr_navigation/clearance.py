"""Geometry for turning a LaserScan into a clearance measurement.

Pure functions, no ROS types, so the arithmetic that produces the headline safety
number is unit-testable without a simulator. Phase 2's SafetyGate reports the same
quantity from the same footprint, and it will use this module rather than
recomputing it.

WHY CLEARANCE IS MEASURED FROM THE FOOTPRINT, NOT FROM THE BASE FRAME
    This robot's origin is its drive axle and its body is offset forward of it
    (commit 79d01a5), so the footprint reaches 0.49 m ahead of the origin and only
    0.21 m behind. A clearance quoted from the origin would overstate the gap in
    front by nearly half a metre - in exactly the direction the robot is moving,
    and exactly where an obstacle matters. Every distance here is measured from
    the footprint polygon, so "0.62 m of clearance" means 0.62 m of free space
    between the vehicle's skin and the obstacle.
"""

import math


def transform_polygon(polygon, x, y, yaw):
    """Return a body-frame polygon placed at a world pose."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [
        (x + px * cos_yaw - py * sin_yaw, y + px * sin_yaw + py * cos_yaw)
        for px, py in polygon
    ]


def _segment_distance(px, py, ax, ay, bx, by):
    """Return the distance from a point to a line segment."""
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / length_squared
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_in_polygon(polygon, px, py):
    """Return whether a point lies inside a polygon (ray casting, any winding)."""
    inside = False
    count = len(polygon)
    for index in range(count):
        ax, ay = polygon[index]
        bx, by = polygon[(index + 1) % count]
        if (ay > py) != (by > py):
            crossing = ax + (py - ay) * (bx - ax) / (by - ay)
            if px < crossing:
                inside = not inside
    return inside


def distance_to_polygon(polygon, px, py):
    """Return the distance from a point to a polygon's boundary.

    Negative inside, so a caller can distinguish "0.0 m clearance, just touching"
    from "0.3 m inside the footprint, which means this return is the robot's own
    body and must be discarded".
    """
    count = len(polygon)
    best = min(
        _segment_distance(px, py, *polygon[i], *polygon[(i + 1) % count])
        for i in range(count)
    )
    return -best if point_in_polygon(polygon, px, py) else best


def scan_to_world(scan, laser_x, laser_y, laser_yaw):
    """Project a LaserScan's valid returns into world coordinates.

    Args:
        scan: Any object exposing the LaserScan fields ``ranges``, ``angle_min``,
            ``angle_increment``, ``range_min`` and ``range_max``.
        laser_x: World x of the laser, NOT of the robot's base frame.
        laser_y: World y of the laser.
        laser_yaw: World heading of the laser.

    Returns:
        A list of ``(x, y, range)`` triples for returns inside the sensor's valid
        range band. Infinite and NaN returns - the sensor's way of saying "nothing
        out there" - are dropped rather than clamped to range_max, which would
        manufacture a ring of phantom obstacles at 12 m.
    """
    points = []
    for index, distance in enumerate(scan.ranges):
        if math.isinf(distance) or math.isnan(distance):
            continue
        if distance < scan.range_min or distance > scan.range_max:
            continue
        angle = laser_yaw + scan.angle_min + index * scan.angle_increment
        points.append(
            (
                laser_x + distance * math.cos(angle),
                laser_y + distance * math.sin(angle),
                distance,
            )
        )
    return points


def path_length(points):
    """Return the length of a polyline given as ``[(x, y), ...]``."""
    return sum(
        math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
        for i in range(len(points) - 1)
    )


def max_deviation(path_a, path_b):
    """Return the largest distance from any point of ``path_a`` to the CURVE ``path_b``.

    A one-sided Hausdorff distance, used to decide whether a freshly published
    global plan is a genuinely different route or the same route recomputed. The
    default Nav2 behaviour tree replans at 1 Hz whether or not anything changed,
    so counting raw plan messages measures the tree's tick rate, not the stack's
    response to the world.

    The distance is to ``path_b``'s SEGMENTS, not to its vertices. Measuring
    vertex-to-vertex would report the same route resampled at a finer spacing as a
    deviation of half the spacing - turning a change in how densely the planner
    sampled its output into a "replan" that never happened.
    """
    if not path_a or not path_b:
        return float("inf")
    if len(path_b) == 1:
        return max(
            math.hypot(px - path_b[0][0], py - path_b[0][1]) for px, py in path_a
        )

    worst = 0.0
    for px, py in path_a:
        best = min(
            _segment_distance(px, py, *path_b[i], *path_b[i + 1])
            for i in range(len(path_b) - 1)
        )
        worst = max(worst, best)
    return worst
