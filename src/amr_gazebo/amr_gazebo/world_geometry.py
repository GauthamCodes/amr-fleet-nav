"""Read the world's real geometry back out of the SDF Gazebo was handed.

Phase 1's measurements need two facts that only the world file can supply: which
scan returns came from something that is supposed to be there, and where a
pedestrian was at a given instant. Both are read from the world itself rather than
restated in Python, because a second copy of the rack pitch or the actor waypoints
in a measurement script is a silent disagreement waiting for the first edit to
``world.yaml``. Phase 1 already paid for one stale copy of a geometric constant
(the laser offset, commit 79d01a5); this module exists so there is not a second.

ACTORS HAVE NO POSE TOPIC
    Gazebo publishes nothing for ``<actor>`` entities. They are absent from
    ``/world/<w>/pose/info``, from ``dynamic_pose/info`` and from
    ``gz model --list`` - Phase 0, smoke test 1. Their position at time *t* is
    therefore RECONSTRUCTED here from the trajectory the world file scripts, which
    is the same interpolation Gazebo performs internally.

    A reconstruction is not a measurement. Anything that depends on it has to
    state its residual against an independent observation, which is why
    ``nav_goal_run.py`` checks this model against the LiDAR returns it attributes
    to the actor and reports the disagreement rather than assuming it away.
"""

import math
import xml.etree.ElementTree as ET


def world_root(path):
    """Return the ``<world>`` element of a rendered SDF file.

    Args:
        path: Path to a concrete ``.sdf`` - normally the file
            ``amr_gazebo.world_builder.render_world`` produced for the run being
            analysed, so the geometry is exactly what the simulator loaded.

    Returns:
        The ``<world>`` element.

    Raises:
        ValueError: If the document contains no world.
    """
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{path} contains no <world> element")
    return world


def render_world_root(xacro_path, mappings=None):
    """Render a world xacro in memory and return its ``<world>`` element.

    Used by tests, which must not depend on a launch having run first. Prefer
    :func:`world_root` when analysing a real run: the rendered file on disk is the
    one the simulator actually loaded.
    """
    import xacro

    doc = xacro.process_file(
        xacro_path, mappings={k: str(v) for k, v in (mappings or {}).items()}
    )
    return ET.fromstring(doc.toxml()).find("world")


def _pose_of(element):
    """Return ``(x, y, z, roll, pitch, yaw)`` from a child ``<pose>``, or zeros."""
    pose = element.find("pose")
    if pose is None or not (pose.text or "").strip():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    values = [float(v) for v in pose.text.split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])


def static_boxes(world, exclude=("lower_floor",)):
    """Return every static box in the world as a plan-view footprint.

    Each entry is a dict with ``name``, centre ``x``/``y`` and half-extents
    ``hx``/``hy`` measured along the WORLD axes.

    The half-extents are the box's ROTATED extent projected onto the ground plane,
    not its raw size: ``sx*cos(pitch) + sz*|sin(pitch)|`` for a slab pitched about
    Y. For this world's 8 degree ramp the two terms nearly cancel and the
    correction is 3 mm, so the general form is here for the sake of being right at
    steeper angles rather than because it currently changes an answer.

    THE PROJECTION IS CONSERVATIVE, AND ONE CASE IS WORTH KNOWING ABOUT.
    It is the smallest axis-aligned rectangle containing the box, which is the safe
    direction to err for what these footprints are used for - deciding whether a
    scan return is explained by static geometry - because a too-large static region
    can only ever attribute a genuinely dynamic return to the static world. It
    under-reports dynamic obstacles; it never invents one.

    The case: the ramp slab is deliberately extended 1.5 m downhill and buried
    under the floor (see warehouse.sdf.xacro), so its PLAN-VIEW footprint reaches
    back to x = 0.96 even though nothing exists above ground before the toe at
    x = 2.44. A dynamic obstacle in x in [0.96, 2.44], |y| < 1.25 is therefore
    classified static. Phase 1's encounters happen at x < 0 and are unaffected;
    Phase 4 works on the ramp and will need to know. Pinned by
    tests/test_world_geometry.py so it stays a known limitation rather than a
    surprise.

    Args:
        world: A ``<world>`` element.
        exclude: Model names to skip. The floor is a plane rather than a box and
            is not returned in any case.

    Returns:
        A list of footprint dicts in document order.
    """
    boxes = []
    for model in world.findall("model"):
        name = model.get("name")
        if name in exclude:
            continue
        size = model.find(".//collision/geometry/box/size")
        if size is None:
            continue
        sx, sy, sz = (float(v) for v in size.text.split())
        x, y, _, _, pitch, yaw = _pose_of(model)

        # Half-extent of the rotated box along each world axis. Pitch and yaw are
        # the only rotations this world uses; roll would need the full 3x3 form.
        px = 0.5 * (abs(sx * math.cos(pitch)) + abs(sz * math.sin(pitch)))
        py = 0.5 * sy
        hx = abs(px * math.cos(yaw)) + abs(py * math.sin(yaw))
        hy = abs(px * math.sin(yaw)) + abs(py * math.cos(yaw))
        boxes.append({"name": name, "x": x, "y": y, "hx": hx, "hy": hy})
    return boxes


def distance_to_boxes(boxes, x, y):
    """Return the distance from a point to the nearest box footprint.

    Zero when the point lies inside one. Used to decide whether a LiDAR return is
    explained by the static world.
    """
    best = float("inf")
    for box in boxes:
        dx = max(0.0, abs(x - box["x"]) - box["hx"])
        dy = max(0.0, abs(y - box["y"]) - box["hy"])
        best = min(best, math.hypot(dx, dy))
    return best


def actor_trajectories(world):
    """Return ``{name: [(time, x, y, z, yaw), ...]}`` for every scripted actor.

    Waypoints are returned in the order the world declares them, which is the
    order their times increase.
    """
    trajectories = {}
    for actor in world.findall("actor"):
        waypoints = []
        for waypoint in actor.findall(".//trajectory/waypoint"):
            time_element = waypoint.find("time")
            pose_element = waypoint.find("pose")
            if time_element is None or pose_element is None:
                continue
            values = [float(v) for v in pose_element.text.split()]
            values += [0.0] * (6 - len(values))
            waypoints.append(
                (float(time_element.text), values[0], values[1], values[2], values[5])
            )
        if waypoints:
            trajectories[actor.get("name")] = waypoints
    return trajectories


def actor_pose_at(waypoints, t):
    """Interpolate a scripted actor's pose at simulation time ``t``.

    Gazebo drives ``<actor>`` trajectories from simulation time, linearly between
    waypoints, wrapping at the last waypoint when ``<loop>`` is set - which every
    actor in this world sets. The final waypoint repeats the first pose, so the
    wrap is continuous.

    Args:
        waypoints: ``[(time, x, y, z, yaw), ...]`` from :func:`actor_trajectories`.
        t: Simulation time in seconds since the world started.

    Returns:
        ``(x, y, z, yaw)`` at that instant.
    """
    period = waypoints[-1][0]
    if period > 0.0:
        t = math.fmod(t, period)
        if t < 0.0:
            t += period

    previous = waypoints[0]
    for waypoint in waypoints[1:]:
        if t <= waypoint[0]:
            span = waypoint[0] - previous[0]
            fraction = 0.0 if span <= 0.0 else (t - previous[0]) / span
            return tuple(
                previous[i] + fraction * (waypoint[i] - previous[i])
                for i in range(1, 5)
            )
        previous = waypoint
    return tuple(waypoints[-1][1:5])


def actor_velocity_at(waypoints, t, dt=0.05):
    """Return the actor's planar velocity, by finite difference of the pose.

    Used to tell an outbound leg from a return leg when timing an encounter. A
    finite difference rather than a per-segment slope so the caller never has to
    know how the trajectory is discretised.
    """
    ahead = actor_pose_at(waypoints, t + dt)
    behind = actor_pose_at(waypoints, t - dt)
    return ((ahead[0] - behind[0]) / (2 * dt), (ahead[1] - behind[1]) / (2 * dt))
