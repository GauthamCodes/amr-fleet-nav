"""Load the typed robot list and turn each entry into xacro arguments.

This module is the single interpreter of ``config/fleet.yaml``. Launch files loop over
:func:`load_fleet` and feed :func:`xacro_mappings` into one shared ``amr.urdf.xacro``.
Keeping the interpretation here is what makes "no per-robot branching" enforceable: a
launch file never learns a robot's name, only that a list has another entry in it.
"""

import os

from ament_index_python.packages import get_package_share_directory
import yaml

#: Fields every fleet entry must define. A typo in fleet.yaml should fail loudly at
#: launch rather than silently spawning a robot with default mass and limits.
REQUIRED_FIELDS = (
    "name",
    "type",
    "base_mass_kg",
    "payload_kg",
    "wheel_separation",
    "wheel_radius",
    "lidar_height",
    "max_vel_x",
    "max_accel_x",
    "max_decel_x",
    "max_vel_theta",
    "max_accel_theta",
)

#: Entry fields passed straight through to xacro as arguments.
_XACRO_PASSTHROUGH = (
    "base_mass_kg",
    "payload_kg",
    "wheel_separation",
    "wheel_radius",
    "base_length",
    "base_width",
    "base_height",
    "lidar_height",
    "max_vel_x",
    "max_accel_x",
    "max_decel_x",
    "max_vel_theta",
    "max_accel_theta",
    # Visual only, and optional: an entry without it renders in the xacro's default
    # body colour. It is here because RViz's RobotModel display has no colour
    # property, so "tell the two robots apart on screen" has to be answered in the
    # description rather than in the RViz config.
    "body_color",
    # Deliberately absent from fleet.yaml: the camera is a per-RUN choice, not a
    # per-robot property. Passing it through here lets the one launch that needs a
    # camera set it on the fleet entry it spawns, without every other run paying
    # for a second rendering sensor. See amr.urdf.xacro for the measurement.
    "camera_enabled",
)


def default_fleet_path():
    """Return the installed path of the fleet configuration."""
    return os.path.join(
        get_package_share_directory("amr_description"), "config", "fleet.yaml"
    )


def load_fleet(path=None):
    """Read the typed robot list.

    Args:
        path: Optional override for the fleet YAML. Defaults to the installed
            ``amr_description/config/fleet.yaml``.

    Returns:
        A list of robot dictionaries in declaration order.

    Raises:
        KeyError: If an entry is missing a required field.
        ValueError: If the file declares no robots or duplicates a robot name.
    """
    path = path or default_fleet_path()
    with open(path, "r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle)

    fleet = (document or {}).get("fleet") or []
    if not fleet:
        raise ValueError(f"{path} declares no robots under the 'fleet' key")

    seen = set()
    for robot in fleet:
        missing = [field for field in REQUIRED_FIELDS if field not in robot]
        if missing:
            raise KeyError(
                f"robot {robot.get('name', '<unnamed>')} in {path} "
                f"is missing required fields: {', '.join(missing)}"
            )
        if robot["name"] in seen:
            raise ValueError(f"duplicate robot name '{robot['name']}' in {path}")
        seen.add(robot["name"])

    return fleet


def frame_prefix(robot):
    """Return the TF/link prefix for a robot, e.g. ``amr1/``.

    The trailing slash is part of the prefix: link names are formed by
    direct concatenation, producing ``amr1/base_link``.
    """
    return f"{robot['name']}/"


def xacro_mappings(robot):
    """Build the xacro argument mapping for one robot.

    Values are stringified because xacro takes all arguments as strings.
    """
    mappings = {
        "name": str(robot["name"]),
        "prefix": frame_prefix(robot),
    }
    for field in _XACRO_PASSTHROUGH:
        if field in robot:
            mappings[field] = str(robot[field])
    return mappings


def body_x_offset(robot):
    """Return how far forward of the drive axle the chassis box sits, in metres.

    Mirrors the ``body_x`` property in ``amr.urdf.xacro``. The frame origin is the drive
    axle because that is what a differential drive rotates about and what wheel odometry
    integrates; the body is offset forward of it.
    """
    return 0.20 * float(robot["base_length"])


def laser_x_offset(robot):
    """Return the laser's x position in the base_footprint frame, in metres.

    Every measurement that converts a scan range into a world position needs this, and
    Phase 1 found the cost of letting each caller recompute it: when the frame origin
    moved onto the drive axle, a stale copy of this expression would have silently
    biased every range comparison by 0.14 m. Derived once, here.
    """
    return body_x_offset(robot) + float(robot["base_length"]) / 2.0 - 0.06


def footprint_polygon(robot):
    """Return the robot's footprint in the base_footprint frame as [[x, y], ...].

    ASYMMETRIC BY CONSTRUCTION. The origin is the drive axle and the body sits forward
    of it, so the polygon extends further ahead than behind. That asymmetry is the whole
    point: a symmetric disc or box would understate how far the nose sweeps when the
    robot turns in place, and Nav2 would drive it into a rack while believing it had
    clearance.
    """
    front = body_x_offset(robot) + float(robot["base_length"]) / 2.0
    back = body_x_offset(robot) - float(robot["base_length"]) / 2.0
    half_w = float(robot["base_width"]) / 2.0
    return [
        [round(front, 4), round(half_w, 4)],
        [round(front, 4), round(-half_w, 4)],
        [round(back, 4), round(-half_w, 4)],
        [round(back, 4), round(half_w, 4)],
    ]


def spawn_pose(robot):
    """Return ``(x, y, z, yaw)`` for a robot's pose, defaulting to the origin."""
    pose = robot.get("spawn", {}) or {}
    return (
        float(pose.get("x", 0.0)),
        float(pose.get("y", 0.0)),
        float(pose.get("z", 0.0)),
        float(pose.get("yaw", 0.0)),
    )
