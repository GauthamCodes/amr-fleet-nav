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


def spawn_pose(robot):
    """Return ``(x, y, z, yaw)`` for a robot's pose, defaulting to the origin."""
    pose = robot.get("spawn", {}) or {}
    return (
        float(pose.get("x", 0.0)),
        float(pose.get("y", 0.0)),
        float(pose.get("z", 0.0)),
        float(pose.get("yaw", 0.0)),
    )
