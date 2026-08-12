"""Every robot must spawn in free space.

Phase 0 shipped fleet.yaml with amr1 at (-8.0, -3.0) and amr2 at (-8.0, 3.0), both
of which put the robot's entire footprint inside a storage rack. Nothing caught it:
the two smoke tests each override the spawn pose, and the fleet-launch check confirmed
that topics appeared and TF resolved without ever asking whether the robot was inside
a solid object.

This test closes that gap for every robot the fleet list will ever contain. It renders
the world the robots are actually spawned into, extracts the real obstacle geometry,
and asserts no robot's footprint overlaps any of it.
"""

import math
import os

import pytest

from amr_description.fleet_config import load_fleet, spawn_pose

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FLEET_PATH = os.path.join(REPO_ROOT, "src", "amr_description", "config", "fleet.yaml")
WORLD_PATH = os.path.join(
    REPO_ROOT, "src", "amr_gazebo", "worlds", "warehouse.sdf.xacro"
)

#: Extra margin required around the footprint, in metres. A robot spawned flush against
#: a rack is not a crash but it is not a usable starting pose either.
SPAWN_MARGIN = 0.25


def _rendered_world():
    """Render the warehouse world and return its root <world> element."""
    import xml.etree.ElementTree as ET

    import xacro

    doc = xacro.process_file(WORLD_PATH, mappings={})
    return ET.fromstring(doc.toxml()).find("world")


def _box_obstacles(world):
    """Return (name, cx, cy, sx, sy) for every static box model in the world.

    The floor is a plane rather than a box, and the ramp is a rotated slab a robot is
    meant to drive onto, so neither is an obstacle for spawn purposes. Racks and the
    upper plateau are.
    """
    obstacles = []
    for model in world.findall("model"):
        name = model.get("name")
        if name in ("lower_floor", "ramp"):
            continue
        pose = model.find("pose")
        box = model.find(".//collision/geometry/box/size")
        if pose is None or box is None:
            continue
        px, py = (float(v) for v in pose.text.split()[:2])
        sx, sy = (float(v) for v in box.text.split()[:2])
        obstacles.append((name, px, py, sx, sy))
    return obstacles


@pytest.fixture(scope="module")
def obstacles():
    return _box_obstacles(_rendered_world())


@pytest.fixture(scope="module")
def fleet():
    return load_fleet(FLEET_PATH)


def test_world_actually_contains_obstacles(obstacles):
    """Guard the guard: an empty obstacle list would make every other test vacuous."""
    assert len(obstacles) >= 10, f"expected the rack rows, found {len(obstacles)}"


def test_no_robot_spawns_inside_an_obstacle(fleet, obstacles):
    """The defect this file exists for."""
    failures = []
    for robot in fleet:
        x, y, _, yaw = spawn_pose(robot)
        # Footprint half-extents rotated into world axes. Spawn yaws are axis-aligned
        # in practice; the absolute-cosine form stays correct if that changes.
        half_l, half_w = robot["base_length"] / 2.0, robot["base_width"] / 2.0
        hx = abs(half_l * math.cos(yaw)) + abs(half_w * math.sin(yaw))
        hy = abs(half_l * math.sin(yaw)) + abs(half_w * math.cos(yaw))

        for name, cx, cy, sx, sy in obstacles:
            gap_x = abs(x - cx) - (hx + sx / 2.0)
            gap_y = abs(y - cy) - (hy + sy / 2.0)
            # Separated on either axis means no overlap.
            if gap_x < 0 and gap_y < 0:
                failures.append(
                    f"{robot['name']} at ({x}, {y}) overlaps {name} at ({cx}, {cy}) "
                    f"by {-gap_x:.2f} x {-gap_y:.2f} m"
                )
    assert not failures, "robots spawned inside world geometry:\n  " + "\n  ".join(
        failures
    )


def test_spawn_poses_have_working_clearance(fleet, obstacles):
    """Not merely non-overlapping: far enough out to plan a first move."""
    failures = []
    for robot in fleet:
        x, y, _, _ = spawn_pose(robot)
        half_l, half_w = robot["base_length"] / 2.0, robot["base_width"] / 2.0
        reach = math.hypot(half_l, half_w) + SPAWN_MARGIN
        for name, cx, cy, sx, sy in obstacles:
            dx = max(0.0, abs(x - cx) - sx / 2.0)
            dy = max(0.0, abs(y - cy) - sy / 2.0)
            distance = math.hypot(dx, dy)
            if distance < reach:
                failures.append(
                    f"{robot['name']} is {distance:.2f} m from {name}, "
                    f"needs {reach:.2f} m"
                )
    assert not failures, "spawn clearance too tight:\n  " + "\n  ".join(failures)


def test_spawn_poses_are_on_the_lower_level(fleet):
    """Phase 1 operates on the lower level; the ramp toe is at x = 2.4423."""
    for robot in fleet:
        x, _, _, _ = spawn_pose(robot)
        assert x < 2.4423, f"{robot['name']} spawns on or past the ramp at x={x}"
