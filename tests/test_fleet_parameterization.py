"""Verify that one xacro plus one YAML produces two genuinely different robots.

This is the test behind the "configuration-driven scaling" claim. It asserts that the
per-robot differences declared in fleet.yaml actually reach the generated URDF, and that
nothing robot-specific is hidden in the description itself.
"""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

from amr_description.fleet_config import (
    frame_prefix,
    load_fleet,
    spawn_pose,
    xacro_mappings,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO_PATH = os.path.join(REPO_ROOT, "src", "amr_description", "urdf", "amr.urdf.xacro")
FLEET_PATH = os.path.join(REPO_ROOT, "src", "amr_description", "config", "fleet.yaml")


def _render(robot):
    """Run xacro for one robot and return the parsed URDF root element."""
    args = [f"{key}:={value}" for key, value in xacro_mappings(robot).items()]
    result = subprocess.run(
        ["xacro", XACRO_PATH, *args],
        capture_output=True,
        text=True,
        check=True,
    )
    return ET.fromstring(result.stdout)


@pytest.fixture(scope="module")
def fleet():
    return load_fleet(FLEET_PATH)


@pytest.fixture(scope="module")
def rendered(fleet):
    return {robot["name"]: _render(robot) for robot in fleet}


def _total_mass(root):
    return sum(float(mass.get("value")) for mass in root.iter("mass"))


def _link_names(root):
    return {link.get("name") for link in root.iter("link")}


def _plugin_value(root, tag):
    for element in root.iter(tag):
        return element.text.strip()
    raise AssertionError(f"<{tag}> not found in generated URDF")


def test_fleet_declares_at_least_two_robots(fleet):
    assert len(fleet) >= 2


def test_every_robot_renders(rendered, fleet):
    assert set(rendered) == {robot["name"] for robot in fleet}


def test_frames_are_prefixed_per_robot(rendered, fleet):
    """Each robot's links must be namespaced, or TF collides the moment two spawn."""
    for robot in fleet:
        prefix = frame_prefix(robot)
        names = _link_names(rendered[robot["name"]])
        assert names, f"{robot['name']} produced no links"
        for name in names:
            assert name.startswith(prefix), f"link {name} lacks prefix {prefix}"
        assert f"{prefix}base_link" in names
        assert f"{prefix}laser" in names
        assert f"{prefix}imu_link" in names


def test_no_frame_name_collisions_across_fleet(rendered, fleet):
    seen = set()
    for robot in fleet:
        names = _link_names(rendered[robot["name"]])
        assert not (names & seen), "link names collide between robots"
        seen |= names


def test_heavier_robot_is_actually_heavier(rendered, fleet):
    """AMR-1 carries the higher payload, so its simulated mass must exceed AMR-2's."""
    by_name = {robot["name"]: robot for robot in fleet}
    heavy = max(
        by_name, key=lambda n: by_name[n]["base_mass_kg"] + by_name[n]["payload_kg"]
    )
    light = min(
        by_name, key=lambda n: by_name[n]["base_mass_kg"] + by_name[n]["payload_kg"]
    )
    assert _total_mass(rendered[heavy]) > _total_mass(rendered[light])


def test_declared_mass_reaches_the_urdf(rendered, fleet):
    """Total link mass must match base + payload, not a default buried in the xacro."""
    for robot in fleet:
        expected = robot["base_mass_kg"] + robot["payload_kg"]
        actual = _total_mass(rendered[robot["name"]])
        # Sensor links add a small fixed mass; the declared mass must dominate.
        assert actual == pytest.approx(
            expected, rel=0.02
        ), f"{robot['name']}: declared {expected} kg, URDF has {actual} kg"


def test_acceleration_limits_reach_the_simulation(rendered, fleet):
    """The lighter scout must be configured to out-accelerate the heavy lead."""
    for robot in fleet:
        root = rendered[robot["name"]]
        assert float(_plugin_value(root, "max_linear_acceleration")) == pytest.approx(
            robot["max_accel_x"]
        )
        assert float(_plugin_value(root, "max_linear_velocity")) == pytest.approx(
            robot["max_vel_x"]
        )

    by_name = {robot["name"]: robot for robot in fleet}
    fastest = max(by_name, key=lambda n: by_name[n]["max_accel_x"])
    slowest = min(by_name, key=lambda n: by_name[n]["max_accel_x"])
    assert (
        by_name[fastest]["payload_kg"] < by_name[slowest]["payload_kg"]
    ), "the higher-acceleration robot should be the lighter one"


def test_wheel_separation_reaches_the_simulation(rendered, fleet):
    for robot in fleet:
        assert float(
            _plugin_value(rendered[robot["name"]], "wheel_separation")
        ) == pytest.approx(robot["wheel_separation"])


def test_lidar_height_is_measured_from_the_ground(rendered, fleet):
    """lidar_height is a ground-referenced tunable; smoke test 2's maths depends on it.

    base_link sits at wheel_radius above the ground, so the laser joint offset must be
    lidar_height - wheel_radius.
    """
    for robot in fleet:
        root = rendered[robot["name"]]
        prefix = frame_prefix(robot)
        for joint in root.iter("joint"):
            if joint.get("name") == f"{prefix}laser_joint":
                z = float(joint.find("origin").get("xyz").split()[2])
                expected = robot["lidar_height"] - robot["wheel_radius"]
                assert z == pytest.approx(
                    expected
                ), f"{robot['name']}: laser z={z}, expected {expected}"
                break
        else:
            raise AssertionError(f"{prefix}laser_joint not found")


def test_spawn_poses_are_distinct(fleet):
    poses = [spawn_pose(robot)[:2] for robot in fleet]
    assert len(set(poses)) == len(poses), "two robots would spawn on top of each other"
