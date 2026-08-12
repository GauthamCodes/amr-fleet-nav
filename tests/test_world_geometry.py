"""The world file is the single source of truth for the world's geometry.

Phase 1 measures clearance to a pedestrian that Gazebo publishes no pose for, by
reconstructing the trajectory the world file scripts. These tests pin the
reconstruction against the world as rendered, so a change to world.yaml that moves
a rack or retimes an actor cannot silently invalidate a recorded measurement.
"""

import math
import os

import pytest

from amr_gazebo.world_geometry import (
    actor_pose_at,
    actor_trajectories,
    actor_velocity_at,
    distance_to_boxes,
    render_world_root,
    static_boxes,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_PATH = os.path.join(
    REPO_ROOT, "src", "amr_gazebo", "worlds", "warehouse.sdf.xacro"
)


@pytest.fixture(scope="module")
def world():
    return render_world_root(WORLD_PATH, {"with_actors": "true"})


@pytest.fixture(scope="module")
def boxes(world):
    return static_boxes(world)


@pytest.fixture(scope="module")
def actors(world):
    return actor_trajectories(world)


def test_every_rack_is_found(boxes):
    racks = [b for b in boxes if b["name"].startswith("rack_")]
    assert len(racks) == 10, "two rows of five racks"


def test_rack_footprints_match_world_yaml(boxes):
    rack = next(b for b in boxes if b["name"] == "rack_north_0")
    assert rack["x"] == pytest.approx(-12.0)
    assert rack["y"] == pytest.approx(3.2)
    assert rack["hx"] == pytest.approx(1.2)
    assert rack["hy"] == pytest.approx(0.45)


def test_ramp_footprint_accounts_for_its_pitch(boxes):
    """The pitched slab's plan-view extent, and where its uphill edge lands."""
    ramp = next(b for b in boxes if b["name"] == "ramp")
    angle = math.radians(8.0)
    slab_length = math.hypot(0.5 / math.tan(angle), 0.5) + 1.5
    expected = 0.5 * (slab_length * math.cos(angle) + 0.4 * math.sin(angle))
    assert ramp["hx"] == pytest.approx(expected, abs=1e-3)
    # The uphill end of the slab is the crest, which is the plateau's edge.
    assert ramp["x"] + ramp["hx"] == pytest.approx(6.0, abs=0.06)


def test_the_ramp_footprint_reaches_under_the_floor(boxes):
    """A known limitation, pinned so Phase 4 inherits it as a fact, not a surprise.

    The slab is extended 1.5 m downhill of the toe and buried, so its plan-view
    footprint starts well before the ramp exists above ground. Anything using these
    footprints to classify a scan return will call a dynamic obstacle in that strip
    static.
    """
    ramp = next(b for b in boxes if b["name"] == "ramp")
    toe_x = 6.0 - 0.5 / math.tan(math.radians(8.0))
    assert toe_x == pytest.approx(2.44, abs=0.01)
    assert ramp["x"] - ramp["hx"] == pytest.approx(0.96, abs=0.02)
    assert distance_to_boxes(boxes, 1.5, 0.0) == pytest.approx(
        0.0
    ), "a point 0.9 m short of the ramp toe sits inside the buried slab's footprint"


def test_aisle_centre_is_clear_of_every_box(boxes):
    """The aisle the survey drives and the actors walk must contain no geometry.

    Bounded at x = -1 by the buried ramp extension above - which is also the east
    end of pedestrian_1's walk, the encounter this phase measures.
    """
    for x in range(-13, 0):
        assert distance_to_boxes(boxes, float(x), 0.0) > 1.9


def test_a_point_on_a_rack_face_has_zero_distance(boxes):
    assert distance_to_boxes(boxes, -12.0, 2.75) == pytest.approx(0.0, abs=1e-9)


def test_three_actors_with_looping_trajectories(actors):
    assert set(actors) == {"pedestrian_1", "pedestrian_2", "pedestrian_3"}
    for waypoints in actors.values():
        assert len(waypoints) == 5
        assert waypoints[0][0] == 0.0


def test_actor_returns_to_its_start_at_the_end_of_a_period(actors):
    waypoints = actors["pedestrian_1"]
    period = waypoints[-1][0]
    start = actor_pose_at(waypoints, 0.0)
    assert actor_pose_at(waypoints, period)[:2] == pytest.approx(start[:2])
    # And the loop wraps rather than sticking at the last waypoint.
    assert actor_pose_at(waypoints, period + 4.0)[:2] == pytest.approx(
        actor_pose_at(waypoints, 4.0)[:2]
    )


def test_actor_interpolates_linearly_along_its_leg(actors):
    """pedestrian_1 walks -11 -> -1 in the first 8 s, so it is halfway at t = 4."""
    waypoints = actors["pedestrian_1"]
    x, y = actor_pose_at(waypoints, 4.0)[:2]
    assert x == pytest.approx(-6.0)
    assert y == pytest.approx(0.0)


def test_actor_velocity_distinguishes_the_return_leg(actors):
    """The encounter gate depends on knowing which way the pedestrian is walking."""
    waypoints = actors["pedestrian_1"]
    assert actor_velocity_at(waypoints, 4.0)[0] > 1.0
    assert actor_velocity_at(waypoints, 13.0)[0] < -1.0


def test_actors_absent_when_the_world_switches_them_off():
    world = render_world_root(WORLD_PATH, {"with_actors": "false"})
    assert actor_trajectories(world) == {}
    assert len(static_boxes(world)) >= 10, "racks stay regardless"
