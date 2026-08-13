"""The fleet frame contract, across the three systems that have to agree on it.

FleetMapNode allocates the fleet grid, both Nav2 global costmaps declare its extent,
and the ramp filter mask is rendered onto it. Those are three separate configuration
mechanisms - Python constants, a rendered YAML template, and a generated PGM - so the
agreement between them is asserted here rather than assumed. A world change that
outgrows the grid, or a costmap extent edited on its own, fails a test instead of
silently truncating the map or putting the ramp cost in the wrong place.
"""

import os

import pytest
import yaml

from amr_description.fleet_config import load_fleet, spawn_pose
from amr_fleet_control.fleet_grid import (
    default_fleet_grid,
    FLEET_FRAME,
    fleet_grid_spec,
    FLEET_MAP_TOPIC,
    FLEET_RESOLUTION,
)
from amr_gazebo.world_geometry import render_world_root, static_boxes
from amr_navigation.params import render_nav2_params, render_slam_params
from amr_navigation.ramp_mask import mask_pixel, MAX_MASK_VALUE, render_mask_array

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_PATH = os.path.join(
    REPO_ROOT, "src", "amr_gazebo", "worlds", "warehouse.sdf.xacro"
)
FLEET_MAP_CONFIG = os.path.join(
    REPO_ROOT, "src", "amr_fleet_control", "config", "fleet_map.yaml"
)


@pytest.fixture(scope="module")
def fleet():
    return load_fleet()


@pytest.fixture(scope="module")
def grid():
    return default_fleet_grid()


@pytest.fixture(scope="module")
def nav2_params(fleet):
    """Render every robot's Nav2 parameters, as the launch would."""
    rendered = {}
    for robot in fleet:
        path = render_nav2_params(robot, namespace=robot["name"])
        with open(path, "r", encoding="utf-8") as handle:
            rendered[robot["name"]] = yaml.safe_load(handle)[robot["name"]]
    return rendered


def global_costmap(params):
    """Return one robot's global costmap parameter block."""
    return params["global_costmap"]["global_costmap"]["ros__parameters"]


# --------------------------------------------------------------- the frames --


def test_global_costmap_plans_in_the_fleet_frame(nav2_params):
    """The line that makes cooperative mapping serve navigation."""
    for name, params in nav2_params.items():
        assert global_costmap(params)["global_frame"] == FLEET_FRAME, name


def test_global_costmap_reads_the_fleet_map(nav2_params):
    for name, params in nav2_params.items():
        static = global_costmap(params)["static_layer"]
        assert static["map_topic"] == FLEET_MAP_TOPIC, name
        # FleetMapNode publishes TRANSIENT_LOCAL; a VOLATILE subscriber here would
        # never match it, with no warning on either end.
        assert static["map_subscribe_transient_local"] is True, name
        # Nothing publishes /fleet_map_updates.
        assert static["subscribe_to_updates"] is False, name


def test_local_costmap_stays_in_odom(nav2_params):
    """Keep three distinct frames alive at once.

    A key-name rewrite could not express this, which is why params.py renders
    templates instead of using RewrittenYaml.
    """
    for name, params in nav2_params.items():
        local = params["local_costmap"]["local_costmap"]["ros__parameters"]
        assert local["global_frame"] == f"{name}/odom"


def test_slam_still_owns_its_own_map_frame(fleet):
    """Leave slam_toolbox owning its own per-robot map frame.

    Repointing __MAP_FRAME__ instead of adding __GLOBAL_FRAME__ would have moved
    slam_toolbox's map frame to fleet_map and collapsed the whole distinction.
    """
    for robot in fleet:
        path = render_slam_params(robot, namespace=robot["name"])
        with open(path, "r", encoding="utf-8") as handle:
            params = yaml.safe_load(handle)[robot["name"]]
        assert params["slam_toolbox"]["ros__parameters"]["map_frame"] == (
            f"{robot['name']}/map"
        )


def test_bt_navigator_and_behaviors_move_with_the_global_costmap(nav2_params):
    for name, params in nav2_params.items():
        assert params["bt_navigator"]["ros__parameters"]["global_frame"] == FLEET_FRAME
        behavior = params["behavior_server"]["ros__parameters"]
        assert behavior["global_frame"] == FLEET_FRAME
        # Spin/BackUp/DriveOnHeading actually run here, and must not move.
        assert behavior["local_frame"] == f"{name}/odom"


# ---------------------------------------------------------------- the extent --


def test_costmap_extent_equals_the_fleet_grid(nav2_params, grid):
    """Pin the duplication this test exists for.

    fleet_map.yaml and nav2_params.yaml describe the same grid through two
    different configuration systems, so the agreement is asserted, not assumed.
    """
    for name, params in nav2_params.items():
        costmap = global_costmap(params)
        assert costmap["origin_x"] == pytest.approx(grid.origin_x), name
        assert costmap["origin_y"] == pytest.approx(grid.origin_y), name
        assert costmap["resolution"] == pytest.approx(grid.resolution), name
        # Nav2 states costmap width/height in METRES, the GridSpec in cells.
        assert costmap["width"] == pytest.approx(grid.width * grid.resolution), name
        assert costmap["height"] == pytest.approx(grid.height * grid.resolution), name


def test_global_costmap_is_not_rolling(nav2_params):
    """A rolling global costmap would discard the fleet map outside a window."""
    for name, params in nav2_params.items():
        assert global_costmap(params)["rolling_window"] is False, name


def test_fleet_map_config_matches_the_canonical_grid(grid):
    """fleet_map.yaml restates the extent as ROS parameters; it must not drift."""
    with open(FLEET_MAP_CONFIG, "r", encoding="utf-8") as handle:
        params = yaml.safe_load(handle)["fleet_map_node"]["ros__parameters"]

    configured = fleet_grid_spec(
        params["x_min"],
        params["x_max"],
        params["y_min"],
        params["y_max"],
        params["resolution"],
        margin=params["margin"],
    )
    assert configured.origin_x == pytest.approx(grid.origin_x)
    assert configured.origin_y == pytest.approx(grid.origin_y)
    assert (configured.width, configured.height) == (grid.width, grid.height)


def test_fleet_grid_contains_every_static_box_in_the_world(grid):
    """Pins the extent against the world it is supposed to cover.

    A rack row moved east, or a second ramp added for the Phase 4 A/B, must either
    fit inside this grid or force a deliberate edit to WAREHOUSE_EXTENT. Silently
    mapping only part of the warehouse is the failure this prevents.
    """
    world = render_world_root(WORLD_PATH, {"with_actors": "false"})
    x_hi = grid.origin_x + grid.width * grid.resolution
    y_hi = grid.origin_y + grid.height * grid.resolution

    outside = []
    for box in static_boxes(world):
        if (
            box["x"] - box["hx"] < grid.origin_x
            or box["x"] + box["hx"] > x_hi
            or box["y"] - box["hy"] < grid.origin_y
            or box["y"] + box["hy"] > y_hi
        ):
            outside.append(box["name"])
    assert not outside, f"world geometry outside the fleet grid: {sorted(outside)}"


def test_every_spawn_pose_is_inside_the_fleet_grid(fleet, grid):
    x_hi = grid.origin_x + grid.width * grid.resolution
    y_hi = grid.origin_y + grid.height * grid.resolution
    for robot in fleet:
        x, y, _, _ = spawn_pose(robot)
        assert grid.origin_x < x < x_hi, robot["name"]
        assert grid.origin_y < y < y_hi, robot["name"]


def test_every_spawn_is_axis_aligned(fleet):
    """Keep every spawn axis-aligned.

    Compositing is a translation on a shared lattice. A rotated spawn would need
    resampling, and FleetMapNode raises rather than skewing the map silently.
    """
    for robot in fleet:
        _, _, _, yaw = spawn_pose(robot)
        assert yaw == pytest.approx(0.0)


# ------------------------------------------------------------------ the mask --


def test_phase3_mask_is_uniformly_free(grid):
    """Phase 3's deliverable is verified plumbing, so the mask must add no cost."""
    image = render_mask_array(grid, regions=())
    assert image.shape == grid.shape
    assert int(image.min()) == 255 and int(image.max()) == 255


def test_mask_value_zero_is_white_and_higher_values_are_darker():
    assert mask_pixel(0) == 255
    assert mask_pixel(50) < mask_pixel(10)
    assert mask_pixel(MAX_MASK_VALUE) < mask_pixel(50)


def test_mask_refuses_a_value_that_would_read_as_in_collision():
    """Refuse a mask value that would read as in-collision.

    100 is LETHAL_OBSTACLE and 253 is already INSCRIBED_INFLATED_OBSTACLE, which
    the footprint collision checkers treat as a collision. An expensive ramp is
    not an impassable one.
    """
    with pytest.raises(ValueError, match="outside"):
        mask_pixel(100)


def test_mask_regions_land_where_the_world_says_the_ramp_is(grid):
    """Put the mask cost where the world says the ramp is.

    Row 0 of a PGM is the TOP - the highest y - which is the opposite of an
    OccupancyGrid. Getting it backwards mirrors the mask about the aisle.
    """
    ramp = (2.4423, 6.0, -1.25, 1.25, 60)
    image = render_mask_array(grid, regions=[ramp])

    def pixel_at(x, y):
        col = int((x - grid.origin_x) / grid.resolution)
        row = int((y - grid.origin_y) / grid.resolution)
        return int(image[grid.height - 1 - row, col])

    assert pixel_at(4.2, 0.0) == mask_pixel(60)
    # Just off the ramp in y, and back down the aisle in x: both untouched.
    assert pixel_at(4.2, 2.0) == 255
    assert pixel_at(-5.0, 0.0) == 255


def test_mask_resolution_matches_the_fleet_grid(grid):
    """A mask on a different lattice puts the cost in the wrong cells."""
    assert grid.resolution == pytest.approx(FLEET_RESOLUTION)
