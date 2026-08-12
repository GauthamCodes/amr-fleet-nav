"""Clearance is the headline safety number, so its arithmetic is pinned here.

The asymmetric footprint is the point of these tests. This robot's origin is its
drive axle with the body offset forward (commit 79d01a5), so a clearance quoted
from the origin overstates the gap ahead by nearly half a metre - in the direction
of travel. Phase 2's SafetyGate reports the same quantity from the same functions.
"""

import math

import pytest

from amr_description.fleet_config import footprint_polygon, load_fleet
from amr_navigation.clearance import (
    distance_to_polygon,
    max_deviation,
    path_length,
    point_in_polygon,
    scan_to_world,
    transform_polygon,
)

SQUARE = [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]


class FakeScan:
    """Minimal stand-in for sensor_msgs/LaserScan."""

    def __init__(self, ranges, angle_min=0.0, angle_increment=math.pi / 2):
        """Build a scan with the fields ``scan_to_world`` actually reads."""
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.range_min = 0.12
        self.range_max = 12.0


def test_point_inside_and_outside_a_square():
    assert point_in_polygon(SQUARE, 0.0, 0.0)
    assert not point_in_polygon(SQUARE, 2.0, 0.0)


def test_distance_is_negative_inside():
    assert distance_to_polygon(SQUARE, 3.0, 0.0) == pytest.approx(2.0)
    assert distance_to_polygon(SQUARE, 0.0, 0.0) == pytest.approx(-1.0)


def test_distance_to_a_corner_is_diagonal():
    assert distance_to_polygon(SQUARE, 2.0, 2.0) == pytest.approx(math.sqrt(2.0))


def test_translation_and_rotation_of_a_polygon():
    moved = transform_polygon(SQUARE, 10.0, 0.0, math.pi / 2)
    assert moved[0][0] == pytest.approx(9.0)
    assert moved[0][1] == pytest.approx(1.0)


def test_the_real_footprint_is_asymmetric_about_the_origin():
    """The defect commit 79d01a5 fixed, asserted as a property of the geometry."""
    robot = {r["name"]: r for r in load_fleet()}["amr1"]
    polygon = footprint_polygon(robot)
    ahead = distance_to_polygon(polygon, 1.0, 0.0)
    behind = distance_to_polygon(polygon, -1.0, 0.0)
    assert ahead == pytest.approx(0.51, abs=1e-3)
    assert behind == pytest.approx(0.79, abs=1e-3)
    assert behind - ahead == pytest.approx(
        0.28, abs=1e-3
    ), "an obstacle the same distance from the origin is nearer the skin in front"


def test_clearance_from_the_origin_would_overstate_the_gap_ahead():
    robot = {r["name"]: r for r in load_fleet()}["amr1"]
    polygon = footprint_polygon(robot)
    obstacle = (1.0, 0.0)
    from_origin = math.hypot(*obstacle)
    from_skin = distance_to_polygon(polygon, *obstacle)
    assert from_origin - from_skin == pytest.approx(0.49, abs=1e-3)


def test_scan_projects_into_world_coordinates():
    scan = FakeScan([2.0, float("inf"), float("nan"), 3.0])
    points = scan_to_world(scan, 1.0, 0.0, 0.0)
    assert len(points) == 2, "inf and nan mean 'nothing there' and are dropped"
    assert points[0][:2] == pytest.approx((3.0, 0.0))
    # Fourth beam is at 3*pi/2 from the laser's heading, i.e. -y.
    assert points[1][:2] == pytest.approx((1.0, -3.0), abs=1e-9)


def test_scan_respects_the_sensors_range_band():
    scan = FakeScan([0.05, 20.0, 1.0, 1.0])
    assert len(scan_to_world(scan, 0.0, 0.0, 0.0)) == 2


def test_scan_rotates_with_the_laser():
    scan = FakeScan([2.0], angle_min=0.0)
    points = scan_to_world(scan, 0.0, 0.0, math.pi / 2)
    assert points[0][:2] == pytest.approx((0.0, 2.0), abs=1e-9)


def test_path_length_of_a_right_angle():
    assert path_length([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]) == pytest.approx(7.0)


def test_identical_plans_have_no_deviation():
    plan = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert max_deviation(plan, plan) == pytest.approx(0.0)


def test_a_detour_registers_as_a_different_route():
    """What separates a genuine replan from Nav2's 1 Hz recompute of the same path."""
    straight = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    detour = [(0.0, 0.0), (1.0, 1.5), (2.0, 0.0)]
    assert max_deviation(detour, straight) == pytest.approx(1.5)
    # Sampling the same route more finely is not a replan.
    finer = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
    assert max_deviation(finer, straight) == pytest.approx(0.0)
