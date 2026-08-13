"""Trajectory prediction and space-time conflict detection.

Named in PLAN.md section 6. These are the algorithms behind the MAPF
requirement, and testing them as pure functions is what makes them general
rather than tuned to one demo run: a conflict test that only works for two
robots crossing at the one intersection we filmed is not a conflict test.

The shipped costmap-layer configuration is asserted here too. The cost ceiling
is a safety property of the layer (a peer's intention must never read as an
obstacle to a footprint collision checker) and it is enforced in C++, so a YAML
edit that violated it would fail at runtime inside a costmap - which is the
worst place to discover it.
"""

import math
import os

import yaml

from amr_description.fleet_config import load_fleet
from amr_fleet_control.trajectory_conflict import (
    closest_approach,
    conflicts,
    first_conflict,
)
from amr_fleet_control.trajectory_predict import (
    nearest_index,
    polyline_length,
    predict_along_plan,
    predict_constant_velocity,
    resample,
    timed,
)
from amr_navigation.params import config_dir, peer_trajectory_topics

# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def test_polyline_length_sums_segments():
    assert polyline_length([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]) == 7.0


def test_polyline_length_of_a_single_point_is_zero():
    assert polyline_length([(1.0, 2.0)]) == 0.0


def test_resample_spaces_points_evenly():
    points = resample([(0.0, 0.0), (10.0, 0.0)], 2.0)
    assert points[0] == (0.0, 0.0)
    for previous, current in zip(points, points[1:]):
        assert math.isclose(
            math.hypot(*(c - p for c, p in zip(current, previous))), 2.0
        )


def test_resample_keeps_the_goal():
    """A plan's last pose is the goal; dropping it truncates the prediction."""
    points = resample([(0.0, 0.0), (5.0, 0.0)], 2.0)
    assert math.isclose(points[-1][0], 5.0)


def test_resample_rejects_a_non_positive_spacing():
    for bad in (0.0, -1.0):
        try:
            resample([(0.0, 0.0), (1.0, 0.0)], bad)
        except ValueError:
            continue
        raise AssertionError(f"spacing {bad} should have raised")


def test_nearest_index_finds_the_robot_on_its_plan():
    plan = [(float(i), 0.0) for i in range(10)]
    assert nearest_index(plan, 4.2, 0.1) == 4


# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------


def test_timed_starts_at_zero_and_increases():
    samples = timed([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], speed=1.0, horizon_s=10.0)
    times = [t for _, _, t in samples]
    assert times[0] == 0.0
    assert times == sorted(times)
    assert math.isclose(times[-1], 2.0)


def test_timed_truncates_at_the_horizon():
    samples = timed([(float(i), 0.0) for i in range(20)], speed=1.0, horizon_s=5.0)
    assert all(t <= 5.0 for _, _, t in samples)
    assert len(samples) < 20


def test_timed_applies_the_speed_floor():
    """A stopped-but-tasked robot predicts conservatively fast, not infinitely slow."""
    stopped = timed([(0.0, 0.0), (1.0, 0.0)], speed=0.0, horizon_s=100.0, min_speed=0.5)
    assert math.isclose(stopped[-1][2], 2.0)


def test_faster_robots_reach_further_within_the_horizon():
    plan = [(float(i) * 0.5, 0.0) for i in range(40)]
    slow = predict_along_plan(plan, 0.0, 0.0, speed=0.3, horizon_s=5.0)
    fast = predict_along_plan(plan, 0.0, 0.0, speed=1.0, horizon_s=5.0)
    assert fast[-1][0] > slow[-1][0]


def test_prediction_drops_the_part_of_the_plan_already_driven():
    plan = [(float(i), 0.0) for i in range(10)]
    samples = predict_along_plan(plan, 6.0, 0.0, speed=1.0, horizon_s=5.0)
    assert samples[0][0] >= 6.0


def test_a_stationary_robot_with_no_plan_still_occupies_its_cell():
    """An empty prediction would make a parked robot invisible to the fleet."""
    samples = predict_constant_velocity(1.0, 2.0, 0.0, speed=0.0)
    assert samples == [(1.0, 2.0, 0.0)]


def test_a_moving_robot_with_no_plan_projects_along_its_heading():
    samples = predict_constant_velocity(0.0, 0.0, math.pi / 2, speed=1.0, horizon_s=2.0)
    assert samples[-1][1] > 1.0
    assert math.isclose(samples[-1][0], 0.0, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# Conflict detection - the three cases PLAN.md section 6 names
# ---------------------------------------------------------------------------

#: Two robots crossing the same cell at the same time.
CROSSING_A = [(float(i) * 0.5, 0.0, float(i) * 0.5) for i in range(11)]
CROSSING_B = [(2.5, float(i) * 0.5 - 2.5, float(i) * 0.5) for i in range(11)]


def test_conflicting_paths_are_detected():
    found = first_conflict(CROSSING_A, CROSSING_B, radius_m=0.9, time_window_s=1.0)
    assert found is not None
    assert math.isclose(found.x, 2.5, abs_tol=0.6)


def test_parallel_paths_in_adjacent_lanes_are_not_a_conflict():
    lane_a = [(float(i) * 0.5, 0.0, float(i) * 0.5) for i in range(11)]
    lane_b = [(float(i) * 0.5, 2.0, float(i) * 0.5) for i in range(11)]
    assert first_conflict(lane_a, lane_b, radius_m=0.9, time_window_s=1.0) is None


def test_distant_paths_are_not_a_conflict():
    far = [(float(i) * 0.5, 30.0, float(i) * 0.5) for i in range(11)]
    assert first_conflict(CROSSING_A, far, radius_m=0.9, time_window_s=1.0) is None


def test_crossing_paths_separated_in_TIME_are_not_a_conflict():
    """Crossing in space but not in time is not a conflict.

    The half that gets forgotten. Robots that cross the same cell minutes apart
    have not conflicted, and treating a shared cell as a conflict would have
    every robot in a warehouse yielding to every other one.
    """
    later = [(2.5, float(i) * 0.5 - 2.5, 60.0 + float(i) * 0.5) for i in range(11)]
    assert first_conflict(CROSSING_A, later, radius_m=0.9, time_window_s=1.0) is None


def test_first_conflict_agrees_with_the_head_of_conflicts():
    every = conflicts(CROSSING_A, CROSSING_B, radius_m=1.2, time_window_s=1.0)
    first = first_conflict(CROSSING_A, CROSSING_B, radius_m=1.2, time_window_s=1.0)
    assert every
    assert min(first.t_a, first.t_b) == min(every[0].t_a, every[0].t_b)


def test_a_wider_radius_never_finds_fewer_conflicts():
    narrow = conflicts(CROSSING_A, CROSSING_B, radius_m=0.5, time_window_s=1.0)
    wide = conflicts(CROSSING_A, CROSSING_B, radius_m=1.5, time_window_s=1.0)
    assert len(wide) >= len(narrow)


def test_closest_approach_reports_the_tightest_time_aligned_separation():
    separation, when = closest_approach(CROSSING_A, CROSSING_B, time_window_s=1.0)
    assert separation < 0.6
    assert when is not None


def test_closest_approach_is_infinite_when_predictions_do_not_overlap_in_time():
    later = [(2.5, 0.0, 600.0)]
    separation, when = closest_approach(CROSSING_A, later, time_window_s=1.0)
    assert separation == float("inf")
    assert when is None


# ---------------------------------------------------------------------------
# The shipped layer configuration
# ---------------------------------------------------------------------------


def _local_costmap_params():
    """Return the local costmap block of the UNRENDERED template."""
    path = os.path.join(config_dir(), "nav2_params.yaml")
    with open(path, "r", encoding="utf-8") as handle:
        # The template carries __PLACEHOLDER__ tokens, which are valid YAML
        # scalars, so it parses without rendering.
        data = yaml.safe_load(handle.read())
    return data["local_costmap"]["local_costmap"]["ros__parameters"]


def test_the_trajectory_layer_is_in_the_LOCAL_costmap():
    """The layer must be in the local costmap, not the global one.

    The requirement is about the local planner. A layer that drifted into the
    global costmap would still look wired up and would satisfy nothing.
    """
    assert "fleet_trajectory_layer" in _local_costmap_params()["plugins"]


def test_the_trajectory_layer_runs_after_inflation():
    plugins = _local_costmap_params()["plugins"]
    assert plugins.index("fleet_trajectory_layer") > plugins.index("inflation_layer")


def test_trajectory_cost_stays_below_the_inscribed_obstacle_value():
    """A peer's intention must be expensive, never impassable.

    253 is INSCRIBED_INFLATED_OBSTACLE and 254 is LETHAL_OBSTACLE; a footprint
    collision checker treats either as a collision. A peer's predicted path
    must be expensive, not impassable, or two robots deadlock on each other.
    """
    layer = _local_costmap_params()["fleet_trajectory_layer"]
    assert 0 < layer["max_cost"] < 253


def test_the_decay_leaves_the_far_horizon_cheap_and_the_near_term_expensive():
    layer = _local_costmap_params()["fleet_trajectory_layer"]
    near = layer["max_cost"] * math.exp(-0.5 / layer["decay_tau_s"])
    far = layer["max_cost"] * math.exp(-layer["horizon_s"] / layer["decay_tau_s"])
    assert near > 0.5 * layer["max_cost"]
    assert far < 0.2 * layer["max_cost"]


def test_every_robot_consumes_every_peer_and_never_itself():
    fleet = load_fleet()
    for robot in fleet:
        topics = peer_trajectory_topics(robot)
        assert len(topics) == len(fleet) - 1
        assert all(robot["name"] not in topic for topic in topics)
        for peer in fleet:
            if peer["name"] != robot["name"]:
                assert any(f"/{peer['name']}/" in topic for topic in topics)


def test_peer_topics_are_absolute():
    """Peer topics must be absolute names.

    A relative name inside /amrN resolves to the robot's own trajectory - a
    robot perfectly avoiding itself, which looks exactly like a working layer.
    """
    for robot in load_fleet():
        assert all(topic.startswith("/") for topic in peer_trajectory_topics(robot))
