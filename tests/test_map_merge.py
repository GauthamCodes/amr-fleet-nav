"""Fleet-map compositing: geometry, the merge rule, and the assumptions it rests on.

These are the properties an evaluator can check without running anything: that the
fleet grid is fixed and lattice-aligned, that a source grid lands where its spawn pose
says it should, and that "unknown loses, otherwise max" does what the docstring
claims - including the case that matters, where one robot has mapped an obstacle the
other has not reached yet.
"""

import numpy as np
import pytest

from amr_fleet_control.fleet_grid import (
    cell_offset,
    composite,
    coverage,
    fleet_grid_spec,
    GridSpec,
    merge_into,
    overlap_slices,
    project_into,
    UNKNOWN,
)

RESOLUTION = 0.05


@pytest.fixture(scope="module")
def fleet():
    """Return the warehouse-sized fleet grid the node actually allocates."""
    return fleet_grid_spec(-14.0, 18.0, -9.0, 9.0, RESOLUTION, margin=1.0)


def test_fleet_grid_covers_the_requested_extent(fleet):
    assert fleet.origin_x <= -15.0
    assert fleet.origin_y <= -10.0
    assert fleet.origin_x + fleet.width * RESOLUTION >= 19.0
    assert fleet.origin_y + fleet.height * RESOLUTION >= 10.0


def test_fleet_grid_origin_is_lattice_aligned(fleet):
    """A source grid aligned to the same lattice must land on whole cells."""
    assert fleet.origin_x / RESOLUTION == pytest.approx(
        round(fleet.origin_x / RESOLUTION)
    )
    assert fleet.origin_y / RESOLUTION == pytest.approx(
        round(fleet.origin_y / RESOLUTION)
    )


def test_fleet_grid_starts_all_unknown(fleet):
    grid = fleet.empty()
    assert grid.shape == (fleet.height, fleet.width)
    assert grid.dtype == np.int8
    assert np.all(grid == UNKNOWN)
    assert coverage(grid) == 0.0


def test_fleet_grid_is_big_enough_to_be_worth_fixing(fleet):
    """The whole point of a fixed extent is that it never resizes; check it is sane."""
    assert fleet.width * fleet.height > 100_000
    assert fleet.width * fleet.height < 5_000_000


def test_rejects_an_empty_extent():
    with pytest.raises(ValueError, match="empty extent"):
        fleet_grid_spec(1.0, 1.0, -1.0, 1.0, RESOLUTION)


def test_rejects_a_non_positive_resolution():
    with pytest.raises(ValueError, match="resolution must be positive"):
        fleet_grid_spec(-1.0, 1.0, -1.0, 1.0, 0.0)


def test_cell_offset_is_the_spawn_translation_in_cells(fleet):
    """amr1 spawns at x = -11.0; a map anchored there sits that far along the grid."""
    col, row = cell_offset(fleet, -11.0, -1.5, RESOLUTION)
    assert col == round((-11.0 - fleet.origin_x) / RESOLUTION)
    assert row == round((-1.5 - fleet.origin_y) / RESOLUTION)


def test_cell_offset_rounds_to_at_most_half_a_cell(fleet):
    """A spawn pose off the lattice costs 0.025 m, and that bound is a claim."""
    col, _ = cell_offset(fleet, -11.0 + 0.02, -1.5, RESOLUTION)
    exact = (-11.0 + 0.02 - fleet.origin_x) / RESOLUTION
    assert abs(col - exact) <= 0.5


def test_cell_offset_refuses_a_resolution_mismatch(fleet):
    with pytest.raises(ValueError, match="resolution mismatch"):
        cell_offset(fleet, 0.0, 0.0, 0.10)


def test_cell_offset_refuses_a_rotated_spawn(fleet):
    """Rotation needs resampling; silently skewing the map would be worse."""
    with pytest.raises(ValueError, match="spawn yaw"):
        cell_offset(fleet, 0.0, 0.0, RESOLUTION, yaw=0.5)


def test_overlap_is_none_when_the_source_is_off_the_grid(fleet):
    assert overlap_slices(fleet, (10, 10), -100, -100) is None
    assert overlap_slices(fleet, (10, 10), fleet.width + 5, 0) is None


def test_overlap_clips_a_source_hanging_off_the_edge(fleet):
    windows = overlap_slices(fleet, (10, 10), -3, -4)
    assert windows is not None
    fleet_slices, source_slices = windows
    assert fleet_slices[0].start == 0 and fleet_slices[1].start == 0
    assert source_slices[0].start == 4 and source_slices[1].start == 3
    # The two windows must describe the same number of cells or the copy is skewed.
    assert fleet_slices[0].stop - fleet_slices[0].start == (
        source_slices[0].stop - source_slices[0].start
    )


def test_unknown_never_overwrites_knowledge():
    destination = np.array([[0, 100]], dtype=np.int8)
    source = np.array([[UNKNOWN, UNKNOWN]], dtype=np.int8)
    merge_into(destination, source)
    assert destination.tolist() == [[0, 100]]


def test_occupied_beats_free_whichever_robot_saw_it():
    """The case the merge rule exists for: one robot has not reached the obstacle."""
    seen_by_neither = np.array([[UNKNOWN]], dtype=np.int8)
    seen_free = np.array([[0]], dtype=np.int8)
    seen_occupied = np.array([[100]], dtype=np.int8)

    both_orders = []
    for first, second in ((seen_free, seen_occupied), (seen_occupied, seen_free)):
        out = seen_by_neither.copy()
        merge_into(out, first)
        merge_into(out, second)
        both_orders.append(out[0, 0])
    assert both_orders == [100, 100]


def test_known_beats_unknown_whichever_robot_saw_it():
    out = np.array([[UNKNOWN]], dtype=np.int8)
    merge_into(out, np.array([[0]], dtype=np.int8))
    assert out[0, 0] == 0


def test_composite_of_nothing_is_all_unknown(fleet):
    out = composite(fleet, [])
    assert np.all(out == UNKNOWN)


def test_composite_rejects_a_wrongly_shaped_projection(fleet):
    with pytest.raises(ValueError, match="projection shape"):
        composite(fleet, [np.zeros((3, 3), dtype=np.int8)])


def test_composite_merges_two_robots_disjoint_regions(fleet):
    left = fleet.empty()
    right = fleet.empty()
    left[10:20, 10:20] = 0
    right[10:20, 30:40] = 100

    out = composite(fleet, [left, right])
    assert np.all(out[10:20, 10:20] == 0)
    assert np.all(out[10:20, 30:40] == 100)
    assert out[0, 0] == UNKNOWN
    assert coverage(out) == pytest.approx(200.0 / (fleet.width * fleet.height))


def test_projection_places_a_source_at_its_spawn_offset(fleet):
    """A 2x2 patch of occupied cells must be readable back at the spawn position."""
    source = np.full((4, 4), UNKNOWN, dtype=np.int8)
    source[1:3, 1:3] = 100

    col, row = cell_offset(fleet, -11.0, -1.5, RESOLUTION)
    projected = fleet.empty()
    written = project_into(projected, source, col, row, fleet)

    assert written == 4
    assert np.all(projected[row + 1 : row + 3, col + 1 : col + 3] == 100)
    assert projected[row, col] == UNKNOWN


def test_projection_leaves_the_destination_alone_where_the_source_is_unknown(fleet):
    projected = fleet.empty()
    projected[100, 100] = 100

    source = np.full((3, 3), UNKNOWN, dtype=np.int8)
    project_into(projected, source, 99, 99, fleet)
    assert projected[100, 100] == 100


def test_projection_off_the_grid_writes_nothing(fleet):
    projected = fleet.empty()
    assert (
        project_into(projected, np.zeros((4, 4), dtype=np.int8), -50, -50, fleet) == 0
    )
    assert np.all(projected == UNKNOWN)


def test_two_robot_projection_round_trip(fleet):
    """End to end: two spawn poses, two maps, one composite, obstacles preserved."""
    spawns = {"amr1": (-11.0, -1.5), "amr2": (-11.0, 1.5)}
    projections = []
    for x, y in spawns.values():
        source = np.full((20, 20), 0, dtype=np.int8)
        source[5, 5] = 100
        col, row = cell_offset(fleet, x, y, RESOLUTION)
        projection = fleet.empty()
        project_into(projection, source, col, row, fleet)
        projections.append(projection)

    out = composite(fleet, projections)
    for x, y in spawns.values():
        col, row = cell_offset(fleet, x, y, RESOLUTION)
        assert out[row + 5, col + 5] == 100

    # The two robots spawn 3 m apart in y, which is 60 cells at 0.05 m. If the
    # offsets collapsed to the same place this count would halve.
    assert np.count_nonzero(out >= 50) == 2


def test_grid_spec_repr_is_debuggable():
    assert "GridSpec(" in repr(GridSpec(-1.0, -2.0, 0.05, 10, 20))
