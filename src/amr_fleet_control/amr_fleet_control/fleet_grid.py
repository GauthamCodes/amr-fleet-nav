"""Occupancy-grid geometry for the fleet map, as pure functions.

No ROS imports. Everything here takes and returns numpy arrays and plain numbers, so
the compositing rules are testable offline without a graph - the same split that makes
``amr_bsp.validators`` testable.

THE FLEET GRID IS FIXED

    The fleet map is allocated once, at a fixed extent covering the whole warehouse,
    and never resized. That is not a convenience: ``Costmap2DROS`` declares its own
    ``width``/``height``/``origin_*``, defaulting to 5 x 5 m at (0, 0), and both robots
    spawn near (-11, +/-1.5) - outside it. A costmap that sizes itself from whatever
    the mapper has explored so far starts too small, reports "Start Coordinates of()
    was outside bounds", and then resizes every time either robot sees a new corridor,
    wiping the global obstacle layer on each resize.

TWO CONVENTIONS THAT ARE EASY TO GET BACKWARDS

    ``nav_msgs/OccupancyGrid`` is row-major with row 0 at the LOWEST y, which is the
    opposite of a PGM image, where row 0 is the top. ``map_report.cell_centres``
    handles the image convention; this module handles the message convention, and the
    two must not be cross-applied.

    Cell (mx, my) covers world x in ``[origin_x + mx*res, origin_x + (mx+1)*res)``, so
    its CENTRE is at ``origin_x + (mx + 0.5) * res``.

WHY THE TRANSFORM IS AN INTEGER CELL OFFSET

    Every robot in fleet.yaml spawns with ``yaw = 0`` and every grid here is 0.05 m,
    so the transform from a robot's map frame into the fleet frame is a pure
    translation on a shared lattice. Projecting a source grid is then a slice
    assignment rather than a per-cell inverse transform - which matters, because a
    per-cell Python loop over 250 000 cells runs on the executor thread and shows up
    as a real-time-factor collapse that reads like a Gazebo problem.

    :func:`cell_offset` rounds to the nearest cell, so a spawn pose that is not a
    multiple of the resolution costs at most half a cell (0.025 m). Non-zero spawn
    yaw is rejected rather than silently skewed.
"""

import math

import numpy as np

#: The one frame the whole fleet plans in. Defined to coincide with the Gazebo world
#: origin, so a goal in world coordinates is already a goal in fleet coordinates.
#: Imported by amr_navigation.params rather than restated there - the frame name and
#: the topic below are a contract between FleetMapNode and both global costmaps, and
#: a contract written down twice is a contract that drifts.
FLEET_FRAME = "fleet_map"

#: Topic the composited map is published on. Absolute: FleetMapNode is unnamespaced
#: and both robots' costmaps subscribe to the same one.
FLEET_MAP_TOPIC = "/fleet_map"

#: Value of a cell no robot has observed, per the OccupancyGrid convention.
UNKNOWN = -1

#: Occupancy probability at or above which a known cell counts as occupied.
OCCUPIED_THRESHOLD = 50


class GridSpec:
    """The geometry of an occupancy grid: where it starts, how big, how fine."""

    def __init__(self, origin_x, origin_y, resolution, width, height):
        """Store the grid's origin corner, cell size and extent in cells."""
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)

    @property
    def shape(self):
        """Return the numpy shape ``(height, width)`` of this grid."""
        return (self.height, self.width)

    def empty(self):
        """Return an all-unknown array with this grid's shape."""
        return np.full(self.shape, UNKNOWN, dtype=np.int8)

    def __repr__(self):
        """Return a debuggable one-line summary."""
        return (
            f"GridSpec(origin=({self.origin_x:.2f}, {self.origin_y:.2f}), "
            f"res={self.resolution}, {self.width}x{self.height})"
        )


def fleet_grid_spec(x_min, x_max, y_min, y_max, resolution, margin=0.0):
    """Return the fixed fleet-grid geometry covering a world extent.

    Args:
        x_min, x_max, y_min, y_max: World-frame bounds to cover, in metres.
        resolution: Cell size in metres.
        margin: Extra metres added on every side.

    Returns:
        A :class:`GridSpec` whose origin is snapped DOWN to the resolution lattice, so
        that a source grid whose origin is also lattice-aligned lands on whole cells.
    """
    if resolution <= 0.0:
        raise ValueError(f"resolution must be positive, got {resolution}")
    if x_max <= x_min or y_max <= y_min:
        raise ValueError(f"empty extent: x[{x_min}, {x_max}] y[{y_min}, {y_max}]")

    origin_x = math.floor((x_min - margin) / resolution) * resolution
    origin_y = math.floor((y_min - margin) / resolution) * resolution
    width = int(math.ceil(((x_max + margin) - origin_x) / resolution))
    height = int(math.ceil(((y_max + margin) - origin_y) / resolution))
    return GridSpec(origin_x, origin_y, resolution, width, height)


#: The warehouse extent the fleet grid covers, as ``(x_min, x_max, y_min, y_max)``.
#: Mirrors amr_gazebo/config/world.yaml: the lower plateau spans x [-14, 2.4] and the
#: upper plateau x [6, 18], both over y [-9, 9]. Pinned against the rendered world by
#: tests/test_fleet_frames.py, so changing the world without changing this fails a
#: test instead of silently truncating the map.
WAREHOUSE_EXTENT = (-14.0, 18.0, -9.0, 9.0)

#: The ramp's plan-view footprint in world metres, as ``(x_min, x_max, y_min, y_max)``.
#: Toe at x = 2.4423 and crest at x = 6.0, both derived in warehouse.sdf.xacro from
#: ``ramp_angle_deg``; width 2.5 m about the centre line.
#:
#: Deliberately NOT taken from ``world_geometry.static_boxes``, which returns the
#: ramp slab's BURIED downhill extension and so reports the footprint starting at
#: x = 0.96 - 1.5 m of floor where nothing is above ground. A mask generated from
#: that would put ramp cost on the flat approach.
RAMP_REGION = (2.4423, 6.0, -1.25, 1.25)

#: Cell size shared by the fleet map, both global costmaps and the ramp filter mask.
#: slam_toolbox publishes at this resolution too, which is what makes compositing an
#: integer cell translation rather than a resampling problem.
FLEET_RESOLUTION = 0.05

#: Extra metres beyond the warehouse extent, so a robot near the wall still has
#: costmap around it.
FLEET_MARGIN = 1.0


def default_fleet_grid():
    """Return the one fleet grid every consumer must agree on.

    FleetMapNode allocates it, both global costmaps declare its extent, and the ramp
    filter mask is generated onto it. Three separate configuration systems, one
    definition here.
    """
    return fleet_grid_spec(*WAREHOUSE_EXTENT, FLEET_RESOLUTION, margin=FLEET_MARGIN)


def cell_offset(fleet, source_origin_x, source_origin_y, source_resolution, yaw=0.0):
    """Return the ``(col_offset, row_offset)`` placing a source grid in the fleet grid.

    Source cell ``(mx, my)`` maps to fleet cell ``(mx + col_offset, my + row_offset)``.

    Args:
        fleet: The destination :class:`GridSpec`.
        source_origin_x, source_origin_y: Source grid origin, ALREADY EXPRESSED IN THE
            FLEET FRAME (i.e. the robot's map origin plus its spawn translation).
        source_resolution: Source cell size; must match the fleet grid's.
        yaw: Rotation between the two frames. Only 0 is supported.

    Raises:
        ValueError: If the resolutions differ or ``yaw`` is non-zero. Both are
            recoverable in principle - by resampling - but doing it silently would
            turn a configuration mistake into a subtly misaligned map, which is far
            harder to notice than a refusal to start.
    """
    if abs(source_resolution - fleet.resolution) > 1e-9:
        raise ValueError(
            f"resolution mismatch: source {source_resolution} vs fleet "
            f"{fleet.resolution}; compositing assumes a shared lattice"
        )
    if abs(yaw) > 1e-9:
        raise ValueError(
            f"spawn yaw {yaw} is not supported: compositing is a translation on a "
            "shared lattice, and a rotated source would need resampling"
        )
    col = int(round((source_origin_x - fleet.origin_x) / fleet.resolution))
    row = int(round((source_origin_y - fleet.origin_y) / fleet.resolution))
    return col, row


def overlap_slices(fleet, source_shape, col_offset, row_offset):
    """Return the ``(fleet_slices, source_slices)`` of the region they share.

    Returns ``None`` when the source falls entirely outside the fleet grid, which is a
    configuration error worth reporting rather than a silent no-op.
    """
    src_h, src_w = source_shape

    row_lo = max(0, row_offset)
    row_hi = min(fleet.height, row_offset + src_h)
    col_lo = max(0, col_offset)
    col_hi = min(fleet.width, col_offset + src_w)
    if row_lo >= row_hi or col_lo >= col_hi:
        return None

    fleet_slices = (slice(row_lo, row_hi), slice(col_lo, col_hi))
    source_slices = (
        slice(row_lo - row_offset, row_hi - row_offset),
        slice(col_lo - col_offset, col_hi - col_offset),
    )
    return fleet_slices, source_slices


def merge_into(destination, source, known_mask=None):
    """Merge ``source`` into ``destination`` in place: unknown loses, otherwise max.

    The rule is deliberately conservative for navigation. A cell either robot has seen
    as occupied stays occupied, because disagreement between two independently built
    maps is far more likely to mean "one of them has not seen this obstacle yet" than
    "one of them is wrong". Unknown never overwrites knowledge, so a robot that has
    not explored a region cannot erase what the other one mapped there.

    Args:
        destination: ``int8`` array, modified in place.
        source: ``int8`` array of the same shape.
        known_mask: Optional precomputed ``source >= 0``.

    Returns:
        The destination array, for chaining.
    """
    known = known_mask if known_mask is not None else (source >= 0)
    np.maximum(destination, source, out=destination, where=known)
    return destination


def composite(fleet, projections):
    """Composite already-projected, fleet-shaped grids into one.

    Args:
        fleet: The fleet :class:`GridSpec`.
        projections: Iterable of fleet-shaped ``int8`` arrays, one per robot.

    Returns:
        A new ``int8`` array: all-unknown where nobody has looked, otherwise the
        per-cell maximum over the robots that have.
    """
    out = fleet.empty()
    for projection in projections:
        if projection is None:
            continue
        if projection.shape != fleet.shape:
            raise ValueError(
                f"projection shape {projection.shape} != fleet shape {fleet.shape}"
            )
        merge_into(out, projection)
    return out


def project_into(fleet_array, source, col_offset, row_offset, fleet):
    """Write a source grid into a fleet-shaped array at an integer cell offset.

    Only cells the source actually knows are written; unknown source cells leave the
    destination untouched. Returns the number of cells written, or 0 if the source
    does not overlap the fleet grid at all.
    """
    windows = overlap_slices(fleet, source.shape, col_offset, row_offset)
    if windows is None:
        return 0
    fleet_slices, source_slices = windows
    patch = source[source_slices]
    known = patch >= 0
    view = fleet_array[fleet_slices]
    np.copyto(view, patch, where=known)
    return int(np.count_nonzero(known))


def occupied_mask(grid):
    """Return the boolean mask of cells known to be occupied."""
    return grid >= OCCUPIED_THRESHOLD


def known_mask(grid):
    """Return the boolean mask of cells any robot has observed."""
    return grid >= 0


def coverage(grid):
    """Return the fraction of cells that are known, in [0, 1]."""
    if grid.size == 0:
        return 0.0
    return float(np.count_nonzero(known_mask(grid))) / float(grid.size)
