"""Generate the ramp costmap filter mask, at launch time, from world geometry.

WHY A FILTER MASK RATHER THAN A COSTMAP LAYER

    The assignment permits "a custom cost function OR a tuned configuration" for the
    ramp. PLAN.md section 5's cut list puts a custom ``RampCostLayer`` above a filter
    mask, and under the delivery schedule that cut was taken. A KeepoutFilter fed a
    graded mask is a tuned configuration in the assignment's own words, and it costs
    a YAML block instead of a C++ pluginlib plugin.

WHY IT IS GENERATED RATHER THAN COMMITTED

    ``ramp_angle_deg`` is a world.yaml knob, and the ramp's toe and crest are derived
    from it. A committed PGM goes stale the first time that angle changes, and a
    stale mask does not fail - it just puts the cost somewhere the ramp is not. So
    the mask is rendered into the generated directory alongside the world SDF and the
    per-robot Nav2 params, by the same pattern and for the same reason.

FOUR THINGS ABOUT THE YAML THAT EACH FAIL SILENTLY

    mode: scale        The default is ``trinary``, which collapses everything between
                       the thresholds to -1. ``CostmapFilter::getMaskCost`` maps -1 to
                       NO_INFORMATION and the filter skips the cell, so a graded mask
                       under the default mode produces no graded cost at all.

    thresholds         ``free_thresh: 0.0`` and ``occupied_thresh: 1.0`` leave the
                       whole grey range to the scale branch. Stock values clip both
                       ends and quietly turn most of the gradient into 0 or 100.

    white background   map_saver writes unknown as 205, which is occupancy 0.196 and
                       therefore a mask value of about 19 - roughly cost 48 over the
                       ENTIRE warehouse. The background here is pure 255: lighter is
                       cheaper, darker is more expensive, and 255 is free.

    value ceiling      Mask 100 becomes cost 254 (LETHAL_OBSTACLE) and 253 is
                       INSCRIBED_INFLATED_OBSTACLE, which the footprint collision
                       checkers treat as in-collision. An "expensive" ramp must stay
                       below both, so :data:`MAX_MASK_VALUE` caps it.

PHASE 3 SHIPS AN ALL-ZERO MASK

    The plumbing is what Phase 3 verifies: the filter loads, the costmap activates,
    and the measured cost over the ramp is nothing. Phase 4 supplies regions and
    tunes the value against the two-route A/B. Passing no regions is therefore a
    supported, deliberate configuration, not a degenerate one.
"""

import os

import numpy as np

from amr_gazebo.world_builder import generated_dir

#: Highest mask value the generator will write. 100 -> cost 254 (lethal) and
#: 99 -> cost 252, still under the 253 inscribed threshold; 90 leaves headroom.
MAX_MASK_VALUE = 90

#: Pixel value meaning "no cost added here". White is free; see the module docstring.
FREE_PIXEL = 255


def mask_pixel(value):
    """Return the PGM pixel encoding a mask value in [0, 100].

    Inverts the map_server convention ``occupancy = (255 - pixel) / 255``, so a mask
    value of 0 is white and higher values are progressively darker.
    """
    if not 0 <= value <= MAX_MASK_VALUE:
        raise ValueError(
            f"mask value {value} outside [0, {MAX_MASK_VALUE}]; 100 is lethal and "
            "253 is already treated as in-collision by the footprint checkers"
        )
    return int(round(FREE_PIXEL * (1.0 - value / 100.0)))


def render_mask_array(grid, regions=()):
    """Return the mask image for a grid, as a PGM-ordered ``uint8`` array.

    Args:
        grid: The :class:`~amr_fleet_control.fleet_grid.GridSpec` to render onto. The
            mask must share the fleet grid's origin and resolution or the cost lands
            in the wrong place.
        regions: Iterable of ``(x_min, x_max, y_min, y_max, value)`` in world metres,
            where ``value`` is a mask value in [0, MAX_MASK_VALUE]. Empty gives a
            uniformly free mask, which is what Phase 3 ships.

    Returns:
        A ``(height, width)`` ``uint8`` array in PGM row order - row 0 is the TOP of
        the image, i.e. the HIGHEST y. That is the opposite of an OccupancyGrid
        message, and getting it backwards mirrors the mask about the aisle.
    """
    image = np.full(grid.shape, FREE_PIXEL, dtype=np.uint8)

    for x_min, x_max, y_min, y_max, value in regions:
        col_lo = int(np.floor((x_min - grid.origin_x) / grid.resolution))
        col_hi = int(np.ceil((x_max - grid.origin_x) / grid.resolution))
        row_lo = int(np.floor((y_min - grid.origin_y) / grid.resolution))
        row_hi = int(np.ceil((y_max - grid.origin_y) / grid.resolution))

        col_lo, col_hi = max(0, col_lo), min(grid.width, col_hi)
        row_lo, row_hi = max(0, row_lo), min(grid.height, row_hi)
        if col_lo >= col_hi or row_lo >= row_hi:
            continue

        # Grid rows count up from the lowest y; image rows count down from the
        # highest. Flip here, once, rather than at every read site.
        top = grid.height - row_hi
        bottom = grid.height - row_lo
        image[top:bottom, col_lo:col_hi] = mask_pixel(value)

    return image


def write_mask(grid, regions=(), stem="ramp_mask"):
    """Write the mask PGM and its map_server YAML, and return the YAML path.

    Args:
        grid: The grid to render onto.
        regions: See :func:`render_mask_array`.
        stem: Basename, without extension, inside the generated directory.

    Returns:
        Absolute path to the written ``.yaml``, ready for map_server's
        ``yaml_filename``.
    """
    image = render_mask_array(grid, regions)
    out_dir = generated_dir()
    pgm_path = os.path.join(out_dir, f"{stem}.pgm")
    yaml_path = os.path.join(out_dir, f"{stem}.yaml")

    with open(pgm_path, "wb") as handle:
        handle.write(f"P5\n{grid.width} {grid.height}\n255\n".encode("ascii"))
        handle.write(image.tobytes())

    # Written by hand rather than through yaml.safe_dump so the comments survive:
    # every one of these keys is a silent failure if it is wrong.
    with open(yaml_path, "w", encoding="utf-8") as handle:
        handle.write(
            f"""# GENERATED by amr_navigation.ramp_mask - edit the generator, not this.
#
# mode: scale is load-bearing. The default 'trinary' collapses every value between
# the thresholds to -1, which the costmap filter reads as NO_INFORMATION and skips,
# so a graded mask would produce no graded cost.
#
# The thresholds are deliberately at the extremes so that the whole grey range
# reaches the scale branch instead of being clipped to 0 or 100 at the ends.
image: {stem}.pgm
mode: scale
resolution: {grid.resolution}
origin: [{grid.origin_x}, {grid.origin_y}, 0.0]
negate: 0
occupied_thresh: 1.0
free_thresh: 0.0
"""
        )
    return yaml_path
