#!/usr/bin/env python3
r"""PHASE 1 - score a saved SLAM map against the world it was built in.

Reads the ``.pgm``/``.yaml`` pair slam_toolbox's map saver wrote and answers the
two questions a map artifact has to answer: what did the survey cover, and is what
it drew in the right place.

Coverage is reported per NAMED REGION rather than as one global percentage. A
single number over the whole grid is close to meaningless - the grid's extent is
itself a product of where the robot drove - whereas "the aisle is 99 % known and
the upper plateau is 0 % known" is a statement about the survey that a reader can
check against the route.

Accuracy is measured against the world file, not eyeballed. Every occupied cell is
compared with the true static geometry the simulator loaded, so a drifted or
double-drawn rack face shows up as a distance in metres instead of as a smudge
somebody has to notice.

Offline: no ROS graph, no simulator. Run it on any saved map.

    ./ws.sh ros2 run amr_navigation map_report.py \\
        --map results/phase1_map --out results/phase1_map.png \\
        --report results/phase1_map_coverage.md
"""

import argparse
import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import yaml

from amr_description.fleet_config import load_fleet, spawn_pose
from amr_gazebo.world_geometry import distance_to_boxes, static_boxes, world_root

# Headless by construction: this runs on a machine whose GPU is busy with Gazebo,
# and on CI where there is no display at all. Switching after the pyplot import
# keeps every import at the top of the file where the linter expects them.
matplotlib.use("Agg")

#: Regions the coverage table reports on, in world coordinates:
#: ``name: (x_min, x_max, y_min, y_max)``. Chosen to separate "the survey drove
#: here" from "the survey never went here", which is the distinction a coverage
#: claim lives or dies on.
REGIONS = {
    "main aisle (driven)": (-13.5, 1.5, -2.70, 2.70),
    "north rack backs": (-13.5, 3.8, 3.80, 6.00),
    "south rack backs": (-13.5, 3.8, -6.00, -3.80),
    "west of the racks": (-18.0, -13.5, -6.00, 6.00),
    "ramp approach": (1.5, 2.44, -1.25, 1.25),
    "ramp surface": (2.44, 6.0, -1.25, 1.25),
    "upper plateau": (6.0, 18.0, -9.00, 9.00),
}

#: An occupied cell further than this from any true static surface is drawn in the
#: wrong place. 0.15 m is three map cells: enough for scan noise and the finite
#: thickness of a mapped surface, tight enough that a drifted second copy of a rack
#: face fails it.
ACCURACY_TOLERANCE = 0.15


def read_pgm(path):
    """Read a binary (P5) PGM into a numpy array of uint8.

    Written by hand rather than pulled from an image library: the map artifact is
    the deliverable, and a reader that fails on it would be discovered at the worst
    possible moment. P5 is a header of three whitespace-separated integer fields
    with '#' comments, then raw bytes.
    """
    with open(path, "rb") as handle:
        data = handle.read()

    fields = []
    index = 0
    while len(fields) < 4:
        while index < len(data) and data[index : index + 1].isspace():
            index += 1
        if data[index : index + 1] == b"#":
            while index < len(data) and data[index] != 0x0A:
                index += 1
            continue
        start = index
        while index < len(data) and not data[index : index + 1].isspace():
            index += 1
        fields.append(data[start:index])
    index += 1  # single whitespace byte after the max value

    magic, width, height = fields[0], int(fields[1]), int(fields[2])
    if magic != b"P5":
        raise ValueError(f"{path} is {magic!r}, not a binary P5 PGM")
    return np.frombuffer(data[index : index + width * height], dtype=np.uint8).reshape(
        height, width
    )


def classify(image, negate, occupied_thresh, free_thresh):
    """Return (occupied, free, unknown) boolean masks for a map image.

    Follows the map_server convention: the stored byte is a whiteness, so occupancy
    is ``(255 - value) / 255`` unless the map declares ``negate``.
    """
    p = image.astype(np.float32) / 255.0
    p = p if negate else 1.0 - p
    occupied = p > occupied_thresh
    free = p < free_thresh
    return occupied, free, ~(occupied | free)


def cell_centres(shape, resolution, origin):
    """Return the world-frame x and y coordinate of every cell centre.

    Row 0 of the image is the TOP of the map, i.e. the highest y. Getting this
    backwards flips the map about its centre line and every accuracy number with
    it, so it is done once, here.
    """
    height, width = shape
    xs = origin[0] + (np.arange(width) + 0.5) * resolution
    ys = origin[1] + (height - 1 - np.arange(height) + 0.5) * resolution
    return np.meshgrid(xs, ys)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, help="map stem, without extension")
    parser.add_argument("--world", default="", help="rendered world SDF")
    parser.add_argument("--robot", default="amr1")
    parser.add_argument("--out", default="", help="PNG to write")
    parser.add_argument("--report", default="", help="markdown report to write")
    parser.add_argument("--title", default="")
    args = parser.parse_args()

    with open(f"{args.map}.yaml", "r", encoding="utf-8") as handle:
        meta = yaml.safe_load(handle)
    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(
            os.path.dirname(os.path.abspath(args.map)), image_path
        )
    image = read_pgm(image_path)
    resolution = float(meta["resolution"])
    occupied, free, unknown = classify(
        image,
        int(meta.get("negate", 0)),
        float(meta.get("occupied_thresh", 0.65)),
        float(meta.get("free_thresh", 0.25)),
    )

    # The map frame is anchored on the robot's spawn pose, so the saved origin is
    # in map coordinates and has to be shifted into world coordinates before any
    # comparison with the world file. amr1 spawns with yaw 0; a rotated spawn would
    # need the full transform here and this asserts rather than silently skewing.
    robot = {r["name"]: r for r in load_fleet()}[args.robot]
    sx, sy, _, syaw = spawn_pose(robot)
    if abs(syaw) > 1e-9:
        raise NotImplementedError(
            f"{args.robot} spawns at yaw {syaw}; map_report assumes an axis-aligned "
            "spawn so that map and world axes coincide"
        )
    origin_world = (meta["origin"][0] + sx, meta["origin"][1] + sy)

    world_sdf = args.world or os.path.join(
        os.environ.get("AMR_GENERATED_DIR", "/tmp/amr_fleet_nav_generated"),
        "warehouse_nav.sdf",
    )
    boxes = static_boxes(world_root(world_sdf))

    gx, gy = cell_centres(image.shape, resolution, origin_world)
    cell_area = resolution * resolution

    lines = []
    add = lines.append
    add("")
    add("=" * 78)
    add(f"PHASE 1 - SLAM map artifact: {os.path.basename(args.map)}")
    add("=" * 78)
    add(
        f"grid:                 {image.shape[1]} x {image.shape[0]} cells "
        f"at {resolution:.3f} m"
    )
    add(
        f"extent (world):       x {gx.min():+.2f} .. {gx.max():+.2f} m, "
        f"y {gy.min():+.2f} .. {gy.max():+.2f} m"
    )
    add(
        f"occupied cells:       {int(occupied.sum()):8d}"
        f"   ({occupied.sum() * cell_area:8.1f} m^2)"
    )
    add(
        f"free cells:           {int(free.sum()):8d}"
        f"   ({free.sum() * cell_area:8.1f} m^2)"
    )
    add(
        f"unknown cells:        {int(unknown.sum()):8d}"
        f"   ({unknown.sum() * cell_area:8.1f} m^2)"
    )
    add("-" * 78)

    add("[COVERAGE] fraction of each region the map has an opinion about")
    add(f"  {'region':<24}{'known':>9}{'free':>9}{'occupied':>10}   extent")
    for name, (x0, x1, y0, y1) in REGIONS.items():
        inside = (gx >= x0) & (gx <= x1) & (gy >= y0) & (gy <= y1)
        total = int(inside.sum())
        if total == 0:
            add(f"  {name:<24}{'0.0 %':>9}{'-':>9}{'-':>10}   outside the grid")
            continue
        known = float((inside & ~unknown).sum()) / total
        free_fraction = float((inside & free).sum()) / total
        occupied_fraction = float((inside & occupied).sum()) / total
        add(
            f"  {name:<24}{known * 100:8.1f} %{free_fraction * 100:8.1f} %"
            f"{occupied_fraction * 100:9.1f} %   "
            f"x [{x0:+.1f},{x1:+.1f}] y [{y0:+.1f},{y1:+.1f}]"
        )
    add("-" * 78)

    add("[ACCURACY] every occupied cell against the world file's true geometry")
    rows, cols = np.nonzero(occupied)
    distances = [
        distance_to_boxes(boxes, gx[r, c], gy[r, c]) for r, c in zip(rows, cols)
    ]
    if distances:
        distances.sort()
        within = sum(1 for d in distances if d <= ACCURACY_TOLERANCE)
        median = distances[len(distances) // 2]
        p95 = distances[int(0.95 * (len(distances) - 1))]
        add(f"  occupied cells scored:              {len(distances):8d}")
        add(f"  median distance to a true surface:  {median:8.3f} m")
        add(f"  p95:                                {p95:8.3f} m")
        add(f"  worst:                              {distances[-1]:8.3f} m")
        add(
            f"  within {ACCURACY_TOLERANCE:.2f} m of a true surface:  "
            f"{100.0 * within / len(distances):8.1f} %"
        )
    else:
        add("  no occupied cells - the map is empty")
    add("=" * 78)
    add("")

    text = "\n".join(lines)
    print(text, flush=True)

    if args.report:
        os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as handle:
            handle.write(f"# Phase 1 - SLAM map artifact: {args.map}\n\n")
            handle.write("```\n")
            handle.write(text)
            handle.write("\n```\n")

    if args.out:
        _plot(args, image, resolution, origin_world, boxes)


def _plot(args, image, resolution, origin_world, boxes):
    """Render the map with the world's true geometry drawn over it."""
    height, width = image.shape
    extent = (
        origin_world[0],
        origin_world[0] + width * resolution,
        origin_world[1],
        origin_world[1] + height * resolution,
    )

    figure, axes = plt.subplots(figsize=(13, 7))
    axes.imshow(image, cmap="gray", vmin=0, vmax=255, extent=extent, origin="upper")
    for box in boxes:
        axes.add_patch(
            plt.Rectangle(
                (box["x"] - box["hx"], box["y"] - box["hy"]),
                2 * box["hx"],
                2 * box["hy"],
                fill=False,
                edgecolor="tab:red",
                linewidth=1.0,
                linestyle="--",
            )
        )
    axes.plot([], [], "--", color="tab:red", label="true static geometry (world file)")
    axes.set_xlabel("world x (m)")
    axes.set_ylabel("world y (m)")
    axes.set_title(args.title or os.path.basename(args.map))
    axes.legend(loc="upper right")
    axes.grid(alpha=0.2)
    figure.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    figure.savefig(args.out, dpi=130)
    print(f"wrote {args.out}", flush=True)


if __name__ == "__main__":
    main()
