#!/usr/bin/env python3
"""PHASE 1 - plot a recorded navigation run.

Reads the CSVs ``nav_goal_run.py`` wrote and draws two panels: the executed path
against the plan and the world's real geometry, and clearance against time.

Offline and re-runnable: the run is expensive, the plot is not, so the recording
and the picture are separate steps. Re-plotting never costs another simulation.

    ./ws.sh ros2 run amr_navigation plot_run.py --tag actors

The dynamic returns are plotted as scattered points. They are the pedestrian's
track as the LiDAR alone saw it - no reconstruction involved - which is why they
are worth drawing next to the robot's path: the gap between the two curves at
their closest is the headline clearance number, visible rather than asserted.
"""

import argparse
import csv
import math
import os

import matplotlib
import matplotlib.pyplot as plt

from amr_gazebo.world_geometry import static_boxes, world_root

# Headless by construction - see map_report.py.
matplotlib.use("Agg")


def read_track(path):
    """Read a track CSV into a dict of columns."""
    columns = None
    with open(path, "r", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if columns is None:
                columns = {key: [] for key in row}
            for key, value in row.items():
                columns[key].append(float(value))
    return columns or {}


def read_plan(path):
    """Read a plan CSV into ``([x], [y])``."""
    xs, ys = [], []
    if not os.path.exists(path):
        return xs, ys
    with open(path, "r", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
    return xs, ys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag", default="run")
    parser.add_argument("--results-dir", default="results")
    parser.add_argument("--world", default="")
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    stem = os.path.join(args.results_dir, f"phase1_nav_{args.tag}")
    track = read_track(f"{stem}_track.csv")
    plan_x, plan_y = read_plan(f"{stem}_plan.csv")
    if not track:
        raise SystemExit(f"no samples in {stem}_track.csv")

    world_sdf = args.world or os.path.join(
        os.environ.get("AMR_GENERATED_DIR", "/tmp/amr_fleet_nav_generated"),
        "warehouse_nav.sdf",
    )
    boxes = static_boxes(world_root(world_sdf))

    t0 = track["t"][0]
    t = [value - t0 for value in track["t"]]
    dynamic = [
        (x, y)
        for x, y in zip(track["dyn_x"], track["dyn_y"])
        if not (math.isnan(x) or math.isnan(y))
    ]
    finite_dynamic = [
        (time, c) for time, c in zip(t, track["clearance_dynamic"]) if math.isfinite(c)
    ]

    figure, (space, series) = plt.subplots(
        1, 2, figsize=(17, 7), gridspec_kw={"width_ratios": [1.35, 1.0]}
    )

    for box in boxes:
        space.add_patch(
            plt.Rectangle(
                (box["x"] - box["hx"], box["y"] - box["hy"]),
                2 * box["hx"],
                2 * box["hy"],
                facecolor="0.85",
                edgecolor="0.4",
                linewidth=0.8,
            )
        )
    if dynamic:
        space.scatter(
            [p[0] for p in dynamic],
            [p[1] for p in dynamic],
            s=9,
            color="tab:orange",
            alpha=0.7,
            label="nearest dynamic return (LiDAR)",
        )
    if plan_x:
        space.plot(plan_x, plan_y, "--", color="tab:green", label="first global plan")
    space.plot(
        track["gt_x"], track["gt_y"], "-", color="tab:blue", lw=2, label="executed path"
    )
    if "slam_x" in track:
        space.plot(
            track["slam_x"],
            track["slam_y"],
            ":",
            color="tab:purple",
            lw=1.2,
            label="SLAM pose estimate",
        )
    space.plot(track["gt_x"][0], track["gt_y"][0], "o", color="black", label="start")
    space.plot(
        track["gt_x"][-1],
        track["gt_y"][-1],
        "*",
        ms=14,
        color="tab:red",
        label="final pose",
    )

    if finite_dynamic:
        index = min(range(len(track["t"])), key=lambda i: _or_inf(track, i))
        space.annotate(
            f"closest approach {track['clearance_dynamic'][index]:.2f} m",
            xy=(track["gt_x"][index], track["gt_y"][index]),
            xytext=(track["gt_x"][index] - 3.0, track["gt_y"][index] - 2.2),
            arrowprops={"arrowstyle": "->", "color": "tab:orange"},
            color="tab:orange",
        )

    space.set_aspect("equal")
    space.set_xlabel("world x (m)")
    space.set_ylabel("world y (m)")
    space.set_title(f"Phase 1 navigation run: {args.tag}")
    space.legend(loc="lower left", fontsize=9)
    space.grid(alpha=0.2)

    series.plot(
        t,
        track["clearance_any"],
        color="0.5",
        lw=1.0,
        label="clearance to any obstacle",
    )
    if finite_dynamic:
        series.plot(
            [p[0] for p in finite_dynamic],
            [p[1] for p in finite_dynamic],
            color="tab:orange",
            lw=1.8,
            label="clearance to a dynamic obstacle",
        )
        floor = min(p[1] for p in finite_dynamic)
        series.axhline(floor, color="tab:orange", ls="--", lw=0.9)
        series.annotate(
            f"min {floor:.2f} m",
            xy=(t[-1], floor),
            xytext=(-70, 8),
            textcoords="offset points",
            color="tab:orange",
        )
    speed = series.twinx()
    speed.plot(t, track["speed"], color="tab:blue", lw=0.9, alpha=0.6)
    speed.set_ylabel("measured speed (m/s)", color="tab:blue")

    series.set_xlabel("time since dispatch (s)")
    series.set_ylabel("clearance from footprint (m)")
    series.set_title("Clearance over the run")
    series.legend(loc="upper right", fontsize=9)
    series.grid(alpha=0.2)

    figure.tight_layout()
    out = args.out or f"{stem}.png"
    figure.savefig(out, dpi=130)
    print(f"wrote {out}", flush=True)


def _or_inf(track, index):
    """Return the dynamic clearance at an index, or infinity when there is none."""
    value = track["clearance_dynamic"][index]
    return value if math.isfinite(value) else float("inf")


if __name__ == "__main__":
    main()
