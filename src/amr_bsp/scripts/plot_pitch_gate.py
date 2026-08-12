#!/usr/bin/env python3
"""PHASE 2 - draw the max-pitch scan, before and after PitchGate.

Offline and re-runnable: the simulation is expensive, the picture is not, so
recording and drawing are separate steps and re-plotting never costs another run.
Same split as Phase 1's plot_run.py.

    ./ws.sh ros2 run amr_bsp plot_pitch_gate.py

Two panels, and the second is the one that carries the argument.

    LEFT - the scan in the sensor plane, raw against validated, with the per-beam
    gate radius drawn through it. The removed returns lie on a straight LINE, and
    that is the tell: a tilted scan plane meets the flat ground in a line, so the
    "obstacle" the raw scan reports is a wall of exactly the shape a floor makes.
    A real wall would be somewhere; this one moves as the robot pitches. The gate
    radius is the same line scaled by the 0.9 margin, which is why it sits parallel
    and just inside it, and why the range at which it cuts depends on azimuth
    (h / (sin|pitch| cos azimuth)) rather than being one radius for the whole scan.

    RIGHT - range against azimuth, which is where the gate curve and the removed
    returns can be compared directly. A reader can check by eye that every removed
    beam sits ABOVE the gate line and every surviving one below it, which is the
    truncation rule stated as a picture.
"""

import argparse
import csv
import math
import os

import matplotlib
import matplotlib.pyplot as plt

# Headless by construction: this runs on a machine with no display, and importing
# pyplot without this picks an interactive backend and fails at draw time.
matplotlib.use("Agg")


def read_scan(path):
    """Read the max-pitch scan CSV into parallel lists."""
    azimuth, raw, validated, gate = [], [], [], []
    with open(path, "r", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            azimuth.append(float(row["azimuth_rad"]))
            raw.append(float(row["raw_range"]))
            validated.append(float(row["validated_range"]))
            gate.append(float(row["gate_range"]))
    return azimuth, raw, validated, gate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", default="results")
    parser.add_argument("--tag", default="pitch_gate")
    args = parser.parse_args()

    stem = os.path.join(args.results_dir, f"phase2_{args.tag}")
    azimuth, raw, validated, gate = read_scan(f"{stem}_scan.csv")

    removed = [
        (a, r)
        for a, r, v in zip(azimuth, raw, validated)
        if math.isfinite(r) and not math.isfinite(v)
    ]
    kept = [(a, v) for a, v in zip(azimuth, validated) if math.isfinite(v)]

    # The gate radius diverges as a beam turns side-on: cos(azimuth) -> 0 means no
    # downward component and no ground intersection, so the true value there is
    # infinity. Drawing it unclipped puts a 150 m arc on a 5 m scan. Clip to the
    # data, and say in the axis label that the clip is a drawing decision.
    finite_raw = [r for r in raw if math.isfinite(r)]
    limit = 1.25 * max(finite_raw) if finite_raw else 12.0

    figure, (plan, profile) = plt.subplots(1, 2, figsize=(13.5, 6.0))

    plan.plot(
        [r * math.cos(a) for a, r in kept],
        [r * math.sin(a) for a, r in kept],
        ".",
        markersize=3,
        color="#1f77b4",
        label=f"kept ({len(kept)} beams)",
    )
    plan.plot(
        [r * math.cos(a) for a, r in removed],
        [r * math.sin(a) for a, r in removed],
        ".",
        markersize=5,
        color="#d62728",
        label=f"truncated by PitchGate ({len(removed)})",
    )
    gate_arc = [
        (a, g) for a, g in zip(azimuth, gate) if math.isfinite(g) and g <= limit
    ]
    plan.plot(
        [g * math.cos(a) for a, g in gate_arc],
        [g * math.sin(a) for a, g in gate_arc],
        ".",
        markersize=2,
        color="#2ca02c",
        label="gate radius, per beam",
    )
    plan.plot(0.0, 0.0, "ks", markersize=6, label="LiDAR")
    plan.set_aspect("equal")
    plan.set_xlim(-limit, limit)
    plan.set_ylim(-limit, limit)
    plan.set_xlabel("x, sensor frame (m)")
    plan.set_ylabel("y, sensor frame (m)  -  gate arc clipped to the scan")
    plan.set_title("Max-pitch scan, before and after")
    plan.grid(alpha=0.3)
    plan.legend(loc="upper right", fontsize=8)

    degrees = [math.degrees(a) for a in azimuth]
    profile.plot(
        degrees,
        [r if math.isfinite(r) else float("nan") for r in raw],
        ".",
        markersize=3,
        color="#999999",
        label="raw",
    )
    profile.plot(
        [math.degrees(a) for a, _ in removed],
        [r for _, r in removed],
        ".",
        markersize=5,
        color="#d62728",
        label="truncated",
    )
    profile.plot(
        degrees,
        [g if math.isfinite(g) else float("nan") for g in gate],
        "-",
        linewidth=1.4,
        color="#2ca02c",
        label="gate radius",
    )
    profile.set_ylim(0.0, limit)
    profile.set_xlabel("beam azimuth (deg)")
    profile.set_ylabel("range (m)")
    profile.set_title("Range against azimuth: every red point is above the gate")
    profile.grid(alpha=0.3)
    profile.legend(loc="upper right", fontsize=8)

    figure.tight_layout()
    figure.savefig(f"{stem}.png", dpi=150)
    print(f"wrote {stem}.png  ({len(removed)} beams truncated, {len(kept)} kept)")


if __name__ == "__main__":
    main()
