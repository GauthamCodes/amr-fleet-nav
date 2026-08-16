# Demo media

Everything here was captured from a live run of a command in
[`../HOW_TO_RUN.md`](../HOW_TO_RUN.md), on the code in this repository. Nothing is a
mock-up, nothing is staged beyond the scenario the launch file stages itself, and
nothing was carried over from an earlier build.

**These are illustrations, not evidence.** The repository's evidence is
[`../results/`](../results/), which is what [`../README.md`](../README.md) §5 quotes
figure by figure. What the files here are for is so that an evaluator can see what a
command is supposed to produce *before* spending five minutes running it, and can hold
their own run against a reference afterwards.

```
media/
├── previews/   short GIFs, one per demo, embedded next to that demo's command
├── verified/   full-size stills of the same runs
└── archive/    reports from recorded runs whose clips are no longer carried
```

## `previews/` — what each command looks like

Cut from a live run, sped up where the robot's travel is uneventful and left near real
time where the behaviour itself is the point.

| Preview | Length / speed | Shows |
|---|---|---|
| `cooperative_mapping.gif` | 11 s at 14× | Gazebo left, RViz right. `/fleet_map` grows from empty to covering the aisle, filling in **from both ends at once** because both robots are contributing, with the rack bays resolving as black cut-outs. Demo A. |
| `concurrent_goals.gif` | 10 s at 2× | Gazebo left, RViz right. Both plans drawn in the `fleet_map` frame before either robot moves — **green AMR-1, cyan AMR-2** — then both robots track their own plan east and **both arrive**. Demo B. |
| `safety_override.gif` | 10 s at **1.05×** | A pedestrian walks into AMR-1's path; the SafetyGate halts the robot short of them and releases on hysteresis once they move away. Demo C. |
| `yield_protocol.gif` | 10 s at 6× | The shared map: both robots converge on the single 3.0 m gap in the barrier, pass through it one at a time, and both reach their goals. Demo D. |

**Three of these four were reframed after a review found the first cut unreadable**, and
the reason is worth stating because it is the same mistake three times. The robots are
about 0.7 m long in a 34 m warehouse, so a full-screen grab at the shipped RViz scale
renders them **about ten pixels across** — `concurrent_goals.gif` in particular read as
"nothing moved" when in fact both robots crossed the aisle and arrived. The fix was a
recording-only RViz camera scale per scene (13.9 m across for the goals, 12.6 m for the
yield) and a crop onto the aisle, plus dropping the speed-up on the safety clip. **No
simulation parameter was changed and no clip is composited from more than one run**; the
camera and the crop are the only editorial choices, exactly as described at the bottom of
this file.

`safety_override.gif` is near real time rather than sped up **because the behaviour it
has to show is 0.3–0.7 s long**. Its halts were being compressed to two or three frames
at the 2.4× the earlier cut used, which is indistinguishable from the robot not stopping
at all. A demonstration that is too fast to read is not a demonstration.

## `verified/` — stills to hold your own run against

| File | Shows |
|---|---|
| `warehouse_both_robots.png` | The warehouse Gazebo should open to — rack rows either side, the 8° ramp and upper plateau beyond, walking pedestrians, and **both robots in the aisle** (AMR-2 amber, AMR-1 blue) |
| `rviz_fleet_map_config.png` | The same run with RViz's **Displays panel open**, so the configuration is legible rather than asserted: **Fixed Frame `fleet_map`**, `Global Status: Ok`, topic **`/fleet_map`**, update topic `/fleet_map_updates`, resolution 0.05 m, **680 × 400**, origin −15 / −10 |
| `cooperative_mapping.png` | Demo A at the end of a survey lap: one `/fleet_map` covering the aisle with the rack bays cut out, built by both robots into a single grid |
| `concurrent_goals.png` | Demo B with **both routes planned at once** in the `fleet_map` frame, green for AMR-1 and cyan for AMR-2, `Global Status: Ok` |
| `safety_override.png` | Demo C: a pedestrian has walked into AMR-1's path and the robot has stopped short of them |
| `yield_gap.png` | Demo D: the staged conflict — both robots approaching the single 3.0 m gap in the red barrier, with the shared map and both plans in RViz |
| `full_system_bringup.png` | Demo E: the whole graph up — Gazebo with both robots and the pedestrians, RViz with `/fleet_map` and both robots' scans. **Nothing is moving, and that is correct**: this demo sends no goals |
| `imu_validation.png` | Demo F: the end of a real injection run — two of the `imu REJECT` warnings and the whole `VERDICT` block, ending `RESULT: PASS`. Cropped above the line where the run prints the path it wrote to |

## What the recorded runs actually measured

The GIFs are illustrations, so where a recorded run's own numbers differ from the
canonical artifact, the difference is stated here rather than hidden.

- **Cooperative mapping.** The recorded run ended **amr1 20 accepted / amr2 22
  accepted**. Per-run counts move a lot — four runs of the same command gave 41/38,
  12/22, 15/65 and 20/22 — because the score depends on where each robot is when a scan
  lands. The figure to quote is the committed one,
  [`../results/phase3_selective_updates.md`](../results/phase3_selective_updates.md):
  **41 scored, 23 accepted, 18 deferred (43.9 %)**, amr1 13/8 and amr2 10/10.

- **Concurrent goals.** The recorded run reached both goals, amr1 18.6 s and amr2
  11.3 s. That is not a one-off, and this run is not an unrecorded extra: it is **run 6**
  of the six in
  [`../results/phase3_concurrent_goals_recheck.md`](../results/phase3_concurrent_goals_recheck.md),
  which also retracts an earlier claim that only one robot ever arrives.

- **Safety override.** The recorded run took **4 halts**, not the 3 in the committed
  [`../results/phase2_safety_suppressed.md`](../results/phase2_safety_suppressed.md).
  The count depends on where the pedestrians happen to be; **the count is not the
  result.** What is invariant across both runs is 0 commands leaking past the latch, 0
  Nav2 recovery behaviours during a halt, and a sensor-stamp-to-zero-command time in
  single-digit milliseconds.

- **The yield.** Escalation is **non-deterministic and the previews prove it**: six
  runs while recording gave **0 escalations at the default prediction window, then 1, 0,
  2, 0 and 1 at `time_window_s:=5.0`**. Three of those six therefore reported
  **`NOT EXERCISED`** — which is the correct verdict for a run in which the robots
  opened the gap between themselves, not a failure.

  The clip carried here is the last of them, and **that run's own two reports are
  committed** so the caption can be checked rather than taken on trust:
  [`../results/phase7_yield_preview_arbiter.md`](../results/phase7_yield_preview_arbiter.md)
  and
  [`../results/phase7_yield_preview_mission.md`](../results/phase7_yield_preview_mission.md).
  **4 conflicts predicted, 3 resolved locally by the robots and 1 escalated**, AMR-2
  yielding for **1.0 s** and released on *conflict cleared* rather than on the 45 s
  fail-safe, **0 recovery behaviours during the hold**, the SafetyGate blocking on
  **0 of 21 held cycles**, and **both goals SUCCEEDED** (amr1 39.2 s, amr2 20.6 s,
  closest approach 1.915 m).
  **You cannot see the 1.0 s hold in the GIF** — at 6× it is two frames — so the clip is
  captioned as what it does show, which is both robots passing the one gap in turn.
  The canonical yield numbers in `../README.md` §5.7 come from
  [`../results/phase7_yield.md`](../results/phase7_yield.md), measured at the **default**
  window.

  **Two of the six runs also ended with AMR-1 stalled at the barrier** — halted by its
  own SafetyGate and never released, while AMR-2 finished. That is a real defect and it
  is written up in `../README.md` §10.9a rather than left out of this list.

## `archive/`

[`archive/run_reports/`](archive/run_reports/) holds the per-run reports from an earlier
round of recordings. **The clips they describe are no longer carried here** — they were
recorded before the chassis colours were split (both robots render blue in them, which
now contradicts every other image in this repository and the `body_color` entries in
`amr_description/config/fleet.yaml`), and the scenarios they showed have been
re-recorded above from the current build.

The reports are kept because they are measurements and one of them is worth reading:
`yield_protocol.md` records a hold that **ended on the 45 s fail-safe ceiling — a
deadlock** — and labels it as one rather than presenting it as a successful yield. A
fail-safe existing, firing, and being reported as a deadlock is worth more than a
re-roll that hides it.

**Nothing in `archive/` describes current behaviour.** For that, read `previews/`,
`verified/`, and `../results/`.

## Figures that live in `results/`, not here

The analysis plots are artifacts rather than illustrations, so they sit with the rest of
the evidence:

| Figure | What it is |
|---|---|
| [`../results/phase5_payload_trace.png`](../results/phase5_payload_trace.png) | The §3.1 deliverable — commanded velocity, acceleration and jerk, loaded against unloaded, for both robots |
| [`../results/phase1_map.png`](../results/phase1_map.png) | The cooperative SLAM map artifact |
| [`../results/phase2_pitch_gate.png`](../results/phase2_pitch_gate.png) | PitchGate truncation on the ramp, raw against validated |
| [`../results/phase1_nav_baseline.png`](../results/phase1_nav_baseline.png), [`../results/phase1_nav_actors.png`](../results/phase1_nav_actors.png) | Planned against executed track, without and with pedestrians |

## How these were captured

Each is one `ros2 launch` of the scenario named beside it, run with `headless:=false`
after `./scripts/clean_processes.sh`, and screen-grabbed at 25 fps. Camera follow is the
Gazebo GUI's own `/gui/follow` service (a *service* in `gz-sim` 8, not a topic), issued
repeatedly during the run because `CameraTracking` ignores a request for an entity that
has not spawned yet and the robots spawn on a stagger.

The two-window scenes use a recording-only RViz configuration — the same displays and
the same `fleet_map` Fixed Frame as the shipped
`src/amr_bringup/rviz/fleet_mapping.rviz`, with the dock panels removed so the map gets
the full half-width. That is a viewing preference and changes nothing about the
simulation. Every run was given its own `tag:=`, so none of these recordings overwrote a
committed artifact in `../results/`.
