# Demo media

Screen recordings of the simulator, captured live from the launches in
[`../docs/DEMO_RUNBOOK.md`](../docs/DEMO_RUNBOOK.md). The Gazebo camera is locked
onto the robot for the whole of each clip, so the vehicle stays in frame rather
than the scene being viewed from a fixed overhead pose.

**Every clip is a real run, and each one's own report is in
[`run_reports/`](run_reports/).** These are *not* the repository's canonical
evidence — that is [`../results/`](../results/), which is what
[`../README.md`](../README.md) §5 quotes. The reports here exist so that anything
visible in a video can be checked against the run that produced it, and so that
where a recorded run differs from the canonical one, the difference is on the
record rather than hidden.

| Clip | Length | Shows | Report |
|---|---|---|---|
| `safety_halt.mp4` | 0:47 | §3.3 — amr1 navigating the aisle with pedestrians; the SafetyGate halting it three times and releasing on hysteresis; goal still reached | [`safety_halt.md`](run_reports/safety_halt.md) |
| `safety_fail_closed.mp4` | 0:20 | §3.3 fail-closed — the SafetyGate is **SIGKILLed at 0.35 m/s** while upstream keeps commanding, and the robot stops anyway | [`safety_fail_closed.md`](run_reports/safety_fail_closed.md) |
| `yield_protocol.mp4` | 3:11 | §3.2b — both robots converge on one 3.0 m gap in a barrier; amr2 is escalated to a yield and holds while amr1 passes | [`yield_protocol.md`](run_reports/yield_protocol.md) · [mission](run_reports/yield_protocol_mission.md) |
| `yield_resolved_locally.mp4` | 1:48 | §3.2 — the **same scenario resolving without arbitration**: conflicts predicted, the predicted approach opens up, the arbiter stands down | [`yield_resolved_locally.md`](run_reports/yield_resolved_locally.md) · [mission](run_reports/yield_resolved_locally_mission.md) |

## What each recorded run measured

**`safety_halt.mp4`** — goal **SUCCEEDED**, **3 halts**, sensor-stamp-to-zero-command
**8.0 / 8.0 / 7.0 ms**, and **0 recovery behaviours during any halt**. Consistent with
the canonical run in `results/phase2_safety_suppressed.md` (mean 8.46 ms, p95 11.00 ms,
3 halts).

**`safety_fail_closed.mp4`** — the gate is killed with **SIGKILL**, so no shutdown
handler runs, and the robot travelled **0.189 m** before coming to rest 0.75 s later.
The canonical run recorded 0.196 m; the control arm without a plant-side watchdog
travelled **3.500 m and was still rolling** when its window closed
(`results/phase2_failclosed_no_watchdog.md`). Watch the aisle floor grid for the
distance.

**`yield_protocol.mp4`** — **3 conflicts predicted, all 3 escalated, amr2 yielded every
time**, and **0 recovery behaviours fired during any hold**. Both goals **SUCCEEDED**
(amr1 77.0 s / 12.19 m, amr2 80.2 s / 14.15 m). Two things to read honestly:

- This run was launched with **`time_window_s:=5.0`**, not the default 3.0 s. Three
  consecutive attempts at the defaults produced **zero** escalations — the local layer
  opened the gap every time — which is the non-determinism `../README.md` §10.9
  describes rather than a surprise. Widening the prediction window is the remedy the
  runbook already documents for exactly this. **The canonical yield numbers quoted in
  `../README.md` §5.7 come from `results/phase7_yield.md`, which ran at the default
  window.**
- **Yield 2 released on the 45 s fail-safe ceiling, not on "conflict cleared".** That
  is a **deadlock**, and the report labels it as one. The canonical run released both
  of its holds on *conflict cleared* and never reached the ceiling. A submission clip
  showing a deadlock is worth more than a re-roll that hides it: the fail-safe existing,
  firing, and being reported as a deadlock rather than as a successful yield is the
  behaviour worth seeing.

**`yield_resolved_locally.mp4`** — the same launch at default settings: conflicts were
predicted, the predicted closest approach opened up, the arbiter **declined to act**,
and both goals still succeeded. Its verdict reads **NOT EXERCISED**, not FAIL — a run in
which the local layer resolved everything is not a failed run. Together with the clip
above this is the **local-first ordering** as data: central arbitration is what happens
*after* the local layer has failed, not instead of it.

## Verified stills — what a working run looks like

`verified/` holds four frames taken from runs that were launched from the documented
command, watched in Gazebo and RViz, and confirmed to match what the documentation
says they should show. They are the reference an evaluator can hold their own run
against, and they are the images embedded in [`../HOW_TO_RUN.md`](../HOW_TO_RUN.md) §4.

| File | Shows |
|---|---|
| `verified/warehouse_both_robots.png` | The warehouse Gazebo should open to — rack rows either side, the 8° ramp and upper plateau beyond, and **both robots in the aisle** (AMR-2 amber, AMR-1 blue) |
| `verified/cooperative_mapping.png` | Demo A: Gazebo left, RViz right, one `/fleet_map` covering the aisle with the rack bays cut out — built by both robots into a single grid |
| `verified/rviz_fleet_map_config.png` | The same demo with RViz's **Displays panel open**, so the configuration is legible rather than asserted: **Fixed Frame `fleet_map`**, `Global Status: Ok`, topic **`/fleet_map`**, update topic `/fleet_map_updates`, resolution 0.05 m, **680 × 400**, origin −15 / −10. Both robots are in the Gazebo view and both appear as markers on the map |
| `verified/rviz_both_plans.png` | Both routes planned at once in the `fleet_map` frame, green for AMR-1 and cyan for AMR-2, `Global Status: Ok`. What happens *after* this is §9.1 of HOW_TO_RUN |
| `verified/safety_override.png` | Demo B: a pedestrian has walked into AMR-1's path and the robot has stopped short of them |

## Stills

| File | Shows |
|---|---|
| `stills/fleet_narrow_gap.png` | Both robots approaching the 3.0 m gap in the red barrier — the staged narrow-intersection conflict, with rack rows either side and the ramp and upper plateau beyond |
| `stills/pedestrian_encounter.png` | amr1 in the aisle with pedestrian traffic ahead of it, approaching the ramp |
| `stills/robot_ramp_approach.png` | amr1 at the ramp approach, showing the 8° ramp and the upper plateau |
| `stills/robot_aisle_chase.png` | Chase view of amr1 running the main aisle between the storage racks |

The analysis figures — the payload velocity/jerk plot, the SLAM map, the PitchGate
before/after, the navigation tracks — are in [`../results/`](../results/) and are the
artifacts `../README.md` §5 cites directly:

| Figure | What it is |
|---|---|
| `../results/phase5_payload_trace.png` | The §3.1 deliverable — commanded velocity, acceleration and jerk, loaded against unloaded, for both robots |
| `../results/phase1_map.png` | The cooperative SLAM map artifact |
| `../results/phase2_pitch_gate.png` | PitchGate truncation on the ramp, raw against validated |
| `../results/phase1_nav_baseline.png`, `../results/phase1_nav_actors.png` | Planned against executed track, without and with pedestrians |

## How these were captured

Each clip is one `ros2 launch` of the corresponding scenario from
[`../docs/DEMO_RUNBOOK.md`](../docs/DEMO_RUNBOOK.md), run with `headless:=false` after
`./scripts/clean_processes.sh`, and screen-grabbed at 25 fps. Camera follow is the
Gazebo GUI's own `/gui/follow` service (a *service* in `gz-sim` 8, not a topic), issued
repeatedly during the run because `CameraTracking` ignores a request for an entity that
has not spawned yet, and the robots spawn on a stagger.

The recordings used a stripped-down GUI layout — the 3D view, the clock and the
play/pause control, with the entity-tree and component-inspector docks omitted — so the
render view fills the frame. That is a local viewing preference and changes nothing
about the simulation.

Each run was given its own `tag:=`, so none of these recordings wrote over the
canonical artifacts in `../results/`.
