# AMR Fleet Navigation — Adaptive Navigation and Conflict-Aware Path Planning

A two-robot (scalable to N) autonomous mobile robot fleet for a logistics warehouse,
built on ROS 2 Jazzy, Gazebo Harmonic and Nav2. It implements cooperative SLAM with a
selective map-update policy feeding one unified `/fleet_map` that is the static layer
of both robots' global costmaps; payload-adaptive velocity and jerk smoothing chained
after the stock Nav2 smoother; conflict-aware local planning in which each robot's
local costmap consumes the *other* robot's predicted trajectory as time-decayed cost;
a central arbiter that enforces a yielding protocol only for conflicts the local layer
could not resolve; and a low-latency, fail-closed safety override implementing
`d_safe = k·v² + d_min`. Every robot difference — mass, geometry, velocity and jerk
limits, safety gain, yield priority — comes from one YAML file; there is no per-robot
branching anywhere in the codebase. **Every number in this README was produced by a
run whose artifact is named beside it in `results/`, and the limitations in §10 are
stated with the same evidence discipline as the results in §5.**

| | |
|---|---|
| **Stack** | ROS 2 Jazzy · Gazebo Harmonic (`gz-sim` 8.11.0) · Nav2 · `slam_toolbox` · Ubuntu 24.04 |
| **Languages** | Python (PEP 8, flake8 + black) · C++ (Google style) where measured latency justifies it |
| **Packages** | 9, all namespaced, all configuration-driven |
| **Tests** | 241 unit tests, pure functions, no simulator required |
| **Evidence** | 82 artifacts in [`results/`](results/) |

**Documentation map**

| Document | What it holds |
|---|---|
| **[`HOW_TO_RUN.md`](HOW_TO_RUN.md)** | **Start here.** Every command to type, in order, in plain language, with what each one does and what you should see |
| [`docs/DEMO_RUNBOOK.md`](docs/DEMO_RUNBOOK.md) | Twelve scenarios, each a self-contained block: purpose, requirement, exact command, whether RViz auto-starts, Fixed Frame, whether actors are on, expected visuals, expected result, failure modes |
| [`docs/ENGINEERING_NOTES.md`](docs/ENGINEERING_NOTES.md) | The eight design invariants the code cites by number |
| [`docs/ASSIGNMENT.pdf`](docs/ASSIGNMENT.pdf) | The source requirements |
| [`media/README.md`](media/README.md) | What each recorded clip shows, and where a recorded run differs from the canonical artifact |

**Status at a glance.** One line per assignment requirement, so the shape of the
submission is visible before any of the detail. `DEMONSTRATED` means the behaviour was
observed and measured, not merely built; `PARTIAL` and `NOT DEMONSTRATED` mean exactly
what they say. The evidence for every row is in [§4](#4-requirement--implementation--evidence),
and every shortfall is in [§10](#10-known-limitations).

| # | Requirement | Status |
|---|---|---|
| 1 | Heterogeneous AMR-1 / AMR-2 in a warehouse with racks, ramp, unknown areas, dynamic obstacles | **DEMONSTRATED** |
| 2 | Cooperative global mapping into one unified fleet map | **DEMONSTRATED** |
| 3 | Selective map updates | **DEMONSTRATED** |
| 4 | Concurrent navigation goals | **DEMONSTRATED** — both goals dispatched together, both robots arrive. An earlier revision reported this as PARTIAL; that is retracted, with the re-measurement in [§10](#10-known-limitations) |
| 5 | Ramp-aware global planning | **PARTIAL** — mask mechanism implemented and loaded; graded cost and the route A/B **not demonstrated** |
| 6 | Payload-aware acceleration / jerk control | **DEMONSTRATED** for the payload response; jerk ceiling **not certified** |
| 7 | MAPF / peer trajectory in the local planner | **IMPLEMENTED**, cost injection measured against a control; autonomous mutual deviation **NOT DEMONSTRATED** |
| 8 | Predefined traffic-control yielding | **DEMONSTRATED**; the escalation trigger is non-deterministic |
| 9 | Low-level safety override, speed-dependent distance | **DEMONSTRATED** |
| 10 | BSP / sensor validation, IMU plausibility | **DEMONSTRATED** |
| 11 | Configuration-driven scalability to ten or more robots | **IMPLEMENTED** by construction and test; **no 10-robot run was performed** |
| 12 | Clean ROS 2 workspace, code quality | **DEMONSTRATED** |
| 13 | Refactoring — plan and implement part of it | **DEMONSTRATED** |
| 14 | README and run instructions | **DEMONSTRATED** |
| 15 | Screenshare demonstration | **DEMONSTRATED**, delivered separately |

**What it looks like running.** Each preview is a short GIF cut from a live run of the
command named beside it, recorded from this commit. The same clips sit next to their
commands in [`HOW_TO_RUN.md`](HOW_TO_RUN.md), which is where an evaluator should start.

| Preview | Shows | Command |
|---|---|---|
| [`media/previews/cooperative_mapping.gif`](media/previews/cooperative_mapping.gif) | Both robots exploring at once and `/fleet_map` filling in from both ends, rack bays cut out as black | `fleet_survey.launch.py` |
| [`media/previews/concurrent_goals.gif`](media/previews/concurrent_goals.gif) | Both plans drawn together — green AMR-1, cyan AMR-2 — and **both robots arriving** | `phase3_fleet_goals.launch.py` |
| [`media/previews/safety_override.gif`](media/previews/safety_override.gif) | A pedestrian walks into AMR-1's path; the gate halts the robot short of them and releases on hysteresis | `phase2_safety_run.launch.py` |
| [`media/previews/yield_protocol.gif`](media/previews/yield_protocol.gif) | Both robots converge on one 3.0 m gap in a barrier and pass through it one at a time | `phase7_yield.launch.py` |

Full-size stills of the same runs are in [`media/verified/`](media/verified/), and
[`media/README.md`](media/README.md) names the run behind every file — including where
a recorded run's numbers differ from the canonical artifact in `results/`.

**The submission screenshare is delivered separately with the submission and is
deliberately not in this repository.** It runs a little over three minutes — the
warehouse and both robots, cooperative mapping and selective updates live in RViz, both
robots reaching concurrent goals, payload-adaptive motion, the safety override stopping
the robot on a pedestrian, the conflict and the yield, sensor validation, and the
architecture with the limitations of §10 stated on screen. Every scene is footage of a
run of a command in [`HOW_TO_RUN.md`](HOW_TO_RUN.md) on this commit. `videos/` is kept
as the place to record into, and `*.mp4` / `*.mkv` / `*.webm` are git-ignored with **no
exception** — the previews above are GIFs precisely so that GitHub plays them inline
instead of asking an evaluator to download a file.

---

## 1. Scope

The warehouse is a two-level structure: a lower aisle with static storage racks and
walking pedestrians, a ramp at 8°, and an upper plateau. Two heterogeneous robots run
in it — **AMR-1** (mapper/lead, 30 kg chassis + 60 kg payload, 0.60 m/s, low
acceleration) and **AMR-2** (scout/follower, 18 kg + 5 kg, 1.00 m/s, high
acceleration). Each robot runs its own namespaced `slam_toolbox` and its own six Nav2
servers; the fleet shares one map, one clock, one TF tree rooted at `fleet_map`, and
one arbiter that is consulted only when the per-robot conflict layer has failed.

The strategy throughout is **extend Nav2, do not duplicate it**. Custom code exists
only where the assignment demands custom work, and each custom component addresses a
real failure mode rather than ticking a requirement box. Where a planned deliverable
did not survive contact with the simulator, it is recorded as a negative result with
its measurement rather than quietly dropped — §10 lists all of them.

---

## 2. Quick Start

Copy-pasteable from a fresh clone on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo
Harmonic installed.

```bash
# 1. Clone. The repository root IS the colcon workspace; packages live in src/.
git clone https://github.com/GauthamCodes/amr-fleet-nav.git
cd amr-fleet-nav

# 2. Dependencies.
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. Build. Expect: "9 packages finished", 0 errors.
./ws.sh colcon build --symlink-install

# 4. Unit tests. Expect: 241 passed.
./ws.sh python3 -m pytest tests/ -q

# 5. Pre-flight — before EVERY launch, no exceptions. Expect the CLEAN banner.
./scripts/clean_processes.sh

# 6. THE FULL FLEET: warehouse + both robots + BSP + SLAM + fleet map +
#    both Nav2 stacks + motion chain + safety gates + trajectory layer + arbiter.
./ws.sh ros2 launch amr_bringup fleet_nav.launch.py headless:=false
```

That is the whole system. `headless:=true` (the default) runs it without the Gazebo
GUI, which is roughly 50–80 % faster.

**To see the cooperative map being built** — the assignment's first evaluation
criterion, and the one thing that is only legible in RViz — run the survey. Gazebo
and RViz both come up from this one command, already configured:

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup fleet_survey.launch.py
```

Both robots drive the same circuit half a lap apart and `/fleet_map` fills in from
both ends of the aisle at once. Toggle *amr1 own map* and *amr2 own map* in the
Displays panel to show the composite really is a fusion of two maps.

**To see concurrent goals dispatched to both robots**, use the fleet with a mission
attached — it dispatches both goals in one pass, writes its report into `results/`
and shuts itself down. Both plans appear in RViz at the same instant, green for AMR-1
and cyan for AMR-2, and **both robots arrive** — amr2 first, because `fleet.yaml`
makes it the faster chassis:

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    headless:=false rviz:=true with_actors:=true tag:=demo
```

`with_actors:=true` puts the walking pedestrians in the world (see
[§2.1](#21-dynamic-obstacles)); `tag:=demo` keeps the run from overwriting the
committed Phase 3 evidence, which was measured without them.

### Two environment rules that are not optional

1. **Every ROS or Gazebo command goes through `./ws.sh` — including `rviz2`.** The
   wrapper sources the workspace and *appends* to `CYCLONEDDS_URI`, raising
   CycloneDDS's `MaxAutoParticipantIndex` from its default of 9 to **400**. The count
   that matters is **participants, not nodes**: a two-robot `fleet_nav` is **23 nodes
   per robot namespace plus 17 shared, 63 unique names measured on this build**, and
   `ros2 node list` can print more entries than that because several nodes advertise
   from more than one participant — so the raw total moves between runs and is not a
   pass/fail number. Past the ceiling, node creation throws
   `Failed to find a free participant index for domain 0` and whichever servers
   started last die while the rest of the fleet comes up perfectly — which reads
   exactly like a namespacing bug in one robot and is a host-wide limit. **This was
   set to 120 and that was not enough**: `phase7_yield` adds RViz, the arbiter, the
   mission node and both trajectory predictors and crossed it, killing nine nodes.
   See [`docs/DEMO_RUNBOOK.md`](docs/DEMO_RUNBOOK.md) §0 and the comment in
   `src/amr_bringup/config/cyclonedds.xml`.

2. **Run `./scripts/clean_processes.sh` before every launch.** It kills the simulator
   and the ROS graph, then prints the surviving process table and exits non-zero if
   anything is left. This is not ceremony: a single leaked `drive_watchdog` from an
   earlier launch once produced a finding that was written down, believed for a
   session, and then retracted (§10). *A process-hygiene failure is a measurement
   failure.*

### 2.1 Dynamic obstacles

The warehouse contains **three scripted pedestrians** — one walking the main aisle,
one crossing the ramp approach into the robot's path, one on the upper plateau. They
are verified to reach the navigation stack: `results/smoke1_actor_visibility.md`
records them raycast by the LiDAR and marked **254 LETHAL in the Nav2 costmap in
100 % of frames**.

They are controlled by `with_actors`, and **the default differs per launch on
purpose** — a walking pedestrian is an uncontrolled variable, so the measurement runs
switch them off and the demonstration runs switch them on. `fleet_nav.launch.py` has
them **on**; `phase2_safety_run.launch.py` has them **hard-wired on**, because the
pedestrian *is* that demo. `phase3_fleet_goals.launch.py` and `phase6_conflict.launch.py`
default them off and take `with_actors:=true`. The full per-launch table is in
[`docs/DEMO_RUNBOOK.md`](docs/DEMO_RUNBOOK.md) §2.

Gazebo actors are **not physics entities** — they cannot generate contacts, so
clearance rather than collision-avoidance is the honest quantity throughout (§10.11).

### 2.2 Visualisation

RViz ships configured. `rviz:=true` starts it inside the launch, already loaded with
`src/amr_bringup/rviz/fleet_mapping.rviz`:

```bash
./ws.sh ros2 launch amr_bringup fleet_survey.launch.py                # RViz on by default
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py rviz:=true headless:=false
```

`rviz:=true` is available on `fleet_nav`, `fleet_survey`, `phase3_fleet_goals`,
`phase6_conflict` and `phase7_yield` — every two-robot demo. Starting it from the
launch is preferred because a launched RViz cannot be the one process in the graph
that missed `./ws.sh`.

The config shows `/fleet_map`, both robot models (**amr1 blue, amr2 amber**, from
`body_color` in `fleet.yaml`), both scans, both `/plan` paths, and the key TF frames
only — with the own-maps, global costmaps, predicted trajectories and local costmaps
one click away as toggles.

**Fixed Frame is `fleet_map`** for anything with two robots, `amr1/odom` for the
single-robot demos. **There is no bare `map` frame in this system** — every robot's
SLAM frame is namespaced `amrN/map` and the shared root is `fleet_map`. A bare
`rviz2` with no config comes up on the stock Fixed Frame `map` and is therefore blank
with *"Frame [map] does not exist"*; use the saved config, or:

```bash
./ws.sh rviz2 -d src/amr_bringup/rviz/fleet_mapping.rviz --ros-args -p use_sim_time:=true
```

Nothing appears for the first ~25 s of any launch — that is the staged bringup, not a
fault. `FleetMapNode` publishes `fleet_map` at t ≈ 16 s.

---

## 3. Architecture

### 3.1 The command chain — serial, and fail-closed at the last link

Every velocity command for every robot passes through this chain, in this order. It is
a **chain, not a mux tree**: each link has exactly one input and one output.

```
  Nav2 controller_server / behavior_server
        │  cmd_vel_nav
        ▼
  ┌──────────────────────────┐
  │ nav2_velocity_smoother   │  STOCK Nav2. Velocity + acceleration clamp.
  └──────────────────────────┘
        │  cmd_vel_smoothed
        ▼
  ┌──────────────────────────┐
  │ PayloadJerkAdapter       │  OURS (amr_motion). Adds ONLY jerk limiting and
  └──────────────────────────┘  payload scaling — never raises a limit.
        │  cmd_vel_shaped
        ▼
  ┌──────────────────────────┐  ◄──── cmd_vel_yield   priority 150
  │ priority mux             │         (TrafficControlNode, zero twist)
  └──────────────────────────┘  ◄──── cmd_vel_shaped  priority 100
        │  cmd_vel_mux
        ▼
  ┌══════════════════════════┐
  ║ SafetyGate  (C++)        ║  OURS. SERIAL, LAST, FAIL-CLOSED.
  ╚══════════════════════════╝  d_safe = k·v² + d_min, v from ODOMETRY.
        │  cmd_vel
        ▼
  drive_watchdog  ──►  cmd_vel_plant  ──►  Gazebo DiffDrive
```

**Why the gate is serial and never a mux input.** A mux selects the highest-priority
*live* input, so a dead safety node lets ordinary navigation straight through — it
fails **open**. That is the wrong failure mode for the one component whose entire job
is to stop the robot. As a serial last link, nothing else publishes the topic the
plant listens to, so if the gate dies nothing moves. Measured under SIGKILL:
**0.196 m of travel with the plant-side watchdog present, against 3.500 m and still
rolling at 0.350 m/s without it** (`results/phase2_failclosed_*.md`).

**Gating without notification is a bug.** Any component that zeroes `cmd_vel` — the
SafetyGate on a halt, the arbiter on a yield — also raises
`controller_server.progress_checker.movement_time_allowance` via a dynamic parameter,
and restores it on release. Without that, Nav2's progress checker declares the robot
stuck *while it is being deliberately held* and dispatches recovery behaviours (spin,
back up) into a vehicle that must not move. Measured in §5.

### 3.2 The sensor path — nothing reaches Nav2 unvalidated

```
  Gazebo  scan ─┐
          imu ──┼──►  SensorBSP  ─┬─ ImuValidator     angular-rate plausibility
   camera/image_raw │   (amr_bsp)  ├─ LidarValidator   NaN / stamp / range
                    │              ├─ CameraValidator  stamp / resolution
                    │              └─ PitchGate        ramp ground-return truncation
                    │                      │
                    └──────────────────────┴──►  validated/{scan,imu,camera/image_raw}
                                                        │
                                     ┌──────────────────┴──────────────────┐
                                     ▼                                     ▼
                              slam_toolbox                        Nav2 costmaps
```

No node in the navigation stack subscribes to a raw sensor topic. The contract is
written down in exactly one place, `amr_bsp/topics.py`, and every consumer imports it.
**Original header stamps are preserved on republish** — rewriting them silently breaks
TF and SLAM.

### 3.3 The fleet map and the global costmaps

The merged map is not a fleet-view convenience; it is what both robots plan against.

```
  amr1/slam_toolbox ──► /amr1/map ──┐
                                     ├──►  FleetMapNode  ──►  /fleet_map
  amr2/slam_toolbox ──► /amr2/map ──┘       (selective            │
                                             update policy)       │  RELIABLE
                                                 │                │  TRANSIENT_LOCAL
                                                 │                │  KEEP_LAST(1)
                                                 │                │
                     TF: fleet_map ──► amrN/map  │     ┌──────────┴──────────┐
                     (static, from spawn poses)  │     ▼                     ▼
                                                    amr1 global_costmap   amr2 global_costmap
                                                    STATIC LAYER          STATIC LAYER
                                                    global_frame:         global_frame:
                                                      fleet_map             fleet_map
```

`/fleet_map` has **2 matched subscribers — both global costmaps** — and both report
`global_frame: fleet_map`. Verifiable live:

```bash
./ws.sh ros2 topic info -v /fleet_map | head -20
./ws.sh ros2 param get /amr1/global_costmap/global_costmap global_frame   # fleet_map
./ws.sh ros2 param get /amr2/global_costmap/global_costmap global_frame   # fleet_map
```

Three distinct frames per robot: `fleet_map` for global planning, `amrN/odom` for the
local costmap, `amrN/map` still owned by that robot's `slam_toolbox`.

```
fleet_map
   ├── amr1/map ── amr1/odom ── amr1/base_footprint ── amr1/base_link ── amr1/{laser,imu_link}
   └── amr2/map ── amr2/odom ── amr2/base_footprint ── amr2/base_link ── amr2/{laser,imu_link}
```

### 3.4 Where the MAPF requirement lives

```
  amr1 TrajectoryPredictor ──► /amr1/predicted_trajectory ──┐
                                (nav_msgs/Path in fleet_map;  │
                                 each pose's stamp = the time  │
                                 that pose is REACHED)         ▼
                                                    amr2 LOCAL costmap
                                                    └─ FleetTrajectoryLayer  (C++ pluginlib)
                                                       deposits max_cost·exp(−Δt/τ)
                                                       combined with std::max

  ... and symmetrically, amr2's prediction into amr1's local costmap.

  TrafficControlNode (central) — consulted ONLY for conflicts the layer did not open up.
```

No central node is in the local planning loop. `TrafficControlNode` handles escalation
only, and refuses to act while the predicted closest approach is still opening.

### 3.5 Packages

| Package | Contents | Lang |
|---|---|---|
| `amr_description` | One parameterised `amr.urdf.xacro`; `config/fleet.yaml`; `fleet_config.py` — the single interpreter of the robot list | Python |
| `amr_gazebo` | Warehouse world (xacro-rendered), actors, racks, ramp; spawn and world-singleton actions | Python |
| `amr_navigation` | Nav2 + `slam_toolbox` config templates rendered per robot by `params.py`; costmap filter launch | Python |
| `amr_costmap_plugins` | `FleetTrajectoryLayer` (costmap plugin) | C++ |
| `amr_fleet_control` | `FleetMapNode`, `TrajectoryPredictor`, `TrafficControlNode`, mission and probe nodes | Python |
| `amr_motion` | `PayloadJerkAdapter` | Python |
| `amr_safety` | `SafetyGate`, priority mux, `twist_mux.yaml`, safety model | C++ / Python |
| `amr_bsp` | `SensorBSP` base + IMU / LiDAR / Camera validators + `PitchGate`; `topics.py` | Python |
| `amr_bringup` | System composition only — no configuration | Python |

---

## 4. Requirement → Implementation → Evidence

The highest-value section. One row per assignment requirement. **Status is written
from what was measured, not from what was built** — three rows below say a claim is
*not* made, and §10 gives each one its evidence.

| § | Requirement | Implementation | Evidence | Status |
|---|---|---|---|---|
| **1** | Heterogeneous fleet (AMR-1 mapper/lead higher payload; AMR-2 scout/follower higher acceleration) in a multi-level Gazebo warehouse with racks, ramp and dynamic obstacles | `amr_description/config/fleet.yaml` — one typed robot list; one shared `amr.urdf.xacro`; `amr_gazebo` warehouse with 8° ramp, two plateaus, rack rows, walking actors | `results/smoke1_actor_visibility.md`, `results/smoke2_ramp_phantom_return.md`, `results/phase1_map.png` | **Verified.** Actors raycast by the LiDAR and reach **254 LETHAL in the Nav2 costmap in 100 % of frames**; ramp max pitch **8.001°**; amr1 90.0 kg / 0.60 m/s vs amr2 23.0 kg / 1.00 m/s, both from the one file |
| **2.1a** | Cooperative global SLAM & map fusion — both robots contribute to a **single unified** occupancy grid | `amr_fleet_control/fleet_map_node.py` composites both `slam_toolbox` maps into `/fleet_map`; wired as the **static layer of both global costmaps**, both planning in `global_frame: fleet_map` | `results/phase3_concurrent_goals.md`; DEMO_RUNBOOK scenario 1 (live in RViz) | **Verified.** 63 unique nodes (23 per robot + 17 shared), every lifecycle node `active`; `/fleet_map` 680 × 400 @ 0.05 m origin (−15, −10), TRANSIENT_LOCAL, **2 matched subscribers = both global costmaps**. Re-verified live in RViz: one `/fleet_map` grows from both ends at once and **both robots appear in the `ACCEPT` stream of the same run** — the per-robot split is in `results/phase3_selective_updates.md` (amr1 13 accepted, amr2 10) |
| **2.1b** | **Selective mapping** — prioritise unexplored boundaries, reduce update frequency for repeatedly traversed areas | Scored policy in `fleet_map_node.py`: `w_f·frontier + w_c·change + w_r·recency − w_v·revisit`; below threshold the merge *and* the composite work are skipped | `results/phase3_selective_updates.md` + `.csv` | **Verified.** **41 candidates scored, 23 accepted, 18 deferred (43.9 %)**, measured while both robots were exploring. Per robot: amr1 13/8, amr2 10/10 |
| **2.2a** | Adaptive global planner — **concurrent goals** for both robots | `amr_fleet_control/fleet_mission.py` dispatches both goals in one pass; each robot's own Nav2 stack plans in the fleet frame | `results/phase3_concurrent_goals_recheck.md` (six consecutive runs on this commit), `results/phase3_concurrent_goals.md` | **Verified.** Both goals dispatched in one pass, both accepted, both plans published in `fleet_map`, and **both robots arrive**: amr1 **18.6–18.9 s** to a 0.009–0.029 m final error, amr2 **11.3 s** every run, closest approach **3.000 m** over 2 163 samples, 0 replans. amr2 arrives first because `fleet.yaml` gives it `max_vel_x` 1.00 against 0.60 — no code distinguishes them. An earlier revision recorded this row as *Partial* on the strength of nine failing runs; those were measured on a contaminated process table and the finding is **retracted** in §10 |
| **2.2b** | **Ramp/slope planning** — custom cost function *or tuned configuration* that costs sloped surfaces, minimising their use unless they are the only viable path | Nav2 `KeepoutFilter` costmap-filter mask over the ramp footprint, generated (not hand-drawn) by `amr_navigation/ramp_mask.py`; loads into both global costmaps via the `filters:` list | `results/phase3_ramp_cost_graded.md` (graded arm, mask value 60) against `results/phase3_concurrent_goals.md` (null-mask arm) | **Partial — plumbing verified, graded cost NOT confirmed.** Both costmaps log `Received filter mask`; the null mask is proven to contribute nothing (**minimum cost over 272 000 known cells = 0**). But at mask value 60 the cost over the 3 550-cell ramp footprint is **0..100 — identical to the null run**. Undiagnosed; hypothesis in §10. The two-route A/B is unstageable in this world (§10) |
| **3.1** | Dynamic velocity and motion smoothing — acceleration and jerk limited by dynamic state and payload; the heavier AMR-1 must have lower acceleration limits than AMR-2 | Stock `nav2_velocity_smoother` **chained into** `amr_motion/payload_jerk_adapter.py`; per-robot limits from `fleet.yaml`, scaled down by payload state, never up | `results/phase5_payload_trace.md`, `.csv`, **`.png`** | **Verified for the payload ratio; jerk ceiling NOT certified.** amr1's peak commanded acceleration falls **×0.38** loaded against amr2's **×0.85**, and **no code distinguishes them**. Peak commanded velocity exactly 0.500 in all four cases. The published stream measures up to ~1.9× the configured jerk bound (§10) |
| **3.2a** | **MAPF element** — the local planner for **each robot** consumes the **projected trajectory of the other robot** | `amr_fleet_control/trajectory_predictor.py` publishes each robot's projected path; `amr_costmap_plugins` `FleetTrajectoryLayer` (C++ pluginlib) deposits `max_cost·exp(−Δt/τ)` into the **other** robot's LOCAL costmap, combined with `std::max` | `results/phase6_cost_injection_layer_on.md` vs `..._off.md` (+ `.csv`) | **Mechanism verified against a control; autonomous mutual deviation NOT claimed.** At the peer's *predicted* cell 2 s ahead: **50/50 samples cost > 0 (100 %) with the layer, 0/59 (0 %) without**; median cost 145, max 240, decay model predicts 125.9. RegulatedPurePursuit paces against that cost rather than deviating laterally (§10) |
| **3.2b** | Yielding protocol — Traffic Control Node enforces a **pre-defined** yield (lighter AMR-2 yields to heavier AMR-1) by commanding a temporary controlled stop | `amr_fleet_control/traffic_control.py`; yield = zero twist on the **priority-150** `cmd_vel_yield` mux channel; release by *ceasing to publish* (mux timeout), so a dead arbiter frees the robot rather than pinning it | `results/phase7_yield.md`, `phase7_yield_mission.md`, `phase7_yield_control.md` | **Verified.** Two escalations, **amr2 yielded both times**, held **1.0 s and 15.2 s**, both released on *conflict cleared* (never the 45 s fail-safe). **0 recovery behaviours during either hold**; SafetyGate blocking on **0 of 326 held cycles**, so the stop was the arbiter's alone. **Both goals SUCCEEDED.** Priority derived from `fleet.yaml` mass, not from a robot name |
| **3.3** | Safety system override — dedicated node monitoring local obstacle detection; below `d_safe = k·v² + d_min` issue a low-level, high-priority **immediate halt that overrides** the navigation stack | `amr_safety` `SafetyGate` (C++), serial last link, fail-closed, `v` from **odometry** not command, per-robot payload-scaled `k`, hysteresis on release | `results/phase2_safety_suppressed.md`, `phase2_stopping_distance.md`, `phase2_failclosed_*.md` | **Verified.** Goal SUCCEEDED with **3 halts and 0 commands leaked past the latch**. Sensor stamp → zero command published: **mean 8.46 ms / p95 11.00 ms / max 12.00 ms** (in-node compute 151/226/711 µs). Stopping sweep at four speeds. Fail-closed under SIGKILL: **0.196 m vs 3.500 m** |
| **4.1** | Validated data consumption (BSP instead of a HAL) — nav stack consumes sensor data only after validation; **log a warning when IMU angular velocity exceeds a plausible limit** | `amr_bsp`: `SensorBSP` base + `ImuValidator`, `LidarValidator`, `CameraValidator`, plus `PitchGate` doing functional work (ramp ground-return truncation). Original header stamps preserved | `results/phase2_imu_injection.md`, `phase2_pitch_gate.md` + `.png` + `.csv`, `phase2_camera.md` | **Verified.** **500 implausible samples injected at 50 rad/s against a 4.0 rad/s bound → 500 rejected, 0 reached `validated/imu`**, while **3 194 healthy samples were accepted** around the window. Stamp preservation: **903 raw/validated pairs matched by exact stamp, 0 restamped, max difference 0 ns**. PitchGate removed the 2.769 m phantom and **kept the 2.732 m real rack** |
| **4.2** | Code quality and scalability — cleanly namespaced, reusable class structure, fleet expandable to **ten or more robots by changing a minimal number of configuration parameters** | Every component is a namespaced, config-driven class. `fleet.yaml` is the only place a robot difference is expressed; launch files loop over it. `cyclonedds.xml` raises the participant ceiling to 400 — far above the ten-robot case | §8 below; `tests/test_fleet_parameterization.py`, `test_fleet_frames.py`, `test_spawn_poses_are_clear.py` | **Verified by construction and by test; a 10-robot run was not performed.** `grep -rn "amr1" src/` finds no behavioural branch — **zero `if robot ==` in the codebase**. Adding `amr3` is one YAML block and **no launch code change** (§8) |
| **5.1** | Environment setup and build — complete documented README, proper colcon workspace, clean build, screenshare video | This README; repository root **is** the colcon workspace; `ws.sh` pins the environment; `scripts/clean_processes.sh` | `./ws.sh colcon build --symlink-install` → **9 packages, 0 errors**; [`HOW_TO_RUN.md`](HOW_TO_RUN.md) with a preview GIF beside every command; [`docs/DEMO_RUNBOOK.md`](docs/DEMO_RUNBOOK.md) | **Verified.** Build clean from scratch; 241 tests pass; every demo has a single self-terminating launch command |
| **5.2** | Code refactoring — identify one area of the standard ROS navigation / Gazebo launch files, propose a refactoring plan, **implement a small part of it** | Monolithic single-robot bringup split into world singletons (`amr_gazebo.spawn.world_actions`) + a reusable per-robot `robot_stack.launch.py` + two thin compositions | §9 below (before/after tree); `src/amr_bringup/launch/` | **Verified — implemented, not just proposed.** The 180-line monolith became a 222-line reusable stack included by both a 179-line single-robot and a 277-line fleet composition; the fleet's only fleet-aware code is one `for` loop |

### A note on the assignment's own inconsistency

The assignment body (§4.1) specifies **LiDAR + IMU** validation; the evaluation table
asks for **IMU + Camera** and calls it a HAL class. All three validators are
implemented and unit-tested, with IMU angular-rate plausibility as the primary
demonstrated check. The camera is validated (`results/phase2_camera.md`, 369 frames)
but **defaulted off, because nothing downstream consumes it** — that is a cost
decision reversible by one xacro argument, not a defect workaround. An earlier claim
that the camera stalled the drive was measured, believed, and then **retracted** (§10).

---

## 5. Results

Every figure below is in a file in `results/`. Nothing is rounded in the robot's
favour.

### 5.1 Safety override — latency and stopping distance

`results/phase2_safety_suppressed.md`, `results/phase2_stopping_distance.md`

| | mean | p95 | max |
|---|---|---|---|
| **sensor stamp → zero command published** | **8.46 ms** | **11.00 ms** | **12.00 ms** |
| in-node compute (steady clock) | 151 µs | 226 µs | 711 µs |

Goal **SUCCEEDED**, **3 halts**, and **0 commands left the gate while latched** —
counted *inside* the gate, comparing the twist about to be published against the latch
at that instant.

| cmd m/s | v at halt | `d_safe` | clearance at halt | settled |
|---|---|---|---|---|
| 0.15 | 0.130 | 0.342 | 0.334 | 0.331 |
| 0.30 | 0.277 | 0.469 | 0.456 | 0.401 |
| 0.45 | 0.426 | 0.680 | 0.653 | 0.402 |
| 0.60 | 0.577 | 0.975 | 0.928 | 0.384 |

`k_model` **1.8750 s²/m** against `k_measured` **1.0208 s²/m** — a factor of 1.84, and
both are quoted because they measure different things. Gazebo's DiffDrive applies
`max_decel_x` as a *kinematic* limit, so the simulated robot brakes as if unloaded;
`k_model` sizes the envelope for a 90 kg vehicle that cannot ignore its 60 kg payload.
`k = 1/(2·a_eff)`, `a_eff = |max_decel_x|·m_base/(m_base + m_payload)`, `[k] = s²/m`.

This is a **low-latency safety override — not hard real-time.** There is no RT kernel
and no scheduling guarantee. The end-to-end figure is quantised by Gazebo's `/clock`
step under `use_sim_time`; the compute figure is not.

### 5.2 Fail-closed — SIGKILL the gate mid-motion

`results/phase2_failclosed_watchdog.md`, `results/phase2_failclosed_no_watchdog.md`

| | distance after the kill | outcome |
|---|---|---|
| plant-side watchdog **present** | **0.196 m** | at rest in 0.75 s |
| watchdog **absent** (control) | **3.500 m** | **still rolling at 0.350 m/s** when the 10 s window closed |

SIGKILL, not SIGTERM — no shutdown handler runs, and upstream keeps commanding
throughout. `gz-sim` 8.11.0's DiffDrive has **no command timeout**, so being the only
publisher of `cmd_vel` stops new commands but not the latched one; the watchdog models
the motor controller that does have one. Real hardware stops when its command stream
stops; the simulator does not, and the difference is stated rather than glossed.

### 5.3 Sensor validation — IMU rejection and stamp preservation

`results/phase2_imu_injection.md`, `results/phase2_pitch_gate.md`

- **500 implausible samples injected** at 50 rad/s on `w_z` against a 4.0 rad/s bound
  → **500 rejected, 0 reached `validated/imu`** (peak |ω| there: 0.001 rad/s), while
  **3 194 healthy samples were accepted** around the window. *Both halves matter — a
  validator that rejected everything would produce an equally impressive rejection
  count.*
- Injection is done by a **separate process**, so the BSP ships with no
  fault-simulation code in it.
- **903 raw/validated pairs matched by exact header stamp, 0 restamped, max difference
  0 ns.** Pairing by exact stamp means a restamp shows up as *zero matched pairs*
  rather than as plausible numbers computed against mismatched data.
- PitchGate at 8.001° nose-up truncated **120 of 360 beams**; **closest return removed
  2.769 m** (the phantom), **closest return kept 2.732 m** (the real rack, untouched).
  Over the run: 24 833 beams truncated, **7.65 %** of all beams seen, with **0** cuts
  pointing uphill, **0** inside their own gate, and **0** inside the braking envelope.

### 5.4 Selective map updates

`results/phase3_selective_updates.md`

| | |
|---|---|
| candidates scored | **41** |
| accepted (merged and published) | **23** |
| **deferred** | **18 — 43.9 %** |
| per robot | amr1 13/8 (38.1 % deferred), amr2 10/10 (50.0 %) |
| composites performed | 17 |
| mean composite + publish | 0.55 ms |
| fleet map at end | 15.6 % known, 1 984 occupied cells |

Measured **while both robots were exploring** — a stationary fleet defers everything
and would make the policy look effective while proving nothing.

**The honest headline is the deferred share, not the milliseconds.** Compositing is
numpy slice arithmetic on a fixed grid and was never the expensive part. What deferral
bounds is how often a 680 × 400 grid is serialised to two global costmaps that each
reprocess it — work inside the Nav2 processes that this report does not measure.

### 5.5 MAPF — one robot's intention as cost in another's local costmap

`results/phase6_cost_injection_layer_on.md` vs `results/phase6_cost_injection_layer_off.md`

| | layer **ON** | layer **OFF** (control) |
|---|---|---|
| samples with the peer's predicted cell inside the window | 50 | 59 |
| **samples with cost > 0** | **50 (100 %)** | **0 (0 %)** |
| median cost at that cell | **145** | 0 |
| max cost | 240 | 0 |
| cost the decay model predicts | 125.9 | 127.3 |

The probe samples where amr1 is **predicted to be 2 s from now**, *not* where it is.
That distinction is the measurement: amr1 is a physical object amr2's LiDAR marks and
inflates regardless, so sampling the peer's *current* cell would measure the obstacle
layer and report it as MAPF. Cost is capped below `INSCRIBED_INFLATED_OBSTACLE` (253)
by construction — a peer's *intention* must never read as a collision to a footprint
checker — and `tests/test_trajectory_conflict.py` asserts the shipped YAML respects
the cap.

### 5.6 Payload-adaptive velocity and jerk

`results/phase5_payload_trace.md` · plot: **`results/phase5_payload_trace.png`**

| robot | payload | peak v cmd | peak a cmd | peak jerk cmd |
|---|---|---|---|---|
| amr1 | unloaded | 0.500 | 0.725 | 1.784 |
| amr1 | **loaded** | 0.500 | **0.272** | 0.634 |
| amr2 | unloaded | 0.500 | 1.112 | 4.722 |
| amr2 | **loaded** | 0.500 | **0.940** | 2.742 |

**amr1's peak commanded acceleration falls ×0.38 loaded, against amr2's ×0.85** — 60 kg
on a 30 kg chassis is a far larger perturbation than 5 kg on 18 kg — **and no code
distinguishes them.** Peak commanded velocity is exactly 0.500 in all four cases,
i.e. the adapter restricts without overshooting.

The jerk column exceeds the configured bound (1.000 / 0.333 / 2.500 / 1.957 m/s³) by
up to ~1.9×; §10 explains exactly what that column does and does not measure.

### 5.7 Yield protocol

`results/phase7_yield.md`, `results/phase7_yield_mission.md`

| | yield 1 | yield 2 |
|---|---|---|
| escalated after | 2.0 s of unresolved conflict | 2.0 s |
| separation at escalation | 1.53 m | 1.76 m |
| predicted separation **gain** over the conflict's life | **−0.14 m** | **+0.04 m** |
| **held** | **1.0 s** | **15.2 s** |
| release condition | conflict cleared (2.43 m) | conflict cleared (2.46 m) |
| **recovery behaviours during the hold** | **0** | **0** |
| SafetyGate blocking during the hold | 0 of 21 cycles | 0 of 305 cycles |

- **Both goals SUCCEEDED** — amr1 40.0 s / 12.72 m driven, amr2 42.9 s / 13.62 m,
  final errors 0.032 m and 0.151 m. Closest actual approach 0.926 m over 5 091
  ground-truth samples.
- `movement_time_allowance` **read back** from `controller_server` at 1 000 000 s on
  entry and 10 s after release — read back, not assumed.
- **The gain column is the escalation argument.** The arbiter refuses to act while the
  predicted closest approach is opening by more than 0.15 m, because that is the local
  layer resolving the conflict on its own. Here it opened by −0.14 m and +0.04 m: the
  local layer had first refusal for 2.0 s in both cases and did not open the gap.
- A second run of the same scenario (`results/phase7_yield_control.md`) shows the
  ordering in **both** directions: **3 conflicts predicted, 2 resolved locally** (the
  approach opened by +0.03 m and +0.05 m and the arbiter stood down) and **1
  escalated**. That nonzero "resolved locally" count is the local-first ordering as
  data rather than as an assertion.

### 5.8 Recovery suppression — measured necessity

`results/phase2_recovery_ab_suppressed.md` vs `results/phase2_recovery_ab_control.md`

| | suppressed | control (`suppress_recovery:=false`) |
|---|---|---|
| halts | 4 | 8 |
| **longest halt held** | **56.69 s** | **10.00 s** |
| `movement_time_allowance` read back | 1 000 000 s at every transition | 10 s at every transition |
| `controller_server: Failed to make progress` | **0** | **6** |
| **recovery behaviours fired DURING a halt** | **0** | **2** (Spin ×1, Wait ×1) |
| commands leaked past the latch | 0 | 0 |

Reproduced: a second run gave 62.99 s against 8.50 s, same structure. **The
discriminator is the halt duration and the six progress-checker failures, not the raw
recovery count** — in the control arm the progress-checker failure and the halt release
happen at the same instant, so a recovery can land just outside the attributed window.

### 5.9 Navigation baseline and regression

`results/phase1_nav_baseline.md`, `phase1_nav_actors.md`, `phase1_nav_regression.md`,
`phase1_map_coverage.md`

| | baseline | with actors | after the Phase 3 re-frame |
|---|---|---|---|
| goal result | SUCCEEDED | SUCCEEDED | **SUCCEEDED** |
| time to goal | 20.5 s | 27.8 s | 20.4 s |
| executed path (ground truth) | 11.01 m | 11.52 m | 11.00 m |
| final position error | 0.079 m | 0.087 m | 0.065 m |
| recovery behaviours | 0 | 1 × Spin | 0 |
| SLAM pose error vs truth (mean / p95) | 0.036 / 0.075 m | 0.027 / 0.058 m | — |

Map accuracy against the world file: **median 0.000 m**, p95 0.056 m, worst 0.156 m,
99.9 % of occupied cells within 0.15 m of a true surface; main aisle **98.9 % known**.

Against a walking pedestrian the robot **stopped twice and arced north**, spending
**+0.51 m of path and +7.3 s** against the baseline — that deviation is the
measurement. Minimum clearance to a dynamic obstacle reads 0.050 m, which is **the
floor of the instrument, not the gap**: 58 returns fell inside the 0.05 m band
discarded as possible self-hits. **No "zero collisions" claim is made from it** —
Gazebo actors are not physics entities, so clearance is the honest quantity.

---

## 6. Tests

```bash
./ws.sh python3 -m pytest tests/ -q        # 241 passed
```

All 241 are **pure functions with no ROS imports and no simulator** — they run in under
a second and prove the algorithms generalise rather than being tuned to one demo run.
The suite grew with the system: 10 → 39 → 110 → 171 → 223 → **241**.

| File | Tests | What it pins |
|---|---|---|
| `test_sensor_validation.py` | 31 | Nominal pass; impossible ω fails; NaN fails; stale stamp fails; camera resolution |
| `test_trajectory_conflict.py` | 27 | Conflicting vs parallel vs distant paths; **the shipped YAML's max cost stays below `INSCRIBED_INFLATED_OBSTACLE`** |
| `test_jerk_limiter.py` | 25 | **The jerk bound holds on the limiter's recursion for every robot at both payload states**; no overshoot past the commanded velocity |
| `test_pitch_gate.py` | 23 | Truncation distance vs pitch angle; the `min_gate_range` interlock; uphill beams never cut |
| `test_map_merge.py` | 23 | Composite correctness; transform application; grid sizing |
| `test_selective_policy.py` | 22 | Frontier accepted; revisited deferred; changed re-accepted; **the weight invariant** (below) |
| `test_traffic_policy.py` | 18 | Conflict radius derivation; **changing the masses in `fleet.yaml` changes the yield direction** |
| `test_safety_distance.py` | 17 | `d_safe(0) == d_min`; monotonic in v; hysteresis band; per-robot `k` |
| `test_fleet_frames.py` | 16 | Frame naming and the three-frame-per-robot structure |
| `test_fleet_parameterization.py` | 12 | **The scan plane clears both the wheel tops and the chassis, for every robot** |
| `test_clearance.py` | 12 | Footprint-polygon clearance against the asymmetric footprint |
| `test_world_geometry.py` | 11 | Pins the buried ramp-slab extension so it cannot change silently |
| `test_spawn_poses_are_clear.py` | 4 | **No robot's footprint starts inside a rack** |

**Three of these exist because a real defect got past a review and was caught by a
run**, and each now fails a test instead of costing a day:

- *Spawn poses.* Both robots were spawned **inside racks**. Phase 0's smoke tests
  override the spawn pose, and the fleet check confirmed topics and TF without ever
  asking whether the robot was in free space.
- *Scan plane.* The LiDAR sat inside the chassis and **saw its own wheels** — 18 beams
  per scan at 0.44–0.49 m, mapped as static obstacles at the robot's own position, and
  the global planner failed every attempt before the robot had moved.
- *The selective-update weight invariant.* The first weight set **re-created the bug
  the policy exists to prevent**: a new obstacle on a corridor driven six times scored
  0.20 and was deferred. The fix was an invariant, not a retune — every positive weight
  must exceed `w_revisit + accept_threshold` — so a later retune fails a test instead
  of silently dropping obstacle updates.

Lint is part of the same gate:

```bash
./ws.sh python3 -m flake8 .
./ws.sh python3 -m black --check .
```

---

## 7. Design decisions

**1. The safety gate is a serial, fail-closed last link — never a `twist_mux` input.**
A mux picks the highest-priority *live* input, so a dead safety node fails **open**.
Serial and last, the gate owns the topic the plant reads, so if it dies nothing moves.
This is the one decision measured directly rather than argued: **0.196 m against
3.500 m under SIGKILL** (§5.2).

**2. Gating without notification is a bug.** Anything that zeroes `cmd_vel` must tell
Nav2, or the progress checker declares the robot stuck and fires recovery behaviours
into a vehicle that must not move — spin-in-place at a narrow intersection during a
yield. Both the gate and the arbiter raise `movement_time_allowance` on entry and
restore it on exit, and both **read the value back** rather than assuming the write
took. Because two nodes now write the same Nav2 parameter, both capture the
**original** value at startup rather than reading whatever is live at entry —
otherwise whichever wrote second would latch the other's 1e6 as its "original" and
restore it forever. *(That interaction is designed for, not exercised — see §10.)*

**3. Safety velocity comes from odometry, never from the command.** A command is what
the stack *wants*; the stopping distance depends on what the vehicle is *doing*. `k`
is per-robot and payload-scaled: `k = 1/(2·a_eff)`,
`a_eff = |max_decel_x|·m_base/(m_base + m_payload)`, and `[k] = s²/m` so that `k·v²`
yields metres.

**4. The conflict radius is derived from geometry, not tuned.**

```
1.94 m  =  r(amr1) 0.539  +  r(amr2) 0.429  +  d_safe(amr1 @ 0.60 m/s) 0.975
```

That is the distance at which one robot's SafetyGate would already be holding the
other, so past it neither can resolve anything by driving. There is no tuned constant
in it, and an `amr3` with a longer stopping distance widens it **by existing**. The
release radius is 1.25× that, for hysteresis.

**5. Yield priority comes from mass, not from names.** The assignment says "the lighter
AMR-2 always yields to the heavier AMR-1". That is implemented as *gross mass, ties
broken by declaration order* — amr1 90.0 kg > amr2 23.0 kg — so **no robot is named in
the arbiter**, and `tests/test_traffic_policy.py` asserts that changing the masses in
`fleet.yaml` changes the yield direction.

**6. Original header timestamps are preserved through the BSP.** Rewriting stamps on
republish silently breaks TF and SLAM. The evidence is the stamp-match count itself
(**903 pairs, 0 restamped, 0 ns**), because pairing by exact stamp means a restamp
shows up as zero matched pairs rather than as plausible numbers computed against
mismatched data.

**7. A yield releases by *ceasing to publish*, not by sending a release message.** The
mux's own 0.5 s timeout drops the channel, so there is no release message to lose and
an arbiter that dies frees the robot rather than pinning it.

**8. No per-robot branching, anywhere.** There is no `if robot == "amr1"` in the
codebase. Every difference — URDF geometry, mass, velocity/acceleration/jerk limits,
safety gain, yield priority — comes from `fleet.yaml`. Instruments that *observe* a
named robot are not part of the system and may name one.

**9. RegulatedPurePursuit ships, MPPI is deferred — recorded, not hidden.** MPPI loads
cleanly and then commands **0.0034 m/s** on this robot, with the global plan, the
transformed plan, the local costmap, the footprint polygon, all 8 critics and every
parameter verified good on the same runs. The leading hypothesis is `ax_max = 0.4`
from `fleet.yaml` against a stock 3.0. The MPPI block is preserved byte-identical in
`nav2_params.yaml` as `FollowPath_mppi_deferred`, so reverting is a rename of two
keys. **The retest was never run** and the hypothesis stands untested (§10).

**10. Language chosen by measured latency, not preemptively.** Python by default; C++
for `SafetyGate` and the costmap plugins, where the latency budget justifies it. The
BSP was instrumented first and its latency never justified a rewrite.

---

## 8. Scaling to N robots

Fleet size is bounded to **one file**. `config/fleet.yaml` is a typed robot list
interpreted by exactly one module (`amr_description/fleet_config.py`); launch files
loop over it and never name a robot.

Adding a third robot is this diff, and nothing else:

```diff
--- a/src/amr_description/config/fleet.yaml
+++ b/src/amr_description/config/fleet.yaml
@@
     max_jerk_x: 2.50
     max_jerk_theta: 6.00
     spawn: {x: -11.0, y: 1.5, z: 0.15, yaw: 0.0}
+
+  # ------------------------------------------------------------------------
+  # AMR-3 "Scout" - a second light chassis. No code anywhere knows it exists.
+  # ------------------------------------------------------------------------
+  - name: amr3
+    type: scout_follower
+    base_mass_kg: 18.0
+    payload_kg: 5.0
+    wheel_separation: 0.36
+    wheel_radius: 0.10
+    base_length: 0.55
+    base_width: 0.38
+    base_height: 0.18
+    lidar_height: 0.35
+    max_vel_x: 1.00
+    max_accel_x: 1.00
+    max_decel_x: -1.40
+    max_vel_theta: 1.80
+    max_accel_theta: 2.50
+    max_jerk_x: 2.50
+    max_jerk_theta: 6.00
+    spawn: {x: -11.0, y: 4.5, z: 0.15, yaw: 0.0}
```

**No launch code changes are required.** That one block is enough because the fleet
launch's only fleet-aware code is a loop:

```python
for index, robot in enumerate(load_fleet()):
    actions.append(TimerAction(period=SPAWN_START_S + index * SPAWN_STAGGER_S,
                               actions=robot_actions(robot, world_name)))
    actions.append(_include("amr_bringup", "robot_stack.launch.py",
                            {"robot": robot["name"], ...,
                             "stagger": str(STACK_STAGGER_S + index * STACK_STAGGER_STEP_S)}))
```

What follows automatically from that one block:

| Derived from the new entry | How |
|---|---|
| URDF, chassis, wheel and mast geometry | the one shared `amr.urdf.xacro`, rendered with this entry's values |
| Namespace `/amr3`, frames `amr3/*` | `robot_actions` + `robot_stack.launch.py` |
| Its own `slam_toolbox` and six Nav2 servers | `robot_stack.launch.py`, staggered by index |
| A third input to `/fleet_map` and a third `fleet_map → amr3/map` transform | `FleetMapNode` loops over `load_fleet()` |
| Safety gain `k`, `d_safe`, `min_gate_range` | computed from this entry's mass and decel |
| Velocity, acceleration and jerk limits, loaded and unloaded | `PayloadJerkAdapter`, per robot |
| Its trajectory published to, and consumed by, every peer's local costmap | `TrajectoryPredictor` + `FleetTrajectoryLayer` |
| Its place in the yield order | gross mass 23.0 kg — it yields to amr1, ties with amr2 broken by declaration order |
| DDS capacity | `cyclonedds.xml` already allows **400** participants — chosen to sit far above the ten-robot case rather than tuned to just clear the largest configuration this repository launches |

**Stated precisely: this is configuration-driven scaling, not free scaling.** It is an
edit to one file, which is bounded, not zero. Three constraints are real and worth
naming: spawn poses must be in free space (asserted by
`tests/test_spawn_poses_are_clear.py`), the `fleet_map` grid must cover the new spawn
pose, and each additional robot adds ~25 nodes and a full Nav2 stack of CPU load. **A
10-robot run was not performed** — the assignment asks for a minimal configuration
change, and that is what is demonstrated.

---

## 9. Refactoring deliverable (§5.2)

**The area identified.** The standard ROS/Nav2 pattern — and what this repository
started with — is a single "bringup" launch file that owns everything: rendering the
world, starting the Gazebo server and the `/clock` bridge, spawning the robot, and
then staging SLAM, Nav2 and the safety stack. `src/amr_bringup/launch/amr1_nav.launch.py`
was exactly that: **180 lines composing one robot and one world in one file, with no
seam between the two.**

The specific defect is that it **conflates world singletons with per-robot actions**.
A world has one Gazebo server, one rendered SDF and one clock bridge no matter how many
robots are in it; a robot has a state publisher, a spawn, a bridge, a watchdog, a BSP,
a SLAM instance, six Nav2 servers and a safety gate. Fused in one file, the only way to
add a second robot is to duplicate the staged block — and the duplicate quietly brings
a second Gazebo server with it.

### Before

```
src/amr_bringup/launch/
└── amr1_nav.launch.py ................................ 180 lines, ONE robot
    ├── render_world(...)              ─┐
    ├── gz_server(...)                  │  world singletons, inline
    ├── clock_bridge()                  │
    ├── TimerAction(5s, robot_actions) ─┘  + per-robot spawn, inline
    ├── TimerAction(10s → amr_bsp/bsp.launch.py)            ─┐
    ├── TimerAction(14s → amr_navigation/slam.launch.py)     │  per-robot stack,
    ├── TimerAction(22s → amr_navigation/nav2.launch.py)     │  inline and
    └── TimerAction(23s → amr_safety/safety_gate.launch.py) ─┘  single-robot only
```

Adding robot two means copy-pasting five `TimerAction`s and every launch argument, then
remembering that three of the actions above them must *not* be copied.

### After

```
amr_gazebo/spawn.py
├── world_actions(...)   ← world singletons, extracted to a library function:
│                          rendered SDF + gz server + clock + ground-truth bridge
└── robot_actions(robot, world_name)   ← everything one robot needs in the simulator

src/amr_bringup/launch/
├── robot_stack.launch.py ............................. 222 lines, REUSABLE
│   │   one robot's SOFTWARE stack. No world, no Gazebo, no clock.
│   │   Parameterised by `robot` and `stagger`; feature flags for every A/B arm.
│   ├── BSP_START_S       10s → amr_bsp/bsp.launch.py
│   ├── SLAM_START_S      14s → amr_navigation/slam.launch.py
│   ├── NAV2_START_S      22s → amr_navigation/nav2.launch.py
│   │                         → amr_motion/motion_chain.launch.py
│   ├── GATE_START_S      23s → amr_safety/safety_gate.launch.py
│   └── PREDICTOR_START_S 24s → amr_fleet_control/trajectory_predictor.launch.py
│
├── amr1_nav.launch.py ................................ 179 lines, ONE robot
│   ├── world_actions(out_name="warehouse_nav.sdf")
│   ├── robot_actions(robot)
│   ├── fleet_map + costmap_filters
│   └── include robot_stack.launch.py  (stagger 0)
│
└── fleet_nav.launch.py ............................... 277 lines, THE FLEET
    ├── world_actions(out_name="warehouse_fleet.sdf")
    ├── for index, robot in enumerate(load_fleet()):        ← the ONLY fleet-aware code
    │       robot_actions(robot)         staggered by index
    │       include robot_stack.launch.py  stagger = f(index)
    ├── fleet singletons: FleetMapNode, costmap filter group
    └── TrafficControlNode (after every predictor)
```

Twelve scenario launches (`phase1_*`, `phase2_*`, `phase3_*`, `phase5_*`, `phase6_*`,
`phase7_*`) sit on top of these two compositions and add only a mission, a probe and
their arguments.

### Why this split, specifically

| Reason | What it bought |
|---|---|
| **Testability** | A/B arms became launch arguments rather than edited files. `with_motion_chain`, `with_trajectory_layer`, `suppress_recovery`, `with_traffic_control` and `with_watchdog` each select a control arm, so **both arms of every A/B in §5 come from the same launch file** — the control cannot drift from the treatment |
| **Composition** | `robot_stack.launch.py` has one input (`robot`) and one timing offset (`stagger`). Two robots is a loop; ten is the same loop |
| **Reuse** | The staging constants (`BSP_START_S` … `PREDICTOR_START_S`) live in one place. Before, the single-robot and fleet timings would have been two copies free to diverge silently |
| **Lifecycle ownership** | The split makes explicit which actions are world singletons and which are per robot. `FleetMapNode` must start **before** Nav2 — `Costmap2DROS::on_activate` blocks on the `fleet_map → amrN/base_footprint` transform and the lifecycle manager's `change_state` call has no timeout, so starting Nav2 first does not fail fast: it hangs for a full `initial_transform_timeout` and then aborts with nothing in the log naming the missing frame |

### The refactor decision that was deliberately *not* taken

`amr1_nav.launch.py` still exists as its own composition rather than becoming a
one-robot special case of `fleet_nav.launch.py`. That simplification is tempting and
would be wrong: `phase1_survey`, `phase1_nav_run` and `phase2_safety_run` all include
it **by filename**, and those runs' numbers are sensitive to graph contention. Folding
it into the fleet launch would silently turn every Phase 1 and Phase 2 evidence run
into a two-robot run — no error, just moved measurements. **Neither launch includes the
other; both call the same two library functions.** Removing duplication between two
files whose measurements must stay comparable is how a benchmark quietly stops
measuring what it claims to.

---

## 10. Known limitations

Stated plainly, each with its evidence. A reader who catches an overclaim discounts
everything else.

> ### Read this first: a limitation that stood here is retracted
>
> Until this revision this section opened by reporting that **only one of the two
> robots ever reached its goal** — the other replanning the full 10.5 m path
> repeatedly, never translating more than about 2 m, `bt_navigator` aborting on
> `Failed to make progress` — reproduced in **nine consecutive runs**. **That does not
> reproduce. It is retracted as a limitation of this code.**
>
> Re-measured on this commit with **no source change**, `phase3_fleet_goals` reaches
> both goals in six consecutive runs:
>
> | run | AMR-1 | AMR-2 | verdict |
> |---|---|---|---|
> | 1 | SUCCEEDED **18.6 s**, 0.016 m final error | SUCCEEDED **11.3 s**, 0.182 m | PASS |
> | 2 | SUCCEEDED 18.9 s, 0.019 m | SUCCEEDED 11.3 s, 0.185 m | PASS |
> | 3 | SUCCEEDED 18.6 s, 0.011 m | SUCCEEDED 11.3 s, 0.182 m | PASS |
> | 4 — one leak generation left alive deliberately | SUCCEEDED 18.6 s, 0.029 m | SUCCEEDED 11.3 s, 0.156 m | PASS |
> | 5 — on a **from-scratch build** (`rm -rf build install log`) | SUCCEEDED 18.6 s, 0.009 m | SUCCEEDED 11.3 s, 0.183 m | PASS |
> | 6 — the run the Demo B preview is cut from | SUCCEEDED 18.6 s, 0.017 m | SUCCEEDED 11.3 s, 0.235 m | PASS |
>
> Committed as `results/phase3_concurrent_goals_recheck.md`. Those timings reproduce
> `results/phase3_concurrent_goals.md` (18.8 s / 11.3 s) — the artifact this file
> previously called historical and unreproducible. It is neither.
>
> **What changed was the machine, not the repository.** Two faults were found in the
> environment the failing runs had been measured in:
>
> 1. **`scripts/clean_processes.sh` never matched four of this workspace's own
>    nodes** — `trajectory_predictor`, `traffic_control`, `payload_jerk_adapter` and
>    `priority_mux`. All four are long-lived subscribers, so they survived every
>    `PROCESS TABLE CLEAN` banner and accumulated across interrupted runs: an audit
>    found **seven generations still running, the oldest 13 h 44 m old**. This is also
>    why the A/B arms that "ruled out" the trajectory layer and the motion chain could
>    not have ruled them out — disabling a component in the *new* launch does nothing
>    about the copy still running from the *previous* one.
> 2. **The disk had 977 MB free of 43 GB**, because those orphans had written **5.3 GB
>    of logs** — 19 files over 50 MB, the largest 389 MB of repeated `TF_OLD_DATA`
>    warnings and still growing when it was found.
>
> **Which of the two mattered is not isolated, and is not claimed.** Deliberately
> leaving one generation of orphans alive did *not* reproduce the failure — that is
> run 4 above — so a single leak is not sufficient on its own. The accumulated load of
> seven generations, the exhausted disk, or the two together all remain candidates.
> What the six runs support is narrower and is all that is asserted here: **the
> shipped code reaches both goals**, and the entry that stood here described the
> machine rather than the repository.
>
> The cleanup script now matches those four nodes by name *and* carries a catch-all on
> this workspace's own install path, so a node added later is covered by construction.
> **This is the second finding in this file traced to leaked processes** — §12 is the
> first, and was retracted the same way — which is why the fix went into the script
> rather than into a note telling the next person to check by hand.

**1. The ramp A/B specified in the plan cannot be staged in this world.** The
experiment — *a flat alternative exists → the ramp is avoided; the flat route is
blocked → the ramp is taken* — needs one goal reachable **both** ways. This world has
no such goal: the **upper plateau is a solid box spanning the full y extent**
(x ∈ [6, 18], y ∈ [−9, 9]), so the ramp is the only route up and nothing on the lower
level needs it. An earlier note claimed a two-route topology already existed in the
aisle; both aisle lanes are flat for every x west of the ramp toe, so **that claim is
retracted**. Fixing it needs the second ramp whose footprint is already reserved in
`world.yaml`.

**2. Graded ramp cost is not confirmed under Jazzy, and it is not diagnosed.** The
mask plumbing is finished and measured: 3 600 pixels at value 102 on a correctly
origined 680 × 400 grid, `mode: scale` with thresholds that leave the gradient intact,
and both costmaps logging `Received filter mask`. **The cost over the 3 550-cell ramp
footprint is 0..100 — identical to the null-mask run.** Leading hypothesis, stated as
a hypothesis: Nav2 Jazzy's `KeepoutFilter` maps mask values to cost **binarily**
(100 → LETHAL, everything below → nothing) rather than proportionally. If that is
right, a graded keepout mask cannot express "expensive but passable" at all, and
graded slope cost needs the custom `RampCostLayer` that was cut from the plan.
**`RampCostLayer` was cut and is not in this repository.** `ramp_mask_value` defaults
to 0.0, so the shipped behaviour is the null mask and every earlier artifact still
describes the costmap it was measured in. A related earlier claim — that the question
was "settled from the source" — is **retracted**: the source read answered the combine
rule, not the value mapping.

**3. Autonomous mutual deviation is NOT claimed.** `FleetTrajectoryLayer` provably
injects the peer's predicted trajectory as cost into the receiving robot's local
costmap — **50/50 samples cost > 0 with the layer against 0/59 without**. But
RegulatedPurePursuit is not a sampling optimiser: it consumes local costmap cost
through cost-regulated velocity scaling and forward collision checking, so the layer
changes how it **paces**, not where it goes. It does not deviate laterally around
graded cost the way MPPI's `CostCritic` would. **No run in this repository should be
read as showing robots mutually deviating without central intervention.** Unresolved
conflicts are handled by centralised arbitration (§5.7), and the escalation ordering —
local first, central only after the local layer has had 2.0 s and failed to open the
gap — is measured. The behavioural difference between the A/B arms is also not
claimed: amr1 drove 8.30 m in 18.0 s with the layer and 9.03 m in 20.6 s without,
closest approach 1.610 m vs 1.529 m, but that is n=1 per arm and an earlier layer-on
run gave 1.538 m, inside the same spread.

**4. The jerk bound holds on the limiter's recursion, not on the published stream.**
`tests/test_jerk_limiter.py` asserts the bound for every robot at both payload states.
The measured column in §5.6 exceeds the configured bound by up to **~1.9×**, and what
it measures is the *published stream*: a 20 Hz signal timestamped **on arrival**,
resampled onto a 50 ms grid and differentiated twice, plus a single-step transient
where the velocity reaches its target. **What §5.6 supports is the payload ratio and
the absence of overshoot, not a certified jerk ceiling.** Closing that gap means
timestamping at the publisher, and it is not done. *(For context on why the shape
matters: the continuous-time S-curve `sqrt(2j|e|)` is the wrong formula for a discrete
loop — it leaves acceleration on the books at arrival, spiking measured jerk to 2.9–3.5×
the limit. The exact discrete form `−jΔt/2 + sqrt((jΔt/2)² + 2j|e|)` brings the worst
case to ~1.1× on the recursion.)*

**5. `twist_mux` is substituted in-repo, and the reason is a binary incompatibility.**
`ros-jazzy-twist-mux` 4.5.0 resolves a `diagnostic_updater` symbol ending `EEdh`
(double, unsigned char); the installed `ros-jazzy-diagnostic-updater` 4.2.6 exports
`...EEd`. Two packages from one apt snapshot, binary-incompatible with each other, and
nothing in this workspace can repair it. `amr_safety/scripts/priority_mux.py`
preserves the architecture exactly — a priority mux between the motion chain and the
fail-closed gate — and reads the **same `twist_mux.yaml` schema**, so restoring the
stock node is a **two-line launch change**. Diagnosed, not worked around silently.

**6. Recovery-suppression necessity rests on the Phase 3 barrier A/B, not on the yield
path.** §5.8 proves necessity: a 56.69 s hold against a 10 s allowance, 6
progress-checker failures and 2 recoveries in the control arm alone. The yield-path
control arm **did not reproduce the encounter** — its hold lasted 1.0 s against a 10 s
allowance, so the progress checker was never going to fire. That run is recorded as a
non-result rather than presented as a second proof. Related: a **full-twist hold
against an obstacle that never moves is a deadlock by design** — the robot has no legal
action that could increase its clearance, so the goal ends in TIMEOUT in *both* arms.
That is correct for a fail-closed gate, and "allow in-place rotation under a hold" is
flagged as future work rather than claimed.

**7. The Phase 1 map covers the main aisle only.** Coverage: main aisle **98.9 %**,
ramp approach 100 %, ramp surface 65.2 % (seen, never driven), **rack backs N and S
0.0 %**, **upper plateau outside the grid**. A 2D LiDAR maps what it can see, and the
robot never drove behind the racks or up the ramp — mapping while pitched writes the
phantom ground return into the map as a wall that is not there.

**8. The inter-map transform is fixed from the spawn poses and never corrected.** A
deliberate cut. Drift is **not measured and is not claimed**. (For anyone revisiting
it: rclpy's `StaticTransformBroadcaster` is append-only — it silently skips a
`child_frame_id` it has already sent, so republishing a corrected transform through it
does nothing.)

**9. The yield scenario is staged, and staged is not deterministic.** Runs of the
identical launch have given **3, 2, 1 and 0 escalations**, and six further runs while
recording the previews gave **0** at the default prediction window and **1, 0, 2, 0 and
1** at `time_window_s:=5.0` — so widening the window raises the odds and does not fix
them. **Three of those six escalated nothing.**
The barrier, the 3.0 m gap and the 5.0 s dispatch offset stage the encounter; the
simulator is under no obligation to stage it identically twice. A run that escalates
nothing reports **NOT EXERCISED**, not FAIL — a run in which the local layer resolved
everything is not a failed run. The escalation that did fire released on *conflict
cleared* after 2.8 s with 0 recovery behaviours during the hold, and both goals still
SUCCEEDED.

> **9a. In this scenario AMR-1 can stop at the barrier and never restart, and that is a
> real defect rather than staging noise.** In **two of the six `phase7_yield` runs on
> record**, AMR-1 did not reach its goal: its own `SafetyGate` halted it beside the
> barrier and never released, while AMR-2 completed normally. In the run measured for
> this entry the final halt was `clearance 0.299 m <= d_safe 0.305 m at measured
> 0.052 m/s`, and the robot was **still held 341 s later** when the run was stopped. No
> `RELEASE` line follows it.
>
> **The mechanism is an interaction between two deliberate behaviours, and neither is
> wrong on its own.** The gate releases when the clearance in the sector the current
> command drives into rises above `d_release`, or when no translation is commanded at
> all (`safety_gate.cpp` `Evaluate`). Nav2 here keeps commanding *forward*, into a
> barrier that is static, so the forward sector never opens. Ordinarily the progress
> checker would time out and Nav2 would run a recovery that backs the robot off — but
> the gate raises `movement_time_allowance` to 10⁶ s for exactly as long as it is
> blocking (§5.8, ENGINEERING_NOTES rule 2), which is right for a pedestrian who will
> walk away and wrong for a barrier that will not. The two together livelock.
>
> **What is not claimed:** a rate. Two of six runs is what was observed, the trigger
> geometry was not isolated, and no fix was attempted — a release path conditioned on
> the obstacle being persistent rather than transient is the obvious direction and it is
> not in this repository. This does not affect the yield measurement in §5.7: that run
> is `results/phase7_yield.md`, in which **both goals SUCCEEDED**.

**10. Two nodes write the same Nav2 parameter, and that interaction is designed for
but not exercised.** `SafetyGate` raises `movement_time_allowance` on a halt and the
arbiter raises it on a yield. Both capture the **original** value at startup, and the
arbiter re-asserts every 2 s while holding. **Neither ordering was exercised in any
run.**

**11. Collisions are never claimed.** Gazebo actors are not physics entities — they are
absent from `pose/info`, `dynamic_pose/info` and `gz model --list`, so they can never
generate contacts. They *are* rendered and *are* raycast by `gpu_lidar`, which is what
navigation needs. **Clearance is the honest quantity throughout.**

**12. One finding was measured, written down, believed for a session, and then
retracted.** "The camera stalls the drive" — re-measured on a verified-clean process
table at **0.3500 m/s both with and without the camera**. The real cause was four
`drive_watchdog` processes leaked from earlier launches, still republishing onto
`/amr1/cmd_vel_plant`; the contention starved the graph until the gate's own
command-timeout fail-safe fired and interleaved **663 zeros into 884 messages**. The
camera stays defaulted off for a smaller and different reason: nothing consumes it.
It was corrected in place with the re-measurement rather than deleted, and the
retraction is stated here rather than dropped. The other corrections made the same
way are items 1 and 2 above.

**13. MPPI does not drive this robot, and the retest was never run.** See §7.9.
`TrajectoryPredictor` also exited 1 once during shutdown of a Phase 5 run, after its
evidence had been written — **not diagnosed**, and recorded rather than ignored.

**14. Most measurement runs were made with the pedestrians switched off, and that is
a deliberate trade rather than an oversight.** The assignment requires a warehouse
with frequent randomly moving dynamic obstacles, and the world has three; they are
verified visible to the LiDAR and lethal in the costmap
(`results/smoke1_actor_visibility.md`). But an actor walking through the measured
region is an uncontrolled variable, so every A/B in §5 — the trajectory-layer cost
injection, the fail-closed distance, the yield escalation, the payload trace — runs
with `with_actors:=false`. **The runs that do exercise dynamic obstacles are the
safety demo** (`results/phase2_safety_suppressed.md`, 3 halts on a pedestrian, goal
still SUCCEEDED) **and the Phase 1 actor comparison**
(`results/phase1_nav_actors.md`: the robot stopped twice and arced north, spending
+0.51 m of path and +7.3 s against the baseline). So dynamic-obstacle handling is
demonstrated and measured, but it is **not** simultaneously present in the multi-robot
conflict and mapping evidence. `with_actors:=true` turns them on in
`phase3_fleet_goals` and `phase6_conflict` for anyone who wants to see it; a check
this session confirmed both goals still SUCCEEDED with all three actors walking.

**15. The cooperative-mapping demo is a demonstration, not a second evidence run.**
`fleet_survey.launch.py` writes its own accept/defer report to
`results/fleet_survey_updates.*`, and those counts move from run to run because the
route, the timing and the SLAM updates are not identical twice. **The committed
selective-mapping numbers are the Phase 3 artifact** (`phase3_selective_updates.md`,
41/23/18, 43.9 % deferred), measured while both robots were under Nav2 control. The
survey artifact is git-ignored for exactly that reason — it must not be mistaken for
the evidence it sits beside.

---

## Repository layout

```
.
├── README.md               this file
├── ws.sh                   the workspace wrapper — every ROS/Gazebo command goes through it
├── scripts/
│   └── clean_processes.sh  pre-flight; prints the surviving process table, exits non-zero
├── src/                    9 ROS 2 packages
│   └── amr_bringup/rviz/   fleet_mapping.rviz — the cooperative mapping view
├── tests/                  241 pure-function unit tests
├── results/                evidence artifacts — every number in this README
├── media/                  preview GIFs and stills, one per demo, plus an archive/
├── videos/                 the submission screenshare is recorded here; git-ignored
└── docs/                   demo runbook, design invariants, the assignment
```
