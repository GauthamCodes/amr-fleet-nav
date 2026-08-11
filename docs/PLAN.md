# RSE Hiring Assignment — Execution Plan V2
**Adaptive Navigation & Conflict-Aware Path Planning (2-Robot AMR Fleet)**

*Revision note: V2 incorporates external review. Changes from V1 are marked ⚡. Rejected review items and rationale are in §9.*

---

## 1. Strategy

Extend Nav2 rather than duplicate it. Write custom code only where the assignment demands custom work, and make each custom component load-bearing — every node should solve a real failure mode, not just tick a requirement box. Every custom component is a namespaced, config-driven class; that single property is what makes fleet scaling configuration-driven rather than code-driven.

**Stack:** ROS2 Jazzy · Gazebo Harmonic · Nav2 (MPPI controller) · slam_toolbox per robot · twist_mux · Python primary, C++ where measurement justifies it.

⚡ **Terminology discipline:** this is a *low-latency, high-priority* safety override. Not "hard real-time" — no RT kernel, no scheduling guarantees, don't claim them.

---

## 2. Architecture

```
GAZEBO ─ 2 plateaus + ramp, racks, actors with collision geometry
   │
   ├─ /amrN/scan_raw, /amrN/imu_raw, /amrN/camera_raw
   │        ↓
   │   ┌────────────────────────────────────────┐
   │   │ SensorBSP (per robot)                  │  ⚡ load-bearing, not decorative
   │   │  ├ ImuValidator   — ω plausibility     │
   │   │  ├ LidarValidator — NaN/stamp/range    │
   │   │  ├ CameraValidator— stamp/resolution   │
   │   │  └ PitchGate      — ramp ground-return │  ⚡ prevents phantom ramp wall
   │   │                     truncation          │
   │   └────────────────────────────────────────┘
   │        ↓ /amrN/scan, /amrN/imu   (ORIGINAL header stamps preserved)
   │
   ├─ slam_toolbox(amr1) ─┐
   ├─ slam_toolbox(amr2) ─┴─► FleetMapNode ─► /fleet_map
   │                              │            + TF: fleet_map → map_amrN  ⚡ correctable
   │                              └─ selective update policy (scored)
   │        ↓
   ├─ Nav2 (ns amr1) ──┐   global costmap: global_frame=fleet_map,          ⚡ fleet map
   ├─ Nav2 (ns amr2) ──┤   static layer=/fleet_map, + RampCostLayer            drives planning
   │                    │   local costmap:  odom_amrN, + FleetTrajectoryLayer ⚡ MAPF here
   │                    │
   │   TrajectoryPredictor (per robot) ─► /amrN/predicted_trajectory
   │        └─► consumed by the OTHER robot's FleetTrajectoryLayer
   │
   └─ cmd_vel chain (per robot):
        Nav2 cmd_vel
          → nav2_velocity_smoother      (accel/vel limits — stock)    ⚡ reuse
          → PayloadJerkAdapter          (jerk + payload scaling — ours)
          → twist_mux                   (nav 100 | yield 150)
          → SafetyGate                  (serial, last link, FAIL-CLOSED) ⚡ was mux input
          → /amrN/cmd_vel → base

   TrafficControlNode (central): conflict arbitration + yield commands
        └─⚡ on yield/halt, ALSO notifies Nav2 (progress-checker param)
```

### ⚡ Architectural principle: gating without notification is a bug

Any component that zeroes or blocks `cmd_vel` **must** tell the navigation stack, or Nav2's progress checker sees a stuck robot and fires recovery behaviors — spin-in-place at a narrow intersection is a catastrophic failure mode during a yield demo. Applies to both `TrafficControlNode` (yield) and `SafetyGate` (halt).

Implementation: on gate entry, dynamic-param `controller_server.progress_checker.movement_time_allowance` to a large value; restore on exit. Alternative (documented, not primary): cancel and re-issue the Nav2 goal.

### ⚡ TF chain (multi-robot, fleet frame)

```
fleet_map
   ├── map_amr1 ── odom_amr1 ── base_link_amr1 ── {laser,imu}_amr1
   └── map_amr2 ── odom_amr2 ── base_link_amr2 ── {laser,imu}_amr2
        ▲                ▲
        │                └─ slam_toolbox publishes
        └─ FleetMapNode publishes, refines on drift correction
```

Global costmaps operate in `fleet_map`; local costmaps in `odom_amrN`. When FleetMapNode corrects an inter-map transform the robot's fleet-frame pose steps — same class of behaviour as an AMCL correction. Document it as expected, keep corrections small and infrequent.

---

## 3. Requirement → Solution Mapping

| Requirement | Approach | Change from V1 |
|---|---|---|
| Cooperative SLAM, unified global map | Per-robot slam_toolbox → FleetMapNode composites into `/fleet_map`; inter-map transform initialised from spawn poses, **periodically refined by occupancy-grid correlation over the overlap region** | ⚡ transform now correctable, drift measured |
| Merged map actually used | `/fleet_map` is the **static layer of both global costmaps** | ⚡ was fleet-view only |
| Selective mapping | Scored policy: `score = w_f·frontier + w_c·occupancy_change + w_r·recency − w_v·revisit`; below threshold → defer merge (and skip the correlation/composite work, so the saving is real and measurable) | ⚡ was visit-count only |
| Ramp/slope cost | Custom `RampCostLayer` costmap plugin (polygon + slope → traversability cost). Costmap filter mask retained as fallback | ⚡ upgraded from filter-only |
| Payload-aware smoothing | Stock `nav2_velocity_smoother` → custom `PayloadJerkAdapter` (jerk limiting + payload-scaled limits) | ⚡ chain, don't replace |
| **MAPF: local planner consumes other robot's trajectory** | `TrajectoryPredictor` publishes each robot's projected path; **`FleetTrajectoryLayer` costmap plugin** stamps time-decayed cost from the *other* robot's trajectory into this robot's local costmap. MPPI evaluates candidates against it | ⚡ **this is the core V1 gap** |
| Yield protocol | `TrafficControlNode` arbitrates unresolvable conflicts, commands AMR-2 stop via twist_mux yield channel, **and suppresses Nav2 recovery** | ⚡ recovery suppression added |
| Safety override, `d_safe = k·v² + d_min` | `SafetyGate`: serial last link, fail-closed, **`v` from odometry not command**, per-robot payload-scaled `k`, **hysteresis on release** | ⚡ three corrections |
| BSP / HAL validation | `SensorBSP` base class + IMU / LiDAR / Camera validators. **PitchGate does functional work** (ramp ground-return truncation) | ⚡ camera added, made load-bearing |
| Scale to 10+ robots | Typed robot list in YAML (`name`/`type`/`payload`/`limits`) + launch loop. No per-robot branching anywhere | typed config ⚡ |
| Refactoring deliverable | Monolithic bringup → modular includes; one module fully implemented, before/after documented | unchanged |

### ⚡ On the IMU/Camera discrepancy

The assignment body specifies LiDAR + IMU validation; the evaluation table says IMU + Camera and calls it a HAL class. State this explicitly in the README and implement all three validators, with IMU angular-rate plausibility as the primary demonstrated check. Proactively naming the inconsistency reads as careful, not pedantic.

---

## 4. Custom Components

| Component | Lang | Rationale |
|---|---|---|
| `SensorBSP` + validators | Python → measure → C++ if needed | ⚡ instrument first; "language chosen by measured latency" beats a preemptive rewrite. **Preserve original header stamps on republish** — rewriting them silently breaks TF and SLAM |
| `FleetMapNode` | Python | Grid ops via numpy/scipy; not in a latency-critical path |
| `RampCostLayer` | C++ | Costmap plugin API |
| `FleetTrajectoryLayer` | C++ | Costmap plugin API; runs at costmap update rate |
| `TrajectoryPredictor` | Python | Low rate, path-level |
| `PayloadJerkAdapter` | Python | Runs at cmd_vel rate (~20 Hz), fine |
| `TrafficControlNode` | Python | Coordination, low rate |
| `SafetyGate` | C++ | Latency-critical, in the command path |

---

## 5. Build Order

⚡ Reordered: safety and sensor validation exist *before* anything drives near an obstacle or a ramp. Dependency chain is now sensors → validation → SLAM/Nav2 → motion → traffic → safety.

**Phase 0 — Environment + smoke tests (1 day)**
- Workspace, parameterized xacro, two plateaus + ramp, racks, actors
- ⚡ **Smoke test 1 (do this first):** Gazebo actor → LiDAR → obstacle layer. Confirms actors have collision geometry. Two days of "why can't Nav2 see pedestrians" avoided.
- ⚡ **Smoke test 2:** teleop up the ramp, echo `/scan`. Measure the phantom ground return. This number sizes the PitchGate work and belongs in the README.

**Phase 1 — Single-robot Nav2 baseline (1 day)**
- slam_toolbox + Nav2 on amr1, namespaced from the start
- De-risking phase; everything downstream assumes this works

**Phase 2 — SensorBSP + SafetyGate (1–1.5 days)** ⚡ moved up
- All three validators; PitchGate truncation validated against the Phase 0 measurement
- SafetyGate: odom velocity, hysteresis, fail-closed, latency instrumentation from day one
- Nav2 rewired to consume `/validated/*` only
- Demo: actor walks into path → halt; speed sweep table; injected bad IMU → WARN

**Phase 3 — Cooperative mapping (1.5 days)**
- Dual namespaced bringup (TF prefix debugging lives here — budget for it)
- FleetMapNode: composite, correctable transform, drift metric
- Selective update scoring
- ⚡ Wire `/fleet_map` into both global costmaps and confirm planning works in `fleet_map` frame

**Phase 4 — Ramp cost + global planning (1 day)**
- `RampCostLayer`; the two-path A/B experiment is the deliverable, not the mechanism
- Demo: flat alternative exists → ramp avoided; flat route blocked → ramp taken

**Phase 5 — Payload-aware velocity (0.5–1 day)**
- Stock smoother + `PayloadJerkAdapter`; velocity/jerk plots loaded vs unloaded, amr1 vs amr2

**Phase 6 — Trajectory exchange + conflict prediction (1.5 days)**
- `TrajectoryPredictor` + `FleetTrajectoryLayer`
- Demo: robots mutually deviate *without* central intervention — this is the MAPF evidence

**Phase 7 — Yield protocol (1 day)**
- `TrafficControlNode` for conflicts the local layer can't resolve
- ⚡ Recovery suppression — verify no spin-in-place during yield, that's the acceptance test
- Demo: forced narrow-intersection conflict, AMR-2 yields cleanly

**Phase 8 — Tests, metrics, refactor, docs, video (1.5–2 days)**
- Unit tests, metrics collection, launch refactor, README, screenshare

**Total: ~9–11 days.** V1 estimated 7–8; the review additions are worth roughly two days.

### ⚡ Cut list (if time compresses)

Cut in this order — never cut upward:
1. Camera validator → stub with a logged TODO
2. rosbag artifacts → screenshots instead
3. `RampCostLayer` → revert to costmap filter mask (explicitly compliant per assignment wording)
4. Drift correction → fixed transform + document the limitation honestly

**Never cut:** MAPF trajectory layer, safety correctness, recovery suppression, requirement→evidence table.

---

## 6. Verification & Evidence

### ⚡ Unit tests (`pytest`, pure functions — nearly free, high signal)
```
tests/
├── test_safety_distance.py       # d_safe(0)==d_min; monotonic in v; hysteresis band
├── test_trajectory_conflict.py   # conflicting vs parallel vs distant paths
├── test_sensor_validation.py     # nominal pass; impossible ω fail; NaN fail; stale stamp fail
├── test_pitch_gate.py            # truncation distance vs pitch angle
├── test_selective_policy.py      # frontier accepted; revisited deferred; changed re-accepted
└── test_map_merge.py             # composite correctness; transform application
```
These prove the algorithms generalise rather than being tuned to one demo run.

### ⚡ Metrics (a table like this is what separates engineering from a course project)

| Domain | Metrics |
|---|---|
| Safety | sensor→zero-command latency (mean / p95 / max), stopping distance vs speed, false-positive rate |
| Mapping | coverage %, merge updates deferred %, CPU time saved, inter-map drift corrected vs uncorrected |
| Traffic | conflicts predicted / resolved locally / escalated to yield, mean + max wait |
| Motion | peak accel, peak jerk, loaded vs unloaded delta |
| Navigation | goal success rate, path length, time to goal, replan count, collisions |

### Dimensional justification for `d_safe`
State units explicitly: `[k] = s²/m` so that `k·v²` yields metres. Give the chosen values, the reasoning, and the measured stopping-distance sweep — not just the constants.

---

## 7. Package Layout

```
src/
├── amr_description/     # parameterized xacro
├── amr_gazebo/          # world, actors, spawn
├── amr_navigation/      # Nav2 + slam_toolbox configs
├── amr_costmap_plugins/ # ⚡ RampCostLayer, FleetTrajectoryLayer
├── amr_fleet_control/   # FleetMapNode, TrajectoryPredictor, TrafficControlNode, missions
├── amr_motion/          # PayloadJerkAdapter
├── amr_safety/          # SafetyGate, twist_mux config
├── amr_bsp/             # SensorBSP + validators
└── amr_bringup/         # ⚡ system composition only
```

---

## 8. README Structure

⚡ V1's README was architecture-heavy. The evaluator wants proof it works.

```
1. Problem & scope
2. Demo (GIF/video up top)
3. Quick start
4. Architecture (diagram + command-path priority)
5. Requirement → Implementation → Evidence table    ← the highest-value section
6. Results (metrics tables, plots)
7. Tests (how to run, what they prove)
8. Design decisions (incl. the IMU/Camera discrepancy note)
9. Known limitations
10. Scaling to N robots (show the amr3 config diff)
11. Refactoring deliverable (before/after tree)
```

⚡ Wording: "low-latency safety override," not "hard real-time." "Configuration-driven fleet scaling," not "for free."

---

## 9. ⚡ Rejected Review Items (and why)

| Item | Verdict |
|---|---|
| "Selective mapping is non-compliant — it only throttles output" | **Reject the framing.** Assignment says verbatim "map updates (publishing to the merged map)". Adopt the better scoring algorithm for quality; do not re-architect slam_toolbox internals |
| "Ramp cost via filter mask is oversold / non-compliant" | **Downgrade.** Assignment permits "custom cost function *or a tuned configuration*." Upgrading to a layer plugin is a quality choice, not a compliance fix |
| "Implement proper multi-robot pose-graph SLAM" | **Scope trap.** Bounded fix only: correctable transform + drift metric. Full distributed SLAM is a research project |
| "Rewrite BSP as C++ zero-copy components for latency" | **Defer.** Instrument first, optimise on data. Real risk in the relay is timestamp rewriting, not throughput |
| rosbag-driven test harness | **Partial.** Record bags as evidence artifacts; skip building a bag-replay test framework |
| Broad package restructuring | **Mostly ignore.** Adopted `amr_bringup` and `amr_costmap_plugins`; the rest is churn |
| "~75–80% compliance" score | **Ignore the number.** Act on the specific findings |
| "slam_toolbox has multi-robot capability" | **Unverified.** Check the repository directly before letting it influence any design decision |
```
