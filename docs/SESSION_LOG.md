# Session Log — AMR Fleet Navigation

One entry per phase: what was built, what was verified, what surprised us,
what carries forward.

---

## Setup — 2026-08-11

**Built:** Workspace skeleton at repo root (colcon workspace == git repo).
Docs in place: PLAN.md (execution plan V2), ASSIGNMENT.pdf (source requirements),
docs/ENGINEERING_NOTES.md (project invariants).

**Verified:**
- ROS_DISTRO = jazzy
- Gazebo Sim 8.11.0 (Harmonic)
- nav2_bringup, slam_toolbox, twist_mux all resolve via `ros2 pkg list`

**Surprises:** None.

**Carries forward:** Phase 0 blocked on smoke test 1 (Gazebo actor collision
geometry visible to LiDAR). Sensor height on the xacro stays a tunable arg until
smoke test 2 reports the phantom ground-return distance.

---

## Phase 0 — Environment + smoke tests — 2026-08-12

**Built:**
- Nine package skeletons per PLAN.md §7. Seven are `package.xml` + build file only;
  `amr_description` and `amr_gazebo` carry the real Phase 0 content.
- One parameterized `amr.urdf.xacro` (mass, payload, wheelbase, sensor height,
  acceleration limits as args) plus `amr_gazebo.xacro` for the Gazebo blocks.
- `config/fleet.yaml`: the typed robot list. `amr_description/fleet_config.py` is its
  only interpreter; launch files loop over it and never name a robot.
- `warehouse.sdf.xacro`: two plateaus, one shallow ramp whose run, slope length and
  slab pose are all **derived from `ramp_angle_deg`**, 10 racks forming an aisle,
  3 pedestrians. `with_actors` arg switches the pedestrians off.
- Smoke tests 1 and 2 with their measurement nodes, plus `bag_probe.py` for offline
  re-derivation from the recorded bag.

**Verified:**
- `colcon build --symlink-install` from scratch: **9 packages finished, 0 errors**.
- `pytest tests/`: 10 passed. Asserts amr1 and amr2 differ in mass, accel limits and
  wheel separation, that frames are prefixed and non-colliding, and that
  `lidar_height` is honoured as a ground-referenced quantity.
- flake8 + black: clean.
- Both robots spawn from the single YAML; `amr2/laser` sits at x=0.215 against
  `amr1`'s 0.29, i.e. per-robot geometry reaches TF.

**SMOKE TEST 1 — actors ARE visible to the LiDAR. No fallback needed.**

| Measurement | Control box (has collision) | Actor (subject) |
|---|---|---|
| scans with a return | 100 % | 92 % |
| median closest return | 4.16 m (predicted 4.23) | 2.82 m (centre line 2.71) |
| peak Nav2 costmap cost | 254 LETHAL, 100 % of frames | 254 LETHAL, 100 % of frames |

The control box is the point of the design: it has ordinary collision geometry, so a
negative actor result could not have been blamed on LiDAR height or configuration.

**SMOKE TEST 2 — ramp-induced phantom returns (ramp 8.00°, lidar h = 0.200 m):**

| Quantity | Measured |
|---|---|
| max pitch on the ramp | **8.002°** (nominal 8.000°) |
| phantom ground return at max pitch | **1.78 m** |
| same, along the direction of travel | **1.77 m** |
| phantom ramp wall, robot level | **1.42 m past the toe** (= h/tan θ) |
| `d_safe` at 0.6 m/s, k=0.5, d_min=0.3 | 0.48 m — phantom does **not** trip it |

Phantom-wall prediction vs measurement agreed to −0.21 m mean over 113 samples.

**Surprises — four, all found by the smoke tests rather than by reading code:**

1. **Actors are not physics entities.** They are absent from `pose/info`,
   `dynamic_pose/info` and `gz model --list`, so they can never generate contacts.
   They ARE rendered and ARE raycast by `gpu_lidar`, which is what navigation needs.
   A witness camera in the smoke-1 world separates "not rendered" from "not raycast".
2. **IMU sensors need their own `gz-sim-imu-system` plugin.** The generic `Sensors`
   system does not serve them. Gazebo still advertises the topic, so it reads as a
   live sensor publishing nothing. Also: the bridge type is `sensor_msgs/msg/Imu`
   against `gz.msgs.IMU`; the wrong spelling fails silently.
3. **The chassis was wrong, twice, and the ramp caught it.** Drive wheels on the
   centre line with one front caster put the centre of mass exactly on the rear edge
   of the support triangle — level at rest, then 24.6° of rocking on an 8° ramp.
   Adding a rear caster stopped the rocking by making the base a rigid four-point
   body, which then could not pitch at all and jammed solid against the ramp toe
   while its wheels kept turning. Final layout is the classic tricycle: drive wheels
   aft of centre, single front caster, three statically determinate contacts.
4. **Wheel odometry lied and looked plausible.** While jammed at the toe the robot
   reported an 11 m journey it never made. Every position-derived number was wrong
   in a way no single-signal check would have caught. The robot now publishes
   simulator ground truth via `PosePublisher`, and smoke test 2 reports odometry-vs-
   truth as its first section. Final discrepancy is −0.000 m, peak +0.035 m.

Also: Gazebo Harmonic 8.11.0 has no `<gz_frame_id>`, and SDFormat's URDF importer
lumps fixed joints, so sensors report `amr1/amr1/base_footprint/lidar`. Link names
carry the prefix directly and identity transforms tie the Gazebo frames into TF.
The standalone `nav2_costmap_2d` is a lifecycle node and publishes nothing without a
manager; its OccupancyGrid is subscriber-gated, so `costmap_raw` is used instead.

**Environment decision:** two Gazebo installs are present (system `gz-sim8 8.13.0`,
ROS vendor `8.11.0`). `LD_LIBRARY_PATH` and `GZ_CONFIG_PATH` resolve entirely to the
**ROS-vendored 8.11.0**, which has all 140 plugins needed. Pinned there. `ws.sh`
wraps every workspace command so the environment is identical each run.

> **Superseded in Phase 1.** `lidar_height` moved from 0.20 m to 0.35 m because the
> scan plane at 0.20 m grazed the robot's own wheel tops. Both smoke-test reports in
> `results/` were re-measured at the new height and no longer carry the numbers quoted
> below. The Phase 0 text is left as the record of what was known at the time.

**Carries forward:**
- `lidar_height` stays at 0.20 m. Smoke test 2 gives no reason to change it: the
  phantom return at max pitch (1.78 m) sits well outside `d_safe` (0.48 m).
- PitchGate in Phase 2 has its sizing table: closest return per pitch bin, 0°→8°.
- The world reserves a second-ramp footprint (commented in `world.yaml`). Phase 4's
  A/B experiment needs the upper level to be a shortcut, which takes two ramps.
- Frame convention is `amr1/base_link`, which differs from PLAN.md §2's diagram
  (`base_link_amr1`). Chosen deliberately; **PLAN.md not edited — ask first.**

---

## Phase 1 — Single-robot Nav2 baseline — 2026-08-12

**Built:**
- `slam_toolbox` + the six Nav2 servers under `/amr1`, from namespace-free YAML
  templates rendered per robot by `amr_navigation/params.py` (commit `03348a9`).
- Two one-command deliverables in `amr_bringup`: `phase1_survey.launch.py` drives
  the mapping circuit and saves the map; `phase1_nav_run.launch.py` sends one goal
  and writes a metrics report. Both shut themselves down when finished.
- `survey_drive.py`, `nav_goal_run.py`, and the offline `map_report.py` /
  `plot_run.py`. `amr_gazebo/world_geometry.py` reads rack, ramp and actor geometry
  back out of the rendered world SDF, so no measurement script restates it.
- `amr_navigation/clearance.py`: the footprint-relative clearance arithmetic, as
  pure functions. Phase 2's SafetyGate reports the same quantity and will use it.

**Verified:** `colcon build --symlink-install` 9 packages / 0 errors, `pytest` 39
passed, flake8 + black clean.

**EXIT 3 — SLAM map artifact** (`results/phase1_map.{pgm,yaml,png}`; 2 laps of the
aisle, 63.6 m in 163 s of sim time):

| Region | Known | Note |
|---|---|---|
| main aisle | **98.9 %** | the circuit |
| ramp approach | 100 % | seen from the aisle |
| ramp surface | 65.2 % | seen, never driven |
| rack backs, N and S | 0.0 % | never driven behind them |
| upper plateau | outside the grid | reachable only up the ramp |

Accuracy against the world file: **median 0.000 m**, p95 0.056 m, worst 0.156 m,
99.9 % of occupied cells within 0.15 m of a true surface.

Two coverage statements worth making plainly. The rack backs and the upper level are
unmapped because the robot never went there — a 2D LiDAR maps what it can see, and
this survey deliberately stayed on the lower level. And the ramp is **not** driven:
mapping while pitched writes the phantom ground return into the map as a wall that is
not there. Phase 2's PitchGate is the prerequisite. The map shows exactly that wall at
x ≈ 4.9 m, which is the toe (2.44) plus h/tan θ (2.49) — the number smoke test 2
measures independently.

**EXIT 4 — goal-to-goal navigation.** World (−11.0, −1.5) → (−0.5, +1.5), the same
goal with and without pedestrians:

| | baseline | with actors |
|---|---|---|
| goal result | **SUCCEEDED** | **SUCCEEDED** |
| time to goal | 20.5 s | 27.8 s |
| planned path (first plan) | 11.16 m | 11.16 m |
| executed path (ground truth) | 11.01 m | 11.52 m |
| final position error | 0.079 m | 0.087 m |
| global plans published | 20 | 23 |
| **replans (route actually changed)** | **5** | **4** |
| recovery behaviours | 0 | 1 × Spin |
| min clearance, any obstacle | 0.916 m | 0.050 m |
| SLAM pose error vs truth, mean / p95 | 0.036 / 0.075 m | 0.027 / 0.058 m |

Replan count is *routes that changed* (Hausdorff distance from the previous plan
> 0.30 m), not plan messages. The tree recomputes at ~0.85 Hz whether or not anything
moved; counting messages would measure the tree's tick rate, not the world.

**EXIT 5 — dynamic-obstacle clearance.** The run is timed so `pedestrian_1` turns back
down the aisle as the robot sets off, which produces a head-on encounter rather than
an actor that happens to be somewhere. 269 of 279 scans carried a dynamic return.

- **Minimum clearance to a dynamic obstacle: 0.050 m**, at t = 56.6 s, robot at
  (−8.01, +0.29), nearest return at (−8.23, +0.10).
- 5th percentile 0.296 m; median while one was visible 2.688 m.
- The robot stopped twice and arced north around the pedestrian: +0.51 m of path and
  +7.3 s against the baseline. That deviation is the measurement.

**What pedestrians do to a map built while they walk through it.** Each nav run saves
its own map, so the two runs are a controlled pair — same route, same duration, only
the actors differ. Scored against the world file:

| | occupied cells within 0.15 m of a true surface | worst |
|---|---|---|
| no actors | **100.0 %** | 0.097 m |
| with actors | 98.9 % | **2.728 m** |

That ~1 % is the pedestrians, smeared up to 2.7 m from anything real. It is why the
committed map artifact is surveyed with `with_actors:=false`, stated as a scoping
decision rather than a clean result: Phase 1's slam_toolbox has no dynamic-object
rejection, and this measures what that costs.

**Read that 0.050 m correctly.** It is the floor of the instrument, not the gap: 58
returns fell inside the 0.05 m band discarded as possible self-hits, so the pedestrian
reached the vehicle skin. It has to — Gazebo actors are not physics entities, they
never yield, and the robot had already stopped. This is a clearance measurement, not a
collision check, and no "zero collisions" claim is made from it. Classification is
LiDAR-only: a return is dynamic when no rack, ramp or plateau in the world file
explains it. The reconstructed actor trajectory only times the encounter and
cross-checks it, and its residual against the LiDAR is reported (median 0.25 m, p95
0.76 m) rather than assumed away.

**Two Phase 0 defects, found while planning Phase 1** (commit `79d01a5`) — both
invisible to Phase 0's own verification, and that is the point:

1. **Both robots spawned inside racks.** `fleet.yaml` put amr1's entire footprint
   inside `rack_south_1`. Phase 0 could not have caught it: both smoke tests override
   the spawn pose, and the fleet-launch check confirmed topics and TF without ever
   asking whether the robot was in free space. A check that only asks "did the graph
   come up" cannot see a robot embedded in a solid.
2. **The frame origin was not the rotation centre.** A differential drive pivots about
   its axle and Gazebo's DiffDrive integrates wheel encoders alone, so the pose it
   publishes *is* the axle midpoint — but the wheels sat 0.14 m aft of
   `base_footprint`. Phase 0 could not have caught this either: a constant frame
   offset cancels exactly under pure translation, and smoke test 2 drives in a
   straight line, so it measured −0.000 m and saw nothing. Rotation exposes it — a
   commanded in-place turn gave 0.0000 m of wheel odometry against 0.2428 m of true
   displacement. The consequence carried into Phase 1: the footprint is asymmetric,
   x ∈ [−0.21, +0.49], and every clearance here is measured from that polygon.

**Three more defects, found by running Phase 1 rather than by reading it:**

3. **slam_toolbox publishes its map on the absolute `/map`,** which escapes the
   namespace — `ros2 topic info -v /map` names `slam_toolbox` in namespace `/amr1`.
   Same class of trap as its shipped `/scan` default, but silent until something
   consumes the map: the map saver subscribed to `/amr1/map`, found no publisher, and
   timed out in a way that reads like a slow map. Phase 3 would have had two robots
   overwriting one global map with no error anywhere. Remapped in `slam.launch.py`;
   the global costmap's `map_topic` is now stated explicitly rather than left to
   relative-name resolution inside `/amr1/global_costmap/…`.
4. **The LiDAR was mounted inside the chassis and saw its own wheels.** At
   `lidar_height` 0.20 m with `wheel_radius` 0.10 m the scan plane lay exactly on the
   wheel tops: 18 beams per scan returned 0.44–0.49 m off the robot itself, at
   base-frame (0.00, ±0.22). slam_toolbox mapped them as static obstacles at the
   robot's own position, inflation grew them into a lethal blob around the start cell,
   and **the global planner failed every attempt** — "Failed to create plan with
   tolerance of: 0.500000", zero paths published, before the robot had moved. The
   sensor now sits on a mast at 0.35 m, above the chassis. Re-measured: 0 self-hits in
   206 scans, real-world returns unchanged. Both smoke tests were re-run at the new
   height: actors are still visible (91.7 % of scans, LETHAL in the costmap) and the
   phantom ramp wall moved to 2.49 m past the toe, still far outside `d_safe`
   (0.48 m). `tests/test_fleet_parameterization.py` now asserts the scan plane clears
   both the wheel tops and the chassis, for every robot in the fleet.
5. **slam_toolbox's map publication is subscriber-gated.** With Nav2 absent — the
   survey's configuration — nothing subscribes, so nothing is published, and its
   TRANSIENT_LOCAL publisher has no sample to hand the map saver. Three minutes of
   driving produced no artifact, twice. `survey_drive.py` now holds a subscription
   open for the whole run and logs the map size at save time, so the artifact does not
   depend on who else happens to be listening.

**Open, and flagged rather than silently worked around: MPPI will not drive this
robot.** It loads cleanly and then commands 0.0034 m/s; its own optimal trajectory
advances 0.4 mm per step; 1.06–1.10 m driven in 154 s, all of it recoveries; goal
ABORTED. Everything it consumes was verified good on the same runs — the global plan
(446 poses, 11.17 m), MPPI's own `transformed_global_plan` (1.75 m), a completely free
local costmap, the correct footprint polygon, all 8 critics loaded with the configured
weights, and every parameter reading back correctly. The same stack drives 63.6 m
under the survey node and reaches this goal in 20.5 s under RegulatedPurePursuit.
`consider_footprint` and `regenerate_noises` were bisected and ruled out. **Phase 1
therefore ships with RegulatedPurePursuit, which is a departure from PLAN.md §1 and
needs a decision** — the MPPI block is preserved verbatim in `nav2_params.yaml` as
`FollowPath_mppi_deferred`, so reverting is a rename of two keys. The largest untested
deviation from stock is the acceleration bound (`ax_max` 0.4 against stock 3.0), which
comes from `fleet.yaml` because PLAN.md asks the planner's model to match the plant.

**Carries forward:**
- **Decision needed:** MPPI vs RegulatedPurePursuit, per the paragraph above.
- `lidar_height` is 0.35 m and is now constrained, not free: it must clear
  `wheel_radius + base_height`. PitchGate's Phase 2 sizing comes from the re-run smoke
  test 2, not the Phase 0 numbers.
- The clearance floor is the 0.05 m self-return discard band. Now that self-returns
  are geometrically impossible, Phase 2 can lower it and measure the true minimum.
- `world_geometry.static_boxes` returns the ramp's *buried* extension, so a dynamic
  obstacle in x ∈ [0.96, 2.44] would be scored static. Phase 1 never goes there;
  Phase 4 works on the ramp and will. Pinned by `tests/test_world_geometry.py`.
- Nav2's `map_saver_cli` needs a TRANSIENT_LOCAL publisher, and gives an identical
  timeout for a missing publisher, a QoS mismatch and a slow map. Phase 3 saves two
  maps and a merged one; it should not learn this a third time.

---
