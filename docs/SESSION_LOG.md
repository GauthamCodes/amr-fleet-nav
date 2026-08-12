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
- **DECIDED (start of Phase 2): RegulatedPurePursuit ships; MPPI is deferred to Phase 5.**
  Recorded in `docs/PLAN.md` §1 as a V2.2 deviation rather than left as a silent departure
  from the plan. The reasoning is that the leading hypothesis — `ax_max = 0.4` against a
  stock 3.0, flattening the sampled rollout set until the optimum collapses toward zero — is
  a question about what acceleration the plant can actually deliver, and Phase 5 is where
  payload-adaptive accel/jerk limits get built and `ax_max` gets derived rather than assumed.
  Re-testing MPPI now would mean tuning it against a number Phase 5 is going to change.
  `FollowPath_mppi_deferred` is left byte-identical — not renamed, not retested.
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

## Phase 2 — SensorBSP + SafetyGate — 2026-08-13

**Built:**
- `amr_bsp`: `validators.py` (ImuValidator / LidarValidator / CameraValidator as pure
  functions, no ROS imports), `pitch_gate.py` (per-beam truncation geometry),
  `sensor_bsp.py` (the relay, counters, diagnostics, latency histogram), `bsp_node.py`
  (composition + the pitch ring buffer), `topics.py` (the one topic contract).
- `amr_safety`: `safety_gate.cpp` (C++, in the command path), header-only
  `safety_model.hpp` / `footprint.hpp`, and the Python sizing mirror `safety_model.py`
  that derives `k` from fleet.yaml and is what `tests/` exercises.
- Plant: `drive_watchdog.py` models a motor-controller command watchdog; DiffDrive
  now listens on `cmd_vel_plant` so `cmd_vel` belongs to the gate alone.
- Five evidence launches in `amr_bringup` (`phase2_pitch_gate`, `phase2_validation`,
  `phase2_stopping_sweep`, `phase2_safety_run`, `phase2_failclosed`), each
  self-shutting-down, each writing to `results/`.

**Verified:** `colcon build --symlink-install` 9 packages / 0 errors, `pytest` **110
passed** (39 pre-existing + 71 new), flake8 + black clean.

**EXIT — PitchGate suppresses the ramp phantom return** (`results/phase2_pitch_gate.*`):

| | |
|---|---|
| raw/validated pairs matched by EXACT header stamp | **903**, 0 restamped, max difference **0 ns** |
| max pitch | 8.001°, nose-UP (so the TAIL beams strike the ground) |
| predicted ground intersection `h/sin\|θ\|` | 2.514 m |
| gate radius at that pitch (0.9×) | 2.263 m |
| beams truncated in that scan | **120** of 360 |
| closest return removed | **2.769 m** — the phantom, against the 2.79 m sizing figure |
| closest return kept, before and after | 2.732 m — the real rack, untouched |
| truncated over the whole run | 24 833 beams, 7.65 % of all beams seen |
| cut pointing uphill / inside its own gate / inside the braking envelope | **0 / 0 / 0** |

The stamp match count is the rule-4 evidence and not bookkeeping: raw and validated
scans are paired by exact stamp, so a restamp would show up as zero matched pairs
rather than as plausible numbers computed against mismatched data.

**EXIT — injected implausible IMU angular velocity** (`results/phase2_imu_injection.md`).
50 rad/s on `w_z` against a 4.0 rad/s bound, injected by a separate process so the BSP
ships with no fault-simulation code in it. 500 injected → **500 rejected, 0 reached
`validated/imu`** (peak `|w|` there: 0.001 rad/s), while **3194** healthy samples were
accepted around the window. Both halves matter — a validator that rejected everything
would produce an equally impressive rejection count.

An unplanned interaction, and a good one: with the IMU stream rejected wholesale,
PitchGate lost its attitude and republished scans **untruncated** with a throttled
WARN — the degraded-but-safe path its docstring specifies, exercised for real.

**EXIT — SafetyGate halts on a pedestrian encounter** (`results/phase2_safety_suppressed.*`).
Goal SUCCEEDED. 3 halts, and **0 commands left the gate while latched** — counted
inside the gate, comparing the twist about to be published against the latch at that
instant, because an external probe sees the latch only through 10 Hz diagnostics and
misreads a legitimate post-release command as a leak.

| sensor stamp → zero command published | mean **8.46 ms** / p95 **11.00 ms** / max **12.00 ms** |
|---|---|
| in-node compute (steady clock) | 151 / 226 / 711 µs |

Low-latency safety override — not hard real-time. The end-to-end figure is quantised
by Gazebo's `/clock` step under `use_sim_time`; the compute figure is not.

**EXIT — stopping distance across four speeds** (`results/phase2_stopping_distance.*`).
Commanded straight at the gate's input, approaching a rack face, so the gate is the
only thing that can stop the robot:

| cmd m/s | v at halt | d_safe | clearance at halt | braking | settled |
|---|---|---|---|---|---|
| 0.15 | 0.130 | 0.342 | 0.334 | 0.010 | 0.331 |
| 0.30 | 0.277 | 0.469 | 0.456 | 0.047 | 0.401 |
| 0.45 | 0.426 | 0.680 | 0.653 | 0.242 | 0.402 |
| 0.60 | 0.577 | 0.975 | 0.928 | 0.521 | 0.384 |

`k_model` = **1.8750** s²/m against `k_measured` = **1.0208** s²/m — a factor of 1.84.
Both are quoted because they measure different things: Gazebo's DiffDrive applies
`max_decel_x` as a KINEMATIC limit, so the simulated robot brakes as if unloaded,
while `k_model` sizes the envelope for a 90 kg vehicle that cannot ignore its 60 kg of
payload. The "settled" column is the `d_min` standoff the robot creeps to with the
command still applied, which is the specified end state of a speed-dependent gate, not
a leak in it.

**EXIT — fail-closed, SIGKILL mid-motion** (`results/phase2_failclosed_*.md`). SIGKILL
rather than SIGTERM, so no shutdown handler runs. Upstream keeps commanding throughout:

| | distance after the kill | outcome |
|---|---|---|
| plant-side watchdog **present** | **0.196 m** | at rest in 0.75 s |
| watchdog **absent** (control) | **3.500 m** | **still rolling at 0.350 m/s** when the 10 s window closed |

That is the whole argument for rule 1 as a measurement rather than an assertion.
gz-sim 8.11.0's DiffDrive has no command timeout, so being the only publisher of
`cmd_vel` stops new commands but not the latched one; the watchdog models the motor
controller that does. The control needed one correction to be meaningful — with the
watchdog simply removed, nothing bridges `cmd_vel` to `cmd_vel_plant` and the robot
never moved at all (peak 0.000 m/s), which measures a severed command path and
nothing else. The gate now addresses the plant directly in that case.

**EXIT — no Nav2 recovery during a halt, and an honest reading of the A/B.**
Suppressed run: 11 recovery behaviours over the run, **0 during any halt**, with
`progress_checker.movement_time_allowance` read back from `controller_server` as
1 000 000 s at every halt entry and release. Control run (`suppress_recovery:=false`):
allowance read back as 10 s throughout, 3 recoveries over the run, and **also 0 during
a halt**.

**So the A/B does not prove the mechanism was needed, and it would be an overclaim to
say it does.** The longest halt in either run was 0.80 s against a 10 s allowance, so
the progress checker was never going to fire during one. What the runs DO establish is
that the mechanism operates end to end — the parameter is written on entry, read back
changed, and restored after the robot has moved. Exercising the necessity needs a halt
longer than `movement_time_allowance`, which this world's walking pedestrians cannot
produce because they clear the sector in under a second. Carried forward.

**Surprises — two, and both were wrong conclusions rather than wrong code:**

1. **The "camera stalls the drive" finding from the previous session is RETRACTED.**
   It was recorded as: camera at 160×120/10 Hz → commanded 0.35 m/s, achieved
   0.00017 m/s; camera absent → 0.35 m/s. Re-measured this session on a verified-clean
   process table, one variable, everything else identical:

   | | peak measured speed | distance in the window |
   |---|---|---|
   | camera **enabled**, driving | **0.3500 m/s** | 13.53 m |
   | camera **disabled**, driving | **0.3500 m/s** | 13.86 m |

   Identical. The real cause was **four `drive_watchdog` processes leaked from earlier
   launches**, still subscribed to `/amr1/cmd_vel` and republishing onto
   `/amr1/cmd_vel_plant`. The contention starved the graph until the gate's own
   command-timeout fail-safe fired and interleaved zeros into the command stream — 663
   zeros in 884 messages, measured — so DiffDrive's acceleration limit never
   integrated. The camera merely happened to be the variable under test when the leak
   was present. The same signature reappeared this session with **no camera in the
   world at all**, which is what exposed it: the first stopping sweep reported NO HALT
   at all four speeds because the robot never moved.

   The camera stays defaulted **off**, but for a different and smaller reason: nothing
   downstream consumes it, so it is load a run does not need. That is a cost decision,
   reversible by one xacro argument, not a defect workaround. `results/phase2_camera.md`
   records 369 frames validated with the robot stationary; `phase2_camera_motion_*.md`
   record the A/B above.

2. **A process-hygiene failure is a measurement failure.** Every launch in this repo
   can leave nodes behind, and a leaked node republishing onto a live topic produces
   results that look like code defects and get written down as findings. The cleanup
   before each run is now exhaustive rather than targeted, and the PitchGate probe
   already reports `raw scans whose stamp did NOT advance` for exactly this reason —
   that counter was added after a stray `gz sim` contaminated a run last session, and
   it is the reason this session's PitchGate numbers can be trusted.

**Three defects found by running Phase 2 rather than by reading it:**

3. **`bt_log_available` was reset to `False` after the subscriber set it**, a few lines
   further down `__init__`. The safety report printed "behaviour-tree log unavailable
   on this Nav2 build" while subscribed to it — the required recovery evidence silently
   absent from a run that had collected it.
4. **The gate's RELEASE log printed the wrong clearance.** It logged the sector
   minimum, but the latch releases on the value it was actually given, which is
   infinity when the upstream has stopped commanding translation. The line read
   "RELEASE: clearance 0.121 m > d_release 0.717 m", which is arithmetic nonsense. It
   now prints the latched value and names the sector value separately.
5. **NavFn intermittently aborts the goal** with "Failed to create a plan from
   potential when a legal potential was found" while the map is still filling in ahead
   of the robot. The safety run now retries a bounded number of times and reports the
   dispatch count, so a mapping transient does not discard the encounter — and a stack
   that genuinely could not navigate would still show up as a retry count.

**Carries forward:**
- **For the README's known-limitations section, in this wording:** the camera is
  present and validated but defaulted off because no component consumes it; on real
  hardware a camera runs its own pipeline and the question does not arise. Do NOT
  describe it as a simulator resource limit — that claim was measured and disproved.
- The recovery-suppression A/B needs a halt longer than `movement_time_allowance`
  (10 s) to demonstrate necessity rather than operation. A static obstacle parked in
  the path would do it; the walking pedestrians will not.
- `k_measured` (1.02 s²/m) is a property of the simulator's kinematic decel, not of the
  safety model. Phase 5 builds payload-adaptive accel/jerk limits and should re-run the
  stopping sweep against them rather than against `max_decel_x` alone.
- The gate zeroes the ENTIRE twist while blocked. Allowing in-place rotation under a
  hold is a Phase 7 refinement, deliberately not assumed here.
- `min_gate_range` is 1.565 m for amr1 and binds only above ~13° of pitch; on this
  warehouse's 8° ramp the unclamped gate (2.263 m) already clears the braking envelope.
  The interlock is held in reserve, not leaned on — pinned by `tests/test_safety_distance.py`.

---
