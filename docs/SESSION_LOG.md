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

## Phase 3 + 4 (merged) — Cooperative mapping and the fleet frame — 2026-08-13

Scoped by `docs/DELIVERY_PLAN_COMPRESSED.md`, Session A. The ramp A/B moves to
Session B by decision, not by overrun: a genuine "flat route available versus
blocked" needs the reserved second ramp built and a fresh two-route survey. What
ships here instead is the ramp filter **plumbing**, wired and measured at zero cost,
so Session B authors only the mask, the ramp and the runs.

**Built:**
- **Bringup split into three.** `amr_gazebo.spawn.world_actions` owns the world
  singletons (rendered SDF, `gz sim`, `/clock` bridge); `robot_stack.launch.py` owns
  one robot's BSP → SLAM → Nav2 → SafetyGate with a `stagger` offset;
  `fleet_nav.launch.py` loops over `load_fleet()`. `amr1_nav.launch.py` is now a thin
  wrapper over the same two pieces and is still strictly single-robot.
- `amr_fleet_control`: `fleet_grid.py` (grid geometry and the merge rule, pure
  functions), `selective_policy.py` (the scored accept/defer rule, pure functions),
  `fleet_map_node.py` (the node), `fleet_mission.py` (concurrent goals).
- `amr_navigation`: `ramp_mask.py` generates the filter mask at launch time;
  `costmap_filters.launch.py` brings up the fleet-wide `map_server` +
  `costmap_filter_info_server` + their own lifecycle manager.
- `scripts/clean_processes.sh` — kills the simulation and ROS graph, then **prints
  the surviving process table** and exits non-zero if anything survived.
- `warehouse.sdf.xacro` gains an optional static barrier (`with_static_obstacle`,
  default off, so no earlier run's world changed).
- Two Phase 1/2 consumers corrected for the new plan frame: `nav_goal_run._on_plan`
  branches on `header.frame_id` instead of always applying the spawn offset, and
  `map_report` gained `--world-frame`. `safety_run` gained a `title` parameter so the
  barrier A/B is not filed under "pedestrian encounter"; its default is unchanged, so
  Phase 2's own artifacts still read as they did.
- `ws.sh` merges a CycloneDDS config that raises the participant-index ceiling, and
  `src/amr_bringup/config/cyclonedds.xml` documents why.

**Verified:** `colcon build --symlink-install` 9 packages / 0 errors, `pytest`
**171 passed** (110 pre-existing + 61 new), flake8 + black clean.

**EXIT — two robots, one graph.** 53 nodes: two Nav2 stacks, two `slam_toolbox`, two
SensorBSP, two SafetyGates, the plant per robot, and the fleet-wide map and filter
servers. No node-name collision, no topic collision. Every lifecycle node reached
`active`, including both filter servers. Every per-robot difference still comes from
`fleet.yaml`; there is no `if robot ==` anywhere.

**EXIT — the fleet map drives planning, and the wiring is checkable.**

| | |
|---|---|
| `/fleet_map` publisher QoS | RELIABLE · TRANSIENT_LOCAL · KEEP_LAST(1) |
| subscribers matched | **2** — `/amr1/global_costmap` and `/amr2/global_costmap` |
| grid | 680 × 400 cells at 0.05 m, origin (−15.0, −10.0) |
| `fleet_map → amr1/map` | (−11.000, −1.500, 0.000) |
| `fleet_map → amr2/map` | (−11.000, +1.500, 0.000) |
| both global costmaps | `global_frame: fleet_map`, 34 × 20 m, `filters: ['ramp_filter']` |
| both local costmaps | still `amrN/odom` |
| `slam_toolbox` | still owns `amrN/map` |

Three distinct frames per robot, which is exactly what a `RewrittenYaml` key rewrite
could not have expressed — hence the new `__GLOBAL_FRAME__` placeholder rather than
repointing `__MAP_FRAME__`.

**EXIT — selective update policy** (`results/phase3_selective_updates.*`), measured
during the concurrent-goal run so both robots were actually exploring:

| | |
|---|---|
| candidates scored | 41 |
| accepted (merged and published) | 23 |
| **deferred** | **18 — 43.9 %** |
| per robot | amr1 13/8 (38.1 % deferred), amr2 10/10 (50.0 %) |
| composites performed | 17 |
| mean composite + publish | 0.55 ms |
| fleet map at end | 15.6 % known, 1 984 occupied cells |

The deferred **share** is the honest headline. The milliseconds saved are small and
the report says why rather than letting the number oversell: compositing is numpy
slice arithmetic on a fixed grid and was never the expensive part. What deferral
actually bounds is how often a 680 × 400 grid is serialised to two global costmaps
that each reprocess it — work that happens inside the Nav2 processes and is not
measured here.

**EXIT — concurrent goals to both robots** (`results/phase3_concurrent_goals.*`).
Dispatched in the same pass, not staggered:

| | amr1 | amr2 |
|---|---|---|
| result | **SUCCEEDED** | **SUCCEEDED** |
| time to goal | 18.8 s | 11.3 s |
| planned / driven (ground truth) | 10.50 / 10.44 m | 10.50 / 10.46 m |
| final position error | 0.062 m | 0.042 m |
| replans (route actually changed) | 0 | 0 |

Closest approach 3.000 m over 2 180 samples, median 3.471 m. amr2 is the faster
chassis by configuration (`max_vel_x` 1.00 against 0.60) and arrives first, from the
same YAML that shapes its URDF. The routes are deconflicted by design — the forced
conflict, mutual local deviation and the yield protocol are Phases 6 and 7.

**EXIT — ramp filter plumbing, contributing provably nothing.** The KeepoutFilter
loads into both global costmaps through the `filters:` list and both costmaps
activate with it:

| | amr1 | amr2 |
|---|---|---|
| known cells | 272 000 | 272 000 |
| free cells (cost 0) | 248 036 | 249 230 |
| costed cells (cost > 0) | 23 964 | 22 770 |
| **minimum cost over known cells** | **0** | **0** |

The measurement that settles it is the minimum, not the ramp region. The first
attempt tested "cost inside the ramp footprint is zero" and it was circular twice
over: most of that region is unexplored, and the part the LiDAR does reach contains
the real plateau face, which reads LETHAL from the static layer whatever the filter
does — it reported `ramp max = 100` and failed its own check. Because the Phase 3
mask is **uniform**, a filter adding cost would raise every cell together and no cell
could read 0. That one number distinguishes "loaded and contributing nothing" from
both "never loaded" and "silently costing the whole warehouse".

**EXIT — the Phase 2 carry-over: recovery suppression, finally proven NECESSARY**
(`results/phase2_recovery_ab_{suppressed,control}.*`). Phase 2 could only show the
mechanism *operates*; its longest halt was 0.80 s against a 10 s allowance, so the
progress checker was never going to fire and the control run's zero proved nothing.
That was recorded honestly and carried forward. The blocker was the obstacle: walking
pedestrians clear the forward sector in under a second.

An immovable barrier spanning the full 5.5 m aisle fixes it. The gate zeroes the
entire twist while blocked, so the robot cannot rotate away, clearance cannot grow,
and the latch does not release on its own:

| | suppressed | control (`suppress_recovery:=false`) |
|---|---|---|
| halts | 4 | 8 |
| **longest halt held** | **56.69 s** | **10.00 s** |
| every halt after the approach | one continuous hold | capped at 9.4–10.0 s each |
| `movement_time_allowance` read back | 1 000 000 s at every transition | 10 s at every transition |
| `controller_server: Failed to make progress` | **0** | **6** |
| recovery behaviours, whole run | **0** | **2** |
| **recovery behaviours fired DURING a halt** | **0** | **2** (Spin ×1, Wait ×1) |
| commands leaked past the latch | 0 | 0 |

That is the necessity result. With the allowance left at its default, Nav2 declares
the robot stuck **while the safety gate is deliberately holding it**, breaks the halt
roughly every ten seconds, and dispatches recovery behaviours into a robot that must
not move — the exact failure docs/ENGINEERING_NOTES.md rule 2 exists to prevent, now measured rather
than asserted. Run twice; the second run reproduced it (62.99 s versus 8.50 s on the
first, same structure).

Note which number is the discriminator. It is NOT the raw recovery count: in the
control arm the progress-checker failure and the halt release happen at the same
instant, so a recovery can land just outside the attributed window. The robust
signals are the halt DURATION (one 56.69 s hold versus eight ~10 s fragments) and the
six `Failed to make progress` errors, which appear only in the control arm.

**A limitation this run makes visible, and which belongs in the README.** A full-twist
hold against an obstacle that never moves is a deadlock: the robot has no legal action
that could increase its clearance, so the goal ends in TIMEOUT in both arms. That is
the correct behaviour for a fail-closed gate and it is also why "allow in-place
rotation under a hold" is already flagged as a Phase 7 refinement. This run is the
evidence for why that refinement is worth doing.

**REGRESSION — Phase 1 still behaves the same after the re-frame**
(`results/phase1_nav_regression.*`). The whole point of checking: both global costmaps
moved into a new frame and the bringup was restructured underneath three existing
evidence launches. Same goal, same world, actors off:

| | Phase 1 baseline | after Phase 3 |
|---|---|---|
| goal result | SUCCEEDED | **SUCCEEDED** |
| time to goal | 20.5 s | **20.4 s** |
| planned path (first plan) | 11.16 m | 11.13 m |
| executed path (ground truth) | 11.01 m | **11.00 m** |
| executed / planned | 0.99 | 0.99 |
| final position error | 0.079 m | 0.065 m |
| recovery behaviours | 0 | 0 |
| **replans (route actually changed)** | **5** | **1** |

Everything matches except the replan count, and that difference is not noise worth
waving away. The global costmap's static layer now reads the fleet map, which is
republished at most once a second and only when the selective policy accepts an
update — where it previously read slam_toolbox's raw stream directly. Fewer changes to
the static layer means fewer recomputed routes. Less plan churn for the same executed
path is a reasonable outcome, but it is a behavioural consequence of selective
updating rather than a free improvement, and it is recorded as one.

**Surprises — five, and two of them were nearly written down as code defects:**

1. **A second robot exhausts CycloneDDS's participant indices, and the error blames
   the wrong thing.** Every ROS 2 node is a DDS participant and the default
   `MaxAutoParticipantIndex` is 9. One robot fits; two do not — this graph is 53
   nodes. Past the limit, node creation throws:

   ```
   [controller_server] Failed to find a free participant index for domain 0
   [rmw_cyclonedds_cpp] rmw_create_node: failed to create domain, error Error
   terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
   ```

   Six of amr2's Nav2 servers died this way while amr1 came up perfectly, which reads
   exactly like a namespacing bug in the second robot's stack. It is a host-wide
   limit and has nothing to do with namespaces. `src/amr_bringup/config/cyclonedds.xml`
   raises it to 120 — room for ten robots, so the assignment's scaling requirement
   does not become a second environment problem to rediscover.

2. **The obvious way to apply that fix does nothing at all.** This environment already
   exports a `CYCLONEDDS_URI` — an inline fragment pinning discovery to the loopback
   interface. Writing `export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://...}"` looks
   correct, reads correctly, and is a no-op, because the variable is already set. The
   inverse (plain assignment) would have silently dropped the interface pinning
   instead. CycloneDDS merges a comma-separated list of URIs, so `ws.sh` APPENDS. Both
   failure modes are silent; only the merged form is right.

3. **The first weights for the selective policy re-created the bug the policy exists
   to prevent, and a test caught it.** With `w_change 0.8` against `w_revisit 0.6` and
   a 0.35 threshold, a NEW OBSTACLE appearing on a corridor the robot had already
   driven six times scored 0.20 and was deferred — which is precisely the failure a
   visit-count-only policy has, reintroduced through a weight choice. The fix is an
   invariant rather than a retune: every positive weight must exceed
   `w_revisit + accept_threshold`, so a saturated frontier push, a saturated occupancy
   change or a fully aged deferral each earns a merge on its own even under the
   maximum penalty. `tests/test_selective_policy.py` asserts it directly, so a later
   retune that breaks it fails a test instead of quietly dropping obstacle updates.

4. **Moving the plan's frame silently corrupted a Phase 1 artifact while leaving
   every number in it correct.** `/plan` is published in the global costmap's frame,
   so it changed from `amr1/map` to `fleet_map` — and `nav_goal_run._on_plan` was
   still applying the spawn-pose conversion on top. The regression run's plan CSV
   starts at **(−22.00, −3.00)** where the robot truly starts at (−11.00, −1.50):
   the offset applied twice, putting the drawn route through a rack.

   What makes this worth recording is that it does not look like a failure. Every
   derived metric is translation-invariant, so path length, deviation and the replan
   count all remained correct and the report beside the corrupted CSV read perfectly
   fine. bt_navigator also transforms incoming goals into its own frame, so goal
   DISPATCH kept working untouched — the break is entirely on the read-back side.
   `_on_plan` now branches on `msg.header.frame_id`, and `map_report` gained a
   `--world-frame` flag so a saved fleet map is not scored against geometry 11 m away
   for the same reason.

5. **The obvious test of "the null mask adds no cost" is circular, and it failed its
   own check.** Measuring cost inside the ramp footprint reported `ramp max = 100` and
   a FAIL — not because the filter was writing cost, but because most of that region
   is unexplored and the part the LiDAR reaches contains the real plateau face, which
   reads LETHAL from the static layer whatever the filter does. Worse, the unexplored
   remainder reads unknown, which would make a filter that never loaded look identical
   to one contributing nothing. The measurement that decides it exploits the mask being
   uniform: the minimum cost over all known cells. It is 0.

**Carries forward:**

- **Session B owns the ramp A/B**, and it needs three things this session did not
  build: the second ramp (its footprint is still reserved and commented in
  `world.yaml`), a survey covering both routes, and a graded mask. The plumbing is
  done and measured — `costmap_filters.launch.py`, the `filters:` key in both global
  costmaps, and `ramp_mask.write_mask`, which already takes regions and a value.
- **Measure this before building the A/B on it, do not assume it.** With
  `track_unknown_space: true` and NavFn's `allow_unknown: true`, unknown cells are
  already expensive. Whether a graded KeepoutFilter cost survives onto
  `NO_INFORMATION` cells lives in `keepout_filter.cpp`, which is not installed here.
  Ten minutes with `ros2 topic echo /amr1/global_costmap/costmap` over a known ramp
  cell settles it; an afternoon in the planner if skipped. If it does not survive, the
  ramp must be surveyed before the A/B, which is another argument for doing the survey
  first anyway.
- Mask values must stay **at or below 90**. 100 becomes cost 254 (LETHAL) and 253 is
  already `INSCRIBED_INFLATED_OBSTACLE`, which the footprint collision checkers treat
  as a collision. An expensive ramp is not an impassable one. `ramp_mask.mask_pixel`
  raises rather than clipping.
- The mask YAML needs `mode: scale` and a pure white background. The default `trinary`
  collapses the whole grey range to unknown, and map_saver's 205 for unknown would
  cost the entire warehouse about 48. Both are generated, not hand-written, for this
  reason.
- **The inter-map transform is fixed and uncorrected**, per the compressed plan's cut.
  If Session C revisits it: rclpy's `StaticTransformBroadcaster` is APPEND-ONLY — it
  skips a `child_frame_id` it has already sent, unlike the C++ one, so republishing a
  corrected transform through it silently does nothing. A raw TRANSIENT_LOCAL publisher
  on `/tf_static` resending the whole set is the way round it.
- `composites (17) < accepted (23)` in the selective-update report is not a bug: the
  publish tick coalesces accepts that land inside the same second, so two robots
  accepting together produce one composite.
- `amr1_nav.launch.py` must stay single-robot and must NOT delegate to
  `fleet_nav.launch.py`. `phase1_survey`, `phase1_nav_run` and `phase2_safety_run`
  include it by filename, and turning it into a two-robot launch would silently double
  the graph load under every Phase 1 and Phase 2 evidence run.
- `scripts/clean_processes.sh` prints the surviving process table and exits non-zero if
  anything is left. Run it before any measurement worth keeping — twice the robots is
  twice the leak surface, and Phase 2 has already paid once for a leaked watchdog.

---

## Phase 6 + 5 (merged) — The MAPF layer and the payload-adaptive chain — 2026-08-13

Scoped by `docs/DELIVERY_PLAN_COMPRESSED.md`, Session B. Built in the order the
requirements are worth rather than the order they are numbered:
`FleetTrajectoryLayer` first, because it is the one an evaluator probes hardest
and the one earlier phases left with nothing at all behind it. **Phase 7
(`TrafficControlNode`) and the ramp A/B did NOT land** — see *Carries forward*.

**Built:**
- `amr_costmap_plugins`: `FleetTrajectoryLayer` (C++, pluginlib) — the package's
  first real target. Deposits `max_cost·exp(-Δt/τ)` over a disc per predicted
  peer pose, combined with `std::max` so it can raise a cell's cost and never
  lower it. Plus `costmap_plugins.xml`, a library target and the pluginlib export.
- `amr_fleet_control`: `trajectory_predict.py` and `trajectory_conflict.py`
  (pure functions), `TrajectoryPredictor` (node), `TrajectoryProbe` (instrument).
- `amr_motion`: `jerk_limiter.py` (pure), `PayloadJerkAdapter`, `payload_trace`.
- `amr_safety`: `priority_mux.py` and `config/twist_mux.yaml`.
- `amr_navigation`: the `velocity_smoother` block, `peer_trajectory_topics()`,
  and two new render flags. `phase6_conflict` and `phase5_payload_trace` launches.

**Verified:** `colcon build --symlink-install` 9 packages / 0 errors, `pytest`
**223 passed** (171 pre-existing + 52 new), flake8 + black clean.

**EXIT — one robot's trajectory becomes cost in another robot's LOCAL costmap**
(`results/phase6_cost_injection_*`). Forced crossing, each robot's goal in the
other's starting lane. Both arms run back to back on a verified-clean table:

| | layer ON | layer OFF |
|---|---|---|
| samples, peer's predicted cell inside the window | 50 | 59 |
| **samples with cost > 0** | **50 (100 %)** | **0 (0 %)** |
| median cost at that cell | **145** | 0 |
| max cost at that cell | 240 | 0 |
| cost the decay model predicts | 125.9 | 127.3 |

The probe samples where amr1 is predicted to be **2 s from now**, not where it
is. That distinction is the measurement: amr1 is a physical object amr2's LiDAR
marks and inflates regardless, so sampling the peer's current cell would measure
the obstacle layer and report it as MAPF. The layer-off arm is what makes the
layer-on number mean anything, and it reads 0 at the same cells in the same
scenario.

**The behavioural difference between the arms is NOT claimed.** amr1 drove
8.30 m in 18.0 s with the layer and 9.03 m in 20.6 s without; closest approach
1.610 m against 1.529 m. That is n=1 per arm, and an earlier layer-on run gave
1.538 m — inside the same spread. The cost-injection measurement is the claim.

**A limitation that belongs in the README, in this wording.**
RegulatedPurePursuit is not a sampling optimiser. It consumes local costmap cost
through cost-regulated velocity scaling and forward collision checking, so the
layer changes how it *paces*, but it does not deviate laterally around graded
cost the way MPPI's CostCritic would. PLAN.md §5's Phase 6 demo — "robots
mutually deviate without central intervention" — is therefore **not**
demonstrated, and no run in this repo should be read as showing it. What is
demonstrated is the mechanism and the cost injection, measured against a control.

**EXIT — payload-adaptive velocity and jerk** (`results/phase5_payload_trace.*`).
A velocity STEP into the chain input, both robots, both payload states:

| robot | payload | peak v cmd | peak a cmd | peak jerk cmd |
|---|---|---|---|---|
| amr1 | unloaded | 0.500 | 0.725 | 1.784 |
| amr1 | loaded | 0.500 | 0.272 | 0.634 |
| amr2 | unloaded | 0.500 | 1.112 | 4.722 |
| amr2 | loaded | 0.500 | 0.940 | 2.742 |

amr1's peak commanded acceleration falls **×0.38** loaded against amr2's ×0.85,
because 60 kg on a 30 kg chassis is a far larger perturbation than 5 kg on 18 kg
— and no code distinguishes them. Peak commanded velocity is exactly 0.500 in
all four cases.

**Read the jerk column honestly.** It exceeds the configured bound (1.000,
0.333, 2.500, 1.957 m/s³ respectively) by up to ~1.9×. The bound holds on the
limiter's own recursion — `tests/test_jerk_limiter.py` asserts it for every
robot at both payload states — so what the column measures is the *published
stream*: a 20 Hz signal timestamped on arrival, resampled onto a 50 ms grid and
differentiated twice, plus a single-step transient where the velocity arrives at
its target. What this run supports is the payload ratio and the absence of
overshoot, not a certified jerk ceiling. Closing that gap means timestamping at
the publisher, and it is not done.

**Surprises — five, and three were in this session's own instruments:**

1. **Stock `twist_mux` does not load in this ROS snapshot.**
   `ros-jazzy-twist-mux` 4.5.0 (built 2026-06-15) resolves `diagnostic_updater`'s
   `Updater(...)` ending `EEdh` — double, unsigned char. The installed
   `ros-jazzy-diagnostic-updater` 4.2.6 (2026-04-12) exports `...EEd`. Two
   packages from one apt snapshot, binary-incompatible with each other, and
   nothing in this workspace can repair it. `amr_safety/scripts/priority_mux.py`
   preserves the architecture — a priority mux between the motion chain and the
   fail-closed gate — and reads the *same* `twist_mux.yaml` schema, so restoring
   the stock node is a two-line launch change. Recorded rather than descoped.
2. **`twist_mux` defaults to `TwistStamped`, and says so exactly once.** It logs
   `"use_stamped" is not declared as parameter, defaulting to "true"`. Every
   other link in this chain is plain `Twist`. Left undeclared, the mux would have
   subscribed to a type nothing publishes: no error, no warning past that line,
   and a robot that does not move. `enable_stamped_cmd_vel: false` is stated
   explicitly on `velocity_smoother` for the same reason.
3. **The velocity limiter overshot, and a unit test caught it before any run.**
   Bounded jerk means acceleration cannot be removed instantly either, so a
   clamp-only limiter sails past the commanded speed by `a²/(2j)` — 0.25 m/s at
   a = 1.0, j = 2.0. This node is DOWNSTREAM of the stock smoother's velocity
   clamp, so that overshoot would have reached the wheels while every other
   component believed `max_vel_x` was being held.
4. **The continuous-time S-curve is the wrong formula for a discrete loop.**
   `sqrt(2j|e|)` is right in continuous time and leaves acceleration on the books
   at arrival, which the velocity clamp then absorbs in a single step: measured
   jerk spiked to **2.9–3.5× `jerk_max`** at the instant the command reached its
   target — precisely the transient a jerk limiter exists to remove. The exact
   discrete form, `-jΔt/2 + sqrt((jΔt/2)² + 2j|e|)`, brings the worst case to
   ~1.1× on the recursion. Found by reproducing the simulator's number offline in
   a 20-line script, which cost nothing and made the fix iterable.
5. **A first hypothesis that was wrong, and saying so.** The inflated derivatives
   were first attributed to transport jitter and "fixed" by resampling onto a
   uniform grid. Re-running produced *identical* numbers, which is what proved
   the hypothesis wrong and sent the search to the arrival transient. The
   resampling is correct and was kept; it simply was not the cause. A fix that
   changes nothing is evidence, and it was treated as such rather than assumed to
   have worked.

Also: `install(PROGRAMS)` cannot set the executable bit through a
`--symlink-install` symlink, so a new script without `chmod +x` fails as
"executable not found on the libexec directory".

**Carries forward:**

- **Phase 7, `TrafficControlNode`, is NOT built.** This is the one requirement on
  Session B's list with no implementation. Its plumbing is in and measured:
  `cmd_vel_yield` is a priority-150 input on every robot's mux, the release
  semantics are timeout-based so an arbiter that dies releases rather than pins,
  and `trajectory_conflict.py` — the conflict predicate the arbiter would use —
  is written and unit-tested. What is missing is the node itself, its recovery
  suppression on yield entry, and the forced-conflict evidence run.
- **The ramp A/B is NOT run, but its blocking question is settled and cost
  nothing.** Nav2 Jazzy's `KeepoutFilter::process()` combines with
  `if (data > old_data || old_data == NO_INFORMATION)`, so graded mask cost IS
  written onto unknown cells. The question carried forward from Phase 3+4 is
  answered from the source rather than by measurement: the ramp does **not** need
  surveying first. A two-route topology also already exists in the world — the
  aisle is clear for |y| < 2.75 and the ramp occupies |y| ≤ 1.25 — so the A/B can
  be a plan-only comparison of published global plans, with no driving at all.
  Arm B needs the existing `with_static_obstacle` barrier split by a gap argument,
  to block the flat lanes while leaving the ramp corridor open.
- **Phase 1/2/3 evidence predates the motion chain** and was not re-run. Those
  artifacts were measured on `cmd_vel_nav -> SafetyGate`; the chain now sits
  between them. `with_motion_chain:=false` reproduces the old path exactly if a
  like-for-like re-measurement is ever wanted.
- The `phase6_crossing_*` reports come from `fleet_mission`, which now takes
  `stem_prefix`, `title` and `separation_note`. All three default to the Phase 3
  text, so Phase 3's own artifacts read exactly as they did.
- `TrajectoryPredictor` died once during shutdown of a Phase 5 run (exit 1, after
  the evidence had been written). Not diagnosed. It does not affect the Phase 5
  artifacts, which the trace node writes itself, but it should not be left
  unexamined before submission.

---

## Phase 7 — The yield protocol — 2026-08-13

The last requirement with no implementation. Phase 6 shipped the local half of
assignment §3.2 — each robot's local costmap consuming its peers' predicted
trajectories — and left the escalation half with plumbing only: a priority-150
`cmd_vel_yield` mux channel, timeout-based release semantics, and a unit-tested
conflict predicate with no caller.

**Built:**
- `amr_fleet_control/traffic_policy.py` — pure functions: priority order, the
  derived conflict radius, the escalation test, the release test.
- `traffic_control.py` (`TrafficControlNode`) and `traffic_report.py` (its
  records and evidence writer), plus `traffic_control.launch.py`.
- `fleet_nav.launch.py` gains `with_traffic_control` (default **true**,
  `TRAFFIC_START_S = 30`, after every predictor) and the arbiter's evidence
  arguments. It is skipped when there is no mux to command through
  (`with_motion_chain:=false`) or no Nav2, which is the survey configuration.
- `warehouse.sdf.xacro` gains `obstacle_gap_y`: the Phase 3 barrier splits into
  two segments around a gap. `0.0` is the solid wall, so every earlier run's
  world is byte-identical — verified by rendering both.
- `fleet_mission` gains `dispatch_offsets`; all-zero is Phase 3's simultaneous
  dispatch and is the default.
- `phase7_yield.launch.py` — the forced-conflict run.

**Verified:** `colcon build --symlink-install` 9 packages / 0 errors, `pytest`
**241 passed** (223 pre-existing + 18 new), flake8 + black clean.

**The arbitration constants are derived from `fleet.yaml`, not tuned.**

| | |
|---|---|
| priority | gross mass, ties broken by declaration order → amr1 (90.0 kg) > amr2 (23.0 kg) |
| conflict radius | **1.94 m** = r(amr1) 0.539 + r(amr2) 0.429 + d_safe(amr1 @ 0.60 m/s) 0.975 |
| release radius | 2.43 m (1.25× hysteresis) |
| escalate after | 2.0 s of conflict whose predicted closest approach is not opening |

The radius is the distance at which one robot's SafetyGate would already be
holding the other, so past it the two cannot resolve anything by driving. That
is the argument for the number; there is no tuned constant in it, and an `amr3`
with a longer stopping distance widens it by existing. **No robot is named in
the arbiter** — the assignment's "AMR-2 yields to AMR-1" falls out of the mass
column, and `tests/test_traffic_policy.py` asserts that changing the masses
changes the yield direction.

**EXIT — forced narrow-intersection conflict** (`results/phase7_yield.*`,
`results/phase7_yield_mission.*`). The Phase 3 barrier at x = −5.0 split around a
3.0 m gap on the aisle centre line; both goals east of it, so both global plans
converge on the same few metres. amr2's dispatch held back 5.0 s, because it is
the faster chassis by configuration and would otherwise be through the gap before
amr1 arrived:

| | yield 1 | yield 2 |
|---|---|---|
| escalated after | 2.0 s of unresolved conflict | 2.0 s |
| separation at escalation | 1.53 m | 1.76 m |
| predicted separation **gain** over the conflict's life | **−0.14 m** | **+0.04 m** |
| **held** | **1.0 s** | **15.2 s** |
| release condition | conflict cleared (2.43 m) | conflict cleared (2.46 m) |
| zero-twist commands on `amr2/cmd_vel_yield` | 21 | 305 |
| `movement_time_allowance` read back at entry / after release | 1 000 000 s / 10 s | 1 000 000 s / 10 s |
| **recovery behaviours during the hold** | **0** | **0** |
| SafetyGate blocking during the hold | 0 of 21 cycles | 0 of 305 cycles |

Both goals **SUCCEEDED** — amr1 in 40.0 s (12.72 m driven against a 10.00 m first
plan), amr2 in 42.9 s (13.62 m against 10.66 m), final position error 0.032 and
0.151 m. Closest actual approach 0.926 m over 5 091 ground-truth samples;
recorded separation starts at exactly 3.000 m, which is the spawn separation, so
the trace begins where the robots actually do.

**The gain column is the escalation argument.** Escalation is refused while the
predicted closest approach is opening by more than 0.15 m, because that is the
local layer resolving the conflict on its own (rule 7). Here it opened by −0.14 m
and +0.04 m — the local layer had first refusal for 2.0 s in both cases and did
not open the gap, which is exactly what a constriction with no lateral room to
deviate into looks like from the arbiter's side.

**Section D is not decoration.** A yield and a safety halt both end with a robot
at a standstill, and the arbiter records the gate's own diagnostics alongside
every hold for that reason. 0 of 326 held cycles had the gate blocking, so amr2
was stopped by the arbiter and by nothing else. The gap was sized for this: at
3.0 m a robot centred in it clears each barrier end by 1.275 m against amr1's
0.975 m braking envelope, and a tighter gap would have measured the gate.

**Surprises — three:**

1. **The 15.2 s hold is 5.2 s longer than `movement_time_allowance`.** That is
   the condition Phase 3 measured recoveries firing under, arriving here for a
   completely different reason — a yield rather than an obstacle — and with the
   robot free to move the whole time. The suppression was on and 0 recoveries
   fired. **The necessity claim still rests on Phase 3's A/B**, not on this run:
   the `suppress_recovery:=false` control arm of this launch exists and is one
   command, and until it is run this is evidence that the mechanism operates on
   the yield path, not that it was needed there.
2. **Two nodes now write the same Nav2 parameter.** SafetyGate raises
   `movement_time_allowance` on a halt and the arbiter raises it on a yield.
   Both capture the ORIGINAL value at startup rather than reading whatever is
   live at entry — otherwise whichever wrote second would latch the other's
   1e6 as its "original" and restore it forever. The arbiter also re-asserts the
   value every 2 s while holding, so a gate release mid-yield cannot hand a 10 s
   allowance back for longer than one period. Neither ordering was exercised in
   this run; the interaction is designed for, not measured.
3. **A yield that releases on a stationary robot restores on the grace timer,
   not on movement.** Hold 2's restore logged "after 0.00 m of motion": the
   conflict cleared while amr2 was still stopped and it had not begun
   accelerating when the 2.0 s grace expired. That is the designed fallback
   (restoring only on movement would never restore a robot that gives up on its
   goal), and it is why the grace exists as well as the movement radius.

**A SECOND RUN OF THE SAME SCENARIO, and what it changed**
(`results/phase7_yield_control.*`). Launched with `suppress_recovery:=false`
intending a recovery A/B on the yield path. It is **not** one, and it produced
something more useful instead:

| | suppressed run | second run |
|---|---|---|
| conflicts predicted | 2 | **3** |
| resolved locally, no escalation | 0 | **2** (opened by +0.03 m and +0.05 m) |
| escalated | 2 | 1 (opened by −0.29 m) |
| yields, held | 1.0 s, 15.2 s | 1.0 s |
| both goals | SUCCEEDED | SUCCEEDED (amr1 37.9 s, amr2 20.3 s) |
| recoveries during a hold | 0 | 0 |

The second run is where the **"resolved without escalation" counter reads
nonzero**, which is the local-first ordering as data rather than as a docstring:
twice the predicted closest approach opened up and the arbiter stood down, once it
did not and the arbiter took over. Both outcomes in one run, from one rule.

**It does not demonstrate that suppression was NEEDED on the yield path**, and
the report must not be read as if it did: its hold lasted 1.0 s against a 10 s
allowance, so the progress checker was never going to fire — the same non-result
Phase 2 recorded before Phase 3's barrier settled it. Necessity still rests on
Phase 3's A/B.

**The scenario is staged, and staged is not deterministic.** Two runs of an
identical launch gave two escalations and one, and an earlier attempt (before the
verdict change below) gave zero — in that one amr2 reached the gap first, no yield
was needed, and amr1's goal ABORTED after 4 recovery behaviours. The barrier, the
3.0 m gap and the 5.0 s dispatch offset stage the encounter; the simulator is
under no obligation to stage it identically twice. That variability is worth
stating in the README next to the yield numbers.

**One report change came out of that zero-escalation run:** it printed
`RESULT: FAIL` for behaving correctly. A run in which the local layer resolved
everything is not a failed run, and an artifact in `results/` reading FAIL would
be read as a broken yield protocol. The verdict now reports **NOT EXERCISED** when
no conflict met the escalation test, with the reason spelled out.

**Carries forward:**

- **`with_traffic_control` defaults to true, so the fleet bringup now contains an
  arbiter that Phase 3's and Phase 6's artifacts were measured without.** Those
  numbers are not invalidated - the arbiter acts only on conflicts the local
  layer did not open up, and neither run produced one - but a re-run of either is
  not automatically like-for-like. `with_traffic_control:=false` reproduces the
  old graph exactly.
- `docs/DEMO_RUNBOOK.md` covers all ten demos: exact commands per terminal, the
  environment traps, RViz displays, what "working" looks like, the numbers to
  quote from `results/`, and per-demo failure modes. Written so the screenshare
  can be driven without debugging live. One measured wall-clock figure in it —
  `phase7_yield` headless at 3 min 37 s — anchors the other durations, which are
  estimates and are marked as such.
- `conflict_radius` is derived from `amr_safety.safety_model`, which is a new
  package dependency for `amr_fleet_control` (acyclic: amr_safety depends on
  amr_description and amr_bsp only). It is what stops the arbiter's idea of "too
  close" drifting away from the gate's.

---
