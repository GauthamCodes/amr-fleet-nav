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

**Carries forward:**
- `lidar_height` stays at 0.20 m. Smoke test 2 gives no reason to change it: the
  phantom return at max pitch (1.78 m) sits well outside `d_safe` (0.48 m).
- PitchGate in Phase 2 has its sizing table: closest return per pitch bin, 0°→8°.
- The world reserves a second-ramp footprint (commented in `world.yaml`). Phase 4's
  A/B experiment needs the upper level to be a shortcut, which takes two ramps.
- Frame convention is `amr1/base_link`, which differs from PLAN.md §2's diagram
  (`base_link_amr1`). Chosen deliberately; **PLAN.md not edited — ask first.**

---
