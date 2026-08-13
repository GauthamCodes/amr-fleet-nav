# Demo Runbook

Everything needed to drive the screenshare without debugging live. Written from
the runs already in `results/`; every number quoted here is in a file on disk, so
nothing has to be re-measured on camera.

**Read section 1 and 2 once before recording. Then each demo is self-contained.**

---

## 0. Conventions

| | |
|---|---|
| **Working directory** | the repo root. Every command assumes it. |
| **T1** | the launch terminal. One launch at a time, always. |
| **T2** | the observer terminal — `ros2 topic echo`, `ros2 param get`, `cat results/…` |
| **T3** | RViz. |
| **T4** | spare, for the pre-flight cleanup between runs. |
| **`./ws.sh`** | the workspace wrapper. **Every** ROS/Gazebo command goes through it, including `rviz2`. Section 2 says why. |

Every evidence launch is **self-terminating**: it brings the world up, runs its
scenario, writes its report into `results/`, prints it to the terminal and shuts
itself down. There is no Ctrl-C step and no "now run the analysis" step. If you
Ctrl-C early you lose that run's report.

Durations below are **wall clock on this machine, headless**. The one hard
measurement is `phase7_yield` at **3 min 37 s**; the others are scaled from their
sim-time scenario length against that, and are marked as estimates. **Adding
`headless:=false` costs roughly 50–80 % more** — the GUI renders every frame.

---

## 1. Pre-flight — before **every** launch, no exceptions

```bash
# T4
./scripts/clean_processes.sh
```

It kills the simulator and the ROS graph, waits, SIGKILLs anything that ignored
SIGTERM, then **prints the surviving process table** and exits non-zero if
anything is left. You are looking for exactly this:

```
==============================================================================
PROCESS TABLE CLEAN - nothing matching a simulation or ROS pattern is running
==============================================================================
```

**If it lists survivors, do not launch.** Run it again; if the same PIDs persist,
kill them by PID. This is not ceremony: a single leaked `drive_watchdog` from an
earlier launch republishing onto `/amr1/cmd_vel_plant` once produced a finding
("the camera stalls the drive") that was written down, believed, and later
retracted — see `docs/SESSION_LOG.md`, Phase 2. A process-hygiene failure is a
measurement failure.

Then, once at the start of the session:

```bash
# T4 — the build is current (only needed after an edit; --symlink-install means
# Python changes are already live, but C++ and launch/config installs are not)
./ws.sh colcon build --symlink-install     # expect: 9 packages finished, 0 errors

# T4 — the test suite
./ws.sh python3 -m pytest tests/ -q        # expect: 241 passed
```

If you are about to re-run a demo whose artifact you want to keep, note that
**the launch overwrites `results/` files with the same tag**. Copy first, or pass
a different `tag:=`.

---

## 2. Environment — what `ws.sh` does, and the failure it prevents

```bash
./ws.sh <any command>
```

It sources `/opt/ros/jazzy/setup.bash` and this workspace's `install/setup.bash`,
points `GZ_SIM_RESOURCE_PATH` at the workspace's worlds and models, and then does
the one thing that is easy to get wrong:

```bash
export CYCLONEDDS_URI="${CYCLONEDDS_URI:+${CYCLONEDDS_URI},}file://…/cyclonedds.xml"
```

**It APPENDS.** This environment already exports a `CYCLONEDDS_URI` — an inline
fragment pinning discovery to loopback — and CycloneDDS merges a comma-separated
list of URIs. Writing `${CYCLONEDDS_URI:-file://…}` looks right and is a no-op
because the variable is already set; a plain assignment would silently drop the
interface pinning. Both failure modes are silent.

What the appended file does: raises `MaxAutoParticipantIndex` from **9 to 120**.
Every ROS 2 node is a DDS participant and a two-robot graph is **53 nodes**. Past
the default limit, node creation throws:

```
[controller_server] Failed to find a free participant index for domain 0
[rmw_cyclonedds_cpp] rmw_create_node: failed to create domain, error Error
terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
```

and six of *amr2's* Nav2 servers die while amr1 comes up perfectly — which reads
exactly like a namespacing bug in the second robot and is not one. **If you see
that error, you ran something without `./ws.sh`.** That includes RViz: it is
another participant, and it needs the same merged config.

---

## 3. RViz — one setup, reused by every demo

```bash
# T3
./ws.sh rviz2 --ros-args -p use_sim_time:=true
```

`use_sim_time` matters: without it RViz reads the wall clock, every transform
looks stale, and the whole scene either flickers or never appears.

**Fixed Frame: `fleet_map`** for anything with two robots (demos 1, 2, 3, 8, 10).
For the single-robot demos (4, 5, 6, 7) use `amr1/odom`.

Add these displays once and save the config (`File → Save Config As`), so the
recording never shows you building it:

| Display | Topic | Notes |
|---|---|---|
| **TF** | — | uncheck *Show Names*; leave *Show Axes* on. Marker Scale 0.5 |
| **Map** — fleet map | `/fleet_map` | Durability **Transient Local**, Color Scheme `map` |
| **Map** — amr1 global costmap | `/amr1/global_costmap/costmap` | Transient Local, Color Scheme `costmap`, Alpha ~0.6 |
| **Map** — amr1 local costmap | `/amr1/local_costmap/costmap` | for demo 8; Color Scheme `costmap` |
| **Map** — amr2 local costmap | `/amr2/local_costmap/costmap` | for demo 8 |
| **Path** — amr1 plan | `/amr1/plan` | green |
| **Path** — amr2 plan | `/amr2/plan` | blue |
| **Path** — amr1 prediction | `/amr1/predicted_trajectory` | **yellow, line width 0.08** — this is the MAPF message |
| **Path** — amr2 prediction | `/amr2/predicted_trajectory` | orange |
| **LaserScan** — amr1 validated | `/amr1/validated/scan` | size 0.05, colour by intensity off |
| **LaserScan** — amr1 raw | `/amr1/scan` | **red**, only enabled for demo 6 |
| **RobotModel** ×2 | Description Topic `/amr1/robot_description`, `/amr2/robot_description` | Description Source: **Topic** |

**If a display stays blank**, the usual cause is a QoS mismatch, not a missing
publisher. Check with `ros2 topic info -v <topic>` in T2: the maps and costmaps
are `RELIABLE` + `TRANSIENT_LOCAL`, everything else is `RELIABLE` + `VOLATILE`.
Set the display's QoS to match.

**Nothing appears for the first ~25 s of any launch.** That is the staged
bringup, not a fault — see the timeline in section 4, demo 1.

---

## 4. The demos

Each block is: what it proves → the commands → what you will see → the numbers to
say out loud → what can go wrong.

---

### Demo 1 — Dual bringup and the cooperative fleet map

**Requirement.** §2.1 cooperative SLAM producing one unified global map — and,
more importantly, that the merged map is *used*: it is the static layer of both
robots' global costmaps, so both plan in the fleet frame.

**Run** (this is the same launch demo 2 and 3 use, so do all three from one run):

```bash
# T4
./scripts/clean_processes.sh
# T1
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py headless:=false
```

**Duration** ~4–5 min with the GUI (est.; ~3 min headless). Bringup timeline, in
sim seconds — useful to narrate while nothing is happening:

| t | what starts |
|---|---|
| 5, 8 | amr1 spawns, then amr2 (Gazebo serialises world/create requests) |
| 10, 12 | each robot's SensorBSP |
| 14, 16 | each robot's `slam_toolbox` |
| 16 | FleetMapNode and the costmap-filter servers |
| 22, 24 | each robot's six Nav2 servers |
| 23, 25 | each robot's SafetyGate |
| 24, 26 | each robot's TrajectoryPredictor |
| 30 | TrafficControlNode (the fleet arbiter) |
| 32 | the mission dispatches both goals |

**Camera.** Gazebo window while the two robots spawn (so the evaluator sees two
distinct chassis — amr2 is visibly smaller), then switch to RViz for the map.

**RViz.** Fixed Frame `fleet_map`; enable the `/fleet_map` Map, both global
costmaps, TF. Point out that the two robots' TF trees hang off **one** frame.

**Verify live, in T2:**

```bash
./ws.sh ros2 topic info -v /fleet_map | head -20      # 2 subscribers, both global costmaps
./ws.sh ros2 param get /amr1/global_costmap/global_costmap global_frame   # fleet_map
./ws.sh ros2 param get /amr2/global_costmap/global_costmap global_frame   # fleet_map
./ws.sh ros2 node list | wc -l                        # 53
```

**Numbers to quote** (`results/phase3_concurrent_goals.md`, SESSION_LOG Phase 3):

- **53 nodes**, two Nav2 stacks, two `slam_toolbox`, two SensorBSP, two
  SafetyGates — no node-name collision, no topic collision, every lifecycle node
  reached `active`.
- `/fleet_map` is `RELIABLE · TRANSIENT_LOCAL · KEEP_LAST(1)` with **2 matched
  subscribers**: both global costmaps.
- Grid **680 × 400 cells at 0.05 m**, origin (−15.0, −10.0).
- `fleet_map → amr1/map` = (−11.000, −1.500, 0.000); `→ amr2/map` = (−11.000,
  +1.500, 0.000).
- Three distinct frames per robot: `fleet_map` for global planning, `amrN/odom`
  for the local costmap, `amrN/map` still owned by that robot's `slam_toolbox`.

**Say the limitation out loud:** the inter-map transform is **fixed from the
spawn poses and not corrected** by occupancy correlation. That was a deliberate
cut; drift is not measured and is not claimed.

**Failure modes.**

| symptom | cause | fix |
|---|---|---|
| six of amr2's servers die, `Failed to find a free participant index` | launched without `./ws.sh` | relaunch through `./ws.sh` |
| bringup hangs at ~22 s with no error naming a frame | FleetMapNode did not start before Nav2 | it is ordered in the launch; this only happens if you start pieces by hand |
| only one robot in Gazebo | two `world/create` requests raced | clean and relaunch; the launch staggers spawns by 3 s |

---

### Demo 2 — Selective map updates

**Requirement.** §2.1 selective mapping: not every candidate update is merged
into the shared map.

**Run.** Same run as demo 1 — the policy is measured *while both robots are
exploring*, because a stationary fleet defers everything and would make the
policy look effective while proving nothing.

**What "working" looks like.** In T1 the `fleet_map_node` logs accept/defer
decisions with the four score terms. The evidence file is written at the end.

**Numbers to quote** (`results/phase3_selective_updates.md`):

- **41 candidates scored, 23 accepted, 18 deferred — 43.9 %**.
- Per robot: amr1 13/8 (38.1 % deferred), amr2 10/10 (50.0 %).
- 17 composites, **mean composite + publish 0.55 ms**; fleet map ended
  15.6 % known with 1 984 occupied cells.
- The score is `w_f·frontier + w_c·occupancy_change + w_r·recency − w_v·revisit`.

**Say the honest version:** the deferred **share** is the headline, not the
milliseconds. Compositing is numpy slice arithmetic and was never the expensive
part. What deferral bounds is how often a 680 × 400 grid is serialised to two
global costmaps that each reprocess it — work inside the Nav2 processes that this
report does not measure.

**Worth mentioning:** the first weight set re-created the bug the policy exists to
prevent — a new obstacle on a corridor driven six times scored 0.20 and was
deferred. The fix was an invariant, not a retune: every positive weight must
exceed `w_revisit + accept_threshold`, and `tests/test_selective_policy.py`
asserts it, so a later retune fails a test instead of silently dropping obstacle
updates.

---

### Demo 3 — Concurrent goals to both robots

**Requirement.** §2.1/§3 — two independent stacks coexisting on one simulator,
one clock, one TF tree and one fleet map.

**Run.** Same run as demos 1 and 2. Goals dispatch at t = 32 s in one pass, not
staggered.

**RViz.** Both `/plan` paths appear at the same instant. Watch amr2 (blue) arrive
first even though both start together.

**Numbers to quote** (`results/phase3_concurrent_goals.md`):

| | amr1 | amr2 |
|---|---|---|
| result | **SUCCEEDED** | **SUCCEEDED** |
| time to goal | 18.8 s | 11.3 s |
| planned / driven (ground truth) | 10.50 / 10.44 m | 10.50 / 10.46 m |
| final position error | 0.062 m | 0.042 m |
| replans (route actually changed) | 0 | 0 |

- Closest approach **3.000 m** over 2 180 samples, median 3.471 m.
- **amr2 arrives first because `fleet.yaml` says it is faster** (`max_vel_x`
  1.00 against 0.60) — the same file that shapes its URDF, its mass, its
  acceleration limits and its safety gain. No code distinguishes the robots.
- Replan count is *routes that changed* (Hausdorff > 0.30 m from the previous
  plan), not plan messages: the tree recomputes at ~0.85 Hz regardless.

**Say it plainly:** these two routes are deconflicted *by design*. The forced
conflict is demo 10.

---

### Demo 4 — Safety halt and sensor-to-zero latency

**Requirement.** §3.3 low-latency safety override, `d_safe = k·v² + d_min`.

```bash
# T4
./scripts/clean_processes.sh
# T1
./ws.sh ros2 launch amr_bringup phase2_safety_run.launch.py headless:=false
```

**Duration** ~2–3 min (est.). Single robot; goal dispatches at t = 27 s.

**Camera.** Gazebo, framed on the aisle so the pedestrian walking into amr1's
path is visible. Then cut to T1 for the `HALT` log lines as they print:

```
HALT 1: clearance 0.412 m at 0.331 m/s, sensor->zero 8.5 ms
RELEASE: latch clearance 0.769 m > d_release 0.677 m …
```

**T2, while it runs:**

```bash
./ws.sh ros2 topic echo /amr1/safety_gate/diagnostics --once
```

**Numbers to quote** (`results/phase2_safety_suppressed.md`,
`results/phase2_stopping_distance.md`):

- Goal **SUCCEEDED**, **3 halts**, and **0 commands left the gate while latched**
  — counted *inside* the gate, comparing the twist about to be published against
  the latch at that instant.
- **sensor stamp → zero command published: mean 8.46 ms, p95 11.00 ms, max
  12.00 ms.** In-node compute on the steady clock: 151 / 226 / 711 µs.
- Stopping sweep, four commanded speeds:

| cmd m/s | v at halt | d_safe | clearance at halt | settled |
|---|---|---|---|---|
| 0.15 | 0.130 | 0.342 | 0.334 | 0.331 |
| 0.30 | 0.277 | 0.469 | 0.456 | 0.401 |
| 0.45 | 0.426 | 0.680 | 0.653 | 0.402 |
| 0.60 | 0.577 | 0.975 | 0.928 | 0.384 |

- `k_model` **1.8750 s²/m** against `k_measured` **1.0208 s²/m** — a factor 1.84,
  and both are quoted because they measure different things. Gazebo's DiffDrive
  applies `max_decel_x` as a **kinematic** limit, so the simulated robot brakes as
  if unloaded; `k_model` sizes the envelope for a 90 kg vehicle that cannot ignore
  its 60 kg payload. `k = 1/(2·a_eff)`, `a_eff = |max_decel_x|·m_base/(m_base+m_payload)`,
  `[k] = s²/m`.

**Wording discipline:** "low-latency safety override". **Not** hard real-time —
no RT kernel, no scheduling guarantees. The end-to-end figure is quantised by
Gazebo's `/clock` step under `use_sim_time`; the compute figure is not.

**Say the honest version:** velocity comes from **odometry, never from the
command** (design rule 3), and the "settled" column is the `d_min` standoff the
robot creeps to with the command still applied — the specified end state of a
speed-dependent gate, not a leak in it.

---

### Demo 5 — Fail-closed: SIGKILL the gate mid-motion

**Requirement.** §3.3 — the safety element must fail **closed**. This is the
argument for why SafetyGate is a serial last link and never a `twist_mux` input.

```bash
# T4
./scripts/clean_processes.sh
# T1 — arm A
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=watchdog headless:=false
# T4
./scripts/clean_processes.sh
# T1 — arm B, the control
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=no_watchdog \
    with_watchdog:=false headless:=false
```

**Duration** ~1–2 min each (est.). No SLAM, no Nav2 — the run needs a constant
command into the gate and nothing else.

**Camera.** Gazebo, following the robot. The moment is the SIGKILL: arm A stops
in **0.196 m**; arm B **keeps going**.

**Numbers to quote** (`results/phase2_failclosed_*.md`):

| | distance after the kill | outcome |
|---|---|---|
| plant-side watchdog present | **0.196 m** | at rest in 0.75 s |
| watchdog absent (control) | **3.500 m** | **still rolling at 0.350 m/s** when the 10 s window closed |

- **SIGKILL, not SIGTERM** — no shutdown handler runs. Upstream keeps commanding
  throughout.
- gz-sim 8.11.0's DiffDrive has **no command timeout**, so being the only
  publisher of `cmd_vel` stops new commands but not the latched one. The watchdog
  models the motor controller that does have one. Real hardware stops when its
  command stream stops; the simulator does not, and the difference is stated
  rather than glossed.
- The control needed one correction to be meaningful: with the watchdog simply
  removed, nothing bridges `cmd_vel` to `cmd_vel_plant` and the robot never moved
  at all (peak 0.000 m/s) — that measures a severed command path and nothing
  else. The gate now addresses the plant directly in that arm.

**Why a mux would be wrong:** a mux picks the highest-priority *live* input, so a
dead safety node lets ordinary navigation straight through — it fails **open**.
The gate is serial: nothing else publishes the topic the plant listens to.

---

### Demo 6 — PitchGate, before and after

**Requirement.** §2.3 BSP/HAL sensor validation, doing functional work rather
than logging.

```bash
# T4
./scripts/clean_processes.sh
# T1
./ws.sh ros2 launch amr_bringup phase2_pitch_gate.launch.py headless:=false
# T2, after it finishes
./ws.sh ros2 run amr_bsp plot_pitch_gate.py
```

**Duration** ~2–3 min (est.; the drive itself is 90 s of sim time).

**Camera.** Gazebo showing the robot climbing the 8° ramp, then RViz with **both**
LaserScan displays on: `/amr1/scan` in red (raw) and `/amr1/validated/scan` in
white. The red returns ahead of the robot on the ramp are the phantom ground
return; the white ones are what Nav2 is allowed to see.

**Numbers to quote** (`results/phase2_pitch_gate.md`):

- Raw/validated pairs matched by **exact header stamp: 903, 0 restamped, max
  difference 0 ns**. That is the design-rule-4 evidence, and it is not
  bookkeeping: pairing by exact stamp means a restamp shows up as *zero* matched
  pairs rather than as plausible numbers computed against mismatched data.
- Max pitch **8.001°**, nose-up, so the **tail** beams strike the ground.
- Predicted ground intersection `h/sin|θ|` = 2.514 m; gate radius at that pitch
  2.263 m; **120 of 360 beams truncated** in that scan.
- **Closest return removed: 2.769 m** — the phantom. **Closest return kept:
  2.732 m — the real rack, untouched.**
- Over the run: 24 833 beams truncated, **7.65 %** of all beams seen.
- **0** cuts pointing uphill, **0** inside their own gate, **0** inside the
  braking envelope. The interlock `min_gate_range` = 1.565 m for amr1 is
  computed from the braking envelope plus the footprint, so truncation can never
  delete an obstacle the SafetyGate would have had to stop for.

**Also worth one sentence:** when the IMU stream was rejected wholesale in the
injection test, PitchGate lost its attitude and republished scans
**untruncated** with a throttled WARN — the degraded-but-safe path its docstring
specifies, exercised for real rather than designed and forgotten.

If you also want the IMU validator on camera:
`./ws.sh ros2 launch amr_bringup phase2_validation.launch.py mode:=imu_injection`
→ **500 injected at 50 rad/s against a 4.0 rad/s bound, 500 rejected, 0 reached
`validated/imu`**, while **3 194** healthy samples were accepted around the
window. Both halves matter — a validator that rejected everything would produce
an equally impressive rejection count.

---

### Demo 7 — Recovery suppression A/B (gating without notification is a bug)

**Requirement.** Not a numbered requirement — a design rule, and the one an
evaluator is most likely to find interesting: anything that zeroes `cmd_vel` must
tell Nav2, or the progress checker declares the robot stuck and fires recovery
behaviours into a robot that must not move.

```bash
# T4
./scripts/clean_processes.sh
# T1 — suppressed
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py tag:=suppressed
# T4
./scripts/clean_processes.sh
# T1 — control
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py \
    tag:=control suppress_recovery:=false
```

**Duration** ~5–8 min per arm (est.): the goal deliberately ends in TIMEOUT
against a 100 s run timeout, because the barrier never moves.

**This one is better *quoted* than run live** — two arms at 5–8 min each is most
of a video. Show `results/phase2_recovery_ab_suppressed.md` and
`results/phase2_recovery_ab_control.md` side by side.

**Numbers to quote:**

| | suppressed | control |
|---|---|---|
| halts | 4 | 8 |
| **longest halt held** | **56.69 s** | **10.00 s** |
| `movement_time_allowance` read back | 1 000 000 s at every transition | 10 s at every transition |
| `controller_server: Failed to make progress` | **0** | **6** |
| recovery behaviours during a halt | **0** | **2** (Spin ×1, Wait ×1) |
| commands leaked past the latch | 0 | 0 |

- Reproduced: a second run gave 62.99 s against 8.50 s, same structure.
- **The discriminator is the halt DURATION and the six progress-checker
  failures, not the raw recovery count** — in the control arm the progress-checker
  failure and the halt release happen at the same instant, so a recovery can land
  just outside the attributed window.
- **This took two phases to prove.** Phase 2 could only show the mechanism
  *operates*: its longest halt was 0.80 s against a 10 s allowance, so the
  checker was never going to fire and the control arm's zero proved nothing. That
  was recorded as an honest non-result and carried forward. An immovable barrier
  spanning the full 5.5 m aisle is what finally produced a halt that outlasts the
  allowance.
- **The limitation this exposes:** a full-twist hold against an obstacle that
  never moves is a deadlock — the robot has no legal action that could increase
  its clearance, so the goal ends in TIMEOUT in *both* arms. That is correct for a
  fail-closed gate, and it is why "allow in-place rotation under a hold" is
  flagged as future work rather than claimed.

---

### Demo 8 — FleetTrajectoryLayer: one robot's intention as cost in another's costmap

**Requirement.** §3.2 MAPF — the local planner consumes the other robot's
predicted trajectory. **This is the requirement an evaluator will probe hardest.**

```bash
# T4
./scripts/clean_processes.sh
# T1 — layer ON
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py headless:=false
# T4
./scripts/clean_processes.sh
# T1 — layer OFF, the control arm
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py \
    with_trajectory_layer:=false tag:=layer_off
```

**Duration** ~5–6 min per arm (est.; the probe runs 150 s of sim time). **Run arm
A live, quote arm B** if time is short.

**Camera.** RViz, Fixed Frame `fleet_map`. The shot is:
`/amr1/predicted_trajectory` (yellow) laid across `/amr2/local_costmap/costmap`.
Zoom so both robots and the crossing are in frame. The yellow path extends
*ahead* of amr1 — that is the point.

**Numbers to quote** (`results/phase6_cost_injection_layer_{on,off}.md`):

| | layer ON | layer OFF |
|---|---|---|
| samples, peer's predicted cell inside the window | 50 | 59 |
| **samples with cost > 0** | **50 (100 %)** | **0 (0 %)** |
| median cost at that cell | **145** | 0 |
| max cost | 240 | 0 |
| cost the decay model predicts | 125.9 | 127.3 |

- The probe samples where amr1 is predicted to be **2 s from now**, *not* where it
  is. That distinction is the measurement: amr1 is a physical object amr2's LiDAR
  marks and inflates regardless, so sampling the peer's *current* cell would
  measure the obstacle layer and report it as MAPF.
- The layer deposits `max_cost · exp(−Δt/τ)` over a disc per predicted peer pose,
  combined with `std::max` so it can raise a cell's cost and never lower it.
- Cost is capped below `INSCRIBED_INFLATED_OBSTACLE` (253) by construction: a
  peer's *intention* must never read as a collision to a footprint checker.
  `tests/test_trajectory_conflict.py` asserts the shipped YAML respects the cap.

**Say the limitation, in these words:** RegulatedPurePursuit is not a sampling
optimiser. It consumes local costmap cost through cost-regulated velocity scaling
and forward collision checking, so the layer changes how it **paces**, but it does
not deviate laterally around graded cost the way MPPI's CostCritic would.
**"Robots mutually deviate without central intervention" is therefore NOT
demonstrated**, and no run in this repo should be read as showing it. What is
demonstrated is the mechanism and the cost injection, measured against a control.

**Do not claim the behavioural difference.** amr1 drove 8.30 m in 18.0 s with the
layer and 9.03 m in 20.6 s without; closest approach 1.610 m against 1.529 m —
but that is n=1 per arm and an earlier layer-on run gave 1.538 m, inside the same
spread. The cost-injection measurement is the claim.

---

### Demo 9 — Payload-adaptive velocity and jerk

**Requirement.** §3.1 payload-aware motion smoothing, extending Nav2 rather than
replacing it.

```bash
# T4
./scripts/clean_processes.sh
# T1
./ws.sh ros2 launch amr_bringup phase5_payload_trace.launch.py
```

**Duration** ~3–4 min (est.). Headless is fine — **the deliverable is the plot**,
`results/phase5_payload_trace.png`. Put it on screen full-width.

**The chain, worth naming link by link:**

```
Nav2 → cmd_vel_nav → nav2_velocity_smoother (STOCK) → cmd_vel_smoothed
     → PayloadJerkAdapter (ours) → cmd_vel_shaped
     → twist_mux (nav 100 | yield 150) → cmd_vel_mux
     → SafetyGate (serial, LAST, fail-closed) → cmd_vel → plant
```

We add **only** jerk limiting and payload scaling. The velocity/acceleration
clamp is stock Nav2.

**Numbers to quote** (`results/phase5_payload_trace.md`):

| robot | payload | peak v cmd | peak a cmd | peak jerk cmd |
|---|---|---|---|---|
| amr1 | unloaded | 0.500 | 0.725 | 1.784 |
| amr1 | loaded | 0.500 | 0.272 | 0.634 |
| amr2 | unloaded | 0.500 | 1.112 | 4.722 |
| amr2 | loaded | 0.500 | 0.940 | 2.742 |

- **amr1's peak commanded acceleration falls ×0.38 loaded against amr2's ×0.85** —
  60 kg on a 30 kg chassis is a far larger perturbation than 5 kg on 18 kg — **and
  no code distinguishes them.** Peak commanded velocity is exactly 0.500 in all
  four cases.

**Read the jerk column honestly, on camera.** It exceeds the configured bound
(1.000 / 0.333 / 2.500 / 1.957 m/s³) by up to ~1.9×. The bound holds on the
limiter's own recursion — `tests/test_jerk_limiter.py` asserts it for every robot
at both payload states — so what that column measures is the **published stream**:
a 20 Hz signal timestamped on arrival, resampled onto a 50 ms grid and
differentiated twice, plus a single-step transient where the velocity reaches its
target. What this run supports is the **payload ratio and the absence of
overshoot**, not a certified jerk ceiling. Closing that gap means timestamping at
the publisher, and it is not done.

**One good war story if there is time:** the continuous-time S-curve `sqrt(2j|e|)`
is the wrong formula for a discrete loop — it leaves acceleration on the books at
arrival, which the velocity clamp then absorbs in one step, spiking measured jerk
to **2.9–3.5× the limit** exactly where a jerk limiter is supposed to help. The
exact discrete form `−jΔt/2 + sqrt((jΔt/2)² + 2j|e|)` brings the worst case to
~1.1× on the recursion.

**Also worth naming:** stock `twist_mux` does not load in this ROS snapshot —
`ros-jazzy-twist-mux` 4.5.0 wants a `diagnostic_updater` symbol ending `EEdh`
that the installed 4.2.6 does not export. `amr_safety/scripts/priority_mux.py`
preserves the architecture and reads the **same** `twist_mux.yaml` schema, so
restoring the stock node is a two-line launch change. Diagnosed, not worked
around silently.

---

### Demo 10 — The yield protocol: escalation from the local layer to the arbiter

**Requirement.** §3.2's second half — a yielding protocol for conflicts the local
layer cannot resolve. **This is the demo that shows the ordering: local first,
central only when local has failed.**

```bash
# T4
./scripts/clean_processes.sh
# T1
./ws.sh ros2 launch amr_bringup phase7_yield.launch.py headless:=false
```

**Duration** **3 min 37 s headless (measured)**; ~5–6 min with the GUI. The
mission dispatches at t = 34 s; amr2's goal is held back a further 5 s.

**Camera.** Gazebo, framed on the barrier at x = −5 so the **3.0 m gap** and both
approaching robots are in shot. The moment to catch: **amr2 stops short of the
gap while amr1 drives through it**, then amr2 resumes.

**RViz.** Fixed Frame `fleet_map`; both `/plan` paths converging on the same gap
is the visual that makes "narrow intersection" obvious.

**T2, to show the yield channel actually carrying commands:**

```bash
./ws.sh ros2 topic hz /amr2/cmd_vel_yield        # ~20 Hz while held, silent otherwise
./ws.sh ros2 param get /amr2/controller_server progress_checker.movement_time_allowance
#   1000000.0 during the hold, 10.0 after the release
```

**Watch T1 for these three lines** — they are the whole story:

```
conflict predicted amr1/amr2: closest approach 1.67 m within 4.0 s (radius 1.94 m) - local layer has it
ESCALATED amr1/amr2: unresolved for 2.0 s (closest approach 1.53 m, opened by -0.14 m) - amr2 yields to amr1
RELEASE amr2 after 15.2 s: conflict cleared (separation 2.43 m …); recoveries during the hold: 0
```

**Numbers to quote** (`results/phase7_yield.md`, `results/phase7_yield_mission.md`):

- Priority is **derived from `fleet.yaml` gross mass** — amr1 90.0 kg > amr2
  23.0 kg — so "AMR-2 yields to AMR-1" falls out of configuration, not out of a
  robot name in the code. `tests/test_traffic_policy.py` asserts that changing the
  masses changes the yield direction.
- Conflict radius **1.94 m = r(amr1) 0.539 + r(amr2) 0.429 + d_safe(amr1 at
  cruise) 0.975**. Derived, not tuned: it is the distance at which one robot's
  SafetyGate would already be holding the other, so past it neither can resolve
  anything by driving.
- Two escalations, **amr2 yielded both times**, held **1.0 s and 15.2 s**, both
  released on **"conflict cleared"** — never on the 45 s fail-safe ceiling.
- **0 recovery behaviours during either hold.** `movement_time_allowance` read
  back from `controller_server` at **1 000 000 s on entry and 10 s after release**
  — read back, not assumed.
- SafetyGate was blocking on **0 of 326 held cycles**, so the stop was the
  arbiter's and nothing else's. A yield and a safety halt both end with a robot at
  a standstill; the report separates them rather than assuming.
- **Both goals SUCCEEDED**: amr1 40.0 s / 12.72 m driven, amr2 42.9 s / 13.62 m,
  final errors 0.032 m and 0.151 m. Closest actual approach 0.926 m.

**The escalation ordering, in one sentence:** the arbiter refuses to act while the
predicted closest approach is still opening — here it opened by **−0.14 m and
+0.04 m**, i.e. the local layer had first refusal for 2.0 s in both cases and did
not open the gap, which is what a constriction with no lateral room to deviate
into looks like from the arbiter's side.

**And the counter-example, on disk:** `results/phase7_yield_control.md` is a
second run of the same scenario. It shows the escalation test working in **both**
directions within one run — **3 conflicts predicted, 2 resolved locally** (the
predicted approach opened by +0.03 m and +0.05 m, and the arbiter stood down) and
**1 escalated** (−0.29 m, amr2 yielded, held 1.0 s, both goals still SUCCEEDED).
That nonzero "resolved locally" count is the local-first ordering as data rather
than as an assertion. If a run ever escalates nothing at all, its verdict reads
**NOT EXERCISED**, not FAIL — a run that needed no arbitration is not a failed
run.

**Say the honest version:** that second run was launched with
`suppress_recovery:=false` intending a recovery A/B on the yield path, and it does
**not** serve as one: its hold lasted 1.0 s against a 10 s
`movement_time_allowance`, so the progress checker was never going to fire — the
same non-result Phase 2 recorded before demo 7's barrier settled it. **The
necessity of recovery suppression rests on demo 7**, not on this. Note also that
two runs of the identical scenario gave two escalations and one: the barrier, the
gap and the 5 s dispatch offset make the encounter *staged*, and staged is not
deterministic.

**Failure modes.**

| symptom | cause | fix |
|---|---|---|
| no `ESCALATED` line ever prints | the encounter did not overlap in time — amr2 got through first | relaunch; or raise `time_window_s:=5.0` |
| both robots stop and neither moves | both entered the gap together and both gates blocked | this is the failure the yield prevents; it means the arbiter did not fire in time — relaunch |
| `RELEASE … max hold elapsed (fail-safe)` | 45 s ceiling broke a deadlock | report it as a deadlock, not as a successful yield — the report already distinguishes them |
| the run reports NOT EXERCISED | no conflict met the escalation test that run | relaunch; the encounter is staged, not deterministic |

---

## 5. Failure modes and recovery — global

| symptom | what it actually is | recovery |
|---|---|---|
| `Failed to find a free participant index for domain 0`, second robot's servers die | a command was run without `./ws.sh`, so the CycloneDDS participant ceiling is still 9 | kill everything, relaunch through `./ws.sh` — **including RViz** |
| numbers look wrong in a way that suggests a code bug | a leaked node from an earlier launch is still publishing | `./scripts/clean_processes.sh`, confirm the CLEAN banner, re-run. Retract nothing until you have re-measured on a verified-clean table |
| launch appears to hang ~20–30 s in with no error | staged bringup, or a lifecycle node waiting on a transform | wait 45 s; if still nothing, clean and relaunch |
| `gz sim` window is black or the world is empty | a previous `gz sim` survived and holds the port | `./scripts/clean_processes.sh` (it matches `gz sim` and `ruby.*gz`) |
| RViz shows nothing, no errors | Fixed Frame not yet published, or a QoS mismatch | set Fixed Frame to `fleet_map` (fleet) or `amr1/odom` (single robot); check `ros2 topic info -v` |
| a robot spins in place at a standstill | Nav2 recovery — the progress checker fired | that is demo 7's failure mode. In a suppressed run it means suppression did not take: check `ros2 param get /amrN/controller_server progress_checker.movement_time_allowance` |
| `map_saver_cli` times out | Nav2 gives an identical timeout for a missing publisher, a QoS mismatch and a genuinely slow map | it needs a `TRANSIENT_LOCAL` publisher; the survey holds a subscription open for this reason |
| a run overwrote an artifact you wanted | same `tag:` | re-run with `tag:=something_else`; `git status` will show what changed |
| goal ABORTED with `Failed to create a plan from potential` | NavFn transient while the map is still filling in ahead of the robot | the evidence runs retry a bounded number of times and report the dispatch count; if it persists, clean and relaunch |

**A note worth saying once on camera:** several of these entries exist because
they *happened* and cost a day each. They are in `docs/SESSION_LOG.md` with the
measurement that resolved them, including one finding that was written down,
believed for a session, and then retracted after re-measuring.

---

## 6. Command sheet

```bash
# every time
./scripts/clean_processes.sh

# demos 1-3: dual bringup, fleet map, selective updates, concurrent goals
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py headless:=false

# demo 4: safety halt + latency
./ws.sh ros2 launch amr_bringup phase2_safety_run.launch.py headless:=false

# demo 5: fail-closed (two arms)
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=watchdog headless:=false
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=no_watchdog with_watchdog:=false

# demo 6: PitchGate
./ws.sh ros2 launch amr_bringup phase2_pitch_gate.launch.py headless:=false
./ws.sh ros2 run amr_bsp plot_pitch_gate.py
./ws.sh ros2 launch amr_bringup phase2_validation.launch.py mode:=imu_injection

# demo 7: recovery suppression A/B  (quote, do not run live)
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py tag:=suppressed
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py tag:=control suppress_recovery:=false

# demo 8: FleetTrajectoryLayer A/B
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py headless:=false
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py with_trajectory_layer:=false tag:=layer_off

# demo 9: payload-adaptive velocity
./ws.sh ros2 launch amr_bringup phase5_payload_trace.launch.py

# demo 10: yield protocol
./ws.sh ros2 launch amr_bringup phase7_yield.launch.py headless:=false

# observers
./ws.sh rviz2 --ros-args -p use_sim_time:=true
./ws.sh ros2 topic hz /amr2/cmd_vel_yield
./ws.sh ros2 param get /amr2/controller_server progress_checker.movement_time_allowance
./ws.sh ros2 topic echo /amr1/safety_gate/diagnostics --once
./ws.sh ros2 node list | wc -l
```

---

## 7. Suggested order for a 7-minute recording

Run **two** things live and quote the rest from `results/`. Live runs eat the
budget: the shortest useful one is 3.5 minutes.

| min | segment | live? |
|---|---|---|
| 0:00–1:00 | Architecture: the command chain, and why the safety gate is a serial fail-closed link rather than a mux input | slides/diagram |
| 1:00–2:00 | Demo 1–3 from the artifacts: 53 nodes, one fleet map driving both global costmaps, both goals reached | quote |
| 2:00–3:00 | Demo 4 + 5: halt, 8.46 ms mean sensor-to-zero, then 0.196 m vs 3.500 m under SIGKILL | quote |
| 3:00–4:00 | Demo 8: cost injection, 100 % vs 0 % against the control — **and the RPP limitation stated plainly** | quote + RViz |
| 4:00–5:00 | Demo 9: the payload plot, ×0.38 vs ×0.85, and the honest reading of the jerk column | quote |
| 5:00–6:30 | **Demo 10 live** — the forced conflict, amr2 yielding, 0 recoveries during a 15.2 s hold | **LIVE** |
| 6:30–7:00 | Known limitations, said plainly: fixed inter-map transform, RPP not MPPI, jerk measured at the subscriber, the retracted camera finding, the yield control arm that did not reproduce | — |

**Closing line worth rehearsing:** every claim in the video points at a file in
`results/`, every limitation is in the README, and the one finding that turned out
to be wrong was retracted in the log rather than quietly deleted.
