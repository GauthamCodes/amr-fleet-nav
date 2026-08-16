# Demo Runbook

Every scenario in this repository as a self-contained instruction block. Each
command was checked against the launch file it names; nothing here is speculative.

Written so that someone who has never run this repository can bring up any demo,
know what should appear on screen, and know what to do when it does not.

---

## 0. Before anything

```bash
# once per session
./ws.sh colcon build --symlink-install     # expect: 9 packages finished, 0 errors
./ws.sh python3 -m pytest tests/ -q        # expect: 241 passed

# before EVERY launch, no exceptions
./scripts/clean_processes.sh               # expect the CLEAN banner
```

**Two rules that are not optional.**

1. **Every ROS or Gazebo command goes through `./ws.sh`, including `rviz2`.** The
   wrapper sources the workspace and *appends* to `CYCLONEDDS_URI`, raising
   CycloneDDS's participant ceiling from its default of 9 to **400**. The count that
   matters is **participants, not nodes**: a two-robot `fleet_nav` is 65 unique node
   names but `ros2 node list` returns **111 entries**, because several nodes
   advertise from more than one participant. Past the ceiling, node creation throws
   `Failed to find a free participant index for domain 0` and whichever servers
   started last die while the rest of the fleet comes up perfectly — which reads
   exactly like a namespacing bug in one robot and is a host-wide limit. **The
   ceiling was 120 and that was not enough**: `phase7_yield` adds RViz, the arbiter,
   the mission node and both trajectory predictors, crossed 120, and nine nodes died
   with exactly this error. Full write-up in `src/amr_bringup/config/cyclonedds.xml`.
2. **Run `./scripts/clean_processes.sh` before every launch** and wait for the CLEAN
   banner. A leaked node from an earlier run once produced a finding that was
   believed for a session and then retracted after re-measurement.

Every evidence launch is **self-terminating**: it brings the world up, runs its
scenario, writes its report into `results/`, prints it and shuts itself down. There
is no Ctrl-C step. Ctrl-C before a run writes its report loses that run's artifact.

**To stop any demo early:** Ctrl-C once in the launch terminal, wait for the process
table to drain, then run `./scripts/clean_processes.sh` and confirm the banner
before the next launch.

---

## 1. RViz — which demos start it, and the Fixed Frame

`rviz:=true` starts RViz **inside the launch**, already loaded with
`src/amr_bringup/rviz/fleet_mapping.rviz`. Prefer this: a launched RViz cannot be
the one process in the graph that missed `./ws.sh`.

| Launch | `rviz:=true`? | Fixed Frame |
|---|---|---|
| `fleet_survey.launch.py` | **yes — on by default** | `fleet_map` |
| `fleet_nav.launch.py` | yes | `fleet_map` |
| `phase3_fleet_goals.launch.py` | yes | `fleet_map` |
| `phase6_conflict.launch.py` | yes | `fleet_map` |
| `phase7_yield.launch.py` | yes | `fleet_map` |
| every `phase1_*`, `phase2_*`, `phase5_*` | no — single robot | `amr1/odom` |

**`fleet_map` is the shared root** both robots' TF trees hang off. `FleetMapNode`
publishes it at t ≈ 16 s, and it is verified to exist at runtime —
`ros2 run tf2_ros tf2_echo fleet_map amr1/base_link` resolves and tracks the moving
robot.

**There is no bare `map` frame in this system.** Every robot's SLAM frame is
namespaced `amrN/map`. A bare `rviz2` with no config comes up on the stock Fixed
Frame `map` and is therefore blank with *"Fixed Frame — Frame [map] does not
exist"*. That is the symptom; the saved config is the fix.

**Single-robot demos have no `fleet_map`.** Do not load the fleet config against
them — it would come up blank for exactly the reason above. Start RViz separately
and set Fixed Frame to `amr1/odom`:

```bash
# second terminal, single-robot demos only
./ws.sh rviz2 --ros-args -p use_sim_time:=true
#   Fixed Frame: amr1/odom
#   add: LaserScan /amr1/scan · Path /amr1/plan
#        Map /amr1/global_costmap/costmap · RobotModel /amr1/robot_description
```

**What the fleet config shows.** On by default: `/fleet_map`, both robot models
(**amr1 blue, amr2 amber**, from `body_color` in `fleet.yaml` — RViz's RobotModel
display has no colour property), `/amr1/scan` green, `/amr2/scan` orange,
`/amr1/plan` green, `/amr2/plan` cyan, and TF reduced to the frames worth seeing.
Off by default, one click each: each robot's own SLAM map, each robot's global
costmap, each robot's predicted trajectory, each robot's local costmap.

Nothing appears for the first ~25 s of any launch. That is the staged bringup.

---

## 2. Dynamic obstacles

The warehouse contains **three scripted pedestrians** — one along the main aisle,
one crossing the ramp approach into the robot's path, one on the upper plateau.
They are verified to reach the navigation stack:
`results/smoke1_actor_visibility.md` records them raycast by the LiDAR and marked
**254 LETHAL in the Nav2 costmap in 100 % of frames**.

`with_actors` controls them, and **the default differs per launch on purpose**: a
walking pedestrian is an uncontrolled variable, so measurement runs disable it and
demonstration runs enable it.

| Launch | Actors | How |
|---|---|---|
| `fleet_nav.launch.py` | **on** | default `with_actors:=true` |
| `phase2_safety_run.launch.py` | **on, always** | hard-wired — the pedestrian *is* the demo |
| `phase1_nav_run.launch.py`, `amr1_nav.launch.py` | **on** | default true |
| `phase3_fleet_goals.launch.py` | off | pass `with_actors:=true` |
| `phase6_conflict.launch.py` | off | pass `with_actors:=true`; off by default because an actor puts cost into the costmap the probe reads |
| `fleet_survey.launch.py` | off | pass `with_actors:=true`; off by default because `slam_toolbox` has no dynamic-object rejection and traces a walker into the map as a smear |
| `phase7_yield.launch.py` | **off, always** | hard-wired — an actor in the 3.0 m gap would stop both robots for a reason that is not the yield |
| other `phase2_*`, `phase1_survey` | off | hard-wired; each is a controlled measurement |

Gazebo actors are **not physics entities** — they cannot generate contacts. They are
rendered and they are raycast, which is what navigation needs. **Clearance, not
collision avoidance, is the honest quantity.**

---

# The scenarios

---

## Scenario 1 — Cooperative mapping

**Purpose.** Show `/fleet_map` starting unknown and filling in from two directions at
once, in a frame both robots share.

**Assignment requirement.** §2.1 cooperative global SLAM and map fusion — both robots
contributing to a single unified occupancy grid. Selective mapping runs live in the
log alongside it.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup fleet_survey.launch.py
```

**RViz.** **Starts automatically** — `rviz:=true` and `headless:=false` are both the
default here. It is the one launch built for the camera.

**Fixed Frame.** `fleet_map`.

**Dynamic actors.** Off by default, and leave them off: `slam_toolbox` has no
dynamic-object rejection, so a walking actor is traced into the map as a smear that
later scans only partly clear. Scenario 2 is where pedestrians are shown.

**Duration.** ~3 min headless, ~4–6 min with the GUI. `laps:=2` roughly doubles the
drive and gives `slam_toolbox` a genuine revisit to close a loop against.

**Expected startup sequence.**

| t (sim s) | what starts |
|---|---|
| 5, 8 | amr1 spawns, then amr2 |
| 10, 12 | each robot's SensorBSP |
| 12 | RViz |
| 14, 16 | each robot's `slam_toolbox` |
| 16 | **FleetMapNode** — publishes `fleet_map → amrN/map` and an empty grid |
| 28 | both survey drives start |

**Expected visual behaviour.** RViz opens on a top-down view with an empty grid. From
t≈16 s a mostly-unknown `/fleet_map` appears. Both robots then drive the same closed
circuit **phase-shifted by half a lap** — eastbound in the south lane, westbound in
the north lane — so the map fills in from **both ends of the aisle at once** and the
rack rows resolve into a toothed pattern north and south.

**What the operator should watch.** The map growing. Both robots moving in RViz (blue
and amber). The accept/defer lines scrolling in the terminal.

**What the presenter should explain.**

1. **One map, two contributors.** Toggle *amr1 own map* and *amr2 own map* on: each
   covers part of the aisle, `/fleet_map` covers the union. That is the fusion.
2. **One frame.** Both robots' TF trees hang off a single `fleet_map` root.
3. **Selective mapping** — the score in the terminal, and what deferral bounds.

**Expected result.** Both drives complete, the launch shuts itself down, and
`results/fleet_survey_updates.{md,csv}` holds that run's accept/defer report. **Both
robots appear in the `ACCEPT` stream** — that, not any particular count, is the
acceptance condition. The counts move a long way between runs because the score
depends on where each robot is when a scan lands: runs on this same command have given
644 candidates / 80 accepted (87.6 % deferred) and 91 / 65 (28.6 % deferred), with
fleet-map known coverage around 17 %.

**Quote the committed numbers, not the live ones.** The canonical selective-mapping
evidence is `results/phase3_selective_updates.md` — **41 scored, 23 accepted, 18
deferred, 43.9 %**. The survey's own artifact is git-ignored precisely so it cannot
be mistaken for that.

**Limitations and failure modes.**

| symptom | cause | fix |
|---|---|---|
| `/amrN/plan` never appears | expected — this run has no Nav2 on purpose, so both Path displays stay blank | use Scenario 2 for plans |
| RViz blank, "Frame [map] does not exist" | RViz started by hand with no config | use the launch's RViz, or `-d src/amr_bringup/rviz/fleet_mapping.rviz` |
| map appears but never grows | a survey drive died | check the terminal for a `survey_drive` traceback; clean and relaunch |
| only one robot in Gazebo | two `world/create` requests raced | clean and relaunch; spawns are staggered by 3 s |
| a smeared wall across the aisle | you passed `with_actors:=true` | expected; leave actors off here |

---

## Scenario 2 — Simultaneous navigation goals

**Purpose.** Both robots navigating to different goals at the same time, against the
shared map, with pedestrians in the aisle.

**Assignment requirement.** §2.2a concurrent goals — and the proof that the merged
map is *used*: it is the static layer of both robots' global costmaps.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    headless:=false rviz:=true with_actors:=true tag:=demo
```

Drop `with_actors:=true tag:=demo` to reproduce the committed artifact exactly.

**RViz.** Starts automatically with `rviz:=true`.

**Fixed Frame.** `fleet_map`.

**Dynamic actors.** **Pass `with_actors:=true`** — off by default here. Keep
`tag:=demo` alongside it so the run does not overwrite the committed Phase 3
evidence, which was measured without them.

**Duration.** ~2 min headless, ~4–5 min with the GUI.

**Expected startup sequence.** As Scenario 1 up to t=16, then: 22/24 each robot's six
Nav2 servers, 23/25 each SafetyGate, 24/26 each TrajectoryPredictor, 30 the arbiter,
**32 the mission dispatches both goals in one pass**.

**Expected visual behaviour.** In Gazebo, two visibly different chassis — amr2
smaller — and pedestrians walking the aisle. In RViz at t≈32 s **both `/plan` paths
appear at the same instant**, green for amr1 and cyan for amr2, running east in their
own lanes. **One robot then drives its route and arrives; the other replans the same
path repeatedly and never gets going.** That is the current behaviour, not a
misconfiguration on your side — see *Expected result* below.

**What the operator should watch.** Both plans appearing together, one robot tracking
its plan to the goal, the pedestrians crossing — and the second robot's plan being
redrawn without the robot advancing.

**What the presenter should explain.**

1. Both plans appear together — concurrent, not sequential.
2. **amr2 arrives first because `fleet.yaml` says it is faster** (`max_vel_x` 1.00
   against 0.60). The same file shapes its URDF, mass, acceleration limits and safety
   gain. **No code distinguishes the robots.**
3. Toggle both **global costmap** displays: the fleet map is arriving as their static
   layer — "the map is used, not just published".

**Verify live, in a second terminal:**

```bash
./ws.sh ros2 topic info -v /fleet_map | head -20    # 2 subscribers, both global costmaps
./ws.sh ros2 param get /amr1/global_costmap/global_costmap global_frame   # fleet_map
./ws.sh ros2 param get /amr2/global_costmap/global_costmap global_frame   # fleet_map
./ws.sh ros2 node list | wc -l                      # 126 entries, 65 unique names
```

The entry count is roughly twice the number of nodes because several nodes advertise
from more than one DDS participant — that gap is the whole reason `cyclonedds.xml`
raises the participant ceiling (§0), and it is why an earlier "53" here was wrong.
Do not treat a specific number as the check; the check is that **every lifecycle node
reaches `active`**.

**Expected result — read this before you run it.**

**Both goals are dispatched, both are accepted, and both plans are published in
`fleet_map`. Only ONE of the two robots reaches its goal.** The other replans the
full 10.5 m path repeatedly, never translates more than about 2 m, and `bt_navigator`
aborts with `Failed to make progress`. Which robot loses depends on the
configuration:

| configuration | amr1 | amr2 |
|---|---|---|
| **as shipped** | SUCCEEDED 18.8–19.5 s | ABORTED, ≤ 0.6 m driven |
| `with_motion_chain:=false` | ABORTED, 2.0 m driven | SUCCEEDED 11.6 s |

Reproduced in **9 consecutive runs**, the most recent against a from-scratch build of
this commit and committed as `results/phase3_concurrent_goals_current.md` — amr1
SUCCEEDED in 18.7 s having driven 10.52 m to a 0.020 m final error, amr2 ABORTED
having driven 0.07 m. Ruled out in turn: the pedestrians, the Gazebo GUI and its
real-time factor, the fleet trajectory layer on its own, and SafetyGate — which never
fires during the stall. **This is a known open defect, written up in README §10.** Do
not present this scenario as both robots arriving.

> **Historical, not current.** `results/phase3_concurrent_goals.md` records **both**
> robots SUCCEEDED — amr1 18.8 s / amr2 11.3 s, final errors 0.062 / 0.042 m, closest
> approach 3.000 m over 2 180 samples. That artifact was measured at commit `8ae594e`,
> **before** the payload motion chain and the fleet trajectory layer were added, and
> no run since reproduces it. Whichever robot does complete reproduces its own
> committed time almost exactly, so those are real measurements of a configuration
> that **is no longer the one that ships.**

What this scenario *does* demonstrate: concurrent **dispatch**, concurrent
**planning** in one shared frame, concurrent **tracking**, and the fleet map in use as
the static layer of both global costmaps. What it does **not** demonstrate is both
robots arriving.

**Limitations and failure modes.**

| symptom | cause | fix |
|---|---|---|
| six of amr2's servers die, participant index error | launched without `./ws.sh` | relaunch through `./ws.sh` |
| goal ABORTED, `Failed to create a plan from potential` | NavFn transient while the map is still filling in ahead of the robot | the run retries a bounded number of times; if it persists, clean and relaunch |
| bringup hangs ~22 s with no error naming a frame | FleetMapNode did not start before Nav2 | it is ordered in the launch; only happens if you start pieces by hand |

**Stated limitation.** These two routes are deconflicted *by design*; the forced
conflict is Scenario 6. The inter-map transform is fixed from the spawn poses and
never corrected — drift is not measured and is not claimed.

---

## Scenario 3 — Ramp / slope planning **(partial — do not claim the behaviour)**

**Purpose.** Show the terrain-cost mechanism, and state what it does not yet do.

**Assignment requirement.** §2.2b — the global planner must cost sloped surfaces,
minimising their use unless they are the only viable path.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    ramp_mask_value:=60.0 tag:=ramp_graded
```

**RViz.** Supports `rviz:=true`.  **Fixed Frame** `fleet_map`.  **Actors** off.

**Duration.** ~2 min headless.

**Expected visual behaviour.** A normal two-robot goal run. The mask is visible only
as costmap colour, which is why this scenario is better quoted than run on camera.

**What is verified.** A Nav2 `KeepoutFilter` mask over the ramp footprint, generated
(not hand-drawn) by `amr_navigation/ramp_mask.py`, loads into both global costmaps —
both log `Received filter mask`. The null mask is proven to contribute nothing:
**minimum cost over 272 000 known cells = 0**.

**What is NOT verified.** At mask value 60 the cost over the 3 550-cell ramp footprint
is **0..100 — identical to the null run** (`results/phase3_ramp_cost_graded.md`).
Graded terrain cost is **not demonstrated**. Leading hypothesis, stated as a
hypothesis: Jazzy's `KeepoutFilter` maps mask values to cost binarily (100 → LETHAL,
below → nothing) rather than proportionally.

**The A/B cannot be staged in this world at all.** The experiment — flat route
available so the ramp is avoided, flat route blocked so the ramp is taken — needs one
goal reachable both ways. The upper plateau is a solid box spanning the full y
extent, so the ramp is the only route up and nothing on the lower level needs it.

**Recommendation.** Quote this from README §10 rather than running it.

---

## Scenario 4 — Payload-aware motion smoothing

**Purpose.** Show that acceleration and jerk limits adapt to payload, and that the
heavier robot is the more conservative one.

**Assignment requirement.** §3.1 dynamic velocity and motion smoothing.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase5_payload_trace.launch.py
```

**RViz.** No `rviz` argument — these are single-robot trace runs. **The deliverable is
the plot**, `results/phase5_payload_trace.png`. Put it on screen full width.

**Fixed Frame.** N/A.  **Actors.** Off, hard-wired.

**Duration.** ~3–4 min. Headless is fine.

**Expected visual behaviour.** Terminal progress per arm — each robot unloaded and
loaded — then the plot is written.

**What the presenter should explain.** The chain, link by link:

```
Nav2 → cmd_vel_nav → nav2_velocity_smoother (STOCK) → cmd_vel_smoothed
     → PayloadJerkAdapter (OURS) → cmd_vel_shaped
     → priority mux → cmd_vel_mux → SafetyGate → cmd_vel → plant
```

We add **only** jerk limiting and payload scaling. The velocity/acceleration clamp is
stock Nav2, and the adapter can restrict a limit but never raise one.

**Expected result** (`results/phase5_payload_trace.md`):

| robot | payload | peak v | peak a | peak jerk |
|---|---|---|---|---|
| amr1 | unloaded | 0.500 | 0.725 | 1.784 |
| amr1 | **loaded** | 0.500 | **0.272** | 0.634 |
| amr2 | unloaded | 0.500 | 1.112 | 4.722 |
| amr2 | **loaded** | 0.500 | **0.940** | 2.742 |

**amr1's peak commanded acceleration falls ×0.38 loaded against amr2's ×0.85** — and
**no code distinguishes them**. Peak commanded velocity is exactly 0.500 in all four
cases: the adapter restricts without overshooting.

**Limitation to state out loud.** The jerk column exceeds the configured bound by up
to ~1.9×. The bound holds on the limiter's own recursion —
`tests/test_jerk_limiter.py` asserts it for every robot at both payload states — so
what that column measures is the **published stream**: a 20 Hz signal timestamped on
arrival, resampled and differentiated twice. **The payload ratio is supported; a
certified jerk ceiling is not.**

---

## Scenario 5 — MAPF: peer trajectory as cost

**Purpose.** Show one robot's projected trajectory being consumed by the other
robot's local planner.

**Assignment requirement.** §3.2a — the local planner for each robot consumes the
projected trajectory of the other robot.

**Command.**

```bash
# arm A — layer ON
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py headless:=false rviz:=true

# arm B — the control
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py \
    with_trajectory_layer:=false tag:=layer_off
```

**RViz.** `rviz:=true`. In the Displays panel switch on the four off-by-default
toggles: **amr1/amr2 predicted trajectory** and **amr1/amr2 local costmap**. Use the
*Aisle close-up* saved view.

**Fixed Frame.** `fleet_map`.  **Actors.** Off by default — an actor would put cost
into the very costmap the probe reads. `with_actors:=true` exists if you want them.

**Duration.** ~5–6 min per arm. Run arm A live, quote arm B.

**Expected visual behaviour.** The shot is `/amr1/predicted_trajectory` (yellow) laid
across `/amr2/local_costmap/costmap`. **The yellow path extends *ahead* of amr1 — that
is the point.**

**Expected result** (`results/phase6_cost_injection_layer_{on,off}.md`):

| | layer ON | layer OFF |
|---|---|---|
| samples in window | 50 | 59 |
| **samples with cost > 0** | **50 (100 %)** | **0 (0 %)** |
| median cost at that cell | **145** | 0 |
| max cost | 240 | 0 |

The probe samples where amr1 is predicted to be **2 s from now**, not where it is —
sampling the current cell would measure the obstacle layer and report it as MAPF.

**Limitation, in these words.** RegulatedPurePursuit is not a sampling optimiser. It
consumes local costmap cost through cost-regulated velocity scaling and forward
collision checking, so the layer changes how it **paces**, not where it goes.
**"Robots mutually deviate without central intervention" is NOT demonstrated**, and no
run in this repository should be read as showing it. What is demonstrated is the
mechanism and the cost injection, measured against a control. Do not claim the
behavioural difference between arms either: n=1 per arm, and an earlier layer-on run
fell inside the same spread.

---

## Scenario 6 — Traffic control: the yield protocol

**Purpose.** A forced narrow-intersection conflict, resolved by the arbiter only after
the local layer has failed.

**Assignment requirement.** §3.2b — the Traffic Control Node enforces a pre-defined
yielding protocol by commanding a temporary controlled stop.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase7_yield.launch.py headless:=false rviz:=true
```

**RViz.** `rviz:=true`.  **Fixed Frame** `fleet_map`. Both `/plan` paths converging on
the same gap is the visual that makes "narrow intersection" obvious.

**Actors.** Off, hard-wired — a pedestrian standing in the 3.0 m gap would stop both
robots for a reason that is not the yield.

**Duration.** **3 min 37 s headless (measured)**, ~5–6 min with the GUI. The mission
dispatches at t = 34 s; amr2's goal is held back a further 5 s.

**Expected visual behaviour.** Frame Gazebo on the barrier at x = −5 so the **3.0 m
gap** and both approaching robots are in shot. The moment: **amr2 stops short of the
gap while amr1 drives through**, then amr2 resumes.

**What the operator should watch.** These three lines in the terminal:

```
conflict predicted amr1/amr2: closest approach 1.67 m within 4.0 s (radius 1.94 m) - local layer has it
ESCALATED amr1/amr2: unresolved for 2.0 s (closest approach 1.53 m, opened by -0.14 m) - amr2 yields to amr1
RELEASE amr2 after 15.2 s: conflict cleared (separation 2.43 m ...); recoveries during the hold: 0
```

**And in a second terminal:**

```bash
./ws.sh ros2 topic hz /amr2/cmd_vel_yield        # ~20 Hz while held, silent otherwise
./ws.sh ros2 param get /amr2/controller_server progress_checker.movement_time_allowance
#   1000000.0 during the hold, 10.0 after release
```

**What the presenter should explain.**

- **Priority comes from `fleet.yaml` gross mass** — amr1 90.0 kg > amr2 23.0 kg — so
  "the lighter AMR-2 yields to AMR-1" falls out of configuration. No robot is named in
  the arbiter, and `tests/test_traffic_policy.py` asserts that changing the masses
  changes the yield direction.
- **Conflict radius 1.94 m is derived, not tuned**: r(amr1) 0.539 + r(amr2) 0.429 +
  d_safe(amr1 at cruise) 0.975 — the distance at which one robot's SafetyGate would
  already be holding the other.
- **Local first.** The arbiter refuses to act while the predicted closest approach is
  still opening.
- **Gating without notification is a bug.** The arbiter raises Nav2's progress-checker
  allowance on entry, restores it on release, and reads it back rather than assuming.

**Expected result** (`results/phase7_yield.md`): two escalations, **amr2 yielded both
times**, held **1.0 s and 15.2 s**, both released on *conflict cleared* — never on the
45 s fail-safe. **0 recovery behaviours during either hold.** SafetyGate blocking on
**0 of 326 held cycles**, so the stop was the arbiter's alone. **Both goals SUCCEEDED.**

**Limitations and failure modes.**

| symptom | cause | fix |
|---|---|---|
| no `ESCALATED` line ever prints | the encounter did not overlap in time | relaunch, or raise `time_window_s:=5.0` |
| the run reports NOT EXERCISED | no conflict met the escalation test | relaunch; this is not a failure |
| `RELEASE ... max hold elapsed (fail-safe)` | the 45 s ceiling broke a deadlock | report it as a deadlock, not a successful yield — the report distinguishes them |
| both robots stop and neither moves | both entered the gap together | this is the failure the yield prevents; relaunch |

**Say this plainly: the encounter is staged, and staged is not deterministic.** Four
runs of this identical launch have given **3, 2, 1 and 0 escalations**. A run that
escalates nothing reports **NOT EXERCISED**, not FAIL.

---

## Scenario 7 — Safety override on a pedestrian

**Purpose.** Show a low-level, speed-dependent halt that overrides the navigation
stack, triggered by a moving obstacle.

**Assignment requirement.** §3.3 — safety system override, `d_safe = k·v² + d_min`,
issuing an immediate halt that bypasses the navigation stack's velocity command. Also
the evaluation criterion's *dynamic obstacles*.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase2_safety_run.launch.py headless:=false
```

**RViz.** **Not started by this launch** — it is a single-robot run with no
`FleetMapNode`, so `fleet_map` does not exist and the fleet config would come up
blank. The Gazebo view alone is sufficient here: the pedestrian, the robot and the gap
between them are all directly visible. If you do want RViz, start it separately with
**Fixed Frame `amr1/odom`**:

```bash
# second terminal
./ws.sh rviz2 --ros-args -p use_sim_time:=true
#   Fixed Frame amr1/odom; add LaserScan /amr1/scan and Path /amr1/plan
```

**Actors.** **On, hard-wired.** The walking pedestrian is the obstacle.

**Duration.** ~2–3 min. The goal dispatches at t = 27 s.

**Expected visual behaviour.** amr1 drives down the aisle, a pedestrian walks into its
path, and the robot **stops short of it**, then resumes once the walker clears. Three
times over the run.

**What the operator should watch.** The terminal:

```
HALT 1: clearance 0.412 m at 0.331 m/s, sensor->zero 8.5 ms
RELEASE: latch clearance 0.769 m > d_release 0.677 m ...
```

and live: `./ws.sh ros2 topic echo /amr1/safety_gate/diagnostics --once`

**What the presenter should explain.**

- `d_safe = k·v² + d_min`, with **v from odometry, never from the command** — a command
  is what the stack *wants*; stopping distance depends on what the vehicle is *doing*.
  `k` is per-robot and payload-scaled.
- **The gate is serial and last — never a `twist_mux` input.** A mux selects the
  highest-priority *live* input, so a dead safety node fails **open**. Serial and last,
  nothing else publishes the topic the plant reads.

**Expected result** (`results/phase2_safety_suppressed.md`,
`results/phase2_stopping_distance.md`): goal **SUCCEEDED**, **3 halts**, **0 commands
left the gate while latched**. Sensor stamp → zero command published: **mean 8.46 ms,
p95 11.00 ms, max 12.00 ms**; in-node compute 151 / 226 / 711 µs. Stopping sweep at
four commanded speeds, 0.15 → 0.60 m/s.

**Wording discipline.** "Low-latency safety override" — **not** hard real-time. No RT
kernel, no scheduling guarantees. The end-to-end figure is quantised by Gazebo's
`/clock` step; the compute figure is not.

**Limitation.** Actors cannot generate contacts, so **clearance is the measured
quantity and no "zero collisions" claim is made**.

---

## Scenario 8 — Fail-closed: SIGKILL the gate mid-motion

**Purpose.** Show that the safety element fails **closed**.

**Assignment requirement.** §3.3 — the argument for why SafetyGate is a serial last
link rather than a mux input.

**Command.**

```bash
# arm A — watchdog present
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=watchdog headless:=false

# arm B — the control
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py \
    tag:=no_watchdog with_watchdog:=false headless:=false
```

**RViz.** Not applicable — no SLAM and no Nav2 in this run.  **Actors** off.

**Duration.** ~1–2 min per arm.

**Expected visual behaviour.** Gazebo following the robot. The moment is the SIGKILL:
**arm A stops in 0.196 m; arm B keeps going.**

**Expected result** (`results/phase2_failclosed_*.md`):

| | distance after the kill | outcome |
|---|---|---|
| watchdog present | **0.196 m** | at rest in 0.75 s |
| watchdog absent | **3.500 m** | **still rolling at 0.350 m/s** when the window closed |

**What the presenter should explain.** SIGKILL, not SIGTERM — no shutdown handler runs
and upstream keeps commanding throughout. gz-sim 8.11.0's DiffDrive has **no command
timeout**, so being the only publisher stops new commands but not the latched one; the
watchdog models the motor controller that does have one.

---

## Scenario 9 — BSP: PitchGate on the ramp

**Purpose.** Show sensor validation doing functional work, not just logging.

**Assignment requirement.** §4.1 — the nav stack consumes sensor data only after a
BSP-style validation routine.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase2_pitch_gate.launch.py headless:=false
# after it finishes
./ws.sh ros2 run amr_bsp plot_pitch_gate.py
```

**RViz.** Not started by this launch. If you want it: **Fixed Frame `amr1/odom`**, and
add two LaserScan displays — `/amr1/scan` in red (raw) and `/amr1/validated/scan` in
white. The red returns ahead of the robot on the ramp are the phantom ground return;
the white ones are what Nav2 is allowed to see.

**Actors.** Off, hard-wired.  **Duration.** ~2–3 min.

**Expected visual behaviour.** The robot climbs the 8° ramp. As it pitches nose-up the
tail beams strike the ground ahead, and the raw scan grows a phantom arc that the
validated scan does not have.

**Expected result** (`results/phase2_pitch_gate.md`): max pitch **8.001°**; **120 of
360 beams truncated** in that scan; **closest return removed 2.769 m** (the phantom)
and **closest return kept 2.732 m** (the real rack, untouched); over the run 24 833
beams truncated, **7.65 %** of all beams; **0** cuts pointing uphill, **0** inside
their own gate, **0** inside the braking envelope. Raw/validated pairs matched by
**exact header stamp: 903, 0 restamped, max difference 0 ns**.

---

## Scenario 10 — BSP: implausible IMU rejection

**Purpose.** The assignment's literal validation requirement — log a warning when IMU
angular velocity exceeds a physically plausible limit.

**Assignment requirement.** §4.1 validation requirement.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase2_validation.launch.py mode:=imu_injection
```

**RViz.** Not applicable — **the terminal output is the demonstration here.**
**Actors** off.  **Duration.** ~2 min.

**Expected visual behaviour.** The injector publishes implausible samples, the
validator rejects them and logs warnings, and the report prints at the end.

**Expected result** (`results/phase2_imu_injection.md`): **500 samples injected at
50 rad/s against a 4.0 rad/s bound → 500 rejected, 0 reached `validated/imu`** (peak
|ω| there 0.001 rad/s), while **3 194 healthy samples were accepted** around the
window.

**What the presenter should explain.** Both halves matter — a validator that rejected
everything would produce an equally impressive rejection count. Injection is done by a
**separate process**, so the BSP ships with no fault-simulation code in it. And no node
in the navigation stack subscribes to a raw sensor topic: the contract is written down
in exactly one place, `amr_bsp/topics.py`.

---

## Scenario 11 — Recovery suppression A/B

**Purpose.** Show why anything that zeroes `cmd_vel` must also tell Nav2.

**Assignment requirement.** Not a numbered requirement — a design rule, and the one an
evaluator is most likely to probe.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py tag:=suppressed
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py \
    tag:=control suppress_recovery:=false
```

**RViz.** Not started.  **Actors** off.  **Duration.** ~5–8 min per arm — the goal
deliberately ends in TIMEOUT because the barrier never moves.

**This one is better quoted than run live.** Show the two reports side by side.

**Expected result:**

| | suppressed | control |
|---|---|---|
| **longest halt held** | **56.69 s** | **10.00 s** |
| `Failed to make progress` | **0** | **6** |
| recovery behaviours during a halt | **0** | **2** |
| commands leaked past the latch | 0 | 0 |

**Limitation.** A full-twist hold against an obstacle that never moves is a deadlock by
design — the goal ends in TIMEOUT in *both* arms. That is correct for a fail-closed
gate, and "allow in-place rotation under a hold" is flagged as future work rather than
claimed.

---

## Scenario 12 — The whole system, no mission attached

**Purpose.** Bring up everything at once: warehouse, both robots, BSP, SLAM, fleet map,
both Nav2 stacks, motion chain, safety gates, trajectory layer and arbiter.

**Assignment requirement.** §5.1 — launch the entire simulation, navigation and custom
control stack.

**Command.**

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup fleet_nav.launch.py headless:=false rviz:=true
```

**RViz.** `rviz:=true`.  **Fixed Frame** `fleet_map`.  **Actors** on by default.

**Duration.** Runs until you Ctrl-C — it has no mission and does not self-terminate.

**Expected result.** **65 unique node names — `ros2 node list` prints 111 entries**,
because several nodes advertise from more than one DDS participant. Every lifecycle
node reaches `active`, and `/fleet_map` is published with both global costmaps
subscribed. Use Scenario 2 to see it do something.

---

## Global failure modes

| symptom | what it actually is | recovery |
|---|---|---|
| `Failed to find a free participant index for domain 0` | a command ran without `./ws.sh` | kill everything, relaunch through `./ws.sh` — **including RViz** |
| RViz blank, **"Frame [map] does not exist"** | RViz is on its stock config; this system publishes no bare `map` frame | use `rviz:=true`, or `-d src/amr_bringup/rviz/fleet_mapping.rviz`; for single-robot demos set Fixed Frame `amr1/odom` |
| RViz shows the config but nothing in it | Fixed Frame not yet published (FleetMapNode starts at t≈16 s), or a QoS mismatch on a hand-added display | wait; check `ros2 topic info -v` |
| **no pedestrians in the world** | that launch defaults `with_actors:=false` | see §2 — pass `with_actors:=true` |
| numbers look wrong in a way that suggests a code bug | a leaked node from an earlier launch is still publishing | clean, confirm the banner, re-run. Retract nothing until you have re-measured on a verified-clean table |
| launch appears to hang 20–30 s in | staged bringup, or a lifecycle node waiting on a transform | wait 45 s; if still nothing, clean and relaunch |
| `gz sim` window black or world empty | a previous `gz sim` survived and holds the port | `./scripts/clean_processes.sh` |
| a robot spins in place at a standstill | Nav2 recovery — the progress checker fired | that is Scenario 11's failure mode; check `movement_time_allowance` |
| a run overwrote an artifact you wanted | same `tag:` | re-run with `tag:=something_else` |

---

## Command sheet

```bash
# every time
./scripts/clean_processes.sh

# 1  cooperative mapping            (GUI + RViz on by default)
./ws.sh ros2 launch amr_bringup fleet_survey.launch.py

# 2  simultaneous goals + pedestrians
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    headless:=false rviz:=true with_actors:=true tag:=demo

# 3  ramp mask  (PARTIAL - prefer quoting README section 10)
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    ramp_mask_value:=60.0 tag:=ramp_graded

# 4  payload-aware motion           (deliverable is results/phase5_payload_trace.png)
./ws.sh ros2 launch amr_bringup phase5_payload_trace.launch.py

# 5  MAPF cost injection A/B
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py headless:=false rviz:=true
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py \
    with_trajectory_layer:=false tag:=layer_off

# 6  yield protocol
./ws.sh ros2 launch amr_bringup phase7_yield.launch.py headless:=false rviz:=true

# 7  safety override on a pedestrian   (actors always on)
./ws.sh ros2 launch amr_bringup phase2_safety_run.launch.py headless:=false

# 8  fail-closed under SIGKILL (two arms)
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=watchdog headless:=false
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py \
    tag:=no_watchdog with_watchdog:=false headless:=false

# 9  BSP PitchGate on the ramp
./ws.sh ros2 launch amr_bringup phase2_pitch_gate.launch.py headless:=false
./ws.sh ros2 run amr_bsp plot_pitch_gate.py

# 10 implausible IMU rejection
./ws.sh ros2 launch amr_bringup phase2_validation.launch.py mode:=imu_injection

# 11 recovery suppression A/B  (quote, do not run live)
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py tag:=suppressed
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py \
    tag:=control suppress_recovery:=false

# 12 the whole system, no mission
./ws.sh ros2 launch amr_bringup fleet_nav.launch.py headless:=false rviz:=true

# observers
./ws.sh rviz2 -d src/amr_bringup/rviz/fleet_mapping.rviz --ros-args -p use_sim_time:=true
./ws.sh ros2 topic hz /amr2/cmd_vel_yield
./ws.sh ros2 param get /amr2/controller_server progress_checker.movement_time_allowance
./ws.sh ros2 topic echo /amr1/safety_gate/diagnostics --once
./ws.sh ros2 node list | wc -l
```
