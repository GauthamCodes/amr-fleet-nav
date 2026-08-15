# Demo Runbook

Everything needed to drive the screenshare without debugging live. Every command in
this file was checked against the launch file it names, and every number quoted is in
a file in `results/`, so nothing has to be re-measured on camera.

**Read sections 1–4 once before recording. Then each demo is self-contained.**

---

## 0. Conventions

| | |
|---|---|
| **Working directory** | the repo root. Every command assumes it. |
| **T1** | the launch terminal. One launch at a time, always. |
| **T2** | the observer terminal — `ros2 topic echo`, `ros2 param get`, `cat results/…` |
| **T3** | spare, for the pre-flight cleanup between runs. |
| **`./ws.sh`** | the workspace wrapper. **Every** ROS/Gazebo command goes through it, including `rviz2`. Section 2 says why. |

RViz no longer needs a terminal of its own: every two-robot demo takes `rviz:=true`
and starts it inside the launch (section 3).

Every evidence launch is **self-terminating**: it brings the world up, runs its
scenario, writes its report into `results/`, prints it to the terminal and shuts
itself down. There is no Ctrl-C step and no "now run the analysis" step. If you
Ctrl-C early you lose that run's report.

**Durations** are wall clock on this machine. Ones marked *(measured)* were timed;
the rest are scaled from their sim-time scenario length and are marked *(est.)*.
**Adding `headless:=false` costs roughly 50–80 % more** — the GUI renders every frame.

---

## 1. Pre-flight — before **every** launch, no exceptions

```bash
# T3
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
retracted — README §10, item 12. A process-hygiene failure is a measurement failure.

Then, once at the start of the session:

```bash
# T3 — the build is current (only needed after an edit; --symlink-install means
# Python changes are already live, but C++ and launch/config/rviz installs are not)
./ws.sh colcon build --symlink-install     # expect: 9 packages finished, 0 errors

# T3 — the test suite
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
that error, you ran something without `./ws.sh`.**

---

## 3. RViz — one saved config, started by the launch

There is a saved config at `src/amr_bringup/rviz/fleet_mapping.rviz`, and a launch
argument that starts RViz with it already loaded:

```bash
# T1 — RViz comes up inside the launch, so it inherits ./ws.sh's environment
./ws.sh ros2 launch amr_bringup fleet_survey.launch.py                    # §5 demo 1
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    headless:=false rviz:=true with_actors:=true tag:=demo
```

`rviz:=true` is available on **`fleet_nav`, `fleet_survey`, `phase3_fleet_goals`,
`phase6_conflict` and `phase7_yield`** — i.e. every two-robot demo.
`rviz_config:=<path>` overrides the file; leaving it empty means the default above.
**Prefer this to starting RViz by hand** — a launched RViz cannot be the one process
in the graph that missed `./ws.sh` (§2).

To attach RViz to a run that is already going, or for the single-robot demos:

```bash
# T2
./ws.sh rviz2 -d src/amr_bringup/rviz/fleet_mapping.rviz --ros-args -p use_sim_time:=true
```

`use_sim_time` matters: without it RViz reads the wall clock, every transform
looks stale, and the whole scene either flickers or never appears.

### Fixed Frame

**`fleet_map`.** That frame is published by `FleetMapNode` as the static parent of
every robot's map frame, and it is the frame both global costmaps plan in. It is
verified to exist at runtime — `ros2 run tf2_ros tf2_echo fleet_map amr1/base_link`
resolves once `FleetMapNode` is up at t ≈ 16 s.

**Do not use `map`.** No node in this system publishes a bare `map` frame: every
robot's SLAM map frame is namespaced `amrN/map`, and the shared root is `fleet_map`.
Starting a bare `rviz2` with no config gives you the stock config, whose Fixed Frame
*is* `map` — which is why an unconfigured RViz comes up blank with
**"Fixed Frame — Frame [map] does not exist"**. That is the symptom, and the saved
config is the fix.

For the single-robot demos (5, 6, 7, 8) there is no `fleet_map`; use `amr1/odom`.

### What is in the config

On by default — this is the cooperative mapping shot:

| Display | Topic | Notes |
|---|---|---|
| **Fleet map** | `/fleet_map` | the composite. Transient Local, colour scheme `map` |
| **amr1 model** | `/amr1/robot_description` | **blue** chassis |
| **amr2 model** | `/amr2/robot_description` | **amber** chassis |
| **amr1 scan** | `/amr1/scan` | green, 0.08 m points |
| **amr2 scan** | `/amr2/scan` | orange, 0.08 m points |
| **amr1 plan** | `/amr1/plan` | green path |
| **amr2 plan** | `/amr2/plan` | cyan path |
| **TF** | — | key frames only: `fleet_map`, each robot's `map`/`odom`/`base_link`/`laser` |

Off by default, one click each in the Displays panel:

| Display | Turn on for |
|---|---|
| amr1 / amr2 **own map** | showing the composite really is a fusion of two maps |
| amr1 / amr2 **global costmap** | showing the fleet map is *used* as the static layer |
| amr1 / amr2 **predicted trajectory** | demo 9 — the MAPF message |
| amr1 / amr2 **local costmap** | demo 9 — where the peer's trajectory is deposited |

Three saved viewpoints are in the Views panel: *Warehouse top-down* (default),
*Aisle close-up*, *Angled (robots in 3D)*.

**The two robots are told apart by chassis colour**, which comes from `body_color`
in `config/fleet.yaml` — RViz's RobotModel display has no colour property, so the
difference has to live in the description. amr2 is also visibly the smaller chassis.

**If a display stays blank**, the usual cause is a QoS mismatch, not a missing
publisher. The maps and costmaps are `RELIABLE` + `TRANSIENT_LOCAL`; everything else
is `RELIABLE` + `VOLATILE`. The saved config already matches; a display you add by
hand will not.

**Nothing appears for the first ~25 s of any launch.** That is the staged bringup,
not a fault.

---

## 4. Dynamic obstacles — which demos have them, and why

The warehouse contains **three scripted pedestrians** (`pedestrian_1` along the main
aisle, `pedestrian_2` across the ramp approach and into the robot's path,
`pedestrian_3` on the upper plateau). They are Gazebo `<actor>` entities on looping
walk trajectories, and they are **verified to reach the navigation stack**:
`results/smoke1_actor_visibility.md` records them raycast by the LiDAR and marked
**254 LETHAL in the Nav2 costmap in 100 % of frames**.

They are controlled by the `with_actors` launch argument, and **the default differs
per launch on purpose**: a walking pedestrian is an uncontrolled variable, so the
measurement runs switch them off and the demonstration runs switch them on.

| Launch | Actors | How |
|---|---|---|
| `fleet_nav.launch.py` | **ON** | default `with_actors:=true` |
| `phase2_safety_run.launch.py` | **ON, always** | hard-wired — the pedestrian *is* the demo |
| `phase1_nav_run.launch.py`, `amr1_nav.launch.py` | **ON** | default true |
| `phase3_fleet_goals.launch.py` | off | pass **`with_actors:=true`** to switch on |
| `phase6_conflict.launch.py` | off | pass `with_actors:=true`; off by default because an actor puts cost into the costmap the probe reads |
| `fleet_survey.launch.py` | off | pass `with_actors:=true`; off by default because `slam_toolbox` has no dynamic-object rejection and traces a walker into the map as a smear |
| `phase7_yield.launch.py` | **off, always** | hard-wired — an actor standing in the 3.0 m gap would stop both robots for a reason that is not the yield |
| all other `phase2_*`, `phase1_survey` | off | hard-wired; each is a controlled measurement |

**For the recording**: demo 5 already has pedestrians. Add `with_actors:=true` to
demo 3 to show them in the two-robot run — and add `tag:=demo` with it, so the run
does not overwrite the committed Phase 3 evidence, which was measured without them.

*Verified this session:* `phase3_fleet_goals.launch.py … with_actors:=true` renders
all three actors into the world and **both goals still SUCCEEDED** (amr1 18.8 s,
amr2 11.3 s, RESULT: PASS).

**One thing not to claim:** Gazebo actors are not physics entities — they are absent
from `pose/info` and `gz model --list` and can never generate contacts. They are
rendered and they are raycast, which is what navigation needs. **Clearance, not
collision avoidance, is the honest quantity** (README §10, item 11).

---

## 5. The demos

Each block is: what it proves → the commands → what you will see → the numbers to
say out loud → what can go wrong → how to stop it.

**Stopping any demo.** All of them self-terminate. To end one early, Ctrl-C in T1
once and wait for the process table to drain, then run
`./scripts/clean_processes.sh` in T3 and confirm the CLEAN banner before the next
launch. Ctrl-C before a run writes its report loses that run's artifact.

---

### Demo 1 — Cooperative mapping: one map, two contributors *(the headline)*

**Requirement.** §2.1 cooperative global SLAM and map fusion — both AMR-1 and AMR-2
contributing to a **single unified** occupancy grid. This is the assignment's first
evaluation criterion, and it is only legible in RViz.

**Purpose.** Show `/fleet_map` starting empty and filling in from two directions at
once, in a frame both robots share.

```bash
# T3
./scripts/clean_processes.sh
# T1 — Gazebo GUI and RViz both come up from this one command
./ws.sh ros2 launch amr_bringup fleet_survey.launch.py
```

**Terminals.** T1 only. RViz (`rviz:=true`) and the Gazebo GUI (`headless:=false`)
are both **on by default in this launch** — it is the one demo built for the camera.

**Duration** ~3 min headless *(measured: 110 s of driving plus bringup)*; ~4–6 min
with the GUI *(est.)*. `laps:=2` roughly doubles the drive and gives `slam_toolbox`
a genuine revisit to close a loop against.

**Startup sequence.**

| t (sim s) | what starts |
|---|---|
| 5, 8 | amr1 spawns, then amr2 (Gazebo serialises world/create requests) |
| 10, 12 | each robot's SensorBSP |
| 12 | **RViz** — before the map, so the first thing it draws is the map appearing |
| 14, 16 | each robot's `slam_toolbox` |
| 16 | **FleetMapNode** — publishes the `fleet_map → amrN/map` frames and an empty grid |
| 28 | both survey drives start |

**What should visibly happen.** RViz opens on the top-down view with an empty grid.
From ~t=16 s a mostly-unknown `/fleet_map` appears. Both robots then drive the same
closed circuit **phase-shifted by half a lap** — eastbound in the south lane,
westbound in the north lane — so they are always on opposite sides of the loop. The
map fills in from **both ends of the aisle at once**, and the rack rows resolve into
the toothed pattern north and south of the aisle.

**What to point out on camera.**

1. **One map, two contributors.** Toggle *amr1 own map* and *amr2 own map* on: each
   covers part of the aisle; `/fleet_map` covers the union. That is the fusion.
2. **One frame.** In the TF display, both robots' trees hang off a single
   `fleet_map` root. Fixed Frame is `fleet_map` and Global Status is Ok.
3. **Two viewpoints of the same racks** — the reason the composite is worth looking
   at rather than one robot's map with a second robot driving over it.
4. **The accept/defer decisions scrolling in T1** — that is demo 2, running live.

**Expected result.** Both drives complete, the launch shuts itself down, and
`results/fleet_survey_updates.{md,csv}` holds that run's accept/defer report.
The file is rewritten after every candidate, so `cat` it mid-run to watch coverage
climb. *Verified this session: 91 candidates, 65 accepted, 26 deferred (28.6 %),
fleet map 17.1 % known with 2 488 occupied cells.*

**Why no Nav2 here.** This run maps; it does not navigate. Twelve Nav2 servers
evaluating plans against a map still being built spend CPU the simulator needs for
scans. **`/amrN/plan` therefore stays empty and the two Path displays stay blank** —
that is expected. Demo 3 is where those displays carry something.

**Failure modes.**

| symptom | cause | fix |
|---|---|---|
| RViz blank, "Frame [map] does not exist" | RViz started by hand with no config | use the launch's `rviz:=true`, or pass `-d src/amr_bringup/rviz/fleet_mapping.rviz` |
| map appears but never grows | a survey drive died | check T1 for a `survey_drive` traceback; clean and relaunch |
| only one robot in Gazebo | two `world/create` requests raced | clean and relaunch; the launch staggers spawns by 3 s |
| a smeared wall across the aisle | you passed `with_actors:=true` | expected — SLAM has no dynamic-object rejection; leave actors off for this demo |

---

### Demo 2 — Selective map updates

**Requirement.** §2.1 selective mapping — prioritise unexplored boundaries, reduce
update frequency for repeatedly traversed areas. Evaluation criterion *System
Optimization*: "programmatically restrict the map update area based on a defined
condition."

**Run.** No separate launch. It is the policy running inside demo 1 and demo 3;
the canonical evidence comes from demo 3's run, because the policy has to be
measured *while both robots are exploring* — a stationary fleet defers everything
and would make the policy look effective while proving nothing.

**What "working" looks like.** In T1, `fleet_map_node` logs a line per candidate
with the four score terms:

```
amr1: ACCEPT score=+2.416 (f=1.00 c=1.00 r=0.05 v=0.06) score >= 0.35
amr2: DEFER  score=+0.216 (f=0.00 c=0.00 r=0.32 v=0.17) score < 0.35
```

**Numbers to quote** — `results/phase3_selective_updates.md`, the committed evidence:

- **41 candidates scored, 23 accepted, 18 deferred — 43.9 %**.
- Per robot: amr1 13/8 (38.1 % deferred), amr2 10/10 (50.0 %).
- 17 composites, **mean composite + publish 0.55 ms**; fleet map ended
  15.6 % known with 1 984 occupied cells.
- The score is `w_f·frontier + w_c·occupancy_change + w_r·recency − w_v·revisit`,
  accept threshold 0.35.

**Quote the committed numbers, not the live ones.** A live run writes its own file
(`results/fleet_survey_updates.md` for demo 1) with its own counts — the deferred
share moves run to run. **43.9 % is the figure on disk in the Phase 3 artifact**;
say that one.

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

### Demo 3 — Dual bringup, concurrent goals, and the fleet map in use

**Requirement.** §2.2a concurrent navigation goals for both robots — and, more
importantly, that the merged map is *used*: it is the static layer of **both**
robots' global costmaps, so both plan in the fleet frame.

```bash
# T3
./scripts/clean_processes.sh
# T1 — with pedestrians, and a tag so it cannot overwrite the committed evidence
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    headless:=false rviz:=true with_actors:=true tag:=demo
```

Drop `with_actors:=true tag:=demo` to reproduce the committed artifact exactly.

**Terminals.** T1 for the launch; T2 for the live verification below.

**Duration** ~2 min headless *(measured: 67 s of graph activity plus bringup)*;
~4–5 min with the GUI *(est.)*.

**Startup sequence**, in sim seconds — useful to narrate while nothing is happening:

| t | what starts |
|---|---|
| 5, 8 | amr1 spawns, then amr2 |
| 10, 12 | each robot's SensorBSP |
| 12 | RViz |
| 14, 16 | each robot's `slam_toolbox` |
| 16 | FleetMapNode and the costmap-filter servers |
| 22, 24 | each robot's six Nav2 servers |
| 23, 25 | each robot's SafetyGate |
| 24, 26 | each robot's TrajectoryPredictor |
| 30 | TrafficControlNode (the fleet arbiter) |
| 32 | the mission dispatches **both goals in one pass** |

**What should visibly happen.** Gazebo first — two visibly different chassis, amr2
smaller, and pedestrians walking the aisle. Then RViz: at t≈32 s **both `/plan`
paths appear at the same instant**, green for amr1 and cyan for amr2, running east
down their own lanes. amr2 arrives first.

**What to point out on camera.**

1. **Both plans appear together** — concurrent, not sequential.
2. **amr2 arrives first because `fleet.yaml` says it is faster** (`max_vel_x` 1.00
   against 0.60). The same file shapes its URDF, mass, acceleration limits and
   safety gain. **No code distinguishes the robots.**
3. Toggle both **global costmap** displays on: the fleet map is arriving as their
   static layer. That is "the map is used, not just published".
4. The pedestrians, and the fact that the robots plan around them.

**Verify live, in T2:**

```bash
./ws.sh ros2 topic info -v /fleet_map | head -20      # 2 subscribers, both global costmaps
./ws.sh ros2 param get /amr1/global_costmap/global_costmap global_frame   # fleet_map
./ws.sh ros2 param get /amr2/global_costmap/global_costmap global_frame   # fleet_map
./ws.sh ros2 node list | wc -l                        # 53
```

**Numbers to quote** (`results/phase3_concurrent_goals.md`):

| | amr1 | amr2 |
|---|---|---|
| result | **SUCCEEDED** | **SUCCEEDED** |
| time to goal | 18.8 s | 11.3 s |
| planned / driven (ground truth) | 10.50 / 10.44 m | 10.50 / 10.46 m |
| final position error | 0.062 m | 0.042 m |
| replans (route actually changed) | 0 | 0 |

- **53 nodes**, two Nav2 stacks, two `slam_toolbox`, two SensorBSP, two SafetyGates
  — no node-name or topic collision, every lifecycle node reached `active`.
- `/fleet_map` is `RELIABLE · TRANSIENT_LOCAL · KEEP_LAST(1)` with **2 matched
  subscribers**: both global costmaps.
- Grid **680 × 400 cells at 0.05 m**, origin (−15.0, −10.0).
- Closest approach **3.000 m** over 2 180 samples, median 3.471 m.

**Say the limitation out loud:** the inter-map transform is **fixed from the spawn
poses and not corrected** by occupancy correlation. That was a deliberate cut; drift
is not measured and is not claimed. And these two routes are deconflicted *by
design* — the forced conflict is demo 10.

**Failure modes.**

| symptom | cause | fix |
|---|---|---|
| six of amr2's servers die, `Failed to find a free participant index` | launched without `./ws.sh` | relaunch through `./ws.sh` |
| bringup hangs at ~22 s with no error naming a frame | FleetMapNode did not start before Nav2 | it is ordered in the launch; only happens if you start pieces by hand |
| goal ABORTED, `Failed to create a plan from potential` | NavFn transient while the map is still filling in ahead of the robot | the run retries a bounded number of times; if it persists, clean and relaunch |

---

### Demo 4 — Ramp / slope planning **(partial — state the limitation)**

**Requirement.** §2.2b the global planner must cost sloped surfaces, minimising
their use unless they are the only viable path.

**Status: plumbing verified, graded cost NOT confirmed.** Run this only if you want
to show the mechanism; **do not claim the behaviour.**

```bash
# T3
./scripts/clean_processes.sh
# T1 — the graded arm: a mask value of 60 over the ramp footprint
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    ramp_mask_value:=60.0 tag:=ramp_graded
```

**Duration** ~2 min headless *(est., same run as demo 3)*.

**What is true.** A Nav2 `KeepoutFilter` costmap-filter mask over the ramp footprint,
generated (not hand-drawn) by `amr_navigation/ramp_mask.py`, loads into both global
costmaps — both log `Received filter mask`. The null mask is proven to contribute
nothing: **minimum cost over 272 000 known cells = 0**.

**What is not true.** At mask value 60 the cost over the 3 550-cell ramp footprint
is **0..100 — identical to the null run** (`results/phase3_ramp_cost_graded.md`).
Leading hypothesis, stated as a hypothesis: Jazzy's `KeepoutFilter` maps mask values
to cost **binarily** (100 → LETHAL, below → nothing) rather than proportionally.

**Also say:** the two-route A/B the plan called for — *flat alternative exists → ramp
avoided; flat route blocked → ramp taken* — **cannot be staged in this world.** The
upper plateau is a solid box spanning the full y extent, so the ramp is the only
route up and nothing on the lower level needs it. README §10, items 1 and 2.

**Recommendation for the video: quote this one from the README rather than running
it.** It costs two minutes of screen time to show a mask that does not change the
cost.

---

### Demo 5 — Safety override: halt on a pedestrian, and the latency

**Requirement.** §3.3 low-latency safety override, `d_safe = k·v² + d_min`, issuing
an immediate halt that bypasses the navigation stack. Evaluation criterion *Safety &
Override*, including **dynamic obstacles**.

```bash
# T3
./scripts/clean_processes.sh
# T1
./ws.sh ros2 launch amr_bringup phase2_safety_run.launch.py headless:=false
```

**Terminals.** T1; T2 for the diagnostics echo below.

**Duration** ~2–3 min *(est.)*. Single robot; goal dispatches at t = 27 s.
**Pedestrians are always on in this launch** — the walking actor is the obstacle.

**RViz** (optional, T2): Fixed Frame **`amr1/odom`**, not `fleet_map` — this is a
single-robot launch and no `FleetMapNode` runs.

**What should visibly happen.** Gazebo framed on the aisle: amr1 drives, a
pedestrian walks into its path, and the robot **stops short of it**, then resumes
once the walker clears. Three times over the run. Then cut to T1 for the log lines:

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

- Goal **SUCCEEDED**, **3 halts**, and **0 commands left the gate while latched** —
  counted *inside* the gate, comparing the twist about to be published against the
  latch at that instant.
- **sensor stamp → zero command published: mean 8.46 ms, p95 11.00 ms, max
  12.00 ms.** In-node compute on the steady clock: 151 / 226 / 711 µs.

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
  its 60 kg payload. `k = 1/(2·a_eff)`, `a_eff = |max_decel_x|·m_base/(m_base+m_payload)`.

**Wording discipline:** "low-latency safety override". **Not** hard real-time — no
RT kernel, no scheduling guarantees. The end-to-end figure is quantised by Gazebo's
`/clock` step under `use_sim_time`; the compute figure is not.

**Say the honest version:** velocity comes from **odometry, never from the command**,
and the "settled" column is the `d_min` standoff the robot creeps to with the
command still applied — the specified end state of a speed-dependent gate, not a
leak in it. And clearance, not collisions, is what is measured: actors cannot
generate contacts.

---

### Demo 6 — Fail-closed: SIGKILL the gate mid-motion

**Requirement.** §3.3 — the safety element must fail **closed**. This is the
argument for why SafetyGate is a serial last link and never a `twist_mux` input.

```bash
# T3
./scripts/clean_processes.sh
# T1 — arm A, the watchdog present
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=watchdog headless:=false
# T3
./scripts/clean_processes.sh
# T1 — arm B, the control
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=no_watchdog \
    with_watchdog:=false headless:=false
```

**Duration** ~1–2 min each *(est.)*. No SLAM, no Nav2 — the run needs a constant
command into the gate and nothing else.

**What should visibly happen.** Gazebo following the robot. The moment is the
SIGKILL: **arm A stops in 0.196 m; arm B keeps going.**

**Numbers to quote** (`results/phase2_failclosed_*.md`):

| | distance after the kill | outcome |
|---|---|---|
| plant-side watchdog present | **0.196 m** | at rest in 0.75 s |
| watchdog absent (control) | **3.500 m** | **still rolling at 0.350 m/s** when the 10 s window closed |

- **SIGKILL, not SIGTERM** — no shutdown handler runs. Upstream keeps commanding
  throughout.
- gz-sim 8.11.0's DiffDrive has **no command timeout**, so being the only publisher
  of `cmd_vel` stops new commands but not the latched one. The watchdog models the
  motor controller that does have one. Real hardware stops when its command stream
  stops; the simulator does not, and the difference is stated rather than glossed.
- The control needed one correction to be meaningful: with the watchdog simply
  removed, nothing bridges `cmd_vel` to `cmd_vel_plant` and the robot never moved at
  all (peak 0.000 m/s) — that measures a severed command path and nothing else. The
  gate now addresses the plant directly in that arm.

**Why a mux would be wrong:** a mux picks the highest-priority *live* input, so a
dead safety node lets ordinary navigation straight through — it fails **open**. The
gate is serial: nothing else publishes the topic the plant listens to.

---

### Demo 7 — BSP / sensor validation: PitchGate and the IMU validator

**Requirement.** §4.1 BSP-style validation — the nav stack consumes LiDAR and IMU
only after validation, and the routine **logs a warning when IMU angular velocity
exceeds a plausible limit**.

```bash
# T3
./scripts/clean_processes.sh
# T1
./ws.sh ros2 launch amr_bringup phase2_pitch_gate.launch.py headless:=false
# T2, after it finishes
./ws.sh ros2 run amr_bsp plot_pitch_gate.py
```

**Duration** ~2–3 min *(est.; the drive itself is 90 s of sim time)*.

**RViz** (optional, T2): Fixed Frame **`amr1/odom`**. Add two LaserScan displays by
hand — `/amr1/scan` in red (raw) and `/amr1/validated/scan` in white. The red
returns ahead of the robot on the ramp are the phantom ground return; the white ones
are what Nav2 is allowed to see.

**What should visibly happen.** Gazebo: the robot climbs the 8° ramp. As it pitches
nose-up, the tail beams start striking the ground ahead and the raw scan grows a
phantom arc that the validated scan does not have.

**Numbers to quote** (`results/phase2_pitch_gate.md`):

- Raw/validated pairs matched by **exact header stamp: 903, 0 restamped, max
  difference 0 ns**. Pairing by exact stamp means a restamp shows up as *zero*
  matched pairs rather than as plausible numbers computed against mismatched data.
- Max pitch **8.001°**, nose-up, so the **tail** beams strike the ground.
- Predicted ground intersection `h/sin|θ|` = 2.514 m; gate radius at that pitch
  2.263 m; **120 of 360 beams truncated** in that scan.
- **Closest return removed: 2.769 m** — the phantom. **Closest return kept:
  2.732 m — the real rack, untouched.**
- Over the run: 24 833 beams truncated, **7.65 %** of all beams seen.
- **0** cuts pointing uphill, **0** inside their own gate, **0** inside the braking
  envelope. The interlock `min_gate_range` = 1.565 m for amr1 is computed from the
  braking envelope plus the footprint, so truncation can never delete an obstacle
  the SafetyGate would have had to stop for.

**The IMU validator**, which is the requirement's literal wording:

```bash
./ws.sh ros2 launch amr_bringup phase2_validation.launch.py mode:=imu_injection
```

→ **500 injected at 50 rad/s against a 4.0 rad/s bound, 500 rejected, 0 reached
`validated/imu`**, while **3 194** healthy samples were accepted around the window.
Both halves matter — a validator that rejected everything would produce an equally
impressive rejection count. Injection is done by a **separate process**, so the BSP
ships with no fault-simulation code in it.

**Also worth one sentence:** when the IMU stream was rejected wholesale in that test,
PitchGate lost its attitude and republished scans **untruncated** with a throttled
WARN — the degraded-but-safe path its docstring specifies, exercised for real.

---

### Demo 8 — Recovery suppression A/B (gating without notification is a bug)

**Requirement.** Not a numbered requirement — a design rule, and the one an
evaluator is most likely to find interesting: anything that zeroes `cmd_vel` must
tell Nav2, or the progress checker declares the robot stuck and fires recovery
behaviours into a robot that must not move.

```bash
# T3
./scripts/clean_processes.sh
# T1 — suppressed
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py tag:=suppressed
# T3
./scripts/clean_processes.sh
# T1 — control
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py \
    tag:=control suppress_recovery:=false
```

**Duration** ~5–8 min per arm *(est.)*: the goal deliberately ends in TIMEOUT
against a 100 s run timeout, because the barrier never moves.

**This one is better *quoted* than run live** — two arms at 5–8 min each is most of
a video. Show `results/phase2_recovery_ab_suppressed.md` and
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
- **The discriminator is the halt DURATION and the six progress-checker failures,
  not the raw recovery count** — in the control arm the progress-checker failure and
  the halt release happen at the same instant, so a recovery can land just outside
  the attributed window.
- **This took two phases to prove.** An earlier attempt could only show the
  mechanism *operates*: its longest halt was 0.80 s against a 10 s allowance, so the
  checker was never going to fire and the control arm's zero proved nothing. That
  was recorded as an honest non-result. An immovable barrier spanning the full 5.5 m
  aisle is what finally produced a halt that outlasts the allowance.
- **The limitation this exposes:** a full-twist hold against an obstacle that never
  moves is a deadlock — the robot has no legal action that could increase its
  clearance, so the goal ends in TIMEOUT in *both* arms. That is correct for a
  fail-closed gate, and it is why "allow in-place rotation under a hold" is flagged
  as future work rather than claimed.

---

### Demo 9 — MAPF: one robot's intention as cost in another's costmap

**Requirement.** §3.2a — the local planner for **each robot** consumes the
**projected trajectory of the other robot**. **This is the requirement an evaluator
will probe hardest.**

```bash
# T3
./scripts/clean_processes.sh
# T1 — layer ON
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py headless:=false rviz:=true
# T3
./scripts/clean_processes.sh
# T1 — layer OFF, the control arm
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py \
    with_trajectory_layer:=false tag:=layer_off
```

**Duration** ~5–6 min per arm *(est.; the probe runs 150 s of sim time)*. **Run arm
A live, quote arm B** if time is short.

**RViz.** Fixed Frame `fleet_map`. In the Displays panel switch on the four
off-by-default toggles: **amr1/amr2 predicted trajectory** and **amr1/amr2 local
costmap**. The shot is `/amr1/predicted_trajectory` (yellow) laid across
`/amr2/local_costmap/costmap`. Use the *Aisle close-up* saved view so both robots
and the crossing are in frame. **The yellow path extends *ahead* of amr1 — that is
the point.**

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
  marks and inflates regardless, so sampling the peer's *current* cell would measure
  the obstacle layer and report it as MAPF.
- The layer deposits `max_cost · exp(−Δt/τ)` over a disc per predicted peer pose,
  combined with `std::max` so it can raise a cell's cost and never lower it.
- Cost is capped below `INSCRIBED_INFLATED_OBSTACLE` (253) by construction: a peer's
  *intention* must never read as a collision to a footprint checker.
  `tests/test_trajectory_conflict.py` asserts the shipped YAML respects the cap.

**Say the limitation, in these words:** RegulatedPurePursuit is not a sampling
optimiser. It consumes local costmap cost through cost-regulated velocity scaling
and forward collision checking, so the layer changes how it **paces**, but it does
not deviate laterally around graded cost the way MPPI's CostCritic would.
**"Robots mutually deviate without central intervention" is therefore NOT
demonstrated**, and no run in this repo should be read as showing it. What is
demonstrated is the mechanism and the cost injection, measured against a control.

**Do not claim the behavioural difference.** amr1 drove 8.30 m in 18.0 s with the
layer and 9.03 m in 20.6 s without; closest approach 1.610 m against 1.529 m — but
that is n=1 per arm and an earlier layer-on run gave 1.538 m, inside the same spread.

---

### Demo 10 — The yield protocol: escalation from the local layer to the arbiter

**Requirement.** §3.2b — the Traffic Control Node enforces a pre-defined yielding
protocol (the lighter AMR-2 yields to the heavier AMR-1) by commanding a temporary
controlled stop. **This is the demo that shows the ordering: local first, central
only when local has failed.**

```bash
# T3
./scripts/clean_processes.sh
# T1
./ws.sh ros2 launch amr_bringup phase7_yield.launch.py headless:=false rviz:=true
```

**Duration** **3 min 37 s headless *(measured)*;** ~5–6 min with the GUI *(est.)*.
The mission dispatches at t = 34 s; amr2's goal is held back a further 5 s.

**What should visibly happen.** Gazebo framed on the barrier at x = −5 so the
**3.0 m gap** and both approaching robots are in shot. The moment to catch: **amr2
stops short of the gap while amr1 drives through it**, then amr2 resumes.

**RViz.** Fixed Frame `fleet_map`; both `/plan` paths converging on the same gap is
the visual that makes "narrow intersection" obvious.

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
- Conflict radius **1.94 m = r(amr1) 0.539 + r(amr2) 0.429 + d_safe(amr1 at cruise)
  0.975**. Derived, not tuned: it is the distance at which one robot's SafetyGate
  would already be holding the other.
- Two escalations, **amr2 yielded both times**, held **1.0 s and 15.2 s**, both
  released on **"conflict cleared"** — never on the 45 s fail-safe ceiling.
- **0 recovery behaviours during either hold.** `movement_time_allowance` read back
  from `controller_server` at **1 000 000 s on entry and 10 s after release**.
- SafetyGate was blocking on **0 of 326 held cycles**, so the stop was the arbiter's
  and nothing else's. A yield and a safety halt both end with a robot at a
  standstill; the report separates them rather than assuming.
- **Both goals SUCCEEDED**: amr1 40.0 s / 12.72 m driven, amr2 42.9 s / 13.62 m,
  final errors 0.032 m and 0.151 m. Closest actual approach 0.926 m.

**And the counter-example, on disk:** `results/phase7_yield_control.md` shows the
escalation test working in **both** directions within one run — **3 conflicts
predicted, 2 resolved locally** (the predicted approach opened by +0.03 m and
+0.05 m, and the arbiter stood down) and **1 escalated**. That nonzero "resolved
locally" count is the local-first ordering as data rather than as an assertion.

**Failure modes.**

| symptom | cause | fix |
|---|---|---|
| no `ESCALATED` line ever prints | the encounter did not overlap in time — amr2 got through first | relaunch; or raise `time_window_s:=5.0` |
| both robots stop and neither moves | both entered the gap together and both gates blocked | this is the failure the yield prevents; relaunch |
| `RELEASE … max hold elapsed (fail-safe)` | 45 s ceiling broke a deadlock | report it as a deadlock, not as a successful yield — the report already distinguishes them |
| the run reports NOT EXERCISED | no conflict met the escalation test that run | relaunch; the encounter is staged, not deterministic |

**Say the honest version:** three runs of this identical launch gave **2, 1 and 0
escalations**. The barrier, the gap and the 5 s dispatch offset make the encounter
*staged*, and staged is not deterministic. A run that escalates nothing reports
**NOT EXERCISED**, not FAIL.

---

### Demo 11 — Payload-adaptive velocity and jerk

**Requirement.** §3.1 payload-aware motion smoothing — acceleration and jerk limited
by dynamic state and payload, with the heavier AMR-1 given lower limits than AMR-2.

```bash
# T3
./scripts/clean_processes.sh
# T1
./ws.sh ros2 launch amr_bringup phase5_payload_trace.launch.py
```

**Duration** ~3–4 min *(est.)*. Headless is fine — **the deliverable is the plot**,
`results/phase5_payload_trace.png`. Put it on screen full-width.

**The chain, worth naming link by link:**

```
Nav2 → cmd_vel_nav → nav2_velocity_smoother (STOCK) → cmd_vel_smoothed
     → PayloadJerkAdapter (ours) → cmd_vel_shaped
     → priority mux (nav 100 | yield 150) → cmd_vel_mux
     → SafetyGate (serial, LAST, fail-closed) → cmd_vel → plant
```

We add **only** jerk limiting and payload scaling. The velocity/acceleration clamp
is stock Nav2.

**Numbers to quote** (`results/phase5_payload_trace.md`):

| robot | payload | peak v cmd | peak a cmd | peak jerk cmd |
|---|---|---|---|---|
| amr1 | unloaded | 0.500 | 0.725 | 1.784 |
| amr1 | loaded | 0.500 | 0.272 | 0.634 |
| amr2 | unloaded | 0.500 | 1.112 | 4.722 |
| amr2 | loaded | 0.500 | 0.940 | 2.742 |

- **amr1's peak commanded acceleration falls ×0.38 loaded against amr2's ×0.85** —
  60 kg on a 30 kg chassis is a far larger perturbation than 5 kg on 18 kg — **and
  no code distinguishes them.** Peak commanded velocity is exactly 0.500 in all four
  cases, i.e. the adapter restricts without overshooting.

**Read the jerk column honestly, on camera.** It exceeds the configured bound
(1.000 / 0.333 / 2.500 / 1.957 m/s³) by up to ~1.9×. The bound holds on the
limiter's own recursion — `tests/test_jerk_limiter.py` asserts it for every robot at
both payload states — so what that column measures is the **published stream**: a
20 Hz signal timestamped on arrival, resampled onto a 50 ms grid and differentiated
twice, plus a single-step transient where the velocity reaches its target. What this
run supports is the **payload ratio and the absence of overshoot**, not a certified
jerk ceiling.

**One good war story if there is time:** the continuous-time S-curve `sqrt(2j|e|)`
is the wrong formula for a discrete loop — it leaves acceleration on the books at
arrival, which the velocity clamp then absorbs in one step, spiking measured jerk to
**2.9–3.5× the limit** exactly where a jerk limiter is supposed to help. The exact
discrete form `−jΔt/2 + sqrt((jΔt/2)² + 2j|e|)` brings the worst case to ~1.1 ×.

**Also worth naming:** stock `twist_mux` does not load in this ROS snapshot —
`ros-jazzy-twist-mux` 4.5.0 wants a `diagnostic_updater` symbol the installed 4.2.6
does not export. `amr_safety/scripts/priority_mux.py` preserves the architecture and
reads the **same** `twist_mux.yaml` schema, so restoring the stock node is a two-line
launch change. Diagnosed, not worked around silently.

---

### Demo 12 — Scalability and the refactoring deliverable *(no launch — show the files)*

**Requirement.** §4.2 fleet expandable to ten or more robots by changing a minimal
number of configuration parameters; §5.2 identify a refactoring area, propose a plan
and implement part of it.

Nothing to run. On camera, show:

1. **`src/amr_description/config/fleet.yaml`** — the one typed robot list. Every
   robot difference is here: mass, geometry, velocity/acceleration/jerk limits,
   safety gain, chassis colour, spawn pose.
2. **README §8** — the exact YAML block that adds `amr3`, and the table of what
   follows automatically. **No launch code change is required**, because the fleet
   launch's only fleet-aware code is a `for` loop over `load_fleet()`.
3. `grep -rn "if robot ==" src/` → **nothing**. There is no per-robot branching.
4. **README §9** — the before/after launch tree for the refactor: a 180-line
   single-robot monolith that conflated world singletons with per-robot actions,
   split into `amr_gazebo.spawn.world_actions` + a reusable
   `robot_stack.launch.py` + two thin compositions.

**Be precise:** this is **configuration-driven scaling, not free scaling** — an edit
to one file, which is bounded, not zero. **A 10-robot run was not performed.**

---

## 6. Global failure modes

| symptom | what it actually is | recovery |
|---|---|---|
| `Failed to find a free participant index for domain 0`, second robot's servers die | a command was run without `./ws.sh`, so the CycloneDDS participant ceiling is still 9 | kill everything, relaunch through `./ws.sh` — **including RViz** |
| RViz blank, **"Fixed Frame — Frame [map] does not exist"** | RViz is on its stock config; this system publishes no bare `map` frame | use `rviz:=true`, or `rviz2 -d src/amr_bringup/rviz/fleet_mapping.rviz` |
| RViz shows the config but nothing in it | Fixed Frame not yet published (FleetMapNode starts at t≈16 s), or a QoS mismatch on a display added by hand | wait; check `ros2 topic info -v` |
| **no pedestrians in the world** | that launch defaults `with_actors:=false` | see §4 — pass `with_actors:=true` |
| numbers look wrong in a way that suggests a code bug | a leaked node from an earlier launch is still publishing | `./scripts/clean_processes.sh`, confirm the CLEAN banner, re-run. Retract nothing until you have re-measured on a verified-clean table |
| launch appears to hang ~20–30 s in with no error | staged bringup, or a lifecycle node waiting on a transform | wait 45 s; if still nothing, clean and relaunch |
| `gz sim` window is black or the world is empty | a previous `gz sim` survived and holds the port | `./scripts/clean_processes.sh` (it matches `gz sim` and `ruby.*gz`) |
| a robot spins in place at a standstill | Nav2 recovery — the progress checker fired | that is demo 8's failure mode. Check `ros2 param get /amrN/controller_server progress_checker.movement_time_allowance` |
| `map_saver_cli` times out | Nav2 gives an identical timeout for a missing publisher, a QoS mismatch and a genuinely slow map | it needs a `TRANSIENT_LOCAL` publisher; the survey holds a subscription open for this reason |
| a run overwrote an artifact you wanted | same `tag:` | re-run with `tag:=something_else`; `git status` will show what changed |
| goal ABORTED with `Failed to create a plan from potential` | NavFn transient while the map is still filling in ahead of the robot | the evidence runs retry a bounded number of times; if it persists, clean and relaunch |

---

## 7. Command sheet

```bash
# every time
./scripts/clean_processes.sh

# demo 1: cooperative mapping, live in RViz  (GUI + RViz on by default)
./ws.sh ros2 launch amr_bringup fleet_survey.launch.py

# demos 2-3: dual bringup, fleet map, selective updates, concurrent goals
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    headless:=false rviz:=true with_actors:=true tag:=demo

# demo 4: ramp mask plumbing  (partial - prefer quoting the README)
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    ramp_mask_value:=60.0 tag:=ramp_graded

# demo 5: safety halt on a pedestrian + latency   (actors always on)
./ws.sh ros2 launch amr_bringup phase2_safety_run.launch.py headless:=false

# demo 6: fail-closed (two arms)
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=watchdog headless:=false
./ws.sh ros2 launch amr_bringup phase2_failclosed.launch.py tag:=no_watchdog \
    with_watchdog:=false headless:=false

# demo 7: BSP - PitchGate and the IMU validator
./ws.sh ros2 launch amr_bringup phase2_pitch_gate.launch.py headless:=false
./ws.sh ros2 run amr_bsp plot_pitch_gate.py
./ws.sh ros2 launch amr_bringup phase2_validation.launch.py mode:=imu_injection

# demo 8: recovery suppression A/B  (quote, do not run live)
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py tag:=suppressed
./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py tag:=control suppress_recovery:=false

# demo 9: MAPF cost injection A/B
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py headless:=false rviz:=true
./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py \
    with_trajectory_layer:=false tag:=layer_off

# demo 10: yield protocol
./ws.sh ros2 launch amr_bringup phase7_yield.launch.py headless:=false rviz:=true

# demo 11: payload-adaptive velocity
./ws.sh ros2 launch amr_bringup phase5_payload_trace.launch.py

# the whole system, no mission attached
./ws.sh ros2 launch amr_bringup fleet_nav.launch.py headless:=false rviz:=true

# observers
./ws.sh rviz2 -d src/amr_bringup/rviz/fleet_mapping.rviz --ros-args -p use_sim_time:=true
./ws.sh ros2 topic hz /amr2/cmd_vel_yield
./ws.sh ros2 param get /amr2/controller_server progress_checker.movement_time_allowance
./ws.sh ros2 topic echo /amr1/safety_gate/diagnostics --once
./ws.sh ros2 node list | wc -l
```
