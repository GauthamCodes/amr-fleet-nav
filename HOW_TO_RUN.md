# How to Run — a step-by-step guide

This file is for someone opening this repository for the first time. It lists
**every command you need, in order, and says what each one does.**

You do not need to read the code to follow this. If you only have ten minutes,
do Part 1 and then Scenario A.

- Deeper per-scenario detail: [`docs/DEMO_RUNBOOK.md`](docs/DEMO_RUNBOOK.md)
- What the system is and what was measured: [`README.md`](README.md)

---

## Part 0 — What you need installed

| | |
|---|---|
| OS | Ubuntu 24.04 |
| ROS | ROS 2 **Jazzy** |
| Simulator | Gazebo **Harmonic** (`gz-sim` 8) |
| Also | Nav2, `slam_toolbox`, `colcon`, `rviz2` |

Everything else is in this repository. There is nothing to install beyond
dependencies.

---

## Part 1 — Set up once

Run these four commands in order, from the repository root.

### 1. Get the code

```bash
git clone https://github.com/GauthamCodes/amr-fleet-nav.git
cd amr-fleet-nav
```

**What it does:** downloads the project. **The repository root is the colcon
workspace** — there is no separate `~/ros2_ws` to create. The ROS packages are in
`src/`.

### 2. Install the ROS dependencies

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**What it does:** reads every `package.xml` in `src/` and installs the ROS
packages they depend on. The first line is harmless if rosdep is already set up.

### 3. Build

```bash
./ws.sh colcon build --symlink-install
```

**What it does:** compiles the workspace — two C++ packages (the safety gate and
the costmap plugin) and seven Python ones.

**You should see:** `Summary: 9 packages finished` and **0 errors**. A few
`setuptools` deprecation warnings on stderr are normal and harmless.

> **What is `./ws.sh`?** A small wrapper that every command in this guide uses.
> It sources ROS and this workspace, points Gazebo at the world files, and raises
> a networking limit. **Always use it — including for `rviz2`.** If you run a ROS
> command without it, the second robot's navigation servers fail to start with
> `Failed to find a free participant index for domain 0`. That looks like a bug in
> the robot; it is only the missing wrapper.

### 4. Run the tests

```bash
./ws.sh python3 -m pytest tests/ -q
```

**What it does:** runs the unit tests. They are pure functions — no simulator
needed — so this takes about a second.

**You should see:** `241 passed`.

Optional style checks, if you want them:

```bash
./ws.sh python3 -m flake8 .          # PEP 8 check — expect no output
./ws.sh python3 -m black --check .   # formatting check — expect "98 files unchanged"
```

---

## Part 2 — The one command you must run before every demo

```bash
./scripts/clean_processes.sh
```

**What it does:** kills any simulator or ROS processes left over from a previous
run, then **prints what is still running**.

**Wait for this banner before launching anything:**

```
==============================================================================
PROCESS TABLE CLEAN - nothing matching a simulation or ROS pattern is running
==============================================================================
```

**Why it matters:** a leftover process from an earlier run keeps publishing and
quietly corrupts the next run's behaviour and numbers. Run this between every
demo. It takes two seconds.

> One thing it does **not** kill is `rviz2`. If you have a stale RViz window from
> an earlier demo, close it, or run `pkill -f rviz2`.

---

## Part 3 — The demos

Each demo is **one command**. Each one starts everything it needs — the
simulator, the robots, navigation, and (where useful) RViz — and most of them
**stop themselves** when finished and write a report into `results/`.

To stop a demo early: press `Ctrl-C` once in that terminal, wait for it to shut
down, then run `./scripts/clean_processes.sh`.

> **Nothing appears for the first ~25 seconds of any demo.** The system starts in
> stages: robots spawn, then sensor validation, then SLAM, then the shared map,
> then navigation. This is normal — do not assume it has hung.

---

### Scenario A — Cooperative mapping ⭐ start here

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup fleet_survey.launch.py
```

**What it does:** starts the warehouse, both robots, and **both Gazebo and RViz**
— you do not need to open RViz yourself. Both robots then drive a lap of the aisle
while building **one shared map together**.

**What you will see:** the Gazebo window shows the warehouse and the robots. The
RViz window shows the shared map `/fleet_map` starting almost empty and **filling
in from both ends of the aisle at once**, because the two robots run the same
circuit half a lap apart. AMR-1 is blue, AMR-2 is amber.

**Worth clicking:** in the RViz "Displays" list, tick **`amr1 own map`** and
**`amr2 own map`**. Each robot's own map covers only part of the aisle; the shared
map covers both. That is the cooperative mapping.

**Takes:** about 4–6 minutes. It stops itself.

**Fixed Frame:** `fleet_map` (already set).

---

### Scenario B — Both robots navigating at once, with pedestrians

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py \
    headless:=false rviz:=true with_actors:=true tag:=demo
```

**What it does:** brings up the full stack and sends **both robots to different
goals at the same time**, with pedestrians walking in the aisle.

**What each part of the command means:**

| part | meaning |
|---|---|
| `headless:=false` | show the Gazebo window (it is hidden by default for speed) |
| `rviz:=true` | start RViz too, already configured |
| `with_actors:=true` | put the walking pedestrians in the warehouse |
| `tag:=demo` | write this run's report under a different name so it does not overwrite the committed results |

**What you will see:** at about 32 seconds both planned paths appear in RViz at
the same instant — **green for AMR-1, cyan for AMR-2** — and both robots drive
them. AMR-2 arrives first because the configuration file says it is faster.

**Takes:** about 4–5 minutes. It stops itself.

> With the Gazebo window open the simulator runs at roughly a quarter of real
> time, and AMR-1's goal sometimes does not finish inside the run's time limit.
> For the measured result, run it without `headless:=false` (i.e. headless) and
> read `results/`.

---

### Scenario C — Safety override on a pedestrian

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase2_safety_run.launch.py headless:=false
```

**What it does:** one robot navigates the aisle while pedestrians walk around it.
A dedicated safety node watches the laser and **stops the robot when someone gets
too close**, overriding whatever the navigation stack was commanding.

**What you will see:** the robot drives, a pedestrian walks into its path, and the
robot **stops short of them**, then continues once they move away. This happens
three times.

**Watch the terminal** for lines like:

```
HALT 1: clearance 0.412 m at 0.331 m/s, sensor->zero 8.5 ms
RELEASE: latch clearance 0.769 m > d_release 0.677 m
```

That is the stop distance rule `d_safe = k·v² + d_min` firing, and the time from
the laser reading to the stop command being published.

**Takes:** about 2–3 minutes. It stops itself.

**RViz:** not started for this one (it is a single-robot demo with no shared map).
The Gazebo view shows everything you need. If you do want RViz here, open it
separately and set **Fixed Frame to `amr1/odom`**, not `fleet_map`.

---

### Scenario D — Two robots forced into a conflict (yielding)

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase7_yield.launch.py headless:=false rviz:=true
```

**What it does:** puts a barrier with a single 3-metre gap between both robots and
their goals, so they must pass through the same spot. A traffic-control node
detects the conflict and **makes the lighter robot wait**.

**What you will see:** both robots approach the gap; **AMR-2 stops, AMR-1 drives
through, then AMR-2 continues.**

**Watch the terminal** for these three lines:

```
conflict predicted amr1/amr2: ... - local layer has it
ESCALATED amr1/amr2: ... - amr2 yields to amr1
RELEASE amr2 after 15.2 s: conflict cleared ...
```

**Takes:** about 5–6 minutes. It stops itself.

> **This encounter is deliberately staged and it is not identical every time.**
> Across repeated runs it has produced 3, 2, 1 and even 0 yields — sometimes the
> robots resolve it between themselves and the traffic controller correctly does
> nothing, which the report calls `NOT EXERCISED` rather than a failure. If you
> want to see the yield and this run does not produce one, just run it again, or
> watch the recorded run at [`media/yield_protocol.mp4`](media/yield_protocol.mp4).

---

### Scenario E — Sensor validation (rejecting impossible IMU data)

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase2_validation.launch.py mode:=imu_injection
```

**What it does:** deliberately injects physically impossible IMU readings and
shows the validation layer **rejecting them before the navigation stack ever sees
them**, with a warning for each.

**Here the terminal output is the demonstration.** You will see:

```
[WARN] imu REJECT (1): implausible angular velocity: |w_z| = 50.000 rad/s > 4.000 rad/s
```

500 bad samples are injected; all 500 are rejected, none reach the validated
topic, and the ~3 200 healthy samples around them are still accepted — which is
the point: it rejects the bad data without rejecting everything.

**Takes:** about 2 minutes. It stops itself.

---

### Scenario F — Payload-aware motion (produces a plot)

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup phase5_payload_trace.launch.py
```

**What it does:** drives each robot loaded and unloaded and records how its
acceleration and jerk are limited.

**The output is a picture.** When it finishes, open:

```
results/phase5_payload_trace.png
```

It shows that the heavy robot becomes much more conservative when loaded than the
light one does — and no code treats the two robots differently; it all comes from
one configuration file.

**Takes:** about 3–4 minutes. No Gazebo window needed.

---

### Everything at once (no goals, just the full system)

```bash
./scripts/clean_processes.sh
./ws.sh ros2 launch amr_bringup fleet_nav.launch.py headless:=false rviz:=true
```

**What it does:** starts the complete stack — warehouse, both robots, sensor
validation, SLAM, the shared map, both navigation stacks, motion smoothing, the
safety gates and the traffic controller — and leaves it running.

**This one does not stop itself.** Press `Ctrl-C` when you are done.

Useful check while it runs, in a second terminal:

```bash
./ws.sh ros2 node list | wc -l
```

**Expect 53** — that is two complete robot stacks plus the shared fleet nodes.

The remaining scenarios (ramp cost, trajectory sharing, fail-closed behaviour,
recovery suppression) are in [`docs/DEMO_RUNBOOK.md`](docs/DEMO_RUNBOOK.md).

---

## Part 4 — Using RViz

For the two-robot demos you do **not** need to open RViz yourself — add
`rviz:=true` and it starts already configured, showing the shared map, both
robots, both laser scans, both planned paths, and the transform tree.

If you want to open it by hand:

```bash
./ws.sh rviz2 -d src/amr_bringup/rviz/fleet_mapping.rviz --ros-args -p use_sim_time:=true
```

**What each part does:** `./ws.sh` gives RViz the same environment as everything
else · `-d …` loads the saved view · `use_sim_time:=true` tells RViz to use the
simulator's clock instead of your computer's — without it nothing appears.

### If RViz is blank, read this

The most likely cause is the **Fixed Frame**.

| demo | Fixed Frame to use |
|---|---|
| any two-robot demo (Scenarios A, B, D, full system) | **`fleet_map`** |
| any single-robot demo (Scenario C, ramp/laser demos) | **`amr1/odom`** |

**There is no frame called `map` in this system** — each robot's own map frame is
named `amr1/map` / `amr2/map`, and the shared one is `fleet_map`. If you start a
plain `rviz2` with no configuration it defaults to `map` and shows
`Fixed Frame — Frame [map] does not exist` with an empty screen. Load the saved
view above, or set the Fixed Frame yourself in the "Global Options" at the top of
the Displays panel.

Also note the shared map only starts being published about **16 seconds** into a
run, so an empty screen before that is expected.

---

## Part 5 — Where the results are

Every demo writes a plain-text report into `results/`, and those files are the
source of every number quoted in the README.

```bash
ls results/                                     # ~75 evidence files
cat results/phase3_concurrent_goals.md          # both robots reaching their goals
cat results/phase2_safety_suppressed.md         # the safety stops and their timing
cat results/phase3_selective_updates.md         # which map updates were kept or skipped
cat results/phase7_yield.md                     # the yield decisions
```

Short recorded clips of real runs are in [`media/`](media/), each with its own
report in `media/run_reports/`.

---

## Part 6 — If something goes wrong

| What you see | What it means | What to do |
|---|---|---|
| `Failed to find a free participant index for domain 0`, and the second robot's navigation dies | a command was run without `./ws.sh` | stop everything, run `./scripts/clean_processes.sh`, relaunch using `./ws.sh` |
| RViz is empty and says `Frame [map] does not exist` | RViz is on its default configuration | see Part 4 — use the saved view, or set Fixed Frame to `fleet_map` (or `amr1/odom` for single-robot demos) |
| RViz is empty but shows no error | the run has not reached ~16 s yet, or it just started | wait; the shared map appears at about 16 s |
| No pedestrians in the warehouse | that demo has them off by default | add `with_actors:=true` (see Scenario B) |
| Nothing happens for 20–30 seconds | normal staged startup | wait 45 s before worrying |
| The Gazebo window is black, or the warehouse is empty | a previous simulator is still running | `./scripts/clean_processes.sh`, then relaunch |
| A robot spins on the spot | navigation recovery behaviour | stop, clean, relaunch |
| Results look strange or inconsistent | a leftover process from an earlier run | `./scripts/clean_processes.sh`, confirm the CLEAN banner, run again |
| A goal ends in `ABORTED` | the planner failed while the map was still filling in ahead of the robot | clean and relaunch; runs retry a bounded number of times |

---

## Part 7 — Honest summary of what these demos do and do not show

The README's §10 lists this in full. The short version:

**Demonstrated and measured:** cooperative mapping into one shared map · selective
map updating · both robots navigating simultaneously · payload-dependent
acceleration limits · the yielding protocol · the safety override and its
fail-closed behaviour · sensor validation rejecting impossible IMU data ·
configuration-driven fleet scaling · a clean modular workspace and a refactoring.

**Partial, and stated as such:**

- **Ramp / slope planning.** The terrain-cost mask loads correctly into both
  robots' planners, but the graded cost over the ramp is not confirmed — it comes
  out the same as with no mask at all. Not diagnosed, and not claimed to work.
- **Trajectory sharing between robots (MAPF).** Each robot's predicted path
  provably becomes cost in the other robot's local map — measured against a
  control run. But the controller in use reacts to that cost by adjusting speed
  rather than steering around it, so **robots autonomously deviating around each
  other is not demonstrated.** Conflicts that the robots cannot resolve are
  handled by the traffic controller (Scenario D).

Also worth knowing: the pedestrians are animated figures rather than solid
physical objects, so they are visible to the laser and treated as obstacles, but
they cannot physically collide. Distances kept are measured; "zero collisions" is
never claimed.

---

## The screenshare video

The submission video is delivered separately and is deliberately **not** stored in
this repository — it is large and not something you need to clone the project to
watch.
