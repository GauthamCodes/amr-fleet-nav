# Engineering Notes — AMR Fleet Navigation

The design invariants this system is built on, the environment it is built for, and
the language rules the documentation follows. Every rule below is cited by number
from the code that depends on it — `grep -rn "rule 1"` finds the fail-closed gate's
callers — so **the numbering is part of the interface and does not get reordered.**

---

## What this is

A two-robot (scalable to N) AMR fleet in a warehouse: cooperative SLAM with
selective map updates, ramp-aware global planning, payload-adaptive motion
smoothing, conflict-aware local planning, and a low-latency safety override.

- `README.md` — architecture, the requirement → implementation → evidence matrix,
  the results, and the known limitations. Every number quoted anywhere comes from
  the `results/` file named beside it.
- `docs/DEMO_RUNBOOK.md` — how to run each demo and what it should show.
- `docs/ASSIGNMENT.pdf` — the source requirements.

## Environment

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (ROS-vendored `gz-sim` 8.11.0)
- The repository root **is** the colcon workspace; packages live in `src/`
- Build: `colcon build --symlink-install` from the repository root
- Every ROS or Gazebo command goes through `./ws.sh`, which sources the workspace
  and **appends** to `CYCLONEDDS_URI` rather than overwriting it. The ceiling it
  raises is on DDS **participants**, not nodes: a two-robot `fleet_nav` is 65 unique
  node names but 111 `ros2 node list` entries, and `phase7_yield` adds more on top.
  The default ceiling is 9 and this repository sets 400; see
  `src/amr_bringup/config/cyclonedds.xml` for why 120 was tried and was not enough,
  and `docs/DEMO_RUNBOOK.md` §0 for why the failure looks like a namespacing bug.
- Target GPU has 6 GB of VRAM — Gazebo scenes stay light, no imported meshes.

---

## Non-negotiable design rules

1. **SafetyGate is a serial gate, fail-closed.** It is the LAST link before
   `/cmd_vel`. Never a `twist_mux` input. If the node dies, nothing moves. A mux
   selects the highest-priority *live* input and therefore fails OPEN; that is the
   wrong failure mode for the one component whose whole job is to stop the robot.
   Measured: 0.196 m of travel after a SIGKILL with the plant-side watchdog present,
   against 3.500 m and still rolling without it.

2. **Gating without notification is a bug.** Anything that zeroes `cmd_vel` (yield,
   halt) must also suppress Nav2 recovery via the progress-checker parameter, or
   Nav2 sees a stuck robot and dispatches recovery behaviours into a vehicle that
   must not move. Measured: with suppression off, 6 `Failed to make progress` errors
   and 2 recoveries fired *during* a deliberate hold.

3. **Safety uses measured velocity from odometry**, never commanded velocity. `k` is
   per-robot and payload-scaled: `k = 1/(2·a_eff)`, `a_eff = |max_decel_x| ·
   m_base/(m_base + m_payload)`, `[k] = s²/m`.

4. **Preserve original header timestamps** when republishing sensor data through the
   BSP. Rewriting stamps silently breaks TF and SLAM. Evidence is the stamp-match
   count: 903 raw/validated pairs matched by exact stamp, 0 restamped, max
   difference 0 ns — a restamp shows up as zero matched pairs rather than as
   plausible numbers computed against mismatched data.

5. **No per-robot branching.** No `if robot == "amr1"` anywhere. Robot differences
   come from `config/fleet.yaml` only — mass, geometry, limits, jerk bounds, safety
   gain, and the traffic priority that decides who yields. Instruments that observe
   a named robot are not part of the system and may name one.

6. **Extend Nav2, do not duplicate it.** The stock `nav2_velocity_smoother` chains
   into our `PayloadJerkAdapter`; we add only jerk limiting and payload scaling.

7. **The MAPF requirement is satisfied by `FleetTrajectoryLayer`** — a costmap plugin
   through which each robot's local planner consumes the other robots' predicted
   trajectories, with no central node in the loop. `TrafficControlNode` handles only
   escalated conflicts, and refuses to act while the predicted closest approach is
   still opening. Do not collapse these into one central node.

8. **Sensor data reaches Nav2 only through SensorBSP.** No node in the navigation
   stack subscribes to a raw sensor topic. `amr_bsp/topics.py` is the one place the
   contract is written down.

---

## Language policy

Python by default. C++ where measured latency justifies it: `SafetyGate` and the
costmap plugins. Do not rewrite Python in C++ without measurement data supporting
it — the BSP was instrumented first, and its latency never justified a rewrite.

Style: PEP 8 enforced by flake8 **and** black (black wins where they disagree) for
Python; Google C++ Style for C++. A docstring on every class.

## Terminology

Wording that is easy to overstate, and the accurate form:

- "**low-latency safety override**" — not "hard real-time". There is no RT kernel and
  no scheduling guarantee. The end-to-end figure is quantised by Gazebo's `/clock`
  step under `use_sim_time`; the in-node compute figure is not.
- "**configuration-driven scaling**" — not "for free". Scaling to a third robot is an
  edit to one file, which is bounded, not free.
- Known limitations belong in the README, stated plainly. A reader who catches an
  overclaim discounts everything else, and this repository has already retracted one
  finding of its own (README section 10, item 12) rather than quietly dropping it.
