# PHASE 3 - concurrent goals - RECHECK

**This artifact retracts a limitation.** An earlier revision of `README.md` section 10
reported that only one of the two robots ever reached its goal, reproduced in nine
consecutive runs, and recorded that in `phase3_concurrent_goals_current.md`. Measured
again on the same commit with **no source change**, both robots arrive, six runs out
of six:

| run | tag | AMR-1 | AMR-2 | verdict |
|---|---|---|---|---|
| 1 | `cleantable` | SUCCEEDED 18.6 s, 10.51 m driven, 0.016 m final error | SUCCEEDED 11.3 s, 10.68 m, 0.182 m | PASS |
| 2 | `repro2` | SUCCEEDED 18.9 s, 10.50 m, 0.019 m | SUCCEEDED 11.3 s, 10.68 m, 0.185 m | PASS |
| 3 | `repro3` | SUCCEEDED 18.6 s, 10.51 m, 0.011 m | SUCCEEDED 11.3 s, 10.68 m, 0.182 m | PASS |
| 4 | `leaktest` | SUCCEEDED 18.6 s, 10.48 m, 0.029 m | SUCCEEDED 11.3 s, 10.66 m, 0.156 m | PASS |
| 5 | `audit_a` | SUCCEEDED 18.6 s, 10.51 m, 0.009 m | SUCCEEDED 11.3 s, 10.68 m, 0.183 m | PASS |
| 6 | `recgoals1` | SUCCEEDED 18.6 s, 10.51 m, 0.017 m | SUCCEEDED 11.3 s, 10.73 m, 0.235 m | PASS |

All six ran `phase3_fleet_goals.launch.py` at its defaults. Runs 1, 3 and 6 had the
Gazebo GUI and RViz attached; runs 2, 4 and 5 were headless. Every run reports 0
replans.

**Runs 5 and 6 were added by a later audit pass** and are the reason this file says six
rather than four. They matter for a specific reason: run 5 was measured on a **build
made from scratch** — `rm -rf build install log` followed by a full
`colcon build --symlink-install` — so it does not inherit any object from the tree the
first four were measured against, and run 6 is the run
[`../media/previews/concurrent_goals.gif`](../media/previews/concurrent_goals.gif) is
cut from, so the preview an evaluator sees is a run that appears in this table rather
than an unrecorded one. Run 5 reported closest approach **3.000 m over 2 166 samples**,
median separation 3.522 m; run 6 **3.000 m over 2 163 samples**, median 3.529 m.

These timings reproduce `phase3_concurrent_goals.md` - amr1 18.8 s, amr2 11.3 s - which
the retracted entry described as historical and no longer reproducible.

## What was wrong with the environment, and what is not claimed

The failing runs were measured on a machine in two bad states at once:

1. `scripts/clean_processes.sh` matched none of `trajectory_predictor`,
   `traffic_control`, `payload_jerk_adapter` or `priority_mux`. All four are
   long-lived subscribers, so they outlived every `PROCESS TABLE CLEAN` banner and
   accumulated across interrupted runs - **seven generations were found still
   running, the oldest 13 h 44 m old.** This also invalidates the A/B arms that
   "ruled out" the trajectory layer and the motion chain: disabling a component in
   the new launch does nothing about the instance still running from the previous one.

2. The root filesystem had **977 MB free of 43 GB**, because those orphans had
   written **5.3 GB** of logs - 19 files over 50 MB, the largest 389 MB of repeated
   `TF_OLD_DATA` warnings and still growing when it was found.

**Run 4 is the control, and it is a negative result.** One generation of orphans was
left alive on purpose - 2 x `trajectory_predictor`, 2 x `payload_jerk_adapter`,
1 x `traffic_control`, confirmed alive at dispatch - and the run still passed. So a
single leak is **not** sufficient to cause the failure, and this artifact does **not**
claim the leak was the cause. The accumulated load of seven generations, the exhausted
disk, or both together remain candidates, and none of them is isolated here.

What these six runs support is only this: **the code in this repository reaches both
goals**, and the entry that stood in section 10 described the machine it was measured
on rather than the repository.

## Run 1 verbatim

```
==============================================================================
PHASE 3 - concurrent goals to the whole fleet
==============================================================================
robots dispatched together:   2
goal frame:                   fleet_map (= the Gazebo world frame)
dispatch:                     all together
------------------------------------------------------------------------------
[A] PER ROBOT

      robot        result        t_goal   planned   driven   final err   replans
      amr1         SUCCEEDED       18.6     10.50    10.51      0.016         0
      amr2         SUCCEEDED       11.3     10.50    10.68      0.182         0

      t_goal seconds, distances metres. 'driven' is ground truth, not
      odometry - Phase 0 measured wheel odometry reporting an 11 m
      journey the robot never made.
------------------------------------------------------------------------------
[B] FLEET SEPARATION
    samples:                                     2163
    closest approach (m):                       3.000
    at t (s):                                    27.3
    median separation (m):                      3.520

    Distance between FOOTPRINT ORIGINS, so it is a clearance measure and
    not a collision check.

    The two routes are deconflicted by design; the forced conflict, mutual local deviation and the yield protocol are Phases 6 and 7.
------------------------------------------------------------------------------
[C] GLOBAL COSTMAP AND THE RAMP FILTER

      robot        frame        size        known    free   costed   min cost
      amr1         fleet_map    680x400     272000  247568    24432          0
      amr2         fleet_map    680x400     272000  248816    23184          0

    Phase 3 ships the KeepoutFilter loaded and wired with an
    all-zero mask, and 'min cost' over known cells is the number
    that shows it contributes nothing. The mask is UNIFORM, so a
    filter adding cost would raise every cell together and no cell
    could read 0 - which is exactly what a background pixel of 205
    instead of white would do, costing the whole warehouse ~48.

    Cost inside the ramp footprint is NOT the test and is reported
    only for reference: that region is partly unexplored and partly
    the real plateau face, so it reads lethal from the static layer
    whatever the filter does. Phase 4 replaces the null mask with a
    graded one and measures the routing change instead.

      amr1: ramp footprint 3550 known cells, 0 unknown, cost 0 .. 100
      amr2: ramp footprint 3550 known cells, 0 unknown, cost 0 .. 100

    With a GRADED ramp mask the discriminator is the ramp's MINIMUM
    cost, not its maximum: the mask raises every cell in the
    footprint together, so the minimum steps off zero while the
    warehouse-wide 'min cost' above stays at zero. One says the cost
    landed; the other says it landed only where it was meant to.
------------------------------------------------------------------------------
[D] VERDICT
    every robot's goal was accepted:                   YES
    every robot reached its goal:                      YES
    both robots were tracked simultaneously:           YES
    the fleet never closed to within 0.5 m:            YES
    both global costmaps planned in the fleet frame:   YES
    the costmaps are populated:                        YES
    mapped free space still reads zero cost:           YES
    RESULT: PASS
==============================================================================
```
