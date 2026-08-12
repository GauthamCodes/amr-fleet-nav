# Phase 1 - goal-to-goal navigation run: baseline

```

==============================================================================
PHASE 1 - goal-to-goal navigation run 'baseline'
==============================================================================
robot:                    amr1
start (world):            (-11.00, -1.50)
goal (world / map):       (-0.50, +1.50) / (+10.50, +3.00)
actors in world:          none
------------------------------------------------------------------------------
[NAVIGATION]
  goal result:                        SUCCEEDED
  time to goal:                           20.5 s  (sim)
  straight-line start to goal:           10.92 m
  planned path length (first plan):      11.16 m
  executed path length (ground truth):   11.01 m
  executed / planned:                     0.99
  final position error vs goal:          0.079 m
  mean speed while navigating:            0.54 m/s
  peak speed:                             0.60 m/s
------------------------------------------------------------------------------
[PLANNING]
  global plans published:                   20
  of which a different route:                5   <- replan count
  plan publication rate:                  0.97 Hz
  recovery behaviours fired:                 0
    none - the stack never stopped making progress
------------------------------------------------------------------------------
[CLEARANCE] measured from the footprint polygon, not the base frame
  scans analysed:                          206
  beams returning a range:                72.8 %
  returns discarded as self-hits:            0   (0.0 per scan)
  min clearance to ANY obstacle:         0.916 m
  5th percentile:                        0.966 m
  median:                                1.392 m
  no dynamic returns seen - no encounter in this run
  NOTE actors generate no contacts in Gazebo, so this is a clearance
  measurement, not a collision check. See the module docstring.
------------------------------------------------------------------------------
[LOCALIZATION] slam_toolbox pose vs simulator ground truth
  samples with a map transform:            206
  mean error:                            0.036 m
  p95 error:                             0.075 m
  max error:                             0.079 m
==============================================================================

```
