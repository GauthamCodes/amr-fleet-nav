# Phase 1 - goal-to-goal navigation run: actors

```

==============================================================================
PHASE 1 - goal-to-goal navigation run 'actors'
==============================================================================
robot:                    amr1
start (world):            (-11.00, -1.50)
goal (world / map):       (-0.50, +1.50) / (+10.50, +3.00)
actors in world:          pedestrian_1, pedestrian_2, pedestrian_3
------------------------------------------------------------------------------
[NAVIGATION]
  goal result:                        SUCCEEDED
  time to goal:                           27.8 s  (sim)
  straight-line start to goal:           10.92 m
  planned path length (first plan):      11.16 m
  executed path length (ground truth):   11.52 m
  executed / planned:                     1.03
  final position error vs goal:          0.087 m
  mean speed while navigating:            0.41 m/s
  peak speed:                             0.60 m/s
------------------------------------------------------------------------------
[PLANNING]
  global plans published:                   23
  of which a different route:                4   <- replan count
  plan publication rate:                  0.83 Hz
  recovery behaviours fired:                 1
    Spin                                     1
------------------------------------------------------------------------------
[CLEARANCE] measured from the footprint polygon, not the base frame
  scans analysed:                          279
  beams returning a range:                71.5 %
  returns discarded as self-hits:           58   (0.2 per scan)
  min clearance to ANY obstacle:         0.050 m
  5th percentile:                        0.331 m
  median:                                1.226 m
  scans with a dynamic return:             269
  MIN CLEARANCE TO A DYNAMIC OBSTACLE:   0.050 m
  5th percentile:                        0.296 m
  median while one was visible:          2.688 m
  closest approach at t =                 56.6 s
    robot at                          (-8.01, +0.29)
    obstacle return at                (-8.23, +0.10)
    pedestrian_1 reconstructed at    (-7.75, +0.00)
    pedestrian_2 reconstructed at    (+0.50, +4.14)
    pedestrian_3 reconstructed at    (+9.00, -3.20)
  actor reconstruction residual:      median 0.25 m, p95 0.76 m, n=269
    (LiDAR-attributed obstacle vs world-file trajectory; the
     clearance number above comes from the LiDAR alone)
  NOTE actors generate no contacts in Gazebo, so this is a clearance
  measurement, not a collision check. See the module docstring.
------------------------------------------------------------------------------
[LOCALIZATION] slam_toolbox pose vs simulator ground truth
  samples with a map transform:            279
  mean error:                            0.027 m
  p95 error:                             0.058 m
  max error:                             0.068 m
==============================================================================

```
