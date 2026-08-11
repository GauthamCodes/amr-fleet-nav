# Smoke Test 1 - Gazebo actor LiDAR visibility

```

==============================================================================
SMOKE TEST 1 - Gazebo actor LiDAR visibility
==============================================================================
laser origin:         (0.29, 0.00), yaw 0.0 deg  [robot amr1]
lidar height:         0.200 m above ground
scan frame_id:        amr1/amr1/base_footprint/lidar
scan geometry:        360 beams, 0.12-12.00 m
scans analysed:       304
------------------------------------------------------------------------------
[CONTROL] box with ordinary collision geometry - MUST be detected
  world position:                     (3.00, -3.50)
  expected bearing:                     -52.25 deg
  expected range to near face:            4.23 m
  scans with a return in sector:         100.0 %
  median per-scan closest return:         4.16 m
  median - expected:                     -0.07 m
  absolute closest over run:              3.58 m
  peak costmap cost at box:                254  (LETHAL)
  costmap frames marking it lethal:      100.0 %
------------------------------------------------------------------------------
[SUBJECT] Gazebo <actor> sweeping the corridor ahead
  corridor bearing window:            -36.4 to +36.4 deg
  corridor range window:              1.96 to 4.12 m
  scans with a return in corridor:        91.8 %
  median per-scan closest return:         2.81 m
  5th percentile:                         2.56 m
  95th percentile:                        3.24 m
  absolute closest over run:              2.49 m
  actor centre-line distance:             2.71 m
  costmap frames analysed:                  80
  peak costmap cost on actor path:         254  (LETHAL)
  costmap frames marking it lethal:      100.0 %
------------------------------------------------------------------------------
[WITNESS] camera watching the actor's path
  frames received:                         303
  mean frame-to-frame change:           1.4739 / 255
  peak frame-to-frame change:           4.7212 / 255
  something is animating in view:     YES
------------------------------------------------------------------------------
[ATTITUDE] resting attitude on flat ground
  IMU samples:                            3034
  mean pitch:                           +0.000 deg
  mean roll:                            -0.000 deg
  max |pitch|:                           0.000 deg
  implied ground return:              none (level)
------------------------------------------------------------------------------
[PHYSICS] Gazebo entity accounting
  actors are absent from /world/<w>/pose/info, dynamic_pose/info
  and 'gz model --list' - they are not physics entities, so they can
  never generate contacts regardless of LiDAR visibility.
------------------------------------------------------------------------------
VERDICT
  Control box detected AND actor detected in the LiDAR.
  Actor reaches the Nav2 obstacle layer: YES
  RESULT: ACTOR VISIBLE TO LIDAR
==============================================================================

```
