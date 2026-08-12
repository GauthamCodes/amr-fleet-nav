# Smoke Test 1 - Gazebo actor LiDAR visibility

```

==============================================================================
SMOKE TEST 1 - Gazebo actor LiDAR visibility
==============================================================================
laser origin:         (0.43, 0.00), yaw 0.0 deg  [robot amr1]
lidar height:         0.200 m above ground
scan frame_id:        amr1/amr1/base_footprint/lidar
scan geometry:        360 beams, 0.12-12.00 m
scans analysed:       305
------------------------------------------------------------------------------
[CONTROL] box with ordinary collision geometry - MUST be detected
  world position:                     (3.00, -3.50)
  expected bearing:                     -53.71 deg
  expected range to near face:            4.14 m
  scans with a return in sector:         100.0 %
  median per-scan closest return:         4.09 m
  median - expected:                     -0.05 m
  absolute closest over run:              3.50 m
  peak costmap cost at box:                254  (LETHAL)
  costmap frames marking it lethal:      100.0 %
------------------------------------------------------------------------------
[SUBJECT] Gazebo <actor> sweeping the corridor ahead
  corridor bearing window:            -37.9 to +37.9 deg
  corridor range window:              1.82 to 4.01 m
  scans with a return in corridor:        91.8 %
  median per-scan closest return:         2.68 m
  5th percentile:                         2.42 m
  95th percentile:                        3.11 m
  absolute closest over run:              2.35 m
  actor centre-line distance:             2.57 m
  costmap frames analysed:                  80
  peak costmap cost on actor path:         254  (LETHAL)
  costmap frames marking it lethal:      100.0 %
------------------------------------------------------------------------------
[WITNESS] camera watching the actor's path
  frames received:                         304
  mean frame-to-frame change:           1.4743 / 255
  peak frame-to-frame change:           4.7212 / 255
  something is animating in view:     YES
------------------------------------------------------------------------------
[ATTITUDE] resting attitude on flat ground
  IMU samples:                            3046
  mean pitch:                           +0.000 deg
  mean roll:                            +0.000 deg
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
