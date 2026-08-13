# PHASE 6 - forced crossing, trajectory layer LAYER_ON

```
==============================================================================
PHASE 6 - forced crossing, trajectory layer LAYER_ON
==============================================================================
robots dispatched together:   2
goal frame:                   fleet_map (= the Gazebo world frame)
------------------------------------------------------------------------------
[A] PER ROBOT

      robot        result        t_goal   planned   driven   final err   replans
      amr1         SUCCEEDED       18.0      7.73     8.30      0.118         3
      amr2         SUCCEEDED        9.0      7.66     7.55      0.087         0

      t_goal seconds, distances metres. 'driven' is ground truth, not
      odometry - Phase 0 measured wheel odometry reporting an 11 m
      journey the robot never made.
------------------------------------------------------------------------------
[B] FLEET SEPARATION
    samples:                                     2097
    closest approach (m):                       1.610
    at t (s):                                    33.5
    median separation (m):                      2.689

    Distance between FOOTPRINT ORIGINS, so it is a clearance measure and
    not a collision check.

    The routes CROSS by construction - each robot's goal is in the other's starting lane. Compare this number across the layer_on and layer_off arms.
------------------------------------------------------------------------------
[C] GLOBAL COSTMAP AND THE RAMP FILTER

      robot        frame        size        known    free   costed   min cost
      amr2         fleet_map    680x400     272000  251339    20661          0
      amr1         fleet_map    680x400     272000  250202    21798          0

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

      amr2: ramp footprint 3550 known cells, max cost 100
      amr1: ramp footprint 3550 known cells, max cost 100
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
