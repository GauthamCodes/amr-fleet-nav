# PHASE 7 - both goals through one 3.0 m gap, staged to conflict

```
==============================================================================
PHASE 7 - both goals through one 3.0 m gap, staged to conflict
==============================================================================
robots dispatched together:   2
goal frame:                   fleet_map (= the Gazebo world frame)
dispatch:                     amr1 +0.0s, amr2 +5.0s
------------------------------------------------------------------------------
[A] PER ROBOT

      robot        result        t_goal   planned   driven   final err   replans
      amr1         SUCCEEDED       37.9     10.00    10.64      0.024         2
      amr2         SUCCEEDED       20.3     11.48    11.98      0.182         1

      t_goal seconds, distances metres. 'driven' is ground truth, not
      odometry - Phase 0 measured wheel odometry reporting an 11 m
      journey the robot never made.
------------------------------------------------------------------------------
[B] FLEET SEPARATION
    samples:                                     4104
    closest approach (m):                       1.938
    at t (s):                                    42.8
    median separation (m):                      3.488

    Distance between FOOTPRINT ORIGINS, so it is a clearance measure and
    not a collision check.

    Both routes pass through the SAME gap, so this separation is the outcome of the yield rather than of route planning. Read it beside the arbiter's own report, which says when the hold started, what released it, and what Nav2 did during it.
------------------------------------------------------------------------------
[C] GLOBAL COSTMAP AND THE RAMP FILTER

      robot        frame        size        known    free   costed   min cost
      amr1         fleet_map    680x400     272000  246299    25701          0
      amr2         fleet_map    680x400     272000  247481    24519          0

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

      amr1: ramp footprint 3550 known cells, max cost 100
      amr2: ramp footprint 3550 known cells, max cost 100
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
