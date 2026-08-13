# PHASE 3 - concurrent goals to the whole fleet

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
      amr1         SUCCEEDED       18.6     10.50    10.50      0.018         0
      amr2         SUCCEEDED       11.3     10.50    10.73      0.231         0

      t_goal seconds, distances metres. 'driven' is ground truth, not
      odometry - Phase 0 measured wheel odometry reporting an 11 m
      journey the robot never made.
------------------------------------------------------------------------------
[B] FLEET SEPARATION
    samples:                                     2156
    closest approach (m):                       3.000
    at t (s):                                    24.7
    median separation (m):                      3.516

    Distance between FOOTPRINT ORIGINS, so it is a clearance measure and
    not a collision check.

    The two routes are deconflicted by design; the forced conflict, mutual local deviation and the yield protocol are Phases 6 and 7.
------------------------------------------------------------------------------
[C] GLOBAL COSTMAP AND THE RAMP FILTER

      robot        frame        size        known    free   costed   min cost
      amr2         fleet_map    680x400     272000  248070    23930          0
      amr1         fleet_map    680x400     272000  246953    25047          0

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

      amr2: ramp footprint 3550 known cells, 0 unknown, cost 0 .. 100
      amr1: ramp footprint 3550 known cells, 0 unknown, cost 0 .. 100

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
