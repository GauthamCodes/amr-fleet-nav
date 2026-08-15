# Phase 2 - recovery suppression against a static barrier

```
==============================================================================
PHASE 2 - recovery suppression against a static barrier
==============================================================================
robot:                    amr1
recovery suppression:     OFF  <- CONTROL RUN
goal result:              TIMEOUT
goal dispatches:          1
k / d_min:                1.8750 s^2/m / 0.300 m
------------------------------------------------------------------------------
[A] HALTS
    halts:                                           8
    GATE-REPORTED leaked commands:                   0   <- must be 0
    observed by this probe (lagged, see below):       2
        The gate counts a leak by comparing the twist it is about to
        publish against its latch at that instant. This probe can only
        see the latch through diagnostics, published once per scan, so
        a command passed legitimately in the ~100 ms after a release
        reads to it as a leak. The gate's own count is the invariant;
        the probe's is kept because a large gap between them would mean
        the gate is releasing far more often than the report suggests.
    upstream kept commanding motion:              1200
    furthest the robot moved while blocked:      0.106 m

      #   clearance   speed    d_safe   held s   latency ms   recov
      1       0.946   0.600     0.975     0.30          9.0      0
      2       0.692   0.475     0.723     0.30          7.0      0
      3       0.503   0.357     0.538     9.50          7.0      0
      4       0.380   0.218     0.389     9.40          7.0      0
      5       0.335   0.121     0.327     9.70          6.0      0
      6       0.335   0.137     0.335     9.70          8.0      1
      7       0.279   0.000     0.300     8.00          5.0      0
      8       0.293   0.000     0.300    10.00          9.0      1
------------------------------------------------------------------------------
[B] SENSOR STAMP -> ZERO COMMAND PUBLISHED
    blocking decisions:                603
    mean:                                         7.49 ms
    p95:                                         11.00 ms
    max:                             14.00 ms
    in-node compute, mean / p95 / max:        232.4 / 383.5 / 681.5 us

    End-to-end covers the simulator's sensor pipeline, the bridge, the
    BSP relay and the gate, and is quantised by Gazebo's /clock step
    under use_sim_time; the compute figure is not. Low-latency safety
    override - not hard real-time: no RT kernel, no scheduling guarantee.
------------------------------------------------------------------------------
[C] NAV2 RECOVERY BEHAVIOUR DURING THE HALT  (ENGINEERING_NOTES rule 2)
    recovery behaviours, whole run:        2
    recovery behaviours fired DURING a halt:         2
        Spin                                        1
        Wait                                        1

    progress_checker.movement_time_allowance, read back from
    controller_server at each transition:
      halt 1: at entry         10 s   at release         10 s
      halt 2: at entry         10 s   at release         10 s
      halt 3: at entry         10 s   at release         10 s
      halt 4: at entry         10 s   at release         10 s
      halt 5: at entry         10 s   at release         10 s
      halt 6: at entry         10 s   at release         10 s
      halt 7: at entry         10 s   at release         10 s
      halt 8: at entry         10 s   at release         10 s

    This is the CONTROL run. Nothing suppresses the progress checker,
    so any recovery counted here is what the suppressed run prevents.
    behavior_server publishes cmd_vel, so a recovery during a halt is
    a second writer contending for a robot the gate has stopped.
------------------------------------------------------------------------------
VERDICT
    the gate halted at least once:                     YES
    no command left the gate while latched:            YES
    sensor-to-zero latency was measured:               YES
    RESULT: PASS
==============================================================================
```
