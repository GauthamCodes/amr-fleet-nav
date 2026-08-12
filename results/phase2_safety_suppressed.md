# Phase 2 - SafetyGate halt on a pedestrian encounter

```
==============================================================================
PHASE 2 - SafetyGate halt on a pedestrian encounter
==============================================================================
robot:                    amr1
recovery suppression:     ON
goal result:              SUCCEEDED
goal dispatches:          1
k / d_min:                1.8750 s^2/m / 0.300 m
------------------------------------------------------------------------------
[A] HALTS
    halts:                                           3
    GATE-REPORTED leaked commands:                   0   <- must be 0
    observed by this probe (lagged, see below):       2
        The gate counts a leak by comparing the twist it is about to
        publish against its latch at that instant. This probe can only
        see the latch through diagnostics, published once per scan, so
        a command passed legitimately in the ~100 ms after a release
        reads to it as a leak. The gate's own count is the invariant;
        the probe's is kept because a large gap between them would mean
        the gate is releasing far more often than the report suggests.
    upstream kept commanding motion:                26
    furthest the robot moved while blocked:      0.201 m

      #   clearance   speed    d_safe   held s   latency ms   recov
      1       0.841   0.600     0.975     0.60          9.0      0
      2       0.010   0.254     0.421     0.39         12.0      0
      3       0.969   0.600     0.975     0.30          7.0      0
------------------------------------------------------------------------------
[B] SENSOR STAMP -> ZERO COMMAND PUBLISHED
    blocking decisions:                 13
    mean:                                         8.46 ms
    p95:                                         11.00 ms
    max:                             12.00 ms
    in-node compute, mean / p95 / max:        151.1 / 225.8 / 710.8 us

    End-to-end covers the simulator's sensor pipeline, the bridge, the
    BSP relay and the gate, and is quantised by Gazebo's /clock step
    under use_sim_time; the compute figure is not. Low-latency safety
    override - not hard real-time: no RT kernel, no scheduling guarantee.
------------------------------------------------------------------------------
[C] NAV2 RECOVERY BEHAVIOUR DURING THE HALT  (docs/ENGINEERING_NOTES.md rule 2)
    recovery behaviours, whole run:        0
    recovery behaviours fired DURING a halt:         0
        none

    progress_checker.movement_time_allowance, read back from
    controller_server at each transition:
      halt 1: at entry    1000000 s   at release    1000000 s
      halt 2: at entry    1000000 s   at release    1000000 s
      halt 3: at entry    1000000 s   at release    1000000 s

    This is the SUPPRESSED run. Read it against the control run
    (suppress_recovery:=false). Zero here means something only if the
    control shows a nonzero count on the same encounter.
------------------------------------------------------------------------------
VERDICT
    the gate halted at least once:                     YES
    no command left the gate while latched:            YES
    sensor-to-zero latency was measured:               YES
    no recovery fired during any halt:                 YES
    the allowance was actually raised at a halt:       YES
    RESULT: PASS
==============================================================================
```
