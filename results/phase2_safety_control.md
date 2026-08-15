# Phase 2 - SafetyGate halt on a pedestrian encounter

```
==============================================================================
PHASE 2 - SafetyGate halt on a pedestrian encounter
==============================================================================
robot:                    amr1
recovery suppression:     OFF  <- CONTROL RUN
goal result:              SUCCEEDED
goal dispatches:          1
k / d_min:                1.8750 s^2/m / 0.300 m
------------------------------------------------------------------------------
[A] HALTS
    halts:                                           4
    GATE-REPORTED leaked commands:                   0   <- must be 0
    observed by this probe (lagged, see below):       0
        The gate counts a leak by comparing the twist it is about to
        publish against its latch at that instant. This probe can only
        see the latch through diagnostics, published once per scan, so
        a command passed legitimately in the ~100 ms after a release
        reads to it as a leak. The gate's own count is the invariant;
        the probe's is kept because a large gap between them would mean
        the gate is releasing far more often than the report suggests.
    upstream kept commanding motion:                38
    furthest the robot moved while blocked:      0.224 m

      #   clearance   speed    d_safe   held s   latency ms   recov
      1       0.871   0.600     0.975     0.70          5.0      0
      2       0.023   0.069     0.309     0.30          7.0      0
      3       0.418   0.364     0.548     0.60          9.0      0
      4       0.012   0.066     0.308     0.30          6.0      0
------------------------------------------------------------------------------
[B] SENSOR STAMP -> ZERO COMMAND PUBLISHED
    blocking decisions:                 19
    mean:                                         7.63 ms
    p95:                                         11.00 ms
    max:                             11.00 ms
    in-node compute, mean / p95 / max:        125.5 / 204.4 / 744.5 us

    End-to-end covers the simulator's sensor pipeline, the bridge, the
    BSP relay and the gate, and is quantised by Gazebo's /clock step
    under use_sim_time; the compute figure is not. Low-latency safety
    override - not hard real-time: no RT kernel, no scheduling guarantee.
------------------------------------------------------------------------------
[C] NAV2 RECOVERY BEHAVIOUR DURING THE HALT  (ENGINEERING_NOTES rule 2)
    recovery behaviours, whole run:        3
    recovery behaviours fired DURING a halt:         0
        BackUp                                      1
        Spin                                        1
        Wait                                        1

    progress_checker.movement_time_allowance, read back from
    controller_server at each transition:
      halt 1: at entry         10 s   at release         10 s
      halt 2: at entry         10 s   at release         10 s
      halt 3: at entry         10 s   at release         10 s
      halt 4: at entry         10 s   at release         10 s

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
