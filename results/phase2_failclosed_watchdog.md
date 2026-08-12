# Phase 2 - fail-closed SIGKILL test

```
==============================================================================
PHASE 2 - fail-closed: SafetyGate SIGKILLed while the robot was moving
==============================================================================
robot:                    amr1
plant-side watchdog:      PRESENT
gate pids killed:         215285
signal:                   SIGKILL (no shutdown handler runs)
upstream:                 kept commanding throughout
------------------------------------------------------------------------------
speed at the kill:                           0.350 m/s
peak speed during the run:                   0.350 m/s
observation window after the kill:            10.0 s
speed at the end of the window:              0.000 m/s
distance travelled after the kill:           0.196 m
came to rest after:                          0.196 m / 0.75 s
------------------------------------------------------------------------------
READING THIS
    With the watchdog present, the plant stops itself when the
    command stream goes quiet, which is what a real motor controller
    does. The gate being the only publisher of cmd_vel is what makes
    the stream go quiet; the watchdog is what acts on it.

    Both runs share the property that matters: no NEW command can reach
    the wheels once the gate is gone, because nothing else publishes the
    topic the plant listens to. A twist_mux in the same position would
    fail OPEN and pass the planner's commands straight through.
==============================================================================
```
