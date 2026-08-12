# Phase 2 - fail-closed SIGKILL test

```
==============================================================================
PHASE 2 - fail-closed: SafetyGate SIGKILLed while the robot was moving
==============================================================================
robot:                    amr1
plant-side watchdog:      ABSENT  <- CONTROL RUN
gate pids killed:         216659
signal:                   SIGKILL (no shutdown handler runs)
upstream:                 kept commanding throughout
------------------------------------------------------------------------------
speed at the kill:                           0.350 m/s
peak speed during the run:                   0.350 m/s
observation window after the kill:            10.0 s
speed at the end of the window:              0.350 m/s
distance travelled after the kill:           3.500 m
came to rest:                             NO - still rolling
------------------------------------------------------------------------------
READING THIS
    Without it, gz-sim's DiffDrive keeps integrating the velocity it
    last latched - it has no command timeout - so the robot rolls on
    with no software alive to stop it. This is the control, and it is
    the reason the watchdog exists rather than being assumed.

    Both runs share the property that matters: no NEW command can reach
    the wheels once the gate is gone, because nothing else publishes the
    topic the plant listens to. A twist_mux in the same position would
    fail OPEN and pass the planner's commands straight through.
==============================================================================
```
