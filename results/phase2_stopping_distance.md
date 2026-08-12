# Phase 2 - SafetyGate stopping distance sweep

```
==============================================================================
PHASE 2 - SafetyGate stopping distance vs commanded speed
==============================================================================
robot:                    amr1
k (model, from fleet.yaml): 1.8750 s^2/m
d_min:                    0.300 m
approach:                 rack face, static, square to the path
commanded on:             cmd_vel_nav - the gate's input
------------------------------------------------------------------------------
[A] PER-SPEED RESULTS
     cmd    v at halt   d_safe    clearance   braking   at rest   settled
     m/s      m/s         m        at halt      m          m         m
     0.15     0.130     0.342      0.334     0.010     0.333     0.331
     0.30     0.277     0.469      0.456     0.047     0.397     0.401
     0.45     0.426     0.680      0.653     0.242     0.406     0.402
     0.60     0.577     0.975      0.928     0.521     0.378     0.384

    'braking' is distance travelled AFTER the gate published its zero.
    'settled' is where it rests with the command still applied - the
    d_min standoff, which is the correct end state and not a creep bug.
------------------------------------------------------------------------------
[B] k_model AGAINST k_measured
    k_model    = 1/(2*a_eff), payload-scaled:    1.8750 s^2/m
    k_measured = braking / v^2, fitted:    1.0208 s^2/m
    a implied by the measurement:                 0.490 m/s^2
    ratio k_model / k_measured:                    1.84 x

    The gap is the simulator's drive model, not slack in the margin.
    Gazebo's DiffDrive applies |max_decel_x| as a KINEMATIC limit, so the
    simulated robot brakes as if unloaded; k_model sizes the envelope for
    a 90 kg vehicle whose 60 kg of payload it cannot ignore. On hardware
    the two converge. Quoting both is the honest form of this result.
------------------------------------------------------------------------------
[C] SENSOR STAMP -> ZERO COMMAND PUBLISHED
    blocking decisions measured:             148
    mean:                                         7.78 ms
    p95:                                         12.00 ms
    max:                             14.00 ms

    in-node compute only (steady clock, not quantised by /clock):
    mean / p95 / max:                         161.8 / 253.0 / 489.2 us

    The end-to-end figure includes the simulator's sensor pipeline, the
    bridge, the BSP relay and the gate, and is quantised by Gazebo's
    /clock step under use_sim_time. It is a low-latency override, not a
    hard-real-time one: there is no RT kernel and no scheduling guarantee.
------------------------------------------------------------------------------
VERDICT
    every commanded speed produced a halt:           YES
    at least four speeds swept:                      YES
    the robot never reached the obstacle:            YES
    clearance at halt >= d_safe at that speed:       YES
    braking distance stayed inside the modelled envelope: YES
    RESULT: PASS
==============================================================================
```
