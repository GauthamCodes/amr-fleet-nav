# Phase 2 - PitchGate: ramp phantom ground return

```
==============================================================================
PHASE 2 - PitchGate: ramp phantom ground return, before and after
==============================================================================
robot:                    amr1
lidar height h:           0.350 m
gate margin:              0.90 x h/sin|pitch|
gate floor (interlock):   1.565 m
validated scans:          903
------------------------------------------------------------------------------
[0] HEADER STAMP PRESERVATION  (ENGINEERING_NOTES rule 4)
    raw/validated pairs matched by EXACT stamp:      903
    RESTAMPED (no matching raw stamp):                 0
    max |stamp difference| within a pair:              0 ns
    skipped, acquired before this probe started:       0
    -> original acquisition stamps are PRESERVED

    raw scans whose stamp did NOT advance:             0
        must be 0. Nonzero means a second publisher is on the raw
        topic - almost always a stray gz sim left from an earlier run -
        and every number below it is measured on a corrupted stream.
------------------------------------------------------------------------------
[A] TRUNCATION BY |PITCH| BIN
      bin    scans   mean beams cut   closest cut   min range after
      0 deg     644            0.0           inf              2.36
      1 deg       8            0.0           inf              2.56
      2 deg       7            4.9          7.96              2.73
      3 deg       7           35.0          5.64              2.73
      4 deg       7           70.9          4.57              2.73
      5 deg       6           84.0          3.83              2.73
      6 deg       7           92.9          3.42              2.73
      7 deg       7          111.7          2.97              2.74
      8 deg     209          105.8          2.77              2.72
------------------------------------------------------------------------------
[B] THE MAX-PITCH SCAN  (the headline before/after)
    |pitch|:                                     8.001 deg
    attitude:                                  nose-UP
    predicted ground intersection h/sin|p|:      2.514 m
    gate radius at that pitch (0.9 x):           2.263 m
    beams in the scan:                             360
    BEAMS TRUNCATED:                               120
    closest return BEFORE:                       2.732 m
    closest return AFTER:                        2.732 m
    closest return that was removed:             2.769 m
------------------------------------------------------------------------------
[C] WHAT WAS NOT TRUNCATED  (the half that matters for safety)
    beams truncated, all scans:                  24833
    as a fraction of all beams seen:              7.65 %

    INVARIANTS - each must be zero, and each is checked per beam:
      cut while pointing UPHILL:                     0
          a beam with no downward component cannot strike the ground,
          so it has no gate radius and must never be cut
      cut while INSIDE its own gate radius:          0
          the truncation rule itself: only returns BEYOND the predicted
          ground intersection are ground
      cut INSIDE the braking envelope:               0
          the floor is 1.565 m = d_safe(v_max) + footprint
          + margin; nonzero means truncation could delete an obstacle
          the SafetyGate would have had to stop for

    DESCRIPTIVE (not pass/fail - the gate is legitimately active here):
      widest |azimuth| truncated:                180.0 deg
      cut within 15 deg of +/-90:                    0
          a beam 30 deg off-axis still carries 87 % of the forward
          depression, so a nonzero count here is geometry, not a fault
------------------------------------------------------------------------------
[D] BSP RELAY LATENCY  (stamp -> validated publish, lidar channel)
    samples:                                        90
    mean of the BSP's own rolling mean:           8.51 ms
    p95:                                          8.89 ms
    max:                                          8.90 ms
    PLAN.md section 4 keeps the BSP in Python until measurement
    says otherwise. This is that measurement.
------------------------------------------------------------------------------
VERDICT
    single publisher on the raw topic: YES
    stamps preserved:                  YES
    phantom truncated at max pitch:    YES
    nothing cut pointing uphill:       YES
    nothing cut inside its gate:       YES
    braking envelope untouched:        YES
    RESULT: PASS
==============================================================================
```
