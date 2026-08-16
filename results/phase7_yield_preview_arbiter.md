# PHASE 7 - forced narrow-intersection conflict, escalated to a yield

```
==============================================================================
PHASE 7 - forced narrow-intersection conflict, escalated to a yield
==============================================================================
PRIORITY, DERIVED FROM fleet.yaml GROSS MASS - no robot is named in code

      rank  robot         gross mass   footprint radius
         1  amr1             90.0 kg          0.539 m
         2  amr2             23.0 kg          0.429 m

    conflict radius:          1.94 m   (r_a + r_b + d_safe_max)
    time window:              5.00 s
    escalate after:           2.00 s of unresolved conflict
    release radius:           2.43 m
    max hold (fail-safe):    45.00 s
------------------------------------------------------------------------------
[A] CONFLICT PIPELINE  (rule 7: the local layer first, this node only after)
    conflicts predicted:                             4
    resolved WITHOUT escalation:                     3
    escalated to a yield:                            1

      #  pair               outcome           age    min sep    gain
      1  amr1/amr2          escalated          2.0s     1.35m   -0.52m
      2  amr1/amr2          resolved locally   0.6s     1.91m   +0.05m
      3  amr1/amr2          resolved locally   0.4s     1.94m   +0.03m
      4  amr1/amr2          resolved locally   0.4s     1.81m   +0.18m

    'gain' is how far the PREDICTED closest approach opened up over the
    life of the conflict. A conflict that opens by more than
    0.15 m is the local layer resolving it, and this node
    leaves it alone. Escalation needs BOTH a conflict older than
    2.0 s and a separation that has stopped improving.
------------------------------------------------------------------------------
[B] YIELDS COMMANDED

      #  robot  yields to   held     release condition        entry  min sep
      1  amr2   amr1          1.0s   conflict cleared        1.35m   1.34m

    yield 1: escalated 2.0 s after the conflict was first predicted, at t = 42.2 s
             21 zero-twist commands published on amr2/cmd_vel_yield (mux priority 150)
             released at t = 43.2 s on: conflict cleared

    A yield is commanded by PUBLISHING zero on the mux's priority-150
    channel and released by ceasing to publish - the mux's own 0.5 s
    timeout drops the channel, so there is no release message to lose and
    an arbiter that dies releases the robot rather than pinning it.
------------------------------------------------------------------------------
[C] NAV2 RECOVERY DURING THE HOLD  (ENGINEERING_NOTES rule 2)
    recovery suppression:                           ON
    recovery behaviours fired DURING a hold:         0
    amr1, whole run:       3   BackUp x1, Spin x1, Wait x1
    amr2, whole run:       0   

    progress_checker.movement_time_allowance, read back from controller_server:
      yield 1 (amr2): at entry    1000000 s   after release         10 s

    Read back rather than assumed. A write that silently failed would
    otherwise be indistinguishable from a mechanism that worked and was
    never needed - which is exactly what Phase 2 could not rule out until
    Phase 3 produced a halt longer than the allowance.
------------------------------------------------------------------------------
[D] SAFETY GATE STATE DURING THE HOLD  (is this a yield or a halt?)
    yield 1 (amr2): SafetyGate was blocking on 0 of 21 held cycles (0 %)

    A yield and a safety halt both end with a robot at a standstill. The
    gate's own diagnostics are recorded alongside the hold so the two are
    never conflated: 0 % means the robot was stopped by this arbiter and
    by nothing else.
------------------------------------------------------------------------------
[E] VERDICT
    a conflict was predicted:                            YES
    a conflict was escalated to a yield:                 YES
    every yield was given by the lower-priority robot:   YES
    every hold ended:                                    YES
    no hold ended on the fail-safe ceiling:              YES
    no recovery behaviour fired during any hold:         YES
    the allowance was read back raised at every hold:    YES
    RESULT: PASS
==============================================================================
```
