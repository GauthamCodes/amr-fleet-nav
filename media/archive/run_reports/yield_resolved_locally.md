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
    time window:              4.00 s
    escalate after:           2.00 s of unresolved conflict
    release radius:           2.43 m
    max hold (fail-safe):    45.00 s
------------------------------------------------------------------------------
[A] CONFLICT PIPELINE  (rule 7: the local layer first, this node only after)
    conflicts predicted:                             4
    resolved WITHOUT escalation:                     4
    escalated to a yield:                            0

      #  pair               outcome           age    min sep    gain
      1  amr1/amr2          resolved locally   0.8s     1.51m   +0.58m
      2  amr1/amr2          resolved locally   0.6s     1.87m   +0.09m
      3  amr1/amr2          resolved locally   0.4s     1.93m   +0.02m
      4  amr1/amr2          resolved locally   0.8s     1.65m   +0.30m

    'gain' is how far the PREDICTED closest approach opened up over the
    life of the conflict. A conflict that opens by more than
    0.15 m is the local layer resolving it, and this node
    leaves it alone. Escalation needs BOTH a conflict older than
    2.0 s and a separation that has stopped improving.
------------------------------------------------------------------------------
[B] YIELDS COMMANDED
    none - no conflict met the escalation test
------------------------------------------------------------------------------
[C] NAV2 RECOVERY DURING THE HOLD  (ENGINEERING_NOTES rule 2)
    recovery suppression:                           ON
    recovery behaviours fired DURING a hold:         0
    amr1, whole run:       2   Spin x1, Wait x1
    amr2, whole run:       0   

    progress_checker.movement_time_allowance, read back from controller_server:

    Read back rather than assumed. A write that silently failed would
    otherwise be indistinguishable from a mechanism that worked and was
    never needed - which is exactly what Phase 2 could not rule out until
    Phase 3 produced a halt longer than the allowance.
------------------------------------------------------------------------------
[D] SAFETY GATE STATE DURING THE HOLD  (is this a yield or a halt?)
    no hold to attribute
------------------------------------------------------------------------------
[E] VERDICT
    conflicts predicted:                             4
    resolved by the local layer:                     4
    escalated:                                       0
    RESULT: NOT EXERCISED

        No conflict in this run met the escalation test, so there is no
        yield to pass or fail on. This says the encounter did not
        require central arbitration - which for a conflict the local
        layer opened up is the CORRECT outcome (rule 7) - and it says
        nothing about whether the protocol works. The run that
        exercises it is the one with a yield in section B.
==============================================================================
```
