# Phase 3 - selective fleet-map update policy

```
==============================================================================
PHASE 3 - selective fleet-map update policy
==============================================================================
fleet grid:               GridSpec(origin=(-15.00, -10.00), res=0.05, 680x400)
robots:                   amr1, amr2
------------------------------------------------------------------------------
[A] POLICY
    score = w_f*frontier + w_c*change + w_r*recency - w_v*revisit
    accept_threshold:               0.350
    change_scale_cells:           120.000
    frontier_scale_cells:         400.000
    recency_horizon_s:             20.000
    revisit_scale:                  6.000
    w_change:                       1.200
    w_frontier:                     1.200
    w_recency:                      1.000
    w_revisit:                      0.600
------------------------------------------------------------------------------
[B] DECISIONS
    candidates scored:                             41
    accepted (merged and published):               23
    deferred:                                      18
    deferred share:                             43.9 %

      robot        accepted   deferred   deferred %
      amr1               13          8        38.1
      amr2               10         10        50.0
------------------------------------------------------------------------------
[C] WHAT DEFERRAL SAVES
    composites performed:                          17
    mean composite + publish (ms):               0.55
    estimated work avoided (ms):                 10.0

    ESTIMATED, and stated as such: it is the deferred count times the
    measured mean cost of the composite and publish those deferrals
    skipped. Scoring still runs on every candidate; that cost is not
    saved and is not claimed.

    The absolute figure is small, and it is worth saying why rather
    than leaving it to look like a result. Compositing is numpy slice
    arithmetic on a fixed grid, so it was never the expensive part.
    What deferral actually bounds is how often a 680x400 grid is
    serialised to TWO global costmaps that each reprocess it - work
    that happens in the Nav2 processes and is not measured here. The
    deferred SHARE above is the honest headline; the milliseconds are
    a lower bound on one end of the saving.
------------------------------------------------------------------------------
[D] FLEET MAP
    known cells:                                15.6 %
    occupied cells:                              1984
==============================================================================
```
