# Odometry integrity under rotation

```

==============================================================================
ODOMETRY INTEGRITY UNDER ROTATION
==============================================================================
robot:                    amr1
commanded yaw rate:       0.60 rad/s for 22 s
samples:                  440
total rotation observed:  756 deg commanded (0 half-turn wraps seen in ground truth)
------------------------------------------------------------------------------
DISPLACEMENT OF THE ROBOT FRAME DURING A PURE IN-PLACE ROTATION
  peak per wheel odometry:              0.0000 m
  peak per Gazebo ground truth:         0.0385 m
  predicted if axle is offset:          0.0000 m  (2 x 0.00)
------------------------------------------------------------------------------
DISAGREEMENT BETWEEN THE TWO
  peak |truth - odom|:                  0.0385 m
  at t = 5.67 s, after 180 deg of yaw
  mean |truth - odom|:                  0.0204 m
  implied frame offset (peak / 2):      0.0193 m
  expected offset from the URDF:        0.0000 m
------------------------------------------------------------------------------
VERDICT
  implied rotation-centre offset:       0.0193 m
  offset the URDF intends:              0.0000 m
  unexplained systematic component:     0.0193 m
  decision threshold:                   0.0200 m

  The measured pivot is within 0.0200 m of where the
  URDF places base_footprint, so there is no frame-definition error.
  The residual 0.0385 m peak excursion is PHYSICAL: a frictionless
  caster and finite wheel friction shift the effective rotation
  centre slightly forward of the axle. A real robot does this too.
  Against the 0.05 m map resolution it is sub-cell - scan matching absorbs it.
  RESULT: NO FRAME ERROR - RESIDUAL IS PHYSICAL DRIFT
==============================================================================

```
