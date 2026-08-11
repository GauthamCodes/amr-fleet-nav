# Smoke Test 2 - ramp-induced phantom ground return

```

==============================================================================
SMOKE TEST 2 - ramp-induced phantom ground return
==============================================================================
world:          ramp_angle = 8.00 deg, rise = 0.500 m
                ramp toe x = 2.4423, crest x = 6.0000
robot:          amr1, lidar height h = 0.200 m
scan:           360 beams, 0.12-12.00 m, sector +/-15 deg
samples:        952 scans
trajectory:     world x -2.000 -> +9.071 m (206 scans taken on the ramp)
                max forward speed +0.350 m/s, max reverse -0.350 m/s
------------------------------------------------------------------------------
[A] PHANTOM RAMP WALL  (robot level, on flat ground, approaching the toe)
    ramp surface reaches scan height at x =   3.8654 m
    i.e. offset past the toe   h/tan(a)  =   1.4231 m
    samples in approach phase:                 113
    mean (measured - predicted) range:      -0.208 m
    max |error| over approach:               0.315 m
    -> the ramp DOES read as a solid wall to a level 2D scan
------------------------------------------------------------------------------
[0] ODOMETRY INTEGRITY  (wheel odometry vs Gazebo ground truth)
    ground-truth samples:                      952
    final odom world x:                     -1.681 m
    final ground-truth world x:             -1.681 m
    final discrepancy:                      -0.000 m
    peak discrepancy:                       +0.035 m
    -> odometry is TRUSTWORTHY
------------------------------------------------------------------------------
[B] ATTITUDE EXTREMES
    max |pitch| over run:                    8.002 deg   at t = 15.40 s
    (IMU reported at that instant:           8.002 deg)
    sign at max:                          nose-UP
    robot world x at that moment:            2.622 m
    on the ramp at that moment:                YES
    nominal ramp angle for comparison:       8.000 deg
    nose-sector min range there:              inf
    tail-sector min range there:              1.78 m
    reference h/sin(|pitch|):                 1.44 m
    scans with |pitch| > 2 deg:                 218
------------------------------------------------------------------------------
[C] RETURN ALONG THE DIRECTION OF TRAVEL  (reverse descent, nose uphill)
    samples while descending pitched:          109
    closest return in travel direction:       1.77 m
    pitch at that moment:                   -8.000 deg
    speed at that moment:                   -0.350 m/s
    reference h/sin(|pitch|):                 1.44 m
------------------------------------------------------------------------------
PITCH GATE SIZING  (input to Phase 2)
    closest return seen, by |pitch| bin:
      |pitch| ~  0 deg:                          1.35 m
      |pitch| ~  1 deg:                          1.44 m
      |pitch| ~  2 deg:                          1.72 m
      |pitch| ~  3 deg:                          1.98 m
      |pitch| ~  4 deg:                          2.40 m
      |pitch| ~  5 deg:                          2.42 m
      |pitch| ~  6 deg:                          2.07 m
      |pitch| ~  7 deg:                          1.86 m
      |pitch| ~  8 deg:                          1.75 m
    d_safe at max_vel_x=0.60 m/s (k=0.5, d_min=0.3):     0.48 m
    closest phantom return vs d_safe:         1.77 m >= 0.48 m
    -> a phantom return WOULD NOT trip the safety gate at full speed

==============================================================================
HEADLINE NUMBERS  (for the README)
    ramp angle:                                      8.00 deg
    max pitch measured on the ramp:                 8.002 deg
    lidar height above ground:                      0.200 m
    phantom ground return at max pitch:              1.78 m
    same, along the direction of travel:             1.77 m
    phantom ramp wall, robot level:                  1.42 m past the toe
==============================================================================

```
