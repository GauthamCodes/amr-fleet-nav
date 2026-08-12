# Phase 2 - SensorBSP validation: camera DISABLED, robot driving

```
==============================================================================
PHASE 2 - SensorBSP validation: camera DISABLED, robot driving
==============================================================================
robot:                    amr1
sample window:            40 s
------------------------------------------------------------------------------
[A] PER-CHANNEL OUTCOMES  (from the BSP's own DiagnosticArray)
      channel   accepted   warned   rejected   relay mean/p95/max ms
          imu      4300        0          0    0.19 /  1.00 /  2.00
                status: nominal
        lidar       429        0          0    6.29 / 10.00 / 14.00
                status: nominal
------------------------------------------------------------------------------
[M] MOTION DURING THE WINDOW  (from odometry)
    peak measured speed:                    0.3500 m/s
    distance travelled:                    13.8572 m
    A stationary run should report ~0 for both; a driving run is
    measuring what the sensor load costs the simulator's drive.
------------------------------------------------------------------------------
VERDICT
    channels carried traffic:                YES
    camera channel present:                  NO
    camera frames accepted:                  NO
    RESULT: FAIL
==============================================================================
```
