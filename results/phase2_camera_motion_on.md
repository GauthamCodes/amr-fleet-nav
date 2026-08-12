# Phase 2 - SensorBSP validation: camera ENABLED, robot driving

```
==============================================================================
PHASE 2 - SensorBSP validation: camera ENABLED, robot driving
==============================================================================
robot:                    amr1
sample window:            40 s
------------------------------------------------------------------------------
[A] PER-CHANNEL OUTCOMES  (from the BSP's own DiagnosticArray)
      channel   accepted   warned   rejected   relay mean/p95/max ms
       camera       419        0          0    8.58 / 13.00 / 15.00
                status: nominal
          imu      4199        0          0    0.25 /  1.00 /  3.00
                status: nominal
        lidar       419        0          0   13.29 / 18.00 / 23.00
                status: nominal
------------------------------------------------------------------------------
[M] MOTION DURING THE WINDOW  (from odometry)
    peak measured speed:                    0.3500 m/s
    distance travelled:                    13.5317 m
    A stationary run should report ~0 for both; a driving run is
    measuring what the sensor load costs the simulator's drive.
------------------------------------------------------------------------------
VERDICT
    channels carried traffic:                YES
    camera channel present:                  YES
    camera frames accepted:                  YES
    RESULT: PASS
==============================================================================
```
