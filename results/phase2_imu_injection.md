# Phase 2 - SensorBSP validation: injected implausible IMU angular velocity

```
==============================================================================
PHASE 2 - SensorBSP validation: injected implausible IMU angular velocity
==============================================================================
robot:                    amr1
sample window:            35 s
------------------------------------------------------------------------------
[A] PER-CHANNEL OUTCOMES  (from the BSP's own DiagnosticArray)
      channel   accepted   warned   rejected   relay mean/p95/max ms
          imu      3194        0        500    0.81 /  2.00 / 22.00
                status: 500 rejected: implausible angular velocity: |w_z| = 50.000 rad/s > 4.000 rad/s
        lidar       369        0          0    7.77 / 11.00 / 16.00
                status: nominal
------------------------------------------------------------------------------
[B] INJECTED FAULT  (implausible IMU angular velocity)
    injected magnitude:                       50.0 rad/s
    corrupt samples seen on the raw input:     500
    validated IMU samples published:          3088
    corrupt samples on validated/imu:            0   <- must be 0
    peak |w| ever seen on validated/imu:     0.001 rad/s

    The two claims are separate. Rejecting the fault is the first; not
    also discarding the healthy stream around it is the second, and the
    accepted counts in [A] are what carry it.
------------------------------------------------------------------------------
[C] WHAT THE NODE LOGGED  (verbatim, from /rosout at WARN or above)
    [amr1.sensor_bsp] imu REJECT (1): implausible angular velocity: |w_z| = 50.000 rad/s > 4.000 rad/s
    [amr1.sensor_bsp] no IMU attitude within 0.150 s of the scan stamp; republishing UNTRUNCATED (degraded, not unsafe - see bsp_node docstring)
    [amr1.sensor_bsp] imu REJECT (201): implausible angular velocity: |w_z| = 50.000 rad/s > 4.000 rad/s
    [amr1.sensor_bsp] imu REJECT (402): implausible angular velocity: |w_z| = 50.000 rad/s > 4.000 rad/s
------------------------------------------------------------------------------
VERDICT
    channels carried traffic:                YES
    the injected fault was rejected:         YES
    the node logged a WARN naming it:        YES
    no corrupt sample reached validated/imu: YES
    the healthy stream still flowed:         YES
    RESULT: PASS
==============================================================================
```
