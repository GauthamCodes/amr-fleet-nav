"""The topic contract between the sensors, the BSP, and everything downstream.

docs/ENGINEERING_NOTES.md rule 8 says sensor data reaches Nav2 only through SensorBSP. A rule like
that is only worth having if there is one place it can be checked, so the names live
here and every consumer imports them: ``amr_navigation.params`` renders them into
the Nav2 and slam_toolbox templates, the SafetyGate launch passes them to the C++
node, and the BSP itself subscribes to the raw side and publishes the validated one.

All names are RELATIVE. Each robot's stack runs under its own namespace, so ``scan``
resolves to ``/amr1/scan`` and ``validated/scan`` to ``/amr1/validated/scan`` without
any name carrying a robot in it (rule 5).

THE ONE DELIBERATE EXEMPTION

    Instruments whose purpose is to characterise the RAW sensor - smoke test 1's
    actor visibility check, smoke test 2's phantom-return measurement, and Phase 2's
    own before/after PitchGate probe - subscribe to the raw names by definition. They
    are measuring the thing the BSP exists to clean up. Every node in the NAVIGATION
    STACK subscribes to the validated names only, which is what the rule is about.
"""

#: What the ros_gz_bridge publishes, straight off the simulated sensors.
RAW_SCAN = "scan"
RAW_IMU = "imu"
RAW_CAMERA = "camera/image_raw"

#: What SensorBSP republishes, with the ORIGINAL header stamps (rule 4).
VALIDATED_SCAN = "validated/scan"
VALIDATED_IMU = "validated/imu"
VALIDATED_CAMERA = "validated/camera/image_raw"

#: The command chain. Nav2 is remapped onto CMD_NAV; SafetyGate consumes it and
#: publishes CMD_GATED, which is the topic the plant's watchdog reads.
#:     Nav2 (controller_server, behavior_server)
#:       -> CMD_NAV       cmd_vel_nav        remapped off Nav2's "cmd_vel"
#:       -> CMD_SMOOTHED  cmd_vel_smoothed   stock nav2_velocity_smoother
#:       -> CMD_SHAPED    cmd_vel_shaped     our PayloadJerkAdapter
#:       -> CMD_MUX       cmd_vel_mux        twist_mux (nav 100 | yield 150)
#:       -> CMD_GATED     cmd_vel            SafetyGate, serial and LAST
#:       -> CMD_PLANT     cmd_vel_plant      drive_watchdog -> DiffDrive
#:
#: SafetyGate is downstream of the mux, never one of its inputs (rule 1). A mux
#: picks the highest-priority live input and so fails OPEN when its own process
#: dies; the gate is a serial link and fails CLOSED. Putting the gate on a mux
#: channel would look like the same wiring and invert the failure mode.
CMD_NAV = "cmd_vel_nav"
CMD_SMOOTHED = "cmd_vel_smoothed"
CMD_SHAPED = "cmd_vel_shaped"
CMD_YIELD = "cmd_vel_yield"
CMD_MUX = "cmd_vel_mux"
CMD_GATED = "cmd_vel"
CMD_PLANT = "cmd_vel_plant"

#: Where a robot advertises its own intentions, and the topic every OTHER robot's
#: FleetTrajectoryLayer subscribes to. nav_msgs/Path in the ``fleet_map`` frame,
#: with each pose's ``header.stamp`` carrying the time that pose is predicted to
#: be REACHED rather than the time it was published - the convention that lets a
#: standard message express a space-TIME trajectory without a custom interface
#: package. See amr_fleet_control.trajectory_predictor for the producer.
PREDICTED_TRAJECTORY = "predicted_trajectory"
