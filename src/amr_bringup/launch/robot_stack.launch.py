"""One robot's software stack: SensorBSP -> slam_toolbox -> Nav2 -> SafetyGate.

This file composes, it does not configure. It contains no world, no Gazebo server and
no clock bridge - those are world singletons and live in
:func:`amr_gazebo.spawn.world_actions`. Splitting them out is what lets one launch
bring up one robot and another bring up the whole fleet without either including the
other.

PHASE 2 ADDED TWO LINKS, AND THIS IS THE ONLY FILE THAT KNOWS BOTH ENDS

    Sensor path:  gz -> /amrN/{scan,imu,camera} -> SensorBSP -> /amrN/validated/*
                  -> slam_toolbox and both Nav2 costmaps (docs/ENGINEERING_NOTES.md rule 8)

    Command path: controller_server and behavior_server -> /amrN/cmd_vel_nav
                  -> SafetyGate -> /amrN/cmd_vel -> drive_watchdog (plant)
                  -> /amrN/cmd_vel_plant -> DiffDrive (rule 1)

    The composition layer is also where the PitchGate/SafetyGate interlock is tied
    together, because it is the one place that legitimately sees both packages.

STAGING

    The default periods below are the Phase 1/2 single-robot values, unchanged. A
    fleet launch passes ``stagger`` to offset a second robot's stack so six more Nav2
    servers and another slam_toolbox do not all initialise in the same instant.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from amr_bsp.topics import CMD_MUX, CMD_NAV
from amr_description.fleet_config import load_fleet
from amr_navigation.params import SCAN_TOPIC
from amr_safety.safety_model import gate_parameters

#: Seconds after launch start at which each layer comes up, before ``stagger``.
BSP_START_S = 10.0
SLAM_START_S = 14.0
NAV2_START_S = 22.0
GATE_START_S = 23.0
PREDICTOR_START_S = 24.0


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    with_nav2 = LaunchConfiguration("with_nav2").perform(context).lower() != "false"
    suppress_recovery = LaunchConfiguration("suppress_recovery").perform(context)
    with_motion_chain = (
        LaunchConfiguration("with_motion_chain").perform(context).lower() != "false"
    )
    stagger = float(LaunchConfiguration("stagger").perform(context))

    fleet = {r["name"]: r for r in load_fleet()}
    if robot_name not in fleet:
        raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
    robot = fleet[robot_name]

    def include(package, launch_file, **extra):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(package), "launch", launch_file
                )
            ),
            launch_arguments={
                "robot": robot_name,
                **extra,
            }.items(),
        )

    def include_scanned(package, launch_file):
        return include(package, launch_file, scan_topic=scan_topic)

    staged = [
        # The BSP comes up BEFORE anything that consumes a scan. It is the only
        # publisher of validated/scan, and slam_toolbox subscribing to a topic with
        # no publisher is the failure mode Phase 1 already paid for once.
        #
        # min_gate_range is computed here rather than inside bsp.launch.py: it is the
        # PitchGate/SafetyGate interlock, so its value belongs to amr_safety, and
        # amr_safety already depends on amr_bsp for the topic contract. Composing it
        # at this layer is what keeps that from becoming a dependency cycle.
        TimerAction(
            period=BSP_START_S + stagger,
            actions=[
                include(
                    "amr_bsp",
                    "bsp.launch.py",
                    min_gate_range=f"{gate_parameters(robot)['min_gate_range']:.4f}",
                )
            ],
        ),
        # SLAM next: it owns amrN/map -> amrN/odom, and from Phase 3 onwards its map
        # is also one of FleetMapNode's two inputs.
        TimerAction(
            period=SLAM_START_S + stagger,
            actions=[include_scanned("amr_navigation", "slam.launch.py")],
        ),
    ]
    # The survey run maps without navigating, and MPPI evaluates 1500 trajectories
    # at 20 Hz whether or not a goal exists. Leaving Nav2 out of that run returns
    # the CPU to the simulator, where it buys real-time factor and therefore scans.
    if with_nav2:
        staged.append(
            TimerAction(
                period=NAV2_START_S + stagger,
                actions=[
                    include(
                        "amr_navigation",
                        "nav2.launch.py",
                        scan_topic=scan_topic,
                        with_motion_chain=str(with_motion_chain).lower(),
                        with_trajectory_layer=LaunchConfiguration(
                            "with_trajectory_layer"
                        ).perform(context),
                    )
                ],
            )
        )
        # The two non-lifecycle links of the motion chain. They go up with Nav2
        # because the stock smoother that feeds them is one of its servers.
        if with_motion_chain:
            staged.append(
                TimerAction(
                    period=NAV2_START_S + stagger,
                    actions=[include("amr_motion", "motion_chain.launch.py")],
                )
            )
        # The gate goes up WITH Nav2, not before it: its only job until Nav2 exists
        # would be to publish zeros, and its recovery-suppression client would spend
        # that time reporting that controller_server is not there yet.
        #
        # ITS INPUT MOVES WITH THE CHAIN. With the motion chain in, the gate reads
        # the mux output; without it, it reads Nav2 directly, which is the Phase 1
        # and Phase 2 wiring unchanged. The gate itself does not know the
        # difference - it is a serial link with one input, and that is the point
        # of rule 1.
        staged.append(
            TimerAction(
                period=GATE_START_S + stagger,
                actions=[
                    include(
                        "amr_safety",
                        "safety_gate.launch.py",
                        scan_topic=scan_topic,
                        suppress_recovery=suppress_recovery,
                        cmd_in_topic=CMD_MUX if with_motion_chain else CMD_NAV,
                    )
                ],
            )
        )
        # The predictor comes up WITH Nav2 because its primary input is /plan.
        # Without a planner it would fall back to constant-velocity projection
        # forever, which is a correct answer to a question nobody asked.
        staged.append(
            TimerAction(
                period=PREDICTOR_START_S + stagger,
                actions=[
                    include("amr_fleet_control", "trajectory_predictor.launch.py")
                ],
            )
        )
    return staged


def generate_launch_description():
    """Return one robot's SensorBSP, SLAM, Nav2 and SafetyGate stack."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            # Imported, not restated. Phase 1 declared the literal "scan" here as
            # well as in amr_navigation.params, so flipping the seam in one place
            # would have left this launch - the one every deliverable run goes
            # through - still pointing at the raw sensor.
            DeclareLaunchArgument("scan_topic", default_value=SCAN_TOPIC),
            DeclareLaunchArgument(
                "with_nav2",
                default_value="true",
                description="Bring up the Nav2 servers. false gives SLAM only, "
                "which is what the mapping survey needs.",
            ),
            DeclareLaunchArgument(
                "suppress_recovery",
                default_value="true",
                description="Set false for the control run that shows Nav2 DOES "
                "fire recovery behaviours during a halt when nothing suppresses "
                "them. Without that control, a zero recovery count proves nothing.",
            ),
            DeclareLaunchArgument(
                "with_motion_chain",
                default_value="true",
                description="Insert nav2_velocity_smoother -> PayloadJerkAdapter "
                "-> twist_mux between Nav2 and SafetyGate. false restores the "
                "direct cmd_vel_nav -> SafetyGate path, which is the Phase 1 and "
                "Phase 2 wiring and the control arm of the Phase 5 A/B.",
            ),
            DeclareLaunchArgument(
                "with_trajectory_layer",
                default_value="true",
                description="Load FleetTrajectoryLayer into this robot's local "
                "costmap. false is the Phase 6 control arm.",
            ),
            DeclareLaunchArgument(
                "stagger",
                default_value="0.0",
                description="Seconds to delay this robot's whole stack. Lets a "
                "fleet launch avoid bringing up two Nav2 stacks in one instant.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
