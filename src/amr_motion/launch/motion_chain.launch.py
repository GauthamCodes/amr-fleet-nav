"""The two command-chain links that are not Nav2 servers.

    Nav2 -> cmd_vel_nav -> [velocity_smoother]   <- nav2.launch.py, lifecycle
                        -> cmd_vel_smoothed
                        -> PayloadJerkAdapter    <- here
                        -> cmd_vel_shaped
                        -> twist_mux             <- here
                        -> cmd_vel_mux
                        -> SafetyGate            <- amr_safety, serial and LAST

The stock smoother lives in ``nav2.launch.py`` because it is a Nav2 lifecycle
node and has to be brought up and managed with the rest of them. These two are
ordinary nodes, so they compose here.

The twist_mux CONFIG comes from ``amr_safety`` (PLAN.md section 7) even though
the node is launched here. The priority table is a safety-policy decision -
which source may override which - and it belongs beside the gate it feeds.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetParameter

from amr_bsp.topics import CMD_MUX


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    mux_config = os.path.join(
        get_package_share_directory("amr_safety"), "config", "twist_mux.yaml"
    )

    return [
        GroupAction(
            [
                PushROSNamespace(robot_name),
                SetParameter("use_sim_time", True),
                Node(
                    package="amr_motion",
                    executable="payload_jerk_adapter",
                    name="payload_jerk_adapter",
                    output="screen",
                    parameters=[{"robot": robot_name}],
                ),
                # amr_safety's priority_mux, not the stock twist_mux binary:
                # ros-jazzy-twist-mux 4.5.0 in this snapshot is linked against a
                # newer diagnostic_updater than is installed and does not load.
                # Same config schema, same semantics - see the config file.
                Node(
                    package="amr_safety",
                    executable="priority_mux.py",
                    name="twist_mux",
                    output="screen",
                    parameters=[mux_config, {"output_topic": CMD_MUX}],
                ),
            ]
        )
    ]


def generate_launch_description():
    """Return one robot's PayloadJerkAdapter and command mux."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            OpaqueFunction(function=_setup),
        ]
    )
