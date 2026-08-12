r"""PHASE 2 EXIT - fail-closed evidence: SIGKILL the gate while the robot is moving.

Run as an A/B, because the interesting quantity is what the plant-side watchdog buys:

    ros2 launch amr_bringup phase2_failclosed.launch.py tag:=watchdog
    ros2 launch amr_bringup phase2_failclosed.launch.py tag:=no_watchdog \
        with_watchdog:=false

No SLAM and no Nav2. The run needs a constant command flowing into the gate and
nothing else; a planner would only add a second reason for the robot to slow down
and make the measurement ambiguous. ``suppress_recovery`` is false for the same
reason it is in the stopping sweep: there is no controller_server here to suppress.

The robot drives down the aisle from its ordinary fleet.yaml spawn, so it has ~12 m
of clear runway before the ramp toe - long enough that the control case (no
watchdog, robot keeps rolling) runs out of observation window rather than road.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_bsp.topics import CMD_GATED, CMD_PLANT
from amr_description.fleet_config import load_fleet
from amr_gazebo.spawn import clock_bridge, gz_server, robot_actions
from amr_gazebo.world_builder import render_world, world_name_of
from amr_safety.safety_model import gate_parameters


def _setup(context, *args, **kwargs):
    gazebo_pkg = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    results_dir = LaunchConfiguration("results_dir").perform(context)
    tag = LaunchConfiguration("tag").perform(context)
    with_watchdog = (
        LaunchConfiguration("with_watchdog").perform(context).lower() != "false"
    )

    world_sdf = render_world(
        os.path.join(gazebo_pkg, "worlds", "warehouse.sdf.xacro"),
        mappings={"with_actors": "false"},
        out_name="warehouse_failclosed.sdf",
    )
    world_name = world_name_of(world_sdf)

    robot = dict(load_fleet()[0])
    derived = gate_parameters(robot)

    actions = [gz_server(world_sdf, headless=headless), clock_bridge()]
    actions += robot_actions(robot, world_name, with_watchdog=with_watchdog)

    def include(package, launch_file, **extra):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(package), "launch", launch_file
                )
            ),
            launch_arguments={"robot": robot["name"], **extra}.items(),
        )

    bsp = include(
        "amr_bsp",
        "bsp.launch.py",
        min_gate_range=f"{derived['min_gate_range']:.4f}",
    )
    # Without the watchdog nothing bridges cmd_vel to cmd_vel_plant, so the gate
    # has to address the plant directly or the robot never moves and the SIGKILL
    # measures nothing. The first attempt at this control did exactly that: peak
    # speed 0.000 m/s, a "result" that says only that the command path was severed.
    gate = include(
        "amr_safety",
        "safety_gate.launch.py",
        suppress_recovery="false",
        cmd_out_topic=CMD_GATED if with_watchdog else CMD_PLANT,
    )

    run = Node(
        package="amr_safety",
        executable="failclosed_run.py",
        name="failclosed_run",
        namespace=robot["name"],
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot["name"],
                "with_watchdog": with_watchdog,
                "results_dir": results_dir,
                "tag": f"failclosed_{tag}",
            }
        ],
    )

    return actions + [
        TimerAction(period=8.0, actions=[bsp]),
        TimerAction(period=11.0, actions=[gate]),
        TimerAction(period=14.0, actions=[run]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=run,
                on_exit=[EmitEvent(event=Shutdown(reason="fail-closed run complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the fail-closed SIGKILL run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("tag", default_value="watchdog"),
            DeclareLaunchArgument(
                "with_watchdog",
                default_value="true",
                description="false runs the control case with no plant-side command "
                "watchdog, which is what shows that the watchdog is doing the work "
                "rather than the simulator being forgiving.",
            ),
            DeclareLaunchArgument(
                "results_dir", default_value=os.path.join(os.getcwd(), "results")
            ),
            OpaqueFunction(function=_setup),
        ]
    )
