"""Phase 1 deliverable: drive the survey circuit and save the SLAM map.

One command, start to artifact:

    ./ws.sh ros2 launch amr_bringup phase1_survey.launch.py

It brings up the world, the robot and slam_toolbox, drives the survey circuit,
calls slam_toolbox's map saver, and shuts itself down. Nav2 is deliberately absent
- this run exists to produce the map Nav2 would need, and MPPI at 20 Hz would take
CPU away from the simulator for no benefit.

The default is ``with_actors:=false``. Phase 1's slam_toolbox has no dynamic-object
rejection, so pedestrians walking through the survey would be traced into the map
as smears that later scans only partly clear. That is a real limitation and it is
stated rather than hidden: the goal-to-goal runs keep the pedestrians and save
their own map, which is what makes the contamination measurable instead of
theoretical.
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

from amr_description.fleet_config import load_fleet, spawn_pose
from amr_navigation.map_artifact import save_map_process

#: When the survey node starts, in seconds after launch. The stack stages the
#: robot at +5 s and slam_toolbox at +14 s (see amr1_nav.launch.py); starting the
#: drive before SLAM is active would throw away the first scans, which are the
#: stationary ones the map is anchored on.
SURVEY_START_S = 22.0


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    map_path = LaunchConfiguration("map_path").perform(context)
    laps = LaunchConfiguration("laps").perform(context)

    robot = {r["name"]: r for r in load_fleet()}[robot_name]
    x, y, _, yaw = spawn_pose(robot)

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_bringup"),
                "launch",
                "amr1_nav.launch.py",
            )
        ),
        launch_arguments={
            "robot": robot_name,
            "headless": LaunchConfiguration("headless"),
            "with_actors": LaunchConfiguration("with_actors"),
            "with_nav2": "false",
        }.items(),
    )

    survey = Node(
        package="amr_navigation",
        executable="survey_drive.py",
        name="survey_drive",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot_name,
                "laps": int(laps),
                # The route is written in world coordinates; odometry starts at the
                # spawn pose, so the node is told where that is rather than
                # discovering it. One robot property, from one place.
                "spawn_x": x,
                "spawn_y": y,
                "spawn_yaw": yaw,
            }
        ],
    )

    save = save_map_process(robot_name, map_path)

    return [
        stack,
        TimerAction(period=SURVEY_START_S, actions=[survey]),
        RegisterEventHandler(OnProcessExit(target_action=survey, on_exit=[save])),
        RegisterEventHandler(
            OnProcessExit(
                target_action=save,
                on_exit=[EmitEvent(event=Shutdown(reason="survey complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the Phase 1 survey-and-save launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("with_actors", default_value="false"),
            DeclareLaunchArgument("laps", default_value="2"),
            DeclareLaunchArgument(
                "map_path",
                default_value=os.path.join(os.getcwd(), "results", "phase1_map"),
                description="Map artifact path without extension; .pgm and .yaml "
                "are written beside each other.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
