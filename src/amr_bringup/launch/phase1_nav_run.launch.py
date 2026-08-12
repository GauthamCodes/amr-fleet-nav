r"""Phase 1 deliverable: one instrumented goal-to-goal navigation run.

One command, start to report:

    ./ws.sh ros2 launch amr_bringup phase1_nav_run.launch.py tag:=baseline \\
        with_actors:=false
    ./ws.sh ros2 launch amr_bringup phase1_nav_run.launch.py tag:=actors \\
        with_actors:=true

The two invocations are the A/B: the same goal, the same stack, the same
parameters, differing only in whether pedestrians are walking the aisle. Metrics
that move between them moved because of the dynamic obstacle and nothing else.

The run saves its own map on the way out. That is not a duplicate of the survey
artifact - it is the measurement of what pedestrians do to a map built while they
walk through it, which is only meaningful against the survey's clean one.
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

#: When the mission node starts, in seconds after launch. Nav2 is staged at +22 s
#: and its lifecycle manager needs a few seconds after that; the node waits for the
#: action server, for TF and for its own settle time regardless, so this only has
#: to be late enough not to waste sim time waiting.
MISSION_START_S = 26.0


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    tag = LaunchConfiguration("tag").perform(context)
    results_dir = LaunchConfiguration("results_dir").perform(context)

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
        }.items(),
    )

    mission = Node(
        package="amr_navigation",
        executable="nav_goal_run.py",
        name="nav_goal_run",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot_name,
                "tag": tag,
                "results_dir": results_dir,
                "goal_x": LaunchConfiguration("goal_x"),
                "goal_y": LaunchConfiguration("goal_y"),
                "goal_yaw": LaunchConfiguration("goal_yaw"),
                "spawn_x": x,
                "spawn_y": y,
                "spawn_yaw": yaw,
            }
        ],
    )

    save = save_map_process(robot_name, os.path.join(results_dir, f"phase1_map_{tag}"))

    return [
        stack,
        TimerAction(period=MISSION_START_S, actions=[mission]),
        RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=[save])),
        RegisterEventHandler(
            OnProcessExit(
                target_action=save,
                on_exit=[EmitEvent(event=Shutdown(reason="nav run complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the Phase 1 instrumented navigation run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("with_actors", default_value="true"),
            DeclareLaunchArgument("tag", default_value="run"),
            DeclareLaunchArgument(
                "results_dir", default_value=os.path.join(os.getcwd(), "results")
            ),
            # World coordinates: down the aisle from (-11, -1.5) and across it, so
            # the plan is 11 m long and has to cross the lane the aisle pedestrian
            # walks. Three constraints put it here rather than further east:
            #   x < 0.96  - clear of the ramp slab's buried plan-view footprint,
            #               inside which a dynamic return would be scored static
            #   |x - 0.5| - a metre clear of pedestrian_2's crossing line, so the
            #               goal cell is never occupied when the robot arrives
            #   y = 1.5   - 1.25 m off the rack faces at y = 2.75
            DeclareLaunchArgument("goal_x", default_value="-0.5"),
            DeclareLaunchArgument("goal_y", default_value="1.5"),
            DeclareLaunchArgument("goal_yaw", default_value="0.0"),
            OpaqueFunction(function=_setup),
        ]
    )
