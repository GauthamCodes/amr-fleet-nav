r"""PHASE 2 EXIT - safety halt on a pedestrian encounter, run as an A/B.

The full stack, the same goal Phase 1 measured, and pedestrians walking the aisle.
Invoked twice with everything identical except the one parameter under test:

    ros2 launch amr_bringup phase2_safety_run.launch.py tag:=suppressed
    ros2 launch amr_bringup phase2_safety_run.launch.py tag:=control \
        suppress_recovery:=false

The second is not optional. "No recovery fired during the halt" is a claim about a
mechanism, and a mechanism that is never exercised produces the same zero as one
that works. The control run is what tells the two apart - and because
behavior_server publishes cmd_vel of its own, a recovery firing during a halt is a
second writer contending for a robot the gate has stopped, not just a wasted spin.

Actors are ON here, unlike every other Phase 2 run: the halt this measures is a halt
for a moving obstacle, which is the case a static rack cannot produce.
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

#: When the mission node starts. Nav2 is staged at +22 s and the gate at +23 s;
#: the node waits for the action server regardless, so this only has to be late
#: enough not to spend sim time waiting.
MISSION_START_S = 27.0


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    tag = LaunchConfiguration("tag").perform(context)
    results_dir = LaunchConfiguration("results_dir").perform(context)
    suppress = LaunchConfiguration("suppress_recovery").perform(context)

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
            "with_actors": "true",
            "suppress_recovery": suppress,
        }.items(),
    )

    mission = Node(
        package="amr_safety",
        executable="safety_run.py",
        name="safety_run",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot_name,
                "tag": f"safety_{tag}",
                "results_dir": results_dir,
                "suppress_recovery": suppress.lower() != "false",
                "goal_x": LaunchConfiguration("goal_x"),
                "goal_y": LaunchConfiguration("goal_y"),
                "spawn_x": x,
                "spawn_y": y,
                "spawn_yaw": yaw,
            }
        ],
    )

    return [
        stack,
        TimerAction(period=MISSION_START_S, actions=[mission]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=mission,
                on_exit=[EmitEvent(event=Shutdown(reason="safety run complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the instrumented safety-halt run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("tag", default_value="suppressed"),
            DeclareLaunchArgument("suppress_recovery", default_value="true"),
            DeclareLaunchArgument(
                "results_dir", default_value=os.path.join(os.getcwd(), "results")
            ),
            # The Phase 1 goal, unchanged, so the two phases are comparable: down
            # the aisle from (-11, -1.5) and across it, crossing the lane the aisle
            # pedestrian walks.
            DeclareLaunchArgument("goal_x", default_value="-0.5"),
            DeclareLaunchArgument("goal_y", default_value="1.5"),
            OpaqueFunction(function=_setup),
        ]
    )
