r"""PHASE 2 CARRY-OVER - recovery suppression, with a halt long enough to prove it.

WHAT PHASE 2 COULD AND COULD NOT SHOW

    Phase 2's A/B established that the mechanism OPERATES end to end: on halt entry
    the SafetyGate writes controller_server's
    ``progress_checker.movement_time_allowance``, the run reads it back changed, and
    it is restored once the robot has moved again. What it could NOT show is that the
    mechanism was NEEDED. The longest halt in either arm was 0.80 s against a 10 s
    allowance, so the progress checker was never going to fire during one, and the
    control run's zero recovery count proved nothing. That was recorded honestly and
    carried forward rather than dressed up.

    The blocker was the obstacle, not the instrumentation: this world's pedestrians
    walk, and they clear the forward sector in under a second.

WHAT THIS RUN CHANGES

    One immovable barrier, spanning the full 5.5 m aisle, parked between the robot
    and its goal. The gate halts on it and the halt does not end - the gate zeroes
    the ENTIRE twist while blocked, so the robot cannot rotate away, clearance cannot
    grow, and nothing releases the latch. That is a halt of tens of seconds against a
    10 s allowance.

    Expected, and what makes the A/B worth running:

      suppressed   allowance raised to 1e6 s on halt entry -> 0 recoveries during
                   the halt, robot holds position safely and indefinitely
      control      allowance left at 10 s -> the progress checker fires, Nav2
                   dispatches Spin, the gate zeroes that too, and recoveries cycle

    The control arm is expected to end with the goal ABORTED. That is the correct
    outcome and not a failure of the run: with the aisle fully blocked there is no
    path, and the point being measured is what Nav2 does DURING the halt.

A LIMITATION THIS RUN MAKES VISIBLE, AND WHICH BELONGS IN THE README

    A full-twist hold plus an obstacle that never moves is a deadlock: the robot has
    no legal action that could increase its clearance. Allowing in-place rotation
    under a hold is already flagged as a Phase 7 refinement; this run is the evidence
    for why it is worth doing.

Run BOTH arms, cleaning the process table between them:
    ./scripts/clean_processes.sh
    ./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py tag:=suppressed
    ./scripts/clean_processes.sh
    ./ws.sh ros2 launch amr_bringup phase2_recovery_ab.launch.py \
        tag:=control suppress_recovery:=false
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

#: When the mission node starts. Nav2 is staged at +22 s and the gate at +23 s.
MISSION_START_S = 27.0

#: Where the barrier sits, between amr1's spawn at x = -11 and its goal at x = -0.5.
OBSTACLE_X = -5.0

#: Long enough that the halt dwarfs the 10 s allowance under test, short enough not
#: to spend minutes of wall clock watching a robot hold still on purpose.
RUN_TIMEOUT_S = 100.0


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
            # No pedestrians. The barrier is the obstacle under test, and an actor
            # wandering into the sector would make it ambiguous which one the gate
            # halted on.
            "with_actors": "false",
            "with_static_obstacle": "true",
            "obstacle_x": str(OBSTACLE_X),
            "obstacle_y": "0.0",
            "obstacle_size_y": "5.5",
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
                "tag": f"recovery_ab_{tag}",
                "title": "recovery suppression against a static barrier",
                "results_dir": results_dir,
                "suppress_recovery": suppress.lower() != "false",
                "goal_x": LaunchConfiguration("goal_x"),
                "goal_y": LaunchConfiguration("goal_y"),
                "spawn_x": x,
                "spawn_y": y,
                "spawn_yaw": yaw,
                "timeout_s": RUN_TIMEOUT_S,
            }
        ],
    )

    return [
        stack,
        TimerAction(period=MISSION_START_S, actions=[mission]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=mission,
                on_exit=[EmitEvent(event=Shutdown(reason="recovery A/B complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the recovery-suppression A/B against a static barrier."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("tag", default_value="suppressed"),
            DeclareLaunchArgument("suppress_recovery", default_value="true"),
            DeclareLaunchArgument(
                "results_dir", default_value=os.path.join(os.getcwd(), "results")
            ),
            # The Phase 2 goal, unchanged, so the two runs are comparable.
            DeclareLaunchArgument("goal_x", default_value="-0.5"),
            DeclareLaunchArgument("goal_y", default_value="1.5"),
            OpaqueFunction(function=_setup),
        ]
    )
