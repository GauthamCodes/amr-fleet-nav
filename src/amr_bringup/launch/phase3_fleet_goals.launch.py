r"""PHASE 3 EXIT - both robots under way at once, on one cooperative map.

Two deliverables from one run, because they are two views of the same thing:

    results/phase3_concurrent_goals.*     both stacks dispatched simultaneously,
                                          each reaching its goal while the other is
                                          driving, and how close they came
    results/phase3_selective_updates.*    every candidate map update either robot
                                          produced, the four score terms behind it,
                                          and whether it was merged or deferred

The selective-update evidence has to come from a run where both robots are actually
exploring: a stationary fleet produces nothing but deferrals, which would make the
policy look effective while proving nothing about it.

THE ROUTES ARE DECONFLICTED, DELIBERATELY

    amr1 runs the south lane and amr2 the north lane of the same aisle, 3 m apart.
    They are close enough to sit inside each other's obstacle layers the whole way -
    each robot IS a lethal obstacle to the other, which is the interaction being
    demonstrated - without being forced into a conflict neither can resolve locally.

    The narrow-intersection conflict, mutual local deviation through
    FleetTrajectoryLayer, and escalation to the yield protocol are Phases 6 and 7.
    This run establishes the thing they all rest on: two full stacks, two SLAM maps,
    one composited map that both of them plan in, all at once.

Run:
    ./scripts/clean_processes.sh
    ./ws.sh ros2 launch amr_bringup phase3_fleet_goals.launch.py
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

#: When the mission node starts. The last robot's gate is up at 27 s in
#: fleet_nav.launch.py; the node waits for both action servers regardless, so this
#: only has to be late enough not to spend sim time waiting.
MISSION_START_S = 32.0

#: One (x, y, yaw) per robot, in fleet.yaml declaration order, expressed in the
#: fleet frame - which IS the Gazebo world frame, so these read directly against
#: world.yaml. amr1 spawns at (-11, -1.5) and amr2 at (-11, +1.5); each drives the
#: length of the aisle in its own lane.
DEFAULT_GOALS = [-0.5, -1.5, 0.0, -0.5, 1.5, 0.0]


def _setup(context, *args, **kwargs):
    results_dir = LaunchConfiguration("results_dir").perform(context)
    tag = LaunchConfiguration("tag").perform(context)

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_bringup"),
                "launch",
                "fleet_nav.launch.py",
            )
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "with_actors": LaunchConfiguration("with_actors"),
            # The selective-update report is written by FleetMapNode itself, so the
            # path is threaded down to it rather than collected by the mission node.
            "decision_log": os.path.join(results_dir, "phase3_selective_updates"),
        }.items(),
    )

    mission = Node(
        package="amr_fleet_control",
        executable="fleet_mission",
        name="fleet_mission",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "goals": LaunchConfiguration("goals"),
                "results_dir": results_dir,
                "tag": tag,
                "timeout_s": 200.0,
            }
        ],
    )

    return [
        stack,
        TimerAction(period=MISSION_START_S, actions=[mission]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=mission,
                on_exit=[EmitEvent(event=Shutdown(reason="fleet goals complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the concurrent two-robot goal run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("with_actors", default_value="false"),
            DeclareLaunchArgument("tag", default_value="concurrent_goals"),
            DeclareLaunchArgument(
                "results_dir", default_value=os.path.join(os.getcwd(), "results")
            ),
            DeclareLaunchArgument(
                "goals",
                default_value=str(DEFAULT_GOALS),
                description="Flat x, y, yaw per robot in fleet.yaml order, in the "
                "fleet frame.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
