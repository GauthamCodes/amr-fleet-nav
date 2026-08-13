r"""PHASE 5 EXIT - payload-adaptive velocity and jerk (assignment section 3.1).

Brings the whole fleet up with the motion chain in place and drives a velocity
STEP into the chain's input, for both robots, unloaded then loaded. Writes
results/phase5_payload_trace.{md,csv,png}.

    ./scripts/clean_processes.sh
    ./ws.sh ros2 launch amr_bringup phase5_payload_trace.launch.py

No goals are dispatched. Nav2 is up because the stock nav2_velocity_smoother is
one of its lifecycle servers, but a Nav2 goal would defeat the measurement: the
controller already emits a smooth reference, so the smoother and the jerk
limiter would both have almost nothing to do and the three traces would sit on
top of each other. The step is what separates the links.

Actors are off. A pedestrian walking into a lane would be stopped by SafetyGate,
which is downstream of everything being measured, and the deceleration in the
trace would then be the gate's rather than the chain's.
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

#: The last robot's stack is up at ~27 s in fleet_nav.launch.py; the smoother is
#: a lifecycle node behind that, so leave it room to activate.
TRACE_START_S = 34.0


def _setup(context, *args, **kwargs):
    results_dir = LaunchConfiguration("results_dir").perform(context)

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
            "with_actors": "false",
            "with_motion_chain": LaunchConfiguration("with_motion_chain"),
        }.items(),
    )

    trace = Node(
        package="amr_motion",
        executable="payload_trace",
        name="payload_trace",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "results_dir": results_dir,
                "tag": LaunchConfiguration("tag"),
                "target_v": LaunchConfiguration("target_v"),
            }
        ],
    )

    return [
        stack,
        TimerAction(period=TRACE_START_S, actions=[trace]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=trace,
                on_exit=[EmitEvent(event=Shutdown(reason="payload trace complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the payload-adaptive motion trace."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("tag", default_value="payload_trace"),
            DeclareLaunchArgument(
                "with_motion_chain",
                default_value="true",
                description="false traces the same step with the chain removed, "
                "which is the Phase 1/2 wiring and shows what the chain adds.",
            ),
            DeclareLaunchArgument(
                "target_v",
                default_value="0.5",
                description="Commanded step, m/s. Chosen below amr1's 0.60 limit "
                "so both robots are commanded the same speed and the difference "
                "between them is their limits, not their ceilings.",
            ),
            DeclareLaunchArgument(
                "results_dir", default_value=os.path.join(os.getcwd(), "results")
            ),
            OpaqueFunction(function=_setup),
        ]
    )
