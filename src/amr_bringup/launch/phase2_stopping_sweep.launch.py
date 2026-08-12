r"""PHASE 2 EXIT - stopping distance across four commanded speeds.

The stack is deliberately thin: world, robot, SensorBSP, SafetyGate, sweep node.
No SLAM and no Nav2, because a planner between the commanded speed and the wheels
would make the measurement a property of the controller rather than of the gate.
The sweep publishes a constant velocity straight at the gate's input, so the gate
is the only thing that can stop the robot.

``suppress_recovery`` is false here for the same reason: there is no
controller_server in this run to suppress. That behaviour is evidence in its own
right and gets its own A/B run (phase2_safety_run.launch.py), where a full Nav2
stack exists to fire recoveries.

The robot is turned to face a rack face across the aisle rather than driven down
it. A rack is static, square to the approach, and its position comes from the world
file, so the clearance the gate reports can be checked against geometry.

Run:
    pkill -f 'gz sim'
    ros2 launch amr_bringup phase2_stopping_sweep.launch.py
"""

import math
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

from amr_description.fleet_config import load_fleet
from amr_gazebo.spawn import clock_bridge, gz_server, robot_actions
from amr_gazebo.world_builder import render_world, world_name_of
from amr_safety.safety_model import gate_parameters

#: Rack row centres, from warehouse.sdf.xacro. The robot is placed on the aisle
#: centre line at one of them and turned to face it, so the approach is normal to
#: a flat face rather than oblique to a corner.
RACK_X = -1.2

#: Start y. Far enough from the SOUTH rack row (face at y = -2.75) that the retreat
#: between speeds does not run the rear sector into its own braking envelope, and
#: far enough from the NORTH row (face at y = +2.75) to reach 0.6 m/s before the
#: gate has anything to say.
START_Y = -1.0


def _setup(context, *args, **kwargs):
    gazebo_pkg = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    results_dir = LaunchConfiguration("results_dir").perform(context)
    speeds = [
        float(v)
        for v in LaunchConfiguration("speeds").perform(context).split(",")
        if v.strip()
    ]

    world_sdf = render_world(
        os.path.join(gazebo_pkg, "worlds", "warehouse.sdf.xacro"),
        mappings={"with_actors": "false"},
        out_name="warehouse_stopping_sweep.sdf",
    )
    world_name = world_name_of(world_sdf)

    robot = dict(load_fleet()[0])
    robot["spawn"] = {"x": RACK_X, "y": START_Y, "z": 0.15, "yaw": math.pi / 2.0}
    derived = gate_parameters(robot)

    actions = [gz_server(world_sdf, headless=headless), clock_bridge()]
    actions += robot_actions(robot, world_name)

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
    gate = include("amr_safety", "safety_gate.launch.py", suppress_recovery="false")

    sweep = Node(
        package="amr_safety",
        executable="stopping_sweep.py",
        name="stopping_sweep",
        namespace=robot["name"],
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot["name"],
                "speeds": speeds,
                "results_dir": results_dir,
                "tag": "stopping_distance",
            }
        ],
    )

    return actions + [
        TimerAction(period=8.0, actions=[bsp]),
        TimerAction(period=11.0, actions=[gate]),
        TimerAction(period=14.0, actions=[sweep]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=sweep,
                on_exit=[EmitEvent(event=Shutdown(reason="stopping sweep complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the stopping-distance sweep run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument(
                "speeds",
                default_value="0.15,0.30,0.45,0.60",
                description="Commanded speeds, m/s. Four is the stated minimum; the "
                "top of the list is amr1's max_vel_x.",
            ),
            DeclareLaunchArgument(
                "results_dir",
                default_value=os.path.join(os.getcwd(), "results"),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
