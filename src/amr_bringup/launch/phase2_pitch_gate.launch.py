r"""PHASE 2 EXIT - PitchGate before/after evidence, driven up the real ramp.

Deliberately NOT the full stack. No SLAM, no Nav2, no SafetyGate: this run measures
one thing, and every other consumer of the scan is noise in the measurement (and CPU
taken from the simulator, which buys scans). The robot drives the ramp under the same
smoke2_drive actuator Phase 0 used, so the trajectory is the one the sizing
measurement was taken on.

Actors are off. A pedestrian crossing the forward scan would be indistinguishable
from the ramp effect being measured - the same reason smoke test 2 disables them.

Run:
    pkill -f 'gz sim'
    ros2 launch amr_bringup phase2_pitch_gate.launch.py
    ros2 run amr_bsp plot_pitch_gate.py
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

from amr_description.fleet_config import load_fleet
from amr_gazebo.spawn import clock_bridge, gz_server, robot_actions
from amr_gazebo.world_builder import render_world, world_name_of
from amr_safety.safety_model import gate_parameters

#: Seconds before the drive starts. The BSP must be up and the pitch buffer primed
#: before the robot reaches the toe, or the first pitched scans go through untruncated.
DRIVE_START_S = 12.0


def _setup(context, *args, **kwargs):
    gazebo_pkg = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    duration = float(LaunchConfiguration("duration").perform(context))
    ramp_angle_deg = float(LaunchConfiguration("ramp_angle_deg").perform(context))
    results_dir = LaunchConfiguration("results_dir").perform(context)

    world_sdf = render_world(
        os.path.join(gazebo_pkg, "worlds", "warehouse.sdf.xacro"),
        mappings={
            "ramp_angle_deg": ramp_angle_deg,
            "upper_plateau_height": 0.50,
            "with_actors": "false",
        },
        out_name="warehouse_pitch_gate.sdf",
    )
    world_name = world_name_of(world_sdf)

    # Same fleet entry, placed on the ramp centre line exactly as smoke test 2 did.
    robot = dict(load_fleet()[0])
    robot["spawn"] = {"x": -2.0, "y": 0.0, "z": 0.15, "yaw": 0.0}
    derived = gate_parameters(robot)

    actions = [gz_server(world_sdf, headless=headless), clock_bridge()]
    actions += robot_actions(robot, world_name)

    bsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_bsp"), "launch", "bsp.launch.py"
            )
        ),
        launch_arguments={
            "robot": robot["name"],
            "min_gate_range": f"{derived['min_gate_range']:.4f}",
        }.items(),
    )

    drive = Node(
        package="amr_gazebo",
        executable="smoke2_drive.py",
        name="smoke2_drive",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot["name"],
                "cruise_speed": 0.35,
                "spawn_x": float(robot["spawn"]["x"]),
                "plateau_x": 9.0,
                "return_x": -1.5,
            }
        ],
    )

    probe = Node(
        package="amr_bsp",
        executable="pitch_gate_probe.py",
        name="pitch_gate_probe",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot["name"],
                "lidar_height": float(robot["lidar_height"]),
                "gate_margin": 0.9,
                "min_gate_range": derived["min_gate_range"],
                "sample_duration_s": duration,
                "results_dir": results_dir,
                "tag": "pitch_gate",
            }
        ],
    )

    return actions + [
        TimerAction(period=8.0, actions=[bsp]),
        TimerAction(period=10.0, actions=[probe]),
        TimerAction(period=DRIVE_START_S, actions=[drive]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=probe,
                on_exit=[EmitEvent(event=Shutdown(reason="pitch gate run complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the PitchGate evidence run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("duration", default_value="90.0"),
            DeclareLaunchArgument(
                "ramp_angle_deg",
                default_value="8.0",
                description="Ramp angle. The world geometry is derived from it, and "
                "so is the phantom return this run exists to remove.",
            ),
            DeclareLaunchArgument(
                "results_dir",
                default_value=os.path.join(os.getcwd(), "results"),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
