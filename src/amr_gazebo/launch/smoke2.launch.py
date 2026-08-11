"""SMOKE TEST 2 - drive the ramp and measure the phantom ground return.

Actors are switched off for this run: a pedestrian crossing the forward scan would be
indistinguishable from the ramp effect being measured.

A rosbag is recorded alongside as an evidence artifact, so the run can be re-examined
without re-running Gazebo.

Run:
    ros2 launch amr_gazebo smoke2.launch.py
    ros2 launch amr_gazebo smoke2.launch.py headless:=false
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_description.fleet_config import load_fleet
from amr_gazebo.spawn import clock_bridge, gz_server, robot_actions
from amr_gazebo.world_builder import render_world, world_name_of


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    duration = float(LaunchConfiguration("duration").perform(context))
    ramp_angle_deg = float(LaunchConfiguration("ramp_angle_deg").perform(context))
    report = LaunchConfiguration("report").perform(context)
    bag_dir = LaunchConfiguration("bag").perform(context)

    upper_plateau_height = 0.50
    crest_x = 6.0
    toe_x = crest_x - upper_plateau_height / math.tan(math.radians(ramp_angle_deg))

    world_sdf = render_world(
        os.path.join(pkg, "worlds", "warehouse.sdf.xacro"),
        mappings={
            "ramp_angle_deg": ramp_angle_deg,
            "upper_plateau_height": upper_plateau_height,
            "with_actors": "false",
        },
        out_name="warehouse_smoke2.sdf",
    )
    world_name = world_name_of(world_sdf)

    # Same robot as the fleet declares, placed on the ramp centre line.
    robot = dict(load_fleet()[0])
    robot["spawn"] = {"x": -2.0, "y": 0.0, "z": 0.15, "yaw": 0.0}
    lidar_x_offset = float(robot["base_length"]) / 2.0 - 0.06

    # Ground truth arrives via the robot's own PosePublisher, bridged in robot_actions.
    actions = [gz_server(world_sdf, headless=headless), clock_bridge()]
    actions += robot_actions(robot, world_name)

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
                # World coordinates: well past the crest at x=6.0, then back down.
                "plateau_x": 9.0,
                "return_x": -1.5,
            }
        ],
    )

    measure = Node(
        package="amr_gazebo",
        executable="smoke2_measure.py",
        name="smoke2_measure",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot["name"],
                "lidar_height": float(robot["lidar_height"]),
                "lidar_x_offset": lidar_x_offset,
                "spawn_x": float(robot["spawn"]["x"]),
                "ramp_angle_deg": ramp_angle_deg,
                "ramp_toe_x": toe_x,
                "ramp_crest_x": crest_x,
                "upper_plateau_height": upper_plateau_height,
                "max_vel_x": float(robot["max_vel_x"]),
                "sample_duration_s": duration,
                "report_path": report,
            }
        ],
    )

    recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            bag_dir,
            f"/{robot['name']}/scan",
            f"/{robot['name']}/imu",
            f"/{robot['name']}/odom",
            "/tf",
            "/tf_static",
            "/clock",
        ],
        output="log",
    )

    return actions + [
        TimerAction(period=8.0, actions=[recorder, measure]),
        TimerAction(period=10.0, actions=[drive]),
    ]


def generate_launch_description():
    """Return the smoke test 2 launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("duration", default_value="90.0"),
            DeclareLaunchArgument(
                "ramp_angle_deg",
                default_value="8.0",
                description="Ramp angle. The world geometry is derived from it.",
            ),
            DeclareLaunchArgument("report", default_value=""),
            DeclareLaunchArgument("bag", default_value="rosbags/smoke2_ramp"),
            OpaqueFunction(function=_setup),
        ]
    )
