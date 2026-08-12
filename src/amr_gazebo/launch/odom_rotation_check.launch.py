"""Spin amr1 in place and compare wheel odometry against Gazebo ground truth.

Diagnostic for the drive-axle offset described in odom_rotation_check.py. Actors are
off and the robot sits on open floor, so nothing but the rotation is measured.

Run:
    ros2 launch amr_gazebo odom_rotation_check.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_description.fleet_config import load_fleet
from amr_gazebo.spawn import clock_bridge, gz_server, robot_actions
from amr_gazebo.world_builder import render_world, world_name_of


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    report = LaunchConfiguration("report").perform(context)

    world_sdf = render_world(
        os.path.join(pkg, "worlds", "warehouse.sdf.xacro"),
        mappings={"with_actors": "false"},
        out_name="warehouse_rotcheck.sdf",
    )
    world_name = world_name_of(world_sdf)

    robot = dict(load_fleet()[0])
    # Open floor, well clear of the racks, so a full turn touches nothing.
    robot["spawn"] = {"x": -6.0, "y": 0.0, "z": 0.15, "yaw": 0.0}

    # Where the URDF says the robot pivots, relative to base_footprint. Since Phase 1
    # moved the frame origin ONTO the drive axle, this is now zero by construction -
    # and the test's job is to confirm the simulator agrees.
    expected_offset = 0.0

    actions = [gz_server(world_sdf, headless=headless), clock_bridge()]
    actions += robot_actions(robot, world_name)

    check = Node(
        package="amr_gazebo",
        executable="odom_rotation_check.py",
        name="odom_rotation_check",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot["name"],
                "yaw_rate": 0.6,
                "settle_s": 4.0,
                "spin_s": 22.0,
                "expected_offset": expected_offset,
                "tolerance": 0.02,
                "map_resolution": 0.05,
                "report_path": report,
            }
        ],
    )

    return actions + [TimerAction(period=8.0, actions=[check])]


def generate_launch_description():
    """Return the odometry rotation check launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("report", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
