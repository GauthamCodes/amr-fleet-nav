"""Phase 1 system composition: warehouse + one robot + slam_toolbox + Nav2.

This file composes, it does not configure. Every parameter comes from
amr_navigation/config via amr_navigation.params, and every robot property comes from
amr_description/config/fleet.yaml. The robot is named exactly once, as a launch
argument, so the same file brings up amr2 by changing that argument.

Run:
    ros2 launch amr_bringup amr1_nav.launch.py
    ros2 launch amr_bringup amr1_nav.launch.py headless:=false
    ros2 launch amr_bringup amr1_nav.launch.py robot:=amr2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from amr_description.fleet_config import load_fleet
from amr_gazebo.spawn import clock_bridge, gz_server, robot_actions
from amr_gazebo.world_builder import render_world, world_name_of


def _setup(context, *args, **kwargs):
    gazebo_pkg = get_package_share_directory("amr_gazebo")
    nav_pkg = get_package_share_directory("amr_navigation")

    robot_name = LaunchConfiguration("robot").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    with_actors = LaunchConfiguration("with_actors").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    with_nav2 = LaunchConfiguration("with_nav2").perform(context).lower() != "false"

    fleet = {r["name"]: r for r in load_fleet()}
    if robot_name not in fleet:
        raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
    robot = fleet[robot_name]

    world_sdf = render_world(
        os.path.join(gazebo_pkg, "worlds", "warehouse.sdf.xacro"),
        mappings={"with_actors": with_actors},
        out_name="warehouse_nav.sdf",
    )
    world_name = world_name_of(world_sdf)

    # Staged deliberately. This launch starts Gazebo, a robot, SLAM and six Nav2
    # servers; starting them together starves Gazebo's own initialisation, and the
    # spawn request then sits waiting for /world/<name>/create forever. The robot
    # never enters the world, no DiffDrive means no odom frame, and the only symptom
    # is Nav2 reporting that amr1/odom "does not exist" - which reads like a frame
    # naming bug rather than a startup race.
    simulation = [
        gz_server(world_sdf, headless=headless),
        clock_bridge(),
        TimerAction(period=5.0, actions=robot_actions(robot, world_name)),
    ]

    def include(package, launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(package), "launch", launch_file
                )
            ),
            launch_arguments={
                "robot": robot_name,
                "scan_topic": scan_topic,
            }.items(),
        )

    _ = nav_pkg
    staged = [
        # SLAM first: Nav2's global costmap needs the map->odom transform and the
        # /map topic before its static layer can configure.
        TimerAction(period=14.0, actions=[include("amr_navigation", "slam.launch.py")]),
    ]
    # The survey run maps without navigating, and MPPI evaluates 1500 trajectories
    # at 20 Hz whether or not a goal exists. Leaving Nav2 out of that run returns
    # the CPU to the simulator, where it buys real-time factor and therefore scans.
    if with_nav2:
        staged.append(
            TimerAction(
                period=22.0, actions=[include("amr_navigation", "nav2.launch.py")]
            )
        )
    return simulation + staged


def generate_launch_description():
    """Return the Phase 1 single-robot navigation stack."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("with_actors", default_value="true"),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument(
                "with_nav2",
                default_value="true",
                description="Bring up the Nav2 servers. false gives world + robot "
                "+ SLAM only, which is what the mapping survey needs.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
