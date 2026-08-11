"""Bring up the warehouse world and spawn the whole fleet from the typed robot list.

This launch file never names a robot. It loops over
``amr_description/config/fleet.yaml`` and applies the same composition to every
entry, so a third robot is three lines of YAML rather than a code change. That
property is what "configuration-driven fleet scaling" means here, and it is
checked by tests/test_fleet_parameterization.py.

Run:
    ros2 launch amr_gazebo warehouse.launch.py
    ros2 launch amr_gazebo warehouse.launch.py headless:=false
    ros2 launch amr_gazebo warehouse.launch.py ramp_angle_deg:=12.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from amr_description.fleet_config import load_fleet
from amr_gazebo.spawn import clock_bridge, gz_server, robot_actions
from amr_gazebo.world_builder import render_world, world_name_of


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    ramp_angle_deg = float(LaunchConfiguration("ramp_angle_deg").perform(context))
    with_actors = LaunchConfiguration("with_actors").perform(context)

    world_sdf = render_world(
        os.path.join(pkg, "worlds", "warehouse.sdf.xacro"),
        mappings={
            "ramp_angle_deg": ramp_angle_deg,
            "with_actors": with_actors,
        },
    )
    world_name = world_name_of(world_sdf)

    actions = [gz_server(world_sdf, headless=headless), clock_bridge()]

    # The only fleet-aware line in the file.
    for robot in load_fleet():
        actions += robot_actions(robot, world_name)

    return actions


def generate_launch_description():
    """Return the warehouse + fleet launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument(
                "ramp_angle_deg",
                default_value="8.0",
                description="Ramp angle in degrees. World geometry is derived from it.",
            ),
            DeclareLaunchArgument("with_actors", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
