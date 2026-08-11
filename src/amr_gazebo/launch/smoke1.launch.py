"""SMOKE TEST 1 - check a Gazebo actor against the LiDAR and a Nav2 obstacle layer.

Brings up a nearly empty world containing one scripted actor, one control box with
ordinary collision geometry, and the first robot from the typed fleet list. A stock
standalone nav2_costmap_2d node provides a genuine Nav2 obstacle layer, and
smoke1_check.py reports the verdict.

Run:
    ros2 launch amr_gazebo smoke1.launch.py
    ros2 launch amr_gazebo smoke1.launch.py headless:=false   # to watch it
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_description.fleet_config import load_fleet
from amr_gazebo.spawn import (
    clock_bridge,
    ground_truth_bridge,
    gz_server,
    robot_actions,
)
from amr_gazebo.world_builder import render_world, world_name_of


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    duration = float(LaunchConfiguration("duration").perform(context))
    report = LaunchConfiguration("report").perform(context)

    world_sdf = render_world(os.path.join(pkg, "worlds", "smoke1_actor.sdf.xacro"))
    world_name = world_name_of(world_sdf)

    # The test characterises the first robot in the fleet; it does not care which.
    # Its warehouse spawn pose is irrelevant here, so place it at the origin facing
    # the actor's path. Everything else about the robot comes from fleet.yaml.
    robot = dict(load_fleet()[0])
    robot["spawn"] = {"x": 0.0, "y": 0.0, "z": 0.15, "yaw": 0.0}

    # Geometry the world file fixes. The checker needs these because Gazebo publishes
    # no pose for actors, so there is no ground-truth stream to compare against.
    actor_x, actor_y_min, actor_y_max = 3.0, -2.0, 2.0
    control_x, control_y, control_size = 3.0, -3.5, 0.4

    actions = [
        gz_server(world_sdf, headless=headless),
        clock_bridge(),
        ground_truth_bridge(world_name),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="witness_camera_bridge",
            output="screen",
            arguments=["/smoke1/witness_camera@sensor_msgs/msg/Image[gz.msgs.Image"],
            parameters=[{"use_sim_time": True}],
        ),
    ]
    actions += robot_actions(robot, world_name)

    costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        namespace="costmap",
        output="screen",
        parameters=[os.path.join(pkg, "config", "smoke_costmap.yaml")],
    )

    # The standalone costmap is a lifecycle node: without a manager to drive it
    # through configure and activate it comes up inert and publishes nothing, which
    # reads exactly like "the obstacle layer never saw the obstacle".
    costmap_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["costmap/costmap"],
                # The standalone costmap does not create a bond, so bond supervision
                # would tear down a node that is running perfectly well.
                "bond_timeout": 0.0,
            }
        ],
    )

    checker = Node(
        package="amr_gazebo",
        executable="smoke1_check.py",
        name="smoke1_check",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot["name"],
                "robot_x": float(robot["spawn"]["x"]),
                "robot_y": float(robot["spawn"]["y"]),
                "robot_yaw": float(robot["spawn"]["yaw"]),
                "lidar_height": float(robot["lidar_height"]),
                # Mirrors the laser joint origin in amr.urdf.xacro.
                "lidar_x_offset": float(robot["base_length"]) / 2.0 - 0.06,
                "actor_x": actor_x,
                "actor_y_min": actor_y_min,
                "actor_y_max": actor_y_max,
                "control_x": control_x,
                "control_y": control_y,
                "control_size": control_size,
                "sample_duration_s": duration,
                "report_path": report,
            }
        ],
    )

    # Gazebo needs to be up and the robot spawned before the costmap starts looking
    # for transforms, and the checker needs the costmap publishing.
    return actions + [
        TimerAction(period=6.0, actions=[costmap]),
        TimerAction(period=9.0, actions=[costmap_manager]),
        TimerAction(period=14.0, actions=[checker]),
    ]


def generate_launch_description():
    """Return the smoke test 1 launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run Gazebo without the GUI.",
            ),
            DeclareLaunchArgument(
                "duration",
                default_value="30.0",
                description="Seconds of scan data to characterise.",
            ),
            DeclareLaunchArgument(
                "report",
                default_value="",
                description="Optional path to write the result markdown to.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
