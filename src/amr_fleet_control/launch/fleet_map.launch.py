"""Bring up FleetMapNode: one composited map, one fleet frame, for the whole fleet.

Deliberately UNNAMESPACED. There is one fleet map, not one per robot, and the frame
it publishes - ``fleet_map`` - is the frame both robots' global costmaps plan in. A
namespaced copy per robot would give each one its own private "fleet" map, which is
the thing this node exists to stop.

Start it BEFORE Nav2. It publishes the static ``fleet_map -> amrN/map`` transforms and
a correctly sized, all-unknown map in its constructor, and Nav2's global costmap
blocks on both during activation - silently, with no timeout on the lifecycle
manager's side. See the node's module docstring.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    config = os.path.join(
        get_package_share_directory("amr_fleet_control"), "config", "fleet_map.yaml"
    )
    decision_log = LaunchConfiguration("decision_log").perform(context)

    return [
        Node(
            package="amr_fleet_control",
            executable="fleet_map_node",
            name="fleet_map_node",
            output="screen",
            parameters=[config, {"decision_log": decision_log}],
        )
    ]


def generate_launch_description():
    """Return the fleet map node."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "decision_log",
                default_value="",
                description="Path stem for the selective-update accept/defer "
                "evidence. Empty writes nothing, which is right for an ordinary "
                "bringup; the evidence run points it at results/.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
