"""System composition: warehouse + ONE robot + its full software stack.

This is deliberately still a single-robot launch. Phase 3 added
``fleet_nav.launch.py`` for two, and the tempting simplification - making this file a
one-robot special case of that one - would silently turn every Phase 1 and Phase 2
evidence run into a two-robot run, because ``phase1_survey``, ``phase1_nav_run`` and
``phase2_safety_run`` all include this file by name. Those runs' numbers are sensitive
to graph contention (SESSION_LOG Phase 2, surprise 1), so the contamination would not
raise an error, it would just move the measurements.

Instead both launches call :func:`amr_gazebo.spawn.world_actions` for the world
singletons and include ``robot_stack.launch.py`` per robot. Neither includes the
other. The staging here is unchanged from Phase 1/2: BSP at 10 s, SLAM at 14 s, Nav2
at 22 s, gate at 23 s.

Run:
    ros2 launch amr_bringup amr1_nav.launch.py
    ros2 launch amr_bringup amr1_nav.launch.py headless:=false
    ros2 launch amr_bringup amr1_nav.launch.py robot:=amr2
    ros2 launch amr_bringup amr1_nav.launch.py suppress_recovery:=false
    ros2 launch amr_bringup amr1_nav.launch.py with_static_obstacle:=true
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
from amr_gazebo.spawn import robot_actions, world_actions
from amr_navigation.params import SCAN_TOPIC

#: When the robot is spawned. Gazebo needs a head start: starting everything together
#: starves its initialisation and the spawn request then sits waiting for
#: /world/<name>/create forever. The robot never enters the world, no DiffDrive means
#: no odom frame, and the only symptom is Nav2 reporting that amr1/odom "does not
#: exist" - which reads like a frame naming bug rather than a startup race.
SPAWN_START_S = 5.0

#: The fleet map is not optional here even though this launch runs ONE robot. From
#: Phase 3 the global costmap plans in the fleet_map frame, and Costmap2DROS blocks
#: on activation until fleet_map -> amrN/base_footprint exists. Without this node a
#: single-robot run does not fail with a missing-frame error; it hangs through
#: initial_transform_timeout and then aborts the whole bringup.
#:
#: FleetMapNode loops over the whole fleet regardless, so it also publishes the
#: absent robot's static frame. That is harmless - its half of the map simply stays
#: unknown - and it is what keeps this file free of any robot-count special case.
FLEET_MAP_START_S = 16.0


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"

    fleet = {r["name"]: r for r in load_fleet()}
    if robot_name not in fleet:
        raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
    robot = fleet[robot_name]

    world, world_name, _ = world_actions(
        mappings={
            "with_actors": LaunchConfiguration("with_actors").perform(context),
            "with_static_obstacle": LaunchConfiguration("with_static_obstacle").perform(
                context
            ),
            "obstacle_x": LaunchConfiguration("obstacle_x").perform(context),
            "obstacle_y": LaunchConfiguration("obstacle_y").perform(context),
            "obstacle_size_y": LaunchConfiguration("obstacle_size_y").perform(context),
        },
        headless=headless,
        out_name="warehouse_nav.sdf",
    )

    with_nav2 = LaunchConfiguration("with_nav2").perform(context)

    def include(package, launch_file, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(package), "launch", launch_file
                )
            ),
            launch_arguments=arguments.items(),
        )

    stack = include(
        "amr_bringup",
        "robot_stack.launch.py",
        {
            "robot": robot_name,
            "scan_topic": LaunchConfiguration("scan_topic").perform(context),
            "with_nav2": with_nav2,
            "suppress_recovery": LaunchConfiguration("suppress_recovery").perform(
                context
            ),
            "stagger": "0.0",
        },
    )

    fleet_map = TimerAction(
        period=FLEET_MAP_START_S,
        actions=[
            include(
                "amr_fleet_control",
                "fleet_map.launch.py",
                {"decision_log": LaunchConfiguration("decision_log").perform(context)},
            ),
            include(
                "amr_navigation",
                "costmap_filters.launch.py",
                # A filter with no costmap to filter is three lifecycle nodes
                # publishing to nobody, so the survey configuration skips them.
                {"enabled": "false" if with_nav2.lower() == "false" else "true"},
            ),
        ],
    )

    return world + [
        TimerAction(period=SPAWN_START_S, actions=robot_actions(robot, world_name)),
        fleet_map,
        stack,
    ]


def generate_launch_description():
    """Return the single-robot navigation stack."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("with_actors", default_value="true"),
            # Imported, not restated. Phase 1 declared the literal "scan" here as
            # well as in amr_navigation.params, so flipping the seam in one place
            # would have left this launch - the one every deliverable run goes
            # through - still pointing at the raw sensor.
            DeclareLaunchArgument("scan_topic", default_value=SCAN_TOPIC),
            DeclareLaunchArgument(
                "with_nav2",
                default_value="true",
                description="Bring up the Nav2 servers. false gives world + robot "
                "+ SLAM only, which is what the mapping survey needs.",
            ),
            DeclareLaunchArgument(
                "suppress_recovery",
                default_value="true",
                description="Set false for the control run that shows Nav2 DOES "
                "fire recovery behaviours during a halt when nothing suppresses "
                "them. Without that control, a zero recovery count proves nothing.",
            ),
            # Defaulted off, so every run recorded before Phase 3 still describes
            # the world it was measured in.
            DeclareLaunchArgument(
                "with_static_obstacle",
                default_value="false",
                description="Park an immovable barrier across the aisle. Phase 2's "
                "recovery-suppression A/B needs a halt that outlasts "
                "movement_time_allowance, which walking pedestrians cannot produce.",
            ),
            DeclareLaunchArgument("obstacle_x", default_value="-5.0"),
            DeclareLaunchArgument("obstacle_y", default_value="0.0"),
            DeclareLaunchArgument("obstacle_size_y", default_value="5.5"),
            DeclareLaunchArgument(
                "decision_log",
                default_value="",
                description="Path stem for the selective-update accept/defer "
                "evidence. Empty writes nothing.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
