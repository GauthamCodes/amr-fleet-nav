"""System composition: warehouse + the WHOLE FLEET + the cooperative map.

The only fleet-aware code here is a loop over ``load_fleet()``. Every robot goes
through the same ``robot_actions`` and the same ``robot_stack.launch.py``, and no
robot is named anywhere in this file (docs/ENGINEERING_NOTES.md rule 5) - so a third entry in
fleet.yaml brings up a third namespaced stack with no code change. That is what
"configuration-driven scaling" means here: bounded to one file, not free.

WHAT IS A SINGLETON AND WHAT IS PER ROBOT

    singleton   the rendered world, the Gazebo server, the /clock bridge,
                FleetMapNode, and the costmap filter group
    per robot   robot_actions (state publisher, spawn, bridge, watchdog, sensor
                frame bridges) and the whole software stack

    The singletons come from :func:`amr_gazebo.spawn.world_actions`, which
    ``amr1_nav.launch.py`` also calls. Neither launch includes the other, deliberately
    - see that file's docstring for what would break if it did.

ORDERING

    FleetMapNode is started BEFORE Nav2, and that is a hard requirement rather than
    tidiness. It publishes the static ``fleet_map -> amrN/map`` transforms and a
    correctly sized fleet map in its constructor; ``Costmap2DROS::on_activate``
    blocks waiting for the first of those, and the lifecycle manager's change_state
    call has no timeout. Start Nav2 first and the bringup does not fail fast - it
    hangs for a full ``initial_transform_timeout`` and then aborts, with nothing in
    the log naming the missing frame.

Run:
    ros2 launch amr_bringup fleet_nav.launch.py
    ros2 launch amr_bringup fleet_nav.launch.py headless:=false
    ros2 launch amr_bringup fleet_nav.launch.py with_actors:=false
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

#: When the first robot is spawned, and how far apart successive ones are. Gazebo
#: serialises /world/<name>/create requests; issuing them together is a race the
#: loser of which never enters the world at all.
SPAWN_START_S = 5.0
SPAWN_STAGGER_S = 3.0

#: Offset applied to the first robot's software stack, and the step between robots.
#: Six Nav2 servers and a slam_toolbox per robot is a lot to initialise at once.
STACK_STAGGER_S = 2.0
STACK_STAGGER_STEP_S = 2.0

#: When the fleet map and the filter group come up. Must be comfortably before the
#: earliest Nav2, which is robot_stack's NAV2_START_S (22) + STACK_STAGGER_S (2).
#: It depends on nothing itself - it publishes its frames and an empty map in its
#: constructor - so early is free, and being subscribed to each /amrN/map before
#: slam_toolbox starts also keeps that subscriber-gated publisher awake.
FLEET_MAP_START_S = 16.0


def _include(package, launch_file, arguments):
    """Return an include of another package's launch file."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments=arguments.items(),
    )


def _setup(context, *args, **kwargs):
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    with_nav2 = LaunchConfiguration("with_nav2").perform(context)
    suppress_recovery = LaunchConfiguration("suppress_recovery").perform(context)
    decision_log = LaunchConfiguration("decision_log").perform(context)
    with_filters = LaunchConfiguration("with_filters").perform(context)

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
        # Distinct from amr1_nav's warehouse_nav.sdf so a fleet run and a
        # single-robot run cannot overwrite each other's rendered world.
        out_name="warehouse_fleet.sdf",
    )

    actions = list(world)

    # The only fleet-aware lines in the file.
    for index, robot in enumerate(load_fleet()):
        actions.append(
            TimerAction(
                period=SPAWN_START_S + index * SPAWN_STAGGER_S,
                actions=robot_actions(robot, world_name),
            )
        )
        actions.append(
            _include(
                "amr_bringup",
                "robot_stack.launch.py",
                {
                    "robot": robot["name"],
                    "scan_topic": scan_topic,
                    "with_nav2": with_nav2,
                    "suppress_recovery": suppress_recovery,
                    "stagger": str(STACK_STAGGER_S + index * STACK_STAGGER_STEP_S),
                },
            )
        )

    actions.append(
        TimerAction(
            period=FLEET_MAP_START_S,
            actions=[
                _include(
                    "amr_fleet_control",
                    "fleet_map.launch.py",
                    {"decision_log": decision_log},
                ),
                _include(
                    "amr_navigation",
                    "costmap_filters.launch.py",
                    # A filter with no costmap to filter is three lifecycle nodes
                    # publishing to nobody, so the survey configuration skips them.
                    {
                        "enabled": (
                            "false" if with_nav2.lower() == "false" else with_filters
                        )
                    },
                ),
            ],
        )
    )
    return actions


def generate_launch_description():
    """Return the whole-fleet navigation stack."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("with_actors", default_value="true"),
            DeclareLaunchArgument("scan_topic", default_value=SCAN_TOPIC),
            DeclareLaunchArgument("with_nav2", default_value="true"),
            DeclareLaunchArgument("suppress_recovery", default_value="true"),
            DeclareLaunchArgument(
                "with_filters",
                default_value="true",
                description="Bring up the fleet-wide costmap filter servers. "
                "Phase 3 ships them with an all-zero mask.",
            ),
            DeclareLaunchArgument(
                "decision_log",
                default_value="",
                description="Path stem for the selective-update accept/defer "
                "evidence. Empty writes nothing.",
            ),
            DeclareLaunchArgument("with_static_obstacle", default_value="false"),
            DeclareLaunchArgument("obstacle_x", default_value="-5.0"),
            DeclareLaunchArgument("obstacle_y", default_value="0.0"),
            DeclareLaunchArgument("obstacle_size_y", default_value="5.5"),
            OpaqueFunction(function=_setup),
        ]
    )
