"""System composition: warehouse + the WHOLE FLEET + the cooperative map.

The only fleet-aware code here is a loop over ``load_fleet()``. Every robot goes
through the same ``robot_actions`` and the same ``robot_stack.launch.py``, and no
robot is named anywhere in this file (ENGINEERING_NOTES rule 5) - so a third entry in
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
    ros2 launch amr_bringup fleet_nav.launch.py headless:=false rviz:=true
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
from launch_ros.actions import Node

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

#: When the arbiter comes up. AFTER every robot's TrajectoryPredictor, which is
#: robot_stack's PREDICTOR_START_S (24) plus the largest stack stagger: its only
#: inputs are those predictions, and a conflict test with one trajectory in it is
#: not a conflict test.
TRAFFIC_START_S = 30.0

#: When RViz comes up, if it was asked for. Before FleetMapNode rather than after:
#: RViz spends several seconds loading OGRE and its plugins, and doing that while
#: twelve Nav2 servers are initialising costs the simulator real-time factor at the
#: worst moment. Starting it early means the first thing it draws is the fleet map
#: appearing, which is the shot.
RVIZ_START_S = 12.0

#: The cooperative mapping view, and the ONE place its path is written down. Every
#: launch that forwards `rviz_config` leaves it empty to mean "this one".
DEFAULT_RVIZ_CONFIG = os.path.join(
    get_package_share_directory("amr_bringup"), "rviz", "fleet_mapping.rviz"
)

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
            "obstacle_gap_y": LaunchConfiguration("obstacle_gap_y").perform(context),
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
                    "with_motion_chain": LaunchConfiguration(
                        "with_motion_chain"
                    ).perform(context),
                    "with_trajectory_layer": LaunchConfiguration(
                        "with_trajectory_layer"
                    ).perform(context),
                    "stagger": str(STACK_STAGGER_S + index * STACK_STAGGER_STEP_S),
                },
            )
        )

    # The arbiter needs the mux to command a yield through and Nav2 to have
    # something to arbitrate between, so it follows both. Neither is an error
    # worth failing on: the survey configuration legitimately runs without them.
    with_motion_chain = LaunchConfiguration("with_motion_chain").perform(context)
    with_traffic = LaunchConfiguration("with_traffic_control").perform(context)
    arbiter_wanted = (
        with_traffic.lower() != "false"
        and with_nav2.lower() != "false"
        and with_motion_chain.lower() != "false"
    )
    if arbiter_wanted:
        actions.append(
            TimerAction(
                period=TRAFFIC_START_S,
                actions=[
                    _include(
                        "amr_fleet_control",
                        "traffic_control.launch.py",
                        {
                            "suppress_recovery": suppress_recovery,
                            "results_stem": LaunchConfiguration(
                                "traffic_results_stem"
                            ).perform(context),
                            "duration_s": LaunchConfiguration(
                                "traffic_duration_s"
                            ).perform(context),
                            "time_window_s": LaunchConfiguration(
                                "traffic_time_window_s"
                            ).perform(context),
                            "title": LaunchConfiguration("traffic_title").perform(
                                context
                            ),
                        },
                    )
                ],
            )
        )

    # RViz is a DDS participant like any other node, so it needs the same merged
    # CYCLONEDDS_URI as the rest of the graph. Started from here it inherits the
    # environment of the ./ws.sh that launched this file, which is the one way of
    # starting it that cannot be got wrong. See DEMO_RUNBOOK section 2.
    if LaunchConfiguration("rviz").perform(context).lower() != "false":
        # An EMPTY value resolves to the default, and that is load-bearing rather
        # than defensive. Launch configurations are inherited by included launch
        # files, and DeclareLaunchArgument only supplies a default for a name that
        # is not already set - so a parent launch that declares its own
        # `rviz_config` with an empty default (phase3, phase6 and phase7 all do,
        # to keep the real path in one file) leaks that empty string down here and
        # wins over the default below. RViz is then started as `-d ""`, silently
        # falls back to its stock config, and comes up on Fixed Frame `map` - a
        # frame this system never publishes, because every map frame is either
        # namespaced `amrN/map` or the shared `fleet_map`. The window is blank and
        # the only clue is "Frame [map] does not exist".
        rviz_config = LaunchConfiguration("rviz_config").perform(context)
        if not rviz_config:
            rviz_config = DEFAULT_RVIZ_CONFIG
        actions.append(
            TimerAction(
                period=RVIZ_START_S,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="log",
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": True}],
                    )
                ],
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
                        ),
                        "ramp_mask_value": LaunchConfiguration(
                            "ramp_mask_value"
                        ).perform(context),
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
                "ramp_mask_value",
                default_value="0.0",
                description="Mask value on the ramp footprint, 0-90. 0 is the "
                "null mask Phase 3 shipped; a positive value makes the ramp "
                "traversable but expensive.",
            ),
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
            DeclareLaunchArgument(
                "with_motion_chain",
                default_value="true",
                description="Insert the payload-adaptive motion chain between "
                "Nav2 and SafetyGate on every robot. false is the control arm "
                "of the Phase 5 A/B and restores the Phase 1/2 wiring.",
            ),
            DeclareLaunchArgument(
                "with_trajectory_layer",
                default_value="true",
                description="Load FleetTrajectoryLayer into every robot's local "
                "costmap. This is where the MAPF requirement is satisfied; false "
                "is the control arm of the Phase 6 A/B.",
            ),
            DeclareLaunchArgument(
                "with_traffic_control",
                default_value="true",
                description="Run the fleet's yield arbiter. It only ever acts on "
                "conflicts the per-robot FleetTrajectoryLayer did not resolve, so "
                "leaving it on costs a run nothing until one occurs.",
            ),
            DeclareLaunchArgument("traffic_results_stem", default_value=""),
            DeclareLaunchArgument("traffic_duration_s", default_value="0.0"),
            DeclareLaunchArgument("traffic_time_window_s", default_value="3.0"),
            DeclareLaunchArgument(
                "traffic_title", default_value="PHASE 7 - yield protocol"
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz alongside the fleet, configured by "
                "rviz_config. Off by default because every evidence run is "
                "headless and a renderer it does not need costs it real-time "
                "factor; on for anything being recorded.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=DEFAULT_RVIZ_CONFIG,
                description="RViz config file. Empty means the default, which is "
                "the cooperative mapping view: /fleet_map in the fleet frame, both "
                "robots, both scans, both plans, key frames only.",
            ),
            DeclareLaunchArgument("with_static_obstacle", default_value="false"),
            DeclareLaunchArgument("obstacle_x", default_value="-5.0"),
            DeclareLaunchArgument("obstacle_y", default_value="0.0"),
            DeclareLaunchArgument("obstacle_size_y", default_value="5.5"),
            DeclareLaunchArgument(
                "obstacle_gap_y",
                default_value="0.0",
                description="Width of a gap left in the middle of the barrier. 0 "
                "is the solid barrier every earlier run was measured against.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
