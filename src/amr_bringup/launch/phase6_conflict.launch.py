r"""PHASE 6 EXIT - one robot's trajectory arriving in another robot's local costmap.

This is the MAPF run (assignment section 3.2). Phase 3 deliberately gave the two
robots deconflicted lanes; this run crosses them.

    amr1  (-11, -1.5) -> (-4.0, +1.5)      south lane to north
    amr2  (-11, +1.5) -> (-4.0, -1.5)      north lane to south

Each robot must cross the other's lane to reach its goal, so their predicted
trajectories overlap in space. Whether they also overlap in TIME is left to the
run rather than staged - amr2 is the faster chassis by configuration and may
clear the crossing first. That is a real outcome and the report records it;
what this run measures does not depend on a dramatic near miss.

WHAT IS MEASURED, AND WHY IT NEEDS TWO ARMS

    results/phase6_cost_injection_layer_on.*    cost in amr2's LOCAL costmap at
                                                the cell amr1 is predicted to
                                                occupy 2 s from now
    results/phase6_cost_injection_layer_off.*   the same probe, same scenario,
                                                with the layer not loaded

    The second arm is not a formality. amr1 is a physical obstacle: amr2's LiDAR
    sees it, the obstacle layer marks it and the inflation layer spreads it. A
    single-arm run reporting "cost was present near the peer" would be measuring
    the obstacle layer and calling it MAPF. The layer-off arm is what makes the
    layer-on number mean anything.

Run both arms:
    ./scripts/clean_processes.sh
    ./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py
    ./scripts/clean_processes.sh
    ./ws.sh ros2 launch amr_bringup phase6_conflict.launch.py \
        with_trajectory_layer:=false tag:=layer_off
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

#: Same as Phase 3: late enough that both stacks are up and both action servers
#: are answering.
MISSION_START_S = 32.0

#: Flat (x, y, yaw) per robot in fleet.yaml order, in the fleet frame. Each goal
#: is in the OTHER robot's starting lane, which is what forces the crossing. The
#: goals are 7 m along rather than the full aisle so the crossing happens while
#: both robots are still moving, instead of after the faster one has arrived.
CROSSING_GOALS = [-4.0, 1.5, 0.0, -4.0, -1.5, 0.0]


def _setup(context, *args, **kwargs):
    results_dir = LaunchConfiguration("results_dir").perform(context)
    tag = LaunchConfiguration("tag").perform(context)
    with_layer = LaunchConfiguration("with_trajectory_layer").perform(context)

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_bringup"),
                "launch",
                "fleet_nav.launch.py",
            )
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "with_actors": LaunchConfiguration("with_actors"),
            "with_trajectory_layer": with_layer,
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
        }.items(),
    )

    mission = Node(
        package="amr_fleet_control",
        executable="fleet_mission",
        name="fleet_mission",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "goals": LaunchConfiguration("goals"),
                "results_dir": results_dir,
                "stem_prefix": "phase6_",
                "tag": f"crossing_{tag}",
                "title": (f"PHASE 6 - forced crossing, trajectory layer {tag.upper()}"),
                "separation_note": (
                    "The routes CROSS by construction - each robot's goal is in "
                    "the other's starting lane. Compare this number across the "
                    "layer_on and layer_off arms."
                ),
                "timeout_s": 200.0,
            }
        ],
    )

    # The probe observes amr2 and reports on amr1, which is the direction the
    # yield protocol also runs in: amr1 is the lead, amr2 is the follower that
    # gives way. Both are named here because a probe is an instrument, not part
    # of the fleet - ENGINEERING_NOTES rule 5 is about the SYSTEM having no per-robot
    # branching, and it does not.
    probe = Node(
        package="amr_fleet_control",
        executable="trajectory_probe",
        name="trajectory_probe",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "observer": "amr2",
                "peer": "amr1",
                "probe_dt_s": 2.0,
                "layer_enabled": with_layer.lower() != "false",
                "results_dir": results_dir,
                "tag": tag,
                "duration_s": 150.0,
            }
        ],
    )

    return [
        stack,
        TimerAction(period=MISSION_START_S, actions=[mission, probe]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=mission,
                on_exit=[EmitEvent(event=Shutdown(reason="conflict run complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the forced-crossing MAPF run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            # Pedestrians off: this run is about robot-robot interaction, and an
            # actor wandering into either lane would put cost into the same
            # costmap the probe is reading.
            DeclareLaunchArgument("with_actors", default_value="false"),
            DeclareLaunchArgument(
                "with_trajectory_layer",
                default_value="true",
                description="false gives the control arm.",
            ),
            DeclareLaunchArgument(
                "tag",
                default_value="layer_on",
                description="Names the evidence files. Use layer_off for the "
                "control arm so the two do not overwrite each other.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz on the cooperative mapping view. For this "
                "demo also switch on its predicted-trajectory and local-costmap "
                "toggles: the shot is one robot's predicted path lying across the "
                "other's local costmap.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="Override the RViz config. Empty takes "
                "fleet_nav.launch.py's default, rviz/fleet_mapping.rviz.",
            ),
            DeclareLaunchArgument(
                "results_dir", default_value=os.path.join(os.getcwd(), "results")
            ),
            DeclareLaunchArgument("goals", default_value=str(CROSSING_GOALS)),
            OpaqueFunction(function=_setup),
        ]
    )
