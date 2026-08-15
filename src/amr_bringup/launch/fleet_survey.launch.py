"""COOPERATIVE MAPPING DEMO - both robots survey the aisle, one map grows.

The shot this exists for: `/fleet_map` starts all-unknown and fills in from two
directions at once, in a frame both robots share. It is the assignment's first
evaluation criterion and the one thing the recorded video could not show, because
mapping is only legible in RViz.

    ./scripts/clean_processes.sh
    ./ws.sh ros2 launch amr_bringup fleet_survey.launch.py

It is the fleet stack with Nav2 left out and a scripted drive per robot, which is
Phase 1's survey configuration scaled to the fleet by the same loop over
``load_fleet()`` as everything else - no robot is named in this file either.

WHY NO NAV2

    This run maps; it does not navigate. Twelve Nav2 servers evaluating plans
    against a map that is still being built spend CPU the simulator needs for
    scans, and the fleet map is the deliverable here. ``/amrN/plan`` therefore stays
    empty for this run and the RViz Path displays stay blank - that is expected, and
    the goal-driven run (``phase3_fleet_goals.launch.py``, also with ``rviz:=true``)
    is where those displays carry something.

WHY THE TWO ROUTES ARE THE SAME LOOP, PHASE-SHIFTED

    Both robots drive the identical closed circuit in the identical direction -
    eastbound in the south lane, westbound in the north lane - with robot *i*
    starting ``i`` corners along it. That keeps them on opposite sides of the loop
    rather than head-on in one lane, which matters because this configuration has
    no Nav2, no local costmap and no SafetyGate: nothing here avoids anything. It
    also means the two maps being composited come from genuinely different
    viewpoints of the same racks, which is what makes the composite worth looking
    at rather than one map with a second robot driving over it.

WHAT IT WRITES

    ``results/fleet_survey_updates.{md,csv}`` - FleetMapNode's accept/defer report,
    rewritten after every candidate, so ``cat`` it mid-run to watch coverage climb.
    A distinct stem from the Phase 3 deliverable, deliberately: this run is a demo,
    and it must not overwrite measured evidence.
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

from amr_description.fleet_config import load_fleet, spawn_pose

#: When the drives start. Each robot's slam_toolbox comes up at robot_stack's
#: SLAM_START_S (14) plus that robot's stagger, which is 18 s for the second robot
#: in a two-robot fleet. Driving before SLAM is listening throws away the
#: stationary first scans the map is anchored on.
SURVEY_START_S = 28.0

#: The circuit, in world coordinates, one lap - the Phase 1 survey route unchanged.
#: The aisle is |y| < 2.75 for x < 2.44; this runs it at |y| = 1.5 and stops short
#: of the ramp toe, because a pitched scan plane writes a phantom ground return
#: into the map as a wall that is not there (results/smoke2_ramp_phantom_return.md).
ROUTE_X = [0.8, 0.8, -13.0, -13.0]
ROUTE_Y = [-1.5, 1.5, 1.5, -1.5]


def _start_corner(index, count, waypoints):
    """Return which corner of the circuit robot ``index`` of ``count`` starts at.

    Spreads the fleet as evenly as four corners allow. For the two-robot fleet -
    the case this demo is checked against - it puts them diagonally opposite, so
    each is entering a lane the other has just left rather than meeting it in one.
    """
    return round(index * waypoints / count) % waypoints


def _rotated(sequence, offset):
    """Return ``sequence`` rotated left by ``offset`` positions."""
    return sequence[offset:] + sequence[:offset]


def _setup(context, *args, **kwargs):
    laps = int(LaunchConfiguration("laps").perform(context))
    results_dir = LaunchConfiguration("results_dir").perform(context)

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
            # SLAM and the fleet map, nothing else. with_nav2:=false also switches
            # off the costmap filters, the motion chain and the arbiter, none of
            # which have anything to filter, shape or arbitrate without a planner.
            "with_nav2": "false",
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "decision_log": os.path.join(results_dir, "fleet_survey_updates"),
        }.items(),
    )

    fleet = load_fleet()
    surveys = []
    for index, robot in enumerate(fleet):
        x, y, _, yaw = spawn_pose(robot)
        corner = _start_corner(index, len(fleet), len(ROUTE_X))
        surveys.append(
            Node(
                package="amr_navigation",
                executable="survey_drive.py",
                name="survey_drive",
                namespace=robot["name"],
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_name": robot["name"],
                        "route_x": _rotated(ROUTE_X, corner),
                        "route_y": _rotated(ROUTE_Y, corner),
                        "laps": laps,
                        "cruise_speed": float(
                            LaunchConfiguration("cruise_speed").perform(context)
                        ),
                        # The route is in world coordinates and odometry starts at
                        # the spawn pose, so the node is told where that is rather
                        # than discovering it. One robot property, from one place.
                        "spawn_x": x,
                        "spawn_y": y,
                        "spawn_yaw": yaw,
                    }
                ],
            )
        )

    # Shut down when the LAST drive finishes, not the first. The two robots start
    # the same distance from their first waypoint but do not finish together - a
    # turn taken slightly wide is seconds of difference over two laps - and killing
    # the world under the straggler would end the demo mid-lap.
    outstanding = [len(surveys)]

    def _on_survey_exit(event, context):
        outstanding[0] -= 1
        if outstanding[0] > 0:
            return []
        return [EmitEvent(event=Shutdown(reason="fleet survey complete"))]

    handlers = [
        RegisterEventHandler(
            OnProcessExit(target_action=survey, on_exit=_on_survey_exit)
        )
        for survey in surveys
    ]

    return [stack, TimerAction(period=SURVEY_START_S, actions=surveys)] + handlers


def generate_launch_description():
    """Return the two-robot cooperative mapping survey."""
    return LaunchDescription(
        [
            # The demo is the picture, so the simulator window is on by default
            # here and off everywhere else in this repository.
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="Override the RViz config. Empty takes "
                "fleet_nav.launch.py's default, rviz/fleet_mapping.rviz.",
            ),
            DeclareLaunchArgument(
                "with_actors",
                default_value="false",
                description="Pedestrians are off: slam_toolbox here has no "
                "dynamic-object rejection, so a walking actor is traced into the "
                "map as a smear that later scans only partly clear. That is a "
                "stated limitation, not a hidden one - see README section 10.",
            ),
            DeclareLaunchArgument(
                "laps",
                default_value="1",
                description="Laps of the circuit per robot. 1 covers the aisle and "
                "keeps the demo recordable; 2 is the Phase 1 map artifact's value "
                "and gives slam_toolbox a genuine revisit to close a loop against.",
            ),
            DeclareLaunchArgument(
                "cruise_speed",
                default_value="0.45",
                description="Survey speed, m/s. Both robots use the same one: at "
                "10 Hz scans this is a scan every 45 mm of travel, and the map is "
                "the deliverable rather than the drive.",
            ),
            DeclareLaunchArgument(
                "results_dir", default_value=os.path.join(os.getcwd(), "results")
            ),
            OpaqueFunction(function=_setup),
        ]
    )
