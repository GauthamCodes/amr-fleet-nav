r"""PHASE 7 EXIT - a forced narrow-intersection conflict, escalated to a yield.

Assignment 3.2's second half. Phase 6 measured the local half - one robot's
predicted trajectory arriving as cost in the other's LOCAL costmap, with no
central node involved. This run builds the case that layer cannot solve.

THE INTERSECTION IS BUILT, NOT HOPED FOR

    ``with_static_obstacle`` with ``obstacle_gap_y`` splits the Phase 3 barrier
    into two segments and leaves a 3.0 m gap on the aisle centre line at
    x = -5.0. Both robots start west of it and both goals are east of it, so
    both global plans converge on the same few metres of aisle. That is what
    makes the conflict unresolvable LOCALLY: there is no lateral room to deviate
    into, so no amount of cost in either local costmap moves either robot out of
    the other's way. The arbiter is the only thing that can order them.

    The gap is 3.0 m rather than tight for a measured reason: a robot centred in
    it clears each barrier end by 1.275 m, against amr1's braking envelope of
    0.975 m at cruise. A narrower gap would have SafetyGate halting inside the
    constriction, and this run would be measuring the gate rather than the yield.

    amr1  (-11.0, -1.5) -> (-1.0, -1.5)      through the gap, south lane
    amr2  (-11.0, +1.5) -> (-1.0, +1.5)      through the gap, north lane

WHY THE DISPATCH IS STAGGERED

    amr2 is the faster chassis by configuration (1.00 m/s against 0.60) and
    would otherwise reach the gap first and be through it before amr1 arrived -
    an encounter that never happens is not evidence about what happens in one.
    ``dispatch_offsets`` holds amr2's goal back so both reach the constriction
    together, which per Phase 3's measured speeds (1.79 s/m and 1.08 s/m over
    the same aisle) puts them there within a second of each other. Staging the
    encounter is the whole point of a FORCED conflict run, and the mission report
    prints the offset it was staged with.

WHY THE TIME WINDOW IS WIDER HERE

    The default 3.0 s treats a conflict as two robots wanting the same place at
    nearly the same moment. A constriction is a resource that stays occupied for
    as long as it takes to traverse it, which for a 3 m gap plus the approach is
    several seconds - so this run widens the window to 4.0 s. Stated here rather
    than changed in the node's defaults, because it is a property of this
    geometry and not of the fleet.

Run:
    ./scripts/clean_processes.sh
    ./ws.sh ros2 launch amr_bringup phase7_yield.launch.py

The control arm, if the necessity of recovery suppression is ever re-argued on
a yield rather than on Phase 3's safety halt:
    ./ws.sh ros2 launch amr_bringup phase7_yield.launch.py suppress_recovery:=false
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

#: Same as Phase 3 and Phase 6: late enough that both stacks are up, both action
#: servers answer, and the arbiter (fleet_nav's TRAFFIC_START_S = 30) is running.
MISSION_START_S = 34.0

#: Flat (x, y, yaw) per robot in fleet.yaml order, in the fleet frame. Both goals
#: are east of the barrier, so both routes must pass the gap.
CONVERGING_GOALS = [-1.0, -1.5, 0.0, -1.0, 1.5, 0.0]

#: Seconds each robot's goal is held back, in fleet.yaml order. See the docstring.
DISPATCH_OFFSETS = [0.0, 5.0]


def _setup(context, *args, **kwargs):
    results_dir = LaunchConfiguration("results_dir").perform(context)
    suppress = LaunchConfiguration("suppress_recovery").perform(context)
    tag = LaunchConfiguration("tag").perform(context)

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
            # Pedestrian_1 walks the aisle centre line straight through the gap.
            # This run is about robot-robot arbitration; an actor standing in the
            # constriction would stop both robots for a reason that is not it.
            "with_actors": "false",
            "with_static_obstacle": "true",
            "obstacle_x": LaunchConfiguration("obstacle_x"),
            "obstacle_gap_y": LaunchConfiguration("obstacle_gap_y"),
            "with_traffic_control": "true",
            "suppress_recovery": suppress,
            "traffic_results_stem": os.path.join(results_dir, f"phase7_{tag}"),
            "traffic_time_window_s": LaunchConfiguration("time_window_s"),
            "traffic_title": (
                "PHASE 7 - forced narrow-intersection conflict, escalated to a yield"
                + ("" if suppress.lower() != "false" else "  [CONTROL: no suppression]")
            ),
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
                "dispatch_offsets": DISPATCH_OFFSETS,
                "results_dir": results_dir,
                "stem_prefix": "phase7_",
                "tag": f"{tag}_mission",
                "title": (
                    "PHASE 7 - both goals through one 3.0 m gap, staged to conflict"
                ),
                "separation_note": (
                    "Both routes pass through the SAME gap, so this separation is "
                    "the outcome of the yield rather than of route planning. Read "
                    "it beside the arbiter's own report, which says when the hold "
                    "started, what released it, and what Nav2 did during it."
                ),
                "timeout_s": 240.0,
            }
        ],
    )

    return [
        stack,
        TimerAction(period=MISSION_START_S, actions=[mission]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=mission,
                # The arbiter writes its report on the way out, so the shutdown
                # is what produces the evidence rather than merely ending the run.
                on_exit=[EmitEvent(event=Shutdown(reason="yield run complete"))],
            )
        ),
    ]


def generate_launch_description():
    """Return the forced-conflict yield run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument(
                "suppress_recovery",
                default_value="true",
                description="false is the control arm: Nav2's progress checker is "
                "left at 10 s while the arbiter holds a robot for longer than that.",
            ),
            DeclareLaunchArgument(
                "tag",
                default_value="yield",
                description="Names the evidence files. Change it for a control "
                "arm so the two do not overwrite each other.",
            ),
            DeclareLaunchArgument("time_window_s", default_value="4.0"),
            DeclareLaunchArgument("obstacle_x", default_value="-5.0"),
            DeclareLaunchArgument(
                "obstacle_gap_y",
                default_value="3.0",
                description="Width of the gap in the barrier. See the docstring "
                "for why it is not tighter.",
            ),
            DeclareLaunchArgument(
                "results_dir", default_value=os.path.join(os.getcwd(), "results")
            ),
            DeclareLaunchArgument("goals", default_value=str(CONVERGING_GOALS)),
            OpaqueFunction(function=_setup),
        ]
    )
