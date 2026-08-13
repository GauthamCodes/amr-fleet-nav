"""Bring up TrafficControlNode: one arbiter for the whole fleet.

Deliberately UNNAMESPACED, and for a stronger reason than FleetMapNode's. An
arbiter has to compare two robots' intentions against each other, so a
namespaced copy per robot would be N arbiters each seeing one side of every
conflict and each entitled to command a yield. This is the one node in the
system that is legitimately central - and it is central ONLY for escalation,
because the routine case is resolved inside each robot's own local costmap by
``FleetTrajectoryLayer`` (ENGINEERING_NOTES rule 7).

IT NEEDS THE MOTION CHAIN. The yield channel is a priority-150 input on each
robot's ``twist_mux``. With ``with_motion_chain:=false`` there is no mux - the
Phase 1/2 wiring runs Nav2 straight into SafetyGate - so a yield command would
be published to nobody. That combination is not an error worth crashing on,
because the survey and single-robot runs use it legitimately; it is simply a
fleet without arbitration, and the bringup does not start this node there.

Start it AFTER the predictors. Its only input is their output, and a conflict
test with one trajectory in it is not a conflict test.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    suppress = LaunchConfiguration("suppress_recovery").perform(context)
    return [
        Node(
            package="amr_fleet_control",
            executable="traffic_control",
            name="traffic_control",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "suppress_recovery": suppress.lower() != "false",
                    "time_window_s": float(
                        LaunchConfiguration("time_window_s").perform(context)
                    ),
                    "results_stem": LaunchConfiguration("results_stem").perform(
                        context
                    ),
                    "duration_s": float(
                        LaunchConfiguration("duration_s").perform(context)
                    ),
                    "title": LaunchConfiguration("title").perform(context),
                }
            ],
        )
    ]


def generate_launch_description():
    """Return the fleet's traffic arbiter."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "suppress_recovery",
                default_value="true",
                description="Raise the yielding robot's progress-checker allowance "
                "for the duration of the hold. false is the control arm: Nav2 then "
                "sees a robot that has not moved for longer than its allowance and "
                "dispatches recovery behaviours into a deliberate stop.",
            ),
            DeclareLaunchArgument(
                "results_stem",
                default_value="",
                description="Path stem for the yield evidence. Empty writes "
                "nothing, which is right for an ordinary bringup.",
            ),
            DeclareLaunchArgument(
                "duration_s",
                default_value="0.0",
                description="Write the report and shut down after this long. 0 "
                "runs until the launch stops the node, which also writes the "
                "report - a run that ends unexpectedly still leaves evidence.",
            ),
            DeclareLaunchArgument(
                "time_window_s",
                default_value="3.0",
                description="How far apart in time two robots may be predicted to "
                "occupy the same place and still count as conflicting. A "
                "constriction stays occupied for as long as it takes to traverse, "
                "so a run built around one legitimately widens this.",
            ),
            DeclareLaunchArgument(
                "title",
                default_value="PHASE 7 - yield protocol",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
