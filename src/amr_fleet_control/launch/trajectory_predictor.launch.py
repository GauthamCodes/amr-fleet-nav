"""One robot's TrajectoryPredictor, namespaced.

NAMESPACED, unlike fleet_map.launch.py. There is one fleet map but there is one
predictor PER ROBOT: each publishes its own intentions on its own
``/amrN/predicted_trajectory``, and its peers subscribe. Collapsing this into a
single fleet-wide node that predicted for everybody would be the central
coordinator ENGINEERING_NOTES rule 7 exists to prevent, wearing a different name.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetParameter


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)

    return [
        GroupAction(
            [
                PushROSNamespace(robot_name),
                SetParameter("use_sim_time", True),
                Node(
                    package="amr_fleet_control",
                    executable="trajectory_predictor",
                    name="trajectory_predictor",
                    output="screen",
                    parameters=[{"robot": robot_name}],
                ),
            ]
        )
    ]


def generate_launch_description():
    """Return one robot's trajectory predictor."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            OpaqueFunction(function=_setup),
        ]
    )
