"""Namespaced slam_toolbox for one robot.

TWO THINGS HERE ARE EASY TO GET WRONG AND SILENT WHEN YOU DO.

1. async_slam_toolbox_node is a LIFECYCLE node in Jazzy. Launched as a plain Node it
   comes up 'unconfigured' and does nothing at all - no map, no map->odom transform, and
   no error message to say so. Verified directly: `ros2 lifecycle get` on a bare node
   returns "unconfigured [1]" and every parameter reads back "Parameter not set". This
   file emits the configure and activate transitions, mirroring upstream's
   online_async_launch.py.

2. /tf IS GLOBAL AND THIS REPO DEPENDS ON THAT. Upstream's slam_launch.py applies
   SetRemap('/tf', 'tf') so each robot gets its own TF tree inside its namespace. We do
   the OPPOSITE: Phase 0 namespaces by FRAME NAME (amr1/base_link) and publishes
   everything to the global /tf, because Gazebo 8.11.0 has no <gz_frame_id> and derives
   sensor frame_id from the link name. Adding the remap would make slam_toolbox
   subscribe to /amr1/tf, which nothing publishes, and it would sit waiting for
   transforms forever. Do not "fix" this.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from amr_description.fleet_config import load_fleet
from amr_navigation.params import render_slam_params, SCAN_TOPIC


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)

    fleet = {r["name"]: r for r in load_fleet()}
    if robot_name not in fleet:
        raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
    robot = fleet[robot_name]

    params = render_slam_params(robot, namespace=robot_name, scan_topic=scan_topic)

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=robot_name,
        output="screen",
        parameters=[params, {"use_sim_time": True}],
        # NO /tf remapping - see the module docstring.
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return [slam, activate, configure]


def generate_launch_description():
    """Return the namespaced slam_toolbox launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            DeclareLaunchArgument(
                "scan_topic",
                default_value=SCAN_TOPIC,
                description=(
                    "Scan topic relative to the robot namespace. Phase 2 points this "
                    "at SensorBSP's validated output without touching any config."
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
