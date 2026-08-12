"""Namespaced Nav2 servers for one robot.

WHY THIS IS NOT AN INCLUDE OF nav2_bringup's navigation_launch.py

Two reasons, both load-bearing.

1. /tf IS GLOBAL AND THIS REPO DEPENDS ON THAT. navigation_launch.py hardcodes
   remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')] on every node, which gives
   each robot a TF tree inside its own namespace. Phase 0 namespaced by FRAME NAME
   instead (amr1/base_link on the global /tf), because Gazebo 8.11.0 has no
   <gz_frame_id> and derives a sensor's frame_id from its link name. Taking upstream's
   remap would point every Nav2 node at /amr1/tf, which nothing publishes; the stack
   would hang on transform timeouts with no error explaining why.

2. navigation_launch.py CONTAINS NO PushROSNamespace. Its `namespace` argument only
   feeds RewrittenYaml(root_key=...); the actual namespacing lives one level up in
   bringup_launch.py, gated on a separate `use_namespace` argument. Including it with
   namespace:=amr1 re-roots the YAML under `amr1:` while leaving nodes at `/`, so every
   parameter is silently ignored and the stack runs on defaults.

Writing the eight lines ourselves is clearer than working around both.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetParameter

from amr_description.fleet_config import load_fleet
from amr_navigation.params import lifecycle_nodes, render_nav2_params, SCAN_TOPIC

#: Executable and node name for each Nav2 server Phase 1 runs.
SERVERS = (
    ("nav2_controller", "controller_server", "controller_server"),
    ("nav2_smoother", "smoother_server", "smoother_server"),
    ("nav2_planner", "planner_server", "planner_server"),
    ("nav2_behaviors", "behavior_server", "behavior_server"),
    ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
    ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
)


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)

    fleet = {r["name"]: r for r in load_fleet()}
    if robot_name not in fleet:
        raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
    robot = fleet[robot_name]

    params = render_nav2_params(robot, namespace=robot_name, scan_topic=scan_topic)

    nodes = [
        Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=[params],
            # NO /tf remapping - see the module docstring.
        )
        for package, executable, name in SERVERS
    ]

    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": True,
                    "node_names": lifecycle_nodes(),
                    # Phase 0 learned this on the standalone costmap: a managed node
                    # that does not create a bond gets torn down by bond supervision
                    # while running perfectly well.
                    "bond_timeout": 0.0,
                }
            ],
        )
    )

    return [
        GroupAction(
            [
                PushROSNamespace(robot_name),
                SetParameter("use_sim_time", True),
                *nodes,
            ]
        )
    ]


def generate_launch_description():
    """Return the namespaced Nav2 launch description."""
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
