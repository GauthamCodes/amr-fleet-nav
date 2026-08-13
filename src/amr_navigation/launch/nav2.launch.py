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

from amr_bsp.topics import CMD_NAV
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

#: Servers that publish cmd_vel, and must therefore be routed through SafetyGate.
#:
#: BOTH of them. controller_server is the obvious one; behavior_server is the one
#: that gets missed, because Spin, BackUp, DriveOnHeading and Wait publish their own
#: velocities directly (nav2_behaviors/timed_behavior.hpp constructs a TwistPublisher
#: on "cmd_vel"). A gate placed on the controller alone would let every recovery
#: behaviour drive the robot ungated - during exactly the situations where recovery
#: fires, which are the situations the gate exists for.
#:
#: Upstream nav2_bringup applies this same remap to every server; this repo builds
#: its own Nav2 launch (see the module docstring) and so has to state it.
CMD_VEL_PUBLISHERS = frozenset({"controller_server", "behavior_server"})

#: The stock velocity smoother, first link of the payload-adaptive motion chain.
#:
#: Listed apart from SERVERS because it is CONDITIONAL: it is a lifecycle node, so
#: it must appear in both this list and lifecycle_nodes() together or not at all.
#: Present in one but not the other, the lifecycle manager waits forever for a node
#: that never appears and the bringup hangs rather than failing.
SMOOTHER_SERVER = ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother")


def _setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)

    fleet = {r["name"]: r for r in load_fleet()}
    if robot_name not in fleet:
        raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
    robot = fleet[robot_name]

    with_layer = (
        LaunchConfiguration("with_trajectory_layer").perform(context).lower() != "false"
    )
    params = render_nav2_params(
        robot,
        namespace=robot_name,
        scan_topic=scan_topic,
        trajectory_layer_enabled=with_layer,
    )

    with_motion_chain = (
        LaunchConfiguration("with_motion_chain").perform(context).lower() != "false"
    )
    servers = list(SERVERS)
    if with_motion_chain:
        servers.append(SMOOTHER_SERVER)

    nodes = [
        Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=[params],
            # NO /tf remapping - see the module docstring.
            #
            # The same remap serves two different roles. For controller_server and
            # behavior_server, "cmd_vel" is what they PUBLISH. For velocity_smoother
            # it is what it SUBSCRIBES to, and pointing it at cmd_vel_nav is what
            # puts the stock smoother first in the chain behind Nav2.
            remappings=(
                [("cmd_vel", CMD_NAV)]
                if name in CMD_VEL_PUBLISHERS or name == SMOOTHER_SERVER[2]
                else []
            ),
        )
        for package, executable, name in servers
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
                    "node_names": lifecycle_nodes(with_motion_chain),
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
            DeclareLaunchArgument(
                "with_motion_chain",
                default_value="true",
                description=(
                    "Bring up the stock nav2_velocity_smoother, first link of the "
                    "payload-adaptive motion chain. It is a lifecycle node, so this "
                    "flag governs both the node and the lifecycle manager's list - "
                    "setting one without the other hangs the bringup."
                ),
            ),
            DeclareLaunchArgument(
                "with_trajectory_layer",
                default_value="true",
                description=(
                    "Load FleetTrajectoryLayer into the local costmap. false is the "
                    "control arm of the Phase 6 A/B: without it, a cost measured at a "
                    "peer's predicted cell proves nothing, because the obstacle layer "
                    "would have marked the peer's body anyway."
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
