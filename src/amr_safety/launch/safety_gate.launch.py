"""SafetyGate for one robot, with every constant derived from fleet.yaml.

The C++ node contains no robot names and no robot knowledge. Everything that
differs between amr1 and amr2 - the stopping gain, the footprint polygon, the laser
offset - is computed here by ``amr_safety.safety_model`` and handed over as
parameters, which is what keeps docs/ENGINEERING_NOTES.md rule 5 true of a safety-critical node and
what makes an amr3 a fleet.yaml edit rather than a code change.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace

from amr_bsp.topics import CMD_GATED, CMD_NAV, VALIDATED_SCAN
from amr_description.fleet_config import laser_x_offset, load_fleet
from amr_safety.safety_model import gate_parameters, load_safety_config


def _setup(context, *args, **kwargs):
    """Resolve the robot, derive its safety constants, and build the node."""
    robot_name = LaunchConfiguration("robot").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    cmd_in_topic = LaunchConfiguration("cmd_in_topic").perform(context)
    cmd_out_topic = LaunchConfiguration("cmd_out_topic").perform(context)
    suppress = LaunchConfiguration("suppress_recovery").perform(context).lower()

    fleet = {r["name"]: r for r in load_fleet()}
    if robot_name not in fleet:
        raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
    robot = fleet[robot_name]

    derived = gate_parameters(robot, load_safety_config())

    node = Node(
        package="amr_safety",
        executable="safety_gate",
        name="safety_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "k": derived["k"],
                "d_min": derived["d_min"],
                "release_margin": derived["release_margin"],
                "min_hold_s": derived["min_hold_s"],
                "sector_half_angle": derived["sector_half_angle"],
                "self_return_margin": derived["self_return_margin"],
                "cmd_timeout_s": derived["cmd_timeout_s"],
                "scan_timeout_s": derived["scan_timeout_s"],
                "footprint_x": derived["footprint_x"],
                "footprint_y": derived["footprint_y"],
                "max_vel_x": derived["max_vel_x"],
                # The laser is rigidly mounted, so its pose comes from the same
                # fleet.yaml entry that builds the URDF. A TF lookup here would
                # re-derive a constant and add a failure mode to the command path.
                "laser_x": laser_x_offset(robot),
                "laser_y": 0.0,
                "laser_yaw": 0.0,
                "scan_topic": scan_topic,
                "odom_topic": "odom",
                "cmd_in_topic": cmd_in_topic,
                "cmd_out_topic": cmd_out_topic,
                "suppress_recovery": suppress != "false",
                "recovery_allowance_s": derived["recovery_allowance_s"],
                "restore_grace_s": derived["restore_grace_s"],
                "publish_rate_hz": 20.0,
            }
        ],
    )

    return [GroupAction([PushROSNamespace(robot_name), node])]


def generate_launch_description():
    """Return the SafetyGate launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            DeclareLaunchArgument(
                "scan_topic",
                default_value=VALIDATED_SCAN,
                description="Validated scan the gate measures clearance from.",
            ),
            DeclareLaunchArgument(
                "cmd_in_topic",
                default_value=CMD_NAV,
                description=(
                    "Upstream command. Phase 5 and 7 splice the velocity smoother, "
                    "PayloadJerkAdapter and twist_mux in ahead of the gate by "
                    "pointing this at the last of them; the gate does not change."
                ),
            ),
            DeclareLaunchArgument(
                "cmd_out_topic",
                default_value=CMD_GATED,
                description=(
                    "What the plant listens to. Only the fail-closed CONTROL run "
                    "changes it: with the drive watchdog removed, nothing bridges "
                    "cmd_vel to cmd_vel_plant, so the gate must address the plant "
                    "directly or the robot never moves and the SIGKILL measures "
                    "nothing."
                ),
            ),
            DeclareLaunchArgument(
                "suppress_recovery",
                default_value="true",
                description=(
                    "Set false to run the control case that shows Nav2 DOES fire "
                    "recovery behaviours during a halt when nothing suppresses them."
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
