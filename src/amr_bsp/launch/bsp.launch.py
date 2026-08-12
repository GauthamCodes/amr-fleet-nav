"""SensorBSP for one robot, with its plausibility bounds derived from fleet.yaml.

WHY THE PITCHGATE FLOOR ARRIVES AS AN ARGUMENT RATHER THAN AN IMPORT

    The floor is the interlock between PitchGate and SafetyGate: truncation must
    never reach inside the braking envelope. Its value therefore belongs to
    ``amr_safety.safety_model``. Importing that here would make amr_bsp depend on
    amr_safety, which already depends on amr_bsp for the topic contract - a cycle,
    and colcon is right to refuse one.

    So the value is supplied by the composition layer, which is the one place that
    legitimately sees both packages (PLAN.md section 7: "amr_bringup - system
    composition only"). Running this launch bare leaves the floor at 0.0, and the
    node says so loudly rather than quietly running without its interlock. The
    relationship itself is pinned independently by tests/test_safety_distance.py,
    so it does not depend on any launch file being correct.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace

from amr_bsp.pitch_gate import DEFAULT_MARGIN
from amr_bsp.topics import RAW_IMU
from amr_description.fleet_config import load_fleet

#: Multiple of the robot's own turn limit that still counts as plausible.
#:
#: A robot commanded at max_vel_theta can legitimately exceed it - a shove, a wheel
#: slipping on the ramp, the yaw transient at the ramp toe. 4x leaves those alone
#: while rejecting the order-of-magnitude nonsense a broken IMU produces. The
#: injected fault used for evidence is 50 rad/s, i.e. ~12x this bound for amr1.
IMU_RATE_MARGIN = 4.0

#: Largest plausible specific force, m/s^2. Three g. A warehouse AMR that
#: experiences three g has been dropped, and its IMU is no longer the problem.
#: NOTE this is specific force and includes gravity: a healthy stationary sensor
#: reads 9.81, so any bound below that rejects a working IMU at rest.
MAX_LINEAR_ACCEL = 3.0 * 9.80665


def _setup(context, *args, **kwargs):
    """Resolve the robot and build its BSP node."""
    robot_name = LaunchConfiguration("robot").perform(context)
    min_gate_range = float(LaunchConfiguration("min_gate_range").perform(context))
    camera_enabled = (
        LaunchConfiguration("camera_enabled").perform(context).lower() != "false"
    )
    camera_width = int(LaunchConfiguration("camera_width").perform(context))
    camera_height = int(LaunchConfiguration("camera_height").perform(context))
    raw_imu_topic = LaunchConfiguration("raw_imu_topic").perform(context)

    fleet = {r["name"]: r for r in load_fleet()}
    if robot_name not in fleet:
        raise ValueError(f"'{robot_name}' is not in fleet.yaml: {sorted(fleet)}")
    robot = fleet[robot_name]

    node = Node(
        package="amr_bsp",
        executable="sensor_bsp",
        name="sensor_bsp",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "lidar_height": float(robot["lidar_height"]),
                "gate_margin": DEFAULT_MARGIN,
                "min_gate_range": min_gate_range,
                "max_angular_rate": (IMU_RATE_MARGIN * float(robot["max_vel_theta"])),
                "max_linear_accel": MAX_LINEAR_ACCEL,
                "camera_enabled": camera_enabled,
                "camera_width": camera_width,
                "camera_height": camera_height,
                "raw_imu_topic": raw_imu_topic,
                # 1.5 scan periods at 10 Hz: long enough that one late scan is not a
                # fault, short enough that a dead sensor is caught within 150 ms.
                "scan_stale_s": 0.15,
                "imu_stale_s": 0.05,
                "camera_stale_s": 0.5,
                "imu_max_age": 0.15,
                "range_max_hint": 12.0,
            }
        ],
    )

    return [GroupAction([PushROSNamespace(robot_name), node])]


def generate_launch_description():
    """Return the SensorBSP launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="amr1"),
            DeclareLaunchArgument(
                "min_gate_range",
                default_value="0.0",
                description=(
                    "PitchGate truncation floor, metres. Supplied by amr_bringup "
                    "from amr_safety.safety_model.min_gate_range; see the module "
                    "docstring for why it is not imported here."
                ),
            ),
            # Kept in step with amr.urdf.xacro's camera_enabled, which is defaulted
            # off because nothing downstream consumes the camera - NOT because it
            # costs motion; that earlier claim was retracted and re-measured, see
            # that file. Turning one on without the other leaves the BSP subscribed
            # to a topic nobody publishes.
            DeclareLaunchArgument("camera_enabled", default_value="false"),
            DeclareLaunchArgument("camera_width", default_value="160"),
            DeclareLaunchArgument("camera_height", default_value="120"),
            # The fault-injection seam. Production runs leave this at the raw
            # sensor; the IMU evidence run points it at imu_fault_injector's
            # output, so the BSP itself is launched exactly as it ships and no
            # fault-simulation code exists inside it. See scripts/
            # imu_fault_injector.py.
            DeclareLaunchArgument("raw_imu_topic", default_value=RAW_IMU),
            OpaqueFunction(function=_setup),
        ]
    )
