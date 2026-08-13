"""Compose the launch actions that bring one robot into simulation.

Every robot in the fleet goes through :func:`robot_actions` unchanged. The function is
handed a dictionary from the typed robot list and never inspects the robot's name to
decide what to do, which is what keeps fleet scaling a matter of editing YAML.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

from amr_description.fleet_config import frame_prefix, spawn_pose, xacro_mappings
from amr_gazebo.world_builder import render_world, world_name_of


def gz_sensor_frame(robot, sensor_name):
    """Return the frame_id Gazebo will stamp on a sensor's messages.

    Gazebo Harmonic 8.11.0 has no ``<gz_frame_id>`` override, so the frame_id is always
    the scoped ``<model>/<link>/<sensor>`` name. Two things make it hard to predict:
    SDFormat's URDF importer lumps fixed joints, collapsing every fixed-attached link
    into the model root, so the sensor's link resolves to the root link rather than the
    link it was declared on.

    The resulting name (``amr1/amr1/base_footprint/lidar``) is not a frame any TF
    consumer knows about, which is why :func:`sensor_frame_bridges` publishes an
    identity transform tying it back to the real link.
    """
    prefix = frame_prefix(robot)
    return f"{robot['name']}/{prefix}base_footprint/{sensor_name}"


def sensor_frame_bridges(robot, use_sim_time=True):
    """Tie Gazebo's scoped sensor frames into the robot's TF tree.

    Each transform is identity: the Gazebo sensor frame and the URDF sensor link denote
    the same physical frame, they simply carry different names. Without these, /scan and
    /imu arrive stamped in a frame TF has never heard of and every costmap silently
    ignores them.

    Derived from the fleet entry, so this scales with the robot list like
    everything else.
    """
    prefix = frame_prefix(robot)
    pairs = (
        (f"{prefix}laser", gz_sensor_frame(robot, "lidar")),
        (f"{prefix}imu_link", gz_sensor_frame(robot, "imu")),
        (f"{prefix}camera_link", gz_sensor_frame(robot, "camera")),
    )
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"gz_frame_{robot['name']}_{index}",
            output="log",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                parent,
                "--child-frame-id",
                child,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        )
        for index, (parent, child) in enumerate(pairs)
    ]


def robot_xacro_path():
    """Return the installed path of the one shared robot description."""
    return os.path.join(
        get_package_share_directory("amr_description"), "urdf", "amr.urdf.xacro"
    )


def render_robot(robot):
    """Render the shared xacro with one robot's parameters into a URDF string."""
    return xacro.process_file(
        robot_xacro_path(), mappings=xacro_mappings(robot)
    ).toxml()


def bridge_arguments(robot):
    """Return ros_gz_bridge topic specifications for one robot.

    Direction syntax: ``[`` is Gazebo to ROS, ``]`` is ROS to Gazebo.
    """
    name = robot["name"]
    return [
        # Sensors. Header stamps arrive as Gazebo wrote them and are not rewritten
        # anywhere in this path.
        f"/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        # ROS type is Imu, Gazebo type is IMU. The bridge silently publishes nothing
        # if the ROS-side spelling is wrong, so it reads as a dead sensor.
        f"/{name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        # Gazebo's R8G8B8 arrives as the ROS encoding "rgb8", which is what
        # amr_bsp.validators.CAMERA_ENCODINGS accepts.
        f"/{name}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        # State.
        f"/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        f"/{name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        # Command. NOT /cmd_vel: the wheel controller's input is one hop further in,
        # behind drive_watchdog. /amrN/cmd_vel is SafetyGate's output and the robot's
        # command topic; see amr_gazebo.xacro's DiffDrive block for why the plant
        # needs a watchdog of its own.
        f"/{name}/cmd_vel_plant@geometry_msgs/msg/Twist]gz.msgs.Twist",
        # Simulator ground truth, for smoke tests and later drift metrics only.
        f"/model/{name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    ]


def robot_actions(robot, world_name, use_sim_time=True, with_watchdog=True):
    """Return every launch action needed to bring one robot up in simulation.

    Args:
        robot: One entry from the typed fleet list.
        world_name: Name of the Gazebo world to spawn into.
        use_sim_time: Whether nodes should follow ``/clock``.
        with_watchdog: Whether to start the plant-side command watchdog. True for
            every ordinary run. The Phase 2 fail-closed evidence run sets it False
            for its control case, which is the only way to show what the watchdog
            is actually buying: with it, a SIGKILLed SafetyGate stops the robot;
            without it, Gazebo's DiffDrive keeps rolling on the latched command.
            An A/B is the difference between measuring that and asserting it.

    Returns:
        A list of launch actions: robot_state_publisher, the Gazebo spawn request,
        the per-robot topic bridge, and (by default) the drive watchdog.
    """
    name = robot["name"]
    prefix = frame_prefix(robot)
    x, y, z, yaw = spawn_pose(robot)
    description = render_robot(robot)

    # Link names already carry the robot prefix, so robot_state_publisher runs
    # WITHOUT frame_prefix. Setting both would produce amr1/amr1/base_link.
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=name,
        output="screen",
        parameters=[
            {
                "robot_description": description,
                "use_sim_time": use_sim_time,
                "frame_prefix": "",
            }
        ],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{name}",
        output="screen",
        arguments=[
            "-world",
            world_name,
            "-string",
            description,
            "-name",
            name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-Y",
            str(yaw),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"bridge_{name}",
        output="screen",
        arguments=bridge_arguments(robot),
        parameters=[{"use_sim_time": use_sim_time}],
        # The differential drive system publishes odom->base_footprint on its own
        # topic; it belongs on /tf like any other transform.
        remappings=[
            (f"/{name}/pose", "/tf"),
            (f"/model/{name}/pose", "/gz_ground_truth"),
        ],
    )

    # Part of the plant, so it comes up with the robot rather than with the nav stack.
    # Since the DiffDrive plugin now listens on cmd_vel_plant, nothing moves at all
    # without this - which is exactly the property that makes rule 1 demonstrable.
    watchdog = Node(
        package="amr_gazebo",
        executable="drive_watchdog.py",
        name="drive_watchdog",
        namespace=name,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "timeout_s": 0.3}],
    )

    _ = prefix  # frames are already baked into the description
    plant = [state_publisher, spawn, bridge]
    if with_watchdog:
        plant.append(watchdog)
    return plant + sensor_frame_bridges(robot, use_sim_time=use_sim_time)


def clock_bridge(use_sim_time=True):
    """Return the single world-wide clock bridge."""
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": use_sim_time}],
    )


def ground_truth_bridge(world_name, use_sim_time=True):
    """Bridge Gazebo's ground-truth pose stream.

    Used only by the Phase 0 smoke tests, which need to know where an obstacle actually
    is in order to decide whether the LiDAR saw it. Nothing in the navigation stack
    consumes this.
    """
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ground_truth_bridge",
        output="screen",
        arguments=[
            f"/world/{world_name}/dynamic_pose/info"
            "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            (f"/world/{world_name}/dynamic_pose/info", "/gz_ground_truth"),
        ],
    )


def gz_server(world_sdf_path, headless=True):
    """Return the Gazebo server process for a rendered world."""
    args = ["-r", "-s", "-v", "2"] if headless else ["-r", "-v", "2"]
    return ExecuteProcess(
        cmd=["gz", "sim", *args, world_sdf_path],
        output="screen",
    )


def world_actions(mappings=None, headless=True, out_name="warehouse_nav.sdf"):
    """Return the world-singleton actions, plus the rendered world's name and path.

    These are the actions there must be exactly ONE of however many robots a launch
    brings up: the rendered SDF, the Gazebo server, and the /clock bridge. Phase 3
    needed a two-robot launch, and the obvious way to get one - having the fleet
    launch include the single-robot launch twice - starts two Gazebo servers and two
    identically named clock bridges.

    The inverse mistake is worse and is the reason this helper exists rather than a
    launch file: if the SINGLE-robot launch delegated to the fleet launch, then
    phase1_survey, phase1_nav_run and phase2_safety_run - which include it by name -
    would each silently become two-robot runs. Their recorded numbers are sensitive
    to graph contention (SESSION_LOG Phase 2, surprise 1), so that would not fail,
    it would just quietly move the measurements.

    Both launches call this instead, and neither includes the other.

    Args:
        mappings: xacro arguments for warehouse.sdf.xacro; values are stringified.
        headless: Run Gazebo without its GUI.
        out_name: Filename for the rendered SDF, unique per launch so that two
            concurrent launches do not overwrite each other's world.

    Returns:
        ``(actions, world_name, world_sdf_path)``.
    """
    world_sdf = render_world(
        os.path.join(
            get_package_share_directory("amr_gazebo"), "worlds", "warehouse.sdf.xacro"
        ),
        mappings=mappings or {},
        out_name=out_name,
    )
    world_name = world_name_of(world_sdf)
    return (
        [gz_server(world_sdf, headless=headless), clock_bridge()],
        world_name,
        world_sdf,
    )
