r"""PHASE 2 EXIT - the sensor validators, exercised on a live graph.

Two runs, one file, because they differ in exactly one thing: which sensor is under
test. Both stand the robot still and let the BSP work on a real stream.

    mode:=imu_injection   imu_fault_injector sits between the bridge and the BSP and
                          replaces angular velocity with 50 rad/s for five seconds.
                          The BSP is launched exactly as it ships - the only change
                          is its raw_imu_topic parameter. Evidence: the IMU channel
                          rejects the corrupt samples, logs a WARN naming the value,
                          and never puts one on validated/imu.

    mode:=camera          the camera is enabled on this run's URDF. Evidence:
                          CameraValidator runs over a sustained stream of real
                          frames and logs its outcome.

THE CAMERA A/B, AND THE FINDING IT CORRECTED

    ``drive`` and ``camera_enabled`` exist so this file can run the camera question
    as a controlled experiment rather than inherit an answer. An earlier session
    concluded that enabling the camera stalled the drive (0.35 m/s commanded,
    0.00017 m/s achieved). It did not. That run was contaminated by drive_watchdog
    processes leaked from previous launches, all republishing onto the plant's
    command topic. Re-run here on a verified-clean process table:

        camera ENABLED,  driving: peak 0.3500 m/s, 13.53 m
        camera DISABLED, driving: peak 0.3500 m/s, 13.86 m

    Identical. The camera stays defaulted off because nothing consumes it, not
    because it breaks anything - see amr.urdf.xacro.

    KILL STRAY PROCESSES BEFORE EVERY RUN. This is not housekeeping advice; it is
    the difference between a measurement and a fiction, and it cost a wrong
    conclusion once already.

Run:
    pkill -f 'gz sim'
    ros2 launch amr_bringup phase2_validation.launch.py mode:=imu_injection
    ros2 launch amr_bringup phase2_validation.launch.py mode:=camera
    ros2 launch amr_bringup phase2_validation.launch.py mode:=camera drive:=true \
        tag:=camera_motion_on
    ros2 launch amr_bringup phase2_validation.launch.py mode:=camera drive:=true \
        camera_enabled:=false tag:=camera_motion_off
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

from amr_description.fleet_config import load_fleet
from amr_gazebo.spawn import clock_bridge, gz_server, robot_actions
from amr_gazebo.world_builder import render_world, world_name_of
from amr_safety.safety_model import gate_parameters

#: Magnitude of the injected fault, rad/s. Twelve times amr1's plausibility bound.
INJECTED_RATE = 50.0


def _setup(context, *args, **kwargs):
    gazebo_pkg = get_package_share_directory("amr_gazebo")
    headless = LaunchConfiguration("headless").perform(context).lower() != "false"
    results_dir = LaunchConfiguration("results_dir").perform(context)
    duration = float(LaunchConfiguration("duration").perform(context))
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("imu_injection", "camera"):
        raise ValueError(f"mode must be 'imu_injection' or 'camera', got '{mode}'")
    override = LaunchConfiguration("camera_enabled").perform(context).lower()
    camera = (mode == "camera") if override == "" else override != "false"
    drive = LaunchConfiguration("drive").perform(context).lower() != "false"
    tag = LaunchConfiguration("tag").perform(context) or mode

    world_sdf = render_world(
        os.path.join(gazebo_pkg, "worlds", "warehouse.sdf.xacro"),
        mappings={"with_actors": "false"},
        out_name=f"warehouse_{tag}.sdf",
    )
    world_name = world_name_of(world_sdf)

    robot = dict(load_fleet()[0])
    if camera:
        # Per-run, not per-robot: fleet.yaml stays silent about the camera so no
        # other run inherits it. See fleet_config._XACRO_PASSTHROUGH.
        robot["camera_enabled"] = "true"
    derived = gate_parameters(robot)

    actions = [gz_server(world_sdf, headless=headless), clock_bridge()]
    actions += robot_actions(robot, world_name)

    bsp_arguments = {
        "robot": robot["name"],
        "min_gate_range": f"{derived['min_gate_range']:.4f}",
        "camera_enabled": "true" if camera else "false",
    }
    if mode == "imu_injection":
        bsp_arguments["raw_imu_topic"] = "imu_injected"

    bsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amr_bsp"), "launch", "bsp.launch.py"
            )
        ),
        launch_arguments=bsp_arguments.items(),
    )

    staged = []
    if drive:
        # The same actuator the Phase 0 smoke tests and the PitchGate run use, so
        # "did the robot move" is measured against a trajectory this repo has
        # already driven successfully many times.
        staged.append(
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package="amr_gazebo",
                        executable="smoke2_drive.py",
                        name="smoke2_drive",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "robot_name": robot["name"],
                                "cruise_speed": 0.35,
                                "spawn_x": float(robot["spawn"]["x"]),
                                "plateau_x": 9.0,
                                "return_x": -1.5,
                            }
                        ],
                    )
                ],
            )
        )
    if mode == "imu_injection":
        staged.append(
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="amr_bsp",
                        executable="imu_fault_injector.py",
                        name="imu_fault_injector",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "robot_name": robot["name"],
                                "inject_after_s": 12.0,
                                "inject_duration_s": 5.0,
                                "injected_rate": INJECTED_RATE,
                            }
                        ],
                    )
                ],
            )
        )

    probe = Node(
        package="amr_bsp",
        executable="validation_probe.py",
        name="validation_probe",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": robot["name"],
                "sample_duration_s": duration,
                "results_dir": results_dir,
                "tag": tag,
                "injected_topic": (
                    f"/{robot['name']}/imu_injected" if mode == "imu_injection" else ""
                ),
                "injected_rate": (INJECTED_RATE if mode == "imu_injection" else 0.0),
                "headline": (
                    "injected implausible IMU angular velocity"
                    if mode == "imu_injection"
                    else (
                        f"camera {'ENABLED' if camera else 'DISABLED'}, "
                        f"robot {'driving' if drive else 'stationary'}"
                    )
                ),
            }
        ],
    )

    return (
        actions
        + [TimerAction(period=8.0, actions=[bsp])]
        + staged
        + [
            TimerAction(period=10.0, actions=[probe]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=probe,
                    on_exit=[EmitEvent(event=Shutdown(reason=f"{mode} run complete"))],
                )
            ),
        ]
    )


def generate_launch_description():
    """Return the sensor-validation evidence run."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("mode", default_value="imu_injection"),
            DeclareLaunchArgument(
                "tag",
                default_value="",
                description="Report name; defaults to the mode. Needed when the "
                "same mode is run twice as an A/B.",
            ),
            DeclareLaunchArgument(
                "camera_enabled",
                default_value="",
                description="Overrides what the mode implies. Empty means 'follow "
                "the mode'. This is the knob the camera A/B turns.",
            ),
            DeclareLaunchArgument(
                "drive",
                default_value="false",
                description="Drive the robot down the aisle during the window. The "
                "camera A/B needs motion; the validator runs do not.",
            ),
            DeclareLaunchArgument("duration", default_value="35.0"),
            DeclareLaunchArgument(
                "results_dir",
                default_value=os.path.join(os.getcwd(), "results"),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
