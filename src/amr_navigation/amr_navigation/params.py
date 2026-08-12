"""Render the per-robot Nav2 and slam_toolbox parameter files.

The YAML templates in ``config/`` name no robot and hardcode no kinodynamic limit.
This module supplies both, so a robot's footprint and velocity limits come from the same
``fleet.yaml`` entry that shapes its URDF and constrains its Gazebo plugin. One source,
three consumers - which is the only way the planner's model and the plant stay in
agreement.

WHY TEMPLATE RENDERING RATHER THAN RewrittenYaml
    Nav2's own launch files specialise parameters with ``RewrittenYaml``, which matches
    on key NAME anywhere in the tree. That cannot express what this stack needs: the
    local costmap's ``global_frame`` must be ``amr1/odom`` while every other server's
    must be ``amr1/map``, and a key rewrite sets both the same. Rendering the
    template to a concrete file avoids the collision, and has the side benefit that the
    exact parameters a run used can be read off disk afterwards.

    This mirrors ``amr_gazebo.world_builder.render_world``, which renders the world SDF
    the same way and for the same reason.

TOPIC INDIRECTION FOR PHASE 2
    ``SCAN_TOPIC`` is threaded through here rather than written into the YAML. Today it
    resolves to ``scan``; when SensorBSP lands it becomes ``validated/scan`` and neither
    ``nav2_params.yaml`` nor any launch file changes. That is what satisfies docs/ENGINEERING_NOTES.md
    rule 8 without pre-building Phase 2.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
import yaml

from amr_description.fleet_config import footprint_polygon, frame_prefix

#: Topic the navigation stack reads scans from, relative to the robot namespace.
#: Phase 2 changes this ONE string to "validated/scan".
SCAN_TOPIC = "scan"


def config_dir():
    """Return the installed config directory for this package."""
    return os.path.join(get_package_share_directory("amr_navigation"), "config")


def generated_dir():
    """Return the directory rendered parameter files are written to."""
    path = os.environ.get(
        "AMR_GENERATED_DIR",
        os.path.join(tempfile.gettempdir(), "amr_fleet_nav_generated"),
    )
    os.makedirs(path, exist_ok=True)
    return path


def frames(robot):
    """Return the prefixed frame names this robot's stack must use."""
    prefix = frame_prefix(robot)
    return {
        "map": f"{prefix}map",
        "odom": f"{prefix}odom",
        "base": f"{prefix}base_footprint",
    }


def _substitutions(robot, scan_topic):
    """Return the placeholder -> value map for one robot."""
    f = frames(robot)
    return {
        "__MAP_FRAME__": f["map"],
        "__ODOM_FRAME__": f["odom"],
        "__BASE_FRAME__": f["base"],
        "__SCAN_TOPIC__": scan_topic,
        # Absolute: this is the topic slam.launch.py remaps slam_toolbox's
        # namespace-escaping /map onto. See that file's docstring.
        "__MAP_TOPIC__": f"/{robot['name']}/map",
        "__FOOTPRINT__": str(footprint_polygon(robot)),
        # Kinodynamic limits, identical to what the Gazebo DiffDrive plugin enforces.
        # A planner that models limits the plant will not deliver produces tracking
        # error that looks exactly like a controller tuning problem.
        "__VX_MAX__": f"{float(robot['max_vel_x']):.4f}",
        "__VX_MIN__": f"{-abs(float(robot['max_vel_x'])) * 0.4:.4f}",
        "__WZ_MAX__": f"{float(robot['max_vel_theta']):.4f}",
        "__AX_MAX__": f"{float(robot['max_accel_x']):.4f}",
        # NEGATIVE by construction: MPPI clamps as vx_last + model_dt * ax_min,
        # so a positive value here inverts the deceleration limit.
        "__AX_MIN__": f"{-abs(float(robot['max_decel_x'])):.4f}",
        "__AZ_MAX__": f"{float(robot['max_accel_theta']):.4f}",
    }


def _render(template_path, robot, scan_topic, namespace, out_name):
    """Substitute placeholders, re-root under the namespace, and write the file."""
    with open(template_path, "r", encoding="utf-8") as handle:
        text = handle.read()

    for placeholder, value in _substitutions(robot, scan_topic).items():
        text = text.replace(placeholder, value)

    if "__" in text.replace("ros__parameters", ""):
        leftovers = {
            token
            for token in text.split()
            if token.startswith("__") and token.endswith("__")
        }
        if leftovers:
            raise ValueError(
                f"{template_path} has unsubstituted placeholders: {sorted(leftovers)}"
            )

    data = yaml.safe_load(text)
    # A node under a namespace is /amr1/controller_server, so a bare
    # 'controller_server:' key does not match it and every parameter is silently
    # ignored - the single most common way a namespaced Nav2 comes up on defaults.
    rooted = {namespace: data} if namespace else data

    out_path = os.path.join(generated_dir(), out_name)
    with open(out_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(rooted, handle, default_flow_style=False, sort_keys=False)
    return out_path


def render_nav2_params(robot, namespace=None, scan_topic=SCAN_TOPIC):
    """Render this robot's Nav2 parameter file and return its path."""
    namespace = robot["name"] if namespace is None else namespace
    return _render(
        os.path.join(config_dir(), "nav2_params.yaml"),
        robot,
        scan_topic,
        namespace,
        f"nav2_params_{robot['name']}.yaml",
    )


def render_slam_params(robot, namespace=None, scan_topic=SCAN_TOPIC):
    """Render this robot's slam_toolbox parameter file and return its path."""
    namespace = robot["name"] if namespace is None else namespace
    return _render(
        os.path.join(config_dir(), "slam_params.yaml"),
        robot,
        scan_topic,
        namespace,
        f"slam_params_{robot['name']}.yaml",
    )


def lifecycle_nodes():
    """Return the Nav2 servers the lifecycle manager brings up, in order.

    slam_toolbox is deliberately absent. It is a lifecycle node too, but it drives its
    own transitions in slam.launch.py exactly as upstream's online_async_launch.py does,
    so a SLAM restart does not tear down navigation.
    """
    return [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]
