"""The fleet-wide costmap filter group: mask server, info server, lifecycle manager.

UNNAMESPACED, ON PURPOSE

    There is one ramp in the world, so there is one mask. Both robots' global
    costmaps subscribe to the same filter-info topic. A per-robot copy would be three
    extra lifecycle nodes publishing identical data.

THE TOPIC THAT IS NOT WHAT IT LOOKS LIKE

    ``filter_info_topic`` is a parameter of the FILTER, inside each robot's costmap,
    and an absolute name there escapes the namespace correctly. But ``mask_topic``
    here is not read by the filter at all - it is copied into the
    ``nav2_msgs/CostmapFilterInfo`` message and resolved by the filter, against
    ``/<robot>/global_costmap/global_costmap``. A relative name therefore resolves to
    ``/amr1/global_costmap/global_costmap/ramp_filter_mask``, which nothing publishes,
    and the symptom is a filter that receives its info perfectly and then logs
    "Filter mask was not received" every two seconds. It must be absolute.

    The mask's ``frame_id`` must likewise be the frame the global costmaps run in.
    map_server defaults it to "map", and the mismatch shows up as "Failed to get
    costmap frame transformation to mask frame".

PHASE 3 SHIPS THIS WITH AN ALL-ZERO MASK

    The deliverable here is verified plumbing: the filter loads, both costmaps
    activate with it, and the cost it contributes is measurably nothing. Phase 4
    supplies the graded ramp regions and runs the two-route A/B on top of it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_fleet_control.fleet_grid import default_fleet_grid, FLEET_FRAME, RAMP_REGION
from amr_navigation.ramp_mask import write_mask

#: Absolute, for the reason in the module docstring.
MASK_TOPIC = "/ramp_filter_mask"
FILTER_INFO_TOPIC = "/ramp_filter_info"

#: nav2_msgs/CostmapFilterInfo type 0 is the keepout/lanes filter. base and
#: multiplier must be their defaults for it, or the filter warns and misreads the
#: mask - confirmed against nav2_costmap_2d/costmap_filters/filter_values.hpp.
KEEPOUT_FILTER_TYPE = 0
BASE_DEFAULT = 0.0
MULTIPLIER_DEFAULT = 1.0


def _setup(context, *args, **kwargs):
    if LaunchConfiguration("enabled").perform(context).lower() == "false":
        return []

    grid = default_fleet_grid()

    # Phase 3 shipped no regions: a uniformly free mask, so the filter's
    # contribution was provably zero. A nonzero ``ramp_mask_value`` puts a graded
    # cost on the ramp footprint and nowhere else - the "tuned configuration" the
    # assignment permits for slope traversability, in place of a custom layer.
    #
    # Default 0 keeps the null mask, so every run recorded before Phase 4 still
    # describes the costmap it was measured in. The cap lives in ramp_mask, which
    # RAISES above MAX_MASK_VALUE rather than clipping: mask 100 becomes cost 254
    # (LETHAL) and 253 is already INSCRIBED_INFLATED_OBSTACLE, which a footprint
    # collision checker treats as a collision. An expensive ramp is not an
    # impassable one.
    ramp_value = float(LaunchConfiguration("ramp_mask_value").perform(context))
    regions = () if ramp_value <= 0.0 else (RAMP_REGION + (ramp_value,),)
    yaml_path = write_mask(grid, regions=regions)

    mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        # Named apart from any future real map_server so the two cannot collide.
        name="filter_mask_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "yaml_filename": yaml_path,
                "topic_name": MASK_TOPIC,
                "frame_id": FLEET_FRAME,
            }
        ],
    )

    info_server = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "type": KEEPOUT_FILTER_TYPE,
                "filter_info_topic": FILTER_INFO_TOPIC,
                "mask_topic": MASK_TOPIC,
                "base": BASE_DEFAULT,
                "multiplier": MULTIPLIER_DEFAULT,
            }
        ],
    )

    manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap_filters",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["filter_mask_server", "costmap_filter_info_server"],
                # Phase 0 learned this on the standalone costmap: a managed node that
                # does not create a bond gets torn down by bond supervision while
                # running perfectly well.
                "bond_timeout": 0.0,
            }
        ],
    )

    return [mask_server, info_server, manager]


def generate_launch_description():
    """Return the fleet-wide costmap filter servers."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enabled",
                default_value="true",
                description="false brings up no filter servers at all. The mapping "
                "survey runs without Nav2, so it has no costmap to filter.",
            ),
            DeclareLaunchArgument(
                "ramp_mask_value",
                default_value="0.0",
                description="Mask value on the ramp footprint, 0-90. 0 is the "
                "null mask Phase 3 shipped and measured as contributing nothing; "
                "a positive value makes the ramp traversable but expensive.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
