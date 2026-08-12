"""Save the live SLAM map to a ``.pgm``/``.yaml`` pair on disk.

WHY THIS CALLS nav2_map_server DIRECTLY RATHER THAN slam_toolbox's save_map
    slam_toolbox's SaveMap service is a thin wrapper: it shells out to
    ``ros2 run nav2_map_server map_saver_cli`` and returns that process's status.
    It offers no way to pass the tool's ``save_map_timeout``, which defaults to
    2.0 seconds - and slam_toolbox publishes its map on a 2.0 second interval.
    The saver therefore has to catch a publication within one publication period.
    It lost that race on the first Phase 1 survey run, returning
    ``result=255`` and leaving no artifact behind after a three minute drive:

        [map_saver]: Saving map from 'map' topic to '.../phase1_map' file
        [map_saver]: Failed to spin map subscription        <- exactly 2.000 s later
        SaveMap_Response(result=255)

    Calling the same underlying tool directly costs nothing, uses no more code,
    and lets the timeout be set to something that cannot race - which is the
    difference between an artifact and a rerun. The topic is passed explicitly
    for the same reason: it is the one thing a namespaced stack most easily gets
    wrong, and a wrong topic fails identically to a lost race.
"""

from launch.actions import ExecuteProcess

#: Seconds the saver waits for a map message. Comfortably more than several of
#: slam_toolbox's publication intervals, so it cannot miss one.
SAVE_TIMEOUT_S = 20.0


def save_map_process(robot_name, path, timeout_s=SAVE_TIMEOUT_S):
    """Return a launch action that writes the map artifact and exits.

    Args:
        robot_name: Robot namespace; the map topic is ``/<robot_name>/map``.
        path: Artifact path WITHOUT extension. ``.pgm`` and ``.yaml`` are written
            beside each other, in nav2's own trinary format.
        timeout_s: How long the saver waits for a map message.

    Returns:
        An ``ExecuteProcess`` action.
    """
    return ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-t",
            f"/{robot_name}/map",
            "-f",
            path,
            "--ros-args",
            "-p",
            f"save_map_timeout:={timeout_s}",
        ],
        output="screen",
    )
