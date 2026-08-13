#!/usr/bin/env bash
# Kill everything a previous run may have left behind, then PRINT WHAT SURVIVED.
#
# WHY THIS EXISTS
#
#   A process-hygiene failure is a measurement failure. Four drive_watchdog
#   processes leaked from earlier launches once stayed subscribed to
#   /amr1/cmd_vel and republished onto /amr1/cmd_vel_plant; the contention
#   starved the graph until the SafetyGate's command-timeout fail-safe fired and
#   interleaved zeros into the command stream. The robot barely moved. That was
#   written down as "the camera stalls the drive", and it took a re-measurement on
#   a verified-clean process table to retract it. See docs/SESSION_LOG.md, Phase 2.
#
#   Phase 3 runs two of everything, so there is twice as much to leak.
#
# THE PRINTED TABLE IS THE POINT
#
#   A cleanup script that exits 0 having killed nothing looks exactly like a
#   cleanup script that worked. This one always lists what is still running, so
#   "clean process table" is something you SAW rather than something you assume.
#   It exits non-zero if anything survived, so a launch can chain on it.
#
# Usage:
#   ./scripts/clean_processes.sh          # kill, wait, report
#   ./scripts/clean_processes.sh --check  # report only, kill nothing

set -uo pipefail

CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

# Matched against the full command line. Ordered roughly plant-outwards, though
# the kill is issued to all of them at once.
PATTERNS=(
  "gz sim"
  "ruby.*gz"
  "parameter_bridge"
  "robot_state_publisher"
  "drive_watchdog"
  "static_transform_publisher"
  "async_slam_toolbox_node"
  "controller_server"
  "planner_server"
  "smoother_server"
  "behavior_server"
  "bt_navigator"
  "waypoint_follower"
  "lifecycle_manager"
  "map_server"
  "costmap_filter_info_server"
  "safety_gate"
  "sensor_bsp"
  "fleet_map_node"
  "fleet_mission"
  "ros2 launch"
  "ros2 run"
  # A recorder left running from an earlier session is a subscriber on every topic
  # it was told to record, which is exactly the kind of hidden graph load that
  # produced the retracted camera finding.
  "rosbag2"
  "ros2 bag"
)

survivors() {
  local pattern found=""
  for pattern in "${PATTERNS[@]}"; do
    # -f matches the full command line; exclude this script and its own pgrep so
    # the cleanup never reports itself as debris.
    while read -r pid rest; do
      [ -z "${pid}" ] && continue
      [ "${pid}" = "$$" ] && continue
      case "${rest}" in
        *clean_processes.sh*) continue ;;
      esac
      found+="${pid} ${rest}"$'\n'
    done < <(pgrep -af "${pattern}" 2>/dev/null || true)
  done
  # One line per pid, deduplicated: a process matching two patterns is one process.
  printf '%s' "${found}" | awk 'NF' | sort -u -k1,1n
}

if [ "${CHECK_ONLY}" -eq 0 ]; then
  for pattern in "${PATTERNS[@]}"; do
    pkill -f "${pattern}" 2>/dev/null || true
  done
  sleep 2
  # SIGKILL whatever ignored SIGTERM. Phase 2's fail-closed run showed that a
  # shutdown handler is not something to rely on.
  for pattern in "${PATTERNS[@]}"; do
    pkill -9 -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
fi

REMAINING="$(survivors)"

echo "=============================================================================="
if [ -z "${REMAINING}" ]; then
  echo "PROCESS TABLE CLEAN - nothing matching a simulation or ROS pattern is running"
  echo "=============================================================================="
  exit 0
fi

echo "SURVIVORS - these are still running and WILL contaminate a measurement:"
echo "------------------------------------------------------------------------------"
echo "${REMAINING}"
echo "=============================================================================="
exit 1
