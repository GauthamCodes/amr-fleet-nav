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
#   a verified-clean process table to retract it. See README section 10.
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

# This workspace's own install tree. Every node a launch file starts from this
# repository runs as <WS>/install/<pkg>/lib/<pkg>/<exe>, so one pattern built from
# the path covers all of them - including any node added later that nobody
# remembers to list below.
#
# THIS ENTRY IS NOT OPTIONAL, AND IT WAS ADDED THE HARD WAY.
#
#   Naming nodes individually missed trajectory_predictor, traffic_control and
#   payload_jerk_adapter, none of which matched anything in the list. They are
#   long-lived subscribers, so they survived every "PROCESS TABLE CLEAN" banner
#   and accumulated: an audit found seven generations of them still running, the
#   oldest 13 hours old, together with 19 log files over 50 MB - one had reached
#   389 MB of TF_OLD_DATA warnings and was still growing.
#
#   A leaked trajectory_predictor still publishes a peer trajectory into the other
#   robot's local costmap, and a leaked payload_jerk_adapter still republishes onto
#   the motion chain. That is the same class of contamination as the four leaked
#   drive_watchdog processes behind the retracted "camera stalls the drive" finding
#   in README section 10 - and it is invisible, because the banner said clean.
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Matched against the full command line. Ordered roughly plant-outwards, though
# the kill is issued to all of them at once.
PATTERNS=(
  "${WS_DIR}/install/amr_"
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
  # Named as well as covered by the install-path pattern above, so that a node run
  # from somewhere else is still caught and so the list reads as the inventory it is.
  # These four are the long-lived ones: they hold subscriptions for the whole run
  # rather than exiting when their measurement is written.
  "trajectory_predictor"
  "traffic_control"
  "payload_jerk_adapter"
  "priority_mux"
  "ros2 launch"
  "ros2 run"
  # A recorder left running from an earlier session is a subscriber on every topic
  # it was told to record, which is exactly the kind of hidden graph load that
  # produced the retracted camera finding.
  "rosbag2"
  "ros2 bag"
  # RViz outlives the launch that started it: it is not matched by any pattern
  # above, so before this entry existed a "clean" process table could still have
  # a stale RViz attached to the previous run's topics. That window then sits on
  # screen showing the OLD run while the new one comes up, which is indistinguishable
  # from the new run being broken.
  #
  # Note for anyone reaching for this by hand: `pkill -f rviz2` typed at a prompt
  # kills the shell that typed it. -f matches the whole command line, the command
  # line contains the pattern, so the shell matches itself and dies with 144 before
  # rviz2 is touched. It is safe HERE only because this script's own command line is
  # "clean_processes.sh" and never contains the pattern text.
  "rviz2"
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
