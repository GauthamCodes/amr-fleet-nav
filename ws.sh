#!/usr/bin/env bash
# Workspace command wrapper.
#
# The environment this project targets does not reliably support inline `source` in shell
# commands, so every workspace-aware invocation goes through this script:
#
#   ./ws.sh ros2 launch amr_gazebo smoke1.launch.py
#
# It sources the ROS distribution and this workspace's install tree, then execs
# whatever it was given.
# Note: no `set -u`. The ROS setup scripts reference unset variables by design and
# will abort under nounset.
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
fi

# Gazebo must find this workspace's worlds and models.
export GZ_SIM_RESOURCE_PATH="${WS_DIR}/src/amr_gazebo/worlds:${WS_DIR}/src/amr_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

# A two-robot graph is roughly forty DDS participants and CycloneDDS defaults to a
# maximum auto-assigned participant index of 9. Past that, node creation throws and
# whichever Nav2 servers happened to start last die - which reads as "the second
# robot is broken" rather than as a host-wide limit. See the config for the full
# symptom.
#
# APPENDED, NOT ASSIGNED. CycloneDDS takes a comma-separated list of URIs and merges
# them, and this environment already sets one - an inline fragment pinning discovery
# to the loopback interface. Writing this as a `${VAR:-default}` fallback looked
# right and did nothing at all, because the variable was already set; overwriting it
# would have silently dropped the interface pinning instead.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:+${CYCLONEDDS_URI},}file://${WS_DIR}/src/amr_bringup/config/cyclonedds.xml"

exec "$@"
