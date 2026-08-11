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

exec "$@"
