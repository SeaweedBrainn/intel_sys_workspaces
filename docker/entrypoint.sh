#!/usr/bin/env bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/intel_sys/third_party/setup.bash
if [ -f /workspaces/intel_sys/install/setup.bash ]; then
  source /workspaces/intel_sys/install/setup.bash
fi
exec "$@"
