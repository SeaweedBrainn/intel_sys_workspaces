#!/usr/bin/env bash
set -e

# Source ROS 2 base environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source third-party built dependencies overlay
if [ -f "/opt/intel_sys/third_party/setup.bash" ]; then
  source "/opt/intel_sys/third_party/setup.bash"
fi

# Source workspace install space if built
if [ -f "/workspaces/intel_sys/install/setup.bash" ]; then
  source "/workspaces/intel_sys/install/setup.bash"
fi

if [ $# -eq 0 ]; then
  exec bash
else
  exec "$@"
fi

