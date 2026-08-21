#!/usr/bin/env bash

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
  export GZ_SIM_RESOURCE_PATH="/workspaces/intel_sys/install/intel_sys_description/share:${GZ_SIM_RESOURCE_PATH}"
  export IGN_GAZEBO_RESOURCE_PATH="/workspaces/intel_sys/install/intel_sys_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
fi

# If being sourced by an interactive shell (.bashrc), return cleanly
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  return 0 2>/dev/null || exit 0
fi

# If executed directly as container entrypoint with arguments
if [ $# -gt 0 ]; then
  exec "$@"
else
  exec bash
fi
