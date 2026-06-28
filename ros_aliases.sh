alias sourceros2='source /opt/ros/humble/setup.bash'
alias sourcelivox='source /root/ws_livox/install/setup.bash'

launchlidar() {
    source /opt/ros/humble/setup.bash
    cd /root/unilidar_sdk2-2.0.4/unitree_lidar_ros2 || return
    source install/setup.sh
    ros2 launch unitree_lidar_ros2 launch.py
}

launchslam() {
    source /opt/ros/humble/setup.bash
    cd /root/catkin_point_lio_unilidar || return
    source /root/ws_livox/install/setup.bash
    source install/setup.sh
    ros2 launch point_lio mapping_unilidar_l2.launch.py
}

buildworkspace() {
    cd "$1" || return
    colcon build
}

buildworkspacepackage() {
    cd "$1" || return
    colcon build --packages-select "$2"
}


# JetAuto robot configuration
export MACHINE_TYPE=JetAuto
export LIDAR_TYPE=A1
export DEPTH_CAMERA_TYPE=None
export need_compile=True

# force color output in tools
export COLCON_LOG_LEVEL=info
export CLICOLOR_FORCE=1
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# colored ls by default
alias ls='ls --color=auto'
alias grep='grep --color=auto'
