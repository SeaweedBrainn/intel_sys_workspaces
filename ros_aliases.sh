alias sourceros2='source /opt/ros/humble/setup.bash'
alias sourcelivox='source /root/ws_livox/install/setup.bash'

# For py-trees-tree-viewer to work
alias py-trees-tree-viewer='py-trees-tree-viewer --no-sandbox'

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

launchros2communicationwithtestbench(){
    sourceros2
    cd /root/ros2_ws && source install/setup.sh
    ros2 launch ros_robot_controller ros_robot_controller.launch.py
}

launchtestbenchodomnode(){
    sourceros2
    cd /root/ros2_ws && source install/setup.sh
    ros2 run controller odom_publisher
}

launchtestbenchteleop(){
    sourceros2
    cd /root/ros2_ws && source install/setup.sh
    ros2 run robotics_URC_package testbench_teleop
}

buildworkspace() {
    cd "$1" || return
    colcon build
}

buildworkspacepackage() {
    cd "$1" || return
    colcon build --packages-select "$2"
}

# recreate /dev/rrc symlink using udev info from host for testbench
if [ ! -e /dev/rrc ]; then
    TARGET=$(udevadm info --query=name --name=/dev/rrc 2>/dev/null || \
             ls /dev/ttyACM* 2>/dev/null | head -1)
    if [ -n "$TARGET" ]; then
        ln -sf "$TARGET" /dev/rrc
        echo "Created /dev/rrc -> $TARGET"
    fi
fi

# JetAuto robot configuration
export MACHINE_TYPE=JetAuto
export LIDAR_TYPE=A1
export DEPTH_CAMERA_TYPE=None
export need_compile=True

# force color output in tools
export CLICOLOR_FORCE=1
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# colored ls by default
alias ls='ls --color=auto'
alias grep='grep --color=auto'
