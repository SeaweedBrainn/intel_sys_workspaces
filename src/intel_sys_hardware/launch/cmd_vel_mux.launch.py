"""Who drives: twist_mux from /cmd_vel (Nav2) and /cmd_vel_teleop, with the /e_stop lock,
out on /cmd_vel_mux (see config/twist_mux.yaml).

If twist_mux is not installed (a rover image built before 2026-11-26), a plain relay
forwards /cmd_vel to /cmd_vel_mux instead, so the robot still drives, but without
priorities or the e-stop; it says so in the log.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_mux(context):
    sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    try:
        get_package_share_directory('twist_mux')
    except PackageNotFoundError:
        return [
            LogInfo(msg='WARNING: twist_mux is not installed (rebuild the image): /cmd_vel is relayed to '
                        '/cmd_vel_mux with no priorities, /cmd_vel_teleop is ignored and /e_stop does nothing.'),
            Node(package='intel_sys_hardware', executable='cmd_vel_relay', name='cmd_vel_relay',
                 output='screen', parameters=[{'use_sim_time': sim_time}]),
        ]
    config = os.path.join(get_package_share_directory('intel_sys_hardware'), 'config', 'twist_mux.yaml')
    return [Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
                 parameters=[config, {'use_sim_time': sim_time}],
                 remappings=[('cmd_vel_out', 'cmd_vel_mux')])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=make_mux),
    ])
