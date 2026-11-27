"""Tag follower, off until ~/enable (Foxglove's Follow tag button).

Runs on Point-LIO and Nav2 in odom, without RTAB-Map: Nav2 plans on the live LiDAR only,
so a target that pauses never becomes a stored obstacle. Started by the robot's `follow`
service (scripts/robot.sh up follow) and by the sim (mode:=follow), so both run it the
same way.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='intel_sys_navigation',
            executable='tag_follower',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
