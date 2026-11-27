"""Frontier explorer, idle until ~/start (Foxglove's Explore button).

Runs on top of localization with RTAB-Map and Nav2 on its /map. Started by the robot's
`explorer` service (scripts/robot.sh up) and by the sim (mode:=explore), so both run it
the same way.
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
            executable='frontier_explorer',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'wait_for_start': True,
            }],
        ),
    ])
