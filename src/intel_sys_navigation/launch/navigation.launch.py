import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('intel_sys_navigation')
    nav2_bringup_pkg = FindPackageShare('nav2_bringup')

    default_nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup Nav2 stack')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_nav2_params, description='Nav2 params YAML')
    map_arg = DeclareLaunchArgument('map', default_value='', description='Full path to map yaml file to load (optional)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    # Nav2 Stack Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_bringup_pkg, 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map': map_file,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        map_arg,
        nav2_bringup_launch
    ])
