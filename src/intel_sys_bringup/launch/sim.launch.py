import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_pkg = FindPackageShare('intel_sys_bringup')
    sim_pkg = FindPackageShare('intel_sys_sim')
    nav_pkg = FindPackageShare('intel_sys_navigation')

    default_rviz_config = PathJoinSubstitution([bringup_pkg, 'rviz', 'nav2_default_view.rviz'])

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2 interface')

    # 1. Simulation Bridge / World
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([sim_pkg, 'launch', 'sim.launch.py']))
    )

    # 2. Navigation with simulation time
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'navigation.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ekf': 'true',
            'use_point_lio': 'false',
            'autostart': 'true',
        }.items()
    )

    # 3. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        sim_launch,
        navigation_launch,
        rviz_node
    ])
