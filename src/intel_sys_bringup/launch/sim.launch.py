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
    loc_pkg = FindPackageShare('intel_sys_localization')
    nav_pkg = FindPackageShare('intel_sys_navigation')

    default_rviz_config = PathJoinSubstitution([bringup_pkg, 'rviz', 'nav2_default_view.rviz'])

    # Declare arguments
    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo without GUI')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2 interface')
    use_nav_arg = DeclareLaunchArgument('use_nav', default_value='false', description='Launch Nav2 navigation stack in sim')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.05', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw orientation')

    # 1. Simulation Bridge / World (Spawns robot, Gazebo plugins, sensor bridges)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([sim_pkg, 'launch', 'sim.launch.py'])),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'use_sim_time': 'true',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )

    # 2. Navigation (Toggled via use_nav argument)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'navigation.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav'))
    )

    # 3. RViz2 Visualizer
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
        headless_arg,
        use_rviz_arg,
        use_nav_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        sim_launch,
        navigation_launch,
        rviz_node
    ])
