import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sim_pkg = FindPackageShare('intel_sys_sim')
    nav_pkg = FindPackageShare('intel_sys_navigation')

    # Declare arguments
    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo without GUI')
    use_nav_arg = DeclareLaunchArgument('use_nav', default_value='false', description='Launch Nav2 navigation stack in sim')
    use_foxglove_arg = DeclareLaunchArgument('use_foxglove', default_value='true', description='Launch Foxglove WebSocket bridge')
    foxglove_port_arg = DeclareLaunchArgument('foxglove_port', default_value='8765', description='Foxglove WebSocket port')
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

    # 3. Foxglove Studio WebSocket Bridge
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'send_buffer_limit': 100000000,
            'use_sim_time': True
        }],
        condition=IfCondition(LaunchConfiguration('use_foxglove'))
    )

    return LaunchDescription([
        headless_arg,
        use_nav_arg,
        use_foxglove_arg,
        foxglove_port_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        sim_launch,
        navigation_launch,
        foxglove_bridge_node
    ])
