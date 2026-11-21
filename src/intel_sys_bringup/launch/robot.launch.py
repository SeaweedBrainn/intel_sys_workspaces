import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    hardware_pkg = FindPackageShare('intel_sys_hardware')
    desc_pkg = FindPackageShare('intel_sys_description')
    loc_pkg = FindPackageShare('intel_sys_localization')
    nav_pkg = FindPackageShare('intel_sys_navigation')

    # Declare user arguments
    port_arg = DeclareLaunchArgument('port', default_value='/dev/rrc', description='STM32 serial port')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='1000000', description='Serial baudrate')
    autostart_nav2_arg = DeclareLaunchArgument('autostart_nav2', default_value='true', description='Autostart Nav2 lifecycle')
    use_foxglove_arg = DeclareLaunchArgument('use_foxglove', default_value='true', description='Launch Foxglove WebSocket bridge')
    foxglove_port_arg = DeclareLaunchArgument('foxglove_port', default_value='8765', description='Foxglove WebSocket port')

    # 1. Robot Description (URDF and TF state publisher)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([desc_pkg, 'launch', 'robot_description.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 2. Hardware Drivers (STM32 bridge, mecanum controller, LiDAR, RealSense)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([hardware_pkg, 'launch', 'hardware.launch.py'])),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
        }.items()
    )

    # 3. State Estimation & Localization (Point-LIO LiDAR-Inertial Odometry)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([loc_pkg, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'use_sim_time': 'false',
            'lidar_type': '5',
        }.items()
    )

    # 4. Autonomous Navigation Stack (Nav2)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'navigation.launch.py'])),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': LaunchConfiguration('autostart_nav2'),
        }.items()
    )

    # 5. Foxglove Studio WebSocket Bridge
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'send_buffer_limit': 100000000,
            'use_sim_time': False
        }],
        condition=IfCondition(LaunchConfiguration('use_foxglove'))
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        autostart_nav2_arg,
        use_foxglove_arg,
        foxglove_port_arg,
        description_launch,
        hardware_launch,
        # localization_launch,
        # navigation_launch,
        foxglove_bridge_node
    ])
