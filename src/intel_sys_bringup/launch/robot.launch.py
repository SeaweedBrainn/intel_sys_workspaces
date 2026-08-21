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
    hardware_pkg = FindPackageShare('intel_sys_hardware')
    desc_pkg = FindPackageShare('intel_sys_description')
    loc_pkg = FindPackageShare('intel_sys_localization')
    nav_pkg = FindPackageShare('intel_sys_navigation')

    default_rviz_config = PathJoinSubstitution([bringup_pkg, 'rviz', 'nav2_default_view.rviz'])

    # Declare user arguments
    port_arg = DeclareLaunchArgument('port', default_value='/dev/rrc', description='STM32 serial port')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='1000000', description='Serial baudrate')
    use_ekf_arg = DeclareLaunchArgument('use_ekf', default_value='true', description='Run EKF sensor fusion')
    use_point_lio_arg = DeclareLaunchArgument('use_point_lio', default_value='false', description='Run Point-LIO LiDAR odometry')
    autostart_nav2_arg = DeclareLaunchArgument('autostart_nav2', default_value='true', description='Autostart Nav2 lifecycle')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz2 navigation interface')

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

    # 3. State Estimation & Localization (Odometry, EKF, Point-LIO)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([loc_pkg, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ekf': LaunchConfiguration('use_ekf'),
            'use_point_lio': LaunchConfiguration('use_point_lio'),
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

    # 5. Optional RViz2 Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        use_ekf_arg,
        use_point_lio_arg,
        autostart_nav2_arg,
        use_rviz_arg,
        description_launch,
        hardware_launch,
        localization_launch,
        navigation_launch,
        rviz_node
    ])
