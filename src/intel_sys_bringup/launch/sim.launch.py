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
    loc_pkg = FindPackageShare('intel_sys_localization')
    nav_pkg = FindPackageShare('intel_sys_navigation')

    # Declare arguments
    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo without GUI')
    use_foxglove_arg = DeclareLaunchArgument('use_foxglove', default_value='true', description='Launch Foxglove WebSocket bridge')
    foxglove_port_arg = DeclareLaunchArgument('foxglove_port', default_value='8765', description='Foxglove WebSocket port')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.05', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw orientation')
    world_arg = DeclareLaunchArgument('world', default_value='default.sdf', description='World file in intel_sys_sim/worlds (e.g. loop.sdf)')
    slam_localization_arg = DeclareLaunchArgument('slam_localization', default_value='false', description='Localize on a saved RTAB-Map database instead of mapping')
    database_path_arg = DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db', description='RTAB-Map database')
    slam_arg = DeclareLaunchArgument('slam', default_value='true', description='Run RTAB-Map and plan on its /map')
    slam_camera_arg = DeclareLaunchArgument('slam_camera', default_value='false',
                                            description='RTAB-Map also uses the simulated D435 (visual loop closures)')

    # 1. Simulation Bridge / World (Spawns robot, Gazebo plugins, sensor bridges)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([sim_pkg, 'launch', 'sim.launch.py'])),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'world': PathJoinSubstitution([sim_pkg, 'worlds', LaunchConfiguration('world')]),
            'use_sim_time': 'true',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )

    # 2. State Estimation & Localization (Point-LIO LiDAR-Inertial Odometry)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([loc_pkg, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'lidar_type': '5',
            'slam': LaunchConfiguration('slam'),
            'slam_localization': LaunchConfiguration('slam_localization'),
            'slam_camera': LaunchConfiguration('slam_camera'),
            'database_path': LaunchConfiguration('database_path'),
        }.items()
    )

    # 3. Navigation Stack (Nav2)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'navigation.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'slam': LaunchConfiguration('slam'),
        }.items()
    )

    # 4. Foxglove Studio WebSocket Bridge
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
        use_foxglove_arg,
        foxglove_port_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        world_arg,
        slam_arg,
        slam_camera_arg,
        slam_localization_arg,
        database_path_arg,
        sim_launch,
        localization_launch,
        navigation_launch,
        foxglove_bridge_node
    ])
