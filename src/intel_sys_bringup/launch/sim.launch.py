"""The simulated robot with the same stacks and modes as the real one (scripts/robot.sh).

    mode:=explore (default)  Point-LIO, RTAB-Map, Nav2 on its map, explorer (idle until Explore)
    mode:=follow             Point-LIO, Nav2 in odom on the live LiDAR, tag follower (off until
                             Follow); walking_tag:=true adds a marker that walks (loop.sdf only)

Switching modes means relaunching, as on the robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sim_pkg = FindPackageShare('intel_sys_sim')
    loc_pkg = FindPackageShare('intel_sys_localization')
    nav_pkg = FindPackageShare('intel_sys_navigation')
    hw_pkg = FindPackageShare('intel_sys_hardware')

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
    slam_continue_arg = DeclareLaunchArgument('slam_continue', default_value='false',
                                              description='Keep mapping into the saved RTAB-Map database (new session)')
    database_path_arg = DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db', description='RTAB-Map database')
    mode_arg = DeclareLaunchArgument('mode', default_value='explore', choices=['explore', 'follow'],
                                     description='explore: map and explore; follow: tag following without RTAB-Map')
    walking_tag_arg = DeclareLaunchArgument('walking_tag', default_value='false',
                                            description='mode:=follow in loop.sdf: spawn a marker that walks a path')
    walking_tag_wait_arg = DeclareLaunchArgument('walking_tag_wait', default_value='30',
                                                 description='Seconds before the marker starts walking')
    mode = LaunchConfiguration('mode')
    # 'true'/'false' strings: the included launch files compare against them.
    explore = PythonExpression(["'true' if '", mode, "' == 'explore' else 'false'"])
    follow = PythonExpression(["'true' if '", mode, "' == 'follow' else 'false'"])
    slam_camera_arg = DeclareLaunchArgument('slam_camera', default_value='false',
                                            description='RTAB-Map also uses the simulated D435 (visual loop closures)')

    # Who drives, as on the robot: Nav2 on /cmd_vel, a person on /cmd_vel_teleop, the
    # /e_stop lock; the Gazebo bridge follows /cmd_vel_mux.
    cmd_vel_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([hw_pkg, 'launch', 'cmd_vel_mux.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items())

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
            'slam': explore,
            'slam_localization': LaunchConfiguration('slam_localization'),
            'slam_continue': LaunchConfiguration('slam_continue'),
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
            'slam': explore,
        }.items()
    )

    # 4. The mode's own node, from the same launch files the robot uses.
    explorer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'explore.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(explore))
    follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'follow.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(follow))
    walking_tag = Node(
        package='intel_sys_sim',
        executable='sim_tag_mover',
        output='screen',
        arguments=['--wait', LaunchConfiguration('walking_tag_wait')],
        condition=IfCondition(PythonExpression(
            ["'true' if '", mode, "' == 'follow' and '", LaunchConfiguration('walking_tag'), "' == 'true' else 'false'"])))

    # 5. Foxglove Studio WebSocket Bridge
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
        mode_arg,
        walking_tag_arg,
        walking_tag_wait_arg,
        slam_camera_arg,
        slam_localization_arg,
        slam_continue_arg,
        database_path_arg,
        sim_launch,
        cmd_vel_mux,
        localization_launch,
        navigation_launch,
        explorer_launch,
        follow_launch,
        walking_tag,
        foxglove_bridge_node
    ])
