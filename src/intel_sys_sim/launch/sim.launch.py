import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sim_pkg = FindPackageShare('intel_sys_sim')
    desc_pkg = FindPackageShare('intel_sys_description')
    ros_gz_sim_pkg = FindPackageShare('ros_gz_sim')

    default_world_path = PathJoinSubstitution([sim_pkg, 'worlds', 'default.sdf'])
    default_bridge_config = PathJoinSubstitution([sim_pkg, 'config', 'ros_gz_bridge.yaml'])

    # Declare arguments
    world_arg = DeclareLaunchArgument('world', default_value=default_world_path, description='SDF world file path')
    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo without GUI')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.05', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw orientation')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Environment variables for Gazebo to find meshes
    resource_path = PathJoinSubstitution([desc_pkg, '..'])
    set_gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[resource_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    set_ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[resource_path, ':', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])

    # 1. Gazebo Sim Server + GUI (or headless)
    gz_args = PythonExpression([
        "'-r ' + ('-s ' if '", headless, "' == 'true' else '') + '", world, "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # 2. Robot State Publisher (publishes /robot_description and TF)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([desc_pkg, 'launch', 'robot_description.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Spawn Robot Model in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'intel_sys_robot',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ]
    )

    # 4. ROS-Gazebo Bridge (Bridges /clock, /cmd_vel, /odom, /lidar/points, /scan, /camera/..., /imu_raw, /joint_states)
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': default_bridge_config,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        world_arg,
        headless_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gz_sim,
        robot_description_launch,
        spawn_robot,
        ros_gz_bridge_node
    ])
