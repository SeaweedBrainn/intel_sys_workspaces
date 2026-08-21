import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('intel_sys_navigation')
    nav2_bringup_pkg = FindPackageShare('nav2_bringup')

    default_nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    default_ekf_params = PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])
    default_point_lio_params = PathJoinSubstitution([pkg_share, 'config', 'point_lio.yaml'])

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup Nav2 stack')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_nav2_params, description='Nav2 params YAML')
    use_ekf_arg = DeclareLaunchArgument('use_ekf', default_value='true', description='Use EKF fusion (odom + point_lio + imu)')
    use_point_lio_arg = DeclareLaunchArgument('use_point_lio', default_value='false', description='Run Point-LIO LiDAR-Inertial Odometry')
    publish_odom_tf_arg = DeclareLaunchArgument('publish_odom_tf', default_value='false', description='Publish odom TF from wheel odom directly')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_ekf = LaunchConfiguration('use_ekf')
    use_point_lio = LaunchConfiguration('use_point_lio')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    # 1. Wheel Odometry Publisher (publishes /odom from mecanum kinematics)
    odom_publisher_node = Node(
        package='intel_sys_navigation',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_tf': publish_odom_tf,
            'odom_topic': 'odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'wheelbase': 0.216,
            'track_width': 0.195,
            'wheel_diameter': 0.097,
        }]
    )

    # 2. Point-LIO LiDAR-Inertial Odometry Node (publishes /point_lio/odom)
    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        output='screen',
        condition=IfCondition(use_point_lio),
        parameters=[default_point_lio_params],
        remappings=[
            ('/unilidar/cloud', '/unilidar/cloud'),
            ('/imu_raw', '/imu_raw'),
            ('/Odometry', '/point_lio/odom'),
        ]
    )

    # 3. Robot Localization (EKF) Node (fuses /odom, /point_lio/odom, and /imu_raw -> publishes odom -> base_footprint TF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[default_ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom/filtered')]
    )

    # 4. Nav2 Stack Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_bringup_pkg, 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        use_ekf_arg,
        use_point_lio_arg,
        publish_odom_tf_arg,
        odom_publisher_node,
        point_lio_node,
        ekf_node,
        nav2_bringup_launch
    ])
