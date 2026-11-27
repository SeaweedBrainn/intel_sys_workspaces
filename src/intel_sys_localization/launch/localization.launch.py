from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('intel_sys_localization')

    default_point_lio_params = PathJoinSubstitution([pkg_share, 'config', 'point_lio.yaml'])

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock')
    lidar_type_arg = DeclareLaunchArgument('lidar_type', default_value='5', description='Point-LIO LiDAR Type (2: VELO16, 5: UNILIDAR)')

    point_lio_params_arg = DeclareLaunchArgument(
        'point_lio_params', default_value=default_point_lio_params, description='Point-LIO parameters YAML')
    slam_camera_arg = DeclareLaunchArgument(
        'slam_camera', default_value='false', description='RTAB-Map also uses the D435 for visual loop closures')
    slam_arg = DeclareLaunchArgument('slam', default_value='false', description='Also run RTAB-Map (map -> odom, /map)')
    slam_localization_arg = DeclareLaunchArgument(
        'slam_localization', default_value='false',
        description='RTAB-Map localizes on the saved database instead of mapping (keeps it)')
    slam_continue_arg = DeclareLaunchArgument(
        'slam_continue', default_value='false',
        description='Keep mapping into the saved database instead of wiping it: a new session that joins '
                    'the old map once RTAB-Map recognizes a place (needs the camera; LiDAR alone cannot '
                    'recognize places globally)')
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/.ros/rtabmap.db', description='RTAB-Map database')

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_type = LaunchConfiguration('lidar_type')

    # 0. LiDAR Body Filter Node (crops points inside robot bounding box for both Sim and Real)
    lidar_body_filter_node = Node(
        package='intel_sys_localization',
        executable='lidar_body_filter',
        name='lidar_body_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/lidar/points_raw',
            'output_topic': '/lidar/points',
            'min_x': -0.30,
            'max_x': 0.08,
            'min_y': -0.18,
            'max_y': 0.18,
            'min_z': -0.20,
            'max_z': 0.12,
        }]
    )

    # 1. Point-LIO LiDAR-Inertial Odometry Node (publishes raw topics with legacy frames)
    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        output='screen',
        parameters=[
            LaunchConfiguration('point_lio_params'),
            {
                'use_sim_time': use_sim_time,
                'preprocess.lidar_type': lidar_type
            }
        ],
        remappings=[
            ('/aft_mapped_to_init', '/point_lio/raw_odom'),
            ('/cloud_registered', '/point_lio/raw_cloud_registered'),
            ('/path', '/point_lio/raw_path'),
        ]
    )

    # 2. Point-LIO Frame Adapter Node (publishes standard /odom, /point_lio/odom and odom -> base_footprint TF)
    point_lio_frame_adapter_node = Node(
        package='intel_sys_localization',
        executable='point_lio_frame_adapter',
        name='point_lio_frame_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'child_frame': 'lidar_link',
            'publish_tf': True,
            'raw_odom_topic': '/point_lio/raw_odom',
            'odom_topic': '/odom',
            'raw_cloud_topic': '/point_lio/raw_cloud_registered',
            'cloud_topic': '/point_lio/cloud_registered',
            'raw_path_topic': '/point_lio/raw_path',
            'path_topic': '/point_lio/path',
        }]
    )

    # 3. RTAB-Map on the LiDAR odometry (owns map -> odom)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'slam.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': LaunchConfiguration('slam_localization'),
            # Mapping starts a fresh database unless slam_continue; localization keeps it.
            'new_map': PythonExpression(["'false' if '", LaunchConfiguration('slam_localization'), "' == 'true' or '",
                                         LaunchConfiguration('slam_continue'), "' == 'true' else 'true'"]),
            'database_path': LaunchConfiguration('database_path'),
            'camera': LaunchConfiguration('slam_camera'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        lidar_type_arg,
        point_lio_params_arg,
        slam_arg,
        slam_camera_arg,
        slam_localization_arg,
        slam_continue_arg,
        database_path_arg,
        lidar_body_filter_node,
        point_lio_node,
        point_lio_frame_adapter_node,
        slam_launch,
    ])
