import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('intel_sys_localization')

    default_point_lio_params = PathJoinSubstitution([pkg_share, 'config', 'point_lio.yaml'])

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock')
    lidar_type_arg = DeclareLaunchArgument('lidar_type', default_value='5', description='Point-LIO LiDAR Type (2: VELO16, 5: UNILIDAR)')

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
            default_point_lio_params,
            {
                'use_sim_time': use_sim_time,
                'preprocess.lidar_type': lidar_type
            }
        ],
        remappings=[
            ('/lidar/points', '/lidar/points'),
            ('/lidar/imu', '/lidar/imu'),
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

    return LaunchDescription([
        use_sim_time_arg,
        lidar_type_arg,
        lidar_body_filter_node,
        point_lio_node,
        point_lio_frame_adapter_node
    ])
