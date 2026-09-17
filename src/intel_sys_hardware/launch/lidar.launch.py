from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.2', description='Unitree LiDAR IP address'),
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen',
            parameters=[{
                'cloud_frame': 'lidar_link',
                'imu_frame': 'lidar_imu_link',
                'use_system_timestamp': True,
            }],
            remappings=[
                ('/unilidar/cloud', '/lidar/points_raw'),
                ('/unilidar/imu', '/lidar/imu'),
            ]
        )
    ])
