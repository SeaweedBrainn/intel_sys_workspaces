from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.2', description='Unitree LiDAR IP address'),
        DeclareLaunchArgument('frame_id', default_value='lidar_link', description='LiDAR frame ID'),
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
            }]
        )
    ])
