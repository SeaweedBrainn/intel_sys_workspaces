from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera', description='Camera namespace'),
        DeclareLaunchArgument('enable_pointcloud', default_value='true', description='Publish pointcloud'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace=LaunchConfiguration('camera_name'),
            output='screen',
            parameters=[{
                'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
                'align_depth.enable': True,
                'enable_sync': True,
            }]
        )
    ])
