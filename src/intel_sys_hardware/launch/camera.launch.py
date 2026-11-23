from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera', description='Camera namespace'),
        DeclareLaunchArgument('enable_pointcloud', default_value='true', description='Publish pointcloud'),
        # width,height,fps. The USB 2.1 port cannot carry colour and depth at 30 fps with the infrared
        # streams on, so the infrared streams are off (they are not used).
        DeclareLaunchArgument('color_profile', default_value='640,480,30', description='Colour stream profile: width,height,fps'),
        DeclareLaunchArgument('depth_profile', default_value='640,360,30', description='Depth stream profile: width,height,fps (640x360x30 is in the D435i list)'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='',
            output='screen',
            parameters=[{
                'camera_name': LaunchConfiguration('camera_name'),
                'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
                'align_depth.enable': True,
                'enable_sync': True,
                'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
                'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
                'enable_infra1': False,
                'enable_infra2': False,
            }],
            remappings=[
                ('/camera/color/image_raw', '/camera/image_raw'),
                ('/camera/aligned_depth_to_color/image_raw', '/camera/depth/image_raw'),
                ('/camera/color/camera_info', '/camera/camera_info'),
            ]
        )
    ])
