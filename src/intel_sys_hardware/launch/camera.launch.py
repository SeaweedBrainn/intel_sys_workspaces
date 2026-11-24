"""RealSense D435i driver: color, depth aligned to color, and a depth cloud for Nav2.

Topics match what RTAB-Map (slam.launch.py) and the sim expect:
    /camera/image_raw, /camera/depth/image_raw (aligned to color), /camera/camera_info,
    /camera/depth/color/points (Nav2 STVL).

The robot description publishes the camera frames (camera_link and its optical
frames), so the driver's own TF is off. The IMU and infrared streams stay off.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera', description='Camera namespace'),
        DeclareLaunchArgument('enable_pointcloud', default_value='true', description='Publish pointcloud'),
        # 15 fps is plenty for RTAB-Map keyframes and Nav2 and halves the alignment
        # and cloud CPU on the Jetson; the D435i is on USB 3, so 30 is available.
        DeclareLaunchArgument('profile', default_value='640,480,15',
                              description='width,height,fps for color and depth'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='',
            output='screen',
            parameters=[{
                'camera_name': LaunchConfiguration('camera_name'),
                # The ARM (NEON) build names the filter pointcloud__neon_; set both names.
                'pointcloud.enable': ParameterValue(LaunchConfiguration('enable_pointcloud'), value_type=bool),
                'pointcloud__neon_.enable': ParameterValue(LaunchConfiguration('enable_pointcloud'), value_type=bool),
                'rgb_camera.color_profile': LaunchConfiguration('profile'),
                'depth_module.depth_profile': LaunchConfiguration('profile'),
                'align_depth.enable': True,
                'enable_sync': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'publish_tf': False,
            }],
            remappings=[
                ('/camera/color/image_raw', '/camera/image_raw'),
                ('/camera/aligned_depth_to_color/image_raw', '/camera/depth/image_raw'),
                ('/camera/color/camera_info', '/camera/camera_info'),
            ]
        )
    ])
