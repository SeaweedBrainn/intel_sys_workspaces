"""robot_health: /diagnostics for the robot's streams and transforms (Foxglove Diagnostics panel).

Each check is name|topic|type|min_hz|stale_s[|optional]. Optional ones (Point-LIO and
RTAB-Map, which run per session; the camera, which can be unplugged) warn instead of
erroring when silent. /map is not checked: RTAB-Map republishes it only when the map
changes, so it is quiet whenever the robot stands still.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CHECKS = [
    'STM32 IMU|/imu_raw|sensor_msgs/msg/Imu|50|1.0',
    'battery|/battery|std_msgs/msg/UInt16|0.2|15',
    'LiDAR IMU|/lidar/imu|sensor_msgs/msg/Imu|100|1.0',
    'Point-LIO odometry|/odom|nav_msgs/msg/Odometry|6|1.0|optional',
    'camera|/camera/camera_info|sensor_msgs/msg/CameraInfo|10|1.0|optional',
    'RTAB-Map|/rtabmap/info|rtabmap_msgs/msg/Info|0.5|5|optional',
]
TF_CHECKS = [
    'odom|base_footprint|0.5|optional',
    'map|odom|2.0|optional',
]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='intel_sys_hardware',
            executable='robot_health',
            name='robot_health',
            output='screen',
            parameters=[{
                'checks': CHECKS,
                'tf_checks': TF_CHECKS,
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
