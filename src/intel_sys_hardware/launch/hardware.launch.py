from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    hardware_pkg = FindPackageShare('intel_sys_hardware')

    stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([hardware_pkg, 'launch', 'stm32.launch.py']))
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([hardware_pkg, 'launch', 'lidar.launch.py']))
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([hardware_pkg, 'launch', 'camera.launch.py']))
    )

    return LaunchDescription([
        stm32_launch,
        lidar_launch,
        camera_launch
    ])
