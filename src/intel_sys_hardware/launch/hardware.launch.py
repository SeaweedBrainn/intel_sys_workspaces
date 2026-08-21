from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    hardware_pkg = FindPackageShare('intel_sys_hardware')

    port_arg = DeclareLaunchArgument('port', default_value='/dev/rrc', description='STM32 serial port device')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='1000000', description='Serial baudrate')

    stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([hardware_pkg, 'launch', 'stm32.launch.py'])),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
        }.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([hardware_pkg, 'launch', 'lidar.launch.py']))
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([hardware_pkg, 'launch', 'camera.launch.py']))
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        stm32_launch,
        lidar_launch,
        camera_launch
    ])
