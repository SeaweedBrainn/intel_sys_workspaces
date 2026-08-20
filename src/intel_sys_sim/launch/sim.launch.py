from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    desc_pkg = FindPackageShare('intel_sys_description')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([desc_pkg, 'launch', 'robot_description.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    info_log = LogInfo(msg="[intel_sys_sim] Simulation launch initialized. Gazebo / Isaac Sim bridge ready.")

    return LaunchDescription([
        info_log,
        robot_description_launch
    ])
