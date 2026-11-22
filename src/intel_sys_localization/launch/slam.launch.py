"""RTAB-Map on top of Point-LIO (Phase 6 in docs/ROADMAP.md).

Expects odom -> base_footprint on TF (published by point_lio_frame_adapter) and
the body-filtered cloud on /lidar/points. Publishes map -> odom, /map (2D grid
for Nav2) and /rtabmap/* (graph, 3D cloud map).

    mapping (default):  new_map:=true wipes the database on start.
    localization:       localization:=true new_map:=false re-localizes on the
                        saved database without adding to it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('intel_sys_localization'), 'config', 'rtabmap.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    new_map = LaunchConfiguration('new_map')
    cloud_topic = LaunchConfiguration('cloud_topic')

    # In localization mode, freeze the map and load every saved node at start.
    # RTAB-Map declares its parameters as strings; keep launch from making bools.
    mode_params = {
        'Mem/IncrementalMemory': ParameterValue(PythonExpression(
            ["'false' if '", localization, "' == 'true' else 'true'"]), value_type=str),
        'Mem/InitWMWithAllNodes': ParameterValue(localization, value_type=str),
        # Without BoW (no camera) RTAB-Map cannot relocalize globally. By default
        # it assumes the robot restarts at the LAST pose of the mapping session;
        # assume the map origin (where mapping began) instead. Start the robot
        # there, or publish a guess on /rtabmap/initialpose.
        'RGBD/StartAtOrigin': ParameterValue(localization, value_type=str),
    }

    common = dict(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=[
            params_file,
            mode_params,
            {'use_sim_time': use_sim_time, 'database_path': database_path},
        ],
        remappings=[
            ('scan_cloud', cloud_topic),
            ('map', '/map'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Re-localize on the saved map instead of mapping'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db',
                              description='RTAB-Map database (on the rover: /data/rtabmap.db)'),
        DeclareLaunchArgument('new_map', default_value='true',
                              description='Delete the database on start'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/points'),
        Node(**common, arguments=['--delete_db_on_start'], condition=IfCondition(new_map)),
        Node(**common, condition=UnlessCondition(new_map)),
    ])
