import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def slam_overlay(params):
    """Phase 7 (docs/ROADMAP.md): plan globally on RTAB-Map's /map.

    The global costmap moves to the map frame and takes RTAB-Map's grid as a
    static layer, with the live cloud (STVL) on top for obstacles the map does
    not have yet. The local costmap, controller and behaviors stay in odom.
    """
    gc = params['global_costmap']['global_costmap']['ros__parameters']
    gc['global_frame'] = 'map'
    # Sized from /map and growing with it, not a rolling window: a 30 m window
    # centred on the robot could not reach the explorer's home from the far end
    # of a 22 m building (office.sdf, 2026-11-26), so the trip home never planned.
    # The cost: goals beyond the edge of the mapped area are off the costmap and
    # don't plan (unknown cells inside it do, allow_unknown). The explorer only
    # sends goals in known space; tag following runs in odom without this overlay.
    gc['rolling_window'] = False
    for key in ('width', 'height'):
        gc.pop(key, None)
    gc['plugins'] = ['static_layer', 'stvl_layer', 'inflation_layer']
    gc['static_layer'] = {
        'plugin': 'nav2_costmap_2d::StaticLayer',
        'map_topic': '/map',
        'map_subscribe_transient_local': True,
    }
    params['bt_navigator']['ros__parameters']['global_frame'] = 'map'
    return params


def make_nav2(context):
    with open(LaunchConfiguration('params_file').perform(context)) as f:
        params = yaml.safe_load(f)
    # Our behavior tree: Nav2's default without the BackUp recovery (see the XML).
    # The path is only known at launch time, so it can't live in the YAML.
    bt_xml = os.path.join(FindPackageShare('intel_sys_navigation').perform(context),
                          'behavior_trees', 'navigate_to_pose_no_backup.xml')
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml
    if LaunchConfiguration('slam').perform(context) == 'true':
        params = slam_overlay(params)
    fd, params_file = tempfile.mkstemp(prefix='nav2_params_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f)

    nav2_bringup_pkg = FindPackageShare('nav2_bringup')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_pkg, 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': params_file,
            'map': LaunchConfiguration('map'),
        }.items()
    )]


def generate_launch_description():
    pkg_share = FindPackageShare('intel_sys_navigation')
    default_nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup Nav2 stack'),
        DeclareLaunchArgument('params_file', default_value=default_nav2_params, description='Nav2 params YAML'),
        DeclareLaunchArgument('map', default_value='', description='Full path to map yaml file to load (optional)'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Plan on RTAB-Map /map in the map frame (needs slam.launch.py running)'),
        OpaqueFunction(function=make_nav2),
    ])
