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
    # A rolling window around the robot, so goals in unmapped space still plan (the
    # rover's main job: drive to a waypoint far beyond the map). 30 m at 5 cm was too
    # small: from the far end of the 22 m office.sdf the explorer's home was off it and
    # the trip home never planned (sim, 2026-11-26), so 60 m.
    # The resolution must equal RTAB-Map's Grid/CellSize (test_config.py checks it). In a
    # rolling window the static layer reads one /map cell per costmap cell, so a coarser
    # costmap drops walls one map cell thick: at 10 cm about half of them were missing,
    # and the explorer put goals inside walls (loop.sdf, 2026-11-26).
    # Outdoors (URC) both grow together, e.g. 200 m with 25 cm cells.
    gc['rolling_window'] = True
    gc['width'] = gc['height'] = 60
    gc['resolution'] = 0.05
    gc['plugins'] = ['static_layer', 'stvl_layer', 'inflation_layer']
    gc['static_layer'] = {
        'plugin': 'nav2_costmap_2d::StaticLayer',
        'map_topic': '/map',
        'map_subscribe_transient_local': True,
    }
    params['bt_navigator']['ros__parameters']['global_frame'] = 'map'
    return params


def follow_overlay(params):
    """Following (no RTAB-Map, everything in odom): forget unseen obstacles within 1.5 s.

    The target is an obstacle that moves, and the robot drives along its path. With the
    normal memory (5 s local, 20 s global) each spot the target stood on stayed lethal: the
    follow goal, 0.46 m short of the target, landed on that trail, and NavFn found a plan
    only some of the time (loop.sdf, 2026-11-27). The LiDAR doesn't sweep the low patch
    right in front of the robot, so decay_acceleration doesn't clear it. The long memory
    stays for mapping and exploring, where it keeps a box the robot turned away from.
    """
    for costmap in ('local_costmap', 'global_costmap'):
        params[costmap][costmap]['ros__parameters']['stvl_layer']['voxel_decay'] = 1.5
    return params


def make_nav2(context):
    with open(LaunchConfiguration('params_file').perform(context)) as f:
        params = yaml.safe_load(f)
    # Our behavior tree: Nav2's default with a shorter, slower back-up (see the XML).
    # The path is only known at launch time, so it can't live in the YAML.
    bt_xml = os.path.join(FindPackageShare('intel_sys_navigation').perform(context),
                          'behavior_trees', 'navigate_to_pose.xml')
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml
    if LaunchConfiguration('slam').perform(context) == 'true':
        params = slam_overlay(params)
    else:
        params = follow_overlay(params)
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
        }.items()
    )]


def generate_launch_description():
    pkg_share = FindPackageShare('intel_sys_navigation')
    default_nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup Nav2 stack'),
        DeclareLaunchArgument('params_file', default_value=default_nav2_params, description='Nav2 params YAML'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Plan on RTAB-Map /map in the map frame (needs slam.launch.py running)'),
        OpaqueFunction(function=make_nav2),
    ])
