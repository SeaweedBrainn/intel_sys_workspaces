"""Settings that must agree across packages."""
import importlib.util
from pathlib import Path

import yaml

SRC = Path(__file__).resolve().parents[2]


def slam_params():
    spec = importlib.util.spec_from_file_location('navigation_launch', SRC / 'intel_sys_navigation/launch/navigation.launch.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    with open(SRC / 'intel_sys_navigation/config/nav2_params.yaml') as f:
        return mod.slam_overlay(yaml.safe_load(f))


def test_global_costmap_matches_the_slam_map_resolution():
    """A rolling global costmap samples one /map cell per costmap cell, so a coarser costmap
    loses walls one map cell thick (loop.sdf, 2026-11-26). Change both together."""
    with open(SRC / 'intel_sys_localization/config/rtabmap.yaml') as f:
        rtabmap = yaml.safe_load(f)['/**']['ros__parameters']
    gc = slam_params()['global_costmap']['global_costmap']['ros__parameters']
    assert gc['rolling_window']
    assert abs(gc['resolution'] - float(rtabmap['Grid/CellSize'])) < 1e-9
