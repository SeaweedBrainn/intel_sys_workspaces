"""Unit tests for dashboard gating (no ROS graph needed).

Run inside the rover container with the workspace overlay sourced:
  python3 -m pytest -q scripts/dashboard/test_dashboard.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import dashboard  # noqa: E402

ZERO = (0.0, 0.0, 0.0)


def fresh():
    return dashboard.State()


def test_idle_publishes_nothing():
    assert fresh().manual_command(100.0) is None


def test_active_manual_drive_is_published():
    s = fresh()
    s.drive, s.drive_t = (0.05, 0.0, 0.0), 100.0
    assert s.manual_command(100.1) == (0.05, 0.0, 0.0)


def test_manual_drive_expires_to_zeros_then_quiet():
    s = fresh()
    s.drive, s.drive_t = (0.05, 0.0, 0.0), 100.0
    assert s.manual_command(100.05) == (0.05, 0.0, 0.0)  # a tick while held
    now = 100.0 + dashboard.DEADMAN_S + 0.01
    outs = [s.manual_command(now + i * 0.1) for i in range(5)]
    assert outs[:3] == [ZERO, ZERO, ZERO]
    assert outs[3:] == [None, None]


def test_stop_request_sends_zeros_not_silence():
    s = fresh()
    s.drive, s.drive_t = (0.1, 0.0, 0.0), 100.0
    assert s.manual_command(100.05) == (0.1, 0.0, 0.0)
    s.drive, s.drive_t = ZERO, 100.1                     # what /api/stop does
    outs = [s.manual_command(100.1 + i * 0.1) for i in range(4)]
    assert outs[:3] == [ZERO, ZERO, ZERO]
    assert outs[3] is None


def test_caps_are_within_envelope():
    assert dashboard.MAX_LIN <= 0.15
    assert dashboard.MAX_ANG <= 0.8
    assert dashboard.DEADMAN_S <= 0.5


def test_rosout_levels_compare_as_ints():
    # Regression: comparing msg.level against Log.WARN (bytes) raised TypeError in the
    # rosout callback and silently killed the executor thread.
    from rcl_interfaces.msg import Log
    m = Log()
    m.level = 30
    assert int(m.level) >= dashboard.LOG_WARN
    m.level = 20
    assert int(m.level) < dashboard.LOG_WARN
    assert dashboard.LOG_ERROR == 40


def test_nearest_needs_several_points():
    assert dashboard.nearest(np.array([0.8])) == math.inf
    assert dashboard.nearest(np.array([0.9, 0.7, 1.2])) == 1.2
    assert dashboard.nearest(np.array([])) == math.inf


def test_autonomy_command_flipped_when_lidar_faces_rear():
    # Explorer "forward" means toward what the LiDAR sees, which is the rover's rear.
    assert dashboard.autonomy_to_cmd_vel(0.1, 0.04, 0.4, faces_rear=True) == (-0.1, -0.04, 0.4)
    assert dashboard.autonomy_to_cmd_vel(0.1, 0.04, 0.4, faces_rear=False) == (0.1, 0.04, 0.4)


def test_lidar_pose_rotates_yaw_when_lidar_faces_rear():
    assert dashboard.lidar_pose(None) is None
    x, y, yaw = dashboard.lidar_pose((1.0, 2.0, 0.5, 0, 0, 0), faces_rear=True)
    assert (x, y) == (1.0, 2.0) and math.isclose(yaw, 0.5 - math.pi)
    assert dashboard.lidar_pose((1.0, 2.0, 0.5, 0, 0, 0), faces_rear=False) == (1.0, 2.0, 0.5)


def test_map_payload_reports_pose_goals_and_trail():
    import json
    import types
    from waypoint import WaypointNav
    s = fresh()
    s.odom = (1.0, 2.0, 0.5, 0, 0, 0)
    s.map.add_pose(1.0, 2.0)
    node = types.SimpleNamespace(nav=WaypointNav())
    node.nav.set_points([(3.0, 4.0)])
    p = dashboard.map_payload(s, node)
    assert p['waypoints'] == [[3.0, 4.0]]
    assert p['pose']['x'] == 1.0 and p['pose']['yaw_odom'] == 0.5
    assert p['pose']['yaw_lidar'] == dashboard.lidar_pose(s.odom)[2]
    assert p['trail'] == [[1.0, 2.0]]
    assert p['nav'] == {'mode': 'off', 'state': 'IDLE', 'index': 0, 'arrived': 0, 'halted': None}
    json.dumps(p)


def test_map_payload_without_odometry_has_no_pose():
    import types
    from waypoint import WaypointNav
    s = fresh()
    node = types.SimpleNamespace(nav=WaypointNav())
    assert dashboard.map_payload(s, node)['pose'] is None


def test_waypoint_limits_are_sane():
    assert dashboard.MAX_WAYPOINTS <= 100
    assert 0 < dashboard.MAX_COORD <= 500


def test_token_roles():
    assert dashboard.token_role('c', 'c', 'v') == 'control'
    assert dashboard.token_role('v', 'c', 'v') == 'view'
    assert dashboard.token_role('x', 'c', 'v') is None
    assert dashboard.token_role('', 'c', 'v') is None
    assert dashboard.token_role('', '', '') is None       # empty never matches
    assert dashboard.token_role('c', 'c', '') == 'control'
    assert dashboard.token_role('anything', 'c', '') is None  # no view token configured
    assert dashboard.token_role('v', '', 'v') == 'view'

def test_decimate_thins_and_keeps_small_clouds():
    big = np.arange(30000, dtype=np.float32).reshape(-1, 3)
    out = dashboard.decimate(big, 4000)
    assert 0 < out.shape[0] <= 4000
    assert out.shape[1] == 3
    small = big[:10]
    assert dashboard.decimate(small, 4000).shape == small.shape
    assert dashboard.decimate(np.zeros((0, 3), np.float32), 4000).shape == (0, 3)

def test_base_and_odom_frames_are_inverse():
    import numpy as np
    pts = np.array([[1.0, 0.5, 0.2], [-0.3, 2.0, -0.1]], dtype=np.float32)
    pose = (3.0, -1.0, 0.7)
    back = dashboard.odom_to_base(dashboard.base_to_odom(pts, pose), pose)
    assert np.allclose(back, pts, atol=1e-5)
    w = dashboard.base_to_odom(np.array([[1.0, 0.0, 0.0]], np.float32), (0.0, 0.0, math.pi / 2))
    assert np.allclose(w[0], [0.0, 1.0, 0.0], atol=1e-6)   # 1 m ahead after a 90 deg turn is +y



def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def test_dome_facing_from_urdf_joint():
    # current lidar_joint (dome along +x, the sensor-side front) and the same mount turned around
    assert not dashboard.dome_faces_rear(_rpy(1.868822, -1.219393, 1.308338))
    assert dashboard.dome_faces_rear(_rpy(1.868822, -1.219393, -1.833255))


def test_lidar_side_frame_puts_seen_points_ahead():
    behind = np.array([[-1.5, 0.3, 0.2]])        # seen by a rear-facing dome, in base_footprint
    out = dashboard.to_lidar_side_frame(behind, tf_rear=True)
    np.testing.assert_allclose(out, [[1.5, -0.3, 0.2]])
    np.testing.assert_allclose(dashboard.to_lidar_side_frame(behind, tf_rear=False), behind)


def test_frame_conflict_only_when_urdf_rear_and_no_flip():
    assert dashboard.frame_conflict(True, faces_rear=False)
    assert not dashboard.frame_conflict(True, faces_rear=True)
    assert not dashboard.frame_conflict(False, faces_rear=True)
    assert not dashboard.frame_conflict(False, faces_rear=False)


def test_flip_follows_the_running_controller():
    assert dashboard.resolve_flip(True) is False       # sensor-side front: drive straight
    assert dashboard.resolve_flip(False) is True       # old controller: rotate 180 deg
    assert dashboard.resolve_flip(None) is None        # no answer: refuse autonomy
    assert dashboard.resolve_flip(None, force=True) is True
    assert dashboard.resolve_flip(True, force=True) is True
