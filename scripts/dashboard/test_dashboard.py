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



def _image(encoding, h, w, step, data):
    import types
    return types.SimpleNamespace(encoding=encoding, height=h, width=w, step=step, data=data)


def test_cloud_payload_is_cached_and_bounded():
    import time
    s = fresh()
    s.odom = (1.0, 2.0, 0.5, 0, 0, 0)
    now = time.monotonic()
    rng = np.random.default_rng(1)
    for i in range(100):
        s.cloud_acc.add(now - 1.0 + i * 0.001, rng.uniform(-5, 5, (1500, 3)).astype(np.float32))
    a = dashboard.cloud_payload(s)
    b = dashboard.cloud_payload(s)
    assert a is b                       # second request inside the TTL reuses the build
    assert a['accumulated'] and a['n'] <= dashboard.CLOUD_MAX
    dashboard.invalidate(s, 'cloud')
    assert dashboard.cloud_payload(s) is not a


def test_map_payload_cache_is_invalidated_on_edit():
    import types
    from waypoint import WaypointNav
    s = fresh()
    node = types.SimpleNamespace(nav=WaypointNav())
    assert dashboard.map_payload(s, node)['waypoints'] == []
    node.nav.set_points([(1.0, 1.0)])
    assert dashboard.map_payload(s, node)['waypoints'] == []   # still inside the TTL
    dashboard.invalidate(s, 'map')
    assert dashboard.map_payload(s, node)['waypoints'] == [[1.0, 1.0]]


def test_centre_depth_is_median_of_valid_patch_pixels():
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[220:260, 300:340] = 2000      # 2 m patch at the centre
    assert dashboard.depth_patch_m(depth) == 2.0


def test_centre_depth_needs_enough_valid_pixels():
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[240, 320] = 1500              # one valid pixel is not a reading
    assert dashboard.depth_patch_m(depth) is None


def test_centre_depth_ignores_row_padding():
    h, w, step = 4, 60, 128            # 120 bytes of data per row, padded to 128
    raw = np.full((h, step // 2), 9999, dtype=np.uint16)   # padding filled with a wrong value
    raw[:, :w] = 3000                  # 3 m in the real pixels
    msg = _image('16UC1', h, w, step, raw.tobytes())
    assert dashboard.center_depth_m(msg) == 3.0
    big = np.zeros((480, 640), dtype=np.uint16)
    big[:, :] = 3000
    msg = _image('16UC1', 480, 640, 1280, big.tobytes())
    assert dashboard.center_depth_m(msg) == 3.0
    assert dashboard.center_depth_m(_image('rgb8', 480, 640, 1920, big.tobytes())) is None


def test_rgb8_is_converted_to_bgr_for_opencv():
    data = bytes([10, 20, 30, 40, 50, 60])      # two RGB pixels, one row
    img = dashboard.image_to_bgr(_image('rgb8', 1, 2, 6, data))
    assert img.shape == (1, 2, 3)
    assert list(img[0, 0]) == [30, 20, 10] and list(img[0, 1]) == [60, 50, 40]
    assert dashboard.image_to_bgr(_image('mono8', 1, 2, 2, b'\x00\x01')) is None


def test_camera_jpeg_none_without_a_frame():
    assert dashboard.camera_jpeg(fresh()) is None


def test_camera_jpeg_encodes_a_small_preview():
    import pytest
    if dashboard.cv2 is None:
        pytest.skip('cv2 not available')
    s = fresh()
    rgb = np.full((480, 640, 3), 128, dtype=np.uint8)
    s.color_msg = _image('rgb8', 480, 640, 640 * 3, rgb.tobytes())
    jpg = dashboard.camera_jpeg(s)
    assert jpg[:2] == b'\xff\xd8'      # JPEG start marker
    decoded = dashboard.cv2.imdecode(np.frombuffer(jpg, np.uint8), dashboard.cv2.IMREAD_COLOR)
    assert decoded.shape[1] == dashboard.CAMERA_PREVIEW_W


def test_status_reports_every_stream_and_marks_silent_ones_stale():
    import time
    s = fresh()
    p = dashboard.status_payload(s)
    assert set(p['sensors']) == set(dashboard.TOPICS)
    assert all(v['stale'] and v['age_s'] is None for v in p['sensors'].values())
    s.mark('depth')
    p = dashboard.status_payload(s)
    assert p['sensors']['depth']['stale'] is False and p['sensors']['depth']['age_s'] < 1.0
    assert p['camera']['depth_center_m'] is None
    s.stamps['camera'].append(time.monotonic() - dashboard.STALE_S - 1)
    assert dashboard.status_payload(s)['sensors']['camera']['stale'] is True


def test_status_has_servo_tag_and_startup_checks():
    s = fresh()
    p = dashboard.status_payload(s)
    assert p['servo']['mode'] == 'off' and p['servo']['state'] == 'IDLE'
    assert p['servo']['target_m'] == 0.4572   # 18 in
    assert p['tag'] is None and p['tf_lidar'] is False and p['camera']['intrinsics'] is False
    s.servo_mode, s.tag, s.tag_t = 'dry', {'id': 6, 'dict': 'DICT_7X7_50', 'z_m': 1.2, 'x_m': 0.0,
                                           'distance_m': 1.2, 'bearing_rad': 0.1}, dashboard.time.monotonic()
    p = dashboard.status_payload(s)
    assert p['servo']['mode'] == 'dry' and p['tag']['id'] == 6 and p['tag']['bearing_deg'] == 5.7


def test_servo_command_flip_settings_are_independent():
    assert dashboard.servo_to_cmd_vel(0.05, -0.2, flip_forward=True, flip_yaw=False) == (-0.05, -0.2)
    assert dashboard.servo_to_cmd_vel(0.05, -0.2, flip_forward=False, flip_yaw=True) == (0.05, 0.2)
    assert dashboard.servo_to_cmd_vel(0.05, -0.2, flip_forward=False, flip_yaw=False) == (0.05, -0.2)
    assert dashboard.SERVO_FLIP_FORWARD is True and dashboard.SERVO_FLIP_YAW is False   # current settings
