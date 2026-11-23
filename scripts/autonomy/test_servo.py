"""Unit tests for the visual servo law (pure logic, no ROS or OpenCV needed)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo import ServoConfig, decide  # noqa: E402


def det(z, bearing=0.0):
    return {'z_m': z, 'bearing_rad': bearing}


def test_no_detection_is_lost_and_zero():
    assert decide(ServoConfig(), None, None) == ('LOST', 0.0, 0.0)
    assert decide(ServoConfig(), det(1.0), 1.0)[0] == 'LOST'   # stale


def test_inside_band_and_heading_holds_still():
    cfg = ServoConfig(target_m=1.0)
    assert decide(cfg, det(1.02, 0.01), 0.05) == ('HOLD', 0.0, 0.0)


def test_far_tag_drives_forward_capped():
    cfg = ServoConfig(target_m=1.0, kp_lin=0.4, max_lin=0.08)
    state, vx, wz = decide(cfg, det(2.0), 0.05)
    assert state == 'APPROACH' and vx == cfg.max_lin and wz == 0.0
    state, vx, _ = decide(cfg, det(1.2), 0.05)
    assert state == 'APPROACH' and 0 < vx < cfg.max_lin


def test_too_close_stops_without_reverse_by_default():
    cfg = ServoConfig(target_m=1.0, min_m=0.35)
    # between min_m and target with reverse off: holds rather than backing up
    assert decide(cfg, det(0.9), 0.05) == ('HOLD', 0.0, 0.0)
    assert decide(cfg, det(0.3), 0.05)[:2] == ('TOO_CLOSE', 0.0)


def test_reverse_when_allowed_is_capped():
    cfg = ServoConfig(target_m=1.0, allow_reverse=True, max_reverse=0.05)
    state, vx, _ = decide(cfg, det(0.6), 0.05)
    assert state == 'BACK' and vx == -0.05


def test_turns_toward_tag_on_the_right_with_negative_wz():
    cfg = ServoConfig(target_m=1.0, kp_yaw=1.0, max_wz=0.3)
    state, vx, wz = decide(cfg, det(1.0, 0.2), 0.05)
    assert state == 'TURN' and vx == 0.0 and wz == -0.2
    _, _, wz_left = decide(cfg, det(1.0, -0.2), 0.05)
    assert wz_left == 0.2
    _, _, capped = decide(cfg, det(1.0, 1.0), 0.05)
    assert capped == -cfg.max_wz


def test_far_beyond_max_range_is_zero():
    cfg = ServoConfig(max_m=4.0)
    assert decide(cfg, det(5.0), 0.05) == ('TOO_FAR', 0.0, 0.0)


def test_hold_distance_default_is_18_inches():
    assert abs(ServoConfig().target_m - 18 * 0.0254) < 1e-3


def test_no_tag_in_view_is_lost_so_the_robot_waits_at_zero():
    # armed with the tag absent: LOST every tick, zero command, until the tag appears
    assert decide(ServoConfig(), None, None) == ('LOST', 0.0, 0.0)
