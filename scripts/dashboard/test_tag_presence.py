"""Tag presence tests: the ArUco tag comes and goes in frame, and the pipeline must follow.

Runs the real detector (scripts/autonomy/tag.py), the real dictionary lock, the real tag worker
thread and the real servo law on rendered frames. Needs OpenCV with aruco; run in the rover container:
  python3 -m pytest -q scripts/dashboard/test_tag_presence.py
"""
import os
import sys
import threading
import time
import types

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
cv2 = pytest.importorskip('cv2')
import dashboard  # noqa: E402
from tag import TAG_ID, TagFinder, detect_tag, scan_markers  # noqa: E402
from servo import ServoConfig, decide  # noqa: E402

K = np.array([[610.0, 0, 320], [0, 610.0, 240], [0, 0, 1]])
DICT_5X5_50 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
DICT_7X7_50 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)   # a 'wrong' printed tag


def frame(tag_id=TAG_ID, side=180, cx=320, cy=240, dict_=DICT_5X5_50):
    """RGB uint8 frame (480x640), grey background, one marker of `side` px centred at (cx, cy)."""
    gray = np.full((480, 640), 190, np.uint8)
    if tag_id is not None:
        m = cv2.aruco.drawMarker(dict_, tag_id, side)
        gray[cy - side // 2:cy + side // 2, cx - side // 2:cx + side // 2] = m
    return np.stack([gray, gray, gray], axis=-1)


def as_image_msg(rgb):
    h, w, _ = rgb.shape
    data = rgb.tobytes()
    return types.SimpleNamespace(encoding='rgb8', height=h, width=w, step=w * 3, data=data)


# ---- sequence through the detector and the lock -------------------------------------------

def test_tag_present_absent_present_is_found_each_time_it_is_visible():
    f = TagFinder()
    gray = lambda rgb: cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)  # noqa: E731
    seq = [True, True, False, False, True, False, True]
    results = [f.find(gray(frame() if vis else frame(tag_id=None)), K) for vis in seq]
    for vis, det in zip(seq, results):
        assert (det is not None) == vis
    assert all(r['id'] == TAG_ID and r['dict'] == 'DICT_5X5_50' for r in results if r)
    assert f.dict_name == 'DICT_5X5_50'           # locked after the first hit


def test_lock_is_released_after_enough_misses_and_relocks_on_return():
    f = TagFinder(relock_after=3)
    gray_tag = cv2.cvtColor(frame(), cv2.COLOR_RGB2GRAY)
    gray_none = cv2.cvtColor(frame(tag_id=None), cv2.COLOR_RGB2GRAY)
    assert f.find(gray_tag, K) is not None and f.dict_name == 'DICT_5X5_50'
    for _ in range(2):
        f.find(gray_none, K)
    assert f.dict_name == 'DICT_5X5_50'           # not yet: below relock_after
    f.find(gray_none, K)
    assert f.dict_name is None                    # released, will search every dictionary
    assert f.find(gray_tag, K) is not None and f.dict_name == 'DICT_5X5_50'


def test_other_dictionary_tag_is_not_mistaken_for_target():
    rgb = frame(tag_id=6, dict_=DICT_7X7_50)     # the printed 7x7 ID 6 is not the target any more
    assert detect_tag(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), K, None) is None
    assert any(m['dict'] == 'DICT_7X7_50' and m['id'] == 6
               for m in scan_markers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)))


def test_small_or_distant_tag_is_still_found_and_reports_distance():
    det = detect_tag(cv2.cvtColor(frame(side=100), cv2.COLOR_RGB2GRAY), K, None)
    assert det is not None
    assert abs(det['z_m'] - 610 * 0.1651 / 100) < 0.08


# ---- the worker thread, end to end ------------------------------------------------------

def _run_worker(state, seconds):
    stop = threading.Event()
    th = threading.Thread(target=dashboard.tag_worker, args=(state, stop), daemon=True)
    th.start()
    time.sleep(seconds)
    stop.set()
    th.join(timeout=2)


def _wait_for(pred, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_worker_sees_tag_appear_then_goes_lost_when_it_leaves():
    s = dashboard.State()
    s.cam_K, s.cam_D = K, None
    stop = threading.Event()
    th = threading.Thread(target=dashboard.tag_worker, args=(s, stop), daemon=True)
    th.start()
    try:
        s.color_msg = as_image_msg(frame())
        assert _wait_for(lambda: s.tag is not None), 'worker never detected the tag'
        assert s.tag['id'] == TAG_ID
        seen_at = s.tag_t
        # tag leaves the frame
        s.color_msg = as_image_msg(frame(tag_id=None))
        time.sleep(0.5)
        assert s.tag_t == seen_at                  # no new detection stamped
        age = time.monotonic() - s.tag_t
        assert age > ServoConfig().stale_s - 0.1
        state, vx, wz = decide(ServoConfig(), s.tag, age)
        assert state == 'LOST' and vx == 0.0 and wz == 0.0
        # tag returns
        s.color_msg = as_image_msg(frame())
        assert _wait_for(lambda: s.tag_t > seen_at), 'worker did not see the tag come back'
        state, _, _ = decide(ServoConfig(), s.tag, time.monotonic() - s.tag_t)
        assert state != 'LOST'
    finally:
        stop.set()
        th.join(timeout=2)


def test_worker_reports_other_markers_while_target_is_absent():
    s = dashboard.State()
    s.cam_K, s.cam_D = K, None
    s.color_msg = as_image_msg(frame(tag_id=6, dict_=DICT_7X7_50))
    stop = threading.Event()
    th = threading.Thread(target=dashboard.tag_worker, args=(s, stop), daemon=True)
    th.start()
    try:
        assert _wait_for(lambda: s.markers_t is not None), 'no diagnostic scan ran'
        assert s.tag is None
        assert {'dict': 'DICT_7X7_50', 'id': 6} in [{'dict': m['dict'], 'id': m['id']} for m in s.markers]
    finally:
        stop.set()
        th.join(timeout=2)


def test_worker_survives_a_bad_frame_and_keeps_going():
    s = dashboard.State()
    s.cam_K, s.cam_D = K, None
    s.color_msg = types.SimpleNamespace(encoding='rgb8', height=2, width=2, step=6, data=b'\x00' * 12)
    stop = threading.Event()
    th = threading.Thread(target=dashboard.tag_worker, args=(s, stop), daemon=True)
    th.start()
    try:
        time.sleep(0.2)
        s.color_msg = as_image_msg(frame())
        assert _wait_for(lambda: s.tag is not None), 'worker died after a bad frame'
        assert th.is_alive()
    finally:
        stop.set()
        th.join(timeout=2)


def test_status_reports_tag_age_and_servo_lost_when_tag_absent():
    s = dashboard.State()
    s.cam_K = K
    s.tag = {'id': 6, 'dict': 'DICT_5X5_50', 'z_m': 1.0, 'x_m': 0.0, 'distance_m': 1.0, 'bearing_rad': 0.0}
    s.tag_t = time.monotonic() - 5.0           # seen five seconds ago
    p = dashboard.status_payload(s)
    assert p['tag']['age_s'] >= 4.9
    state, _, _ = decide(ServoConfig(), s.tag, p['tag']['age_s'])
    assert state == 'LOST'
