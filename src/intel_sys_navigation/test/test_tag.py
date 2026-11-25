"""ArUco detection on rendered markers (from Ahmad's scripts/autonomy/test_tag.py)."""
import cv2
import numpy as np

from intel_sys_navigation import tag

K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])


def _scene(marker_px, cx=320, cy=240, shape=(480, 640), marker_id=tag.TAG_ID):
    """Grey image with a marker of marker_px side centred at (cx, cy)."""
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    m = cv2.aruco.drawMarker(dic, marker_id, marker_px)
    img = np.full(shape, 200, dtype=np.uint8)
    y0, x0 = cy - marker_px // 2, cx - marker_px // 2
    img[y0:y0 + marker_px, x0:x0 + marker_px] = m
    return img


def test_centred_tag_is_found_with_right_distance():
    # 200 px side at f=600 px and a 0.1651 m tag: z = f * size / px = 0.4953 m
    det = tag.detect_tag(_scene(200), K)
    assert det is not None and det['id'] == tag.TAG_ID
    assert det['dict'] == 'DICT_5X5_50'
    assert abs(det['z_m'] - 600 * tag.MARKER_M / 200) < 0.03
    assert abs(det['bearing_rad']) < 0.02


def test_offset_tag_gives_bearing_to_the_right():
    det = tag.detect_tag(_scene(120, cx=460), K)
    assert det is not None and det['bearing_rad'] > 0.2


def test_other_ids_are_ignored():
    assert tag.detect_tag(_scene(200, marker_id=3), K) is None


def test_blank_image_has_no_tag():
    assert tag.detect_tag(np.full((480, 640), 200, np.uint8), K) is None


def test_finder_locks_the_matching_dictionary():
    f = tag.TagFinder()
    assert f.find(_scene(200), K) is not None
    assert f.dict_name == 'DICT_5X5_50'


def test_printable_marker_is_6p5_inch_at_300dpi():
    img = tag.render_marker(dpi=300, quiet_in=0.5)
    side = round(6.5 * 300)
    assert img.shape[0] == img.shape[1] == side + 2 * 150
    inner = img[150:150 + side, 150:150 + side, 0]
    assert inner[0, 0] == 0             # black border starts at the marker edge
    assert img[0, 0, 0] == 255          # white quiet zone


def test_printable_marker_is_detected():
    img = cv2.cvtColor(tag.render_marker(dpi=60), cv2.COLOR_BGR2GRAY)
    small = cv2.resize(img, (480, 480))
    det = tag.detect_tag(small, K)
    assert det is not None and det['id'] == tag.TAG_ID
