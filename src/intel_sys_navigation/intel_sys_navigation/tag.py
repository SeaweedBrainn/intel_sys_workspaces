"""ArUco tag detection and pose (OpenCV only, no ROS).

From Ahmad's scripts/autonomy/tag.py (ahmad-zenoh, 6f0d28d), trimmed to what tag following
needs: one tag ID from the 5x5 family, its position in the camera optical frame (x right,
y down, z forward), and a printable image of it.

Size convention: marker_m is the printed outer edge of the black border (OpenCV's corners).
"""
import math

import cv2
import numpy as np

TAG_ID = 10
TAG_SIZE_IN = 6.5
MARKER_M = TAG_SIZE_IN * 0.0254          # 0.1651 m
DICT_NAMES = ('DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250', 'DICT_5X5_1000')

_dicts = {}


def _dictionary(name):
    if name not in _dicts:
        _dicts[name] = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    return _dicts[name]


def detect_tag(gray, K, D=None, tag_id=TAG_ID, marker_m=MARKER_M, dict_names=DICT_NAMES):
    """Pose of tag_id in the camera optical frame, or None.

    gray is a single-channel image, K the 3x3 intrinsics, D the distortion (may be None).
    The first dictionary in dict_names that contains the tag wins.
    """
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    D = np.zeros(5) if D is None or np.size(D) == 0 else np.asarray(D, dtype=np.float64)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    for name in dict_names:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, _dictionary(name), parameters=params)
        if ids is None:
            continue
        for corner, marker_id in zip(corners, ids.ravel()):
            if int(marker_id) != tag_id:
                continue
            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([corner], marker_m, K, D)
            x, y, z = (float(v) for v in tvecs[0].ravel())
            return {
                'id': tag_id,
                'dict': name,
                'corners': corner.reshape(4, 2).round(1).tolist(),
                'x_m': x,
                'y_m': y,
                'z_m': z,                                  # forward distance
                'distance_m': math.sqrt(x * x + y * y + z * z),
                'bearing_rad': math.atan2(x, z),           # + means the tag is to the right
            }
    return None


class TagFinder:
    """Finds one tag ID and remembers the dictionary that matched.

    Searching every 5x5 dictionary costs about 4x one search, so after a hit only that
    dictionary is tried; after relock_after frames without a hit, all are tried again.
    """

    def __init__(self, tag_id=TAG_ID, marker_m=MARKER_M, dict_names=DICT_NAMES, relock_after=60):
        self.tag_id = tag_id
        self.marker_m = marker_m
        self.dict_names = tuple(dict_names)
        self.relock_after = relock_after
        self.dict_name = None
        self.misses = 0

    def find(self, gray, K, D=None):
        names = (self.dict_name,) if self.dict_name else self.dict_names
        det = detect_tag(gray, K, D, self.tag_id, self.marker_m, names)
        if det is not None:
            self.dict_name = det['dict']
            self.misses = 0
        else:
            self.misses += 1
            if self.dict_name is not None and self.misses >= self.relock_after:
                self.dict_name = None
                self.misses = 0
        return det


def render_marker(tag_id=TAG_ID, dict_name='DICT_5X5_50', dpi=300, quiet_in=0.5, size_in=TAG_SIZE_IN):
    """Printable image of the tag, size_in across the black border, with a white quiet zone (BGR)."""
    side = round(size_in * dpi)
    marker = cv2.aruco.drawMarker(_dictionary(dict_name), tag_id, side)
    pad = round(quiet_in * dpi)
    canvas = np.full((side + 2 * pad, side + 2 * pad), 255, dtype=np.uint8)
    canvas[pad:pad + side, pad:pad + side] = marker
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
