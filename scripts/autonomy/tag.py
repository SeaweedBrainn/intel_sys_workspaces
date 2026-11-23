"""ArUco detection and pose for visual servoing (OpenCV ArUco, no ROS).

  * detect_tag(gray, K, D): pose of one tag ID in the camera optical frame
    (x right, y down, z forward), or None. Tries every 7x7 dictionary, so the printed
    dictionary does not need to be known in advance. The matching one is reported.
  * overlay(img, det): draws the detected outline and a label on a BGR image.

Size convention: marker_m is the printed outer edge of the black border (OpenCV's corners).
"""
import math

import cv2
import numpy as np

TAG_ID = 10                       # servo target: 5x5 marker, ID 10 (control tag)
TAG_DICT_NAMES = ('DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250', 'DICT_5X5_1000')
TAG_SIZE_IN = 6.5
MARKER_M = TAG_SIZE_IN * 0.0254          # 0.1651 m
DICT_NAMES = ('DICT_7X7_50', 'DICT_7X7_100', 'DICT_7X7_250', 'DICT_7X7_1000')   # printable 7x7 family

_dicts = None


def _load():
    global _dicts
    if _dicts is None:
        _dicts = [(n, cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, n)))
                  for n in TAG_DICT_NAMES + DICT_NAMES]
    return _dicts


def detect_tag(gray, K, D=None, tag_id=TAG_ID, marker_m=MARKER_M, dict_names=TAG_DICT_NAMES):
    """Pose of tag_id, or None. K is the 3x3 intrinsic matrix, D the distortion (may be None)."""
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    D = np.zeros(5) if D is None or np.size(D) == 0 else np.asarray(D, dtype=np.float64)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    for name, dic in _load():
        if name not in dict_names:
            continue
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dic, parameters=params)
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
                'x_m': round(x, 4),
                'y_m': round(y, 4),
                'z_m': round(z, 4),                       # forward distance: what the servo holds
                'distance_m': round(math.sqrt(x * x + y * y + z * z), 4),
                'bearing_rad': round(math.atan2(x, z), 4),  # + means the tag is to the right
            }
    return None


class TagFinder:
    """Finds one tag ID and remembers the dictionary that matched.

    Searching all four 7x7 dictionaries costs about 4x a single search, so after a hit only that
    dictionary is tried. After relock_after frames without a hit, all dictionaries are tried again.
    """

    def __init__(self, tag_id=TAG_ID, marker_m=MARKER_M, relock_after=60):
        self.tag_id = tag_id
        self.marker_m = marker_m
        self.relock_after = relock_after
        self.dict_name = None
        self.misses = 0

    def find(self, gray, K, D=None):
        names = (self.dict_name,) if self.dict_name else TAG_DICT_NAMES
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


SCAN_DICT_NAMES = ('DICT_4X4_50', 'DICT_5X5_50', 'DICT_6X6_50', 'DICT_7X7_50')


def scan_markers(gray):
    """Every ArUco marker in the frame, from one dictionary per family, with its family name.

    Diagnostic only: it tells you what the camera actually sees when the target ID is absent.
    Slower than detect_tag because it runs four searches, so the caller should throttle it.
    """
    out = []
    params = cv2.aruco.DetectorParameters_create()
    for name in SCAN_DICT_NAMES:
        dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dic, parameters=params)
        if ids is None:
            continue
        for c, i in zip(corners, ids.ravel()):
            out.append({'dict': name, 'id': int(i), 'corners': c.reshape(4, 2).round(1).tolist()})
    return out


def overlay(img, det, text=True):
    """Draw the tag outline, its ID and distance on a BGR image (in place). Returns img."""
    if det is None:
        return img
    pts = np.array(det['corners'], dtype=np.int32)
    cv2.polylines(img, [pts], True, (0, 255, 0), 2)
    if text:
        label = f"ID {det['id']}  z {det['z_m']:.2f} m  bearing {math.degrees(det['bearing_rad']):+.1f} deg"
        x, y = int(pts[:, 0].min()), max(14, int(pts[:, 1].min()) - 6)
        cv2.putText(img, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return img


def render_marker(tag_id=6, dict_name='DICT_7X7_50', dpi=300, quiet_in=0.5):
    """Printable image of the tag at TAG_SIZE_IN, with a white quiet zone. Returns a BGR array."""
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    side = round(TAG_SIZE_IN * dpi)
    marker = cv2.aruco.drawMarker(dic, tag_id, side)   # black border is part of the marker
    pad = round(quiet_in * dpi)
    canvas = np.full((side + 2 * pad, side + 2 * pad), 255, dtype=np.uint8)
    canvas[pad:pad + side, pad:pad + side] = marker
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
