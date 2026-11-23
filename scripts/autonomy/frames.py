"""Gravity alignment and a sliding time window for LiDAR clouds (pure logic, no ROS).

  * align_rotation(up): rotation that takes the measured gravity-up direction to +z, so the
    cloud's z is true vertical. Yaw is left as it was.
  * WindowedCloud: points kept for window_s seconds, in a fixed (odometry) frame. Older
    scans expire, so the accumulated picture follows recent geometry only.
"""
import collections
import math

import numpy as np


def align_rotation(up):
    """3x3 rotation R with R @ up = +z (up is any nonzero 3-vector)."""
    u = np.asarray(up, dtype=np.float64)
    u = u / np.linalg.norm(u)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(u, z)
    c = float(np.dot(u, z))
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        # already up, or exactly down (flip about x)
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0.0, -v[2], v[1]],
                   [v[2], 0.0, -v[0]],
                   [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def tilt_deg(up):
    """Angle in degrees between the gravity-up vector and +z."""
    u = np.asarray(up, dtype=np.float64)
    c = float(np.dot(u, [0.0, 0.0, 1.0]) / np.linalg.norm(u))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class WindowedCloud:
    """Rolling buffer of scans: keeps points from the last window_s seconds and at most
    max_points in total. Oldest scans are dropped first, so memory stays bounded."""

    def __init__(self, window_s, max_points=None):
        self.window_s = window_s
        self.max_points = max_points
        self._scans = collections.deque()   # (t, (N, 3) array)
        self._count = 0

    def add(self, t, pts):
        if pts.shape[0]:
            self._scans.append((t, pts))
            self._count += pts.shape[0]
        self.prune(t)

    def prune(self, now):
        while self._scans and (
                now - self._scans[0][0] > self.window_s
                or (self.max_points is not None and self._count > self.max_points)):
            _, old = self._scans.popleft()
            self._count -= old.shape[0]

    def clear(self):
        self._scans.clear()
        self._count = 0

    def parts(self, now=None):
        """The scan arrays inside the window, oldest first. Shared references, not copies."""
        if now is not None:
            self.prune(now)
        return [p for _, p in self._scans]

    def stack(self, now=None):
        parts = self.parts(now)
        if not parts:
            return np.zeros((0, 3), dtype=np.float32)
        return np.concatenate(parts, axis=0)

    def __len__(self):
        return self._count
