"""Map and point-to-point waypoint following for the dashboard (pure logic, no ROS).

Frames
  * The map and waypoints live in the odometry frame: the origin is where wheel odometry
    started (boot or the last odom reset). Wheel odometry drifts, so the map is a local
    picture for short runs, not a survey. There is no SLAM or loop closure yet
    (docs/ROADMAP.md, Phase 4).
  * Headings use the explorer's LiDAR frame ("forward" is toward what the LiDAR sees, see
    dashboard.lidar_pose). Commands are returned in that frame; the dashboard rotates them
    before publishing, exactly as it does for autonomous exploration.

Map
  * OccupancyMap counts LiDAR returns per cell (hits only, no free-space carving) and keeps
    a decimated trail of the odometry path.

Waypoint following
  * WaypointNav walks a list of (x, y) goals in order. Per goal it turns in place until the
    heading is within turn_tol, then drives forward at up to fwd_speed, slowing near the goal.
  * It stops (not reverses) when the front clearance is at or below stop_front, resumes
    when the path clears, and stops for good on a per-goal timeout or when LiDAR data is
    stale. Forward-only motion keeps the front sensor in the direction of travel.
"""
import collections
import math
import time
from dataclasses import dataclass

import numpy as np


def wrap(a):
    """Angle wrapped to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(v, lim):
    return max(-lim, min(lim, v))


@dataclass
class NavConfig:
    goal_tol: float = 0.10      # m, goal reached inside this radius
    turn_tol: float = 0.25      # rad, drive only when heading error is inside this
    fwd_speed: float = 0.10     # m/s, cap (matches the manual and explorer caps)
    turn_gain: float = 1.0      # rad/s per rad of heading error
    max_ang: float = 0.30       # rad/s cap for waypoint turns
    stop_front: float = 0.45    # m, stop forward motion at or below this front clearance
    slow_front: float = 1.00    # m, ramp speed down between slow_front and stop_front
    min_speed: float = 0.03     # m/s, creep speed near the goal
    stale_s: float = 0.5        # s, LiDAR silence that stops the run
    goal_timeout_s: float = 60.0  # s per goal before the run stops for good


class OccupancyMap:
    """Per-cell LiDAR return counts plus the odometry trail, in the odometry frame.

    With window_s set, a cell expires once no return has landed in it for window_s seconds,
    so the map follows recent geometry instead of growing without limit.
    """

    def __init__(self, cell=0.10, max_cells=20000, trail_step=0.05, max_trail=3000, window_s=None):
        self.cell = cell
        self.max_cells = max_cells
        self.trail_step = trail_step
        self.window_s = window_s
        self.hits = {}
        self.last = {}          # cell -> time of its most recent return
        self.trail = collections.deque(maxlen=max_trail)
        self._last = None

    def clear(self):
        self.hits.clear()
        self.last.clear()
        self.trail.clear()
        self._last = None

    def prune(self, now):
        if self.window_s is None:
            return
        for k in [k for k, t in self.last.items() if now - t > self.window_s]:
            del self.hits[k]
            del self.last[k]

    def add_pose(self, x, y):
        if self._last is None or math.hypot(x - self._last[0], y - self._last[1]) >= self.trail_step:
            self.trail.append((x, y))
            self._last = (x, y)

    def add_points(self, pose, px, py, now=None):
        """Add LiDAR returns (base-frame px, py, numpy arrays) seen from pose = (x, y, yaw)."""
        if px.size == 0:
            return
        if now is None:
            now = time.monotonic()
        x0, y0, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x0 + c * px - s * py
        wy = y0 + s * px + c * py
        ix = np.floor(wx / self.cell).astype(np.int64)
        iy = np.floor(wy / self.cell).astype(np.int64)
        keys, counts = np.unique(np.stack([ix, iy], axis=1), axis=0, return_counts=True)
        if len(self.hits) + keys.shape[0] > self.max_cells:
            self.prune(now)   # make room from expired cells before dropping new ones
        for (kx, ky), n in zip(keys.tolist(), counts.tolist()):
            k = (kx, ky)
            if k in self.hits:
                self.hits[k] += n
                self.last[k] = now
            elif len(self.hits) < self.max_cells:
                self.hits[k] = n
                self.last[k] = now

    def snapshot(self, now=None):
        if now is None:
            now = time.monotonic()
        self.prune(now)
        return {
            'cell_m': self.cell,
            'cells': len(self.hits),
            'window_s': self.window_s,
            'hits': [[kx, ky, n] for (kx, ky), n in self.hits.items()],
            'trail': [[round(x, 3), round(y, 3)] for x, y in self.trail],
        }


class WaypointNav:
    """Walks a list of goals. Call step() once per control tick.

    step() returns (state, vx, wz) in the LiDAR frame. States:
      IDLE, NO_POSE, STALE, TURN, DRIVE, BLOCKED, ARRIVED, DONE, TIMEOUT
    TIMEOUT latches until reset(). ARRIVED is a zero-command tick on a goal.
    """

    def __init__(self, cfg=None):
        self.cfg = cfg or NavConfig()
        self.points = []
        self.index = 0
        self.arrived = 0
        self.halted = None      # latched reason ('TIMEOUT')
        self._t0 = None

    def reset(self):
        self.index = 0
        self.arrived = 0
        self.halted = None
        self._t0 = None

    def set_points(self, points):
        self.points = [(float(x), float(y)) for x, y in points]
        self.reset()

    def step(self, now, pose, front, age_s):
        if self.halted:
            return self.halted, 0.0, 0.0
        if not self.points:
            return 'IDLE', 0.0, 0.0
        if self.index >= len(self.points):
            return 'DONE', 0.0, 0.0
        if pose is None:
            return 'NO_POSE', 0.0, 0.0
        if age_s is None or age_s > self.cfg.stale_s:
            return 'STALE', 0.0, 0.0

        if self._t0 is None:
            self._t0 = now
        if now - self._t0 > self.cfg.goal_timeout_s:
            self.halted = 'TIMEOUT'
            return self.halted, 0.0, 0.0

        cfg = self.cfg
        x, y, yaw = pose
        gx, gy = self.points[self.index]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist <= cfg.goal_tol:
            self.index += 1
            self.arrived += 1
            self._t0 = None
            return 'ARRIVED', 0.0, 0.0

        err = wrap(math.atan2(dy, dx) - yaw)
        wz = _clamp(cfg.turn_gain * err, cfg.max_ang)
        if abs(err) > cfg.turn_tol:
            return 'TURN', 0.0, wz

        if front <= cfg.stop_front:
            return 'BLOCKED', 0.0, 0.0

        speed = cfg.fwd_speed
        if front < cfg.slow_front:
            frac = (front - cfg.stop_front) / (cfg.slow_front - cfg.stop_front)
            speed = min(speed, cfg.fwd_speed * max(0.0, frac))
        speed = max(cfg.min_speed, min(speed, 0.3 * dist))
        return 'DRIVE', speed * math.cos(err), wz

    def stats(self):
        return {'goals': len(self.points), 'index': self.index, 'arrived': self.arrived,
                'halted': self.halted}
