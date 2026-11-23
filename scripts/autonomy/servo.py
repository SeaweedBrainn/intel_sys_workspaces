"""Visual servo law: hold a forward distance and heading to an ArUco tag (pure logic, no ROS).

Inputs are the tag pose in the camera optical frame (z forward, x right) from tag.detect_tag.
Outputs are robot-frame commands: vx forward (+x, the camera looks along +x per the URDF),
wz yaw (+ is left, counter-clockwise).

States
  LOST       no detection newer than stale_s: zero command
  TOO_FAR    tag beyond max_m: zero command (too small to trust)
  TOO_CLOSE  tag nearer than min_m: zero command, and no reverse unless allow_reverse
  HOLD       inside the distance band and heading deadband
  APPROACH   forward motion toward the target distance
  BACK       reverse motion (only if allow_reverse)
  TURN       heading correction only
"""
import math
from dataclasses import dataclass


@dataclass
class ServoConfig:
    target_m: float = 0.4572    # forward distance to hold: 18 in
    tol_m: float = 0.05         # no forward or back motion inside this band
    min_m: float = 0.35         # never closer than this
    max_m: float = 4.0          # ignore tags farther than this
    kp_lin: float = 0.4         # m/s per m of distance error
    max_lin: float = 0.08       # m/s cap (manual pad cap is 0.10)
    allow_reverse: bool = False  # the LiDAR cannot see behind the robot, so reverse is off by default
    max_reverse: float = 0.05
    kp_yaw: float = 1.0         # rad/s per rad of bearing error
    max_wz: float = 0.3         # rad/s cap (manual pad cap is 0.5)
    yaw_tol_rad: float = 0.03   # about 1.7 deg heading deadband
    stale_s: float = 0.4        # a detection older than this counts as lost


def _clamp(v, lim):
    return max(-lim, min(lim, v))


def decide(cfg, det, age_s):
    """(state, vx, wz) for a tag detection `det` seen age_s seconds ago, or None if no detection."""
    if det is None or age_s is None or age_s > cfg.stale_s:
        return 'LOST', 0.0, 0.0
    z, bearing = float(det['z_m']), float(det['bearing_rad'])
    if z < cfg.min_m:
        return 'TOO_CLOSE', 0.0, 0.0
    if z > cfg.max_m:
        return 'TOO_FAR', 0.0, 0.0

    err = z - cfg.target_m
    vx = 0.0 if abs(err) < cfg.tol_m else _clamp(cfg.kp_lin * err, cfg.max_lin)
    if vx < 0.0 and not cfg.allow_reverse:
        vx = 0.0
    elif vx < 0.0:
        vx = _clamp(vx, cfg.max_reverse)
    # tag to the right (bearing > 0) means turn right, which is negative wz
    wz = 0.0 if abs(bearing) < cfg.yaw_tol_rad else _clamp(-cfg.kp_yaw * bearing, cfg.max_wz)

    if vx == 0.0 and wz == 0.0:
        state = 'HOLD'
    elif vx > 0.0:
        state = 'APPROACH'
    elif vx < 0.0:
        state = 'BACK'
    else:
        state = 'TURN'
    return state, vx, wz


def bearing_deg(rad):
    return math.degrees(rad)
