"""Reactive exploration: forward wander with obstacle slow-down, reverse and turn.

Pure decision logic (no ROS), so it can be unit tested. The dashboard node feeds it
clearance distances measured in base_footprint and publishes the result.

Behaviour
  * clear ahead (d_front >= slow_dist)       : drive forward at fwd_speed
  * object ahead, closing (stop < d <= slow) : forward speed ramps linearly to zero
  * too close (d_front <= stop_dist)         : reverse at rev_speed if rear is clear
                                               (rear > rear_stop), until d_front >= clear_dist
  * too close and rear blocked, or reverse
    timed out                                : turn in place toward the side with more room
                                               until d_front >= clear_dist
  * no sensor data for stale_s               : stop

Distances are the 3rd-closest return in each sector (see dashboard.py), so one stray point
does not change behaviour. The caller passes math.inf for empty sectors.

Speeds are capped in Config; the caller must not exceed the dashboard caps.
"""
import math
from dataclasses import dataclass


@dataclass
class Config:
    slow_dist: float = 1.5      # start slowing below this (m)
    stop_dist: float = 0.6      # too close below this (m)
    clear_dist: float = 0.9     # front clear again above this (m)
    rear_stop: float = 0.4      # rear clearance needed to reverse (m)
    fwd_speed: float = 0.10     # m/s, cap
    rev_speed: float = 0.08     # m/s, cap
    turn_speed: float = 0.40    # rad/s, cap 0.5
    reverse_max_s: float = 4.0
    turn_max_s: float = 10.0
    stale_s: float = 0.5

    def validate(self):
        if not (0 < self.stop_dist < self.clear_dist <= self.slow_dist):
            raise ValueError('need 0 < stop_dist < clear_dist <= slow_dist')
        if self.fwd_speed > 0.10 or self.rev_speed > 0.10 or self.turn_speed > 0.5:
            raise ValueError('speed above dashboard caps')


class Explorer:
    IDLE, FORWARD, REVERSE, TURN, STUCK, STALE = 'IDLE', 'FORWARD', 'REVERSE', 'TURN', 'STUCK', 'STALE'

    def __init__(self, cfg: Config):
        cfg.validate()
        self.cfg = cfg
        self.state = self.IDLE
        self.t_state = 0.0
        self.turn_sign = 0.0  # +1 turn left (CCW), -1 turn right

    def decide(self, now, d_front, d_rear, d_left, d_right, age_s):
        """Return (state, vx, wz) for the current sensor snapshot."""
        c = self.cfg
        if age_s is None or age_s > c.stale_s:
            self.state = self.STALE
            return self.state, 0.0, 0.0

        front_clear = d_front >= c.clear_dist
        rear_clear = d_rear > c.rear_stop

        if self.state == self.STUCK:
            return self.state, 0.0, 0.0

        if self.state == self.TURN:
            if front_clear:
                self._enter(self.FORWARD, now)
            elif now - self.t_state > c.turn_max_s:
                self._enter(self.STUCK, now)
                return self.state, 0.0, 0.0
            else:
                return self.state, 0.0, self.turn_sign * c.turn_speed

        if self.state == self.REVERSE:
            if front_clear:
                self._enter(self.FORWARD, now)
            elif rear_clear and now - self.t_state <= c.reverse_max_s:
                return self.state, -c.rev_speed, 0.0
            else:
                return self._start_turn(now, d_left, d_right)

        if d_front <= c.stop_dist:
            if rear_clear:
                self._enter(self.REVERSE, now)
                return self.state, -c.rev_speed, 0.0
            return self._start_turn(now, d_left, d_right)

        self._enter(self.FORWARD, now)
        if d_front >= c.slow_dist:
            return self.state, c.fwd_speed, 0.0
        frac = (d_front - c.stop_dist) / (c.slow_dist - c.stop_dist)
        return self.state, c.fwd_speed * max(0.0, min(1.0, frac)), 0.0

    def _enter(self, state, now):
        if state != self.state:
            self.state = state
            self.t_state = now

    def _start_turn(self, now, d_left, d_right):
        # Turn toward the side with more room. Left is +y, which is CCW (+wz).
        left = d_left if math.isfinite(d_left) else 99.0
        right = d_right if math.isfinite(d_right) else 99.0
        self.turn_sign = 1.0 if left >= right else -1.0
        self._enter(self.TURN, now)
        return self.state, 0.0, self.turn_sign * self.cfg.turn_speed

    def reset(self):
        self.state = self.IDLE
        self.t_state = 0.0
        self.turn_sign = 0.0
