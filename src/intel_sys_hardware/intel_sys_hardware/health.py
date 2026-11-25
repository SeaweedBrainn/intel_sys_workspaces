"""Stream health for robot_health (pure logic, no ROS).

A Check watches one topic: how often messages arrive and how long since the last one.
status() turns that into a diagnostic level and a short message for Foxglove's
Diagnostics panel.
"""
import collections
from dataclasses import dataclass, field

OK, WARN, ERROR = 0, 1, 2


@dataclass
class Check:
    name: str
    topic: str
    msg_type: str            # e.g. sensor_msgs/msg/Imu
    min_hz: float            # below this rate: WARN
    stale_s: float           # nothing for this long: ERROR (or WARN if optional)
    optional: bool = False   # e.g. the map, which only exists while RTAB-Map runs
    window: int = 50
    _times: collections.deque = field(default_factory=collections.deque, repr=False)

    def mark(self, t):
        self._times.append(t)
        while len(self._times) > self.window:
            self._times.popleft()

    def rate(self, now, horizon=5.0):
        """Messages per second over the last `horizon` seconds (0 if fewer than two)."""
        recent = [t for t in self._times if now - t <= horizon]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0

    def age(self, now):
        return now - self._times[-1] if self._times else None

    def status(self, now):
        """(level, message) for this check."""
        age = self.age(now)
        if age is None:
            return (WARN if self.optional else ERROR), 'no messages yet'
        if age > self.stale_s:
            return (WARN if self.optional else ERROR), f'silent for {age:.0f} s'
        hz = self.rate(now)
        if hz < self.min_hz:
            return WARN, f'{hz:.1f} Hz (expected at least {self.min_hz:g})'
        return OK, f'{hz:.1f} Hz'


def parse_check(spec):
    """Check from 'name|topic|type|min_hz|stale_s[|optional]'."""
    parts = [p.strip() for p in spec.split('|')]
    if len(parts) not in (5, 6):
        raise ValueError(f'bad check spec {spec!r}: name|topic|type|min_hz|stale_s[|optional]')
    name, topic, msg_type, min_hz, stale_s = parts[:5]
    optional = len(parts) == 6 and parts[5].lower() in ('optional', 'true', '1')
    return Check(name, topic, msg_type, float(min_hz), float(stale_s), optional)
