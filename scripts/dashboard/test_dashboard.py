"""Unit tests for dashboard gating (no ROS graph needed).

Run inside the rover container with the workspace overlay sourced:
  python3 -m pytest -q scripts/dashboard/test_dashboard.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import dashboard  # noqa: E402

ZERO = (0.0, 0.0, 0.0)


def fresh():
    return dashboard.State()


def test_idle_publishes_nothing():
    assert fresh().manual_command(100.0) is None


def test_active_manual_drive_is_published():
    s = fresh()
    s.drive, s.drive_t = (0.05, 0.0, 0.0), 100.0
    assert s.manual_command(100.1) == (0.05, 0.0, 0.0)


def test_manual_drive_expires_to_zeros_then_quiet():
    s = fresh()
    s.drive, s.drive_t = (0.05, 0.0, 0.0), 100.0
    assert s.manual_command(100.05) == (0.05, 0.0, 0.0)  # a tick while held
    now = 100.0 + dashboard.DEADMAN_S + 0.01
    outs = [s.manual_command(now + i * 0.1) for i in range(5)]
    assert outs[:3] == [ZERO, ZERO, ZERO]
    assert outs[3:] == [None, None]


def test_stop_request_sends_zeros_not_silence():
    s = fresh()
    s.drive, s.drive_t = (0.1, 0.0, 0.0), 100.0
    assert s.manual_command(100.05) == (0.1, 0.0, 0.0)
    s.drive, s.drive_t = ZERO, 100.1                     # what /api/stop does
    outs = [s.manual_command(100.1 + i * 0.1) for i in range(4)]
    assert outs[:3] == [ZERO, ZERO, ZERO]
    assert outs[3] is None


def test_caps_are_within_envelope():
    assert dashboard.MAX_LIN <= 0.15
    assert dashboard.MAX_ANG <= 0.8
    assert dashboard.DEADMAN_S <= 0.5


def test_rosout_levels_compare_as_ints():
    # Regression: comparing msg.level against Log.WARN (bytes) raised TypeError in the
    # rosout callback and silently killed the executor thread.
    from rcl_interfaces.msg import Log
    m = Log()
    m.level = 30
    assert int(m.level) >= dashboard.LOG_WARN
    m.level = 20
    assert int(m.level) < dashboard.LOG_WARN
    assert dashboard.LOG_ERROR == 40


def test_nearest_needs_several_points():
    assert dashboard.nearest(np.array([0.8])) == math.inf
    assert dashboard.nearest(np.array([0.9, 0.7, 1.2])) == 1.2
    assert dashboard.nearest(np.array([])) == math.inf
