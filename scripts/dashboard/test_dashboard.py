"""Unit tests for the dashboard's command gating (no ROS graph needed).

Run inside the rover container with the workspace overlay sourced:
  python3 -m pytest -q scripts/dashboard/test_dashboard.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import dashboard  # noqa: E402

ZERO = (0.0, 0.0, 0.0)


def fresh():
    return dashboard.State()


def test_idle_publishes_nothing():
    s = fresh()
    assert s.command(100.0) is None


def test_active_drive_is_published():
    s = fresh()
    s.drive, s.drive_t = (0.05, 0.0, 0.0), 100.0
    assert s.command(100.1) == (0.05, 0.0, 0.0)


def test_drive_expires_to_explicit_zeros_then_quiet():
    s = fresh()
    s.drive, s.drive_t = (0.05, 0.0, 0.0), 100.0
    assert s.command(100.05) == (0.05, 0.0, 0.0)  # a tick while the button is held
    now = 100.0 + dashboard.DEADMAN_S + 0.01      # then the browser goes silent
    outputs = [s.command(now + i * 0.1) for i in range(5)]
    assert outputs[:3] == [ZERO, ZERO, ZERO]
    assert outputs[3:] == [None, None]


def test_stop_request_sends_zeros_not_silence():
    s = fresh()
    s.drive, s.drive_t = (0.1, 0.0, 0.0), 100.0
    assert s.command(100.05) == (0.1, 0.0, 0.0)
    # /api/stop sets the drive back to zero
    s.drive, s.drive_t = ZERO, 100.1
    outputs = [s.command(100.1 + i * 0.1) for i in range(4)]
    assert outputs[:3] == [ZERO, ZERO, ZERO]
    assert outputs[3] is None


def test_estop_always_zero_and_blocks_drive():
    s = fresh()
    s.estop = True
    assert s.command(1.0) == ZERO
    assert s.command(2.0) == ZERO  # keeps asserting zero while engaged


def test_caps_are_sane():
    # Hard caps must stay within the documented safety envelope.
    assert dashboard.MAX_LIN <= 0.15
    assert dashboard.MAX_ANG <= 0.8
    assert dashboard.DEADMAN_S <= 0.5


def test_rosout_levels_compare_as_ints():
    # Regression: comparing msg.level against Log.WARN (bytes) raised TypeError in
    # the rosout callback and silently killed the executor thread.
    from rcl_interfaces.msg import Log
    m = Log()
    m.level = 30
    assert int(m.level) >= dashboard.LOG_WARN
    m.level = 20  # INFO
    assert int(m.level) < dashboard.LOG_WARN
    assert dashboard.LOG_ERROR == 40
