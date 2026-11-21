"""Tests for gravity alignment and the sliding cloud window (no ROS needed)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import frames  # noqa: E402


def test_align_rotation_takes_up_to_z():
    rng = np.random.default_rng(1)
    for _ in range(50):
        u = rng.normal(size=3)
        u /= np.linalg.norm(u)
        R = frames.align_rotation(u)
        assert np.allclose(R @ u, [0, 0, 1], atol=1e-9)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert math.isclose(np.linalg.det(R), 1.0, rel_tol=1e-9)


def test_align_rotation_edge_cases():
    assert np.allclose(frames.align_rotation([0, 0, 2]), np.eye(3))
    assert np.allclose(frames.align_rotation([0, 0, -1]) @ np.array([0, 0, -1.0]), [0, 0, 1])


def test_tilt_deg():
    assert math.isclose(frames.tilt_deg([0, 0, 9.8]), 0.0, abs_tol=1e-9)
    assert math.isclose(frames.tilt_deg([9.8, 0, 0]), 90.0, rel_tol=1e-9)


def test_window_keeps_recent_scans_only():
    w = frames.WindowedCloud(window_s=5.0)
    w.add(0.0, np.ones((2, 3), np.float32))
    w.add(4.0, np.ones((3, 3), np.float32))
    assert len(w) == 5
    w.add(7.0, np.ones((1, 3), np.float32))         # scan at t=0 is now 7 s old
    assert w.stack().shape == (4, 3)
    assert w.stack(now=20.0).shape == (0, 3)


def test_window_empty_stack_shape():
    assert frames.WindowedCloud(1.0).stack().shape == (0, 3)


def test_rolling_buffer_caps_points_oldest_first():
    w = frames.WindowedCloud(window_s=1000.0, max_points=5)
    w.add(0.0, np.zeros((3, 3), np.float32))
    w.add(1.0, np.ones((3, 3), np.float32))          # 6 points > 5: oldest scan goes
    assert len(w) == 3
    assert np.all(w.stack() == 1.0)
    w.add(2.0, np.full((2, 3), 2.0, np.float32))
    assert len(w) == 5
    assert w.stack().shape == (5, 3)


def test_rolling_buffer_count_tracks_time_expiry():
    w = frames.WindowedCloud(window_s=5.0, max_points=100)
    w.add(0.0, np.zeros((4, 3), np.float32))
    w.prune(10.0)
    assert len(w) == 0

