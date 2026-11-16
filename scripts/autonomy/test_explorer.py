"""Unit tests for the reactive explorer (pure logic, no ROS).

  python3 -m pytest -q scripts/autonomy/test_explorer.py
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from explorer import Config, Explorer  # noqa: E402

INF = math.inf


def step(ex, now, df, dr=INF, dl=INF, dr_side=INF, age=0.05):
    return ex.decide(now, df, dr, dl, dr_side, age)


def test_no_data_stops():
    ex = Explorer(Config())
    assert step(ex, 0.0, 3.0, age=None) == ('STALE', 0.0, 0.0)
    assert step(ex, 0.0, 3.0, age=2.0) == ('STALE', 0.0, 0.0)


def test_open_space_forward_at_cap():
    ex = Explorer(Config())
    state, vx, wz = step(ex, 0.0, 3.0)
    assert state == 'FORWARD' and vx == pytest.approx(0.10) and wz == 0.0


def test_speed_ramps_down_monotonically():
    ex = Explorer(Config())
    vs = [step(ex, 0.0, d)[1] for d in (1.5, 1.2, 1.0, 0.8, 0.7)]
    assert vs[0] == pytest.approx(0.10)
    assert vs == sorted(vs, reverse=True)
    assert 0.0 < vs[-1] < 0.10


def test_too_close_reverses_when_rear_clear():
    ex = Explorer(Config())
    state, vx, wz = step(ex, 0.0, 0.5, dr=2.0)
    assert state == 'REVERSE' and vx == pytest.approx(-0.08) and wz == 0.0


def test_reverse_until_clear_then_forward():
    ex = Explorer(Config())
    step(ex, 0.0, 0.5, dr=2.0)
    assert step(ex, 0.2, 0.7, dr=1.5)[0] == 'REVERSE'
    assert step(ex, 0.4, 0.95, dr=1.5)[0] == 'FORWARD'


def test_rear_blocked_turns_toward_open_side():
    ex = Explorer(Config())
    state, vx, wz = step(ex, 0.0, 0.5, dr=0.2, dl=0.3, dr_side=2.0)
    assert state == 'TURN' and vx == 0.0
    assert wz < 0.0  # right side has more room: turn right (CW)


def test_turn_left_when_left_is_open():
    ex = Explorer(Config())
    state, vx, wz = step(ex, 0.0, 0.5, dr=0.2, dl=2.0, dr_side=0.3)
    assert state == 'TURN' and wz > 0.0


def test_turn_continues_until_front_clears():
    ex = Explorer(Config())
    step(ex, 0.0, 0.5, dr=0.2, dl=2.0, dr_side=0.3)
    assert step(ex, 0.5, 0.7)[0] == 'TURN'
    assert step(ex, 1.0, 0.95)[0] == 'FORWARD'


def test_reverse_timeout_switches_to_turn():
    ex = Explorer(Config(reverse_max_s=1.0))
    step(ex, 0.0, 0.5, dr=2.0)
    state, vx, wz = step(ex, 1.5, 0.6, dr=2.0, dl=2.0, dr_side=0.2)
    assert state == 'TURN' and wz > 0.0


def test_turn_timeout_gets_stuck_until_reset():
    ex = Explorer(Config(turn_max_s=1.0))
    step(ex, 0.0, 0.5, dr=0.2)                    # rear blocked -> turn
    assert step(ex, 2.0, 0.6)[0] == 'STUCK'
    assert step(ex, 3.0, 3.0) == ('STUCK', 0.0, 0.0)  # sticky: open space does not resume
    ex.reset()
    assert step(ex, 4.0, 3.0)[0] == 'FORWARD'


def test_config_rejects_bad_thresholds_and_caps():
    with pytest.raises(ValueError):
        Explorer(Config(stop_dist=1.0, clear_dist=0.9))
    with pytest.raises(ValueError):
        Explorer(Config(fwd_speed=0.2))
