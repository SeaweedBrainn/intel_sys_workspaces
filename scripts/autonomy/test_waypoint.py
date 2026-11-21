"""Unit tests for map and waypoint logic (no ROS needed).

Run: python3 -m pytest -q scripts/autonomy/test_waypoint.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import waypoint as wp  # noqa: E402

CLEAR = 10.0   # front clearance well outside every threshold


def nav(*goals):
    n = wp.WaypointNav()
    n.set_points(goals)
    return n


def test_no_goals_is_idle():
    assert nav().step(0.0, (0, 0, 0), CLEAR, 0.0) == ('IDLE', 0.0, 0.0)


def test_needs_pose_and_fresh_lidar():
    n = nav((1.0, 0.0))
    assert n.step(0.0, None, CLEAR, 0.0)[0] == 'NO_POSE'
    assert n.step(0.0, (0, 0, 0), CLEAR, None)[0] == 'STALE'
    assert n.step(0.0, (0, 0, 0), CLEAR, 0.6)[0] == 'STALE'


def test_turns_in_place_when_heading_is_off():
    n = nav((0.0, 1.0))                       # goal to the left, robot faces +x
    state, vx, wz = n.step(0.0, (0, 0, 0), CLEAR, 0.0)
    assert state == 'TURN' and vx == 0.0
    assert wz > 0.0 and wz <= n.cfg.max_ang  # turn toward +y


def test_drives_forward_when_aligned():
    n = nav((2.0, 0.0))
    state, vx, wz = n.step(0.0, (0, 0, 0), CLEAR, 0.0)
    assert state == 'DRIVE'
    assert 0.0 < vx <= n.cfg.fwd_speed + 1e-9
    assert abs(wz) < 1e-9


def test_speed_capped_and_slows_near_goal():
    n = nav((2.0, 0.0))
    _, far, _ = n.step(0.0, (0, 0, 0), CLEAR, 0.0)
    n2 = nav((0.2, 0.0))
    _, near, _ = n2.step(0.0, (0, 0, 0), CLEAR, 0.0)
    assert near < far
    assert near >= n.cfg.min_speed - 1e-9


def test_blocked_stops_and_resumes_when_path_clears():
    n = nav((2.0, 0.0))
    assert n.step(0.0, (0, 0, 0), 0.3, 0.0) == ('BLOCKED', 0.0, 0.0)
    assert n.halted is None                   # blocked is not latched
    assert n.step(0.1, (0, 0, 0), CLEAR, 0.0)[0] == 'DRIVE'


def test_slows_as_front_closes_in():
    n = nav((5.0, 0.0))
    _, open_v, _ = n.step(0.0, (0, 0, 0), 5.0, 0.0)
    n = nav((5.0, 0.0))
    _, close_v, _ = n.step(0.0, (0, 0, 0), 0.6, 0.0)
    assert 0.0 < close_v < open_v


def test_arrival_advances_then_done():
    n = nav((1.0, 0.0), (1.0, 1.0))
    assert n.step(0.0, (0.8, 0.0, 0.0), CLEAR, 0.0)[0] == 'DRIVE'
    assert n.step(0.1, (1.0, 0.0, 0.0), CLEAR, 0.0)[0] == 'ARRIVED'
    assert n.index == 1 and n.arrived == 1
    # second goal is straight up: heading error of 90 deg means a turn
    assert n.step(0.2, (1.0, 0.0, 0.0), CLEAR, 0.0)[0] == 'TURN'
    assert n.step(0.3, (1.0, 1.0, math.pi / 2), CLEAR, 0.0)[0] == 'ARRIVED'
    assert n.step(0.4, (1.0, 1.0, 0.0), CLEAR, 0.0)[0] == 'DONE'


def test_timeout_latches_until_reset():
    n = nav((5.0, 0.0))
    assert n.step(0.0, (0, 0, 0), CLEAR, 0.0)[0] == 'DRIVE'
    t = n.cfg.goal_timeout_s + 1.0
    assert n.step(t, (0, 0, 0), CLEAR, 0.0) == ('TIMEOUT', 0.0, 0.0)
    assert n.step(t + 1, (0, 0, 0), CLEAR, 0.0)[0] == 'TIMEOUT'  # latched
    n.reset()
    assert n.halted is None and n.index == 0
    assert n.step(t + 2, (0, 0, 0), CLEAR, 0.0)[0] == 'DRIVE'


def test_new_points_reset_the_run():
    n = nav((1.0, 0.0))
    n.step(0.0, (1.0, 0.0, 0.0), CLEAR, 0.0)
    n.set_points([(3.0, 0.0)])
    assert n.index == 0 and n.arrived == 0


def test_wrap():
    assert math.isclose(wp.wrap(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(wp.wrap(-3 * math.pi / 2), math.pi / 2)
    assert math.isclose(wp.wrap(0.1), 0.1)


def test_map_counts_hits_in_odom_frame():
    m = wp.OccupancyMap(cell=0.1)
    # robot at (1, 0) facing +x; a return 2 m ahead lands at world (3, 0)
    m.add_points((1.0, 0.0, 0.0), np.array([2.0]), np.array([0.0]))
    assert m.hits == {(30, 0): 1}
    # same return again increments the cell
    m.add_points((1.0, 0.0, 0.0), np.array([2.0]), np.array([0.0]))
    assert m.hits == {(30, 0): 2}


def test_map_rotates_points_with_yaw():
    m = wp.OccupancyMap(cell=0.1)
    m.add_points((0.0, 0.0, math.pi / 2), np.array([1.0]), np.array([0.0]))
    assert m.hits == {(0, 10): 1}            # 1 m along +y after a 90 deg yaw


def test_map_respects_cell_cap_but_still_counts_known_cells():
    m = wp.OccupancyMap(cell=0.1, max_cells=2)
    m.add_points((0, 0, 0), np.array([0.5, 1.0, 1.5]), np.array([0.0, 0.0, 0.0]))
    assert len(m.hits) == 2
    m.add_points((0, 0, 0), np.array([0.5]), np.array([0.0]))
    assert m.hits[(5, 0)] == 2


def test_trail_is_decimated_and_clears():
    m = wp.OccupancyMap(trail_step=0.05)
    m.add_pose(0.0, 0.0)
    m.add_pose(0.01, 0.0)                    # too close, skipped
    m.add_pose(0.1, 0.0)
    assert list(m.trail) == [(0.0, 0.0), (0.1, 0.0)]
    m.clear()
    assert len(m.trail) == 0 and m.hits == {}


def test_snapshot_is_json_ready():
    import json
    m = wp.OccupancyMap()
    m.add_pose(0.0, 0.0)
    m.add_points((0, 0, 0), np.array([1.0]), np.array([0.5]))
    s = json.loads(json.dumps(m.snapshot()))
    assert s['cells'] == 1 and s['hits'][0][2] == 1 and s['trail'] == [[0.0, 0.0]]


def test_map_window_expires_old_cells():
    m = wp.OccupancyMap(cell=0.1, window_s=10.0)
    m.add_points((0, 0, 0), np.array([1.0]), np.array([0.0]), now=0.0)
    m.add_points((0, 0, 0), np.array([2.0]), np.array([0.0]), now=8.0)
    assert m.snapshot(now=11.0)['cells'] == 1       # first cell is 11 s old
    assert m.snapshot(now=30.0)['cells'] == 0       # both expired
    assert m.hits == {} and m.last == {}


def test_map_window_refreshes_on_new_returns():
    m = wp.OccupancyMap(cell=0.1, window_s=10.0)
    m.add_points((0, 0, 0), np.array([1.0]), np.array([0.0]), now=0.0)
    m.add_points((0, 0, 0), np.array([1.0]), np.array([0.0]), now=9.0)
    assert m.snapshot(now=15.0)['cells'] == 1       # kept alive by the t=9 return


def test_map_without_window_never_expires():
    m = wp.OccupancyMap(cell=0.1)
    m.add_points((0, 0, 0), np.array([1.0]), np.array([0.0]), now=0.0)
    assert m.snapshot(now=1e6)['cells'] == 1
