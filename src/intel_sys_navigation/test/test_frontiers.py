import math

import numpy as np

from intel_sys_navigation.frontiers import Frontier, choose, find_frontiers, still_frontier, target_cell

RES = 0.1


def room(free_cols):
    """20x20 unknown grid with free columns [0, free_cols) and walls top and bottom."""
    g = np.full((20, 20), -1, dtype=np.int8)
    g[:, :free_cols] = 0
    g[0, :free_cols] = 100
    g[19, :free_cols] = 100
    return g


def test_frontier_at_edge_of_free_space():
    fs = find_frontiers(room(10), RES, (0.0, 0.0), min_size=3, obstacle_clearance=0.15, goal_clearance=0)
    assert len(fs) == 1
    f = fs[0]
    # frontier column is 9 (x = 0.95); cells near the walls are dropped by the clearance
    assert math.isclose(f.goal[0], 0.95, abs_tol=1e-6)
    assert 0.3 < f.goal[1] < 1.7
    assert f.size == 14


def test_no_frontier_when_fully_known():
    g = np.zeros((10, 10), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    assert find_frontiers(g, RES, (0.0, 0.0), min_size=1) == []


def test_small_frontiers_are_noise():
    g = np.zeros((10, 10), dtype=np.int8)
    g[5, 5] = -1  # one unknown cell inside known free space
    assert find_frontiers(g, RES, (0.0, 0.0), min_size=8) == []
    assert len(find_frontiers(g, RES, (0.0, 0.0), min_size=1, obstacle_clearance=0)) == 1


def test_origin_offsets_goal():
    fs = find_frontiers(room(10), RES, (-1.0, 2.0), min_size=3, goal_clearance=0)
    assert math.isclose(fs[0].goal[0], -0.05, abs_tol=1e-6)
    assert fs[0].goal[1] > 2.0


def test_choose_prefers_ahead_over_behind():
    ahead = Frontier(20, (2.0, 0.0), (2.0, 0.0))
    behind = Frontier(20, (-2.0, 0.0), (-2.0, 0.0))
    assert choose([behind, ahead], (0.0, 0.0), 0.0) is ahead
    assert choose([behind, ahead], (0.0, 0.0), math.pi) is behind


def test_choose_skips_blacklist_and_goals_underfoot():
    near = Frontier(50, (0.1, 0.0), (0.1, 0.0))
    far = Frontier(10, (3.0, 0.0), (3.0, 0.0))
    other = Frontier(10, (0.0, 3.0), (0.0, 3.0))
    assert choose([near, far], (0.0, 0.0), 0.0) is far
    assert choose([near, far, other], (0.0, 0.0), 0.0, blacklist=[(3.1, 0.0)]) is other
    assert choose([near], (0.0, 0.0), 0.0) is None


def test_goal_stays_valid_while_its_frontier_shrinks():
    goal = find_frontiers(room(10), RES, (0.0, 0.0), min_size=3, goal_clearance=0)[0].goal
    g = room(10)
    g[:9, 10:] = 0  # the top half of the unknown side gets mapped; the frontier moves down
    fs = find_frontiers(g, RES, (0.0, 0.0), min_size=3, goal_clearance=0)
    assert fs and math.dist(fs[0].goal, goal) > 0.3  # the centre moved
    assert still_frontier(fs, goal)                   # but cells remain near the goal
    assert not still_frontier(find_frontiers(np.zeros((20, 20), np.int8), RES, (0, 0)), goal)


def test_goal_keeps_clear_of_walls():
    g = room(10)
    g[8:12, 6] = 100  # a post near the frontier
    f = find_frontiers(g, RES, (0.0, 0.0), min_size=3, goal_clearance=0.45)[0]
    gx, gy = f.goal
    occ = [((c + 0.5) * RES, (r + 0.5) * RES) for r, c in zip(*np.nonzero(g > 25))]
    assert min(math.dist(f.goal, o) for o in occ) >= 0.45 - 1e-6
    assert gx < 0.95  # pulled back from the frontier column into known space


def test_pocket_without_room_is_dropped():
    g = np.full((20, 20), 100, dtype=np.int8)
    g[5:15, 5:15] = 0     # a free room, with walls around it
    g[9:11, 15:18] = 0    # a 0.2 m wide slot...
    g[9:11, 18] = -1      # ...ending in unknown space
    assert find_frontiers(g, RES, (0.0, 0.0), min_size=1, obstacle_clearance=0,
                          goal_clearance=0.45, goal_reach=0.5) == []


def two_rooms():
    """Robot's room on the left, a free room behind a wall on the right, both with frontiers."""
    g = np.full((20, 30), -1, dtype=np.int8)
    g[2:18, 1:12] = 0      # robot's room
    g[2:18, 12] = 100      # wall
    g[2:18, 13:24] = 0     # free space seen through the wall
    return g


def test_frontier_behind_a_wall_is_unreachable():
    g = two_rooms()
    everything = find_frontiers(g, RES, (0.0, 0.0), min_size=3)
    mine = find_frontiers(g, RES, (0.0, 0.0), min_size=3, robot_xy=(0.6, 1.0))
    assert any(f.goal[0] > 1.3 for f in everything)
    assert mine and all(f.goal[0] < 1.2 for f in mine)


def test_gap_narrower_than_robot_does_not_connect():
    g = two_rooms()
    g[9:11, 12] = 0        # a 0.2 m gap in the wall
    assert all(f.goal[0] < 1.2 for f in
               find_frontiers(g, RES, (0.0, 0.0), min_size=3, robot_xy=(0.6, 1.0)))
    g[6:14, 12] = 0        # a 0.8 m doorway does
    assert any(f.goal[0] > 1.3 for f in
               find_frontiers(g, RES, (0.0, 0.0), min_size=3, robot_xy=(0.6, 1.0)))


def test_robot_off_the_map_skips_the_reachability_filter():
    g = two_rooms()
    assert len(find_frontiers(g, RES, (0.0, 0.0), min_size=3, robot_xy=(50.0, 50.0))) == \
        len(find_frontiers(g, RES, (0.0, 0.0), min_size=3))


def test_goal_set_back_from_frontier_is_still_valid_on_the_same_map():
    # Regression (robot, 2026-11-24): with the goal set back from the frontier, checking
    # the goal itself against frontier cells found none, and every goal was replaced
    # each second. Watch the frontier cell nearest the goal instead.
    cells = np.array([[1.0, y] for y in np.arange(0.5, 1.6, 0.1)])
    f = Frontier(len(cells), (1.0, 1.0), (0.3, 1.0), cells)
    assert not still_frontier([f], f.goal)
    assert target_cell(f) == (1.0, 1.0) or math.isclose(target_cell(f)[1], 1.0, abs_tol=0.06)
    assert still_frontier([f], target_cell(f))
