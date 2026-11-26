import itertools
import math

import numpy as np

from intel_sys_navigation.frontiers import Frontier
from intel_sys_navigation.tour import PathGraph, candidates, leg_costs, pick, solve, view_gain

RES = 0.1


def half_known(width=40, free_cols=20):
    """40x40 grid at 0.1 m: known free on the left, unknown on the right."""
    g = np.full((40, width), -1, dtype=np.int8)
    g[:, :free_cols] = 0
    return g


def test_heading_faces_the_unknown_side():
    gain, heading = view_gain(half_known(), RES, (0.0, 0.0), (1.5, 2.0), fov=math.pi, max_range=2.0)
    assert gain > 1.0
    assert abs(heading) < math.radians(10)   # unknown is toward +x


def test_no_gain_when_all_known():
    gain, _ = view_gain(np.zeros((40, 40), np.int8), RES, (0.0, 0.0), (2.0, 2.0), max_range=1.5)
    assert gain == 0.0


def test_walls_block_the_view():
    g = half_known()
    g[:, 19] = 100   # a wall between the robot and the unknown
    gain, _ = view_gain(g, RES, (0.0, 0.0), (1.0, 2.0), max_range=0.9)  # stays on the map
    assert gain == 0.0


def test_gain_is_area():
    # Everything unknown, 360 deg view, range 1 m: about pi m^2.
    gain, _ = view_gain(np.full((40, 40), -1, np.int8), RES, (0.0, 0.0), (2.0, 2.0),
                        fov=2 * math.pi, max_range=1.0)
    assert math.isclose(gain, math.pi, rel_tol=0.05)


def test_off_the_map_counts_as_unknown():
    g = np.zeros((40, 40), np.int8)
    gain, heading = view_gain(g, RES, (0.0, 0.0), (3.8, 2.0), max_range=2.0)
    assert gain > 1.0 and abs(heading) < math.radians(10)


def test_candidates_drop_low_gain_frontiers():
    g = half_known()
    useful = Frontier(10, (1.9, 2.0), (1.5, 2.0))
    known = Frontier(10, (1.0, 2.0), (1.0, 2.0))
    g2 = np.zeros((40, 40), dtype=np.int8)   # all mapped, walls round the edge
    g2[0, :] = g2[-1, :] = g2[:, 0] = g2[:, -1] = 100
    out = candidates([useful], g, RES, (0.0, 0.0), max_range=2.0, min_gain=0.5)
    assert len(out) == 1 and out[0].frontier is useful
    assert candidates([known], g2, RES, (0.0, 0.0), max_range=2.0, min_gain=0.5) == []


def wall_room():
    """4 x 4 m free room with a wall from the bottom up to y = 3 m at x = 2 m."""
    g = np.zeros((40, 40), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[:30, 20] = 100
    return g


def test_path_goes_around_the_wall():
    pg = PathGraph(wall_room(), RES, (0.0, 0.0), passage_clearance=0.25, cell=0.1)
    dist, depart, arrive = pg.legs([(1.0, 1.0), (3.0, 1.0)])
    assert dist[0, 1] > 4.0                 # up past y = 3 and back down, not 2 m straight
    assert math.isclose(dist[0, 1], dist[1, 0], rel_tol=1e-6)
    assert depart[0, 1] > math.radians(45)  # sets off upward, not toward the goal
    assert arrive[0, 1] < -math.radians(45)  # arrives heading down


def test_unreachable_points_have_infinite_distance():
    g = wall_room()
    g[:, 20] = 100     # the wall now closes off the right half
    pg = PathGraph(g, RES, (0.0, 0.0), cell=0.1)
    dist, _, _ = pg.legs([(1.0, 1.0), (3.0, 1.0), (50.0, 50.0)])
    assert np.isinf(dist[0, 1]) and np.isinf(dist[0, 2])


def test_coarse_grid_does_not_join_rooms_through_a_wall():
    g = wall_room()
    g[:, 20] = 100
    pg = PathGraph(g, RES, (0.0, 0.0), cell=0.3)
    dist, _, _ = pg.legs([(1.0, 2.0), (3.0, 2.0)])
    assert np.isinf(dist[0, 1])


def test_turning_costs_time():
    dist = np.array([[0.0, 2.0], [2.0, 0.0]])
    bearing = np.array([[0.0, 0.0], [math.pi, 0.0]])
    ahead = leg_costs(dist, bearing, bearing, [0.0, 0.0], speed=0.2, turn_rate=0.5)
    behind = leg_costs(dist, bearing, bearing, [math.pi, 0.0], speed=0.2, turn_rate=0.5)
    assert math.isclose(ahead[0, 1], 10.0)
    assert math.isclose(behind[0, 1], 10.0 + math.pi / 0.5)


def brute_force(cost):
    best = (None, math.inf)
    for p in itertools.permutations(range(1, len(cost))):
        c = sum(cost[a, b] for a, b in zip((0,) + p, p))
        if c < best[1]:
            best = (list(p), c)
    return best


def test_exact_solver_matches_brute_force():
    rng = np.random.default_rng(1)
    for n in (2, 4, 7):
        cost = rng.uniform(1, 10, (n, n))
        order, total = solve(cost)
        assert math.isclose(total, brute_force(cost)[1])
        assert math.isclose(total, sum(cost[a, b] for a, b in zip([0] + order, order)))


def test_heuristic_solver_is_close_to_exact():
    rng = np.random.default_rng(2)
    pts = rng.uniform(0, 10, (9, 2))
    cost = np.hypot(*(pts[:, None] - pts[None]).T)
    exact = solve(cost)[1]
    order, total = solve(cost, exact_max=0)
    assert sorted(order) == list(range(1, 9))
    assert total <= exact * 1.1


def test_nearby_stop_is_taken_on_the_way():
    # Robot at 0, a big far frontier at 10 m and a small one at 3 m on the way:
    # greedy-by-size would go far first and come back; the tour takes the near one first.
    x = np.array([0.0, 10.0, 3.0])
    cost = np.abs(x[:, None] - x[None])
    order, total = solve(cost)
    assert order == [2, 1] and total == 10.0


def test_fixed_first_stop():
    x = np.array([0.0, 10.0, 3.0])
    cost = np.abs(x[:, None] - x[None])
    order, total = solve(cost, first=1)
    assert order == [1, 2] and total == 17.0


def test_hysteresis_keeps_the_current_goal_unless_clearly_worse():
    x = np.array([0.0, 5.0, -5.2])
    cost = np.abs(x[:, None] - x[None])
    # Going to 2 (-5.2) first costs 15.4, going to 1 first costs 15.2: keep 2.
    nxt, _, switched = pick(cost, current=2, hysteresis=0.15)
    assert nxt == 2 and not switched
    x = np.array([0.0, 1.0, -8.0])
    cost = np.abs(x[:, None] - x[None])
    nxt, _, switched = pick(cost, current=2, hysteresis=0.15)   # 10 vs 17
    assert nxt == 1 and switched
    nxt, _, switched = pick(cost)
    assert nxt == 1 and not switched
