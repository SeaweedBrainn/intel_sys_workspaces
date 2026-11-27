import itertools
import math

import numpy as np

from intel_sys_navigation.frontiers import Chunk, Grid, frontier_chunks
from intel_sys_navigation.tour import (INSCRIBED, Candidate, PathGraph, best_viewpoint, coverage, leg_costs,
                                       merge, pick, samples, seen_from, solve, unknown_area)

RES = 0.1


def half_known(rows=40, cols=40, free_cols=20):
    """Known free on the left, unknown on the right; the frontier is column free_cols - 1."""
    g = np.full((rows, cols), -1, dtype=np.int8)
    g[:, :free_cols] = 0
    return Grid(g, RES, (0.0, 0.0))


def costmap_from(map_grid, inscribed_cells=2):
    """A costmap like Nav2's: known free 0, obstacles 100, their surroundings 99, unknown -1."""
    d = np.where(map_grid.data < 0, -1, 0).astype(np.int8)
    occ = map_grid.data > 25
    near = occ.copy()
    for _ in range(inscribed_cells):
        n = near.copy()
        n[1:] |= near[:-1]; n[:-1] |= near[1:]; n[:, 1:] |= near[:, :-1]; n[:, :-1] |= near[:, 1:]
        near = n
    d[near] = INSCRIBED
    d[occ] = 100
    return Grid(d, map_grid.res, map_grid.origin)


def test_coverage_counts_visible_frontier_cells_and_faces_them():
    g = half_known()
    chunk = frontier_chunks(g, min_size=3, max_radius=10)[0]
    n, heading = coverage(g, (1.0, 2.0), chunk.cells, fov=math.pi, max_range=5.0)
    assert n == len(chunk.cells)              # the whole edge, 0.95 m away, fits in 180 deg
    assert abs(heading) < math.radians(5)     # facing +x, toward it
    n_back, _ = coverage(g, (1.0, 2.0), chunk.cells, fov=math.radians(30), max_range=5.0)
    assert n_back < n                         # a narrow view sees less


def test_walls_hide_frontier_cells():
    g = half_known()
    chunk = frontier_chunks(g, min_size=3, max_radius=10)[0]
    g.data[:, 15] = 100                       # wall between viewpoint and frontier
    assert coverage(g, (1.0, 2.0), chunk.cells)[0] == 0


def test_range_limits_coverage():
    g = half_known(cols=80, free_cols=70)
    chunk = frontier_chunks(g, min_size=3, max_radius=10)[0]
    assert coverage(g, (0.5, 2.0), chunk.cells, max_range=5.0)[0] == 0   # 6.5 m away


def test_unknown_area_is_what_a_view_is_worth():
    open_side = half_known(cols=120, free_cols=20)
    a = unknown_area(open_side, (1.5, 2.0), heading=0.0, max_range=3.0)
    assert 4.0 < a < math.pi * 9 / 2           # most of a 3 m half-disc
    assert unknown_area(open_side, (1.5, 2.0), heading=math.pi, max_range=1.0) < 0.1   # facing known space
    sliver = half_known(rows=120, cols=120, free_cols=20)   # tall enough that rays stay on the map
    sliver.data[:, 21] = 100                   # a wall right behind the unknown edge
    assert unknown_area(sliver, (1.5, 6.0), heading=0.0, max_range=3.0) < 1.0   # a 10 cm strip, ~6 m of it in view


def wall_room():
    """4 x 4 m free room with a wall from the bottom up to y = 3 m at x = 2 m."""
    g = np.zeros((40, 40), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[:30, 20] = 100
    return Grid(g, RES, (0.0, 0.0))


def test_path_goes_around_the_wall():
    pg = PathGraph(costmap_from(wall_room()))
    dist, depart, arrive = pg.legs([(1.0, 1.0), (3.0, 1.0)])
    assert dist[0, 1] > 4.0                   # up past y = 3 and back down, not 2 m straight
    assert math.isclose(dist[0, 1], dist[1, 0], rel_tol=1e-6)
    assert depart[0, 1] > math.radians(45)
    assert arrive[0, 1] < -math.radians(45)


def test_closed_wall_or_unknown_space_is_unreachable():
    g = wall_room()
    g.data[:, 20] = 100
    pg = PathGraph(costmap_from(g))
    assert np.isinf(pg.legs([(1.0, 1.0), (3.0, 1.0)])[0][0, 1])
    unknown = half_known()                    # unknown cells are not drivable
    pg = PathGraph(costmap_from(unknown))
    assert pg.node_at((3.0, 2.0)) is None and pg.node_at((1.0, 2.0)) is not None


def test_robot_inside_inflation_still_gets_a_start_node():
    pg = PathGraph(costmap_from(wall_room()))
    assert pg.node_at((1.95, 1.0)) is None    # beside the wall: inscribed
    assert pg.node((1.95, 1.0)) is not None   # nearest drivable cell within 0.5 m


def test_best_viewpoint_is_reachable_and_sees_the_frontier():
    m = half_known()
    cm = costmap_from(m)
    pg = PathGraph(cm)
    chunk = frontier_chunks(m, min_size=3, max_radius=1.5)[0]
    c = best_viewpoint(chunk, m, pg, pg.distances_from((0.5, 2.0)))
    assert c is not None and c.gain > 0
    assert cm.data[cm.cell(c.xy)] < INSCRIBED and m.data[m.cell(c.xy)] == 0
    assert abs(c.heading) < math.radians(60)  # looks toward the unknown side (+x)
    assert c.new_area > 1.0


def test_viewpoints_keep_room_from_walls():
    m = half_known()
    cm = costmap_from(m)
    cm.data[(cm.data >= 0) & (cm.data < INSCRIBED)] = 50   # everywhere drivable but tight
    pg = PathGraph(cm)
    chunk = frontier_chunks(m, min_size=3, max_radius=1.5)[0]
    assert best_viewpoint(chunk, m, pg, pg.distances_from((0.5, 2.0)), max_cost=30) is None
    assert best_viewpoint(chunk, m, pg, pg.distances_from((0.5, 2.0)), max_cost=60) is not None


def test_best_viewpoint_respects_skip_and_reachability():
    m = half_known()
    pg = PathGraph(costmap_from(m))
    chunk = frontier_chunks(m, min_size=3, max_radius=1.5)[0]
    everywhere = [(tuple(p), 0.1) for p in samples(chunk.centroid)]
    assert best_viewpoint(chunk, m, pg, pg.distances_from((0.5, 2.0)), skip=everywhere) is None
    assert best_viewpoint(chunk, m, pg, np.full(len(pg.rc), np.inf)) is None


def test_seen_from_only_counts_cells_inside_the_view():
    g = half_known()
    chunk = frontier_chunks(g, min_size=3, max_radius=10)[0]
    assert seen_from(g, (1.0, 2.0), 0.0, chunk.cells) == len(chunk.cells)    # facing it
    assert seen_from(g, (1.0, 2.0), math.pi, chunk.cells) == 0                # facing away
    g.data[:, 15] = 100
    assert seen_from(g, (1.0, 2.0), 0.0, chunk.cells) == 0                    # behind a wall


def viewpoints(m, max_radius):
    pg = PathGraph(costmap_from(m))
    start = pg.distances_from((0.5, 2.0))
    chunks = frontier_chunks(m, min_size=3, max_radius=max_radius)
    return [c for c in (best_viewpoint(ch, m, pg, start) for ch in chunks) if c is not None]


def test_one_stop_for_chunks_one_view_covers():
    m = half_known()                           # a 4 m straight edge, split into ~0.5 m chunks
    cands = viewpoints(m, max_radius=0.5)
    assert len(cands) >= 4
    keep, covers = merge(cands, m)
    assert len(keep) < len(cands)
    assigned = sorted(j for cov in covers for j in cov)
    assert assigned == list(range(len(cands)))  # every chunk belongs to exactly one stop
    for i, cov in zip(keep, covers):
        assert i in cov


def test_no_merge_across_a_wall():
    g = np.full((40, 60), -1, dtype=np.int8)
    g[:, :20] = 0                              # left room, unknown beyond x = 2 m
    g[:, 40:] = 0                              # right room, unknown before x = 4 m
    g[:, 30] = 100                             # and a wall in the unknown between them
    m = Grid(g, RES, (0.0, 0.0))
    chunks = frontier_chunks(m, min_size=3, max_radius=10)
    assert len(chunks) == 2
    a = Candidate(chunks[0], (1.0, 2.0), seen_from(m, (1.0, 2.0), 0.0, chunks[0].cells), 0.0)
    b = Candidate(chunks[1], (5.0, 2.0), seen_from(m, (5.0, 2.0), math.pi, chunks[1].cells), math.pi)
    assert a.gain > 0 and b.gain > 0
    keep, _ = merge([a, b], m)
    assert keep == [0, 1]


def test_current_goal_is_kept_first_and_takes_its_own_chunk():
    m = half_known()
    cands = viewpoints(m, max_radius=0.5)
    weak = cands[0]
    cur = Candidate(weak.chunk, weak.xy, weak.gain, weak.heading)   # the goal being driven to
    keep, covers = merge([cur] + cands, m, first=0)
    assert keep[0] == 0
    assert 1 in covers[0]                      # its chunk's fresh viewpoint gets no stop of its own


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
