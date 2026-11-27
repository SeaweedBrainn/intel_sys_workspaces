"""Tour-based goal choice for frontier exploration (no ROS imports).

For every frontier chunk (frontiers.frontier_chunks) a viewpoint is sampled
nearby: the reachable spot from which the sensors, which only see the front
180 degrees, see the most of the chunk's frontier cells, and the heading to see
them from. Then the order of visiting all viewpoints is planned as an open
travelling-salesman path from the robot, with costs in seconds (turn, drive the
path, turn). The robot drives to the first stop; the tour is re-planned as the
map grows, and the goal only changes when another tour is clearly cheaper.

Where the robot can drive comes from one place, Nav2's global costmap: known
cells cheaper than "inscribed" (the robot's radius from an obstacle). So a
viewpoint this module calls reachable is one Nav2 can plan to.

After FUEL (Zhou et al., RA-L 2021): frontier chunks, viewpoints scored by the
frontier cells they cover, an asymmetric TSP over them, hysteresis.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from intel_sys_navigation.frontiers import FREE_MAX

INSCRIBED = 99   # nav2 costmap as OccupancyGrid: 99 inscribed, 100 lethal, -1 unknown


@dataclass
class Candidate:
    chunk: object      # frontiers.Chunk
    xy: tuple          # viewpoint (the goal), map frame
    gain: int          # frontier cells of the chunk visible from it
    heading: float     # goal yaw: the centre of the cells seen
    new_area: float = 0.0   # m^2 of unknown space in view from there, facing heading


def coverage(grid, xy, cells, fov=math.pi, max_range=5.0):
    """(count, heading): how many of cells a sensor at xy sees within one fov-wide view.

    A cell is visible when it is within max_range and the straight line to it
    crosses no occupied map cell. The fov window holding the most visible cells
    gives the count, and their middle direction the heading.
    """
    cells = np.asarray(cells, dtype=float)
    d = cells - np.asarray(xy, dtype=float)
    dist = np.hypot(d[:, 0], d[:, 1])
    m = (dist <= max_range) & (dist > 1e-6)
    if not m.any():
        return 0, 0.0
    d, dist = d[m], dist[m]
    k = max(2, int(math.ceil(dist.max() / (grid.res / 2))) + 1)
    t = np.linspace(0.0, 1.0, k)
    px = xy[0] + d[:, 0:1] * t
    py = xy[1] + d[:, 1:2] * t
    rows = np.floor((py - grid.origin[1]) / grid.res).astype(int)
    cols = np.floor((px - grid.origin[0]) / grid.res).astype(int)
    inside = grid.inside(rows, cols)
    occ = np.zeros(rows.shape, dtype=bool)
    occ[inside] = grid.data[rows[inside], cols[inside]] > FREE_MAX
    seen = ~occ.any(axis=1)
    if not seen.any():
        return 0, 0.0
    a = np.sort(np.arctan2(d[seen, 1], d[seen, 0]))
    ring = np.concatenate([a, a + 2 * math.pi])
    end = np.searchsorted(ring, a + fov, side='left')       # first angle past each window
    counts = end - np.arange(len(a))
    i = int(np.argmax(counts))
    last = ring[end[i] - 1]
    return int(counts[i]), math.atan2(math.sin((a[i] + last) / 2), math.cos((a[i] + last) / 2))


def unknown_area(grid, xy, heading, fov=math.pi, max_range=5.0, rays=72):
    """m^2 of unknown space a sensor at xy facing heading would see: what a visit is worth.

    Frontier cells seen say how much edge a viewpoint covers, not how much new space is
    behind it; a sliver in front of a wall has cells but nothing to map. Rays stop at
    the first occupied cell; unknown space doesn't stop them (it may well be free). Cells
    off the map count as unknown, since the map grows as it is explored.
    """
    h, w = grid.data.shape
    th = heading + (np.arange(rays) - (rays - 1) / 2) * (fov / rays)
    dr = grid.res / 2
    r = np.arange(dr, max_range + 1e-9, dr)
    x = xy[0] + np.outer(np.cos(th), r)
    y = xy[1] + np.outer(np.sin(th), r)
    col = np.floor((x - grid.origin[0]) / grid.res).astype(int)
    row = np.floor((y - grid.origin[1]) / grid.res).astype(int)
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    v = np.full(x.shape, -1, dtype=np.int16)
    v[inside] = grid.data[row[inside], col[inside]]
    blocked = np.maximum.accumulate(v > FREE_MAX, axis=1)
    unknown = (v < 0) & ~blocked
    return float((unknown * r).sum() * (fov / rays) * dr)


def samples(centroid, radii=(0.75, 1.25), n=12):
    """Candidate viewpoints on rings around a chunk's centroid, plus the centroid."""
    th = np.arange(n) * 2 * math.pi / n
    pts = [np.asarray(centroid, dtype=float)[None, :]]
    for r in radii:
        pts.append(np.column_stack([centroid[0] + r * np.cos(th), centroid[1] + r * np.sin(th)]))
    return np.vstack(pts)


def best_viewpoint(chunk, grid, graph, robot_dist, fov=math.pi, max_range=5.0, skip=(), max_cost=30):
    """The reachable sample that sees the most of the chunk (ties: nearer the robot), or None.

    robot_dist is graph.distances_from(robot); skip holds points (with radius) to avoid,
    e.g. blacklisted goals. A viewpoint must also sit on a costmap cell no dearer than
    max_cost: room to stop and turn. Drivable alone (under inscribed) is only the robot's
    radius from a wall, and goals there wedged the robot on a door jamb it couldn't back
    out of (office.sdf, 2026-11-26; and on 2026-11-24 before goals kept their distance).
    """
    best = None
    for p in samples(chunk.centroid):
        node = graph.node_at(p)
        if node is None or not np.isfinite(robot_dist[node]):
            continue
        r, c = graph.rc[node]
        if graph.grid.data[r, c] > max_cost:
            continue
        if any(math.dist(p, q) < r for q, r in skip):
            continue
        gain, heading = coverage(grid, p, chunk.cells, fov, max_range)
        key = (gain, -robot_dist[node])
        if gain > 0 and (best is None or key > best[0]):
            best = (key, Candidate(chunk, (float(p[0]), float(p[1])), gain, heading))
    if best is None:
        return None
    c = best[1]
    c.new_area = unknown_area(grid, c.xy, c.heading, fov, max_range)
    return c


class PathGraph:
    """Shortest drivable paths on Nav2's global costmap.

    Drivable: known and cheaper than inscribed, i.e. where Nav2's planner may put the
    robot's centre. Unknown cells are left out, so a frontier is only reachable through
    mapped space (space seen through glass or over a low wall is not).
    """

    def __init__(self, costmap):
        data = costmap.data
        self.grid = costmap
        self.passable = (data >= 0) & (data < INSCRIBED)
        self.res = costmap.res
        self.origin = costmap.origin
        H, W = data.shape
        self.index = np.full(data.shape, -1, dtype=np.int64)
        rows, cols = np.nonzero(self.passable)
        self.index[rows, cols] = np.arange(rows.size)
        self.rc = np.column_stack([rows, cols])
        src, dst, wt = [], [], []
        for dr, dc, d in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
            a = self.index[max(0, -dr):H - max(0, dr), max(0, -dc):W - max(0, dc)]
            b = self.index[max(0, dr):H - max(0, -dr), max(0, dc):W - max(0, -dc)]
            ok = (a >= 0) & (b >= 0)
            src.append(a[ok])
            dst.append(b[ok])
            wt.append(np.full(int(ok.sum()), d * self.res))
        n = rows.size
        self.graph = coo_matrix((np.concatenate(wt), (np.concatenate(src), np.concatenate(dst))),
                                shape=(n, n)).tocsr()

    def node_at(self, xy):
        """The drivable cell under xy, or None."""
        r, c = self.grid.cell(xy)
        if not self.grid.inside(r, c):
            return None
        i = int(self.index[r, c])
        return None if i < 0 else i

    def distances_from(self, xy):
        """Path length from xy to every node (inf where unreachable); all inf if xy is off the graph."""
        node = self.node(xy)
        if node is None:
            return np.full(len(self.rc), np.inf)
        return dijkstra(self.graph, directed=False, indices=node)

    def xy(self, node):
        r, c = self.rc[node]
        return (self.origin[0] + (c + 0.5) * self.res, self.origin[1] + (r + 0.5) * self.res)

    def node(self, xy, radius=0.5):
        """Nearest drivable cell within radius of xy, or None (the robot may stand in inflation)."""
        r = int(math.floor((xy[1] - self.origin[1]) / self.res))
        c = int(math.floor((xy[0] - self.origin[0]) / self.res))
        k = int(math.ceil(radius / self.res))
        H, W = self.passable.shape
        r0, r1, c0, c1 = max(r - k, 0), min(r + k + 1, H), max(c - k, 0), min(c + k + 1, W)
        if r0 >= r1 or c0 >= c1:
            return None
        pr, pc = np.nonzero(self.passable[r0:r1, c0:c1])
        if pr.size == 0:
            return None
        i = int(np.argmin((pr + r0 - r) ** 2 + (pc + c0 - c) ** 2))
        return int(self.index[pr[i] + r0, pc[i] + c0])

    def legs(self, points, look=1.0):
        """Path length, departure bearing and arrival bearing between every pair of points.

        Returns three (k, k) arrays. Lengths are inf where there is no path. The
        bearings point along the path about look metres from each end, which is the
        way the robot faces when it sets off and when it arrives.
        """
        k = len(points)
        dist = np.full((k, k), np.inf)
        depart = np.zeros((k, k))
        arrive = np.zeros((k, k))
        nodes = [self.node(p) for p in points]
        valid = [i for i, n in enumerate(nodes) if n is not None]
        if not valid:
            return dist, depart, arrive
        d, pred = dijkstra(self.graph, directed=False, indices=[nodes[i] for i in valid],
                           return_predecessors=True)
        steps = max(1, int(round(look / self.res)))
        for a, i in enumerate(valid):
            for j in valid:
                if i == j:
                    dist[i, j] = 0.0
                    continue
                dist[i, j] = d[a, nodes[j]]
                if not np.isfinite(dist[i, j]):
                    continue
                path = [nodes[j]]
                while path[-1] != nodes[i]:
                    path.append(int(pred[a, path[-1]]))
                path.reverse()
                p, q = points[i], points[j]
                ahead = self.xy(path[min(steps, len(path) - 1)])
                behind = self.xy(path[max(len(path) - 1 - steps, 0)])
                depart[i, j] = _bearing(p, ahead if math.dist(p, ahead) > 0.05 else q)
                arrive[i, j] = _bearing(behind if math.dist(behind, q) > 0.05 else p, q)
        return dist, depart, arrive


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _bearing(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def leg_costs(dist, depart, arrive, yaws, speed=0.2, turn_rate=0.6):
    """Seconds to go from point i (facing yaws[i]) to point j and face yaws[j].

    Turn to face the path, drive it, turn to the goal heading. Every turn is counted
    in full, whatever the controller does with it: the sensors only see forward, so
    the robot has to face its way and its goal either way. speed and turn_rate are
    the top forward speed and turn rate (Nav2's velocity smoother limits).
    """
    yaws = np.asarray(yaws, dtype=float)
    turn_out = np.abs(_wrap(depart - yaws[:, None]))
    turn_in = np.abs(_wrap(yaws[None, :] - arrive))
    cost = (turn_out + turn_in) / turn_rate + dist / speed
    near = dist < 1e-9   # same spot: just turn
    cost[near] = np.abs(_wrap(yaws[None, :] - yaws[:, None]))[near] / turn_rate
    np.fill_diagonal(cost, 0.0)
    return cost


def solve(cost, first=None, exact_max=9):
    """Cheapest open path from node 0 through every other node: (order, total seconds).

    order lists the other nodes in visiting order. first fixes the first stop. Up to
    exact_max stops it is exact (Held-Karp dynamic programming); beyond that it is
    nearest neighbour improved by moving segments of up to three stops (or-opt), which
    keeps each segment's direction, as asymmetric costs need.
    """
    cost = np.asarray(cost, dtype=float)
    rest = list(range(1, len(cost)))
    if not rest:
        return [], 0.0
    if first is not None:
        others = [i for i in rest if i != first]
        sub = cost[np.ix_([first] + others, [first] + others)]
        order, total = solve(sub, exact_max=exact_max)
        return [first] + [others[i - 1] for i in order], float(cost[0, first] + total)
    if len(rest) <= exact_max:
        return _held_karp(cost)
    return _or_opt(cost, _nearest_neighbour(cost))


def _held_karp(cost):
    m = len(cost) - 1
    c = cost[1:, 1:]
    full = 1 << m
    dp = np.full((full, m), np.inf)
    parent = np.full((full, m), -1, dtype=int)
    for j in range(m):
        dp[1 << j, j] = cost[0, j + 1]
    for mask in range(1, full):
        ends = [j for j in range(m) if mask >> j & 1 and np.isfinite(dp[mask, j])]
        if not ends:
            continue
        for k in range(m):
            if mask >> k & 1:
                continue
            via = dp[mask, ends] + c[ends, k]
            b = int(np.argmin(via))
            nmask = mask | 1 << k
            if via[b] < dp[nmask, k]:
                dp[nmask, k] = via[b]
                parent[nmask, k] = ends[b]
    j = int(np.argmin(dp[full - 1]))
    total = float(dp[full - 1, j])
    order, mask = [], full - 1
    while j >= 0:
        order.append(j + 1)
        mask, j = mask & ~(1 << j), parent[mask, j]
    return order[::-1], total


def _nearest_neighbour(cost):
    left, at, order = set(range(1, len(cost))), 0, []
    while left:
        at = min(left, key=lambda j: cost[at, j])
        order.append(at)
        left.remove(at)
    return order


def _path_cost(cost, order):
    nodes = [0] + order
    return float(sum(cost[a, b] for a, b in zip(nodes, nodes[1:])))


def _or_opt(cost, order):
    best = _path_cost(cost, order)
    improved = True
    while improved:
        improved = False
        for seg in (1, 2, 3):
            for i in range(len(order) - seg + 1):
                piece, rest = order[i:i + seg], order[:i] + order[i + seg:]
                for k in range(len(rest) + 1):
                    if k == i:
                        continue
                    trial = rest[:k] + piece + rest[k:]
                    c = _path_cost(cost, trial)
                    if c < best - 1e-9:
                        order, best, improved = trial, c, True
                        break
                if improved:
                    break
            if improved:
                break
    return order, best


def pick(cost, current=None, hysteresis=0.15, exact_max=9):
    """Next stop (index into cost, 1-based), the planned order, and whether to switch.

    current is the stop the robot is driving to, if any. It is kept unless the best
    tour is more than `hysteresis` (a fraction) cheaper than the best tour that
    starts with it, so small changes in the map don't flip the goal back and forth.
    """
    order, total = solve(cost, exact_max=exact_max)
    if current is None or not order:
        return (order[0] if order else None), order, current is not None
    keep_order, keep_total = solve(cost, first=current, exact_max=exact_max)
    if total < (1 - hysteresis) * keep_total:
        return order[0], order, order[0] != current
    return current, keep_order, False
