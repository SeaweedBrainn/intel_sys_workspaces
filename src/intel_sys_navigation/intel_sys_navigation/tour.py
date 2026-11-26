"""Tour-based goal choice for frontier exploration (no ROS imports).

The greedy explorer drives to whichever frontier scores best right now, which
zig-zags and leaves small patches behind. Here every frontier cluster gets a
viewpoint (its goal), the unknown area the sensors would see from it (gain) and
the heading that sees the most (the sensors only cover the front half). Then the
order of visiting all of them is planned as an open travelling-salesman path
from the robot, with costs in seconds: turn, drive the real path, turn. The robot
drives to the first stop; the tour is re-planned as the map grows, and the goal
only changes when another tour is clearly cheaper (hysteresis).

The method follows FUEL (Zhou et al., RA-L 2021) and roadmap-explorer
(FIT-SLAM 2): field-of-view gain and goal heading per frontier, a visiting order
over all of them, and hysteresis before switching.

Grid values follow nav_msgs/OccupancyGrid: -1 unknown, 0..100 occupancy.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from intel_sys_navigation.frontiers import FREE_MAX, _clearance, wrap


@dataclass
class Candidate:
    frontier: object   # frontiers.Frontier
    gain: float        # m^2 of unknown space in the sensors' view from the goal
    heading: float     # goal yaw: the centre of that view


def view_gain(grid, resolution, origin, xy, fov=math.pi, max_range=5.0, rays=72):
    """(gain, heading): the unknown area a sensor with this field of view sees from xy.

    Rays are cast all the way round and stop at the first occupied cell; unknown
    space does not block them (it may well be free). Cells off the map count as
    unknown, since the map grows as it is explored. Each sample is weighted by the
    area it stands for, so gain is in m^2. The fov-wide window with the most unknown
    area gives the gain, and its centre the heading.
    """
    grid = np.asarray(grid)
    h, w = grid.shape
    dth = 2 * math.pi / rays
    th = np.arange(rays) * dth
    dr = resolution / 2
    r = np.arange(dr, max_range + 1e-9, dr)
    x = xy[0] + np.outer(np.cos(th), r)
    y = xy[1] + np.outer(np.sin(th), r)
    col = np.floor((x - origin[0]) / resolution).astype(int)
    row = np.floor((y - origin[1]) / resolution).astype(int)
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    v = np.full(x.shape, -1, dtype=np.int16)
    v[inside] = grid[row[inside], col[inside]]
    blocked = np.maximum.accumulate(v > FREE_MAX, axis=1)
    unknown = (v < 0) & ~blocked
    per_ray = (unknown * r).sum(axis=1) * dth * dr
    n = min(rays, max(1, int(round(fov / dth))))
    ring = np.concatenate([per_ray, per_ray[:n - 1]])
    sums = np.convolve(ring, np.ones(n), mode='valid')[:rays]
    s = int(np.argmax(sums))
    return float(sums[s]), wrap(th[s] + (n - 1) / 2 * dth)


def candidates(frontiers, grid, resolution, origin, fov=math.pi, max_range=5.0, min_gain=0.5):
    """A Candidate per frontier whose goal would reveal at least min_gain m^2."""
    out = []
    for f in frontiers:
        gain, heading = view_gain(grid, resolution, origin, f.goal, fov, max_range)
        if gain >= min_gain:
            out.append(Candidate(f, gain, heading))
    return out


class PathGraph:
    """Shortest drivable paths over the map, on a coarser grid for speed.

    A cell is drivable when it is known free and at least passage_clearance from an
    occupied cell (the reachability rule in frontiers.py). A coarse cell is drivable
    when any of its fine cells is: the band each wall leaves undrivable is wider
    than a coarse cell, so this does not join rooms through walls.
    """

    def __init__(self, grid, resolution, origin, passage_clearance=0.25, cell=0.15):
        grid = np.asarray(grid)
        free = (grid >= 0) & (grid <= FREE_MAX)
        occupied = grid > FREE_MAX
        passable = free & (_clearance(occupied, resolution) >= passage_clearance)
        f = max(1, int(round(cell / resolution)))
        h, w = passable.shape
        H, W = -(-h // f), -(-w // f)
        padded = np.zeros((H * f, W * f), dtype=bool)
        padded[:h, :w] = passable
        self.passable = padded.reshape(H, f, W, f).any(axis=(1, 3))
        self.res = resolution * f
        self.origin = origin
        self.index = np.full(self.passable.shape, -1, dtype=np.int64)
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

    def xy(self, node):
        r, c = self.rc[node]
        return (self.origin[0] + (c + 0.5) * self.res, self.origin[1] + (r + 0.5) * self.res)

    def node(self, xy, radius=0.5):
        """Nearest drivable coarse cell within radius of xy, or None."""
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


def leg_costs(dist, depart, arrive, yaws, speed=0.2, turn_rate=0.5):
    """Seconds to go from point i (facing yaws[i]) to point j and face yaws[j].

    Nav2's Rotation Shim turns in place to face the path before driving, and the
    goal checker turns to the goal heading on arrival, so the three parts add up.
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
