"""Frontier detection and goal choice on an occupancy grid (no ROS imports).

A frontier cell is a free cell next to unknown space. Neighbouring frontier cells
form a frontier; the explorer drives to one, which turns unknown space into map.
Grid values follow nav_msgs/OccupancyGrid: -1 unknown, 0..100 occupancy.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

FREE_MAX = 25  # cells at or below this occupancy count as free
CROSS = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))


def _dilate(mask, iterations=1):
    return cv2.dilate(mask.astype(np.uint8), CROSS, iterations=iterations).astype(bool)


def _label(mask):
    """8-connected components: (labels, count); label 0 is the background."""
    n, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    return labels, n - 1


def _clearance(occupied, resolution):
    """Distance in metres from each cell to the nearest occupied cell."""
    return cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * resolution


@dataclass
class Frontier:
    size: int                    # cells
    centroid: tuple              # (x, y) in the map frame, metres
    goal: tuple                  # (x, y) to drive to: clear of obstacles, near the frontier
    cells: np.ndarray = None     # (n, 2) x, y of every cell, metres


def reachable(free, clear, resolution, origin, robot_xy, passage_clearance=0.25, seed_radius=0.5):
    """Cells the robot can reach from robot_xy through known free space.

    Only cells with at least passage_clearance to an occupied cell are passable, so
    gaps narrower than the robot don't connect regions. The robot's own cell may be
    closer to a wall than that; the nearest passable cell within seed_radius seeds
    the fill instead. Returns None when there is no seed (robot off the map).
    """
    passable = free & (clear >= passage_clearance)
    r = int((robot_xy[1] - origin[1]) / resolution)
    c = int((robot_xy[0] - origin[0]) / resolution)
    k = int(math.ceil(seed_radius / resolution))
    r0, r1 = max(r - k, 0), min(r + k + 1, free.shape[0])
    c0, c1 = max(c - k, 0), min(c + k + 1, free.shape[1])
    if r0 >= r1 or c0 >= c1:
        return None
    pr, pc = np.nonzero(passable[r0:r1, c0:c1])
    if pr.size == 0:
        return None
    i = int(np.argmin((pr + r0 - r) ** 2 + (pc + c0 - c) ** 2))
    labels, _ = _label(passable)
    return labels == labels[pr[i] + r0, pc[i] + c0]


def find_frontiers(grid, resolution, origin, min_size=8, obstacle_clearance=0.15,
                   goal_clearance=0.45, goal_reach=1.5, robot_xy=None, passage_clearance=0.25):
    """Frontiers in a (height, width) int8 grid.

    origin is the map position (x, y) of cell (0, 0). Frontier cells closer than
    obstacle_clearance to an occupied cell are dropped (gaps the robot can't see
    through, and edges of walls). Frontiers smaller than min_size cells are noise.
    The goal is the known free cell nearest the frontier's centroid with at least
    goal_clearance to any occupied cell, within goal_reach of it; a frontier without
    one (a pocket the robot does not fit into) is dropped. goal_clearance=0 puts the
    goal on the frontier itself.
    With robot_xy, goals must also be reachable from the robot (see reachable), so
    frontiers behind walls, e.g. free space mapped through glass or over a low wall,
    are dropped.
    """
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid <= FREE_MAX)
    unknown = grid < 0
    occupied = grid > FREE_MAX
    # 4-connected: a free cell with an unknown cell directly beside it
    near_unknown = _dilate(unknown)
    frontier = free & near_unknown
    if obstacle_clearance > 0 and occupied.any():
        r = max(1, math.ceil(obstacle_clearance / resolution - 1e-6))  # at least the clearance
        frontier &= ~_dilate(occupied, iterations=r)
    labels, n = _label(frontier)
    clear = _clearance(occupied, resolution)
    reach = None
    if robot_xy is not None:
        reach = reachable(free, clear, resolution, origin, robot_xy, passage_clearance)
    if goal_clearance > 0:
        ok = free & (clear >= goal_clearance)
        if reach is not None:
            ok &= reach
        gr, gc = np.nonzero(ok)
        gx = origin[0] + (gc + 0.5) * resolution
        gy = origin[1] + (gr + 0.5) * resolution
    out = []
    for i in range(1, n + 1):
        rows, cols = np.nonzero(labels == i)
        if rows.size < min_size:
            continue
        xs = origin[0] + (cols + 0.5) * resolution
        ys = origin[1] + (rows + 0.5) * resolution
        cx, cy = float(xs.mean()), float(ys.mean())
        if goal_clearance > 0:
            if gx.size == 0:
                continue
            d2 = (gx - cx) ** 2 + (gy - cy) ** 2
            k = int(np.argmin(d2))
            if d2[k] > goal_reach ** 2:
                continue
            goal = (float(gx[k]), float(gy[k]))
        else:
            if reach is not None:
                inside = reach[rows, cols]
                if not inside.any():
                    continue
                xs_, ys_ = xs[inside], ys[inside]
            else:
                xs_, ys_ = xs, ys
            k = int(np.argmin((xs_ - cx) ** 2 + (ys_ - cy) ** 2))
            goal = (float(xs_[k]), float(ys_[k]))
        out.append(Frontier(int(rows.size), (cx, cy), goal, np.column_stack([xs, ys])))
    return out


def target_cell(f):
    """The frontier cell nearest the goal: what to watch for being mapped.

    The goal itself sits up to goal_reach back from the frontier, in known space.
    """
    i = int(np.argmin(np.hypot(*(f.cells - np.asarray(f.goal)).T)))
    return float(f.cells[i, 0]), float(f.cells[i, 1])


def still_frontier(frontiers, goal, radius=0.4):
    """True while any frontier cell lies within radius of goal.

    A frontier shrinks and its centre moves as the robot maps it, so comparing
    goals would drop a goal that is still worth driving to.
    """
    g = np.asarray(goal)
    return any(f.cells is not None and len(f.cells)
               and np.min(np.hypot(*(f.cells - g).T)) < radius for f in frontiers)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def score(f, robot_xy, robot_yaw, size_weight=0.02, turn_weight=0.5):
    """Higher is better: large, near, and ahead of the robot.

    The sensors only see the front half, so frontiers behind cost a turn first.
    Distance is metres; size_weight is metres of distance one cell is worth;
    turn_weight is metres per radian of turning.
    """
    dx, dy = f.goal[0] - robot_xy[0], f.goal[1] - robot_xy[1]
    dist = math.hypot(dx, dy)
    turn = abs(wrap(math.atan2(dy, dx) - robot_yaw)) if dist > 1e-6 else 0.0
    return size_weight * f.size - dist - turn_weight * turn


def choose(frontiers, robot_xy, robot_yaw, blacklist=(), blacklist_radius=0.5, min_distance=0.3):
    """Best frontier, skipping goals near a blacklisted point or under the robot."""
    best, best_score = None, -math.inf
    for f in frontiers:
        if any(math.dist(f.goal, b) < blacklist_radius for b in blacklist):
            continue
        if math.dist(f.goal, robot_xy) < min_distance:
            continue
        s = score(f, robot_xy, robot_yaw)
        if s > best_score:
            best, best_score = f, s
    return best
