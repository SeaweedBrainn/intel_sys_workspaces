"""Frontier detection on an occupancy grid (no ROS imports).

A frontier cell is a free cell next to unknown space. Neighbouring frontier cells
form a cluster, and clusters wider than a sensor footprint are split into chunks,
each small enough for one viewpoint to see (as in FUEL, Zhou et al. 2021).
Grid values follow nav_msgs/OccupancyGrid: -1 unknown, 0..100 occupancy.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

FREE_MAX = 25  # map cells at or below this occupancy count as free
CROSS = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))


@dataclass
class Grid:
    """An OccupancyGrid as an array: data[row, col]; origin is the corner of cell (0, 0)."""
    data: np.ndarray
    res: float
    origin: tuple

    def cell(self, xy):
        """(row, col) of a map point, possibly outside the grid."""
        return (int(math.floor((xy[1] - self.origin[1]) / self.res)),
                int(math.floor((xy[0] - self.origin[0]) / self.res)))

    def xy(self, rows, cols):
        """Map coordinates of cell centres."""
        return (self.origin[0] + (np.asarray(cols) + 0.5) * self.res,
                self.origin[1] + (np.asarray(rows) + 0.5) * self.res)

    def inside(self, rows, cols):
        h, w = self.data.shape
        return (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)


@dataclass
class Chunk:
    cells: np.ndarray   # (n, 2) x, y of its frontier cells, metres
    centroid: tuple     # (x, y)


def frontier_cells(grid):
    """Boolean mask of free cells with an unknown cell directly beside them."""
    free = (grid.data >= 0) & (grid.data <= FREE_MAX)
    unknown = cv2.dilate((grid.data < 0).astype(np.uint8), CROSS).astype(bool)
    return free & unknown


def split(points, max_radius):
    """Split points along their main axis until every piece fits in max_radius."""
    c = points.mean(axis=0)
    if len(points) < 2 or np.max(np.hypot(*(points - c).T)) <= max_radius:
        return [points]
    _, _, vt = np.linalg.svd(points - c, full_matrices=False)
    side = (points - c) @ vt[0] > 0
    return split(points[side], max_radius) + split(points[~side], max_radius)


def frontier_chunks(grid, min_size=8, max_radius=1.5):
    """Frontier clusters (8-connected, at least min_size cells), split into chunks."""
    n, labels = cv2.connectedComponents(frontier_cells(grid).astype(np.uint8), connectivity=8)
    flat = labels.ravel()
    idx = np.flatnonzero(flat)
    if idx.size == 0:
        return []
    order = idx[np.argsort(flat[idx], kind='stable')]
    counts = np.bincount(flat[order], minlength=n)[1:]
    out = []
    for cells in np.split(order, np.cumsum(counts)[:-1]):
        if cells.size < min_size:
            continue
        rows, cols = np.divmod(cells, grid.data.shape[1])
        pts = np.column_stack(grid.xy(rows, cols))
        for part in split(pts, max_radius):
            c = part.mean(axis=0)
            out.append(Chunk(part, (float(c[0]), float(c[1]))))
    return out


def still_frontier(chunks, point, radius=0.4):
    """True while any frontier cell lies within radius of point.

    A frontier shrinks and its centre moves as the robot maps it, so the goal stays
    valid while cells remain near the one it was aimed at.
    """
    p = np.asarray(point)
    return any(len(c.cells) and np.min(np.hypot(*(c.cells - p).T)) < radius for c in chunks)
