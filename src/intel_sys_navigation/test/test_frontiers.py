import math

import numpy as np

from intel_sys_navigation.frontiers import Chunk, Grid, frontier_cells, frontier_chunks, split, still_frontier

RES = 0.1


def room(free_cols, rows=20, cols=20):
    """Unknown grid with free columns [0, free_cols) and walls top and bottom."""
    g = np.full((rows, cols), -1, dtype=np.int8)
    g[:, :free_cols] = 0
    g[0, :free_cols] = 100
    g[-1, :free_cols] = 100
    return Grid(g, RES, (0.0, 0.0))


def test_grid_cells_and_coordinates():
    g = room(10)
    assert g.cell((0.95, 0.05)) == (0, 9)
    x, y = g.xy(0, 9)
    assert math.isclose(x, 0.95) and math.isclose(y, 0.05)
    assert not g.inside(np.array(-1), np.array(0))


def test_frontier_is_the_free_edge_next_to_unknown():
    mask = frontier_cells(room(10))
    assert mask[:, 9][1:-1].all()          # last free column, between the walls
    assert not mask[:, :9].any() and not mask[:, 10:].any()


def test_no_frontier_when_fully_known():
    g = np.zeros((10, 10), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    assert frontier_chunks(Grid(g, RES, (0.0, 0.0)), min_size=1) == []


def test_small_clusters_are_noise():
    g = np.zeros((10, 10), dtype=np.int8)
    g[5, 5] = -1   # one unknown cell: four frontier cells around it
    assert frontier_chunks(Grid(g, RES, (0.0, 0.0)), min_size=8) == []
    assert len(frontier_chunks(Grid(g, RES, (0.0, 0.0)), min_size=1)) == 1


def test_long_frontier_is_split_into_chunks_one_viewpoint_can_cover():
    g = room(10, rows=60)                  # a 6 m long frontier edge
    chunks = frontier_chunks(g, min_size=3, max_radius=1.5)
    assert len(chunks) >= 2
    for c in chunks:
        assert np.max(np.hypot(*(c.cells - c.centroid).T)) <= 1.5 + 1e-9
    assert sum(len(c.cells) for c in chunks) == int(frontier_cells(g).sum())


def test_split_keeps_small_sets_whole():
    pts = np.array([[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]])
    assert len(split(pts, 1.5)) == 1
    assert len(split(np.array([[0.0, 0.0], [4.0, 0.0]]), 1.5)) == 2


def test_origin_offsets_cells():
    g = room(10)
    g.origin = (-1.0, 2.0)
    c = frontier_chunks(g, min_size=3)[0]
    assert np.allclose(c.cells[:, 0], -0.05)
    assert c.centroid[1] > 2.0


def test_goal_stays_valid_while_its_frontier_shrinks():
    chunks = frontier_chunks(room(10), min_size=3)
    target = tuple(chunks[0].cells[len(chunks[0].cells) // 2])
    g = room(10)
    g.data[:9, 10:] = 0          # the top half of the unknown side gets mapped
    assert still_frontier(frontier_chunks(g, min_size=3), target, radius=0.6)
    done = Grid(np.zeros((20, 20), np.int8), RES, (0.0, 0.0))
    assert not still_frontier(frontier_chunks(done, min_size=3), target)


def test_still_frontier_on_empty_chunks():
    assert not still_frontier([Chunk(np.zeros((0, 2)), (0.0, 0.0))], (0.0, 0.0))
