"""3.77: the numpy frontier search gives exactly the answers of the
pure-Python one it replaced. The full criterion-1 sweep is
`test_frontier_fast_sweep.py` (slow tier); this is its fast-tier sample."""
import random

import numpy as np

from brain import frontier as new
from tests import frontier_compare as fc
from tests import frontier_reference as old


def test_a_live_slam_map_gives_the_old_answers():
    rng = random.Random(fc.SEED)
    m = fc.load(fc.MAPS[0])
    seens = (set(), fc.seen_set(m, 8, rng))
    for x, y in fc.poses(m, 3, rng):
        assert fc.differences(m, x, y, fc.LEAVE, seens) == []


def test_the_grid_passes_match_on_a_live_map():
    m = fc.load(fc.MAPS[1])
    assert new._clearance(m, 8) == old._clearance(m, 8)
    assert new.frontier_cells(m) == old.frontier_cells(m)
    # The set's own iteration order decides `_clusters`' order.
    assert list(new.frontier_cells(m)) == list(old.frontier_cells(m))
    cells = old.frontier_cells(m)
    assert new._clusters(cells) == old._clusters(cells)
    rng = random.Random(1)
    for x, y in fc.poses(m, 5, rng):
        view = rng.uniform(0, 360)
        assert new.camera_seen(m, x, y, view) == old.camera_seen(m, x, y, view)


def test_components_join_diagonal_runs_and_nothing_else():
    mask = np.array([[1, 1, 0, 0, 1],
                     [0, 0, 1, 0, 0],
                     [1, 0, 0, 0, 1],
                     [1, 0, 1, 1, 1]], dtype=bool)
    labels, n = new._components(mask)
    assert n == 4
    assert labels[0, 0] == labels[0, 1] == labels[1, 2]           # a diagonal step
    assert labels[2, 0] == labels[3, 0]
    assert labels[2, 4] == labels[3, 2] == labels[3, 4] != labels[0, 4]
    assert (labels[~mask] == -1).all()


def test_off_map_and_unusable_answer_as_before():
    m = fc.load(fc.MAPS[0])
    far = (m["origin_x_m"] - 5.0, m["origin_y_m"] - 5.0)
    assert new.reachable(m, *far) == old.reachable(m, *far)
    assert new.find_frontiers(m, *far) == [] and new.find_view_gaps(m, *far, set()) == []
    dead = {**m, "usable": False}
    assert new.reachable(dead, 0.0, 0.0) == old.reachable(dead, 0.0, 0.0)
    assert len(new.reach(dead, 0.0, 0.0)) == 0


def test_a_reach_for_another_pose_or_clearance_is_refused():
    import pytest
    m = fc.load(fc.MAPS[0])
    (x, y), (x2, y2) = fc.poses(m, 2, random.Random(5))
    r = new.reach(m, x, y, fc.LEAVE)
    with pytest.raises(ValueError):
        new.find_frontiers(m, x2, y2, clearance_m=fc.LEAVE, reach_map=r)
    with pytest.raises(ValueError):
        new.find_view_gaps(m, x, y, set(), clearance_m=0.1, reach_map=r)
