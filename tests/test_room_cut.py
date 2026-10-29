"""3.118 criterion 1 (fast tier): the fast `segment()` and `clearance()`
give exactly the frozen copy's answers -- the same room ids and the
clearance bit for bit -- on random grids, the suite's own two-room map, one
live SLAM map and the simulator's houses. The long version (every live map,
120 houses, 500 large random grids, every cut of the 9 sim surveys) is
`test_room_cut_sweep.py`."""
import random

import numpy as np
import pytest

from brain import room_labels as rl
from tests import frontier_compare as fc
from tests import room_cut_compare as rc
from tests import room_cut_reference as ref
from tests.test_survey import _two_rooms
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN


def test_random_small_grids_give_the_old_answers():
    rng = random.Random(3118)
    bad = []
    for n in range(200):
        m = rc.random_map(rng, 40)
        bad += [(n, m["width"], m["height"], *b) for b in rc.differences(m)]
    assert bad == []


def test_the_two_room_map_gives_the_old_answers():
    assert rc.differences(_two_rooms()) == []


def test_a_live_slam_map_gives_the_old_answers():
    assert rc.differences(fc.load(fc.MAPS[0]), ratios=(rl.NECK_RATIO,)) == []


@pytest.mark.parametrize("house", ["starter_house", "scaled_house", "home_first_floor",
                                   "complex_house"])
def test_the_sim_houses_give_the_old_answers(house):
    assert rc.differences(rc.house_map(house)) == []


@pytest.mark.parametrize("cells", [
    [CELL_FREE], [CELL_OCCUPIED], [CELL_UNKNOWN],
    [CELL_FREE] * 7, [CELL_UNKNOWN, CELL_FREE, CELL_OCCUPIED, CELL_FREE],
], ids=["one-free", "one-occupied", "one-unknown", "a-row", "a-mixed-row"])
def test_one_cell_and_one_row_maps_give_the_old_answers(cells):
    for w, h in ((len(cells), 1), (1, len(cells))):
        assert rc.differences({"width": w, "height": h, "resolution_m": 0.1,
                               "cells": cells}) == []


def test_clearance_alone_is_the_old_one_bit_for_bit():
    rng = np.random.default_rng(3118)
    for _ in range(50):
        h, w = rng.integers(1, 60, size=2)
        g = rng.choice([CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN], size=(h, w), p=[0.7, 0.1, 0.2])
        free, occ = g == CELL_FREE, g == CELL_OCCUPIED
        assert np.array_equal(rl.clearance(free, occ), ref.clearance(free, occ))


def test_an_empty_grid_gives_the_old_clearance():
    free = np.zeros((0, 0), dtype=bool)
    got, want = rl.clearance(free, free), ref.clearance(free, free)
    assert got.shape == want.shape == (0, 0) and got.dtype == want.dtype
    for shape in ((0, 3), (3, 0)):             # the old one raised here: so does the new
        free = np.zeros(shape, dtype=bool)
        for f in (ref.clearance, rl.clearance):
            with pytest.raises(IndexError):
                f(free, free)
