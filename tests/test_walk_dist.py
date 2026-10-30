"""3.120 criterion 1 (fast tier): `_walk_dist()` gives exactly the frozen
`_walk()`'s distances on random cuts, the suite's two-room map, one live
SLAM map and the simulator's houses, with seeds on floor, on walls, off the
map and repeated. The long version is `test_walk_dist_sweep.py`."""
import random

import numpy as np
import pytest

from brain import room_labels as rl
from tests import frontier_compare as fc
from tests import room_cut_compare as rc
from tests import walk_compare as wc
from tests import walk_reference as ref
from tests.test_survey import _two_rooms


def test_random_small_cuts_give_the_old_distances():
    rng = random.Random(3120)
    bad = []
    for n in range(200):
        m = rc.random_map(rng, 40)
        bad += [(n, m["width"], m["height"], *b)
                for b in wc.map_differences(rng, m, counts=(0, 1, 3, 8))]
    assert bad == []


def test_the_two_room_map_gives_the_old_distances():
    assert wc.map_differences(random.Random(1), _two_rooms()) == []


def test_a_live_slam_map_gives_the_old_distances():
    assert wc.map_differences(random.Random(2), fc.load(fc.MAPS[0]), counts=(20,)) == []


@pytest.mark.parametrize("house", ["starter_house", "scaled_house", "home_first_floor",
                                   "complex_house"])
def test_the_sim_houses_give_the_old_distances(house):
    assert wc.map_differences(random.Random(3), rc.house_map(house)) == []


def test_a_cut_with_no_floor_or_no_cells():
    for seg in (np.full((4, 5), -1), np.zeros((0, 0), dtype=int)):
        assert wc.differences(seg, [(0, 0), (-1, -1)], 0.05) == []
        assert wc.differences(seg, [], 0.05) == []


def test_wants_reads_the_fast_walk(monkeypatch):
    """`_wants()` -- a survey's per-cut question -- uses `_walk_dist()`, and
    asks the same as it did on `_walk()`'s distances."""
    calls = []
    fast = rl._walk_dist

    def spy(*a):
        calls.append(1)
        return fast(*a)

    monkeypatch.setattr(rl, "_walk_dist", spy)
    m = _two_rooms()
    lab = rl.RoomLabeller(lambda frame: None, mode="spacing")
    seg, _ = rl.segment(m)
    fl = np.argwhere(seg >= 0)
    x0, y0 = (fl[0][1] + 0.5) * m["resolution_m"], (fl[0][0] + 0.5) * m["resolution_m"]
    lab.seeds = [(x0, y0, "kitchen")]
    got = [lab._wants(m, (i + 0.5) * m["resolution_m"], (j + 0.5) * m["resolution_m"])
           for j, i in fl]
    assert calls
    d0, _ = ref._walk(seg, [(int(fl[0][0]), int(fl[0][1]))], m["resolution_m"], rl.DOOR_M)
    assert got == [bool(d0[j, i] > lab.spacing_m) for j, i in fl]
