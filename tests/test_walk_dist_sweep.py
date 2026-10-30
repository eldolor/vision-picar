"""3.120 criterion 1 (slow tier): `_walk_dist()` gives exactly the frozen
`_walk()`'s distances on every live SLAM map, the sim houses at the car's
0.05 m cells, 100 ProcTHOR houses (20 of them at 0.05 m), 500 random grids,
and every call `_wants()` makes during the 9 in-process `spacing` surveys."""
import random

import numpy as np
import pytest

from brain import room_labels as rl
from tests import frontier_compare as fc
from tests import room_cut_compare as rc
from tests import survey_sweep as sw
from tests import walk_compare as wc
from tests import walk_reference as ref

HOUSES = ["starter_house", "scaled_house", "home_first_floor", "complex_house"]


@pytest.mark.parametrize("path", fc.MAPS, ids=lambda p: p.name.split(".")[0])
def test_live_slam_maps_give_the_old_distances(path):
    assert wc.map_differences(random.Random(path.name), fc.load(path)) == []


@pytest.mark.parametrize("house", HOUSES)
def test_the_sim_houses_at_the_cars_cells_give_the_old_distances(house):
    m = rc.house_map(house, k=6, pad=20)
    assert wc.map_differences(random.Random(house), m) == []


def test_procthor_houses_give_the_old_distances():
    rng = random.Random(31200)
    bad = []
    for i in range(100):
        name = f"procthor:val:{i}"
        bad += [(name, 1, *b) for b in wc.map_differences(rng, rc.house_map(name))]
        if i < 20:
            m = rc.house_map(name, k=6, pad=20)
            bad += [(name, 6, *b) for b in wc.map_differences(rng, m, doors=(rl.DOOR_M,))]
    assert bad == []


def test_500_random_grids_give_the_old_distances():
    rng = random.Random(31201)
    bad = []
    for n in range(500):
        m = rc.random_map(rng, 200)
        bad += [(n, m["width"], m["height"], *b)
                for b in wc.map_differences(rng, m, counts=(0, 1, 5, 20))]
    assert bad == []


def test_every_walk_of_the_sim_surveys_gives_the_old_distances(monkeypatch):
    seen, bad = [], []
    fast = rl._walk_dist

    def both(seg, sources, res, door_m):
        d1 = fast(seg, sources, res, door_m)
        d0, _ = ref._walk(seg, sources, res, door_m)
        seen.append(len(sources))
        if not (d0.dtype == d1.dtype and np.array_equal(d0, d1)):
            bad.append((seg.shape, len(sources)))
        return d1

    monkeypatch.setattr(rl, "_walk_dist", both)
    rows = [sw.run(h, st, mode="spacing") for h in sw.HOUSES for st in sw.starts(h)]
    assert [r["outcome"] for r in rows] == ["exhausted"] * 9, rows
    # Each survey's label calls as on dev before 3.120 (607bcbc).
    assert [r["label_calls"] for r in rows] == [6, 6, 6, 33, 29, 28, 42, 44, 37]
    assert len(seen) > 50
    assert bad == []
