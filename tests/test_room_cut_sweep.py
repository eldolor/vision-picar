"""3.118 criterion 1 (slow tier, SLOW_FILES): the fast `segment()` gives
exactly the frozen copy's answers -- the same room ids, the clearance bit for
bit, at three neck ratios -- on the six live 0.05 m SLAM maps, the
simulator's houses at 0.30 m and at the car's 0.05 m (3.70's car-resolution
maps), ProcTHOR `val` 0-99 at 0.30 m and 0-19 at 0.05 m, 500 seeded random
grids up to 200 x 200, and every cut the labeller makes during the 9
`spacing` surveys of `tests/survey_sweep.py`."""
import random

import numpy as np
import pytest

from brain import room_labels as rl
from tests import frontier_compare as fc
from tests import room_cut_compare as rc
from tests import room_cut_reference as ref
from tests import survey_sweep as sw

HOUSES = ["starter_house", "scaled_house", "home_first_floor", "complex_house"]


@pytest.mark.parametrize("path", fc.MAPS, ids=lambda p: p.name.split(".")[0])
def test_live_slam_maps_give_the_old_answers(path):
    assert rc.differences(fc.load(path)) == []


@pytest.mark.parametrize("house", HOUSES)
def test_the_sim_houses_at_the_cars_cells_give_the_old_answers(house):
    assert rc.differences(rc.house_map(house, k=6, pad=20)) == []


def test_procthor_houses_give_the_old_answers():
    bad = []
    for i in range(100):
        name = f"procthor:val:{i}"
        bad += [(name, 1, *b) for b in rc.differences(rc.house_map(name))]
        if i < 20:
            bad += [(name, 6, *b) for b in rc.differences(rc.house_map(name, k=6, pad=20),
                                                          ratios=(rl.NECK_RATIO,))]
    assert bad == []


def test_500_random_grids_give_the_old_answers():
    rng = random.Random(31180)
    bad = []
    for n in range(500):
        m = rc.random_map(rng, 200)
        bad += [(n, m["width"], m["height"], *b) for b in rc.differences(m)]
    assert bad == []


def test_every_cut_of_the_sim_surveys_gives_the_old_answers(monkeypatch):
    seen, bad = [], []
    fast = rl.segment

    def both(m, neck_ratio=rl.NECK_RATIO, *a, **kw):
        s1, d1 = fast(m, neck_ratio, *a, **kw)
        s0, d0 = ref.segment(m, neck_ratio, *a, **kw)
        seen.append(1)
        if not (np.array_equal(s0, s1) and d0.dtype == d1.dtype and np.array_equal(d0, d1)):
            bad.append((m.get("map_version"), m["width"], m["height"]))
        return s1, d1

    monkeypatch.setattr(rl, "segment", both)
    rows = [sw.run(h, st, mode="spacing") for h in sw.HOUSES for st in sw.starts(h)]
    assert [r["outcome"] for r in rows] == ["exhausted"] * 9, rows
    assert len(seen) > 50
    assert bad == []
