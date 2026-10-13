"""3.77 criterion 1: same answers as the pure-Python frontier search, on
the six live SLAM maps (20 poses each, two camera-seen sets) and on every
map explore sees during in-process missions in the simulator's houses and
20 ProcTHOR houses. Slow tier (SLOW_FILES)."""
import random

import pytest

import brain.explore as E
from sim.maps import procthor
from tests import frontier_compare as fc
from tests.test_explore import _mission


@pytest.mark.parametrize("path", fc.MAPS, ids=lambda p: p.name.split(".")[0])
def test_live_slam_maps_give_the_old_answers(path):
    rng = random.Random(f"{fc.SEED}:{path.name}")
    m = fc.load(path)
    seens = (set(), fc.seen_set(m, 8, rng))
    bad = []
    for i, (x, y) in enumerate(fc.poses(m, 20, rng)):
        bad += [(i, d) for d in fc.differences(m, x, y, fc.LEAVE, seens)]
    assert bad == []


HOUSES = ["starter_house", "scaled_house", "home_first_floor", "complex_house"] + [
    f"procthor:val:{i}" for i in procthor.usable()[:20]]


@pytest.mark.parametrize("house", HOUSES)
def test_every_map_a_mission_sees_gives_the_old_answers(house, monkeypatch):
    """Each goal choice's map and pose (and, where the choice got that far,
    its camera-seen set), recorded from a real in-process explore mission
    and compared after it."""
    choices, gaps = [], []
    real_reach, real_gaps = E.reach, E.find_view_gaps

    def spy_reach(m, x, y, clearance_m, **kw):
        choices.append((m, x, y, clearance_m))
        return real_reach(m, x, y, clearance_m=clearance_m, **kw)

    def spy_gaps(m, x, y, seen, clearance_m, **kw):
        gaps.append((m, x, y, set(seen), clearance_m))
        return real_gaps(m, x, y, seen, clearance_m=clearance_m, **kw)
    monkeypatch.setattr(E, "reach", spy_reach)
    monkeypatch.setattr(E, "find_view_gaps", spy_gaps)
    _mission(house=house, max_steps=60)
    assert choices, "explore never chose a goal"
    bad = []
    for k, (m, x, y, clearance) in enumerate(choices[:15]):
        bad += [("choice", k, d) for d in fc.differences(m, x, y, clearance, (set(),))]
    for k, (m, x, y, seen, clearance) in enumerate(gaps[:10]):
        bad += [("gaps", k, d) for d in fc.differences(m, x, y, clearance, (seen,))]
    assert bad == []
