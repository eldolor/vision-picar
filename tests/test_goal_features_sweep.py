"""3.109 criterion 1 (slow tier, SLOW_FILES): the numpy `features()` gives
exactly the loop's answers on 3.96's benchmark -- the six live SLAM maps x
20 poses x two camera-seen sets, every candidate -- and on every call
explore itself makes (its shared context) during real in-process missions
in the simulator's four houses and 20 ProcTHOR houses."""
import random

import pytest

import brain.explore as E
from sim.maps import procthor
from tests import frontier_compare as fc
from tests import goal_features_compare as gc
from tests import goal_features_reference as ref
from tests.test_explore import _mission


@pytest.mark.parametrize("path", fc.MAPS, ids=lambda p: p.name.split(".")[0])
def test_live_slam_maps_give_the_old_answers(path):
    rng = random.Random(f"3109:{path.name}")
    m = fc.load(path)
    seens = (set(), fc.seen_set(m, 8, rng))
    bad, n = [], 0
    for x, y in fc.poses(m, 20, rng):
        pose = {"x_m": x, "y_m": y, "heading_deg": rng.uniform(0, 360)}
        for seen in seens:
            goals = gc.candidates(m, x, y, seen)
            n += len(goals)
            bad += gc.differences(m, pose, goals, seen)
    assert n > 500
    assert bad == []


HOUSES = ["starter_house", "scaled_house", "home_first_floor", "complex_house"] + [
    f"procthor:val:{i}" for i in procthor.usable()[:20]]


@pytest.mark.parametrize("house", HOUSES)
def test_every_call_a_mission_makes_gives_the_old_answers(house, monkeypatch):
    """Explore under the network goal policy (which scores every candidate
    with these features), each call checked against the reference as it is
    made, with the context explore built."""
    calls, bad = [0], []
    real = E.goal_features

    def check(m, pose, goal, **kw):
        got = real(m, pose, goal, **kw)
        want = ref.features(m, pose, goal, **{k: v for k, v in kw.items() if k != "ctx"})
        calls[0] += 1
        if got != want or [type(v) for v in got] != [type(v) for v in want]:
            bad.append((calls[0], goal, want, got))
        return got
    monkeypatch.setattr(E, "goal_features", check)
    _mission(house=house, max_steps=60, explore_goal_policy="network")
    assert calls[0] > 0, "explore never scored a candidate"
    assert bad == []
