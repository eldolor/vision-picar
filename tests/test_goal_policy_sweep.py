"""
tests/test_goal_policy_sweep.py -- 3.65 phase 2 criterion 6: the network
goal choice (`brain/explore_goal_net.json`, amendment 2) on the 200 test
houses (usable 0-99 and 200-299), pinned at what the one test run measured
(2026-10-12): 196 found (bar >= 196), median 8 goals per found mission
(the rule: 11; bar <= 8.8), every mission clean. On SLOW_FILES: 200 missions.
"""
import importlib.util
import os
import statistics

from brain.goal_policy import NetGoalPolicy
from sim.maps.procthor import usable

HERE = os.path.dirname(__file__)


def _harness():
    path = os.path.join(HERE, "..", "evaluations", "explore-365", "harness.py")
    spec = importlib.util.spec_from_file_location("explore365_harness", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_network_holds_its_test_set_numbers():
    run_house, net, u = _harness().run_house, NetGoalPolicy.load(), usable()
    rows = [run_house(h, policy=net) for h in u[0:100] + u[200:300]]
    bad = [r for r in rows if r["outcome"] in ("exception", "running") or r.get("capped")]
    assert not bad, bad
    found = [r for r in rows if r["outcome"] == "found"]
    assert len(found) >= 196
    assert statistics.median(r["goals"] for r in found) <= 8


def test_rooms_as_wired_holds_its_selection_numbers():
    """3.96.C: `explore_goal_policy: rooms` through MissionRunner, with no room
    labels (a find mission's map has none), on the 100 selection houses in
    both target modes, pinned at what it measured on 2026-10-24
    (`evaluations/goal-learn-396/wired/select.json`): R 100 found, median 7
    goals (the rule 100, 7); U 100, 8 (the rule 99, 10). 200 missions."""
    import sys
    root = os.path.join(HERE, "..", "evaluations", "goal-learn-396")
    if root not in sys.path:
        sys.path.insert(0, root)
    import train as T
    for mode, goals in (("R", 7), ("U", 8)):
        rows = [T.mission(s, h, mode, "wired") for s, h in T.select_houses()]
        bad = [r for r in rows if r["outcome"] in ("exception", "running", None) or r.get("capped")]
        assert not bad, bad
        found = [r for r in rows if r["outcome"] == "found"]
        assert len(found) >= 100, mode
        assert statistics.median(r["goals"] for r in found) <= goals, mode
