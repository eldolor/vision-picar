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
