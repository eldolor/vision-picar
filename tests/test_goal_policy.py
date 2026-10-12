"""3.65 phase 2: explore's learned goal choice, behind
`explore_goal_policy` (default "rule")."""
import json

import pytest

from brain.goal_policy import FEATURES, WEIGHTS_PATH, LinearGoalPolicy, goal_policy_for
from control.brain_config import load_brain_config
from tests.test_explore import _mission

# The rule's own order as a linear score: frontiers by path - size, views by
# path - area, every view after every frontier.
RULE_AS_WEIGHTS = [-10.0, 0.0, -100.0, 3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def _sets(nav):
    return [c for c in nav.calls if c[0] == "set"]


def test_the_default_is_the_rule_and_changes_nothing(monkeypatch):
    monkeypatch.delenv("EXPLORE_GOAL_POLICY", raising=False)
    assert load_brain_config()["explore_goal_policy"] == "rule"
    runner, _nav, _grid = _mission()
    assert runner.agent.goal_policy is None


def test_an_unknown_policy_name_is_refused(tmp_path, monkeypatch):
    monkeypatch.delenv("EXPLORE_GOAL_POLICY", raising=False)
    p = tmp_path / "robot.yaml"
    p.write_text("brain:\n  explore_goal_policy: clever\n")
    with pytest.raises(ValueError, match="explore_goal_policy"):
        load_brain_config(p)
    with pytest.raises(ValueError):
        goal_policy_for("clever")


def test_the_environment_can_switch_it_and_is_validated(monkeypatch):
    monkeypatch.setenv("EXPLORE_GOAL_POLICY", "network")
    assert load_brain_config()["explore_goal_policy"] == "network"
    monkeypatch.setenv("EXPLORE_GOAL_POLICY", "clever")
    with pytest.raises(ValueError, match="explore_goal_policy"):
        load_brain_config()


def test_the_committed_weights_match_this_codes_features():
    data = json.loads(WEIGHTS_PATH.read_text())
    assert list(data["features"]) == list(FEATURES)
    assert len(LinearGoalPolicy.load().weights) == len(FEATURES)
    with pytest.raises(ValueError):
        LinearGoalPolicy([1.0])


def test_the_rule_written_as_weights_sends_the_rules_goals():
    """The learned path pools the same candidates the rule would send: with
    the rule's order as weights, every goal is the rule's goal."""
    _r, rule_nav, _g = _mission()
    runner, nav, _g = _mission(explore_goal_policy="rule")
    runner_l, nav_l, _g = _mission_with(RULE_AS_WEIGHTS)
    assert _sets(nav) == _sets(rule_nav)
    assert _sets(nav_l) == _sets(rule_nav)
    assert runner_l.status()["outcome"] == "found"


def _mission_with(weights, house="scaled_house"):
    """`_mission`, with the agent's goal choice replaced before the first tick."""
    import tests.test_explore as te
    orig = te.ExploreAgent.__init__

    def init(self, *a, **kw):
        orig(self, *a, **kw)
        self.goal_policy = LinearGoalPolicy(weights)
    te.ExploreAgent.__init__ = init
    try:
        return _mission(house=house)
    finally:
        te.ExploreAgent.__init__ = orig


def test_a_policy_never_sends_a_goal_the_rules_would_not():
    """Weights that love far, failed, already-reached places still pick only
    among what the rules allow: every frontier goal it sends was reached
    fewer than twice before and was available in the retry book, and the
    mission still ends."""
    import brain.explore as E
    offered = []
    orig = E.ExploreAgent._send

    def spy(self, x, y, kind, now, **info):
        if kind == "frontier" and "learned" in info:
            r = self.retry.radius_m
            offered.append((sum((x - rx) ** 2 + (y - ry) ** 2 < r * r for rx, ry in self._reached),
                            self.retry.available(x, y, now)))
        return orig(self, x, y, kind, now, **info)
    E.ExploreAgent._send = spy
    try:
        runner, _nav, _g = _mission_with([10.0, 0, 0, 0, 0, 0, 0, 10.0, 10.0, 0])
    finally:
        E.ExploreAgent._send = orig
    assert runner.status()["outcome"] in ("found", "searched", "max_steps")
    assert offered, "the learned path never ran"
    assert all(reached < 2 and ok for reached, ok in offered), offered


def test_turn_is_measured_as_explores_face_turn_measures_it():
    """Heading 0 points to -y_m in the simulator (explore's FACE turn uses
    atan2(dx, -dy)): a goal straight ahead needs no turn, one behind 180."""
    from brain.goal_policy import features
    m = {"resolution_m": 0.3, "width": 4, "height": 4, "origin_x_m": 0.0,
         "origin_y_m": 0.0, "cells": [0] * 16}
    pose = {"x_m": 0.5, "y_m": 0.8, "heading_deg": 0.0}
    turn = FEATURES.index("turn")
    assert features(m, pose, (0.5, 0.2), kind="frontier", path_m=0.6)[turn] == 0.0
    assert features(m, pose, (0.5, 1.1), kind="frontier", path_m=0.3)[turn] == 1.0
    assert features(m, {**pose, "heading_deg": 90.0}, (1.1, 0.8), kind="frontier",
                    path_m=0.6)[turn] == 0.0


def test_a_crashed_mission_never_scores_above_a_miss():
    from tests.test_goal_policy_sweep import _harness
    harness = _harness()
    worst_miss = -1 - harness.GOAL_COST * 200 - harness.METRE_COST * 2000
    assert -harness.CRASH_COST < worst_miss


def test_the_network_starts_as_the_linear_score():
    """Amendment 2: score = linear + MLP; with the MLP's last layer at zero
    (where training starts) it is the linear score exactly, and a network
    that does not take the ten features is refused."""
    import numpy as np
    from brain.goal_policy import NetGoalPolicy
    rng = np.random.default_rng(0)
    w = LinearGoalPolicy.load().weights
    layers = [(rng.normal(size=(32, 10)), rng.normal(size=32)),
              (rng.normal(size=(32, 32)), rng.normal(size=32)),
              (np.zeros((1, 32)), np.zeros(1))]
    net, lin = NetGoalPolicy(w, layers), LinearGoalPolicy(w)
    for _ in range(20):
        f = list(rng.uniform(0, 1, size=10))
        assert net.score(f) == pytest.approx(lin.score(f))
    layers[-1] = (rng.normal(size=(1, 32)), np.zeros(1))
    assert NetGoalPolicy(w, layers).score(f) != pytest.approx(lin.score(f))
    with pytest.raises(ValueError):
        NetGoalPolicy(w, [(np.zeros((1, 9)), np.zeros(1))])
    with pytest.raises(ValueError, match="layer 1"):   # 32 out, then a layer taking 16
        NetGoalPolicy(w, [layers[0], (np.zeros((1, 16)), np.zeros(1))])
    with pytest.raises(ValueError, match="layer 0"):   # bias the wrong length
        NetGoalPolicy(w, [(layers[0][0], np.zeros(31))] + layers[1:])


def test_the_runner_loads_each_committed_policy():
    """The wiring and the committed files, in the fast tier: MissionRunner
    builds explore with each flag value, and a non-finite score is ranked
    last rather than ending the choice."""
    from brain.goal_policy import NetGoalPolicy
    for name, kind in (("rule", type(None)), ("learned", LinearGoalPolicy),
                       ("network", NetGoalPolicy)):
        runner, _nav, _g = _mission(explore_goal_policy=name)
        assert type(runner.agent.goal_policy) is kind
    pol = LinearGoalPolicy([1.0] + [0.0] * 9)
    assert pol.best([[float("nan")] + [0.0] * 9, [0.5] + [0.0] * 9]) == 1
    assert pol.best([[float("nan")] + [0.0] * 9]) == 0
