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


def test_the_default_is_the_rule_and_changes_nothing():
    assert load_brain_config()["explore_goal_policy"] == "rule"
    runner, _nav, _grid = _mission()
    assert runner.agent.goal_policy is None


def test_an_unknown_policy_name_is_refused(tmp_path):
    p = tmp_path / "robot.yaml"
    p.write_text("brain:\n  explore_goal_policy: clever\n")
    with pytest.raises(ValueError, match="explore_goal_policy"):
        load_brain_config(p)
    with pytest.raises(ValueError):
        goal_policy_for("clever")


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
