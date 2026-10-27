"""3.96.C: explore's goal choice with room features, behind
`explore_goal_policy: rooms` (default "rule")."""
import importlib.util
import json
import os

import numpy as np
import pytest

from brain.goal_policy import (FEATURES, ROOM_FEATURES, ROOMS_PATH, RoomGoalPolicy,
                               goal_policy_for)
from control.brain_config import load_brain_config
from tests.test_explore import _mission

HERE = os.path.dirname(__file__)
N = len(FEATURES) + len(ROOM_FEATURES)


def _model():
    """The test run's own numpy scorer (`evaluations/goal-learn-396/model.py`)."""
    path = os.path.join(HERE, "..", "evaluations", "goal-learn-396", "model.py")
    spec = importlib.util.spec_from_file_location("goal_learn_396_model", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_flag_selects_it_and_the_default_stays_the_rule(monkeypatch):
    monkeypatch.delenv("EXPLORE_GOAL_POLICY", raising=False)
    assert load_brain_config()["explore_goal_policy"] == "rule"
    monkeypatch.setenv("EXPLORE_GOAL_POLICY", "rooms")
    assert load_brain_config()["explore_goal_policy"] == "rooms"
    assert type(goal_policy_for("rooms")) is RoomGoalPolicy


def test_the_committed_file_is_the_tested_checkpoint():
    """The 17 features in this code's order, and the parameters and prior
    the test run used, value for value."""
    data = json.loads(ROOMS_PATH.read_text())
    assert data["features"] == list(FEATURES) + list(ROOM_FEATURES)
    root = os.path.join(HERE, "..", "evaluations", "goal-learn-396")
    best = json.load(open(os.path.join(root, "cloud", "ckpt", "F-large", "best.json")))
    assert data["params"] == best["params"]
    assert data["prior"] == json.load(open(os.path.join(root, "prior.json")))


def test_it_scores_as_the_test_run_scored():
    """Exactly the test run's float32 score, and its pick: argmax, the first
    of a tie, a non-finite score last."""
    model = _model()
    pol = RoomGoalPolicy.load()
    best = json.load(open(os.path.join(HERE, "..", "evaluations", "goal-learn-396",
                                       "cloud", "ckpt", "F-large", "best.json")))
    P = model.to_np(best["params"])
    X = np.random.default_rng(0).random((40, N)).astype(np.float32)
    assert np.array_equal(pol.scores(X), model.score_np(P, "F", X))
    assert pol.best(list(X)) == int(np.argmax(model.score_np(P, "F", X)))
    assert pol.best([]) is None
    nan = [float("nan")] * N
    assert pol.best([nan, list(X[0])]) == 1
    assert pol.best([list(X[0]), list(X[0])]) == 0


def test_a_bad_file_is_refused_at_load(tmp_path):
    data = json.loads(ROOMS_PATH.read_text())
    bad = dict(data, features=data["features"][:10])
    p = tmp_path / "rooms.json"
    p.write_text(json.dumps(bad))
    with pytest.raises(ValueError, match="features"):
        RoomGoalPolicy.load(p)
    params = dict(data["params"], W1=[row[:10] for row in data["params"]["W1"]])
    with pytest.raises(ValueError, match="W1"):
        RoomGoalPolicy(params, data["prior"])


def test_a_mission_under_it_finds_and_keeps_a_bounded_trail():
    """Through MissionRunner from the flag: every goal it chooses is the
    policy's (logged `learned`), the mission finds, and the trail holds one
    point per TRAIL_M square -- never a second for the same square."""
    from brain.explore import TRAIL_M
    runner, nav, _g = _mission(explore_goal_policy="rooms")
    agent = runner.agent
    assert type(agent.goal_policy) is RoomGoalPolicy
    assert runner.status()["outcome"] == "found"
    assert agent._trail
    keys = [(round(x / TRAIL_M), round(y / TRAIL_M)) for x, y in agent._trail]
    assert len(keys) == len(set(keys)) == len(agent._trail_keys)
    rule, _nav, _g = _mission()
    assert rule.agent._trail == []


def test_room_types_come_from_the_maps_labels():
    """A goal on a labelled cell, or next to one, takes that room's type;
    off every label, or on a map with none, it has none."""
    from brain.explore import ExploreAgent
    m = {"resolution_m": 1.0, "width": 4, "height": 1, "origin_x_m": 0.0,
         "origin_y_m": 0.0, "cells": [0] * 4,
         "room_labels": {"names": ["bedroom 2", "kitchen"], "cells": [0, -1, -1, 1]}}
    agent = ExploreAgent.__new__(ExploreAgent)
    goals = [(0.5, 0.5), (1.5, 0.5), (3.5, 0.5)]
    assert agent._goal_types(m, goals) == ["bedroom", "bedroom", "kitchen"]
    m["room_labels"]["cells"] = [-1, -1, -1, 1]
    assert agent._goal_types(m, [(0.5, 0.5)]) == [None]
    del m["room_labels"]
    assert agent._goal_types(m, goals) == [None, None, None]


def test_ask_mode_under_it_records_all_seventeen_features():
    """3.88 A's choice log keeps every feature the policy scored, in order
    (bug pass); the default is the policy's pick."""
    from tests.test_explore_ask import _runner, _until_asked
    runner, _nav, _grid = _runner()
    runner.agent.goal_policy = pol = RoomGoalPolicy.load()
    runner.start()
    ask = _until_asked(runner)
    for c in ask["candidates"]:
        assert list(c["features"]) == list(FEATURES) + list(ROOM_FEATURES)
    feats = [list(c["features"].values()) for c in ask["candidates"]]
    assert ask["default"] == ask["candidates"][pol.best(feats)]["id"]


def test_unknown_room_types_and_short_rows_are_handled():
    """A room type the target's prior does not name reads as no type (as in
    training); ten-feature rows are refused, never reshaped (review)."""
    pol = RoomGoalPolicy.load()
    m = {"resolution_m": 0.3, "width": 3, "height": 1, "origin_x_m": 0.0,
         "origin_y_m": 0.0, "cells": [0, 0, 0]}
    pose = {"x_m": 0.15, "y_m": 0.15, "heading_deg": 0.0}
    goals = [(0.45, 0.15), (0.75, 0.15)]
    f = [[0.0] * len(FEATURES)] * 2
    out = pol.extend(m, pose, goals, f, trail=[], types=["office", "bedroom"],
                     target="red backpack")
    tk = len(FEATURES) + ROOM_FEATURES.index("type_known")
    assert [row[tk] for row in out] == [0.0, 1.0]
    with pytest.raises(ValueError, match="17"):
        pol.best([[0.0] * len(FEATURES)] * 17)


def test_the_trail_begins_again_on_a_new_map():
    from brain.explore import ExploreAgent
    agent = ExploreAgent.__new__(ExploreAgent)
    agent.goal_policy, agent._last_map = RoomGoalPolicy.load(), None
    agent._trail, agent._trail_keys, agent._trail_map = [], set(), None
    agent._note_room({"x_m": 1.0, "y_m": 1.0, "map_id": "a"})
    agent._note_room({"x_m": 2.0, "y_m": 1.0, "map_id": "a"})
    agent._note_room({"x_m": 3.0, "y_m": 1.0, "map_id": "b"})
    assert agent._trail == [(3.0, 1.0)]


def test_labels_are_searched_in_metres_not_cells():
    """On SLAM's 0.05 m map a goal 0.2 m off a labelled cell still takes its
    type, as it would one training cell away (review)."""
    from brain.explore import ExploreAgent
    cells = [-1] * 10
    cells[0] = 0
    m = {"resolution_m": 0.05, "width": 10, "height": 1, "origin_x_m": 0.0,
         "origin_y_m": 0.0, "cells": [0] * 10,
         "room_labels": {"names": ["kitchen"], "cells": cells}}
    agent = ExploreAgent.__new__(ExploreAgent)
    assert agent._goal_types(m, [(0.225, 0.025), (0.475, 0.025)]) == ["kitchen", None]


def test_the_config_accepts_exactly_the_policies_there_are(monkeypatch):
    from brain.goal_policy import POLICIES
    for name in POLICIES:
        monkeypatch.setenv("EXPLORE_GOAL_POLICY", name)
        assert load_brain_config()["explore_goal_policy"] == name


def _train():
    import sys
    root = os.path.join(HERE, "..", "evaluations", "goal-learn-396")
    if root not in sys.path:
        sys.path.insert(0, root)
    import train
    return train


def test_the_test_runs_numbers_are_pinned():
    """Criterion 8: the C test run's committed rows still score as recorded
    (`test-c/score.json`) -- R 569 found, 4 goals (the rule 570, 7); U 583,
    5 (582, 9); labels half withheld R 572, 5."""
    T = _train()
    root = os.path.join(HERE, "..", "evaluations", "goal-learn-396", "test-c")
    score = json.load(open(os.path.join(root, "score.json")))["summary"]
    for name, want in score.items():
        assert T.summary(json.load(open(os.path.join(root, f"{name}.json")))) == want, name
    got = {k: (v["found"], v["median_goals_found"]) for k, v in score.items()}
    assert got["F-large-R"] == (569, 4.0) and got["rule-R"] == (570, 7.0)
    assert got["F-large-U"] == (583, 5.0) and got["rule-U"] == (582, 9.0)
    assert got["F-large-drop-R"] == (572, 5.0)



def test_a_pool_job_frees_its_house_before_the_next(monkeypatch):
    """3.113: a mission's runner, agent and house are reference cycles; a
    PPO worker that left them to the automatic collector grew ~0.5 MB a
    mission. `_job` collects (and freezes what survives) before returning."""
    import gc
    import weakref
    T = _train()
    best = json.load(open(os.path.join(HERE, "..", "evaluations", "goal-learn-396",
                                       "cloud", "ckpt", "F-large", "best.json")))
    built = []
    real = T.build_world

    def keep_a_weak_ref(name):
        world = real(name)
        built.append(weakref.ref(world))
        return world

    monkeypatch.setattr(T, "build_world", keep_a_weak_ref)
    gc.collect()
    gc.disable()
    try:
        row = T._job(("val", T.usable("val")[0], "R", ("greedy", best["kind"], best["params"])))
        assert row["reward"] is not None
        assert built and built[0]() is None
    finally:
        gc.unfreeze()
        gc.enable()
