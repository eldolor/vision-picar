"""3.116.B: a find mission names rooms as a survey does, behind
`brain.find_room_labels` (default "off"), for explore_goal_policy "rooms"'s
room types only."""
import pytest
from fastapi.testclient import TestClient

from control.brain_config import load_brain_config
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_brain_server import _survey_app
from tests.test_explore import FakeNav, _mission


def test_the_default_is_off_with_a_cap_of_twenty(monkeypatch):
    monkeypatch.delenv("FIND_ROOM_LABELS", raising=False)
    c = load_brain_config()
    assert c["find_room_labels"] == "off"          # a string: bare `off` in YAML is False
    assert c["find_room_label_max_calls"] == 20


@pytest.mark.parametrize("extra, why", [
    ({"FIND_ROOM_LABELS": "always"}, "must be 'off', 'frame' or 'cloud'"),
    ({"FIND_ROOM_LABELS": "frame", "EXPLORE_GOAL_POLICY": "rule"}, "feeds explore_goal_policy"),
])
def test_the_config_refuses_what_could_not_work(monkeypatch, extra, why):
    for k, v in extra.items():
        monkeypatch.setenv(k, v)
    with pytest.raises(ValueError, match=why):
        load_brain_config()


@pytest.mark.parametrize("kw", [
    {"policy": "explore"},                                   # the rule chooses
    # tier_search counts under tiered only: frontier never runs explore (review).
    {"policy": "frontier", "tier_search": "explore", "explore_goal_policy": "rooms"},
])
def test_the_runner_refuses_labels_no_chooser_would_read(kw):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    with pytest.raises(ValueError, match="explore_goal_policy 'rooms' only"):
        MissionRunner(robot, target_object="red backpack", navigator=FakeNav(robot),
                      world=MockWorld(grid), find_room_labels=True, **kw)


@pytest.fixture
def backpack_informative(monkeypatch):
    """The houses here hide the red backpack, whose shipped prior is flat
    (3.116 amendment 7 asks nothing for it); tests of the asking itself
    give it a prior that singles out a room."""
    import control.mission_runner as mr
    monkeypatch.setattr(mr, "flat_prior_ok", lambda target: True)


def test_a_find_mission_asks_and_its_labels_reach_only_the_room_policy(backpack_informative):
    """It asks (free, from the sim's frames), reports its asks under
    `room_labels` (never `survey`), keeps them out of the map explore reads
    for its room goals, and gives the policy a layer with names."""
    runner, _nav, grid = _mission(explore_goal_policy="rooms", find_room_labels=True,
                                  room_label_max_calls=20)
    s = runner.status()
    assert s["outcome"] == "found", s["log_tail"]
    assert s["survey"] is None
    assert 1 <= s["room_labels"]["label_calls"] <= 20
    assert runner.room_labels() is None                # not a survey's labels
    m = runner.world.get_map()
    assert "room_labels" not in m
    cm, layer = runner.agent.goal_labels(m)
    assert cm["resolution_m"] >= m["resolution_m"] and layer["names"]
    assert len(layer["cells"]) == cm["width"] * cm["height"]
    assert set(layer["names"]) <= {n for n in grid.rooms}


def test_types_are_read_from_the_find_layer_when_the_map_has_none():
    from brain.explore import ExploreAgent
    m = {"resolution_m": 0.30, "width": 4, "height": 1, "origin_x_m": 0.0,
         "origin_y_m": 0.0, "cells": [0] * 4}
    agent = ExploreAgent.__new__(ExploreAgent)
    agent.goal_labels = lambda mm: (mm, {"names": ["kitchen"], "cells": [0, 0, -1, -1]})
    assert agent._goal_types(m, [(0.1, 0.1), (1.1, 0.1)]) == ["kitchen", None]
    # Looked up by metres on the map the layer was cut on, not this one.
    wide = {**m, "resolution_m": 0.60, "width": 2}
    agent.goal_labels = lambda mm: (wide, {"names": ["kitchen"], "cells": [-1, 0]})
    assert agent._goal_types(m, [(0.1, 0.1), (1.1, 0.1)]) == ["kitchen", "kitchen"]
    agent.goal_labels = lambda mm: None
    assert agent._goal_types(m, [(0.1, 0.1)]) == [None]


def test_the_cap_ends_asking_not_the_mission(monkeypatch, backpack_informative):
    """At the cap it stops asking -- and the labeller is handed no map to
    coarsen from then on (review) -- while the mission goes on."""
    from brain.room_labels import RoomLabeller
    maps_after_cap = []
    real = RoomLabeller.observe

    def observe(self, m, *a, **k):
        if self.capped:
            maps_after_cap.append(m)
        return real(self, m, *a, **k)

    monkeypatch.setattr(RoomLabeller, "observe", observe)
    runner, _nav, _g = _mission(explore_goal_policy="rooms", find_room_labels=True,
                                room_label_max_calls=1)
    s = runner.status()
    assert s["outcome"] == "found", s["log_tail"]
    assert s["room_labels"]["label_calls"] == 1 and s["room_labels"]["capped"]
    assert maps_after_cap and all(m is None for m in maps_after_cap)


def test_the_flat_prior_rule():
    """3.116 amendment 7: a rate that singles out a room (max >= 2 x min,
    the min floored at 0.01); on the shipped prior only the red backpack is
    flat, and a target with no prior is not informative either."""
    from brain.goal_policy import RoomGoalPolicy, prior_is_informative
    assert prior_is_informative({"kitchen": 1.0, "bedroom": 0.0})
    assert prior_is_informative({"kitchen": 0.02, "bedroom": 0.0})       # 0 reads as 0.01
    assert not prior_is_informative({"kitchen": 0.019, "bedroom": 0.01})
    assert not prior_is_informative({}) and not prior_is_informative(None)
    pol = RoomGoalPolicy.load()
    flat = sorted(t for t in pol.prior if not pol.informative_for(t))
    assert flat == ["red backpack"]
    assert pol.informative_for("Garbage Bag") and not pol.informative_for("a sock")


@pytest.mark.parametrize("source", ["frame", "cloud"])
def test_a_flat_prior_target_asks_nothing(source, monkeypatch):
    """The backpack (flat) builds no labeller -- no paid call -- reports why,
    and searches exactly as with the flag off."""
    kw = {} if source == "frame" else {"room_label_fn": lambda frame: pytest.fail("asked")}
    runner, _nav, _g = _mission(explore_goal_policy="rooms", find_room_labels=True, **kw)
    off, _nav2, _g2 = _mission(explore_goal_policy="rooms")
    s, o = runner.status(), off.status()
    assert runner.room_labeller is None and runner.agent.goal_labels is None
    assert s["room_labels"] == {"skipped": "flat_prior", "label_calls": 0}
    assert (s["outcome"], s["step"], s["explore"]["goals_sent"]) == \
        (o["outcome"], o["step"], o["explore"]["goals_sent"])


def test_off_asks_nothing():
    runner, _nav, _g = _mission(explore_goal_policy="rooms")
    assert runner.room_labeller is None and runner.status()["room_labels"] is None


# ---------- through the brain server ----------

def test_the_server_builds_the_cloud_source_for_a_find_with_its_own_cap(tmp_path, monkeypatch):
    import control.brain_server as bs
    seen = {}

    def fake_for(**k):
        seen.update(k)
        return lambda frame: "kitchen"

    monkeypatch.setattr(bs, "room_label_fn_for", fake_for)
    monkeypatch.delenv("EXPLORE_GOAL_POLICY", raising=False)
    monkeypatch.delenv("FIND_ROOM_LABELS", raising=False)
    app = _survey_app(tmp_path, monkeypatch, explore_goal_policy="rooms",
                      find_room_labels="cloud", find_room_label_max_calls=9,
                      vision_url="https://vision.example")
    with TestClient(app) as client:
        resp = client.post("/mission/start", json={"policy": "explore",
                                                   "target_object": "garbage bag"})
        runner = app.state.brain_state["runner"]
        client.post("/mission/stop")
    assert resp.status_code == 200, resp.text
    assert seen["vision_url"] == "https://vision.example"
    assert runner.find_room_labels and runner.room_labeller.max_calls == 9
    assert runner.metrics_config["find_room_labels"] == "cloud"
    assert runner.metrics_config["find_room_label_max_calls"] == 9


def test_the_server_asks_nothing_for_a_search_explore_does_not_choose(tmp_path, monkeypatch):
    import control.brain_server as bs
    monkeypatch.setattr(bs, "room_label_fn_for",
                        lambda **k: pytest.fail("frontier does not read room labels"))
    monkeypatch.delenv("EXPLORE_GOAL_POLICY", raising=False)
    monkeypatch.delenv("FIND_ROOM_LABELS", raising=False)
    app = _survey_app(tmp_path, monkeypatch, explore_goal_policy="rooms",
                      find_room_labels="cloud", vision_url="https://v")
    with TestClient(app) as client:
        resp = client.post("/mission/start", json={"policy": "frontier",
                                                   "target_object": "red backpack"})
        runner = app.state.brain_state["runner"]
        client.post("/mission/stop")
    assert resp.status_code == 200, resp.text
    assert runner.room_labeller is None
    assert runner.metrics_config["find_room_labels"] == "off"   # this mission asked nothing


def test_the_server_builds_nothing_for_a_flat_prior_find(tmp_path, monkeypatch):
    """No cloud source, no vision service needed, and the metrics row says
    this mission asked nothing (review)."""
    import control.brain_server as bs
    monkeypatch.setattr(bs, "room_label_fn_for",
                        lambda **k: pytest.fail("a flat-prior find builds no source"))
    monkeypatch.delenv("EXPLORE_GOAL_POLICY", raising=False)
    monkeypatch.delenv("FIND_ROOM_LABELS", raising=False)
    monkeypatch.delenv("VISION_URL", raising=False)
    app = _survey_app(tmp_path, monkeypatch, explore_goal_policy="rooms",
                      find_room_labels="cloud")
    with TestClient(app) as client:
        resp = client.post("/mission/start", json={"policy": "explore",
                                                   "target_object": "red backpack"})
        runner = app.state.brain_state["runner"]
        client.post("/mission/stop")
    assert resp.status_code == 200, resp.text
    assert runner.room_labeller is None
    assert runner.metrics_config["find_room_labels"] == "off"


def test_a_cloud_find_with_no_vision_service_is_refused_at_start(tmp_path, monkeypatch):
    monkeypatch.delenv("EXPLORE_GOAL_POLICY", raising=False)
    monkeypatch.delenv("FIND_ROOM_LABELS", raising=False)
    monkeypatch.delenv("VISION_URL", raising=False)
    app = _survey_app(tmp_path, monkeypatch, explore_goal_policy="rooms",
                      find_room_labels="cloud")
    with TestClient(app) as client:
        resp = client.post("/mission/start", json={"policy": "explore",
                                                   "target_object": "garbage bag"})
    assert resp.status_code == 400, resp.text
    assert "find_room_labels" in resp.text
