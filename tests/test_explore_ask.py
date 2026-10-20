"""
tests/test_explore_ask.py

`PLAN-ros-alignment.md` 3.88 A: explore's ask mode. Each goal choice pauses
and publishes the candidates the rules allow; a person's pick, by id, is
sent; a stale or unknown id is refused and sends nothing; no pick in 60 s
takes the rule's pick (`rule_timeout`); the pause is outside the time limit;
the robot is stopped throughout, and a person's Stop or D-pad during it ends
the mission as on `dev` (3.84). Every choice is one durable JSON line.

In process over `tests/test_explore.py`'s FakeNav (the policy's decisions,
not nav2's driving), and over the robot server in process where the latch is
the subject.
"""

import json
import math

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from brain.choice_log import (ASK_TIMEOUT_S, BY_PERSON, BY_RULE_TIMEOUT, ChoiceLog, GoalAsk,
                              read_choices)
from brain.explore import ASK_PROBE_S
from brain.goal_policy import FEATURES
from control import brain_server
from control.brain_config import load_brain_config
from control.mission_runner import CANCELLED, MissionRunner
from control.remote_robot import RemoteRobot
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.conftest import ASGI_BASE_URL, asgi_client
from tests.test_explore import FakeNav


def _runner(house="scaled_house", ask=True, limit=None, max_steps=80, robot_hook=None):
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    if robot_hook is not None:
        robot_hook(robot)
    nav = FakeNav(robot)
    runner = MissionRunner(robot, target_object="red backpack", policy="explore",
                           navigator=nav, world=MockWorld(grid), max_steps=max_steps,
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           mission_time_limit_s=limit,
                           explore_ask_timeout_s=ASK_TIMEOUT_S if ask else None,
                           house=house)
    return runner, nav, grid


def _truth(grid):
    return (grid.x, grid.y, grid.theta)


def _sets(nav):
    return [c for c in nav.calls if c[0] == "set"]


def _until_asked(runner, limit=400):
    for _ in range(limit):
        ask = (runner.status()["explore"] or {}).get("ask")
        if ask is not None:
            return ask
        assert runner.tick(), runner.status()
    raise AssertionError("explore never asked")


def _run(runner, picker=None, limit=5000):
    """Tick to the end; `picker(ask)` answers each new choice (None: never)."""
    answered = set()
    for _ in range(limit):
        ask = runner.status()["explore"]["ask"]
        if picker is not None and ask is not None and ask["choice"] not in answered:
            answered.add(ask["choice"])
            ok, why = runner.pick_goal(ask["choice"], picker(ask))
            assert ok, why
        if not runner.tick():
            return
    raise AssertionError("the mission never ended")


# ---------- criterion 1: ask mode ----------


def test_ask_mode_is_off_by_default_and_a_mission_without_it_asks_nothing(monkeypatch):
    monkeypatch.delenv("EXPLORE_ASK", raising=False)
    assert load_brain_config()["explore_ask"] is False
    runner, nav, _g = _runner(ask=False)
    runner.start()
    _run(runner)
    st = runner.status()
    assert st["outcome"] == "found", st
    assert runner.agent.ask is None
    assert st["explore"]["asks"] is False and st["explore"]["ask"] is None
    assert runner.choices == [] and st["budget"]["paused_s"] == 0.0


def test_the_environment_turns_it_on_and_is_validated(monkeypatch):
    monkeypatch.setenv("EXPLORE_ASK", "1")
    assert load_brain_config()["explore_ask"] is True
    monkeypatch.setenv("EXPLORE_ASK", "maybe")
    with pytest.raises(ValueError, match="EXPLORE_ASK"):
        load_brain_config()


def test_a_bad_timeout_is_refused(tmp_path):
    p = tmp_path / "robot.yaml"
    p.write_text("brain:\n  explore_ask_timeout_s: 0\n")
    with pytest.raises(ValueError, match="explore_ask_timeout_s"):
        load_brain_config(p)


def test_a_choice_pauses_and_publishes_the_candidates_with_their_features():
    runner, nav, grid = _runner()
    runner.start()
    ask = _until_asked(runner)
    assert ask["choice"] == 1 and ask["candidates"], ask
    for c in ask["candidates"]:
        assert set(c) >= {"id", "kind", "x_m", "y_m", "features"}, c
        assert c["kind"] in ("frontier", "view")
        assert list(c["features"]) == list(FEATURES)
        assert c["id"].startswith("1.")
    assert ask["default"] == ask["candidates"][0]["id"]
    assert ask["remaining_s"] <= ASK_TIMEOUT_S


def test_the_candidates_are_the_ones_the_rule_would_choose_among():
    """The offered pool is 3.65's pool: the rule's own first goal is the
    default, and every frontier offered is one the rule would send."""
    rule, rule_nav, _g = _runner(ask=False)
    rule.start()
    for _ in range(400):
        rule.tick()
        if _sets(rule_nav):
            break
    runner, nav, _g = _runner()
    runner.start()
    ask = _until_asked(runner)
    first = next(c for c in ask["candidates"] if c["id"] == ask["default"])
    assert ("set", round(first["x_m"], 2), round(first["y_m"], 2)) == _sets(rule_nav)[0]


def test_a_pick_by_id_is_sent_and_recorded_as_the_persons():
    runner, nav, grid = _runner()
    runner.start()
    ask = _until_asked(runner)
    pick = ask["candidates"][-1]
    assert pick["id"] != ask["default"] or len(ask["candidates"]) == 1
    ok, why = runner.pick_goal(ask["choice"], pick["id"])
    assert ok, why
    before = len(_sets(nav))
    runner.tick()
    sent = _sets(nav)[before:]
    assert sent == [("set", round(pick["x_m"], 2), round(pick["y_m"], 2))], nav.calls[-5:]
    rec = runner.choices[-1]
    assert rec["by"] == BY_PERSON and rec["pick"] == pick["id"]
    assert rec["default"] == ask["default"] and rec["house"] == "scaled_house"
    assert len(rec["candidates"]) == len(ask["candidates"])
    assert runner.status()["explore"]["ask"] is None


@pytest.mark.parametrize("bad", ["stale", "unknown", "second"])
def test_a_stale_or_unknown_id_is_refused_and_sends_nothing(bad):
    runner, nav, grid = _runner()
    runner.start()
    ask = _until_asked(runner)
    if bad == "stale":
        choice, cid = ask["choice"] - 1 or 99, ask["candidates"][0]["id"]
    elif bad == "unknown":
        choice, cid = ask["choice"], "1.999"
    else:
        assert runner.pick_goal(ask["choice"], ask["candidates"][0]["id"])[0]
        choice, cid = ask["choice"], ask["candidates"][-1]["id"]
    sets = len(_sets(nav))
    ok, why = runner.pick_goal(choice, cid)
    assert not ok and why, why
    if bad != "second":
        for _ in range(8):
            runner.tick()
        assert len(_sets(nav)) == sets, "a refused pick sent a goal"
        assert runner.status()["explore"]["ask"]["choice"] == ask["choice"]


def test_a_pick_from_an_earlier_choice_is_refused_once_a_new_one_is_open():
    runner, nav, grid = _runner()
    runner.start()
    first = _until_asked(runner)
    assert runner.pick_goal(first["choice"], first["default"])[0]
    for _ in range(400):
        runner.tick()
        ask = runner.status()["explore"]["ask"]
        if ask is not None and ask["choice"] > first["choice"]:
            break
    else:
        pytest.skip("found before a second choice")
    ok, why = runner.pick_goal(first["choice"], first["candidates"][0]["id"])
    assert not ok and "not the open one" in why


def test_no_pick_in_60_s_takes_the_rules_pick_recorded_as_rule_timeout():
    runner, nav, grid = _runner()
    runner.start()
    ask = _until_asked(runner)
    t0 = grid.sim_time
    while not runner.choices:
        assert runner.tick()
    assert grid.sim_time - t0 >= ASK_TIMEOUT_S - 0.5
    rec = runner.choices[-1]
    assert rec["by"] == BY_RULE_TIMEOUT and rec["pick"] == ask["default"], rec
    d = next(c for c in ask["candidates"] if c["id"] == ask["default"])
    assert _sets(nav)[-1] == ("set", round(d["x_m"], 2), round(d["y_m"], 2))
    # A pick after the timeout is refused: the rule's stands.
    assert not runner.pick_goal(ask["choice"], ask["candidates"][-1]["id"])[0]


def test_a_mission_answered_only_by_timeouts_sends_the_rules_goals():
    """The rule's own pick, every choice: the same first goals as the rule."""
    rule, rule_nav, _g = _runner(ask=False)
    rule.start()
    _run(rule)
    runner, nav, _g = _runner()
    runner.start()
    _run(runner)
    assert runner.status()["outcome"] == rule.status()["outcome"] == "found"
    assert all(r["by"] == BY_RULE_TIMEOUT for r in runner.choices)
    n = min(3, len(_sets(rule_nav)))
    assert _sets(nav)[:n] == _sets(rule_nav)[:n]


def test_time_paused_does_not_count_toward_the_time_limit():
    runner, nav, grid = _runner(limit=20.0)
    runner.start()
    _until_asked(runner)
    t0 = grid.sim_time
    while grid.sim_time - t0 < 40.0:       # twice the limit, paused
        assert runner.tick(), runner.status()
    st = runner.status()
    assert st["running"] and st["budget"]["paused_s"] >= 39.0, st["budget"]
    # ...and the limit still ends a search that is not paused.
    rule, _n, _g = _runner(ask=False, limit=5.0)
    rule.start()
    _run(rule)
    assert rule.status()["outcome"] == "out_of_budget"


# ---------- criterion 2: safety unchanged ----------


def test_the_robot_does_not_move_during_a_pause():
    runner, nav, grid = _runner()
    runner.start()
    _until_asked(runner)
    at = _truth(grid)
    sets = len(_sets(nav))
    t0 = grid.sim_time
    while grid.sim_time - t0 < ASK_TIMEOUT_S - 2:
        assert runner.tick()
    assert _truth(grid) == at, "the robot moved during a pause"
    moved = math.hypot(grid.x - at[0], grid.y - at[1]) * 0.30 * 100
    assert moved == 0.0
    assert len(_sets(nav)) == sets, "a goal was sent during a pause"


def test_a_brain_stop_during_a_pause_ends_it_and_a_later_pick_is_refused():
    runner, nav, grid = _runner()
    runner.start()
    ask = _until_asked(runner)
    sets = len(_sets(nav))
    runner.stop()
    ok, why = runner.pick_goal(ask["choice"], ask["candidates"][0]["id"])
    assert not ok and "no mission" in why
    assert not runner.tick()
    assert len(_sets(nav)) == sets
    assert runner.status()["explore"]["ask"] is None


def _latched_robot_hook(client):
    """The sim robot in process, whose camera centring goes through the robot
    server -- where a person's Stop latches the brain out (3.84)."""
    remote = RemoteRobot(ASGI_BASE_URL, client=client)

    def hook(robot):
        robot.look_center = remote.look_center
    return hook


@pytest.fixture
def plain(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = asgi_client(server.create_app())
    yield client
    client.close()


@pytest.mark.parametrize("who,how,by", [
    ("twin-dpad", "stop", "stop"),
    ("unnamed", "stop", "stop"),
    ("twin-dpad", "dpad", "dpad"),
])
def test_a_persons_stop_or_dpad_during_a_pause_ends_the_mission_as_on_dev(plain, who, how, by):
    """Red-first: a pause sends nothing, so the latch was heard only when
    the pause ended -- up to 60 s of a mission reading `running` after a
    person's Stop. The pause centres the camera every ASK_PROBE_S."""
    runner, nav, grid = _runner(robot_hook=_latched_robot_hook(plain))
    runner.start()
    ask = _until_asked(runner)
    sets, at = len(_sets(nav)), _truth(grid)
    headers = {} if who == "unnamed" else {"x-driver": who}
    if how == "stop":
        assert plain.post("/stop", headers=headers).json()["executed"]
    else:
        plain.post("/action", json={"action": "LOOK_LEFT"}, headers=headers)
    t0 = grid.sim_time
    while runner.tick():
        assert grid.sim_time - t0 < 2 * ASK_PROBE_S + 1, "the pause did not hear the person"
    st = runner.status()
    assert st["outcome"] == CANCELLED and st["cancelled_by"] == by, st
    assert len(_sets(nav)) == sets and _truth(grid) == at
    assert not runner.pick_goal(ask["choice"], ask["candidates"][0]["id"])[0]


def test_a_policy_or_a_person_can_only_pick_what_the_rules_allow():
    """Every pick, whatever it is, is among the rule's pool: frontiers not
    reached twice and available in the retry book."""
    runner, nav, grid = _runner()
    agent = runner.agent
    checked = []
    orig = agent._send_pooled

    def spy(kind, c, rank, now, n, skipped, **info):
        if "picked_by" in info:
            r = agent.retry.radius_m
            reached = sum(math.hypot(c.goal[0] - x, c.goal[1] - y) < r for x, y in agent._reached)
            ok = agent.retry.available(c.goal[0], c.goal[1], now)
            checked.append(ok and (reached < 2 if kind == "frontier" else agent._view_allowed(c)))
        return orig(kind, c, rank, now, n, skipped, **info)
    agent._send_pooled = spy
    runner.start()
    _run(runner, picker=lambda ask: ask["candidates"][-1]["id"])
    assert checked and all(checked), checked
    assert runner.status()["outcome"] in ("found", "exhausted", "out_of_budget")


def test_a_sighting_during_a_pause_withdraws_it_unrecorded():
    runner, nav, grid = _runner()
    runner.start()
    _until_asked(runner)
    agent = runner.agent
    orig = agent._sighting
    agent._sighting = lambda frame: (grid.x * 0.3 + 1.0, grid.y * 0.3)
    runner.tick()
    agent._sighting = orig
    assert runner.status()["explore"]["ask"] is None
    assert runner.choices == []
    assert any(e["event"] == "ask_withdrawn" for e in agent.events)


# ---------- criterion 3: the log ----------


def test_every_choice_is_one_durable_line_in_one_file_a_mission(tmp_path, monkeypatch):
    import brain.choice_log as cl
    synced = []
    real = cl._fsync
    monkeypatch.setattr(cl, "_fsync", lambda fd: (synced.append(fd), real(fd)))
    runner, nav, grid = _runner()
    runner.mission_id = "m1"
    runner.choice_log = ChoiceLog(tmp_path / "m1.jsonl")
    runner.start()
    _run(runner, picker=lambda ask: ask["default"])
    recs = read_choices(tmp_path / "m1.jsonl")
    assert recs and len(recs) == len(runner.choices)
    assert len(synced) >= len(recs)
    assert [r["choice"] for r in recs] == sorted({r["choice"] for r in recs})
    for r in recs:
        assert r["mission_id"] == "m1" and r["house"] == "scaled_house"
        assert r["by"] == BY_PERSON and r["pick"] in {c["id"] for c in r["candidates"]}
        assert {"step", "pose", "candidates", "default", "waited_s"} <= set(r)
        text = json.dumps(r)
        assert "image" not in text and "base64" not in text
    assert list(tmp_path.iterdir()) == [tmp_path / "m1.jsonl"]


def test_a_failing_write_never_fails_the_mission(tmp_path, monkeypatch):
    import brain.choice_log as cl

    def broken(fd):
        raise OSError("disk gone")
    monkeypatch.setattr(cl, "_fsync", broken)
    runner, nav, grid = _runner()
    runner.choice_log = ChoiceLog(tmp_path / "m.jsonl")
    runner.start()
    _run(runner, picker=lambda ask: ask["default"])
    assert runner.status()["outcome"] == "found"
    assert runner.choices


def test_a_cut_off_last_line_is_skipped(tmp_path):
    p = tmp_path / "m.jsonl"
    p.write_text('{"choice": 1}\n{"choice": 2, "cand')
    assert read_choices(p) == [{"choice": 1}]


def test_the_ask_box_refuses_after_the_mission_ended():
    t = [0.0]
    ask = GoalAsk(clock=lambda: t[0], timeout_s=5)
    n, _ = ask.open([{"kind": "frontier"}], 0)
    t[0] = 2.0
    ask.end()
    assert ask.pick(n, "1.0") == (False, "no goal choice is open")
    assert ask.paused_s() == 2.0
    assert ask.open([{"kind": "frontier"}], 0) is None


# ---------- the route ----------


class _StubRunner:
    def __init__(self, answer):
        self.answer, self.calls = answer, []

    def pick_goal(self, choice, cid):
        self.calls.append((choice, cid))
        return self.answer

    def start(self):
        pass

    def tick(self):
        import time
        time.sleep(0.02)
        return True

    def stop(self, reason=""):
        pass

    def is_running(self):
        return True

    def status(self):
        return {"running": True}


def _brain(monkeypatch, runner):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    app = brain_server.create_app(robot_factory=lambda: object(),
                                  runner_factory=lambda robot, req: runner)
    return TestClient(app)


def test_the_pick_route_takes_a_pick_and_answers_409_for_a_stale_one(monkeypatch):
    runner = _StubRunner((True, ""))
    brain = _brain(monkeypatch, runner)
    assert brain.post("/mission/explore/pick", json={"choice": 1, "candidate": "1.0"}
                      ).status_code == 409             # no mission yet
    r = brain.post("/mission/start", json={"target_object": "x", "policy": "explore"})
    assert r.status_code == 200, r.text
    r = brain.post("/mission/explore/pick", json={"choice": 3, "candidate": "3.1"})
    assert r.status_code == 200 and r.json()["accepted"], r.text
    runner.answer = (False, "choice 2 is not the open one (choice 3)")
    r = brain.post("/mission/explore/pick", json={"choice": 2, "candidate": "2.0"})
    assert r.status_code == 409 and "not the open one" in r.json()["detail"]
    assert brain.post("/mission/explore/pick", json={"choice": 0, "candidate": "x"}
                      ).status_code == 422
    assert brain.post("/mission/explore/pick", json={"choice": 1, "candidate": "1.0",
                                                     "x_m": 1}).status_code == 422
    assert runner.calls == [(3, "3.1"), (2, "2.0")]


def test_explore_ask_on_a_policy_without_explore_is_refused(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    brain = TestClient(brain_server.create_app(robot_factory=lambda: object()))
    r = brain.post("/mission/start", json={"target_object": "x", "policy": "frontier",
                                           "explore_ask": True})
    assert r.status_code == 400 and "explore_ask" in r.json()["detail"], r.text


# ---------- review (2026-10-20) ----------


def test_a_tiered_search_does_not_ask(monkeypatch):
    """Review round 2: a pause's free scene in a tiered search marked the
    target found without the arrival check, and its real questions would run
    on through a pause. Ask mode is for the plain explore policy only."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    brain = TestClient(brain_server.create_app(robot_factory=lambda: object()))
    r = brain.post("/mission/start", json={"target_object": "x", "policy": "tiered",
                                           "explore_ask": True})
    assert r.status_code == 400 and "explore_ask" in r.json()["detail"], r.text


def test_a_runner_refuses_ask_mode_for_a_tiered_search():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    with pytest.raises(ValueError, match="explore"):
        MissionRunner(robot, target_object="red backpack", policy="tiered",
                      vision_fn=lambda f: {}, tier_search="explore", navigator=FakeNav(robot),
                      world=MockWorld(grid), explore_ask_timeout_s=60.0)


def test_a_wait_elsewhere_keeps_the_question_and_its_ids():
    """/code-review: a tick that WAITed before reaching the goal choice (the
    local tier's next answer) withdrew the choice; the next opened a new one
    and the person's tap on the old ids was a 409."""
    runner, nav, grid = _runner()
    runner.start()
    ask = _until_asked(runner)
    agent = runner.agent
    real = agent._next_frontier
    agent._next_frontier = lambda now: ("WAIT", True, "elsewhere")
    runner.tick()
    agent._next_frontier = real
    assert runner.status()["explore"]["ask"]["choice"] == ask["choice"]
    assert runner.pick_goal(ask["choice"], ask["candidates"][-1]["id"])[0]


def test_a_choice_withdrawn_after_its_pick_is_not_recorded():
    """Both reviewers: the record was written at the decision, before the
    send -- a pick then set aside stayed in the log as a label."""
    runner, nav, grid = _runner()
    runner.start()
    ask = _until_asked(runner)
    assert runner.pick_goal(ask["choice"], ask["default"])[0]
    agent = runner.agent
    real = agent.retry.available
    agent.retry.available = lambda *a, **k: False
    runner.tick()
    agent.retry.available = real
    assert runner.choices == [], runner.choices
    assert any(e["event"] == "ask_withdrawn" for e in agent.events)


def test_a_stop_while_choosing_opens_nothing_and_fails_nothing():
    """/code-review: `open()` after the mission's `end()` raised, logged as
    'mission step failed' for an ordinary Stop."""
    t = [0.0]
    ask = GoalAsk(clock=lambda: t[0])
    ask.end()
    assert ask.open([{"kind": "frontier"}], 0) is None


def test_the_ask_mints_its_ids_on_copies():
    ask = GoalAsk(clock=lambda: 0.0)
    given = [{"kind": "frontier"}, {"kind": "view"}]
    n, cands = ask.open(given, 1)
    assert [c["id"] for c in cands] == [f"{n}.0", f"{n}.1"] and "id" not in given[0]
    assert ask.snapshot()["default"] == f"{n}.1"


def test_a_refused_approach_after_a_sighting_still_withdraws_the_choice():
    """Thermos round 4: a sighting whose approach was refused returns WAIT,
    and the stale choice stayed open -- a tap or the timeout could then send
    a frontier away from a target in view."""
    runner, nav, grid = _runner()
    runner.start()
    _until_asked(runner)
    agent = runner.agent
    real_s, real_a = agent._sighting, agent._send_approach
    agent._sighting = lambda frame: (grid.x * 0.3 + 1.0, grid.y * 0.3)
    agent._send_approach = lambda *a, **k: ("WAIT", False, "approach refused")
    try:
        runner.tick()
    finally:
        agent._sighting, agent._send_approach = real_s, real_a
    assert runner.status()["explore"]["ask"] is None


def test_a_queued_verb_that_waits_still_withdraws_the_choice():
    runner, nav, grid = _runner()
    runner.start()
    _until_asked(runner)
    agent = runner.agent
    pose = agent._pose()
    # A real queued FACE at a point dead ahead: "already facing it", a WAIT.
    h = math.radians(pose["heading_deg"])
    agent._pending = [("FACE", (pose["x_m"] + math.sin(h), pose["y_m"] - math.cos(h)))]
    runner.tick()
    assert any(e["event"] == "ask_withdrawn" for e in agent.events)
    assert runner.status()["explore"]["ask"] is None or \
        runner.status()["explore"]["ask"]["choice"] > 1
