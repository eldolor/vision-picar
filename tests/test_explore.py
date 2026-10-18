"""
tests/test_explore.py

PLAN-ros-alignment.md 3.31 -- the `explore` policy's LOGIC, in process.

nav2 is replaced here by `FakeNav`, which moves the sim robot to a goal only
if the house's true floor connects the two (movers included) and says
`aborted` otherwise. That is a statement about the policy's decisions --
which frontier, when to approach, when to set a place aside, when to give
up -- not about driving, which is nav2's. The acceptance numbers for 3.31
come from real nav2 (`python -m tests.demo_explore`), never from this fake.
"""

import math
from collections import deque

import pytest

from brain.explore import ExploreAgent
from brain.frontier import RetryBook
from brain.memory import MissionMemory
from control.mission_runner import EXHAUSTED, FOUND, MissionRunner
from sim.grid_world import CELL_WALL
from sim.maps import build_movers, build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld

CELL = 0.30


class FakeNav:
    """nav2 in miniature: a goal succeeds on the next poll if the true
    floor connects the robot to it, and the robot is then there, facing the
    way it travelled; otherwise it aborts. Every call is recorded."""

    def __init__(self, robot, refuse_first=0):
        self.robot, self.grid = robot, robot.world
        self.goal = None
        self.calls = []
        self.refuse = refuse_first

    def set_goal(self, x_m, y_m):
        self.calls.append(("set", round(x_m, 2), round(y_m, 2)))
        if self.refuse:
            self.refuse -= 1
            return {"accepted": False, "reason": "preempted"}
        self.goal = {"state": "active", "x_m": x_m, "y_m": y_m}
        return {"accepted": True}

    def _connected(self, a, b):
        g = self.grid
        seen, q = {a}, deque([a])
        while q:
            c = q.popleft()
            if c == b:
                return True
            for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
                if (n not in seen and g._cell(*n) != CELL_WALL and n not in g.objects):
                    seen.add(n)
                    q.append(n)
        return False

    def get_goal(self):
        self.calls.append(("get",))
        if self.goal and self.goal["state"] == "active":
            g = self.grid
            to = (math.floor(self.goal["x_m"] / CELL), math.floor(self.goal["y_m"] / CELL))
            if to not in g.objects and self._connected((g.robot_x, g.robot_y), to):
                tx, ty = self.goal["x_m"] / CELL, self.goal["y_m"] / CELL
                g.theta = math.atan2(ty - g.y, tx - g.x) if (tx, ty) != (g.x, g.y) else g.theta
                g.x, g.y = tx, ty
                self.robot.pass_time(2.0)
                self.goal["state"] = "succeeded"
            else:
                self.robot.pass_time(2.0)
                self.goal["state"] = "aborted"
        return {"goal": self.goal, "plan": []}

    def cancel_goal(self):
        self.calls.append(("cancel",))
        if self.goal and self.goal["state"] == "active":
            self.goal["state"] = "canceled"
        return {"canceled": True}



class WalkingNav(FakeNav):
    """FakeNav that drives: each poll moves the robot one cell (0.3 m) along
    the shortest 4-connected floor path and passes 1 s, so the tier sees a
    frame per cell while a goal runs, as it does behind live nav2. FakeNav
    jumps to the goal in one poll (four frames a goal in all), which starves
    a blind search of questions (3.80.O amendment 1; `evaluations/tiered-380/O/sweep_inprocess.py`)."""

    def _path(self, a, b):
        g, prev, q = self.grid, {a: None}, deque([a])
        while q:
            c = q.popleft()
            if c == b:
                out = []
                while c is not None:
                    out.append(c)
                    c = prev[c]
                return out[::-1]
            for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
                if n not in prev and g._cell(*n) != CELL_WALL and n not in g.objects:
                    prev[n] = c
                    q.append(n)
        return None

    def get_goal(self):
        self.calls.append(("get",))
        if self.goal and self.goal["state"] == "active":
            g = self.grid
            to = (math.floor(self.goal["x_m"] / CELL), math.floor(self.goal["y_m"] / CELL))
            path = None if to in g.objects else self._path((g.robot_x, g.robot_y), to)
            self.robot.pass_time(1.0)
            if path is None:
                self.goal["state"] = "aborted"
            elif len(path) <= 2:
                tx, ty = self.goal["x_m"] / CELL, self.goal["y_m"] / CELL
                if (tx, ty) != (g.x, g.y):
                    g.theta = math.atan2(ty - g.y, tx - g.x)
                g.x, g.y = tx, ty
                self.goal["state"] = "succeeded"
            else:
                nx, ny = path[1][0] + 0.5, path[1][1] + 0.5
                g.theta = math.atan2(ny - g.y, nx - g.x)
                g.x, g.y = nx, ny
        return {"goal": self.goal, "plan": []}


def _mission(target="red backpack", house="scaled_house", movers=None, max_steps=80,
             refuse_first=0, **kw):
    grid = build_world(house)
    for m in movers or ():
        grid.add_mover(m)
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot, refuse_first=refuse_first)
    runner = MissionRunner(robot, target_object=target, policy="explore", navigator=nav,
                           world=MockWorld(grid), max_steps=max_steps,
                           clock=lambda: grid.sim_time, idle=robot.pass_time, **kw)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 5000:
        ticks += 1
    return runner, nav, grid


def test_it_finds_the_backpack_in_a_house_it_has_not_mapped():
    runner, nav, grid = _mission()
    s = runner.status()
    assert s["outcome"] == FOUND, s["log_tail"]
    assert s["explore"]["goals_sent"] >= 2


def test_a_target_in_sight_is_approached_along_its_bearing_at_the_lidar_range():
    """Standing in the kitchen door facing the backpack ~2 m away, the first
    decision is a goal just short of it -- on the line the camera saw it
    along, at the range the LIDAR reads there."""
    grid = build_world("scaled_house")
    grid.x, grid.y, grid.theta = 16.5, 4.5, math.radians(5)   # facing east
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="red backpack"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid))
    result = agent.step()
    assert result.action == "GOAL" and agent._goal["kind"] == "approach"
    gx, gy = agent._goal["x_m"], agent._goal["y_m"]
    tx, ty = agent._goal["target"]
    # the backpack is the cell (23, 5): its near face is x = 23 * 0.3
    assert 6.6 <= tx <= 7.1 and 1.3 <= ty <= 1.9
    from brain.explore import APPROACH_FAR_M, APPROACH_NEAR_M
    assert APPROACH_NEAR_M - 0.01 <= math.hypot(tx - gx, ty - gy) <= APPROACH_FAR_M + 0.01


def test_an_absent_target_ends_exhausted_not_blocked():
    runner, nav, grid = _mission(target="purple elephant", max_steps=200)
    s = runner.status()
    assert s["outcome"] == EXHAUSTED, s["log_tail"][-5:]


def test_a_goal_refused_for_authority_is_waited_on_not_failed():
    runner, nav, grid = _mission(refuse_first=2)
    s = runner.status()
    assert s["outcome"] == FOUND
    assert s["explore"]["goals_failed"] == 0


def test_no_verb_is_sent_while_a_goal_is_live():
    """3.23: a live goal makes the robot server refuse every other
    autonomous /action, which the runner would read as a preemption."""
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    sent = []
    real = robot.look_left
    robot.look_left = lambda *a, **k: (sent.append(nav.goal and nav.goal["state"]), real())[1]
    runner = MissionRunner(robot, target_object="red backpack", policy="explore",
                           navigator=nav, world=MockWorld(grid), max_steps=80,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    while runner.tick():
        pass
    assert sent and all(state != "active" for state in sent)


def test_a_blocked_doorway_is_set_aside_and_retried_after_it_clears():
    """Criterion 3's shape, on the fake: the person in the kitchen door makes
    every way in fail until they leave at 60 s; the search ends found."""
    runner, nav, grid = _mission(movers=build_movers("scaled_house", "kitchen_door_sitter"),
                                 max_steps=120)
    s = runner.status()
    assert s["outcome"] == FOUND, s["log_tail"][-8:]


def test_a_place_that_never_clears_is_dropped_after_the_limit():
    book = RetryBook(cooldown_s=30, limit=3)
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid), retry=book)
    for t in (0, 40, 80):
        book.fail(5.0, 1.0, t)
    assert not book.available(5.0, 1.0, 500)
    assert agent.retry is book


def test_explore_needs_a_navigator():
    grid = build_world("scaled_house")
    with pytest.raises(ValueError, match="navigator"):
        MissionRunner(MockRobot(grid, render=False), target_object="x", policy="explore")


class StuckNav(FakeNav):
    """nav2 that accepts a goal and then never finishes it."""

    def get_goal(self):
        self.calls.append(("get",))
        self.robot.pass_time(10.0)
        return {"goal": self.goal, "plan": []}


def test_a_goal_nav2_never_finishes_times_out_is_cancelled_and_set_aside():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = StuckNav(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid),
                         goal_timeout_s=60.0)
    first = agent.step()
    assert first.action == "GOAL"
    for _ in range(10):
        agent.step()
    assert ("cancel",) in nav.calls
    assert agent.goals_failed >= 1
    assert "timeout" in agent.last_event or agent.goals_sent >= 2


class WedgedNav(FakeNav):
    """nav2 that aborts every goal without the robot moving -- the furnished
    home's sofa-and-coffee-table trap."""

    def get_goal(self):
        self.calls.append(("get",))
        if self.goal and self.goal["state"] == "active":
            self.robot.pass_time(2.0)
            self.goal["state"] = "aborted"
        return {"goal": self.goal, "plan": []}


def test_a_goal_that_fails_without_moving_is_an_escape_not_a_failed_place():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=WedgedNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    assert agent.step().action == "GOAL"
    second = agent.step()
    assert "without moving" in agent.last_event or "without moving" in str(second.detail)
    assert second.action == "REVERSE"
    assert agent.retry.entries == []          # the place was not held against
    assert agent.step().action in ("LEFT", "RIGHT")
    assert agent.escapes == 1


def _agent_with_refusal(message):
    from robot.interface import Preempted
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))

    def refuse(action, **kw):
        raise Preempted(message)
    agent.safety.check_and_execute = refuse
    agent._pending = ["LOOK_LEFT"]
    return agent


def test_a_move_refused_for_the_missions_own_goal_waits():
    agent = _agent_with_refusal("a nav2 goal is active -- one autonomous driver at a time")
    action, executed, _ = agent._do_pending()
    assert action == "WAIT" and agent._pending == ["LOOK_LEFT"]


def test_a_move_refused_because_a_person_took_over_still_ends_the_mission():
    from robot.interface import Preempted
    agent = _agent_with_refusal("preempted by twin-dpad")
    with pytest.raises(Preempted):
        agent._do_pending()


def _agent_that_saw_the_backpack_once(nav_cls):
    """Kitchen door, backpack in view on the first frame only -- after that
    the camera never sees it again, as when an escape turns the robot away."""
    grid = build_world("scaled_house")
    grid.x, grid.y, grid.theta = 16.5, 4.5, math.radians(5)
    robot = MockRobot(grid, render=False)
    nav = nav_cls(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="red backpack"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid))
    real, calls = agent._sighting, []

    def once(frame):
        calls.append(1)
        return real(frame) if len(calls) == 1 else None
    agent._sighting = once
    agent._glimpse = lambda frame: None     # nor on a pan (3.69)
    return agent, nav


class OnceWedgedNav(FakeNav):
    """The first goal aborts without the robot moving; then nav2 as usual."""

    def get_goal(self):
        if self.goal and self.goal["state"] == "active" and not getattr(self, "_done", False):
            self._done = True
            self.calls.append(("get",))
            self.robot.pass_time(2.0)
            self.goal["state"] = "aborted"
            return {"goal": self.goal, "plan": []}
        return super().get_goal()


def test_a_lost_approach_goes_back_to_the_last_sighting():
    """The den run: it saw the backpack, the approach failed without moving,
    the escape turned it away -- and it never went back."""
    agent, nav = _agent_that_saw_the_backpack_once(OnceWedgedNav)
    first = agent.step()
    assert first.action == "GOAL" and agent._goal["kind"] == "approach"
    target = agent._goal["target"]
    # The failed try puts the spot on the retry cooldown, so a frontier may
    # come first -- but the robot must come back to it.
    back = None
    for _ in range(80):
        r = agent.step()
        if r.action == "GOAL" and agent._goal and agent._goal["kind"] == "approach":
            back = agent._goal["target"]
            break
    assert agent.escapes == 1
    assert back == target


def test_a_sighting_that_always_wedges_runs_out_of_tries():
    from brain.frontier import RETRY_LIMIT
    agent, nav = _agent_that_saw_the_backpack_once(WedgedNav)
    approaches = 0
    for _ in range(200):
        r = agent.step()
        if r.action == "GOAL" and agent._goal and agent._goal["kind"] == "approach":
            approaches += 1
    assert 1 < approaches <= RETRY_LIMIT
    assert agent._last_seen is None


def test_a_place_that_wedges_the_robot_again_runs_out_of_tries():
    """3.38's acceptance run: one goal, wedged against an armchair, sent
    eleven times -- a wedge was never held against the place."""
    from brain.frontier import RETRY_LIMIT
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = WedgedNav(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid))
    for _ in range(300):
        agent.step()
        robot.pass_time(1.0)
    sent = [c[1:] for c in nav.calls if c[0] == "set"]
    worst = max(sum(math.hypot(a[0] - b[0], a[1] - b[1]) < agent.retry.radius_m for b in sent)
                for a in sent)
    assert worst <= 1 + RETRY_LIMIT, worst


def test_an_escape_turn_refused_on_one_side_tries_the_other():
    """3.43: 2026-10-06's wedged runs asked for the same refused side every
    time while the other was free. The escape now tries the other way."""
    from robot.safety import SafetyViolation
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    asked = []

    def guard(action, **kw):
        asked.append(action)
        if action == "RIGHT":
            raise SafetyViolation("turn right clamped: a corner would come within 1.2cm")
        return {"executed": action}
    agent.safety.check_and_execute = guard
    agent.robot.get_scan = lambda: {"usable": False}       # no scan: RIGHT is tried first
    agent._pending = [("OPEN",)]
    agent._do_pending()
    agent._do_pending()
    assert asked == ["RIGHT", "LEFT"]


def test_every_goal_its_end_and_the_escape_are_logged_as_events(caplog):
    """3.45: a live run's brain log must say what explore chose before a
    stall and whether its escape ran -- 3.44's runs could say neither."""
    import json
    import logging
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=WedgedNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    with caplog.at_level(logging.INFO, logger="explore"):
        for _ in range(3):
            agent.step()
    names = [e["event"] for e in agent.events]
    assert names[:4] == ["goal_sent", "goal_ended", "escape", "verb"], names
    sent, ended, escape, verb = agent.events[:4]
    assert sent["kind"] == "frontier" and {"rank", "score", "path_m", "size_m"} <= set(sent)
    # 3.77: how long the choice took, wall clock, before nav2 was asked;
    # and every choice is counted, not only those that sent a goal.
    assert isinstance(sent["choice_ms"], float) and sent["choice_ms"] >= 0
    stats = agent.choice_stats()
    assert stats["n"] >= 1 and 0 <= stats["p50_ms"] <= stats["p90_ms"] <= stats["max_ms"]
    assert sent["choice_ms"] in agent.choice_times_ms    # one number per choice
    assert ended["state"] == "aborted" and ended["moved_m"] is not None
    assert escape["why"] == "wedged" and verb["action"] == "REVERSE"
    logged = [json.loads(r.getMessage().split(" ", 1)[1]) for r in caplog.records
              if r.getMessage().startswith("explore {")]
    assert logged == agent.events[:len(logged)] and len(logged) >= 4


def test_logging_is_configured_only_when_asked(monkeypatch):
    """3.45: PICAR_LOG_LEVEL unset leaves logging as found (deployments and
    the suite); set, one handler, however many apps are built."""
    import logging
    from robot.identity import configure_logging
    root = logging.getLogger()
    before, level = list(root.handlers), root.level
    try:
        monkeypatch.delenv("PICAR_LOG_LEVEL", raising=False)
        assert configure_logging() is None and root.handlers == before
        monkeypatch.setenv("PICAR_LOG_LEVEL", "info")
        assert configure_logging() == logging.INFO and configure_logging() == logging.INFO
        assert len(root.handlers) == len(before) + 1 and root.level == logging.INFO
        monkeypatch.setenv("PICAR_LOG_LEVEL", "LOUD")
        assert configure_logging() is None    # a typo warns; it never stops a server
    finally:
        root.handlers[:] = before
        root.setLevel(level)


def test_a_frontier_goal_leaves_the_safety_layer_room_to_drive_away():
    """3.45: on the SLAM map the furnished-home run really had 41 s in
    (2026-10-07 baseline run 1, the foyer where four of six earlier runs
    parked for up to 134 s), explore must not choose a goal the robot cannot
    drive away from. Today's 0.22 m goal clearance chose (4.17, 12.6), inside
    furniture on ground truth at every heading; the goal now keeps the
    chassis' half-length plus the stopping distance from the map's walls,
    and at every heading it can stand at, forward or reverse is allowed."""
    import gzip
    import importlib.util
    import json
    import os
    here = os.path.dirname(__file__)
    spec = importlib.util.spec_from_file_location(
        "slam345_goal_sweep", os.path.join(here, "..", "evaluations", "slam-345", "goal_sweep.py"))
    gs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gs)          # by path: no lasting sys.path entry
    path = os.path.join(os.path.dirname(__file__), "data", "explore_foyer_snapshot.json.gz")
    with gzip.open(path, "rt") as f:
        snap = json.load(f)
    kind, (x, y) = gs.choose(snap)
    ok, headings = gs.leavable(x, y)
    assert kind == "frontier" and ok, ((round(x, 2), round(y, 2)), headings)


def test_the_leave_clearance_is_the_half_length_plus_the_stopping_distance():
    from brain.explore import leave_clearance_m
    from robot.safety import FOOTPRINT_LENGTH_M
    assert leave_clearance_m(20.0) == pytest.approx(FOOTPRINT_LENGTH_M / 2 + 0.20)


def test_a_nook_with_no_leavable_goal_backs_out_before_ending_exhausted():
    """3.45 review: goals need the leave clearance (0.35 m on a 5 cm map),
    so "boxed in" must be counted at it too. A 0.55 m corridor whose open
    end is unknown has room for nav2 (0.22 m) but nowhere to stop and leave
    by: counted at 0.22 m it read as roomy, and the mission ended
    `searched` with the corridor's end unexplored."""
    from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN
    w, h, res = 40, 80, 0.05
    cells = [CELL_UNKNOWN] * (w * h)
    for y in range(9, 70):
        for x in range(13, 26):
            wall = x in (13, 25) or y == 9
            cells[y * w + x] = CELL_OCCUPIED if wall else CELL_FREE
    m = {"usable": True, "map_id": "nook", "resolution_m": res, "width": w, "height": h,
         "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": cells}
    pose = {"usable": True, "map_id": "nook", "x_m": 19.5 * res, "y_m": 20.5 * res,
            "heading_deg": 0.0}

    class Nook:
        def get_map(self):
            return m

        def get_pose(self):
            return pose

    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=FakeNav(robot), clock=lambda: 0.0, world=Nook())
    action, _ok, detail = agent._next_frontier(0.0)
    assert not agent.exhausted and action != "STOP", detail
    assert [e["why"] for e in agent.events if e["event"] == "escape"] == ["boxed"]


# ---------- 3.69: explore's misses in the imported houses ----------

def _kitchen_door(theta_deg, pan=0):
    grid = build_world("scaled_house")
    grid.x, grid.y, grid.theta = 16.5, 4.5, math.radians(theta_deg)
    grid.pan = pan
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="red backpack"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid))
    return agent, nav, grid


def test_a_target_seen_only_on_a_pan_turns_the_body_and_is_approached():
    """3.66 class A (8 of 16 misses): the backpack in view only while the
    camera looked left. A panned bearing cannot be approached, and nothing
    turned the body to look again -- so the next decision was a frontier."""
    agent, nav, grid = _kitchen_door(5 + 90, pan=-1)   # body south, camera east
    assert any(d["label"] == "red backpack"
               for d in agent.robot.get_camera_frame()["detections"])
    first_goal = None
    for _ in range(6):
        r = agent.step()
        if r.action == "GOAL":
            first_goal = agent._goal["kind"]
            break
    assert first_goal == "approach"
    assert grid.pan == 0


@pytest.mark.parametrize("house, fault", [
    # B: arrived inside the found radius, the turn to face it refused by
    # the pivot guard; smaller turns, then back off and come in facing it.
    (101, "face refused"),
    (132, "face refused"),
    # C: the lidar window at the sighting's bearing is two surfaces (1.97,
    # 2.2, 3.76 m); the median took the wall behind the backpack.
    (125, "edge range"),
    # D: the view goal stood the robot behind a wall from the unseen patch.
    (108, "look-from with no line of sight"),
    # The back-off's guard: going back to a sighting it could not face from
    # looped until the step budget ran out (found on dev; lost mid-fix).
    (36, "back-off loop"),
])
def test_the_imported_house_misses_are_found(house, fault):
    """3.69: one ProcTHOR house per 3.66 miss class (the 100-house sweep in
    tests/test_procthor_sweep.py is the measurement; these name the fault)."""
    runner, nav, grid = _mission(house=f"procthor:val:{house}", max_steps=200)
    assert runner.status()["outcome"] == FOUND, (fault, runner.status()["log_tail"][-3:])


def _agent_that_cannot_turn():
    from robot.safety import SafetyViolation
    agent, nav, grid = _kitchen_door(5)

    def guard(action, **kw):
        if action in ("LEFT", "RIGHT"):
            raise SafetyViolation(f"Blocked {action}: a corner would come within 0.0cm")
        return {"action": action}
    agent.safety.check_and_execute = guard
    return agent, grid


def test_a_refused_turn_toward_an_unseen_patch_keeps_the_targets_sighting():
    """3.69 review: only a turn toward the TARGET backs off and lets its
    sighting go; a view goal's turn that is refused must not forget it."""
    agent, grid = _agent_that_cannot_turn()
    agent._last_seen = (6.9, 1.6)                        # the backpack
    agent._pending = [("FACE", (grid.x * CELL - 2.0, grid.y * CELL))]   # a patch due west
    for _ in range(8):
        if not agent._pending:
            break
        agent._do_pending()
    assert agent._last_seen == (6.9, 1.6)
    assert "REVERSE" not in agent._pending and "FORWARD" not in agent._pending


def test_a_backed_off_turn_that_fails_again_does_not_drive_forward_blind():
    """3.69 review: REVERSE, the turn, then FORWARD to come in facing it --
    if the turn is refused too, the FORWARD would go wherever it points."""
    agent, grid = _agent_that_cannot_turn()
    agent._last_seen = (grid.x * CELL - 0.6, grid.y * CELL)  # behind the robot's west side
    agent._pending = [("FACE", agent._last_seen, 90, False, True)]   # toward the target
    sent = []
    real = agent.safety.check_and_execute

    def spy(action, **kw):
        sent.append(action)
        return real(action, **kw)
    agent.safety.check_and_execute = spy
    for _ in range(12):
        if not agent._pending:
            break
        agent._do_pending()
    assert "REVERSE" in sent and "FORWARD" not in sent


def test_an_edge_return_at_arms_length_gives_no_range():
    """3.69 reviews: at an edge with the nearest return inside the approach
    band, neither reading can be trusted -- the nearest may be a jamb (read
    as "already there"), the median the wall behind a target that close (a
    goal past it). No sighting; another frame decides."""
    agent, nav, grid = _kitchen_door(5)
    scan = {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
            "ranges_m": [2.0] * 360}
    hits = [{"label": "red backpack", "bearing_deg": 0.0}]
    for i in range(178, 183):                       # beams -2..+2 deg
        scan["ranges_m"][i] = 0.3 if i == 178 else 2.0
    agent.robot.get_scan = lambda: scan
    assert agent._sighting({"detections": hits}) is None
    scan["ranges_m"][178] = 1.2                     # beyond the band: the nearest
    tx, ty = agent._sighting({"detections": hits})
    assert math.hypot(tx - grid.x * CELL, ty - grid.y * CELL) == pytest.approx(1.2, abs=0.1)


def test_a_refused_glimpse_turn_never_fails_a_place_or_forgets_a_sighting():
    """3.69 review 2: a glimpse's point is a direction 1 m out, not where the
    target is -- a refusal may back off, but must not put a made-up point in
    the retry book or drop a real sighting."""
    agent, grid = _agent_that_cannot_turn()
    agent._last_seen = (6.9, 1.6)
    agent._pending = [("FACE", (grid.x * CELL - 1.0, grid.y * CELL), 90, False, "glimpse")]
    for _ in range(16):
        if not agent._pending:
            break
        agent._do_pending()
    assert agent._last_seen == (6.9, 1.6)
    assert agent.retry.available(grid.x * CELL - 1.0, grid.y * CELL, grid.sim_time)


def test_a_close_target_with_one_beam_past_it_keeps_its_range():
    """3.69 review 3: 0.5 m ahead, four beams on it and one past it -- the
    median is the target; dropping the sighting drove the robot away."""
    agent, nav, grid = _kitchen_door(5)
    scan = {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
            "ranges_m": [2.0] * 360}
    for i in range(178, 182):
        scan["ranges_m"][i] = 0.5
    agent.robot.get_scan = lambda: scan
    tx, ty = agent._sighting({"detections": [{"label": "red backpack", "bearing_deg": 0.0}]})
    assert math.hypot(tx - grid.x * CELL, ty - grid.y * CELL) == pytest.approx(0.5, abs=0.1)


def test_a_glimpse_backs_off_at_most_once_from_one_place():
    """3.69 review 3: each new glimpse from the same spot queued another
    REVERSE, walking the robot backwards a step at a time."""
    agent, grid = _agent_that_cannot_turn()
    sent = []
    real = agent.safety.check_and_execute

    def spy(action, **kw):
        sent.append(action)
        if action == "REVERSE":
            return {"action": action}             # backs off -- but does not move here
        return real(action, **kw)
    agent.safety.check_and_execute = spy
    for _ in range(3):                            # three glimpses, one spot
        agent._pending = [("FACE", (grid.x * CELL - 1.0, grid.y * CELL), 90, False, "glimpse")]
        for _ in range(16):
            if not agent._pending:
                break
            agent._do_pending()
    assert sent.count("REVERSE") == 1


def test_a_creep_is_logged_with_the_floor_band_reading(caplog):
    """3.74: whether the band's blind range needs closing waits on how often
    a creep runs on the car, and whether the band saw something ahead just
    before -- so every creep is an event carrying both readings."""
    import json
    import logging
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    agent.safety.check_and_execute = lambda action, **kw: {"executed": action}
    agent._pending = [("CREEP", ["FORWARD"])]
    with caplog.at_level(logging.INFO, logger="explore"):
        agent._do_pending()
    creeps = [json.loads(r.getMessage()[len("explore "):]) for r in caplog.records
              if r.getMessage().startswith("explore {") and '"creep"' in r.getMessage()]
    assert len(creeps) == 1
    assert creeps[0]["way"] == "FORWARD" and creeps[0]["executed"] is True
    assert creeps[0]["floor"][1] in ("floor_band", "floor_band_no_target")
    assert "forward" in creeps[0] and creeps[0]["waited"] is False
    assert isinstance(creeps[0]["floor_blind"], int)


def test_a_sock_inside_the_bands_blind_range_shows_as_blind_zones(caplog):
    """3.74 review: inside its minimum range the band's own answer reads
    clear, so the creep event also counts the band zones that read
    unusable -- what the car's count looks for."""
    import json
    import logging
    from sim.grid_world import GridWorld, Heading
    layout = ["#" * 10, "#........#", "#........#", "#........#", "#" * 10]
    grid = GridWorld(layout, {"r": set()}, objects={(2, 2): "sock"}, robot_x=1, robot_y=2,
                     heading=Heading.E, object_heights={"sock": 0.05})
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    agent.safety.check_and_execute = lambda action, **kw: {"executed": action}
    agent._pending = [("CREEP", ["FORWARD"])]
    with caplog.at_level(logging.INFO, logger="explore"):
        agent._do_pending()
    creep = [json.loads(r.getMessage()[len("explore "):]) for r in caplog.records
             if '"creep"' in r.getMessage()][0]
    # Since 3.76 K1 the band's own answer says it is blind there (it read
    # "floor_band_no_target", clear, when every zone was unusable).
    assert creep["floor"][1] == "floor_band_unusable"
    assert creep["floor_blind"] > 0


def test_a_failing_reading_never_stops_the_creep():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    asked = []
    agent.safety.check_and_execute = lambda action, **kw: asked.append(action) or {"executed": action}

    def broken(*a, **k):
        raise RuntimeError("sensor gone")
    agent.safety.floor_clearance = broken
    agent._pending = [("CREEP", ["FORWARD"])]
    agent._do_pending()
    assert asked == ["FORWARD"]


class FirstRefusedNav(FakeNav):
    """Refuses each new approach goal once with `preempted`, then takes it --
    live, the brain's own FACE turn still held driving authority when the
    approach after it was sent (3.83's pantry run)."""

    def __init__(self, robot):
        super().__init__(robot)
        self.refused = set()

    def set_goal(self, x_m, y_m):
        key = (round(x_m, 1), round(y_m, 1))
        if key not in self.refused:
            self.refused.add(key)
            self.calls.append(("set", round(x_m, 2), round(y_m, 2)))
            return {"accepted": False, "reason": "preempted"}
        return super().set_goal(x_m, y_m)


def test_a_refused_approach_is_tried_again_not_given_up_for_a_look():
    """3.85: in 3.83's pantry run an approach refused `preempted` (the
    brain's own FACE turn still held authority) was still recorded as made,
    so the next frames' sightings of the same spot never re-sent it, and
    `_back_to_sighting` only faced the target from outside the door --
    "already facing it" for 1,145 s, the backpack half behind the jamb."""
    grid = build_world("home_first_floor")
    grid.move_object((46, 19), (40, 23))                 # 3.83's pantry spot
    grid.x, grid.y = 12.928 / CELL, 7.321 / CELL
    grid.theta = math.radians(299.47) - math.pi / 2
    robot = MockRobot(grid, render=False)
    nav = FirstRefusedNav(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="red backpack"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid))
    target = (12.15, 7.05)
    first = agent._send_approach(*target, grid.sim_time)
    assert first[0] == "WAIT" and "preempted" in first[2]
    # The same sighting on the next frame: the approach goes out again.
    again = agent._send_approach(*target, grid.sim_time) if agent._approach is None else None
    assert again is not None and again[0] == "GOAL"


class SwitchNav(FakeNav):
    """Takes goals until `refuse` is set, then refuses them `preempted`."""

    refuse = False

    def set_goal(self, x_m, y_m):
        if self.refuse:
            self.calls.append(("set", round(x_m, 2), round(y_m, 2)))
            return {"accepted": False, "reason": "preempted"}
        return super().set_goal(x_m, y_m)


def test_a_refused_approach_leaves_no_older_approach_on_record():
    """3.85 review: approach A taken, then B (> APPROACH_MOVED_M away)
    refused -- the record must not still say A, or a sighting back near A
    is never approached again (the pantry livelock by another road)."""
    grid = build_world("home_first_floor")
    grid.move_object((46, 19), (40, 23))
    grid.x, grid.y = 12.928 / CELL, 7.321 / CELL
    robot = MockRobot(grid, render=False)
    nav = SwitchNav(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="red backpack"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid))
    a, b = (12.15, 7.05), (12.15, 5.9)
    assert agent._send_approach(*a, grid.sim_time)[0] == "GOAL"
    nav.refuse = True
    agent._send_approach(*b, grid.sim_time)
    nav.refuse = False
    agent._goal = None
    agent._pending = []
    agent._sighting = lambda frame: a                     # seen near A again
    sent_before = len([c for c in nav.calls if c[0] == "set"])
    agent.step()
    assert len([c for c in nav.calls if c[0] == "set"]) > sent_before


class SlowNav(FakeNav):
    """nav2 that keeps a goal active; each poll is 2 s, and the robot moves
    `step_m` (east) per poll -- 0 is 3.83's wedge under the dining table."""

    def __init__(self, robot, step_m=0.0):
        super().__init__(robot)
        self.step_m = step_m

    def get_goal(self):
        self.calls.append(("get",))
        self.robot.pass_time(2.0)
        self.grid.x += self.step_m / CELL
        return {"goal": self.goal, "plan": []}


def _slow_agent(step_m):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = SlowNav(robot, step_m)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid))
    return grid, nav, agent


def test_a_goal_that_makes_no_progress_is_cut_as_stalled_and_backed_out_of():
    """3.83: wedged under the dining table, every goal ran its full 120 s.
    No STALL_M of progress over STALL_S ends it early, and the wedge
    handling backs out -- a keepout spot cannot free a robot the
    controller keeps pushing forward (3.68)."""
    from brain.explore import STALL_S
    grid, nav, agent = _slow_agent(0.0)
    assert agent.step().action == "GOAL"
    t0 = grid.sim_time
    while agent._goal is not None and grid.sim_time - t0 < 60:
        agent.step()
    ended = [e for e in agent.events if e["event"] == "goal_ended"][0]
    assert ended["state"] == "stalled"
    assert STALL_S <= ended["seconds"] <= STALL_S + 4.0
    assert ("cancel",) in nav.calls
    assert [e for e in agent.events if e["event"] == "escape"][0]["why"] == "wedged"


def test_a_goal_still_moving_is_not_cut():
    from brain.explore import GOAL_TIMEOUT_S
    grid, nav, agent = _slow_agent(0.1)          # 0.1 m a 2 s poll: slow, but moving
    agent.goal_timeout_s = GOAL_TIMEOUT_S
    assert agent.step().action == "GOAL"
    t0 = grid.sim_time
    while agent._goal is not None and grid.sim_time - t0 < 60:
        agent.step()
    assert not [e for e in agent.events if e["event"] == "goal_ended"]


def test_a_goal_is_not_cut_before_stall_s():
    from brain.explore import STALL_S
    grid, nav, agent = _slow_agent(0.0)
    assert agent.step().action == "GOAL"
    t0 = grid.sim_time
    while grid.sim_time - t0 < STALL_S - 4.0:
        agent.step()
    assert agent._goal is not None


def test_an_outage_longer_than_stall_s_is_not_read_as_a_stall():
    """3.83 review: with the pose unusable for longer than STALL_S, the
    first tick back must not cut the goal -- an outage is not a wedge
    (3.76: not the place's failure either). The window starts again."""
    from brain.explore import STALL_S
    grid, nav, agent = _slow_agent(0.0)
    assert agent.step().action == "GOAL"
    real = agent.world.get_pose
    t0 = grid.sim_time
    agent.world.get_pose = lambda: {"usable": False}
    while grid.sim_time - t0 < STALL_S + 10:
        agent.step()
    agent.world.get_pose = real
    agent.step()
    assert agent._goal is not None, [e for e in agent.events if e["event"] == "goal_ended"]


def test_a_stall_back_out_does_not_spend_the_boxed_in_budget():
    """3.83 review: escapes after a stall cut must not use up MAX_ESCAPES,
    or a robot still boxed in under the table ends the search `exhausted`
    instead of backing out again."""
    from brain.explore import MAX_ESCAPES
    grid, nav, agent = _slow_agent(0.0)
    for _ in range(MAX_ESCAPES + 1):
        assert agent.step().action == "GOAL" or agent._goal is not None
        t0 = grid.sim_time
        while agent._goal is not None and grid.sim_time - t0 < 60:
            agent.step()
        while agent._pending:
            agent.step()
    assert agent.escapes >= MAX_ESCAPES
    assert agent.escapes_charged == 0


def test_a_stall_after_moving_says_so():
    grid, nav, agent = _slow_agent(0.0)
    assert agent.step().action == "GOAL"
    agent._goal["from"] = (agent._goal["from"][0] - 2.0, agent._goal["from"][1])
    t0 = grid.sim_time
    while agent._goal is not None and grid.sim_time - t0 < 60:
        agent.step()
    assert "without moving" not in agent.last_event and "stalled" in agent.last_event


def test_a_stall_with_no_pose_at_its_end_still_reads_cleanly():
    grid, nav, agent = _slow_agent(0.0)
    assert agent.step().action == "GOAL"
    agent._goal["from"] = None
    t0 = grid.sim_time
    while agent._goal is not None and grid.sim_time - t0 < 60:
        agent.step()
    assert "None" not in agent.last_event and "stalled" in agent.last_event


def _escape_agent(front_cm, rear_cm=1.3):
    from robot.safety import SafetyViolation
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))

    def guard(action, **kw):
        if action == "REVERSE":
            raise SafetyViolation(f"Blocked REVERSE: rear clearance={rear_cm}cm < min=20cm")
        return {"executed": action}
    agent.safety.check_and_execute = guard
    agent.safety.forward_clearance = lambda: (front_cm, "scan_footprint")
    agent.safety.reverse_clearance = lambda: (rear_cm, "scan_footprint")
    agent._pending = ["REVERSE", ("CREEP",), ("OPEN",)]
    return agent


def test_an_escape_with_the_rear_shut_tries_the_open_front():
    """3.85: in 3.83's breakfast pocket the rear was 1.3 cm and the front
    about 30 cm, and the escape (REVERSE, CREEP, OPEN) never tried ahead."""
    agent = _escape_agent(front_cm=30.0)
    agent._do_pending()
    assert agent._pending[0] == "FORWARD"


def test_an_escape_with_both_ends_shut_creeps_and_turns_as_before():
    agent = _escape_agent(front_cm=15.0)
    agent._do_pending()
    assert agent._pending[0] == ("CREEP",)


def _ended_as(state, moved_m):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    pose = agent._pose()
    goal = (pose["x_m"] + 2.0, pose["y_m"])
    agent._goal = {"x_m": goal[0], "y_m": goal[1], "kind": "frontier", "sent_at": 0.0,
                   "from": (pose["x_m"] - moved_m, pose["y_m"])}
    agent._goal_ended(state, grid.sim_time)
    return agent.retry.failures_near(*goal)


def test_a_stall_after_moving_counts_against_the_place_at_once():
    """3.85: 3.83's breakfast trap frontier (8.63, 10.23) was sent three
    times -- a stall was forgiven as a first wedge."""
    assert _ended_as("stalled", moved_m=1.39) == 1


def test_a_goal_that_never_left_keeps_the_first_wedge_forgiven():
    assert _ended_as("stalled", moved_m=0.0) == 0


def test_a_front_just_past_the_stop_is_not_worth_a_forward():
    """3.85 review (both reviewers): FORWARD is queued only with room to
    gain over the bar it is vetted against -- at 20.5 cm it moves 0.5 cm."""
    agent = _escape_agent(front_cm=20.5)
    agent._do_pending()
    assert agent._pending[0] == ("CREEP",)


def test_the_forward_bar_is_the_speed_dependent_one(monkeypatch):
    """The FORWARD is vetted at its own speed (3.44's stop distance), so
    a faster default speed raises the front it needs."""
    agent = _escape_agent(front_cm=30.0)
    monkeypatch.setattr(agent.safety, "required_clearance_cm", lambda v, creep=False: 40.0)
    agent._do_pending()
    assert agent._pending[0] == ("CREEP",)
