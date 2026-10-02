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
from control.mission_runner import FOUND, MissionRunner, SEARCHED
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


def test_an_absent_target_ends_searched_not_blocked():
    runner, nav, grid = _mission(target="purple elephant", max_steps=200)
    s = runner.status()
    assert s["outcome"] == SEARCHED, s["log_tail"][-5:]


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
