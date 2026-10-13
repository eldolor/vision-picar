"""
tests/test_mission_ends.py

PLAN-ros-alignment.md 3.70 Part C (E1-E5): the four end states -- Found,
Exhausted, Out of budget, Cancelled -- beside the fault outcomes, in process.

* E1: every ending maps to one of the four or to a fault, `searched`,
  `stopped`, `preempted` and `max_steps` are never written again, and old
  records that carry them are READ as the new names, never rewritten.
* E3: a labelled room the robot never entered keeps a mission from ending
  Exhausted.
* E4: a time limit and a battery floor end a mission Out of budget, with
  `max_steps` only a backstop derived from the time limit.
* E5: Stop and a D-pad takeover both end it Cancelled, with `cancelled_by`.

E2 (Found) is the existing arrival tests, unchanged; E6 (the phone) is
tests/test_ui_mission_ends.py.
"""

import math
import time

import pytest

from brain.explore import ExploreAgent
from brain.memory import MissionMemory
from control import mission_runner as mr
from control.mission_runner import (
    ARRIVED_UNCONFIRMED,
    BLOCKED,
    CANCELLED,
    END_STATES,
    EXHAUSTED,
    FAILED,
    FOUND,
    OUT_OF_BUDGET,
    ROOM_REACHED,
    MissionRunner,
    derived_max_steps,
    read_outcome,
)
from robot.interface import Preempted, RobotInterface, unusable_battery
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.conftest import fresh_mock_runner
from tests.test_explore import FakeNav
from world.interface import CELL_FREE, CELL_OCCUPIED


def _drive(runner, max_ticks=5000):
    ticks = 0
    while runner.tick():
        ticks += 1
        assert ticks < max_ticks, "runner never terminated"
    assert runner.report_done.wait(10), "the report never landed"
    return runner.status()


# ---------- E1: the names ----------

def test_the_four_end_states_are_named():
    assert END_STATES == ("found", "exhausted", "out_of_budget", "cancelled")
    assert (EXHAUSTED, OUT_OF_BUDGET, CANCELLED) == ("exhausted", "out_of_budget", "cancelled")


def test_the_retired_names_are_gone_from_the_runner():
    """No constant left that could write one."""
    for old in ("SEARCHED", "STOPPED", "PREEMPTED", "MAX_STEPS"):
        assert not hasattr(mr, old), old


@pytest.mark.parametrize("old, new, by", [
    ("searched", "exhausted", None),
    ("max_steps", "out_of_budget", None),
    ("stopped", "cancelled", "stop"),
    ("preempted", "cancelled", "dpad"),
    ("found", "found", None),
    ("failed", "failed", None),
    ("blocked", "blocked", None),
    ("arrived_unconfirmed", "arrived_unconfirmed", None),
    ("room_reached", "room_reached", None),
    ("exhausted", "exhausted", None),
    ("cancelled", "cancelled", None),
])
def test_old_records_are_read_as_the_new_names(old, new, by):
    assert read_outcome(old) == (new, by)


def test_a_new_record_keeps_its_own_cancelled_by():
    assert read_outcome("cancelled", "dpad") == ("cancelled", "dpad")
    assert read_outcome("stopped", "dpad") == ("cancelled", "dpad")


@pytest.mark.parametrize("outcome, end_state", [
    (FOUND, "found"), (EXHAUSTED, "exhausted"), (OUT_OF_BUDGET, "out_of_budget"),
    (CANCELLED, "cancelled"), (FAILED, None), (BLOCKED, None),
    (ARRIVED_UNCONFIRMED, None), (ROOM_REACHED, None),
])
def test_status_carries_the_end_state_and_its_reason(outcome, end_state):
    runner = fresh_mock_runner(target_object="red backpack")
    runner.start()
    runner._finish(outcome, "why it ended")
    s = runner.status()
    assert s["outcome"] == outcome
    assert s["end_state"] == end_state
    assert s["end_reason"] == "why it ended"


def test_a_running_mission_has_no_end_state():
    runner = fresh_mock_runner(target_object="red backpack")
    runner.start()
    s = runner.status()
    assert s["end_state"] is None and s["end_reason"] is None and s["cancelled_by"] is None


# ---------- E5: Cancelled ----------

def test_stop_ends_cancelled_by_stop_with_the_wheels_at_zero():
    runner = fresh_mock_runner(target_object="red backpack", max_steps=150)
    runner.start()
    for _ in range(5):
        runner.tick()
    t0 = time.monotonic()
    runner.stop()
    s = runner.status()
    assert s["outcome"] == CANCELLED and s["end_state"] == "cancelled"
    assert s["cancelled_by"] == "stop"
    w = runner.robot.get_wheel_state()
    assert w["left"]["velocity_rad_s"] == 0 and w["right"]["velocity_rad_s"] == 0
    assert time.monotonic() - t0 < 0.5


class _TakenOver(RobotInterface):
    """A body whose every move is refused because a person took it (M4)."""

    def __init__(self, inner):
        self.inner = inner
        self.stops = 0

    def __getattr__(self, name):
        return getattr(self.inner, name)

    message = "twin-dpad is driving -- brain is lower priority and was refused"

    def _taken(self, *a, **k):
        raise Preempted(self.message)

    drive_forward = reverse = turn_left = turn_right = _taken
    look_left = look_right = _taken
    set_wheel_velocity = _taken

    def verb_plan(self, *a, **k):
        raise Preempted(self.message)

    def look_center(self):
        return {}

    def stop(self):
        self.stops += 1
        return self.inner.stop()

    def get_camera_frame(self):
        return self.inner.get_camera_frame()

    def get_distance(self):
        return self.inner.get_distance()

    def get_depth_grid(self):
        return self.inner.get_depth_grid()

    def get_odometry(self):
        return self.inner.get_odometry()

    def get_wheel_state(self):
        return self.inner.get_wheel_state()

    def get_scan(self, max_range_m=None):
        return self.inner.get_scan(max_range_m=max_range_m)


def test_a_dpad_takeover_ends_cancelled_by_dpad():
    from tests.conftest import fresh_mock_robot, mock_world_for
    inner = fresh_mock_robot()
    robot = _TakenOver(inner)
    runner = MissionRunner(robot, target_object="red backpack", world=mock_world_for(inner))
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == CANCELLED, s["log_tail"]
    assert s["cancelled_by"] == "dpad"
    assert robot.stops >= 1


def test_cancelled_carries_where_it_looked():
    runner = fresh_mock_runner(target_object="red backpack", max_steps=150)
    runner.start()
    for _ in range(10):
        runner.tick()
    runner.stop()
    assert runner.report_done.wait(10)
    report = runner.status()["report"]
    assert report is not None
    assert report["rooms_entered"] == runner.status()["rooms_visited"]
    assert set(report) >= {"rooms_entered", "coverage", "goals_reached", "map_id",
                           "map_version", "mapped_free_m2"}


# ---------- E4: Out of budget ----------

def test_max_steps_is_derived_from_the_time_limit():
    assert derived_max_steps(1800) == 7200       # 30 min, a photo every 0.5 s, x2
    assert derived_max_steps(120) == 480
    assert derived_max_steps(None) == mr.DEFAULT_MAX_STEPS


def _sim_runner(**kw):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="purple elephant", world=MockWorld(grid),
                           **kw)
    return runner, robot, grid


def test_the_time_limit_ends_out_of_budget_within_one_tick():
    runner, robot, grid = _sim_runner(mission_time_limit_s=20.0)
    assert runner.max_steps == derived_max_steps(20.0)
    runner.start()
    t0 = grid.sim_time
    last = t0
    while runner.tick():
        last = grid.sim_time
    s = runner.status()
    assert s["outcome"] == OUT_OF_BUDGET, s["log_tail"]
    assert s["end_state"] == "out_of_budget"
    assert "time limit" in s["end_reason"]
    # Ended on the first tick that started past the limit: the limit plus
    # one tick at most.
    assert last - t0 < 20.0 + 5.0
    w = robot.get_wheel_state()
    assert w["left"]["velocity_rad_s"] == 0 and w["right"]["velocity_rad_s"] == 0
    assert s["budget"]["time_limit_s"] == 20.0
    assert s["budget"]["elapsed_s"] >= 20.0


def test_the_step_backstop_ends_out_of_budget_when_the_clock_is_stuck():
    """The clock check disabled (a clock that never moves): the derived step
    count still ends it, as Out of budget -- never `max_steps`."""
    runner, robot, grid = _sim_runner(mission_time_limit_s=5.0, clock=lambda: 0.0)
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == OUT_OF_BUDGET, s["log_tail"]
    assert s["step"] == derived_max_steps(5.0)
    assert "step backstop" in s["end_reason"]


def test_an_explicit_max_steps_still_ends_out_of_budget():
    runner = fresh_mock_runner(target_object="purple elephant", max_steps=7)
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == OUT_OF_BUDGET
    assert s["step"] == 7


class _Battery(RobotInterface):
    """A sim body whose battery reads from a script."""

    def __init__(self, inner, volts):
        self.inner, self.volts, self.reads = inner, list(volts), 0

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def get_battery(self):
        self.reads += 1
        v = self.volts.pop(0) if len(self.volts) > 1 else self.volts[0]
        return {"usable": True, "v": v}

    # RobotInterface's abstract methods, passed through.
    def drive_forward(self, *a, **k): return self.inner.drive_forward(*a, **k)
    def reverse(self, *a, **k): return self.inner.reverse(*a, **k)
    def turn_left(self, *a, **k): return self.inner.turn_left(*a, **k)
    def turn_right(self, *a, **k): return self.inner.turn_right(*a, **k)
    def look_left(self): return self.inner.look_left()
    def look_right(self): return self.inner.look_right()
    def look_center(self): return self.inner.look_center()
    def stop(self): return self.inner.stop()
    def get_camera_frame(self): return self.inner.get_camera_frame()
    def get_distance(self): return self.inner.get_distance()
    def get_depth_grid(self): return self.inner.get_depth_grid()
    def get_odometry(self): return self.inner.get_odometry()
    def get_wheel_state(self): return self.inner.get_wheel_state()
    def get_scan(self, max_range_m=None): return self.inner.get_scan(max_range_m=max_range_m)
    def verb_plan(self, *a, **k): return self.inner.verb_plan(*a, **k)
    def verb_done(self, *a, **k): return self.inner.verb_done(*a, **k)
    def set_wheel_velocity(self, *a): return self.inner.set_wheel_velocity(*a)


def _battery_runner(volts, floor=10.5):
    grid = build_world("scaled_house")
    inner = MockRobot(grid, render=False)
    robot = _Battery(inner, volts)
    runner = MissionRunner(robot, target_object="purple elephant", world=MockWorld(grid),
                           mission_time_limit_s=600.0, battery_floor_v=floor,
                           clock=lambda: grid.sim_time, idle=inner.pass_time)
    return runner, robot, grid


def test_a_battery_below_the_floor_ends_out_of_budget_within_two_seconds():
    runner, robot, grid = _battery_runner([12.0] * 6 + [10.0])
    runner.start()
    crossed_at = None
    while runner.tick():
        if crossed_at is None and robot.volts == [10.0]:
            crossed_at = grid.sim_time
    s = runner.status()
    assert s["outcome"] == OUT_OF_BUDGET, s["log_tail"]
    assert "battery" in s["end_reason"]
    assert crossed_at is not None and grid.sim_time - crossed_at <= 2.0
    assert s["budget"]["battery_v"] == 10.0


def test_one_low_reading_is_a_sag_not_an_empty_battery():
    """A single reading under the floor (a motor's inrush) does not end it;
    two in a row do."""
    runner, robot, grid = _battery_runner([12.0, 12.0, 10.0] + [12.0] * 200)
    runner.start()
    for _ in range(40):
        if not runner.tick():
            break
    assert runner.is_running(), runner.status()["log_tail"]


def test_no_battery_reading_never_ends_a_mission():
    runner = fresh_mock_runner(target_object="red backpack", max_steps=150,
                               battery_floor_v=10.5)
    assert runner.robot.get_battery() == unusable_battery()
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == FOUND


def test_out_of_budget_reports_where_it_looked():
    runner, robot, grid = _sim_runner(mission_time_limit_s=30.0)
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == OUT_OF_BUDGET
    report = s["report"]
    assert report["rooms_entered"] == s["rooms_visited"]
    assert report["rooms_entered"], "the robot entered no room in 30 s"
    assert 0.0 < report["coverage"] <= 1.0
    assert report["mapped_free_m2"] > 0
    assert report["map_version"] is not None


# ---------- the fake board's battery (E4's sim trigger) ----------

def test_the_fake_board_drains_its_battery_and_the_host_reads_it():
    from robot.hardware_robot import HardwareRobot
    from sim.fake_esp32 import FakeEsp32
    body = MockRobot(build_world("scaled_house"), render=False)
    board = FakeEsp32(body, battery_v=11.0, drain_v_per_s=2.0)
    robot = HardwareRobot(board.path)
    try:
        deadline = time.time() + 2.0
        while not robot.get_battery().get("usable") and time.time() < deadline:
            time.sleep(0.02)
        first = robot.get_battery()
        assert first["usable"] and 10.0 < first["v"] <= 11.0
        time.sleep(0.6)
        later = robot.get_battery()
        assert later["v"] < first["v"] - 0.5
    finally:
        robot.close()
        board.close()


def test_a_draining_board_ends_a_mission_out_of_budget_within_two_seconds():
    """E4 through the hardware path: the motor board's `v` read by
    `HardwareRobot`, a frontier mission on it, the floor crossed while it
    runs -- the mission ends Out of budget within 2 s of the crossing, the
    wheels commanded to zero."""
    from robot.hardware_robot import HardwareRobot
    from sim.fake_esp32 import FakeEsp32
    grid = build_world("scaled_house")
    body = MockRobot(grid, render=False)
    floor, v0, drain = 10.5, 11.5, 0.5          # crosses the floor 2.0 s in
    board = FakeEsp32(body, battery_v=v0, drain_v_per_s=drain)
    robot = HardwareRobot(board.path, sensors=body)
    try:
        deadline = time.time() + 2.0
        while not robot.get_battery().get("usable") and time.time() < deadline:
            time.sleep(0.02)
        runner = MissionRunner(robot, target_object="purple elephant", world=MockWorld(grid),
                               mission_time_limit_s=600.0, battery_floor_v=floor,
                               clock=time.monotonic, idle=time.sleep)
        crossing = board._battery_since + (v0 - floor) / drain
        runner.start()
        while runner.tick():
            assert time.monotonic() < crossing + 10, "never ended"
        ended = time.monotonic()
        s = runner.status()
        assert s["outcome"] == OUT_OF_BUDGET, s["log_tail"]
        assert "battery" in s["end_reason"]
        assert ended - crossing <= 2.0, ended - crossing
        assert robot._cmd == (0.0, 0.0)
    finally:
        robot.close()
        board.close()


def test_the_fake_board_holds_its_voltage_without_a_drain():
    from sim.fake_esp32 import FakeEsp32
    body = MockRobot(build_world("scaled_house"), render=False)
    board = FakeEsp32(body)
    try:
        assert board.battery_v() == 12.0
    finally:
        board.close()


# ---------- E3: Exhausted waits for every labelled room ----------

class _KnownLabelledWorld(MockWorld):
    """The sim world with its map fully known (no frontier anywhere) and a
    room-label layer over it (what a survey will attach, S2) for the rooms
    given -- so the only thing left to do is enter a labelled room."""

    def __init__(self, grid, rooms):
        super().__init__(grid)
        self._rooms = sorted(rooms)

    def get_map(self):
        m = super().get_map()
        g = self.world
        names, cells, labels = self._rooms, [], []
        for y in range(g.height):
            for x in range(g.width):
                free = g._is_passable(x, y)
                cells.append(CELL_FREE if free else CELL_OCCUPIED)
                room = g.room_at(x, y)
                labels.append(names.index(room) if free and room in names else -1)
        return {**m, "cells": cells, "room_labels": {"names": names, "cells": labels}}


def _known_house_agent(rooms_fn):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    here = grid.room_at(grid.robot_x, grid.robot_y)
    rooms = rooms_fn(grid, here)
    world = _KnownLabelledWorld(grid, rooms)
    nav = FakeNav(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=nav, clock=lambda: grid.sim_time, world=world)
    # The camera has looked at every free cell already: no view gap either.
    m = world.get_map()
    for j in range(m["height"]):
        for i in range(m["width"]):
            if m["cells"][j * m["width"] + i] == CELL_FREE:
                agent.seen.add((round((i + 0.5) * m["resolution_m"], 3),
                                round((j + 0.5) * m["resolution_m"], 3)))
    return agent, grid, here, rooms


def test_a_labelled_room_never_entered_keeps_it_from_ending_exhausted():
    """No frontier and no view gap left, one labelled room the robot never
    entered: the policy goes there before it ends Exhausted (red first: it
    ended at once)."""
    agent, grid, here, rooms = _known_house_agent(
        lambda g, here: [here, next(r for r in sorted(g.rooms) if r != here)])
    other = next(r for r in rooms if r != here)
    first = agent.step()
    assert not agent.exhausted, first.detail
    assert first.action == "GOAL" and "room" in first.detail, first.detail
    for _ in range(40):
        if agent.exhausted:
            break
        agent.step()
    assert other in agent.rooms_entered, agent.last_event
    assert grid.room_at(grid.robot_x, grid.robot_y) == other
    assert agent.exhausted, agent.last_event


def test_a_map_without_labels_ends_on_the_frontier_rule_alone():
    agent, grid, here, rooms = _known_house_agent(lambda g, here: [])
    result = agent.step()
    assert agent.exhausted, result.detail
    assert result.action == "STOP"


def test_the_runner_ends_exhausted_once_every_labelled_room_is_entered():
    agent, grid, here, rooms = _known_house_agent(
        lambda g, here: [here, next(r for r in sorted(g.rooms) if r != here)])
    runner = MissionRunner(agent.robot, target_object="purple elephant", policy="explore",
                           navigator=agent.navigator, world=agent.world, max_steps=400,
                           clock=lambda: grid.sim_time, idle=agent.robot.pass_time)
    runner.agent.seen = agent.seen
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == EXHAUSTED, s["log_tail"]
    assert set(rooms) <= set(runner.agent.rooms_entered)
    assert s["report"]["rooms_entered"]


# ---------- E4: the report's coverage against truth ----------

def _truth_coverage(world_map, grid):
    """`tests/demo_explore.coverage()`, in process: the share of
    truth-reachable floor whose centre is known on the robot's map."""
    from tests.demo_explore import reachable_cells
    reach = reachable_cells(grid)
    m = world_map
    res, w, h = m["resolution_m"], m["width"], m["height"]
    known = 0
    for cx, cy in reach:
        i = int(((cx + 0.5) * 0.3 - m["origin_x_m"]) / res)
        j = int(((cy + 0.5) * 0.3 - m["origin_y_m"]) / res)
        if 0 <= i < w and 0 <= j < h and m["cells"][j * w + i] != -1:
            known += 1
    return known / len(reach)


@pytest.mark.parametrize("house", ["home_first_floor", "complex_house"])
def test_the_coverage_estimate_is_within_two_points_of_truth_at_the_end_of_a_search(house):
    """E4's bar (2 points of `demo_explore.coverage()`), met where the map
    is complete. Measured 2026-10-12: -1.5 and -0.1 points. NOT met part way
    through a search -- 60 s in, +20.7 and +24.6 points here, and +69.3 and
    +77.0 on the live ROS stack: the brain cannot count floor its map has not
    reached (recorded in 3.70 E4; the phone shows m² mapped instead)."""
    from brain.mission_report import map_coverage
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    world = MockWorld(grid)
    runner = MissionRunner(robot, target_object="purple elephant", policy="explore",
                           navigator=FakeNav(robot), world=world, mission_time_limit_s=600,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    s = _drive(runner, max_ticks=20000)
    assert s["outcome"] in (EXHAUSTED, OUT_OF_BUDGET), s["log_tail"]
    m = world.get_map()
    assert abs(map_coverage(m)["coverage"] - _truth_coverage(m, grid)) <= 0.02
    assert s["report"]["coverage"] == map_coverage(m)["coverage"]


def test_elapsed_stops_at_the_end():
    runner, robot, grid = _sim_runner(mission_time_limit_s=10.0)
    runner.start()
    _drive(runner)
    first = runner.status()["budget"]["elapsed_s"]
    robot.pass_time(30.0)
    assert runner.status()["budget"]["elapsed_s"] == first



# ---------- review fixes (2026-10-12) ----------

@pytest.mark.parametrize("message, outcome, by", [
    ("twin-dpad is driving -- brain is lower priority and was refused", CANCELLED, "dpad"),
    ("a nav2 goal is active -- one autonomous driver at a time, so brain was refused "
     "until it ends or is cancelled", CANCELLED, "dpad"),
    ("ros is driving -- one autonomous driver at a time, so brain was refused until "
     "it lapses", FAILED, None),
    ("brain is driving -- one autonomous driver at a time, so brain was refused until "
     "it lapses", FAILED, None),
])
def test_only_a_person_s_takeover_is_a_cancel(message, outcome, by):
    """Code review: every refusal for authority ended `cancelled` by the
    D-pad, including one that names another AUTONOMOUS holder -- an
    arbitration fault, which ends `failed` (red first: those two ended cancelled)."""
    from tests.conftest import fresh_mock_robot, mock_world_for
    inner = fresh_mock_robot()
    robot = _TakenOver(inner)
    robot.message = message
    runner = MissionRunner(robot, target_object="red backpack", world=mock_world_for(inner))
    runner.start()
    s = _drive(runner)
    assert (s["outcome"], s["cancelled_by"]) == (outcome, by), s["log_tail"]


class _SlowMapWorld(MockWorld):
    def __init__(self, grid, delay_s):
        super().__init__(grid)
        self.delay_s = delay_s

    def get_map(self):
        time.sleep(self.delay_s)
        return super().get_map()


def test_stop_never_waits_on_the_report_s_map_read():
    """Thermos: `_finish` built the report (a map read, up to the world's
    timeout) before returning, and the brain's loop calls `stop()` on its
    event loop. The stop now returns at once; the report lands after."""
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="purple elephant",
                           world=_SlowMapWorld(grid, 0.0))
    runner.start()
    for _ in range(3):
        runner.tick()
    runner.world.delay_s = 1.5
    t0 = time.monotonic()
    runner.stop()
    assert time.monotonic() - t0 < 0.5
    assert runner.status()["outcome"] == CANCELLED
    assert runner.report_done.wait(5)
    assert runner.status()["report"]["map_version"] is not None


def test_a_time_limit_must_be_finite_and_positive():
    from pydantic import ValidationError
    from control.brain_server import MissionStartRequest
    for bad in (float("inf"), 0, -5, 10 ** 9):
        with pytest.raises(ValidationError):
            MissionStartRequest(target_object="x", mission_time_limit_s=bad)
    assert MissionStartRequest(target_object="x", mission_time_limit_s=600).mission_time_limit_s == 600
    with pytest.raises(ValueError):
        MissionRunner(MockRobot(build_world("scaled_house"), render=False),
                      target_object="x", mission_time_limit_s=float("inf"))


class _RefusingRoomNav(FakeNav):
    """nav2 that aborts every goal into one room."""

    def __init__(self, robot, room):
        super().__init__(robot)
        self.room, self.room_goals = room, 0

    def set_goal(self, x_m, y_m):
        out = super().set_goal(x_m, y_m)
        if self.grid.room_at(int(x_m / 0.3), int(y_m / 0.3)) == self.room:
            self.room_goals += 1
            self.goal["state"] = "aborted"
        return out


def test_a_room_nav2_keeps_refusing_is_given_up_by_name():
    """Code review: a failed room goal was set aside only at its own cell,
    so the next try picked another cell of the same room, without end. It is
    given up after the retry limit, and the search ends Exhausted."""
    agent, grid, here, rooms = _known_house_agent(
        lambda g, here: [here, next(r for r in sorted(g.rooms) if r != here)])
    other = next(r for r in rooms if r != here)
    nav = _RefusingRoomNav(agent.robot, other)
    agent.navigator = nav
    for _ in range(200):
        agent.step()
        if agent.exhausted:
            break
        grid.advance_time(5.0)
    assert agent.exhausted, agent.last_event
    assert other not in agent.rooms_entered
    # This nav2 refuses without the robot moving: a wedge, whose first at
    # each place is forgiven (as for any goal), so each counted failure may
    # follow one forgiven one -- bounded, never cell by cell without end.
    assert agent.retry.limit <= nav.room_goals <= 2 * agent.retry.limit - 1


def test_a_room_goal_aims_inside_the_room_not_at_its_doorway():
    """Thermos: the nearest labelled cell is the room's edge at the door;
    nav2's tolerance could stop the robot outside the label. The goal sits
    ROOM_INSET_M inside where the room allows."""
    import math as _math
    from brain.explore import ROOM_INSET_M
    agent, grid, here, rooms = _known_house_agent(
        lambda g, here: [here, next(r for r in sorted(g.rooms) if r != here)])
    other = next(r for r in rooms if r != here)
    first = agent.step()
    assert first.action == "GOAL", first.detail
    gx, gy = agent._goal["x_m"], agent._goal["y_m"]
    m = agent.world.get_map()
    labels, w = m["room_labels"], m["width"]
    i, j = int(gx / m["resolution_m"]), int(gy / m["resolution_m"])
    k = labels["names"].index(other)
    n = max(1, _math.ceil(ROOM_INSET_M / m["resolution_m"]))
    assert all(labels["cells"][(j + dj) * w + i + di] == k
               for di in range(-n, n + 1) for dj in range(-n, n + 1))


# ---------- E4: a `vision` mission's cloud calls are capped ----------
# Raised by /code-review on the 1200 s limit (2026-10-12): under `vision`
# every step is a paid /navigate call, and a time limit is not a call limit.
# Before 3.70 the 120-step cap bounded both; the user chose a 120-call cap.

def _counting_scene():
    calls = []

    def scene(frame):
        calls.append(frame)
        return {"obstacles_ahead": [], "free_space": "clear", "doorway_visible": False,
                "important_objects": [], "safest_direction": "FORWARD"}
    return scene, calls


def test_a_vision_mission_ends_out_of_budget_at_its_cloud_call_cap():
    scene, calls = _counting_scene()
    runner, robot, grid = _sim_runner(policy="vision", vision_fn=scene,
                                      mission_time_limit_s=1200.0, vision_max_calls=5)
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == OUT_OF_BUDGET, s["log_tail"]
    assert len(calls) == 5, "the cap is the number of calls made, no more"
    assert "cloud call" in s["end_reason"]
    assert s["budget"]["cloud_calls"] == 5 and s["budget"]["cloud_call_limit"] == 5
    w = robot.get_wheel_state()
    assert w["left"]["velocity_rad_s"] == 0 and w["right"]["velocity_rad_s"] == 0


def test_a_failed_cloud_call_counts_against_the_cap():
    """A call that failed was still sent, and may still be billed."""
    def failing(frame):
        raise RuntimeError("503")
    runner, _, _ = _sim_runner(policy="vision", vision_fn=failing,
                               vision_max_calls=2, max_vision_failures=10)
    runner.start()
    s = _drive(runner)
    assert s["outcome"] == OUT_OF_BUDGET, s["log_tail"]
    assert s["budget"]["cloud_calls"] == 2


@pytest.mark.parametrize("policy", ["frontier", "tiered"])
def test_the_call_cap_is_for_the_vision_policy_only(policy):
    """frontier makes no paid call; tiered has its own cap (`tier_max_calls`)."""
    scene, calls = _counting_scene()
    runner, _, _ = _sim_runner(policy=policy, vision_fn=scene, max_steps=12,
                               vision_max_calls=3)
    runner.start()
    s = _drive(runner)
    assert "cloud call" not in (s["end_reason"] or ""), s["log_tail"]
    assert s["budget"]["cloud_call_limit"] is None


def test_a_vision_mission_counts_its_calls_with_the_cap_off():
    """Review (bug pass): with the cap off is when nothing bounds spend, so
    the count must still show."""
    scene, calls = _counting_scene()
    runner, _, _ = _sim_runner(policy="vision", vision_fn=scene, max_steps=4)
    runner.start()
    s = _drive(runner)
    assert s["budget"]["cloud_call_limit"] is None
    assert s["budget"]["cloud_calls"] == len(calls) == 4


@pytest.mark.parametrize("bad", [-1, 2.5])
def test_a_negative_or_fractional_call_cap_is_refused_at_load(tmp_path, bad):
    """Review (both): -1 read as 'unlimited' would end every vision mission
    before its first call, reported as a cap reached."""
    from control.brain_config import load_brain_config
    cfg = tmp_path / "robot.yaml"
    cfg.write_text(f"brain:\n  vision_max_calls: {bad}\n")
    with pytest.raises(ValueError, match="vision_max_calls"):
        load_brain_config(str(cfg))


def test_a_room_goal_s_first_wedge_is_forgiven_like_any_goal_s():
    """Code review (after the 3.65 rebase): a room goal that failed with the
    robot wedged and never moving counted against the room at once, where
    every other goal kind forgives the first wedge at a place. Wedged
    `retry.limit` times, a reachable room would be given up by name."""
    agent, grid, here, rooms = _known_house_agent(
        lambda g, here: [here, next(r for r in sorted(g.rooms) if r != here)])
    other = next(r for r in rooms if r != here)
    pose = agent._pose()
    goal = {"x_m": 5.0, "y_m": 5.0, "kind": "room", "room": other,
            "sent_at": grid.sim_time, "from": (pose["x_m"], pose["y_m"])}
    agent._goal = dict(goal)
    agent._goal_ended("aborted", grid.sim_time)
    assert agent._room_goals_failed.get(other, 0) == 0, "the first wedge counted"
    agent._pending = []
    agent._goal = dict(goal)
    agent._goal_ended("aborted", grid.sim_time)
    assert agent._room_goals_failed.get(other, 0) == 1, "wedged AGAIN must count"
