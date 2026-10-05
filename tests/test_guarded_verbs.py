"""
tests/test_guarded_verbs.py

`PLAN-ros-alignment.md` 3.22 -- guarded verbs (`PLAN-guarded-verbs.md`).
One test per criterion, written with its threshold before the fix, and
confirmed red against the code that let a direct-mode verb drive blind.

Criteria 1, 2 and 4 are judged on GROUND TRUTH by `tests/verb_sweep.py`
(the chassis rectangle against the house's occupied cells, sampled after
every sim sub-step); this pins a seeded sample of that sweep, and
`python -m tests.demo_verb_sweep` runs the full one. Criterion 3 runs the
hardware backend over the fake ESP32 on a real pseudo-terminal, on the wall
clock, because that is the only backend whose verbs take real time.
"""

import logging
import threading
import time

import pytest
from fastapi.testclient import TestClient

from robot.hardware_robot import HardwareRobot
from robot.safety import SafetyController
from sim.fake_esp32 import FakeEsp32
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests import verb_sweep as vs


@pytest.fixture(scope="module")
def sweeps():
    logging.disable(logging.WARNING)    # every refused verb logs a line
    try:
        return vs.straight_sweep(), vs.turn_sweep()
    finally:
        logging.disable(logging.NOTSET)


@pytest.fixture(scope="module")
def verdicts(sweeps):
    return vs.verdicts(*sweeps)


def test_criterion_1_a_move_stops_at_the_line(verdicts):
    ok, detail = verdicts["1 moves stop at the line"]
    assert ok, detail


def test_criterion_1b_a_cut_short_move_stops_at_the_line_not_past_it(verdicts):
    ok, detail = verdicts["1b a cut-short move stops AT the line"]
    assert ok, detail


def test_criterion_2_a_turn_does_not_touch(verdicts):
    ok, detail = verdicts["2 turns do not touch"]
    assert ok, detail


def test_criterion_4a_a_clear_move_is_still_a_whole_cell(verdicts):
    ok, detail = verdicts["4a a clear move is a whole cell"]
    assert ok, detail


def test_criterion_4b_a_clear_turn_is_still_the_whole_angle(verdicts):
    ok, detail = verdicts["4b a clear turn is the whole angle"]
    assert ok, detail


# ---------- criterion 3: a stop ends a verb ----------

@pytest.fixture
def hardware():
    """The hardware backend over the fake board, in the scaled house's
    living room facing a clear run east -- a verb there has room to be
    interrupted rather than refused."""
    body = MockRobot(build_world("scaled_house"), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path, sensors=body)
    time.sleep(0.2)                      # the board reads the host's set-up
    yield robot, board, body
    robot.close()
    board.close()


def test_criterion_3_a_stop_mid_verb_zeroes_the_wheels_and_they_stay_zero(hardware):
    robot, board, _body = hardware
    safety = SafetyController(robot, 20.0)
    verb = threading.Thread(target=lambda: safety.check_and_execute(
        "FORWARD", speed=50, duration=1.0))           # one cell at 0.3 m/s: ~1 s
    verb.start()
    time.sleep(0.3)
    assert board.setpoint != [0.0, 0.0], "the verb never started"
    robot.stop()
    t_stop = time.time()
    late = []
    while verb.is_alive() or time.time() - t_stop < 0.3:
        dt = time.time() - t_stop
        if dt > 0.1 and board.setpoint != [0.0, 0.0]:
            late.append((round(dt, 3), list(board.setpoint)))
        time.sleep(0.01)
    verb.join(timeout=5)
    assert not late, f"the wheels were driven again after the stop: {late[:3]}"


@pytest.fixture
def hardware_server(monkeypatch, sim_programs):
    """The robot server in `mode: hardware` over the fake board -- the real
    route a /stop takes, which is not the same as calling `stop()`."""
    monkeypatch.setenv("ROBOT_MODE", "hardware")
    # 3.36: the fake board's body runs as the split simulator's programs.
    programs = sim_programs(SIM_MAP="scaled_house").apply(monkeypatch)
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    monkeypatch.setenv("WORLD_MODE", "none")
    monkeypatch.delenv("ROBOT_DRIVE", raising=False)
    import robot.server as server
    with TestClient(server.create_app()) as c:
        time.sleep(0.3)
        yield c


def test_criterion_3_a_stop_over_http_ends_a_hardware_verb(hardware_server):
    """Found while building 3.22: `/action` held `motion_lock` for the whole
    verb, and the wheel loop takes that lock ON THE EVENT LOOP -- so during a
    hardware verb the server could not even receive a /stop. Two cells at
    0.3 m/s is ~2 s; a /stop 0.5 s in must end it there."""
    c = hardware_server
    d0 = c.get("/odometry").json()["distance_m"]
    verb = threading.Thread(target=lambda: c.post(
        "/action", json={"action": "FORWARD", "speed": 50, "duration": 2.0}))
    verb.start()
    time.sleep(0.5)
    c.post("/stop")
    time.sleep(0.15)                         # the board's feedback, and one period
    at_stop = c.get("/odometry").json()["distance_m"] - d0
    time.sleep(0.5)
    later = c.get("/odometry").json()["distance_m"] - d0
    verb.join(timeout=5)
    assert at_stop < 0.35, f"the verb was not interrupted: {at_stop:.3f} m at the stop"
    assert later - at_stop < 0.01, f"it kept moving after the stop: {at_stop:.3f} -> {later:.3f} m"


def test_criterion_3_the_watchdog_does_not_cut_a_verb_it_outlasts(monkeypatch, sim_programs):
    """A verb longer than `watchdog_timeout_s` (1 s) is a busy brain, not a
    silent one. Two cells at 0.3 m/s is ~2 s; it must cover them."""
    monkeypatch.setenv("ROBOT_MODE", "hardware")
    # 3.36: the fake board's body runs as the split simulator's programs.
    programs = sim_programs(SIM_MAP="scaled_house").apply(monkeypatch)
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    monkeypatch.setenv("WORLD_MODE", "none")
    monkeypatch.delenv("ROBOT_DRIVE", raising=False)
    import robot.server as server
    with TestClient(server.create_app()) as c:
        time.sleep(0.3)
        d0 = c.get("/odometry").json()["distance_m"]
        r = c.post("/action", json={"action": "FORWARD", "speed": 50, "duration": 2.0}).json()
        time.sleep(0.3)
        moved = c.get("/odometry").json()["distance_m"] - d0
        assert r["executed"], r
        assert moved == pytest.approx(0.60, abs=0.03), moved
