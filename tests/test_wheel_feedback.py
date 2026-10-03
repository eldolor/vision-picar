"""
tests/test_wheel_feedback.py

PLAN-ros-alignment.md 3.34: a body that cannot measure its wheels does not
drive them, and a move that fell short is not a move.

The body is the real backend, `HardwareRobot`, over the fake ESP32 on a
pseudo-terminal. "Silence" is the board still hearing commands but no longer
reporting (`feedback_continuous = False`, what `T:131 cmd 0` does on the real
firmware); "link lost" is the board's end of the line closing. Before 3.34
the body answered `usable` forever after its first frame, a verb closed on
frozen encoders drove on to its period cap, and a stop on a dead port raised.

Criteria 1-5 are here directly; 6 goes through the robot server and the
remote body; 7 through the agent and the mission runner.
"""

import time

import httpx
import pytest
from fastapi.testclient import TestClient

from brain.agent import ConstrainedAgent
from control.mission_runner import BLOCKED, MissionRunner
from control.remote_robot import RemoteRobot, RobotTransportError
from robot.hardware_robot import FEEDBACK_STALE_S, HardwareRobot
from robot.interface import RobotInterface, WheelFeedbackLost
from robot.safety import SafetyController
from sim.fake_esp32 import FakeEsp32
from sim.maps.scaled_house import build_scaled_world
from sim.mock_robot import MockRobot


def _wait(cond, timeout=2.0, step=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(step)
    return cond()


@pytest.fixture
def car():
    """The car with the simulated body as its sensors, so a verb is guarded
    by a real scan and clear ground ahead (the scaled house's start)."""
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path, sensors=body)
    assert _wait(lambda: robot.get_wheel_state()["usable"]), "no first frame"
    yield robot, board, body
    robot.close()
    board.close()


def _silence(board):
    """Stop the board reporting; return the time of its last frame."""
    board.feedback_continuous = False
    time.sleep(0.06)            # let a frame already being built go out
    return time.monotonic()


def test_the_feedback_window_sits_below_both_deadmen():
    # Above it the server's 1 s watchdog or the board's 1.5 s heartbeat would
    # act first, and this rule would never be the one that stops the wheels.
    assert 0.1 < FEEDBACK_STALE_S < 1.0


# ---- criterion 1: silence is unusable --------------------------------------

def test_silence_makes_wheels_and_odometry_unusable(car):
    robot, board, _ = car
    last = _silence(board)
    assert _wait(lambda: not robot.get_wheel_state()["usable"], timeout=1.0)
    assert time.monotonic() - last <= 0.35 + 0.05
    assert not robot.get_odometry()["usable"]


# ---- criterion 2: the host stops the wheels --------------------------------

def test_a_standing_command_is_zeroed_by_the_body_when_feedback_stops(car):
    robot, board, _ = car
    robot.set_wheel_velocity(3.0, 3.0)
    assert _wait(lambda: board.setpoint != [0.0, 0.0], timeout=0.5)
    last = _silence(board)
    assert _wait(lambda: board.setpoint == [0.0, 0.0], timeout=1.0), board.setpoint
    # The board's own heartbeat is 1.5 s; this is the host acting.
    assert time.monotonic() - last <= 0.4 + 0.05
    assert not board.heartbeat_stopped


# ---- criterion 3: motion refused while stale; a stop never is -------------

def test_motion_is_refused_and_zero_is_accepted_while_stale(car):
    robot, board, _ = car
    _silence(board)
    assert _wait(lambda: not robot.get_wheel_state()["usable"], timeout=1.0)
    with pytest.raises(WheelFeedbackLost):
        robot.set_wheel_velocity(3.0, 3.0)
    time.sleep(0.1)
    assert board.setpoint == [0.0, 0.0]
    robot.set_wheel_velocity(0.0, 0.0)
    robot.stop()


def test_stop_does_not_raise_after_the_link_is_gone():
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path, sensors=body)
    try:
        assert _wait(lambda: robot.get_wheel_state()["usable"])
        board.close()
        time.sleep(0.1)
        robot.stop()                                  # used to raise OSError
        robot.set_wheel_velocity(0.0, 0.0)
        with pytest.raises(WheelFeedbackLost):
            robot.set_wheel_velocity(3.0, 3.0)
        assert _wait(lambda: not robot.get_wheel_state()["usable"], timeout=1.0)
        assert robot.feedback_status()["fresh"] is False
    finally:
        robot.close()


# ---- criterion 4: a verb in progress ends promptly -------------------------

def test_a_verb_whose_feedback_stops_ends_promptly(car):
    robot, board, _ = car
    safety = SafetyController(robot)
    plan = robot.verb_plan("FORWARD", speed=30, duration=3.0)   # ~2 moves, ~3.3 s
    assert plan is not None
    import threading
    out = {}
    t = threading.Thread(target=lambda: out.update(o=safety.run_verb(plan)))
    t.start()
    time.sleep(0.3)
    last = _silence(board)
    t.join(timeout=5.0)
    assert not t.is_alive()
    ended_at = time.monotonic()
    assert out["o"]["ended"] == "no_feedback", out["o"]
    assert ended_at - last <= 0.5 + 0.1
    time.sleep(0.1)
    assert board.setpoint == [0.0, 0.0]


def test_check_and_execute_raises_when_feedback_stops_mid_verb(car):
    robot, board, _ = car
    safety = SafetyController(robot)
    import threading
    err = {}

    def go():
        try:
            safety.check_and_execute("FORWARD", speed=30, duration=3.0)
        except WheelFeedbackLost as e:
            err["e"] = e

    t = threading.Thread(target=go)
    t.start()
    time.sleep(0.3)
    _silence(board)
    t.join(timeout=5.0)
    assert "e" in err


def test_a_verb_is_refused_up_front_while_stale(car):
    robot, board, _ = car
    _silence(board)
    assert _wait(lambda: not robot.get_wheel_state()["usable"], timeout=1.0)
    with pytest.raises(WheelFeedbackLost):
        SafetyController(robot).check_and_execute("FORWARD", speed=50, duration=0.5)
    # A turn too: the encoders close every verb.
    with pytest.raises(WheelFeedbackLost):
        SafetyController(robot).check_and_execute("LEFT", angle=45)


# ---- criterion 5: recovery is clean ----------------------------------------

def test_readings_and_motion_return_when_frames_resume(car):
    robot, board, _ = car
    before = robot.get_odometry()
    _silence(board)
    assert _wait(lambda: not robot.get_wheel_state()["usable"], timeout=1.0)
    board.feedback_continuous = True
    assert _wait(lambda: robot.get_wheel_state()["usable"], timeout=1.0)
    after = robot.get_odometry()
    assert after["usable"]
    # Standing still across the gap: no more than one odometer unit (1 cm).
    assert abs(after["distance_m"] - before["distance_m"]) <= 0.01
    robot.set_wheel_velocity(1.0, 1.0)
    robot.stop()


# ---- criterion 6: the robot server names it --------------------------------

@pytest.fixture
def car_server(monkeypatch):
    monkeypatch.setenv("ROBOT_MODE", "hardware")
    monkeypatch.setenv("SIM_MOTOR_BOARD", "fake")
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    monkeypatch.delenv("ROBOT_DRIVE", raising=False)
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    import robot.server as server
    built = {}
    real = server.get_robot
    monkeypatch.setattr(server, "get_robot", lambda *a: built.setdefault("r", real(*a)))
    app = server.create_app()
    robot = built["r"]
    with TestClient(app) as client:
        assert _wait(lambda: robot.get_wheel_state()["usable"]), "no first frame"
        yield client, robot
    robot.close()
    robot.fake_board.close()


def _board_of(robot):
    # The factory keeps the fake board beside the robot it built.
    return robot.fake_board


def test_server_answers_no_feedback_and_describes_the_link(car_server):
    client, robot = car_server
    health = client.get("/health").json()
    assert health["motor_board"]["fresh"] is True
    assert health["motor_board"]["frames"] > 0
    _silence(_board_of(robot))
    assert _wait(lambda: not robot.get_wheel_state()["usable"], timeout=1.0)
    r = client.post("/action", json={"action": "FORWARD"}, headers={"x-driver": "brain"})
    assert r.status_code == 200, r.text
    assert r.json()["executed"] is False
    assert r.json()["reason"] == "no_feedback"
    r = client.post("/wheels", json={"left_rad_s": 2.0, "right_rad_s": 2.0},
                    headers={"x-driver": "twin-dpad"})
    assert r.status_code == 200, r.text
    assert r.json()["reason"] == "no_feedback"
    assert client.post("/stop").status_code == 200
    health = client.get("/health").json()
    assert health["motor_board"]["fresh"] is False
    assert health["motor_board"]["age_s"] >= FEEDBACK_STALE_S
    # Description, never a verdict (M5): the server is still "ok".
    assert health["status"] == "ok"


def test_remote_body_raises_a_transport_error_on_no_feedback(car_server):
    client, robot = car_server
    remote = RemoteRobot("http://testserver", client=client)
    _silence(_board_of(robot))
    assert _wait(lambda: not robot.get_wheel_state()["usable"], timeout=1.0)
    with pytest.raises(RobotTransportError, match="no_feedback"):
        remote.drive_forward()


# ---- criterion 7: a short move is not a move --------------------------------

class _StallingBody(RobotInterface):
    """A body whose every FORWARD ends `timeout` short of its target, as a
    wheel snagged on a rug does on the car; turns complete."""

    def __init__(self, ended="timeout"):
        self.ended = ended
        self.forwards = 0

    def drive_forward(self, speed=50, duration=0.5):
        self.forwards += 1
        return {"action": "drive_forward", "requested": 1, "moved": 0.02,
                "stopped_short": self.ended, "reason": None}

    def reverse(self, speed=50, duration=0.5):
        return {"action": "reverse"}

    def turn_left(self, angle=90):
        return {"turned_deg": -angle}

    def turn_right(self, angle=90):
        return {"turned_deg": angle}

    def stop(self):
        return {"action": "stop"}

    def look_left(self):
        return {"pan": -90}

    def look_right(self):
        return {"pan": 90}

    def look_center(self):
        return {"pan": 0}

    def get_camera_frame(self):
        return {"image": b"", "media_type": "image/jpeg", "metadata": {}}

    def get_distance(self):
        return 200.0


def _forward_only(scene, frame):
    return "FORWARD"


@pytest.mark.parametrize("ended", ["timeout", "stalled"])
def test_a_stalled_forward_is_not_executed(ended):
    body = _StallingBody(ended)
    agent = ConstrainedAgent(body, vision_fn=lambda f: {})
    agent.decide = _forward_only
    step = agent.step()
    assert step.action == "FORWARD"
    assert step.executed is False
    assert ended in str(step.detail)


def test_a_clamped_forward_stays_executed():
    body = _StallingBody("clamped")
    agent = ConstrainedAgent(body, vision_fn=lambda f: {})
    agent.decide = _forward_only
    assert agent.step().executed is True


def test_five_stalled_forwards_end_a_mission_blocked():
    body = _StallingBody("timeout")
    runner = MissionRunner(
        body, target_object="red backpack", policy="vision", max_steps=20,
        vision_fn=lambda frame: {"safest_direction": "FORWARD", "objects": [],
                                 "target_visible": False})
    runner.start()
    while runner.tick():
        pass
    status = runner.status()
    assert status["outcome"] == BLOCKED, status
    assert body.forwards == 5
