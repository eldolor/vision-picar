"""
tests/test_wheels_command.py

Phase R2b (`PLAN-ros-alignment.md` 3.10) -- `POST /wheels`, a standing
wheel-velocity command: the first way to move the robot without a verb, and
what R4's `picar_sim_hardware.write()` will send. One test per acceptance
criterion, against a real app with its control loop and watchdog running on
real time (TestClient as a context manager runs the lifespan).
"""

import time

import pytest
from fastapi.testclient import TestClient

from robot.server import create_app
from sim.mock_robot import WHEEL_RADIUS_M

# 0.1 m/s forward, as wheel rad/s. Slow enough that one 50ms control period
# is half a centimetre of travel.
FWD = 0.1 / WHEEL_RADIUS_M


@pytest.fixture
def client():
    with TestClient(create_app()) as c:
        yield c


def _clearance(c):
    return c.get("/depth").json()["path"]["clearance_cm"]


def _wheels(c, left, right, driver="ros"):
    return c.post("/wheels", json={"left_rad_s": left, "right_rad_s": right},
                  headers={"x-driver": driver}).json()


def _hold(c, left, right, seconds, driver="ros"):
    """Re-send a wheel command every 0.3s, as a controller publishing at a
    rate would, so the 1.0s watchdog never lapses it. Returns the last reply."""
    end, reply = time.monotonic() + seconds, None
    while time.monotonic() < end:
        reply = _wheels(c, left, right, driver)
        time.sleep(0.3)
    return reply


def _act(c, action, driver="twin-dpad", **kw):
    return c.post("/action", json={"action": action, **kw},
                  headers={"x-driver": driver}).json()


def test_criterion_1_a_standing_forward_command_stops_short_of_the_wall(client):
    # From the start (2.5, 2.5) face north: the wall's face is 1.5 cells away.
    _act(client, "LEFT", angle=90)
    time.sleep(1.2)  # let the D-pad's authority lapse before ROS drives
    start = _clearance(client)
    assert start >= 20, start
    reply = _wheels(client, FWD, FWD)
    assert reply["executed"] is True and reply["clamped"] is None
    # 3s at 0.1 m/s is 30cm of travel -- past the wall, never mind the
    # collar -- so only the clamp can explain stopping short of it.
    _hold(client, FWD, FWD, 3.0)
    end = _clearance(client)
    assert end < start, "the command must actually move the robot"
    # Stops within one control period's travel (0.5cm) of the threshold.
    assert end >= 20 - 0.5 - 0.1, f"drove to {end}cm, under the 20cm collar"
    health = client.get("/health").json()
    assert health["last_refusal"]["reason"] in ("safety_distance", "watchdog")


def test_criterion_2_a_pivot_against_the_wall_still_turns(client):
    _act(client, "LEFT", angle=90)
    time.sleep(1.2)
    last = _hold(client, FWD, FWD, 3.0)
    assert last["clamped"], "not yet against the wall"
    before = client.get("/world/pose").json()["heading_deg"]
    reply = _wheels(client, -FWD * 3, FWD * 3)   # pure rotation, to the left
    assert reply["clamped"] is None, "rotation is never clamped"
    time.sleep(0.3)
    after = client.get("/world/pose").json()["heading_deg"]
    assert after != pytest.approx(before, abs=1.0), "the pivot did not turn"


def test_criterion_3_reverse_is_checked_against_the_rear_beams(client):
    """Facing east from the start, the west wall is 1.5 cells behind: rear
    clearance 45cm - 15cm to the bumper = 30cm. One REVERSE is allowed; the
    next would put the bumper into the wall and is refused -- on /action, and
    a negative /wheels velocity is clamped for the same reason."""
    assert _act(client, "REVERSE")["executed"] is True
    second = _act(client, "REVERSE")
    assert second["executed"] is False and second["reason"] == "safety_distance"
    assert "rear" in second["detail"]
    time.sleep(1.2)
    reply = _wheels(client, -FWD, -FWD)
    assert reply["clamped"] and "reverse" in reply["clamped"]


def test_criterion_4_silence_zeroes_a_standing_command(client):
    # Face into the open hallway so nothing clamps it first.
    _act(client, "FORWARD")
    _act(client, "FORWARD")
    time.sleep(1.2)
    _wheels(client, FWD, FWD)
    assert client.get("/wheels").json()["left"]["velocity_rad_s"] > 0
    time.sleep(1.6)  # past the 1.0s watchdog
    w = client.get("/wheels").json()
    assert w["left"]["velocity_rad_s"] == 0 and w["right"]["velocity_rad_s"] == 0


def test_criterion_5_authority(client):
    """The D-pad preempts ROS; brain and ROS exclude each other; two D-pad
    taps never fight."""
    assert _act(client, "LEFT", angle=15)["executed"] is True
    assert _wheels(client, 1.0, 1.0)["reason"] == "preempted", "a person outranks ROS"
    assert _act(client, "RIGHT", angle=15)["executed"] is True, "taps must not fight"

    time.sleep(1.2)
    assert _act(client, "LEFT", driver="brain", angle=15)["executed"] is True
    refused = _wheels(client, 1.0, 1.0)
    assert refused["reason"] == "preempted" and "one autonomous driver" in refused["detail"]

    time.sleep(1.2)
    assert _wheels(client, 0.5, -0.5)["executed"] is True
    brain = _act(client, "LEFT", driver="brain", angle=15)
    assert brain["reason"] == "preempted", "and the other way round"


def test_criterion_6_a_backend_with_no_motors_refuses(monkeypatch):
    monkeypatch.setenv("ROBOT_MODE", "teleop")
    monkeypatch.setenv("WORLD_MODE", "none")
    with TestClient(create_app()) as c:
        reply = _wheels(c, 1.0, 1.0)
    assert reply["executed"] is False and reply["reason"] == "unsupported"
