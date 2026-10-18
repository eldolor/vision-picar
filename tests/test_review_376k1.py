"""
tests/test_review_376k1.py

`PLAN-ros-alignment.md` 3.76, step K1: the bugs the review of the robot
server and safety layer as other sessions changed them after batches A-B
(3.70 E, 3.74 B, 3.82, 3.84, 3.87.A) confirmed. Each test was written
before its fix and seen to fail on `dev` for the reason it names.
"""

import threading
import time

import pytest

import robot.server as server
from robot import interface
from robot.safety import PATH_HALF_ANGLE_DEG, SafetyController
from tests.conftest import asgi_client
from tests.test_floor_hazards import _robot
from tests.test_refused_goal_hold import _server_over
from tests.test_review_376a import _keep_robot


@pytest.fixture
def plain(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = asgi_client(server.create_app())
    yield client
    client.close()


# ---------- K1-1: a person's STOP through /action latches autonomy ----------


@pytest.mark.parametrize("who", [{"x-driver": "twin-dpad"}, {}])
def test_a_persons_stop_through_action_latches_autonomy(plain, who):
    """Only the /stop route latched (3.84): a person's, a script's or curl's
    `/action STOP` zeroed the wheels and the brain's next verb drove on."""
    client = plain
    assert client.post("/action", json={"action": "LOOK_LEFT"},
                       headers={"x-driver": "brain"}).json()["executed"]
    client.post("/action", json={"action": "STOP"}, headers=who)
    time.sleep(1.2)                                  # past the claim's lapse
    r = client.post("/action", json={"action": "LOOK_RIGHT"},
                    headers={"x-driver": "brain"}).json()
    assert r["executed"] is False and r["reason"] == "person_stop", r


def test_a_persons_stop_through_action_ends_the_nav2_goal(monkeypatch):
    client, bridge = _server_over(monkeypatch, None)
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]
    assert bridge.goal and bridge.goal["state"] == "active"
    client.post("/action", json={"action": "STOP"}, headers={"x-driver": "twin-dpad"})
    def active():
        return bool(bridge.goal) and bridge.goal.get("state") == "active"
    deadline = time.monotonic() + 5.0
    while active() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not active(), "nav2 drove on after a person's STOP"


def test_the_brains_stop_through_action_still_latches_nothing(plain):
    client = plain
    client.post("/action", json={"action": "STOP"}, headers={"x-driver": "brain"})
    r = client.post("/action", json={"action": "LOOK_RIGHT"},
                    headers={"x-driver": "brain"}).json()
    assert r["executed"], r


# ---------- K1-2: a brain goal inside a person's Stop ----------


def test_a_brain_goal_landing_between_the_latch_and_the_stop_count_is_refused(monkeypatch):
    """/stop sets the latch, then robot.stop() (a serial write on the real
    board), then counts the stop. A brain goal that passed the route's first
    latch check before the Stop and took the goal lock in that window saw the
    latch but no new stop count, went to nav2 and was answered `accepted`."""
    built = _keep_robot(monkeypatch)
    client, bridge = _server_over(monkeypatch, None)
    robot = built["robot"]
    bridge.post_gate = threading.Event()
    answers = {}
    script = threading.Thread(target=lambda: answers.setdefault("script", client.post(
        "/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()))
    script.start()                                   # holds the goal lock in the bridge
    assert bridge.post_entered.wait(5)
    brain = threading.Thread(target=lambda: answers.setdefault("brain", client.post(
        "/world/goal", json={"x_m": 2.5, "y_m": 1.0}, headers={"x-driver": "brain"}).json()))
    brain.start()
    time.sleep(0.3)                                  # past its first latch check, at the lock
    real = robot.stop

    def slow_stop():
        bridge.post_gate.set()                       # the script's goal lands...
        script.join(5)
        brain.join(5)                                # ...and the brain's takes the lock
        return real()

    monkeypatch.setattr(robot, "stop", slow_stop)
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    r = answers["brain"]
    assert r["accepted"] is False and r["reason"] == "person_stop", r
    assert len(bridge.posted) == 1, f"the brain's goal reached nav2 after a person's Stop: {bridge.posted}"


# ---------- K1-3: the floor band never raises out of the safety check ----------


def _band(**over):
    band = {"fov_deg": 73.0, "pan_deg": 0.0, "origin_x_cm": 6.5,
            "zones": [{"status": interface.ZONE_RANGE, "distance_cm": 40.0, "bearing_deg": 0.0}]}
    band.update(over)
    return {"floor": band}


@pytest.mark.parametrize("bad", [
    {"pan_deg": "x"}, {"origin_x_cm": [1]},
    {"zones": [{"status": interface.ZONE_RANGE, "distance_cm": "far", "bearing_deg": 0.0}]},
    {"zones": [{"status": interface.ZONE_RANGE, "distance_cm": 40.0, "bearing_deg": "left"}]},
])
def test_a_malformed_band_is_unusable_never_an_exception(bad):
    """Only fov_deg was parsed safely: any other malformed field raised out
    of forward_clearance -- a 400 on every FORWARD, and in the wheel loop a
    skipped re-vet, so a standing command ran un-vetted."""
    safety = SafetyController(_robot(None), 20.0)
    cm, source = safety.floor_clearance(_band(**bad))
    assert cm is None and source == "floor_band_unusable", (cm, source)


def test_a_band_whose_zones_disagree_with_its_cols_is_unusable():
    """Zone bearings came from len(zones), not the published cols: one zone
    dropped put every bearing-less zone at the wrong angle."""
    safety = SafetyController(_robot(None), 20.0)
    zones = [{"status": interface.ZONE_RANGE, "distance_cm": 40.0, "bearing_deg": None}] * 7
    assert safety.floor_clearance(_band(cols=8, zones=zones)) == (None, "floor_band_unusable")


def test_a_band_that_sees_nothing_says_so():
    """An all-unusable band read `floor_band_no_target`, as a clear corridor."""
    safety = SafetyController(_robot(None), 20.0)
    zones = [{"status": interface.ZONE_UNUSABLE, "distance_cm": None}] * 32
    assert safety.floor_clearance(_band(zones=zones)) == (None, "floor_band_unusable")


def test_a_clear_band_still_reads_clear():
    safety = SafetyController(_robot(None), 20.0)
    zones = [{"status": interface.ZONE_RANGE, "distance_cm": 300.0, "bearing_deg": 30.0}]
    assert safety.floor_clearance(_band(zones=zones)) == (None, "floor_band_no_target")


# ---------- K1-4: a battery reading no pack gives ----------


@pytest.mark.parametrize("volts", [0.0, 5.0])
def test_a_reading_no_pack_gives_is_no_reading(volts):
    """A board on USB on the bench (about 5 V) or with a failed INA219 read
    (0) was a usable reading below the 10.5 V floor: every mission ended
    `out_of_budget` after two readings."""
    from robot.hardware_robot import HardwareRobot
    from sim.fake_esp32 import FakeEsp32
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    body = MockRobot(build_world("scaled_house"), render=False)
    board = FakeEsp32(body, battery_v=volts)
    robot = HardwareRobot(board.path)
    try:
        deadline = time.time() + 2.0
        while robot.frames < 3 and time.time() < deadline:
            time.sleep(0.02)
        assert robot.frames >= 3, "the fake board sent no frames -- the test proves nothing"
        assert robot.get_battery()["usable"] is False, robot.get_battery()
    finally:
        robot.close()
        board.close()


# ---------- K1-5: refused on arrival under the latch, the brain lets go ----------


def test_a_brain_refused_on_arrival_under_a_persons_stop_lets_go(plain):
    """Only a brain command refused at execution let go of the robot; one
    refused on arrival kept the claim from before the Stop for the watchdog's
    timeout, refusing a person's goal's ros wheel posts meanwhile."""
    client = plain
    assert client.post("/action", json={"action": "LOOK_LEFT"},
                       headers={"x-driver": "brain"}).json()["executed"]
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    r = client.post("/action", json={"action": "LOOK_RIGHT"},
                    headers={"x-driver": "brain"}).json()
    assert r["reason"] == "person_stop", r
    assert client.get("/health").json()["authority_holder"] is None, "the refused brain kept the robot"
    r = client.post("/wheels", json={"left_rad_s": 1.0, "right_rad_s": 1.0},
                    headers={"x-driver": "ros"}).json()
    assert r.get("reason") != "preempted", r



# ---------- the fixes' review, round 2 ----------


def test_a_band_that_sees_nothing_in_range_is_clear_not_blind():
    """Only range zones counted as seen: an open-floor band (every zone
    no_target) read `floor_band_unusable`."""
    safety = SafetyController(_robot(None), 20.0)
    zones = [{"status": interface.ZONE_NO_TARGET, "distance_cm": None}] * 32
    assert safety.floor_clearance(_band(zones=zones)) == (None, "floor_band_no_target")


@pytest.mark.parametrize("grid", [
    {"floor": ["not", "a", "band"]},
    _band(origin_x_cm=float("nan")),
    _band(cols=float("inf")),
])
def test_bands_the_first_fix_still_misread(grid):
    """A non-dict band and `cols: Infinity` still raised; a NaN origin read
    as a clear corridor."""
    safety = SafetyController(_robot(None), 20.0)
    assert safety.floor_clearance(grid) == (None, "floor_band_unusable")


def test_a_persons_goal_tap_inside_a_stop_is_stale_not_accepted(monkeypatch):
    """The Stop counted itself after robot.stop(): a tap that took the goal
    lock in between was answered accepted, then cancelled by the Stop's loop
    -- the twin showed a goal that silently died."""
    built = _keep_robot(monkeypatch)
    client, bridge = _server_over(monkeypatch, None)
    robot = built["robot"]
    bridge.post_gate = threading.Event()
    answers = {}
    script = threading.Thread(target=lambda: answers.setdefault("script", client.post(
        "/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()))
    script.start()
    assert bridge.post_entered.wait(5)
    tap = threading.Thread(target=lambda: answers.setdefault("tap", client.post(
        "/world/goal", json={"x_m": 2.5, "y_m": 1.0}, headers={"x-driver": "twin-dpad"}).json()))
    tap.start()
    time.sleep(0.3)
    real = robot.stop

    def slow_stop():
        bridge.post_gate.set()
        script.join(5)
        tap.join(5)
        return real()

    monkeypatch.setattr(robot, "stop", slow_stop)
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    assert answers["tap"]["accepted"] is False and answers["tap"]["reason"] == "stopped", answers
    assert len(bridge.posted) == 1, bridge.posted


def test_a_stop_whose_robot_stop_raises_still_ends_the_goal(monkeypatch):
    """The goal-ending loop started only after robot.stop() returned: a
    serial error there left nav2 driving under a latched Stop."""
    built = _keep_robot(monkeypatch)
    client, bridge = _server_over(monkeypatch, None)
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]

    def broken():
        raise OSError("serial write failed")

    monkeypatch.setattr(built["robot"], "stop", broken)
    with pytest.raises(OSError):
        client.post("/stop", headers={"x-driver": "twin-dpad"})
    deadline = time.monotonic() + 5.0
    while bridge.goal and bridge.goal.get("state") == "active" and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not (bridge.goal and bridge.goal.get("state") == "active"), "nav2 drove on"


def test_the_brains_stop_through_action_is_never_refused(monkeypatch):
    """"Stop is everyone's right": the brain's /action STOP was arbitrated,
    so refused while a nav2 goal was in progress."""
    client, bridge = _server_over(monkeypatch, None)
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()["accepted"]
    r = client.post("/action", json={"action": "STOP"}, headers={"x-driver": "brain"}).json()
    assert r["executed"] is True, r


# ---------- the fixes' review, round 3 ----------


def test_a_band_blind_ahead_with_open_floor_beside_is_unusable():
    """Round 2 counted any no_target zone as seen: a band blind across the
    corridor (a sock inside its minimum range) but open to the sides read
    clear -- the "blind is not clear" case again."""
    safety = SafetyController(_robot(None), 20.0)
    n = 32
    zones = []
    for i in range(n):
        centre = -36.5 + (i + 0.5) * 73.0 / n
        ahead = abs(centre) <= PATH_HALF_ANGLE_DEG + 0.5
        zones.append({"status": interface.ZONE_UNUSABLE if ahead else interface.ZONE_NO_TARGET,
                      "distance_cm": None})
    assert safety.floor_clearance(_band(zones=zones)) == (None, "floor_band_unusable")


@pytest.mark.parametrize("bad", [{"fov_deg": 10 ** 400},
                                 {"zones": [{"status": interface.ZONE_RANGE, "distance_cm": 10 ** 400}]}])
def test_huge_numbers_never_raise(bad):
    safety = SafetyController(_robot(None), 20.0)
    cm, source = safety.floor_clearance(_band(**bad))
    assert cm is None, (cm, source)


def test_a_persons_stop_lets_the_brain_go_even_if_it_sends_nothing(plain):
    """The claim lapsed only when the brain's next command was refused: a
    brain stuck on a cloud call kept the slot for the watchdog's timeout."""
    client = plain
    assert client.post("/action", json={"action": "LOOK_LEFT"},
                       headers={"x-driver": "brain"}).json()["executed"]
    client.post("/stop", headers={"x-driver": "twin-dpad"})
    assert client.get("/health").json()["authority_holder"] is None
