"""
tests/test_goal_arbitration.py

PLAN-ros-alignment.md 3.21: a nav2 goal is an AUTONOMOUS driver.

Before this, `POST /world/goal` called no `arbitrate()`, and nav2 drives on
`cmd_vel/nav` at the same twist_mux priority as `cmd_vel/brain` -- so a goal
sent during a brain mission was refused nowhere and ordered by nothing,
against R2b's "one autonomous driver at a time". In-process, against the fake
bridge tests/test_ros_goals.py uses; the numbers are the criteria in 3.21.
"""

import time

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from tests.test_ros_goals import world_with
from tests.test_ros_world import FakeTruth

# LEFT is never vetoed by the distance check, so "passes" means exactly
# "was not refused by arbitration".
TURN = {"action": "LEFT"}


@pytest.fixture
def served(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    client = TestClient(server.create_app())

    def act(driver=None, body=TURN):
        headers = {"x-driver": driver} if driver else {}
        return client.post("/action", json=body, headers=headers).json()

    def goal():
        return client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).json()

    def heading():
        return client.get("/odometry").json()["heading_deg"]

    return client, bridge, act, goal, heading


# 1 ---------------------------------------------------------------------

def test_a_goal_is_refused_while_the_brain_holds_the_robot(served):
    _, bridge, act, goal, _ = served
    assert act("brain")["executed"] is True
    r = goal()
    assert r["accepted"] is False and r["reason"] == "preempted", r
    assert bridge.goal is None, "the refused goal still reached nav2"


# 2 ---------------------------------------------------------------------

@pytest.mark.parametrize("driver", ["brain", "teleop"])
def test_an_autonomous_driver_is_refused_while_a_goal_is_active(served, driver):
    _, bridge, act, goal, heading = served
    assert goal()["accepted"] is True and bridge.goal["state"] == "active"
    before = heading()
    r = act(driver, {"action": "FORWARD"})
    assert r["executed"] is False and r["reason"] == "preempted", r
    assert "goal" in r["detail"], r["detail"]
    assert act(driver)["executed"] is False
    assert heading() == before, "the robot moved for a refused driver"


# 3 ---------------------------------------------------------------------

@pytest.mark.parametrize("driver", ["twin-dpad", None])
def test_a_person_is_never_refused_because_of_a_goal(served, driver):
    client, _, act, goal, _ = served
    assert goal()["accepted"] is True
    assert act(driver)["executed"] is True
    assert client.post("/stop").status_code == 200


# 4 ---------------------------------------------------------------------

@pytest.mark.parametrize("ended", ["succeeded", "aborted", "canceled"])
def test_a_goal_that_has_ended_blocks_nothing(served, ended):
    _, bridge, act, goal, _ = served
    assert goal()["accepted"] is True
    bridge.goal["state"] = ended
    assert act("brain")["executed"] is True


# 5 ---------------------------------------------------------------------

def test_a_lapsed_claim_does_not_block_a_goal(served, monkeypatch):
    _, bridge, act, goal, _ = served
    assert act("brain")["executed"] is True
    real = time.monotonic
    monkeypatch.setattr(server.time, "monotonic", lambda: real() + 5.0)
    assert goal()["accepted"] is True and bridge.goal is not None
