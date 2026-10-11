"""
tests/test_random_house.py -- PLAN-ros-alignment.md 3.72: the robot server
puts the simulated robot in a random one of the 100 ProcTHOR houses, in
place, without a restart; and refuses whenever that would be unsafe or
meaningless, changing nothing.
"""
import os

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from sim.maps.procthor import usable

LIST = {f"procthor:val:{i}" for i in usable()[:server.RANDOM_HOUSES]}


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setenv("ROBOT_MODE", "sim")
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    for k in ("SIM_MOTOR_BOARD", "WORLD_MODE", "ROBOT_DRIVE", "APP_SHARED_SECRET"):
        monkeypatch.delenv(k, raising=False)
    with TestClient(server.create_app()) as c:
        yield c


def test_the_house_changes_in_place(client):
    pid = os.getpid()
    before = client.get("/world/map").json()
    r = client.post("/sim/house/random")
    assert r.status_code == 200, r.text
    name = r.json()["sim_map"]
    assert name in LIST and r.json()["previous"] == "scaled_house"
    assert client.get("/health").json()["sim_map"] == name           # criterion 1
    m = client.get("/world/map").json()
    assert m["map_version"] != before["map_version"]
    from sim.maps import build_world
    fresh = build_world(name)
    assert (m["width"], m["height"]) == (fresh.width, fresh.height)
    pose = client.get("/world/pose").json()
    assert pose["x_m"] == pytest.approx(fresh.x * 0.3, abs=0.01)
    assert pose["y_m"] == pytest.approx(fresh.y * 0.3, abs=0.01)
    odo = client.get("/odometry").json()
    assert abs(odo.get("distance_m", odo.get("path_m", 0.0))) < 1e-6
    assert os.getpid() == pid                                          # no restart


def test_the_readings_safety_vets_on_are_the_new_houses(client):
    """Criterion 2: after a swap, the distance and depth safety reads come
    from the NEW walls -- equal to a fresh body standing at the new start --
    and a verb still runs."""
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    name = client.post("/sim/house/random").json()["sim_map"]
    fresh = MockRobot(build_world(name), render=False)
    assert client.get("/distance").json()["distance_cm"] == fresh.get_distance()
    zones = [z.get("distance_cm") for z in client.get("/depth").json()["zones"]]
    assert zones == [z.get("distance_cm") for z in fresh.get_depth_grid()["zones"]]
    r = client.post("/action", json={"action": "LOOK_LEFT"}, headers={"x-driver": "twin-dpad"})
    assert r.status_code == 200, r.text


def test_never_while_someone_drives(client):
    client.post("/action", json={"action": "LOOK_LEFT"}, headers={"x-driver": "brain"})
    house = client.get("/health").json()["sim_map"]
    r = client.post("/sim/house/random")
    assert r.status_code == 409 and r.json()["detail"]["reason"] == "driving"
    assert client.get("/health").json()["sim_map"] == house


def test_needs_the_secret(monkeypatch):
    monkeypatch.setenv("ROBOT_MODE", "sim")
    monkeypatch.setenv("APP_SHARED_SECRET", "s3cret")
    for k in ("SIM_MOTOR_BOARD", "WORLD_MODE", "ROBOT_DRIVE"):
        monkeypatch.delenv(k, raising=False)
    with TestClient(server.create_app()) as c:
        house = c.get("/health").json()["sim_map"]
        assert c.post("/sim/house/random").status_code == 401
        assert c.get("/health").json()["sim_map"] == house


def test_not_under_another_backend(monkeypatch):
    monkeypatch.setenv("ROBOT_MODE", "teleop")
    monkeypatch.setenv("WORLD_MODE", "none")
    for k in ("SIM_MOTOR_BOARD", "ROBOT_DRIVE", "APP_SHARED_SECRET"):
        monkeypatch.delenv(k, raising=False)
    with TestClient(server.create_app()) as c:
        r = c.post("/sim/house/random")
        assert r.status_code == 409 and r.json()["detail"]["reason"] == "not_sim"


def test_random_over_the_list_never_the_same_twice_running(client):
    seen, last = set(), None
    for _ in range(200):
        name = client.post("/sim/house/random").json()["sim_map"]
        assert name in LIST and name != last
        seen.add(name)
        last = name
    assert len(seen) >= 80                                             # criterion 4
