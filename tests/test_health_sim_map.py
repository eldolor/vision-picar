"""
tests/test_health_sim_map.py

`/health`'s `sim_map` names the house the factory BUILT, for every body
standing in a sim house -- docs-review/SPEC-REVIEW.md finding 2.

The live suites (test_ros_chain_live.py, test_nav_live.py, test_slam_live.py)
skip unless the server says it is in the house they were judged on.
`sim_map` used to be re-read from SIM_MAP only under `mode: sim`, so with
`ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake` -- exactly 3.33's G4 run on the
Jetson -- it was None and every live test skipped: "5 consecutive passes"
would have been five runs of skips.
"""

import pytest
from fastapi.testclient import TestClient

import robot.server as server


@pytest.mark.parametrize("mode, env, expected", [
    ("sim", {}, "scaled_house"),
    ("hardware", {"SIM_MOTOR_BOARD": "fake"}, "scaled_house"),   # G4's configuration
    ("teleop", {"WORLD_MODE": "none"}, None),                    # no sim house behind it
])
def test_sim_map_names_the_house_that_was_built(monkeypatch, mode, env, expected):
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    monkeypatch.setenv("ROBOT_MODE", mode)
    for k in ("SIM_MOTOR_BOARD", "WORLD_MODE", "ROBOT_DRIVE"):
        monkeypatch.delenv(k, raising=False)
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    with TestClient(server.create_app()) as client:
        assert client.get("/health").json()["sim_map"] == expected


def test_the_name_comes_from_the_world_not_the_environment(monkeypatch):
    # Built in the scaled house; the variable changing afterwards must not
    # change the answer -- the server is still standing where it was built.
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    monkeypatch.setenv("ROBOT_MODE", "sim")
    with TestClient(server.create_app()) as client:
        monkeypatch.setenv("SIM_MAP", "starter_house")
        assert client.get("/health").json()["sim_map"] == "scaled_house"
