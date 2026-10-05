"""PLAN-ros-alignment.md 3.37: the robot server freezes its start-up
objects out of the collector while it serves, and releases them after."""

import gc

from fastapi.testclient import TestClient

import robot.server as server


def test_start_up_is_frozen_while_serving_and_released_after(monkeypatch):
    monkeypatch.setenv("ROBOT_MODE", "sim")
    before = gc.get_freeze_count()
    with TestClient(server.create_app()) as client:
        assert client.get("/health").status_code == 200
        assert gc.get_freeze_count() > before
    assert gc.get_freeze_count() == before
