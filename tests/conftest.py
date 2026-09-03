"""
conftest.py

Shared fixtures for the Stage 2 ("put the brain on the wire") tests.

Two ways to reach a robot server, deliberately:

- `robot_over_asgi` mounts robot/server.py in-process via FastAPI's
  TestClient (an httpx.Client whose transport runs the ASGI app). Real
  routing, real JSON serialisation, no socket and no subprocess -- fast
  enough to use everywhere. httpx.ASGITransport itself is async-only, so
  it cannot back the sync client RemoteRobot uses.
- `live_robot_server` runs an actual `uvicorn robot.server:app` on a free
  port. Slower, and the only way to prove the thing that matters: that a
  real network boundary between brain and robot changes nothing.

Both hand back a *fresh* world per test (MockRobot state lives in the
server process/app instance), so mission outcomes are reproducible.
"""

import socket
import subprocess
import sys
import time
from pathlib import Path

import httpx
import pytest

from robot.interface import RobotInterface

REPO_ROOT = Path(__file__).resolve().parent.parent
SERVER_START_TIMEOUT_S = 30.0
ASGI_BASE_URL = "http://robot.test"


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def asgi_client(app) -> "TestClient":
    """A sync httpx client bound to an ASGI app, for injecting into
    RemoteRobot. TestClient is httpx.Client plus a transport that drives
    the app on its own event loop."""
    from fastapi.testclient import TestClient

    return TestClient(app, base_url=ASGI_BASE_URL)


@pytest.fixture
def robot_over_asgi():
    """A RemoteRobot talking to an in-process robot/server.py app."""
    from control.remote_robot import RemoteRobot
    from robot.server import create_app

    client = asgi_client(create_app())
    robot = RemoteRobot(ASGI_BASE_URL, client=client)
    yield robot
    client.close()


@pytest.fixture
def live_robot_server():
    """A real `uvicorn robot.server:app` subprocess. Yields its base URL."""
    port = free_port()
    url = f"http://127.0.0.1:{port}"
    proc = subprocess.Popen(
        [
            sys.executable, "-m", "uvicorn", "robot.server:app",
            "--host", "127.0.0.1", "--port", str(port), "--log-level", "warning",
        ],
        cwd=REPO_ROOT,
    )
    try:
        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pytest.fail(f"robot server exited early with code {proc.returncode}")
            try:
                if httpx.get(f"{url}/health", timeout=0.5).status_code == 200:
                    break
            except httpx.HTTPError:
                time.sleep(0.1)
        else:
            pytest.fail(f"robot server was not healthy within {SERVER_START_TIMEOUT_S}s")
        yield url
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


class RecordingRobot(RobotInterface):
    """Wraps any RobotInterface and records the calls made through it.

    Used by the failsafe tests: "the robot received a STOP" is the whole
    assertion, and nothing in the HTTP surface exposes it.
    """

    def __init__(self, delegate: RobotInterface):
        self.delegate = delegate
        self.calls: list = []

    def _record(self, name, result):
        self.calls.append(name)
        return result

    def drive_forward(self, speed: int = 50, duration: float = 0.5) -> dict:
        return self._record("drive_forward", self.delegate.drive_forward(speed, duration))

    def reverse(self, speed: int = 50, duration: float = 0.5) -> dict:
        return self._record("reverse", self.delegate.reverse(speed, duration))

    def turn_left(self, angle: int = 90) -> dict:
        return self._record("turn_left", self.delegate.turn_left(angle))

    def turn_right(self, angle: int = 90) -> dict:
        return self._record("turn_right", self.delegate.turn_right(angle))

    def stop(self) -> dict:
        return self._record("stop", self.delegate.stop())

    def look_left(self) -> dict:
        return self._record("look_left", self.delegate.look_left())

    def look_right(self) -> dict:
        return self._record("look_right", self.delegate.look_right())

    def look_center(self) -> dict:
        return self._record("look_center", self.delegate.look_center())

    def get_camera_frame(self) -> dict:
        return self._record("get_camera_frame", self.delegate.get_camera_frame())

    def get_distance(self) -> float:
        return self._record("get_distance", self.delegate.get_distance())

    def get_depth_grid(self) -> dict:
        return self._record("get_depth_grid", self.delegate.get_depth_grid())


def fresh_mock_robot():
    """A MockRobot on a brand-new starter house."""
    from sim.maps.starter_house import build_starter_world
    from sim.mock_robot import MockRobot

    return MockRobot(build_starter_world())
