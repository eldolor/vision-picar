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
def world_over_asgi():
    """A RemoteWorld talking to an in-process robot/server.py app -- N1.

    Its own fixture rather than a field on `robot_over_asgi`, because the
    two are different abstractions over the same server: body state and
    world state. A test that wants both asks for both, which is the seam
    made visible in the test suite.
    """
    from control.remote_world import RemoteWorld
    from robot.server import create_app

    client = asgi_client(create_app())
    world = RemoteWorld(ASGI_BASE_URL, client=client)
    yield world
    client.close()


@pytest.fixture
def robot_and_world_over_asgi():
    """BOTH halves over HTTP, against ONE in-process `robot/server.py`.

    `robot_over_asgi` and `world_over_asgi` each build their own app, which
    is right when a test exercises one abstraction and wrong the moment it
    needs both: two apps are two `GridWorld`s, so the world would be a
    correct map of a different house and the robot's pose would be reported
    against a layout it is not standing in. `world/factory.py` refuses that
    configuration in production for exactly this reason -- "and it would
    look right, because both houses have the same walls".

    Yields `(robot, world)`.
    """
    from control.remote_robot import RemoteRobot
    from control.remote_world import RemoteWorld
    from robot.server import create_app

    client = asgi_client(create_app())
    yield RemoteRobot(ASGI_BASE_URL, client=client), RemoteWorld(ASGI_BASE_URL, client=client)
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

    def get_odometry(self) -> dict:
        # Phase B. This wrapper HAD fallen behind -- it inherited the
        # honest no-op while wrapping a MockRobot with working odometry,
        # which is exactly the failure
        # tests/test_robot_contract.py::test_a_wrapper_reports_the_odometry
        # _of_what_it_WRAPS is written against, arriving in the test
        # helpers rather than in shipped code.
        return self._record("get_odometry", self.delegate.get_odometry())

    def get_wheel_state(self) -> dict:
        # R2: pass through, or this helper falls behind the interface the
        # way it once did for odometry.
        return self._record("get_wheel_state", self.delegate.get_wheel_state())

    def get_scan(self) -> dict:
        return self._record("get_scan", self.delegate.get_scan())


def fresh_mock_robot():
    """A MockRobot on a brand-new starter house."""
    from sim.maps.starter_house import build_starter_world
    from sim.mock_robot import MockRobot

    return MockRobot(build_starter_world())


def fresh_mock_runner(**kwargs):
    """A `MissionRunner` on a brand-new starter house, with BOTH halves wired.

    A test that builds a runner directly is standing in for
    `control/brain_server.py`, which builds a body client and a world client
    and hands over both. Before the frontier policy became allocentric
    (`PLAN-ros-alignment.md`) there was only one half to wire, so every such
    test passed a robot and nothing else; this keeps that one line long while
    making the pairing automatic, which is the same trap the brain server's
    own `default_world_factory` was changed to close.

    Pass `world=` explicitly to override -- `world=NullWorld()` is how a test
    says "no mapper", which is a real configuration and not an oversight.
    """
    from control.mission_runner import MissionRunner

    robot = fresh_mock_robot()
    kwargs.setdefault("world", mock_world_for(robot))
    return MissionRunner(robot, **kwargs)


def mock_world_for(robot):
    """The WORLD half of the simulator, sharing the body's own `GridWorld`.

    Needed by any test that drives the frontier-preference policy, which is
    allocentric as of `PLAN-ros-alignment.md`: it asks `get_pose()` where it
    is and buckets that at the map's resolution, where it used to read a
    grid cell off the camera frame. A test that constructs a `MissionRunner`
    directly is standing in for `robot/server.py`, which builds both halves
    through their factories -- so it supplies both, exactly as it already
    supplies the robot.

    **The SAME GridWorld, never a second one.** `world/factory.py` refuses
    `world: sim` for a robot with no grid for this reason: a world model that
    built its own would report the robot's pose against a layout it is not
    standing in, and both houses have the same walls, so it would look right.
    """
    from sim.mock_world import MockWorld

    # Follow a wrapper chain. `RecordingRobot` and `MissionRunner`'s own
    # `_HaltGate` both delegate method by method, so the GridWorld can be
    # one or two objects down -- and a test that wrapped its robot should
    # not have to know that to get the matching world.
    grid = robot
    while grid is not None and not hasattr(grid, "world"):
        grid = getattr(grid, "delegate", None)
    if grid is None:
        raise AttributeError(
            f"{robot!r} has no GridWorld to build a world model on. "
            "Only the grid-world backend can -- see world/factory.py.")
    return MockWorld(grid.world)
