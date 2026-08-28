"""
remote_robot.py

Phase B0 (= Phase S3 of PLAN-sim-hardening.md) -- put the brain on the wire.

`RemoteRobot` implements `RobotInterface` by calling robot/server.py over
HTTP instead of driving a backend in-process. Nothing in brain/ changes:
an agent handed a RemoteRobot runs exactly the same loop it runs against
MockRobot, so where the robot lives stops being an architectural question
and becomes a base-URL one:

    on the Pi        RemoteRobot("http://localhost:8000")
    from the MacBook RemoteRobot("http://192.168.1.50:8000")

That equivalence is the whole point, and tests/test_remote_robot.py
asserts it directly: the same mission produces an identical action
sequence in-process and over HTTP.

Two details are load-bearing for that transparency:

1. **Safety vetoes have to look the same.** In-process, a blocked action
   raises SafetyViolation out of robot/safety.py. Over HTTP the server
   swallows it and answers 200 with {"executed": false}. RemoteRobot
   re-raises SafetyViolation so the agent's except-clause fires either
   way. (The brain keeps its own SafetyController too -- the server's
   check is authoritative, the brain's is just an early out. Both must be
   able to veto; neither may be the only one that can.)

2. **JSON has no tuples.** grid-world frames carry `position` as a tuple,
   and MissionAgent uses it as a set key. A round trip through JSON turns
   it into a list, which is unhashable -- so positions are coerced back on
   the way in. This is exactly the sort of return-shape detail Phase S1 is
   meant to pin down properly; until it does, the coercion lives here.
"""

import logging
from typing import Optional

import httpx

from robot.interface import RobotInterface
from robot.safety import SafetyViolation

logger = logging.getLogger("remote_robot")

DEFAULT_TIMEOUT_S = 10.0


class RobotTransportError(RuntimeError):
    """The robot server was unreachable, timed out, or answered with an
    error status. Distinct from SafetyViolation: that means the robot
    heard the command and refused it, this means nobody heard it."""


def _tupleize(payload: dict) -> dict:
    """JSON round-trips tuples into lists. `position` is the one field
    whose tuple-ness is depended on downstream (MissionAgent keys a set
    with it), so restore it."""
    if isinstance(payload, dict) and isinstance(payload.get("position"), list):
        return {**payload, "position": tuple(payload["position"])}
    return payload


class RemoteRobot(RobotInterface):
    """HTTP client of robot/server.py, implementing the same interface
    the in-process backends do."""

    def __init__(
        self,
        base_url: str,
        secret: Optional[str] = None,
        timeout: float = DEFAULT_TIMEOUT_S,
        client: Optional[httpx.Client] = None,
    ):
        self.base_url = base_url.rstrip("/")
        self.headers = {"x-app-secret": secret} if secret else {}
        # An injected client is how tests mount the robot app in-process
        # (httpx.ASGITransport) without a real socket; production passes
        # nothing and gets a plain pooled client.
        self._client = client or httpx.Client(base_url=self.base_url, timeout=timeout)
        self._owns_client = client is None

    # ---------- driving ----------

    def drive_forward(self, speed: int = 50, duration: float = 0.5) -> dict:
        return self._action("FORWARD", speed=speed, duration=duration)

    def reverse(self, speed: int = 50, duration: float = 0.5) -> dict:
        return self._action("REVERSE", speed=speed, duration=duration)

    def turn_left(self, angle: int = 90) -> dict:
        return self._action("LEFT", angle=angle)

    def turn_right(self, angle: int = 90) -> dict:
        return self._action("RIGHT", angle=angle)

    def stop(self) -> dict:
        """Uses the server's dedicated /stop route rather than
        /action {"action": "STOP"} -- /stop is the one the robot server
        guarantees is always available."""
        body = self._request("POST", "/stop")
        return _tupleize(body.get("result", {}))

    # ---------- camera pan ----------

    def look_left(self) -> dict:
        return self._action("LOOK_LEFT")

    def look_right(self) -> dict:
        return self._action("LOOK_RIGHT")

    def look_center(self) -> dict:
        return self._action("LOOK_CENTER")

    # ---------- sensing ----------

    def get_camera_frame(self) -> dict:
        return _tupleize(self._request("GET", "/frame"))

    def get_distance(self) -> float:
        return float(self._request("GET", "/distance")["distance_cm"])

    # ---------- internal ----------

    def _action(self, action: str, **kwargs) -> dict:
        body = self._request("POST", "/action", json={"action": action, **kwargs})
        if body.get("executed") is False:
            # The server's safety layer vetoed it. Raise the same
            # exception the in-process path would have raised, so agent
            # code cannot tell the two apart.
            raise SafetyViolation(body.get("detail", f"{action} blocked by robot server"))
        return _tupleize(body.get("result", {}))

    def _request(self, method: str, path: str, json: Optional[dict] = None) -> dict:
        try:
            response = self._client.request(
                method, f"{self.base_url}{path}", json=json, headers=self.headers
            )
        except httpx.HTTPError as e:
            raise RobotTransportError(f"{method} {path} failed: {e}") from e

        if response.status_code == 400:
            # Matches robot/safety.py's own dispatch error for an unknown verb.
            raise ValueError(response.json().get("detail", "bad request"))
        if response.status_code >= 400:
            raise RobotTransportError(
                f"{method} {path} -> HTTP {response.status_code}: {response.text[:200]}"
            )
        return response.json()

    def close(self) -> None:
        if self._owns_client:
            self._client.close()

    def __enter__(self) -> "RemoteRobot":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
