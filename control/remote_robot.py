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

from robot.interface import (Preempted, RobotInterface, unusable_grid,
                             unusable_scan, unusable_wheels,
                             unusable_odometry)
from robot.safety import SafetyViolation

logger = logging.getLogger("remote_robot")

DEFAULT_TIMEOUT_S = 10.0

# Phase M4. Every command names its driver so `robot/server.py` can apply a
# decided priority order rather than letting the last writer win. "brain" is
# the autonomous rank: a person tapping the twin's D-pad outranks it and
# takes the robot, which is the whole point -- see AGENT-HARNESS.md.
DEFAULT_DRIVER = "brain"


class RobotTransportError(RuntimeError):
    """The robot server was unreachable, timed out, or answered with an
    error status. Distinct from SafetyViolation: that means the robot
    heard the command and refused it, this means nobody heard it."""


# A `_tupleize` helper used to live here, turning a JSON list back into the
# tuple `MissionAgent` keyed its visited set with. Gone with the grid cell it
# existed for (`PLAN-ros-alignment.md`): the agent buckets a world pose now,
# and no frame or action ack carries a `position` to restore.


class RemoteRobot(RobotInterface):
    """HTTP client of robot/server.py, implementing the same interface
    the in-process backends do."""

    def __init__(
        self,
        base_url: str,
        secret: Optional[str] = None,
        timeout: float = DEFAULT_TIMEOUT_S,
        client: Optional[httpx.Client] = None,
        driver: str = DEFAULT_DRIVER,
    ):
        self.base_url = base_url.rstrip("/")
        self.driver = driver
        self.headers = {"x-driver": driver}
        if secret:
            self.headers["x-app-secret"] = secret
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
        return body.get("result", {})

    # ---------- camera pan ----------

    def look_left(self) -> dict:
        return self._action("LOOK_LEFT")

    def look_right(self) -> dict:
        return self._action("LOOK_RIGHT")

    def look_center(self) -> dict:
        return self._action("LOOK_CENTER")

    # ---------- sensing ----------

    def get_camera_frame(self) -> dict:
        return self._request("GET", "/frame")

    def get_distance(self) -> float:
        return float(self._request("GET", "/distance")["distance_cm"])

    def get_odometry(self) -> dict:
        """Phase B. Overrides the all-unusable default for the same reason
        `get_depth_grid()` does: the robot on the other end may well have
        encoders, and inheriting the default would report that it did not.

        A 404 means the server predates the route and is answered with the
        honest no-op. Every other status raises, so a *broken* encoder is
        never quietly reported as an absent one."""
        try:
            return self._request("GET", "/odometry")
        except RobotTransportError as e:
            if "HTTP 404" in str(e):
                logger.info(
                    "robot server has no /odometry route -- reporting no odometry")
                return unusable_odometry()
            raise

    def get_wheel_state(self) -> dict:
        """Phase R2. Same 404 rule as `get_odometry()`: a server that predates
        the route has no wheel state to give and says so; any other failure
        raises, so a broken encoder never reads as an absent one."""
        try:
            return self._request("GET", "/wheels")
        except RobotTransportError as e:
            if "HTTP 404" in str(e):
                logger.info("robot server has no /wheels route -- reporting no wheel state")
                return unusable_wheels()
            raise

    def get_scan(self, max_range_m: Optional[float] = None) -> dict:
        """Phase R2. Same 404 rule: an older server is honestly lidar-less.
        The range hint (3.18) travels as a query parameter, so the robot
        server's own sim casts only as far as the safety check needs."""
        path = "/scan" if max_range_m is None else f"/scan?max_range_m={max_range_m:g}"
        try:
            return self._request("GET", path)
        except RobotTransportError as e:
            if "HTTP 404" in str(e):
                logger.info("robot server has no /scan route -- reporting no scan")
                return unusable_scan()
            raise

    def get_depth_grid(self) -> dict:
        """Phase M2. Overrides `RobotInterface`'s all-unusable default,
        because the robot on the other end of this socket may well have a
        depth sensor and inheriting the default would report that it did
        not -- a silent lie in exactly the direction a safety consumer
        must not be lied to.

        A 404 is the one error not treated as a transport failure: it means
        this server predates the route, which is a real state (the deployed
        stacks are redeployed one at a time), and the honest answer for it
        is the same all-unusable grid any sensorless backend gives. Every
        other status still raises, so a broken sensor is not quietly
        reported as an absent one."""
        try:
            return self._request("GET", "/depth")
        except RobotTransportError as e:
            if "HTTP 404" in str(e):
                logger.info("robot server has no /depth route -- reporting no depth sensor")
                return unusable_grid()
            raise

    # ---------- internal ----------

    def _action(self, action: str, **kwargs) -> dict:
        body = self._request("POST", "/action", json={"action": action, **kwargs})
        if body.get("executed") is False:
            detail = body.get("detail", f"{action} blocked by robot server")
            # Phase M4: which refusal this is decides what the caller
            # should do, and the two answers are opposites. A safety veto
            # means *this move* was unsafe and the next one may be fine, so
            # it raises what the in-process path raises and the agent's
            # existing except-clause treats it as an implicit STOP. A
            # preemption means *this driver* is no longer in charge, and
            # retrying is exactly wrong -- MissionRunner ends the mission.
            #
            # Branching on `reason` rather than on the prose: a server
            # older than M4 sends no reason at all, and everything it
            # refuses is a safety veto, which is what the fallback below
            # preserves.
            if body.get("reason") == "preempted":
                raise Preempted(detail)
            raise SafetyViolation(detail)
        return body.get("result", {})

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
