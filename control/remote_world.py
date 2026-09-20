"""
remote_world.py

Phase N1 (`PLAN-mapping.md`) -- `WorldInterface` over HTTP.
`control/remote_robot.py`'s sibling, and deliberately the same shape.

The reasoning is B0's, one interface over: where the map is computed
stops being an architectural question and becomes a base-URL one.

    in the sim        RemoteWorld("http://localhost:8000")   -> MockWorld
    on the robot      RemoteWorld("http://raspberrypi:8000") -> RosWorld
                                                                -> the SLAM
                                                                   container

**This is the consumer side of the wall** that `PLAN-onboard-perception.md`
3.3 chose over adopting ROS outright. Note what is NOT in this file: no
`rclpy`, no TF lookup, no message types, no notion that ROS exists at
all. It reads JSON. When N6 puts `slam_toolbox` behind `/world/map`,
nothing here changes -- which is the test of whether (b+) was really
built or whether (c) arrived wearing its clothes.

Why `control/` rather than `world/`: the same rule that puts `RemoteRobot`
here. `control/` may import `robot/interface.py` and `world/interface.py`
and little else, and a backend that reaches the robot only over HTTP is
what makes "run the brain on the Pi" a config change. A `world/` module
importing `httpx` to call a server would invert that.
"""

import logging
from typing import Optional

import httpx

from world.interface import WorldInterface, unusable_map, unusable_pose

logger = logging.getLogger("remote_world")

DEFAULT_TIMEOUT_S = 10.0


class WorldTransportError(RuntimeError):
    """The world server could not be reached, or answered an error.

    Its own type rather than `RobotTransportError`, because the two
    failures mean different things to a caller: a robot that cannot be
    reached must be assumed to be moving and is an emergency, while a map
    that cannot be reached is a degraded mission -- the reactive tier
    still has a lidar ring and a safety collar. Collapsing them would
    make a map outage look like a runaway robot.
    """


class RemoteWorld(WorldInterface):
    """HTTP client of `robot/server.py`'s `/world/*` routes."""

    def __init__(
        self,
        base_url: str,
        secret: Optional[str] = None,
        timeout: float = DEFAULT_TIMEOUT_S,
        client: Optional[httpx.Client] = None,
    ):
        self.base_url = base_url.rstrip("/")
        self.headers = {}
        if secret:
            self.headers["x-app-secret"] = secret
        # An injected client is how tests mount the app in-process
        # (httpx.ASGITransport) with no socket; production passes nothing.
        self._client = client or httpx.Client(base_url=self.base_url, timeout=timeout)
        self._owns_client = client is None

    def get_pose(self) -> dict:
        """A 404 means this server predates the route, which is a real
        state -- the deployed stacks are redeployed one at a time -- and
        the honest answer for it is the same unusable pose a backend with
        no mapper gives. Every other status raises, so a mapper that is
        BROKEN is never quietly reported as one that is absent.

        That distinction is the whole reason this is not a bare
        try/except: 'I have no map' is a fact the planner can work with
        (1.5's bootstrap rule -- an empty map is not a special mode), and
        'the mapper crashed' is not.
        """
        try:
            return self._request("/world/pose")
        except WorldTransportError as e:
            if "HTTP 404" in str(e):
                logger.info("server has no /world/pose route -- reporting no pose")
                return unusable_pose()
            raise

    def get_map(self) -> dict:
        """Same 404 rule as `get_pose()`.

        **No caching here, deliberately, even though `map_version` exists
        to enable it.** A cache belongs where the polling policy lives --
        the twin, or a planner that knows how often it needs the house --
        and a backend that silently served a stale map would make
        `map_version` a lie rather than a tool. The field is published; the
        decision to skip a fetch is the caller's.
        """
        try:
            return self._request("/world/map")
        except WorldTransportError as e:
            if "HTTP 404" in str(e):
                logger.info("server has no /world/map route -- reporting no map")
                return unusable_map()
            raise

    def _request(self, path: str) -> dict:
        try:
            response = self._client.request(
                "GET", f"{self.base_url}{path}", headers=self.headers
            )
        except httpx.HTTPError as e:
            raise WorldTransportError(f"GET {path} failed: {e}") from e
        if response.status_code >= 400:
            raise WorldTransportError(f"GET {path} failed: HTTP {response.status_code}")
        return response.json()

    def close(self) -> None:
        if self._owns_client:
            self._client.close()
