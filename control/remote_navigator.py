"""
remote_navigator.py -- nav2's goals, over HTTP (PLAN-ros-alignment.md 3.31).

The `Navigator` that `brain/explore.py` declares, implemented over the robot
server's existing `/world/goal` routes. `RemoteWorld`'s sibling and the same
shape, but deliberately not part of it: `WorldInterface` is asked what is
true about the house and is never asked to drive, and a goal is a drive
command (`tests/test_world_contract.py`).

Like `RemoteWorld`, it reads JSON and contains no hint that ROS exists. A
server whose world cannot take goals answers 501; that is reported as
`unsupported`, never as an accepted goal, and `available()` lets the brain
refuse an `explore` mission at start rather than on its first tick.
"""

from typing import Optional

import httpx

DEFAULT_TIMEOUT_S = 10.0


class RemoteNavigator:
    """HTTP client of `/world/goal` (POST to send, GET to read, DELETE to
    cancel), in the house frame."""

    def __init__(self, base_url: str, secret: Optional[str] = None,
                 timeout: float = DEFAULT_TIMEOUT_S,
                 client: Optional[httpx.Client] = None):
        self.base_url = base_url.rstrip("/")
        self.headers = {"x-app-secret": secret} if secret else {}
        # An injected client is how tests mount the app in-process.
        self._client = client or httpx.Client(base_url=self.base_url, timeout=timeout)

    def _call(self, method: str, json: Optional[dict] = None) -> httpx.Response:
        return self._client.request(method, "/world/goal", json=json, headers=self.headers)

    def available(self) -> bool:
        """Can this server take goals at all? (A world with nav2 behind it.)"""
        r = self._call("GET")
        return r.status_code == 200

    def set_goal(self, x_m: float, y_m: float) -> dict:
        r = self._call("POST", {"x_m": x_m, "y_m": y_m})
        if r.status_code == 501:
            return {"accepted": False, "reason": "unsupported"}
        r.raise_for_status()
        return r.json()

    def get_goal(self) -> dict:
        r = self._call("GET")
        if r.status_code == 501:
            return {"goal": None, "plan": []}
        r.raise_for_status()
        return r.json()

    def cancel_goal(self) -> dict:
        r = self._call("DELETE")
        if r.status_code == 501:
            return {"canceled": False, "reason": "unsupported"}
        r.raise_for_status()
        return r.json()
