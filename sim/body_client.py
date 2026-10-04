"""
body_client.py

The robot server's side of `sim/body_server.py` (PLAN-ros-alignment.md
3.36): the fake motor board's simulated body, read over HTTP from the
process that owns it. `robot/factory.py` hands it to `HardwareRobot` as
`sensors` when `SIM_BODY_URL` is set, in place of an in-process `MockRobot`
-- so the robot server opens the board's pty as a serial device and reads
its camera, depth grid and scan from outside itself, as it will on the car.

Sim plumbing, so it lives in sim/ and imports nothing of the simulator:
importing this must not pull a GridWorld into the robot server, which is
the whole point (`tests/test_sim_body_process.py`, criterion 1).

**A read that cannot reach the body fails SAFE, never raises.** The scan and
the depth grid answer `usable: false` and the scalar distance `0.0` -- the
same dropout reading that always trips the veto (sim/sensors.py). Raising
would be worse: the robot server's wheel loop catches a failed tick and
carries on, so a vet that raised would leave the standing command driving
unvetted until the next tick. With the body gone the board is gone too,
its feedback goes stale, and 3.34's rule zeroes the wheels.
"""

import time
from typing import Optional

import httpx

from robot.interface import unusable_grid, unusable_scan

# Inside one 50 ms wheel-loop period with room to spare; a localhost round
# trip is a few ms (3.17). A body slower than this is treated as gone.
READ_TIMEOUT_S = 0.25
START_TIMEOUT_S = 30.0


class SimBodyClient:
    """The sensing half of `RobotInterface` (and the sim-only extras the
    robot server's ground-truth routes read), served by another process."""

    def __init__(self, url: str, secret: str = "", timeout_s: float = READ_TIMEOUT_S,
                 start_timeout_s: float = START_TIMEOUT_S):
        headers = {"x-app-secret": secret} if secret else {}
        self._http = httpx.Client(base_url=url.rstrip("/"), headers=headers, timeout=timeout_s)
        health = self._wait_for(start_timeout_s)
        self.board_path = health["board_path"]
        self.world = RemoteGrid(self, health.get("sim_map"))

    def _wait_for(self, start_timeout_s: float) -> dict:
        """The body server must be up before the robot server builds its
        HardwareRobot, which opens the board's pty at once. Waiting here
        rather than failing makes the start-up order forgiving."""
        deadline = time.monotonic() + start_timeout_s
        while True:
            try:
                r = self._http.get("/health", timeout=1.0)
                if r.status_code == 200:
                    return r.json()
            except httpx.HTTPError:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError(f"no sim body server at {self._http.base_url} "
                                   "(start sim/body_server.py first -- 3.36)")
            time.sleep(0.2)

    def _get(self, path: str, **params):
        r = self._http.get(path, params={k: v for k, v in params.items() if v is not None})
        r.raise_for_status()
        return r.json()

    def _post(self, path: str, body: Optional[dict] = None):
        r = self._http.post(path, json=body or {})
        r.raise_for_status()
        return r.json()

    # ---------- sensing: fail safe, never raise ----------

    def get_scan(self, max_range_m: Optional[float] = None) -> dict:
        try:
            return self._get("/scan", max_range_m=max_range_m)
        except (httpx.HTTPError, ValueError):
            return unusable_scan()

    def get_depth_grid(self) -> dict:
        try:
            return self._get("/depth")
        except (httpx.HTTPError, ValueError):
            return unusable_grid()

    def get_distance(self) -> float:
        try:
            return float(self._get("/distance")["distance_cm"])
        except (httpx.HTTPError, ValueError, KeyError):
            return 0.0

    def get_camera_frame(self) -> dict:
        # Raises like a camera with no driver does: there is no honest
        # "unusable" picture, and the callers already handle the raise.
        return self._get("/frame")

    # ---------- the pan head ----------

    def look_left(self) -> dict:
        return self._post("/look/left")

    def look_right(self) -> dict:
        return self._post("/look/right")

    def look_center(self) -> dict:
        return self._post("/look/center")

    # ---------- ground truth: sim only, no decision may read it ----------

    def get_truth(self) -> dict:
        return self._get("/truth")

    def close(self) -> None:
        self._http.close()


class RemoteGrid:
    """What the robot server's sim-only routes and the world factory read
    off `robot.world` -- the house's name, its objects, moving one, and the
    truth -- for a GridWorld that lives in the body server's process."""

    def __init__(self, client: SimBodyClient, map_name: Optional[str]):
        self._client = client
        self.map_name = map_name

    def describe_objects(self) -> dict:
        return self._client._get("/sim/objects")

    def move_object(self, src, dst) -> None:
        r = self._client._http.post("/sim/objects/move", json={"src": list(src), "dst": list(dst)})
        if r.status_code == 409:
            raise ValueError(r.json().get("detail", "refused"))
        r.raise_for_status()

    def get_truth(self) -> dict:
        return self._client.get_truth()
