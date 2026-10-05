"""
body_client.py

The robot server's side of the split simulator (PLAN-ros-alignment.md
3.36): `sim/body_server.py` (physics, the fake board, the truth) and
`sim/sensor_server.py` (the sensors, several worker processes).
`robot/factory.py` hands this to `HardwareRobot` as `sensors` when
`SIM_MOTOR_BOARD=fake` -- so the robot server opens the board's pty as a
serial device and reads its sensors from outside itself, as on the car.

Sim plumbing, so it lives in sim/ and imports nothing of the simulator:
importing this must not pull a GridWorld into the robot server
(`tests/test_sim_body_process.py`, criterion 1).

**What the safety layer reads never crosses the network while it reads it.**
`robot/safety.py` vets every wheel-loop period, inside `motion_lock`; the
first build of 3.36 made that an HTTP round trip and the lock was held
across it -- the wheel loop then skipped most of its ticks. Here a
background thread keeps the latest *safety bundle* (the scan the vet asks
for, the depth grid, the scalar distance -- cast from one snapshot) at the
lidar's own rate, and the vet reads that copy. That is also the car's
shape: a lidar delivers scans when it has them, and nothing can make one
arrive on demand.

**A copy older than `SENSOR_STALE_S` is not a reading.** It answers
`usable: false` (scan), all-unusable zones (depth) and `0.0` (distance) --
the existing fail-safe path, where a blind path vetoes forward. A dead
sensor program therefore stops forward motion within the stale bound plus
one wheel-loop period. Never raises into the wheel loop: a raised tick
would leave a standing command driving unvetted.
"""

import threading
import time
from typing import Optional

import httpx

from robot.interface import unusable_grid, unusable_scan

# Three wheel-loop periods (0.05 s); the real lidar's own period is 0.1 s.
# Criterion 8 measures what a copy this old costs the stopping distance.
SENSOR_STALE_S = 0.15
POLL_S = 0.05                     # the robot server's wheel-loop period
READ_TIMEOUT_S = 0.5              # a poll; staleness, not this, is the safety bound
FRAME_TIMEOUT_S = 3.0             # a rendered camera frame
START_TIMEOUT_S = 30.0


class SimBodyClient:
    """The sensing half of `RobotInterface`, and the sim-only extras the
    robot server's ground-truth routes read, served by two other programs."""

    def __init__(self, body_url: str, sensors_url: str, secret: str = "",
                 start_timeout_s: float = START_TIMEOUT_S, poll: bool = True):
        headers = {"x-app-secret": secret} if secret else {}
        self._body = httpx.Client(base_url=body_url.rstrip("/"), headers=headers,
                                  timeout=READ_TIMEOUT_S)
        # Three clients, one per thread that uses them: the poller, the
        # request threads (frames, unhinted scans) -- httpx clients are not
        # promised safe to share across threads under load.
        self._sensors = httpx.Client(base_url=sensors_url.rstrip("/"), headers=headers,
                                     timeout=READ_TIMEOUT_S)
        self._poll_http = httpx.Client(base_url=sensors_url.rstrip("/"), headers=headers,
                                       timeout=READ_TIMEOUT_S)
        health = _wait_for(self._body, "/health", start_timeout_s, "sim/body_server.py")
        _wait_for(self._sensors, "/health", start_timeout_s, "sim/sensor_server.py")
        self.board_path = health["board_path"]
        self.world = RemoteGrid(self, health.get("sim_map"))
        self._hint: Optional[float] = None       # the scan range the vet asks for
        self._bundle: Optional[dict] = None
        self._bundle_at = 0.0
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._fresh = threading.Condition(self._lock)
        self._running = True
        self.polls = 0
        self.poll_failures = 0
        self._thread = None
        if poll:
            self._thread = threading.Thread(target=self._poller, daemon=True,
                                            name="sim-sensor-poll")
            self._thread.start()

    # ---------- the safety bundle ----------

    def _fetch_bundle(self, http: httpx.Client, hint: Optional[float]) -> None:
        params = {} if hint is None else {"max_range_m": hint}
        r = http.get("/safety", params=params)
        r.raise_for_status()
        bundle = r.json()
        with self._lock:
            if hint == self._hint:               # a bundle for a stale hint is not kept
                self._bundle, self._bundle_at = bundle, time.monotonic()
                self._fresh.notify_all()

    def _poller(self) -> None:
        while self._running:
            started = time.monotonic()
            try:
                self._fetch_bundle(self._poll_http, self._hint)
                self.polls += 1
            except (httpx.HTTPError, ValueError):
                self.poll_failures += 1
            self._wake.wait(max(0.0, POLL_S - (time.monotonic() - started)))
            self._wake.clear()

    def _fresh_bundle(self) -> Optional[dict]:
        with self._lock:
            if self._bundle is not None and time.monotonic() - self._bundle_at <= SENSOR_STALE_S:
                return self._bundle
        return None

    def bundle_age_s(self) -> Optional[float]:
        with self._lock:
            return None if self._bundle is None else time.monotonic() - self._bundle_at

    def sensor_age_s(self) -> Optional[float]:
        """How old the readings the safety layer gets are: it takes the way
        covered since off the clearance (`SafetyController._aged`, 3.36)."""
        return self.bundle_age_s()

    # ---------- sensing ----------

    def get_scan(self, max_range_m: Optional[float] = None) -> dict:
        """With a range hint (the safety layer's call): the latest bundle's
        scan, or `usable: false` once it is older than SENSOR_STALE_S. The
        first hinted call names the range the poller fetches from then on;
        until its first bundle lands the answer is the honest unusable one.
        Without a hint (`/scan` for ROS and the twin, never under the motion
        lock): a direct read."""
        if max_range_m is None:
            try:
                r = self._sensors.get("/scan")
                r.raise_for_status()
                return r.json()
            except (httpx.HTTPError, ValueError):
                return unusable_scan()
        if self._hint != max_range_m:
            # Once per process in practice: the vet's range is a constant.
            # The poller fetches for the new range at once and this waits
            # for it (a wait, not a socket read on this thread).
            with self._lock:
                self._hint, self._bundle = max_range_m, None
            if self._thread is None:             # no poller (a test): fetch now
                try:
                    self._fetch_bundle(self._sensors, max_range_m)
                except (httpx.HTTPError, ValueError):
                    pass
            else:
                self._wake.set()
                with self._lock:
                    self._fresh.wait_for(lambda: self._bundle is not None, timeout=2 * POLL_S + READ_TIMEOUT_S)
        bundle = self._fresh_bundle()
        return bundle["scan"] if bundle else unusable_scan()

    def get_depth_grid(self) -> dict:
        bundle = self._fresh_bundle()
        return bundle["depth"] if bundle else unusable_grid()

    def get_distance(self) -> float:
        bundle = self._fresh_bundle()
        return float(bundle["distance_cm"]) if bundle else 0.0

    def get_camera_frame(self) -> dict:
        # Raises like a camera with no driver does: there is no honest
        # "unusable" picture, and the callers already handle the raise.
        r = self._sensors.get("/frame", timeout=FRAME_TIMEOUT_S)
        r.raise_for_status()
        return r.json()

    # ---------- the pan head (physics owns it) ----------

    def _look(self, side: str) -> dict:
        r = self._body.post(f"/look/{side}")
        r.raise_for_status()
        return r.json()

    def look_left(self) -> dict:
        return self._look("left")

    def look_right(self) -> dict:
        return self._look("right")

    def look_center(self) -> dict:
        return self._look("center")

    # ---------- ground truth: sim only, no decision may read it ----------

    def get_truth(self) -> dict:
        r = self._body.get("/truth")
        r.raise_for_status()
        return r.json()

    def close(self) -> None:
        self._running = False
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        for c in (self._body, self._sensors, self._poll_http):
            c.close()


def _wait_for(http: httpx.Client, path: str, timeout_s: float, program: str) -> dict:
    """The simulator programs must be up before the robot server builds its
    HardwareRobot, which opens the board's pty at once. Waiting here makes
    the start-up order forgiving."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            r = http.get(path, timeout=1.0)
            if r.status_code == 200:
                return r.json()
        except httpx.HTTPError:
            pass
        if time.monotonic() > deadline:
            raise RuntimeError(f"no {program} at {http.base_url} -- start it first (3.36)")
        time.sleep(0.2)


class RemoteGrid:
    """What the robot server's sim-only routes and the world factory read
    off `robot.world` -- the house's name, its objects, moving one, and the
    truth -- for a GridWorld that lives in the physics program."""

    def __init__(self, client: SimBodyClient, map_name: Optional[str]):
        self._client = client
        self.map_name = map_name

    def describe_objects(self) -> dict:
        r = self._client._body.get("/sim/objects")
        r.raise_for_status()
        return r.json()

    def move_object(self, src, dst) -> None:
        r = self._client._body.post("/sim/objects/move", json={"src": list(src), "dst": list(dst)})
        if r.status_code == 409:
            raise ValueError(r.json().get("detail", "refused"))
        r.raise_for_status()

    def get_truth(self) -> dict:
        return self._client.get_truth()
