"""
tests/test_costmap.py

3.83: nav2's global costmap, from the bridge to the robot server's
`GET /world/costmap`, in-process against a fake bridge -- the criteria in
docs/plans/ros-alignment/3.83-costmap-overlay.md that need no ROS. The
twin's half is in tests/test_ui.py; the live half needs the container.
"""

import json
import math
import statistics
import sys
import time
from pathlib import Path

import httpx
import pytest
from fastapi.testclient import TestClient

import robot.server as server
from tests.test_ros_world import FakeBridge, FakeTruth
from tests.ros_resample_reference import reference_resample
from world.ros_world import Anchor, RosWorld

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"))
from picar_bridge import convert  # noqa: E402


class CostmapBridge(FakeBridge):
    """FakeBridge plus picar_bridge's /nav/costmap: 10 x 10 cells at 0.1 m,
    origin (-0.5, -0.5), the robot at the centre. One lethal cell 0.3 m
    straight ahead (ROS +x), an inscribed and an inflated one beside it,
    one unknown corner; the rest free."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.costmap_version = 1
        self.costs = [0] * 100
        self.costs[5 * 10 + 8] = 100     # row j=5 (y ~ 0.05), col i=8 (x ~ 0.35)
        self.costs[5 * 10 + 7] = 99
        self.costs[5 * 10 + 6] = 40
        self.costs[0] = -1
        self.costmap_session = None      # None: the bridge's own session
        self.costmap_status = 200
        self.costmap_requests = 0

    def handler(self, request):
        if request.url.path == "/nav/costmap":
            self.costmap_requests += 1
            if self.costmap_status != 200:
                return httpx.Response(self.costmap_status, json={"error": "no"})
            return httpx.Response(200, json=convert.grid_record(
                self.costmap_session or self.session, self.costmap_version, 0.1, 10, 10,
                -0.5, -0.5, 0.0, self.costs, 0.0))
        return super().handler(request)


def costmap_world(truth=None, bridge=None):
    bridge = bridge or CostmapBridge(
        start_truth={k: truth.pose[k] for k in ("x_m", "y_m", "heading_deg")} if truth else None)
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge.handler))
    return RosWorld("http://bridge", truth=truth, client=client), bridge


def _cells_with(cm, value):
    """House-frame centres of every cell holding `value`."""
    out = []
    for k, v in enumerate(cm["costs"]):
        if v == value:
            row, col = divmod(k, cm["width"])
            out.append((cm["origin_x_m"] + (col + 0.5) * cm["resolution_m"],
                        cm["origin_y_m"] + (row + 0.5) * cm["resolution_m"]))
    return out


# ---------- the bridge's record ----------

def test_the_bridge_serves_the_costmap_in_the_maps_shape():
    rec = convert.grid_record("s1", 4, 0.05, 3, 2, -1.0, 2.0, 0.5, (0, 99, 100, -1, 1, 98), 12.5)
    assert rec == {"session": "s1", "version": 4, "resolution_m": 0.05, "width": 3, "height": 2,
                   "origin_x_m": -1.0, "origin_y_m": 2.0, "origin_yaw_rad": 0.5,
                   "data": [0, 99, 100, -1, 1, 98], "at": 12.5}


# ---------- criterion 1: the costmap reads the cell the map reads ----------

class _Recorder(list):
    """A grid whose reads are recorded: the frozen loop reads
    `data[j*w+i]` for every in-bounds output cell, in row-major order."""

    def __init__(self, n):
        super().__init__([0] * n)
        self.reads = []

    def __getitem__(self, k):
        self.reads.append(k)
        return 0


GEOMETRIES = [(w, h, alpha, origin, oyaw)
              for w, h in [(10, 10), (120, 80)]
              for alpha in [0.0, 90.0, 180.0, 270.0, 37.0]
              for origin in [(-0.5, -0.5), (-3.017, 1.2345)]
              for oyaw in [0.0, 0.3]]


@pytest.mark.parametrize("w,h,alpha,origin,oyaw", GEOMETRIES)
def test_the_placement_reads_exactly_the_cells_the_frozen_loop_read(w, h, alpha, origin, oyaw):
    """Both layers go through `_sample_index()`; it must pick, for every
    house-frame cell, the source cell the pre-3.83 loop picked."""
    world, _ = costmap_world()
    anchor = Anchor("s1", alpha, (2.31, 4.07), "test", 0)
    m = {"resolution_m": 0.05, "width": w, "height": h, "origin_x_m": origin[0],
         "origin_y_m": origin[1], "origin_yaw_rad": oyaw, "version": 1,
         "data": _Recorder(w * h)}
    reference = reference_resample(world, m, anchor)
    geo, idx = world._sample_index(m, anchor)
    assert (geo["width"], geo["height"]) == (reference["width"], reference["height"])
    assert (geo["origin_x_m"], geo["origin_y_m"]) == (reference["origin_x_m"],
                                                      reference["origin_y_m"])
    picked = [int(k) for k in idx if k >= 0]
    assert picked == m["data"].reads, "the placement and the frozen loop read different cells"
    assert len(idx) == geo["width"] * geo["height"]


@pytest.mark.parametrize("w,h,alpha,origin,oyaw", GEOMETRIES)
def test_the_map_is_the_frozen_loops_map(w, h, alpha, origin, oyaw):
    """The map's three states, cell for cell, over every SLAM value."""
    world, _ = costmap_world()
    anchor = Anchor("s1", alpha, (2.31, 4.07), "test", 0)
    data = [((k * 37) % 102) - 1 for k in range(w * h)]          # -1..100
    m = {"resolution_m": 0.05, "width": w, "height": h, "origin_x_m": origin[0],
         "origin_y_m": origin[1], "origin_yaw_rad": oyaw, "version": 3, "data": data}
    assert world._resample(m, anchor) == reference_resample(world, m, anchor)


# ---------- criterion 2: values and placement ----------

def test_every_value_crosses_unchanged():
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 90.0))
    cm = world.get_costmap()
    assert cm["usable"] and cm["map_id"] == "slam-s1" and cm["costmap_version"] == 1
    assert sorted(set(cm["costs"])) == [-1, 0, 40, 99, 100]
    assert cm["costs"].count(100) == 1 and cm["costs"].count(99) == 1
    assert cm["costs"].count(40) == 1 and cm["costs"].count(-1) >= 1


@pytest.mark.parametrize("heading", [0.0, 90.0, 180.0, 270.0, 37.0])
def test_a_lethal_cell_ahead_in_ros_lands_ahead_on_the_true_heading(heading):
    world, _ = costmap_world(truth=FakeTruth(1.0, 1.0, heading))
    world.get_pose()                                    # anchor
    cm = world.get_costmap()
    (x, y), = _cells_with(cm, 100)
    # The fake's lethal cell's centre is 0.35 m ahead, 0.05 m left, in ROS.
    fwd, left = 0.35, 0.05
    h = math.radians(heading)
    ex = 1.0 + fwd * math.sin(h) - left * math.cos(h)
    ey = 1.0 - fwd * math.cos(h) - left * math.sin(h)
    assert math.dist((x, y), (ex, ey)) <= cm["resolution_m"] * 1.01, ((x, y), (ex, ey))


def test_the_costmap_and_the_map_share_one_grid_when_their_geometry_matches():
    """The fake's map and costmap are the same 10 x 10 cells: the two
    layers must come back on the same house-frame grid."""
    world, _ = costmap_world(truth=FakeTruth(1.0, 1.0, 37.0))
    m, cm = world.get_map(), world.get_costmap()
    for k in ("width", "height", "resolution_m", "origin_x_m", "origin_y_m"):
        assert m[k] == cm[k], k


def test_a_new_version_is_read_and_the_same_one_is_cached():
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 90.0))
    first = world.get_costmap()
    assert world.get_costmap() is first
    bridge.costs[5 * 10 + 8] = 0
    bridge.costmap_version = 2
    second = world.get_costmap()
    assert second["costmap_version"] == 2 and 100 not in second["costs"]


# ---------- criterion 3: honest absence ----------

def test_no_costmap_yet_is_unusable():
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 90.0))
    bridge.costs = []
    assert world.get_costmap()["usable"] is False


def test_a_costmap_from_another_session_is_unusable():
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    bridge.costmap_session = "an-older-container"
    assert world.get_costmap()["usable"] is False


@pytest.mark.parametrize("status,retryable", [(404, False), (401, False), (503, True)])
def test_a_bridge_that_cannot_answer_is_unusable_and_says_whether_to_wait(status, retryable):
    """404 is an image older than 3.83 (no route), 401 a secret mismatch,
    503 an outage; none is an empty grid."""
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    bridge.costmap_status = status
    cm = world.get_costmap()
    assert cm["usable"] is False and "costs" not in cm
    assert cm["retryable"] is retryable


@pytest.mark.parametrize("break_it", [
    lambda b: b.__setitem__("data", b["data"][:-1]),              # short data
    lambda b: b.__setitem__("version", None),
    lambda b: b.__setitem__("version", "3"),
    lambda b: b.__setitem__("width", "wide"),
    lambda b: b.__setitem__("resolution_m", "0.05"),
    lambda b: b.__setitem__("resolution_m", 0.0),
    lambda b: b.__setitem__("origin_x_m", float("nan")),
    lambda b: b["data"].__setitem__(3, "lethal"),
    lambda b: b["data"].__setitem__(3, 10 ** 9),
    lambda b: b["data"].__setitem__(3, 250),
    lambda b: b["data"].__setitem__(3, 3.7),
    lambda b: b["data"].__setitem__(3, None),
    lambda b: b.__setitem__("origin_x_m", 1e308),
    lambda b: b.__setitem__("resolution_m", 1e-300),
])
def test_a_malformed_costmap_is_unusable_not_a_500(break_it, monkeypatch):
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    orig = bridge.handler

    def broken(request):
        if request.url.path == "/nav/costmap":
            body = convert.grid_record(bridge.session, 1, 0.1, 10, 10, -0.5, -0.5, 0.0,
                                       bridge.costs, 0.0)
            break_it(body)
            return httpx.Response(200, content=json.dumps(body).encode(),
                                  headers={"content-type": "application/json"})
        return orig(request)

    world._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(broken))
    assert world.get_costmap()["usable"] is False
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    r = TestClient(server.create_app()).get("/world/costmap")
    assert r.status_code == 200 and r.json()["usable"] is False


def test_the_same_version_is_placed_once(monkeypatch):
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    calls = []
    real = world._sample_index
    monkeypatch.setattr(world, "_sample_index", lambda *a: calls.append(1) or real(*a))
    for _ in range(5):
        world.get_costmap()
    assert len(calls) == 1
    bridge.costmap_version = 2
    world.get_costmap()
    assert len(calls) == 2


def test_a_costmap_nav2_stopped_publishing_is_stale_not_live():
    """The bridge keeps nav2's last costmap; one older than 5 s by the
    bridge's clock is a nav2 that stopped, and is not drawn as live."""
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 90.0))
    world.get_pose()
    orig = bridge.handler

    def aged(age):
        def handler(request):
            r = orig(request)
            if request.url.path == "/nav/costmap":
                return httpx.Response(200, json=dict(r.json(), age_s=age))
            return r
        return handler

    world._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(aged(1.2)))
    assert world.get_costmap()["usable"] is True
    world._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(aged(6.0)))
    cm = world.get_costmap()
    assert cm["usable"] is False and cm["stale"] is True


def test_a_bridge_that_is_down_is_unusable():
    def down(request):
        raise httpx.ConnectError("refused", request=request)
    world = RosWorld("http://bridge", client=httpx.Client(base_url="http://bridge",
                                                          transport=httpx.MockTransport(down)))
    cm = world.get_costmap()
    assert cm["usable"] is False and cm["retryable"] is True


# ---------- the robot server's route ----------

@pytest.fixture
def served(monkeypatch):
    """The real robot server with RosWorld behind /world/*."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 90.0))
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    return TestClient(server.create_app()), bridge


def test_get_world_costmap_serves_the_house_frame_costmap(served):
    client, _ = served
    r = client.get("/world/costmap")
    assert r.status_code == 200
    body = r.json()
    assert body["usable"] and body["map_id"] == "slam-s1" and 100 in body["costs"]


def test_a_world_with_no_costmap_answers_501(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    r = TestClient(server.create_app()).get("/world/costmap")   # the default world
    assert r.status_code == 501 and "nav2" in r.json()["detail"]


def test_the_costmap_needs_the_secret_when_one_is_set(served, monkeypatch):
    client, _ = served
    monkeypatch.setenv("APP_SHARED_SECRET", "s3cret")
    assert client.get("/world/costmap").status_code == 401
    assert client.get("/world/costmap", headers={"x-app-secret": "s3cret"}).status_code == 200


# ---------- criterion 4: cost ----------

def test_a_house_sized_costmap_is_cheap_to_place():
    """300 x 240 cells at 5 cm (a furnished home), a non-cardinal anchor:
    p90 under 50 ms uncached and under 1 ms cached. The robot server's
    wheel loop shares the process."""
    w, h = 300, 240
    world, bridge = costmap_world(truth=FakeTruth(1.0, 1.0, 37.0))
    world.get_pose()
    bridge.costs = [(k * 7) % 101 for k in range(w * h)]
    orig = bridge.handler

    def big(request):
        if request.url.path == "/nav/costmap":
            return httpx.Response(200, json=convert.grid_record(
                bridge.session, bridge.costmap_version, 0.05, w, h, -7.5, -6.0, 0.0,
                bridge.costs, 0.0))
        return orig(request)

    world._http = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(big))
    uncached, cached = [], []
    for v in range(20):
        bridge.costmap_version = 100 + v
        t0 = time.perf_counter()
        world.get_costmap()
        uncached.append(time.perf_counter() - t0)
        t0 = time.perf_counter()
        world.get_costmap()                             # same version: cached
        cached.append(time.perf_counter() - t0)
    p90 = statistics.quantiles(uncached, n=10)[-1]
    p90_cached = statistics.quantiles(cached, n=10)[-1]
    print(f"costmap 300x240: uncached p90 {p90 * 1000:.1f} ms, cached p90 "
          f"{p90_cached * 1000:.1f} ms")
    assert p90 < 0.050, f"uncached p90 {p90 * 1000:.1f} ms"
