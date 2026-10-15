"""
tests/test_wedge_watch.py

PLAN-ros-alignment.md 3.68 -- when the safety layer keeps refusing nav2's
forward moves, the robot server marks a keepout spot ahead of the robot so
nav2's planner can route around it (robot/wedge_watch.py). Pinned here:

  * the watch: fires after HOLD_S of unbroken forward clamps, not on a
    brief one, not when a forward move goes through in between, not on a
    reverse or pivot clamp, and again only HOLD_S later;
  * the spot: ahead of the robot on the compass, in the house frame;
  * the wall: RosWorld converts spots to ROS's frame as it does goals;
  * the bridge's mask: discs drawn as nav2's KeepoutFilter reads them;
  * the server: a wedge while a goal is active reaches the bridge, and
    with no goal in progress it does not.
"""

import json
import math
import sys
import time
from pathlib import Path

import httpx
import pytest
from fastapi.testclient import TestClient

import robot.server as server
from robot import wedge_watch as ww
from robot.safety import SafetyController
from tests.test_ros_goals import GoalBridge, world_with
from tests.test_ros_world import FakeTruth

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"))
from picar_bridge import keepout  # noqa: E402

CLAMP = "forward clamped: 12cm <= 23cm (scan)"


# ---------- the watch ----------

def feed(watch, t0, t1, step=0.05, left=2.0, right=2.0, reason=CLAMP):
    fired = []
    for k in range(int(round((t1 - t0) / step)) + 1):
        t = round(t0 + k * step, 6)
        if watch.wheel(t, left, right, reason):
            fired.append(round(t, 2))
    return fired


def test_three_seconds_of_forward_clamps_is_a_wedge():
    w = ww.WedgeWatch()
    assert feed(w, 0.0, 2.9) == []
    assert feed(w, 2.95, 3.1) == [3.0]


def test_it_fires_again_only_a_hold_later():
    w = ww.WedgeWatch()
    assert feed(w, 0.0, 6.5) == [3.0, 6.0]


def test_a_forward_move_that_goes_through_resets_it():
    w = ww.WedgeWatch()
    feed(w, 0.0, 2.5)
    assert not w.wheel(2.55, 2.0, 2.0, None)            # drove forward
    assert feed(w, 2.6, 5.5) == []                       # a fresh run from 2.6
    assert feed(w, 5.55, 5.7) == [5.6]


def test_a_gap_longer_than_gap_s_starts_a_new_run():
    w = ww.WedgeWatch()
    feed(w, 0.0, 2.0)
    assert feed(w, 3.5, 6.4) == []                       # 1.5 s of silence
    assert feed(w, 6.45, 6.6) == [6.5]


def test_turning_in_place_between_clamps_is_not_progress():
    w = ww.WedgeWatch()
    for i in range(70):
        t = i * 0.05
        if i % 2:
            assert not w.wheel(t, -1.0, 1.0, None)       # a pivot: no forward command
        elif w.wheel(t, 2.0, 2.0, CLAMP):
            assert t >= ww.HOLD_S
            return
    pytest.fail("a robot turning between refused forward moves is still wedged")


@pytest.mark.parametrize("left,right,reason", [
    (-2.0, -2.0, "reverse clamped: 4cm <= 23cm (scan rear)"),
    (-1.0, 1.0, "pivot slowed: 1.0cm"),
    (2.0, 2.0, "pivot slowed: 1.0cm"),                   # a pivot clamp alone: forward went through
])
def test_only_a_refused_forward_move_counts(left, right, reason):
    assert feed(ww.WedgeWatch(), 0.0, 5.0, left=left, right=right, reason=reason) == []


# ---------- the spot ----------

@pytest.mark.parametrize("heading,expected", [
    (0.0, (1.0, 0.7)), (90.0, (1.3, 1.0)), (180.0, (1.0, 1.3)), (270.0, (0.7, 1.0))])
def test_the_spot_is_ahead_on_the_compass(heading, expected):
    assert ww.spot_ahead(1.0, 1.0, heading) == pytest.approx(expected)


def test_spots_expire_and_are_capped():
    book = ww.SpotBook(ttl_s=10.0, max_spots=2)
    book.add(0.0, 1.0, 1.0)
    book.add(1.0, 2.0, 2.0)
    live = book.add(2.0, 3.0, 3.0)
    assert [s["x_m"] for s in live] == [2.0, 3.0]        # the oldest dropped
    assert [s["x_m"] for s in book.live(11.5)] == [3.0]  # the second expired
    assert book.live(11.5)[0]["ttl_s"] == pytest.approx(0.5)
    assert book.live(11.98)[0]["ttl_s"] == 0.1             # never 0: the bridge refuses it


def test_the_spot_clears_the_chassis_it_is_marked_for():
    """The disc must not reach under the robot's own footprint, or the
    robot stands on lethal cells the moment it is marked."""
    half_len, half_w = 0.1265, 0.1155
    nearest = math.hypot(ww.SPOT_AHEAD_M - half_len, 0.0)      # the nose's centre
    corner = math.hypot(ww.SPOT_AHEAD_M - half_len, half_w)
    assert min(nearest, corner) > ww.SPOT_R_M


# ---------- the wall ----------

class KeepoutBridge(GoalBridge):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.keepout = None

    def handler(self, request):
        if request.url.path == "/nav/keepout":
            self.keepout = json.loads(request.content)["spots"]
            return httpx.Response(200, json={"spots": len(self.keepout)})
        return super().handler(request)


def test_a_spot_crosses_the_wall_in_ros_frame():
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0), bridge=KeepoutBridge(
        start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0}))
    assert world.set_keepout([{"x_m": 1.0, "y_m": 0.5, "r_m": 0.15, "ttl_s": 300}])["accepted"]
    s = bridge.keepout[0]
    # Facing east, 0.5 m north is 0.5 m to the LEFT: ROS +y.
    assert (s["x_m"], s["y_m"]) == pytest.approx((0.0, 0.5))
    assert (s["r_m"], s["ttl_s"]) == (0.15, 300)


# ---------- the bridge's mask ----------

def test_the_mask_draws_each_disc_and_nothing_else():
    spots = keepout.parse([{"x_m": 1.0, "y_m": -2.0, "r_m": 0.15, "ttl_s": 5}], now=0.0)
    m = keepout.mask(spots)
    w, res = m["width"], m["resolution_m"]
    marked = [((i + 0.5) * res + m["origin_x_m"], (j + 0.5) * res + m["origin_y_m"])
              for j in range(m["height"]) for i in range(w) if m["data"][j * w + i] == 100]
    assert marked and all(math.hypot(x - 1.0, y + 2.0) <= 0.15 for x, y in marked)
    assert set(m["data"]) == {0, 100}
    assert len(marked) == pytest.approx(math.pi * 0.15 ** 2 / res ** 2, rel=0.3)


def test_an_empty_set_is_still_a_mask_so_it_replaces_the_last():
    assert keepout.mask([])["data"] == [0]


def test_spots_expire_in_the_bridge():
    spots = keepout.parse([{"x_m": 0, "y_m": 0, "r_m": 0.1, "ttl_s": 5},
                           {"x_m": 1, "y_m": 0, "r_m": 0.1, "ttl_s": 50}], now=100.0)
    assert [s["x_m"] for s in keepout.live(spots, 106.0)] == [1.0]


@pytest.mark.parametrize("bad", [
    {"x_m": 0, "y_m": 0, "r_m": 0, "ttl_s": 5},
    {"x_m": 0, "y_m": 0, "r_m": 5.0, "ttl_s": 5},
    {"x_m": "nan", "y_m": 0, "r_m": 0.1, "ttl_s": 5},
    {"x_m": 0, "y_m": 0, "r_m": 0.1},
])
def test_a_malformed_spot_is_refused_not_drawn(bad):
    with pytest.raises((ValueError, KeyError)):
        keepout.parse([bad], now=0.0)


def test_keepout_module_needs_no_rclpy():
    src = Path(keepout.__file__).read_text()
    assert "import rclpy" not in src and "from rclpy" not in src


# ---------- the server ----------

@pytest.fixture
def wedged_server(monkeypatch):
    """The real robot server, RosWorld behind it, every forward wheel
    command refused, and a hold short enough for a test."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    truth = FakeTruth(1.0, 1.0, 90.0)
    bridge = KeepoutBridge(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    world, _ = world_with(truth=truth, bridge=bridge)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    monkeypatch.setattr(server, "WedgeWatch", lambda: ww.WedgeWatch(hold_s=0.2))
    monkeypatch.setattr(SafetyController, "vet_wheel_velocity",
                        lambda self, l, r: (0.0, 0.0, CLAMP) if l + r > 0 else (l, r, None))
    return TestClient(server.create_app()), bridge


def drive_refused(client, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        client.post("/wheels", json={"left_rad_s": 2.0, "right_rad_s": 2.0},
                    headers={"x-driver": "ros"})
        time.sleep(0.02)


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not cond():
        time.sleep(0.02)
    return cond()


def test_a_wedge_during_a_goal_marks_a_spot_ahead(wedged_server):
    client, bridge = wedged_server
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    drive_refused(client, 0.5)
    assert wait_for(lambda: bridge.keepout)
    # The robot stands at the session's start (ROS origin) facing ROS +x:
    # the spot is SPOT_AHEAD_M straight ahead.
    s = bridge.keepout[0]
    assert (s["x_m"], s["y_m"]) == pytest.approx((ww.SPOT_AHEAD_M, 0.0), abs=1e-3)
    assert s["r_m"] == ww.SPOT_R_M


def test_a_mark_without_a_slam_session_is_not_logged_as_marked(wedged_server, monkeypatch, caplog):
    client, bridge = wedged_server
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    world = server.get_world()
    monkeypatch.setattr(world, "set_keepout", lambda spots: {"accepted": False,
                                                             "reason": "no SLAM session yet"})
    with caplog.at_level("WARNING", logger="server"):
        drive_refused(client, 0.5)
        assert wait_for(lambda: "keepout: not marked" in caplog.text)
    assert "keepout: marked" not in caplog.text


def test_no_goal_in_progress_marks_nothing(wedged_server):
    client, bridge = wedged_server
    drive_refused(client, 0.5)
    time.sleep(0.2)
    assert bridge.keepout is None
