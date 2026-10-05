"""
tests/test_ros_goals.py

R6's goal path and R5's error readout, in-process, against a fake bridge.

Written 2026-09-27 to close a gap found while auditing the ROS tests: the
tap-to-goal chain -- `POST /world/goal` on the robot server -> RosWorld's
house-to-ROS conversion -> the bridge's `/goal` -- was only exercised by
`tests/demo_nav_goals.py` and the live suite, both of which need the
container. The conversion is the part that can be silently wrong (a
mirrored goal is a robot driving confidently to the wrong room), so it is
pinned here without ROS, the way tests/test_ros_world.py pins the pose.
"""

import math

import httpx
import pytest
from fastapi.testclient import TestClient

import robot.server as server
from tests.test_ros_world import FakeBridge, FakeTruth
from world.ros_world import RosWorld


class GoalBridge(FakeBridge):
    """FakeBridge plus picar_bridge's /goal routes, in ROS's frame."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.goal = None
        self.plan = []
        self.goal_session = None
        self.cancelled = 0

    def handler(self, request):
        if request.url.path == "/goal":
            if request.method == "POST":
                import json
                body = json.loads(request.content)
                self.goal = {**body, "state": "active"}
                self.goal_session = self.session
                return httpx.Response(200, json={"state": "sent"})
            if request.method == "DELETE":
                self.cancelled += 1
                self.goal = None
                return httpx.Response(200, json={"cancelled": True})
            return httpx.Response(200, json={"session": self.goal_session,
                                             "goal": self.goal, "plan": self.plan})
        return super().handler(request)


def world_with(truth=None, bridge=None):
    bridge = bridge or GoalBridge(
        start_truth={k: truth.pose[k] for k in ("x_m", "y_m", "heading_deg")} if truth else None)
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge.handler))
    return RosWorld("http://bridge", truth=truth, client=client), bridge


# ---------- RosWorld: the conversion ----------

def test_a_goal_straight_ahead_reaches_nav2_as_ros_plus_x():
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))    # facing east
    assert world.set_goal(2.0, 1.0)["accepted"]                     # 1 m east = ahead
    assert bridge.goal["x_m"] == pytest.approx(1.0) and bridge.goal["y_m"] == pytest.approx(0.0)


def test_a_goal_to_the_left_reaches_nav2_as_ros_plus_y():
    """Facing east, north (smaller y_m) is LEFT, which is ROS +y. A y-flip
    dropped from `_ours_to_ros` sends the robot to the mirror-image room;
    the straight-ahead case above cannot see that."""
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))
    world.set_goal(1.0, 0.5)
    assert bridge.goal["x_m"] == pytest.approx(0.0, abs=1e-9)
    assert bridge.goal["y_m"] == pytest.approx(0.5)


@pytest.mark.parametrize("heading", [0.0, 90.0, 180.0, 270.0, 37.0])
@pytest.mark.parametrize("goal", [(2.0, 1.0), (1.0, 0.2), (0.3, 2.4)])
def test_a_goal_comes_back_where_it_was_sent(heading, goal):
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, heading))
    world.set_goal(*goal)
    g = world.get_goal()["goal"]
    assert (g["x_m"], g["y_m"]) == pytest.approx(goal, abs=1e-9)
    assert "yaw_rad" not in g, "a ROS-frame angle must not cross the wall"


def test_the_plan_is_converted_point_by_point():
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))
    world.set_goal(2.0, 1.0)
    bridge.plan = [(0.0, 0.0), (0.5, 0.5)]           # ahead-and-left in ROS
    plan = world.get_goal()["plan"]
    assert plan[0] == pytest.approx([1.0, 1.0])
    assert plan[1] == pytest.approx([1.5, 0.5]), "left of east-facing is north"


def test_a_goal_from_an_earlier_container_is_not_shown():
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))
    world.set_goal(2.0, 1.0)
    bridge.session = "s2"                             # the container restarted
    world.get_pose()
    assert world.get_goal() == {"goal": None, "plan": []}


def test_no_goal_is_sent_before_there_is_a_map_to_send_it_on():
    class Dead(GoalBridge):
        def handler(self, request):
            if request.url.path == "/slam/pose":
                raise httpx.ConnectError("no container")
            return super().handler(request)
    world, bridge = world_with(bridge=Dead())
    reply = world.set_goal(2.0, 1.0)
    assert reply["accepted"] is False and "session" in reply["reason"]
    assert bridge.goal is None


def test_cancel_reaches_the_bridge():
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))
    world.set_goal(2.0, 1.0)
    assert world.cancel_goal() == {"cancelled": True} and bridge.cancelled == 1


# ---------- the robot server's routes ----------

@pytest.fixture
def served(monkeypatch):
    """The real robot server with RosWorld behind /world/*."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    truth = FakeTruth(1.0, 1.0, 90.0)
    world, bridge = world_with(truth=truth)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    return TestClient(server.create_app()), bridge, truth


def test_post_world_goal_drives_the_conversion(served):
    client, bridge, _ = served
    r = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    assert r.status_code == 200 and r.json()["accepted"]
    assert bridge.goal["x_m"] == pytest.approx(1.0)
    g = client.get("/world/goal").json()["goal"]
    assert (g["x_m"], g["y_m"]) == pytest.approx((2.0, 1.0))
    assert client.delete("/world/goal").json()["cancelled"] is True


def test_a_world_that_cannot_plan_answers_501_not_a_silent_no_op(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = TestClient(server.create_app())           # the default world
    for method in ("post", "get", "delete"):
        kw = {"json": {"x_m": 1.0, "y_m": 1.0}} if method == "post" else {}
        r = getattr(client, method)("/world/goal", **kw)
        assert r.status_code == 501, (method, r.status_code)
        assert "nav2" in r.json()["detail"]


def test_goals_need_the_secret_when_one_is_set(served, monkeypatch):
    client, bridge, _ = served
    monkeypatch.setenv("APP_SHARED_SECRET", "s3cret")
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}).status_code == 401
    assert bridge.goal is None
    ok = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}, headers={"x-app-secret": "s3cret"})
    assert ok.status_code == 200


def test_world_error_measures_slam_and_odometry_against_the_truth(served):
    client, bridge, truth = served
    client.get("/world/pose")                           # anchor at the session start
    bridge.map_pose = {"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}         # SLAM: 0.5 m ahead
    bridge.odom_pose = {"x_m": 0.6, "y_m": 0.0, "yaw_rad": math.radians(-3)}  # odom: 0.6, 3 deg right
    truth.pose.update(x_m=1.52)                          # truly 0.52 m
    e = client.get("/world/error").json()
    assert e["usable"] and e["source"] == "slam-s1"
    assert e["position_error_m"] == pytest.approx(0.02, abs=1e-6)
    assert e["odom_position_error_m"] == pytest.approx(0.08, abs=1e-6)
    assert e["odom_heading_error_deg"] == pytest.approx(3.0, abs=1e-3)


def test_world_error_is_unusable_without_a_truth(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    world, _ = world_with(truth=None)                   # hardware: no truth
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    e = TestClient(server.create_app()).get("/world/error").json()
    assert e["usable"] is False and e["position_error_m"] is None


# ---------- handoff 4d: the goal routes when the bridge is down ----------
# They used to answer 500 (no except around httpx). Now they refuse by name.
# RosWorld itself still RAISES -- the stop's goal-ending loop
# (robot/server.py _end_goal_after_stop) retries on exactly that.

class _DeadBridge(GoalBridge):
    def handler(self, request):
        if request.url.path == "/goal":
            raise httpx.ConnectError("bridge gone", request=request)
        return super().handler(request)


@pytest.fixture
def dead_goal_routes(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    world, _ = world_with(truth=FakeTruth(1.0, 1.0, 90.0), bridge=_DeadBridge(
        start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0}))
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    return TestClient(server.create_app())


def test_setting_a_goal_with_the_bridge_down_is_refused_by_name(dead_goal_routes):
    r = dead_goal_routes.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["accepted"] is False and body["reason"] == "ros_unavailable", body


def test_reading_and_cancelling_with_the_bridge_down_are_refused_by_name(dead_goal_routes):
    for method in ("get", "delete"):
        r = getattr(dead_goal_routes, method)("/world/goal")
        assert r.status_code == 503, (method, r.status_code, r.text)
        assert "ros_unavailable" in r.text


class _RefusingBridge(GoalBridge):
    def handler(self, request):
        if request.url.path == "/goal" and request.method in ("GET", "DELETE"):
            return httpx.Response(401, json={"error": "bad secret"})
        return super().handler(request)


def test_rosworld_raises_on_an_error_status_rather_than_reporting_it_as_an_answer():
    """V28 (spec review 3): a 401 came back as a 200 body, so a cancel the
    bridge refused 'succeeded' and the stop's loop stopped trying."""
    world, _ = world_with(truth=FakeTruth(1.0, 1.0, 90.0), bridge=_RefusingBridge(
        start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0}))
    with pytest.raises(httpx.HTTPStatusError):
        world.cancel_goal()
    with pytest.raises(httpx.HTTPStatusError):
        world.get_goal()
