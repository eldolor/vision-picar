"""
tests/test_tiered_explore.py

3.80.G (`docs/plans/ros-alignment/3.80-tiered-identity.md`): explore drives a
tiered mission's search. Explore's nav2 goals choose where to look; the tiers'
perception supplies the sightings; the last half metre is the tiers' steering
verbs; found comes only from the arrival rule and the identity verdict (local
first). In process, with `tests/test_explore.py`'s FakeNav standing in for
nav2 and the local tier and the cloud faked.
"""

import logging
import math

import pytest

from brain.explore import ExploreAgent
from brain.memory import MissionMemory
from brain.perceive import FrameReportedPipeline
from brain.tiered import TieredVision
from control.mission_runner import FOUND, MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav

TARGET = "red backpack"


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


class Cloud:
    def __init__(self):
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        return {"obstacles_ahead": [], "free_space": "clear", "doorway_visible": False,
                "important_objects": [], "safest_direction": "LEFT",
                "_navigate": {"reasoning": "cloud", "target_visible": True}}


class Local:
    """The local VLM: `score` for every question (identity and sighting)."""

    def __init__(self, score):
        self.score = score
        self.asked = 0

    def __call__(self, frame, box=False):
        self.asked += 1
        return {"status": "answered", "score": self.score}


def _mission(local_score=0.9, house="scaled_house", max_steps=200, **kw):
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    cloud, confirm, local = Cloud(), Cloud(), Local(local_score)
    tier = TieredVision(FrameReportedPipeline(TARGET), cloud, confirm_vision_fn=confirm,
                        local_identity_fn=local, local_sighting_fn=local,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(robot, target_object=TARGET, policy="tiered", vision_fn=tier,
                           tier_search="explore", navigator=nav, world=MockWorld(grid),
                           max_steps=max_steps, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, vision_timeout_s=5.0, **kw)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 5000:
        ticks += 1
    return runner, nav, grid, tier, cloud, confirm


def _target_distance_m(grid):
    cells = [c for c, name in grid.objects.items() if name == TARGET]
    return min(math.dist((grid.x, grid.y), (cx + 0.5, cy + 0.5)) for cx, cy in cells) * 0.3


def test_g_explore_finds_it_and_the_local_tier_confirms_with_no_cloud_call():
    runner, nav, grid, tier, cloud, confirm = _mission()
    s = runner.status()
    assert s["outcome"] == FOUND, s["log_tail"]
    assert s["tier_search"] == "explore"
    assert s["explore"]["goals_sent"] >= 2
    # Found through the arrival rule and the local yes, never the cloud.
    assert tier.stats.local_calls >= 1 and tier.stats.local.get("yes", 0) >= 1
    assert cloud.calls == 0 and confirm.calls == 0
    # ...and the robot is at the target on ground truth (the sim's 0.9 m).
    assert _target_distance_m(grid) <= 0.9
    # The last half metre was the tiers' verbs.
    assert runner.agent.close_ins >= 1


def test_g_a_local_no_is_never_found():
    runner, nav, grid, tier, cloud, confirm = _mission(local_score=0.0, max_steps=60)
    s = runner.status()
    assert s["outcome"] != FOUND
    assert tier.stats.local.get("no", 0) >= 1
    assert confirm.calls == 0


def test_g_the_simulators_detections_are_not_its_eyes():
    """Sightings come from the scene's perception: a frame that carries the
    simulator's detections but a scene whose perception saw nothing gives
    no sighting."""
    grid = build_world("scaled_house")
    grid.x, grid.y, grid.theta = 16.5, 4.5, math.radians(5)   # facing the backpack
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid), sight_from="perception")
    frame = robot.get_camera_frame()
    assert any(d.get("label") == TARGET for d in frame.get("detections") or ())
    agent._scene = {"_perception": {"status": "absent", "bearing_deg": None}}
    assert agent._sighting(frame) is None
    agent._scene = {"_perception": {"status": "detected", "bearing_deg": 2.0, "pan_deg": 0.0}}
    assert agent._sighting(frame) is not None


def test_g_the_explore_policy_still_reads_the_frame():
    grid = build_world("scaled_house")
    grid.x, grid.y, grid.theta = 16.5, 4.5, math.radians(5)
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    agent._scene = {}
    assert agent._sighting(robot.get_camera_frame()) is not None


def test_g_close_in_only_on_a_detection_and_only_steering_verbs():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid), sight_from="perception")
    seen = {"status": "detected", "bearing_deg": 10.0}
    # The scene's own direction is never what it runs (a cloud answer's, a
    # held goal's): the bearing decides. The arrival rule holding stops it.
    held = agent._close_in({"safest_direction": "LEFT", "_perception": seen,
                            "_arrival": {"state": "checking"}}, (1.0, 1.0))
    assert held[0] == "WAIT" and agent.close_ins == 0
    assert agent._close_in({"safest_direction": "FORWARD",
                            "_perception": {"status": "absent"}}, (1.0, 1.0)) is None
    out = agent._close_in({"safest_direction": "LEFT", "_perception": seen}, (1.0, 1.0))
    assert out[0] == "RIGHT" and agent.close_ins == 1
    centred = {"status": "detected", "bearing_deg": 32.0, "pan_deg": 30.0}
    assert agent._close_in({"_perception": centred}, (1.0, 1.0))[0] == "FORWARD"
    frame_agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                               navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                               world=MockWorld(grid))
    assert frame_agent._close_in({"safest_direction": "RIGHT", "_perception": seen}, (1.0, 1.0)) is None


def test_g_needs_a_navigator_and_a_known_search():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), Cloud())
    with pytest.raises(ValueError, match="navigator"):
        MissionRunner(robot, target_object=TARGET, policy="tiered", vision_fn=tier,
                      tier_search="explore")
    with pytest.raises(ValueError, match="tier_search"):
        MissionRunner(robot, target_object=TARGET, policy="tiered", vision_fn=tier,
                      tier_search="map")
    with pytest.raises(ValueError, match="sight_from"):
        ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                     navigator=FakeNav(robot), sight_from="radar")


def test_g_tier_search_cloud_is_todays_tiered():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(TARGET), Cloud())
    runner = MissionRunner(robot, target_object=TARGET, policy="tiered", vision_fn=tier)
    assert type(runner.agent).__name__ == "VisionAgent"
    assert runner.status()["tier_search"] == "cloud" and runner.status()["explore"] is None


def test_g_the_brain_builds_the_navigator_for_an_explore_search(monkeypatch):
    from control.brain_config import load_brain_config
    monkeypatch.setenv("TIER_SEARCH", "explore")
    assert load_brain_config()["tier_search"] == "explore"
    monkeypatch.setenv("TIER_SEARCH", "radar")
    with pytest.raises(ValueError, match="tier_search"):
        load_brain_config()


def test_g_in_sight_anywhere_in_the_approach_band_closes_in():
    """The first live run (procthor val 5) stood 0.6 m out with the target
    centred, and "here and not seeing it" turned it to face the target on
    every frame, forever. Anywhere in the band an approach can leave it, a
    sighting closes in."""
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid), sight_from="perception")
    pose = agent._pose()
    a = math.radians(pose["heading_deg"])
    for d, near in ((0.45, True), (0.65, True), (0.75, False)):
        tx, ty = pose["x_m"] + d * math.sin(a), pose["y_m"] - d * math.cos(a)
        assert agent._at_target(tx, ty) is near, d


# --- the reviews, 2026-10-16 -------------------------------------------------

def _agent(**kw):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    return ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                        navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                        world=MockWorld(grid), sight_from="perception", **kw), robot


def test_review_a_refused_identity_drops_the_place_for_good():
    agent, _ = _agent()
    agent._last_seen = (3.0, 3.0)
    agent._note_refusal({"_arrival": {"state": "refused"}})
    assert agent.retry.dropped(3.0, 3.0) and agent._last_seen is None
    # Once per refusal; a new arrival elsewhere is judged afresh.
    agent._note_refusal({"_arrival": {"state": "approaching"}})
    assert agent._refusal_noted is False


def test_review_a_local_no_mission_does_not_keep_asking():
    runner, nav, grid, tier, cloud, confirm = _mission(local_score=0.0, max_steps=60)
    # Each wrong place is asked about once per approach, not again and again.
    assert tier.stats.local_calls <= 4, tier.stats.local
    assert confirm.calls == 0


def test_review_a_close_in_that_waits_is_not_queued_as_a_bare_quarter_turn():
    from robot.interface import Preempted
    from brain.explore import OWN_GOAL_REFUSAL
    agent, robot = _agent()

    def refuse(*a, **k):
        raise Preempted(f"refused: {OWN_GOAL_REFUSAL}")

    agent.safety.check_and_execute = refuse
    out = agent._close_in({"_perception": {"status": "detected", "bearing_deg": 12.0}},
                          (1.0, 1.0))
    assert out[0] == "WAIT" and "RIGHT" not in agent._pending


def test_review_a_refused_small_turn_cannot_loop_forever():
    from robot.safety import SafetyViolation
    agent, robot = _agent()

    def refuse(*a, **k):
        raise SafetyViolation("Blocked RIGHT: pivot guard")

    agent.safety.check_and_execute = refuse
    seen = {"_perception": {"status": "detected", "bearing_deg": 4.0}}
    for _ in range(3):
        agent._pending.clear()
        agent._close_in(seen, (2.0, 2.0))
    # Three refusals: the sighting has a failure and the robot backs off.
    assert agent._pending[:1] == ["REVERSE"]
    assert agent.retry._near(2.0, 2.0) is not None


def test_review_the_local_vlms_located_yes_is_a_sighting():
    agent, robot = _agent()
    agent._scene = {"_perception": {"status": "absent"},
                    "_tier": {"local_landed": {"answer": "yes", "bearing_deg": 8.0}}}
    assert agent._hits(robot.get_camera_frame()) == [{"bearing_deg": 8.0}]
    agent._scene = {"_perception": {"status": "absent"},
                    "_tier": {"local_landed": {"answer": "no", "bearing_deg": None}}}
    assert agent._hits(robot.get_camera_frame()) == []


def test_review_under_an_explore_search_a_cloud_failure_ends_nothing():
    from brain.navigate import CloudUnavailable
    from tests.test_tiered_frames import P, Pipeline, frame
    from brain.perceive import ABSENT

    def down(frame):
        raise CloudUnavailable("refused")

    tier = TieredVision(Pipeline([P(ABSENT)]), down, cloud_errors_fatal=False)
    for _ in range(3):
        tier(frame())
    assert tier.stats.cloud_errors >= 1
    fatal = TieredVision(Pipeline([P(ABSENT)]), down)
    with pytest.raises(CloudUnavailable):
        fatal(frame())


def test_review_the_brain_refuses_an_explore_search_with_no_local_tier(monkeypatch):
    from control.brain_config import load_brain_config
    from control.brain_server import _tiered_vision_fn
    monkeypatch.setenv("TIER_SEARCH", "explore")
    monkeypatch.setenv("LOCAL_VLM_URL", "http://127.0.0.1:9")
    tier = _tiered_vision_fn("backpack", Cloud(), load_brain_config(), simulated=True)
    assert tier.cloud_errors_fatal is False
    tier.close()
    monkeypatch.setenv("TIER_SEARCH", "cloud")
    tier = _tiered_vision_fn("backpack", Cloud(), load_brain_config(), simulated=True)
    assert tier.cloud_errors_fatal is True
    tier.close()


def test_review_a_refused_explore_search_start_builds_no_tier_clients(tmp_path, monkeypatch):
    """No local tier: refused at start, before any tier client exists; and
    with one but no nav2, refused before the tier's clients are built too."""
    import control.brain_server as bs
    from fastapi.testclient import TestClient
    from tests.test_brain_server import (RecordingRobot, capture_tiered, fresh_mock_robot,
                                         tiered_config)
    monkeypatch.setattr(bs, "_validate_navigate_choices", lambda *a, **k: None)
    seen = capture_tiered(monkeypatch)
    monkeypatch.setenv("TIER_SEARCH", "explore")
    monkeypatch.delenv("LOCAL_VLM_URL", raising=False)
    app = bs.create_app(config_path=tiered_config(tmp_path),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        resp = client.post("/mission/start", json={"target_object": "red backpack",
                                                   "policy": "tiered"})
    assert resp.status_code == 400 and "local tier" in resp.json()["detail"]
    assert "target" not in seen
    monkeypatch.setenv("LOCAL_VLM_URL", "http://127.0.0.1:9")
    with TestClient(bs.create_app(config_path=tiered_config(tmp_path),
                                  robot_factory=lambda: RecordingRobot(fresh_mock_robot()))) as c:
        resp = c.post("/mission/start", json={"target_object": "red backpack",
                                              "policy": "tiered"})
    assert resp.status_code == 400 and "explore search" in resp.json()["detail"], resp.text
    assert "target" not in seen


# --- 3.85 at merge: its explore fixes beside 3.80.G's close-in ---------------

def _ahead(agent, d):
    pose = agent._pose()
    a = math.radians(pose["heading_deg"])
    return pose, (pose["x_m"] + d * math.sin(a), pose["y_m"] - d * math.cos(a))


def _stalled_approach(agent, d):
    pose, (tx, ty) = _ahead(agent, d)
    agent._goal = {"x_m": tx, "y_m": ty, "kind": "approach", "target": (tx, ty),
                   "sent_at": 5.0, "from": (pose["x_m"] - 0.5, pose["y_m"])}
    agent._goal_ended("stalled", 30.0)
    return tx, ty


def test_385_a_stalled_approach_in_the_band_is_a_plain_failure_not_an_escape():
    """3.85's stall cut backs a wedged goal out; in the band (perception
    sightings) that reversed and turned the robot away from a target 3.80.G's
    close-in finishes. It is a plain failure there, as its timeout was."""
    agent, _ = _agent()
    tx, ty = _stalled_approach(agent, 0.62)
    assert "REVERSE" not in agent._pending, agent._pending
    assert agent.retry.failures_near(tx, ty) == 1 and agent._approach is None


def test_385_a_stalled_approach_out_of_the_band_still_backs_out():
    agent, _ = _agent()
    _stalled_approach(agent, 2.0)
    assert agent._pending[:1] == ["REVERSE"]


def test_385_a_refused_approach_in_the_band_stays_on_record_for_close_in():
    """3.85 clears `_approach` on a refused send (the pantry livelock, frame
    sightings); in the band with perception sightings that re-sent a goal
    every frame instead of 3.80.G's close-in. Kept on record there."""
    from tests.test_explore import FirstRefusedNav
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                         navigator=FirstRefusedNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid), sight_from="perception")
    _, (tx, ty) = _ahead(agent, 0.62)
    out = agent._send_approach(tx, ty, grid.sim_time)
    assert out[0] != "GOAL" and agent._approach == (tx, ty), out
    # Out of the band, or with frame sightings, 3.85's rule stands.
    frame_agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                               navigator=FirstRefusedNav(robot), clock=lambda: grid.sim_time,
                               world=MockWorld(grid))
    frame_agent._send_approach(tx, ty, grid.sim_time)
    assert frame_agent._approach is None
