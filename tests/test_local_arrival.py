"""
tests/test_local_arrival.py

3.80.O (`docs/plans/ros-alignment/3.80-tiered-identity.md`): under explore's
tiered search, the local VLM's box finishes the approach and judges the
arrival when perception cannot see the target; an arrival it led to is
confirmed by the cloud, never by the local VLM itself. In process, with
FakeNav for nav2, perception blind to the target, and stand-ins for the local
VLM and the cloud that read the simulator's report.
"""

import logging
import math

import pytest

from brain.arrival import APPROACHING, ARRIVED, SOURCE_LOCAL, SOURCE_PERCEPTION, ArrivalCheck
from brain.navigate import CloudUnavailable
from brain.perceive import FrameReportedPipeline
from brain.tiered import TRIGGER_TRACK, TieredVision
from control.mission_runner import ARRIVED_UNCONFIRMED, FOUND, MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav

TARGET = "red backpack"
HFOV = 60.0


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


def _seen(frame):
    return [d for d in frame.get("detections") or () if d.get("label") == TARGET
            and d.get("bearing_deg") is not None]


class TruthLocal:
    """The local VLM, answering from the simulator's report: a yes with a
    box at the target's bearing when it is in view, else a no."""

    def __init__(self):
        self.asked = 0

    def __call__(self, frame, box=False):
        self.asked += 1
        seen = _seen(frame)
        if not seen:
            return {"status": "answered", "score": 0.0}
        x = seen[0]["bearing_deg"] / HFOV + 0.5
        return {"status": "answered", "score": 0.95, "box_frac": [x - 0.02, 0.4, x + 0.02, 0.6]}


class TruthCloud:
    """The cloud: sees the target iff the simulator reports it in the frame."""

    def __init__(self, answer=None, raises=False):
        self.calls = 0
        self.answer = answer
        self.raises = raises

    def __call__(self, frame):
        self.calls += 1
        if self.raises:
            raise CloudUnavailable("refused (test)")
        visible = bool(_seen(frame)) if self.answer is None else self.answer
        return {"obstacles_ahead": [], "free_space": "clear", "doorway_visible": False,
                "important_objects": [], "safest_direction": "LEFT",
                "_navigate": {"reasoning": "cloud", "target_visible": visible}}


def _blind_mission(cloud=None, max_steps=400, house="scaled_house", async_cloud=False, **kw):
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    pipeline = FrameReportedPipeline(TARGET, blind=True)
    pipeline.hfov_deg = HFOV
    sighting, identity = TruthLocal(), TruthLocal()
    cloud = cloud or TruthCloud()
    tier = TieredVision(pipeline, TruthCloud(answer=False), confirm_vision_fn=cloud,
                        local_identity_fn=identity, local_sighting_fn=sighting,
                        async_cloud=async_cloud)
    runner = MissionRunner(robot, target_object=TARGET, policy="tiered", vision_fn=tier,
                           tier_search="explore", navigator=FakeNav(robot),
                           world=MockWorld(grid), max_steps=max_steps,
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           vision_timeout_s=5.0, **kw)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 6000:
        ticks += 1
    return runner, grid, tier, sighting, identity, cloud


def _target_distance_m(grid):
    cells = [c for c, name in grid.objects.items() if name == TARGET]
    return min(math.dist((grid.x, grid.y), (cx + 0.5, cy + 0.5)) for cx, cy in cells) * 0.3


# ---------- the arrival rule's second source ----------

class _Scan:
    def __init__(self, r):
        self.r = r

    def get_scan(self):
        return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
                "ranges_m": [self.r] * 360}


def _scene(local=None, detected_bearing=None):
    p = ({"status": "detected", "bearing_deg": detected_bearing, "pan_deg": 0.0}
         if detected_bearing is not None else {"status": "absent", "bearing_deg": None})
    landed = None
    if local == "no":
        landed = {"answer": "no", "bearing_deg": None}
    elif local is not None:
        landed = {"answer": "yes", "bearing_deg": local}
    return {"_perception": p, "_tier": {"local_landed": landed}}


def test_o_two_local_answers_within_the_window_arrive_and_say_local():
    t = [0.0]
    check = ArrivalCheck(clock=lambda: t[0])
    robot = _Scan(0.3)
    assert check.observe(_scene(local=1.0), robot)["state"] == APPROACHING
    t[0] = 0.4
    held = check.observe(_scene(), robot)          # between answers: held, not missed
    assert held["state"] == APPROACHING and held["streak"] == 1
    t[0] = 1.2
    r = check.observe(_scene(local=-0.5), robot)
    assert r["state"] == ARRIVED and r["source"] == SOURCE_LOCAL
    t[0] = 2.0
    assert check.observe(_scene(), robot)["state"] == ARRIVED   # held arrived


def test_o_a_stale_answer_does_not_run_on_and_a_no_resets():
    t = [0.0]
    check = ArrivalCheck(clock=lambda: t[0])
    robot = _Scan(0.3)
    check.observe(_scene(local=0.0), robot)
    t[0] = 3.5                                      # past LOCAL_FRESH_S
    assert check.observe(_scene(), robot)["streak"] == 0
    r = check.observe(_scene(local=0.0), robot)
    assert r["state"] == APPROACHING and r["streak"] == 1
    t[0] = 4.0
    assert check.observe(_scene(local="no"), robot)["streak"] == 0


def test_o_far_or_off_centre_local_answers_never_arrive():
    t = [0.0]
    check = ArrivalCheck(clock=lambda: t[0])
    for i in range(4):
        t[0] = i * 0.5
        assert check.observe(_scene(local=0.0), _Scan(1.5))["state"] == APPROACHING
        assert check.observe(_scene(local=8.0), _Scan(0.3))["state"] == APPROACHING


def test_o_without_a_clock_the_rule_reads_perception_alone():
    check = ArrivalCheck()
    for _ in range(3):
        assert check.observe(_scene(local=0.0), _Scan(0.3))["state"] == APPROACHING
    check.observe(_scene(detected_bearing=0.0), _Scan(0.3))
    r = check.observe(_scene(detected_bearing=0.0), _Scan(0.3))
    assert r["state"] == ARRIVED and r["source"] == SOURCE_PERCEPTION


def test_o_a_streak_with_any_local_frame_is_local():
    t = [0.0]
    check = ArrivalCheck(clock=lambda: t[0])
    check.observe(_scene(detected_bearing=0.0), _Scan(0.3))
    t[0] = 0.5
    r = check.observe(_scene(local=0.0), _Scan(0.3))
    assert r["state"] == ARRIVED and r["source"] == SOURCE_LOCAL


# ---------- the tier's local_track trigger ----------

def test_o_track_asks_the_local_vlm_every_frame_and_never_the_cloud():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    pipeline = FrameReportedPipeline(TARGET, blind=True)
    cloud = TruthCloud(answer=False)
    asked = []

    def unable(frame, box=False):
        asked.append(1)
        # The opening question answered (no), every later one unable.
        return {"status": "answered", "score": 0.0} if len(asked) == 1 else {"status": "unavailable"}

    tier = TieredVision(pipeline, cloud, local_identity_fn=unable, local_sighting_fn=unable)
    tier(robot.get_camera_frame())                 # the opening question
    tier.set_local_track(True)
    before = len(asked)
    for _ in range(6):
        tier(robot.get_camera_frame())
    assert tier.stats.local_triggers.get(TRIGGER_TRACK, 0) >= 1
    assert len(asked) > before
    # A tracking question is never handed to the cloud; after two unable
    # answers tracking stops and the usual triggers decide (review).
    assert TRIGGER_TRACK not in tier.stats.triggers
    assert tier.stats.local_triggers[TRIGGER_TRACK] <= tier.local_max_unable


def test_o_blind_perception_never_reports_the_target():
    grid = build_world("scaled_house")
    grid.x, grid.y, grid.theta = 16.5, 4.5, math.radians(5)   # facing the backpack
    frame = MockRobot(grid, render=False).get_camera_frame()
    assert _seen(frame)
    assert FrameReportedPipeline(TARGET).perceive(frame).status == "detected"
    assert FrameReportedPipeline(TARGET, blind=True).perceive(frame).status == "absent"


# ---------- whole missions ----------

def test_o_the_local_box_alone_finds_it_and_only_the_cloud_confirms():
    runner, grid, tier, sighting, identity, cloud = _blind_mission()
    s = runner.status()
    assert s["outcome"] == FOUND, s["log_tail"]
    assert _target_distance_m(grid) <= 0.9
    assert identity.asked == 0                      # never the local VLM's own word
    assert cloud.calls >= 1 and tier.stats.guarded >= 1
    assert runner.agent.close_ins >= 1


def test_o_a_cloud_no_is_never_found():
    runner, grid, tier, sighting, identity, cloud = _blind_mission(
        cloud=TruthCloud(answer=False), max_steps=120)
    s = runner.status()
    assert s["outcome"] != FOUND
    assert identity.asked == 0
    assert tier.stats.guarded >= 1


def test_o_a_cloud_outage_ends_arrived_unconfirmed_never_found():
    runner, grid, tier, sighting, identity, cloud = _blind_mission(
        cloud=TruthCloud(raises=True))
    s = runner.status()
    assert s["outcome"] == ARRIVED_UNCONFIRMED, s["log_tail"]
    assert identity.asked == 0


def test_o_async_the_local_box_tracks_and_finds_it():
    """The brain runs the tier asynchronously: an answer lands a frame or more
    after its question, so the close-in waits for each one and tracks."""
    runner, grid, tier, sighting, identity, cloud = _blind_mission(async_cloud=True)
    s = runner.status()
    assert s["outcome"] == FOUND, s["log_tail"]
    assert _target_distance_m(grid) <= 0.9
    assert identity.asked == 0 and tier.stats.guarded >= 1


# ---------- review fixes ----------

def test_review_a_late_check_of_a_local_arrival_asks_the_cloud_not_the_local_vlm():
    cloud = TruthCloud(raises=True)
    runner, grid, tier, sighting, identity, _ = _blind_mission(cloud=cloud)
    assert runner.status()["outcome"] == ARRIVED_UNCONFIRMED
    assert runner._arrival.get("source") == SOURCE_LOCAL
    cloud.raises = False
    guarded = tier.stats.guarded
    verdict = runner.late_ask()
    assert verdict is not None and verdict.get("tier") == "cloud" and verdict["cloud_called"]
    assert tier.stats.guarded == guarded + 1 and identity.asked == 0


def test_review_the_streak_freshness_runs_from_its_last_frame_not_its_last_local_one():
    t = [0.0]
    check = ArrivalCheck(clock=lambda: t[0])
    robot = _Scan(0.3)
    check.observe(_scene(local=0.0), robot)
    for i in range(1, 11):                          # perception holds it for 5 s
        t[0] = i * 0.5
        check.observe(_scene(detected_bearing=0.0), robot)
    t[0] = 5.2
    r = check.observe(_scene(local=0.0), robot)
    assert r["state"] == ARRIVED and r["streak"] == 12


def test_review_a_panned_frame_is_not_held():
    t = [0.0]
    check = ArrivalCheck(clock=lambda: t[0])
    robot = _Scan(0.3)
    check.observe(_scene(local=0.0), robot)
    t[0] = 0.5
    scene = _scene()
    scene["_perception"]["pan_deg"] = 30.0
    assert check.observe(scene, robot)["state"] == "not_judged"


def test_review_an_unranged_centred_yes_does_not_park_the_search():
    from brain.explore import LOCAL_WAIT_MAX_S, ExploreAgent
    from brain.memory import MissionMemory
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    t = [0.0]
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                         navigator=FakeNav(robot), clock=lambda: t[0],
                         world=MockWorld(grid), sight_from="perception")
    agent._sighting_now = lambda frame: None       # the lidar cannot range it
    frame = robot.get_camera_frame()
    scene = {"_perception": {"status": "absent", "bearing_deg": None},
             "_tier": {"local_landed": {"answer": "yes", "bearing_deg": 0.5}}}
    run, longest = 0, 0
    for i in range(int(6 * LOCAL_WAIT_MAX_S)):
        t[0] = float(i)
        action, _, detail = agent._choose(frame, scene)
        agent._update_track()
        waiting = action == "WAIT" and "local tier" in str(detail)
        run = run + 1 if waiting else 0
        longest = max(longest, run)
    assert longest >= 1                              # it does wait for the next answer...
    assert longest <= LOCAL_WAIT_MAX_S + 1           # ...but not for the whole time limit


def _agent_at(t):
    from brain.explore import ExploreAgent
    from brain.memory import MissionMemory
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=TARGET),
                         navigator=FakeNav(robot), clock=lambda: t[0],
                         world=MockWorld(grid), sight_from="perception")
    agent._sighting_now = lambda frame: None       # the lidar cannot range it
    return agent, robot.get_camera_frame()


YES = {"_perception": {"status": "absent", "bearing_deg": None},
       "_tier": {"local_landed": {"answer": "yes", "bearing_deg": 0.5}}}
NOTHING = {"_perception": {"status": "absent", "bearing_deg": None}, "_tier": {}}


def test_review2_the_wait_cap_runs_a_stretch_at_a_time_and_from_the_first_yes():
    from brain.explore import LOCAL_FRESH_S, LOCAL_WAIT_MAX_S
    t = [0.0]
    agent, frame = _agent_at(t)
    agent._goal = None
    agent._choose(frame, NOTHING)                   # tick 0: a frontier goal
    agent._goal = None
    t[0] = 1.0
    action, _, detail = agent._choose(frame, YES)   # the first yes: waits at once
    assert action == "WAIT" and "local tier" in detail
    # A long stretch away and back: the cap starts again.
    t[0] = 1.0 + LOCAL_WAIT_MAX_S + LOCAL_FRESH_S + 5
    agent._choose(frame, NOTHING)
    agent._goal = None
    t[0] += 1.0
    action, _, detail = agent._choose(frame, YES)
    assert action == "WAIT" and "local tier" in detail


def test_review2_no_tracking_while_an_arrival_is_judged():
    t = [0.0]
    agent, frame = _agent_at(t)
    calls = []
    agent._local_track_fn = calls.append
    agent._choose(frame, YES)
    agent._update_track()
    assert calls == [True]
    agent._scene = {**YES, "_arrival": {"state": "checking"}}
    agent._update_track()
    assert calls == [True, False]


def test_review2_a_held_arrival_ends_when_the_lidar_says_so():
    t = [0.0]
    check = ArrivalCheck(clock=lambda: t[0])
    check.observe(_scene(local=0.0), _Scan(0.3))
    t[0] = 0.5
    assert check.observe(_scene(local=0.0), _Scan(0.3))["state"] == ARRIVED
    t[0] = 0.8
    assert check.observe(_scene(), _Scan(0.3))["state"] == ARRIVED      # held
    t[0] = 1.0
    assert check.observe(_scene(), _Scan(1.2))["state"] == APPROACHING  # moved away


def test_review2_a_stop_during_the_wait_pays_nothing():
    cloud = TruthCloud()
    runner, grid, tier, sighting, identity, _ = _blind_mission(cloud=cloud)
    before = cloud.calls
    verdict = runner._guarded_confirm(grid and runner.robot.get_camera_frame(), local_ok=False)
    assert verdict["cloud_called"] is False and cloud.calls == before


def test_review2_a_confirmer_without_local_ok_leaves_it_unconfirmed_not_refused():
    from brain.agent import MissionAgent
    from brain.memory import MissionMemory
    grid = build_world("scaled_house")
    agent = MissionAgent(MockRobot(grid, render=False),
                         MissionMemory(mission="m", target_object=TARGET),
                         arrival_confirm_fn=lambda frame: {"confirmed": True})
    out = agent._confirm_identity({"state": ARRIVED, "source": SOURCE_LOCAL}, {})
    assert out["state"] == "checking" and not agent._identity_refused


def test_review3_a_slow_identity_check_does_not_restart_the_arrival():
    t = [0.0]
    check = ArrivalCheck(clock=lambda: t[0])
    check.observe(_scene(local=0.0), _Scan(0.3))
    t[0] = 0.5
    assert check.observe(_scene(local=0.0), _Scan(0.3))["state"] == ARRIVED
    t[0] = 10.5                                     # the cloud took its whole guard
    check.refresh()
    t[0] = 11.0
    assert check.observe(_scene(), _Scan(0.3))["state"] == ARRIVED


def test_review3_a_refused_local_arrival_is_not_paid_for_twice_at_the_same_place():
    from brain.agent import MissionAgent
    from brain.memory import MissionMemory
    grid = build_world("scaled_house")
    asked = []

    def confirm(frame, local_ok=True):
        asked.append(local_ok)
        return {"confirmed": False, "cloud_called": True, "reason": "not it"}

    agent = MissionAgent(MockRobot(grid, render=False),
                         MissionMemory(mission="m", target_object=TARGET),
                         arrival_confirm_fn=confirm, world=MockWorld(grid))
    readout = {"state": ARRIVED, "source": SOURCE_LOCAL, "bearing_deg": 0.0, "range_m": 0.35}
    assert agent._confirm_identity(dict(readout), {})["state"] == "refused"
    agent._identity_refused = False                 # the hold lapsed between answers
    assert agent._confirm_identity(dict(readout), {})["state"] == "refused"
    assert asked == [False]                          # one paid question, cloud only


def test_review3_tracking_frames_do_not_build_a_cold_search():
    from brain.perceive import ABSENT, Perception
    tier = TieredVision(FrameReportedPipeline(TARGET, blind=True), TruthCloud(answer=False),
                        local_identity_fn=TruthLocal(), local_sighting_fn=TruthLocal())
    absent = Perception(status=ABSENT, synthesised=True)
    tier._trigger_for(absent)                       # the opening question
    tier.set_local_track(True)
    for _ in range(12):                             # no answer lands meanwhile
        assert tier._trigger_for(absent) == TRIGGER_TRACK
    assert tier._absent_streak < tier._cold_search_bar()


def test_391c_once_the_cloud_spoke_the_arrival_is_the_clouds():
    # 3.91.C, the tiers' order (the user, 2026-10-24): a perception-led
    # arrival asks the local VLM first -- unless the cloud has answered about
    # this candidate; then the cloud decides.
    from brain.agent import MissionAgent
    from brain.memory import MissionMemory
    from brain.arrival import SOURCE_PERCEPTION
    calls = []

    def confirm(frame, local_ok=True):
        calls.append(local_ok)
        return {"confirmed": True, "cloud_called": not local_ok}

    class Vision:
        spoke = False

        def cloud_spoke(self):
            return "ask" if self.spoke else None

    grid = build_world("scaled_house")
    agent = MissionAgent(MockRobot(grid, render=False),
                         MissionMemory(mission="m", target_object=TARGET),
                         arrival_confirm_fn=confirm)
    agent.vision_fn = Vision()
    agent._confirm_identity({"state": ARRIVED, "source": SOURCE_PERCEPTION}, {})
    agent.vision_fn.spoke = True
    agent._confirm_identity({"state": ARRIVED, "source": SOURCE_PERCEPTION}, {})
    assert calls == [True, False]



def _detected():
    from brain.perceive import DETECTED, Perception
    return Perception(status=DETECTED, reason="test")


def _runner_at_rest(cloud=None, **kw):
    """A tiered explore runner, started, no tick run: the arrival step is
    called by hand through MissionRunner._guarded_confirm (round 5: the
    agent's vision_fn is the runner's wrapper, so only this path is real)."""
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    pipeline = FrameReportedPipeline(TARGET, blind=True)
    pipeline.hfov_deg = HFOV
    identity = TruthLocal()
    cloud = cloud or TruthCloud(answer=True)
    tier = TieredVision(pipeline, TruthCloud(answer=False), confirm_vision_fn=cloud,
                        local_identity_fn=identity, local_sighting_fn=TruthLocal(), **kw)
    runner = MissionRunner(robot, target_object=TARGET, policy="tiered", vision_fn=tier,
                           tier_search="explore", navigator=FakeNav(robot),
                           world=MockWorld(grid), max_steps=10,
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           vision_timeout_s=5.0)
    runner.start()
    return runner, robot, tier, identity, cloud


def test_391c_through_the_runner_the_cloud_decides_once_it_spoke():
    runner, robot, tier, identity, cloud = _runner_at_rest()
    try:
        tier._lost, tier._episode = 0, 1
        tier._on_yes("cloud", 1)                  # the cloud said yes about this run
        assert tier.cloud_spoke() == "ask"
        verdict = runner._guarded_confirm(robot.get_camera_frame())
        assert cloud.calls == 1 and identity.asked == 0 and verdict["cloud_called"]
    finally:
        runner.stop()


def test_391c_through_the_runner_a_held_cloud_no_refuses_free():
    runner, robot, tier, identity, cloud = _runner_at_rest()
    try:
        tier._lost, tier._episode = 0, 1
        tier._reject("cloud", None, 1)
        tier._perception_this_frame = _detected()    # the arrival frame shows that run
        verdict = runner._guarded_confirm(robot.get_camera_frame())
        assert verdict["confirmed"] is False and verdict["cloud_called"] is False
        assert cloud.calls == 0 and identity.asked == 0
    finally:
        runner.stop()


def test_391c_through_the_runner_nothing_spoken_asks_the_local_vlm_first():
    runner, robot, tier, identity, cloud = _runner_at_rest()
    try:
        runner._guarded_confirm(robot.get_camera_frame())
        assert identity.asked == 1 and cloud.calls == 0
    finally:
        runner.stop()



def test_391c_through_the_runner_a_spent_cap_with_the_clouds_no_held_waits():
    runner, robot, tier, identity, cloud = _runner_at_rest(max_calls=1)
    try:
        tier._lost, tier._episode = 0, 1
        tier._reject("cloud", None, 1)
        tier._episode = 2                     # a later run, the no still held
        tier._perception_this_frame = _detected()
        tier.stats.cloud_calls = 1
        verdict = runner._guarded_confirm(robot.get_camera_frame())
        assert verdict.get("undecided") is True and verdict["confirmed"] is False
        assert identity.asked == 0 and cloud.calls == 0
    finally:
        runner.stop()
