"""
tests/test_review_376e.py

`PLAN-ros-alignment.md` 3.76, batch E: the bugs the whole-file review of
the perception, arrival and cloud-call code (`brain/tiered.py`,
`brain/navigate.py`, `brain/explore.py`, ...) confirmed. Each test was
written before its fix and seen to fail on `dev`.
"""

import time
from concurrent.futures import Future

import httpx
import pytest

from brain.navigate import CloudUnavailable, navigate_scene
from brain.perceive import ABSENT, DETECTED, Perception
from brain.tiered import TRIGGER_COLD_SEARCH, TierStats, TieredVision
from tests.test_tiered import FakeCloud, ScriptedPipeline

FRAME = {"image_base64": "x", "image_width": 640}


# ---------- a Stop mid-frame buys no cloud call ----------


class _StopDuringPerception:
    """A Stop landing while this frame is in perception -- 61-110 ms on the
    Jetson, on the tick thread, while Stop runs on the brain's loop."""

    tier = None

    def perceive(self, frame):
        self.tier.close()
        return Perception(status=DETECTED, reason="scripted")


@pytest.mark.parametrize("async_cloud", [True, False])
def test_a_stop_during_perception_sends_no_cloud_call(async_cloud):
    """close() reset the epoch and dropped the worker pool, but the frame
    already past perception went on to dispatch: a new pool nobody shut
    down, a call stamped with the new epoch -- billed after Stop."""
    cloud = FakeCloud()
    pipeline = _StopDuringPerception()
    tier = TieredVision(pipeline, cloud, async_cloud=async_cloud)
    pipeline.tier = tier
    tier(FRAME)                                   # the first frame always fires
    time.sleep(0.2)
    assert cloud.calls == 0, "a paid call went out after Stop"
    assert tier._executor is None, "a worker pool was created after close()"


# ---------- a landed answer is not a turn the robot made ----------


def test_a_landed_async_answer_does_not_feed_the_spin_guard():
    """`_collect_inflight` annotates a landed answer for its side effects;
    that annotation also counted its turn in the spin guard."""
    tier = TieredVision(ScriptedPipeline([ABSENT]), FakeCloud(), async_cloud=True)
    landed = Future()
    landed.set_result({"safest_direction": "RIGHT", "important_objects": [],
                       "_navigate": {"reasoning": "turn"}})
    tier._inflight, tier._inflight_trigger = landed, TRIGGER_COLD_SEARCH
    tier._inflight_perception = Perception(status=ABSENT, reason="x")
    tier._consecutive_turn_deg = 0.0
    tier._collect_inflight()
    assert tier._consecutive_turn_deg == 0.0, tier._consecutive_turn_deg


# ---------- a sighting held back by a call in flight still fires ----------


class _SlowCloud(FakeCloud):
    def __init__(self):
        super().__init__()
        self.triggers = []

    def __call__(self, frame):
        time.sleep(0.3)
        return super().__call__(frame)


def test_a_sighting_during_a_call_in_flight_fires_once_it_lands():
    """The edge into `detected` was spent before the one-call gate held the
    call back, so the sighting never got its own call -- the cloud next
    looked only on the staleness floor."""
    cloud = _SlowCloud()
    statuses = [ABSENT, DETECTED, DETECTED, DETECTED, DETECTED, DETECTED]
    tier = TieredVision(ScriptedPipeline(statuses), cloud, async_cloud=True,
                        stale_after=50)
    tier(FRAME)                                   # start: a call goes out
    tier(FRAME)
    tier(FRAME)                                   # DETECTED twice: the edge, held back
    time.sleep(0.5)                               # the first call lands
    tier(FRAME)
    tier(FRAME)
    time.sleep(0.5)
    assert tier.stats.triggers.get("candidate_sighting", 0) == 1, tier.stats.triggers


# ---------- the stats sample the whole mission ----------


def test_late_samples_reach_the_percentiles():
    """The first 500 samples were kept and every later one dropped: a
    20-minute mission reported its first two minutes."""
    stats = TierStats()
    for _ in range(2000):
        stats.record("cloud_ms", 100.0)
    for _ in range(2000):
        stats.record("cloud_ms", 5000.0)
    assert any(v == 5000.0 for v in stats.cloud_ms), "a later slowdown never shows"
    assert len(stats.cloud_ms) <= TierStats.MAX_SAMPLES
    stats.record("cloud_ms", 90000.0)                # one very slow call
    p = stats.as_dict()["cloud_ms"]
    assert p["n"] == 4001 and p["max"] == 90000, p


# ---------- a bad URL is ours, not an unreachable cloud ----------


def test_a_vision_url_without_a_scheme_is_not_cloud_unavailable():
    """Every httpx TransportError read as `CloudUnavailable`, a URL with no
    scheme included: an arrival ended `arrived_unconfirmed` and the late
    check probed for ten minutes instead of the mission failing on a
    configuration error."""
    with pytest.raises(Exception) as caught:
        navigate_scene({"image_base64": "x"}, "mug", "vision.example/no-scheme",
                       timeout_s=1.0)
    assert not isinstance(caught.value, CloudUnavailable), repr(caught.value)


# ---------- explore and other people's goals ----------


from brain.explore import ExploreAgent  # noqa: E402
from brain.memory import MissionMemory  # noqa: E402
from robot.interface import Preempted  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402
from tests.test_explore import FakeNav  # noqa: E402


class _HeldNav(FakeNav):
    """nav2 holding whatever goal it has: nothing finishes on its own."""

    def get_goal(self):
        self.calls.append(("get",))
        return {"goal": self.goal, "plan": []}


def _explore(nav_cls=_HeldNav):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = nav_cls(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="red backpack"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid))
    return agent, nav


def test_a_persons_goal_replacing_the_missions_ends_it():
    """`_goal_state()` took whatever goal nav2 held for the mission's own: a
    person's tap was waited on, then recorded as the mission's place."""
    agent, nav = _explore()
    for _ in range(30):
        agent.step()
        if agent._goal is not None:
            break
    assert agent._goal is not None, "explore never sent a goal"
    nav.goal = {"state": "active", "x_m": agent._goal["x_m"] + 2.0, "y_m": agent._goal["y_m"]}
    with pytest.raises(Preempted):
        agent.step()


def test_a_pan_does_not_cancel_a_persons_goal():
    """`_verb()` cancelled any live goal before a pan or escape -- DELETE is
    not arbitrated, so a person's tap was deleted without a word."""
    agent, nav = _explore()
    agent._last_sent = (1.0, 1.0)                    # the mission had sent its own
    nav.goal = {"state": "active", "x_m": 5.0, "y_m": 5.0}          # a person's tap
    agent._pending = ["LOOK_LEFT"]
    with pytest.raises(Preempted):
        agent._do_pending()
    assert nav.goal["state"] == "active", "the person's goal was cancelled"


class _BlipNav(_HeldNav):
    blip = False

    def get_goal(self):
        if self.blip:
            raise self.blip
        return super().get_goal()


def test_one_unreadable_goal_poll_is_a_wait_not_a_failed_mission():
    """One transient error reading nav2's goal raised out of `step()`, and
    the runner ended the whole search `failed`; a map outage only waits."""
    agent, nav = _explore(_BlipNav)
    for _ in range(30):
        agent.step()
        if agent._goal is not None:
            break
    assert agent._goal is not None
    nav.blip = httpx.ConnectError("bridge blip")
    result = agent.step()                          # must not raise
    assert result.action == "WAIT", result


def test_a_goal_route_that_refuses_us_still_ends_the_search():
    """...but a 401 (a rotated secret) is not a blip: waiting on it would
    sit the robot still for the whole time limit."""
    agent, nav = _explore(_BlipNav)
    for _ in range(30):
        agent.step()
        if agent._goal is not None:
            break
    nav.blip = httpx.HTTPStatusError("401", request=httpx.Request("GET", "http://r"),
                                     response=httpx.Response(401))
    with pytest.raises(httpx.HTTPStatusError):
        agent.step()


def test_a_persons_goal_that_finished_between_polls_is_not_the_missions():
    agent, nav = _explore()
    for _ in range(30):
        agent.step()
        if agent._goal is not None:
            break
    nav.goal = {"state": "succeeded", "x_m": agent._goal["x_m"] + 2.0, "y_m": agent._goal["y_m"]}
    with pytest.raises(Preempted):
        agent.step()


def test_a_leftover_goal_before_the_missions_first_is_cancelled_not_blamed_on_a_person():
    agent, nav = _explore()
    nav.goal = {"state": "active", "x_m": 5.0, "y_m": 5.0}          # a previous mission's
    agent._pending = ["LOOK_LEFT"]
    agent._do_pending()                              # must not raise
    assert nav.goal["state"] == "canceled"


def test_an_approach_already_at_the_target_cancels_a_live_frontier_goal():
    """Within standoff of a sighting, explore answered WAIT and left the
    frontier goal live: nav2 drove past the target before arrival fired."""
    agent, nav = _explore()
    for _ in range(30):
        agent.step()
        if agent._goal is not None:
            break
    assert agent._goal is not None
    pose = agent._pose()
    agent._send_approach(pose["x_m"] + 0.3, pose["y_m"], agent.clock())
    assert nav.goal["state"] == "canceled", nav.goal


def test_a_failure_records_how_much_of_the_map_was_known_there():
    """`RetryBook.fail()` was never given `known`, so the 'try again early
    once the map has grown there' rule never ran."""
    agent, nav = _explore()
    for _ in range(30):
        agent.step()
        if agent._goal is not None:
            break
    g = agent._goal
    pose = agent._pose()
    g["from"] = (pose["x_m"] - 2.0, pose["y_m"])   # it travelled: a real failure, not a wedge
    agent._goal_ended("aborted", agent.clock())
    entry = agent.retry._near(g["x_m"], g["y_m"])
    assert entry is not None and entry.known_at_failure > 0, entry


def test_a_found_target_ends_found_even_if_a_foreign_goal_is_live():
    """The found path cancels the mission's goal first; with someone else's
    goal live that cancel raised Preempted, and a confirmed find (its cloud
    call paid) ended `cancelled`."""
    agent, nav = _explore()
    for _ in range(30):
        agent.step()
        if agent._goal is not None:
            break
    nav.goal = {"state": "active", "x_m": agent._goal["x_m"] + 2.0, "y_m": agent._goal["y_m"]}
    agent.memory.is_complete = lambda: True
    action, executed, detail = agent._choose({}, {})
    assert action == "STOP" and "found" in detail


class _LostAnswerNav(_HeldNav):
    """The bridge takes the goal, then our read of its answer times out."""

    lose = False

    def set_goal(self, x_m, y_m):
        reply = super().set_goal(x_m, y_m)
        if self.lose:
            raise httpx.ReadTimeout("answer lost")
        return reply


def test_a_goal_whose_answer_was_lost_is_still_the_missions_own():
    """A send whose answer was lost left that goal driving and unknown to
    the mission; the next pan took it for a person's and ended the mission
    'cancelled by the D-pad'."""
    agent, nav = _explore(_LostAnswerNav)
    nav.lose = True
    agent._send(1.25, 1.25, "frontier", agent.clock())
    agent._pending = ["LOOK_LEFT"]
    agent._do_pending()                              # must not raise
    assert nav.goal["state"] == "canceled"


# ---------- cold search after a slow call (the user's option 3) ----------


def test_no_second_call_fires_as_soon_as_a_slow_call_lands():
    """Every pacing count kept running while an async call was out (absent
    frames, frames since the call, distance), so the frame after a slow call
    landed was already past a bar and a second paid call went out back to
    back. With the shipped defaults (cold search 6, staleness 8)."""
    cloud = _SlowCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 40), cloud, async_cloud=True)
    tier(FRAME)                                   # start: a call goes out
    for _ in range(10):                           # ten absent frames while it is out
        tier(FRAME)
    tier._inflight.result(timeout=5)              # it has landed
    tier(FRAME)                                   # the first frame after
    time.sleep(0.5)
    assert cloud.calls == 1, f"{cloud.calls} calls: {tier.stats.triggers}"
