"""
tests/test_tier_cloud_answers.py

3.91 A, criteria 1-2: every cloud answer that lands is carried out exactly
once, on the frame whose decision it landed by, as `_tier.cloud_answers`,
and the runner files it on that frame's decision row (`decision_for`).

The 2026-10-21 phone walk held a REVERSE for 50 s and its rows could not
say whether the cloud meant "too close" or "not the target": the decision
was recorded, the answer behind it was not.

No models and no network: a scripted pipeline and stub clouds, as in
tests/test_tiered.py.
"""

import threading
import time
from types import SimpleNamespace

from brain.perceive import ABSENT, DETECTED
from brain.tiered import TieredVision
from control.mission_runner import MissionRunner
from tests.test_tiered import ScriptedPipeline


class NumberedCloud:
    """Each answer says which call it was."""

    def __init__(self, delay=0.0):
        self.calls = 0
        self.delay = delay
        self.lock = threading.Lock()

    def __call__(self, frame):
        with self.lock:
            self.calls += 1
            n = self.calls
        time.sleep(self.delay)
        return {"action": "REVERSE", "safest_direction": "REVERSE",
                "target_visible": False, "target_direction": None,
                "reasoning": f"answer {n} " + "x" * 600, "model_id": "stub",
                "obstacles_ahead": [], "free_space": "blocked",
                "doorway_visible": False, "important_objects": []}


class FailingCloud:
    def __init__(self):
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        raise RuntimeError("bedrock said no")


def _frame(seq):
    return {"image_base64": "x", "image_width": 640, "metadata": {"seq": seq}}


def _answers(scenes):
    return [a for s in scenes for a in (s.get("_tier") or {}).get("cloud_answers") or []]


def test_every_async_answer_lands_once_with_the_frame_it_was_asked_on():
    cloud = NumberedCloud(delay=0.01)
    tier = TieredVision(ScriptedPipeline([ABSENT] * 200), cloud,
                        cold_search_after=1, async_cloud=True)
    scenes, asked = [], []
    for seq in range(1, 61):
        s = tier(_frame(seq))
        scenes.append(s)
        if s["_tier"].get("trigger"):
            asked.append(seq)
        time.sleep(0.005)
    tier.wait_inflight()
    scenes.append(tier(_frame(61)))          # carries the last one out
    got = _answers(scenes)
    calls = cloud.calls
    assert calls >= 3
    assert len(got) == calls, (len(got), calls)
    numbers = [int(a["reasoning"].split()[1]) for a in got]
    assert sorted(numbers) == list(range(1, calls + 1))        # none doubled
    assert [a["asked_seq"] for a in got] == asked[:len(got)]
    first = got[0]
    assert first["safest_direction"] == "REVERSE" and first["action"] == "REVERSE"
    assert first["model_id"] == "stub" and first["target_visible"] is False
    assert len(first["reasoning"]) == 500
    assert first["trigger"] == "mission_start"
    tier.close()


def test_an_answer_collected_by_the_runners_wait_is_carried_by_the_next_frame():
    cloud = NumberedCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 5), cloud, async_cloud=True)
    first = tier(_frame(1))                     # dispatches mission_start
    assert not first["_tier"].get("cloud_answers")
    tier.wait_inflight()                        # the runner's thread takes it
    second = tier(_frame(2))
    assert [a["asked_seq"] for a in second["_tier"]["cloud_answers"]] == [1]
    third = tier(_frame(3))
    assert not third["_tier"].get("cloud_answers")
    tier.close()


def test_a_failed_call_is_recorded_once_even_when_its_frame_raises():
    tier = TieredVision(ScriptedPipeline([ABSENT] * 10), FailingCloud(),
                        async_cloud=True)
    tier(_frame(1))
    tier.wait_inflight()
    raised, scenes = 0, []
    for seq in range(2, 6):
        try:
            scenes.append(tier(_frame(seq)))
        except RuntimeError:
            raised += 1
    assert raised == 1                          # B3.2 still sees the failure
    errors = [a for a in _answers(scenes) if "cloud_error" in a]
    assert len(errors) == 1 and errors[0]["asked_seq"] == 1
    assert "bedrock said no" in errors[0]["cloud_error"]
    tier.close()


def test_a_synchronous_answer_is_on_its_own_frame():
    cloud = NumberedCloud()
    tier = TieredVision(ScriptedPipeline([DETECTED] * 3), cloud)
    out = tier(_frame(7))                       # mission_start, answered now
    (answer,) = out["_tier"]["cloud_answers"]
    assert answer["asked_seq"] == 7 and answer["verdict"] == "no_claim"


def test_the_runner_files_answers_from_this_ticks_scene_only():
    """The runner holds `_tier` across frames; reading answers from it would
    copy one answer onto every later row."""
    runner = SimpleNamespace(
        _perception={"status": "absent"}, _last_reasoning="r",
        _tier={"trigger": None, "cloud_answers": [{"asked_seq": 1}]},
        _decisions_lock=threading.Lock(), _decisions=__import__("collections").OrderedDict(),
        _last_decided=None, DECISIONS_KEPT=MissionRunner.DECISIONS_KEPT)

    def tick(seq, scene):
        MissionRunner._note_decision(runner, SimpleNamespace(
            frame=_frame(seq), scene=scene, action="REVERSE"))
        return MissionRunner.decision_for(runner, seq)

    landed = tick(1, {"_tier": {"cloud_answers": [{"asked_seq": 1}]}})
    assert landed["cloud_answers"] == [{"asked_seq": 1}]
    later = tick(2, {"_tier": {}})
    assert "cloud_answers" not in later
