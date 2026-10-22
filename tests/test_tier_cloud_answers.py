"""
tests/test_tier_cloud_answers.py

3.91 A, criteria 1-2: every cloud answer that lands, and every failed call,
is filed exactly once -- on the decision row of the frame by whose decision
it had landed (`decision_for`), or, if it landed after the last decision, in
the walk's meta (`cloud_answers_after_last_frame`).

The 2026-10-21 phone walk held a REVERSE for 50 s and its rows could not
say whether the cloud meant "too close" or "not the target": the decision
was recorded, the answer behind it was not.

The policy only queues the answers (`TieredVision.take_cloud_answers()`);
the runner takes them when it files a decision. Review (3.91 A): taken in
`__call__` instead, an answer rode on a scene that could be thrown away --
a guard that gave up on the frame, a halt before the decision was noted.

No models and no network: a scripted pipeline and stub clouds, as in
tests/test_tiered.py.
"""

import collections
import json
import threading
import time
from types import SimpleNamespace

import pytest

from brain.local_vlm import LocalVLMAbsent
from brain.navigate import to_scene
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
        # The real shape: /navigate's reply through `to_scene` (review: a flat
        # stub passed while the real scene's fields went unrecorded).
        return to_scene({"action": "REVERSE", "target_visible": False,
                         "target_direction": "not_visible", "target_reached": False,
                         "obstacle_ahead": True, "room_guess": "bedroom",
                         "reasoning": f"answer {n} " + "x" * 600,
                         "model_id": "stub"}, "red backpack")


class FailingCloud:
    def __init__(self):
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        raise RuntimeError("bedrock said no at https://bedrock.example/model/x?id=1")


def _frame(seq):
    return {"image_base64": "x", "image_width": 640, "metadata": {"seq": seq}}


def test_every_async_answer_is_taken_once_with_the_frame_it_was_asked_on():
    cloud = NumberedCloud(delay=0.01)
    tier = TieredVision(ScriptedPipeline([ABSENT] * 200), cloud,
                        cold_search_after=1, async_cloud=True)
    got, asked = [], []
    for seq in range(1, 61):
        if tier(_frame(seq))["_tier"].get("trigger"):
            asked.append(seq)
        got += tier.take_cloud_answers()
        time.sleep(0.005)
    tier.wait_inflight()
    got += tier.take_cloud_answers()
    assert tier.take_cloud_answers() == []                    # taken once
    calls = cloud.calls
    assert calls >= 3 and len(got) == calls, (len(got), calls)
    numbers = [int(a["reply"]["reasoning"].split()[1]) for a in got]
    assert sorted(numbers) == list(range(1, calls + 1))        # none doubled
    assert [a["asked_seq"] for a in got] == asked
    first = got[0]
    reply = first["reply"]
    assert first["action"] == "REVERSE" and reply["target_visible"] is False
    assert reply["target_direction"] == "not_visible" and reply["obstacle_ahead"] is True
    assert reply["room_guess"] == "bedroom" and reply["model_id"] == "stub"
    assert reply["reply_action"] == "REVERSE"
    assert len(reply["reasoning"]) == 500
    assert first["trigger"] == "mission_start" and "via_local_tier" not in first
    tier.close()


def test_scenes_carry_no_answers_so_a_discarded_scene_loses_none():
    cloud = NumberedCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 5), cloud, async_cloud=True)
    tier(_frame(1))                              # dispatches mission_start
    tier.wait_inflight()                         # the runner's thread collects it
    thrown_away = tier(_frame(2))                # a guard gave up on this frame
    assert "cloud_answers" not in thrown_away["_tier"]
    assert [a["asked_seq"] for a in tier.take_cloud_answers()] == [1]
    tier.close()


def test_a_failed_call_is_recorded_once_by_its_type_never_its_text():
    tier = TieredVision(ScriptedPipeline([ABSENT] * 10), FailingCloud(),
                        async_cloud=True)
    tier(_frame(1))
    tier.wait_inflight()
    raised = 0
    for seq in range(2, 6):
        try:
            tier(_frame(seq))
        except RuntimeError:
            raised += 1
    assert raised == 1                           # B3.2 still sees the failure
    (error,) = tier.take_cloud_answers()
    assert error == {"trigger": "mission_start", "asked_seq": 1, "error": "RuntimeError"}
    tier.close()


def test_a_synchronous_answer_is_recorded_with_its_frame():
    tier = TieredVision(ScriptedPipeline([DETECTED] * 3), NumberedCloud())
    tier(_frame(7))                              # mission_start, answered now
    (answer,) = tier.take_cloud_answers()
    assert answer["asked_seq"] == 7 and answer["verdict"] == "no_claim"


@pytest.mark.parametrize("cloud", [NumberedCloud(), FailingCloud()],
                         ids=["answer", "failure"])
def test_the_synchronous_local_first_fallback_to_the_cloud_is_recorded(cloud):
    def absent(frame, box=False):
        raise LocalVLMAbsent("down")
    tier = TieredVision(ScriptedPipeline([ABSENT] * 3), cloud,
                        local_sighting_fn=absent, cloud_errors_fatal=False)
    tier(_frame(4))
    assert cloud.calls == 1
    (entry,) = tier.take_cloud_answers()
    assert entry["asked_seq"] == 4
    assert ("error" in entry) == isinstance(cloud, FailingCloud)


def test_closing_the_policy_drops_what_was_not_taken():
    tier = TieredVision(ScriptedPipeline([DETECTED] * 3), NumberedCloud())
    tier(_frame(1))
    tier.close()
    assert tier.take_cloud_answers() == []


class _Policy:
    def __init__(self):
        self.queue = []

    def take_cloud_answers(self):
        got, self.queue = self.queue, []
        return got


def _runner(policy):
    runner = SimpleNamespace(
        vision_fn=policy, _perception={"status": "absent"}, _last_reasoning="r",
        _tier={"trigger": None}, _decisions_lock=threading.Lock(),
        _decisions=collections.OrderedDict(), _last_decided=None,
        DECISIONS_KEPT=MissionRunner.DECISIONS_KEPT)
    runner._take_cloud_answers = lambda: MissionRunner._take_cloud_answers(runner)
    return runner


def test_the_runner_files_each_answer_on_one_decision():
    policy = _Policy()
    runner = _runner(policy)

    def tick(seq):
        MissionRunner._note_decision(runner, SimpleNamespace(
            frame=_frame(seq), scene={"_tier": {}}, action="REVERSE"))
        return MissionRunner.decision_for(runner, seq)

    policy.queue = [{"asked_seq": 1}]
    assert tick(1)["cloud_answers"] == [{"asked_seq": 1}]
    assert "cloud_answers" not in tick(2)
    # A tick that halted before its decision was noted left one queued:
    # the next decision carries it.
    policy.queue = [{"asked_seq": 3}]
    assert tick(4)["cloud_answers"] == [{"asked_seq": 3}]


def test_answers_after_the_last_decision_go_to_the_walks_meta(tmp_path):
    from control.stream_recorder import StreamRecorder
    from control.walk_store import LocalWalkStore
    store = LocalWalkStore(str(tmp_path))
    runner = SimpleNamespace(is_running=lambda: False,
                             decision_for=lambda seq, boot=None: None,
                             decided_past=lambda seq, boot=None: True,
                             unfiled_cloud_answers=[{"asked_seq": 9, "verdict": "no_claim"}])
    frames = {"frames": [{"image_base64": "/9j/", "metadata": {"seq": 1}}],
              "latest_seq": 1, "boot_id": "b"}
    rec = StreamRecorder(store, "w", lambda after: frames if after < 1 else
                         {"frames": [], "latest_seq": 1, "boot_id": "b"}, runner)
    rec.start(after=0)
    assert rec.finished.wait(5)
    meta = json.loads((tmp_path / "w" / "meta.json").read_text())
    assert meta["cloud_answers_after_last_frame"] == [{"asked_seq": 9, "verdict": "no_claim"}]


def test_a_frame_with_no_number_drops_the_answers():
    """Review round 3: no frame number means no row and no stream recorder
    (only a numbering body has one); kept, they grew for the whole mission."""
    policy = _Policy()
    runner = _runner(policy)
    policy.queue = [{"asked_seq": 1}]
    MissionRunner._note_decision(runner, SimpleNamespace(
        frame={"image_base64": "x"}, scene={}, action="LEFT"))
    assert policy.queue == []


def test_a_frame_decided_again_keeps_its_answers_and_takes_no_new_ones():
    """Review round 2: a stalled phone repeats the newest frame; the recorder
    may have filed its first decision, so a later answer waits for a new
    frame rather than vanishing into a row already written."""
    policy = _Policy()
    runner = _runner(policy)

    def tick(seq):
        MissionRunner._note_decision(runner, SimpleNamespace(
            frame=_frame(seq), scene={}, action="REVERSE"))
        return MissionRunner.decision_for(runner, seq)

    policy.queue = [{"asked_seq": 1}]
    tick(1)
    policy.queue = [{"asked_seq": 2}]
    assert tick(1)["cloud_answers"] == [{"asked_seq": 1}]
    assert tick(2)["cloud_answers"] == [{"asked_seq": 2}]


def test_the_final_take_notes_a_landed_call_without_applying_it():
    """Review round 2: a call that landed after the last frame was lost when
    `close()` dropped it. Round 3: collected at the end, it was applied from
    the Stop's thread beside a tick mid-frame, before the motors stopped."""
    cloud = NumberedCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 3), cloud, async_cloud=True)
    tier(_frame(1))
    tier._inflight.result(timeout=5)             # landed; nothing collected it
    assert tier.take_cloud_answers() == []
    (answer,) = tier.take_cloud_answers(final=True)
    assert answer["asked_seq"] == 1
    assert tier._last_cloud_scene is None          # noted, not applied
    tier(_frame(2))                                # a tick collects it after all
    assert tier._last_cloud_scene is not None
    assert tier.take_cloud_answers() == []         # and does not note it twice
    tier.close()


def test_an_async_local_first_question_the_cloud_answered_is_the_clouds():
    """Code review round 2: tagged by the job's tier, the cloud's own paid
    answer read as the local tier's."""
    def absent(frame, box=False):
        raise LocalVLMAbsent("down")
    cloud = NumberedCloud()
    tier = TieredVision(ScriptedPipeline([ABSENT] * 3), cloud,
                        local_sighting_fn=absent, async_cloud=True)
    tier(_frame(5))
    tier.wait_inflight()
    (answer,) = tier.take_cloud_answers()
    assert cloud.calls == 1 and answer["via_local_tier"] is True
    assert answer["action"] == "REVERSE" and "error" not in answer
    tier.close()


def test_an_http_error_keeps_its_status_and_not_its_host():
    class Unreachable:
        def __call__(self, frame):
            raise RuntimeError("HTTP 502 from https://secret.ngrok.app/brain/navigate: "
                               "endpoint secret.ngrok.app is offline")
    tier = TieredVision(ScriptedPipeline([ABSENT] * 3), Unreachable(),
                        cloud_errors_fatal=False)
    tier(_frame(1))
    (error,) = tier.take_cloud_answers()
    assert error == {"trigger": "mission_start", "asked_seq": 1,
                     "error": "RuntimeError", "status": 502}


def test_the_synchronous_local_first_answer_is_tagged_the_clouds():
    def absent(frame, box=False):
        raise LocalVLMAbsent("down")
    tier = TieredVision(ScriptedPipeline([ABSENT] * 3), NumberedCloud(),
                        local_sighting_fn=absent)
    tier(_frame(4))
    (answer,) = tier.take_cloud_answers()
    assert answer["via_local_tier"] is True and answer["action"] == "REVERSE"


def test_the_queue_is_bounded_when_nobody_takes_it():
    from brain.tiered import CLOUD_ANSWERS_KEPT
    tier = TieredVision(ScriptedPipeline([DETECTED] * 3), NumberedCloud())
    for seq in range(CLOUD_ANSWERS_KEPT + 5):
        tier._note_cloud_answer("t", seq)
    kept = tier.take_cloud_answers()
    assert len(kept) == CLOUD_ANSWERS_KEPT and kept[0]["asked_seq"] == 5
    assert tier.stats.cloud_answers_dropped == 5


def test_a_take_that_raises_a_cancelled_error_cannot_stop_the_stop():
    """Review round 3: `_finish` takes before the stop, and a cancelled
    future's CancelledError is a BaseException."""
    import concurrent.futures

    class Cancelled:
        def take_cloud_answers(self, final=False):
            raise concurrent.futures.CancelledError()
    runner = _runner(Cancelled())
    assert MissionRunner._take_cloud_answers(runner, final=True) == []


def test_the_final_take_reads_the_verdict_too():
    """Round 4: noted at the end, the answer had no verdict."""
    tier = TieredVision(ScriptedPipeline([DETECTED] * 3), NumberedCloud(delay=0.05),
                        async_cloud=True)
    tier(_frame(1))
    tier._inflight.result(timeout=5)
    (answer,) = tier.take_cloud_answers(final=True)
    assert answer["verdict"] == "no_claim"
    tier.close()


def test_the_final_take_keeps_the_queue_when_noting_fails():
    """Round 4: the swap in a `finally` threw the queue away with the error."""
    tier = TieredVision(ScriptedPipeline([DETECTED] * 3), NumberedCloud())
    tier(_frame(1))                                # one answer queued
    tier._note_landed_unapplied = lambda: (_ for _ in ()).throw(ValueError("odd"))
    (answer,) = tier.take_cloud_answers(final=True)
    assert answer["asked_seq"] == 1


def test_a_stray_unnumbered_frame_on_a_numbering_body_keeps_the_answers():
    """Round 4: dropped only where no frame has ever been numbered."""
    policy = _Policy()
    runner = _runner(policy)
    MissionRunner._note_decision(runner, SimpleNamespace(
        frame=_frame(1), scene={}, action="LEFT"))
    policy.queue = [{"asked_seq": 1}]
    MissionRunner._note_decision(runner, SimpleNamespace(
        frame={"image_base64": "x"}, scene={}, action="LEFT"))
    assert policy.queue == [{"asked_seq": 1}]


@pytest.mark.parametrize("scene", [{"safest_direction": "LEFT", "_navigate": "reasoning"},
                                   {"safest_direction": "LEFT", "_navigate": ["x"]},
                                   "not a scene"])
def test_an_odd_scene_is_noted_without_raising(scene):
    tier = TieredVision(ScriptedPipeline([ABSENT] * 3), NumberedCloud())
    tier._note_cloud_answer("t", 1, scene=scene)
    (entry,) = tier.take_cloud_answers()
    assert entry["asked_seq"] == 1 and entry["action"] in (None, "LEFT")
    assert entry["reply"] in (None, "reasoning", ["x"])


def test_a_coerced_action_is_told_apart_from_the_models():
    """Code review: `to_scene` makes an unknown or missing action STOP; the
    record keeps what the reply said."""
    tier = TieredVision(ScriptedPipeline([ABSENT] * 3), NumberedCloud())
    tier._note_cloud_answer("t", 1, scene=to_scene({"action": "APPROACH"}, "bag"))
    tier._note_cloud_answer("t", 2, scene=to_scene({}, "bag"))
    tier._note_cloud_answer("t", 3, scene=to_scene({"action": None}, "bag"))
    odd, missing, null = tier.take_cloud_answers()
    assert odd["action"] == "STOP" and odd["reply"]["reply_action"] == "APPROACH"
    assert missing["action"] == "STOP" and "reply_action" not in missing["reply"]
    assert "reply_action" not in null["reply"]


def test_a_reply_cannot_write_the_records_own_keys_or_grow_it_without_bound():
    """Code review: copied field by field, a reply's `error`, `verdict` or
    `via_local_tier` could pose as the entry's own; NaN broke strict JSON."""
    tier = TieredVision(ScriptedPipeline([ABSENT] * 3), NumberedCloud())
    nav = {"error": "x", "verdict": "x", "via_local_tier": True, "nan": float("nan"),
           "box": {"x": [1, 2, {"deep": {"deeper": 1}}]},
           **{f"k{i}": i for i in range(40)}}
    tier._note_cloud_answer("t", 1, scene={"safest_direction": "LEFT", "_navigate": nav},
                            verdict="no_claim")
    (entry,) = tier.take_cloud_answers()
    assert set(entry) == {"trigger", "asked_seq", "verdict", "action", "reply"}
    assert entry["verdict"] == "no_claim" and entry["action"] == "LEFT"
    reply = entry["reply"]
    assert len(reply) == 16 and reply["nan"] is None
    assert reply["box"] == {"x": [1, 2, "dict"]}         # three levels kept
    json.dumps(entry, allow_nan=False)
