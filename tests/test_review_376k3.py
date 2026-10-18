"""
tests/test_review_376k3.py

`PLAN-ros-alignment.md` 3.76, step K3: the bugs the review of the brain's
tiered policy, explore and arrival as other sessions changed them after
batch E (3.70 S1-S3, 3.77, 3.80 C-P, 3.81.E, 3.83, 3.85) confirmed. Each
test was written before its fix and seen to fail on `dev` for the reason it
names.
"""

from brain.navigate import CloudUnavailable
from tests.test_tiered_frames import (
    ABSENT, Cloud, Local, P, frame, tier_for, unable, vocab)


# ---------- K3.1: a failing cloud kept the local tier locked out ----------


def test_a_failing_cloud_hands_the_question_back_to_the_local_tier():
    """Two unable answers hand a frame's question to the cloud; only a cloud
    ANSWER started the count again. Under an explore search (cloud errors
    not fatal) with the cloud down, the local VLM was never asked again --
    no out-of-vocabulary question, no tracking -- for the whole mission."""
    local = Local(unable())
    tier, cloud = tier_for([P(ABSENT)], local, cloud=Cloud(fail=CloudUnavailable("down")),
                           vocabulary=vocab(False), cloud_errors_fatal=False,
                           cold_search_after=3)
    for _ in range(12):
        tier(frame())
    assert cloud.calls >= 1, "the escalation never reached the cloud"
    assert local.asked > 2, f"the local tier was asked {local.asked} times, then never again"


# ---------- K3.2: an arrival check still out made frame questions read unable ----------


def test_frame_questions_wait_out_an_arrival_check_still_running():
    """A frame question never looked at the arrival's identity call. One the
    runner gave up on (abandon_local) still holds the VLM, so each frame
    question came back `busy`, was counted unable, and after two the next
    went to the cloud as a paid call -- the tier competing with itself."""
    import threading
    release = threading.Event()

    def identity(f):
        release.wait(5)
        return {"status": "answered", "score": 0.9}

    local = Local(unable("busy"))
    tier, cloud = tier_for([P(ABSENT)], local, vocabulary=vocab(False),
                           local_identity_fn=identity, cold_search_after=1)
    t = threading.Thread(target=tier.confirm_local, args=(frame(),), daemon=True)
    t.start()
    for _ in range(50):
        if tier._local_inflight is not None:
            break
        threading.Event().wait(0.01)
    tier.abandon_local()                  # the runner's share ran out
    try:
        for _ in range(6):
            tier(frame())
        assert local.asked == 0, f"{local.asked} frame questions sent beside the arrival check"
        assert cloud.calls == 0, "a paid call bought because the tier was busy with itself"
    finally:
        release.set()
        t.join(5)
    tier(frame())
    assert local.asked == 1, "the question was not asked once the check ended"


# ---------- K3.3: close-in's refused count carried from one target to the next ----------


def test_close_in_refusals_at_one_target_do_not_count_at_the_next():
    """`_close_in_refused` reset only on an executed verb, an accepted
    approach goal or a refusal noted. Two refused verbs at one sighting,
    then a target reached by a frontier goal (no approach sent): its first
    refused verb was the third, and the new target was failed and backed
    away from at once."""
    from tests.test_tiered_explore import _agent
    agent, _ = _agent()
    agent._verb = lambda action, **kw: ("FORWARD", False, "refused: too close")
    scene = {"_perception": {"status": "detected", "bearing_deg": 0.0, "pan_deg": 0.0}}
    first, second = (3.0, 3.0), (6.0, 1.0)
    agent._close_in(scene, first)
    agent._close_in(scene, first)
    agent._close_in(scene, second)
    assert agent.retry.failures_near(*second) == 0, "a new target failed on its first refusal"
    assert "REVERSE" not in agent._pending, agent._pending
