"""3.90 A criterion 4: the counted rules keep their time.

`candidate_sighting` (`tier_consecutive_frames`) and arrival
(`ARRIVAL_FRAMES`) are counted in frames. At the brain's 0.25 s tick two
frames span 0.25 s; at a camera's 9 fps they would span 0.111 s, so the rule
would quietly get shorter (R1c). With capture stamps, a run must also cover
0.25 s: at least that, and no more than one frame interval beyond it.
Unstamped frames (the simulator's) are counted exactly as before.
"""

import pytest

from brain.arrival import ARRIVED, SPAN_SLACK_S, ArrivalCheck
from brain.perceive import ABSENT, DETECTED
from brain.tiered import TRIGGER_CANDIDATE, TieredVision
from tests.test_arrival import _FakeRobot, _scene
from tests.test_tiered import FakeCloud, ScriptedPipeline

SPAN_S = 0.25          # today's two frames at the 0.25 s tick
T0 = 1_700_000_000.0


def _sighting_span(fps, stamped=True):
    """Seconds from the first `detected` frame to the frame that fires
    `candidate_sighting`, and that frame's index."""
    statuses = [ABSENT, ABSENT] + [DETECTED] * 12
    tier = TieredVision(ScriptedPipeline(statuses), FakeCloud(), cold_search_after=99)
    first = None
    for i in range(len(statuses)):
        t = T0 + i / fps
        frame = {"image_base64": "x", "image_width": 640}
        if stamped:
            frame["captured_at"] = t
        scene = tier(frame)
        if statuses[i] == DETECTED and first is None:
            first = t
        if scene["_tier"].get("trigger") == TRIGGER_CANDIDATE:
            return t - first, i
    raise AssertionError("candidate_sighting never fired")


def _arrival_span(fps, stamped=True):
    check, robot = ArrivalCheck(), _FakeRobot([0.2] * 360)
    for i in range(20):
        t = T0 + i / fps
        readout = check.observe(_scene(0.0), robot, stamp=t if stamped else None)
        if readout["state"] == ARRIVED:
            return t - T0, i
    raise AssertionError("never arrived")


@pytest.mark.parametrize("fps", [4.0, 9.0])
def test_a_sighting_needs_the_same_time_at_any_frame_rate(fps):
    span, _ = _sighting_span(fps)
    assert SPAN_S - SPAN_SLACK_S <= span <= SPAN_S + 1.0 / fps


@pytest.mark.parametrize("fps", [4.0, 9.0])
def test_an_arrival_needs_the_same_time_at_any_frame_rate(fps):
    span, _ = _arrival_span(fps)
    assert SPAN_S - SPAN_SLACK_S <= span <= SPAN_S + 1.0 / fps


def test_unstamped_frames_are_counted_as_before():
    # Two frames, whatever the (unknown) time between them: the simulator's
    # sweeps and every pinned tiered number stay where they are.
    _, sighting_frame = _sighting_span(9.0, stamped=False)
    _, arrival_frame = _arrival_span(9.0, stamped=False)
    assert sighting_frame == 3          # the 2nd detected frame (index 2, 3)
    assert arrival_frame == 1           # the 2nd frame


def test_a_slow_camera_still_needs_two_frames():
    # At 2 fps two frames already span 0.5 s: the count, not the time, binds.
    span, frame = _sighting_span(2.0)
    assert frame == 3 and span == pytest.approx(0.5)


def test_a_span_of_zero_counts_frames_only():
    statuses = [ABSENT, DETECTED, DETECTED, ABSENT]
    tier = TieredVision(ScriptedPipeline(statuses), FakeCloud(), cold_search_after=99,
                        sighting_span_s=0)
    for i in range(len(statuses)):
        tier({"image_base64": "x", "image_width": 640, "captured_at": T0 + i / 30})
    assert tier.stats.triggers[TRIGGER_CANDIDATE] == 1


def test_a_break_in_the_run_restarts_the_clock():
    statuses = [ABSENT, DETECTED, ABSENT] + [DETECTED] * 6
    tier = TieredVision(ScriptedPipeline(statuses), FakeCloud(), cold_search_after=99)
    fired = None
    for i in range(len(statuses)):
        scene = tier({"image_base64": "x", "image_width": 640, "captured_at": T0 + i / 9})
        if scene["_tier"].get("trigger") == TRIGGER_CANDIDATE:
            fired = i
            break
    # The run that counts starts at index 3; 0.222 s later is index 5.
    assert fired == 5


def test_a_clock_step_back_counts_the_run_rather_than_holding_it():
    statuses = [ABSENT, DETECTED, DETECTED]
    tier = TieredVision(ScriptedPipeline(statuses), FakeCloud(), cold_search_after=99)
    for stamp in (T0, T0 + 10.0, T0 + 10.05 - 3600):    # the second detected stamp jumped back
        tier({"image_base64": "x", "image_width": 640, "captured_at": stamp})
    assert tier.stats.triggers[TRIGGER_CANDIDATE] == 1
    check, robot = ArrivalCheck(), _FakeRobot([0.2] * 360)
    check.observe(_scene(0.0), robot, stamp=T0)
    assert check.observe(_scene(0.0), robot, stamp=T0 - 3600)["state"] == ARRIVED


def test_ticks_on_a_30fps_grid_still_confirm_on_the_second_frame():
    # Review: the brain reads the newest 30 fps frame each 0.25 s tick, so
    # stamps fall 0.217-0.283 s apart. Two ticks must stay two frames.
    for gap in (0.217, 0.25, 0.283):
        statuses = [ABSENT, DETECTED, DETECTED]
        tier = TieredVision(ScriptedPipeline(statuses), FakeCloud(), cold_search_after=99)
        for i, stamp in enumerate((T0, T0 + gap, T0 + 2 * gap)):
            tier({"image_base64": "x", "image_width": 640, "captured_at": stamp})
        assert tier.stats.triggers[TRIGGER_CANDIDATE] == 1, gap
        check, robot = ArrivalCheck(), _FakeRobot([0.2] * 360)
        check.observe(_scene(0.0), robot, stamp=T0)
        assert check.observe(_scene(0.0), robot, stamp=T0 + gap)["state"] == ARRIVED, gap


def _vote_run(statuses, seqs=None, vote=(2, 3), fps=4.0):
    tier = TieredVision(ScriptedPipeline(statuses), FakeCloud(), cold_search_after=99,
                        frame_vote=vote)
    for i in range(len(statuses)):
        seq = seqs[i] if seqs else i + 1
        tier({"image_base64": "x", "image_width": 640, "captured_at": T0 + seq / fps,
              "metadata": {"seq": seq}})
    return tier


def test_two_of_three_fires_through_one_missed_frame():
    tier = _vote_run([ABSENT, DETECTED, ABSENT, DETECTED])
    assert tier.stats.triggers.get(TRIGGER_CANDIDATE) == 1
    off = TieredVision(ScriptedPipeline([ABSENT, DETECTED, ABSENT, DETECTED]), FakeCloud(),
                       cold_search_after=99)
    for i in range(4):
        off({"image_base64": "x", "image_width": 640})
    assert TRIGGER_CANDIDATE not in off.stats.triggers   # off: today's rule, no fire


def test_one_frame_read_twice_is_one_vote():
    # Review: the same seq read twice must not make 2 of 3 out of one look.
    tier = _vote_run([ABSENT, DETECTED, DETECTED], seqs=[1, 2, 2])
    assert TRIGGER_CANDIDATE not in tier.stats.triggers
    assert tier.stats.vote["detected"] == 1


def test_skipped_frames_are_counted():
    tier = _vote_run([ABSENT, ABSENT, ABSENT], seqs=[1, 4, 5])
    assert tier.stats.frames_skipped == 2


def test_frame_vote_names():
    from brain.tiered import parse_frame_vote
    assert parse_frame_vote("off") is None and parse_frame_vote(None) is None
    assert parse_frame_vote("2of3") == (2, 3) and parse_frame_vote("3of5") == (3, 5)
    with pytest.raises(ValueError):
        parse_frame_vote("4of3")
