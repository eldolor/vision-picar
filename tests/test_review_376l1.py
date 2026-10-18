"""
tests/test_review_376l1.py

`PLAN-ros-alignment.md` 3.76, step L1: the bugs the review of brain
perception (`brain/perceive.py`, `brain/text_encoder.py`,
`brain/perceive_lab.py`, whole files) confirmed. Each test was written
before its fix and seen to fail on `dev` for the reason it names. Fakes
only, as tests/test_perceive.py.
"""

import threading

import pytest

from brain.perceive import (
    ABSENT, CROP_LABEL_GATE, CROP_LOW_CONFIDENCE, CROP_PATH_AUTO, CROP_SOFT_GATE,
    DETECTED, UNAVAILABLE, Box, Detection, OpenVocabCropSource, PerceptionPipeline)
from tests.test_perceive import FakeDetector, FakeScorer, det, frame


# ---------- L1.1: a second frame entered models the first was still in ----------


class FirstCallBlocks(FakeDetector):
    """The first call waits (a new target's text encode: 11-36 s on the
    board, 3.81); later calls return at once."""

    def __init__(self, detections, gate):
        super().__init__(detections)
        self.gate = gate
        self.entered = threading.Event()

    def detect(self, image, confidence):
        first = not self.calls
        out = super().detect(image, confidence)
        if first:
            self.entered.set()
            self.gate.wait(5)
        return out


def test_a_frame_still_in_perception_keeps_the_next_one_out():
    """No lock on the pipeline: a frame abandoned by B3.2's 20 s guard was
    still in the detector when the next tick's frame went in beside it --
    a second text-encoder child, two predicts on one ultralytics model, and
    the first frame's release under the second (all three reviewers)."""
    gate = threading.Event()
    detector = FirstCallBlocks([det("backpack")], gate)
    p = PerceptionPipeline(detector, FakeScorer(), "backpack",
                           crop_path=CROP_LOW_CONFIDENCE)
    first = threading.Thread(target=p.perceive, args=(frame(),), daemon=True)
    first.start()
    assert detector.entered.wait(5)
    try:
        second = p.perceive(frame())
        assert len(detector.calls) == 1, "a second frame entered the detector"
        assert second.status == UNAVAILABLE and "still in perception" in second.reason
    finally:
        gate.set()
        first.join(5)
    assert p.perceive(frame()).status != UNAVAILABLE, "the pipeline stayed shut"


# ---------- L1.2: the crop gates with a text-prompted detector ----------


class TextPrompted:
    """YOLOE's shape: every box is labelled with the prompt itself."""

    weights = "yoloe-11s-seg.pt"

    def detect_text(self, image, text):
        return [Detection(box=Box(100.0, 100.0, 150.0, 200.0), label=text, confidence=0.4)]


def test_auto_with_a_text_prompted_detector_takes_the_open_vocabulary_path():
    """`auto` picked the label gate for a target with a COCO word; YOLOE
    labels every box with the whole target text ("red backpack"), never
    the COCO word, so every frame read `absent` -- the setting the yaml
    tells an operator to try."""
    p = PerceptionPipeline(OpenVocabCropSource(TextPrompted(), "red backpack"),
                           FakeScorer({"_default": (0.95, 0.01)}), "red backpack",
                           crop_path=CROP_PATH_AUTO)
    out = p.perceive(frame())
    assert out.status == DETECTED, (out.status, out.reason)


@pytest.mark.parametrize("path", [CROP_LABEL_GATE, CROP_SOFT_GATE])
def test_a_label_gate_on_a_text_prompted_detector_is_refused(path):
    with pytest.raises(ValueError, match="text-prompted"):
        PerceptionPipeline(OpenVocabCropSource(TextPrompted(), "red backpack"),
                           FakeScorer(), "red backpack", crop_path=path)


def test_a_closed_vocabulary_detector_keeps_the_label_gate():
    p = PerceptionPipeline(FakeDetector([det("backpack")]), FakeScorer(), "red backpack",
                           crop_path=CROP_PATH_AUTO)
    assert p.crop_source == CROP_LABEL_GATE
    assert p.perceive(frame()).status in (ABSENT, DETECTED)


def test_a_wedged_frame_is_a_failure_not_a_blind_search(monkeypatch):
    """Fixes' round 1 (both reviewers): with the lock, a wedged model left
    every later frame `unavailable` -- a successful tick each, the failure
    budget reset, the mission searching blind to its time limit. Past the
    bound the busy answer is raised, for B3.2 to count."""
    import brain.perceive as perceive_mod
    from brain.perceive import PerceptionWedged
    gate = threading.Event()
    detector = FirstCallBlocks([det("backpack")], gate)
    p = PerceptionPipeline(detector, FakeScorer(), "backpack",
                           crop_path=CROP_LOW_CONFIDENCE)
    first = threading.Thread(target=p.perceive, args=(frame(),), daemon=True)
    first.start()
    assert detector.entered.wait(5)
    try:
        assert p.perceive(frame()).status == UNAVAILABLE      # slow, not wedged
        monkeypatch.setattr(perceive_mod, "BUSY_WEDGED_S", 0.0)
        with pytest.raises(PerceptionWedged):
            p.perceive(frame())
    finally:
        gate.set()
        first.join(5)


def test_a_one_word_target_keeps_the_label_gate_on_a_text_prompted_detector():
    """Round 1 (both reviewers): YOLOE's label IS the COCO word for
    "backpack", so the label gate kept every box and still may."""
    p = PerceptionPipeline(OpenVocabCropSource(TextPrompted(), "backpack"),
                           FakeScorer({"_default": (0.95, 0.01)}), "backpack",
                           crop_path=CROP_LABEL_GATE)
    assert p.perceive(frame()).status == DETECTED
