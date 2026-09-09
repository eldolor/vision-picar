"""
tests/test_perceive_lab.py

The candidate perception backends (`PLAN-onboard-perception.md` 4.11's open
question), against fakes.

Same rule as `tests/test_perceive.py`: **no model is loaded here.** Grounding
DINO, OWLv2, SAM and YOLO-World are hundreds of megabytes each and none of
them is a part this robot will carry -- they exist to answer *"would a Jetson
buy anything"* off the robot. What is testable without them is the part that
would be wrong silently: which pipeline a spec string builds, and how an
open-vocabulary detector's output is turned into the same `Perception` the
shipped pipeline emits.

The model calls themselves are `# pragma: no cover` and are exercised by
`python -m control.perception_eval score --detector <name>` against the real
corpus, which is how the numbers in 4.11 are produced.

Run with: pytest tests/test_perceive_lab.py -v
"""

import base64

import pytest

from brain.perceive import (ABSENT, DETECTED, UNAVAILABLE, Box, Detection,
                            PerceptionPipeline)
from brain.perceive_lab import (DEFAULT_OPEN_VOCAB_CONFIDENCE, NullDetector,
                                OpenVocabPipeline, pipeline_for_spec)

IMAGE = base64.b64encode(b"not-really-a-jpeg").decode()


def frame(width=640, **extra):
    f = {"image_base64": IMAGE, "media_type": "image/jpeg", "image_width": width}
    f.update(extra)
    return f


class FakeOpenVocab:
    """A text-conditioned detector that answers from a script."""

    weights = "fake-open-vocab"
    model_name = "fake-open-vocab"

    def __init__(self, detections=(), error=None):
        self.detections = list(detections)
        self.error = error
        self.asked = []

    def detect_text(self, image, text):
        self.asked.append(text)
        if self.error:
            raise self.error
        return self.detections


def _box(x1=0.0, confidence=0.9, label="bottle"):
    return Detection(box=Box(x1, 0.0, x1 + 20.0, 20.0), label=label,
                     confidence=confidence)


# ---------------------------------------------------------------------------
# OpenVocabPipeline: same output shape as the shipped one, different evidence
# ---------------------------------------------------------------------------

def test_the_target_string_goes_into_the_detector():
    """That is the whole difference: no crops, no distractors, no CLIP. 4.11
    compared YOLO-World as a REPLACEMENT for the pipeline rather than as a
    stage of it, because that is the shape of the "run arbitrary Hugging Face
    models" argument."""
    backend = FakeOpenVocab([_box()])
    OpenVocabPipeline(backend, "blue bottle").perceive(frame())
    assert backend.asked == ["blue bottle"]


def test_a_confident_box_is_detected():
    p = OpenVocabPipeline(FakeOpenVocab([_box(confidence=0.9)]),
                          "blue bottle").perceive(frame())
    assert p.status == DETECTED
    assert p.best.detection.confidence == pytest.approx(0.9)


def test_no_box_is_absent_rather_than_unavailable():
    """The model looked and found nothing. That is information, and 1.12's
    tri-state exists so it cannot be confused with not having looked."""
    p = OpenVocabPipeline(FakeOpenVocab([]), "blue bottle").perceive(frame())
    assert p.status == ABSENT


def test_a_weak_box_is_absent_but_keeps_its_candidates():
    """Kept so the corpus sweep can re-threshold without re-running the
    model -- `control/perception_eval.py` scores once and applies the gate
    afterwards, and a discarded candidate is a gate baked in."""
    weak = _box(confidence=DEFAULT_OPEN_VOCAB_CONFIDENCE - 0.01)
    p = OpenVocabPipeline(FakeOpenVocab([weak]), "blue bottle").perceive(frame())
    assert p.status == ABSENT
    assert len(p.candidates) == 1


def test_the_score_to_threshold_on_is_the_confidence_not_a_probability():
    """There are no distractors and no softmax here, so `probability` is
    meaningless and would read as 1.0 for any single box. A table that
    compared one against the shipped pipeline's P would be comparing two
    different quantities."""
    from control.perception_eval import METRIC_CONFIDENCE, best_score

    p = OpenVocabPipeline(FakeOpenVocab([_box(confidence=0.42)]),
                          "x").perceive(frame())
    assert best_score(p, METRIC_CONFIDENCE) == pytest.approx(0.42)


def test_a_wedged_model_is_unavailable_and_not_an_empty_room():
    p = OpenVocabPipeline(FakeOpenVocab(error=RuntimeError("cuda gone")),
                          "x").perceive(frame())
    assert p.status == UNAVAILABLE
    assert "cuda gone" in p.reason


def test_an_undecodable_frame_is_unavailable():
    p = OpenVocabPipeline(FakeOpenVocab([_box()]), "x").perceive(
        {"media_type": "image/jpeg"})
    assert p.status == UNAVAILABLE


def test_the_bearing_comes_off_the_box_and_the_frames_own_width():
    """Same rule as the shipped pipeline: a bearing scaled by a guessed
    width is wrong by exactly the ratio nobody checked."""
    centred = OpenVocabPipeline(FakeOpenVocab([_box(x1=310.0)]),
                                "x").perceive(frame(width=640))
    assert centred.bearing_deg == pytest.approx(0.0, abs=1.0)
    left = OpenVocabPipeline(FakeOpenVocab([_box(x1=0.0)]),
                             "x").perceive(frame(width=640))
    assert left.bearing_deg < -20


def test_no_width_means_no_bearing_rather_than_a_wrong_one():
    p = OpenVocabPipeline(FakeOpenVocab([_box()]), "x").perceive(
        {"image_base64": IMAGE, "media_type": "image/jpeg"})
    assert p.bearing_deg is None


def test_the_strongest_box_wins():
    p = OpenVocabPipeline(
        FakeOpenVocab([_box(x1=0, confidence=0.4), _box(x1=100, confidence=0.8)]),
        "x").perceive(frame())
    assert p.best.detection.confidence == pytest.approx(0.8)


def test_an_empty_target_is_refused():
    with pytest.raises(ValueError):
        OpenVocabPipeline(FakeOpenVocab(), "   ")


# ---------------------------------------------------------------------------
# NullDetector -- the row that made the floor mask's finding legible
# ---------------------------------------------------------------------------

def test_the_null_detector_proposes_nothing():
    """The floor mask measured 59% alone, 86% for the detector alone and 94%
    together -- *worse alone and better together*, which only a
    proposer-only row can show. Any new proposer gets the same three rows or
    it cannot be compared with it."""
    assert list(NullDetector().detect(b"x", 0.1)) == []


def test_a_proposer_only_pipeline_still_scores_the_proposers_regions():
    class OneRegion:
        model_name = "fake-proposer"

        def propose(self, image):
            return [Box(0, 0, 10, 10)]

    class Scorer:
        model_name = "fake-clip"

        def score(self, image, box, texts):
            return [0.9] + [0.0] * (len(texts) - 1)

    pipeline = PerceptionPipeline(NullDetector(), Scorer(), "blue bottle",
                                  proposer=OneRegion())
    assert pipeline.perceive(frame()).status == DETECTED


# ---------------------------------------------------------------------------
# pipeline_for_spec -- the two-line substitution 4.11 asks for
# ---------------------------------------------------------------------------

def test_an_open_vocabulary_detector_may_not_be_given_a_proposer():
    """It takes the target string into the detector, so there is no crop
    stage for a proposer to feed. Refused with the reason rather than
    silently ignored -- a config that quietly dropped the proposer would
    produce a table row labelled with a crop source it never used."""
    with pytest.raises(ValueError, match="no crop stage"):
        pipeline_for_spec("blue bottle", detector="gdino", proposer="sam")


def test_an_unknown_proposer_is_named():
    with pytest.raises(ValueError, match="unknown proposer"):
        pipeline_for_spec("blue bottle", proposer="lidar")


def test_the_open_vocabulary_names_are_the_ones_4_11_left_untried():
    """Grounding DINO, OWLv2 and SAM are named in 4.11 as the models that
    were NOT tried and that a Jetson would be bought for. If a name here
    drifts, the evidence stops matching the argument it was gathered for."""
    from brain.perceive_lab import OPEN_VOCAB, OPEN_VOCAB_BACKENDS

    assert set(OPEN_VOCAB) == {"gdino", "owlv2", "yoloworld"}
    assert set(OPEN_VOCAB_BACKENDS) == set(OPEN_VOCAB)


def test_every_candidate_backend_is_too_heavy_for_the_part_and_that_is_the_point():
    """A guard against a confusing future edit rather than a behaviour test:
    nothing in this module is a shipped part, and `brain/perceive.py` is
    where a promoted model goes (4.7's promotion rule). If something here
    ever becomes the robot's detector it should move, not be re-exported."""
    import brain.perceive as shipped

    from brain.perceive_lab import (GroundingDino, Owlv2, SamProposer,
                                    YoloWorld)
    for cls in (GroundingDino, Owlv2, SamProposer, YoloWorld):
        assert not hasattr(shipped, cls.__name__), (
            f"{cls.__name__} is exported from brain/perceive.py -- promote it "
            "deliberately or leave it in the lab")


def test_a_backend_that_fails_on_every_frame_reports_unavailable_not_absent():
    """The bug this file's OWLv2 backend actually shipped with, 2026-09-08.

    `Owlv2Processor` has only `post_process_grounded_object_detection`; the
    `Owlv2ImageProcessor` one layer down has only `post_process_object_
    detection`. Calling the wrong one raised on every frame, and the run came
    back **299 `unavailable` out of 299**.

    That is the whole value of 1.12's third state, arriving on the harness
    rather than on the robot: had the failure been folded into `absent`, the
    table would have read *"OWLv2: 0% recall, 0 false positives"* -- a clean,
    publishable, entirely false finding about a model that was never asked a
    question. `control/perception_eval.py` prints the count with an arrow
    pointing at it for the same reason.
    """
    backend = FakeOpenVocab(error=AttributeError("no such post_process"))
    pipeline = OpenVocabPipeline(backend, "blue bottle")
    results = [pipeline.perceive(frame()) for _ in range(3)]
    assert all(r.status == UNAVAILABLE for r in results)
    assert not any(r.status == ABSENT for r in results)
