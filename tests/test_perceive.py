"""
tests/test_perceive.py

Phase P1 (`PLAN-onboard-perception.md` 4.10) -- the on-board perception
pipeline, off the robot.

**Everything here runs against fakes, deliberately.** `ultralytics` and
`torch` are a multi-gigabyte install, and the repo's standing rule is that
the automated suite never needs the expensive thing: API calls are mocked,
paid runs are demo scripts, and now the models are protocols. What is
tested here is the logic that decides *which crops are scored*, *what
counts as a match*, and *how the three states are told apart* -- none of
which is the model's job, and all of which is where a wrong answer would
be invisible.

The model itself is exercised by hand, against real pixels, per the module
docstring. No test in this file can tell you whether YOLO finds a backpack
at 10cm; that is 1.16 #10's corpus and it is the whole point of the
harness rather than a gap in it.

Run with: pytest tests/test_perceive.py -v
"""

import base64

import pytest

from brain.perceive import (
    ABSENT,
    COCO_CLASSES,
    CROP_LABEL_GATE,
    CROP_LOW_CONFIDENCE,
    DETECTED,
    UNAVAILABLE,
    Box,
    Detection,
    PerceptionPipeline,
    PerceptionUnavailable,
    CROP_PATH_AUTO,
    LOW_CONFIDENCE,
    DEFAULT_MATCH_PROBABILITY,
    coco_class_for,
)

IMAGE = base64.b64encode(b"not-really-a-jpeg").decode()


def frame(image=IMAGE, width=640, **extra):
    f = {"image_base64": image, "media_type": "image/jpeg",
         "image_width": width}
    f.update(extra)
    return f


class FakeDetector:
    """Returns a fixed list, filtered by confidence like a real one."""

    def __init__(self, detections=(), fail=None):
        self.detections = list(detections)
        self.fail = fail
        self.calls = []

    def detect(self, image, confidence):
        self.calls.append(confidence)
        if self.fail:
            raise self.fail
        return [d for d in self.detections if d.confidence >= confidence]


class FakeScorer:
    """Scores by a per-label table: {label: (target_score, distractor)}."""

    def __init__(self, table=None, fail=None):
        self.table = table or {}
        self.fail = fail
        self.scored = []

    def score(self, image, box, texts):
        if self.fail:
            raise self.fail
        self.scored.append(box)
        target, distractor = self.table.get("_default", (0.2, 0.2))
        for label, pair in self.table.items():
            if label != "_default" and abs(box.x1 - pair[2]) < 1e-9:
                target, distractor = pair[0], pair[1]
        return [target] + [distractor] * (len(texts) - 1)


def det(label, conf=0.9, x1=100.0, w=50.0):
    return Detection(box=Box(x1, 100.0, x1 + w, 200.0), label=label,
                     confidence=conf)


def pipeline(target, detections=(), table=None, **kwargs):
    """4.2's own rule, selected explicitly.

    `crop_path` defaulted to this until 2026-09-07, when two rig walks
    measured the label gate losing most of the true positives and the
    default flipped to the open-vocabulary path. These tests are about the
    RULE, not about which one ships, so they name it -- otherwise flipping
    the default silently stops testing 4.2 at all."""
    kwargs.setdefault("crop_path", CROP_PATH_AUTO)
    return PerceptionPipeline(FakeDetector(detections), FakeScorer(table),
                              target, **kwargs)


# ---------- 4.2: the class list decides who proposes ----------


def test_a_coco_target_takes_the_label_gate_and_an_unknown_one_does_not():
    """The rule the whole section rests on. `backpack` is in COCO's 80 and
    `charging cable` is not, and that -- not a fallback after failure --
    is what picks the crop path."""
    assert pipeline("red backpack").crop_source == CROP_LABEL_GATE
    assert pipeline("charging cable").crop_source == CROP_LOW_CONFIDENCE


def test_the_open_vocabulary_path_asks_the_detector_for_weaker_proposals():
    """4.2's low-confidence trick. The threshold is what makes an
    out-of-vocabulary object surface at all, since an anchor-free YOLO has
    no objectness score to read instead."""
    gated = pipeline("red backpack")
    open_vocab = pipeline("charging cable")
    gated.perceive(frame())
    open_vocab.perceive(frame())
    assert gated.detector.calls[0] > open_vocab.detector.calls[0]


def test_the_label_gate_keeps_only_the_targets_class():
    """The gate is a cheap prefilter, which is what keeps CLIP's work down
    to a handful of crops per frame where the target has a COCO word."""
    p = pipeline("red backpack",
                 [det("backpack", x1=100.0), det("chair", x1=200.0),
                  det("person", x1=300.0)],
                 {"_default": (0.9, 0.2)})
    p.perceive(frame())
    assert len(p.scorer.scored) == 1
    assert p.scorer.scored[0].x1 == 100.0


def test_the_open_vocabulary_path_scores_every_proposal_regardless_of_label():
    """No word to gate on, so CLIP decides identity outright."""
    p = pipeline("charging cable",
                 [det("microwave", 0.08, x1=100.0), det("book", 0.06, x1=200.0)],
                 {"_default": (0.9, 0.2)})
    p.perceive(frame())
    assert len(p.scorer.scored) == 2


def test_every_stage_zero_target_is_a_coco_class():
    """4.2's finding that lowers the risk, pinned. If this ever fails, the
    zero-training claim for the Stage 0 targets has stopped being true."""
    assert coco_class_for("red backpack") == "backpack"
    assert coco_class_for("the blue bottle") == "bottle"


def test_the_longest_coco_match_wins():
    """"wine glass" must not be read as some shorter entry -- the crop gate
    would then filter on the wrong class and silently score nothing."""
    assert coco_class_for("a wine glass on the table") == "wine glass"
    assert len(COCO_CLASSES) == 80


# ---------- 1.12: three states, and the one that must not be confused ----------


def test_a_good_frame_with_nothing_in_it_is_absent_not_unavailable():
    """`absent` is information -- the frame was fine and the thing is not
    there. A mission may act on it."""
    p = pipeline("red backpack", [det("chair")], {"_default": (0.9, 0.2)})
    result = p.perceive(frame())
    assert result.status == ABSENT
    assert result.matched is False


def test_a_frame_with_no_image_is_unavailable_not_absent():
    """The failure this designs out: a wedged capture reading as "the
    target is gone", ending a mission `lost_target` when the camera died.
    Same class as M9, and the reason M3's tri-state exists one sensor
    over."""
    p = pipeline("red backpack")
    assert p.perceive({"media_type": "image/jpeg"}).status == UNAVAILABLE


def test_undecodable_image_bytes_are_unavailable():
    p = pipeline("red backpack")
    result = p.perceive(frame(image="!!!not base64!!!"))
    assert result.status == UNAVAILABLE
    assert "decode" in result.reason


def test_a_wedged_detector_is_unavailable_and_never_absent():
    """A model that raised is not a room that is empty. Any exception, not
    just the declared one -- a third-party model can fail in ways this
    module has never heard of, and every one of them means 'could not
    tell'."""
    p = PerceptionPipeline(FakeDetector(fail=RuntimeError("cuda gone")),
                           FakeScorer(), "red backpack")
    result = p.perceive(frame())
    assert result.status == UNAVAILABLE
    assert "detector failed" in result.reason


def test_a_wedged_scorer_is_unavailable_too():
    p = PerceptionPipeline(FakeDetector([det("backpack")]),
                           FakeScorer(fail=RuntimeError("no weights")),
                           "red backpack")
    assert p.perceive(frame()).status == UNAVAILABLE


def test_a_scorer_returning_the_wrong_number_of_scores_is_unavailable():
    """Silent misalignment between texts and scores would make the margin
    meaningless while still producing a number. Refuse instead."""

    class ShortScorer:
        def score(self, image, box, texts):
            return [0.9]

    p = PerceptionPipeline(FakeDetector([det("backpack")]), ShortScorer(),
                           "red backpack")
    assert p.perceive(frame()).status == UNAVAILABLE


# ---------- the margin, not the similarity ----------


def test_a_match_needs_to_beat_the_distractors_not_a_bare_threshold():
    """CLIP returns a similarity, not a probability, so a threshold with
    nothing to compare against will always pick something. The margin is
    what the decision gates on -- a crop that scores 0.9 against the target
    and 0.9 against "a wall" has told you nothing."""
    strong = pipeline("red backpack", [det("backpack")], {"_default": (0.9, 0.2)})
    assert strong.perceive(frame()).status == DETECTED

    ambiguous = pipeline("red backpack", [det("backpack")], {"_default": (0.9, 0.89)})
    assert ambiguous.perceive(frame()).status == ABSENT


def test_the_best_candidate_is_the_one_with_the_widest_margin():
    """Two backpacks in frame; the mission wants the red one. This is the
    "this one, not that one" 1.10 says CLIP buys over a bare COCO label."""
    p = pipeline(
        "red backpack",
        [det("backpack", x1=100.0), det("backpack", x1=400.0)],
        {"_default": (0.3, 0.25), "red": (0.9, 0.2, 400.0)},
    )
    result = p.perceive(frame())
    assert result.status == DETECTED
    assert result.best.detection.box.x1 == 400.0


def test_the_crop_count_is_capped():
    """2.9's gate. Scoring every proposal is what turns a 61%-duty
    schedule into an over-budget one; off the robot it is merely slow."""
    p = pipeline("charging cable",
                 [det("book", 0.06, x1=float(i * 10)) for i in range(20)],
                 {"_default": (0.9, 0.2)}, max_crops=3)
    p.perceive(frame())
    assert len(p.scorer.scored) == 3


# ---------- bearing ----------


def test_bearing_is_measured_from_the_centre_of_frame():
    """1.11 gives bearing to the detector because a bounding box is a
    bearing and "slightly to the left" is not."""
    left = pipeline("red backpack", [det("backpack", x1=0.0, w=40.0)],
                    {"_default": (0.9, 0.2)}, hfov_deg=66.0)
    right = pipeline("red backpack", [det("backpack", x1=600.0, w=40.0)],
                     {"_default": (0.9, 0.2)}, hfov_deg=66.0)
    assert left.perceive(frame()).bearing_deg < 0
    assert right.perceive(frame()).bearing_deg > 0


def test_a_centred_target_reads_about_zero():
    p = pipeline("red backpack", [det("backpack", x1=310.0, w=20.0)],
                 {"_default": (0.9, 0.2)}, hfov_deg=66.0)
    assert p.perceive(frame(width=640)).bearing_deg == pytest.approx(0.0, abs=0.5)


def test_the_pan_angle_rides_with_the_frame_and_shifts_the_bearing():
    """1.15.3, amending 1.11: both servo angles must ride in the frame's
    own metadata, or a servo still moving corrupts the bearing silently.
    Reading pan from the frame rather than from the servo is what makes
    that impossible rather than merely unlikely."""
    p = pipeline("red backpack", [det("backpack", x1=310.0, w=20.0)],
                 {"_default": (0.9, 0.2)}, hfov_deg=66.0)
    straight = p.perceive(frame()).bearing_deg
    panned = p.perceive(frame(pan_deg=30.0)).bearing_deg
    assert panned == pytest.approx(straight + 30.0, abs=0.5)


def test_a_frame_that_does_not_say_how_wide_it_is_reports_no_bearing():
    """An in-frame bearing needs the frame's width. Guessing one would put
    a number on the wire that nothing measured -- M7's rule."""
    p = pipeline("red backpack", [det("backpack")], {"_default": (0.9, 0.2)})
    f = frame()
    del f["image_width"]
    assert p.perceive(f).bearing_deg is None


# ---------- the published shape ----------


def test_the_published_shape_marks_itself_as_not_synthesised():
    """1.12 requires a synthesised detection to be marked as such, so no
    run can look as though it exercised perception it never had. This is
    the other side of that: a real one says so too."""
    p = pipeline("red backpack", [det("backpack")], {"_default": (0.9, 0.2)})
    published = p.perceive(frame()).as_dict()
    assert published["synthesised"] is False
    assert published["status"] == DETECTED
    assert published["crop_source"] == CROP_LABEL_GATE
    assert published["label"] == "backpack"


def test_a_pipeline_needs_a_target():
    with pytest.raises(ValueError):
        pipeline("   ")


def test_the_real_backends_say_what_to_install_rather_than_traceback():
    """A missing optional dependency is a setup problem and should read
    like one. Skips if the dependency happens to be present."""
    from brain.perceive import YoloDetector

    try:
        import ultralytics  # noqa: F401
        pytest.skip("ultralytics is installed -- nothing to assert")
    except ImportError:
        pass
    with pytest.raises(PerceptionUnavailable) as exc:
        YoloDetector()
    assert "requirements-perception.txt" in str(exc.value)


# ---------- the bearing, which had never once been a number ----------
#
# Found 2026-09-07 by running a real walk end to end: `bearing_deg` was
# `None` on every frame, because **no RobotInterface backend publishes
# `image_width`** and `_bearing()` needs a width to place a box in the
# frame. 1.11 makes the bearing perception's own answer to "which way is
# it", so an output that can never populate is a gap, not a default.
#
# The width is read off the image itself rather than widening the frame
# contract across every backend and the conformance suite.

pil_image = pytest.importorskip(
    "PIL.Image", reason="the width fallback needs Pillow (perception extras)")


def real_png(width: int, height: int = 32) -> str:
    """A real image of a known size, base64'd -- so the width is READ and
    not asserted into existence."""
    import io

    buf = io.BytesIO()
    pil_image.new("RGB", (width, height), (10, 20, 30)).save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode()


def undeclared(image):
    """A frame shaped like sim/replay_robot.py's: pixels, media type, room
    -- and no width, which is the whole defect."""
    return {"image_base64": image, "media_type": "image/jpeg", "room": "unknown"}


def test_the_bearing_is_a_number_even_when_the_frame_declares_no_width():
    """The defect exactly. Every recorded walk in the corpus produced a null
    bearing, so 1.11's "which way is it" had never been answered once."""
    pipeline = PerceptionPipeline(
        FakeDetector([det("backpack", x1=150.0, w=40.0)]),
        FakeScorer({"_default": (0.9, 0.1)}), "red backpack")
    perception = pipeline.perceive(undeclared(real_png(200)))

    assert perception.status == DETECTED
    assert perception.bearing_deg is not None, "bearing is still null"
    # The box sits right of centre in a 200px frame, so the bearing does too.
    assert perception.bearing_deg > 0


def test_a_declared_width_still_wins_over_the_image():
    """A backend that declares one knows something the pixels cannot -- a
    frame downscaled after its boxes were computed, say."""
    image = real_png(200)
    pipeline = PerceptionPipeline(
        FakeDetector([det("backpack", x1=150.0, w=40.0)]),
        FakeScorer({"_default": (0.9, 0.1)}), "red backpack")

    from_image = pipeline.perceive(undeclared(image)).bearing_deg
    declared = pipeline.perceive(
        dict(undeclared(image), image_width=400)).bearing_deg
    # Same box, a frame declared twice as wide: it sits nearer the centre.
    assert abs(declared) < abs(from_image)


def test_an_unreadable_image_leaves_the_bearing_null_rather_than_failing():
    """A width we cannot read is a bearing we do not have, which is already
    a state -- it must never fail a perception step the detector completed.
    Every other test in this file passes bytes PIL cannot open, and they all
    have to keep working."""
    pipeline = PerceptionPipeline(
        FakeDetector([det("backpack")]),
        FakeScorer({"_default": (0.9, 0.1)}), "red backpack")
    perception = pipeline.perceive(undeclared(IMAGE))

    assert perception.status == DETECTED
    assert perception.bearing_deg is None


# ---------- forcing 4.2's crop path (measured 2026-09-07) ----------
#
# 4.2's rule is that the target's COCO word picks the path. The first
# VALID rig walk -- camera at floor height, target on the floor -- found
# that rule losing 11 of 18 true positives, and losing them at the worst
# possible moment: **at close range YOLO relabels the object.** A bottle
# 30cm from a 10cm camera is a large blue cylinder, which COCO calls a
# `vase`, so the label gate discarded every frame from the approach
# onward. Forcing the open-vocabulary path recovered 18/18 with no false
# positives.
#
# The override does not overturn 4.2 -- it makes it measurable.


def test_the_open_vocabulary_path_is_the_default_now():
    """Flipped 2026-09-07 on two rig walks. 4.2's rule is still selectable
    and still right about who proposes -- but gating crops on the label
    costs most of the true positives, because at close range the detector
    relabels the object."""
    p = PerceptionPipeline(FakeDetector(), FakeScorer(), "blue bottle")
    assert p.crop_source == CROP_LOW_CONFIDENCE
    assert p.coco_class == "bottle", "the COCO word is still recorded, it just does not gate"


def test_auto_is_still_available_and_is_still_4_2s_rule():
    kw = dict(detector=FakeDetector(), scorer=FakeScorer(), target="blue bottle")
    assert PerceptionPipeline(crop_path=CROP_PATH_AUTO, **kw).crop_source == CROP_LABEL_GATE
    assert PerceptionPipeline(detector=FakeDetector(), scorer=FakeScorer(),
                              crop_path=CROP_PATH_AUTO,
                              target="charging cable").crop_source == CROP_LOW_CONFIDENCE


def test_forcing_the_open_vocabulary_path_stops_the_label_gating():
    """The measured fix. A `vase` box must survive when the target is a
    bottle, because at 10cm that is what the bottle is labelled."""
    detector = FakeDetector([det("vase"), det("bottle", x1=300.0)])
    forced = PerceptionPipeline(detector, FakeScorer({"_default": (0.9, 0.1)}),
                                "blue bottle", crop_path=CROP_LOW_CONFIDENCE)
    auto = PerceptionPipeline(FakeDetector([det("vase"), det("bottle", x1=300.0)]),
                              FakeScorer({"_default": (0.9, 0.1)}), "blue bottle",
                              crop_path=CROP_PATH_AUTO)

    assert len(forced.perceive(frame()).candidates) == 2
    assert len(auto.perceive(frame()).candidates) == 1, "auto should keep only the bottle"


def test_forcing_the_open_vocabulary_path_lowers_the_detector_threshold():
    """It is not only the gate: 4.2's path B harvests low-confidence
    proposals, so the confidence handed to the detector changes with it."""
    detector = FakeDetector([det("vase", conf=0.08)])
    p = PerceptionPipeline(detector, FakeScorer({"_default": (0.9, 0.1)}),
                           "blue bottle", crop_path=CROP_LOW_CONFIDENCE)
    p.perceive(frame())
    assert detector.calls == [LOW_CONFIDENCE]


def test_forcing_the_label_gate_on_a_target_with_no_coco_word_is_refused():
    """It would discard every proposal and report `absent` on every frame --
    a configuration that cannot work should say so at construction, not
    look like an empty room."""
    with pytest.raises(ValueError) as exc:
        PerceptionPipeline(FakeDetector(), FakeScorer(), "charging cable",
                           crop_path=CROP_LABEL_GATE)
    assert "COCO word" in str(exc.value)


def test_an_unknown_crop_path_is_refused():
    with pytest.raises(ValueError):
        PerceptionPipeline(FakeDetector(), FakeScorer(), "bottle", crop_path="magic")


# ---------- the gate is a probability, not a margin (measured 2026-09-07) ----------
#
# Two valid rig walks showed the raw margin cannot be thresholded
# consistently, because a CLIP similarity is not comparable across text
# queries: margin 0.05 detected 0 of 18 on "blue bottle" and 8 of 31 on
# "red backpack"; margin 0.02 fixed the bottle and gave the backpack false
# positives. A softmax over the same scores separates both at one number.


def scored(target_score, *distractors):
    """A pipeline whose single crop gets exactly these scores."""
    class Scorer:
        def score(self, image, box, texts):
            # Padded to the full text tuple: the pipeline treats a short
            # score list as a broken scorer (`unavailable`), which is
            # correct and not what these tests are about.
            vals = list(distractors) or [0.0]
            while len(vals) < len(texts) - 1:
                vals.append(vals[-1])
            return [target_score] + vals[:len(texts) - 1]
    return PerceptionPipeline(FakeDetector([det("bottle")]), Scorer(), "blue bottle")


def test_the_probability_is_computed_over_every_score_not_just_the_winner():
    """A softmax over two numbers is a different function from a softmax over
    seven, so the whole score vector has to be carried."""
    p = scored(0.31, 0.28, 0.27, 0.26, 0.25, 0.24, 0.23)
    best = max(p.perceive(frame()).candidates, key=lambda c: c.probability)
    assert len(best.scores) == 7
    assert 0.0 < best.probability < 1.0


def test_a_small_margin_over_many_weak_distractors_still_passes():
    """The bottle case. Margin +0.03 is under the old 0.05 gate, but the
    target beats every distractor, which is what the question actually is."""
    p = scored(0.31, 0.28, 0.20, 0.19, 0.18, 0.17, 0.16)
    r = p.perceive(frame())
    assert r.status == DETECTED
    assert r.best.margin < 0.05, "this is exactly the case the old gate rejected"
    assert r.best.probability >= 0.8


def test_a_crop_the_distractors_beat_is_absent():
    p = scored(0.20, 0.31, 0.30)
    r = p.perceive(frame())
    assert r.status == ABSENT
    assert "P " in r.reason, r.reason


def test_the_margin_override_still_works_for_sweeping_a_corpus():
    """Kept so the raw number can be swept on a new corpus -- and when set,
    it wins, so a walk scored under it is scored under it entirely."""
    p = scored(0.31, 0.28, 0.20)
    p.match_margin = 0.05
    r = p.perceive(frame())
    assert r.status == ABSENT
    assert "margin" in r.reason


def test_both_numbers_are_published_so_a_walk_can_be_rescored():
    p = scored(0.31, 0.28, 0.20)
    d = p.perceive(frame()).as_dict()
    assert d["match_probability"] is not None
    assert d["match_margin"] is not None


# ---------- 4.2's second crop source: the floor mask (measured 2026-09-07) ----------
#
# Of the 11 labelling errors in the three-walk corpus, NINE were crop
# proposals and none were matching. The detector can only propose what COCO
# has a word for, and stops proposing at all once the object fills the
# view. A floor mask proposes by geometry instead: not floor, standing on
# the floor, not reaching the top of the frame.
#
# Unioned with the detector, never substituted -- measured at P >= 0.8
# against the adjudicated labels: detector 55/64, mask alone 38/64, both
# 60/64. Worse alone, better together, because they fail on different
# frames.


class FakeProposer:
    """Region proposals with no class attached, which is the whole point."""

    def __init__(self, boxes=()):
        self.boxes = list(boxes)
        self.calls = 0

    def propose(self, image):
        self.calls += 1
        return self.boxes


def test_floor_regions_are_added_to_the_detectors_boxes_not_swapped_for_them():
    """The measurement says union. A pipeline that replaced the detector
    would score 59% where it now scores 94%."""
    proposer = FakeProposer([Box(400.0, 100.0, 480.0, 200.0)])
    p = PerceptionPipeline(FakeDetector([det("bottle")]),
                           FakeScorer({"_default": (0.9, 0.1)}), "blue bottle",
                           proposer=proposer)
    result = p.perceive(frame())
    assert proposer.calls == 1
    assert len(result.candidates) == 2, "the detector's box and the region's"


def test_a_pipeline_with_no_proposer_never_asks_for_one():
    """Off by default: it is a third model per frame, and 2.9's budget says
    segmentation must not run at the detector's rate on the real part."""
    p = PerceptionPipeline(FakeDetector([det("bottle")]),
                           FakeScorer({"_default": (0.9, 0.1)}), "blue bottle")
    assert p.proposer is None
    assert p.perceive(frame()).status == DETECTED


def test_a_region_with_no_class_survives_when_the_detector_proposes_nothing():
    """The two close-ups in the corpus: the target fills the view and the
    detector proposes NOTHING at all. The mask is the only thing that can
    put a crop there."""
    p = PerceptionPipeline(FakeDetector([]),
                           FakeScorer({"_default": (0.9, 0.1)}), "blue bottle",
                           proposer=FakeProposer([Box(0.0, 0.0, 600.0, 400.0)]))
    assert p.perceive(frame()).status == DETECTED


def test_the_label_gate_plus_a_proposer_is_refused_rather_than_silently_useless():
    """A class-agnostic region has no class to gate on, so the label gate
    discards every one of them -- the exact combination that would look
    configured and do nothing."""
    with pytest.raises(ValueError) as exc:
        PerceptionPipeline(FakeDetector(), FakeScorer(), "blue bottle",
                           crop_path=CROP_LABEL_GATE, proposer=FakeProposer())
    assert "no class to gate on" in str(exc.value)


def test_the_crop_budget_grows_when_a_second_source_is_added():
    """A bug shipped and caught the same day. `max_crops` was sized for one
    crop source; with the floor mask unioned in, the area-descending sort
    let furniture crowd the target out and the bottle walk fell from 18 of
    18 detections to 11. The cap has to scale with the number of sources."""
    from brain.perceive import DEFAULT_MAX_CROPS, DEFAULT_MAX_CROPS_WITH_PROPOSER

    alone = PerceptionPipeline(FakeDetector(), FakeScorer(), "blue bottle")
    both = PerceptionPipeline(FakeDetector(), FakeScorer(), "blue bottle",
                              proposer=FakeProposer())
    assert alone.max_crops == DEFAULT_MAX_CROPS
    assert both.max_crops == DEFAULT_MAX_CROPS_WITH_PROPOSER
    assert both.max_crops > alone.max_crops


def test_an_explicit_crop_budget_still_wins():
    """It is 2.9's per-frame cost knob, so a caller that has a budget must
    be able to state it."""
    p = PerceptionPipeline(FakeDetector(), FakeScorer(), "blue bottle",
                           proposer=FakeProposer(), max_crops=2)
    assert p.max_crops == 2
