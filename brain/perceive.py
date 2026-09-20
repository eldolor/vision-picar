"""
perceive.py

The on-board perception pipeline -- **off the robot**.

`PLAN-onboard-perception.md` 2.8 walks one mission through three models:
a detector says *where the objects are*, CLIP says *whether this crop is
the thing we want*, and only then does a cloud VLM get asked *what to do
about it*. This module is the first two, running wherever Python runs
instead of on a Hailo.

Phase **P1** of that plan's section 4.10.

## Why this is not waiting for the accelerator

Nothing here needs the Hailo, and that is the point rather than a
concession. The compile chain is PyTorch -> ONNX -> Hailo's Dataflow
Compiler -> HEF, and **only the last step is Hailo-specific** (4.8): the
export, the calibration corpus and the harness that scores a candidate
model over recorded walks are all platform-independent. Running a model to
find out whether it *finds things* needs none of it.

That inverts the usual order in a useful way. 1.10 item 1 asks for the
compile loop before hardware day and warns that without it *"the Hailo is
a fixed-function part and the IMX500 was cheaper"*. This is the four
fifths of that loop which can exist today, and 4.7's promotion rule is the
same idea stated as policy: **experiments run off-robot behind 2.7's
perception seam and are promoted to the Hailo by compile once they have
earned it in replay.** Until this module existed, nothing had ever gone
through that seam.

## What it can and cannot tell you

It runs against **real pixels** -- `sim/replay_robot.py`'s recorded walks
and `sim/teleop_robot.py`'s live phone camera -- never against
`sim/renderer.py`. 1.12 settled that: flat-shaded raycaster walls contain
nothing a COCO detector can find, so the sim synthesises detections from
grid truth and *"the sim leg tests the detector's consumers, never the
detector."* Pointing this at a rendered frame would produce a number about
raycasting, and the tempting repair -- texture-mapping a photograph of a
backpack onto a sprite -- is worse, because "YOLO detects a sprite" reads
as success while measuring nothing. That is the standing-height corpus
error in a new costume.

What it cannot answer at all is throughput. 2.9 budgets the three models
against an 8L's frame time; a laptop is not one, so a timing taken here
says nothing about whether the pipeline fits. Latency, HEF context-switch
cost and anything involving the lidar are hardware day.

## The two crop paths, and why the target string chooses between them

4.2's rule is that **the class list decides who proposes, never who
confirms**. COCO has `backpack` but not `red backpack`, and no word at all
for `router` or `charging cable`. So:

| Target | Crops from | CLIP's job |
|---|---|---|
| in COCO's 80 | boxes already labelled with that class | disambiguate -- "the red one" |
| outside it | low-confidence proposals, labels discarded | decide identity outright |

Chosen once, at construction, from whether the target string mentions a
COCO class. **It is not a fallback chain**, and 4.2 is emphatic about why:
for an out-of-vocabulary object the detector reports *nothing*, which is
indistinguishable from an empty room -- there is no "unknown" class and no
error to trigger on. A failure-triggered handoff would never fire.

The low-confidence path is the **noisiest** of the three sources 4.2
lists, not the cleanest: YOLOv8 and YOLO11 are anchor-free and dropped the
objectness head, so the class score *is* the confidence and thresholding
low harvests weak class activations rather than reading a clean
"something is here" signal. The two better sources -- floor segmentation
and lidar clusters -- need a model this project has not compiled and a
sensor it has not bought, so they are named in `CROP_SOURCES` and not
implemented. Do not let that absence read as a preference.

## Three states, for M3's reason one sensor over

`detected` / `absent` / `unavailable`, exactly as 1.12 requires and for
the same reason the depth grid distinguishes `ZONE_RANGE` /
`ZONE_NO_TARGET` / `ZONE_UNUSABLE`: "nothing there" and "I could not tell"
mean opposite things. The failure this designs out is concrete -- **a
wedged capture reading as "the target is gone"**, ending a mission
`lost_target` when the truth is that the camera died.

## The heavy dependencies are optional on purpose

`ultralytics` and `torch` are a multi-gigabyte install and nothing in the
automated suite may need them, so they are imported lazily inside the two
concrete backends and everything else here is testable against fakes. Same
rule the rest of the repo follows for paid API calls: the logic is
exercised in CI, the model is exercised by hand. See
`requirements-perception.txt`.
"""

from __future__ import annotations

import base64
import binascii
import io
import logging
import math
from dataclasses import dataclass, field
from typing import Optional, Protocol, Sequence

logger = logging.getLogger("perceive")

# The three outcomes a perception step can have (1.12). `ABSENT` is
# information -- the frame was good and the thing is not in it -- and
# `UNAVAILABLE` is the absence of information.
DETECTED = "detected"
ABSENT = "absent"
UNAVAILABLE = "unavailable"

# Where crops come from (4.2). Only the first two are reachable off the
# robot; the other two are listed so the vocabulary is complete and so a
# reader can see what is missing rather than infer that it was rejected.
CROP_LABEL_GATE = "label_gate"
CROP_LOW_CONFIDENCE = "low_confidence"
CROP_SOFT_GATE = "soft_gate"
CROP_FLOOR_MASK = "floor_mask"            # needs 4.3's segmentation model
CROP_LIDAR_CLUSTER = "lidar_cluster"      # needs the sensor
CROP_SOURCES = (CROP_LABEL_GATE, CROP_LOW_CONFIDENCE, CROP_SOFT_GATE,
                CROP_FLOOR_MASK, CROP_LIDAR_CLUSTER)

# Which of 4.2's two reachable paths to take. "auto" is 4.2's own rule --
# the target's COCO word picks the path -- and stays the default.
#
# The override exists because the first VALID rig walk (2026-09-07,
# blue-bottle, camera at floor height, target on the floor) measured the
# auto rule losing 11 of 18 true positives, and losing them exactly where
# it matters most: **at close range YOLO relabels the object.** A bottle
# 30cm from a 10cm-high camera is a large blue cylinder, and COCO's word
# for that is `vase` -- so the label gate, which keeps only crops labelled
# `bottle`, discarded every frame from the approach onward. Forcing the
# open-vocabulary path recovered 18/18 with zero false positives.
#
# This does not overturn 4.2's rule, which is about who PROPOSES and is
# still right in the general case. It makes the rule measurable on one
# walk instead of settled by argument.
CROP_PATH_AUTO = "auto"
CROP_PATHS = (CROP_PATH_AUTO, CROP_LABEL_GATE, CROP_LOW_CONFIDENCE,
              CROP_SOFT_GATE)

# **The default is the open-vocabulary path, since 2026-09-07.** 4.2's rule
# ("auto") is still selectable and still right about who PROPOSES; what two
# valid rig walks measured is that gating the crops on the label costs most
# of the true positives:
#
#   target          auto (label gate)   open vocabulary
#   blue bottle     7/18,  0 false pos  18/18, 0 false pos
#   red backpack    9/31,  1 false pos  25/31, 2 false pos
#
# The cause is identity, not rate: at close range YOLO relabels the object
# (a bottle 30cm from a 10cm camera is a `vase`, once a `refrigerator`), so
# the gate discards exactly the frames where the target fills the view.
#
# **Recall is worth more than precision here, and that is 1.11's design, not
# a preference.** Arbitration is split by question: on-board PROPOSES, the
# cloud CONFIRMS identity. So a false positive costs one deliberation call
# that the VLM then rejects -- money -- while a miss means the robot drives
# past the target. Trading 1 extra false positive for 16 recovered
# detections is the trade this architecture was built to make.
DEFAULT_CROP_PATH = CROP_LOW_CONFIDENCE

# Ship `s`, not `n` -- 4.3.1 measured +7.6 mAP on-chip for a third of the
# frame rate, and 92 FPS is still 3x the camera. `n` is the day-one
# baseline that needs no HEF (4.4), not the shipped detector.
#
# **Superseded as the DEFAULT on 2026-09-19 (P22), kept as a name.** The
# reasoning above is a Hailo argument -- mAP per HEF context, 92 FPS
# on-chip -- and the Hailo path is closed. YOLO11s is a closed-vocabulary
# COCO detector, which 4.2's label gate exists to work around; YOLOE takes
# the target string and needs no gate at all. Keep this constant: it is
# the comparison baseline every 4.11/P9/P22 row is read against, and the
# `label_gate`/`soft_gate` crop paths are only meaningful for it.
DEFAULT_YOLO_DETECTOR = "yolo11s.pt"

# YOLOE, promoted 2026-09-19 (P22) and PINNED to `11s` by P23 the same day.
# `-seg` is the only published form; only the boxes are read, so the mask
# head is paid for and discarded.
#
# **Why the small one.** P22 ranked `26l` first on 365 frames. P23 re-ran
# the family on all 1234 labelled frames -- P22's set was a 30% subsample --
# and the ordering INVERTED: at a 3-false-positive budget `11s` reads 82%
# against `26l`'s 72%, and at the shipped gate 80% against 76% with two
# false positives against five, at 139ms against 296. It wins on recall,
# precision and latency simultaneously, so there is no trade being made
# here. `26x` is dominated at every budget and 3.6x slower.
#
# **And it needs no gate change**, which `26l` did: `26l` reads 76% at
# DEFAULT_MATCH_PROBABILITY and 91% only at a gate near 0.4, where `11s`
# reads 80% against its own best of 82%. That is why this promotion is one
# constant and not three -- and why it does not increase the
# candidate_sighting triggers that cost money (6.1).
DEFAULT_YOLOE = "yoloe-11s-seg.pt"

DEFAULT_DETECTOR = DEFAULT_YOLOE

# 4.9's choice between the two CLIP encoders Hailo ports, which 4.3 lists
# without choosing: ResNet-50 is 7-10ms per crop against ViT-B/32's 25-50,
# and ~25M parameters against ~88M -- which matters twice over on a part
# with no local DRAM, since non-resident weights re-stream over the one
# PCIe lane (2.9). Off-robot the difference is only speed.
DEFAULT_CLIP = "RN50"

# The floor segmenter (4.3). SegFormer-B0 on ADE20K: ~14MB, and ADE20K is
# one of the few segmentation sets that carries `floor`, `rug` and `earth`
# as classes at all.
DEFAULT_SEGMENTER = "nvidia/segformer-b0-finetuned-ade-512-512"

# Which ADE20K labels count as floor. Words rather than indices because the
# index set is a property of the checkpoint and the words are not.
FLOOR_WORDS = ("floor", "rug", "carpet", "earth", "ground")

# Keep the detector's own boxes when the target has a COCO word.
DEFAULT_CONFIDENCE = 0.25
# 4.2's class-agnostic path. Deliberately far below it.
LOW_CONFIDENCE = 0.05

# Above this CLIP similarity a crop is a candidate worth a cloud call.
# **Provisional and uncalibrated**, and it must stay that way until it is
# measured on the re-recorded corpus (1.16 #10): CLIP returns a similarity,
# not a probability, so a threshold with no competing strings will always
# pick something. `score_crops()` therefore scores against the target AND a
# set of distractors, and the margin is what this gates on.
DEFAULT_MATCH_MARGIN = 0.05

# CLIP's own logit scale, used to turn the raw similarities over
# (target + distractors) into a probability. Not a tuning knob -- it is the
# constant the model was trained with.
CLIP_LOGIT_SCALE = 100.0

# **The gate, since 2026-09-07.** P(target | this crop, these texts), from a
# softmax over the same scores the margin was computed from -- so it costs
# nothing extra and uses no new information.
#
# The margin above cannot be thresholded consistently, and two valid rig
# walks measured exactly how badly. A raw CLIP similarity is not comparable
# ACROSS text queries -- a well-known property of the model, and this
# project's own data now shows it:
#
#   target          margin gate 0.05     margin gate 0.02
#   blue bottle     0 of 18 detected     18 of 18, 0 false positives
#   red backpack    8 of 31 detected     24 of 31, 2 false positives
#
# There is no single margin that serves both. Under the softmax the same
# two walks separate at one number: the bottle's target frames sit at
# P >= 0.85 against a non-target maximum of 0.39, and the backpack's median
# is 0.96. P >= 0.8 gives 18/18 and 25/31.
#
# **What would falsify this**: a target whose non-target frames also reach
# 0.8. The backpack walk already shows two, so this is a better metric and
# not a solved problem -- the distractor set is what bounds it, which is why
# DEFAULT_DISTRACTORS exists and why near-neighbours ("a vase" for a bottle)
# are the next thing to try.
DEFAULT_MATCH_PROBABILITY = 0.8

# How many crops per frame reach CLIP. One image encode each, so this is
# 2.9's per-frame budget knob -- and it has to grow with the number of crop
# sources. See PerceptionPipeline.max_crops for the measurement.
#
# **4 -> 16 on 2026-09-19 (P23), measured on the shipped detector.** On all
# 1234 labelled frames, `yoloe-11s` reads 258 true positives at 16 and 251 at
# 4, same two false positives, for 139ms against 127 -- so seven frames for
# 12ms. It is a small gain and it is the shipped model's own, which 4 never
# was: every earlier crop-budget row (P18's 8, P20's 8/16/48) was measured
# with the floor mask on and a different detector, and this default was
# reached by nobody's measurement at all.
#
# It does NOT keep growing. P20 found 16 and 48 byte-identical without the
# mask, because an open-vocabulary detector proposes about one box a frame and
# the budget stops binding long before 48.
DEFAULT_MAX_CROPS = 16
# Held at 2x the single-source budget, which is the ratio this pair has always
# had (4/8). **Provisional**: it is the only number in this block not measured
# on the shipped detector, because no YOLOE row had been run with the floor
# mask at all until 2026-09-19 and those rows are what should set it. The
# invariant it must not break is the one test_perceive.py pins -- a second
# crop source needs MORE budget, or it crowds out the first, which is P18's
# area-ranking defect arriving by a different door.
DEFAULT_MAX_CROPS_WITH_PROPOSER = 32

# Scored alongside the target so a similarity has something to be
# relative to. Deliberately bland and household-generic: they exist to
# absorb "some object, but not that one", not to enumerate a house.
DEFAULT_DISTRACTORS = (
    "a wall", "a floor", "a piece of furniture", "a doorway",
    "an empty room", "a household object",
)

# COCO's 80. Kept here rather than read from the model so the crop-path
# decision can be made before any model is loaded -- and so a test can
# make it with no model at all. Verify against the model's own label file
# before trusting it for a fine-tune (4.2 says so).
COCO_CLASSES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
)


def coco_class_for(target: str) -> Optional[str]:
    """The COCO word in a target string, if there is one.

    "red backpack" -> "backpack"; "the blue bottle" -> "bottle";
    "charging cable" -> None. Longest match wins, so "wine glass" is not
    read as "glass"-that-isn't-a-class or shadowed by a shorter entry.

    This is what picks the crop path, and it is deliberately dumb: a
    target the matcher misses simply takes the open-vocabulary path, which
    is correct but slower. A target it matches wrongly would gate crops on
    the wrong class, which is why matching is on whole words.
    """
    words = set(_words(target))
    hits = [c for c in COCO_CLASSES if set(_words(c)) <= words]
    return max(hits, key=len) if hits else None


def _words(text: str) -> list:
    return [w for w in "".join(
        ch.lower() if ch.isalnum() else " " for ch in text).split() if w]


# ---------------------------------------------------------------------------
# The soft label gate, and the vocabulary verdict (added 2026-09-12)
# ---------------------------------------------------------------------------
#
# 4.2's gate is a string equality: keep the crops YOLO labelled with the
# target's COCO word, discard the rest. Two rig walks measured what that
# costs -- at close range the detector *relabels* the object (`bottle` ->
# `vase` -> `refrigerator`), so the gate discards exactly the frames where
# the target fills the view -- and the default moved to the open-vocabulary
# path on 2026-09-07 as a result.
#
# That fixed recall by removing the gate entirely, which leaves 2.9's
# per-frame budget paying for it: every surviving proposal is one more CLIP
# image encode, and on an 8L the three models together have 3.1x headroom,
# not 10-30x. So the question this answers is not "hard gate or no gate" --
# that is settled -- but whether there is a useful point BETWEEN them.
#
# `label_affinity()` builds that point. The COCO class names are scored in
# CLIP's *text* space against the mission's target string, once, at mission
# start (on the robot: the Pi's CPU, alongside the target encode 2.8 step 1
# already pays for), and the top `k` become the gate's accept set. So:
#
#   k = 1   ~ the hard gate, with the affinity set standing in for the
#             literal COCO word -- and note it need not AGREE with it
#   k = 80  = the open-vocabulary path exactly, every label admitted
#
# which makes the whole thing one swept axis between the two paths already
# measured, rather than a third path with its own separate argument.
#
# **The known risk, stated up front so a null result is readable.** CLIP was
# trained to align image and text, not text with text; text-text cosine in
# this space is usable but not calibrated, and short noun phrases tend to
# crowd together near the top of the range. That is why this ranks and takes
# top-k rather than thresholding on an absolute similarity, and it is why the
# sweep is the deliverable rather than one chosen k.
DEFAULT_AFFINITY_K = 8


class TextScorer(Protocol):
    """A `CropScorer` that can also compare text with text.

    Optional: `PerceptionPipeline` checks for it and falls back to the
    literal COCO word when a scorer does not provide it, so a fake, a HEF
    that ships only an image encoder, and a full CLIP all stay usable
    through the one Protocol. Same reason `RobotInterface.get_depth_grid()`
    carries an honest all-unusable default rather than being mandatory.
    """

    def text_similarity(self, anchor: str,
                        texts: Sequence[str]) -> Sequence[float]: ...


@dataclass(frozen=True)
class Vocabulary:
    """Whether the local tier can even propose on this target, decided once.

    4.2's real problem is that `absent` from a class-gated pipeline is
    indistinguishable from an empty room: there is no `unknown` class and
    no error, so a target the detector has no word for reports exactly what
    a clear room reports. This is that distinction, made explicit at
    mission start instead of inferred three frames in when `cold_search`
    happens to fire.

    `coco_class` is the literal word, `affinity` is the accept set the soft
    gate actually uses, and `in_vocabulary` says whether COCO has a word at
    all. **None of them is a promise that the target is findable** -- the
    blue-bottle walk is `in_vocabulary` and the local tier still lost 11 of
    18 sightings -- so `advisory` says so in words rather than letting a
    caller read the flag as a capability.
    """

    target: str
    coco_class: Optional[str]
    affinity: tuple
    crop_source: str
    affinity_k: int
    ranked: tuple = ()

    @property
    def in_vocabulary(self) -> bool:
        return self.coco_class is not None

    @property
    def advisory(self) -> str:
        if not self.in_vocabulary:
            return ("no COCO word: the detector cannot propose on this "
                    "target by name, so `absent` from this pipeline is weak "
                    "evidence and the cloud tier owns identity")
        return ("COCO word present, which is not a promise of visibility -- "
                "the detector relabels objects at close range")

    def as_dict(self) -> dict:
        return {
            "target": self.target,
            "coco_class": self.coco_class,
            "in_vocabulary": self.in_vocabulary,
            "affinity": list(self.affinity),
            "affinity_k": self.affinity_k,
            "crop_source": self.crop_source,
            "advisory": self.advisory,
        }


def label_affinity(target: str, scorer, k: int = DEFAULT_AFFINITY_K,
                   classes: Sequence[str] = COCO_CLASSES):
    """The `k` COCO labels closest to `target` in CLIP's text space.

    Returns `(accept_set, ranked)` where `ranked` is every class paired
    with its similarity, best first -- kept because a gate that cannot be
    inspected is a gate nobody can debug, and because the sweep wants the
    whole ordering rather than one cut of it.

    The literal COCO word, when there is one, is **always** in the accept
    set regardless of where the text encoder ranks it. It is the one label
    that is correct by construction, and letting a text-space ranking drop
    it would be a regression dressed as a tuning result.
    """
    k = max(0, int(k))
    literal = coco_class_for(target)
    fn = getattr(scorer, "text_similarity", None)
    if fn is None:
        # No text encoder: fall back to the literal word. Reported, never
        # silent -- a run that quietly became a hard gate would look like a
        # measurement of the soft one.
        accept = (literal,) if literal else ()
        return accept, ()

    sims = list(fn(target, list(classes)))
    if len(sims) != len(classes):
        raise PerceptionUnavailable(
            f"text_similarity returned {len(sims)} scores for "
            f"{len(classes)} classes")
    ranked = tuple(sorted(zip(classes, (float(v) for v in sims)),
                          key=lambda pair: pair[1], reverse=True))
    accept = [name for name, _ in ranked[:k]]
    if literal and literal not in accept:
        accept.append(literal)
    return tuple(accept), ranked


@dataclass(frozen=True)
class Box:
    """A detection box in pixels, origin top-left."""

    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def centre_x(self) -> float:
        return (self.x1 + self.x2) / 2

    @property
    def area(self) -> float:
        return max(0.0, self.x2 - self.x1) * max(0.0, self.y2 - self.y1)


@dataclass(frozen=True)
class Detection:
    """One proposal, before CLIP has an opinion about it."""

    box: Box
    label: str
    confidence: float


@dataclass
class Candidate:
    """A detection that has been scored against the mission's target."""

    detection: Detection
    similarity: float = 0.0
    best_distractor: float = 0.0
    bearing_deg: Optional[float] = None
    # Every score this crop got, target first. Kept so `probability` can be
    # computed over the whole set rather than just the winner -- a softmax
    # over two numbers is not the same thing.
    scores: tuple = ()

    @property
    def margin(self) -> float:
        """How much better the target string fits this crop than the best
        competing string does. Reported and still useful for reading a walk
        by eye, but **not the gate** -- see DEFAULT_MATCH_PROBABILITY for
        the measurement that retired it."""
        return self.similarity - self.best_distractor

    @property
    def confidence(self) -> float:
        """The proposal's own confidence, before CLIP had an opinion.

        Not the gate here and never was -- the shipped pipeline thresholds
        on `probability` and this number only says the detector was sure
        there was *an object*. It is named because an open-vocabulary
        detector (4.11) has no CLIP stage and no distractors, so this is
        the only score it produces, and `control/perception_eval.py` has to
        be able to ask for it by name rather than reach through the
        detection.
        """
        return self.detection.confidence

    @property
    def probability(self) -> float:
        """P(target | this crop, these texts). **This is what to threshold
        on.** Comparable across different target strings, which the margin
        provably is not."""
        scores = self.scores or (self.similarity, self.best_distractor)
        xs = [s * CLIP_LOGIT_SCALE for s in scores]
        top = max(xs)
        exps = [math.exp(x - top) for x in xs]
        return exps[0] / sum(exps)


@dataclass
class Perception:
    """One frame's worth of on-board perception.

    `status` is the tri-state. `best` is the strongest candidate if there
    is one. `bearing_deg` is body-relative **only when the camera is
    fixed**: with a pan/tilt head it is `pan + tilt-corrected in-frame
    bearing` (1.11, amended by 1.15.3), and both servo angles have to ride
    in the frame's own metadata or a servo still moving corrupts it
    silently. Off the robot there is no pan and no tilt, so the two
    coincide -- `pan_deg` records which case produced this.
    """

    status: str
    candidates: list = field(default_factory=list)
    best: Optional[Candidate] = None
    crop_source: str = CROP_LABEL_GATE
    reason: str = ""
    pan_deg: float = 0.0
    tilt_deg: float = 0.0

    @property
    def matched(self) -> bool:
        return self.status == DETECTED and self.best is not None

    @property
    def bearing_deg(self) -> Optional[float]:
        return self.best.bearing_deg if self.best else None

    def as_dict(self) -> dict:
        """The shape that crosses the wire in phase C5, and that the twin
        draws in 6.3. Marked `synthesised: False` for the same reason
        `MockRobot` marks its own the other way -- no run may look as
        though it exercised perception it never had."""
        return {
            "status": self.status,
            "crop_source": self.crop_source,
            "reason": self.reason,
            "synthesised": False,
            "pan_deg": self.pan_deg,
            "tilt_deg": self.tilt_deg,
            "bearing_deg": self.bearing_deg,
            "match_margin": round(self.best.margin, 4) if self.best else None,
            "match_probability": (round(self.best.probability, 4)
                                  if self.best else None),
            "similarity": round(self.best.similarity, 4) if self.best else None,
            "label": self.best.detection.label if self.best else None,
            "candidates": len(self.candidates),
        }


class Detector(Protocol):
    """Anything that turns image bytes into proposals.

    A Protocol so the pipeline can be driven by a fake in tests and by a
    HEF on the robot later, with nothing in between changing. Same reason
    `RobotInterface` exists one layer down.
    """

    def detect(self, image: bytes, confidence: float) -> Sequence[Detection]: ...


class CropScorer(Protocol):
    """Anything that scores an image region against text."""

    def score(self, image: bytes, box: Box, texts: Sequence[str]) -> Sequence[float]: ...


class RegionProposer(Protocol):
    """Anything that proposes object regions **without a class list**.

    4.2's second crop source, and the reason it matters is measured: of the
    11 labelling errors in the three-walk corpus, **9 were crop proposals
    and 0 were matching**. The detector can only propose what COCO has a
    word for, and it stops proposing at all once the object fills the view.

    A Protocol for the same reason `Detector` is one: the concrete
    implementation today is a semantic segmenter run on a laptop, and on
    the robot it is a HEF. Nothing between changes.
    """

    def propose(self, image: bytes) -> Sequence[Box]: ...


class PerceptionUnavailable(Exception):
    """The pipeline could not look, as distinct from looking and finding
    nothing. Raised by a backend; converted to `UNAVAILABLE` by the
    pipeline, never allowed to look like `ABSENT`."""


class PerceptionPipeline:
    """detector -> crops -> CLIP -> match, for one mission's target.

    Construction binds the target, which is what 2.8 step 1 does when it
    encodes the target string once and caches the vector for the mission:
    the string does not change, so the text encode is a per-mission cost
    and not a per-frame one.
    """

    def __init__(
        self,
        detector: Detector,
        scorer: CropScorer,
        target: str,
        *,
        hfov_deg: float = 66.0,
        match_margin: Optional[float] = None,
        match_probability: float = DEFAULT_MATCH_PROBABILITY,
        distractors: Sequence[str] = DEFAULT_DISTRACTORS,
        max_crops: Optional[int] = None,
        crop_path: str = DEFAULT_CROP_PATH,
        proposer: Optional["RegionProposer"] = None,
        affinity_k: int = DEFAULT_AFFINITY_K,
        confidence: Optional[float] = None,
    ):
        if not target or not target.strip():
            raise ValueError("PerceptionPipeline needs a target string")
        self.detector = detector
        self.scorer = scorer
        self.target = target.strip()
        self.hfov_deg = hfov_deg
        # The gate is the probability. `match_margin` is an override kept
        # for the walks and tests written against the old metric, and for
        # sweeping the raw number on a new corpus -- set it and it wins.
        self.match_margin = match_margin
        self.match_probability = match_probability
        self.distractors = tuple(distractors)
        # 2.9's gate: one CLIP image encode per crop, so this is the per-frame
        # cost knob. Off-robot it is only slow; on the part it is the
        # difference between a 61%-duty schedule and an over-budget one.
        #
        # **It scales with the number of crop SOURCES, and getting that wrong
        # silently costs detections.** Measured 2026-09-07 on the bottle walk
        # with the floor mask on: at 4 the pipeline found 11 of 18, at 8 it
        # found 18 of 18, with no false positives at either. The cause is the
        # sort below -- crops are ranked by AREA, and a target is usually much
        # smaller than the furniture it sits beside, so a cap sized for one
        # source lets the other source's large regions crowd the target out.
        #
        # Ranking by area is itself the weak part and is left alone
        # deliberately: replacing it is a design change that wants its own
        # measurement, and raising the cap is the fix the data actually
        # supports.
        self.max_crops = (
            max_crops if max_crops is not None
            else (DEFAULT_MAX_CROPS_WITH_PROPOSER if proposer is not None
                  else DEFAULT_MAX_CROPS))

        if crop_path not in CROP_PATHS:
            raise ValueError(
                f"Unknown crop_path {crop_path!r}. Known: {', '.join(CROP_PATHS)}")
        # `soft_gate` is deliberately NOT in this guard. The proposer's
        # boxes carry no class, and the soft gate passes them through
        # untouched for exactly the reason the hard gate cannot: an accept
        # SET is a filter on labels that exist, not a requirement that one
        # does. So floor-mask regions and gated detector boxes compose,
        # which is the union the 94%-recall row above is measured on.
        if crop_path == CROP_LABEL_GATE and proposer is not None:
            raise ValueError(
                "crop_path='label_gate' discards every region the proposer "
                "produces -- they have no class to gate on, which is why the "
                "proposer exists. Use 'low_confidence' (the default) or drop "
                "the proposer.")
        self.crop_path = crop_path
        # UNIONED with the detector's boxes, never substituted for them.
        # Measured over the three-walk corpus at P >= 0.8, against the
        # adjudicated labels in each walk's labels.json:
        #
        #   detector only    55/64 recall 86%, 0 false positives
        #   floor mask only  38/64 recall 59%, 1 false positive
        #   BOTH             60/64 recall 94%, 1 false positive
        #
        # The mask is *worse alone* -- it proposes coarse regions and misses
        # small distant targets the detector finds easily -- and better
        # together, because the two fail on different frames. It recovers
        # both close-ups the detector could not propose on at all.
        self.proposer = proposer
        self.coco_class = coco_class_for(self.target)
        if crop_path == CROP_LOW_CONFIDENCE:
            # Forced open vocabulary: the COCO word is still recorded (it is
            # useful to know there was one) but it gates nothing.
            self.crop_source = CROP_LOW_CONFIDENCE
        elif crop_path == CROP_LABEL_GATE:
            if not self.coco_class:
                raise ValueError(
                    f"crop_path='label_gate' needs a target with a COCO word; "
                    f"{self.target!r} has none, so the gate would discard "
                    "every proposal and the pipeline would report `absent` "
                    "on every frame.")
            self.crop_source = CROP_LABEL_GATE
        elif crop_path == CROP_SOFT_GATE:
            self.crop_source = CROP_SOFT_GATE
        else:
            self.crop_source = (
                CROP_LABEL_GATE if self.coco_class else CROP_LOW_CONFIDENCE)

        # The accept set, built once. `ranked` is kept whole so the sweep
        # can re-cut it at any k without re-encoding, and so a gate that
        # behaves oddly can be read rather than guessed at.
        self.affinity_k = max(0, int(affinity_k))
        self.affinity: tuple = ()
        self.affinity_ranked: tuple = ()
        if self.crop_source == CROP_SOFT_GATE:
            self.affinity, self.affinity_ranked = label_affinity(
                self.target, self.scorer, self.affinity_k)
            if not self.affinity_ranked:
                # No text encoder on this scorer. The gate has silently
                # become the hard one, which is a different measurement --
                # say so rather than let a run be mislabelled.
                logger.warning(
                    "crop_path='soft_gate' but the scorer has no "
                    "text_similarity(); falling back to the literal COCO "
                    "word %r. This run is a LABEL GATE, not a soft one.",
                    self.coco_class)

        # The confidence the detector is asked for. The soft gate sits on
        # the open-vocabulary path's threshold, not the hard gate's: the
        # whole point is to admit the relabelled crops, and a `vase` the
        # detector is only 0.1 sure of is exactly one of those. An explicit
        # `confidence=` overrides it so a sweep can hold this fixed and
        # vary only k -- otherwise the axis moves two things at once.
        self.confidence = (
            confidence if confidence is not None
            else DEFAULT_CONFIDENCE if self.crop_source == CROP_LABEL_GATE
            else LOW_CONFIDENCE)

    @property
    def vocabulary(self) -> "Vocabulary":
        """The up-front verdict: can the local tier propose on this target?

        Read at mission start by `brain/tiered.py`, so an out-of-vocabulary
        search declares itself instead of being discovered three frames in
        when `cold_search` happens to fire.
        """
        return Vocabulary(
            target=self.target,
            coco_class=self.coco_class,
            affinity=self.affinity,
            crop_source=self.crop_source,
            affinity_k=self.affinity_k,
            ranked=self.affinity_ranked,
        )

    def perceive(self, frame: dict) -> Perception:
        """One frame in, one `Perception` out.

        Takes a `RobotInterface` frame dict rather than raw bytes, so that
        the same call works against every backend that has published
        `image_base64` since S2 -- and so that pan/tilt angles can be read
        from the frame's own metadata when a head exists (1.15.3).
        """
        pan = float(frame.get("pan_deg") or 0.0)
        tilt = float(frame.get("tilt_deg") or 0.0)

        try:
            image = _decode(frame)
        except PerceptionUnavailable as exc:
            return Perception(status=UNAVAILABLE, reason=str(exc),
                              crop_source=self.crop_source,
                              pan_deg=pan, tilt_deg=tilt)

        try:
            proposals = list(self.detector.detect(image, self.confidence))
            if self.proposer is not None:
                # A region with no class. The label is a placeholder that
                # exists only so the crop reads sensibly in a log -- the
                # label gate is never applied to these (they have no class
                # to gate on, which is the entire point), so a `crop_path`
                # of `label_gate` silently drops them. That combination is
                # refused at construction rather than surprising anyone.
                proposals += [Detection(box=b, label=CROP_FLOOR_MASK,
                                        confidence=0.0)
                              for b in self.proposer.propose(image)]
        except PerceptionUnavailable as exc:
            return Perception(status=UNAVAILABLE, reason=str(exc),
                              crop_source=self.crop_source,
                              pan_deg=pan, tilt_deg=tilt)
        except Exception as exc:  # a wedged model is not an empty room
            logger.warning("detector failed: %s", exc)
            return Perception(status=UNAVAILABLE, reason=f"detector failed: {exc}",
                              crop_source=self.crop_source,
                              pan_deg=pan, tilt_deg=tilt)

        crops = self._crops(proposals)
        if not crops:
            # The frame was good and there is nothing to score. That is
            # information, and it is the state a mission may act on.
            return Perception(status=ABSENT, crop_source=self.crop_source,
                              reason="no proposals survived the crop gate",
                              pan_deg=pan, tilt_deg=tilt)

        width = float(frame.get("image_width") or 0.0) or _image_width(image)
        try:
            candidates = self._score(image, crops, width, pan, tilt)
        except PerceptionUnavailable as exc:
            return Perception(status=UNAVAILABLE, reason=str(exc),
                              crop_source=self.crop_source,
                              pan_deg=pan, tilt_deg=tilt)
        except Exception as exc:
            logger.warning("crop scorer failed: %s", exc)
            return Perception(status=UNAVAILABLE, reason=f"scorer failed: {exc}",
                              crop_source=self.crop_source,
                              pan_deg=pan, tilt_deg=tilt)

        if self.match_margin is not None:
            best = max(candidates, key=lambda c: c.margin)
            passed, got, gate, unit = (best.margin >= self.match_margin,
                                       best.margin, self.match_margin, "margin")
        else:
            best = max(candidates, key=lambda c: c.probability)
            passed, got, gate, unit = (best.probability >= self.match_probability,
                                       best.probability, self.match_probability, "P")
        if not passed:
            return Perception(
                status=ABSENT, candidates=candidates,
                crop_source=self.crop_source, pan_deg=pan, tilt_deg=tilt,
                reason=(f"{len(candidates)} proposal(s), best {unit} "
                        f"{got:.3f} < {gate}"))

        return Perception(status=DETECTED, candidates=candidates, best=best,
                          crop_source=self.crop_source, pan_deg=pan,
                          tilt_deg=tilt,
                          reason=f"{best.detection.label} @ {unit} {got:.3f}")

    def _crops(self, proposals: Sequence[Detection]) -> list:
        """4.2's gate. With a COCO word the label is a cheap prefilter and
        few crops survive it; without one every proposal is a candidate and
        CLIP does the whole job. `soft_gate` is the interior of that axis:
        the prefilter is an accept SET from CLIP's text space rather than
        one string, so a relabelled target survives and an unrelated label
        still does not."""
        if self.crop_source == CROP_LABEL_GATE:
            kept = [d for d in proposals
                    if d.label.strip().lower() == self.coco_class]
        elif self.crop_source == CROP_SOFT_GATE:
            # A class-agnostic proposal has no label to judge, so it is
            # never judged -- see the construction guard above.
            kept = [d for d in proposals
                    if d.label == CROP_FLOOR_MASK
                    or d.label.strip().lower() in self.affinity]
        else:
            kept = list(proposals)
        kept.sort(key=lambda d: d.box.area, reverse=True)
        return kept[:self.max_crops]

    def _score(self, image: bytes, crops: Sequence[Detection],
               width: float, pan: float, tilt: float) -> list:
        texts = (self.target,) + self.distractors
        out = []
        for det in crops:
            scores = list(self.scorer.score(image, det.box, texts))
            if len(scores) != len(texts):
                raise PerceptionUnavailable(
                    f"scorer returned {len(scores)} scores for {len(texts)} texts")
            out.append(Candidate(
                detection=det,
                similarity=scores[0],
                best_distractor=max(scores[1:]) if len(scores) > 1 else 0.0,
                scores=tuple(scores),
                bearing_deg=self._bearing(det.box, width, pan),
            ))
        return out

    def _bearing(self, box: Box, width: float, pan: float) -> Optional[float]:
        """In-frame bearing, plus the pan angle the frame was taken at.

        **No tilt correction, and that is not an oversight.** 1.15.3
        requires the body-relative bearing to be `pan + tilt-corrected
        in-frame bearing`, and the correction needs a camera intrinsic
        matrix this project has never measured. Off the robot both servos
        are absent, so `tilt` is 0 and the omission is exact; on the robot
        it is a calibration item (1.16 #4, #16) and this is where it lands.
        """
        if width <= 0:
            return None
        return (box.centre_x / width - 0.5) * self.hfov_deg + pan


def _image_width(image: bytes) -> float:
    """The frame's pixel width, read from the image itself.

    **No `RobotInterface` backend publishes `image_width`** -- checked
    2026-09-07, and the consequence was that `bearing_deg` was `None` on
    every frame this project can produce, so 1.11's "which way is it"
    output existed and had never once been a number. Widening the frame
    contract would touch every backend and the conformance suite for a
    value the pixels already carry, so it is read here instead.

    The frame's own `image_width` still wins when present: a backend that
    declares one knows something this cannot, such as a frame that was
    downscaled after the box coordinates were computed.

    Returns 0.0 rather than raising, and `_bearing()` maps that to `None`.
    Anything unreadable here is a bearing we do not have -- which is
    already a state -- and never a reason to fail a perception step that
    the detector itself completed. PIL arrives with the optional extras
    (both concrete backends import it), so a fake-driven test simply gets
    the old behaviour.
    """
    try:
        from PIL import Image

        with Image.open(io.BytesIO(image)) as img:
            return float(img.width)
    except Exception:  # noqa: BLE001 -- an unreadable size is not an error
        return 0.0


def _decode(frame: dict) -> bytes:
    image_base64 = frame.get("image_base64")
    if not image_base64:
        raise PerceptionUnavailable("frame carries no image_base64")
    try:
        return base64.b64decode(image_base64, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise PerceptionUnavailable(f"image_base64 did not decode: {exc}")


# --------------------------------------------------------------------------
# The two concrete backends. Both import their heavy dependency lazily, so
# importing this module costs nothing and the automated suite never needs
# a multi-gigabyte download. See requirements-perception.txt.
# --------------------------------------------------------------------------


class YoloDetector:
    """Ultralytics YOLO, on whatever device this machine has.

    The same weights that 4.3.1's table measures on an 8L -- so a result
    here is about the *model*, and a result there is about the *part*.
    Keeping them the same file is the whole reason the Hailo was chosen
    over the IMX500, which could not be fed a stored frame at all (1.10).
    """

    def __init__(self, weights: str = DEFAULT_DETECTOR, device: Optional[str] = None,
                 imgsz: Optional[int] = None):
        """`imgsz` is the detector's input resolution.

        Default (None) leaves Ultralytics on 640, which is what 4.3.1's 92 FPS
        on the 8L was measured at. Raising it is the standard fix for small
        distant objects -- and on the 8L it is a real option rather than a
        wish, because the headroom is there: 92 FPS against a 30fps camera
        buys roughly a 4x cost increase before the detector stops clearing
        camera rate. **A HEF is compiled for a fixed input size**, so changing
        this is a compile-time decision on the robot, not a runtime one.
        """
        try:
            from ultralytics import YOLO
        except ImportError as exc:  # pragma: no cover - exercised by hand
            raise PerceptionUnavailable(
                "ultralytics is not installed. `pip install -r "
                "requirements-perception.txt` -- it is deliberately kept out "
                "of requirements.txt so the test suite stays light."
            ) from exc
        self.model = YOLO(weights)
        self.device = device
        self.imgsz = imgsz
        # Named so 6.3's readout and every saved record say which resolution
        # produced a number -- 640 and 1280 are different detectors for this
        # purpose, and a table that cannot tell them apart is unreadable.
        self.weights = weights if not imgsz else f"{weights}@{imgsz}"

    def detect(self, image: bytes, confidence: float) -> Sequence[Detection]:  # pragma: no cover
        from PIL import Image

        img = Image.open(io.BytesIO(image)).convert("RGB")
        results = self.model.predict(img, conf=confidence, verbose=False,
                                     device=self.device,
                                     **({"imgsz": self.imgsz} if self.imgsz else {}))
        names = self.model.names
        out = []
        for r in results:
            for b in getattr(r, "boxes", []):
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
                out.append(Detection(
                    box=Box(x1, y1, x2, y2),
                    label=str(names[int(b.cls[0])]),
                    confidence=float(b.conf[0]),
                ))
        return out


class ClipScorer:
    """OpenCLIP's image encoder over a crop, against cached text vectors.

    The text encode is done once per distinct text tuple and cached, which
    is 2.8 step 1's *"CLIP encodes the target string once and the vector is
    cached for the mission"* -- on the robot the text encoder runs on the
    Pi's CPU for exactly this reason (4.2).
    """

    def __init__(self, model_name: str = DEFAULT_CLIP,
                 pretrained: str = "openai", device: Optional[str] = None):
        try:
            import open_clip
            import torch
        except ImportError as exc:  # pragma: no cover - exercised by hand
            raise PerceptionUnavailable(
                "open_clip_torch / torch are not installed. `pip install -r "
                "requirements-perception.txt`."
            ) from exc
        self._torch = torch
        self.device = device or ("cuda" if torch.cuda.is_available()
                                 else "mps" if torch.backends.mps.is_available()
                                 else "cpu")
        self.model, _, self.preprocess = open_clip.create_model_and_transforms(
            model_name, pretrained=pretrained, device=self.device)
        self.model.eval()
        self.tokenizer = open_clip.get_tokenizer(model_name)
        self.model_name = model_name
        self._text_cache: dict = {}

    def _text_features(self, texts: Sequence[str]):  # pragma: no cover
        key = tuple(texts)
        if key not in self._text_cache:
            tokens = self.tokenizer(list(texts)).to(self.device)
            with self._torch.no_grad():
                feats = self.model.encode_text(tokens)
                feats /= feats.norm(dim=-1, keepdim=True)
            self._text_cache[key] = feats
        return self._text_cache[key]

    def text_similarity(self, anchor: str,
                        texts: Sequence[str]) -> Sequence[float]:  # pragma: no cover
        """Cosine between `anchor` and each of `texts`, in CLIP text space.

        Used once per mission to build the soft gate's accept set, so it
        costs one text encode of the 80 COCO names on top of the target
        encode 2.8 step 1 already pays for -- on the Pi's CPU, where the
        text tower lives on the robot (4.2), and never per frame.

        **Not calibrated, and it must not be read as a probability.** CLIP
        aligns image with text; text-text cosine here is a ranking signal
        and nothing stronger, which is why `label_affinity()` takes a top-k
        of it rather than thresholding it.
        """
        feats = self._text_features(tuple(texts))
        anchor_feat = self._text_features((anchor,))
        sims = (anchor_feat @ feats.T)[0]
        return [float(v) for v in sims]

    def score(self, image: bytes, box: Box, texts: Sequence[str]) -> Sequence[float]:  # pragma: no cover
        from PIL import Image

        img = Image.open(io.BytesIO(image)).convert("RGB")
        crop = img.crop((int(box.x1), int(box.y1),
                         max(int(box.x2), int(box.x1) + 1),
                         max(int(box.y2), int(box.y1) + 1)))
        tensor = self.preprocess(crop).unsqueeze(0).to(self.device)
        with self._torch.no_grad():
            feats = self.model.encode_image(tensor)
            feats /= feats.norm(dim=-1, keepdim=True)
            sims = (feats @ self._text_features(texts).T)[0]
        return [float(v) for v in sims]


class SegformerFloorProposer:
    """Class-agnostic region proposals from a floor mask (4.2, 4.3).

    **The rule is geometry, not vocabulary**: anything that is not floor,
    stands on the floor, and does not reach the top of the frame is an
    object worth cropping. A wall reaches the top; a ceiling reaches the
    top; a shoe does not.

    SegFormer-B0 fine-tuned on ADE20K is the model, chosen because ADE20K
    carries `floor`, `rug` and `earth` as classes and B0 is ~14MB. **Its
    class map is used for exactly two things** -- deciding which pixels are
    floor, and splitting the not-floor region into connected components --
    and its labels are then thrown away. That matters: the first attempt
    here took connected components of a binary not-floor mask and produced
    *zero* proposals on every frame, because wall, furniture and object are
    one blob that touches the ceiling. Splitting by class region is what
    separates the object from the wall it stands against, without the
    object's own class ever being consulted.

    This is P4's first compile subject (4.3), so keeping it behind the
    `RegionProposer` Protocol is the point rather than a nicety.
    """

    def __init__(self, model: str = DEFAULT_SEGMENTER,
                 min_area_frac: float = 0.0008,
                 max_area_frac: float = 0.5,
                 max_regions: int = 8,
                 device: Optional[str] = None):
        try:
            import numpy  # noqa: F401
            import torch
            from scipy import ndimage  # noqa: F401
            from transformers import (SegformerForSemanticSegmentation,
                                      SegformerImageProcessor)
        except ImportError as exc:  # pragma: no cover - exercised by hand
            raise PerceptionUnavailable(
                "the floor mask needs transformers + scipy. `pip install -r "
                "requirements-perception.txt`."
            ) from exc
        self._torch = torch
        self.processor = SegformerImageProcessor.from_pretrained(model)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model).eval()
        self.model_name = model
        self.min_area_frac = min_area_frac
        self.max_area_frac = max_area_frac
        self.max_regions = max_regions
        self.floor_ids = [i for i, label in self.model.config.id2label.items()
                          if any(w in label.lower() for w in FLOOR_WORDS)]

    def propose(self, image: bytes) -> Sequence[Box]:  # pragma: no cover
        import numpy as np
        from PIL import Image
        from scipy import ndimage

        img = Image.open(io.BytesIO(image)).convert("RGB")
        with self._torch.no_grad():
            logits = self.model(
                **self.processor(images=img, return_tensors="pt")).logits
        seg = self._torch.nn.functional.interpolate(
            logits, size=img.size[::-1], mode="bilinear", align_corners=False
        ).argmax(1)[0].numpy()

        floor = np.isin(seg, self.floor_ids)
        height, width = floor.shape
        # Dilated, so a region resting ON the floor counts as touching it
        # even where the segmentation leaves a one-pixel seam.
        near_floor = ndimage.binary_dilation(floor, iterations=4)
        found = []
        for cls in np.unique(seg):
            if cls in self.floor_ids:
                continue
            labelled, count = ndimage.label(seg == cls)
            for i in range(1, count + 1):
                region = labelled == i
                area = int(region.sum())
                if not (self.min_area_frac * height * width <= area
                        <= self.max_area_frac * height * width):
                    continue
                if region[0, :].any():          # reaches the ceiling: not an object
                    continue
                if not (region & near_floor).any():   # not standing on the floor
                    continue
                ys, xs = np.where(region)
                found.append((area, Box(float(xs.min()), float(ys.min()),
                                        float(xs.max()), float(ys.max()))))
        found.sort(key=lambda pair: -pair[0])
        return [box for _, box in found[:self.max_regions]]


class UltralyticsTextDetector:
    """Shared `detect_text` for ultralytics' text-prompted detectors.

    Extracted 2026-09-19, when YOLOE was promoted out of
    `brain/perceive_lab.py`. YOLO-World and YOLOE differ in exactly two
    lines -- which ultralytics class to construct, and whether
    `set_classes` takes strings or precomputed text embeddings -- so the
    box extraction lives here once and each subclass supplies its own
    `__init__` and, if it needs to, its own `_set_text`.

    Note this is a `detect_text(image, target)` backend, NOT the
    `Detector` Protocol's `detect(image, confidence)`. Wrap it in
    `OpenVocabCropSource` to put it in a pipeline; `pipeline_for` does
    that for you.
    """

    def _set_text(self, text: str):  # pragma: no cover - needs the model
        self.model.set_classes([text])

    def detect_text(self, image: bytes, text: str):  # pragma: no cover
        # PIL is imported here, not at module scope, because every other
        # model-touching method in this file does the same: it is an optional
        # perception dependency and a module-level import would make
        # `import brain.perceive` fail on a machine without the extras.
        # Getting this wrong cost two full-corpus runs -- the detector raised
        # `name 'Image' is not defined`, the tri-state caught it, and 1234
        # frames came back `unavailable` at 2ms rather than crashing.
        from PIL import Image

        if self._classes != [text]:
            self._set_text(text)
            self._classes = [text]
        img = Image.open(io.BytesIO(image)).convert("RGB")
        results = self.model.predict(img, conf=self.confidence,
                                     verbose=False, device=self.device)
        out = []
        for r in results:
            for b in getattr(r, "boxes", []):
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
                out.append(Detection(box=Box(x1, y1, x2, y2), label=text,
                                     confidence=float(b.conf[0])))
        return out


class YoloE(UltralyticsTextDetector):
    """Ultralytics YOLOE -- the detector P22 measured and this module ships.

    Promoted out of `perceive_lab.py` on 2026-09-19. It earned the move on
    two numbers against OWLv2, on identical frames, mask and gate: **91%
    against 90% at a 3-false-positive budget and 97% against 96% at 16**,
    at **398 ms against 2679**. It is also better SEPARATED -- its 3-FP gate
    sits at 0.391 where YOLO-World's best checkpoint needed 0.026, which is
    the non-separability P7 disqualified a VLM for.

    Two things to know before changing anything around it.

    **The gate travels with the detector.** YOLOE's scores sit lower than
    OWLv2's, so at the old `P>=0.8` it reads 74% -- *worse* than OWLv2's
    82% -- and only reaches 91% at a gate near 0.4. Swapping the detector
    without the gate is a 17-point regression that looks like the model's
    fault. See DEFAULT_MATCH_PROBABILITY.

    **It proposes about one crop per frame**, where OWLv2 proposes four, so
    CLIP verifies a single candidate rather than ranking several. That is
    why the whole tier is fast, and it means recall is bounded by the
    detector's own recall: a frame it skips, CLIP cannot recover.
    """

    def __init__(self, weights: str = None,
                 confidence: float = LOW_CONFIDENCE,
                 device: Optional[str] = None):
        try:
            from ultralytics import YOLOE as _YE
        except ImportError as exc:  # pragma: no cover
            raise PerceptionUnavailable(
                "YOLOE needs ultralytics. `pip install -r "
                "requirements-perception.txt`.") from exc
        self.model = _YE(weights or DEFAULT_YOLOE)
        self.confidence = confidence
        self.device = device
        self.weights = weights or DEFAULT_YOLOE
        self.model_name = self.weights
        self._classes = None

    def _set_text(self, text: str):  # pragma: no cover - needs the model
        # YOLOE wants precomputed text embeddings, where YOLO-World takes
        # the strings themselves. The one line the two families differ by.
        self.model.set_classes([text], self.model.get_text_pe([text]))


class OpenVocabCropSource:
    """A text-prompted detector used as a CROP SOURCE, not a classifier.

    Moved here from `perceive_lab.py` on 2026-09-19 with YOLOE, because a
    shipped pipeline now needs it and `perceive.py` may not import the lab.

    It keeps the detector's BOXES and throws away its scores, so CLIP ranks
    the crops against the target and the distractors exactly as it does for
    a YOLO detector's. P9's finding is the reason: an open-vocabulary model
    wins as a crop source rather than as a replacement -- the gain is the
    regions, not its own confidence.
    """

    def __init__(self, backend, target: str):
        self.backend = backend
        self.target = target
        self.weights = f"crops:{getattr(backend, 'weights', 'open-vocab')}"

    def detect(self, image: bytes, confidence: float):
        # The backend's own threshold already gated this; `confidence` is
        # the pipeline's crop-path knob and is deliberately not applied a
        # second time, or the two gates would compound invisibly.
        return [Detection(box=d.box, label=d.label, confidence=d.confidence)
                for d in self.backend.detect_text(image, self.target)]


def pipeline_for(target: str, *, weights: str = DEFAULT_DETECTOR,
                 clip_model: str = DEFAULT_CLIP,
                 device: Optional[str] = None,
                 floor_mask: bool = False,
                 segmenter: str = DEFAULT_SEGMENTER,
                 **kwargs) -> PerceptionPipeline:
    """The real pipeline, models loaded. Raises `PerceptionUnavailable`
    with a usable message if the optional dependencies are absent.

    `floor_mask` adds 4.2's class-agnostic crop source alongside the
    detector's. It is **off by default** despite measuring better (86% ->
    94% recall on the corpus) for one reason: it is a third model per
    frame, and 2.1's 15-30Hz row plus 2.9's budget mean segmentation has no
    business running at the detector's rate on the real part. Turning it on
    off-robot costs only time; deciding its schedule is C6's job.
    """
    return PerceptionPipeline(
        detector=detector_for(weights, target, device=device),
        scorer=ClipScorer(clip_model, device=device),
        target=target,
        proposer=SegformerFloorProposer(segmenter, device=device) if floor_mask else None,
        **kwargs,
    )


def is_text_prompted(weights: str) -> bool:
    """Does this checkpoint name take the target string into the detector?

    A name test rather than a registry, because `perception_detector` in
    config/robot.yaml is free text and the point is that an operator can
    name a checkpoint this module has never heard of.
    """
    return str(weights).startswith("yoloe")


def detector_for(weights: str, target: str, *, device: Optional[str] = None):
    """The `Detector` for a checkpoint name -- the one place that chooses.

    Two shapes exist and they are not interchangeable. A YOLO checkpoint is
    closed-vocabulary and implements `detect(image, confidence)` directly.
    YOLOE is text-prompted, implements `detect_text(image, target)`, and has
    to be wrapped so the pipeline's crop stage sees the same Protocol. This
    function is what keeps that difference out of every caller.
    """
    if is_text_prompted(weights):
        return OpenVocabCropSource(YoloE(weights, device=device), target)
    return YoloDetector(weights, device=device)


__all__ = [
    "ABSENT", "DETECTED", "UNAVAILABLE",
    "CROP_LABEL_GATE", "CROP_LOW_CONFIDENCE", "CROP_SOFT_GATE",
    "CROP_SOURCES",
    "CROP_PATH_AUTO", "CROP_PATHS", "DEFAULT_CROP_PATH",
    "COCO_CLASSES", "coco_class_for", "label_affinity", "Vocabulary",
    "TextScorer", "DEFAULT_AFFINITY_K",
    "Box", "Detection", "Candidate", "Perception",
    "Detector", "CropScorer", "RegionProposer", "PerceptionPipeline",
    "PerceptionUnavailable", "SegformerFloorProposer",
    "YoloDetector", "ClipScorer", "pipeline_for",
    "YoloE", "UltralyticsTextDetector", "OpenVocabCropSource",
    "detector_for", "is_text_prompted",
    "DEFAULT_YOLOE", "DEFAULT_YOLO_DETECTOR",
    "DEFAULT_DETECTOR", "DEFAULT_CLIP", "DEFAULT_MATCH_MARGIN",
    "DEFAULT_MATCH_PROBABILITY", "CLIP_LOGIT_SCALE",
    "DEFAULT_SEGMENTER", "FLOOR_WORDS",
    "DEFAULT_MAX_CROPS", "DEFAULT_MAX_CROPS_WITH_PROPOSER",
    "DEFAULT_DISTRACTORS", "LOW_CONFIDENCE",
]
