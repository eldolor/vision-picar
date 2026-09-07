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
CROP_FLOOR_MASK = "floor_mask"            # needs 4.3's segmentation model
CROP_LIDAR_CLUSTER = "lidar_cluster"      # needs the sensor
CROP_SOURCES = (CROP_LABEL_GATE, CROP_LOW_CONFIDENCE,
                CROP_FLOOR_MASK, CROP_LIDAR_CLUSTER)

# Ship `s`, not `n` -- 4.3.1 measured +7.6 mAP on-chip for a third of the
# frame rate, and 92 FPS is still 3x the camera. `n` is the day-one
# baseline that needs no HEF (4.4), not the shipped detector.
DEFAULT_DETECTOR = "yolo11s.pt"

# 4.9's choice between the two CLIP encoders Hailo ports, which 4.3 lists
# without choosing: ResNet-50 is 7-10ms per crop against ViT-B/32's 25-50,
# and ~25M parameters against ~88M -- which matters twice over on a part
# with no local DRAM, since non-resident weights re-stream over the one
# PCIe lane (2.9). Off-robot the difference is only speed.
DEFAULT_CLIP = "RN50"

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

    @property
    def margin(self) -> float:
        """How much better the target string fits this crop than the best
        competing string does. **This, not `similarity`, is the number to
        threshold on** -- see DEFAULT_MATCH_MARGIN."""
        return self.similarity - self.best_distractor


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
        match_margin: float = DEFAULT_MATCH_MARGIN,
        distractors: Sequence[str] = DEFAULT_DISTRACTORS,
        max_crops: int = 4,
    ):
        if not target or not target.strip():
            raise ValueError("PerceptionPipeline needs a target string")
        self.detector = detector
        self.scorer = scorer
        self.target = target.strip()
        self.hfov_deg = hfov_deg
        self.match_margin = match_margin
        self.distractors = tuple(distractors)
        # 2.9's gate. Scoring every proposal is what turns a 61%-duty
        # schedule into an over-budget one, and off-robot it is just slow.
        self.max_crops = max_crops

        self.coco_class = coco_class_for(self.target)
        self.crop_source = (
            CROP_LABEL_GATE if self.coco_class else CROP_LOW_CONFIDENCE)
        self.confidence = (
            DEFAULT_CONFIDENCE if self.coco_class else LOW_CONFIDENCE)

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

        width = float(frame.get("image_width") or 0.0)
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

        best = max(candidates, key=lambda c: c.margin)
        if best.margin < self.match_margin:
            return Perception(
                status=ABSENT, candidates=candidates,
                crop_source=self.crop_source, pan_deg=pan, tilt_deg=tilt,
                reason=(f"{len(candidates)} proposal(s), best margin "
                        f"{best.margin:.3f} < {self.match_margin}"))

        return Perception(status=DETECTED, candidates=candidates, best=best,
                          crop_source=self.crop_source, pan_deg=pan,
                          tilt_deg=tilt,
                          reason=f"{best.detection.label} @ margin {best.margin:.3f}")

    def _crops(self, proposals: Sequence[Detection]) -> list:
        """4.2's gate. With a COCO word the label is a cheap prefilter and
        few crops survive it; without one every proposal is a candidate and
        CLIP does the whole job."""
        if self.coco_class:
            kept = [d for d in proposals
                    if d.label.strip().lower() == self.coco_class]
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

    def __init__(self, weights: str = DEFAULT_DETECTOR, device: Optional[str] = None):
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
        self.weights = weights

    def detect(self, image: bytes, confidence: float) -> Sequence[Detection]:  # pragma: no cover
        from PIL import Image

        img = Image.open(io.BytesIO(image)).convert("RGB")
        results = self.model.predict(img, conf=confidence, verbose=False,
                                     device=self.device)
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


def pipeline_for(target: str, *, weights: str = DEFAULT_DETECTOR,
                 clip_model: str = DEFAULT_CLIP,
                 device: Optional[str] = None,
                 **kwargs) -> PerceptionPipeline:
    """The real pipeline, models loaded. Raises `PerceptionUnavailable`
    with a usable message if the optional dependencies are absent."""
    return PerceptionPipeline(
        detector=YoloDetector(weights, device=device),
        scorer=ClipScorer(clip_model, device=device),
        target=target,
        **kwargs,
    )


__all__ = [
    "ABSENT", "DETECTED", "UNAVAILABLE",
    "CROP_LABEL_GATE", "CROP_LOW_CONFIDENCE", "CROP_SOURCES",
    "COCO_CLASSES", "coco_class_for",
    "Box", "Detection", "Candidate", "Perception",
    "Detector", "CropScorer", "PerceptionPipeline", "PerceptionUnavailable",
    "YoloDetector", "ClipScorer", "pipeline_for",
    "DEFAULT_DETECTOR", "DEFAULT_CLIP", "DEFAULT_MATCH_MARGIN",
    "DEFAULT_DISTRACTORS", "LOW_CONFIDENCE",
]
