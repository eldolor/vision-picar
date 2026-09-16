"""
perceive_lab.py

Perception backends that are **candidates, not parts**.

`brain/perceive.py` holds what the robot is planned to run: YOLO11s, CLIP
RN50, and SegFormer-B0's floor mask, all of which are small enough for a
Hailo-8L and all of which have earned their place by measurement. This
module holds the models that have *not*, and it exists because
`PLAN-onboard-perception.md` 4.11 closed with an honest limit:

> *"Only one open-vocabulary model was tested. Grounding DINO and OWLv2 are
> stronger and too heavy for a Hailo, and SAM would give class-agnostic
> proposals directly -- the exact weakness measured. **None of those has
> been tried**, and a Jetson is the only way to run them."*

That is a hardware argument resting on an untested assumption, and the
assumption is testable for free. 4.7's promotion rule already says how:
**experiments run off-robot behind the perception seam and are promoted by
compile once they have earned it.** Everything here plugs into the
`Detector` and `RegionProposer` Protocols, so the substitution really is
two lines and `control/perception_eval.py` scores all of them against the
same adjudicated labels.

## Why these three, and what each one is evidence about

| | what it is | what it would settle |
|---|---|---|
| **Grounding DINO** | text-conditioned detection, DETR-shaped | whether a stronger open-vocabulary detector beats the composed pipeline where YOLO-World did not |
| **OWLv2** | ViT + CLIP-style text tower, one-shot | the same question with a very different architecture, so a single model's weakness is not mistaken for the family's |
| **SAM** | class-agnostic segment-everything | **the measured failure directly** -- 9 of the corpus's 11 errors are crop proposals and 0 are matching, so region proposals are what is actually short |

SAM is the one that matters. The other two replace the whole pipeline; SAM
replaces only the part the corpus says is broken, and feeds the CLIP
matcher that the corpus says is not.

## None of these can go on a Hailo, which is the point

They are here to answer *"would a Jetson buy anything"*, so a result that
says "yes, and by this much" is a purchase argument and a result that says
"no" is a saved $400 and a simpler robot. Either is worth an afternoon.
Read every number against 4.11's three cautions -- throughput is not
measurable on a laptop, a high match rate is not automatically good, and
this measures finding rather than navigating.

## Weights are large and are not in the repo

`weights/` and `*.pt` are gitignored, and that rule exists because a 338MB
CLIP checkpoint was swept into a commit and GitHub's 100MB limit caught
it. Everything here downloads to the Hugging Face cache on first use and
is reproducible from the model name, which is the same argument the
detector weights get.
"""

from __future__ import annotations

import io
import logging
from typing import Optional, Sequence

from brain.perceive import (ABSENT, DETECTED, UNAVAILABLE, Box, Candidate,
                            DEFAULT_CLIP, DEFAULT_DETECTOR, DEFAULT_CROP_PATH,
                            DEFAULT_SEGMENTER, Detection, Perception,
                            PerceptionPipeline, PerceptionUnavailable,
                            SegformerFloorProposer, ClipScorer, YoloDetector)

logger = logging.getLogger("perceive_lab")

# Open-vocabulary detectors: the target string goes INTO the detector, so
# there are no crops, no distractors and no CLIP. The number they threshold
# on is their own box confidence -- which is why
# control/perception_eval.py's METRIC_CONFIDENCE exists and why a table may
# never compare one of these against a probability without saying so.
GDINO = "gdino"
OWLV2 = "owlv2"
YOLOWORLD = "yoloworld"
OMDET = "omdet"
LLMDET = "llmdet"
TRTOWLV2 = "trtowlv2"
VLM = "vlm"
OPEN_VOCAB = (GDINO, OWLV2, YOLOWORLD, VLM, OMDET, LLMDET, TRTOWLV2)

DEFAULT_GDINO = "IDEA-Research/grounding-dino-tiny"
DEFAULT_OWLV2 = "google/owlv2-base-patch16-ensemble"
DEFAULT_OMDET = "omlab/omdet-turbo-swin-tiny-hf"
DEFAULT_LLMDET = "iSEE-Laboratory/llmdet_base"
# A path, not a hub id: an engine is built for one GPU and one
# precision, so it is never a default that could be silently reused.
DEFAULT_TRTOWLV2 = ""
DEFAULT_YOLOWORLD = "yolov8s-worldv2.pt"

# The VLM default. Qwen2.5-VL-3B for three reasons that are not "it scores
# well": its grounding is trained rather than emergent, its vision encoder
# tiles at native resolution (the mechanism actually under test), and it is
# Apache-2.0. InternVL3-2B and MiniCPM-V are the other two worth trying --
# pass them as `vlm:<model id>`. Check the licence before any commercial use;
# they differ across this family.
DEFAULT_VLM = "Qwen/Qwen2.5-VL-3B-Instruct"

# SlimSAM-77, ~40MB, rather than SAM ViT-H at 2.4GB. The small one is the
# honest choice for a first pass: if class-agnostic proposals are the
# missing ingredient, the cheapest model that produces them should already
# show it, and a null result from the big one costs an hour of downloads to
# reach the same place. `--sam-model` swaps it.
DEFAULT_SAM = "Zigeng/SlimSAM-uniform-77"

# The proposal filters, shared with SegformerFloorProposer so a region from
# either source is admitted on the same terms. Without these SAM returns
# the whole wall, the whole floor and every highlight on a cushion.
DEFAULT_MIN_AREA_FRAC = 0.0008
DEFAULT_MAX_AREA_FRAC = 0.5
DEFAULT_MAX_REGIONS = 8

# What an open-vocabulary detector calls a detection when nobody is
# sweeping. Only the tri-state depends on it -- every table in
# perception_eval re-thresholds from the raw scores.
DEFAULT_OPEN_VOCAB_CONFIDENCE = 0.30


def _pil(image: bytes):
    from PIL import Image

    return Image.open(io.BytesIO(image)).convert("RGB")


class NullDetector:
    """Proposes nothing, so a `RegionProposer` can be measured alone.

    The floor mask was measured this way (59% alone, 86% for the detector
    alone, 94% together) and the finding was that it is **worse alone and
    better together** -- which only a proposer-only row can show. Any new
    proposer gets the same three rows or it cannot be compared with it.
    """

    weights = "none"

    def detect(self, image: bytes, confidence: float) -> Sequence[Detection]:
        return ()


class ReplayDetector:
    """A `Detector` whose proposals were recorded by another run.

    The point is to measure what QUANTIZATION costs the TIER, not just the
    detector. P16 phase 2 measured a Hailo-10H's YOLO-World at the detector
    level -- 99% proposal agreement at 0.5 confidence, 22% class-score
    recall against fp32's 34% -- and those numbers cannot say what the
    detector -> crops -> CLIP -> match tier is worth, because CLIP runs in
    fp32 on the Pi's CPU either way. Multiplying a retention percentage by
    P9's end-to-end 72% would produce a figure that looks measured and is
    not.

    So the recorded boxes go through the REAL pipeline, scored by the real
    `control/perception_eval.py` against each walk's adjudicated
    `labels.json`. `Detector` is a Protocol precisely so "a fake in tests
    and a HEF later" can drive it; a HEF's recorded output is that seam.

    **Keyed by the SHA1 of the image bytes**, not by a frame name. The
    pipeline hands a detector bytes and nothing else, so hashing is what
    lets this drop in without the scorer's own loop having to know it is
    replaying. It also fails loudly rather than silently: an image the
    detections file never saw raises instead of scoring as "found
    nothing", which would quietly become a miss in the denominator.
    """

    def __init__(self, detections: str, recordings: str = "recordings"):
        import hashlib
        import json
        from pathlib import Path

        self.weights = f"replay:{Path(detections).name}"
        raw = json.loads(Path(detections).read_text())
        rec = Path(recordings)

        # frame key -> boxes, seeding EVERY key the file carries so that a
        # frame the detector said nothing about is still a scored frame.
        by_key: dict = {key: [] for key in raw}
        for key, dets in raw.items():
            for d in dets:
                # Detections files are CENTRE-based [cx, cy, w, h]; `Box`
                # is corner-based, origin top-left. Getting this wrong does
                # not crash, it silently crops the wrong region.
                cx, cy, w, h = d["box"]
                by_key[key].append(Detection(
                    box=Box(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2),
                    label=str(d.get("cls", "")),
                    confidence=float(d["score"])))

        self._by_hash: dict = {}
        self.missing: list = []
        for key, boxes in by_key.items():
            path = rec / key
            if not path.exists():
                self.missing.append(key)
                continue
            digest = hashlib.sha1(path.read_bytes()).hexdigest()
            self._by_hash[digest] = boxes

    def detect(self, image: bytes, confidence: float) -> Sequence[Detection]:
        import hashlib
        digest = hashlib.sha1(image).hexdigest()
        if digest not in self._by_hash:
            raise KeyError(
                "replay detector was handed an image its detections file "
                "does not contain. Scoring it as 'found nothing' would "
                "turn a coverage gap into a miss, so this raises instead.")
        return [d for d in self._by_hash[digest] if d.confidence >= confidence]


class SamProposer:
    """SAM's automatic mask generation as class-agnostic region proposals.

    **This is the model 4.11 says would address the measured failure
    directly.** 9 of the 11 errors in the corpus are crop-source failures
    and 0 are matching failures: the detector proposes nothing when the
    target fills the frame and only furniture when it is far away. SAM's
    whole premise is proposing regions without a class list, which is
    exactly the gap.

    Segment-everything is expensive -- a grid of point prompts, one mask
    decode per point -- so it is seconds per frame rather than
    milliseconds, and `control/perception_eval.py --save` exists so a
    corpus is paid for once. That cost is also the finding: 2.9 budgets
    three models against an 8L's 33ms frame and this is not a model that
    fits on an 8L at any rate, which is the whole reason it is evidence
    about a Jetson.
    """

    def __init__(self, model: str = DEFAULT_SAM,
                 points_per_side: int = 16,
                 min_area_frac: float = DEFAULT_MIN_AREA_FRAC,
                 max_area_frac: float = DEFAULT_MAX_AREA_FRAC,
                 max_regions: int = DEFAULT_MAX_REGIONS,
                 device: Optional[str] = None):
        try:
            import numpy  # noqa: F401
            import torch  # noqa: F401
            from transformers import pipeline as hf_pipeline
        except ImportError as exc:  # pragma: no cover - exercised by hand
            raise PerceptionUnavailable(
                "SAM needs transformers + torch. `pip install -r "
                "requirements-perception.txt`.") from exc
        self.model_name = model
        self.points_per_side = points_per_side
        self.min_area_frac = min_area_frac
        self.max_area_frac = max_area_frac
        self.max_regions = max_regions
        # CPU by default: SAM's mask decoder loops over a point grid and MPS
        # has repeatedly been slower than CPU on that shape. `device` forces
        # it either way, and neither number means anything about the robot.
        self.pipe = hf_pipeline("mask-generation", model=model,
                                device=device or "cpu")

    def propose(self, image: bytes) -> Sequence[Box]:  # pragma: no cover
        import numpy as np

        img = _pil(image)
        out = self.pipe(img, points_per_batch=64,
                        points_per_crop=self.points_per_side)
        height, width = img.size[1], img.size[0]
        found = []
        for mask in out.get("masks", []):
            m = np.asarray(mask)
            area = int(m.sum())
            if not (self.min_area_frac * height * width <= area
                    <= self.max_area_frac * height * width):
                continue
            ys, xs = np.where(m)
            if not len(xs):
                continue
            # Same "reaches the top of the frame" rule the floor mask uses:
            # a wall and a ceiling do, an object standing on the floor does
            # not. Kept identical on purpose -- if SAM wins it should be
            # because its regions are better, not because it was admitted on
            # easier terms.
            if m[0, :].any():
                continue
            found.append((area, Box(float(xs.min()), float(ys.min()),
                                    float(xs.max()), float(ys.max()))))
        found.sort(key=lambda pair: -pair[0])
        return [box for _, box in found[:self.max_regions]]


class OpenVocabPipeline:
    """A text-conditioned detector standing in for the whole pipeline.

    Same `perceive(frame) -> Perception` shape as `PerceptionPipeline`, so
    `control/perception_eval.py` and `brain/tiered.py` take it unchanged --
    but there is no crop stage and no CLIP, so `Candidate.probability` is
    meaningless here and `confidence` is the number to threshold on.

    Deliberately NOT a `Detector` feeding the existing matcher. 4.11
    compared YOLO-World as a *replacement* -- one model instead of three --
    because that is the shape of the "run arbitrary Hugging Face models"
    argument, and comparing it any other way would answer a question nobody
    asked.
    """

    def __init__(self, backend, target: str, *, hfov_deg: float = 66.0,
                 match_confidence: float = DEFAULT_OPEN_VOCAB_CONFIDENCE):
        if not target or not target.strip():
            raise ValueError("OpenVocabPipeline needs a target string")
        self.backend = backend
        self.target = target.strip()
        self.hfov_deg = hfov_deg
        self.match_confidence = match_confidence
        self.crop_source = "open_vocabulary"
        # Named so 6.3's readout and every saved record say which model
        # produced a number. `detector`/`scorer` mirror PerceptionPipeline's
        # attribute names so TieredVision's _name_of() finds them.
        self.detector = backend
        self.scorer = None
        self.proposer = None

    def perceive(self, frame: dict) -> Perception:
        from brain.perceive import _decode

        pan = float(frame.get("pan_deg") or 0.0)
        tilt = float(frame.get("tilt_deg") or 0.0)
        try:
            image = _decode(frame)
        except PerceptionUnavailable as exc:
            return Perception(status=UNAVAILABLE, reason=str(exc),
                              crop_source=self.crop_source,
                              pan_deg=pan, tilt_deg=tilt)
        try:
            detections = list(self.backend.detect_text(image, self.target))
        except PerceptionUnavailable as exc:
            return Perception(status=UNAVAILABLE, reason=str(exc),
                              crop_source=self.crop_source,
                              pan_deg=pan, tilt_deg=tilt)
        except Exception as exc:   # a wedged model is not an empty room
            logger.warning("open-vocabulary detector failed: %s", exc)
            return Perception(status=UNAVAILABLE,
                              reason=f"detector failed: {exc}",
                              crop_source=self.crop_source,
                              pan_deg=pan, tilt_deg=tilt)

        if not detections:
            return Perception(status=ABSENT, crop_source=self.crop_source,
                              reason="no box for the target string",
                              pan_deg=pan, tilt_deg=tilt)

        width = float(frame.get("image_width") or 0.0)
        candidates = [
            Candidate(detection=d, similarity=d.confidence,
                      best_distractor=0.0, scores=(d.confidence,),
                      bearing_deg=((d.box.centre_x / width - 0.5) * self.hfov_deg
                                   + pan) if width > 0 else None)
            for d in detections]
        best = max(candidates, key=lambda c: c.detection.confidence)
        if best.detection.confidence < self.match_confidence:
            return Perception(status=ABSENT, candidates=candidates,
                              crop_source=self.crop_source,
                              pan_deg=pan, tilt_deg=tilt,
                              reason=(f"{len(candidates)} box(es), best "
                                      f"conf {best.detection.confidence:.3f} "
                                      f"< {self.match_confidence}"))
        return Perception(status=DETECTED, candidates=candidates, best=best,
                          crop_source=self.crop_source, pan_deg=pan,
                          tilt_deg=tilt,
                          reason=(f"{best.detection.label} @ conf "
                                  f"{best.detection.confidence:.3f}"))


class GroundingDino:
    """IDEA-Research's Grounding DINO, text-conditioned detection.

    The prompt convention is the model's own and is not decoration: text
    queries are lowercased and terminated with a period, and a query
    without one silently detects worse.
    """

    def __init__(self, model: str = DEFAULT_GDINO,
                 box_threshold: float = 0.05,
                 text_threshold: float = 0.05,
                 device: Optional[str] = None):
        try:
            import torch
            from transformers import (AutoModelForZeroShotObjectDetection,
                                      AutoProcessor)
        except ImportError as exc:  # pragma: no cover
            raise PerceptionUnavailable(
                "Grounding DINO needs transformers + torch. `pip install -r "
                "requirements-perception.txt`.") from exc
        self._torch = torch
        self.device = device or "cpu"
        self.processor = AutoProcessor.from_pretrained(model)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
            model).to(self.device).eval()
        # Low on purpose. The sweep is what picks an operating point, so a
        # threshold here would throw away the scores the sweep needs -- the
        # same "score once, threshold afterwards" rule perception_eval
        # follows one layer up.
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold
        self.weights = model
        self.model_name = model

    def detect_text(self, image: bytes, text: str) -> Sequence[Detection]:  # pragma: no cover
        img = _pil(image)
        prompt = text.strip().lower()
        if not prompt.endswith("."):
            prompt += "."
        inputs = self.processor(images=img, text=prompt,
                                return_tensors="pt").to(self.device)
        with self._torch.no_grad():
            outputs = self.model(**inputs)
        results = self.processor.post_process_grounded_object_detection(
            outputs, inputs["input_ids"], threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            target_sizes=[img.size[::-1]])[0]
        out = []
        labels = results.get("text_labels", results.get("labels", []))
        for box, score, label in zip(results["boxes"], results["scores"], labels):
            x1, y1, x2, y2 = (float(v) for v in box.tolist())
            out.append(Detection(box=Box(x1, y1, x2, y2),
                                 label=str(label) or text,
                                 confidence=float(score)))
        return out


class Owlv2:
    """Google's OWLv2 -- a ViT detector with a CLIP-style text tower.

    Included as the architectural counterweight to Grounding DINO: two
    open-vocabulary detectors that fail the same way is a family result,
    and two that disagree says the family is not the variable.
    """

    def __init__(self, model: str = DEFAULT_OWLV2,
                 threshold: float = 0.02,
                 device: Optional[str] = None,
                 dtype: Optional[str] = None):
        try:
            import torch
            from transformers import Owlv2ForObjectDetection, Owlv2Processor
        except ImportError as exc:  # pragma: no cover
            raise PerceptionUnavailable(
                "OWLv2 needs transformers + torch. `pip install -r "
                "requirements-perception.txt`.") from exc
        self._torch = torch
        self.device = device or "cpu"
        self.processor = Owlv2Processor.from_pretrained(model)
        # fp16 is not a micro-optimisation here: it is the precision an Orin
        # Nano would most plausibly run this at, and a ViT's accuracy under
        # reduced precision is exactly the thing the hardware decision assumes
        # and has never measured. `None` keeps the historical fp32 behaviour so
        # existing scored records stay reproducible.
        self._dtype = {"fp16": torch.float16, "float16": torch.float16,
                       "bf16": torch.bfloat16, "fp32": torch.float32,
                       None: None}[dtype]
        self.model = Owlv2ForObjectDetection.from_pretrained(
            model, torch_dtype=self._dtype).to(self.device).eval()
        self.dtype = dtype or "fp32"
        self.threshold = threshold
        self.weights = model
        self.model_name = model

    def detect_text(self, image: bytes, text: str) -> Sequence[Detection]:  # pragma: no cover
        img = _pil(image)
        inputs = self.processor(text=[[text]], images=img,
                                return_tensors="pt").to(self.device)
        if self._dtype is not None:
            # Only pixel_values: input_ids must stay integral.
            inputs["pixel_values"] = inputs["pixel_values"].to(self._dtype)
        with self._torch.no_grad():
            outputs = self.model(**inputs)
        # post_process wants fp32 -- it compares against a float threshold and
        # builds boxes in image coordinates.
        outputs.logits = outputs.logits.float()
        outputs.pred_boxes = outputs.pred_boxes.float()
        target_sizes = self._torch.tensor([img.size[::-1]])
        # `post_process_grounded_object_detection`, NOT
        # `post_process_object_detection`: the Owlv2*Processor* has only the
        # first, while the Owlv2*ImageProcessor* one layer down has only the
        # second. Checking the wrong one of the two cost a full 299-frame
        # run that came back `unavailable` on every single frame -- which
        # 1.12's tri-state is exactly why that read as "the harness is
        # broken" rather than as "OWLv2 finds nothing".
        results = self.processor.post_process_grounded_object_detection(
            outputs=outputs, threshold=self.threshold,
            target_sizes=target_sizes)[0]
        out = []
        for box, score in zip(results["boxes"], results["scores"]):
            x1, y1, x2, y2 = (float(v) for v in box.tolist())
            out.append(Detection(box=Box(x1, y1, x2, y2), label=text,
                                 confidence=float(score)))
        return out



class OmDetTurbo:
    """omlab's OmDet-Turbo -- open-vocabulary detection built for real time.

    Added 2026-09-11 because the bench had never been checked against the
    HuggingFace zero-shot-detection list, only against the models this
    document happened to name. OmDet-Turbo is the one candidate whose stated
    goal is the axis the edge actually cares about: open vocabulary AT frame
    rate, rather than open vocabulary at any cost.

    Its processor takes a LIST of classes rather than a sentence, which is a
    different prompt convention from Grounding DINO's period-terminated
    phrase -- and getting it wrong degrades quietly rather than raising.
    """

    def __init__(self, model: str = DEFAULT_OMDET,
                 threshold: float = 0.02,
                 device: Optional[str] = None):
        try:
            import torch
            from transformers import (AutoProcessor,
                                      OmDetTurboForObjectDetection)
        except ImportError as exc:  # pragma: no cover
            raise PerceptionUnavailable(
                "OmDet-Turbo needs transformers + torch. `pip install -r "
                "requirements-perception.txt`.") from exc
        self._torch = torch
        self.device = device or "cpu"
        self.processor = AutoProcessor.from_pretrained(model)
        self.model = OmDetTurboForObjectDetection.from_pretrained(
            model).to(self.device).eval()
        # Low on purpose -- score once, sweep the gate afterwards.
        self.threshold = threshold
        self.weights = model
        self.model_name = model

    def detect_text(self, image: bytes, text: str) -> Sequence[Detection]:  # pragma: no cover
        img = _pil(image)
        inputs = self.processor(img, text=[text],
                               return_tensors="pt").to(self.device)
        with self._torch.no_grad():
            outputs = self.model(**inputs)
        results = self.processor.post_process_grounded_object_detection(
            outputs, text_labels=[[text]], threshold=self.threshold,
            nms_threshold=0.5, target_sizes=[img.size[::-1]])[0]
        out = []
        labels = results.get("text_labels", results.get("labels", []))
        for box, score, label in zip(results["boxes"], results["scores"],
                                     labels):
            x1, y1, x2, y2 = (float(v) for v in box.tolist())
            out.append(Detection(box=Box(x1, y1, x2, y2),
                                 label=str(label) or text,
                                 confidence=float(score)))
        return out


class LlmDet(GroundingDino):
    """iSEE-Laboratory's LLMDet.

    A Grounding DINO derivative trained with an LLM supplying richer
    captions, so it loads through the same Auto classes and keeps the same
    period-terminated prompt convention -- which is why this subclasses
    rather than copies. If a future transformers ships a dedicated
    LlmDetForObjectDetection, only the parent needs to change.

    166k downloads on the hub against Grounding DINO's 1.65M, and never
    tested here.
    """

    def __init__(self, model: str = DEFAULT_LLMDET, **kwargs):
        super().__init__(model, **kwargs)


class YoloWorld:
    """Ultralytics YOLO-World, the model 4.11 already measured.

    Kept here so the table can be re-run rather than quoted: 4.11's numbers
    were produced by an ad-hoc script that no longer exists, which is
    precisely the problem P3 was written to stop.
    """

    def __init__(self, weights: str = DEFAULT_YOLOWORLD,
                 confidence: float = 0.02,
                 device: Optional[str] = None):
        try:
            from ultralytics import YOLOWorld as _YW
        except ImportError as exc:  # pragma: no cover
            raise PerceptionUnavailable(
                "YOLO-World needs ultralytics. `pip install -r "
                "requirements-perception.txt`.") from exc
        self.model = _YW(weights)
        self.confidence = confidence
        self.device = device
        self.weights = weights
        self.model_name = weights
        self._classes = None

    def detect_text(self, image: bytes, text: str) -> Sequence[Detection]:  # pragma: no cover
        if self._classes != [text]:
            self.model.set_classes([text])
            self._classes = [text]
        results = self.model.predict(_pil(image), conf=self.confidence,
                                     verbose=False, device=self.device)
        out = []
        for r in results:
            for b in getattr(r, "boxes", []):
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
                out.append(Detection(box=Box(x1, y1, x2, y2), label=text,
                                     confidence=float(b.conf[0])))
        return out


class VlmDetector:
    """An open-weight VLM asked whether the target is in the frame.

    Added 2026-09-09, and it is aimed at ONE measured failure rather than at
    "a better model". The three basket walks isolated the local tier's limit:
    **86% recall on a close target against 6% on a distant one**, and the
    cause is the crop source, not the classifier -- CLIP scores the close
    basket 0.99 and the distant one 0.10 because the detector proposes no
    crop containing it at that range.

    A VLM of this family does not letterbox to a fixed 640. Qwen2.5-VL and
    InternVL tile at native resolution and can emit boxes directly, which is
    a different mechanism aimed exactly at the frames that fail. That makes
    this worth measuring where Grounding DINO was not: DINO added capacity on
    the axis that was already working (4.11 -- it loses at every matched
    operating point), and this one addresses the axis that is not.

    ## What it thresholds on, and why it is not the box

    Every other backend here produces a sweepable score, and the whole
    harness -- `recall_at_fp_budget`, the matched-precision tables -- depends
    on that. A grounding output has no such number: a box is present or it is
    not, which gives one operating point and no curve, and a single point is
    how a model gets compared at whatever threshold happens to flatter it.

    So the score is **P(yes)** for a yes/no question, read out of the logits
    of the first generated token rather than from the generated text. That is
    continuous, comparable across models, and sweepable like everything else.
    The grounding pass runs only to place a box (and therefore a bearing),
    and only when the answer was yes.

    ## Read the cautions before trusting a number from this

    **Benchmark scores do not transfer here.** A camera at 10-13cm in one
    basement is out of distribution for every model in this family, and their
    published MMBench-style figures say nothing about it. This corpus is the
    only eval that means anything, which is the entire reason it exists.

    **And it has to beat the cheap fix, not the current state.** If a tiling
    VLM recovers the distant basket, that is evidence the floor mask and
    lidar clusters would too -- on a $70 part rather than a $400 one. The
    comparison that matters is against those, not against today's 6%.
    """

    def __init__(self, model: str = DEFAULT_VLM,
                 ground: bool = True,
                 max_pixels: Optional[int] = None,
                 device: Optional[str] = None):
        try:
            import torch
            from transformers import AutoModelForImageTextToText, AutoProcessor
        except ImportError as exc:  # pragma: no cover
            raise PerceptionUnavailable(
                "the VLM backend needs transformers + torch. `pip install -r "
                "requirements-perception.txt`.") from exc
        self._torch = torch
        # cuda -> mps -> cpu, matching ClipScorer. Omitting mps here cost a
        # smoke test 4-16 MINUTES a frame on an Apple laptop against a couple
        # of minutes, and the number would have read as "a VLM is hopeless at
        # the edge" when it was really "this backend picked the wrong device".
        self.device = device or ("cuda" if torch.cuda.is_available()
                                 else "mps" if torch.backends.mps.is_available()
                                 else "cpu")
        kwargs = {}
        if max_pixels:
            # The tiling budget IS the variable under test -- it is what gives
            # a small distant object enough pixels to survive the encoder.
            # Left at the model's default unless asked, so a run says which.
            kwargs["max_pixels"] = max_pixels
        self.processor = AutoProcessor.from_pretrained(model, **kwargs)
        self.model = AutoModelForImageTextToText.from_pretrained(
            model, dtype="auto").to(self.device).eval()
        self.ground = ground
        self.weights = model
        self.model_name = model
        self._yes, self._no = self._answer_tokens()

    def _answer_tokens(self):
        """Token ids that count as yes and as no.

        Several spellings each, because tokenizers differ on leading spaces
        and case, and a missing variant silently costs probability mass to
        the wrong side -- which would look like a badly calibrated model
        rather than a harness bug.
        """
        tok = self.processor.tokenizer
        def ids(words):
            out = set()
            for w in words:
                for form in (w, " " + w, w.capitalize(), " " + w.capitalize()):
                    enc = tok.encode(form, add_special_tokens=False)
                    if len(enc) == 1:
                        out.add(enc[0])
            return sorted(out)
        return ids(["yes"]), ids(["no"])

    def _ask(self, img, prompt: str, max_new_tokens: int = 1):  # pragma: no cover
        messages = [{"role": "user", "content": [
            {"type": "image", "image": img}, {"type": "text", "text": prompt}]}]
        inputs = self.processor.apply_chat_template(
            messages, add_generation_prompt=True, tokenize=True,
            return_dict=True, return_tensors="pt").to(self.device)
        with self._torch.no_grad():
            out = self.model.generate(**inputs, max_new_tokens=max_new_tokens,
                                      do_sample=False, output_scores=True,
                                      return_dict_in_generate=True)
        return inputs, out

    def detect_text(self, image: bytes, text: str) -> Sequence[Detection]:  # pragma: no cover
        """One grounding pass. The score is **P(it localises)**, not P(yes).

        Measured 2026-09-09 on three frames, and it changed this backend's
        design within an hour of writing it:

        | frame | P(yes) | grounding |
        |---|---|---|
        | basket close | 0.925 | `[918, 287, 1073, 442]` |
        | **basket distant** | **0.884** | **`[]`** |
        | no basket in the room | 0.097 | `[]` |

        **The model answers "yes, visible" at 0.88 on a frame it then refuses
        to point at.** A yes/no question can be answered from prior -- a home
        gym plausibly contains a laundry basket -- without anything having
        been found, and that is exactly the confabulation this project already
        measured in the cloud tier at 23% precision. Grounding cannot be
        answered that way: a box is a claim about a location.

        So the score comes from the localisation, and it is still continuous
        and sweepable: after the opening `[` the model emits either `{` (a box
        follows) or `]` (empty), and the softmax over those two IS the
        probability that it found something. Same shape as the yes/no readout,
        asked about the question that cannot be bluffed.
        """
        img = _pil(image)
        p_found, box = self._locate(img, text)
        if box is not None:
            return [Detection(box=box, label="vlm:grounded", confidence=p_found)]
        # No box, but the probability it *nearly* emitted one is the score --
        # dropping to 0.0 here would collapse the curve to two points and make
        # every matched-precision comparison meaningless.
        return [Detection(box=Box(0.0, 0.0, float(img.width), float(img.height)),
                          label="vlm:ungrounded", confidence=p_found)]

    def _locate(self, img, text: str):  # pragma: no cover
        """-> (probability it localised, Box or None).

        An unparseable or empty answer keeps its probability and returns no
        box: a full-frame box would put the bearing dead ahead, and a wrong
        bearing is worse than an absent one (1.16 #4, which cost this project
        a field that was never once a number).
        """
        import json as _json
        import re
        _, out = self._ask(
            img, f"Output the bounding box of the {text} as JSON: "
                 '[{"bbox_2d": [x1, y1, x2, y2]}]. If it is not visible, '
                 "output []. Output JSON only.", max_new_tokens=96)
        ids = out.sequences[0][-len(out.scores):].tolist()
        p_found = self._p_localised(ids, out.scores)
        # ONLY the generated tokens. Decoding the whole sequence includes the
        # prompt, and the prompt contains the literal example
        # `[{"bbox_2d": [x1, y1, x2, y2]}]` -- which the regex below matches
        # first, json.loads then chokes on "x1", and a perfectly good
        # grounding is reported as `vlm:ungrounded`. It read as the model
        # refusing to localise a basket it had in fact boxed correctly.
        text_out = self.processor.batch_decode(
            [ids], skip_special_tokens=True)[0]
        m = re.search(r"\[\s*\{.*?\}\s*\]", text_out, re.S)
        if not m:
            return p_found, None
        try:
            items = _json.loads(m.group(0))
            x1, y1, x2, y2 = (float(v) for v in items[0]["bbox_2d"])
        except Exception:
            return p_found, None
        if x2 <= x1 or y2 <= y1:
            return p_found, None
        return p_found, Box(x1, y1, x2, y2)

    def _p_localised(self, ids, scores) -> float:  # pragma: no cover
        """Softmax over `{` against `]` at the step right after the opening
        `[` -- the exact token where the model commits to having found
        something or not."""
        tok = self.processor.tokenizer
        opens = {i for w in ("[", " [") for i in tok.encode(w, add_special_tokens=False)[:1]}
        brace = [i for w in ('{', ' {', '{"') for i in tok.encode(w, add_special_tokens=False)[:1]]
        close = [i for w in ("]", " ]") for i in tok.encode(w, add_special_tokens=False)[:1]]
        for pos, tid in enumerate(ids):
            if tid in opens and pos + 1 < len(scores):
                probs = self._torch.nn.functional.softmax(
                    scores[pos + 1][0].float(), dim=-1)
                b = float(probs[brace].sum()); c = float(probs[close].sum())
                return b / (b + c) if (b + c) > 0 else 0.0
        return 0.0


OPEN_VOCAB_BACKENDS = {
    GDINO: (GroundingDino, DEFAULT_GDINO),
    OWLV2: (Owlv2, DEFAULT_OWLV2),
    YOLOWORLD: (YoloWorld, DEFAULT_YOLOWORLD),
    OMDET: (OmDetTurbo, DEFAULT_OMDET),
    LLMDET: (LlmDet, DEFAULT_LLMDET),
    TRTOWLV2: (None, DEFAULT_TRTOWLV2),   # resolved lazily -- importing
                                          # TensorRT at module load would
                                          # break every machine without it
    VLM: (VlmDetector, DEFAULT_VLM),
}


def _open_vocab(spec: str, device=None, dtype=None, **extra):
    name, _, override = spec.partition(":")
    if name == TRTOWLV2:
        # Imported here, not at module scope: TensorRT exists on one rented
        # GPU box and nowhere else, and `import brain.perceive_lab` must keep
        # working on a laptop.
        from tools.trt.trt_owlv2 import TrtOwlv2
        if not override:
            raise ValueError("trtowlv2 needs an engine path: "
                             "--detector trtowlv2:/path/to/owlv2_int8.engine")
        return TrtOwlv2(override, device=device)
    cls, default = OPEN_VOCAB_BACKENDS[name]
    kwargs = {}
    if name == OWLV2 and dtype:
        # Only OWLv2 carries a dtype today; passing it to a backend that does
        # not would be a TypeError three minutes into a corpus run, which is
        # the same trap the VLM branch below exists to avoid.
        kwargs["dtype"] = dtype
    if name == VLM:
        # Only the VLM takes these, and passing them to a detector that does
        # not would be a TypeError three minutes into a corpus run.
        if extra.get("vlm_max_pixels"):
            kwargs["max_pixels"] = extra["vlm_max_pixels"]
        kwargs["ground"] = extra.get("vlm_ground", True)
    return cls(override or default, device=device, **kwargs)


def pipeline_for_spec(target: str, *, detector: str = DEFAULT_DETECTOR,
                      clip_model: str = DEFAULT_CLIP,
                      proposer: str = "none",
                      segmenter: str = DEFAULT_SEGMENTER,
                      sam_model: str = DEFAULT_SAM,
                      crop_path: str = DEFAULT_CROP_PATH,
                      affinity_k: Optional[int] = None,
                      confidence: Optional[float] = None,
                      max_crops: Optional[int] = None,
                      device: Optional[str] = None,
                      dtype: Optional[str] = None,
                      imgsz: Optional[int] = None,
                      vlm_max_pixels: Optional[int] = None,
                      vlm_ground: bool = True,
                      **kwargs):
    """One string per stage -> something with `perceive(frame)`.

    `detector` is a YOLO weights file (the shipped path), `none` (so a
    proposer can be measured alone, which is the row that showed the floor
    mask is worse alone and better together), or one of `gdino` / `owlv2` /
    `yoloworld` -- optionally `name:model_id` -- which replace the whole
    pipeline rather than a stage of it.
    """
    if detector.partition(":")[0] in OPEN_VOCAB:
        if proposer != "none":
            raise ValueError(
                f"{detector} takes the target string into the detector, so "
                "there is no crop stage for a proposer to feed. Compare it as "
                "a replacement for the pipeline (4.11), or use a YOLO "
                "detector with --proposer.")
        return OpenVocabPipeline(
            _open_vocab(detector, device=device, dtype=dtype,
                        vlm_max_pixels=vlm_max_pixels, vlm_ground=vlm_ground),
            target, **kwargs)

    if proposer == "floor":
        region = SegformerFloorProposer(segmenter, device=device)
    elif proposer == "sam":
        region = SamProposer(sam_model, device=device)
    elif proposer == "none":
        region = None
    else:
        raise ValueError(f"unknown proposer {proposer!r}: none, floor or sam")

    if detector == "none":
        backend = NullDetector()
    elif detector.startswith("replay:"):
        # replay:<detections.json>[:<recordings dir>] -- recorded boxes
        # from a quantized run, so the TIER can be scored rather than just
        # the detector. Everything downstream is unchanged, which is the
        # point: the only variable is where the proposals came from.
        parts = detector.split(":")
        backend = ReplayDetector(parts[1],
                                 parts[2] if len(parts) > 2 else "recordings")
    else:
        backend = YoloDetector(detector, device=device, imgsz=imgsz)
    return PerceptionPipeline(
        detector=backend,
        scorer=ClipScorer(clip_model, device=device),
        target=target,
        proposer=region,
        crop_path=crop_path,
        **({"max_crops": max_crops} if max_crops is not None else {}),
        **({"affinity_k": affinity_k} if affinity_k is not None else {}),
        **({"confidence": confidence} if confidence is not None else {}),
        **kwargs,
    )


__all__ = [
    "GDINO", "OWLV2", "YOLOWORLD", "OPEN_VOCAB", "OPEN_VOCAB_BACKENDS",
    "DEFAULT_GDINO", "DEFAULT_OWLV2", "DEFAULT_YOLOWORLD", "DEFAULT_SAM",
    "DEFAULT_OPEN_VOCAB_CONFIDENCE",
    "GroundingDino", "Owlv2", "YoloWorld", "VlmDetector", "DEFAULT_VLM",
    "OmDetTurbo", "LlmDet", "OMDET", "LLMDET", "DEFAULT_OMDET",
    "DEFAULT_LLMDET",
    "VLM", "SamProposer", "NullDetector",
    "OpenVocabPipeline", "pipeline_for_spec",
]
