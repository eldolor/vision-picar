"""YOLO-World as a CROP SOURCE for the shipped CLIP matcher.

`brain/perceive_lab.py`'s `OpenVocabPipeline` deliberately runs YOLO-World
as a **replacement** for all three models, and says so:

    "Deliberately NOT a `Detector` feeding the existing matcher. 4.11
     compared YOLO-World as a *replacement* -- one model instead of three
     -- because that is the shape of the 'run arbitrary Hugging Face
     models' argument, and comparing it any other way would answer a
     question nobody asked."

Someone asked it (2026-09-13). The reasoning behind that comment has also
weakened: it framed YOLO-World as the model a Jetson would be bought for,
and P7 falsified 4.11's "the three-model pipeline dominates at every
operating point", so which models to COMPOSE is open again.

The substantive case for this config: YOLO11s proposes crops from a fixed
COCO list, and "woven laundry basket" is not on it -- which is why
`crop_source` degrades to `low_confidence` on those walks and why 4.2's
label gate discards the close-range frames where YOLO calls the basket a
`vase`. YOLO-World proposes from the TARGET STRING, so CLIP would re-rank
crops that are already about the right object. That is a third pipeline,
not either measured one.

This is the adapter and nothing else -- the two speak different protocols
(`detect(image, confidence)` vs `detect_text(image, text)`). Scoring is
`control/perception_eval.py`'s, unchanged: that module is the instrument
and it has been rewritten from scratch three times already.
"""
from __future__ import annotations

from typing import Optional, Sequence


class YoloWorldCrops:
    """Satisfies `brain/perceive.Detector` by asking YOLO-World for the
    mission's target string.

    The target is fixed at construction because `Detector.detect()` has no
    text argument -- the pipeline is built per mission, exactly as
    `pipeline_for(target)` already is.

    `confidence` is deliberately LOW by default. These are proposals, not
    decisions: CLIP applies the real gate downstream, and a proposal the
    detector never emits is one CLIP can never rescue. That asymmetry is
    the whole reason a crop source and a classifier are different jobs.
    """

    def __init__(self, target: str, weights: str = "yolov8s-worldv2.pt",
                 confidence: float = 0.02, device: Optional[str] = None):
        if not target or not target.strip():
            raise ValueError("YoloWorldCrops needs a target string")
        from brain.perceive_lab import YoloWorld

        self.target = target.strip()
        self._backend = YoloWorld(weights=weights, confidence=confidence,
                                  device=device)
        self.weights = weights
        self.model_name = f"{weights} <- {self.target!r}"

    def detect(self, image: bytes, confidence: float) -> Sequence:
        # `confidence` is the PIPELINE's proposal floor; the backend was
        # built with its own. Take the lower of the two so this can never
        # silently discard crops the caller expected to see.
        dets = self._backend.detect_text(image, self.target)
        floor = min(confidence, self._backend.confidence)
        return [d for d in dets if d.confidence >= floor]


def pipeline_with_yoloworld(target: str, *, clip_model: str = "RN50",
                            clip_pretrained: str = "openai",
                            weights: str = "yolov8s-worldv2.pt",
                            floor_mask: bool = False,
                            segmenter: str = None,
                            device: Optional[str] = None,
                            **kwargs):
    """The shipped `PerceptionPipeline`, with YOLO-World as its crop
    source instead of YOLO11s. Same CLIP scorer, same gate, same
    `Perception` out -- so `control/perception_eval.py` takes it unchanged
    and the only variable between the two configs is the crop source.

    `floor_mask` adds 4.2's class-agnostic proposer alongside it, exactly
    as `brain.perceive.pipeline_for` does, so the four-way comparison
    (detector x floor mask) varies one thing at a time.
    """
    from brain.perceive import (DEFAULT_SEGMENTER, ClipScorer,
                                PerceptionPipeline, SegformerFloorProposer)

    proposer = None
    if floor_mask:
        proposer = SegformerFloorProposer(segmenter or DEFAULT_SEGMENTER,
                                          device=device)
    return PerceptionPipeline(
        detector=YoloWorldCrops(target, weights=weights, device=device),
        scorer=ClipScorer(clip_model, pretrained=clip_pretrained,
                          device=device),
        target=target,
        proposer=proposer,
        **kwargs,
    )
