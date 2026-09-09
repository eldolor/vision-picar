"""The half of OWLv2 that stays on the Pi's CPU.

The accelerator returns five per-patch tensors and knows nothing about the
target string. This joins them to a text embedding and produces exactly what
`Owlv2ForObjectDetection` produces: logits, boxes, objectness.

It is numpy and about twenty lines of arithmetic, which is the point. The
text tower runs once per target -- not once per frame -- so the expensive
part is the ViT and the cheap part is here. Same division CLIP already uses
on this project (4.2), and the same division Hailo's own CLIP port uses.

Two `head` modes, matching `export_owlv2_onnx.py`'s two exports:

* `full`   -- the accelerator did the L2 normalise, the ELU and the box
              sigmoid. This only does the text einsum.
* `minimal` -- those three ops stayed here, because they are the ops in that
              graph least likely to exist in a CNN toolchain and they cost
              microseconds on [3600, 512] and [3600, 1] tensors. Numerically
              identical; `export_owlv2_onnx.verify` asserts it for both.

`verify` imports this rather than duplicating it, so the two halves cannot
drift.
"""

from __future__ import annotations

from typing import Mapping, Sequence

import numpy as np


def normalize(vectors: np.ndarray) -> np.ndarray:
    """L2-normalise along the last axis, matching transformers' epsilon."""
    return vectors / (np.linalg.norm(vectors, axis=-1, keepdims=True) + 1e-6)


def join(parts: Mapping[str, np.ndarray], query_embeds: np.ndarray,
         head: str = "full", box_bias: np.ndarray | None = None):
    """Reassemble accelerator outputs into (logits, boxes, objectness).

    `parts` holds the five ONNX/HEF outputs by name; `query_embeds` is
    [n_queries, dim] or [batch, n_queries, dim] from the text tower,
    un-normalised (this normalises it, as the original head does).
    `box_bias` is required for the `minimal` head and ships in the export's
    metadata JSON.
    """
    image_class_embeds = np.asarray(parts["image_class_embeds"],
                                    dtype=np.float32)
    logit_shift = np.asarray(parts["logit_shift"], dtype=np.float32)
    logit_scale = np.asarray(parts["logit_scale"], dtype=np.float32)
    pred_boxes = np.asarray(parts["pred_boxes"], dtype=np.float32)

    if head == "minimal":
        if box_bias is None:
            raise ValueError("the minimal head needs box_bias from "
                             "owlv2_export.json")
        image_class_embeds = normalize(image_class_embeds)
        # ELU + 1, elementwise on [B, P, 1].
        logit_scale = np.where(logit_scale > 0, logit_scale,
                               np.expm1(np.minimum(logit_scale, 0.0))) + 1.0
        pred_boxes = 1.0 / (1.0 + np.exp(
            -(pred_boxes + np.asarray(box_bias, dtype=np.float32))))
    elif head != "full":
        raise ValueError(f"head must be 'full' or 'minimal', got {head!r}")

    query = np.asarray(query_embeds, dtype=np.float32)
    if query.ndim == 2:
        query = query[None, ...]
    query = normalize(query)

    # The one op the accelerator never does: [B,P,512] x [B,512,Q].
    logits = image_class_embeds @ np.swapaxes(query, -1, -2)
    logits = (logits + logit_shift) * logit_scale

    return logits, pred_boxes, np.asarray(parts["objectness_logits"],
                                          dtype=np.float32)


def detections(parts: Mapping[str, np.ndarray], query_embeds: np.ndarray,
               threshold: float = 0.02,
               size: Sequence[int] = (960, 960),
               head: str = "full", box_bias: np.ndarray | None = None):
    """Post-process to (score, xyxy) pairs in pixels.

    Mirrors `Owlv2Processor.post_process_grounded_object_detection` for the
    single-query case, so a HEF can be scored by `control/perception_eval.py`
    through the same `Detector` Protocol as everything else.

    Note the coordinate convention OWLv2 actually uses: boxes are cxcywh
    relative to the PADDED square the processor produces, not to the original
    frame. `brain/perceive_lab.py` gets that undone by the processor; a
    caller here has to undo the padding itself.
    """
    logits, boxes, _ = join(parts, query_embeds, head=head, box_bias=box_bias)
    scores = (1.0 / (1.0 + np.exp(-logits)))[0].max(axis=-1)
    boxes = boxes[0]

    height, width = size
    keep = scores >= threshold
    cx, cy, w, h = (boxes[keep] * np.array([width, height, width, height])).T
    xyxy = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=-1)
    return list(zip(scores[keep].tolist(), xyxy.tolist()))


__all__ = ["join", "detections", "normalize"]
