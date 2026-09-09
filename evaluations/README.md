# Scored perception records, 2026-09-08/09

Per-frame output of `control/perception_eval.py`, one file per
(model x corpus) cell. **Kept because they cost about six hours of model
inference** and because `compare` reads them without re-running anything:

    python -m control.perception_eval compare \
        evaluations/basket-owlv2-215252.json \
        evaluations/vlm-qwen3-4b.json --fp-budget 0,3

Each file carries its own `config` block (detector, imgsz, proposer,
crop_path, metric), so a row can always say what produced it.

**Read every number at a matched false-positive budget AND check
`separable`.** Qwen3-VL-4B's headline 91% on the distant basket rested on a
separating margin of 4e-07 -- min true positive 0.000000 against max false
positive 1.000000 -- which is saturation, not ranking.
`recall_at_fp_budget()` reports `separable: False` for exactly this.

The reference for every score is each walk's adjudicated `labels.json`,
never `walk.jsonl`. Walk `...215252`'s labels were corrected on 2026-09-09
(a second visible span, frames 81-85, was missed on the first pass), so any
number computed against the earlier version understates every model.
