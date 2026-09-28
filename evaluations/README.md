# Scored perception records, 2026-09-08/09

> **Index, added 2026-09-28.** This file describes only the records in this
> directory's root. The subdirectories each have their own README. **Never
> compare rows across corpora** -- P7 and P23 each found a headline that was
> an artefact of mixing a subset with the whole. Frame counts below are
> read from the records themselves.
>
> | directory | what | corpus (frames / walks) |
> |---|---|---|
> | `.` (this file) | the 2026-09-08/09 CPU records: per-walk runs plus 4.11/P5's comparison rows | 299 / 4 for the comparison rows; single-walk files (38-209 frames) otherwise |
> | `gpu/` | P7: 11 configs on an A10G, 2026-09-11/12 | 610 / 8 (`a10g-fp32-4walk.json` is the 299 / 4 control) |
> | `gpu/softgate/` | the soft label gate swept, A10G, 2026-09-12 | 610 / 8 |
> | `gpu/pacing/` | Phase D: pacing the deliberation call, cloud stubbed | 610 / 8, as 15 visible spans (not per-frame records) |
> | `gpu-yoloe/` | P24: the YOLOE family with and without the floor mask, A10G, 2026-09-20 | 1234 / 11 -- do not merge with CPU rows (see its README) |
> | `trt/` | TensorRT fp16 vs INT8 on OWLv2, 2026-09-12 | 610 / 8 |
> | `hailo/` | P6's OWLv2 -> Hailo-8L compile attempt; `zoo-probe/` holds P15-P18. Hailo path closed 2026-09-19 | compile reports; `zoo-probe/`'s tier rows are 365 / 11 |
> | `tier-decomp/` | P19-P24's CPU latency lineage and the records behind the shipped default | 365 / 11 (P19's pinned subsample) and 1234 / 11 |
>
> The labelled corpus has since grown to 21 walks / 2261 frames, and
> `perception_eval score` scores every labelled walk -- pin the walk set
> with `--walk` (or `--frames-from`) before comparing a new run to any row
> here.

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
