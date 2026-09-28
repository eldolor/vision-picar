# A10G corpus runs, 2026-09-11/12

Per-frame scores from `control/perception_eval.py` on an **NVIDIA A10G**
(g5.xlarge, ~$1/hr, torn down). Two instances, ~3.7 h, **~$3.70** total.

**Why these exist:** every row published in `PLAN-onboard-perception.md`
4.11 and P5 was scored on **four** walks. The corpus has **eight** labelled
walks -- 610 frames, 159 visible. So the published comparison mixes a subset
with a corpus, and no model had ever been scored against all of it. These
records fix that: **11 valid configs, all on the same 610 frames.**

    python -m control.perception_eval compare \
        evaluations/gpu/a10g-fp16.json \
        evaluations/gpu/a10g-shipped-8walk.json --fp-budget 0,3,16

## The control that licenses reading any of this

`a10g-fp32-4walk.json` is `a10g-fp32.json` filtered to the four published
walks. It reproduces the committed CPU record **exactly** -- 26% / 96% /
100% at gates 0.677 / 0.119 / 0.067, same TP, same FP. A GPU changes
nothing about what a detection is, so these numbers are comparable to the
older ones. Without that check every row would carry an unfalsifiable
"different hardware" caveat.

> **Does not generalise -- 2026-09-20 (P24).** This control holds for
> OWLv2, a ViT. It does NOT hold for YOLOE, a CNN: `yoloe-11s-seg` reads 251
> true positives on an A10G (`evaluations/gpu-yoloe/yoloe-11s-seg_none.json`)
> against 258 on the laptop CPU (`evaluations/tier-decomp/F_yoloe-11s.json`),
> same 1234 frames. TF32 was ruled out; the software stack differs. Never
> merge GPU and CPU rows for a CNN detector.

## Results, all 8 walks, at matched false-positive budgets

| config | @0 FP | @3 FP | @16 FP | ms |
|---|---|---|---|---|
| OWLv2-large | 8% | **86%** | **97%** | 717 |
| **OWLv2-base fp16** | 12% | **82%** | 92% | **111** |
| Qwen2.5-VL-3B | 42% | 79% | 79% | 725 |
| InternVL3-2B | 3% | 75% | 89% | 625 |
| YOLO11s + floor + CLIP (shipped *as of 2026-09-12; since P23/P24 the tier ships `yoloe-11s-seg` + CLIP, no floor mask*) | 8% | 58% | 70% | 1210 |
| **Grounding DINO** | **50%** | 55% | 71% | 226 |
| YOLO-World | 47% | 53% | 74% | **16** |
| LLMDet | 11% | 18% | 72% | 276 |
| OmDet-Turbo | 7% | 7% | 13% | 49 |

`a10g-fp32.json` is the same model as `-fp16` at full precision: **identical
recall at every budget**, gates matching to three decimals, 1.8x slower.
That is the precision question answered -- it had been assumed everywhere
and measured nowhere.

Latency is A10G, PyTorch eager, and **includes CPU preprocessing** (~17 ms
VGA, ~92 ms at 1280 -- OWLv2's own anti-aliased resize). It compares models
to each other, never to the robot's budget (2.9).

## Two runs are INVALID and are kept only so nobody re-derives them

- **`a10g-sam-8walk.json`** -- every frame scored exactly 0.0. Run with
  `--metric confidence`, but SAM + CLIP yields a probability, not a detector
  confidence. Wrong metric, not a weak model.
- **`a10g-owlvit-8walk.json`** -- 610/610 `unavailable`. Loaded via the
  `gdino:` prefix, so OWL-ViT got Grounding DINO's post-processing. The
  model never ran.

## Not run, and why

**Qwen3-VL-2B and -4B.** Three independent problems, all measured here:

1. **Upstream performance bug.** 5262 ms/token against Qwen2.5-VL-3B's 77 --
   68x slower, on a *smaller* model, with *fewer* input tokens (897 vs
   1224), same `sdpa` attention. Known and unfixed upstream:
   QwenLM/Qwen3-VL#1811, sgl-project/sglang#14078. 4.5 h per model.
2. **`--vlm-max-pixels` is silently ignored.** 897 input tokens at budget
   `None` and at `921600`. It works on Qwen2.5-VL (1224 -> 1153). **So P5's
   "Qwen3-VL-4B at native tiling" was not at native tiling** -- the flag was
   passed and discarded. The tiling sweep that established "native is the
   peak" ran on Qwen2.5-VL-3B, the default, and that conclusion was carried
   to a model where the control does not exist.
3. Its score is **non-separable** (4e-07 margin), so it cannot be read at a
   matched budget at all.

$9 of GPU time to add two rows that are mislabelled, unreadable and losing.
The diagnosis above is worth more than the rows.
