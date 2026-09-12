# The soft label gate, swept -- A10G, 2026-09-12

`control/perception_eval.py` records from `tools/gpu/sweep.py`, on an
**NVIDIA A10G** (g5.xlarge, us-east-2, ~4.2 h, **~$4.20**, torn down). Same
GPU as P7, which is why these rows can sit beside `evaluations/gpu/`'s.

**The question.** 4.2's crop gate has had two settings and both are
measured: the hard `label_gate`, and -- since 2026-09-07, after it lost 11
of 18 true positives on the bottle walk -- the open-vocabulary
`low_confidence`. The hard gate failed because at 10cm the detector
*relabels* the target (`bottle` -> `vase`, once `refrigerator`), discarding
exactly the approach frames. The fix deleted the gate, and **2.9's
per-frame budget now pays for it**: every surviving proposal is one more
CLIP image encode, against 3.1x headroom on an 8L rather than the 10-30x
this plan assumed for a while.

`brain/perceive.py`'s `soft_gate` is the interior of that axis. COCO's 80
class names are ranked against the mission's target string in CLIP's
**text** space, once at mission start, and the top `k` become the accept
set. k=1 is close to the hard gate; **k=80 is the open-vocabulary path
exactly**, so the sweep contains its own control.

## The three controls, all of which hold

- **`soft-k80` == `shipped-lowconf`**, frame for frame on all 610.
- **`soft-k80+floor` == `shipped-lowconf+floor`**, likewise. Both checked by
  comparison, not by eye.
- **`shipped-lowconf+floor` reproduces P7's committed published row**:
  7.5% / 57.9% / 69.8% against 8% / 58% / 70%. A rebuilt harness scoring
  the shipped pipeline to within rounding is what licenses the rest.

## Results, all 8 walks, 610 labelled frames, 159 visible

`crops/fr` is CLIP image encodes per frame -- 2.9's actual cost, and the
thing the gate exists to reduce.

### Detector only

| config | crops/fr | @0 FP | @3 FP | @16 FP |
|---|---|---|---|---|
| `auto` (4.2's rule) | 1.65 | 39.6% | 45.9% | 56.0% |
| **`shipped-lowconf`** | 2.58 | **44.7%** | **52.8%** | **57.2%** |
| `soft-k1` | **0.17** | 43.4% | 43.4% | 43.4% |
| `soft-k2` | 0.67 | 35.2% | 44.7% | 47.8% |
| `soft-k4` | 0.90 | 35.2% | 44.7% | 47.2% |
| `soft-k8` | 1.10 | 37.1% | 48.4% | 52.8% |
| `soft-k16` | 1.33 | 37.1% | 42.1% | 50.3% |
| `soft-k32` | 1.62 | 42.1% | 49.7% | 56.6% |
| `soft-k80` (= shipped) | 2.58 | 44.7% | 52.8% | 57.2% |

### Detector UNION floor mask

| config | crops/fr | @0 FP | @3 FP | @16 FP |
|---|---|---|---|---|
| `shipped-lowconf+floor` | 5.51 | 7.5% | 57.9% | 69.8% |
| `soft-k1+floor` | **3.39** | 6.9% | 63.5% | **73.0%** |
| `soft-k2+floor` | 3.79 | 6.9% | 53.5% | 71.1% |
| `soft-k8+floor` | 4.18 | 6.9% | 54.1% | **73.0%** |
| `soft-k80+floor` (= shipped) | 5.51 | 7.5% | 57.9% | 69.8% |

## The result, in one line

**Alone the gate strictly costs recall. With a class-agnostic proposer
beside it, it cuts 24-38% of the CLIP budget for accuracy that is a wash.**
It is worth having as an option and is not a new default.

## Why it fails alone, and the reason generalises

The accept sets were recorded with every run rather than summarised --
written that way *before* the run, because a flat curve alone cannot
distinguish "the gate does not matter" from "the ordering is nonsense." For
`"blue bottle"` CLIP's text space ranks:

    bottle(0.792) bicycle(0.736) cup(0.735) apple(0.726) bird(0.725) ...

**`vase` is 31st of 80. `refrigerator` is 61st.** Those are the two labels
that caused the defect the gate was built to fix. Admitting them needs
k >= 31 and k >= 61, by which point the gate has stopped gating. The spread
explains it -- 0.79 down to 0.71 across the top ten, and 0.704 to 0.685 for
`"blue and yellow running shoes"` -- CLIP was trained to align image with
text, not text with text.

**The empirical table says it from the other side.** What YOLO11s actually
calls the target, over the open path, on frames where it is visible:

| target | YOLO11s says |
|---|---|
| blue bottle | **vase x12**, bottle x5, refrigerator x1 |
| blue and yellow running shoes | **bed x5**, couch x5, suitcase x2, chair x1 |
| burgundy backpack | backpack x27, handbag x1, bed x1, suitcase x1 |
| woven laundry basket | **handbag x20**, skateboard x1 |

The detector's confusions are **geometric at 10cm** -- a bottle from below
is a vase, a shoe on the floor is a large flat thing, a basket is a
handbag. CLIP's text space encodes **semantic** relatedness. They coincide
on `backpack` and on `basket -> handbag` and miss completely on
`bottle -> vase` and `shoes -> bed`.

**That is the general result, and it is not about this gate.** Any scheme
that derives an in-vocabulary noun from the target string predicts what the
target *is*, where the gate needs to predict what the detector will *say*.
This is 4.3.1's standing-height caveat arriving a third time, now as a
statement about label space rather than about mAP.

## Why it works beside the floor mask

The mask proposes on geometry and carries no class, so it passes the gate
untouched by construction. It therefore supplies the recall the gate would
otherwise cost, leaving the gate to do the one thing it is good at:
**suppressing the detector's junk boxes.** That shows in the non-target
score distribution, which is the mechanism rather than an inference --
the 17th-highest non-target score is **0.683** for the shipped path and
**0.469-0.502** for the gated ones. A lower false-positive floor means a
lower threshold fits inside the same budget.

This is the same "they fail on different frames" result already recorded in
`PerceptionPipeline`'s own comment (detector 86%, mask 59%, both 94%),
observed from the precision side.

## The methodological finding, which outlasts the result

**A matched-false-positive comparison at a SMALL budget is hostage to
single frames.** At 3 FP over 610 frames the gate is set by the 4th-highest
non-target score:

| config | top non-target scores | gate | TP |
|---|---|---|---|
| `shipped+floor` | 1.0, 0.948, 0.910, **0.883** | 0.883 | 92 |
| `soft-k1+floor` | 1.0, 0.948, 0.854, **0.743** | 0.743 | 101 |
| `soft-k2+floor` | 1.0, 0.948, 0.910, **0.883** | 0.883 | 85 |

k=1 excludes **one** crop scoring 0.910. That single crop moves the gate by
0.14 and carries 16 true positives with it -- and it is the whole of k=1's
apparent +5.6 and k=2's apparent -4.4. The 3-FP column swung ten points on
one frame.

P7 added `recall_at_fp_budget` because a single *fixed* threshold discarded
the best model in the bench. This is the opposite failure and needs the
opposite guard: **quote @16 FP**, where the gate sits in a smooth part of
the distribution, and read the per-walk table beside it.

Per walk, at each walk's own 0-FP point -- the stable read -- the four
configs are a wash: **100 / 105 / 100 / 102** true positives out of 159.

| walk | shipped | k=1 | k=2 | k=8 |
|---|---|---|---|---|
| blue-bottle ...142454 | **100%** | 72% | 72% | 72% |
| blue-bottle ...185007 | 10% | 10% | 10% | 10% |
| blue-shoes ...152528 | 15% | 15% | 15% | 15% |
| blue-shoes ...210511 | 60% | 68% | 68% | 68% |
| red-backpack ...144856 | 100% | 100% | 100% | 100% |
| basket ...215140 (nothing to find) | 0% | 0% | 0% | 0% |
| basket ...215252 | 26% | 22% | 26% | **35%** |
| basket ...215351 | 68% | **92%** | 76% | 76% |

## What this leaves open

1. **Crop ranking.** `max_crops` sorts by AREA, which `PerceptionPipeline`
   already calls "the weak part". A better ranking cuts encodes with no
   gate at all, and would not carry the gate's recall cost.
2. **An empirical confusion map** built from the corpus -- which labels the
   detector puts on a target's box at 10cm -- rather than a language
   model's opinion. The table above is that map for four targets. Whether
   it generalises to a target with no labelled walk is exactly the case a
   gate has to serve, and is untested.

## A run note worth keeping

The first sweep **died silently about three hours in** -- no OOM, no
traceback, the log simply stopped mid-walk and read as a slow run rather
than a dead one. Cause: `nohup ... &` under SSM RunShellScript, whose agent
reaps the document's process group when the command completes.
`tools/gpu/ec2.sh` uses `setsid` now, and `sweep.py` grew `--only` so a
resumed run pays for the missing configs rather than all of them. This is
the second time in this project an environment failure arrived dressed as
an answer (P6's `KeyError: 'USER'`, P7d's fp16-behind-an-INT8-flag), which
makes it a pattern rather than an anecdote.
