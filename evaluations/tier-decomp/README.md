# The on-board tier, decomposed -- CPU records, 2026-09-18/20

Per-frame output of `control/perception_eval.py score --save`, all run on
the **laptop CPU, fp32**, for `PLAN-onboard-perception.md` P19-P24. This
directory is the **latency lineage** for the shipped tier: every `ms` a
P20-P24 table quotes comes from here, measured on one machine, so rows
compare to each other and never to a robot (the Orin is a projection,
P7b / P20). GPU rows live in `evaluations/gpu-yoloe/` and must not be merged
with these -- see that README's first caution.

Each file carries its own `config` block (detector, proposer, crop path,
`max_crops`, metric) and one record per frame with `walk`, `frame`,
`visible`, `status`, `score` and `ms`. Written 2026-09-28 from the records
and the PLAN text; the phase attribution below is by content (config and
frame count), since the files carry no phase or date field.

## Which file backs the shipped default

**`F_yoloe-11s.json`** -- `crops:yoloe:yoloe-11s-seg.pt`, CLIP RN50,
`low_confidence`, 16 crops, no proposer (no floor mask), `metric
probability`. It is what `brain/perceive.py`'s defaults reproduce. On all
1234 frames (323 visible): **258 true positives / 2 false positives at the
shipped `P >= 0.8` gate (80%)**, 82% at 3 FP, mean 139 ms. `G_11s_crops4.json`
is the same config at 4 crops (251 TP, mean 127 ms), which is the
measurement behind `DEFAULT_MAX_CROPS = 16`.

## Two corpora -- read the frame count before any row

| frames / walks | files | what |
|---|---|---|
| **365 / 11** | `R*`, `C_*`, `W_*`, `E_*` | **P19's subsample**, pinned in `p19_frames.json` |
| **1234 / 11** | `F_*`, `G_*`, `H_*` | every labelled frame of the 11-walk corpus as it stood 2026-09-19 |

**`p19_frames.json`** is a JSON list of 365 `"<walk>/<frame>.jpg"` keys. P19
restricted OWLv2 to the frames its YOLO-World replay covered (~40 per walk),
and that restriction is invisible in a saved `config` -- so P20 extracted the
keys to a file and every 365-frame row here was scored with
`--frames-from evaluations/tier-decomp/p19_frames.json`. P23 found that P20,
P21 and P22 had all inherited this 30% subsample, and that on all 1234
frames the detector ranking inverts. **Quote a 365-frame row only against
another 365-frame row.**

The 1234 set is not pinned by a file. It is the 11 walks whose `labels.json`
has no `proposed_by` field (the 2026-09-07/08/12/13 rig walks), and the
records' own `walk` fields list them. The labelled corpus has since grown to
21 walks / 2261 frames and `score` covers every labelled walk, so reproduce
a 1234-frame row by passing those 11 walks with `--walk`.

## The files, by prefix

| prefix | phase | what |
|---|---|---|
| `R1`-`R7` | P20 | the tier around OWLv2, taken apart: Grounding DINO crops + mask (`R1`), OWLv2 + SAM (`R2`), OWLv2 without the mask at 48 / 16 crops (`R3`, `R7`), with the mask at 16 / 8 crops (`R4_*`), OWLv2 as the whole pipeline (`R5`, **invalid** -- below), floor mask alone (`R6`) |
| `C_world_s_floor48` | P21 | live YOLO-World v2s in P19's baseline config (mask, 48 crops) -- the run that showed P19's replayed YOLO-World file was not what the live model emits |
| `W_{s,m,l,x}_{floor,none}` | P21 | the YOLO-World v2 size sweep, with and without the mask |
| `E_yoloe-*` | P22 | the YOLOE checkpoints on the 365 frames |
| `F_*` | P23 | the leaders re-run on all 1234 frames: `yoloe-11s` (shipped), `26l`, `26x`, `v8l`, and OWLv2 crops (62% at 3 FP here, against P20's 90% at 3 FP on the subsample) |
| `G_*` | P23/P24 | further 1234-frame rows: the remaining YOLOE checkpoints, and `G_11s_crops4` (the crop-budget measurement) |
| `H_yoloe-{11s,26l}_floor` | P24 | the floor mask back on, on CPU: 836 ms mean for `11s` against 139 without, and at the 0.8 gate 254 TP / 9 FP against 258 / 2 -- why the mask is off |

## Invalid rows

**`R5_owlv2_detector_only.json` is INVALID as accuracy.** It reads 100%
recall with 78 false positives: `--detector owlv2` is the whole-pipeline
replacement, which thresholds on box confidence, but it was scored with
`--metric probability`, and 78 non-visible frames score >= 0.99. Kept only
for its latency (2433 ms), which the mismatch does not affect.
`PLAN-onboard-perception.md` P20, "One run in `evaluations/tier-decomp/` is
INVALID", records it as the third time that metric mismatch produced a
number. (At the `P >= 0.8` gate it reads 152 / 2; the 100% figure is the
operating point the score distribution implies, where every visible frame
and 78 non-visible ones score >= 0.99 -- the ranking is saturated, not
separating.)
