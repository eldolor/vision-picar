# P24 -- the YOLOE family, with and without the floor mask

22 rows, one `g5.2xlarge` (A10G), 2.37 h, $2.90, torn down. All 1234
labelled frames of the 11-walk corpus, 323 visible. Config is P23's shipped
one except for the swept axes: `--clip RN50 --crop-path low_confidence
--affinity-k 8 --max-crops 16 --device cuda --dtype fp32 --metric
probability`.

Files are `<checkpoint>_<proposer>.json`, proposer `none` or `floor`.

## Read these three cautions first

1. **Do not merge these rows with CPU rows.** `yoloe-11s_none` reads 251
   true positives here against 258 in `evaluations/tier-decomp/F_yoloe-11s.json`
   on the same frames -- so P7's control ("CUDA fp32 reproduced the CPU
   record exactly") does NOT generalise from OWLv2's ViT to a CNN. TF32 was
   tested with `NVIDIA_TF32_OVERRIDE=0` and is **not** the cause; the
   software stack differs (torch 2.7.0+cu128 / ultralytics 8.4.156 here,
   2.14.0 / 8.4.142 on the laptop). All 22 rows share one box and one stack,
   so rankings WITHIN this directory are sound.
2. **There are no usable latency figures here.** The floor-mask rows are
   CPU-bound -- SegFormer plus region extraction, single-threaded, GPU at 0%
   -- so the matrix was run 6-way parallel and per-frame ms is meaningless
   by construction. Latency lives in `evaluations/tier-decomp/`.
3. **The two OWLv2 rows are missing**, not failed-and-hidden: the job list
   built `crops:owlv2:owlv2` from its own `<backend>:<checkpoint>` format
   and the model id was invalid. OWLv2's full-corpus row exists locally as
   `F_owlv2.json` (62% at 3 FP) and it is out of contention, so it was not
   worth re-renting for.

## What the rows say

The mask is negative or neutral on 9 of 11 checkpoints and positive only on
`26m` (+2) and `26x` (+4) -- the two weakest no-mask rows. `yoloe-11s` leads
at a 3-FP budget by 7 points; `yoloe-26l` is the best single row at 16 FP.
See `PLAN-onboard-perception.md` P24.
