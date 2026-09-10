# Edge Perception Bench

**Phases P1-P6, 2026-09-07 -> 2026-09-09.** Nine model configurations scored on
adjudicated rig walks, then one toolchain experiment that overturned the
hardware plan.

Published version: https://claude.ai/code/artifact/f3dd0506-0e26-4cd2-aea9-e64998966380

This document summarises. `PLAN-onboard-perception.md` sections 4.10, 4.11, P5
and P6 hold the reasoning and the dated findings; `evaluations/` and
`evaluations/hailo/` hold the records every number here is computed from.

---

## Recommendation

**Run OWLv2 as the search-proposal tier, on a Jetson Orin Nano -- not on a
Hailo-8L.**

OWLv2 won the accuracy bench outright: 68 of 68 visible frames across three
targets at zero false positives, beating three VLMs, two open-vocabulary
detectors and the cloud model itself, at ~150M parameters and 2.1s/frame.

It does **not** compile to a Hailo-8L. The image tower translates and quantizes
cleanly; it fails at hardware allocation on 73 layernorm and 38 softmax layers
-- every per-token reduction in the transformer. That is an allocator limit,
not a capacity limit, so no smaller input and no compiler flag moves it.

The upside: this is a far cheaper Jetson than previously costed. The board was
going to be bought to run a 2-4B VLM at INT4 in 8GB. It is now bought to run a
**150M ViT**, which leaves the memory for SLAM and nav2 -- removing the main
objection to the Jetson rather than paying for it.

---

## 1. The question

The robot has three jobs for a camera: *don't hit things*, *steer toward the
target*, and *find the target in the first place*. The first two work. The
third was failing, measurably.

The shipped on-board pipeline -- a COCO detector proposing boxes, a
floor-segmentation mask proposing more, and CLIP scoring each crop against the
target string -- recovers **86% of sightings at close range and 6% at
distance**. On the one walk built as a genuine search it found the target in
**1 of 23 visible frames**.

That is not a weak tier, it is an absent one. With search proposal effectively
silent, the architecture falls back to calling the cloud on a timer and the
robot can drive past its target indefinitely.

## 2. The corpus, and the one that was deleted

Every number is scored against **hand-adjudicated per-frame labels**
(`labels.json`), never the robot's own recorded claims. A walk without labels
is refused by the scorer rather than scored on weaker evidence.

Eleven walks, 968 frames, phone on a wheeled rig at floor height (10-13cm),
target on the floor, landscape locked. Four targets -- red backpack, blue
bottle, blue shoes, woven laundry basket -- two COCO nouns and two deliberately
not.

**An earlier corpus of 39 walks was deleted on purpose.** Reading the frames
rather than the JSON showed the camera at standing height looking *down* onto
furniture, with the target on an ottoman or a console table -- a position a
floor robot cannot reach. Every number measured on it answered a question about
a task the robot cannot perform, from a viewpoint it will never have. An
invalid corpus left in the bucket is how it gets scored against by accident.

The scorer (`control/perception_eval.py`) reproduced a previously published row
exactly on first run -- 62/74 at a gate of 0.8, three false positives -- which
is the only validation a scorer can have.

## 3. Three method rules, each bought with a wrong answer

**Score on grounding, never on yes/no.** Asked "is the basket visible?", a VLM
answered `P(yes) = 0.884` on a frame it then refused to draw a box on. A yes/no
question can be answered from prior -- a home gym plausibly contains a laundry
basket -- which is the cloud tier's own 23%-precision failure reproduced in a
3B model. The score became *P(it localises)*, read at the token where the model
commits to a bracket. A box is a claim about a location and cannot be bluffed.

**Read recall only at a matched false-positive budget, and check
separability.** A recall figure without its false-positive count is how a
detector run at low confidence looks like a win. The second half matters as
much: one model's headline 91% rested on a separating margin of `4e-07` --
scores of exactly 1.0 or 0.0, nothing between. That is saturation, not ranking;
it yields one operating point, not a curve, and `recall_at_fp_budget()` now
reports `separable: false` for exactly this shape.

**A contiguous run of false positives is a labelling error until proven
otherwise.** A model flagged frames 0081-0085 that adjudication had marked
absent. Opening them: the basket is plainly visible through a doorway, and the
first labelling pass had missed the whole second span where the rig turns back.
The labels were corrected and everything rescored -- which moved the *baseline
down*, from 6% to 4%.

## 4. The bench

### Whole corpus -- 299 frames, 74 visible

| configuration | @ 0 FP | @ 3 FP | shape |
|---|---|---|---|
| **OWLv2** (ViT-B/16, ~150M) | 26% | **96%** | ranks |
| YOLO-World | **51%** | 51% | flat -- cannot be swept |
| YOLO11s + floor mask + CLIP (shipped) | 15% | 85% | ranks |

**YOLO-World genuinely wins at exactly zero false positives** -- 51% against
OWLv2's 26%. But it is flat: identical recall at 0 and 3 FP, because its
confidence saturates. OWLv2 climbs 26% -> 96% over the same budget. That is why
the claim is worded as *"wins at every operating point that admits a single
false positive."*

### The distant-target walk -- 86 frames, 23 visible

| configuration | params | s/frame | @ 0 FP | @ 3 FP |
|---|---|---|---|---|
| **OWLv2** | ~150M | 2.1 | **74%** | **83%** |
| Qwen3-VL-2B | 2B | 8.4 | 22% | 39% |
| InternVL3-2B | 2B | 17 | 17% | 72% |
| YOLO11s + floor + CLIP (shipped) | ~9M | 1.2 | 4% | -- |

Timings are laptop-relative and compare models to each other, never model to
robot. Qwen3-VL-4B posted a higher headline on grounding, but its score is
non-separable (`4e-07` margin), so it has no curve to read at a matched budget.

### Where it beats the cloud outright

On the 209-frame search walk a rack of storage bins repeatedly convinced the
cloud model it had found the target.

| tier | recall | precision | behaviour on negatives |
|---|---|---|---|
| Claude Opus 4.5 (cloud) | 10/10 | 23% | confident, specific, wrong |
| **OWLv2** (on-board) | 10/10 | **100%** | emits no box at all |

True sightings score 0.73-0.76; the bins top out at 0.035. A 20x gap with
nothing in it. This is a **correctness** role for the on-board tier, not a cost
role -- nothing in the cloud tier can catch the cloud tier being wrong.

### Three targets, three capture resolutions, zero false positives

| walk | target | capture | OWLv2 @ 0 FP |
|---|---|---|---|
| blue-shoes-...210511 | out of vocabulary | 1280 | 25/25 = 100% |
| red-backpack-...144856 | COCO `backpack` | VGA | 33/33 = 100% |
| blue-bottle-...185007 | the 209-frame search | VGA | 10/10 = 100% |

A method note worth more than the result: OWLv2's *fixed-gate* table reads 3/25
on the shoes walk while its swept table reads 25/25 -- its confidence scale
simply sits low. A single-threshold comparison would have discarded the best
model in the bench.

## 5. The compile experiment (P6)

The accuracy result made the recommendation *Pi 5 + Hailo-8L* -- a $70
accelerator against a $400 board -- and rested it entirely on one untested
question: **can OWLv2 compile to a Hailo executable?** Hailo's own
compatibility notes list OWLv2 under *"does not fit: anything attention-heavy
... no efficient attention path and no memory for the weights."*

Only the image side needs to compile. The text tower stays on the CPU -- it
runs once per *target string*, not once per frame -- so the graph is cut one
operation before the text mixes in, and a five-tensor join happens on the host.
That split was verified against the unmodified model before anything was
rented: max |d score| `1.4e-05`, top-50 patch set identical.

**Dataflow Compiler 3.34.0, `hw_arch=hailo8l`, r6i.4xlarge, 3.1 hours, $3.20:**

| variant | translate | optimize | compile | failing layers |
|---|---|---|---|---|
| 960px, stock graph | ok | ok | FAIL | `conv1` |
| 640px -- 1600 tokens | ok | ok | FAIL | `conv1` |
| 640px + `automatic_reshapes=enabled` | ok | ok | FAIL | `conv1` |
| 960px, patch conv factored | ok | ok | FAIL | 73 layernorm, 38 softmax |

**Translation and quantization were never the problem.** The ViT-B/16 parses in
44-107s and quantizes with no memory failure, at 3600 tokens *and* at 1600. The
compiler carries a LayerNorm Decomposition pass, Matmul Equalization and
MatmulDecompose, and used all three. The published "no memory for the weights"
is simply false here.

**One convolution hid the real wall for three attempts.** The first three
variants all died on `conv1` -- the *single* convolution in a 575-node graph,
the 16x16-stride-16 patch embedding:

```
Reshape is needed for layers: conv1, but adding a reshape has failed.
```

The compiler wants to rewrite that convolution as a space-to-depth plus a 1x1
and cannot place the reshape. So the rewrite was done in the export instead,
where it is exact arithmetic rather than a heuristic -- `3->48 k4s4` with
one-hot weights (a space-to-depth by 4), then `48->768 k4s4` carrying the
original weights re-indexed. A 16x16 patch is a 4x4 grid of 4x4 blocks, so the
composition is an identity, verified at max |d score| `1.4e-05`.

It worked. `conv1` vanished from the error and this appeared in its place:

| layers named in the final failure | count |
|---|---|
| layer normalization (reduce_mean / sub / square / mult) | 73 |
| softmax (reduce_max / sub / sum / mult) | 38 |
| precision change | 36 |
| matmul | 9 |
| convolution | 8 |

**Every per-token reduction in the transformer, not a subset.** The allocator
cannot place the reshapes they require. That is an architectural property of
the dataflow design rather than a size limit -- which is exactly why no smaller
input and no flag helped, and why the convolution failing first was actively
misleading.

## 6. Architecture

Nothing here is one model doing everything. Three tiers run at three rates
against three different questions, and the slow one never blocks the fast one.

```
 ON THE ROBOT (Jetson Orin Nano)                        |  CLOUD
 ------------------------------------------------------+------------------
 continuous - safety
   RPLidar C1  --range ring-->  safety collar  --veto-->  [ DRIVE LOOP ]
                                path_clearance()          holds a goal
                                no model                  never blocks
 15-30 Hz - reactive                                      -> motors
   Camera Mod 3 --frame-->  YOLO11s INT8  --bearing-->        ^
                            86% close range                   |
 0.2-1 Hz - deliberation                                      | egocentric
   Camera ------frame--->  OWLv2 ViT-B/16                     | goal, never
                           image tower -> accelerator  2.1s/f | a motor
                           text tower  -> CPU   once/target   | command
                           join [3600x512].[512xQ]            |
                                 |                            |
                              detected                        |
                                 v                            |
                           TRIGGER GATE                       |
                           mission_start                      |
                           candidate_sighting                 |
                           cold_search                        |
                                 |                            |
                       1 call / ~4 frames ------> Claude Opus 4.5
                                                  event-driven, ~3.5s
                                                  whole frame, not a crop
```

The lidar answers *am I about to hit something* and is the only path that can
stop the car. YOLO answers *which way to steer*. OWLv2 answers *have I found
it* and decides whether the cloud is called at all. The cloud returns a
bearing, not a move -- which is why a 3.5s round trip costs goal staleness
rather than stopping the robot.

### What the compile result actually changed

```
  REJECTED - Pi 5 + Hailo-8L ($70)      RECOMMENDED - Jetson Orin Nano (~$400)
  ---------------------------------     -------------------------------------
  YOLO11s reactive   --------> gate     YOLO11s reactive   --------> gate
  compiles, 92 FPS                      unchanged
                                                          candidate_sighting
  OWLv2 search        ...x... gate      OWLv2 search       --------> gate
  DOES NOT ALLOCATE                     runs, 2.1 s/frame
  73 layernorm, 38 softmax              ~150M params, INT8

  Only cold_search fires - a timer.     The sighting trigger fires on evidence.
  Search proposal recall: 4%            Search proposal recall: 74-100%
  The robot can drive past its target.  8 GB left for SLAM and nav2.
```

**One missing edge is the whole decision.** Both boards run the reactive tier
identically -- YOLO compiles for the Hailo off the shelf. The difference is the
single arrow from search proposal into the trigger gate: without a model that
can serve it, `candidate_sighting` never fires on evidence and the system falls
back to a timer. The board is not being bought for throughput; it is being
bought for one arrow.

## 7. Decision

**A Jetson Orin Nano, sized for a 150M ViT rather than a 2-4B VLM.** The board
costs roughly $320-430 more than the Pi 5 plan it replaces, on a build
previously budgeted around $555-620.

**The case is materially cheaper than it was.** Every previous costing assumed
the board existed to run a multi-billion-parameter VLM at INT4, which raised
the objection that 8GB shared with the GPU cannot hold SLAM, nav2, a detector
and a VLM at once. Dropping the local VLM removes that objection rather than
paying for it.

### Three things the compile result does not touch

- **The reactive tier's part.** YOLO11 n/s/m compile for the Hailo-8L off the
  shelf and measure 86% close range at 92 FPS. A two-board split -- Hailo for
  reactive, a second board for search -- stays a real option, and a worse one
  on power and cost than one Jetson doing both.
- **OWLv2's accuracy.** 68/68 visible frames, zero false positives, 100%
  precision against the cloud's 23% on the walk that broke the identity tier.
  Those are PyTorch numbers and they are why the model is worth a board at all.
- **The host/accelerator split.** Text tower on the CPU, five per-patch tensors
  joined with one matmul on the host (`tools/hailo/owlv2_host_head.py`). That
  division is correct on a Jetson too; only the accelerator changes.

### What is still open

- **Nothing here was measured on silicon.** No executable was produced, so INT8
  quantization effects on accuracy are unmeasured. Hardware-day item.
- **Calibration data is at the limit.** The compiler wants 1024 frames for its
  higher optimization levels; the entire corpus is 968. More calibration means
  more walks, not a bigger machine -- and a statistics pass over the full
  corpus runs ~5.4h per configuration on CPU, so a deployable build wants a GPU
  host.
- **A newer compiler could reverse this.** The limit is the allocator, not the
  model. `tools/hailo/ec2.sh up` re-runs the whole loop -- which is the reason
  it was built rather than the answer being looked up.

---

## Reproducing any of it

```bash
# score a config over the corpus, then compare at a matched FP budget
python -m control.perception_eval score --detector owlv2 --save owlv2.json
python -m control.perception_eval compare evaluations/owlv2.json \
    evaluations/shipped-floor.json --fp-budget 0,3

# the compile loop
python -m tools.hailo.export_owlv2_onnx --out build/owlv2 --image <frame> --text "<target>"
python -m tools.hailo.calibration_set --out build/owlv2 --n 128
tools/hailo/ec2.sh up && tools/hailo/ec2.sh push && tools/hailo/ec2.sh setup
tools/hailo/ec2.sh compile && tools/hailo/ec2.sh pull && tools/hailo/ec2.sh down
```

Per-frame scores are under `evaluations/`; compile reports and compiler logs
under `evaluations/hailo/`. Scoring is `control/perception_eval.py`; the compile
loop is `tools/hailo/`. Timings compare models to each other on one laptop,
never model to robot.
