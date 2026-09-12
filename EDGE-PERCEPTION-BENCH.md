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

OWLv2 won the accuracy bench outright. Scored across all eight labelled
walks -- 610 frames, 159 visible -- it reads **82% at three false
positives** against the shipped pipeline's 58%, at a ninth of the latency,
and it beats four VLMs and four other open-vocabulary detectors. Per walk,
tuned per walk, it reaches **68 of 68 visible frames at zero false
positives** across three targets. Those are different measurements and the
difference matters: one gate per walk is not one gate for a house.

It does **not** compile to a Hailo-8L. The image tower translates and quantizes
cleanly; it fails at hardware allocation on 73 layernorm and 38 softmax layers
-- every per-token reduction in the transformer. That is an allocator limit,
not a capacity limit, so no smaller input and no compiler flag moves it.

The upside: this is a far cheaper Jetson than previously costed. The board was
going to be bought to run a 2-4B VLM at INT4 in 8GB. It is now bought to run a
**150M ViT**, which leaves the memory for SLAM and nav2 -- removing the main
objection to the Jetson rather than paying for it.

---

## 0. Orientation: what the system is

An indoor autonomous robot car. A differential-drive chassis carrying a camera,
a 360-degree lidar and a single-board computer, driving itself around a house
looking for a named object -- *"find the woven laundry basket"* -- with a vision
LLM in the cloud for the reasoning it cannot do on board.

Four parts, four different questions. The whole engineering problem is deciding
which part answers which, because they differ by three orders of magnitude in
speed and by real money per call.

```
 THE ROBOT CAR - differential drive, ~0.15-0.5 m/s        |  CLOUD - Bedrock
 ---------------------------------------------------------+------------------
  ┌────────────────────┐                                   |
  │ Camera Module 3    │--frames-->┌──────────────────┐    |
  │ 30 fps             │           │  AI AT THE EDGE  │    |
  │ 10-13 cm off floor │           │                  │    |
  │ "what is in front  │           │ reactive detector│    |
  │  of me"            │           │ open-vocab search│    |
  └────────────────────┘           │ safety collar    │    |
                                   │                  │    |
  ┌────────────────────┐           │ free, every frame│    |
  │ RPLidar C1         │--stop---->│ "is the target   │    |
  │ 360° metric ring   │ authority │   here"          │    |
  │ "am I about to hit │           └────────┬─────────┘    |
  │  something"        │                    │              |
  └────────────────────┘                    │  ~1 call per 4 frames,
                                            │  on a trigger
                                            ├──────────────>  Claude Opus 4.5
                                            │              |  vision LLM
                                            v              |  ~3.5 s round trip
                                   ┌──────────────────┐    |  costs money/call
                                   │ Drive loop       │<---+-- a goal,
                                   │  -> motors       │    |   not a move
                                   │ "where do I go"  │    |
                                   └──────────────────┘    |  "what should I
                                                           |   do next"
```

The camera is the only sensor that can tell a laundry basket from a bin; the
lidar is the only one trusted to stop the car. The edge board runs perception
on every frame for free, and decides whether the cloud is worth waking. The
cloud reasons about the whole scene and answers in seconds -- so it returns a
goal the drive loop can hold, never a move to execute now.

### Why the work is split at all

Sending every frame to the cloud would be simpler. Three measured reasons it is
not done that way, and the third is the one nobody predicts.

- **Cost.** A five-minute walk is hundreds of frames. Paying a frontier model
  for each one turns a hobby robot into a metered service. With the trigger
  discipline in place, a measured mission spent **4 paid calls over 18 steps**.
- **Latency.** A cloud round trip is ~3.5s. A robot moving at 0.3 m/s covers a
  metre in that time, so the cloud *cannot* sit in the driving loop. It returns
  an egocentric goal and the drive loop holds it; staleness is the cost, not
  stopping.
- **Correctness.** The surprise. On one walk a rack of storage bins convinced
  the cloud model it had found the target -- **23% precision across 44 claims**.
  The on-board tier rejected all 34 bin frames. Nothing in the cloud tier can
  catch the cloud tier being wrong; only a second, independent pair of eyes can.

That last point is what turns on-board perception from an optimisation into a
requirement -- and it is why the rest of this document is about finding a model
good enough to hold that job, and a board that can run it.

Everything here is validated simulation-first: a digital twin drives the same
control API the hardware will, so a change is proven on a phone before it is put
on a car. The evaluation below is the exception that cannot be simulated -- a
detector has to see real pixels, so the corpus is real photographs from a rig
pushed across a real floor.

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

### The whole corpus -- 610 frames, 159 visible, 11 models

**Corrected 2026-09-11.** This section previously read "whole corpus -- 299
frames" and reported 96% for OWLv2. Those were **four** of the corpus's
**eight** labelled walks. Nothing had ever been scored against all of it.
Re-run on one GPU, every model on the same 610 frames:

| configuration | @ 0 FP | @ 3 FP | @ 16 FP | ms |
|---|---|---|---|---|
| OWLv2-large | 8% | **86%** | **97%** | 717 |
| **OWLv2-base fp16** | 12% | **82%** | 92% | **111** |
| Qwen2.5-VL-3B | 42% | 79% | 79% | 725 |
| InternVL3-2B | 3% | 75% | 89% | 625 |
| YOLO11s + floor + CLIP (shipped) | 8% | 58% | 70% | 1210 |
| **Grounding DINO** | **50%** | 55% | 71% | 226 |
| YOLO-World | 47% | 53% | 74% | **16** |
| LLMDet | 11% | 18% | 72% | 276 |
| OmDet-Turbo | 7% | 7% | 13% | 49 |

**Every headline moved, and the conclusion survived.** OWLv2 reads 82%, not
96%. But the shipped pipeline falls further -- 85% to 58% -- so OWLv2's
margin over it **widens from 11 points to 24**, at 11x the speed. The four
excluded walks were the hard ones, including 95 frames with nothing in them
at all: a pure false-positive test that can only ever cost recall.

**Grounding DINO owns the zero-false-positive point** -- 50% where the
shipped pipeline gets 8% and OWLv2 gets 12%. That falsifies a published
claim that the composed pipeline "dominates at every operating point," and
it suggests something useful: a model that is *never wrong* is a better
corroborator than one that is more often right.

**And two models nobody had tried both lose.** The earlier bench tested the
models this project happened to name; HuggingFace's own zero-shot detection
list had three untried families. OmDet-Turbo (7% at 3 FP) and LLMDet (18%)
are real results rather than null ones, for about forty cents of GPU time.

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
                            TensorRT, 86% close range         |
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
  compiles, 92 FPS (HEF)                same role, TensorRT
                                                          candidate_sighting
  OWLv2 search        ...x... gate      OWLv2 search       --------> gate
  DOES NOT ALLOCATE                     runs, 2.1 s/frame
  73 layernorm, 38 softmax              ~150M params, INT8

  Only cold_search fires - a timer.     The sighting trigger fires on evidence.
  Search proposal recall: 4%            Search proposal recall: 74-100%
  The robot can drive past its target.  8 GB left for SLAM and nav2.
```

### Does YOLO survive the board change?

**Yes -- but for one reason, and it is rate, not accuracy.** OWLv2 runs at
2.1s/frame, which is 0.5 Hz. At 0.3 m/s that is **63 cm of travel between
decisions**. Nothing steers on that. The reactive tier needs 15-30 Hz and OWLv2
cannot serve it at any input size.

This is *not* an accuracy argument. At close range OWLv2 is the better detector
-- 33/33 on the approach walk against the shipped pipeline's 86%. YOLO stays
because it is fast, and only because it is fast.

**Its justification changes between the two boards.** On Pi + Hailo, YOLO was
the only detector that could run at all. On a Jetson it is a deliberate choice
to spend a cheap model on the fast loop. Three concrete things change with it:

- **The runtime is not a HEF.** It becomes a TensorRT INT8 engine. The compile
  step still exists; it is a different toolchain, and a far less exotic one.
- **The 92 FPS figure does not transfer.** That is 10.9ms per invocation
  measured on the Hailo-8 series (4.3.1). YOLO11s on an Orin Nano is
  *unmeasured in this project*. It will comfortably clear 30 Hz, but that is an
  expectation, not a number.
- **The 86% does transfer** -- it is a property of the model and the corpus, not
  of the chip.

**And two models now share one processor, one of them 63x the other's frame
budget.** This is the real cost of consolidating onto one board. On Pi + Hailo
the accelerator ran the detector while the CPU stayed free for the text tower.
On a Jetson, YOLO and OWLv2 contend for the same GPU. The reactive frame budget
at 30 Hz is 33.3ms; an OWLv2 inference is 2.1s -- **63 reactive frames long**.

Contention itself is not new (2.9 already budgeted a detector, a floor mask and
CLIP against one 33ms frame), but the magnitude is: those were 7-50ms jobs. A
2.1-second one needs a real answer -- preemption, separate CUDA streams with
priority, or simply accepting dropped reactive frames while a search inference
runs. **There is no answer yet, and it is an open item, not a solved one.**

**One missing edge is the whole decision.** Both boards run the reactive tier
identically -- YOLO compiles for the Hailo off the shelf. The difference is the
single arrow from search proposal into the trigger gate: without a model that
can serve it, `candidate_sighting` never fires on evidence and the system falls
back to a timer. The board is not being bought for throughput; it is being
bought for one arrow.

## 6b. What the GPU re-run changed -- 2026-09-11

Two A10G instances, 3.7 hours, $3.70, both torn down. Records in
`evaluations/gpu/`.

**fp16 costs no accuracy.** Identical true positives at every budget, gates
matching to three decimals, 1.8x faster. Every latency figure and every
hardware argument in this document assumed a quantised ViT keeps its
accuracy; none had measured it. **INT8 still has not been measured**, and
the projections below assume INT8 -- that is now the load-bearing gap.

**The Orin projection moved 2.4x when assumptions became measurements** --
from 51 ms to **124 ms** (8 Hz). Measuring GPU and CPU separately rather
than deriving them corrected a derivation that was 65% wrong on the CPU
term.

**And the bottleneck is not the model.** At INT8 on a 1280 capture an Orin
would spend **36 ms detecting and 229 ms resizing the photograph** --
OWLv2's own anti-aliased resize, in Python, on the CPU. Fix the resize
(GPU, or capture nearer 960); do not fix it by capturing at VGA, because VGA
is upscaled to 960 and loses the detail the distant targets need.

**Three consequences for the architecture**, all recorded in
`PLAN-onboard-perception.md` P7c:

- **YOLO is not redundant.** At 8 Hz OWLv2 cannot serve a 15-30 Hz reactive
  tier. An earlier claim in this document's own figures that one model could
  replace three is withdrawn.
- **The out-of-vocabulary gap is an odometry problem, not a perception
  one.** YOLO has no COCO class for a laundry basket, so only OWLv2 can
  identify it -- at 8 Hz. But the target is *static*: the only thing moving
  the bearing is the robot, which encoders and the IMU already measure. A
  detection becomes a goal pose; the reactive tier recomputes the bearing
  from odometry at 30 Hz and re-detection corrects drift.
- **Arrival must move off the cloud.** `target_reached` is read from the
  cloud reply today, so it lands ~5.6 s late -- 1.7 m of overshoot at
  0.3 m/s, past the thing the robot was stopping at. It should be the lidar
  measuring range at the bearing OWLv2 supplies.

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
- **OWLv2's accuracy.** 82% at three false positives over the full 610-frame
  corpus -- 24 points clear of the shipped pipeline at a ninth of the
  latency -- and, per walk, 68/68 visible frames at zero false positives
  with 100% precision against the cloud's 23% on the walk that broke the
  identity tier. Those are PyTorch numbers and they are why the model is
  worth a board at all.
- **The host/accelerator split.** Text tower on the CPU, five per-patch tensors
  joined with one matmul on the host (`tools/hailo/owlv2_host_head.py`). That
  division is correct on a Jetson too; only the accelerator changes.

### What is still open

- **Two tiers now share one GPU.** An OWLv2 inference is 63 reactive frames
  long. Scheduling that against a 15-30 Hz loop is unsolved, and it is a problem
  the two-chip Hailo plan did not have.
- **YOLO11s on an Orin Nano is unmeasured.** The 92 FPS everyone quotes is a
  Hailo-8 number. The expectation is comfortable, but a hardware-day benchmark
  should replace the expectation.
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
