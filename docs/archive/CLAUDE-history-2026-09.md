# CLAUDE.md history, moved out 2026-09-28

These are the dated narrative sections that used to sit inline in
`CLAUDE.md` -- moved here **verbatim** on 2026-09-28 so the orientation file
states current facts rather than making every session read 700 lines of
reversals first (`docs-review/REPORT.md`, fix #20). Nothing was edited below
this header. Several statements below were later reversed; where they
conflict with `CLAUDE.md`, `CLAUDE.md` governs.

---

## From section 3 (after the status table): the P-series, the hardware reversals, and the handoffs

**THE NEXT PHASE, decided 2026-09-19: map the house while it searches it
(`PLAN-mapping.md`, **N1 BUILT 2026-09-20**, N2-N7 proposed).** That is the stated trigger in
`PLAN-onboard-perception.md` 3.3 -- (b+) was "CHOSEN as the target **if mapping
proves to be the point**" -- so **ROS 2 enters the project**, as exactly one
containerised service exposing `GET /pose`, `GET /map`, `POST /goto` and
nothing else. Never (c)'s full adoption, and never anywhere near `brain/`,
`control/` or `RobotInterface`; `robot/safety.py` still owns the veto, because
nav2 plans on top of a safety layer rather than replacing one. Two of (b+)'s
three costs evaporated by accident: JetPack is Ubuntu 22.04 (the OS dilemma)
and the local VLM was dropped, leaving the 8GB to SLAM and nav2 (the memory
objection). The learning cost -- TF, URDF, nav2 params, launch files -- stands.
**N1-N4 need no hardware**, and N1 is the important one: define the wall
against `MockRobot` *before* any ROS exists, or the wall gets drawn around
whatever ROS emits and (b+) quietly becomes (c). Two prerequisites are
promoted from optional: **C2's continuous pose** (a SLAM pose cannot be
represented in `grid_world.py`'s integer cells and cardinal `Heading`, so the
twin cannot show this working -- a section 7 blocker) and **C1's pose method**.
`sweep` (1.7) is unblocked by N3 but its gating rule is unchanged.

**N1 is built and watchable** (2026-09-20, not deployed): `world/`,
`sim/mock_world.py`, `control/remote_world.py`, `GET /world/pose` +
`GET /world/map` with their routing entries, and the twin's map view under
the depth strip. Two things to know before extending it. **The map is
DISCOVERED, not copied** -- `MockWorld` casts a 360-degree ring from
wherever the robot stands and leaves everything behind a wall unknown;
handing over `GridWorld`'s layout would have been three lines, drawn a
finished house at mission start, and left `CELL_UNKNOWN` untested in the
only place it can be exercised without hardware. And **C2 turned out NOT
to gate this**: a quantised pose (cell centres, multiples of 90 degrees)
is a perfectly good pose, `x_m` is already a float, so the twin draws the
robot today and C2 makes the motion smooth later with nothing changing on
either side of the wall. `world.mode` must track `mode` -- the factory
refuses `world: sim` for a robot that has no grid, rather than returning
a plausible map of a house the robot is not in, which is why
`teleop-robot.yaml` and `service/tunnel/run.sh` both set `WORLD_MODE`.

**Session handoff, 2026-09-15: `HANDOFF-2026-09-15.md`** -- P9-P15, the
corpus now at 11 labelled walks, and the AWS near-miss where 18 of 22
walks existed only on the laptop. Read its section 4 before running
anything on a rented box, and section 5 for what is open. **Its "order
the Pi + Hailo-8L" headline was re-opened and partly overturned the same
day by P15**: the 8L's 45% still stands, but the 10H is no longer blocked
and the decision now turns on one unmeasured test -- see the 10H
paragraphs below.

**Previous handoff, 2026-09-13: `HANDOFF-2026-09-13.md`.** Phases A-G (the
deliberation call stops blocking; odometry; distance pacing; the interval
measured; the stand-in stops scanning; a local sighting steers), plus tier
latency instrumentation and an observability dashboard at `/metrics`. Read
its section 6 for what is still open and section 7 for what was wrong
along the way.

**HARDWARE DECISION -- CLOSED 2026-09-19: the board is a JETSON, and the
Hailo path is not being pursued.** Stated by the user. Everything below
about Pi-plus-Hailo, HEFs, DFC versions, allocation, INT8-on-Hailo and the
10H is **history, not an open question** -- keep it for the reasoning and
do not re-open it, and do not spend on a Hailo compile run. Two practical
consequences for anything after this line:

* **Compilability stops being a model-selection criterion.** P17's "OWLv2
  compiles to no Hailo" and P10/P13/P16's YOLO-World results no longer
  gate anything. On a Jetson every candidate runs, so the axes are recall
  and latency only.
* **Latency is the whole remaining question**, and it is the one the user
  raised: the tier is ~4.3 Hz projected (P20), against 1.14's assumption
  of a fast board. Model work should be judged on that. **ANSWERED the
  same day by P22, and the answer moves the problem off the model:**
  `yoloe-11s` reads **90% at 3 FP, equal to OWLv2, in 170 ms against
  2679** (15.8x), and `yoloe-26l` beats it outright at **91% / 97%** in
  398 ms. Ten YOLOE checkpoints land in an 85-91% band at 117-526 ms with
  well-separated gates (0.32-0.62, against YOLO-World v2-l's knife-edge
  0.026). **So the detector stops being the bottleneck and P7b's untouched
  finding becomes it: the Orin spends 36 ms detecting and 229 ms resizing
  a photograph.** The next latency work is P7b's preprocessing fixes and
  P7c's odometry, not another model. Detector pick: `yoloe-26l` for
  accuracy, `yoloe-11s` for latency -- decide after preprocessing, when
  latency is measured rather than projected. **OWLv2 is no longer the
  reason for the board**; it is one candidate and the slowest by 6-16x,
  and the Jetson's justification is that it runs whatever wins plus a
  depth model plus a local VLM. Note both sweeps came back NON-monotonic
  in model size (P21, P22), which on 195 visible frames is noise -- quote
  the band, not the ordering.

P21's confound still matters, but for a different reason than it was
recorded for -- not because it affects a purchase, which is now settled,
but because it means **the cheap detectors were never fairly measured
against OWLv2**, and one of them (YOLOE) beats it at a seventh of the
latency. That is a Jetson question.

**Superseded, kept for the reasoning -- REVERSED 2026-09-17 to the
JETSON:** This file carried "no Jetson, on cost" from
2026-09-13, and every clause of that argument has since failed:

* **The cost gap was wrong.** It mixed 3.6's ESTIMATED Pi prices with
  VERIFIED Jetson ones -- 3.6 has a Pi 5 at $80 against a real $175.
  Priced consistently (`BOM-COMPARISON.md`) the delta is **$59-86**, not
  the ~$261 implied. 4.7 set ~$170 as the re-opening threshold.
* **The Hailo-10H middle option is gone**: $200-224 verified, not the
  ~$130 assumed, and it cannot run OWLv2 (P17), so it scores like the 8L
  while costing more than the Jetson.
* **P19 settled the question the whole thing turned on.** OWLv2 run as a
  crop source INSIDE the tier reads **83% against the tier's 50%** on
  identical frames, mask and gate -- seven walks better, one tied, none
  worse. OWLv2 compiles to no Hailo (P17), so the Jetson is the only
  route to it.

**The part is a Jetson Orin Nano Super, ~$944 all-in** (`JETSON-BOM.md`).
The open question is LATENCY on the board, not price or accuracy: ~5 Hz
projected against the Hailo path's 92 FPS. **Nothing is ordered.**

**P25 -- BUILT 2026-09-22, default OFF, and the A/B could not be run.**
`brain/goal_pose.py` + the tier's new rung (`steer or reckoned or held or
SCAN`) are in. But the comparison that would earn the default flip **has
nowhere to run**: the sim turns in **90-degree quanta against a 10-degree
centre band**, so a target off a cardinal direction can never be centred --
every turn overshoots and flips the error's sign, and both arms alternated
LEFT/RIGHT and closed zero distance. **So the flicker has a SECOND cause
that dead-reckoning does not touch: a discrete action space cannot track a
continuous bearing.** That makes **C2 (continuous pose in the sim) a
blocker**, not a tidy-up -- `grid_world.py`'s cardinal `Heading` is the last
discrete thing in the stack and `sim/renderer.py` already takes a float pose
in radians. Also note `TeleopRobot` has NO odometry, so this feature is
inert on any phone walk; `MockRobot` is the only backend that can exercise
it.

**C2 IS NO LONGER A BLOCKER -- it was built as R0 on 2026-09-25**
(`PLAN-ros-alignment.md`, section 3.1, not deployed). `sim/grid_world.py`
holds float `x`/`y`/`theta`, `sim/mock_robot.py` integrates left/right wheel
angular velocities with the chassis constants from `HARDWARE-BOM.md` 4.3,
collision is a ray, and `turn_left(45)` now turns 45 degrees instead of
rounding up to 90. **So P25's A/B has somewhere to run**, and running it is
R1 -- which also owes `control/walk_eval.py` the median-run-length and
reversal metrics that P25 measured by hand. The paragraph above stands as
written; this is the correction to its last two sentences, not to its
finding. `TeleopRobot` still has no odometry, so the feature is still inert
on a phone walk and `MockRobot` is still the only backend that can exercise
it.

**P25 (2026-09-21, free) -- THE COMMAND CHANGES EVERY FRAME, and this now
outranks the model work.** Spotted by the operator watching a rig walk, and
measured on six: the **median run of a single command is 1.0 frames** on
three of them -- 24 changes across 65 frames with **11 immediate
reversals** (FORWARD, LEFT, FORWARD) on one. `control/walk_eval.py` already
raises `unstable-identity` on three of five; nobody had read it as a
headline. The guards do not cover it: `tier_consecutive_frames` gates cloud
TRIGGERS not the action, Phase G's hold is what P7e watched drive into a
basket, and the safety collar has no opinion about a FORWARD that merely
reverses the last one. **The fix is P7c item 2 and is still unbuilt** -- a
detection becomes a goal pose in the ODOM frame and the bearing is
recomputed from encoders and IMU at 30 Hz, so re-detection corrects drift
rather than supplying the answer. P7c derived it from latency; P25 arrives
at the same repair from stability, which is why it should now be built.
**P20-P24 moved recall ~50% -> ~90% and latency 2679ms -> 139ms and touched
none of this**; a better detector answering afresh every frame just
flickers better-founded, and 1.14's continuous motion turns that into
weaving. Build order: odometry in the SIM first (MockRobot has it,
`teleop_robot` cannot), a median-run/reversal metric in `walk_eval.py`, then
a rig walk showing the run length rise.

**P24 (2026-09-20, $2.90 on a rented A10G, torn down) -- the shipped tier
is SETTLED.** `brain/perceive.py` now defaults to **`yoloe-11s-seg.pt`,
`max_crops` 16, gate 0.8 unchanged, NO floor mask** -- two models, 139 ms on
laptop CPU. Every YOLOE checkpoint was run with and without the mask on all
1234 labelled frames. Three answers. **The mask loses on 9 of 11 variants**
and helps only the two weakest (`26m` +2, `26x` +4), costing the shipped
model 15 points and 836 ms against 139 -- so P16's "the mask rescues a
destroyed detector" is a FLOOR, never a contribution, and SegFormer stays out
of the tier. **"The 26 generation is worse" is FALSE** -- the generations
interleave and `26l` is the single best row at 16 FP (91%); what is true is
that `11s` leads at 3 FP by 7 points and is within 2 at 16 FP, and is the
cheapest of the leaders. **`max_crops` 4 -> 16 is worth 7 frames for 12 ms**
and is the first crop-budget number ever measured on the shipped detector.
One methodological correction that affects older phases: **P7's licence that
"a GPU changes nothing about what a detection is" does NOT generalise** --
it was established on OWLv2, a ViT, and YOLOE reads 251 true positives on an
A10G against 258 on the laptop. TF32 was tested and is NOT the cause; the
stack is (torch 2.7/ultralytics .156 against 2.14/.142). **Never merge GPU
and CPU rows** -- latency lineage stays on the laptop.

**P23 (2026-09-19, free) -- READ THIS BEFORE ANY RECALL NUMBER BELOW.**
Every row in P20, P21 and P22 was scored on **365 of the corpus's 1234
labelled frames** -- P19 subsampled to the frames its YOLO-World replay
covered and three phases inherited the set without re-examining it. S3
holds nothing extra (S3 and `recordings/` are identical); the frames were
always on disk. Re-run on all 1234, **the detector ranking INVERTS**:
`yoloe-11s` reads 82% at 3 FP against `yoloe-26l`'s 72%, and at the shipped
gate 80% against 76% with two false positives against five, at **139 ms
against 296**. **Corrected the same day**: paired on identical frames the two agree on 242-285 of 323 and their median scores are both 0.986, so the 82/72 gap is where each model's FP curve sits, not frame-level dominance. What survives for `11s` is **latency (2.1x), two false positives against five, and +12 frames at the shipped gate** -- not that the `26` generation is worse. Pair the frames before believing a gap. **So
`brain/perceive.py` now ships `yoloe-11s-seg.pt`** (`DEFAULT_YOLOE`), and
promoting on the subsample would have shipped the third-best model at twice
the latency. Two consequences: the gate needs NO change (`11s` reads 80% at
the shipped 0.8 against its own best 82%, where `26l` needed ~0.4), so this
is one constant and no extra paid cloud calls; and `26x` is dominated at
every budget, with P22's "6 points worse" revealed as noise. **1512
unlabelled frames remain** across 11 walks -- the cheapest corpus growth
there is, and corpus size is visibly deciding conclusions.

**P21 (2026-09-19, free) CONFOUNDS the row the reversal rests on, and
found a better model than either candidate.** P19's 83%-vs-50% compared
OWLv2 against a *replayed* YOLO-World detection file carrying ~45
proposals per frame; the live model emits ~1-4 at any threshold, and the
same tier config live reads **83%, not 50%** -- so the accuracy gap is ~7
points, not 33, and `BOM-COMPARISON.md`'s $59-86 premium was argued on the
33. **Root cause found the same day: `tools/hailo/quantized_detect.py`
applies NO NMS** -- it keeps every anchor above threshold out of 8400,
because YOLO's NMS lives in ultralytics' post-process and not in the ONNX
graph. The boxes are right and the post-processing is missing, and the
tier reads worse for having 45 redundant crops because they flood the
area-ranked budget. **So P16's phase-2 table is pre-NMS too** (50/49/49
with the mask, 20/13/3 without): its part ORDERING stands, its absolute
numbers understate all six rows, and its "mask rescues a wrecked
detector" mechanism now has a rival -- the mask may simply be supplying
clean regions where raw anchors supply none. Re-running those detections
through NMS is free and settles it. Separately, **YOLOE-26l reads 91% / 97% at 3 / 16 FP against OWLv2's
90% / 96%, at 398 ms against 2679** -- the best tier number measured here,
never named in any plan doc, and YOLO-shaped, so it may run on the $70
Hailo rather than only the $399 Jetson. **Do not order on P19.** Two free
next steps: regenerate `det_fp32.json` live and re-run P19; then P10's
compile loop on YOLOE with P13's INT8 trap in mind. Also note
`--confidence` never reached an open-vocabulary backend, so every such
threshold in these docs was measured at 0.02 (now `--detector-confidence`).

**P20 (2026-09-18, free) sharpened both sides of that.** OWLv2's real
operating point is **90% at 3 FP, not 83%** -- P19 read it at the `P>=0.8`
gate inherited from a YOLO-World tier, and at a matched FP budget it lands
on 0.594 and gains seven points. The tier is also **two models, not
three** (the floor mask is droppable) with **`max_crops` 16 rather than
8** (worth five points). And the latency is no longer confounded: measured
by subtraction, **the detector is ~85% of the tier** (CLIP ~370 ms of
~2800 on a laptop CPU), so P7b's ~205 ms fp16 Orin projection for OWLv2
alone is within ~15% of the whole tier -- **~235 ms, ~4.3 Hz**. SAM is now
validly measured and LOSES (85%, 4.5x slower); Grounding DINO loses as a
detector (80%) and wins at zero false positives, which makes it 1.11a's
corroborator rather than a crop source. See `PLAN-onboard-perception.md`
P20 and `evaluations/tier-decomp/`.

The 2026-09-13 reasoning is kept below because the Orin's power, camera
stack and thermals are still real costs, and because the shape of the
mistake matters -- an estimate compared against a verified figure. Any
text in
`PLAN-onboard-perception.md` 4.7/4.8/P7c/P7d that assumes an Orin is
recorded but not actionable; its "DECISION 2026-09-13" section is the
one that governs. The consequence to know before reading 4.11 or P7:
**the best model measured, OWLv2 at 82%, cannot run on the chosen
board** -- P6 proved it dies at allocation on 73 layernorm and 38
softmax layers, which also retires Grounding DINO, DINOv2 and SAM. The
on-board tier can only be a CNN proposer plus CLIP.

**That test has now been run and it PASSED (P10, 2026-09-13, $1.05):
YOLO-World compiles to a Hailo-8L** -- translate/optimize/compile all ok,
25.5 MB HEF, 4 contexts, cut at the six Conv end nodes the DFC's own
error recommends. That cut also keeps the vocabulary open at runtime (the
text einsum moves to the Pi CPU, as CLIP's text encoder already does), so
the reactive tier is worth **72%, not 45%**, on a $70 part. **INT8 DOES NOT PRESERVE IT, at any optimization level (P13,
2026-09-14) -- so the reactive tier on a Hailo-8L is NOT P9's 72%.** The
best of five configurations reaches **5% where fp32 reaches 34%**; QAT
(level 2, needs a GPU) is worth 55x the detections over level 1 and
16-bit embedding convs roughly double it again, and the two compose, but
the ceiling is not usable. The rig is verified exact -- Hailo native
emulation reproduces onnxruntime at corr +1.0000 -- so this is the model
meeting INT8, not the harness. **The cheapest decisive test left, and the
one to run before ordering: does YOLO11s + CLIP survive INT8?** It is the
shipped pipeline and a Hailo-native model; its fp32 number is 45%. Two
traps that silently produce a wrong answer are recorded in P13.

Superseded, kept for the shape of the mistake: P12 and With the rig now verified exact -- Hailo native
emulation reproduces onnxruntime at corr +1.0000 -- quantized activations
correlate only 0.67-0.82 with fp32 and the detector's usable output
collapses (29,331 detections -> 83). Neither 16-bit promotion of the
embedding convs nor level-1 Bias Correction recovers it; Bias Correction
made it worse. **AdaRound and QAT (levels 2+) are untested and need a
GPU, ~$1.50** -- and QAT is the standard answer for exactly this, because
the head is a cosine similarity and depends on the DIRECTION of a 512-d
vector. So the reactive tier is still 45% or 72% and that run decides it.
An earlier attempt was WITHDRAWN (P11). The DFC silently dropped to optimization level 0,
because the calibration set was 128 frames where it wants 1024 and the
instance had no GPU, so bias correction, AdaRound and QAT were all
skipped. That measured the crudest possible quantization, not INT8.
**Do not quote "INT8 destroys YOLO-World".** The corrected run needs
1024+ calibration frames (the corpus has 1234) on a GPU instance, ~$1.50.
P7d's "INT8 destroys OWLv2" is a separate, properly-run result and stands.

**The 10H is NOT blocked -- P14 measured our own installation, and P15
overturned it (2026-09-15, $1.30).** P14 had concluded DFC 5.x "cannot
parse real models". Run inside Hailo's own AI Software Suite container,
carrying **the same DFC 5.4.0**, `hailomz parse` succeeds on yolov11s,
yolov8s and yolo_world_v2s for `hailo10h` -- and on OUR opset-13 export,
and on **P14's exact pre-cut graph through the raw `ClientRunner` API it
used**. Six candidates are eliminated: the compiler, the 10H, the model
family, YOLO11's C2PSA attention `Split`, our opset, our end-node cut.
The only variable left is the environment we assembled around the wheel,
which makes P14 the same failure P11 was -- an installation reported as a
property of a part. **Use the vendor container** (`tools/hailo/zoo_probe.sh`);
the exact root cause inside our own host is NOT established, and the
obvious theory (a missing ONNX simplifier) was tested and is false.
`hailo10h` is `parse`'s DEFAULT arch and 224 of the zoo's 233 networks
declare it. NOTE the two compiler lines are still DISJOINT: 3.34.0
rejects `hailo10h`, 5.4.0 rejects `hailo8l`, so both wheels stay in S3
and `ec2.sh` picks by arch. And 5.4.0 needs Python **3.10 only** (it
requires torch==2.9.1); the v5.1/5.2 docs saying 3.8/3.9/3.10 are stale.

**MEASURED 2026-09-16 (P16), and it moves the question off the
accelerator entirely.** YOLO-World compiles to a 10H (11.9 MB HEF, 5
contexts) and the 10H PRESERVES it: 22% detector recall against the 8L's
5% (fp32 34%), and 99%/61% proposal agreement at 0.50/0.05 confidence
against 69%/23%. But replayed through the REAL pipeline and scored by
`control/perception_eval.py`, **the tier does not care**: YOLO-World +
floor mask + CLIP reads 50% / 49% / 49% for fp32 / 10H / 8L. Turn the
floor mask OFF and it is 20% / 13% / 3% -- the detector ordering exactly.
**The floor mask is the load-bearing crop source and it fully rescues a
wrecked detector**, so the 45%-vs-72% gap is worth about one point of
recall in the shipped configuration. **Bounded 2026-09-18 by P20: that is
a FLOOR, not a contribution.** With OWLv2's boxes the tier reads 90% at 3
FP with the mask and 90% without (96% vs 93% at 16 FP -- slightly *worse*
with it), so the mask is worth one true positive and is DROPPED. It is
insurance against a bad detector, and OWLv2 is the alternative to having
one.

**REVERSED 2026-09-17 by P19 -- read this first.** Run as a CROP SOURCE
inside the real tier (`crops:owlv2`, identical frames / floor mask /
48-crop budget / `P>=0.8` gate), **OWLv2 reads 83% against the tier's
50%** -- 162 true positives of 195 against 98, seven walks better and one
tied. 83% at 3 FP also reproduces P7's standalone 82% at 3 FP, so it
loses nothing by being placed in the pipeline where the YOLO-World tier
gives most of its detector away. **P17 says OWLv2 compiles to no Hailo**,
and `BOM-COMPARISON.md` prices the Jetson at **+$59-86**, not the
+$220-300 this file briefly carried. 4.7's re-opening threshold was $170.
**So the Jetson is now the defensible buy**, and it restores the NVMe the
purchasable Pi build cannot have. The open question is LATENCY, not
accuracy: P7b's honest fp16 projection is ~205 ms/frame (4.9 Hz) against
the 8L's 92 FPS, and 1.14's continuous motion assumed the fast one.

**Two consequences. The first is ANSWERED (P18, 2026-09-17): SegFormer
compiles to BOTH parts** -- hailo10h 6.4 MB / 13 contexts, hailo8l 20.0 MB
-- **and INT8 barely touches it**: floor-mask IoU 0.988 mean / 0.995
median against fp32 over 120 frames, 0 frames below 0.5, at optimization
level **0** (every accuracy pass skipped, so a LOWER bound). The mask that
carries the tier is therefore real on hardware, and **the 10H is NOT
mandatory** -- the 8L runs the whole tier. It remains a headroom purchase
at +$60. ~~Still open: the mask is a third model against the Pi's four
cores (handoff open item 1, now first-order).~~ **CLOSED 2026-09-18 by
P20 -- by deletion: the mask earns nothing beside OWLv2, so the on-board
tier is two models (detector + CLIP), and no SegFormer HEF is needed. The
IoU 0.988 result below stands as a measurement and is off the critical
path.** And the tier has a real defect: at the shipped
`max_crops` of 8 the SAME data gives the 8L 30% against fp32's 25%,
because crops are ranked by AREA and more proposals crowd a small target
out of a fixed cap -- **a better detector can make the tier worse.**

**The Hailo-10H -- DROPPED 2026-09-17 on verified pricing.** It was
carried here at ~$130; real listings are **$200 for the in-stock AI HAT+ 2
(8GB)** and **$212.50 for the 2242 M.2 module (4GB, backordered to
October + 4 weeks)**. That puts a Pi + 10H build at **~$992-1,018 all-in
against the Jetson's ~$944** -- and P17 measured OWLv2 failing to compile
on the 10H, so it delivers the same ~50% tier as the $858 8L build. It is
on neither frontier: cheaper accuracy is the 8L, better accuracy is the
Jetson. The live comparison is two rows, **$858 for 50% or $944 for
83%**. Kept below for the reasoning, which stands. **DFC 3.34.0 cannot target it** ("Please use Dataflow
Compiler v5.x") -- but v5.x is downloaded now, along with the AI Software
Suite container and the 5.4.0 Model Zoo, all in
`s3://vision-picar-deploy-.../hailo/`. The gate was a login, and it is
open. The Hailo-8 buys
nothing: same dataflow architecture, same failure on attention.

**The question that test answered, kept for context: does
YOLO-World compile to a Hailo-8L HEF?** P9 (2026-09-13) measured
YOLO-World + CLIP at **72% recall / 99% precision** at the shipped
P>=0.8 gate against YOLO11s + CLIP's **45% / 94%**, on 11 walks / 1234
frames, at half the latency -- so the tier is worth 45% or 72% on that
one answer. `tools/hailo/` exists and cost $3.20 last time. **Run it
before ordering the accelerator.**

**Added 2026-09-13 (same day, later session), and the one thing to know
before reading any tiered walk's outcome: a walk ARRIVED and the system
did not notice.** `woven-laundry-basket-20260913-115703` frames 0204-0209
are the target at touching distance; the cloud had said `STOP` and it was
being held, local perception read P = 0.998, and Phase G's
steer-over-hold precedence drove FORWARD into it until `max_steps`. Two
composing defects -- `safest_direction` is an overloaded channel (a mode
change loses to a bearing), and `_held_direction()` drops everything but
the direction, so `target_reached` never survives to a free frame.
Written up as **`PLAN-onboard-perception.md` P7e**, which also records
why "make a held STOP un-overridable" is the wrong repair. **Deliberately
NOT built**: the fix resolves differently once a lidar exists, so it is
settled on hardware day. **Consequence until then -- every tiered walk
ends `max_steps` even when it physically arrives, and
`control/walk_eval.py`'s completion score (0.25 of the total) is
structurally zero for all of them. Do not read a tiered walk's outcome as
a navigation result.** **Corrected 2026-09-26 for the sim only**
(`PLAN-ros-alignment.md` 3.11): a tiered mission on `MockRobot` now ends
`found` when it arrives, because the arrival rule reads the lidar scan. On
a phone walk there is no scan, the rule refuses to judge, and this
paragraph still holds until the car has a lidar.

Two defects found alongside it WERE fixed, because both are about being
able to read the record later. `_tier.cloud_called` was False on every
frame of every async mission since Phase A (the dispatch branch returns
the stand-in, which hardcoded it), so no recorded walk could say which
frame the cloud was shown -- the twin's counter was always right, it
reads `stats.cloud_calls`. And `tests/test_serverless_routes.py` was
blind to every route mounted via `include_router` on FastAPI 0.141, which
made the guard against silently-404ing routes pass vacuously. **Note the
repo is currently run under two Pythons with two FastAPI versions
(`.venv` 0.141, system Anaconda 0.136) and they disagreed about the
suite** -- `pytest` from `.venv` is the one to trust.

---

## From section 5, Stage 0: what the premise gate showed (2026-08-30 to 2026-09-08)

Measured on the 39-walk corpus that was DELETED on 2026-09-07 as invalid by
its own viewpoint. Kept for the arguments; none of the numbers can be re-run.

**The corpus every finding below was measured on was DELETED on
2026-09-07** -- all 39 walks, from S3 and from the local backup, on purpose.
They were invalid by their own viewpoint (standing height, target on
furniture), so their numbers were suspect regardless, and an invalid corpus
sitting in the bucket is how it gets scored against by accident. **Treat
every number in this section as a recorded observation that can no longer be
re-run**, and re-derive anything you intend to rely on from the new corpus,
which is currently one walk: `blue-bottle-20260907-142454`. The findings are
kept because the arguments they support are still the best available -- see
`PLAN-onboard-perception.md` 4.10, which names the two claims that are now
assertions rather than measurements.

**What the gate has actually shown, as of 2026-08-30** -- run it yourself
before trusting any of it, but this is where it stands:

- Walks now reach the target, which they never did before. Claude Opus 4.5
  and Qwen3-VL both arrive in 6-14 frames; Sonnet 4.5 stalls (one FORWARD in
  22 frames, turning on the spot with the target centred) and Nova Lite
  wanders without arriving.
- **The failure is obstacle routing, not object recognition.** Every model
  identifies a red backpack; the ones that fail refuse to close distance over
  open floor because "is there an obstacle directly ahead" reads as "is there
  furniture anywhere in front of me".
- **`obstacle_ahead` is not calibrated and should not be trusted.** On the
  same frames Opus reports it on ~100% and Qwen on ~0%. On a frame that is
  nothing but a wall, Opus and Sonnet turn away; Qwen and Nova drive into it.
  On hardware the lidar is the real obstacle sensor -- do not let the
  vision policy be the thing relying on this field.
- **The prompt is at least as strong a lever as the model, and the obvious
  fix is wrong.** Rewording the obstacle question to be about the next step
  takes Sonnet from 0% FORWARD to 100% FORWARD on the same frames -- which is
  the *other* degenerate failure. Three wordings now ship (`default`,
  `next-step-obstacle`, `next-step-and-walls`).
- **The full 3x3 was finally run on 2026-08-30, and the third wording does
  not work.** All 22 frames of `red-backpack-20260829-195904`, every model x
  every variant, replayed at full coverage. FORWARD rate:

  |                   | default | next-step-obstacle | next-step-and-walls |
  |---|---|---|---|
  | Claude Opus 4.5   | 0.591 | 0.318 | 0.455 |
  | Claude Sonnet 4.5 | 0.000 | 1.000 | 0.955 |
  | Qwen3-VL          | 0.773 | 1.000 | 1.000 |

  `next-step-and-walls` was written to keep the next-step framing while
  restoring "a surface filling the frame is a stop condition". It buys one
  frame in 22 on Sonnet and nothing on Qwen -- still the always-FORWARD
  degenerate mode, still colliding. **Do not promote it.** `default` is the
  only column that avoids that mode on all three models, which is the
  evidence for leaving it as the default. No cell reached the target, and
  every cell except Sonnet/`default` was flagged `collision`. Next attempt
  should change the *shape* of the question -- the single-step /navigate
  contract has no memory of which way it already turned -- not its wording.
- **These nine numbers replaced nine that were wrong, and the way they were
  wrong is the cautionary tale.** The same matrix had been run before and
  reported 33/33/33 across the variants, which reads as "the wording makes
  no difference". It was really "the vision service timed out": those cells
  completed 3, 9 and 2 of 22 frames. `control/admin_server.py`'s retry
  branched on a status code while httpx *raises* a timeout, so the backoff
  never ran on the failure that dominated, and the scorer graded whatever
  came back. Fixed 2026-08-30 (`replay_timeout_s`, a retry that catches
  `httpx.TransportError`, and `REPLAY_MIN_COVERAGE`, below which a replay is
  stored but deliberately left unscored). **A replay's `coverage` field is
  now the first thing to read**: a score computed over a third of a walk is
  not a weaker measurement, it is a different one.

- **An ordinal distance question was tried and does not yet work
  (2026-08-31, Lab only).** `/navigate`'s `default-with-distance` variant
  asks how many robot moves of clearance there are ahead --
  `within_one_step` / `a_few_steps` / `far` -- ordinal rather than metric,
  because a single monocular frame cannot give metric depth. Replayed over
  80 frames from five recorded walks (two targets, 100% coverage) against
  Lab's vision service: **`within_one_step` 60%, `far` 5%**. Someone
  walking across a house does not spend three frames in five one step from
  a collision. It is the same over-reading `obstacle_ahead` already shows,
  and it is not the target being miscounted -- the skew holds at 55% on
  the frames where the target is not visible at all. The two fields agree
  with each other 85% of the time, so they are wrong together rather than
  independently noisy. `brain/agent.py`'s proximity veto exists but stays
  **off by default**: wired on, this would block three FORWARDs in five
  and reproduce the never-FORWARD stall. Next attempt should change the
  question's shape -- what is in the centre third and in the path, not
  what is nearest anywhere in frame.

- **A fifth wording, `bearing-only`, deletes the obstacle question instead
  of rewording it. MEASURED 2026-09-02, and it is the first wording that is
  degenerate on no model.** Phase M1 of
  `PLAN-microduck-transplants.md`. The argument is that a single monocular
  frame does not contain metric depth, so no wording recovers it: `/navigate`
  keeps *what* and *which way*, and a distance sensor owns *how far*. Two
  lines are removed from `default` and nothing else -- the question and its
  schema line -- so `obstacle_ahead` is **absent from the reply**, not false.
  `brain/navigate.py` maps that absence to `free_space: "unknown"`, and the
  only obstacle logic left on the path is `robot/safety.py`'s
  `get_distance()` re-check before every FORWARD. Under a replay
  (`ReplayRobot` has no sensor) `control/walk_eval.py` will therefore flag
  the run `collision`: **record that column as "what the sensor must catch",
  not as a defect** -- it is M10's specification.

- **The full 5x3, at 100% coverage, replayed 2026-09-02** (same 22 frames of
  `red-backpack-20260829-195904`, every model x every wording). FORWARD rate:

  |                   | default | next-step-obstacle | next-step-and-walls | center-third-path | bearing-only |
  |---|---|---|---|---|---|
  | Claude Opus 4.5   | 0.591 | 0.318 | 0.455 | 0.591 | **0.591** |
  | Claude Sonnet 4.5 | 0.000 | 1.000 | 0.955 | 0.091 | **0.591** |
  | Qwen3-VL          | 0.773 | 1.000 | 1.000 | 0.818 | **0.773** |

  Three things this settles.

  **`bearing-only` is the only column with no degenerate cell.** No `stalled`
  flag, no `degenerate` flag, all three models inside 0.591-0.773. Sonnet's
  never-FORWARD stall -- the failure that started this whole investigation --
  is gone without flipping to the always-FORWARD failure that every previous
  attempt traded it for. Deleting the question did what five rewordings of it
  could not. **This is evidence about the degenerate modes, not about
  navigation:** no cell in the table reaches the target, and every
  `bearing-only` cell is still flagged `collision` -- 12 of 12 checked
  FORWARDs on all three models. That column is M10's specification, not a
  defect: `ReplayRobot` has no distance sensor, and the whole argument is
  that the sensor is what refuses those moves.

  **Question 5 was what broke `next-step-obstacle`, not the region change.**
  `center-third-path` moves question 2 alone and takes Sonnet from 0.000 to
  0.091; `next-step-obstacle` moved questions 2 and 5 together and took it to
  1.000. That attribution is exactly what the variant was built for, and it
  cost one replay to get. Neither is worth promoting -- 0.091 is still the
  stall.

  **Qwen was never answering the question anyway.** Its `obstacle_rate` is
  0.000 under `default`, and its `bearing-only` numbers are identical to its
  `default` ones in every field -- same FORWARD rate, same 12/12 collisions,
  same score, same agreement. Removing a question a model was already
  ignoring changes nothing, which is the cleanest possible confirmation that
  `obstacle_ahead` is uncalibrated rather than merely noisy.

- **The closed-loop sim run was made, and it does NOT settle the gate
  question -- the renderer is the blocker (2026-09-02).** Stage 1's "done
  when" ran for real: `tests/demo_sim_mission.py`, `policy: "vision"`, the
  grid world, the deployed `/navigate`, `sim.sensor_noise.enabled: true`,
  Opus 4.5, 40 paid steps each.

  | wording | steps | wall clock | ended | distance to target | outcome |
  |---|---|---|---|---|---|
  | `default` | 40 | 142.2s (3.6s/step) | (2,1) | 14 cells | `max_steps` |
  | `bearing-only` | 40 | 146.1s (3.7s/step) | (2,2) -- never moved | 13 cells | `max_steps` |

  Neither reached the target; neither ever claimed to. Action spread was 31
  RIGHT / 6 STOP / 2 FORWARD / 1 LEFT and 39 RIGHT / 2 FORWARD. **But the
  reason was not the policy.** Nearly every decision's reasoning said some
  version of "the image is very dark and unclear" or "a blank gray wall", and
  the model was right: the render painted its ceiling and floor with the
  twin's `--wall` / `--floor` CSS variables -- two near-blacks meant for dark
  UI chrome -- so a room came back as a black void with two grey slabs. The
  policy spun looking for a view it never got.

  **Both renderers now carry their own lit ceiling and floor** and no longer
  read the app's theme, so restyling the twin cannot change what the model
  sees (`sim/renderer.py`, `renderFPV` in `web-twin/app.js`, golden
  re-blessed, browser parity test still green). **That fix is unmeasured:**
  it makes the frames legible to a human eye, and whether a closed-loop run
  can now discriminate between wordings is the next paid run's question.
  Until then the replay table above is the instrument for anything about the
  *seeing*, and a sim run measures the *loop* -- cost, wall clock, budgets,
  the veto. One half of the diagnosis is still open and is a map question,
  not a renderer one: the starter house's start pose faces a near wall, so
  even a lit first frame shows little of the room.

  Two things the runs did prove, which no replay can. The **safety collar is
  live and fired** (1 veto on `default`, 2 on `bearing-only`) -- the only
  obstacle logic left under `bearing-only`, exactly as designed. And the
  brain-side `min_distance_cm` was **30.0, exactly one grid cell**, so with
  S5's 3cm jitter the veto was near a coin flip at one cell of clearance
  (`28.0`, `28.9`, `27.7` all blocked live; a test written for it measures
  90/200). **Now 20.0**, matching `safety.min_distance_cm` that
  `robot/server.py` has always used, so the brain-side pre-check and the
  robot-side veto agree on one number. Against the noiseless sensor this
  changes nothing -- an exact reading is only ever a multiple of 30, so any
  threshold in (0, 30] blocks exactly the one case that matters -- which is
  why 30.0 sat there unremarked until noise was switched on.

- **That shape change was built as `center-third-path` and measured on
  2026-09-02. It does not work, and the corpus it was measured on turned
  out to be invalid. Both halves matter.**

  It asks what is in the bottom half of the centre third -- the ground the
  next step crosses -- as `open_floor` / `blocked` / `unclear`, with
  `obstacle_ahead` defined as a restatement of it. Eight walks, 96 frames,
  two targets, replayed against `default` under two models, every cell at
  coverage 1.0. Frame-weighted FORWARD rate:

  |                   | default | center-third-path |
  |---|---|---|
  | Claude Opus 4.5   | 0.323 | 0.365 |
  | Qwen3-VL          | 0.354 | 0.573 |

  The collision flag did not move at all -- 6/8 walks for Opus, 7/8 for
  Qwen, under both wordings -- and the models moved *apart* rather than
  together. **The per-frame field explains why, and it is worth knowing:**
  Opus answers `blocked` on 66%, Qwen on 5%, agreement 40% against a chance
  rate of 35% (kappa 0.075). But the disagreement is perfectly **nested**:
  all 32 frames Opus called `open_floor`, Qwen called `open_floor` too, and
  of the 63 it called `blocked` Qwen called 58 of them open. Neither model
  ever contradicts the other's ordering. They read the picture the same way
  and cut the threshold in different places -- and **a threshold has no
  wording**, which is why four rewordings have now failed and a fifth
  should not be written. The instruction itself was followed exactly:
  `obstacle_ahead` restates `path_ahead` on 99%/100% of frames.

- **The Stage 0 corpus does not test what it claims to, and every number
  above and below inherits the problem (found 2026-09-02, by reading the
  frames instead of the JSON).** In every walk sampled, across both
  targets:

  1. **The target is on raised furniture.** The red backpack sits on an
     ottoman; the blue bottle sits on a console table. The car is a floor
     robot. It cannot arrive at either, so `target_reached` is not merely
     rare in these walks -- it is unachievable, and the collision flags are
     *correct*: the only way to approach the target is to drive into the
     furniture holding it. Opus says so in its own reasoning, answering
     `blocked` and then FORWARD "since reaching the backpack requires
     moving toward the couch". That is not incoherence. It is a real
     dilemma handed to it by an impossible task.
  2. **The camera is at standing height, not 10cm.** The frames look *down*
     onto a ~45cm ottoman and a ~75cm table. Stage 0's own instructions
     above say to hold the phone at ~10cm and that "a chest-height view is
     not the robot's view" -- the recordings did not follow it. From 10cm
     that ottoman is a wall, and none of these scenes resolve the same way.

  **Consequence: the four-wording failure is not established as a prompt
  problem or a policy-shape problem.** It was measured on a task the robot
  cannot perform, from a viewpoint it will never have. The 3x3 matrix, the
  `distance_estimate` skew and the table above are all reproducible and all
  suspect for the same reason. **Nothing further should be spent on prompt
  wording until the corpus is re-recorded:** a target on the floor, the
  camera at robot height on a wheeled rig (see the Stage 0 rig note above --
  do not hold the phone, and the "10cm" figure is a retired PiCar-X number),
  both rooms, several walks. That is a phone and twenty minutes, and it is
  the cheapest high-value item left in Stage 0.

- **Five live walks on 2026-09-02 were evaluated on 2026-09-03. None of them
  is usable as evidence about wording, and the reasons are worth more than
  the numbers.** All Opus 4.5, target "Bottle", all scored `poor`, none
  reached:

  | time | wording | n | FWD | obstacle | target visible | vis-flips | score | judge |
  |---|---|---|---|---|---|---|---|---|
  | 16:35:29 | default | 19 | 0.00 | 1.00 | 0.21 | 5 | 24 | 0.12 |
  | 16:36:32 | default | 21 | 0.00 | 0.95 | 0.62 | 9 | 20 | 0.12 |
  | 16:37:47 | `bearing-only` | 33 | **0.03** | -- | 0.12 | 3 | 18 | 0.14 |
  | 16:39:23 | `center-third-path` (+1 stray) | 17 | 0.18 | 0.65 | 0.59 | 3 | 40 | 0.50 |
  | 16:40:17 | default | 47 | 0.17 | 0.96 | 0.43 | 9 | 26 | 0.12 |

  **`bearing-only` went degenerate live -- 30 of 33 frames RIGHT -- which
  looks like a flat contradiction of the replay table and is not one.** Read
  the reasoning: every spin says "no bottle is visible... turning right to
  scan more of the room". It found the bottle at frame 8 (FORWARD, "centre
  third, still across the room"), turned LEFT twice to centre on it,
  overshot, lost it at frame 11 and resumed spinning. **That is a search and
  memory failure, not an obstacle failure** -- deleting the obstacle
  question cannot help a model that has lost the target and has no record of
  which way it already turned. It is the first live evidence for M12, and it
  says nothing about M1's reading either way.

  **The corpus defects recorded above are fully reproduced.** The frames were
  opened, not just the JSON: the camera is at standing height looking *down*
  onto furniture, and the bottle is on a round cafe table (~75cm). A floor
  robot cannot arrive at it, so `collision` is again the correct flag and
  the walk again tests a task the robot cannot perform. Also 3 of 33 frames
  are portrait against 28 landscape -- the model says so itself ("blurry and
  rotated", "sideways image"). **The re-recording called for above is still
  the cheapest high-value item in Stage 0, and it has not been done.**

- **A recording bug found while reading those walks, and it undercuts
  attribution generally.** Walk `bottle-opus-4-5-center-third-path-20260902-163923`
  contains sixteen `center-third-path` frames and one `bearing-only` frame at
  `seq: 37` -- a `/navigate` call still in flight when the previous walk was
  stopped, written into the *next* walk's directory with its old wording and
  its old sequence number. Same orphaned-in-flight-call class the Robot-view
  HUD already fixed with `guidanceEpoch`, one layer down: the recorder needs
  the same epoch. **Until it is fixed, no walk recorded right after another
  in one session is safely attributable.** Fix is specified in M7b.

- **Decision, 2026-09-03: ship one wording.** `default` on the live path;
  all five kept in replay, where they are the only controlled comparison
  this project has. The argument is that the models' disagreement is a
  *threshold* and a threshold has no wording (see `center-third-path` above),
  that Opus 4.5 reads 0.591 under both `default` and `bearing-only` so the
  choice is already a no-op for the shipped model, and that each live variant
  is one more way for a walk to be unattributable -- three of which have now
  cost real measurements. `bearing-only` is deliberately **not** promoted:
  best-on-one-walk is how the `NavigateModelId` mistake happened. Phase M7b
  of `PLAN-microduck-transplants.md`, gated on M7.

- **The re-recording two bullets above call "the cheapest high-value item
  left in Stage 0" WAS done, on 2026-09-07.** Those bullets are dated
  findings and are left as written; this is the correction. The corpus is
  now **four valid rig walks** -- floor height, target on the floor, each
  with an adjudicated `labels.json` beside it -- and the 39 invalid walks
  were deleted the same day. Everything the new corpus settled is in
  `PLAN-onboard-perception.md` 4.10.

- **What to record next is a different question now, and it has an
  answer**: two out-of-vocabulary **searches** plus one control, driven
  under `policy: "tiered"` so 1.11a's corroboration verdict is measured
  live. `PLAN-onboard-perception.md`'s "What to record next, and why these
  walks" (2026-09-08) has the targets, the shape and the two COCO nouns
  that look out-of-vocabulary and are not. **This is now the cheapest
  high-value item left**, and unlike the last one it is not about the
  viewpoint -- it is the falsifier for an amendment that is otherwise
  going to be decided on one walk.
