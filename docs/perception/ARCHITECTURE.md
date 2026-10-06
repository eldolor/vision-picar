---
kind: architecture
domain: perception
status: current
verified: 2026-10-02
---

# Perception -- architecture

The on-board perception tier answers one question per camera frame, locally
and for free: **is the mission's target in this frame, and at what bearing?**
It is what lets the tiered policy call the cloud only on events, what gives
the robot a bearing to steer on, and what the arrival rule pairs with the
lidar. It does not decide moves ([policy](../policy/ARCHITECTURE.md)) and it
is not meant to confirm identity -- the cloud model is, at arrival too (see
"Detect, then verify"). Read this for why the tier
is a detector plus a verifier with a three-state answer; read the
[engineering spec](../engineering/perception/ENGINEERING.md) for the shipped
models, thresholds, commands and recorded numbers.

## Purpose

A cloud vision call per frame costs money and seconds; a robot that only
sees through the cloud cannot steer between calls. Perception supplies
the cheap, fast, per-frame half of the split the project settled in
`PLAN-onboard-perception.md` 1.11: **local perception says *where*, the cloud
says *what*.** Its dependants are the tiered policy (triggers and steering),
the arrival rule (a centred detection), the twin's readouts (state,
probability, model names), and every perception finding in the plan, which is
measured through the same pipeline the robot runs.

## Components and boundaries

```text
   frame (JPEG, pan angle)                  sim frame (reported detections)
        |                                              |
        v                                              v
  +-- perception pipeline ------------------+   +-- frame-reported pipeline --+
  | detector  -> region proposals           |   | read the detections the sim |
  | crop rule -> which proposals to verify  |   | computed from its geometry, |
  | verifier  -> P(target | crop, texts)    |   | occlusion-aware; marked     |
  | gate      -> detected / absent /        |   | "synthesised"               |
  |              unavailable, with bearing  |   +-----------------------------+
  +-----------------------------------------+               |
        |                                                   |
        +-------------- one result shape -------------------+
                               |
                               v
                     tiered policy, arrival rule, status

  measuring instruments (off the mission path):
    corpus scorer  - every labelled walk, against human labels
    target probe   - does a target string fire on unrelated frames?
    latency bench  - per-frame GPU/CPU split on pinned frames
    lab backends   - candidate models behind the same seams
```

| Part | Owns | Must not |
|---|---|---|
| Detector seam | turning image bytes into region proposals | decide identity |
| Verifier seam | scoring a crop against the target string and a fixed set of distractors | be read as a probability of presence on its own (only the softmax over target + distractors is) |
| Region-proposer seam | class-agnostic regions (a floor mask today; off) | replace the detector's boxes -- only add to them |
| Pipeline | crop rule, gate, tri-state, bearing | load any model at import time; run on a raycaster render |
| Frame-reported pipeline | the sim's stand-in, read from frame data | compute anything; claim a model ran |
| Lab backends | candidates on equal footing (open-vocabulary detectors, a small VLM, SAM) | be imported by the shipped pipeline |
| Corpus scorer, target probe, latency bench | measurement | feed the robot; the recordings domain owns the corpus itself |

**Boundaries.** The seams are protocols, so a fake drives every test and a
different runtime (a TensorRT engine, say) can replace a backend without the
pipeline changing. The model libraries are an optional install, imported
lazily inside the concrete backends; nothing in the automated suite needs
them.

## Decisions

### Perception runs in the brain process

**Decision.** The pipeline is built and run inside the brain, in the same
process as the mission loop, called once per frame by the tiered policy.
That is why a tiered mission needs the model libraries wherever the brain
runs, and why the brain runs on a machine that can hold the models (a laptop
today, the board later) rather than in the cloud deployment.

**Rejected, for now:** perception as its own process that publishes
features, not frames (`PLAN-onboard-perception.md` 2.7). That would let a
stalled detector degrade perception instead of stalling the control loop,
but it adds a second service, a second wire format and a second liveness
signal before any board measurement says the detector stalls.
**Trade-off.** A hung model call hangs the tick. The mission's per-call
vision timeout and the hung-tick dead-man ([mission](../mission/ARCHITECTURE.md))
are what catch it, and they end the mission rather than degrade it. Kept as
an open question below.

### Detect, then verify

**Decision.** A detector proposes regions; an image-text verifier scores
each crop against the target string and a set of bland distractors; a gate on
the resulting probability decides. The verifier raises a candidate and never
confirms one -- identity belongs to the cloud (1.11). Which models fill the
two seams is an engineering choice, measured and recorded in the
[engineering spec](../engineering/perception/ENGINEERING.md).

**At arrival too.** "Never confirms" holds for the cloud policy's `found`,
and since 2026-10-02 for arrival as well. On the tiered path the arrival
rule ([policy](../policy/ARCHITECTURE.md)) judges distance from a local
detection centred at close lidar range; before that ends a mission `found`,
one paid cloud call on the arrival frame must agree the target is in it
(decided by the user 2026-10-02, built the same day). Until then a verifier
false positive that kept passing the gate could end a mission at the wrong
object. Corroboration (below) still measures the local tier's agreement and
is not enforced. The rule, its rejected
alternatives and its trade-off are the [policy](../policy/ARCHITECTURE.md)'s
("The cloud confirms identity at arrival; the lidar decides distance"). It
fits this domain's split: the cloud owns identity, and the local tier is
recall-first.

**Rejected.** A single open-vocabulary detector as the whole pipeline (OWLv2,
Grounding DINO, YOLO-World): measured as replacements in P7, P9 and P21, an
open-vocabulary model won as a *crop source* rather than as a classifier,
because the gain is its regions, not its own confidence. A small local VLM
(Qwen) was not run in P7 because its score is not separable into a gate and it
had a known upstream performance bug. **Rejected also:** a third model per
frame. The floor-mask proposer measured +8 points on an early detector, then
lost on 9 of 11 checkpoints with the shipped detector at six times the whole
tier's cost (P24), so the tier is two models (P20).

### A text-prompted detector, and the open-vocabulary crop rule

**Decision.** The detector takes the target string as its prompt, and every
proposal reaches the verifier regardless of label. The model family and
variant are tuning, chosen by corpus score and latency at a matched
false-positive budget. The current choice and its evidence are in the
engineering spec.

**Rejected.** 4.2's original rule -- use a closed-vocabulary COCO detector and
keep only crops labelled with the target's COCO word. On the first valid rig
walk it lost 11 of 18 true positives, because at close range the detector
relabels the object (a bottle becomes a vase), so the gate discarded exactly
the frames where the target filled the view. **Rejected also:** choosing a
model on a subsample of the corpus. A ranking that leads on a fraction of the
frames has inverted on the whole labelled set before (P23), so a model choice
is made on every labelled frame.

**Trade-off.** Recall is worth more than precision here by design: a miss
means driving past the target. The price is not small. A false positive that
passes the gate is a sighting: it may fire a paid cloud call, it steers the
robot on that frame (steering has no frame hysteresis, and the cloud's answer
does not override a local sighting), and near the wrong object it can satisfy
the arrival rule's distance half. It cannot end the mission `found` there:
the cloud check at arrival (see "At arrival too" above, and the failure-mode
table) refuses it. The rest stands.

### Gate on a probability, not a raw similarity

**Decision** (2026-09-07). The gate is P(target | crop, target + distractors),
a softmax over the verifier's scores. **Rejected:** a threshold on the raw
margin over the best distractor, because an image-text similarity is not comparable
across text queries -- no single margin served both rig walks (one that found
none of the bottle's sightings, and a lower one that gave the backpack walk
false positives); under the softmax one bar served both.

### Three states, never two

**Decision** (1.12, M3's argument one sensor over). Every frame is
**detected** (with a bearing), **absent** (the frame was good and the target
is not in it -- information), or **unavailable** (no frame, undecodable, a
model raised -- the absence of information). **Rejected:** collapsing
unavailable into absent, which makes a wedged camera read as "the target is
gone" and would let a stalled capture end a search.

### The simulator reports detections; no detector ever runs on a render

**Decision** (1.12, built at R1). In the sim, detections come from the
simulator's geometry -- in the field of view and at least partly visible,
reported at the bearing of the visible part and not at all when hidden (3.32)
-- and are marked synthesised. The sim tests the
detector's *consumers*, never the detector. **Rejected:** keeping the sim out
of the loop (no steering could be tested), and running real models on
raycaster frames or on textured sprites -- "the detector finds a sprite"
reads as success while measuring nothing. Accuracy claims come only from real
photographs.

**Consequence.** The brain chooses the sim stand-in by the frame's provenance
at mission start; if it cannot tell, it assumes a real camera, because loading
models that see nothing is cheaper than steering a real robot on detections
nobody measured.

### The board is a Jetson; the Hailo path is closed

**Decision** (2026-09-19, by the user). Perception runs on a Jetson Orin Nano
Super's GPU. **Rejected:** Pi 5 + Hailo-8L, chosen earlier on cost and power
(4.9, "DECISION 2026-09-13") -- the compile loop showed OWLv2 cannot be
allocated on the 8L (P6), and a dataflow accelerator makes compilability the
dominant model-selection criterion; also the IMX500 AI camera, which cannot be
fed a recorded frame (1.10). **Consequence.** The only axes left for a model
are recall and latency. Do not re-open the Hailo path or fund a compile run.

### A latency budget, measured on the board

**Commitment** (`PLAN-ros-alignment.md` 3.33, confirmed by the user
2026-10-02): **250 ms a frame at the board's 15 W mode.** Over it, the first
fix is moving image handling off the CPU (P26), then TensorRT. P26 is built
only if the board's own GPU/CPU split says the handling matters; its
acceptance bar is a plan criterion (`PLAN-onboard-perception.md` P26), not a
commitment of this spec. **Measured 2026-10-04:** the shipped pipeline is
well inside the budget on the board at 15 W (3.33), so P26 is not needed to
meet it.

### Measure against people, at a matched cost

**Decision** (P3). Perception is scored against each walk's adjudicated human
labels, never against the cloud model's own claims (on one search walk the
cloud claimed the target on 44 frames, 34 of them a storage bin). A run
records every frame's best score once and sweeps the gate afterwards; two
configurations are compared only at the same false-positive budget; GPU and
CPU rows are never merged (P24); the frame set is pinned before comparing.
**Rejected:** recall without its false positives -- that is how a
low-threshold open-vocabulary detector looks like a win.

### Corroboration and vocabulary are reported, not enforced

**Decision.** Two verdicts are computed every time they apply and published
on the twin and in recorded walks, and change no decision:

- **Corroboration** (1.11a, proposed 2026-09-07, not decided): does local
  perception support a sighting the cloud claimed, at a lower bar than
  detection? Measured live on three walks it was net negative -- it caught no
  false claim and suppressed one true, distant sighting.
- **Vocabulary**: whether the target string has a COCO word. It is evidence
  about the detector's vocabulary, never a capability claim (an in-vocabulary
  bottle was still lost at close range).

**Rejected:** enforcing either now. A target the local tier cannot see would
make every cloud sighting "unclear" and leave the robot steering forever
without committing. **Rejected also, on 2026-10-02, as the guard on a wrong
`found`:** enforcing corroboration at arrival. It is free, but measured
net-negative and unproven; the user chose a cloud identity check at arrival
instead ([policy](../policy/ARCHITECTURE.md)).

## Contracts

| With | Direction | Category | Ownership |
|---|---|---|---|
| Tiered policy ([policy](../policy/ARCHITECTURE.md)) | policy calls the pipeline once per frame | in-process: frame in, tri-state result out | perception owns what was seen; the policy owns what to do |
| Arrival rule (policy) | reads the result's bearing and pan | in-process, via the scene | arrival refuses a panned bearing; perception only reports it. Holds on real frames only: a sim frame carries no pan, so the sim stand-in always reports pan 0 with a camera-relative bearing (harmless while no tiered policy pans) |
| Mission service ([mission](../mission/ARCHITECTURE.md)) | builds the pipeline at mission start | in-process construction | a pipeline that cannot load is a start-time refusal, never a mid-mission vision failure |
| Body ([body](../body/ARCHITECTURE.md)) | supplies the frame and its pan angle | frame dict | the body owns pixels and servo angles; perception never asks for more |
| Simulator ([simulator](../simulator/ARCHITECTURE.md)) | supplies reported detections in sim frames | frame data | the simulator owns visibility geometry |
| Recordings ([recordings](../recordings/ARCHITECTURE.md)) | instruments read walks and human labels | files | recordings owns the corpus and its labels |
| Cloud vision ([cloud-vision](../cloud-vision/ARCHITECTURE.md)) | none directly | -- | the cloud owns identity; perception never overrides it |

## Failure modes and resilience targets

| Failure | Response | Target |
|---|---|---|
| Model libraries not installed, or weights fail to load | the mission refuses to start, naming the install command | never discovered after a motor turns |
| Camera frame missing or undecodable; a model raises | **unavailable** | a broken sensor never reads as an empty room |
| Target the detector has no word for | open-vocabulary path; the vocabulary verdict says so; the cloud's cold search covers it | absence from a blind detector is never trusted as absence |
| Close-range relabelling | crops are not gated on label | the frames where the target fills the view are kept |
| Local false positive | each frame that passes the gate is a sighting. It may fire a cloud trigger. It steers the robot on that frame, because steering has no frame hysteresis. Near the object it can satisfy arrival. | for steering, the per-frame probability gate is the only bound today. For ending `found`, the arrival rule adds consecutive frames, a centred bearing, a lidar range within the radius and one surface -- all local. The cloud's identity does not override a local sighting. Before `found`, the cloud must confirm identity on the arrival frame. The target "a false positive never confirms a target" is **met** on the tiered arrival path (the cloud confirms identity at arrival; decided by the user 2026-10-02 and built the same day, owned by [policy](../policy/ARCHITECTURE.md)). A false positive can still steer the robot to the wrong object |
| Camera panned | on real frames: pan is added to the bearing; tilt is not corrected; arrival refuses to judge. Sim frames carry no pan, so neither happens in the sim (harmless today: the cloud-driven policies never peek and a mission starts centred) | no panned bearing is treated as body-relative without composing it with range |
| Over the latency budget on the board | P26, then TensorRT | 250 ms a frame at 15 W |
| A real camera mistaken for the sim | provenance read once; failure assumes a real camera | real frames never get synthetic detections |

## Open questions

- **Enforce corroboration (1.11a)?** Decided by the user after two
  out-of-vocabulary searches and one control walk under the tiered policy
  (`PLAN-onboard-perception.md`, "What to record next"). Not as the guard on
  arrival: that was decided 2026-10-02 in favour of a cloud identity check
  ([policy](../policy/ARCHITECTURE.md)).
- **Is the matcher or the proposer the weak half?** The labels carry
  per-frame booleans and no boxes, so the two failures are not separable on
  this corpus.
- **INT8 on the Jetson** is unmeasured; TensorRT INT8 destroyed OWLv2 (P7d).
- **Tilt correction** needs camera intrinsics nobody has measured, and the
  Rover's camera differs from the one the field of view was set for.
- **Its own process** (2.7: features, not frames) so a stalled detector
  degrades perception rather than the control loop -- proposed, not built
  (see "Perception runs in the brain process").
