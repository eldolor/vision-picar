# Plan: perception on the car itself

Status: **design settled, nothing built** · Date: 2026-09-03, detector revised 2026-09-04 · Phase IDs: none assigned yet

Started as a holding pen after reading Microduck -- *what could run on the car
itself?* -- and became the place where a chain of hardware and architecture
decisions got made. Sections 1-3 are those decisions and the reasoning behind
them. Nothing here is implemented.

**Revised 2026-09-04:** the on-board detector moved from the Raspberry Pi AI
Camera (Sony IMX500) to a **Hailo-8L AI HAT+ with a Camera Module 3**, before
anything was ordered. 1.10 holds the decision; §4 holds the three-way
comparison (IMX500 vs Hailo vs Jetson) that made it; 3.6's bill of materials
is updated. Everything else in the plan stands.

**Also 2026-09-04:** 1.13 records a decision *not* to add a room-classification
CNN beside it -- room identity stays a VLM field on the deliberation reply and
the lidar owns the room *transition*. It is the first worked example of the rule
that a model earns accelerator space by having a consumer in the fast loop.

**Also 2026-09-06:** 1.15 settles the **physical layout** -- the lidar is the
highest point on the robot, which eliminates the under-furniture collision by
construction and makes a 3D lidar (~$400-750) unnecessary; the camera lands at
~10cm as a *consequence* of the stack rather than a wish; and the camera keeps
**both** pan and tilt, which adds a report-only goal type to 1.7. **1.16 is the
gap register** from that review -- nine items, two closed by 1.15, three of the
rest changing what gets bought.

**Decided 2026-09-06: motion becomes continuous** (1.14). The robot will hold a
velocity and perceive while moving, instead of stopping between timed bursts.
That makes §2's tiering mandatory rather than an optimisation, **puts the
accelerator on the first order**, un-retires the continuous-pose half of S6 at a
fraction of its estimated cost, and makes two safety numbers wrong -- the 1.0s
watchdog and the fixed 20cm collar. C1-C5 in 1.14 are the phasing; none of them
needs hardware.

**Reviewed 2026-09-06:** 4.8 re-examines the three-way against what the
delivery-robot companies actually run and against two price moves -- the Jetson
line was repriced upward in July 2026 and the AI HAT+ 2 is now shipping at $130.
The Pi-plus-Hailo decision stands. **4.9 then prices the option space inside
that family** and settles the part: a **Hailo-8L in M.2 module form**, because
the module is what survives a Jetson pivot and shares the PCIe lane with the
NVMe, and because the 10H's measured tokens/s make its generative half a
fallback rather than a capability. Both sections argue for buying the
accelerator *after* the first order rather than with it.

**It supersedes parts of two other documents.** Section 5 lists exactly what,
because the chassis decision in 1.1 makes `HARDWARE-READINESS.md` partly wrong
and retires a phase of `PLAN-sim-hardening.md`.

Companion to `PLAN-microduck-transplants.md` (§2's sensor-split argument, which
this assumes rather than repeats). Section 7 is a glossary for all three.

---

## 0. The question, and what it is not

Microduck runs three model families on the robot, all local, all small:

| | What | Where | How |
|---|---|---|---|
| Locomotion | 9 ONNX policies, `obs[1,61] -> act[1,14]` | `robotd`, inside the 50Hz tick | ONNX Runtime, `dlopen`'d |
| `duck-detect` | `yolo11n` @ 320, **one class**, 3.9MB INT8, mAP50 0.976 | `mediad`, off the control loop | RK3566 NPU via `librknnrt.so` |
| `pet-detect` | ~20KB audio CNN over log-mel, sub-ms | `robotd`'s audio worker | ONNX |

Only the middle one transfers. A PiCar has no gait to learn and no head to
scratch.

**Two things to hold onto.** `duck-detect` is **single-class** -- it finds
Microduck's own duck and reduces the box to a `bearing()` in -1..1. This
project's target is arbitrary text typed at mission start, which is
open-vocabulary and much harder on-device (§4.2 is why that turns out not to
matter for the targets in use).

And **recognition was never the failing half.** Stage 0 established that every
cloud model identifies a red backpack; four wordings failed on *depth*. So no
detector closes the Stage 0 gate. **A lidar does** (§3.2), which is why this
document ends somewhere different from where it started.

---

## 1. Decisions

Made 2026-09-03, in conversation, in this order. Each has its reasoning in the
section named.

### 1.1 Chassis: **differential drive**, not Ackermann

Reasoning in §3.5. The PiCar-X's Ackermann steering cannot rotate in place,
which is the single best input to lidar scan matching and the assumption behind
essentially every navigation stack and hobby SLAM tutorial in existence.

**It also retroactively makes the simulator correct**, which is a real dividend
and is documented in §5:

| Item | Was | Now |
|---|---|---|
| `sim/grid_world.py`'s pivot-in-place assumption | wrong about the hardware | **correct** |
| Phase **S6** (Ackermann turns, continuous pose, scaled map) | open, deferred | **unnecessary** |
| `HARDWARE-READINESS.md` §5.2 -- `LEFT`/`RIGHT` skip the distance check, "correct for a pivot and wrong for an arc" | the wrong branch | **the right branch** |
| `RobotInterface`'s `LEFT`/`RIGHT` verbs | approximate | literal |

**Cost:** the PiCar-X's integrated pan/tilt camera mount and ultrasonic are no
longer included, and `HARDWARE-READINESS.md` is written for that kit
throughout. A 2-axis SG90 pan/tilt bracket is ~$10-15 separately.

**And it leaves the camera height unspecified, which reaches further than it
looks.** Stage 0's instruction to shoot at "10cm" was justified in `CLAUDE.md`
as *the PiCar-X camera height*; nothing here replaces it. Estimating this
stack -- 65mm wheels, chassis plate ~40mm, a Pi deck on standoffs, the
bracket above -- lands near **10-13cm**, so the old figure survives by
accident rather than by reasoning. It is close enough for the re-recorded
Stage 0 corpus, whose defect is ~150cm against ~12cm, but the real number is
a **hardware-day pre-flight item**: it sets the viewpoint every recorded walk
is supposed to imitate, and `robot/safety.py`'s `CHASSIS_WIDTH_CM` wants
measuring on the same day for the same reason (§5.1).

### 1.2 Lidar: start at **(a)**, target **(b+)**. Model: **RPLidar C1**

The four ways to use a lidar are in §3.3. Decided:

- **Start at (a)** -- a 360-degree metric clearance ring, no SLAM. It closes the
  Stage 0 gate on its own, it is a strictly better M10 than the ultrasonic, and
  it is a driver plus trigonometry rather than an architecture.
- **If mapping proves to be the point, go to (b+)** -- ROS behind an HTTP wall,
  never (b)'s drifting hand-rolled SLAM and never (c)'s full ROS adoption.
- **RPLidar C1**, ~$100, USB.

**The model choice shifted, and why is worth recording.** The earlier lean was
the LDROBOT LD06, on weight (~50g vs ~190g) and power (~200mA vs ~500mA), for a
small Ackermann car. Both premises then changed: a differential chassis carries
the mass without complaint, and committing to (a) makes **driver quality the
binding constraint** -- the `rplidar` package is `for scan in
lidar.iter_scans()`, where LD06 means owning a community parser. `rplidar_ros`
also exists for the day (b+) arrives.

### 1.3 Power: **two rails, one ground**

```text
  [ power bank ]──USB──► Pi 5                 clean, regulated, protected
       └───────────────► lidar                data-only line to the Pi
  [ 2S Li-ion  ]───────► motor driver         separate, noisy, isolated
                            └── common ground to the Pi, nothing else shared
```

**Power the lidar from the bank, not through the Pi.** If the bank turns out to
be 5V/3A rather than 5A, the Pi 5 runs in its reduced mode and caps total USB
peripheral current near 600mA -- which an RPLidar's ~500mA would nearly consume.
A powered hub or the adapter's separate power input removes the whole class of
problem for a few dollars, whatever the bank measures.

**Motors never share the Pi's rail.** DC motors produce current spikes and
back-EMF that a USB bank's protection may simply trip on, cutting power to the
Pi mid-mission.

The 50000mAh / 22.5W bank already owned is **an excellent bench supply**, and
goes on the robot only if it weighs under ~400g and does 5V/5A. Four numbers to
read off the unit: weight, 5V current per port, USB-PD or QC only, and whether
outputs are independently regulated.

### 1.4 The tiered architecture

Reasoning in §2. The VLM stays as the **deliberation tier**; a local loop owns
the **reactive tier**.

- Deliberation is **event-driven, never periodic** (§2.4).
- Its output is an **egocentric goal**, never a coordinate the reactive tier
  cannot resolve (§2.3) -- though §3.2 relaxes this if a map lands.
- The reactive tier **never blocks** on it. It always holds a current goal;
  a new one arrives asynchronously (§2.5).
- **`brain/agent.py` becomes the degraded mode.** The rule-based frontier
  explorer that this project keeps but does not extend is what runs when the
  link is down. It stops being dead weight.

### 1.5 Map and memory persistence

Reasoning in §3.4. Ten decisions, taken together:

| | Decision |
|---|---|
| **Persistence scope** | **Only the map persists.** `MissionMemory` stays in RAM in the brain, unchanged |
| **Ownership** | The planner is a **pure function** -- context in, goal out, writes nothing. The brain is the sole writer |
| **Embedding source** | **Moot -- the visual-edge design is not being built.** See 1.6. Had it been, a separate small encoder, not the detector's features and not ORB |
| **Match handling** | Design the confirm path, **ship always-confirm**, tune the threshold on real matches later |
| **Views per edge** | **Schema for a bag, store one.** One memory per *direction of travel* |
| **House id** | **Config.** SSID as a later convenience |
| **Map key** | **By house**, visual memories scoped per camera-configuration |
| **Staleness** | Every edge carries **last-confirmed + success/failure counts**, both visible to the planner. Eviction deferred |
| **Bootstrap** | An empty graph is **not a special mode** -- but the planner's context says "map is empty" explicitly |
| **Store** | DynamoDB · needs a VPC endpoint (`network.yaml` has no NAT) · **AWS durable, Pi working copy** · `schema_version` from the first write · stored map text is data, never instruction |

**Every row above survives. One mechanism does not** -- see 1.6.

### 1.6 The visual-edge memory is **not being built**

Q1 designed a topological map whose edges carry a *visual memory* -- an
embedding of what a doorway looked like -- so a robot with no coordinates could
navigate by searching for a remembered view. That design was correct for the
problem it was given, and it is now **cancelled**.

**The reason is that (b+) deletes it.** Coordinates need no interpretation, so a
map replaces embeddings, bags of views, per-direction memories, aliasing
mitigation and match confirmation -- all of it. It is the most speculative and
most laborious part of anything designed here, and it exists *only* to work
around not having a map.

**Building it would mean building toward something already scheduled for
demolition.** So: the topological graph stays as a **placeholder that (b+) fills
in with real geometry**, and nothing implements the visual half.

**The cost, stated honestly.** Under (a) there is no map, so navigation memory is
room labels and nothing else -- roughly what exists today, and not good. That is
an honest interim rather than an elaborate workaround.

**One consequence that reached Q4, then reversed:** "can the detector emit an
embedding, or only boxes?" was load-bearing while the visual-edge design was
live, and became moot when the design was cancelled. The 2026-09-04 detector
choice (1.10) restores the capability anyway -- the Hailo model zoo carries
re-identification and place-recognition CNNs -- so if the topological map ever
comes back, the embedding is available without a hardware change. Nothing is
being built on that.

### 1.7 Goal vocabulary: **closed and versioned, three verbs**

- `approach(target, stop_within_cm)` · `traverse(bearing)` · `explore(bearing)`.
- **An unknown verb refuses by name.** Never a best-effort approximation --
  M7's rule, and the two silent-fallback bugs this project has already paid for
  (`prompt_variant` dropped by pydantic; the `NavigateModelId` env-var trap).
- The vocabulary carries a **version**, and the robot publishes which verbs it
  has, so skew between planner and robot is a named refusal rather than a
  behaviour difference.
- **`sweep` is deliberately left out.** Its stop condition ("have I covered the
  room?") is a mapping question, and under (a) there is no map. Add a fourth
  verb when the trigger log shows the planner reaching for it -- a measured
  signal, not a guess.
- **`traverse` is in the contract but not implementable until the lidar is
  fitted** -- its completion test is geometric (1.8). `approach` and `explore`
  are the two that work first.

**Why closed.** The reactive tier is a bearing, a lidar ring and a motor. It
should not become a natural-language system: that puts an ambiguity-tolerant
component inside the fast, safety-adjacent loop. A closed vocabulary is also
finite, so it is testable without hardware and drivable from the twin -- which
open-ended text is not, since its test surface is "whatever the model might
say".

**Expressiveness costs little**, because the planner runs every few seconds
anyway. "Back out, then take the other door" is two goals in sequence, and
sequencing is the planner's job -- a compound instruction is the planner doing
the reactive tier's work.

### 1.8 Stop conditions: **success from the planner, failure from the robot**

- **Success is semantic, so the goal carries it**, with a threshold. The verb
  sets the shape; the parameter tunes it.
- **Failure is physical, so the reactive tier infers it -- and names it.**
  A boolean is useless to a planner: `blocked`, `lost_target`, `no_progress`,
  `oscillating` and `timeout` lead to different next goals. This is
  `AGENT-HARNESS.md` §4.2 and M4's rule one level up.
- **The detectors already exist.** `control/walk_eval.py` computes *degenerate*,
  *stalled* and *oscillating* over recorded walks. The same logic runs live in
  the reactive tier instead of only post-hoc.
- **Success is a claim, not a fact.** The planner verifies on its next call --
  the same confirm pattern as a candidate sighting. Under (a) the robot cannot
  know it went through the *intended* doorway.

**Where a stop condition physically lives: lidar x bearing.** "Close to the
backpack" is not directly checkable -- a lidar reads geometry, never semantics.
The fusion is the same split as everything else here: **the camera says which
direction, the lidar says how far in that direction.** So `approach` resolves to
*the nearest lidar return within +/-N degrees of the target bearing is under the
threshold*. Bounding-box height as a distance proxy is uncalibrated in precisely
the way `obstacle_ahead` was -- a large object far away looks like a small one
near. Do not build on it.

**Two thresholds, two owners.** The goal's `stop_within_cm` is the planner's
intent; `safety.min_distance_cm` is the collar, non-negotiable and server-side.
**A goal asking to close inside the collar refuses at parse time, by name** --
never silently clamped, never discovered as a mysteriously unfinishable goal.
That is 1.7's closed vocabulary paying for itself immediately.

**`traverse` has the hard stop condition, and 360 degrees is what answers it.**
`approach` shrinks a distance; `explore` runs a heading. "Have I passed through
the doorway" is a fact about *where you are*, with no map to check against. But a
doorway is a narrow gap between two returns, and passing through means those
returns slide from ahead, to beside, to behind -- **the lidar watches the frame
go past.** A forward-only sensor cannot do this at all, which is another point
for 360 degrees over the 8x8 ToF originally proposed, and it only surfaced by
working a stop condition through properly.

### 1.9 A live tension -- **CLOSED by 1.10**

**"Accelerator possibly never" and 1.8's `approach` pulled against each
other.** With no on-device detector the target bearing arrives at deliberation
rate -- **1-3 seconds stale** -- so `approach` is open loop on the semantic half
while the lidar closes the loop on clearance. This project has been burned by an
open-loop assumption before (`sim/replay_robot.py`'s docstring).

Three ways out were weighed: a **cheap CPU detector** (stock YOLO11n on the Pi
5's own cores, a few Hz, nothing to buy); accept open-loop `approach` over short
hops and let the planner re-issue, which erodes the trigger discipline; or **let
the lidar carry it once locked** -- take one bearing fix, then track that
geometric feature in the scan while closing.

**Resolved by putting a detector on the car (1.10).** With an on-board detector
the bearing is fresh at camera rate, so `approach` is closed loop on both halves
-- the detector for direction, the lidar for distance. None of the three
workarounds is needed. The CPU-only detector survives as the day-one baseline
(4.4), not as the design.

### 1.10 The reactive tier: **two layers**, and a Hailo

**Decided 2026-09-04: a Hailo-8L AI HAT+ and a Camera Module 3.**
**Amended 2026-09-06: a Hailo-8L in M.2 module form** -- briefly the AI HAT+ 2
by item 6's own test (4.8), reverted the same day on the 10H's measured
tokens/s (4.9). The platform decision below is unchanged; only the part and its
form are, and 4.9 argues for buying either one *after* the first order. The first
decision here (2026-09-03) was the Raspberry Pi AI Camera, whose Sony IMX500
runs a detector on the sensor itself. It was reversed a day later, before
anything was ordered, on one requirement the original evaluation had not
weighed: **the detector must not be locked to the nano tier.** §4 holds the
full three-way comparison (IMX500 vs Hailo vs Jetson) and the reasoning; this
section holds the decision and what it commits to.

**Why not the IMX500.** Its ~8MB of on-sensor memory is a silicon ceiling, not
a tuning limit: one nano-class model resident at a time, no path to feed it a
stored frame, and every capability that makes perception more interesting than
"a box for a COCO class" sits above that line -- the `s` and `m` YOLO tiers for
a small target across a room, floor segmentation and depth beside the detector,
CLIP over crops for "the red one". Buying it would have paid $70 to close the
door the project wants open. Its two real advantages -- zero Pi CPU and a free
PCIe slot -- are worth less than that door (4.6).

**Why not a Jetson.** A Jetson Orin Nano is the only board on which a Hugging
Face model runs without a compile step. It was ruled out **for now** on cost
(~$170-220 over the Pi 5 plan at the time -- **~$320-430 since the July 2026
repricing**, 4.8), power (three times a Pi at 15W, and no 5V USB
supply), the loss of the Pi camera stack, and 8GB shared with the GPU that
cannot hold SLAM, nav2, a detector and a local VLM at once. 4.7 keeps the
evaluation, because it is the on-board upgrade path if flexible inference ever
becomes the point rather than a wish.

**And the decision simplified the design it was made for**, exactly as the
IMX500 one had. A three-layer scheme had been proposed: VLM for identity,
**lidar blob tracking** for continuous bearing, and a CPU detector to
re-acquire when tracking broke. Layer two existed for one reason -- *you should
not have to re-detect the target ten times a second*. With a continuous
on-board detector that is free, so:

| | Was going to be | Now |
|---|---|---|
| Continuous bearing | lidar blob tracking + data association | **the detector, directly** |
| Range to target | lidar at the tracked blob | **lidar at the detector's bearing** -- 1.8, already decided |
| Re-acquisition | a CPU detector as backup | **not a thing** -- tracking cannot break if detection never stopped |

What remains is what 1.8 already specified -- the detector points, the lidar
measures -- plus the VLM as the deliberation tier that already exists. No new
tracking machinery.

#### What it commits to

1. **A compile step per model.** Every model goes PyTorch -> ONNX -> Hailo's
   Dataflow Compiler -> HEF, on an x86-64 Ubuntu host with a calibration set
   of a few hundred frames. There is no Mac path; the practical compile host
   is an EC2 instance for an hour per model (4.3). **Build that loop before
   the hardware arrives** -- one YOLO11n from Hugging Face to a HEF, and a
   script that scores it over the recorded walks on EFS. If the loop exists
   on day one the Hailo is a sandbox; if it never gets built, the Hailo is a
   fixed-function part and the IMX500 was the cheaper way to get one.
2. **The PCIe lane.** The AI HAT+ takes the Pi 5's single PCIe connector, and
   so does the NVMe HAT that 3.6 calls the one optional item worth buying.
   Decide at ordering time between a high-endurance microSD plus a
   clean-shutdown habit, or the M.2-module form of the Hailo-8L on a
   dual-slot switch board (~$40, reported to work, **unverified**).
3. **5V/5A is firm.** The HAT draws ~1.5W typical and peaks higher, all from
   the Pi's rail (1.3).
4. **CPU is small but not zero.** The chip does the convolutions; the Pi still
   captures, resizes and runs whatever post-processing Hailo leaves on the
   host -- a few percent of one core for a nano detector. Fine under (a),
   minor under (b+).
5. **Not the Hailo-8.** The 26 TOPS part buys nothing here: the camera's 30fps
   bounds a nano or small detector either way (4.4).
6. **Check the AI HAT+ 2 once, then stop waiting for it.** If the Hailo-10H is
   in stock at ~$130 and its small-VLM support is documented for real models,
   the extra $60 turns a detection sandbox into a broader one. If either is
   unverified at ordering time, take the 8L. **Checked 2026-09-06: both
   conditions read yes** -- shipping at $130 with 8GB of its own RAM and named
   1.5B-class LLMs and a VLM through `hailo-ollama`. That briefly settled it for
   the 10H (4.8), and **4.9 reversed it the same day**: at 5.89 tokens/s on a
   1.5B model the generative half is a network-down fallback, not the broader
   sandbox this item was buying. **Take the 8L, as a module.** The item was
   written to test availability when it should have tested throughput.
7. **Stacking, half-answered 2026-09-06 (4.9).** The HAT sits above the active
   cooler on 16mm standoffs. **The 40-pin header does pass through, but only
   deliberately**: the AI HAT+ 2 ships an extension header that, seated fully,
   leaves no pins accessible, so a 2x20 extra-tall stacking header is a line
   item and clearance against cooler, HAT and lidar deck is a measurement. This
   robot needs that header for the motor driver, two servos, the encoders and
   the IMU. Whether the Yahboom encoder board wants the header or wires to it
   is still a question for 3.8's seller list.

#### What it buys that the plan had not weighed

- **CLIP over crops answers "the red one".** COCO says `backpack`, not `red
  backpack`. 4.2 had proposed an HSV check on the crop. With the detector for
  boxes and CLIP's image encoder (in the Hailo model zoo, in both ViT-B/32 and
  ResNet-50 forms) scoring each crop against the mission's actual target
  string, the on-board layer gets a text-conditioned re-ranker -- not full
  open-vocabulary detection, but "this one, not that one" without colour code.
- **The detector can be scored on the recorded corpus.** A Hailo runs on
  frames the Pi captured, so the same HEF can be fed the walks already on EFS
  and scored with `control/walk_eval.py` before it ever drives the car. The
  IMX500 could not be fed a stored image at all. For a project whose rule is
  "prove it first" (`CLAUDE.md` §7), that is a material difference.
- **Any camera, and the same frame the VLM sees.** A Camera Module 3 with
  autofocus, a wide-angle or global-shutter module, or a USB camera all work,
  and the detector and the cloud VLM look at the identical frame -- which
  keeps 1.11's arbitration honest.
- **Several models resident at once.** HailoRT's scheduler time-slices HEFs,
  so detector + CLIP + a floor-segmentation model can run together. The
  IMX500 runs one.

### 1.11 Arbitration: split by question, not by authority

Microduck's `architecture.md` §6 rule -- *decided priority, not
last-writer-wins* -- governs **control**. This governs **facts**, so the rule is
different: each question goes to whichever source can actually answer it.

| Question | Who wins | Why |
|---|---|---|
| **Is it a red backpack?** | **VLM** | more capable at semantics, and it holds the mission's definition of the target |
| **What bearing?** | **detector** | geometric precision beats a natural-language direction. **Body-relative bearing = pan + tilt-corrected in-frame bearing (1.15.3)** -- both servo angles must ride in the frame's own metadata, or a servo still moving corrupts it silently |
| **How far?** | **lidar** | 1.8's fusion -- range at the detector's bearing |
| **Is it there right now?** | **detector** (fresh) | but a VLM "not here" **triggers re-confirmation**, never a silent override |
| **Which room is this?** | **VLM** | semantics again -- and it arrives free on a reply already being paid for (1.13) |
| **Did I just change rooms?** | **lidar** | a transition is geometric: the doorframe's returns slide ahead -> beside -> behind (1.8, 1.13) |
| **Can I reach it?** | **planner** | it knows the mission, the memory and the map |
| **Will I hit something?** | **lidar + collar** | no vote, ever |

**The instinct that "if Bedrock is more capable, the planner wins" holds where it
should and only there.** It is right for *identity* and wrong for *geometry*: a
COCO detector is measurably better than a VLM at the one thing `approach` needs,
because a bounding box is a bearing and "slightly to the left" is not. Stage 0
already established that every model identifies a red backpack -- capability was
never the scarce thing. Precision was.

**CLIP is a pre-filter, not a vote.** 1.10's crop re-ranker scores a candidate
against the target string *before* the cloud is asked, so §2.4's
candidate-sighting trigger fires on "backpack, and probably the red one" rather
than on every backpack. Identity still belongs to the VLM: a high CLIP score
raises a candidate, it never confirms one.

---

### 1.12 The sim participates, with **synthesised detections**

`sim/renderer.py` draws flat-shaded raycaster walls. A COCO detector finds
nothing in them, so the reactive tier cannot be tested against rendered pixels.
Three options were weighed: the sim does not participate; the sim synthesises
detections from grid truth; or the renderer is made detectable by drawing real
objects.

**Decided: synthesise from grid truth.** `MockRobot` knows where the target is
and emits a bounding box directly -- no rendering, no model.

**What that tests, and what it cannot.** The sim's job here is the **loop**, not
the model: is a goal issued, executed and reported; does the collar veto; does
the trigger discipline fire when it should; does arbitration behave when sources
disagree. Synthetic detections exercise all of it, deterministically and for
free. **The sim leg tests the detector's consumers, never the detector.** Any
claim about detection accuracy comes from real frames -- Stage 0, the recorded
walks and the replay harness, which use real pixels. The renderer's fidelity
note already says why a sim result is not a statement about real rooms.

The precedent is `MockRobot` already keeping grid facts beside its pixels, and
`unusable_grid()` one sensor over: **a backend that says honestly what it does
and does not have**, so no run can look as though it exercised perception it
never had. A synthesised detection is marked as such.

#### Perfect, but occlusion-aware -- fidelity, not noise

"Perfect" must not mean *the sim always says where the target is*. A detector
that sees through walls would make the sim useless for exactly the behaviours
most worth testing -- `explore`, room-to-room movement, `lost_target`.

The baseline is **exact bearing when the target is genuinely visible, nothing
when it is not**: in the camera's field of view *and* unoccluded.
`sim/renderer.py:cast_ray()` already exists, so the occlusion test is a ray to
the target and a comparison. **Build it from the start; it is not noise.**

#### Three-way output from day one, for M3's exact reason

The depth grid distinguishes `ZONE_RANGE` / `ZONE_NO_TARGET` / `ZONE_UNUSABLE`
because "nothing there" and "I could not tell" mean opposite things. A detector
needs the same three:

| | Means |
|---|---|
| `detected` | with bearing and confidence |
| `absent` | the frame was good and the target is not in it -- **information** |
| `unavailable` | no frame, stalled pipeline, camera fault -- **the absence of information** |

The failure this designs out is concrete: **a wedged `picamera2` capture reading
as "the target is gone"**, ending a mission `lost_target` when the truth is that
the camera died. Same class as M9, and free to prevent now.

#### Noise behind a flag, mirroring `sim.sensor_noise`

Off by default, same shape as the existing `config/robot.yaml` block. Four
kinds, each mapped to something in 1.11:

| Noise | Tests |
|---|---|
| Bearing jitter | that `approach` does not oscillate when the bearing wobbles |
| Dropout | that a missed frame needs hysteresis before it becomes `lost_target` |
| False positive | 1.11's rule that a VLM "not here" triggers re-confirmation, never a silent override |
| Misclassification | identity arbitration -- the VLM wins |

**Not on that list: box-size error.** 1.8 gives distance to the lidar, so the
detector's box size is never load-bearing. One fewer failure mode, and a
confirmation that the sensor split was right.

#### Write the noise tests *with* the flag

S5 is the argument. Sensor noise shipped off by default -- correct -- and
`min_distance_cm: 30.0` therefore sat **exactly on a quantization boundary,
undiscovered for months**: a noiseless reading is only ever a multiple of 30, so
every threshold in (0, 30] behaves identically. Switching noise on surfaced it
at once, and it was a real defect -- the veto was near a coin flip at one cell
of clearance.

The repo's answer was not to make noise default-on. It was
`test_min_distance_cm_is_load_bearing_at_a_non_multiple_of_30`: **a specific
test that turns noise on to pin a property that only exists under noise.**

So: off for the bulk of the suite, plus a handful of tests that enable it
deliberately, written at the same time as the flag. **A flag no test ever
enables is decorative** -- the same failure as an encoder nothing reads (3.8).

---

### 1.13 Room identity: the **VLM names it, the lidar detects the change**

**Decided 2026-09-04.** A room-classification CNN on the Hailo (a Places365-class
model on a ResNet backbone -- it compiles on the 8-series without difficulty) was
proposed and **rejected**. `room_guess` stays a VLM field on the deliberation
reply, and the *transition* between rooms becomes a lidar event.

**On the live path it is already free**, which is the economic half of the
argument. Two mechanisms produce a room label in this repo, and only one of them
is on the hardware path:

| Route | How | On the hardware path? |
|---|---|---|
| `/analyze` | `identify_room()` -- `brain/rooms.py`'s substring matcher over the VLM's `important_objects` (`service/vision_analyze/app.py:219`) | no |
| **`/navigate`** | **a field the VLM emits directly**, a line in the prompt's own JSON schema beside `action` and `reasoning` (`vision_core.py:254`, `:391`, `:467`) | **yes** |

So the marginal cost of a room label on the deliberation path is **one field in a
reply already being paid for**. A CNN would spend a compile cycle, accelerator
capacity and a calibration corpus to replace something that currently costs
nothing.

#### The circularity that made a classifier look necessary

The real argument for on-board classification was structural, not economic. §2.4
fires **"room change suspected"** from the *reactive* tier -- but if room labels
only arrive on deliberation replies, that trigger is circular: the cloud has to
be called to learn that the cloud should be called. The only escapes are polling,
which violates §2.4's *event-driven, never periodic*, or a local classifier.

**The reactive tier does not need a label, it needs a transition -- and that is
geometric.** 1.8 already worked this out for `traverse`'s stop condition: a
doorway is a narrow gap between two returns, and passing through means those
returns slide from ahead, to beside, to behind. The lidar watches the doorframe
go past.

So the split is the one the rest of this plan already runs on: **the lidar
detects that the room changed; the VLM names the room.** No classifier, no
circularity, and the trigger fires on a physical event rather than on a semantic
guess. §2.4's table is updated accordingly.

#### The flicker argument reverses on inspection

§6.1 measured `room_guess` changing on **12.7% of frames** -- `control/walk_eval.py`'s
`unstable-identity` flag seen from the cost side -- which reads as an argument for
a stable local model with frame-level hysteresis. It is not. That number was
measured on a corpus where the question was asked **every frame, including frames
pointed at a blank wall in the middle of a room**: most of the flicker is an
unanswerable question asked repeatedly.

Sampling only at lidar-detected transitions removes the noise **by construction
rather than by filtering** -- the question is asked exactly when the view is most
informative, five to eight times a mission. That is a better estimator than
hysteresis over a noisy stream, and it needs no new model. It does **not** repeal
§6.1's hysteresis finding: `target_visible` flips on 24.1% of frames and has no
geometric event to gate it, so it still needs the filter.

#### Where the CNN would have been actively worse

Places365-class models are trained on **photographs taken by standing humans**.
This camera sits at 10-13cm (1.1) looking at chair legs, table undersides and
floor. That is out of distribution in precisely the way Stage 0 spent a month
learning to care about -- **the standing-height corpus defect, re-introduced as a
model choice instead of a data choice.**

Fine-tuning it out would need a calibration set of labelled floor-height walks
across every room type, and that corpus does not exist: the 39 walks on EFS are
the invalid standing-height ones. A VLM is markedly more robust to that viewpoint
shift than a fixed classifier, because it reasons from what is visible rather
than matching a learned scene prior.

#### And 1.6's argument applies unchanged

Under (b+), §3.2 has rooms becoming **regions of an occupancy grid**, with
`room_guess` labelling a place whose geometry is already known -- room identity
turns into a *localisation* answer applied once per region, not a per-frame
perception answer at all.

A Hailo room classifier would therefore be built toward something already
scheduled for demolition. That is the argument that cancelled the visual-edge
memory in 1.6, and if it was good enough to kill the most elaborate design in
this plan it is good enough to kill a bonus CNN.

**What the accelerator's cycles should go to instead** is unchanged from 4.3:
floor segmentation first, then a larger YOLO tier, then CLIP over crops. Each has
a consumer in the fast loop. **A room label has no consumer faster than the
planner, and the planner is the thing producing it** -- which is the general rule
this section is one instance of.

### 1.14 Motion becomes **continuous**, not discrete

**Decided 2026-09-06.** The robot holds a velocity and perceives *while moving*.
Timed bursts stop being the only motion primitive.

Today every verb on `RobotInterface` is a bounded burst --
`drive_forward(speed=50, duration=0.5)`, `turn_left(angle=90)` -- so a tick is
capture, decide, move, stop, capture. **The robot is stationary the whole time
it thinks**, which is why 4.4 could conclude that the Pi's own cores clear the
reaction budget: a 3.6s decision cost 3.6s parked, never a centimetre of travel.
That stops being true here, and several numbers stop being true with it.

#### 1. It makes §2's tiering mandatory rather than an optimisation

**You cannot drive continuously from a 0.5Hz cloud loop.** Something on-board
has to close the loop at 10-50Hz. 1.4 already says the reactive tier *"never
blocks"* on deliberation and *"always holds a current goal"* -- but under
discrete driving that text was aspirational, because nothing was moving to hold
a goal *for*. **It is operational now.** The reactive tier stops being the
interesting future half of this plan and becomes the thing that drives the car.

2.1's diagnosis -- *"vision-picar collapses all three rates into one"* -- was a
description of a design smell. It is now a blocker.

#### 2. The accelerator stops being deferrable

4.9's tier table gives the Pi 5's own cores 5-13 FPS, **below** 2.1's 15-30Hz
perception row. Discrete driving hid that, because a slow perception tier only
cost wall-clock while parked. Continuous driving spends it in centimetres.

**Put the Hailo-8L on the first order.** 4.8 and 4.9 both argued for deferring
it, and both named this decision as the one trigger that would reverse them.
The rest of 4.9's recommendation is unchanged -- an **8L in M.2 module form**,
which clears the perception row by 10-30x with headroom for the larger YOLO
tiers, floor segmentation and CLIP.

#### 3. `RobotInterface` gains a held-velocity verb

Natural form for a differential chassis (1.1): a linear and an angular rate,
e.g. `set_velocity(linear_cm_s, angular_deg_s)`. Four constraints on it:

- **It is a held command with an expiry, not fire-and-forget.** The watchdog is
  the expiry (item 4).
- **The burst verbs stay.** The twin's D-pad, `ReplayRobot`, `TeleopRobot` and
  most of the suite depend on them, and each is expressible as velocity plus a
  duration plus a stop. This is an addition, not a replacement.
- **All five backends implement it**, and the two with no motor no-op it
  honestly -- the pattern `replay_robot.py` and `teleop_robot.py` already use.
- **`tests/test_robot_contract.py` gains cases**, since it is the thing that
  makes a new verb mean the same on every backend.

#### 4. `watchdog_timeout_s: 1.0` becomes unsafe

At 1.0s and 50cm/s the robot travels **50cm after the commanding process dies**
-- two and a half times the 20cm collar. The timeout has to come down to roughly
**100-200ms**, which means the drive loop must command faster than that, which
is item 1 restated from the safety side.

**The watchdog stops being a backstop and becomes the primary deadman.** Its
decision function is already pure and tested (`robot/server.py:143`), and M4
already made authority lapse on the same clock, so the shape is right and the
number is not.

#### 5. The collar becomes speed-dependent, and 20cm is already marginal

`SafetyController` holds a fixed `min_distance_cm = 20.0` and `path_clearance()`
compares against it (`robot/safety.py:188`, `:257`). The right quantity under
continuous motion is **stopping distance**:

```text
stopping distance = v * t_react + v^2 / (2a)
```

At 200ms of reaction and 1 m/s^2 of braking:

| Speed | Reaction | Braking | Total | Against a 20cm collar |
|---|---|---|---|---|
| 30 cm/s | 6cm | 4.5cm | **10.5cm** | fine |
| 45 cm/s | 9cm | 10cm | **19cm** | **the limit** |
| 50 cm/s | 10cm | 12.5cm | **22.5cm** | already past it |
| 100 cm/s | 20cm | 50cm | **70cm** | 3.5x the collar |

**So the collar as it stands is good for about 0.45 m/s**, and 3.8's vendor
figures put the chassis's upper bound near 1 m/s. Two ways out, and the second
is better: make the collar a function of the commanded speed, or **make the
speed a function of the measured clearance**. The second degrades smoothly
instead of vetoing, it is what a real robot does, and it gives M3's
`path_clearance()` a second consumer -- it already returns the one number this
would divide by.

#### 6. Encoders become load-bearing, and closed-loop

A timed burst can be open loop. A held velocity cannot: without a controller on
wheel speed the robot curves, and the error integrates for as long as it drives.
That means a PID on encoder counts. This retroactively validates 3.6's
**encoder-motor** chassis choice and promotes the **IMU** from "strongly
recommended" to effectively essential for heading.

#### 7. Continuous pose in the sim -- and it is far cheaper than S6 assumed

`sim/grid_world.py` holds `robot_x, robot_y` as **integers** and a four-value
`Heading` enum, and `move(cells)` steps whole cells. That has to become
`(x, y, theta)` floats integrated over dt.

**But `sim/renderer.py` needs no change at all.** It is already continuous:
`cast_ray(layout, px: float, py: float, angle: float)` and
`render(..., px, py, base_angle)` take floats and radians, and the *discretising
step is a two-line conversion at the boundary* --
`base_angle = renderer.HEADING_ANGLE[view.name]` and
`px, py = self.world.robot_x + 0.5, self.world.robot_y + 0.5`
(`sim/mock_robot.py:227-228`). S2's port was written against a float pose from
the start.

**This un-retires the continuous-pose half of S6**, which §5 had deliberately
left alive -- *"continuous pose and a to-scale map may still be wanted if a
lidar lands"* -- at a small fraction of S6's estimated cost, because the
expensive half was already built for a different reason.

#### 8. `MissionRunner`'s tick decouples into two loops

`tick()` is one blocking sense-decide-act step today. Continuous motion needs a
**drive loop** at 10-50Hz holding the current goal and a **mission loop**
updating that goal asynchronously. `AGENT-HARNESS.md`'s tick contract, its
concurrency section and the status shape all move; the failsafes B3.2 and B3.3
keep their jobs but change what they are timing.

#### 9. What it owes the twin

Per §7 of `CLAUDE.md`. The D-pad becomes press-and-hold; the FPV canvas and the
depth strip update *while the robot moves* rather than between moves; and the
proof to press is **watching it cross a room without stopping**, with the
clearance-derived speed visible as it slows near a wall.

#### What it does not change

The chassis (1.1 was already differential), the lidar (1.2), the two power rails
(1.3), the camera, the arbitration order (M4), the goal vocabulary (1.7), and
1.13's room-identity split. **And the cloud VLM stays exactly where 2.1 put it**
-- event-driven, ~0.5Hz, off-board. Continuous driving does not make the cloud
call faster; it makes it *stop blocking the wheels*, which was always the point
of tiering.

#### Scope, honestly, and a phasing

**This is the largest change in the plan since the chassis decision** --
interface, safety, sim, runner and twin, plus a reactive drive loop that does
not exist yet (2.7). Proposed phases, IDs not yet assigned:

| | What | Depends on hardware? |
|---|---|---|
| **C1** | `set_velocity` on `RobotInterface`, all five backends, contract tests. Nothing uses it yet. **Plus two items 1.16 found**: a pose/odometry method (#5) and a timestamp on every reading (#3) -- both are interface changes and belong in the same conformance pass | no |
| **C2** | Continuous pose in `grid_world.py`; delete the boundary conversion; press-and-hold D-pad in the twin | no |
| **C3** | Watchdog timeout down to ~150ms; clearance-derived speed replacing the fixed collar. **Plus the ESP32's own deadman** (1.16 #6) -- the innermost guard, and the only one that survives the Pi locking up. It needs a drill, per §7 | no |
| **C4** | The reactive drive loop as its own process (2.7), holding a goal | no |
| **C5** | Deliberation becomes event-driven against it (2.4) | no |

**None of C1-C5 needs the robot**, which means all of it is provable in the twin
before hardware day -- and C3 in particular is much better discovered in the sim
than on a chassis moving at half a metre per second.

### 1.15 Physical layout: **the lidar is the highest point**

**Decided 2026-09-06**, working through the first gap 1.16 found. The plan had a
sensor suite and a bill of materials but never a *stack-up*, and continuous
driving (1.14) plus a pan/tilt camera turned that omission into a collision
hazard.

#### The failure it fixes

A 2D lidar scans **one horizontal plane**. Let the robot occupy heights
`[0, H_max]` and the plane sit at `h_lidar`:

| Obstacle | Detected? | Actually a collision? |
|---|---|---|
| entirely above `H_max` | no | **no** -- the robot fits under it |
| intersecting the plane | yes | yes |
| below the plane, inside `[0, H_max]` | **no** | **yes** |

**Set `h_lidar` = `H_max` and the middle failure mode cannot occur.** A table
stops being an obstacle and becomes a tunnel the robot genuinely fits through.
The hazard that prompted this -- driving under a chair and striking a camera
mast -- exists only when something is mounted *above* the scan plane, so the
rule is: **nothing on the robot may be taller than the lidar's scan plane.**

What survives is the **under-plane residual**: shoes, cables, thresholds, pet
bowls, a low sofa rail. That is a real gap and it is answered in 1.15.4, not by
the lidar.

#### A 3D lidar would also fix it, and is the most expensive way to

| Option | ~USD | Against a ~$500 build |
|---|---|---|
| Unitree 4D L2 | ~400 | 80% |
| Livox Mid-360 (360° x 59°) | ~749 | 150% |
| Orbbec Gemini 335 depth camera | ~250 | 50% |
| **RPLidar C1 mounted on top** | **0** | -- |

And it is not only money. The Mid-360 is ~265g and ~6.5W on a chassis whose
whole compute stack is ~10W, and its ~200k points/s needs real processing --
which walks straight back into the Jetson comparison 4.8 closed. **Rejected.**

**If the under-plane residual proves real in testing, the upgrade to reach for
is a depth camera (~$250), not a 3D lidar** -- because it also feeds the floor
segmentation model 4.3 already wants, which a lidar does not.

#### 1.15.1 The stack-up

**Verified figures**, from vendor documentation on 2026-09-06:

| Part | Dimensions | Source |
|---|---|---|
| RPLidar C1 | **55.6 x 55.6 x 41.3mm, 110g**, 5V, UART | Slamtec / retailer specs |
| Pi 5 + Active Cooler -> HAT | **16mm board-to-board** (Raspberry Pi: "at least 15mm; 16mm ideal"; the M.2 HAT+ ships a 16mm stacking header and spacers) | Raspberry Pi docs |
| 2-axis SG90 pan/tilt bracket | **32 x 28 x 65mm, 42g**; pan 180°, tilt 130°; fits a 28x28mm camera | retailer specs |
| Camera Module 3 | ~25 x 24 x 11.5mm | general knowledge, **unverified** |

**Estimated stack**, and every row is an estimate until measured:

| Height (mm) | What |
|---|---|
| 0-45 | Wheels (65mm, axle at 32.5), motors, chassis plate, Waveshare driver board, 3S pack |
| ~55-60 | Pi 5 PCB, on standoffs off the plate |
| +16 | HAT / M.2 carrier board -- **verified spacing** |
| ~85-90 | Top of the electronics stack |
| **45-110, at the front** | **Pan/tilt bracket -- lens centre lands at ~100mm** |
| **~123-140** | **Lidar scan plane**, on a pedestal above the stack |
| ~155 | `H_max` -- top of the lidar body |

**Two results fall out, and the first is a relief.** The camera lands at
**~10cm** without anyone choosing it -- inside the 10-13cm band `CLAUDE.md`
asks the Stage 0 corpus to be recorded at. That number survives the chassis
change after all, and 1.16's gap #2 closes: **camera height stops being a wish
and becomes a consequence of the stack.**

#### 1.15.2 The conflict, and the number to optimise

The naive arrangement -- bolt the lidar straight onto the HAT -- puts its scan
plane at roughly **123mm** while the pan/tilt bracket tops out at **110mm**, and
a camera tilted up rises further. **That is single-digit millimetres of
clearance, which is not a margin**, and if the camera ever enters the plane it
blinds the lidar in the forward arc -- the one arc that matters.

So the lidar needs a **deliberate pedestal**, not a bracket. And the quantity to
optimise is precise:

> **Put the scan plane as low as it can go while still clearing the camera's
> swept envelope at full tilt.**

Every millimetre higher raises `H_max`, and `H_max` *is* the under-plane blind
volume. Raising the lidar to buy clearance is not free -- it is paid for in
exactly the blind spot 1.15.4 then has to cover. A shorter pan/tilt bracket buys
back margin at both ends.

#### 1.15.3 Pan **and** tilt, both angles in the frame

**Decided 2026-09-06, after an argument that reversed the first answer.** The
first take kept pan and dropped the tilt servo, reasoning that tilt only buys
looking up onto furniture, which Stage 0 ruled out as a navigation target. **That
was a category error: a navigation constraint used to rule out a perception
capability.** Stage 0 says the goal you *drive to* must be reachable. It never
said the robot should be blind above the floor.

The geometry settles it. Camera at 12cm with Camera Module 3's ~41° vertical
field, pointed level -- the top of frame sits at `12cm + 0.374 x distance`:

| Distance | Highest thing in frame | |
|---|---|---|
| 1 m | **49 cm** | below a table (75cm), below a counter (90cm) |
| 2 m | 87 cm | table height, barely |
| 3 m | 124 cm | |

**At normal indoor working distance a level camera cannot see a tabletop at
all** -- it sees table legs, and the failure is silent, because the object is
not misidentified, it is simply out of frame. Tilting +30° at 1m swings the view
to **29-133cm**: tables, counters, sofa seats, low shelves. That is the
difference between a floor inspector and a robot you can ask questions.

**And M3 is what makes it safe.** While the camera (or a camera-aimed
ultrasonic) was the obstacle sensor, tilting up while driving would have blinded
the safety layer. Post-M3 the collar reads the lidar and 1.11 gives the camera
identity while the lidar keeps geometry -- **the sensor split is exactly what
frees the camera to look wherever it likes.** Tilt is a capability M3 unlocked
and nobody noticed.

Two consequences, both worth having:

- **The goal vocabulary gains a report-only type.** 1.7's three verbs all assume
  you drive to the thing. Tilt makes *"find and report"* a distinct legitimate
  goal -- *"it is on the console table, here is the frame"* -- whose outcome is
  **success**, not failure. 1.8's stop conditions need a "found, not reachable,
  and that is fine" terminal state. **This is a design gain that came out of the
  hardware argument, not the other way round.**
- **Both angles ride with the frame.** The detector's body-relative bearing is
  `pan + tilt-corrected in-frame bearing`, so the commanded angles must be
  captured *in the frame metadata*, not read separately afterwards -- otherwise a
  servo still moving silently corrupts the bearing. 1.11's "What bearing? ->
  detector" row is amended accordingly.
- **Capture must be pan-settled.** An SG90 is ~0.1s/60° plus settling; driving
  and panning smears a frame twice. Discrete pan positions, settle, capture --
  **not** a continuous sweep. This bounds how fast a scan can sweep and is a
  requirement, not an implementation detail.

#### 1.15.4 The under-plane residual, for ~$20

What the lidar-on-top rule does not cover, and what does:

| Mitigation | ~USD | Covers |
|---|---|---|
| **Floor segmentation** on the camera | 0 | The drivable-floor question directly. 4.3 already calls it "the first experiment worth running" -- it is now **safety-relevant**, not exploratory |
| **Camera tilt-down** | 0 | Near floor ahead, using the axis 1.15.3 just added |
| **2x VL53L1X ToF**, forward-down | ~12 | Cliffs, thresholds, low obstacles the plane misses |
| **Compliant bumper + microswitches** | ~5 | The physical last resort, and the only one that works when every model is wrong |

**The bumper is not optional.** Everything above it is an inference; the bumper
is a measurement. On a robot that now moves continuously it is the cheapest
guard in the build.

#### 1.15.5 What must be measured before this is real

Hardware-day pre-flight, and the first three block the pedestal design:

1. **Where the C1's scan plane sits inside its 41.3mm body.** Not published in
   the datasheet's specification tables -- it is in the mechanical drawing
   (Figure 4-1). Budget ~25-30mm above the base and **confirm on the bench**;
   whatever body sits above the plane is unprotected by the rule in 1.15.
2. **The real stack height**, with your cooler, standoffs and M.2 carrier. Sets
   everything else, including whether the camera actually lands at 10cm.
3. **The camera's swept envelope at full tilt**, which sets the pedestal.
4. **The C1's current draw** (5V confirmed, current not) -- it goes in the power
   budget 1.16 #9 says does not exist.
5. **Lidar occlusion check, all 360°** -- with the pedestal built, confirm no
   part of the robot enters the plane at any pan/tilt position.

---

### 1.16 Gap register, 2026-09-06

Found by review after 1.14 and 1.15 were decided. **#1 and #2 are closed by
1.15**; the rest are open and are recorded here so they are not rediscovered on
hardware day. Three were verified against the code, not guessed.

| # | Gap | Status |
|---|---|---|
| 1 | 2D lidar plane routes the robot under furniture into its own mast | **CLOSED by 1.15** -- lidar is the highest point |
| 2 | Camera height determined by the stack, not chosen; the 10-13cm corpus instruction assumed otherwise | **CLOSED by 1.15.1** -- it lands at ~10cm, and is now a measurement |
| 3 | **Time synchronisation does not exist** | **OPEN.** Parked, a frame + scan + encoder count + pan angle were all "now". At 0.4 m/s with a 10Hz lidar they are up to 100ms and 4cm apart, so 1.8's *"lidar range at the detector's bearing"* mixes a bearing from one pose with a range from another. **Nothing in `RobotInterface` carries a timestamp.** Belongs in C1 |
| 4 | **Camera-lidar extrinsic calibration unspecced**, and pan/tilt makes it time-varying | **OPEN.** 1.8's fusion assumes a known transform between the two sensors; with two servo angles it is a function, not a constant. No procedure, no accuracy target |
| 5 | **`RobotInterface` reports no pose or odometry** | **OPEN, verified**: eleven methods, none says where the robot is. Discrete driving counted moves instead. A drive loop holding a goal must know how far it has got -- a new method in `get_depth_grid()`'s class (honest default, five backends, conformance suite). **1.14's C1-C5 missed it.** Note also that encoders measure *wheel* rotation, not ground travel: on carpet that drifts, the IMU fixes only heading, and scan matching is the real answer -- which pulls 3.3's (b+) forward |
| 6 | **The ESP32 needs its own deadman -- a fourth failsafe** | **OPEN.** B3.1/2/3 cover the robot watchdog, the vision budget and a hung tick. None covers the Pi-to-ESP32 link dying while the ESP32 holds a velocity. It is also the **only** guard that can stop the wheels if the Pi itself locks up. Per §7 of `CLAUDE.md` it needs a drill |
| 7 | **Open-loop servos: pan/tilt is commanded, not measured** | **OPEN.** SG90s have no feedback. A stalled, slipped or knocked servo makes every bearing wrong by that amount with nothing to notice -- the same silent-corruption class as the stray-frame bug. Since bearing is the camera's one job (1.11), this may justify **ST3215 bus servos** (position feedback, driven natively by the Waveshare board). Minimum: startup homing plus a plausibility check |
| 8 | **Tilt cannot be represented in the twin at all** | **OPEN, verified**: `renderer.render(layout, objects, px, py, base_angle, ...)` is a 2D raycaster with **no pitch parameter**, and `grid_world.look_left()` sets `pan = -1` -- pan is tri-state snapped to cardinal headings, not a continuous servo. Tilt would be **the first feature to reach hardware with no twin representation**, which `CLAUDE.md` §7 forbids. Either the renderer gains a pitch (real work; it is 2D by construction) or tilt takes §7's written once-per-phase exemption. **Do not let this one pass silently** |
| 9 | **No power budget and no runtime estimate** | **OPEN.** Pi 5 under load + Hailo + lidar + camera + servos, now *sustained* rather than bursty. A Pi 5 with a HAT wants 5V/5A and many banks will not hold 25W -- and 3.6 carries the bank as "already owned, 0". Servos are motors, so 1.3's own rule about keeping motor noise off the compute rail applies to them; SG90s on the Pi's 5V rail is the textbook brownout. Runtime decides how long a test session can be, which decides how the corpus gets recorded |

**Three of these change what gets bought** -- #7 (which servos), #9 (whether the
power bank is adequate, and whether servos need their own supply) and 1.15.4's
bumper and ToF pair. The rest are design work that can proceed while parts ship.

---

## 2. The tiered architecture

### 2.1 Three rates

Microduck separates three, and the separation is the point:

```text
   ~50 Hz    reflex        ONNX policies, on-board, no network
                           robotd's tick; safety owns the only motor write handle

  ~15-30 Hz  perception    NPU detector -> bearing
                           tofd         -> distance     each question to the
                           BLE beacon   -> identity     sensor that answers it

   ~0.5 Hz   deliberation  the LLM, off-board, over WebSocket
                           sends intents; never trusted to execute them
```

**vision-picar collapses all three into one.** A single cloud `/navigate` call
is asked for bearing *and* distance *and* the next action, at 1-3s, and its
answer drives the robot.

`bearing-only` (M1, measured 2026-09-02) was the first half of un-collapsing
that: `/navigate` keeps *what* and *which way*, something else owns *how far*.
The lidar is that something else (§3.2).

### 2.2 One function becomes three

Today the seam is `vision_fn(frame) -> scene`, called once per tick, doing three
jobs:

```text
                     today                        tiered
  ─────────────────────────────────────────────────────────────────────
  perceive    what's in frame?        cloud        on-board
  decide      which way now?          cloud        local
  deliberate  where next, and why?    cloud¹       cloud, on trigger
```

¹ -- and barely. CLAUDE.md's own note: *"the single-step `/navigate` contract has
no memory of which way it already turned."* The deliberation tier is mostly
**not happening today**; `searched_rooms`/`room_guess` is the one thread of it,
bolted onto a per-step call.

### 2.3 The deliberation tier already has a name

**It is `brain/planner.py`** -- "NOT BUILT. Designed but never written to disk --
a `PlannerAgent` calling Claude with `MissionMemory.as_context()` as the
prompt," which CLAUDE.md calls the main hardware-path gap.

And `as_context()` **feeds nothing today** -- one test calls it, nothing else.
It formats `MISSION / MEMORY / CURRENT OBSERVATION / AVAILABLE TOOLS`, which is
a *deliberation* prompt, not a per-step one.

So tiering is not a new architecture. It is the reason the component this repo
already named has never had a job: with one call doing everything at one rate,
there was nowhere for a planner to sit. **Splitting the rates creates the slot.**

### 2.4 Trigger discipline -- where the cost saving lives

Tiering pays off only if deliberation is **event-driven, never periodic**.

| Trigger | Fired by | Why |
|---|---|---|
| Mission start | runner | there is no goal yet |
| Goal achieved | reactive | "I am at the doorway. Now what?" |
| Goal impossible | reactive | boxed in, or the target left frame and did not return |
| Room change suspected | reactive (**lidar geometry**, 1.13) | crossed a doorway; memory needs updating. Not a semantic classifier -- the transition is geometric, the label comes back on the reply |
| **Candidate sighting** | reactive | detector thinks it sees the target; cloud confirms identity and reachability |
| Staleness | timer | a goal older than N seconds is suspect |

The saving is exactly **reactive steps per goal**. It was derived here as ~8x
(sim runs took 40 steps; real walks reached target in 6-22 frames) and flagged
as underived-from-data. **§6.1 has now measured it: 4.1x pooled, 2.2x-5.7x by
mission length. Quote 4-6x.** Two things that estimate missed, both in §6.1:
the saving is bounded by the *event* rate rather than by the staleness timer,
and ~40% of a naive trigger count is `target_visible` / `room_guess` flicker
rather than events -- so **the trigger policy needs the same hysteresis
`lost_target` does**, or it fires on noise.

**The candidate-sighting trigger is the one that does real work**, not just cost
saving: it is what keeps a cheap local detector honest. The detector says
`backpack`; the cloud says whether it is the *red* one and whether the robot can
reach it.

### 2.5 Never block, and the degraded mode

Microduck's rule (`architecture.md` §2.4): the control path reads a cached
latest value and never waits on another service. Applied here:

- The reactive tier always holds a current goal; a new one arrives
  **asynchronously** and replaces it.
- Mission start is the one genuinely blocking call -- or it is not, if the
  opening default is "look around", which is safe and useful.
- **When the cloud is unreachable, `brain/agent.py` takes over.** The rule-based
  frontier explorer that `PLAN-sim-hardening.md` 2.2 says to keep and not extend
  becomes the *defined degradation*: no cloud, no planner, deterministic
  wall-following until the link returns.

That last is the only sensible answer to "what does the car do when Wi-Fi drops
mid-mission", and it costs nothing -- the code already exists and is tested.

### 2.6 What must not change

Three constraints a new perception process is exactly the sort of thing to
violate quietly:

1. **`brain/` and `robot/server.py` talk only to `RobotInterface`.** Whatever
   the reactive tier publishes arrives through the abstraction or alongside it,
   never around it.
2. **Safety is enforced server-side, always.** A bearing is an input to a
   decision, never a movement path of its own.
3. **`control/` may not import a backend or the simulator.** If the detector's
   output reaches the brain, it reaches it over HTTP like everything else.

### 2.7 Its own process

The open **"Perception in its own process"** row in
`PLAN-microduck-transplants.md` §1. Microduck's rule is **features, not
frames**: perception next to the sensor, publish derived features, control path
reads a cached snapshot non-blocking.

The failure mode it buys: **a stalled detector degrades perception rather than
adding jitter to motor control.** Here that is the difference between a wedged
`picamera2` capture and a `/stop` that still answers -- today undefined, and
exactly what M9 exists to test.

---

## 3. Mapping

### 3.1 Why there was no map

`MissionMemory` holds `visited_rooms`, `searched_rooms`, sightings and actions.
It holds **no geometry**. No pose, no occupancy grid, no "the kitchen is north
of here". The grid world has coordinates; a real house would not.

So a goal like "go to the kitchen" is **not executable** by a reactive tier that
has no idea which way that is. Only egocentric goals work:

| Goal form | Executable without a map? |
|---|---|
| `approach(backpack)` | **yes** -- bearing, servo toward it, collar stops it |
| `traverse(doorway at +0.4)` | **yes** -- same mechanism |
| `explore(bearing -0.6)` | **yes** -- a heading plus a stop condition |
| `go_to(kitchen)` | **no** -- needs a map |

**And the project had already hit this wall.** `ObjectSearchAgent.decide()` has
two branches: with grid `position` and `facing` -- *sim only* -- it does
frontier-preference exploration; without them, "which is what a real camera
frame will look like", it degrades to a right-hand wall-follower
(`AGENT-HARNESS.md` §3). That degradation **is** the no-map problem, met and
documented before this discussion started.

### 3.2 What the lidar changes -- more than "it adds a map"

The first reading of this was that a metric map is *not derivable* on this
hardware but *buyable*. True, and it undersold the purchase.

**A lidar solves the no-odometry problem, which was the harder half.**
Consecutive scans register against each other, so scan matching recovers
relative motion directly: **the lidar is the odometer.** Hector SLAM was
designed precisely for platforms with no wheel encoders. The chain

> no encoders -> no odometry -> no localisation -> no map

is not repaired link by link. The lidar replaces it.

Knock-on effects:

- **Clearance becomes metric and 360-degree** at ~10Hz instead of one ultrasonic
  beam. **This closes the Stage 0 gate** more completely than the ToF, any
  prompt wording, or any accelerator. The entire five-wording investigation was
  about a question a lidar answers directly.
- **"Go to the kitchen" becomes executable** -- a coordinate goal and a path,
  not a visual search.
- **The topological map stops being a substitute and becomes a labelling.**
  Rooms become regions of an occupancy grid; `room_guess` labels a place whose
  geometry is already known. Strictly better than the visual-edge design.

**All three of those arrive with (b)/(b+), not with the lidar.** Option (a) is a
clearance ring and has **no map at all** -- so under (a), `go_to(kitchen)` stays
unexecutable and navigation memory stays at room labels. Buying the sensor and
having a map are separated by however long you stay at (a). An earlier draft of
this section conflated the two.

**What it does not do: see a backpack.** A lidar reads geometry, never
semantics. Camera and lidar are complementary, not alternatives.

**What a 2D lidar still misses:** it sees one plane. Chair legs, not the seat.
Glass, mirrors and dark matte surfaces are unreliable. The camera still has to
cover everything above the scan plane.

### 3.3 The four ways to use a lidar

**(a) Clearance ring, no SLAM.** "At every angle, how far is the nearest thing."
No map, no idea where it is. Closes the gate. A driver and trigonometry -- days.
Fits the existing architecture entirely. **Chosen as the starting point.**

**(b) Lightweight scan-matching SLAM.** Draws a floor plan and roughly tracks
itself on it. Makes coordinate goals real. **Drifts** -- no loop closure, so a
lap of the house may not line up with itself. Fine for one session, poor for an
accumulated map. Weeks of work, stays hand-rolled.

**(c) ROS 2 + `slam_toolbox` / Cartographer.** Loop closure, a full nav stack,
durable reusable maps. **Swallows the project** -- its own IPC (DDS), build
system, node lifecycle and test model; typically wants Ubuntu, which may cost
the vendor library `robot/hardware_robot.py` is meant to be built on.

**(b+) ROS behind an HTTP wall. CHOSEN as the target if mapping proves to be
the point.** Run ROS 2 as *one isolated service* on the Pi exposing a tiny API
-- `GET /pose`, `GET /map`, `POST /goto` -- and nothing else in the system knows
ROS exists.

> This is exactly Microduck's `tofd` pattern: it owns one sensor, publishes,
> reads nothing, and consumers reach it through its own socket without knowing
> how it talks to hardware. Same rule, bigger sensor.

Gets loop closure and a real nav stack; keeps `RobotInterface`, the FastAPI
control plane, the tests and the brain. Costs one process boundary and enough
ROS to *configure* a node, not to rebuild in it.

**Note the chassis decision made (b+) and (c) more viable**, not less: the ROS
navigation stack assumes differential drive throughout. Under Ackermann it would
have fought back.

#### (a) is not a fork -- it is the first two weeks of (b+)

Whatever the destination, the opening is identical: assemble, **get scans into
Python and verify they are sane in the actual house** (glass, mirrors, mounting
vibration), then **wire the safety collar to them**. `robot/safety.py` still owns
the veto under (b+) -- nav2 plans on top of a safety layer, it does not replace
one.

So the (a) work is ~80% reusable: the driver, the `get_depth_grid()` feed, §5.1's
angular fix and the twin readout are all needed either way. **The choice is not
"(a) or (b+)", it is whether you stop at (a).**

And stopping there first is worth it on its own: layering SLAM onto scans you
have not validated is how a mounting-vibration problem gets debugged as a
mapping problem.

#### What (b+) costs, honestly

(a) is days; **(b+) is weeks, and most of it is learning rather than writing** --
TF frames, a URDF describing where the lidar sits relative to the wheels, nav2
parameters, launch files, `slam_toolbox` config. None of it hard; all of it
unfamiliar.

It also forces **an OS decision that (a) does not**. ROS 2's well-trodden path is
Ubuntu, and Raspberry Pi OS is what `picamera2` and `rpicam-apps` target:

1. **Ubuntu on the Pi** -- the standard ROS route, at the cost of the camera stack.
2. **ROS in Docker on Pi OS** -- keeps the camera tooling and, since
   2026-09-04, the Hailo driver too (`hailo-all` is a Pi OS package, 4.3);
   adds container plumbing for the lidar device and networking. **Probably
   right**, and it fits (b+)'s behind-a-wall shape.
3. Two boards. Overkill.

#### What (b+) changes about the shopping list

| | Under (a) alone | With (b+) as the near plan |
|---|---|---|
| **Encoder motors** | nice to have -- the lidar is the odometer | **required.** nav2's local planner wants wheel odometry fused with scan matching; scan matching alone is meaningfully worse |
| **IMU** | not needed | **desirable** -- ~$10, stabilises heading between scans |

### 3.4 Persistence

Decided in 1.5. The reasoning behind the two that matter most:

**Only the map persists.** `MissionMemory` is constructed at
`control/mission_runner.py:255`, held on the runner, kept in
`control/brain_server.py`'s `state`, running on ECS Fargate -- **RAM only, no
serialisation anywhere, one mission's lifetime, no copy on the Pi.** Persisting
it would buy mid-mission resumption that cannot be used: a robot that crashed
does not know where it is any more, which is the no-map problem again. The map
is what needs to outlive a mission, because the whole value of mapping a house
is that the second trip is cheaper.

**Tiering is what makes remote state affordable.** Per-tick memory access over a
WAN would violate the never-block rule. At goal boundaries -- 5-10 times a
mission -- a round trip is invisible. Same convergence as B5: whoever holds the
memory is the planner's client, and a stateless cloud planner plus Pi-side
memory survives the link dying with its memory intact.

### 3.5 Why differential

| | Ackermann (PiCar-X) | Differential |
|---|---|---|
| Rotate in place | **no** | yes |
| Best scan-matching input (pure rotation) | unavailable | available |
| SLAM examples / nav stacks | few apply | the assumed case |
| Recovery when stuck | reverse and arc | rotate and re-plan |
| Pan/tilt camera mount | included | **source separately** |
| Ultrasonic, motor driver, battery | integrated | assemble |
| `HARDWARE-READINESS.md` | written for it | **needs revision** |

---

### 3.6 What it costs

Approximate US prices, from general knowledge on 2026-09-03. **Not verified
against a retailer**, and several move a lot with sales. Every figure here
should be checked before ordering.

**Essential -- the robot does not work without these**

| Item | ~USD | Note |
|---|---|---|
| Raspberry Pi 5 (8GB) | 80 | If one is not already owned |
| Active cooler | 8 | The Pi 5 throttles without it, and SLAM is a sustained load |
| microSD 64GB A2 | 12 | See the SSD row below |
| **Slamtec RPLidar C1** | 99 | 1.2 |
| Differential chassis kit, **encoder motors** | 69 | Yahboom 2WD, chosen -- 3.8 |
| Motor driver board | 0-30 | **Included** with the Yahboom kit -- IC unconfirmed (3.8). **Under 1.14 the recommendation is the Waveshare General Driver for Robots (~30) instead**: its **ESP32** runs the velocity PID off Linux's scheduler, which a 20-50Hz loop in CPython cannot do reliably. Verified 2026-09-06 to carry TB6612FNG, encoder inputs for 2 motors, a **9-axis IMU** (QMI8658C + AK09918), a lidar interface, current monitoring and 7-13V input taking the 3S pack directly. **It collapses the separate IMU line below.** Cost is a new Pi-to-ESP32 serial protocol, which becomes a seam under `RobotInterface` -- and 1.16 #6's fourth deadman |
| Camera Module 3 | 30 | Any CSI camera works now; autofocus. 1.10 |
| **Hailo-8L, M.2 module form** | 70 | The on-board detector -- 1.10, §4, and 4.9 configuration C/D. Replaced the AI Camera 2026-09-04. Briefly the 10H at 130 on 2026-09-06, reverted the same day on its measured tokens/s (4.9). **The module, not the soldered AI HAT+** -- it is the form that survives a Jetson pivot and the form the NVMe needs anyway. The AI Kit that used to bundle it is out of production, so this is a standalone module plus a carrier. 4.8 and 4.9 argued for leaving this off the first order; **1.14 reversed that the same day** -- continuous motion puts the perception tier above what the Pi's cores deliver, so it ships with the first order |
| 2-axis pan/tilt bracket + servos | 12 | Replaces what the PiCar-X bundled. **Both axes are kept -- 1.15.3.** Bracket measures 32 x 28 x 65mm and takes a 28x28mm camera, which the Camera Module 3 fits. **Two open questions**: the Waveshare board's PWM output does **not** support MG90S/SG90-class servos (drive them from the Pi's own GPIO instead), and 1.16 #7 asks whether bearing-critical axes justify **ST3215 bus servos** with position feedback (~25 each, driven natively by that board) |
| **3S** Li-ion pack + charger | 35 | Motor rail only (1.3). **Not 2S** -- see the correction below |
| Wiring, connectors, switch, XT60 | 15 | |
| Standoffs, M2.5/M3 hardware | 10 | For stacking decks |
| | **~440** | Chassis priced down (3.8); camera + Hailo replace the AI Camera, +30 (1.10) |

**Strongly recommended -- each avoids a failure already discussed here**

| Item | ~USD | Avoids |
|---|---|---|
| Powered USB hub | 15 | The 600mA USB cap browning out the Pi (1.3) |
| IMU (MPU6050 / BNO055) | 0-10 | Heading drift between scans -- **matters much more under (b+)**, and more again under 1.14, where heading error integrates for as long as the robot drives. **0 if the Waveshare board is taken**: verified 2026-09-06 to carry a 9-axis QMI8658C + AK09918, which is better than the part specced here |
| **2x VL53L1X ToF, forward-down** | 12 | The under-plane residual 1.15.4 names -- cliffs, thresholds and low obstacles the scan plane misses |
| **Compliant bumper + microswitches** | 5 | 1.15.4. **Not optional under 1.14**: everything else in the safety chain is an inference, and this is the only measurement. The cheapest guard in the build |
| 5V buck converter | 8 | If the Pi is ever taken off the bank and onto the pack |
| **Lidar pedestal**, 3D printed | 15 | 0 with a printer. **Not a bracket -- it has a job (1.15.2)**: hold the scan plane as low as it can go while still clearing the camera's swept envelope at full tilt. Too low blinds the lidar in the forward arc; too high grows the under-plane blind volume |
| Jumper wires, misc | 10 | |
| | **~75** | |

**Worth considering**

| Item | ~USD | Why |
|---|---|---|
| NVMe SSD + dual-slot PCIe base | 83 | **SD cards corrupt on brownout**, which is the exact failure 1.3 is written about. The one item here that prevents losing work rather than an annoyance. **The Hailo takes the Pi's one PCIe lane** (1.10 item 2), so this means the M.2-module form of the Hailo-8L on a dual-slot switch board plus the drive, not the plain M.2 HAT at 45. **Now vendor-documented rather than rumoured** (4.9): the Pineboards HatDrive! Dual, ~48, carries an ASM1182e PCIe switch and names Hailo support -- but it is **discontinued at some retailers, so check stock**, and its slots are 2230/2242 only, so the drive must be short. Drive ~35 |

**Totals**

| Scenario | ~USD |
|---|---|
| Essential only | 440 |
| **+ recommended** | **515** |
| + NVMe and the dual-slot base | 598 |
| Already own a Pi 5 | subtract ~100 |
| ~~Accelerator deferred~~ | **No longer on offer** -- 1.14 item 2 |

Already owned, 0: the power bank (1.3). **Budget ~515-600**, and the two
variables that move it are whether a Pi 5 is already owned and whether the NVMe
plus its dual-slot base is taken. **Raised ~17 on 2026-09-06** by 1.15.4's ToF
pair and bumper; the Waveshare board and the IMU it absorbs roughly cancel. **The accelerator is no longer one of them:
4.8 and 4.9 had it as deferrable, and 1.14 put it back on the first order.** Was ~460-510 under the IMX500 (2026-09-03): the Hailo decision
added ~30 to the essentials and ~30 to the NVMe line (1.10). Before that,
revised down from ~450-500 once the chassis was priced against real listings
rather than estimated (3.8).

**Correction, 2026-09-03: the pack is 3S, not 2S.** The first draft of this
table specced a 2S (7.4V) pack. Every serious differential chassis surveyed --
iDili R3, Yahboom's 520-motor kits -- ships **12V** gear motors, which run
underpowered and sluggish on 7.4V. **3S (11.1V)** is the right pack. Found by
pricing real products rather than by reasoning, which is the argument for
pricing real products.

**Two line items may collapse into one.** A combined motor-driver board -- the
Waveshare General Driver for Robots is the example that came up, ~30 -- can
carry the TB6612-class driver, the encoder inputs, servo ports **and an IMU**
on one board. If it does, it replaces the separate motor driver and the
separate IMU together. **Verify the IMU is present before relying on it**; the
saving is real but the claim is second-hand.

**A chassis's own battery may replace the pack.** Some kits (the iDili R3) ship
a protected lithium pack. Check its voltage against 1.3's two-rail plan before
buying a second one.

### 3.7 Two ways to buy this, and the second was nearly missed

The table above prices **parts, integrated by hand**. There is a second path,
and it was almost filtered out of a market survey on a false premise -- that a
lidar had already been bought, when nothing has been ordered at all.

**Complete ROS 2 platforms** (Yahboom ROSMASTER, Hiwonder LanderPi and similar)
ship chassis, encoder motors, driver, lidar and often a depth camera as one
unit, with working launch files, a URDF, TF frames already configured, and
documentation. Roughly 400-700.

**The comparison is not really about money.** (b+)'s cost was named in 3.3 as
*weeks, and most of it learning rather than writing* -- TF frames, a URDF,
nav2 parameters, launch files. A platform that boots into a working nav2 stack
deletes most of that, and against ~465 of separate parts plus the integration
it may be the cheaper path in the only currency that is scarce here.

**What it costs instead:** less learned by assembling it, possible lock-in to
the vendor's software stack, and the lidar is inherited rather than chosen --
which would reopen 1.2.

**Surveyed 2026-09-03, and the path is closed.** Every bundled ROS 2 platform
findable on Amazon US fails on drive geometry or on compute topology:

| Platform | Why it fails |
|---|---|
| Yahboom ROSMASTER X3 / M1 / M3 | mecanum |
| Yahboom ROSMASTER R2 / A1 | Ackermann |
| Hiwonder LanderPi | mecanum / Ackermann / tank only; off-Amazon; $732-1046 |
| Hiwonder Tank, Swaytail MC400 | tracked; mecanum |
| **Yahboom MicroROS-Pi5** ($180, all-in) | **compute topology -- see below** |

**The MicroROS-Pi5 came closest and fails on the one thing this project cannot
concede.** It is a good kit -- 4x MD310Z20 quadrature encoder motors, an ORBBEC
MS200 lidar included, anodised aluminium, a six-axis IMU on its ESP32S3 board,
ROS 2 Humble and Python 3 confirmed. But it is "designed to run ROS 2 on a PC
via WiFi UDP, not directly on a Pi 5 -- your Pi 5 would act as a remote compute
node, not an onboard controller."

That contradicts `HARDWARE-READINESS.md` §7, which names "the robot only works
when the laptop is on, awake, and on the same Wi-Fi" as the reason the brain
moves onto the Pi at all, and it fails `CLAUDE.md` §7's Stage 4 done-when
outright: *start a mission with no laptop on the network at all.*

**The technical caveat does not rescue it.** micro-ROS's agent is only a
process, so running it plus nav2 on the Pi 5 is probably possible against the
vendor's documented setup. But the kit's whole value was that its documentation
deletes the learning curve; deviating from its documented architecture forfeits
exactly that. Technically possible, strategically pointless.

**So: parts, integrated by hand.** 3.8 records what was chosen and what is still
unverified.

**One specification the survey added that this document had not:** a
**circular** footprint. The stated reason -- fewer lidar shadow zones -- is
weak, since the lidar sits on a raised plate and the chassis barely occludes
it. The real reason is nav2: a circular robot has a constant turning radius, so
its costmap footprint is a single `robot_radius` rather than a polygon, and
**rotating in place is always safe**. A rectangular robot sweeps a circle wider
than itself when it pivots and can clip a doorframe it is nominally clear of.
Given 1.1 chose differential drive *specifically to rotate in place*, that is
worth more than it first appears -- though not unconditionally worth a large
price premium from an unknown vendor with no ROS 2 community.

---

### 3.8 The chassis, chosen -- and what is still unverified

**Yahboom 2WD encoder chassis (~$69) + Slamtec RPLidar C1 (~$99) = ~$168.**

| | Yahboom + C1 | iDili R3 + C1 + driver |
|---|---|---|
| Total | **~168** | ~279, plus a top plate to source |
| Motor driver | **included** | not included |
| Pi 5 as onboard controller | yes | yes |
| Footprint | rectangular, 228 x 148mm | **circular**, 205 x 192mm -- 3.7's nav2 win |
| Payload | ~2kg | 3kg |
| Vendor | established, ROS 2 resources | unknown, very recent listing |
| Shipping | normal | Sep 22 - Oct 1 |

The circular footprint is genuinely the better nav2 chassis. It costs **$111
more, arrives with no motor driver and no top plate, and comes from a vendor
with no track record**. Not worth it at that price; revisit if the Yahboom's
deck proves too cramped.

**A note on the deck.** The survey marked 228 x 148mm as passing a "15 x 15cm"
floor. It does not, literally -- 148 < 150. It passes in practice, because a Pi
5 is 85 x 56mm and the lidar needs its own raised plate regardless, so the
binding dimension is never 148mm. Worth recording that the checkmark was
applied without checking.

#### What the vendor's own documentation settled

Amazon listings had, across four rounds, failed to answer the questions that
actually gate the purchase. Yahboom's product pages answered two of them.

**The encoder question -- the one that could have made a $69 chassis the wrong
buy -- is answered: yes.** A motor driver that spins motors but never counts
pulses leaves the encoders decorative and nav2 with no wheel odometry, which is
the requirement that disqualified every other chassis. Yahboom's encoder motor
drive module carries an **STM32F103RCT6 coprocessor that both drives the motors
and obtains encoder data**, and talks to the host over **I2C or serial**. That
is the right shape: an MCU counts edges in real time and hands the Pi a number.
**Unverified:** whether the board bundled in the $69 kit is that same module.
The listing says only "Expansion Board Driver". One question to the seller.

**3S is confirmed by the vendor, not inferred.** Yahboom: *"If you want to
connect 520 motor, please choose a 12.6V power supply."* 12.6V is a charged 3S
pack. 3.6's correction was made by reasoning from motor ratings; this is the
manufacturer saying it.

**Two Amazon variants exist** -- `B0F3CZ3WYB` (no battery, the ~$69 one) and
`B0F3CYDQ21` (**with battery**). Since a 12.6V pack is needed anyway, check
whether the bundled one qualifies: it could remove a ~$35 line item and the
voltage-matching problem with it.

**No IMU.** Yahboom lists the MPU6050 as an add-on module, not bundled, so
3.6's separate IMU line stays.

#### The finding that reaches back into 4.4

Yahboom's own figures: **L-type 520, 1:40 reduction, ~300 RPM after
reduction** (the market survey said 1:30; the vendor says 1:40). On the 65mm
wheels these kits ship, that is roughly **1 m/s top speed**.

Which lands exactly on the worst row of 4.4's reaction-budget table: at 100cm/s
a 200ms detector latency consumes **the entire 20cm collar before a decision is
made**.

**The chassis can outrun its own reaction budget.** Not a defect -- indoors it
will run at a fraction of that, and nav2 caps velocity regardless. But it makes
the **velocity cap a safety parameter rather than a comfort setting**, and it
gives 4.4's explicitly-unmeasured "how fast does the car actually move" a
concrete upper bound to design against, months before any hardware arrives.

#### Still unverified before ordering

1. Whether the bundled expansion board is the STM32-based encoder module.
2. **Encoder CPR/PPR.** "High-precision Hall encoder, built-in shaping and
   pull-up, direct square-wave output" -- but no number on any page. Likely in
   the tutorial documentation.
3. Which motor driver IC (TB6612FNG preferred over L298N).
4. Whether the with-battery variant's pack is 12.6V/3S.
5. Whether the expansion board mounts on the 40-pin header or wires to it --
   the AI HAT+ occupies the HAT position (1.10 item 7).
6. **Which motor RPM variant ships** (added 2026-09-06). Yahboom sells several
   and the figure is printed on the motor label. On a 65mm wheel the 520's
   quoted 333 RPM is **1.13 m/s** top speed, which covers 1.14's spec-for-0.5,
   run-at-0.3 comfortably (26% and 44% duty). **A 130 RPM variant tops out at
   0.44 m/s and fails the spec.** This is the one question on this list that
   cannot be fixed after delivery.
7. **The motors' running and stall current** (added 2026-09-06). TB6612FNG is
   ~1.2A continuous per channel. Under 1.14's continuous driving that draw is
   sustained rather than bursty -- probably fine on hard floor at 26-44% duty,
   marginal near stall or on carpet.

None is a blocker except 6; all seven are one email to the seller.

---

## 4. The on-board detector: IMX500 vs Hailo vs Jetson

**Decided 2026-09-04: Hailo-8L.** This section used to be headed "Accelerator
options -- now probably unnecessary", and its conclusion has changed twice:
first to "buy the sensor" (the IMX500, 2026-09-03), then to the Hailo. The
reaction-budget argument in 4.4 survived both and is referenced from 3.8; the
rest is rewritten around the comparison that actually decided it.

### 4.1 The requirement that changed the answer

The 2026-09-03 evaluation asked one question: *what is the cheapest way to get
a fresh bearing on a COCO-class target?* The IMX500 won it cleanly -- zero Pi
CPU, no PCIe slot, one part instead of two.

The 2026-09-04 evaluation added a second: **the hardware must leave room to
experiment with models beyond what fits on the IMX500, including ones pulled
from Hugging Face.** That is a requirement about the *ceiling*, and the
IMX500's ceiling is silicon.

### 4.2 The finding that lowers the risk -- still true

**`backpack` and `bottle` are both COCO classes.** The Stage 0 targets are in
the standard 80-class label set essentially every off-the-shelf detector
predicts, so a stock pre-compiled YOLO11n finds them with **zero training, zero
calibration set, zero distillation**. Fine-tuning becomes the project only past
COCO's 80 -- and on the Hailo that is a documented retraining container plus a
compile, not a research project. *(Confirm against the model's own label
file.)*

**It does not give you the colour.** COCO says `backpack`, not `red backpack`.
The first draft proposed an HSV check on the crop. **Superseded by CLIP over
crops** (1.10): the Hailo model zoo carries the CLIP image encoder, so each
detected crop is scored against the mission's own target string. The same
mechanism handles "the blue bottle" and "my backpack, not the other one".

### 4.3 The Hailo parts, and which one

| | AI HAT+ (Hailo-8L) | AI HAT+ (Hailo-8) | AI HAT+ 2 (Hailo-10H) |
|---|---|---|---|
| Rated | 13 TOPS INT8 | 26 TOPS INT8 | ~40 TOPS, plus 8GB of its own LPDDR4X |
| Price | ~$70 | ~$110 | **$130, shipping** -- verified 2026-09-06 (4.8) |
| Architecture | dataflow, no external memory, weights streamed from the host | same, larger | on-module memory, built for LLMs and transformers |
| Runs well | CNN detection, segmentation, pose, depth, classification; CLIP via Hailo's port | same, faster or at larger inputs | the above plus small language and vision-language models |
| Form | the Pi 5's single PCIe FFC connector, HAT position | same | same |

**The 8L.** The Hailo-8 buys nothing here: a nano or small detector already
runs above camera frame rate on the 8L, and 4.4's budget is bounded by the
camera's 30fps. The 10H is the only alternative worth a look, and only for the
VLM question -- 1.10 item 6 says how much looking.

`sudo apt install hailo-all` on Pi OS brings the driver, HailoRT, the
GStreamer bits and `rpicam-apps` with Hailo post-processing; `picamera2` ships
Hailo examples that return boxes directly; pre-compiled YOLO11n HEFs exist for
both 8-series parts. That is the turnkey starting point, and it keeps Pi OS --
one more reason (b+)'s OS decision lands on "ROS in Docker on Pi OS" (3.3).

#### What "experiment with Hugging Face models" means on a Hailo

There is no `pip install` path. Every model goes **PyTorch -> ONNX -> Hailo
Dataflow Compiler -> HEF**, on x86-64 Ubuntu, with a Hailo developer account
and a calibration set of a few hundred representative frames for INT8
quantisation. A GPU speeds the optimisation step but is not required. On a Mac
the practical compile host is an EC2 instance for an hour per model. DeGirum's
cloud compiler and hosted Hailo zoo would remove the local toolchain entirely
-- **unverified**.

| | On the 8-series |
|---|---|
| **Compiles well** | the YOLO family incl. v8n/11n and the `s`/`m` tiers, EfficientDet-Lite, NanoDet, YOLOX, CenterNet, MobileNet / ResNet / EfficientNet backbones, DeepLabv3+, FCN, STDC; RT-DETR reported in newer zoo releases (**verify**) |
| **Via Hailo's own ports** | CLIP ViT-B/32 and ResNet-50 image encoders (text encoder on the Pi CPU); FastDepth / SCDepth monocular depth; OSNet / RepVGG re-identification embeddings |
| **Does not fit** | anything attention-heavy: DINOv2, SAM, Depth Anything, Grounding DINO, OWLv2, any VLM or LLM. The dataflow design has no efficient attention path and no memory for the weights |
| **The 10H's job** | exactly that gap. Which models, and how well, is the thing to verify before paying $60 more |

So: **a Hailo-8L satisfies the requirement for the CNN detection,
segmentation, depth and classification families, and for CLIP. A 10H probably
extends it to small VLMs. Neither gives you arbitrary Hugging Face models** --
only a Jetson does (4.7).

#### Models worth trying first, and why

Within the size class either chip holds, YOLO is already at the accuracy
frontier among CNN detectors: EfficientDet-Lite, NanoDet, YOLOX and CenterNet
are peers, not upgrades. "More capable than YOLO" therefore means one of two
things, and the Hailo allows both where the IMX500 allowed neither:

- **A bigger YOLO.** YOLO11s or m at 640 or 1024 input is what improves recall
  on a small target across a room. The IMX500 was capped at the nano tier.
- **A different kind of model.** Floor segmentation (DeepLabv3+, Fast-SCNN,
  STDC, YOLOv8-seg) gives a per-pixel drivable-area mask from one frame -- the
  obstacle question five prompt wordings failed at, answered geometrically.
  It does not replace the lidar, which sees a chair leg the mask cannot, but
  it is the first experiment worth running. Monocular depth (FastDepth,
  SCDepth) is relative, not metric, so no use for the collar; beside the
  floor mask it flags a drop or a low obstacle the lidar plane misses.

### 4.4 The reaction-budget bar

**An earlier recommendation contained a circular argument and is withdrawn.**
It said to benchmark YOLO11n on the Pi 5 CPU and concluded *"if it clears ~10
FPS, no accelerator was ever needed for a 1-3s loop"* -- judging the edge option
against the cloud latency the edge option removes.

The bar comes from **how far the car travels before it can react**:

```text
reaction budget = clearance to preserve / speed
```

At 10 FPS, worst-case latency is ~200ms (100ms inference plus up to 100ms
waiting for the next frame):

| Speed | Travel in 200ms | Against a 20cm collar |
|---|---|---|
| 30 cm/s | 6cm | comfortable |
| 60 cm/s | 12cm | tight |
| 100 cm/s | 20cm | the whole collar, before deciding |

**The binding number is unmeasured: how fast the car actually moves.** 3.8
gives it an upper bound of ~1 m/s from the vendor's motor figures, which is the
worst row. It belongs on the hardware-day pre-flight list.

Two things relax the bar. **The range sensor owns emergency stop, not the
camera** -- so the detector's latency budget is about steering, not collision.
And **motion is discrete today** (speed 50 for 0.5s per move), capping decisions
near 2Hz. The FPS question only sharpens with *continuous* driving, which is a
design choice not yet made. **Made 2026-09-06: continuous (1.14).** So the
second relaxation above expires, this section's CPU-only conclusion expires with
it, and the accelerator moves onto the first order -- 1.14 item 2.

**On the Hailo-8L the bar is not close.** A nano detector runs at well over
camera rate with single-digit-millisecond inference, so the 200ms row is the
CPU-only case, not the shipped one. **Stock YOLO11n on the Pi 5's own cores is
still the day-one baseline** -- it needs no HEF, and it measures the real
reaction budget on the real chassis before any accelerator is trusted.

### 4.5 Why "accelerator possibly never" was wrong, and what was right in it

**Buying the lidar did weaken the case for an accelerator as a speed device.**
Obstacle avoidance moved to the lidar; navigation moves to the map; visual
edge-matching disappeared (1.6, 3.2). The only remaining job for on-device
vision is a fresh bearing, which is low-rate and not safety-critical. That
reasoning stands.

**What it missed is that the accelerator is a capability device here, not a
speed one.** The reasons the Hailo is in the bill (1.10) are what it can hold
-- the larger YOLO tiers, a floor mask, depth, CLIP, several at once -- and
that the same HEF can be scored on the recorded corpus before it drives. None
of that is about frames per second, which is why "the loop is slow anyway"
never bore on it.

### 4.6 The three-way comparison

Against the job the plan gives the detector -- find a target, report its
bearing at camera rate, leave range to the lidar and identity to the VLM --
plus the requirement in 4.1.

| | IMX500 AI Camera | Hailo-8L AI HAT+ | Jetson Orin Nano Super | Pi 5 CPU only |
|---|---|---|---|---|
| Cost | ~$70, replaces the ~$30 camera | ~$70 (8L) / $110 (8) / $130 (10H) plus a ~$30 camera -- **4.9 prices the whole family and picks the 8L as a module** | **~$399-480** replacing the ~$80 Pi 5, plus camera and power conversion -- repriced July 2026, was ~$249 | $0 |
| Where inference runs | on the sensor | on a PCIe module | on-board GPU, CUDA | the four A76 cores |
| Pi CPU cost | ~0 | a few percent of a core | n/a | one to two cores for a nano model |
| Model ceiling | ~8MB on-chip, nano class, one at a time | hundreds of MB, several resident; CNNs only | anything that fits 8GB shared with the OS | limited by speed, not memory |
| Custom / HF models | Sony toolchain, or Ultralytics `format=imx` for YOLO | ONNX -> DFC on x86 Linux, per model | `transformers` just works | plain PyTorch / ONNX, no compile step |
| Open-vocabulary | no | CLIP re-ranking over crops | YOLO-World, OWLv2, Grounding DINO at usable rates | too slow to matter |
| Score on recorded walks | **no** -- cannot be fed a stored frame | yes | yes | yes |
| Camera choice | fixed: the IMX500 itself, fixed focus | any CSI or USB camera | IMX219 / IMX477 out of the box, others need drivers, **no IMX500** | any |
| Extra power | well under 1W | ~1.5W typical, peaks higher | 7 / 15 / 25W modes, 9-19V input, no 5V USB | cores at load |
| PCIe slot | free for NVMe | **taken** -- 1.10 item 2 | own NVMe slot | free |
| OS | Pi OS | Pi OS (`hailo-all`) | Ubuntu 22.04 / JetPack | Pi OS |

**Read down the "model ceiling" and "score on recorded walks" rows and the
decision is there.** The IMX500 loses on both; the Jetson wins on both and on
open vocabulary, and loses on cost, power and the camera stack; the Hailo-8L
is the compromise that keeps the Pi plan intact.

### 4.7 The Jetson path, kept for later

**Ruled out for now, 2026-09-04** -- not on capability, where it is plainly the
strongest, but on what it costs the rest of the plan. Recorded so the
re-evaluation does not start from zero.

The part is the **Jetson Orin Nano Super Developer Kit**, launched at ~$249
list: the older Orin Nano 8GB kit with a firmware and JetPack update that raised
the clocks and halved the price. Supply has been tight since; expect reseller
markups. **Repriced to $399 in July 2026 with no announcement, ~$480 street**
(4.8) -- every dollar figure below predates that and is corrected there.

| | Pi 5 (8GB) | Jetson Orin Nano Super |
|---|---|---|
| CPU | 4x Cortex-A76 @ 2.4GHz | 6x Cortex-A78AE @ 1.7GHz |
| Inference | none on-board | 1024 CUDA cores, 32 tensor cores, ~67 TOPS INT8 sparse |
| Memory | 8GB | 8GB LPDDR5, **shared** CPU/GPU |
| OS | Pi OS | Ubuntu 22.04 via JetPack 6 |
| Power | ~5W, 5V USB-C | 7 / 15 / 25W, 9-19V barrel jack |
| Storage | microSD, or NVMe via the one PCIe lane | microSD plus a dedicated M.2 Key M slot; Wi-Fi on a Key E slot, included |
| Cameras | full Pi camera stack | 2x CSI; IMX219 / IMX477 out of the box, others need vendor drivers |
| Size / mass | 85x56mm, ~50g | ~103x91x35mm with fan, ~175g (**verify**) |

**What it would change in this plan.** 1.10's detector becomes a TensorRT model
on the GPU and the COCO limit dissolves -- YOLO-World, OWLv2 and Grounding DINO
tiny run at usable rates, so the detector can match the mission's actual target
string. A local 2-3B VLM (Qwen2-VL-2B, Moondream, SmolVLM, via
`jetson-containers` or Ollama) becomes a genuine offline deliberation tier --
**not** a replacement for Opus 4.5, since Stage 0 shows even Sonnet 4.5 stalls
on the navigation question, but an upgrade over the rule-based wall-follower as
the degraded mode (2.5). 3.3's (b+) OS dilemma dissolves: JetPack is Ubuntu
22.04, ROS 2 Humble's native platform, and Isaac ROS adds GPU perception nodes.
1.3's power plan has to change: a USB-PD trigger board pulling 12-15V from a PD
bank, a filtered buck from the 3S pack, or a second pack -- the 3S range sits
inside the Jetson's input range, but running it directly puts motor noise on
the compute rail, which 1.3 forbids. Net bill change roughly **+$170-220** at the launch price, and
**+$320-430** at the July 2026 one (4.8).

**What ruled it out.** 8GB shared memory is the binding limit -- SLAM, nav2, an
open-vocabulary detector and a local VLM will not all be resident, so the plan
would have to pick two. JetPack upgrades and NVMe flashing go through NVIDIA's
SDK Manager, which needs an x86 Ubuntu host (initial microSD setup works from a
Mac). The camera ecosystem is thinner: `nvarguscamerasrc` or V4L2, and anything
beyond IMX219 / IMX477 means a vendor driver and a device-tree overlay. PyTorch
comes from NVIDIA's wheel index, not plain pip. The fan is always on under
load. And nothing in the code cares: the brain talks to `RobotInterface` over
HTTP, so the swap is a board change, not a rewrite -- which is exactly why it
can wait.

**When to re-open it.** Two tests, either one a yes: the robot must run an
open-vocabulary detector for the mission's target string with no cloud in the
loop; or a new Hugging Face model needs to run *on the robot* most weeks. Until
then experiments run off-robot behind the perception seam (2.7), fed by the
robot's frames or the recorded walks, and are promoted to the Hailo by compile
once they have earned it in replay.

### 4.8 Re-examined 2026-09-06: what the industry runs, and two price moves

**The decision does not change -- Pi 5 plus a Hailo -- but three of its inputs
do, and one of them flips a choice this plan had already pre-registered.**
Prompted by a direct question: is the Hailo a regrettable spend against a later
pivot to a Jetson?

#### The two price moves

| | 2026-09-04, as written | 2026-09-06, checked |
|---|---|---|
| Jetson Orin Nano Super devkit | ~$249 list | **$399 list**, ~$480 street. NVIDIA repriced the whole Jetson line in July 2026 with no announcement -- Orin NX 8GB module $399 -> $649, up to 101% across the range. LPDDR pricing is the implicated cause, so it may not be permanent |
| AI HAT+ 2 (Hailo-10H) | "~$130, announced late 2025, **verify availability**" | **$130, shipping.** 8GB of its own LPDDR4X, 40 TOPS INT4, named models (Llama 3.2 1B, Qwen2.5 1.5B, DeepSeek-R1-Distill 1.5B), a `hailo-ollama` backend, documented LoRA fine-tuning |

**4.7's "+$170-220" for the Jetson path is wrong and is corrected to
+$320-430.** The Jetson did not become a worse board; it became a 1.6x more
expensive one, against a plan whose entire compute budget is ~$150.

**And 1.10 item 6's pre-registered test is now satisfied.** It said: *if the
Hailo-10H is in stock at ~$130 and its small-VLM support is documented for real
models, the extra $60 turns a detection sandbox into a broader one. If either is
unverified at ordering time, take the 8L.* Both conditions now read yes, so
**the order is the AI HAT+ 2, not the 8L.** Writing that condition down in
advance is what turned this into a lookup instead of an argument.

**Amended hours later by 4.9, and the amendment is the more useful half.** The
test passed on its own terms, but its *purpose* was to buy a broader sandbox,
and the 10H's published generative numbers -- 5.89 tokens/s on a 1.5B model --
make a local VLM slower than the cloud call it would replace. **The order goes
back to a Hailo-8L, in M.2 module form.** A pre-registered test is only as good
as the quantity it names, and this one named availability where it should have
named throughput.

#### What the delivery-robot companies actually run

| Company | On-board compute | Notes |
|---|---|---|
| **Coco Robotics** (Coco 2, Feb 2026) | **Jetson Orin NX** | Explicitly "without needing the cloud". Trained in Isaac Sim / Isaac Lab on Cosmos-generated synthetic data |
| **Serve Robotics** (Gen3) | **Jetson Orin** | 5x the Gen2 Xavier; the stated wins are hardware video encode and 12h of battery |
| **Cartken** | **Jetson AGX Orin** | six cameras for mapping and navigation plus wheel odometry, running SLAM |
| **Kiwibot** | Jetson TX2 -> Xavier | sidewalk-centring, obstacle avoidance, traffic-light recognition |
| **Starship** | Tegra TK1, then **x86 AMD Ryzen** + an FPGA | the exception -- left the NVIDIA path entirely. Some signal processing is too time-sensitive even for the CPU |

**The unanimity is real, and four of the five reasons for it do not apply
here.**

- **Multi-camera visual odometry.** Cartken runs six cameras through SLAM. This
  project has one camera and a 2D lidar, and 3.2 already hands the geometry to
  the lidar.
- **Outdoor, safety-critical, at pedestrian speed.** Their perception owns
  collision avoidance among people. Here the lidar and the collar own it (1.11:
  *"Will I hit something?" -- lidar + collar -- no vote, ever*), and 4.4's
  reaction budget is set by discrete ~2Hz motion indoors.
- **CUDA as an ecosystem, not as TOPS.** Isaac ROS, cuVSLAM, Isaac Sim, GPU
  nav2 nodes. They are buying a stack, not an accelerator. (b+) here is
  `slam_toolbox` and nav2 on CPU, which is what a 2D lidar map costs.
- **Fleet economics invert the per-unit comparison.** $400 against a $10k robot
  amortised over thousands of units, with an NRE budget behind it, is noise.
  $400 against this bill of materials is most of the robot. **A fleet buys the
  ceiling once and uses it thousands of times; one robot pays for it once and
  uses it once.**

**The fifth reason does apply, and it is the finding worth keeping: hardware
video encode.** Every teleoperated fleet above streams camera video to a remote
operator, and NVENC is why a Jetson is the natural home for that. **The Pi 5 has
no hardware H.264 or H.265 encoder at all** -- it lost the one the Pi 4 had, and
only decodes; a software 1080p stream is roughly the four A76 cores. If a live
operator video feed ever becomes a requirement here, that is a **Pi 5 problem
that no accelerator fixes** -- neither the 8L nor the 10H encodes video. It is a
reason to move to a Jetson, and the only one on this list that could arrive
without warning.

#### The one thing that does transfer, and it argues *for* deferring

All five run perception on-board and treat the network as optional. That is the
endgame this plan describes too (2.5's degraded mode, 4.7's re-open tests) --
but note **how they got there**. Coco ran *teleoperated*, with remote humans
taking control in under 300ms, for years before Coco 2 moved the intelligence
on-board. The remote intelligence came first and the silicon followed the
evidence.

That is this project's architecture at a different latency scale: a cloud VLM as
the deliberation tier, a local collar owning safety, and an on-board tier that
has to earn its place. **The industry's trajectory is an argument for buying the
accelerator late, not for buying a bigger one early.**

#### Pi-only, stated fairly

The plan has treated "no accelerator" as the null option. It is stronger than
that as a *starting* position. Stock YOLO11n on the Pi 5's own cores runs at
roughly 5-13 FPS depending on export path -- ONNX INT8 at the low end, NCNN and
quantised builds at the high end -- which **already clears 4.4's reaction bar**,
because motion is discrete at ~2Hz and the range sensor owns the emergency stop.
Pi-only fails on the **ceiling** (4.1), never on the rate. 4.4 said so; the
numbers agree.

The consequence is a sequencing fact worth more than either chip. **The
accelerator is the only line in 3.6 that is purely deferrable.** It changes no
other decision -- same chassis, same lidar, same camera, same two power rails,
same Pi OS, same code -- and it plugs into a machine that will already be
running. Every other item on that list is load-bearing on hardware day.

#### So: is it a regrettable spend?

Three questions, and only the first is about money.

**1. Does the hardware transfer? Yes -- if you buy the module, not the board.**
HailoRT runs on aarch64 as well as x86-64, and a Hailo-8 M.2 module has been
brought up on a Jetson Orin Nano Super under JetPack 6.2 / Ubuntu 22.04
(community-reported, **unverified here**). The Orin Nano has an M.2 Key M slot,
and the Hailo-10H is sold as a standalone Key-M module as well as soldered onto
the AI HAT+ 2. **The soldered HAT does not move; the M.2 module does.**

**This collapses into a decision 1.10 item 2 was going to make anyway.** The
PCIe lane is contested between the accelerator and the NVMe, and the resolution
already on the table was the M.2-module form on a dual-slot switch board (~$40).
That same purchase shape is the one that survives a Jetson pivot. **One
decision, two problems** -- and it is the whole difference between a part that
moves and a part that does not. *(Verify the switch board against the 10H
specifically: 3.6 flags the combination as unverified even for the 8L, the
standalone 10H module's price is not confirmed and may exceed the $130 HAT, and
its power and thermal envelope is its own question.)*

**2. Does the work transfer? Mostly -- and it is the larger cost.** The compile
loop 1.10 item 1 insists on building first is PyTorch -> ONNX -> {Hailo DFC ->
HEF | TensorRT engine}. The ONNX export, the calibration corpus, and the harness
that scores a candidate model over the recorded walks with
`control/walk_eval.py` are all platform-independent. **Only the final step is
Hailo-specific**, and on a Jetson it becomes `trtexec`. A pivot therefore loses
one compile step, not the pipeline -- and the pipeline was always the expensive
part, which is exactly why item 1 asks for it before the hardware arrives.

**3. What is the exposure? $130, on a ~$560 build, for a part that resells.**
Set against a pivot that costs +$320-430 at current prices, redesigns 1.3's
power plan, and gives up the Pi camera stack. And on the specific wish --
larger models from Hugging Face -- **the 10H's 8GB is dedicated, where the
Jetson's 8GB is shared with the OS, SLAM and nav2.** That shared memory is the
binding reason 1.10 gave for ruling the Jetson out, and it does not apply to the
HAT. For model work specifically the $130 part has the better memory position.
Neither runs an arbitrary Hugging Face checkpoint without a compile step *and* a
size ceiling -- the 10H's is around the 1.5B class, the Jetson's is 8GB minus
whatever else is resident -- so the honest gap is narrower than "a Jetson runs
anything".

**The answer is no, and the better move is a third one.** The regrettable
purchase in this project is not the wrong accelerator; it is **any accelerator
bought before the corpus is valid.** Stage 0's gate is not cleared and its
recordings are unusable for reasons that have nothing to do with compute -- a
target on furniture a floor robot cannot reach, and a camera at standing height
(`CLAUDE.md` Stage 0). No chip fixes a data problem. So:

- **Order the chassis, Pi, lidar and camera. Leave the accelerator off the first
  order.**
- Re-record the corpus at 10-13cm on the wheeled rig -- needs none of this
  hardware.
- Build the compile loop against a rented x86 host and score YOLO11n over the
  walks -- also needs none of it.
- Then buy the AI HAT+ 2, in **M.2 module form**, once a detector has earned its
  place in replay.

That sequencing costs one extra shipping charge and removes the question
entirely: by the time the $130 is spent, the evidence for spending it exists. It
is 4.7's own promotion rule -- *experiments run off-robot behind the perception
seam and are promoted to the Hailo by compile once they have earned it in
replay* -- applied to the purchase as well as to the models.

#### Re-open triggers, extended

4.7 lists two conditions for re-opening the Jetson. Two more, from this review:

3. **A live operator video feed becomes a requirement.** The Pi 5 has no
   hardware encoder; every teleoperated fleet above has one. The only trigger
   here that is a *Pi* problem rather than an accelerator one.
4. **Jetson pricing returns to roughly $249.** At that number the delta is ~$170
   and the comparison is genuinely close. At $399-480 it is not.

### 4.9 The Pi-plus-Hailo option space, priced and checked

**Checked 2026-09-06**, after 4.8 narrowed the platform question to this family.
Three axes -- which accelerator, which *form* it comes in, and how it coexists
with the rest of the robot -- and the third is the one that has been invisible
in this plan so far.

#### The parts, as actually sold

| Configuration | Part | Form | ~USD | Notes |
|---|---|---|---|---|
| **A. Nothing** | -- | -- | **0** | YOLO11n at ~5-13 FPS on the Pi 5's own cores, by export path. Clears 4.4's bar. Fails on the ceiling (4.1), never on the rate |
| **B. AI HAT+ 13 TOPS** | Hailo-8L | **soldered** | **70** | The plan's original choice. Cheapest real accelerator |
| **C. Hailo-8L M.2 + M.2 HAT+** | Hailo-8L | **module**, 2242 B+M | **~70** | Same silicon as B. This was the **AI Kit**, now *out of production* -- so it means a standalone module plus the M.2 HAT+. **The module moves** (4.8) |
| **D. C, on a dual-slot switch board** | Hailo-8L | **module** | **~70 + 48** | Pineboards HatDrive! Dual: an ASMedia ASM1182e PCIe Gen 2 switch, two M-key 2230/2242 slots, and **documented Hailo support** (`dtoverlay=pineboards-hat-ai`). Accelerator *and* NVMe on the one lane. **Discontinued at some retailers -- check stock** |
| **E. AI HAT+ 26 TOPS** | Hailo-8 | soldered | 110 | 4.3 already rules this out: the camera's 30fps bounds a nano or small detector either way |
| **F. AI HAT+ 2** | Hailo-10H | **soldered** + a separate 8GB chip | **130** | CV throughput equivalent to the 26 TOPS part, plus generative. See the measured numbers below |
| **G. Hailo-10H M.2 module** | Hailo-10H | module | **unverified** | Exists as a standalone Key-M module through distributors. Industrial part; **assume it is not $130** until priced. Whether it carries its own 8GB (the HAT's is a separate IC on the board) is **unverified** |

Not evaluated: the **CM5** ($45 and up, ECC RAM, same single PCIe Gen 2 lane).
It is the right module for a finished product and the wrong one for a first
build -- it needs a carrier board, which is a project of its own.

#### The two integration facts that actually choose between these

**1. There is one PCIe lane, and the accelerator wants it.** Known since 1.10
item 2. What is new is that the resolution is a **verified product**, not a
hope: the HatDrive! Dual carries a real PCIe switch and Pineboards document the
Hailo module in one slot beside another device. 1.10 item 2 recorded this as
"reported to work, **unverified**" -- it is now vendor-documented for the 8L
module, which is configuration D. *(Slots are 2230/2242 only, so the NVMe must
be a short one.)*

**2. Every one of these sits on the 40-pin header, and this robot needs it.**
The motor driver, two pan/tilt servos, the wheel encoders and the IMU all want
GPIO. The AI HAT+ 2 ships a 40-pin extension header, and the reviews are
explicit about the trap: seat it fully and **the pins are no longer
accessible** -- you must leave it proud, or fit a taller stacking header, to put
anything else on top. **This answers 1.10 item 7**, which had it as an open
question for the seller: the header passes through, but only deliberately, and
the mounting hardware in the box does not do it for you. Budget a 2x20 extra-tall
stacking header and check clearance against the active cooler, the HAT and the
lidar deck.

#### First -- which of 2.1's three rates any of this touches

**Scoping this explicitly, because the first draft of this section did not and
was misread accordingly.** 2.1's three rates are the whole reason an accelerator
is in the bill at all, and the tokens/s below bear on exactly one of them:

| 2.1's tier | Rate | Who answers it | What an 8L does for it |
|---|---|---|---|
| reflex | ~50 Hz | lidar + collar, no model at all | **nothing** -- it needs no accelerator |
| **perception** | **15-30 Hz** | **the detector -> bearing** | **the entire job.** YOLOv8n at 640 is ~2-7ms on an 8-series part (431 FPS batch-1 reported on the Hailo-8; ~137 FPS batch-8 on an 8L for a custom nano; YOLO11m 24-50 FPS). The Pi 5's own cores manage 5-13 FPS -- **below this row, not above it** |
| deliberation | ~0.5 Hz | the LLM, **off-board by design** | **nothing** -- 2.1 already puts it in the cloud |

**On the question it actually answers, on-board inference is roughly 500-1500x
faster than the cloud call** -- single-digit milliseconds against the 3.6s/step
measured to Opus 4.5. That gap is the argument for the accelerator, it is the
middle row of 2.1, and nothing below disturbs it.

**This also sharpens 4.4, which is easy to read as the opposite.** 4.4 concludes
that stock YOLO11n on the Pi's cores clears the reaction budget -- true, but only
because motion today is *discrete*, 0.5s per move at ~2Hz. Against 2.1's
perception row the CPU is a tier short. **The accelerator is what makes that row
exist**, and the day driving becomes continuous -- which a differential chassis
with encoders and a 360-degree lidar invites -- it stops being a capability
argument and becomes a latency one.

#### What the 10H's generative capability actually measures, and what it does not

**It bears on the deliberation row only, and only on a proposal to move that row
on-board** -- which 2.1 never asked for and which is not why the accelerator is
being bought.

4.8 recommended the 10H partly because a local VLM would make 2.5's degraded
mode real. **The published review numbers weaken that argument and it should be
weakened in writing:**

| Model | tokens/s on the Hailo-10H |
|---|---|
| DeepSeek-R1-Distill 1.5B | 6.72 |
| Qwen2 1.5B | 5.89 |
| Llama 3.2 3B | 2.60 |

Qwen2-VL-2B-Instruct does describe a camera image. But this project's
deliberation reply is an action plus its reasoning -- call it 60-100 tokens --
which at ~6 tokens/s is **ten to seventeen seconds, before image prefill**.
The measured cloud round trip to Opus 4.5 in the sim runs was **3.6s/step**
(`CLAUDE.md` Stage 0). So the on-board VLM is **slower than the cloud call it
would replace, and worse at the task** -- Stage 0 has Sonnet 4.5 stalling on
this question, and a 2B model is well below that.

**None of that is an argument against on-board inference, and it must not be
read as one.** It is an argument against one *reason* that was offered for
paying $130 instead of $70 -- a local VLM standing in for the cloud one. The
detector's case is in the table above and is untouched: it is the fastest thing
in the system by three orders of magnitude. It lands exactly where 4.7 put the Jetson's local VLM -- *"not a
replacement for Opus 4.5 ... an upgrade over the rule-based wall-follower as the
degraded mode"* -- and now there is a number attached. **A local VLM here is a
2.5 fallback for when the network is gone, not a deliberation tier.**

Power is the quiet win instead: 7.2-7.6W running a 1.5B model on the HAT against
10.2-10.6W doing it on the CPU. On a battery robot, offloading is worth more than
the tokens are.

#### Recommendation, amending 4.8

**Configuration C or D with a Hailo-8L, not F.** 4.8 said take the 10H on
1.10 item 6's pre-registered test, and that test did pass on its own terms --
in stock at $130, real named models. But item 6's *purpose* was "turn a
detection sandbox into a broader one", and the tokens-per-second above say the
broader half is a fallback rather than a capability. Meanwhile:

- **The job 1.10 gives the detector is detection**, and 4.3 already establishes
  the 8L runs a nano or small detector above camera rate. The families that
  matter here -- larger YOLO tiers, floor segmentation, monocular depth, CLIP
  over crops -- are 8-series families, not 10H-only ones.
- **The module form is worth more than the extra TOPS.** It moves to a Jetson
  (4.8), it shares the lane with the NVMe (D), and it is the only form that
  keeps both of those open. The 10H is soldered at $130; as a module it is
  unpriced.
- **~~$60 saved is not the point; the $130 not yet spent is.~~ Superseded the
  same day by 1.14.** The sequencing argument -- leave the accelerator off the
  first order and buy once a detector has earned its place in replay -- was
  explicitly conditional on motion staying discrete, which was the one thing
  that made a 5-13 FPS perception tier survivable. **Motion is now continuous,
  so the accelerator ships with the first order.** The rest stands: re-record
  the corpus and build the compile loop against a rented x86 host regardless,
  because neither needs the part.

**And on the Hugging Face wish specifically, the honest answer is that no
accelerator is the way to satisfy it.** The Hailo family's generative ceiling is
the 1.5-3B class at single-digit tokens/s, behind a compile step; a Jetson's is
8GB shared. Any model worth evaluating runs at full size, unquantised, on a
rented GPU against the recorded walks -- which is 2.7's perception seam and
4.7's promotion rule doing exactly what they were written for. **The car does
not need to be the laboratory.** Buy the accelerator for what has to be *on* the
car at frame rate, and nothing else.

---

## 5. What this invalidates elsewhere

The chassis decision has documentation consequences, and **1.14's move to
continuous driving has more**. Recorded here so they are not discovered on
hardware day.

| Document | What is now wrong |
|---|---|
| `HARDWARE-READINESS.md` | Written for the PiCar-X **throughout**. §1's parts table, §4's verb-to-motor path and §5's pre-flight checklist all assume Ackermann + Robot HAT + `picarx`. **§5.2's arc concern resolves to the pivot branch.** §5.3 (where the ultrasonic is mounted) is superseded by the lidar |
| `PLAN-sim-hardening.md` | **S6's Ackermann half is unnecessary** -- `grid_world.py`'s pivot assumption is now correct, and §3.3's divergence closes by hardware choice rather than by code. **But 1.14 un-retires S6's continuous-pose half (2026-09-06)**, for a different reason than S6 gave and at a much smaller cost: `sim/renderer.py` already takes a float pose, so only `grid_world.py` and a two-line boundary conversion in `mock_robot.py` are discrete |
| `AGENT-HARNESS.md` | **1.14 splits the tick.** Its tick contract, concurrency model and status shape all assume one blocking sense-decide-act step. Continuous driving needs a drive loop and a mission loop at different rates. B3.2 and B3.3 keep their jobs but change what they time |
| `robot/safety.py`, `config/robot.yaml` | **Two numbers are wrong under 1.14, in code, today.** `watchdog_timeout_s: 1.0` is 50cm of travel at 50cm/s, and the fixed `min_distance_cm: 20.0` is a stopping distance good for only ~0.45 m/s. Neither is wrong for discrete motion, which is why neither was caught |
| `robot/interface.py` | Has no way to express a held velocity. 1.14 item 3 adds one, and `tests/test_robot_contract.py` has to carry it across all five backends |
| `PLAN-microduck-transplants.md` | **M2/M3 are built (2026-09-03) and the seam holds.** `PATH_FRACTION` did not -- §5.1, the one concrete defect this decision created in existing code, **fixed 2026-09-03**. **M10** (clearance from a real sensor) is satisfied far better by 360-degree metric returns than by one ultrasonic beam |
| `CLAUDE.md` | The status table and build order referenced S6, the PiCar-X hardware path and, for one day, the IMX500. **Updated 2026-09-04** for the Hailo decision and the new bill |

`HARDWARE-READINESS.md` and `PLAN-sim-hardening.md` have had staleness notes
added pointing here; `CLAUDE.md`'s status row and buy list were updated on
2026-09-04. **The four rows added for 1.14 on 2026-09-06 are recorded, not yet
acted on** -- the last two name defects in shipped code, and C1-C3 of 1.14's
phasing are where they get fixed.

### 5.1 `PATH_FRACTION` breaks on a 360-degree sensor -- **FIXED 2026-09-03**

**The good news first: M2's seam already accommodates a lidar, and by design.**
`get_depth_grid()` carries `rows` and `cols` **in the data** rather than pinning
them in the contract, and publishes `rows: 1` when there is no elevation to
report -- which is exactly what a 2D lidar is. The tri-state
(`ZONE_RANGE` / `ZONE_NO_TARGET` / `ZONE_UNUSABLE`) is precisely what a lidar
needs, since a return, an empty sweep and a failed measurement are three
different facts. A 360-degree unit slots in as `rows: 1, cols: N`. No interface
change.

**The defect is one layer up, in M3's consumer.** `robot/safety.py` selects the
path zones as `PATH_FRACTION = 0.5` -- *the middle half of the columns* -- and
its own comment derives that from **field of view**:

> the middle half of the sim's 60-degree render is +/-15 degrees, which at one
> grid cell (30cm) ahead spans 16cm [...] The middle half of a VL53L5CX's
> 45-degree field is +/-11.25 degrees: 12cm at one cell. Both bracket the
> robot's own width, which is the number that matters.

Every sensor considered when that was written had a **narrow, forward** field --
60 degrees in the sim, 45 on the ToF -- so a fraction-of-columns rule was a
sound proxy for an angle.

**On a 360-degree lidar the middle half of the columns is +/-90 degrees**: the
entire forward hemisphere, not the path. That is precisely the first failure
mode the same comment warns about -- "take the WHOLE grid and the outermost rays
[...] the robot is permanently vetoed in every corridor it is supposed to drive
down."

**The grid carries `rows` and `cols` but not the angular span they cover.** With
every sensor so far that span was implicit and similar, so nothing needed it.
A 360-degree sensor makes the omission load-bearing.

**The fix is small and belongs before M10, not during it:** carry the field of
view (or per-zone bearings) in the grid, and have `path_zone_indices()` select
by *angle* rather than by fraction-of-columns. The angle it should select is
already worked out in that comment -- bracket the chassis width at the stop
threshold -- so this is a change of input, not of reasoning.

M3's own note anticipates a revisit: *"Revisited in M10 against the real
sensor's actual field of view."* It anticipates a ToF-shaped one. This is
larger, and cheap now.

#### Done, and what it turned up

`get_depth_grid()` carries **`fov_deg`**; `path_zone_indices()` takes it and
selects every column whose bearing is within
`atan(half the chassis width / one move's travel)` -- about **+/-15.4
degrees** -- of straight ahead. A grid that declares no field of view keeps
the old fraction rule, because every backend predating this has a narrow
forward field where it is a fair proxy; a *wide* grid arriving without one
**warns**, since a silent fallback here is how the hemisphere bug ships (M7's
rule).

**It is a change of input rather than of behaviour, and that is checkable.**
On the sim's 60-degree 8-column grid both rules select the middle four zones,
so the corridor and doorway tests that validate the cone still mean what they
meant. On a 72-bin lidar the new rule selects **6 zones spanning +/-15
degrees** where the old one selected **36 spanning +/-90**.

Two things fell out of writing it that the section did not anticipate.

**The fraction was already wrong for the ToF it was justified against.** The
old comment claimed the middle half of a VL53L5CX's 45-degree field brackets
the chassis; it spans 12cm against a 16.5cm robot, and the angular rule
correctly widens that selection from four columns to six. The arithmetic
happened to land close enough on the one sensor it was checked against, which
is the same shape of error as the `NavigateModelId` trap.

**A fixed-angle cone under-covers at close range, and always did.** Zones are
selected by centre bearing, so the cone is sized at one move's travel (30cm,
where it spans 16.1cm against the 16.5cm chassis) and at the **20cm stop
threshold subtends only ~10.7cm**. An obstacle at the chassis corner *at the
threshold* is therefore not what the veto reads. This is **pre-existing** --
the fraction rule selected the identical zones -- and it is not fixable in the
grid world, whose walls are axis-aligned and 30cm apart. The real answer is a
cone that widens as range shortens, which needs a sensor whose geometry is
known. **Measure it in M10**; `tests/test_depth_veto.py` pins it so it is
found deliberately rather than rediscovered.

`CHASSIS_WIDTH_CM` is still the PiCar-X's 16.5cm, deliberately: the chosen
differential chassis is 148mm wide, so the cone errs wide, which is the safe
direction. **Re-measure it on the real chassis** -- a hardware-day pre-flight
item, not a guess to leave standing.

---

## 6. Open questions

### 6.1 Measurable today, for free -- **MEASURED 2026-09-03**

**Replay the trigger policy over recorded walks.** Every walk on EFS carries
`walk.jsonl` -- frames plus the `/navigate` reply at the time, including
`room_guess`. Running a candidate trigger policy over that data offline counts
**how many deliberation calls it would have fired** versus the number of frames.
No new inference, no Bedrock charge, no hardware. It turns §2.4's cost claim
into a number, on data already owned -- the same move `control/walk_replay.py`
already makes for prompts.

**Run: `python -m tests.manual_trigger_count`** -- 39 walks, 821 recorded
frames, §2.4's six triggers, `stale_n=8`. Two of them are not witnessable in
a recorded walk and are therefore *undercounted*: "goal impossible / boxed in"
needs the lidar this corpus predates, and a goal the reactive tier would have
finished early leaves no trace in a walk the model drove step by step. So the
saving below is if anything **over**-stated, which is the safe direction.

**§2.4's "~8x" is 4.1x, and the two refinements matter more than the
headline.**

**First: ~40% of the naive trigger count is field noise, not events.** Firing
on every raw `target_visible` / `room_guess` edge gives 2.8x. Requiring two
consecutive frames of agreement before believing an edge -- the same
hysteresis `lost_target` always needed -- gives **4.1x**; three frames, 4.5x.
`target_visible` flips on **24.1%** of frames and `room_guess` changes on
12.7%, which is `control/walk_eval.py`'s `unstable-identity` flag seen from
the cost side. **A trigger policy is only as stable as the fields it triggers
on**, and §2.4 does not say so. Hysteresis is not tuning; it is what makes
trigger discipline work at all.

**Second: the saving scales with mission length**, because `start` is one
fixed trigger per mission (39 of 199 triggers here, over a corpus full of very
short walks):

| mission length | walks | frames | saving |
|---|---|---|---|
| < 10 frames | 12 | 75 | 2.2x |
| 10-19 | 15 | 205 | 3.4x |
| 20-39 | 9 | 295 | 4.8x |
| 40+ | 3 | 246 | **5.7x** |

So ~8x is an over-estimate for the missions actually recorded, but it is the
right order and the trend runs toward it. **Quote 4-6x, not 8x.**

**Third, and load-bearing for the design: the staleness timer is not the cost
driver.** Past `stale_n` ~10 it stops binding entirely (5.7% of triggers at 8),
so cost is set by the *event* rate, not by how long a goal is allowed to run.
Tuning the timer to save money will not work; stabilising the fields will.

**The caveat this section was written with does not bite.** The worry was that
the corpus is the invalid one (target on raised furniture, camera at standing
height). Splitting it three ways -- different targets, rooms and dates --
gives 4.2x / 3.7x / 4.7x and a visibility-flip rate of 24.8% / 24.4% / 21.2%:

| group | walks | frames | saving | `target_visible` flip rate |
|---|---|---|---|---|
| red-backpack (Aug 28-30) | 25 | 508 | 4.2x | 24.8% |
| blue-bottle (Aug 30-31) | 9 | 176 | 3.7x | 24.4% |
| bottle (Sep 02) | 5 | 137 | 4.7x | 21.2% |

The flicker is a property of the perception, not of one bad session. Re-run it
on the re-recorded corpus anyway -- it costs nothing -- but the number is not
waiting on that.

### 6.2 The five questions, and where they landed

**All five are settled.** Q2 goal vocabulary (1.7) · Q3 stop conditions (1.8) ·
Q4 the detector and arbitration (1.10, 1.11) · Q5 the sim (1.12). Q1's map and
persistence questions are 1.5-1.6.

What remains open is **not design** but measurement and verification: 3.8's
five seller questions, 1.10's ordering-time checks (the storage decision, the
AI HAT+ 2), the compile loop 1.10 item 1 asks for before hardware day, and the
re-recorded Stage 0 corpus.
**6.1's free trigger-count experiment is done** (2026-09-03) -- it is the one
item on that list that needed neither a seller nor a walk.

Kept as an index into where each landed:

| # | Question | Where it landed |
|---|---|---|
| Q1 | Is there a map, and where does memory live? | **1.5** persistence -- only the map persists, planner is a pure function · **1.6** the visual-edge mechanism is cancelled |
| Q2 | Closed or open goal vocabulary? | **1.7** closed and versioned, three verbs, unknown verbs refuse by name |
| Q3 | Who owns the stop condition? | **1.8** typed success from the planner, typed failure from the robot, lidar x bearing |
| Q4 | Is there an on-device detector, and who arbitrates? | **1.10** a Hailo-8L, two layers (the IMX500 for one day -- §4 has the comparison) · **1.11** arbitration split by question, not authority · **1.13** room identity is not one of the detector's jobs · **4.8**/**4.9** re-checked against industry practice and 2026 prices; the part settles as a Hailo-8L in M.2 module form · **1.15** the physical layout it has to live in |
| Q5 | Does the sim participate? | **1.12** yes, with synthesised detections; occlusion-aware, tri-state, noise behind a flag |

### 6.3 What it owes the twin

Per `CLAUDE.md` §7 -- a phase is done when someone holding a phone can watch it
work, not when its tests pass.

- **The lidar:** a live 360-degree clearance ring under the FPV canvas. Drive at
  a chair leg the old ultrasonic beam would have missed and watch the collar
  fire.
- **The detector:** its boxes drawn on the FPV canvas, the HEF's name and the
  CLIP score against the mission's target string, and a tri-state readout
  (`detected` / `absent` / `unavailable`, 1.12) -- so a wedged capture never
  looks like a missing target. Swap the HEF and the name on screen changes;
  that is the experiment loop made watchable.
- **The tiers:** the current goal, when it was set, and what triggered it -- plus
  a deliberation-call counter that **visibly does not climb every step**. That
  single number makes the whole architecture watchable.
- **The map:** the graph or occupancy grid, the edge being executed, and a way
  to delete a bad edge (§1.5's staleness decision guarantees there will be some).

---

## 7. Glossary

### 7.1 On-device vision and accelerators

| | |
|---|---|
| **TOPS** | Tera-Operations Per Second. A throughput rating for accelerators, usually at INT8. Marketing-adjacent: real speed depends on the model |
| **INT8** | 8-bit integer arithmetic. Quantising from 32-bit floats makes a model ~4x smaller and much faster, at some accuracy cost |
| **NPU** | Neural Processing Unit -- on-chip inference accelerator. The RK3566 has one; **the Pi 5 does not**, which is why the Hailo is a separate HAT |
| **IMX500** | Sony's stacked image sensor with an inference DSP and ~8MB of model memory on the die -- the Raspberry Pi AI Camera. Evaluated and not chosen (§4): the ceiling is silicon |
| **Hailo-8L / 8 / 10H** | Hailo's dataflow inference chips, on the Raspberry Pi AI HAT+ (8L, 8) and AI HAT+ 2 (10H). The 8-series runs CNNs; the 10H adds on-module memory for transformers. **The 8L is chosen** (1.10) |
| **HailoRT** | Hailo's runtime -- loads HEFs, schedules several at once, exposes a Python API. Installed by `hailo-all` on Pi OS |
| **Jetson / JetPack / TensorRT** | NVIDIA's embedded GPU boards / their Ubuntu-based OS image / NVIDIA's inference compiler. The on-board upgrade path, ruled out for now (4.7) |
| **ONNX** | Open Neural Network Exchange. A portable model format |
| **HEF** | Hailo Executable Format -- Hailo's compiled model file; the chip will not take an ONNX |
| **DFC** | Dataflow Compiler. Hailo's ONNX-to-HEF toolchain. x86-64 Linux only |
| **YOLO** | "You Only Look Once" -- single-pass object detectors. The `n` is "nano" |
| **COCO** | Common Objects in Context. The 80-class label set most detectors predict; includes `backpack` and `bottle` |
| **mAP50 / IoU** | mean Average Precision at 50% Intersection over Union -- the usual detection score, and the box-overlap measure it thresholds on |
| **XNNPACK / ncnn / TFLite** | Optimised CPU inference backends and runtimes for ARM |
| **CLIP** | Contrastive Language-Image Pre-training. Matches images to text; what makes open-vocabulary detection possible. On the Hailo it re-ranks detector crops against the mission's target string (1.10, 4.2) |
| **VLM / LLM** | Vision-Language Model / Large Language Model. The cloud tier -- what `/navigate` calls |
| **MiDaS / Depth Anything** | Monocular depth models. **Relative**, not metric -- which is why they do not close the gate |
| **HSV** | Hue-Saturation-Value. The colour space the first draft proposed for an "is that backpack red" check -- superseded by CLIP over crops (4.2) |

### 7.2 Mapping and navigation

| | |
|---|---|
| **SLAM** | Simultaneous Localization and Mapping. Building a map while working out where you are on it |
| **Scan matching** | Registering one lidar scan against the previous to recover relative motion -- **lidar odometry**, which is why no wheel encoders are needed |
| **Loop closure** | Recognising a previously visited place and snapping the map back into consistency. The thing (b) lacks and (c) has |
| **Occupancy grid** | A map as a grid of cells, each free / occupied / unknown |
| **Odometry** | Estimating motion from sensors -- wheel encoders classically, or the lidar itself |
| **Dead reckoning** | Estimating position by accumulating motion with no external reference. Drifts without bound |
| **Metric vs topological map** | A floor plan with coordinates, vs. a graph of places and how they connect |
| **Egocentric vs allocentric** | Relative to the robot ("doorway at +0.4") vs. relative to the world ("the kitchen") |
| **ICP** | Iterative Closest Point. A classic scan-registration algorithm |
| **DWA** | Dynamic Window Approach. A common local planner -- and one that assumes differential drive |
| **Ackermann / differential drive** | Car-like front-wheel steering that cannot pivot, vs. independently driven wheels that can |
| **ROS 2** | Robot Operating System 2. A robotics framework -- its own IPC (DDS), build system (colcon) and node model |
| **DDS / colcon** | ROS 2's transport layer / its build tool |
| **`slam_toolbox` / Cartographer / Hector SLAM** | Production SLAM implementations. Hector notably needs no odometry |
| **Costmap** | An occupancy grid inflated by robot radius, so a planner can treat the robot as a point |
| **Back-EMF** | Voltage a spinning motor generates back into its supply. Why motors get their own rail |
| **BEC / buck converter** | A step-down regulator -- how to take a clean 5V off a higher-voltage pack |
| **Discrete vs continuous driving** | Motion as bounded timed bursts with the robot stopped between them, perceiving only while parked -- versus holding a velocity and perceiving *while moving*. This project was the first and became the second on 2026-09-06 (**1.14**). The distinction decides whether perception latency costs wall-clock or centimetres |
| **Stopping distance** | `v * t_react + v^2 / (2a)` -- how far the robot travels between an obstacle becoming visible and the wheels being stopped. The quantity a safety collar must exceed once motion is continuous (1.14 item 5) |
| **Deadman** | A control that stops the machine unless it is actively and repeatedly reasserted. `robot/server.py`'s watchdog is one, and 1.14 promotes it from backstop to primary |

### 7.3 Hardware and buses

| | |
|---|---|
| **SoC** | System on Chip. The RK3566, or the Pi 5's BCM2712 |
| **HAT** | Hardware Attached on Top. The Pi's 40-pin add-on spec |
| **PCIe / FPC / M.2** | The high-speed bus the AI HAT+ uses / the ribbon cable carrying it on a Pi 5 / the Hailo module's form factor |
| **CSI** | Camera Serial Interface. The Pi's ribbon camera port |
| **UART / I2C** | Low-speed serial buses. Some lidars use UART; the Robot HAT may occupy the GPIO pins |
| **USB-PD / QC** | USB Power Delivery / Quick Charge. Voltage-negotiation standards; a Pi 5 negotiates over PD |
| **ADC** | Analog-to-Digital Converter |
| **ToF** | Time of Flight. A depth sensor that times a light pulse |
| **IMU** | Inertial Measurement Unit. Accelerometer + gyroscope |
| **HFOV** | Horizontal Field Of View, in degrees |
| **VPU / ISP** | Video Processing Unit (hardware codec) / Image Signal Processor |
| **RKNN / rknpu2** | Rockchip's NPU model format and runtime |

### 7.4 Microduck's system

| | |
|---|---|
| **RL / PPO / MuJoCo / sim2real** | Reinforcement Learning · Proximal Policy Optimization · the physics simulator the policies train in · getting a sim-trained policy onto real hardware |
| **FK** | Forward Kinematics. Joint angles to a position in space |
| **SFLP** | Sensor Fusion Low Power. The IMU's on-chip orientation fusion |
| **JSON-RPC / NDJSON** | A JSON RPC convention / newline-delimited JSON, one object per line |
| **UDS / IPC / RPC** | Unix Domain Socket · Inter-Process Communication · Remote Procedure Call |
| **SO_PEERCRED** | A socket option returning the caller's uid/gid/pid -- the basis of audit and enforcement |
| **BLE / GATT / RSSI** | Bluetooth Low Energy · how it exposes characteristics · signal strength, used as coarse distance |
| **WebRTC** | Real-time media and data channels, browser-native |
| **SDP / ICE / STUN / TURN** | WebRTC's connection machinery: what peers offer · finding a path · discovering your public address · a relay when NAT defeats you |
| **DTLS-SRTP / SCTP** | WebRTC's media encryption / the transport under its data channels |
| **NAT** | Network Address Translation. Why a home robot is not directly reachable |
| **SSE** | Server-Sent Events. One-way HTTP push; considered and declined |
| **D-Bus / BlueZ / NetworkManager** | Linux's message bus / Bluetooth stack / network configuration daemon |
| **EMA** | Exponential Moving Average. The battery-voltage smoothing that stops a load sag tripping shutdown |
| **ULD** | Ultra Lite Driver. ST's vendored C driver for the ToF sensors |
| **shm / dmabuf** | Shared memory / Linux buffer sharing -- the zero-copy escape hatch |
| **NV12 / UYVY / MJPEG / H.264** | Two raw YUV layouts, a per-frame JPEG stream, and the codec WebRTC carries |
| **V4L2 M2M** | Video4Linux2 Memory-to-Memory. The kernel API for hardware encode |
| **flock / fsync / rename(2) / inotify** | Linux primitives for a safe file write, and change notification |
| **SHA-256 / minisign** | A cryptographic hash / a signature tool. How a release is verified |
| **OTA / RTT / SDK** | Over-The-Air updates · Round-Trip Time · Software Development Kit |

### 7.5 This project

| | |
|---|---|
| **ALB / NLB** | Application / Network Load Balancer -- the pair all five services share |
| **ECS / Fargate / ECR** | Elastic Container Service · its serverless mode · Elastic Container Registry |
| **EFS** | Elastic File System. Holds recorded walks; survives a redeploy |
| **DynamoDB** | AWS's key-value store -- §1.5's choice for the map |
| **VPC / IAM** | Virtual Private Cloud · Identity and Access Management |
| **CDN / CloudFront** | The HTTPS front door |
| **IaC** | Infrastructure as Code. The CloudFormation templates |
| **FPV / AR** | First-Person View, the twin's camera canvas · Augmented Reality, the Guide tab |
| **DOM / jsdom** | Document Object Model / a headless JS implementation -- why UI tests are Playwright |
| **CORS** | Cross-Origin Resource Sharing |
| **CI** | Continuous Integration |

---

## 8. Sources

- Microduck `docs/design/architecture.md` §2.4 (features not frames), §5.3
  (server-side agents over WebSocket), §6 (safety and authority), §1 (`tofd`
  owns one sensor and reads nothing -- the pattern behind (b+))
- Microduck `docs/design/robotd-design.md` §1.4-§1.5, §2.4
- Microduck `docs/ideas/autonomous_behavior.md` (duck detector; the camera / ToF
  / BLE sensor split)
- Microduck `docs/project/npu-bringup.md` (`yolo11n` numbers, `dlopen` over
  linking)
- Microduck `tof/src/lib.rs` (`Range` / `NoTarget` / `Unusable`)
- This repo: `CLAUDE.md` Stage 0 notes and §7 · `AGENT-HARNESS.md` §3, §5, §10 ·
  `PLAN-microduck-transplants.md` §1-§2 · `HARDWARE-READINESS.md` §1, §5 ·
  `control/mission_runner.py:255` · `brain/memory.py`

**Checked on the web 2026-09-06, for 4.8** (prices and availability, not
bench-verified):

- Raspberry Pi, *Introducing the Raspberry Pi AI HAT+ 2* -- Hailo-10H, 8GB
  on-board RAM, 40 TOPS INT4, $130, `hailo-ollama`, LoRA, and the named
  1.5B-class models
- CNX Software / Hardware Busters, July 2026 -- the Jetson repricing: Orin Nano
  Super devkit $249 -> $399, Orin NX 8GB module $399 -> $649
- NVIDIA case study and Serve Robotics' own blog -- Serve Gen3 on Jetson Orin,
  5x over Xavier · NVIDIA blog -- Cartken on AGX Orin, six cameras and SLAM ·
  Kiwibot on TX2 / Xavier
- Coco Robotics' Coco 2 launch coverage (PR Newswire, Semafor, The Robot
  Report), Feb 2026 -- Jetson Orin NX, Isaac Sim / Cosmos, and the earlier
  sub-300ms teleoperation
- Wevolver / Starship engineering blog -- Tegra TK1 then x86 AMD Ryzen, plus an
  FPGA for the time-critical signal processing
- Hailo community and product pages -- HailoRT on aarch64, a Hailo-8 M.2 brought
  up on a Jetson Orin Nano Super under JetPack 6.2, and the standalone Hailo-10H
  Key-M module
- Raspberry Pi forums / Ultralytics docs -- no hardware H.264 or H.265 encoder
  on the Pi 5, and YOLO11n at roughly 5-13 FPS on its CPU by export path

**Also checked 2026-09-06, for 4.9:**

- Raspberry Pi, *Introducing the Raspberry Pi AI HAT+* and the AI HATs
  documentation -- the accelerator is soldered on the AI HAT+ where the AI Kit
  used an M.2 connector; 13 TOPS $70 / 26 TOPS $110; **the AI Kit is out of
  production**
- CNX Software's AI HAT+ 2 review, Jan 2026 -- the Hailo-10H and an 8GB IC both
  soldered; the GPIO extension header must be left proud to stay usable;
  DeepSeek-R1 1.5B at 6.72 tok/s, Qwen2 1.5B at 5.89, Llama 3.2 3B at 2.60;
  Qwen2-VL-2B-Instruct describing a camera image; 7.2-7.6W against 10.2-10.6W
  on CPU; the SDK's Python-version sensitivity and the Docker workaround
- Hackster's AI HAT+ 2 review -- the bundled mounting hardware exposes no pins
- Pineboards HatDrive! Dual product and documentation pages, and Jeff Geerling's
  Raspberry Pi PCIe database -- ASMedia ASM1182e PCIe Gen 2 switch, two M-key
  2230/2242 slots, **documented Hailo-8L support** via
  `dtoverlay=pineboards-hat-ai`; listed discontinued by at least one retailer
- Hailo product pages / PCIe database -- the Hailo-8L M.2 in 2242 B+M and 2230
  A+E, and the standalone Hailo-10H Key-M module
- Raspberry Pi, *Compute Module 5 on sale now* -- from $45, ECC LPDDR5, one
  PCIe Gen 2 lane

**Also checked 2026-09-06, for 1.15 and 1.16** (vendor specs, none bench-verified):

- Slamtec RPLidar C1 datasheet and retailer specs -- **55.6 x 55.6 x 41.3mm,
  110g**, 5V UART, 10Hz typical (8-12Hz), 5kHz sample rate, 0.72° angular
  resolution, 0.05-12m. **The scan-plane height inside the body is in the
  mechanical drawing (Figure 4-1), not the specification tables** -- 1.15.5
  item 1
- Raspberry Pi Active Cooler mechanical drawing and M.2 HAT+ documentation --
  **16mm board-to-board** clears the cooler ("at least 15mm; 16mm ideal"), and
  the M.2 HAT+ ships that stacking header and spacers
- 2-axis SG90/MG90S pan-tilt bracket, retailer specs -- **32 x 28 x 65mm, 42g**,
  180° pan, 130° tilt, fits a 28 x 28mm camera module
- Livox Mid-360 (~$749, 360° x 59°, ~265g, ~6.5W), Unitree 4D L2 (~$400),
  Orbbec Gemini 335 (~$250) -- the 3D options 1.15 rejects
- Waveshare General Driver for Robots wiki and product page -- ESP32-WROOM-32,
  **TB6612FNG**, 2 encoder-motor channels, **9-axis IMU (QMI8658C + AK09918)**,
  lidar interface, current monitoring, 7-13V input. **Its PWM servo output does
  not support MG90S or MG996R**, which conflicts with 3.6's SG90-class pan/tilt
- Yahboom motor documentation -- the 520 encoder motor quoted at 333 RPM after
  reduction, and the RPM printed on the motor label (3.8 question 6)

Hardware claims about the Pi 5, the AI HAT+ and AI HAT+ 2, the AI Camera, the
Jetson Orin Nano and the lidars are otherwise from general knowledge as of this
date, **not verified against a board**.
Every one is cheap to check and should be checked before money moves.
