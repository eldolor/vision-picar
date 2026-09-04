# Plan: perception on the car itself

Status: **design settled, nothing built** · Date: 2026-09-03 · Phase IDs: none assigned yet

Started as a holding pen after reading Microduck -- *what could run on the car
itself?* -- and became the place where a chain of hardware and architecture
decisions got made. Sections 1-3 are those decisions and the reasoning behind
them. Nothing here is implemented.

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

**One consequence that reaches Q4:** "can the IMX500 emit an embedding, or only
boxes?" was load-bearing while the visual-edge design was live. It is now
**moot**, and Q4 turns on detection quality alone.

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

**4.5's "accelerator possibly never" and 1.8's `approach` pull against each
other.** With no on-device detector the target bearing arrives at deliberation
rate -- **1-3 seconds stale** -- so `approach` is open loop on the semantic half
while the lidar closes the loop on clearance. This project has been burned by an
open-loop assumption before (`sim/replay_robot.py`'s docstring).

Three ways out: a **cheap CPU detector** (stock YOLO11n on the Pi 5's own cores,
a few Hz, no accelerator and no purchase); accept open-loop `approach` over
short hops and let the planner re-issue, which erodes the trigger discipline; or
**let the lidar carry it once locked** -- take one bearing fix, then track that
geometric feature in the scan while closing. The first is cheapest, the third is
the most elegant and the most work. **Q4's subject.**

**Resolved by buying the sensor (1.10).** With an on-sensor detector the bearing
is fresh at sensor rate, so `approach` is closed loop on both halves -- the
detector for direction, the lidar for distance. None of the three workarounds is
needed.

### 1.10 The reactive tier: **two layers**, and an IMX500

**Decided: the AI Camera (Sony IMX500) up front**, not a Camera Module 3 with an
accelerator deferred. It runs its network **on the sensor**, so detection costs
the Pi's cores nothing -- which matters most exactly where it would be felt,
under (b+), sharing four cores with `slam_toolbox` and nav2.

**And that decision simplified the design it was made for.** A three-layer
scheme had been proposed: VLM for identity, **lidar blob tracking** for
continuous bearing, and a CPU detector to re-acquire when tracking broke. Layer
two existed for one reason -- *you should not have to re-detect the target ten
times a second*. With an on-sensor detector that is free and continuous, so:

| | Was going to be | Now |
|---|---|---|
| Continuous bearing | lidar blob tracking + data association | **the detector, directly** |
| Range to target | lidar at the tracked blob | **lidar at the detector's bearing** -- 1.8, already decided |
| Re-acquisition | a CPU detector as backup | **not a thing** -- tracking cannot break if detection never stopped |

**Same shape as 1.6:** an intricate mechanism designed around an absence,
cancelled once the absence was filled. What remains is what 1.8 already
specified -- the detector points, the lidar measures -- plus the VLM as the
deliberation tier that already exists. No new tracking machinery.

#### Why not a CPU detector

Under (a) a single-class detector at a few Hz is comfortable on four
Cortex-A76 cores and costs nothing. Under (b+) it shares those cores with SLAM,
which is a sustained load. Paying ~$40 to move the work onto the sensor buys
CPU headroom at exactly the point the design gets tight, and closes the camera
question rather than deferring it.

#### What it commits to, and the one thing to verify first

The IMX500 runs models compiled through **Sony's toolchain**, not arbitrary
ONNX -- the same class of friction as Hailo's DFC.

1. **Verify the bundled model zoo covers COCO before ordering.** The Raspberry
   Pi AI Camera ships example models (MobileNet-SSD class); COCO's 80 should
   include `backpack` and `bottle`. If so, 4.2's zero-training finding holds.
   **This is the one check that could undermine the choice.**
2. **A custom class later is a toolchain project**, not an afternoon. Fine
   while COCO covers the targets.

**A limitation that no longer matters:** the IMX500 emits detections, not
embeddings. That was the sub-question withdrawn when 1.6 cancelled the
visual-edge design, so the decisions stay consistent.

### 1.11 Arbitration: split by question, not by authority

Microduck's `architecture.md` §6 rule -- *decided priority, not
last-writer-wins* -- governs **control**. This governs **facts**, so the rule is
different: each question goes to whichever source can actually answer it.

| Question | Who wins | Why |
|---|---|---|
| **Is it a red backpack?** | **VLM** | more capable at semantics, and it holds the mission's definition of the target |
| **What bearing?** | **detector** | geometric precision beats a natural-language direction |
| **How far?** | **lidar** | 1.8's fusion -- range at the detector's bearing |
| **Is it there right now?** | **detector** (fresh) | but a VLM "not here" **triggers re-confirmation**, never a silent override |
| **Can I reach it?** | **planner** | it knows the mission, the memory and the map |
| **Will I hit something?** | **lidar + collar** | no vote, ever |

**The instinct that "if Bedrock is more capable, the planner wins" holds where it
should and only there.** It is right for *identity* and wrong for *geometry*: a
COCO detector is measurably better than a VLM at the one thing `approach` needs,
because a bounding box is a bearing and "slightly to the left" is not. Stage 0
already established that every model identifies a red backpack -- capability was
never the scarce thing. Precision was.

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
| Room change suspected | reactive | crossed a doorway; memory needs updating |
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
2. **ROS in Docker on Pi OS** -- keeps the camera tooling, adds container
   plumbing for the lidar device and networking. **Probably right**, and it fits
   (b+)'s behind-a-wall shape.
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
| Motor driver (TB6612FNG) | 0 | **Included** with the Yahboom kit -- IC unconfirmed (3.8) |
| **AI Camera (IMX500)** | 70 | Q4 decided -- 1.10. Was Camera Module 3 at 30 |
| 2-axis pan/tilt bracket + SG90s | 12 | Replaces what the PiCar-X bundled |
| **3S** Li-ion pack + charger | 35 | Motor rail only (1.3). **Not 2S** -- see the correction below |
| Wiring, connectors, switch, XT60 | 15 | |
| Standoffs, M2.5/M3 hardware | 10 | For stacking decks |
| | **~405** | Chassis priced down (3.8), camera up (1.10) |

**Strongly recommended -- each avoids a failure already discussed here**

| Item | ~USD | Avoids |
|---|---|---|
| Powered USB hub | 15 | The 600mA USB cap browning out the Pi (1.3) |
| IMU (MPU6050 / BNO055) | 10 | Heading drift between scans -- **matters much more under (b+)**. May be **0** if the motor-driver board carries one; verify before buying separately |
| 5V buck converter | 8 | If the Pi is ever taken off the bank and onto the pack |
| Lidar mount, 3D printed | 15 | 0 with a printer |
| Jumper wires, misc | 10 | |
| | **~58** | |

**Worth considering**

| Item | ~USD | Why |
|---|---|---|
| NVMe SSD + M.2 HAT | 45 | **SD cards corrupt on brownout**, which is the exact failure 1.3 is written about. The one item here that prevents losing work rather than an annoyance |
| ~~AI Camera instead of Camera Module 3~~ | -- | **Decided (1.10)** -- moved into the essential list |

**Totals**

| Scenario | ~USD |
|---|---|
| Essential only | 405 |
| **+ recommended** | **463** |
| + NVMe | 508 |
| ~~+ AI Camera instead~~ | now in the essential list |
| Already own a Pi 5 | subtract ~100 |

Already owned, 0: the power bank (1.3). Deferred, possibly never: any AI
accelerator (4.5). **Budget ~460-510**, and the two variables that move it are whether a Pi 5 is
already owned and how Q4 resolves. Revised down from ~450-500 once the chassis
was priced against real listings rather than estimated (3.8).

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

None is a blocker; all four are one email to the seller.

---

## 4. Accelerator options -- now probably unnecessary

Kept because the analysis is sound and the conclusion changed.

### 4.1 The four paths

**The Pi 5 has no NPU** -- the biggest difference from the RK3566 -- but four
Cortex-A76 cores at 2.4GHz are a stronger CPU host than the Rockchip's A55s, so
CPU-only is a real option here in a way it is not on a duck.

| Option | Silicon | Inference runs | Rated | Fit |
|---|---|---|---|---|
| **CPU only** | 4x Cortex-A76 @ 2.4GHz | on the Pi | -- | Nothing to buy or mount |
| **AI Camera** | Sony IMX500 | **on the sensor** | ~3 TOPS | CSI swap, negligible Pi CPU |
| **AI HAT+** | Hailo-8L / 8 | on the module, over PCIe | 13 / 26 TOPS | GPIO/PCIe contention; real watts |
| **Coral USB** | Edge TPU | on the stick | 4 TOPS | Pi 5 kernel support has been rough |

Vendor ratings at INT8. **None measured on a board.**

### 4.2 The finding that lowers the risk

**`backpack` and `bottle` are both COCO classes.** The Stage 0 targets are in
the standard 80-class label set essentially every off-the-shelf detector
predicts, so a stock pre-compiled YOLO11n finds them with **zero training, zero
calibration set, zero distillation**. Distillation becomes the project only past
COCO's 80. *(Confirm against the model's own label file.)*

**It does not give you the colour.** COCO says `backpack`, not `red backpack`.
Two backpacks in a room means an HSV check on the crop -- cheap and probably
sufficient -- or a genuinely open-vocabulary model, which is a much larger
commitment.

### 4.3 YOLO11n on the AI HAT+

Yes, and it is the most turnkey combination on the list: `sudo apt install
hailo-all` brings the driver, HailoRT, the GStreamer bits and `rpicam-apps` with
Hailo post-processing; pre-compiled YOLO11n HEFs exist for both parts.

**The friction is custom classes** -- the chip wants a HEF from Hailo's
Dataflow Compiler, which needs **x86-64 Linux**, plus a calibration set. And it
is **enormously overprovisioned**: high-tens-to-hundreds of FPS against a
decision loop measured in seconds.

### 4.4 The corrected benchmark bar

**The original recommendation contained a circular argument and is withdrawn.**
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

**The binding number is unmeasured: how fast the car actually moves.** It
belongs on the hardware-day pre-flight list.

Two things relax the bar. **The range sensor owns emergency stop, not the
camera** -- so the detector's latency budget is about steering, not collision.
And **motion is discrete today** (speed 50 for 0.5s per move), capping decisions
near 2Hz. The FPS question only sharpens with *continuous* driving, which is a
design choice not yet made.

### 4.5 Why the conclusion changed

**Buying the lidar weakens the case for an accelerator rather than
strengthening it.** Obstacle avoidance moves to the lidar; navigation moves to
the map; visual edge-matching disappears (§3.2). The only remaining job for
on-device vision is "is the target in view" -- lower-rate and not
safety-critical, which the cloud VLM can keep doing at deliberation rate.

**Sequencing: lidar first, accelerator possibly never.**

**But 1.9 records a live tension with this**, found while working through
`approach`'s stop condition: with no on-device detector the target bearing is
1-3 seconds stale, which makes `approach` open loop on the semantic half. A
**CPU-only** detector -- no accelerator, no purchase -- may re-enter the design
there. "Accelerator possibly never" is not the same claim as "no on-device
detection", and this section should not be read as settling Q4.

---

## 5. What this invalidates elsewhere

The chassis decision has documentation consequences. Recorded here so they are
not discovered on hardware day.

| Document | What is now wrong |
|---|---|
| `HARDWARE-READINESS.md` | Written for the PiCar-X **throughout**. §1's parts table, §4's verb-to-motor path and §5's pre-flight checklist all assume Ackermann + Robot HAT + `picarx`. **§5.2's arc concern resolves to the pivot branch.** §5.3 (where the ultrasonic is mounted) is superseded by the lidar |
| `PLAN-sim-hardening.md` | **S6 (Ackermann turns, continuous pose, scaled map) is unnecessary** -- `grid_world.py`'s pivot assumption is now correct. §3.3's divergence is closed by hardware choice rather than by code |
| `PLAN-microduck-transplants.md` | **M2/M3 are built (2026-09-03) and the seam holds.** `PATH_FRACTION` did not -- §5.1, the one concrete defect this decision created in existing code, **fixed 2026-09-03**. **M10** (clearance from a real sensor) is satisfied far better by 360-degree metric returns than by one ultrasonic beam |
| `CLAUDE.md` | The status table and build order reference S6 and the PiCar-X hardware path |

`HARDWARE-READINESS.md` and `PLAN-sim-hardening.md` have had staleness notes
added pointing here. Nothing else has been edited.

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

What remains open is **not design** but measurement and verification: 3.8's four
seller questions, 1.10's model-zoo check, and the re-recorded Stage 0 corpus.
**6.1's free trigger-count experiment is done** (2026-09-03) -- it is the one
item on that list that needed neither a seller nor a walk.

Kept as an index into where each landed:

| # | Question | Where it landed |
|---|---|---|
| Q1 | Is there a map, and where does memory live? | **1.5** persistence -- only the map persists, planner is a pure function · **1.6** the visual-edge mechanism is cancelled |
| Q2 | Closed or open goal vocabulary? | **1.7** closed and versioned, three verbs, unknown verbs refuse by name |
| Q3 | Who owns the stop condition? | **1.8** typed success from the planner, typed failure from the robot, lidar x bearing |
| Q4 | Is there an on-device detector, and who arbitrates? | **1.10** IMX500 on-sensor, two layers · **1.11** arbitration split by question, not authority |
| Q5 | Does the sim participate? | **1.12** yes, with synthesised detections; occlusion-aware, tri-state, noise behind a flag |

### 6.3 What it owes the twin

Per `CLAUDE.md` §7 -- a phase is done when someone holding a phone can watch it
work, not when its tests pass.

- **The lidar:** a live 360-degree clearance ring under the FPV canvas. Drive at
  a chair leg the old ultrasonic beam would have missed and watch the collar
  fire.
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
| **NPU** | Neural Processing Unit -- on-chip inference accelerator. The RK3566 has one; **the Pi 5 does not** |
| **ONNX** | Open Neural Network Exchange. A portable model format |
| **HEF** | Hailo Executable Format -- Hailo's compiled model file; the chip will not take an ONNX |
| **DFC** | Dataflow Compiler. Hailo's ONNX-to-HEF toolchain. x86-64 Linux only |
| **YOLO** | "You Only Look Once" -- single-pass object detectors. The `n` is "nano" |
| **COCO** | Common Objects in Context. The 80-class label set most detectors predict; includes `backpack` and `bottle` |
| **mAP50 / IoU** | mean Average Precision at 50% Intersection over Union -- the usual detection score, and the box-overlap measure it thresholds on |
| **XNNPACK / ncnn / TFLite** | Optimised CPU inference backends and runtimes for ARM |
| **CLIP** | Contrastive Language-Image Pre-training. Matches images to text; what makes open-vocabulary detection possible |
| **VLM / LLM** | Vision-Language Model / Large Language Model. The cloud tier -- what `/navigate` calls |
| **MiDaS / Depth Anything** | Monocular depth models. **Relative**, not metric -- which is why they do not close the gate |
| **HSV** | Hue-Saturation-Value. The colour space for an "is that backpack red" check |

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

Hardware claims about the Pi 5, the AI HAT+, the AI Camera, Coral and the lidars
are from general knowledge as of this date, **not verified against a board**.
Every one is cheap to check and should be checked before money moves.
