# Plan: perception on the car itself

Status: **design settled, nothing built** · Date: 2026-09-03, detector revised 2026-09-04, models and phasing revised 2026-09-06 · Phase IDs: **C1-C9 assigned** (1.14); the rest still unassigned

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
highest point on the robot, which *reduces* the under-furniture collision to a
~15mm slab (**not** "eliminates by construction", which 1.15.1's own stack-up
contradicts -- corrected below) and makes a 3D lidar (~$400-750) unnecessary;
the camera lands at ~10cm as a *consequence* of the stack rather than a wish;
and the camera keeps **both** pan and tilt, which adds a report-only goal type
to 1.7. **1.16 is the gap register** from that review -- eleven items, two
closed by 1.15, three of the rest changing what gets bought, and #10 (the
invalid corpus) blocked on nothing at all.

**Reviewed again 2026-09-06, and the corrections are load-bearing.** The gap
register runs to **twenty** items. The perception budget is costed for the first
time (**2.9**) and the three models do *not* fit at camera rate -- "10-30x
headroom" was read off the wrong chip's column, the same error 4.3.1 exists to
prevent. 1.14 item 5's `t_react` and the collar's missing sensor-to-bumper term
roughly **halve** the safe speed. 1.14 item 7's "the renderer needs no change at
all" is false, and it is the claim the continuous-pose cost estimate rested on.
C5 and C7 specify an **ALB that was deleted the day before they were written**.
The bill of materials under-read by ~8% and its "essential only" build could not
be assembled. And the tilt axis turns out to be triple-booked, which is why
1.15.3 now recommends dropping it for v1. **One defect was fixed in code the
same day** -- the collar had no term for where the sensor sits, which is exactly
zero today and 11-14cm the moment a lidar is fitted.

**Decided 2026-09-06: motion becomes continuous** (1.14). The robot will hold a
velocity and perceive while moving, instead of stopping between timed bursts.
That makes §2's tiering mandatory rather than an optimisation, **puts the
accelerator on the first order**, un-retires the continuous-pose half of S6 at a
fraction of its estimated cost, and makes two safety numbers wrong -- the 1.0s
watchdog and the fixed 20cm collar. C1-C9 in 1.14 are the phasing; none of them
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

**Also 2026-09-06, on the models rather than the parts.** 4.3.1 replaces the
Hailo-8 marketing chart with the **8L's own measured benchmarks** -- YOLO11
n/s/m are all downloadable HEFs for the 8L, `s` is the tier to ship, and even
`m` clears camera rate, which *confirms* 1.10 item 5 rather than weakening it.
4.2 then opens the vocabulary: CLIP scores crops, so **crops from a
class-agnostic source make the search open-vocabulary on-board**, and the class
list decides who *proposes*, never who *confirms*. Two cautions land with it --
the COCO and CLIP benchmarks are **standing-height numbers** (1.13's own
argument, never previously applied to the detector), and this is not a fallback
chain: an out-of-vocabulary object produces silence or a confident wrong label,
never a failure signal to hand off on. **2.8 walks one mission end to end**
through all three models, and 4.3's floor-segmentation note records that it now
carries three consumers and should be the compile loop's first subject.

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

### 1.3 Power: **one pack, three rails, one ground**

**Revised 2026-09-06 by review.** The original diagram said **2S**, which 3.6
had already corrected to 3S without the diagram following -- and it is the
diagram a reader builds from. The bank has been removed from the robot entirely
(it fails both of this section's own tests, below), the servos have been given a
supply they never had, and 1.16 #9's missing power budget is now written down.

```text
  [ 3S Li-ion, 11.1V nom / 12.6V full ]
       ├──────────────────────────────► Waveshare driver board (7-13V direct)
       │                                   motors, encoders, IMU, ESP32
       ├── buck #1, 5V/5A + bulk cap ──► Pi 5, Hailo, camera, lidar
       └── buck #2, 5V/2-3A ──────────► pan/tilt servos ONLY
                                   common ground, nothing else shared
```

**One pack, three rails -- and that is not a relaxation of the old rule.** The
rule is *motors never share the Pi's **rail***, and the first draft read it as
*never share the cell pack*, which is a different and much more expensive
claim. Separate buck converters with their own bulk capacitance off one pack is
how every robot of this class is built. It removes USB-PD negotiation from the
design entirely, and it gives one battery and one state of charge to reason
about instead of two.

**Motors never share the Pi's rail.** DC motors produce current spikes and
back-EMF that a USB bank's protection may simply trip on, cutting power to the
Pi mid-mission. That is why buck #1 exists and why it wants real bulk
capacitance, not why a second battery does.

**The servos get their own rail, because the plan contradicted itself about
them.** 3.6 says the Waveshare board's PWM does not drive SG90/MG90S-class
servos, so drive them from the Pi's GPIO; 1.16 #9 says SG90s on the Pi's 5V
rail is the textbook brownout. Both are in this document and only one can be
the build. An SG90 stalls at ~600-750mA, two repositioning together with inrush
is a ~1.5A transient, and 1.15.3 mandates repeated pan-settle-capture cycles --
so this is a routine event, not a fault case. Buck #2 is ~$8 and closes it.

#### The bank is a bench supply, and that is now settled rather than conditional

The first draft said the 50000mAh / 22.5W bank already owned goes on the robot
"only if it weighs under ~400g and does 5V/5A". **Checked 2026-09-06: it fails
both, and both were knowable without a scale.**

- 50Ah at 3.7V nominal is **185Wh**. At a realistic 150-200 Wh/kg that is
  **0.9-1.2kg**, not 400g. (It is also well over the 100Wh air-travel limit,
  which is a free sanity check on any bank's stated capacity.)
- **"22.5W" is a QC/PD headline measured at 9V or 12V.** Such banks deliver
  5V/3A at best per port, commonly 5V/2.4A. 5V/5A is a Raspberry-Pi-specific PD
  profile that almost no bank advertises.

So the Pi would negotiate <=3A, run in its reduced mode, and cap total USB
peripheral current near 600mA -- which an RPLidar's ~500mA nearly consumes.
That was written above as a contingency (*"if the bank turns out to be 5V/3A"*);
it is the **expected outcome**. It stays an excellent bench supply and comes off
the robot's parts list, which also removes 3.6's "already owned, 0" line.

#### The power budget 1.16 #9 says does not exist

| Load | Typical | Peak |
|---|---|---|
| Pi 5 (8GB), sustained load | 7-9W | 12W |
| Hailo-8L M.2 | 1.5-2.5W | ~4W |
| Camera Module 3 | 0.8W | 1W |
| RPLidar C1 | 1.5-2.5W | higher at spin-up |
| NVMe, if fitted | 1-3W | 5W |
| 2x SG90 | 0.5W idle | 7W stalled |
| 2x 12V gear motors at 26-44% duty | 7-14W | 30W+ at stall |
| **Total** | **~20-30W** | **~55W** |

On a typical 3S 2200-2600mAh pack (24-29Wh) at 85% buck efficiency that is
**40-60 minutes of driving**, less on carpet. **Buy two packs.** This is the
number 1.16 #9 says decides how long a test session can be, and therefore how
the re-recorded corpus gets captured.

It also corrects 1.15's aside that this is *"a chassis whose whole compute stack
is ~10W"*, used to reject the Livox Mid-360. It is 12-17W. The rejection stands
on price and mass regardless.

#### Two electrical items the plan had not carried at all

- **The TB6612FNG has no current limiting -- only thermal shutdown.** 1.2A per
  channel continuous, 3.2A peak, against a 12V gear motor whose stall is
  typically 1.5-3A. 3.8 item 7 files this as a question for the seller; it is a
  **design decision**, because 1.15.4's bumper strategy *guarantees* stall
  events (hitting things is what a bumper is for) and 1.14's velocity PID
  commands full duty into a blocked wheel. Either choose a driver with current
  sense and limiting, or cap current in ESP32 firmware -- the Waveshare board
  has current monitoring, so use it. This is a fourth job for 1.16 #6's deadman.
- **Regenerative braking has ~3% of headroom.** A charged 3S is 12.6V against
  the board's 7-13V input, and 1.14 item 5's whole design brakes actively at
  1 m/s^2. A decelerating DC motor pumps its rail up. The TB6612's absolute-max
  Vm of 15V survives it; the board's 13V rating has no room. Add bulk
  electrolytic across the motor rail (>=470uF) and a TVS -- about $2, and
  currently absent from the bill.

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

### 1.7 Goal vocabulary: **closed and versioned, four verbs**

**Was three until 2026-09-06**, when 1.15.3's tilt argument added a report-only
goal and this section was not updated with it -- nor were 1.8, 6.2 or C4, so the
fourth type had fallen out of the phasing entirely. Corrected here.

- `approach(target, stop_within_cm)` · `traverse(bearing)` · `explore(bearing)`
  · **`report(target)`**.
- **`report(target)` is 1.15.3's**: find the thing, aim the camera at it, return
  the frame and the bearing, and **succeed without driving to it**. Its outcome
  is success, not failure. It exists because a floor robot can legitimately be
  asked about a thing it cannot reach -- which is not a hypothetical but the
  exact defect that invalidated the Stage 0 corpus, where every target sat on
  furniture. It is also the terminal state `approach` degrades *into* when
  1.15.3's plane-consistency precondition fails.
- **A note on `explore`, before C4 makes these types.** The name collides with
  the twin's existing `btn-explore` (`web-twin/index.html:1460`) and with
  `brain/agent.py`'s frontier explorer -- which 2.5 assigns the *opposite* role,
  the degraded mode that runs when the planner is unreachable. Rename one of
  them in C4; a goal verb and a fallback behaviour sharing a word is how a log
  line stops being readable.
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
- **`found_not_reachable` is a success, not a failure -- added 2026-09-06 with
  1.7's fourth verb.** `report(target)` terminates on it, and `approach`
  degrades into it when the target is real but unreachable (on furniture, or
  off the lidar's scan plane per 1.15.3's precondition). A planner that reads
  "could not reach the bottle" as failure will re-plan forever against a house
  that is not going to change.
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
   script that scores it over the recorded walks on S3. If the loop exists
   on day one the Hailo is a sandbox; if it never gets built, the Hailo is a
   fixed-function part and the IMX500 was the cheaper way to get one.
   **BUILT 2026-09-09 as `tools/hailo/` -- see P6.** Not against YOLO11n,
   which 4.3.1 shows is simply downloadable for the 8L so a loop built
   against it would prove only that the loop runs, and no longer against the
   floor mask either: P5 put a harder and more decisive subject in front of
   both. It is pointed at **OWLv2**, and it **RAN 2026-09-09 on DFC 3.34.0**:
   OWLv2 translates and quantizes and does not allocate, the wall being every
   layernorm and softmax rather than the weights. See P6. The loop cost $3.20
   and answered a $400 question, which is the case 1.10 item 1 was making.
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
  frames the Pi captured, so the same HEF can be fed the walks already on S3
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

#### Two more to synthesise, added 2026-09-06

This section was written when the reactive tier consumed **one detector**. It
now consumes two more things (4.2, 4.3), and per §7 of `CLAUDE.md` the twin
cannot show C6 working without both:

- **A floor mask.** Nearly free, and by an argument this project has already
  used: the grid world knows exactly where the floor is, the same way M2
  synthesised a depth grid out of `renderer.cast_ray()`. Emit the mask from grid
  truth, not from the render.
- **A CLIP score.** Harder, but not hard. The grid world already carries *named*
  objects (`starter_house.py`'s red backpack), so a synthesised similarity is a
  comparison against that name plus a plausible margin. It must be marked
  synthesised like everything else here.

**The rule above governs both**: the sim leg tests the *consumers* -- does a
match gate a trigger, does a mask reach the drive loop, does arbitration behave
when the mask and the lidar disagree -- and **never** the models. No claim about
CLIP's accuracy or a mask's quality may come from a synthesised one. Those come
from real frames, which is 1.16 #10.

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
across every room type, and that corpus does not exist: the 39 walks on S3 are
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
The rest of 4.9's recommendation is unchanged -- an **8L in M.2 module form**.

**Corrected 2026-09-06, hours after it was written: "10-30x with headroom for
the larger YOLO tiers, floor segmentation and CLIP" is wrong, and it is wrong in
the exact way 4.3.1 exists to prevent.** The figure traces to 4.9's tier table,
which sources it from *"431 FPS batch-1 reported on the **Hailo-8**"* and
*"~137 FPS **batch-8** on an 8L"* -- the wrong chip's column, and a throughput
number standing in for a latency one. 4.3.1 is the section that caught precisely
this and said so: *"reading the 8L's table rather than the 8's settles three
things this section had been asserting from the wrong column."* Then 1.14 and
4.9 asserted from it again.

Against 4.3.1's own verified 8L batch-1 numbers and the detector this plan
actually ships (YOLO11s, 92 FPS): **92 / 30 = 3.1x at camera rate**, 6.1x at
the 15Hz floor. Not 10-30x, and not headroom -- see the budget in 2.9, which is
where the three models are costed together for the first time. The conclusion of
this item is unchanged and if anything strengthened: 3.1x from the accelerator
against **0.2-0.4x** from the Pi's own cores is still the difference between
having a perception tier and not having one. **Caveat added 2026-09-06: that figure was
costed for a single nano detector.** Running the three together spends the
headroom rather than leaving it spare -- see C6's note in the phasing below, and
4.3's *"several resident"*, which is a claim about **memory, not throughput**.

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

##### Both terms above are optimistic, and the review that found it is worth recording

**Revised 2026-09-06.** The table's arithmetic is exact -- every cell recomputes,
and 0.2v + v^2/2 = 0.2 does solve at 0.46 m/s. But `t_react = 200ms` was
**asserted rather than composed**, and the collar is measured from the wrong
origin. Both errors cut the same way.

**(a) 200ms is a ceiling, not a budget.** Built from its parts:

| Term | ms |
|---|---|
| RPLidar C1 scan period at 10Hz (avg 150 to a given bearing's re-measurement) | 100 |
| Serialise and publish the feature over C5's transport | 2-10 |
| Drive-loop period at 30Hz | up to 33 |
| Pi -> ESP32 serial, plus the ESP32's own loop | 5-15 |
| Motor current reversal, mechanical | 20-50 |
| **Collar path, total** | **160-260ms** |

So `t_react` is **250-300ms**, not 200. At 0.3s the collar is good for
**0.40 m/s**.

**(b) The collar has never had a footprint term, and today that is harmless.**
`path_clearance()` returns a range and `check_and_execute()` compares it
directly to `min_distance_cm` (`robot/safety.py:257`). The PiCar-X's ultrasonic
points forward from the front of the chassis, so sensor origin and bumper
coincide and the missing term is exactly zero; the sim casts rays from a robot
that is a point and has no bumper at all.

**A 360-degree lidar moves the sensor origin to the middle of a 228 x 148mm
deck** -- roughly **11-14cm behind the leading edge**. A reading of "20cm" is
then 6-9cm of real gap, which is inside the travel of the compliant bumper that
1.15.4 calls the last resort. And it changes **silently**, because the same
number keeps working with a different physical meaning.

**Fixed in code 2026-09-06, while the number is still zero.** `robot/safety.py`
carries `SENSOR_TO_BUMPER_CM`, `path_clearance()` returns bumper-relative
clearance, `config/robot.yaml` has `safety.sensor_to_bumper_cm: 0.0`, and
`GET /depth` publishes it so the twin's strip shows what the veto reads rather
than re-deriving it. `tests/test_depth_veto.py` pins that a real mount turns a
passing clearance into a veto. This is S5's argument applied on purpose: pin the
property while the flag is off, or it sits undiscovered until a sensor moves.

**(c) Composed, the two terms roughly halve the answer.** At `t_react` = 0.3s
with 11-14cm of the collar spent on the offset, the usable collar is ~7cm and
the safe speed is **roughly 0.15-0.2 m/s** -- below this section's own "spec for
0.5, run at 0.3", and it inverts 3.8 item 6's conclusion that the 333 RPM
variant "covers 1.14's spec comfortably." The preferred fix (speed as a function
of measured clearance) survives intact; **the constant in it is about half what
this section assumed.**

**(d) And there is no closing-speed term anywhere.** Every row above is a
stationary obstacle. A person walking toward the robot at 1 m/s makes the
closing speed 1.5 m/s and the required stopping distance ~1.4m. Dynamic
obstacles are absent from the whole safety analysis, and a house has them.

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

**`sim/renderer.py`'s primitives need no change.** They are already
continuous: `cast_ray(layout, px: float, py: float, angle: float)` and
`render(..., px, py, base_angle)` take floats and radians. S2's port was written
against a float pose from the start.

**Corrected 2026-09-06: the stronger claim this section made -- that
`sim/renderer.py` needs no change *at all*, and that the discretising step is
*a* two-line conversion at `sim/mock_robot.py:227-228` -- is false, and it is
the claim the cost estimate rested on.** Those two lines are real and exact, but
they are inside `get_depth_grid()`. The **camera** path has its own copy:

```text
sim/renderer.py:273-279   view = world._view_heading()
                          return render(world.layout, world.objects,
                                        world.robot_x + 0.5, world.robot_y + 0.5,
                                        HEADING_ANGLE[view.name], ...)
```

`MockRobot.get_camera_frame()` reaches it through `render_world_base64()`
(`sim/mock_robot.py:158`), and `render_world_image` / `render_world` /
`render_world_base64` all take a `GridWorld` and discretise it. Two further
discrete sites the estimate omitted: `grid_world.distance_ahead()`
(`:147-155`) marches integer cells along `_view_heading().value`, and
`frame_description()` (`:163-186`) reads three cells ahead the same way.
Neither is a boundary conversion.

**So C2's "delete the boundary conversion" is plural**, and the *"far cheaper
than S6 assumed"* conclusion needs re-costing against four sites rather than
one. It is still cheaper than S6 -- the ray-casting mathematics genuinely is
float-native already, which was the expensive half -- but "no change at all" was
the wrong summary of "no change to the primitives".

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

The chassis (1.1 was already differential), the lidar (1.2), the power design
(1.3 -- one pack and three rails since the 2026-09-06 review), the camera, the
arbitration order (M4), the goal vocabulary (1.7), and
1.13's room-identity split. **And the cloud VLM stays exactly where 2.1 put it**
-- event-driven, ~0.5Hz, off-board. Continuous driving does not make the cloud
call faster; it makes it *stop blocking the wheels*, which was always the point
of tiering.

#### Scope, honestly, and a phasing -- **C1-C9, assigned 2026-09-06**

**This is the largest change in the plan since the chassis decision** --
interface, safety, sim, runner and twin, plus a reactive drive loop that does
not exist yet (2.7).

**It was five provisional labels until 2026-09-06, and it was five because
2.8's mission had never been walked step by step against the repo.** Doing that
found four things the phasing did not cover at all -- the goal vocabulary as a
*type*, the feature transport, the cloud contracts, and `brain/planner.py`
itself -- plus one interface method (range at a bearing). The old C4 and C5 are
now **C6 and C9**; C1-C3 keep their numbers and their scope.

| | What | Depends on hardware? |
|---|---|---|
| **C1** | **The interface pass.** `set_velocity` on `RobotInterface`, all five backends, contract tests. **Plus three items**: a pose/odometry method (1.16 #5); a timestamp (1.16 #3) that rides the published *feature* and not only the sensor read -- 2.8's fusion is three-way (bearing from a detection, range from a scan, a CLIP score from a crop of a frame) and 2.7's rule is *features, not frames*; and **range at a bearing**, which `approach` needs in C6 and which `get_depth_grid()` does not give (M2's grid is forward-facing; a 360 ring sampled at an arbitrary bearing is a different question). One conformance pass, not three | no |
| **C2** | Continuous pose in `grid_world.py`; delete the boundary conversion; press-and-hold D-pad in the twin | no |
| **C3** | Watchdog timeout down to ~150ms; clearance-derived speed replacing the fixed collar, **against 1.14 item 5's corrected constants** -- `t_react` ~250-300ms and a sensor-to-bumper offset, which roughly halve the safe speed. **Plus the ESP32's own deadman** (1.16 #6) -- the innermost guard, and the only one that survives the Pi locking up. It needs a drill, per §7 | **yes, partly** -- see below |
| **C4** | **The goal vocabulary as a type.** 1.7's **four** verbs (the fourth is 1.15.3's report-only goal, which had fallen out of this phasing) and 1.8's stop conditions **including `found_not_reachable`**, plus 1.15.3's plane-consistency precondition on `approach` and the `explore` rename, as data with validation -- **`grep` finds no `approach`/`traverse`/`explore` anywhere in the repo today; §1.7 is prose.** Pure, no I/O, testable alone, and it unblocks C5-C9. Nothing produces or consumes a goal until this exists, which is what made the old C4 ("holding a goal") rest on air | no |
| **C5** | **The feature transport.** 2.6 requires perception to reach the brain over HTTP like everything else: routes on `robot/server.py`, `RemoteRobot` over them, conformance cases, **and the routing entries** whose absence has shipped five times as a silently dead feature. **Corrected 2026-09-06: that is no longer an ALB path pattern.** Every ECS/ALB stack was deleted 2026-09-05, one day before this phasing was written; routing is now a **CloudFront behaviour *and* an API Gateway route** (`cloudformation/serverless.yaml`), so the failure mode has *doubled* rather than disappeared -- two tables to keep in step. `tests/test_serverless_routes.py` is the successor to `test_alb_routes.py` and is where the drift check lives. Carries **1.12's synthesised** detections, floor mask and CLIP scores -- so it is provable in the twin with no HEF, no accelerator and no corpus. **Note this phase is larger than "transport"**: producing those synthesised features in `MockRobot` -- with 1.12's occlusion ray, its three-way output and its noise flag written *with* the flag -- is the bigger half and had fallen between C5 and C6 | no |
| **C6** | **The reactive drive loop as its own process** (2.7) -- *was C4* -- now holding **and executing** goals, with C4's type, C5's pipe and C1's methods already in place. It hosts a *pipeline*, not a detector: detector -> crops -> CLIP -> match (4.2) plus floor segmentation (4.3), all owing 2.1's 15-30Hz row, so it also owes a **throughput budget and a scheduling policy** (segmentation has no reason to run at the detector's rate) | no -- against synthesised features. **The real HEFs are hardware day**, and so is confirming the budget on the real part |
| **C7** | **The two cloud contracts.** `/navigate` returns an *action*; 2.8 needs a call that returns a **goal** (step 1) and one that answers **identity and reachability** (step 4). New routes on `service/vision_analyze/`, server-side allow-lists, validation at mission start the way M1 validated `model_id`, and their own **CloudFront behaviour + API Gateway route** (not ALB patterns -- see C5) | no |
| **C8** | **`brain/planner.py`** -- the deliberation tier itself, over `MissionMemory.as_context()`, emitting C4 goals through C7's contract. 2.3 already says *"it is `brain/planner.py`"*; `CLAUDE.md` calls it the main hardware-path gap; the file does not exist | no |
| **C9** | **Deliberation becomes event-driven** against 2.4 -- *was C5* -- including the `cold search` trigger, whose "found nothing for a while" condition inherits §6.1's hysteresis requirement like every other field-derived trigger | no |

#### Why this order, so it can be argued with

Numbering is sequencing, so the dependencies are stated rather than implied:

- **C1 first** because everything downstream calls it, and because four interface
  additions in one conformance pass is cheaper than four passes.
- **C2, C3 before anything perceptual** -- unchanged reasoning: C3 in particular
  is *"much better discovered in the sim than on a chassis moving at half a metre
  per second."* Safety precedes capability.
- **C4 before C5-C9** because it is the vocabulary all four speak. It is also the
  cheapest phase here, which makes putting it late strictly worse.
- **C5 before C6**, which is the ordering most likely to be questioned. The
  instinct is to build the pipeline and then ship its output. The argument for
  the reverse is M2's, which this project has already run once: `get_depth_grid()`
  landed with an honest all-unusable default and a synthesised implementation
  *before* any depth sensor existed, and the twin could draw it immediately. Build
  the pipe against 1.12's synthesised features and C6 plugs into something already
  proven; build it after, and the transport's own failure mode -- a missing ALB
  pattern, five times now -- surfaces while the pipeline is also new.
- **C7 after C6, not before**, which is the other arguable call. C7 is independent
  of every motion phase and could be done at any point. It is placed here because
  **a reply shape should be designed against a consumer that exists**: settle what
  an executor actually needs from a goal (C6) before fixing the contract that
  delivers one. The cost of being wrong the other way is a route on a deployed
  service, with an allow-list and an ALB rule, that nothing can use.
- **C8 before C9** because C9 schedules C8. The old C5 read *"deliberation becomes
  event-driven"*, which presumed a deliberation tier existed; it never has.

**None of C1-C9 needs the robot** -- the property the five-phase version had, kept
deliberately, **with two asterisks rather than the one this claimed.**

C6's was acknowledged: it is provable in the twin against synthesised features,
and the real HEFs and the real throughput measurement are hardware day.

**C3's was not, and it is unqualified.** The ESP32's deadman lives on the
Waveshare board, which is unordered, and nothing in this repo simulates a serial
peer -- so a deadman on a microcontroller you do not have cannot be built and
its drill cannot be run. Either that half moves to hardware day, or C3 grows a
simulated serial peer first, which is real work nobody has costed.

**And C3's other half has no consumer until C6.** Clearance-derived speed
replaces a fixed collar, but nothing holds a velocity until the drive loop
exists; 1.14 item 9 names the twin proof as *"watching it cross a room without
stopping, with the clearance-derived speed visible as it slows near a wall"*,
which is a **C6** observation. As ordered, C3 ships a rule nothing exercises and
a proof nobody can press -- which §7 forbids. The "safety precedes capability"
argument justifies the *rule* landing first; it does not exempt the phase from
shipping something to press. Ship C3's speed law with a **pressable stand-in**
(the D-pad's commanded speed clamped by live clearance) or accept the exemption
in writing.

**C1 also silently contains a piece of C5.** `RemoteRobot` is one of the five
backends, so `set_velocity` needs a route on `robot/server.py`, a client, and
its routing entries -- exactly the work C5 is defined as owning, and exactly the
failure that has shipped five times. Do it once, in C1, and say so.

**One concrete C1 hazard no phase names.** `_HaltGate.MOVEMENT`
(`control/mission_runner.py:125-128`) is a **hardcoded tuple of method names**.
A `set_velocity` not added to it passes straight through the mission-end gate --
the stop that Stage 2 promises *"is enforced at the robot, not just in the
loop"* would silently stop enforcing for the one verb that holds a velocity.
Same silent-fallback class as 1.7 and M7. **Nor does any of it need 1.16 #10's re-recorded corpus**: 1.12's
synthesised features carry the whole sequence, so the re-recording proceeds in
parallel rather than in series.

**A note on what the 2026-09-06 perception changes did and did not do.** They are
about perception *content* -- which model, which vocabulary, which crops -- while
C1-C3 are about motion and the loop, on opposite sides of the seam 2.6 exists to
keep. A perception rewrite that forces no change to `set_velocity`, the watchdog
or the collar is evidence the layering is right. What it *did* expose is that the
phasing had been written for the motion half only, which is what C4-C9 above
repair.

**The one number to watch is C6's.** 4.9's tier table gives the 8L *"10-30x with
headroom"* over 2.1's perception row -- **costed for a single nano detector**.
YOLO11s at 92 FPS (4.3.1) plus per-crop CLIP plus a floor mask spends that
headroom rather than leaving it spare, and 4.3's *"several resident"* is a claim
about **memory, not throughput**. Nothing in this plan has yet costed the three
together. If they do not fit, the resolutions are ordinary -- run segmentation at
a fraction of camera rate, gate CLIP on the crop count -- but the budget has to be
written down before C6 is built against an assumption of slack.

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

#### The rule is contradicted by 1.15.1's own stack-up -- **corrected 2026-09-06**

`h_lidar = H_max` is **impossible by construction, and this document's own table
says so two subsections later**: the scan plane lands at ~123-140mm while
`H_max` is listed at ~155mm, *the top of the lidar body*. The C1's plane sits
~25-30mm inside a 41.3mm housing, so **11-18mm of lidar is always above its own
scan plane**. 1.15.5 item 1 notices the fact -- *"whatever body sits above the
plane is unprotected by the rule in 1.15"* -- and never propagates it back here.
The symbol also changes meaning between the two: `H_max` is "top of the robot"
in the rule and "top of the lidar body" in the table.

**The honest statement is that the class is reduced to a ~15mm slab, not
eliminated.** And that slab is the original hazard in miniature: a low shelf
lip, a bed frame or a sofa rail at 125-155mm is invisible and is struck by the
top cover of the $99 sensor. Mitigation is a chamfered or sacrificial top cap,
and accepting that the sensor is the part that takes the hit.

#### Four cases the argument misses, and the first is the largest

1. **Pitch, which is unmentioned anywhere in this plan.** The rule is a
   statement about a *static horizontal* plane. A 2WD chassis with a caster
   pitches under acceleration and braking (1 m/s^2 against a ~8cm centre of
   gravity) and over thresholds (a 10mm lip against a 32.5mm wheel radius is a
   transient of several degrees). **At 5 degrees of pitch the plane is
   displaced +/-17cm at 2m range** -- diving into the floor for spurious near
   returns and a spurious veto, or lifting over a real obstacle. This is the
   largest single error source in the whole geometry. The mitigation is cheap
   and already bought: the Waveshare board's IMU gives pitch, so reject or flag
   scans while it exceeds a threshold -- **exactly M3's rule for an unusable
   zone**, one sensor over.
2. **Dynamic obstacles.** The whole analysis is of stationary furniture; 1.14
   item 5 has no closing-speed term either.
3. **The near-field blind box, sized below.**
4. **Descending profiles.** *"Entirely above `H_max` -> the robot fits under"*
   is a claim about a volume the lidar never samples, and it holds only where an
   object's lowest point is where the plane crosses it. True of table legs;
   false of a recliner footrest, a wall-shelf bracket, a hanging coat, or a
   chair pushed in at an angle.

#### The under-plane residual, sized

With the camera at 12cm and a level 41-degree vertical field, the lower ray
strikes the floor at `0.12 / tan(20.5 deg)` = **32cm ahead**. The lidar plane is
at 12-14cm. So the volume from the bumper out to **32cm**, below **12cm**, is
seen by **nothing** -- not the lidar, not the camera, not the floor mask. At
0.5 m/s the robot crosses it in 0.64s, which is *less than* 1.14 item 5's
corrected reaction budget.

**That is the specification for 1.15.4's ToF pair, and it makes them mandatory
rather than "strongly recommended".** It gets worse if the camera captures
16:9 -- see 1.15.3.

What survives beyond it is the rest of the **under-plane residual**: shoes,
cables, thresholds, pet bowls, a low sofa rail. That is a real gap and it is
answered in 1.15.4, not by the lidar.

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
exactly the blind spot 1.15.4 then has to cover.

**And that means the pedestal is the wrong lever -- corrected 2026-09-06.**
Going 123 -> 140mm to buy camera clearance makes roughly **14% more of the
frontal area blind**, immediately after a section spent eliminating that blind
volume. The better fix is named here in a single clause and then not adopted:
*a shorter pan/tilt bracket buys back margin at both ends.* **Adopt it.** Mount
the camera on the front edge of the chassis at **70-90mm** rather than on a
65mm mast. That lowers `H_max`, shortens the pitch lever arm above, and brings
the near floor into frame -- which is the blind box in 1.15 and the floor mask
in 1.15.3, both improved by the same change. It costs the tilt range that
1.15.3 argues for, which is the trade that subsection now has to answer.

#### 1.15.3 Pan **and** tilt, both angles in the frame

**Decided 2026-09-06, after an argument that reversed the first answer.** The
first take kept pan and dropped the tilt servo, reasoning that tilt only buys
looking up onto furniture, which Stage 0 ruled out as a navigation target. **That
was a category error: a navigation constraint used to rule out a perception
capability.** Stage 0 says the goal you *drive to* must be reachable. It never
said the robot should be blind above the floor.

The geometry settles it. Camera at 12cm with Camera Module 3's ~41° vertical
field, pointed level -- the top of frame sits at `12cm + 0.374 x distance`.
**(Two caveats added 2026-09-06.** 1.15.1's stack-up says the lens lands at
~**100mm**, not 120 -- at 10cm the table below reads 47/85/122cm, conclusions
unchanged, but 1.15.2's clearance analysis uses 110mm and this uses 120mm and
the pedestal cannot be designed until one number is picked. And **41° is the
4:3 full-sensor figure**; capture 16:9 and the vertical field drops to ~33°,
which moves "top of frame at 1m" from 49cm to 44cm and pushes the nearest
visible floor from 32cm to **41cm** -- making 1.15's blind box worse. **Pin the
capture mode.)**

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

Three consequences, all worth having:

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

#### The tilt axis is triple-booked -- **found 2026-09-06, and it is a conflict, not a number**

Three consumers want the tilt axis in three incompatible positions, and no
section had put them beside each other. At 12cm with a 41-degree vertical field:

| Tilt | Nearest floor in frame |
|---|---|
| **+30°** (this subsection, for tabletops) | **never** -- the lower ray is +9.5°, the floor is not in the image at all |
| 0° | 32cm |
| −10° | 20cm, i.e. just reaches the collar |
| −20° | 14cm |

**Tilt-up and the floor mask are mutually exclusive.** And the floor mask is not
a nice-to-have: 1.15.4 lists it as mitigation #1 for the under-plane residual
and 4.3 promotes it to *safety-relevant*. It needs **at least −10°** merely to
see as far in as the collar distance.

Meanwhile a reposition costs ~0.3-0.5s (0.1s/60° plus settling), during which
the perception tier's bearing output is invalid -- against 2.1's 15-30Hz row.
**That is a servo duty cycle nobody has written**, and it belongs in C6's
scheduling policy beside 2.9's throughput budget.

**Tilt also breaks `approach`'s stop condition, which 1.8 and this subsection
were never reconciled about.** A 2D lidar cannot range anything off its plane.
Detect a bottle on a console table with the camera tilted +30°, and the lidar at
that azimuth returns the range to the **table edge** -- or to a person walking
past at the same azimuth. `approach` then terminates on the wrong object with no
way to notice. So `approach` needs a **plane-consistency precondition**: if the
detection's elevation is inconsistent with the scan plane at the returned range,
the goal degrades to this subsection's own report-only type rather than being
driven. That is a genuine addition to 1.8, and it is C4-shaped.

**Which reopens the decision, honestly.** 1.15.2 now argues for a low fixed
front-edge camera at 70-90mm; this subsection argues for a 65mm mast with two
axes. The tilt case remains sound in the abstract -- *a navigation constraint
must not rule out a perception capability* -- but its price is now legible:
raising `H_max` and the blind volume with it, a time-varying extrinsic (1.16
#4), open-loop bearing corruption (1.16 #7), the first feature to reach hardware
with no twin representation (1.16 #8), a servo schedule that competes with the
perception row, a new precondition on `approach`, and up to $50 if bearing
accuracy forces ST3215 bus servos. **The recommendation is to drop the tilt
servo for v1**: fix the camera low and at a slight downward pitch, which serves
the floor mask and the near-field blind box for free, keep pan, and revisit tilt
when a mission actually fails for want of it. That is the plan's own rule from
1.7 -- *add it when the log shows the planner reaching for it, a measured signal
rather than a guess* -- applied to a servo instead of a verb.

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
hardware day. Three were verified against the code, not guessed -- #3, #5 and
#8, of which only the last two are labelled as such below. **Extended to twenty
items on 2026-09-06** by a second review; see the table after this one.

| # | Gap | Status |
|---|---|---|
| 1 | 2D lidar plane routes the robot under furniture into its own mast | **CLOSED by 1.15** -- lidar is the highest point |
| 2 | Camera height determined by the stack, not chosen; the 10-13cm corpus instruction assumed otherwise | **CLOSED by 1.15.1** -- it lands at ~10cm, and is now a measurement |
| 3 | **Time synchronisation does not exist** | **OPEN.** Parked, a frame + scan + encoder count + pan angle were all "now". At 0.4 m/s with a 10Hz lidar they are up to 100ms and 4cm apart, so 1.8's *"lidar range at the detector's bearing"* mixes a bearing from one pose with a range from another. **Nothing in `RobotInterface` carries a timestamp.** Belongs in C1 |
| 4 | **Camera-lidar extrinsic calibration unspecced**, and pan/tilt makes it time-varying | **OPEN.** 1.8's fusion assumes a known transform between the two sensors; with two servo angles it is a function, not a constant. No procedure, no accuracy target |
| 5 | **`RobotInterface` reports no pose or odometry** | **OPEN, verified**: eleven methods, none says where the robot is. Discrete driving counted moves instead. A drive loop holding a goal must know how far it has got -- a new method in `get_depth_grid()`'s class (honest default, five backends, conformance suite). **1.14's phasing missed it until C1-C9 was assigned.** Note also that encoders measure *wheel* rotation, not ground travel: on carpet that drifts, the IMU fixes only heading, and scan matching is the real answer -- which pulls 3.3's (b+) forward |
| 6 | **The ESP32 needs its own deadman -- a fourth failsafe** | **OPEN.** B3.1/2/3 cover the robot watchdog, the vision budget and a hung tick. None covers the Pi-to-ESP32 link dying while the ESP32 holds a velocity. It is also the **only** guard that can stop the wheels if the Pi itself locks up. Per §7 of `CLAUDE.md` it needs a drill |
| 7 | **Open-loop servos: pan/tilt is commanded, not measured** | **OPEN.** SG90s have no feedback. A stalled, slipped or knocked servo makes every bearing wrong by that amount with nothing to notice -- the same silent-corruption class as the stray-frame bug. Since bearing is the camera's one job (1.11), this may justify **ST3215 bus servos** (position feedback, driven natively by the Waveshare board). Minimum: startup homing plus a plausibility check |
| 8 | **Tilt cannot be represented in the twin at all** | **OPEN, verified**: `renderer.render(layout, objects, px, py, base_angle, ...)` is a 2D raycaster with **no pitch parameter**, and `grid_world.look_left()` sets `pan = -1` -- pan is tri-state snapped to cardinal headings, not a continuous servo. Tilt would be **the first feature to reach hardware with no twin representation**, which `CLAUDE.md` §7 forbids. Either the renderer gains a pitch (real work; it is 2D by construction) or tilt takes §7's written once-per-phase exemption. **Do not let this one pass silently** |
| 9 | **No power budget and no runtime estimate** | **OPEN.** Pi 5 under load + Hailo + lidar + camera + servos, now *sustained* rather than bursty. A Pi 5 with a HAT wants 5V/5A and many banks will not hold 25W -- and 3.6 carries the bank as "already owned, 0". Servos are motors, so 1.3's own rule about keeping motor noise off the compute rail applies to them; SG90s on the Pi's 5V rail is the textbook brownout. Runtime decides how long a test session can be, which decides how the corpus gets recorded |
| 10 | **The recorded corpus is invalid, and three things now wait on it** (added 2026-09-06) | **OPEN, and the only item here blocked on nothing at all.** Every walk on S3 was shot at standing height with the target on raised furniture (`CLAUDE.md` Stage 0) -- a viewpoint the robot will never have, at a task a floor robot cannot perform. It was already invalidating the five-wording prompt result. It now also blocks **4.2's caveat** (whether a COCO detector and CLIP work at all at 10cm -- 4.3.1's `45.1 mAP` is a standing-height number) and **4.3's floor-segmentation score**, which is the compile loop's first subject. Needs no seller, no part and no hardware: a phone on a wheeled rig, a target on the floor, landscape locked, rig height written into the walk's own `meta` note. **Four to six short walks** |
| 11 | **Does the floor mask get a veto?** (added 2026-09-06) | **OPEN.** 4.3 says floor segmentation *"does not replace the lidar, which sees a chair leg the mask cannot"* -- but **"does not replace" is not "has no vote"**, and the plan never says which. M3 already built the precedent one sensor over: `path_clearance()` reduces a depth grid to one number and a failed zone never enters the comparison **in either direction**, because as a distance it stops the robot on every dropout and as clear it drives through what the sensor could not see. A mask has exactly that tri-state and exactly that trap. Decide it explicitly: an input to the drive loop's steering, or a veto beside the collar. **If a veto, it is C3-shaped, not C6-shaped**, and `robot/safety.py` grows a second consumer -- which 5.1's `PATH_FRACTION` bug says is where this project's sensor reductions go wrong |

#### Extended 2026-09-06, by a second review

| # | Gap | Status |
|---|---|---|
| 12 | **The collar compared a sensor-frame range to `min_distance_cm`, with no term for where the sensor sits** | **CLOSED 2026-09-06, in code.** Exactly zero for a front-mounted ultrasonic, 11-14cm for a deck-centre lidar -- so "20cm" would have become 6-9cm of real gap, silently, on fitting day. `SENSOR_TO_BUMPER_CM` + `safety.sensor_to_bumper_cm`, `path_clearance()` returns bumper-relative, `GET /depth` publishes the offset, `tests/test_depth_veto.py` pins it. Fixed while the number is still zero |
| 13 | **The perception budget does not fit at camera rate, and "10-30x headroom" came off the wrong chip's column** | **CLOSED as an analysis, OPEN as a C6 requirement** -- 2.9. YOLO11s + segmentation + CLIP is 3.1x, not 10-30x, and the three together need a rate-division schedule. Also: "several resident" is a *streaming-bandwidth* problem on a part with no on-chip DRAM, not the benign memory claim 1.14 called it, and the NVMe contends for the same lane |
| 14 | **Pitch is absent from the entire lidar geometry** | **OPEN.** ±17cm of plane displacement at 2m for 5° of pitch -- the largest single error source in 1.15, and it appears under exactly the braking 1.14 introduces. The Waveshare IMU already measures it; the fix is M3's own rule (reject or flag the scan) one sensor over |
| 15 | **The fusion timing analysis picks the benign error term** | **OPEN.** 1.16 #3 reasons about 4cm of translation, but 1.8's fusion is a *bearing* operation: 90°/s gives 9° in 100ms = **31cm of lateral error at 2m**, and it appears while pivoting, where the translation term is zero. Also unaddressed: **scan de-skewing** (a 10Hz spinning lidar's samples are not simultaneous, which directly breaks `traverse`'s "watch the doorframe go past"), and C1's timestamp is specified on the **publish** event where fusion needs the **acquisition** event -- plus one shared clock across Pi/ESP32/lidar and an interpolatable pose history, or a timestamp has nothing to query against |
| 16 | **The camera-lidar lever arm makes the bearing comparison invalid, not merely imprecise** | **OPEN, and it re-shapes 1.16 #4.** Bearings measured from origins ~11cm apart are not comparable: a target at 0.5m and 30° off-axis is at 24.7° from the lidar and 30° from the camera -- 5.3° of systematic error, growing as range shortens, i.e. worst exactly while `approach` is closing. The fix is **formulating the fusion in the body frame**, not calibrating harder: it is a rewrite of 1.8's sentence. And 1.16 #7 makes calibration unsolvable as stated anyway -- you cannot calibrate a transform whose parameters you cannot observe, and SG90 repeatability of ±1-2° is ±5-16cm at 3m. Short of bus servos: **trust bearing only at a mechanically-homed pan = 0**, and use pan only while stopped |
| 17 | **The tilt axis is triple-booked and unscheduled** | **OPEN** -- 1.15.3. Tilt-up and the floor mask are mutually exclusive, the mask needs ≥−10° to reach the collar distance, and each reposition costs 0.3-0.5s of invalid bearing against a 15-30Hz row. Belongs in C6's scheduling policy. **The recommendation is to drop the tilt servo for v1** |
| 18 | **The TB6612FNG has no current limit, only thermal shutdown** | **OPEN** -- 1.3. 1.2A/channel against a 1.5-3A stall, with a bumper strategy that *guarantees* stalls and a velocity PID that commands full duty into a blocked wheel. 3.8 files it as a seller question; it is a design decision, and the Waveshare board's current monitoring is the cheap answer |
| 19 | **No physical emergency stop** | **OPEN, and absent from the plan entirely.** A robot that holds a velocity, whose innermost deadman (#6) is an unbuilt firmware feature on an unordered board. A latching button in the motor rail is ~$3 and is not in the bill |
| 20 | **No plan for a dead-on-arrival or backordered part, and no tool inventory** | **OPEN.** 1.15.5 asks for three bench measurements with no calipers in the bill; 3.6 prices a 3D-printed pedestal at "0 with a printer" without asking whether there is one, and the pedestal blocks the build. Seven seller questions, one of which (3.8 #6) cannot be fixed after delivery, and no plan for "the answer is wrong" |

**Five of these change what gets bought** -- #7 (which servos), #9 (the power
supply, now settled in 1.3), 1.15.4's bumper and ToF pair, #19's e-stop, and #17
(dropping the tilt servo). The rest are design work that can proceed while parts
ship, except #12 and #13, which are done.

**#10 is in neither group, and that is the point of listing it here.** It is not
design and it is not a purchase question -- it is a *measurement*, and unlike
every other row it waits on nothing: no seller reply, no delivery, no decision.
It is also the only row that can invalidate work already done rather than merely
delay work not yet started, which is how it earned a place in a register
otherwise about hardware. It had been tracked only in `CLAUDE.md`'s Stage 0
notes, where a reader of this plan would not find it.

**A pattern behind several of these, recorded in 4.10 and pointed at from
here** so a reader of the register finds it: three of this document's sections
argue that off-robot evaluation is possible -- it is a *deciding* argument for
the Hailo over the IMX500 -- and none of them assigned it to a phase. 1.10 item
1's compile loop was in the same state. **A reason to buy does not
automatically become a thing to do**, and this document had no mechanism
carrying a capability from a purchasing section into a phasing one. When a
section argues that something is possible, ask in the same breath which phase
owns doing it.

**#11 is a decision, not a discovery**, and it is listed because the plan
currently contains both halves of it and neither is marked as the answer. It
should be settled before C3 or C6 is built, since which phase owns it depends on
the answer.

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
| **Cold search** (added 2026-09-06) | runner / reactive | nothing on-board has a lock -- see below. Runs the opposite way to candidate sighting: **cloud proposes, on-board tracks** |
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

#### Cold search -- the trigger that runs the other way

**Added 2026-09-06.** Candidate sighting is **on-board proposes, cloud
confirms**, and it therefore requires the detector to fire first. For a target
outside COCO's 80 -- and absent 4.2's open-vocabulary crop path -- it never
does, so the trigger never fires and **the robot can drive past its target
indefinitely without ever thinking to ask.** A trigger table with only that
direction in it has no cold-start entry.

The inverse: at mission start, on a room change, or after a stretch of finding
nothing, send **one** frame up and ask *"is what I am looking for in this room,
and roughly which way?"* The VLM is bounded by no class list, which is exactly
the property being bought. On-board then carries it, because at the 3.6s/step
measured to Opus 4.5 the cloud cannot steer anything.

**Ask it *what*, never precisely *where*.** Stage 0's whole finding is that the
model is unreliable about position and distance -- `obstacle_ahead`
uncalibrated across models, no metric depth in a monocular frame -- which is why
`bearing-only` (M1) exists at all. So the reply is a coarse *"off to your
left"*, and the bearing that gets driven on comes from something geometric.
Same division of labour as 2.1: the cloud owns identity, never geometry.

Cost is bounded the way every other row here is -- it is an event, not a timer,
and §6.1's hysteresis applies to its "found nothing" condition as much as to
`target_visible`.

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

### 2.8 A mission, end to end -- the worked example

**Added 2026-09-06.** Everything above is stated as rates, seams and rules.
This is the same design as one mission, in order, because the split is much
easier to get wrong when it is only ever described in the abstract. Nothing
here is new -- it is 2.1's three rates, 2.4's triggers, 1.8's stop conditions
and 4.2's two CLIP paths, walked through once.

#### The cast, and the one question each is asked

| | Runs where | Rate | The question it answers -- **and only this one** |
|---|---|---|---|
| **lidar** | robot | 10-50 Hz | *how far?* Knows nothing about what anything is |
| **detector** (YOLO11s) | robot, Hailo | ~92 FPS (4.3.1) | *where are the objects?* -- boxes, plus one of COCO's 80 words |
| **CLIP** | robot, Hailo | ms per crop | *is this crop the thing we want?* -- against arbitrary text |
| **the VLM** (Opus 4.5) | AWS, `service/vision_analyze/` | ~3.6 s/step, paid | *what should we do about it?* -- reasoning, in sentences |

**The failure this prevents is the one vision-picar has today**: 2.1's
*"collapses all three into one"* -- a single cloud call asked for bearing and
distance and the next action at 1-3s, with its answer driving the robot.

#### The mission

Target: **"red backpack"**.

**1. Start -- one paid call.** CLIP encodes the target string once and the
vector is cached for the mission (4.2: the text encoder runs on the Pi CPU, and
the string does not change). One frame goes up with it. The VLM replies with a
goal in 1.7's vocabulary -- *"living room, no backpack visible, the doorway at
+0.4 rad probably leads to the hall"* -> `traverse(doorway at +0.4)`. This is
2.4's `mission start` trigger, and 2.5 notes it is the one genuinely blocking
call -- or it is not, if the opening default is "look around".

**2. Driving -- no calls at all.** The reactive tier holds that goal and drives
continuously (1.14). The lidar keeps the collar honest at ~50 Hz; the detector
looks at every frame at ~92 FPS. **The cloud is silent, and the robot is not
waiting on it** -- 2.5's rule.

**3. A candidate -- still no call.** The detector returns `backpack` at some
bearing. COCO has no colour (4.2), so this could be anyone's backpack. The crop
goes to CLIP, scored against the cached vector: strong match. Note what has
*not* happened -- no network, no seconds, no money, and the target string did
the work a fine-tune would otherwise have done.

**4. Confirmation -- the second paid call.** 2.4's **candidate sighting**, and
the trigger that does real work rather than merely saving money. One frame up:
*"I think I have it -- is this the red backpack, and can I reach it?"* The VLM
answers what neither on-board model can: **reachability**. *"Yes, but it is on
a chair; a floor robot cannot arrive at the top of a chair."* That is not a
hypothetical -- it is the exact defect that invalidated the whole Stage 0
corpus (`CLAUDE.md`), discovered by reading frames rather than by any model
saying so.

**5. Approach -- no calls.** `approach(backpack)`, 1.8. The **detector** supplies
a fresh bearing at camera rate: which way to steer. The **lidar** supplies range
at that bearing: how far, and when to stop. This is 1.10's *"the detector points,
the lidar measures"*, and 1.9's open-loop tension resolved -- both halves are
closed-loop, and neither asks the cloud anything.

**6. Ending -- one more call.** Arrival fires 2.4's `goal achieved`; the target
leaving frame and not returning fires `goal impossible`. Either way, one call:
*"what next?"*

#### What that costs

Roughly **four to six paid calls for the whole mission**, against thousands of
detector frames and hundreds of CLIP scores -- which is exactly the 4-6x §6.1
measured, arrived at from the other direction. And the calls are **events**, not
a timer: §6.1's third finding is that the staleness timer stops binding past
`stale_n` ~10, so cost is set by the event rate, and stabilising the fields that
generate events (hysteresis) is the lever, not tuning the clock.

> **The paragraph above is FALSE in practice, measured 2026-09-12.** The six
> tiered walks in `recordings/` carry their own trigger in every status
> record. Across **70 cloud calls**:
>
> | trigger | calls | share | who fires it |
> |---|---|---|---|
> | `cold_search` | 55 | **79%** | a counter on consecutive `absent` |
> | `mission_start` | 6 | 9% | the harness, once |
> | `candidate_sighting` | 7 | **10%** | **the local tier** |
> | `staleness` | 2 | 3% | a clock |
>
> **`candidate_sighting` -- the only trigger perception owns -- fired on two
> walks of six.** The call rate is one per 8.9 frames against a
> `cold_search_after` of 6, so a bare counter with no models at all
> reproduces very nearly the whole of the "4-6x saving" this section claims
> for the trigger discipline.
>
> **The reasoning was sound and the conclusion did not survive contact.**
> §6.1's finding is about the *staleness* timer, and that is correct -- it
> contributes 3%. What it missed is that `cold_search` is also a countdown,
> written as a backstop (*"without it a target outside COCO's 80 never fires
> anything"*) and doing the work in practice. 4.10's own note two sections
> down says the same thing from the other end: *"on the search walk the
> saving came from the timer, not the tier."*
>
> This is not a tuning failure. §4.9's re-reading records why: **search
> proposal is not a tier the 8L can serve at all**, so on a distant or
> out-of-vocabulary target there is no event to fire on. Phases A-D below
> accept that and pace the timer deliberately instead of pretending it is a
> backstop.

#### Why it has to be three, in one line each

- **The detector** is fast but knows 80 words and has no judgement. It says
  *where*, never whether it matters.
- **CLIP** has unlimited vocabulary but can only *compare* -- it cannot reason,
  and it cannot find anything unaided; something must hand it a crop (4.2).
- **The VLM** reasons about anything, and 3.6 s is an eternity at 0.5 m/s.
  **You cannot steer with it.**

The rule underneath, and the one to protect: **the fast tiers never think, and
the thinking tier is never in the driving loop.**

#### And when the link dies

The detector and CLIP are on the robot, so perception does not degrade at all --
the car keeps seeing and keeps avoiding. What it loses is *new goals*, and 2.5
already names the fallback: `brain/agent.py`'s rule-based explorer, which
`PLAN-sim-hardening.md` 2.2 says to keep and not extend, becomes the defined
degradation. **This is the payoff of the split**, and it is worth noticing that
it is free: the code exists and is tested.

### 2.9 The perception budget, costed -- **added 2026-09-06**

1.14's closing note said *"the one number to watch is C6's"* and that nothing in
this plan had yet costed the three models together. This is that costing, and
**they do not fit at camera rate.**

Chip occupancy per invocation on the 8L, from 4.3.1 where it exists and
comparable 8-series zoo figures derated to the 8L's measured 75-85% where it
does not:

| Stage | Per invocation | Source |
|---|---|---|
| YOLO11s @640 | **10.9ms** | 4.3.1, 92 FPS |
| Floor segmentation @512 (DeepLabv3+/MobileNetV2, Fast-SCNN, STDC class) | **20-40ms**, call it 25 | 8-series zoo, derated |
| CLIP image encoder per crop -- **ViT-B/32** (88M params) | **25-50ms** | attention-heavy on a part with no local DRAM |
| CLIP image encoder per crop -- **ResNet-50** (25M params) | **7-10ms** | the alternative 4.3 also lists |

The frame budget at 30Hz is 33.3ms.

| Schedule | Chip time per frame | Verdict |
|---|---|---|
| detector alone | 10.9ms (33%) | fits |
| detector + segmentation every frame | ~36ms (108%) | **already over** |
| detector + seg + 3 crops, ViT-B/32 | 111-186ms | **5-9Hz achieved** |
| detector + seg + 2 crops, ResNet-50 | ~52ms | **19Hz** -- bottom of 2.1's row, no margin, before host cost |

**A schedule that does fit:** detector at 30Hz (327ms/s) + segmentation at 5Hz
(125ms/s) + ResNet-50 CLIP on <=2 crops at 10Hz (160ms/s) = **61% duty.** That
is the resolution 1.14 guessed at -- *"run segmentation at a fraction of camera
rate, gate CLIP on the crop count"* -- but it is a **rate-division requirement,
not spare capacity**, and C6 must be specified against it.

**Pick ResNet-50 over ViT-B/32.** 4.3 lists both and never chooses; the choice
is 3-5x on compute and again on weight-streaming below.

#### "Several models resident" is the dangerous claim, not the benign one

1.14 item 2 wrote that 4.3's *"several resident"* is *"a claim about memory, not
throughput"*, filing residency as the harmless axis. **On this part it is the
opposite.** 4.3's own table says the 8-series is *"dataflow, no external memory,
weights streamed from the host"*. There is no on-chip DRAM, so a HEF that is not
resident is re-streamed over the Pi 5's **single PCIe Gen 2 x1 lane** -- ~450
MB/s usable at best. CLIP ViT-B/32's ~88MB of INT8 weights is **~200ms of lane
time per swap**: six frame periods, dwarfing every inference figure in the table
above. If HailoRT's scheduler round-robins three HEFs per frame, swap cost
dominates the budget entirely.

**And configuration D puts the NVMe on that same lane** (1.10 item 2, 4.9). An
SSD write burst steals directly from weight-streaming bandwidth. The "solve the
one-lane problem" fix and the "several models resident" claim are competing for
the same 450 MB/s, and nothing in this plan had noticed.

**Three consequences for C6.** Pin the detector resident and never swap it.
Express the budget as a schedule with fixed sub-rates, not as headroom. And add
**"measure HEF context-switch cost"** to the hardware-day list beside 1.15.5's
mechanical measurements -- it is the single number most likely to invalidate
this design, and it cannot be looked up.

#### One more latency correction, for 1.14 item 5's benefit

4.9's *"single-digit milliseconds"* and 4.4's FPS figures are **inference-kernel
throughput**. Real capture-to-feature latency on this pipeline -- capture, ISP,
resize, transfer, inference, post-process -- is **40-80ms**. 4.9's
"500-1500x faster than the cloud call" survives comfortably. But it is the wrong
number to build a reaction budget from, and 1.14 item 5's `t_react` is built
from exactly this.

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
| **Encoder motors** | ~~nice to have~~ **required** (1.14 item 6) | **required.** nav2's local planner wants wheel odometry fused with scan matching; scan matching alone is meaningfully worse |
| **IMU** | ~~not needed~~ **required** (1.14 item 6, 1.16 #14) | **desirable** -- ~$10, stabilises heading between scans |

*(Both left-hand cells updated 2026-09-06: 1.14 item 6 makes encoders and the
IMU load-bearing under (a) as well, because a held velocity needs a PID on wheel
speed and heading error integrates for as long as the robot drives. 1.16 #14
adds a third job for the IMU -- rejecting a scan taken while the chassis is
pitched.)*

### 3.4 Persistence

Decided in 1.5. The reasoning behind the two that matter most:

**Only the map persists.** `MissionMemory` is constructed at
`control/mission_runner.py:255`, held on the runner, kept in
`control/brain_server.py`'s `state` -- **RAM only, no serialisation anywhere,
one mission's lifetime, no copy on the Pi.** *(Corrected 2026-09-06: this said
"running on ECS Fargate". That stack was deleted 2026-09-05 and the brain is
deployed nowhere at present -- which is B5's whole point, and which also means
C5's "provable in the twin" assumes a locally-run robot server.)* Persisting
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

> **SUPERSEDED FOR PRICING BY 3.6a (2026-09-16), WHICH IS RETAILER-VERIFIED.**
> Read this section for *what the parts are and why*; take no figure from it.
> The Pi 5 8GB line is wrong by **$95** (the $80 board no longer exists), the
> pan/tilt bracket row names a part the chosen servo does not fit, and the
> bare Hailo-8L M.2 module **has no in-stock source at any price**. 3.6a also
> carries the **Jetson Orin Nano Super BOM** beside this one; the premium is
> **$102**, not the +$320-430 the 2026-09-13 decision rejected.

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
| 2-axis bracket, **pan axis only**, **ST3215 bus servo** | 25 | **DECIDED 2026-09-16, replacing the two-SG90 row.** Buy the 2-axis bracket (32 x 28 x 65mm, takes the 28x28mm Camera Module 3) but **fit only pan** -- shim or bolt the tilt joint at a slight downward pitch. Two changes from what this row used to say, and they are separable. **(a) Drop tilt for v1**, which is 1.15.3's own recommendation: a fixed downward pitch serves the floor mask and 1.15.4's near-field blind box for free, where tilt raises `H_max` and the blind volume, makes the camera-lidar extrinsic time-varying (1.16 #4), and would be the first feature to reach hardware with no twin representation (1.16 #8). **P16 (2026-09-16) strengthens this**: the floor mask turns out to be the load-bearing crop source, and a mask wants consistent floor geometry rather than a moving horizon. Keep the bracket so tilt is recoverable when a log shows the planner reaching for it -- 1.7's rule. **(b) ST3215 rather than SG90 on the axis that survives**, closing 1.16 #7: an SG90 is open-loop, so a stalled or knocked servo makes every bearing wrong with nothing to notice, and bearing is the camera's one job (1.11). The ST3215 has position feedback, is driven natively by the Waveshare board, and needs no Pi GPIO PWM. Pan is also the axis the twin already models -- M2's depth strip is cast off the *view* heading. **Still required**: startup homing plus a plausibility check, and `tilt_deg` becomes a CONSTANT from config (the mount's real pitch, **not zero**) rather than a servo reading |
| **3S** Li-ion pack + charger, **x2** | 70 | 1.3. **Not 2S** -- see the correction below. Two packs since 2026-09-06: the bank is off the robot, so this pack now feeds everything through two bucks, and 1.3's budget gives one pack 40-60 minutes of driving |
| Wiring, connectors, switch, XT60 | 15 | |
| Standoffs, M2.5/M3 hardware | 10 | For stacking decks |
| **Hailo carrier** (M.2 HAT+, or configuration D's dual-slot board) | 20-48 | **Added 2026-09-06.** The row above buys a bare M.2 module and 4.9's own table prices configuration C as *"a standalone module plus the M.2 HAT+"* and D as *"~70 + 48"*. The note on that row even says "a standalone module plus a carrier" while pricing no carrier. **The "essential only" build as previously listed could not be assembled.** Take D's $48 board if the NVMe is wanted, ~$20 for a plain M.2 HAT+ if not |
| Buck converter #2, 5V/2-3A | 0-8 | 1.3. Was the pan/tilt servos' own rail, because two SG90s stalling at 600-750mA on the Pi's 5V is the textbook brownout (1.16 #9). **Possibly unnecessary as of 2026-09-16**: one ST3215 driven by the Waveshare board (7-13V in) may not need a separate 5V rail at all. **Check the servo's voltage range against the board's servo output before buying this** -- it is the one line here that a part swap might have retired |
| **Bulk electrolytic (>=470uF) + TVS, motor rail** | 2 | 1.3. Regenerative braking against a 3% input-voltage margin |
| | **~495-531** | Was "~440" -- wrong on three counts (2026-09-06): it took the motor driver at **0** while 1.14 recommends the Waveshare board at **30**, it bought an M.2 module with no carrier, and it had no servo rail. Then **~490-518**. Now **~495-531** (2026-09-16): the servo row went 12 -> 25 (pan-only ST3215), and buck #2 went 8 -> 0-8 pending the voltage check |

**Strongly recommended -- each avoids a failure already discussed here**

| Item | ~USD | Avoids |
|---|---|---|
| Powered USB hub | 15 | The 600mA USB cap browning out the Pi (1.3) |
| IMU (MPU6050 / BNO055) | 0-10 | Heading drift between scans -- **matters much more under (b+)**, and more again under 1.14, where heading error integrates for as long as the robot drives. **0 if the Waveshare board is taken**: verified 2026-09-06 to carry a 9-axis QMI8658C + AK09918, which is better than the part specced here |
| **2x VL53L1X ToF, forward-down** | 12 | The under-plane residual 1.15.4 names -- cliffs, thresholds and low obstacles the scan plane misses. **Effectively mandatory since 2026-09-06**: 1.15 sizes the volume nothing sees at bumper-to-32cm, below 12cm, which the robot crosses in 0.64s at 0.5 m/s. Also the only sensor here that sees a descending step |
| **Compliant bumper + microswitches** | 5 | 1.15.4. **Not optional under 1.14**: everything else in the safety chain is an inference, and this is the only measurement. The cheapest guard in the build |
| ~~5V buck converter~~ | -- | **Moved to essentials as buck #1** (1.3): the Pi is off the bank and onto the pack by design now, not conditionally |
| **Lidar pedestal**, 3D printed | 15 | 0 with a printer. **Not a bracket -- it has a job (1.15.2)**: hold the scan plane as low as it can go while still clearing the camera's swept envelope at full tilt. Too low blinds the lidar in the forward arc; too high grows the under-plane blind volume |
| Jumper wires, misc | 10 | |
| | **~65** | The IMU falls to 0 (absorbed by the Waveshare board) and the 5V buck moves to essentials as buck #1, since 1.3 no longer runs the Pi off a bank |

**Worth considering**

| Item | ~USD | Why |
|---|---|---|
| NVMe SSD + dual-slot PCIe base | 83 | **SD cards corrupt on brownout**, which is the exact failure 1.3 is written about. The one item here that prevents losing work rather than an annoyance. **The Hailo takes the Pi's one PCIe lane** (1.10 item 2), so this means the M.2-module form of the Hailo-8L on a dual-slot switch board plus the drive, not the plain M.2 HAT at 45. **Now vendor-documented rather than rumoured** (4.9): the Pineboards HatDrive! Dual, ~48, carries an ASM1182e PCIe switch and names Hailo support -- but it is **discontinued at some retailers, so check stock**, and its slots are 2230/2242 only, so the drive must be short. Drive ~35 |

**Totals**

| Scenario | ~USD |
|---|---|
| Essential only (plain M.2 carrier, no NVMe) | **~500** |
| **+ recommended** | **~565** |
| + NVMe and the dual-slot base (carrier becomes D's 48) | **~628** |
| Already own a Pi 5 | subtract ~100 |
| **+ Hailo-10H M.2 instead of the 8L** | **add ~60** |
| ~~+ ST3215 bus servos, if 1.16 #7 forces them~~ | **Taken, 2026-09-16** -- one pan ST3215 is in the essential row now (+13, not +38, because tilt is dropped) |
| ~~Accelerator deferred~~ | **No longer on offer** -- 1.14 item 2 |

**Accelerator, 2026-09-16.** The 8L row prices the measured-cheapest build.
P16 measured the 10H preserving YOLO-World's embedding head where the 8L
destroys it (22% vs 5%), and the 8L's limit is architectural rather than
numeric -- so the 10H is what keeps open-vocabulary models on the table at
all. It buys about **one point of tier recall today** (49% vs 49%, because
the floor mask absorbs the damage), so it is a headroom purchase, not a
performance one. **The 8L row's "reverted on its measured tokens/s" reason
is void**: tokens/s bounds on-device LLM/VLM, which this robot does not do
-- the deliberation tier is a cloud call and 4.9's own 5.89 tok/s says it
should stay one.

**Recomputed 2026-09-06, and the old totals under-read by ~8%.** The previous
440 / 515 / 598 summed correctly *only* by taking the motor driver at $0 and the
IMU at $10 -- i.e. by pricing the bundled Yahboom board, which is not what 1.14
recommends. This document reconciled that with *"the Waveshare board and the IMU
it absorbs roughly cancel"*, but the IMU line was **$0-10** against a **$30**
board, so it is a net **+$20-30, not a cancellation.** Three items were also
missing entirely: the Hailo's carrier, the servo rail, and the motor-rail bulk
capacitance.

**The power bank is no longer "already owned, 0".** 1.3 removes it from the
robot -- it fails both of that section's own tests, and the design now runs
everything off the 3S pack through two bucks. **Buy a second 3S pack** (1.3's
runtime budget is 40-60 minutes), which is the ~$35 line doubled.

**Budget ~555-620**, and the variables that move it are whether a Pi 5 is
already owned, whether the NVMe and its dual-slot board are taken, and whether
bearing accuracy forces bus servos. **Raised ~17 on 2026-09-06** by 1.15.4's ToF
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

#### 3.6.1 Shopping prompts, for a voice assistant

Written 2026-09-16, from the rows above as they stand after the servo and
accelerator decisions. Paste in order; the first five only load context,
and the sixth triggers the search. They are kept HERE, beside the table
they were generated from, so that a part swap and the prompt that buys it
cannot drift apart -- which is the same failure the `NavigateModelId`
mistake was, one layer out.

**Three parts are unlikely to be on a general retailer.** The **Hailo-10H
M.2** and the **Slamtec RPLidar C1** come from distributors (Mouser,
DigiKey, Seeed; Slamtec or Robotshop), and the **Pineboards HatDrive!
Dual** is discontinued at some retailers -- 3.6's own note. Expect the
final prompt to report those as not found; that is the prompt working,
not failing.

**1 -- context**

> I'm building an indoor autonomous robot on a Raspberry Pi 5. I'm going
> to give you a parts list in several messages. Don't search yet -- just
> hold the list and wait until I say "find these". Confirm you're ready.

**2 -- compute and AI accelerator**

> Add to the list: Raspberry Pi 5 with 8GB RAM. Raspberry Pi 5 Active
> Cooler. A 64GB A2-rated microSD card. A Raspberry Pi Camera Module 3
> with autofocus. A Hailo-10H M.2 AI acceleration module, M.2 Key M, 2242
> or 2280. A dual-slot M.2 PCIe base for Raspberry Pi 5 that supports both
> an AI accelerator and an NVMe drive at the same time. And a short NVMe
> SSD, 2230 or 2242 size, 256GB or larger.

**3 -- chassis and motion**

> Add to the list: a Yahboom 2WD differential drive robot chassis kit with
> encoder motors. A Waveshare General Driver for Robots board. A 2-axis
> pan and tilt camera bracket that fits a 28 by 28 millimetre camera. And
> one Waveshare ST3215 serial bus servo with position feedback.

**4 -- sensors**

> Add to the list: a Slamtec RPLidar C1 360 degree laser scanner. Two
> VL53L1X time-of-flight distance sensor breakout boards. And a set of
> microswitches with levers for a robot bumper.

**5 -- power and wiring**

> Add to the list: two 3S 11.1 volt lithium-ion battery packs with a
> balance charger. Two adjustable DC-DC buck converters rated 5 volts at 3
> amps or more. A 470 microfarad or larger electrolytic capacitor and a
> TVS diode for motor noise. XT60 connectors, a power switch, and hookup
> wire. And an M2.5 and M3 brass standoff and screw assortment kit.

**6 -- trigger the search**

> Find these. For each item show me the best-rated option with price and
> delivery date, and tell me clearly which ones you couldn't find so I can
> source them elsewhere.

##### Four things to check against the results, not against the prompt

These are the places where a plausible-looking substitution is wrong, and
each one is a row above rather than a preference:

1. **The second buck may be unnecessary.** It exists in prompt 5 because
   the table still carries it at 0-8. One ST3215 off the Waveshare board
   (7-13V in) may retire it -- check the servo's voltage range against
   that board's servo output before buying it.
2. **The NVMe must be SHORT.** The dual-slot base takes 2230/2242 only.
   A 2280 drive is the common default and will not fit.
3. **Do not substitute the driver board.** The Waveshare General Driver is
   chosen for its **ESP32** -- it runs the velocity PID off Linux's
   scheduler, which a 20-50Hz loop in CPython cannot do reliably (1.14) --
   and for the 9-axis IMU it carries, which is why there is no separate
   IMU line to buy.
4. **One servo, not two.** Pan only; the tilt joint is shimmed at a fixed
   downward pitch (1.15.3, and P16's floor-mask finding).

### 3.6a Verified prices, and the Jetson BOM beside them -- 2026-09-16

3.6's figures are estimates *"from general knowledge on 2026-09-03. **Not
verified against a retailer**"*. Every line below except four was read off a
retailer page on 2026-09-16. **3.6 is left exactly as written**: its numbers
are what the decisions were argued against, and overwriting them would make
the reasoning unauditable. This section is the correction, not a replacement.

**Not verified, and why:** Amazon and Micro Center both block automated
reads, rpilocator would not load, and the Yahboom ASINs' bundled-battery
voltage is only stated in listing images. Micro Center Westmont stock is
therefore still an in-person question, and it is the one that could move the
Pi line most.

#### The finding that reframes everything: **the $80 Pi 5 no longer exists**

Raspberry Pi raised the 8GB three times on DRAM cost -- **$80 -> $95 (Dec
2025) -> $125 (Feb 2026) -> $175 (Apr 2026)**. $175 is the floor, in stock at
PiShop and CanaKit; Adafruit is $200. So the $200 Amazon sighting was ~14%
over list rather than a scalper, and **3.6's single largest line was wrong by
$95**.

This is the same memory-price event 4.8 already recorded from the other side
-- NVIDIA's July 2026 Jetson repricing, *"LPDDR pricing is the implicated
cause"*. **The whole board market moved together**, which is exactly why a
*relative* part decision had to be re-checked rather than assumed stable.

And the Jetson moved the other way: **Newegg lists the Orin Nano Super devkit
at $399**, NVIDIA's list, not 4.8's "~$480 street".

#### Common parts -- identical on both paths

| Item | 3.6 est | **verified** | Source / note |
|---|---|---|---|
| RPLidar C1 | 99 | **69.00** | DFRobot or Seeed, ships from China; $71.92 RobotShop US. **$30 under estimate** |
| Yahboom 2WD chassis | 69 | **69.00** | Motors confirmed L-type 520, 1:40, **11-line Hall encoder** -- 3.8's open encoder question is answered. ~79 with battery, voltage unconfirmed |
| Waveshare General Driver | 30 | **27.99** | Spec matches |
| ST3215 bus servo | (in the 25) | **21.99** | Waveshare 12V variant. **Avoid the 7.4V part** (Seeed C046) |
| Pan/tilt bracket | (in the 25) | **3.99, and it does not fit** | See the flag below -- this line is wrong |
| 2x 3S pack + charger | 70 | **44.98** | Ovonic 2x2200mAh + iMAX B3. **No BMS** |
| Wiring, switch, XT60 | 15 | **~22.00** | |
| Standoffs | 10 | **12.95** | |
| Bulk cap + TVS | 2 | **~2.00** | |
| 2x VL53L1X | 12 | **29.90** | Adafruit #3967 -- **2.5x the estimate**, and both boards are I2C 0x29, so XSHUT sequencing or a mux is required |
| Bumper microswitches | 5 | **~11.00** | |
| Powered USB hub | 15 | **19.99** | Waveshare's 7-36V hub at 17.99 is the better part: it runs off the pack and keeps load off the 5V rail |
| Jumper wires | 10 | **11.85** | |
| | | **~346.64** | |

**Buck #2 is confirmed deleted.** Waveshare's wiki states the 7-13V input
"directly powers the serial bus servo", the board is good for 5A continuous
and the ST3215 stalls at 2.7A. 3.6's "0-8, pending the voltage check" resolves
to **0**.

#### The bracket decided on 2026-09-16 does not exist

**Every ~32x28x65mm pan/tilt bracket is built for SG90 micro servos. The
ST3215 is 45x35x25mm and will not fit any of them.** That invalidates the $25
row two days after it was written, and the ST3215 half of it is the half worth
keeping (1.16 #7: an open-loop servo makes every bearing wrong with nothing to
notice). Two options:

- **Waveshare's complete 2-axis ST3215 pan-tilt module, $109.99** -- +$84 over
  the row, and it buys a tilt axis 1.15.3 deliberately chose not to use.
- **Print it.** Waveshare publishes free STEP files. This is the recommendation:
  the mount is a fixed downward pitch plus one pan axis, which is the simplest
  possible bracket, and the lidar pedestal already needs a printer or a print
  service.

#### Part B, side by side

| | **Pi 5 + Hailo-8L** | **Jetson Orin Nano Super** |
|---|---|---|
| Board | Pi 5 8GB **175.00** | Orin Nano Super devkit **399.00** (Newegg, list) |
| Cooling | Active Cooler **10.95** | in the box |
| Wi-Fi / BT | on board | **in the box** -- 3.6's assumed ~$20 M.2 card is not needed |
| Accelerator | **AI HAT+ 13T, 76.95** -- see below | not needed |
| Carrier | included in the HAT+ | not needed |
| Camera | Camera Module 3 **29.25 + 3.95 cable** (the box ships only the 15-pin) | Arducam IMX219 **19.95**, SparkFun |
| Storage | microSD 64GB A2 **31.99** | SanDisk 128GB A2 **42.99** |
| 5V/5A buck | **39.95** Pololu D36V50F5 | **not needed** -- takes 9-20V from the pack directly |
| **Part B subtotal** | **368.04** | **461.94** |
| **All-in** (+8.75% tax, ~$50 shipping) | **~827** | **~929** |

> **The Jetson premium is $102**, against the **~$170** re-open threshold
> 4.7 sets. The 2026-09-13 decision ruled the Jetson out at a computed
> +$320-430. **That premium no longer exists**, and it collapsed from the
> *Pi* side -- a direction none of 4.7's four re-open conditions anticipated.

#### The Hailo line is the Pi path's real problem, and it is not a price

**There is no in-stock source for the bare $70 M.2 module.** The AI Kit that
bundled the removable 2242 part is discontinued -- CanaKit sold out, SparkFun
retired it, Seeed out of stock, last eBay unit $124.99. What is in stock:

| form | price | problem |
|---|---|---|
| **AI HAT+ 13T**, soldered | 76.95, PiShop | **On this plan's own reject list** |
| Hailo-8L **2280** B+M, UP Shop | 89.00 | Too long for the M.2 HAT+ *and* for HatDrive! Dual. Needs a 2280 carrier; Geekworm X1001 is $13 and **out of stock** |
| Hailo-8L 2230 **A+E**, UP Shop | 99.00 | Wrong key |
| M.2 HAT+ | 12.00 | Fits 2230/2242 -- i.e. only the module that cannot be bought |
| HatDrive! Dual | -- | **Sold out at Pineboards, discontinued at The Pi Hut** |

So the cheapest in-stock Pi build requires **buying the soldered AI HAT+**,
which 3.6 rejected for two stated reasons: *"it is the form that survives a
Jetson pivot and the form the NVMe needs anyway."*

**Both reasons are now live rather than hypothetical.**

1. *Survives a Jetson pivot* -- the pivot is being decided **now**. Paying a
   premium for pivot-insurance while making the pivot decision is incoherent;
   either pivot, or buy the cheap soldered part and stop paying for optionality
   you just declined.
2. *The NVMe needs it* -- the AI HAT+ consumes the Pi's single PCIe lane, so
   **the Pi path with an AI HAT+ boots from SD and cannot have an NVMe**. 3.6
   calls that row *"the one item that prevents losing work rather than an
   annoyance,"* because **SD cards corrupt on brownout** and 1.3 is written
   about exactly that failure. The Jetson devkit has a free 2280 slot and does
   not face the trade at all.

This is a cost the dollar column cannot show: the Pi path's only buyable
configuration gives up a documented design constraint *and* the brownout
protection, while the Jetson path gives up neither.

#### What the $102 buys

| | Pi 5 + Hailo-8L | Orin Nano Super |
|---|---|---|
| Search-proposal model | YOLO-World (P9/P10) | **OWLv2** |
| Recall @3 FP, 8-walk corpus | **72% or 45%** -- turns on an unrun INT8 test | **82%**, measured (P7) |
| Compile risk | P10 compiled, **INT8 accuracy unmeasured**; P7d found INT8 destroys OWLv2 | **none** -- PyTorch fp16, and P7 measured fp16 as accuracy-free |
| NVMe | **no** (PCIe lane consumed) | yes, 2280 slot free |
| Frame rate (P7b projection) | 92 FPS reactive | ~2.6-4.9 Hz fp16 search tier, 36ms detect |
| Memory | 8GB, detector off-board | 8GB shared -- 1.10's objection, **dropped** once the local VLM went (OWLv2 is 150M) |

**The honest summary:** $102 converts a 45-72% tier that depends on an unrun
measurement into a measured 82% tier with no compile step, and returns the
NVMe. That is the trade the 2026-09-13 decision would have been taking at
+$320-430 and declined. At +$102 it is a different decision.

#### Verified at the counter: Micro Center Westmont, 2026-09-16

The one source Cowork could not read is now read, and it lands on the Jetson
side. **Micro Center Westmont, SKU 812057, $399.00, 7 NEW IN STOCK**, 18-minute
pickup, aisle 1. Three things follow.

**Availability now decides more than price does.** The Jetson board can be in
hand tomorrow morning. The Pi path's accelerator **cannot be bought in the
form this plan specifies at any price** -- the bare 2242 module is discontinued
everywhere, both carriers are sold out or discontinued, and the only in-stock
option is the soldered AI HAT+ that 3.6 rejected. A part that is purchasable
beats a part that is cheaper-on-paper and out of stock.

**Local pickup also trims the premium.** Board shipping goes to zero on the
Jetson side while the Pi still ships from PiShop or CanaKit, so the all-in gap
falls from $102 to roughly **$87**.

**And it buys a return path against a real failure mode.** The listing's one
1-star review is a bricked board after a required firmware update, *"which I
cannot return"*; another reports theirs arrived pre-flashed. Micro Center takes
returns with no receipt for Insider accounts. **Check the window at the
counter**: the general policy is 30 days, but the 15-day list names
*motherboards*, and this SKU's own component type is "Development Board /
Mainboards". Worth thirty seconds to ask.

##### Spec-sheet corrections to the table above

| item | what the counter page says |
|---|---|
| **Wi-Fi/BT** | The spec lists *"M.2 Key E **for** Wi-Fi/Bluetooth"* -- **a slot, not a card**. Cowork's "in the box, $0" is unconfirmed. **Verify at the counter**; budget ~$20 if not. |
| **Power** | **7-25W**, confirming that 1.3's 40-60 min pack budget must be re-derived, not assumed |
| **Storage** | microSD **and** external NVMe (M.2 Key M) -- the free 2280 slot is confirmed, which is the Pi path's lost NVMe |
| **Camera** | 2x MIPI CSI-2 **22-pin** -- so the Arducam's 22-pin cable must be confirmed present (SparkFun's page does not say; Arducam's does) |
| **Power input** | **USB-C is debug/data only.** Power is a **5.5/2.5mm barrel jack**, 9-20V. A9's wiring line needs a barrel pigtail added |
| **Display** | DisplayPort, not HDMI. Irrelevant headless, but bring-up day needs a DP cable |
| **Memory bandwidth** | **102 GB/s**, exactly the figure P7b flagged as unmeasurable from an A10G: OWLv2's attention matrices are 3.7 GB/frame, so ~37ms of pure bandwidth, about half the INT8 GPU estimate |
| **AI performance** | 67 TOPS **sparse INT8** -- and P7d found INT8 destroys OWLv2, so the operating mode is fp16 and this number does not describe it |

#### What does NOT change

- **Mecanum is still disqualifying.** This is an argument for a **bare Orin
  Nano Super devkit on the Yahboom differential chassis**, never for a bundled
  ROS 2 platform (3.7's closed survey, and P7c's odometry argument).
- **The chassis, lidar, driver, servo, power and sensing parts are identical
  on both paths.** Only Part B moves.
- **P10's YOLO-World INT8 test is still worth its ~$1**, and should be run
  before ordering either way: it is the only thing that can put the Pi path
  back at 72% and make the $102 arguable.

#### Two corrections to the sourcing report itself

- **There is no Pi 5 4GB to reuse.** The report suggests recovering one "from
  your PiCar-X build". No hardware has ever been ordered for this project --
  3.8 says *"nothing has been ordered at all"* -- and the PiCar-X was retired
  as a design in 1.1 before any purchase. The $110 4GB board is a fresh buy,
  and 8GB is what SLAM + nav2 + a detector was sized against.
- **The 3S pack is marginal on the Jetson.** 9.9V empty against a 9V floor is
  0.9V of headroom, so the cutoff must sit near 3.3V/cell -- which strands
  usable capacity and eats into 1.3's 40-60 minute figure. The packs found have
  **no BMS**, so that cutoff is not automatic. Re-derive 1.3's budget before
  ordering the Jetson, and price a BMS or the Waveshare UPS Module 3S ($28.95,
  BMS + 12.6V charger + 5V/5A out, but needs loose 18650s).

#### Retailer-verified totals, 2026-09-17 -- see `HARDWARE-BOM.md`

A second sourcing pass produced a full Jetson BOM with exact part numbers,
vendor plan, bring-up order and a power budget. It is filed as
**`HARDWARE-BOM.md`** with an editor's note; its arithmetic was re-checked
here and is exact. **Quote that file, not this section, for any part number.**

Three of the open questions above are answered:

- **Wi-Fi is included** -- the devkit ships an **RTL8822CE** in the M.2 Key E
  slot with two antennas in the base. The ~$20 contingency is deleted. (Avoid
  the Intel AX210: it needs a kernel rebuild on JetPack 6.2.)
- **The Arducam ships both cables**, 22-22 and 22-15. Use the supplied one --
  a generic 22-22 cable is reported to need flipping and to short.
- **A BMS does not solve the power problem.** Generic 3S BMS boards cut off at
  **8.4-9.0V, at or below the Jetson's own 9V floor**, so a BMS protects the
  cells and not the board. The answer is a software cutoff off the driver
  board's **INA219** plus a buzzer backstop -- which is new work in
  `robot/safety.py` and owes a twin readout under CLAUDE.md section 7.

| Scenario | 3.6 said | 3.6a est. | **verified 2026-09-17** |
|---|---|---|---|
| Jetson, essential (A+B, no NVMe) | -- | -- | **~$857-877** |
| Jetson, full (A+B+C+NVMe) | -- | ~1,047 | **~$1,019-1,039** |
| Jetson, A+B+C without NVMe | -- | ~929 | **~$944-964** |
| Pi path, same parts list | ~565 | ~827 | **~$835-855**, and **not buildable** in the specified form |

**The premium holds at ~$100-110 across three independent passes** ($102, $87,
now ~$109). It is stable, and it is well inside 4.7's ~$170 re-open threshold.

The rise over 3.6a is not a price move -- it is **parts 3.6a did not know
about**: a DisplayPort cable (the devkit has no HDMI), a 5.5x2.5mm barrel
pigtail (USB-C is debug-only), a low-voltage buzzer, and a 10A inline fuse that
is still unpriced. All are real and all were missing.

#### Superseded totals



| Scenario | 3.6 said | **verified 2026-09-16** |
|---|---|---|
| Pi path, essential + recommended, all-in | ~565 | **~827** |
| **Jetson path**, essential + recommended, all-in | not costed | **~929** |
| Pi path + NVMe | ~628 | **not available with an AI HAT+** |
| Jetson path + NVMe (Kingston NV3 500GB, 109) | -- | **~1,047** |

Both are over the ~$500-750 the plan has carried since 2026-09-03, and **the
Pi 5's $95 rise is the largest single reason**. That target should be restated
rather than quietly missed.


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

#### But the crops still come from the class list, and they need not

**Added 2026-09-06.** As written above, CLIP sits *downstream* of the detector:
YOLO draws a box and labels it `backpack`, and CLIP decides whether it is the
red one. That inherits COCO's 80 classes wholesale -- for a target with no COCO
word (`router`, `slipper`, `charging cable`) the detector never draws the box,
so CLIP never receives a crop to score.

**CLIP does not need the label; it needs the crop.** It scores an image region
against arbitrary text, so anything that can say *"there is some object here"*
without saying what it is called will do:

| Source of crops | How it is class-agnostic |
|---|---|
| the detector's own proposals at a **very low confidence threshold**, labels discarded | *see the correction below -- noisier than it sounds on YOLO11* |
| **floor segmentation** (4.3's first experiment) | anything that is not floor but stands on it is an object |
| **lidar clusters** projected into the image | geometry, no vocabulary at all |

Feed any of those to CLIP and the mission's own target string becomes the
classifier -- **open-vocabulary search, on-board, at frame rate, with no
retraining and no cloud call**, on parts already in the bill (4.3 carries the
CLIP image encoder; the *text* encoder runs on the Pi CPU **once per mission**,
since the target string does not change, so the per-frame cost is one image
encode and a dot product).

So the rule is: **the class list decides who proposes, never who confirms.**
Fine-tuning past COCO's 80 (above) stays the answer only for something that
must be *proposed* fast and repeatedly -- and it needs labelled floor-height
data that does not exist.

Two limits, so this is not read as more than it is. CLIP returns a *similarity*,
not a probability, so it needs competing strings and a threshold or it will
always pick something. And it does not localise -- bearing still comes from
whatever drew the crop, which is the same division of labour 1.8 and M1 already
impose.

**Correction, same day: YOLO11 has no objectness head.** The first draft of the
table above justified the low-threshold trick as *"YOLO proposes regions before
it classifies them"*. True of YOLOv5-era anchor-based models, which carried a
separate objectness score; **YOLOv8 and YOLO11 are anchor-free and dropped it**,
so the class score *is* the confidence. The trick still works -- threshold at
~0.05, keep the boxes, discard the labels, and an out-of-vocabulary object
surfaces as a weak `microwave` -- but it harvests weak class activations rather
than reading a clean "something is here" signal. **It is the noisiest of the
three crop sources, not the cleanest**, which argues for floor segmentation or
lidar clusters as the primary one.

#### It is not a fallback -- both paths run, and the target string picks which

**The handoff people expect does not exist, because there is nothing to hand off
*on*.** For an out-of-vocabulary object the detector reports *nothing*, and
nothing is indistinguishable from an empty room -- there is no "unknown" class
and no error. The likelier case is worse: asked to score a box-shaped object
against 80 classes, it returns a confident wrong label, so a failure-triggered
fallback would never fire at all. Same silent-corruption class as 1.16 #7.

So CLIP is not a rescue path. It runs every frame, and what changes with the
mission's target is only **where the crops come from**, decided once at mission
start:

| Target | Crops from | CLIP's job | Cost |
|---|---|---|---|
| **in COCO's 80** -- `backpack`, `bottle`, the Stage 0 targets | the boxes the detector already labelled with that class | disambiguate: *"the red one"* | cheap -- the label is a **gate**, and few crops survive it |
| **outside it** -- `router`, `slipper`, `charging cable` | the class-agnostic sources above | decide identity outright | more crops to score, still on-board, still milliseconds |

**The detector's labels are therefore a cheap prefilter, not a first attempt.**
Where the target has a COCO word, that gate is what keeps CLIP's work down to a
handful of crops per frame; the open-vocabulary path replaces the gate only when
there is no word to gate on.

#### The COCO numbers are standing-height numbers -- **caveat added 2026-09-06**

1.13 rejected a Places365 room classifier because such models are *"trained on
photographs taken by standing humans"* while this camera sits at 10-13cm --
*"the standing-height corpus defect, re-introduced as a model choice instead of
a data choice."*

**That argument applies to a COCO detector too, and to CLIP, and this plan had
not been applying it.** COCO is web photography shot from about 150cm. A chair
from 150cm is a chair; a chair from 10cm is four poles and the underside of a
seat. So 4.3.1's `45.1 mAP` is a COCO-validation number, **not a prediction of
performance at 10cm**, and it may be materially worse there.

This is not an argument against the part -- no detector is trained at 10cm, and
the accelerator is bought for the ceiling (4.5), not for this number. It is a
warning against reading a zoo benchmark as a promise. **The measurement that
would settle it needs the re-recorded floor-height corpus** (`CLAUDE.md` Stage
0), which is now blocking two things rather than one.

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
camera's 30fps. **Re-checked against the 8L's own benchmarks 2026-09-06 and it
survives -- even the `m` tier clears 30fps on the 8L (4.3.1).** The 10H is the
only alternative worth a look, and only for the VLM question -- 1.10 item 6
says how much looking.

`sudo apt install hailo-all` on Pi OS brings the driver, HailoRT, the
GStreamer bits and `rpicam-apps` with Hailo post-processing; `picamera2` ships
Hailo examples that return boxes directly; pre-compiled HEFs exist for both
8-series parts, and for the whole `n`/`s`/`m` range rather than nano alone
(4.3.1). That is the turnkey starting point, and it keeps Pi OS -- one more
reason (b+)'s OS decision lands on "ROS in Docker on Pi OS" (3.3).

#### 4.3.1 The 8L's own numbers, measured -- **verified 2026-09-06**

The zoo publishes per-chip benchmarks, and reading the **8L's** table rather
than the 8's settles three things this section had been asserting from the
wrong column. Batch 1, COCO, 640x640:

| model | mAP float | **mAP on-chip** | **FPS on the 8L** | FPS on the 8 |
|---|---|---|---|---|
| yolo11n | 39.0 | 37.5 | 157 | ~185 |
| yolo11s | 46.3 | **45.1** | **92.0** | ~115 |
| yolo11m | 51.1 | **49.9** | **35.3** | ~50 |

**1. Ship `s`, not `n`.** +7.6 mAP on-chip for a third of the frame rate, and
92 FPS is still 3x the camera. 4.4's "stock YOLO11n" is the *day-one baseline*
that needs no HEF -- it is not the shipped detector, and this table is why.

**2. `m` clears camera rate on the 8L, which confirms 1.10 item 5 rather than
weakening it.** 35.3 FPS > 30fps. The line "the 26 TOPS part buys nothing here"
was argued for a nano or small detector; it holds for the medium tier too. *An
intermediate reading of the Hailo-8 chart -- ~50 FPS for `m`, halved for the
8L -- suggested the 8L was capped below `m` and that the $40 was worth
re-opening. The 8L is 75-85% of the 8 on these models, not 50%. It is not.*

**3. Quantisation is nearly free here.** 1.2-1.5 mAP from float to on-chip
across all three tiers -- small enough that the tier choice dominates it. Read
the zoo's chart with **Quantized** selected, not **Original**: the "Original"
column is the fp32 source model (it matches Ultralytics' published mAP to
within noise) and is not what the HEF delivers.

**The public comparison tool is not the availability index.** Switching its
chip selector clears the model multi-select, so the chart empties and reads as
"no models for this part". Availability is
`hailo_model_zoo/docs/public_models/HAILO8L/`, which carries downloadable
pre-compiled HEFs (DFC v3.33.0) for yolov8n/s/m and yolov11n/s/m alike. *(A
DeepWiki summary of the same repo omits YOLO11 from its 8L list. The `.rst` is
the source of truth.)*

**HEFs are architecture-specific.** A `hailo8` HEF does not run on a `hailo8l`.
Download from the `HAILO8L/` directory. This is the sort of thing that costs an
afternoon on hardware day.

**None of this removes the compile loop (1.10 item 1), and the loop should not
be built against YOLO.** Every capability past the pre-compiled zoo -- floor
segmentation, CLIP over crops, any fine-tune past COCO's 80 -- still needs
ONNX -> DFC -> HEF on x86-64. Building that loop against a model you could
simply download proves less than building it against one you actually need, so
**make floor segmentation its first subject**. *(Ultralytics now documents a
Hailo export path, which may shorten the YOLO half considerably -- **verify**;
this plan predates it.)*

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

#### Floor segmentation is the priority model, and it now carries three jobs

**Added 2026-09-06.** It entered this plan as one of two ideas above. It has
since accumulated two more consumers without anyone deciding it should, which
makes it the highest-value non-detector model here and the right first subject
for 1.10 item 1's compile loop:

1. **A drivable-area mask** -- the original reason. The obstacle question five
   `/navigate` wordings failed at, answered geometrically instead of by asking
   a model to describe depth it cannot see.
2. **Floor-level hazards COCO cannot name.** What stops a 10cm robot is cables,
   socks, shoes, rug thresholds, floor vents and drops -- almost none are COCO
   classes, and most sit *below* the lidar plane. A mask needs no vocabulary:
   it does not have to know what a cable is, only that the floor is interrupted.
   **This is the honest answer to "are 80 classes enough for a house"** -- for
   targets, largely yes; for hazards, the class list is the wrong instrument.
3. **Class-agnostic crops for CLIP** (4.2). Promoted to *primary* by the
   objectness correction there: not-floor-but-standing-on-floor is a cleaner
   "something is here" signal than a weak class activation from an anchor-free
   detector.

**So build the compile loop against this, not against YOLO.** 1.10 item 1 asks
for the loop before hardware day and 4.3.1 establishes that YOLO11 n/s/m are
simply downloadable for the 8L -- which means a loop built against YOLO proves
only that the loop runs. Building it against a model you actually need proves
that the Hailo is the sandbox 1.10 item 1 says it must be, rather than the
fixed-function part it warns it will otherwise become.

It still **does not replace the lidar**, which sees a chair leg the mask cannot.

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
-- the larger YOLO tiers, a floor mask, depth, CLIP, several at once (**how
many at once, at what rate, is C6's unwritten budget** -- 1.14) -- and that the
same HEF can be scored on the recorded corpus before it drives, which 1.16 #10
says is not yet a corpus worth scoring against. None
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

#### Pi-only, stated fairly -- and then overtaken the same day

> **Superseded by 1.14.** Everything in this subsection is conditional on
> discrete motion, which was decided against hours after it was written. Kept
> because the reasoning is sound on its own premise, and because the premise's
> failure is the interesting part.

The plan has treated "no accelerator" as the null option. It is stronger than
that as a *starting* position. Stock YOLO11n on the Pi 5's own cores runs at
roughly 5-13 FPS depending on export path -- ONNX INT8 at the low end, NCNN and
quantised builds at the high end -- which **already cleared 4.4's reaction
bar**, *because motion was discrete at ~2Hz* and the range sensor owns the
emergency stop. Pi-only fails on the **ceiling** (4.1), never on the rate.
**Under 1.14 it fails on the rate too**: 5-13 FPS is 0.2-0.4x of 2.1's
perception row, and continuous driving spends the shortfall in centimetres
rather than in wall-clock.

The consequence was a sequencing fact worth more than either chip: **the
accelerator was the only line in 3.6 that was purely deferrable.** It changes no
other decision -- same chassis, same lidar, same camera, same power design,
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

**3. What is the exposure? $70-130, on a ~$555-620 build (3.6), for a part that
resells.** *(Written when the answer was the $130 10H; 4.9 reverted it to the
$70 8L hours later, and 3.6's totals were recomputed 2026-09-06. The argument
below still reads as a case for the 10H and is superseded on that point -- kept
for the memory-position comparison, which stands.)*
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

- ~~**Order the chassis, Pi, lidar and camera. Leave the accelerator off the
  first order.**~~ **Reversed by 1.14 item 2 the same day** -- continuous motion
  puts the perception tier above what the Pi's cores deliver, so the accelerator
  ships with the first order. The two bullets below still stand and need none of
  the hardware.
- Re-record the corpus at 10-13cm on the wheeled rig -- needs none of this
  hardware.
- Build the compile loop against a rented x86 host and score YOLO11n over the
  walks -- also needs none of it.
- ~~Then buy the AI HAT+ 2, in **M.2 module form**, once a detector has earned
  its place in replay.~~ **Doubly reversed**: the part is the **8L** (4.9, on
  the 10H's measured tokens/s) and the timing is the first order (1.14).

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
5. **An open-weight VLM is measured beating Opus 4.5 on PRECISION over the
   search walk.** Asked directly 2026-09-08 -- *"what if I distilled an
   open-weight frontier model from Hugging Face and ran it on a Jetson?"* --
   and it is the strongest form of the Jetson case, because 4.9 only ever
   costed the **Hailo** version of it. 5.89 tok/s is a statement about a 26
   TOPS part with no memory bandwidth, not about an 8GB board at ~102GB/s.
   Sized fairly, decode there is bandwidth-bound at roughly `bandwidth / weight
   bytes`: a 3B at INT4 lands near **4s** for a 60-100 token reply and a 7B
   near **8s**, against the measured **3.6s** cloud round trip. **Parity at 3B,
   worse at 7B -- estimates, not measurements**, and this document's own rule
   is not to trust an unmeasured number.

   **What it would genuinely buy is not speed**, and 4.9 undersells this
   because it was arguing about a different part: the per-call cost disappears
   (which retires most of 2.4), the network stops being a failure mode (2.5's
   degraded mode becomes the normal one and B3.2 stops ending missions), the
   deliberation rate becomes unbounded, and camera frames from inside a house
   never leave it.

   **But distillation specifically is the wrong first move, for three reasons
   this corpus already supports.** The cloud tier's measured failure is
   **precision, not capability** -- recall 10/10, precision 10/44 -- and
   distillation transfers a teacher's behaviour including its failure modes,
   so a smaller student grounds worse and the confabulation is what gets
   compressed. Opus cannot be the teacher anyway (closed weights), so the
   ceiling is the open model's own grounding and the honest experiment is to
   run that model directly. And **there is no eval to distil against**: this
   corpus scores *perception*, while `control/walk_eval.py`'s judge over
   navigation is explicitly advisory, so the metric the student would be
   optimised on does not exist yet.

   **The cheap experiment that settles it, and it needs no Jetson.** Run a
   full-size open-weight VLM on a rented GPU against the four walks and score
   its precision on the 209-frame search walk with `control/perception_eval.py`
   -- can it beat **10/44** on the bin frames? If yes, that is the first real
   evidence for on-board deliberation and the Jetson is worth costing
   properly. If no, distilling it smaller cannot fix it. That is 4.7's
   promotion rule and 4.9's own closing line doing their job: **the car does
   not need to be the laboratory.**

   **And note what it still would not fix**, which is why this is a re-open
   trigger rather than a plan: the local tier's measured bottleneck is crop
   proposals (9 of 11 errors, 0 matching) and a VLM does not propose crops;
   the navigation gap is that a single-step `/navigate` has no memory of which
   way it already turned, which is C8 and is inherited whole by any model.
   4.11's own measurements are the pattern -- Grounding DINO added capacity on
   the wrong axis and lost at every matched operating point, while SAM
   addressed the measured weakness and took the search walk from 2/10 to 8/10.

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

### 4.10 The harness: running these models before buying the part -- **P1-P4**

**Added 2026-09-06, and P1/P2 are built.** Everything above decides *which*
models go on the car. This is how they get tried first, and the question that
prompted it is worth recording because the obvious answer is wrong:

> *Can the digital twin test YOLO + CLIP + Opus 4.5 before I buy the hardware?*

**Not the twin -- and you can do better than the twin.** Three questions are
hiding in one, and they have three different instruments:

| Question | Instrument | Needs hardware? |
|---|---|---|
| Does the **tiered loop** work -- goal issued, trigger fires, arbitration behaves, collar vetoes | the twin, on 1.12's synthesised features | no |
| Do **YOLO and CLIP actually find things at 10cm** | real pixels through `ReplayRobot` / `TeleopRobot`, models on a laptop | **no** |
| Throughput, HEF context-switch cost, latency, anything lidar | -- | yes |

The middle row is the one that gates a purchase, and **the twin cannot answer
it, by 1.12's deliberate design**: `sim/renderer.py` draws flat-shaded raycaster
walls, a COCO detector finds nothing in them, and *"the sim leg tests the
detector's consumers, never the detector."*

**And the tempting repair is worse than the gap.** 1.12 weighed "make the
renderer detectable by drawing real objects" and rejected it. Texture-map a
photograph of a backpack onto a sprite, have YOLO find it, and what you have
learned is that YOLO detects sprites -- a **false** positive signal, which is
worse than none, and the standing-height corpus error wearing a new costume.

#### What makes this possible at all

Two things this project already owns, neither of which was bought for it.

**Two `RobotInterface` backends made of real photons.** `sim/replay_robot.py`
plays a recorded walk back one frame per move -- open loop, which does not
matter here, because a detector does not care whether the next frame was caused
by its own decision. `sim/teleop_robot.py` is the closed-loop one: a live phone
camera and a human as the motor, already built and deployed as T1-T4.

**And the models do not need the Hailo.** The chain is PyTorch -> ONNX -> DFC ->
HEF and **only the last step is Hailo-specific** (4.8). Running a model to find
out whether it *finds things* needs nothing but `pip`. That is the whole reason
the Hailo beat the IMX500, which *"could not be fed a stored image at all"*
(1.10) -- and until now nothing had exercised it.

So the highest-fidelity pre-hardware test available is: **phone on the wheeled
rig -> `TeleopRobot` -> real frames -> YOLO11s + CLIP locally -> candidate
sighting -> Opus 4.5 confirms identity and reachability -> the decision comes
back to the phone.** That is 2.8's entire mission, all three tiers in order, on
real pixels, closed loop, with no robot in existence.

#### The phases

| | What | Status |
|---|---|---|
| **P1** | **The pipeline.** `brain/perceive.py` -- detector -> crops -> CLIP -> match, with 4.2's two crop paths chosen by whether the target has a COCO word, 1.12's three-way output, and bearing from the box plus the frame's own pan angle (1.15.3). `Detector` and `CropScorer` are Protocols, so a HEF substitutes later with nothing in between changing | **BUILT 2026-09-06.** `tests/test_perceive.py`, 22 tests, all against fakes |
| **P2** | **The trigger discipline.** `brain/tiered.py` -- a `vision_fn` that runs perception locally and calls the cloud only on `mission_start`, `candidate_sighting` or `cold_search`, with 6.1's two-frame hysteresis and a call counter. Plugs into `MissionRunner`'s existing seam, so nothing in `control/` learns perception grew a tier (2.6) | **BUILT 2026-09-06; twin surface 2026-09-07.** `tests/test_tiered.py`, 26 tests, plus `policy: "tiered"` end to end -- see below |
| **P3** | **Score it on the corpus.** Run P1 over the whole corpus against each walk's adjudicated `labels.json`, sweep the gate, and report recall and precision per walk and in total. `tests/manual_perceive_walk.py` is the single-walk version | **BUILT 2026-09-08** -- `control/perception_eval.py`, beside `control/walk_eval.py` as the phase always said, with `tests/test_perception_eval.py` (31 tests, all against fakes). It reproduces 4.11's shipped row exactly on first run, which is the only validation a scorer can have. See "P3 is a tool now" below |
| **P4** | **The compile step.** ONNX -> Hailo DFC -> HEF on a rented x86-64 host, with floor segmentation as its first subject rather than YOLO (4.3). This is 1.10 item 1, and it is the remaining fifth | **NOT BUILT, but its subject now exists.** `SegformerFloorProposer` (2026-09-07) is the model 4.3 says to compile first, in this repo behind a `RegionProposer` Protocol and measured at +8 points of recall. Still needs an EC2 hour and a Hailo developer account. No robot |

**P1-P4 is where 1.10 item 1's compile loop lives, and its absence from C1-C9
was a real gap** -- the consistency review found that the one thing this plan
says to do *before hardware day* had no phase at all. It has one now, and it is
a different series on purpose: C1-C9 is about motion and the loop, P1-P4 is
about perception content, and 1.14's own note says a perception change that
forces no change to `set_velocity` is evidence the layering is right.

#### What it owes the twin -- **PAID 2026-09-07 for P2**

6.3 already specifies this and P1's output shape was built to match it: the
detector's boxes on the FPV canvas, the model's name, the CLIP score against the
mission's target string, and the tri-state readout so a wedged capture never
looks like a missing target. **Plus the counter** -- 6.3's *"a
deliberation-call counter that visibly does not climb every step. That single
number makes the whole architecture watchable."* `TierStats.as_dict()` is that
number, and it reports `frames_per_call` directly comparable to 6.1's measured
4-6x.

**Built, and it needed no transport phase.** The estimate above said *"C5's
transport plus a panel"*, and C5 turned out not to be on the path at all: C5
moves *synthesised* features off the robot for the sim leg, and P2's models run
in the **brain process**, where the panel already polls `GET /mission/status`.
So the whole surface is one new policy plus four readouts:

- **`policy: "tiered"`** on `control/mission_runner.py`'s `POLICIES`, a branch
  in `control/brain_server.py` that wraps `brain/navigate.py`'s `vision_fn_for()`
  in `brain/tiered.py`'s `TieredVision`, and the option in the Remote brain
  panel's policy picker beside "frontier" and "vision".
- **Validated at mission start**, exactly as M1 validated `model_id` and for the
  same reason: `ultralytics`/`torch` are an optional install, and a missing one
  discovered *inside* a tick is counted by B3.2 as a vision failure -- so the
  mission would limp through three of them and die reporting "vision unavailable
  3 times in a row" with a robot standing in a room throughout. It is a 400
  naming the pip command instead. `GET /health` publishes
  `perception_available` so the panel can say so before Start is pressed.
- **Four readouts**, hidden under any policy with no perception tier (a counter
  reading "0 calls over 0 frames" would say the architecture had stopped
  deliberating rather than that it was never running): the tri-state with the
  matched label and bearing, the **CLIP margin** rather than the bare similarity,
  the detector and encoder by name, and the counter as **calls *and* frames** --
  a call count climbing by one is indistinguishable from a call every step,
  which is precisely the thing this policy claims not to do.
- **The mission log names the paid steps**, `[cloud: candidate_sighting]`, so
  the saving is legible line by line and not only as a ratio.

Not built here, because both need something that does not exist yet: the
detector's **boxes on the FPV canvas** (the frames a tiered mission perceives are
the robot's own, and drawing boxes over the twin's FPV would be drawing them over
a raycaster render -- 1.12's ban, since the sim leg tests the detector's
consumers and never the detector), and the **HEF name changing on a swap**, which
is P4. The name shown is whatever `brain.perception_detector` pins, which is the
same mechanism one file earlier.

A live check on 2026-09-07, against two real uvicorns with the extras
deliberately *not* installed: the policy appears, the panel warns before Start,
and the mission refuses with `ultralytics is not installed. pip install -r
requirements-perception.txt` and no mission left running. Playwright covers all
four readouts at a 390px viewport, and found a pre-existing bug on the way in --
`.select-input` had no `min-width: 0`, so `#brain-fault` was already pushing the
Sim tab 21px wider than a phone and the tiered option's label took it to 213px.

#### Why this was not in C1-C9, which is the more useful question

Asked directly on 2026-09-07, and the answer is worth recording because the
plan was *right about this capability every time it mentioned it* and still
did not schedule it. Four reasons, and the third is the general one.

**1. P3's synthesis half was in C5 -- as a clause, not as scope.** C5 reads
*"Carries 1.12's synthesised detections, floor mask and CLIP scores"*, while its
actual scope is transport: routes, `RemoteRobot`, conformance cases, routing
patterns. The synthesis -- the occlusion ray, the tri-state, the noise flag and
the tests written *with* the flag -- is the larger half, and it appears as a
subordinate clause inside another phase's sentence. **Half-counted is worse than
absent**, because it reads as covered.

**2. C6 already schedules this pipeline -- on the robot, and only there.** C6
*"hosts a pipeline, not a detector: detector -> crops -> CLIP -> match (4.2)
plus floor segmentation (4.3)"*. So the work was phased. What the phasing never
considered is that **the identical pipeline runs on a laptop against recorded or
teleop frames, months earlier, with no HEF, no accelerator and no robot.**
1.14's closing note diagnoses the shape of the miss without quite catching it:
it frames the split as *"perception content"* against *"motion and the loop"*
and says C4-C9 repaired the gap. C4-C9 repaired the **goal and planner** gap.
The missing piece is neither content nor motion -- it is *where the content
runs*, and that axis had no representation in the phasing at all.

**3. The capability was named three times as a reason to BUY, and never once as
a task.** This is the general failure and the one to guard against. Every
mention of off-robot evaluation in this document lives in a **purchasing
argument**:

| Where | Section | What it says |
|---|---|---|
| 1.10 | *"What it buys that the plan had not weighed"* | *"The detector can be scored on the recorded corpus... The IMX500 could not be fed a stored image at all"* |
| 4.7 | *"The Jetson path, kept for later"* | *"experiments run off-robot behind the perception seam... promoted to the Hailo by compile"* |
| 4.8 | *"So: is it a regrettable spend?"* | the same promotion rule, as risk mitigation |

It is **load-bearing in all three** -- it is a deciding argument for the Hailo
over the IMX500, which is to say the plan spent $70 partly on the grounds that
this was possible, and then never wrote down that it should happen.

> **A reason to buy does not automatically become a thing to do.** Purchasing
> sections argue; phasing sections assign. Nothing in this document's structure
> carried a capability across that boundary, and nothing was shaped to notice
> the omission.

1.10 item 1's compile loop was in exactly the same state and was found by the
same review -- *the one thing this plan says to do before hardware day*, in no
phase at all. Two instances of one pattern is a pattern. **When a section
argues that something is possible, ask in the same breath which phase owns
doing it.**

**4. And the derivation method had a bias worth naming.** C1-C9 was produced by
walking **2.8's mission** against the repo, and 2.8 is a mission *on the robot*.
Walking a robot mission end to end reliably produces robot-shaped phases. The
complementary exercise -- **what could be tested today, with no robot at all**
-- was never run, and it yields a different list, which is this one. The rule
already exists in 4.7; it had simply never been turned on the plan itself.

**None of the four is a reasoning error**, which is what makes them worth
recording. Each is a bookkeeping failure of the same family as 1.15.3 adding a
fourth goal verb that 1.7 never heard about, and 3.6's totals pricing a
configuration 1.14 tells you not to buy: correct local reasoning, with no
mechanism to propagate it.

#### Two findings from the same review, recorded so they are not lost

**The project's own introduction now describes a different robot, and nothing
said so.** `INTRODUCTION.md` defines the project *against* mapping -- *"This
project takes the other road: **no map at all**"* -- while this document buys a
lidar, moves depth off the VLM, and names ROS 2 with `slam_toolbox` and nav2 as
the destination. **The reversal is correct**: Stage 0 tested the original
premise honestly and it lost. But §0 of this document only says that *this
document* ends somewhere different from where it started. It never says the
**project** does, and a reader arriving at the introduction and stopping would
carry away a description of a robot that is no longer being built. Fixed in
`INTRODUCTION.md` on 2026-09-07; recorded here because the class of error --
a decision propagating into the planning documents but not the explaining ones
-- is the same one this whole subsection is about.

**And the honest note on pace.** The stretch that produced 1.14, 1.15, 1.16 and
C1-C9 -- four major decisions and roughly 1,500 lines of plan -- contained **no
code and no measurements**, in a project whose expensive errors have every time
been found by contact with reality rather than by planning: the invalid corpus
(found by opening the frames), the `NavigateModelId` env-var trap, the model
picker that rendered 40px wide. Planning here has genuinely paid -- §5.1's
`PATH_FRACTION` bug was found and fixed the same day, the 2S/3S correction came
from pricing real parts, §6.1 turned a guessed 8x into a measured 4.1x -- but it
paid most while it was **pruning**, and it became expensive once it started
**generating** work and purchases on unmeasured premises. 1.14 is the worked
example: it is the only major decision in this document with **no stated
requirement**, and it makes a $70 part non-deferrable and a nine-phase rewrite
necessary. Before C1-C3 is built, answer the question that section skipped --
*what does continuous motion buy, and what fails without it?* -- and if the
honest answer is a preference rather than a requirement, measure a room crossing
first and let the number decide.

#### The corpus these were measured on **no longer exists** -- deleted 2026-09-07

Everything in the next three subsections was measured on the 39-walk
standing-height corpus, which was deleted from S3 and from the local backup
on 2026-09-07, deliberately and on the owner's instruction: the walks were
invalid by their own viewpoint, so measurements taken on them were suspect
anyway, and leaving them in the bucket is how someone scores against them by
accident -- a mistake this document has already recorded twice.

**Read every number below as a recorded observation, not a reproducible
one.** They are kept because the *reasoning* they support is still the best
available and because deleting the conclusions along with the data would
lose the argument as well as the evidence. But nothing here can be re-run,
and **anything load-bearing must be re-derived on the new corpus**. Two
things in particular are now assertions rather than measurements:

- **The handbag negative control** (+0.039 against `"red backpack"`), which
  is the entire basis for *"one threshold does not serve both targets"*.
  Re-measure it on the next backpack rig walk before trusting the claim.
- **6.1's 4.1x trigger count**, measured over 821 frames of that corpus. The
  live tiered runs since (1 per 3.5, 1 per 5.17) are consistent with it, and
  are on frames that still exist.

The corpus is now **three rig walks**, all valid (floor height, target on
the floor), in
`s3://vision-picar-recordings-303351622021-us-east-2/recordings/`:
`blue-bottle-20260907-142454`, `red-backpack-20260907-144856`,
`blue-shoes-20260907-152528`.

**Recording writes to the MacBook, not to S3** -- `brain.recording_backend`
is `local`, so a walk lands in `./recordings/` and has to be uploaded. Two
of these three existed only on one disk until someone thought to check.

#### The first real run: **`DEFAULT_MATCH_MARGIN` is about 2x too high**, measured 2026-09-07

`python -m tests.manual_perceive_walk` over
`red-backpack-20260829-195904` (22 frames, the **invalid** standing-height
corpus), YOLO11s + CLIP RN50 on a MacBook, ~206ms/frame. **0 of 22
detected** -- and the reason is not the detector.

| | n | min | mean | max |
|---|---|---|---|---|
| crops YOLO labelled `backpack`, scored against `"red backpack"` | 14 | **+0.016** | +0.028 | **+0.034** |
| every other crop in the same frames | 78 | -0.109 | -0.034 | +0.039 |

**The detector found the backpack on 14 of 22 frames** (best confidence
0.62; 21 frames at `conf 0.05`), and CLIP scored every one of those crops
*positive* against the distractors, in a tight band. The pipeline still
returned `absent` on all 22 because `DEFAULT_MATCH_MARGIN` is **0.05** and
no true positive ever got there. That constant's own comment calls itself
*"provisional and uncalibrated"*; this is the first evidence of which way,
and it says the whole true-positive band sits under the threshold.

**It has NOT been changed, and should not be until a rig walk exists.** Two
reasons, and the second is the one that matters. The corpus is the invalid
one -- standing height, target across the room on an ottoman -- so a
threshold fitted to it would be fitted to the wrong viewpoint, which is the
`NavigateModelId` mistake in a new place. And the negative column shows the
distributions **overlap**: the five highest non-target margins are all
`handbag`, topping out at **+0.039**, above every true backpack. The label
gate happens to filter those out on this target (4.2's path A keeps only
crops YOLO labelled `backpack`), so the overlap costs nothing *here* -- but
on 4.2's open-vocabulary path there is no label gate, and a threshold
tuned to +0.015 on this data would admit every handbag in the house.

**What this does settle**: the harness works, the two models load and run
on any machine, RN50's absolute similarities are ~0.18-0.21 with
distractors at ~0.16, and the quantity to calibrate is a **band, not a
point** -- which is what P3's corpus-wide scoring should report when there
is finally something valid to run it on.

#### And the corpus-wide version, run the same day -- **the target string matters more than the walk**

All **39 walks, 821 frames**, YOLO11s + CLIP RN50, at the shipped 0.05
threshold. `unavailable`: **0**, everywhere -- 1.12's wedged-capture state
never fired on a real corpus, which is the first evidence that the tri-state
is not hiding a decoding problem.

| target string | walks | frames | label-gated | detected @0.05 | best margin seen |
|---|---|---|---|---|---|
| `"red backpack"` | 25 | 508 | 41% | **13%** | +0.100 |
| `"blue bottle"` | 9 | 176 | 49% | **0%** | +0.041 |
| `"bottle"` | 5 | 137 | 23% | **0%** | +0.014 |

**The single biggest lever is the words in the target string, not the
walk.** `"blue bottle"` is gated *more* often than `"red backpack"` -- YOLO
finds bottles fine -- and never once clears the threshold, while `"bottle"`
tops out at +0.014, an order of magnitude below the backpack walks. That is
CLIP behaving exactly as its own caveat predicts: a generic noun sits close
to `"a household object"` in the distractor set, so the margin collapses
even when the detection is perfect. **A two-word target with a colour and a
noun is worth roughly 5x the margin of the bare noun.** 1.7's goal
vocabulary should say so, and P3's scoring should report margin *per target
string*, not only per walk.

The spread across walks of one target is real but secondary: for
`"red backpack"`, the Aug 30 sessions reach +0.06-0.10 while
`red-backpack-20260829-184355` (120 frames) gates 3 times with **negative**
margins. Best walk of the corpus is
`red-backpack-qwen3-vl-235b-a22b-20260829-214954` at 8 detected of 11.

#### And the whole chain, end to end -- **YOLO -> CLIP -> Opus 4.5, 2026-09-07**

`python -m tests.demo_replay_mission <walk> "red backpack" --policy tiered`
over `red-backpack-opus-4-5-20260829-214849` (14 frames; 12 label-gated, 6
over the threshold -- picked from the table above as a walk with *both*
states, so all three implementable triggers could fire). Real deployed
`/navigate`, Opus 4.5, **shipped 0.05 threshold, nothing tuned**:

| | |
|---|---|
| outcome | `found` -- the model reported `target_reached` |
| steps | 18 (perception: 12 `absent`, 6 `detected`) |
| **paid calls** | **4** |
| triggers | `mission_start` 1, `candidate_sighting` 1, `cold_search` 2 |
| saving | 1 call per 4.5 steps -- **1 per 3.5 distinct frames** |
| wall clock | 15.0s, 0.8s/step |

**2.4's claim is now measured live rather than replayed, and it lands
inside 6.1's 4-6x band** on the first attempt, at a threshold nobody tuned
for it. All three triggers that P2 can honestly fire did fire, and
`cold_search` earned its place twice: the run spent six consecutive frames
`absent` before the cloud found the backpack that perception had missed --
which is exactly 2.4's inverse case, the cloud proposing and on-board
tracking.

**Read the saving as 1-per-3.5, not 1-per-4.5.** The walk is 14 frames and
the mission took 18 steps, so the last four re-perceive `frame-0013.jpg` --
`ReplayRobot` running off the end. Those frames are free and inflate the
denominator. And `found` is **not** a navigation result: a replay is open
loop, and the arrival came on the walk's final frame, which is where the
person holding the phone had already chosen to stop.

**Two defects in P1/P2 were found by this run, both now fixed**, and both
are the kind only a real run surfaces:

1. **`bearing_deg` had never once been a number.** `_bearing()` needs the
   frame's pixel width and **no `RobotInterface` backend publishes
   `image_width`** -- so 1.11's "which way is it", the output the whole
   bearing-from-the-box design exists for, was `None` on every frame this
   project can produce. Fixed inside `brain/perceive.py` by reading the
   width off the image itself when the frame declares none, rather than
   widening the frame contract across every backend and the conformance
   suite for a value the pixels already carry.
2. **A detected target logged as `not_visible`.** `brain/tiered.py`'s local
   scene set `target_visible` from the tri-state but hardcoded
   `target_direction: "not_visible"`, so a frame matched at +0.068 read
   "target not_visible" in the mission log -- the two fields contradicting
   each other on precisely the frames perception got right. The direction
   now comes from the bearing above (`left`/`center`/`right`), or
   `unknown` when there is no bearing to report, which is a third state
   and not the same as `not_visible`.

#### The first VALID walk, and it overturns a design decision -- 2026-09-07

`recordings/blue-bottle-20260907-142454`. **33 frames, camera at floor
height on a wheeled rig, the target standing on the floor** -- the
re-recording 1.16 #10 has been asking for since 2026-09-02, and the first
walk in this project that is not disqualified by its own viewpoint. Four
things were blocked on it. Two of them move here.

**1. The threshold. `DEFAULT_MATCH_MARGIN` is measured wrong, and now by two
independent corpora.**

| | n | min | median | max |
|---|---|---|---|---|
| frames the VLM called the bottle visible | 18 | **+0.025** | +0.032 | **+0.038** |
| frames it called it not visible | 15 | -0.067 | -0.013 | **+0.004** |

A gap of 0.021 with nothing in it. At **0.02: 18 of 18, zero false
positives.** At the shipped 0.05: **none**. The old corpus put the band at
+0.016..+0.034; this one puts it at +0.025..+0.038. The module default is
**still not changed**, and the reason is in the old corpus's negative
column: handbags scored +0.039 against `"red backpack"`, which 0.02 would
admit. **One threshold does not serve both targets** -- either it becomes
per-target, or the distractor set has to carry the near-neighbours
(`"a handbag"` for a backpack, `"a vase"` for a bottle). That is the real
open question, and it is now a sharp one rather than a guess.
`brain.perception_match_margin` exists so it can be varied on a walk;
`config/robot.yaml` sets 0.02 with this measurement written beside it.

**2. The bigger finding: 4.2's label gate is losing most of the true
positives, and losing them at the worst possible moment.**

YOLO11s found the bottle on **7 of the 18 frames** the VLM called it
visible. Zero false positives -- but eleven misses, and reading *what the
detector said instead* explains all of them:

| frames | YOLO's label | what is actually in shot |
|---|---|---|
| 0026-0030, 0032 | `vase` | the bottle, filling the centre of the frame |
| 0031 | `refrigerator` | the bottle, closer still |
| 0019, 0020, 0024 | `vase` | the bottle on the approach |

**At close range the detector relabels the object.** A bottle 30cm from a
10cm-high camera is a large blue cylinder, and COCO's word for a large
cylinder is `vase`. 4.2's label gate keeps only crops labelled `bottle`, so
it discarded every frame **from the approach onward** -- precisely the
frames where the target is unmissable and where a robot most needs to know.

Forcing 4.2's other path -- open vocabulary, labels discarded, CLIP deciding
identity outright -- recovers **18 of 18, still with zero false positives**
at 0.02. The path 4.2 calls *"the noisiest of the three sources"* is, on
this walk, strictly better than the one it recommends. **The noise is in the
labels, not in CLIP.**

This is 4.3.1's standing-height caveat arriving with a number:
*"a chair from 150cm is a chair; a chair from 10cm is four poles and the
underside of a seat."* It was written about detection rate. The measured
failure is not rate, it is **identity** -- and a class-gated pipeline
inherits that failure whole.

**Not resolved by fiat.** 4.2's rule is about who *proposes*, and is still
right in the general case: an out-of-vocabulary target has no label to gate
on at all. What has changed is that the rule is now measurable --
`brain.perception_crop_path` takes `auto` (4.2's rule, still the default),
`label_gate` or `low_confidence`. The next two rig walks, on different
targets, decide whether the default moves. **Do not move it on one walk**;
that is the mistake this document has recorded twice already.

**3. And the whole chain ran on it.** `--policy tiered --margin 0.02`
against the deployed `/navigate`: outcome **`found`**, arrived at step 30,
**6 paid calls over 31 frames -- 1 per 5.17**, triggers `mission_start` 1 /
`candidate_sighting` 2 / `cold_search` 3. Inside 6.1's 4-6x band on a real
walk, at a threshold measured rather than guessed. The label-gate defect is
legible in that run too: frames 0026-0029 read `absent` with the bottle
filling the view, and `cold_search` -- not `candidate_sighting` -- is what
eventually fired the arrival call.

**What is still owed from this walk**: 3 of 33 frames are portrait again
(the model has complained about that twice before), and the walk's meta note
does not record the rig height, which Stage 0 asks for in writing so the
corpus describes its own viewpoint.

#### The second valid walk, and the three things it changed -- 2026-09-07

`recordings/red-backpack-20260907-144856`, 38 frames, floor height, backpack
on the floor, 37/38 landscape. Two walks and two targets is what 4.10 said
would decide the crop path. It decided it, and it overturned one more thing
on the way.

**1. The threshold is not a margin at all.** The backpack walk did NOT
confirm the bottle's number -- it contradicted it, which is the useful
outcome:

| target | margin gate 0.05 | margin gate 0.02 |
|---|---|---|
| blue bottle | 0 of 18 | 18 of 18, 0 false pos |
| red backpack | 8 of 31 | 24 of 31, **2 false pos** |

No single margin serves both, and the reason is a property of the model
rather than of this corpus: **a raw CLIP similarity is not comparable across
text queries.** `"red backpack"` is simply a stronger phrase than
`"blue bottle"` -- its target frames sit at a median margin of +0.069
against the bottle's +0.032.

The fix is the standard one and costs nothing: **softmax over the same
scores**, giving `P(target | crop, texts)`. That IS comparable across
targets, and one number now serves both -- the bottle's target frames sit
at P >= 0.85 against a non-target maximum of 0.39. `P >= 0.8` is the
shipped gate (`DEFAULT_MATCH_PROBABILITY`); the margin remains reported,
and settable as an override for sweeping a corpus. **What would falsify
it**: a target whose non-target frames also reach 0.8 -- the backpack walk
already shows two, so this is a better metric, not a solved problem, and
near-neighbour distractors are the next thing to try.

**2. The crop path flips to open vocabulary.** Both walks, one gate:

| target | `auto` (4.2's label gate) | open vocabulary |
|---|---|---|
| blue bottle | 7/18, 0 false pos | **18/18, 0 false pos** |
| red backpack | 9/31, 1 false pos | **25/31, 2 false pos** |

Consistent in direction on both, and large: 16 recovered detections for one
extra false positive. **And the trade is 1.11's, not a preference.**
Arbitration is split by question -- on-board *proposes*, the cloud
*confirms identity* -- so a false positive costs one deliberation call that
the VLM then rejects, while a miss means the robot drives past its target.
`auto` remains selectable and remains right about who proposes.

**3. And better perception broke the mission, which is the finding that
generalises.** With the two changes above, perception reported `detected`
on 23 of 38 and 32 of 43 frames -- and both walks went from `found` to
`max_steps`, never arriving. The cause is structural:

> `candidate_sighting` fires on the **edge** into `detected`. `cold_search`
> needs a run of `absent`. So a robot that can see its target continuously
> fires nothing after the opening call -- and **arrival is the VLM's call**
> (1.11), so it is never made. An all-detected walk made exactly **one**
> cloud call, because the opening call also seeds the edge detector.

A deliberation tier that can be silenced by things going well is
edge-driven, not event-driven. 2.4's own list already carried the fix:
**`staleness`**, which this module had listed as unavailable ("needs a goal
with an age") on a reading that was too strict -- the *call* has an age, and
that is enough. `DEFAULT_STALE_AFTER = 8`, 6.1's own figure, which it
measured contributing 5.7% of triggers. It is a floor, not a metronome: any
real event resets it, and the call cap still bounds it.

With all three, both walks arrive:

| walk | outcome | paid calls | saving | triggers |
|---|---|---|---|---|
| blue bottle | **found** (step 37) | 6 / 38 frames | 1 per 6.33 | start 1, candidate 2, cold 1, stale 2 |
| red backpack | **found** (step 35) | 5 / 36 frames | 1 per 7.2 | start 1, candidate 1, stale 3 |

Both above 6.1's 4-6x band rather than inside it -- better perception really
does buy fewer calls, once the floor stops it buying zero.

#### The third walk: the out-of-vocabulary case, and the target STRING dominates -- 2026-09-07

`recordings/blue-shoes-20260907-152528`, 19 frames, 19/19 landscape, one
shoe stood upright, ending with the pair filling the view. The first target
with **no COCO word at all**, which is the case 4.2's open-vocabulary path
exists for and which neither earlier walk could test -- `bottle` and
`backpack` are both COCO classes, so the detector was always proposing boxes
on the target even when it mislabelled them.

**The detector never proposes a shoe.** At conf 0.05 over the whole walk the
labels are `couch` 12, `bed` 11, `chair` 8, `vase` 6, and one each of `cat`,
`potted plant`, `book`. Not one proposal is of the target. Only 2 of 19
frames had no proposal at all, so crops exist -- they are just **furniture
boxes that happen to overlap the shoes**, which is 4.2's low-confidence path
working exactly as described and no better.

**And then the target string moved the result 7x.** Same frames, same gate,
same crops:

| target string | detected / 13 visible | false pos | best P |
|---|---|---|---|
| `"shoes on the floor"` | **0** | 0 | 0.75 |
| `"blue shoes"` | 1 | 0 | 0.84 |
| `"sneakers"` | 1 | 0 | 0.89 |
| `"running shoes"` | 3 | 0 | 0.99 |
| `"a pair of running shoes"` | 4 | 0 | 0.99 |
| `"blue and yellow running shoes"` | **7** | 0 | **1.00** |

Zero false positives in every row, so this is recall bought for nothing.
Two things in that table are worth more than the headline:

**The best string is the most ACCURATE one, not the longest.** The shoes are
navy with lime accents. `"blue shoes"` is a poor description of them and
scores like one; `"blue and yellow running shoes"` is what they actually
look like. This is not prompt engineering -- it is the target string being a
*description* that CLIP matches against pixels, and 1.7's goal vocabulary
currently treats it as a label.

**`"shoes on the floor"` scores WORST, and that confirms the distractor
collision.** `"a floor"` is in `DEFAULT_DISTRACTORS`, so putting the word in
the target string hands the softmax a competitor built from the same words.
This is the near-neighbour problem predicted at the end of the second walk,
arriving from the opposite direction: not a distractor too close to the
target, but a target too close to a distractor.

**Both strings still complete the mission**, which is the tiering working as
designed -- the cloud is what recovers a weak local signal:

| target string | outcome | perception | paid calls | triggers |
|---|---|---|---|---|
| `"blue shoes"` | **found** (step 19) | 1 detected, 19 absent | 4 / 20, 1 per 5.0 | start 1, cold_search 3 |
| `"blue and yellow running shoes"` | **found** (step 22) | 7 detected, 16 absent | 4 / 23, 1 per 5.75 | start 1, candidate 2, cold 1 |

Read the trigger columns: with a poor description the mission runs almost
entirely on `cold_search` -- the on-board tier contributes nothing and the
cloud does all the work, at the same price. That is the degraded mode 2.5
describes, reached not by a failure but by a badly chosen noun.

**What this says for the hardware.** 4.2 lists four crop sources and only
two are reachable today; this walk is the first evidence about what the
other two are worth. An out-of-vocabulary target gets crops **only where
furniture happens to overlap it**, so floor segmentation (4.3) is not an
optimisation for this case -- it is the difference between proposing on the
object and proposing near it. That is a reason to build the compile loop
(P4) around segmentation, which 4.3's note already suggested for a different
reason.

#### Near-neighbour distractors: **measured, and rejected** -- 2026-09-07

The experiment the previous two sections kept pointing at. Three
configurations over all three walks, offline, no cloud calls: the current
generic set; that set plus a global near-neighbour list (`"a vase"`,
`"a cup"`, `"a handbag"`, `"a cushion"`, `"a slipper"`,
`"a cardboard box"`); and that set plus per-target near neighbours.

Scored **threshold-free** on purpose -- the metric is how many visible
frames are detected at the strictest threshold that still admits zero false
positives. A fixed `P >= 0.8` would have confounded "cleaner signal" with
"more competitors in a softmax mechanically lowers every P".

| walk | base | +global | +per-target |
|---|---|---|---|
| blue bottle | **18**/18 | 18/18 | 18/18 |
| red backpack | **15**/31 | 13/31 | 14/31 |
| shoes | **10**/13 | 9/13 | 10/13 |

**No configuration beats the base set on any walk, and two are worse.**
The mechanism is visible in the numbers: adding near neighbours does lower
the top score on non-target frames (the backpack's worst case falls from
0.97 to 0.61), but it lowers the score on *target* crops by as much. The
distractors compete with the truth as effectively as with the error.
**Do not add them.** `DEFAULT_DISTRACTORS` stands.

#### ...and the experiment was measuring a phantom, which is the larger finding

The two backpack "false positives" the near-neighbour idea existed to
suppress were **opened and looked at**. The backpack is plainly visible in
both, sitting under the table, and CLIP scored it 0.97 and 0.93. What the
VLM said was:

> *"No red backpack is visible in the current view, only a burgundy/maroon
> fabric item under the table."*

It rejected the target **over the colour word**, having accepted the same
object on the frames either side. So the backpack walk has **zero** false
positives, not two, and the recall figures for it are understated.

**Every TP/FP number in this section uses the VLM's own `target_visible` as
ground truth, and it is not ground truth.** It disagrees with itself about
one object across adjacent frames. That is a methodological hole under all
three walks, and the fix is cheap and manual: a per-frame human label in the
walk's own directory, which is what P3's corpus-wide scoring should read
instead of `walk.jsonl`.

**And the two tiers fail the same way, which is the part worth carrying
forward.** An inaccurate colour word costs recall in *both*:

| target string | frames detected of 38 |
|---|---|
| `"red backpack"` | 27 |
| `"burgundy backpack"` | **30** |
| `"maroon backpack"` | **30** |
| `"a dark red backpack"` | 30 |

CLIP loses three frames to the wrong colour word; the VLM rejects two
outright and says so in its reasoning. This is the third walk's target-string
finding arriving independently on the second walk's data, and it converges
on the same instruction: **1.7's goal vocabulary carries a description, and
the description has to be accurate.** `"red backpack"` for a burgundy bag is
not a small imprecision -- it is the single biggest lever measured in this
document that costs nothing to pull.

#### The corpus has ground truth now, and it cost eleven frames -- 2026-09-07

The hole the previous section opened -- *"the VLM's `target_visible` is not
ground truth"* -- turned out to be cheap to close, and the reason is worth
keeping as a method rather than a one-off.

**Label by agreement, adjudicate only the disagreements.** Across all three
walks, CLIP and the VLM agree on **79 of 90 frames**. Agreement is taken as
correct; only the **11** disputed frames need an eye, and they were opened
and judged. Twelve per cent of the corpus, not all of it.

**All eleven have the target plainly in view.** So:

| | frames | verdict |
|---|---|---|
| VLM wrong | 2 | the colour-word rejections (§ above) |
| CLIP wrong | 9 | every one a **crop-source** failure |

The labels are now `labels.json` beside each walk, in the directory and in
S3, carrying the per-frame flag, which frames were adjudicated, and a note
saying plainly that the VLM's own answer is not the reference. **P3's
scoring should read that file**, never `walk.jsonl`.

#### And the nine misses are all the same failure, which settles where the effort goes

Each of the nine was re-run with the proposals printed:

| what happened | n | frames |
|---|---|---|
| **no proposal at all** | 2 | both **close-ups**, target filling the view |
| proposals existed, none on the target | 7 | all `chair` / `couch` / `vase` / `bed` |

**Not one is a CLIP failure.** In the seven, CLIP was shown furniture and
correctly scored furniture low; in the two, it was shown nothing. The
matcher is not the bottleneck -- **the crop source is**, and it fails in
both directions at once: it proposes nothing when the object is close
enough to fill the frame, and proposes only furniture when it is further
away.

That is the third walk's conclusion arriving again on the second walk's
data, and it now has a number: **9 of the 11 errors in the corpus are crop
proposals, 0 are matching.** 4.2 lists four crop sources and ships the
weakest of them. Floor segmentation (4.3) is not an optimisation -- on this
evidence it is the single highest-value thing left before hardware, and it
is what P4's compile loop should be built around.

#### Floor segmentation: **built and measured** -- 2026-09-07

The previous section said this was *"the single highest-value thing left
before hardware"*. It was built the same day, and the measurement is more
interesting than that claim was.

**What it does.** 4.2's second crop source, as pure geometry: *anything
that is not floor, stands on the floor, and does not reach the top of the
frame is an object worth cropping.* No class list at any point, which is
exactly where the corpus's errors live. `SegformerFloorProposer` in
`brain/perceive.py`, behind a `RegionProposer` Protocol so the laptop model
and P4's HEF are interchangeable. SegFormer-B0 on ADE20K, ~14MB, chosen
because ADE20K is one of the few segmentation sets carrying `floor`, `rug`
and `earth` as classes at all.

**The first implementation returned zero proposals on every frame**, and
the reason is worth keeping. Taking connected components of a *binary*
not-floor mask finds one enormous blob per frame -- wall, furniture and
object are all touching -- and it reaches the ceiling, so the "does not
reach the top" test rejects everything. The fix is to split the not-floor
region **by class region** first: the segmenter's labels are used for
exactly two things, deciding which pixels are floor and separating the
object from the wall it stands against, and are then discarded. The
object's own class is never consulted, so the method stays open-vocabulary.

**Measured at `P >= 0.8` over all three walks against the adjudicated
labels in `labels.json`:**

| crop source | detected of 64 visible | false pos | recall |
|---|---|---|---|
| detector only (shipped) | 55 | 0 | 86% |
| **floor mask only** | 38 | 1 | **59%** |
| **both** | **60** | 1 | **94%** |

**It is worse alone and better together**, which contradicts the previous
section's framing and is the useful finding. The mask proposes coarse
regions and misses small distant targets the detector finds easily; the
detector cannot propose on an out-of-vocabulary object and stops proposing
at all once one fills the view. They fail on different frames. Of the nine
crop-source failures, the union recovers **five** -- including **both**
close-ups that previously produced no proposal whatsoever, at P=0.99 and
P=1.00.

**Shipped off by default** (`brain.perception_floor_mask`), and not because
of the false positive. It is a **third model per frame**, and 2.1's 15-30Hz
row with 2.9's budget say segmentation has no business running at the
detector's rate on the real part. Turning it on off-robot costs only time;
**deciding its schedule is C6's job**, and this measurement is what that
decision now has to work from -- +8 points of recall for one more model per
frame, or for whatever slower rate C6 chooses to run it at.

**And it makes P4 concrete.** 4.3 says to make floor segmentation the
compile loop's first subject rather than YOLO, on the grounds that
compiling something you could simply download proves less. That model now
exists, in this repo, behind a Protocol, with a number attached to what it
buys.

#### Two things measured while choosing a fourth walk -- 2026-09-07

**Precision is not the problem, and it was free to check.** Ask each walk
for the *other* walks' targets: every off-diagonal cell should be zero,
because that object is not in that room.

| walk | `"blue bottle"` | `"burgundy backpack"` | `"...running shoes"` |
|---|---|---|---|
| blue-bottle | **11/33*** | 0/33 | 0/33 |
| red-backpack | 0/38 | **23/38*** | 0/38 |
| blue-shoes | 1/19 | 0/19 | **9/19*** |

*(\* = the walk's own target, where a hit is correct.)* **One false positive
in 171 wrong-target frames.** The distractor set is doing its job, which is
a second reason the near-neighbour experiment found nothing to fix.

**And that table exposed a bug shipped the same hour.** The diagonal reads
11/33 for the bottle where the section above measured 18 of 18. The cause is
`max_crops`, which was 4 -- sized for **one** crop source. With the floor
mask unioned in, crops are ranked by **area**, and a target is usually much
smaller than the furniture beside it, so the mask's large regions crowded
the target out of the budget:

| `max_crops` | bottle detected / 18 visible | false pos |
|---|---|---|
| 4 | 11 | 0 |
| 6 | 16 | 0 |
| **8** | **18** | 0 |

The cap now scales with the number of sources (`DEFAULT_MAX_CROPS` 4,
`DEFAULT_MAX_CROPS_WITH_PROPOSER` 8) and the shipped defaults reproduce the
measured 60/64 again. **Ranking by area is the underlying weakness** and is
deliberately left alone: replacing it is a design change that wants its own
measurement, and raising the cap is the fix this data supports. Note the
shape of the failure -- adding a capability silently *reduced* accuracy
through a constant sized for the old one, and only a cross-check caught it.

#### The fourth walk: a 209-frame two-room SEARCH, and the arbitration is backwards -- 2026-09-07

`recordings/blue-bottle-20260907-185007`. **209 frames, 209/209 landscape**,
living room -> doorway -> second room, bottle on the floor under a desk.
The first walk that is a *search* rather than an approach: the target is
visible on **10 of 209 frames**, and everything before that is looking.

**The cost claim survives the case that could have broken it.** Every
saving measured before this came from a successful approach, where the
target is in view most of the time. A search is the opposite, and
`cold_search` fires on a timer, so it could plausibly have cost *more* per
step:

| | |
|---|---|
| paid calls | **35** over 209 frames -> **1 per 5.97** |
| triggers | `cold_search` 34, `mission_start` 1 |
| a per-frame policy | 209 calls |

1-per-6 sits inside the same band as the approach walks (3.5-7.2). **2.4's
economics hold on searches.**

**But read the trigger column before celebrating it.** 34 of 35 calls were
`cold_search`. The on-board tier contributed **one** trigger, at start. The
saving came from the *timer*, not from event-driven deliberation -- which is
2.5's degraded mode, and a saving achieved by idling at a fixed rate is not
the architecture working. Same shape as the poorly-described shoes walk,
reached this time by distance rather than wording.

Also, first real per-frame number: **609ms/frame** for three models on a
MacBook. Not comparable to an 8L's 33ms budget (2.9), but it is a number.

#### And then the reference disagreed with the pixels, 34 times

The VLM called the bottle visible on **44** frames. The bottle is in the
second room. All 44 were opened, in two contact sheets, and **34 of them are
a teal storage bin behind the living-room couch** -- confirmed by the owner
-- which the model described in confident, specific, entirely invented
detail: *"A blue bottle is visible under the couch in the center of the
image, but the couch is blocking direct forward movement."*

Adjudicated ground truth is now `labels.json` beside the walk. It gives:

| tier | recall | precision |
|---|---|---|
| **VLM (Opus 4.5)** | **10/10** | **10/44 (23%)** |
| **on-board (YOLO + CLIP)** | 2/10 (20%) | 2/4 (50%) |

**The two tiers fail in opposite directions, and the split is stark.** The
VLM misses nothing and invents constantly; the on-board tier finds little
and invents little. **The on-board tier rejected all 34 bin frames.**

#### Why this is an architecture finding and not a model complaint

1.11 splits arbitration by question and gives **identity to the VLM**:
*on-board proposes, the cloud confirms.* This walk is that rule's worst
case. Under it the robot would have accepted "the bottle is under the
couch, turn right" **34 times** and driven at a storage bin in the wrong
room -- and the one component that knew better, on every single frame,
has no vote on identity by design.

Three things follow, and none of them is "use a better model":

- **A confident VLM identity claim needs corroboration on a search.** Not on
  an approach, where it is right; on a search, where 77% of its claims were
  wrong. The cheapest form is the one already built: require the local tier
  to agree before a sighting is believed, and treat disagreement as
  `unclear` rather than as either answer. That is M3's tri-state argument
  one tier up, and 1.16 #11 already asks the analogous question of the floor
  mask.
- **`target_reached` inherits the problem.** Arrival is the VLM's call
  (1.8), and on this walk it fired only on the two genuinely-arrived frames
  -- but nothing in the design would have stopped it firing on frame 20.
- **The corpus's reference has to be adjudicated, always.** This is the
  second walk where `walk.jsonl` was wrong, and this time by 77% rather than
  by two frames. **`labels.json` is the reference; `walk.jsonl` records what
  a model said at the time and is evidence about the model, not about the
  room.**

The on-board tier's own 20% recall on this walk is a separate and real
problem -- the target is small and distant for most of its ten frames, which
is exactly where the floor mask does not help either -- but it is the
failure that makes a robot *slow*, not the one that makes it confidently
wrong about where it is going.

#### P3 is a tool now, and it agrees with every number above -- 2026-09-08

`control/perception_eval.py`, beside `control/walk_eval.py`, which is where
4.10 said it belonged. Everything in this section and in 4.11 had been
computed by an ad-hoc script written into a scratchpad and thrown away --
three times, on three different days -- so no figure here could be re-derived
without rewriting the instrument that produced it. That is the same defect as
quoting a walk's prompt wording from prose instead of from its own recorded
`model_id`, one level up: the measurement was reproducible in principle and
not in practice.

**What it does.** Reads each walk's adjudicated `labels.json`, runs the
pipeline over every labelled frame, and reports recall and precision per walk
and in total, at any number of gates. Three decisions in it are worth stating,
because each one is a mistake this document has already made:

- **The reference is `labels.json` and a walk without one cannot be scored at
  all.** Not a default, a refusal -- with the reason in the error message. An
  unlabelled walk scored against `walk.jsonl` produces a number that looks
  exactly like a real one, and on the search walk that number would have been
  wrong by 77%.
- **Score once, threshold afterwards.** A run records `max(candidate
  probability)` per frame, gate-free, and every table is arithmetic over
  those. Re-running three models per gate measures nothing new, and `--save`
  makes a slow config -- SAM is 14 seconds a frame -- payable once.
- **Recall is never printed without its false-positive count**, and `compare`
  matches two configs on a false-positive budget *before* reporting either
  one's recall. 4.11's own caution is the reason: a detector run at low enough
  confidence beats anything on recall while inventing targets.

**It reproduces 4.11's shipped row exactly, on the first run**, which is the
only validation a scorer can really have -- 62/74 at `P>=0.8` with 3 false
positives, 68 at 0.60 with 13, 69 at 0.50 with 16. The per-walk split it adds
is new and is worth reading, because the corpus total hides the shape
completely:

| walk | visible | detected at 0.8 | FP | recall |
|---|---|---|---|---|
| blue-bottle (approach) | 18 | 18 | 0 | **100%** |
| red-backpack | 33 | 32 | 0 | **97%** |
| blue-shoes | 13 | 10 | 1 | 77% |
| blue-bottle (**search**) | 10 | 2 | 2 | **20%** |

**The local tier is near-perfect on approaches and nearly blind on the
search**, and the corpus-wide 84% is an average over two different problems
rather than a description of either. That is the same split 1.11a and 4.11
each found from their own direction, arriving here as one table.

**And matching on false positives immediately found something a single-gate
table cannot show.** The floor mask's own result -- *"+8 points of recall for
one false positive"* -- is true at `P >= 0.8` and stops being true at zero
tolerance:

| FP budget | detector only | detector + floor mask |
|---|---|---|
| 0 | **59/74 (80%)** | **11/74 (15%)** |
| 3 | 59/74 (80%) | 63/74 (85%) |
| 16 | 62/74 (84%) | 69/74 (93%) |

**The mask's single false positive scores `P = 0.9998`** -- higher than all
but eleven of the 74 true positives -- so a threshold that excludes it
excludes almost every true positive with it. The mask is not
"better by 8 points"; it is better *given a tolerance for error at all*, and
strictly worse without one. Nothing here overturns the decision to union the
sources -- 1.11's arbitration means a false positive costs one deliberation
call the VLM then rejects, while a miss means driving past the target -- but
it does say the mask must not be described as free, and it is a second reason
C6 owns the mask's schedule rather than this section.

### 1.11a Corroborated identity -- **PROPOSED 2026-09-07, not decided**

An amendment to 1.11, written up because the fourth walk broke that
section's rule badly enough that leaving it unstated would be worse than
arguing about it. **Nothing here is implemented.** It needs a decision.

#### The rule as it stands, and what happened to it

1.11 splits arbitration by question and gives **identity to the VLM**:
*on-board proposes, the cloud confirms.* On the 209-frame search walk that
produced:

| tier | recall | precision |
|---|---|---|
| VLM (Opus 4.5) | 10/10 | **10/44 (23%)** |
| on-board (YOLO + CLIP) | 2/10 | 2/4 |

34 confident, specifically-worded claims about a teal storage bin in the
wrong room. Under 1.11 as written, the robot believes all 34 -- and the one
component that was right on every single one of them has no vote.

#### Two cheaper fixes were tried first, and both fail

**A better target string does not help.** Walk 2 and walk 3 both showed
description accuracy moving results a long way, so it was the first thing
to test. Re-asking eight of the bin frames as *"a light blue metal water
bottle, tall and cylindrical"* rather than *"Blue bottle"*: **8 of 8 still
claimed**, with the four true frames still found. The confabulation is not
a wording problem.

**Asking again from somewhere else does not help either.** The obvious
mitigation for a one-off error is a second look, and the bin claim survived
**34 frames from many angles across two thirds of a walk**. It is stable,
not transient.

#### The proposal: asymmetric corroboration, at a lower bar than detection

Three parts, and the second is the one that makes it work.

**1. The VLM's negative is trusted; only its positive needs support.** Its
recall was 10/10 here and has never missed a sighting on any walk. There is
no evidence it says "not visible" when the target is there, so nothing is
gained by second-guessing that direction and recall would be all that was
lost.

**2. Corroboration uses a LOWER threshold than detection.** This is the
part that is easy to get wrong, and the naive version -- require the local
tier to *detect*, at its shipped `P >= 0.8` -- keeps only **2 of 10** true
sightings and is unusable. But detection and corroboration are different
questions: `P >= 0.8` asks *"is this a sighting on its own evidence"*, and
corroboration asks *"is the local tier seeing anything consistent with a
claim the cloud has already made"*. The second deserves a lower bar,
because the VLM has already contributed evidence. Measured on the walk:

| bar | true sightings kept | bin claims rejected |
|---|---|---|
| 0.80 (the detection gate) | 2/10 | 34/34 |
| 0.60 | 7/10 | 34/34 |
| **0.50** | **8/10** | **34/34** |
| 0.30 | 9/10 | 33/34 |
| 0.25 | 10/10 | 31/34 |

**0.50 rejects every false claim and keeps 8 of 10 true ones.** The local
scores separate cleanly: the bin frames run 0.00-0.44 (median 0.14) and the
true frames 0.25-0.98 (median 0.69).

**3. Disagreement is `unclear`, not either answer** -- M3's tri-state
argument one tier up, and for the same reason it was right about a depth
zone: read as "absent" it discards a real sighting, read as "present" it
keeps the failure this amendment exists to stop. An `unclear` sighting may
**steer** (turning toward something costs little and resolves itself) but
may not **commit** -- not `target_reached`, not a `found` outcome, not a
sighting written to `MissionMemory`. That split matters because 1.8 makes
arrival the planner's call too, and it inherits this exposure whole.

#### What it costs, checked against the other three walks

| walk | true sightings | corroborated at 0.5 | lost |
|---|---|---|---|
| blue-bottle (approach) | 18 | 18 | **0** |
| red-backpack | 33 | 33 | **0** |
| blue-shoes | 13 | 10 | **3** |
| blue-bottle (search) | 10 | 8 | 2 |

**Free on two walks, and it costs three sightings on the shoes** -- which is
the out-of-vocabulary target the local tier is weakest on, exactly as
expected. On the search walk the two it loses are the most extreme
close-ups; note that **arrival still survives**, because of the two frames
where the VLM fired `target_reached` one corroborates at 0.62.

#### The walk replayed through the real policy, which is what 1.11a is for

`--policy tiered` over all 209 frames against the deployed `/navigate`.
**Outcome `found`, 36 paid calls over 212 steps, 1 per 5.89**, arrival at
step 211. Read as a headline it looks like a success. It is worth reading
line by line instead:

| | |
|---|---|
| perception across the whole mission | **211 `absent`, 1 `detected`** |
| `candidate_sighting` triggers | **0** |
| `cold_search` triggers | 35 |
| paid calls claiming the target | 8 -- **7 of them the storage bin** |
| actions those 7 commanded | FORWARD x6, LEFT |

**The on-board tier was silent for an entire mission.** Zero
`candidate_sighting`, one detection in 212 frames. The tiered architecture
ran as a **six-frame timer** from start to finish -- it never once did the
thing it exists to do, and the 1-per-5.89 saving is entirely the timer's.

**Seven paid calls steered the robot at the storage bin**, six of them
FORWARD. Those moves are inert in a replay because the frames come from the
walk rather than from the robot's own decisions -- **which is exactly why
the run reports success.** Closed loop, on a real robot, six FORWARDs
toward a bin in the wrong room is a collision and a mission that never
leaves the living room. The `found` at the end belongs to the person who
walked to the desk, not to the policy.

**Two things did work, and both deserve saying.** The room memory is real:
28 of the 36 calls answer "not visible", and from step 25 onward the model
reasons *"this appears to be a living room which has already been searched,
so turn right to explore new areas"* -- that is `searched_rooms` crossing
the seam and being used. And the final two calls are correct and decisive:
it finds the bottle under the desk at step 206 and calls arrival at 212.

Under 1.11a's rule, the seven bin calls read `unclear` -- local `P` on those
frames is 0.00-0.44, well under the 0.50 bar -- so they could steer but not
commit, and the six FORWARDs would not have been issued as approach moves
toward a confirmed target. The two real calls corroborate. **That is the
whole amendment, on one walk, and it is also the strongest argument that
the amendment is not free**: a policy whose local tier is silent for 212
frames gets very little from a rule that asks the local tier for a second
opinion.

#### What would falsify it, and what it still needs

**The failure mode to look for is a target the local tier cannot see at
all.** On such a walk every sighting reads `unclear`, the mission can steer
but never commit, and the robot circles its target forever -- strictly worse
than believing a VLM that is right most of the time. The shoes walk is the
near miss: 10 of 13 corroborate, and a harder target could go the other way.
**Two more searches, on targets with no COCO word, before this ships.**

Three smaller things unsettled, listed so they are not discovered later:
`unclear` needs a representation on `Perception` and in the status payload
(it currently has three states, and this is a fourth *relationship*, not a
fourth state); the counter in 6.3 should show corroborated-versus-claimed,
or the twin cannot show this working; and if the bar is a constant it wants
the same treatment `DEFAULT_MATCH_PROBABILITY` got -- measured, named, and
settable in config rather than compiled in.

**Status: proposed. Not implemented, and it should not be until the two
extra searches exist.** One walk that breaks a rule is a reason to write
this down; it is not yet a reason to change the robot's mind about what it
is looking at.

#### Reported, not enforced -- **BUILT 2026-09-08**

The status above stands unchanged: **no behaviour is different.** What was
built is the measurement, and the reason it is worth building before the
decision is that the decision needs evidence a replay cannot supply.

1.11a's own falsifier is *"a target the local tier cannot see at all"*, and
no walk in the corpus is one -- so replaying the four walks can only ever
re-derive the numbers already in this section. The next walks can answer it,
but only if the verdict is being computed while they are recorded. Hence the
split: **compute it, count it, show it, act on none of it.**

- `brain/tiered.py`'s `corroboration_for(scene, perception, bar)` is the rule
  as a pure function. It costs nothing: `_annotate()` already holds the
  `Perception` beside the cloud's answer, so this is a comparison between two
  numbers both of which were already computed.
- Four verdicts. `corroborated`, `unclear`, `no_claim` (the cloud says not
  visible -- **its negative is trusted**, part 1 of the proposal), and
  `unavailable` (the local tier could not tell, which 1.12 forbids reading as
  disagreement). Note where they live: `_tier.corroboration`, **not**
  `Perception.status`. This section's own open list asked for that and named
  the reason -- `unclear` is a *relationship between two tiers*, not a fourth
  perception state, and putting it on the tri-state would make a wedged camera
  and a disputed sighting the same kind of thing.
- `tier_corroboration_bar: 0.5` in `config/robot.yaml`, which is the treatment
  this section asked for -- *"measured, named, and settable in config rather
  than compiled in"*.
- The twin's Remote brain panel gains a fifth readout: this step's verdict
  with the local probability and the bar it was read against, the running
  **corroborated-of-claimed tally** (the other thing this section's open list
  asked for), and the words **"not enforced"** on every line.

**That last phrase is load-bearing and is pinned by a test.** A reporting-only
variant that quietly began gating would be the worst available outcome,
because the walks meant to decide the amendment would then be measuring the
decision. `tests/test_tiered.py` asserts that a frame reading `unclear`
passes `target_visible` **and** `target_reached` through untouched.

**What to read on the next walks.** The tally is the whole instrument. A
search walk whose claims come back mostly `unclear` is the storage-bin failure
being caught live; a walk on an out-of-vocabulary target whose claims come
back mostly `unclear` *and which still arrives* is the falsifier firing, and
says the amendment must not ship as written.

**Watched working end to end, 2026-09-08**, against two real uvicorns and a
free stand-in for `/navigate` -- the stand-in claims the target on every
frame, which is the storage-bin case stated as a fixture, and it costs
nothing so the check can be repeated:

```
step 0: FORWARD (ok) -- [cloud: mission_start] target center
        -- "a red backpack is visible under the couch"
perception   : absent | 3 proposal(s), best P 0.113 < 0.8
claims       : 1   corroborated 0   verdicts {'unclear': 1}
```

Three things in that trace are the whole design. The cloud is confident and
specific and wrong, in almost the wording the real model used on the bin. The
local tier scores it **0.113**, far under the 0.5 bar, so the verdict is
`unclear`. **And the robot drove FORWARD on it anyway** -- which is correct
today and is exactly what 1.11a would change. The final status carries
`corroboration: null` because the last step was a free one with no claim to
corroborate, which is why the running tally exists beside the per-step
verdict rather than instead of it.

#### The three untried models, tested -- **2026-09-08**

4.11 closed by naming its own limit: *"only one open-vocabulary model was
tested... none of those has been tried, and a Jetson is the only way to run
them."* All four are now tested, off the robot, behind the shipped
`Detector` and `RegionProposer` Protocols
(`brain/perceive_lab.py`), scored by `control/perception_eval.py` against the
same adjudicated labels. **Recall over the 74 visible frames, read at four
false-positive budgets** -- matched on false positives first, because that is
the only comparison that means anything:

| config | 0 FP | 1 FP | 3 FP | 16 FP | ms/frame |
|---|---|---|---|---|---|
| YOLO + floor + CLIP (**shipped**) | 15% | 55% | 85% | 93% | 847 |
| YOLO + CLIP (detector only) | **80%** | 80% | 80% | 84% | **214** |
| YOLO + SAM + CLIP | 47% | 85% | 85% | 95% | 9 620 |
| SAM + CLIP (no detector) | 47% | 74% | 82% | 88% | 9 535 |
| Grounding DINO | 72% | 77% | 78% | 84% | 4 802 |
| **OWLv2** | 26% | **91%** | **96%** | **100%** | 2 380 |
| YOLO-World | 51% | 51% | 51% | 85% | 196 |

Four things, and the third is the one that changes a decision.

**1. The "one model instead of three" thesis fails again, on a stronger
model.** Grounding DINO loses at every matched point -- 72/77/78/84 against
the shipped 15/55/85/93 and the detector-only 80/80/80/84. It reads **100%
recall at confidence 0.30**, which quoted alone looks decisive and costs
**205 false positives on 225 non-target frames**. That is this section's own
caution arriving as a live example rather than a warning.

**2. SAM confirms the crop-proposal diagnosis, and goes further than the floor
mask did.** 9 of the corpus's 11 errors are crop proposals and 0 are matching,
so a class-agnostic proposer is aimed at the measured weakness. It lands: 63/74
at **one** false positive where the floor mask needs three for the same recall.
And the per-walk split is the finding, because the totals hide it completely --
63 against 62 is a wash, and the composition is opposite:

| walk | floor mask | SAM | neither |
|---|---|---|---|
| blue-bottle (approach) | **18**/18 | 11/18 | **18**/18 |
| blue-bottle (**209-frame search**) | 2/10 | **8**/10 | 1/10 |
| blue-shoes (out of vocabulary) | 10/13 | **11**/13 | 7/13 |
| red-backpack (approach) | 32/33 | **33**/33 | 30/33 |

**SAM fixes the search walk** -- the one walk where the local tier was silent
for 212 frames and where 1.11a's whole problem lives -- and gives up ground
only on an approach that was already solved. **And unlike the floor mask it is
better than the detector ALONE** (82% against 76%), where the mask measured
59% against 86%. That is a stronger claim than "worse alone, better together":
SAM's proposals are simply better than YOLO's on this corpus.

**3. OWLv2 wins, decisively, and that reverses part of this section's
conclusion.** 91% at one false positive, 96% at three, **100% at sixteen** --
against a shipped pipeline that manages 55/85/93. Per walk at three false
positives: **18/18, 10/10, 13/13, 30/33**, including 13 of 13 on the
out-of-vocabulary shoes walk where the shipped pipeline gets 10. It scored
every visible frame, so its ceiling really is 74/74 rather than an artifact of
a truncated curve.

**And it is the only model that cleanly separates the storage bin**, which is
the exact failure 1.11a exists for. Scores on the search walk's 34
confabulated frames against its 10 true sightings:

| config | bin max | bin median | true min | separated? |
|---|---|---|---|---|
| **OWLv2** | **0.035** | 0.025 | **0.369** | **yes -- an order of magnitude, no overlap** |
| Grounding DINO | 0.786 | 0.446 | 0.861 | yes, but by 0.075 |
| YOLO + SAM + CLIP | 0.300 | 0.187 | 0.255 | **no -- overlaps** |
| YOLO + floor + CLIP | 0.436 | 0.144 | 0.255 | **no -- overlaps** |

Every other config's bin scores run into its true scores, so corroboration
under 1.11a has to pick a bar inside an overlap. **OWLv2 has no overlap at
all**, and would corroborate correctly at any bar between 0.04 and 0.36.

**4. And this is the first real evidence FOR a heavier part** -- which
everything else measured here argues against, so it should be stated plainly
rather than buried. This section concluded *"a Jetson buys model capacity; the
evidence says capacity is not what is short."* On the crop-proposal axis that
still holds. On **identity under adversarial conditions** it does not: OWLv2 is
a ViT-B/16 dual-tower model at high input resolution, it is not an 8L-class
part, and it is the only thing tested that gets the bin right by construction
rather than by luck.

#### The architecture that falls out of it -- **PROPOSED, not decided**

The obvious reading of the row above is "replace YOLO with OWLv2", and that is
wrong on 2.9's budget: **2 380ms a frame on a laptop** is nowhere near the
reactive tier's 15-30Hz row, and 2.1 puts the detector there for good reasons.

But **corroboration does not run at the reactive tier's rate.** 1.11a asks the
local tier a question only when the cloud has already claimed a sighting --
44 frames of 209 on the search walk, about one in five, and it is allowed to
take a deliberation-tier interval to answer. So the shape the measurement
actually suggests is **three local roles, not one model**:

| role | rate | model | why |
|---|---|---|---|
| obstacle / reactive | 15-30Hz | YOLO11s + depth | 214ms, and 2.9's budget is built around it |
| candidate proposal | every frame | YOLO, + a proposer | SAM's row above; the crop source is the measured bottleneck |
| **identity corroboration** | **only on a cloud claim** | **OWLv2-class** | the only thing that separates the bin, and it can afford a second |

**Not decided, and deliberately not built.** It needs the two
out-of-vocabulary searches below before 1.11a is settled at all, it needs
OWLv2 timed on the real part rather than on a MacBook, and P4's compile loop
has never been run on anything -- a ViT at OWLv2's input resolution is a much
harder first HEF than the floor mask 4.3 nominates. What this table does
establish is that the **corroboration role is worth a model of its own**,
which is not something this plan had considered.

#### Three cautions, so nothing above is over-read

**The timings are relative, not predictive.** 2.9 budgets against an 8L's
33ms frame and a MacBook is not one. The `ms/frame` column ranks models
against each other and says nothing about whether any of them fits.

**An internally-thresholded detector has a truncated curve.** Each
open-vocabulary model applies its own confidence floor before this harness ever
sees a score, so at a loose budget the comparison quietly favours whichever
model has the lowest floor. It bites once here: **YOLO-World produced no score
at all on 222 of 299 frames, 11 of them containing the target**, so its
ceiling is 63/74 no matter how low the gate goes. Grounding DINO and OWLv2
scored every frame and their curves are complete.

**And the corpus is four walks and 74 visible frames.** OWLv2's margin is
large and consistent across all four, but 1.16 #10's lesson is that a corpus
can be wrong in a way no amount of internal consistency reveals. The two
searches below are what this result should be re-checked against.

#### The first live tiered walks, and the defect they exposed -- 2026-09-08

Three walks for a **woven laundry basket**, 358 frames, all 1280x720, all under
`policy: "tiered"` with recording on -- which was impossible until the same day
(the twin made "Record this walk" and "Drive via brain" mutually exclusive on a
reason that expired at T4, so the one walk where YOLO and CLIP see real pixels
was the one walk it refused to keep).

**Three things worked for the first time.**

| | |
|---|---|
| `candidate_sighting` fired | **twice**, on walk 3 -- 5 detections in 38 steps. Every previous search walk fired zero |
| an **out-of-vocabulary** target was found locally | "woven laundry basket" has no COCO word; YOLO proposed it as `handbag` and CLIP scored **0.999**. 4.2's open-vocabulary crop path, working as designed |
| 1.11a **corroborated**, correctly | 2 claims, 2 corroborated, both genuine sightings at P ~ 0.998 |

Cost held at 6 calls per walk, 1 per 5.7-6.3 frames -- inside 6.1's band on a
search, with the local tier contributing triggers rather than idling.

#### ...and it nearly published the opposite finding

Read straight from `walk.jsonl`, those two corroborations sat beside frames
showing **a bare wall**. That reads as the worst possible result: both tiers
confabulating a basket at P=0.998, which would have destroyed 1.11a's premise
that the local tier is an independent check.

It was wrong, and the reason is a measurement defect rather than a model one.
**Under "Drive via brain" the recorded frame and the recorded decision are not
the same frame.** The twin pushes frames on one timer, polls `/mission/status`
on another, and the mission ticks on a third: measured at a **median of 2.5
pushed frames per mission step, range 1-10**. So the status saved beside frame
N routinely describes a decision taken seconds earlier.

Frame-exact perception settles it -- the basket really is visible where the
decision was made and really is absent where the decision was filed:

| frames | perception | P |
|---|---|---|
| 0017, 0019, 0022 | **detected** | 0.999, 0.996, 0.807 |
| 0026-0028 (*where the verdict was filed*) | absent | 0.02-0.05 |
| 0080-0083 | **detected** | 0.991-0.998 |
| 0087-0089 (*where the verdict was filed*) | absent | 0.03-0.08 |

**Fixed the same day, on both ends.** `sim/teleop_robot.py` already stamped
every pushed frame with a sequence number and echoed it to the pusher; what was
missing was carrying it through. `MissionRunner` now records the id of the
frame its last decision was taken on and publishes it as `last_frame_seq`; the
twin records the id it got back from the push as `teleop_seq`; and
`control/perception_eval.py`'s `decisions_by_frame()` joins the two. A walk
with no alignment returns **nothing** rather than the naive pairing -- which is
the whole point, because the naive pairing looks identical to a real one.

**The general lesson is the one this document keeps relearning.** A walk's own
log is evidence about a model only if you know which pixels the model saw. That
was `walk.jsonl` vs `labels.json` at the corpus level (the VLM is not ground
truth); it is now frame alignment at the row level. Both failures produce
confident, plausible, wrong numbers.

#### 1.11a measured live, with exact alignment -- and it is NET NEGATIVE on
#### these walks -- 2026-09-08

Three more laundry-basket walks, 267 frames, all 1280x720, all under
`policy: "tiered"` with recording on **and with the frame/decision alignment
built earlier the same day**. These are the first walks in the project where a
decision can be tied to the exact pixels that produced it.

**The alignment was not a theoretical worry.** Measured on these walks: the
recorded frame runs a **median of 4 frames ahead** of the decision stored
beside it, **maximum 12**. On a walking rig four frames is a different view of
a different part of the room. Every per-frame number below would have been
wrong without it, and the previous session's walks can only be scored in
aggregate for exactly this reason.

**Every cloud claim, adjudicated by eye at the frame it was actually made on:**

| walk | frame | cloud said | verdict | local P | truth |
|---|---|---|---|---|---|
| 215252 | 0012 | *"a woven laundry basket is visible in the center"* | **`unclear`** | 0.10 | **basket IS there** -- small and distant, beside the grey chair |
| 215351 | 0017 | *"the woven laundry basket is visible in the center"* | `corroborated` | 0.99 | correct, close |
| 215351 | 0076 | *"a woven basket is visible in the center"* | `corroborated` | 0.86 | correct, close |
| (11 others) | -- | *"not visible"* | `no_claim` | -- | spot-checked; correct |

**So on these three walks the amendment caught nothing and cost one true
sighting.** There were no confabulations to reject -- the VLM was right on all
14 calls -- and the one claim it did gate was correct and got suppressed.

**And the shape of the failure is the one that matters for a search.** The two
corroborated frames are close-range; the rejected one is the distant one. The
local tier's confidence tracks distance, so `unclear` will preferentially fire
on **exactly the sightings a search depends on** -- the first, far-away glimpse
of the target across a room. That is the falsifier 1.11a wrote down for itself,
arriving on the third walk of a target it was designed for.

**One more number, from the negatives.** On walk 215140 frame 0044 the VLM
correctly said "not visible" -- a living room with no basket in it -- and the
local tier scored **0.70** on some piece of furniture. That is above 1.11a's
0.50 corroboration bar. The asymmetry saved it (only a positive needs support,
so an uncontested local 0.70 does nothing) but it says the local scores are not
clean in the 0.5-0.7 band in this house, and a symmetric rule would have
invented a sighting there.

#### What this does and does not settle

**It does not overturn the search-walk result.** On `blue-bottle-...-185007`
the VLM claimed a storage bin 34 times and the local tier rejected all 34;
1.11a would have prevented six FORWARDs at the wrong object in the wrong room.
That measurement stands.

**What it settles is that the rule is not free, and the cost is not evenly
spread.** Both findings are now on the table:

| the cloud tier is... | 1.11a | why |
|---|---|---|
| confabulating (search walk, 23% precision) | **strongly positive** -- 34/34 rejected | the local tier disagrees with an invented object |
| accurate (these walks, 14/14 correct) | **negative** -- 1 true sighting suppressed, 0 errors caught | the local tier merely fails to see a distant one |

**And there is no threshold that separates those two cases**, because the
quantity that decides them is *how far away the target is*, not how confident
either tier is. Lowering the bar to admit the 0.10 frame would admit the 0.70
furniture too.

**Recommendation, and it is a change of position.** Do not ship 1.11a as a
gate on `target_visible`. Two alternatives are better supported by the same
data and neither needs a new measurement:

- **Gate the commitment, not the sighting.** 1.11a's own text already says an
  `unclear` sighting *"may steer but may not commit"*. These walks say the
  steering half is the valuable half and the suppression half is the costly
  one. Applying the rule only to `target_reached` / `found` / a `MissionMemory`
  sighting -- never to whether the robot may turn toward something -- keeps the
  storage-bin protection (six FORWARDs at a bin are steering, but arrival is a
  commitment) while costing nothing on a distant true sighting.
- **Or make it a two-frame rule.** The bin claim survived 34 frames; the
  distant basket was corroborated within a few frames once the rig closed on
  it. Requiring disagreement to *persist* before it suppresses anything
  distinguishes a stable confabulation from a target the local tier has not
  resolved yet.

**Status: still proposed, still not implemented, and now with evidence on both
sides.** The reporting-only variant is doing exactly the job it was built for.

#### And the local tier's real limit, finally isolated

The three walks split cleanly by distance, which no earlier corpus did -- one
walk never reaches the basket at all, one sees it only from across a doorway,
one starts on top of it. Scored frame-exact against adjudicated labels:

| walk | frames | visible | recall @P>=0.8 | false pos @0.8 | false pos @0.5 |
|---|---|---|---|---|---|
| 215140 -- **basket never present** | 95 | 0 | -- | 1 | **7** |
| 215252 -- **distant**, through a doorway | 86 | 18 | **1/18 = 6%** | 0 | 0 |
| 215351 -- **close**, same room | 86 | 37 | **32/37 = 86%** | 2 | 6 |
| total | 267 | 55 | 33/55 = 60% | 3 (92% precision) | 13 |

**6% against 86% is the whole story of this tier.** It is not a
model-quality problem and no threshold reaches it: the distant basket is
simply not resolvable from the crops the detector proposes at that range. It
is the same axis 4.11 identified from the other direction ("its recall is
excellent where the target is large and poor where it is small"), now measured
on one object at two distances in one house.

**Two consequences.**

**For 1.11a**: the amendment's cost is not a tuning constant, it is this curve.
Corroboration will reject a distant sighting roughly fifteen times out of
sixteen, and distant sightings are what a search consists of.

**For the part**: this is the sharpest statement yet of what on-board
perception is *for*. It works, and it works at close range -- which is where
collisions and arrival happen (4.11's own defence of the tier). It does not
work at search range, and buying a bigger accelerator does not change that,
because the limit is the crop source at distance, not the classifier. The
honest reading is that the reactive/arrival tier is real and the
search-assistance tier is not, on this hardware, at this camera height.

**And the false-positive column is the argument against lowering the gate.**
Walk 215140 contains no basket anywhere in 95 frames, and the local tier still
puts **7 frames over 0.5**. A corroboration bar there is not free precision --
it is a bar most of the room can clear.

#### P5: an open-weight VLM as the crop source -- **HARNESS BUILT 2026-09-09**

Asked directly after the basket walks: *would distilled open-weight (Chinese)
VLMs run at the edge beat the models measured here?* It is the first VLM
proposal in this document aimed at the failure that was actually measured, so
it gets a phase rather than a paragraph.

**Why this one is different from 4.11's losers.** Grounding DINO added
capacity on the axis that already worked and lost at every matched operating
point. The measured failure is elsewhere: **86% recall close against 6%
distant**, caused by the crop source proposing nothing containing a small far
object. Qwen2.5-VL and InternVL do not letterbox to a fixed 640 -- they tile at
native resolution and can emit boxes directly. That is a different mechanism
pointed at the frames that fail, which is the only reason to spend a run on it.

**But it forces the Jetson, and that is the honest cost.** The Hailo-8L is a
13 TOPS CNN part with no practical generative path, and even the 10H measured
**5.89 tok/s** on a 1.5B model (4.9). "VLM at the edge" and "Pi + Hailo-8L" are
mutually exclusive, so this experiment is the strongest remaining route to
re-opening the part decision -- which is exactly why it should be run off-robot
first.

**Do not distil.** Three reasons, none of them about model quality. The vendors
already ship the small sizes (Qwen2.5-VL-3B, InternVL3-2B, MiniCPM-V) -- that
distillation is done. Opus cannot be the teacher (closed weights), so the
ceiling is the open model's own grounding and the honest experiment is to run
that model directly. And fine-tuning on this corpus would overfit to one
basement: ~900 frames from one house is far too few, and it would look like it
worked *on this corpus*.

**What was built.** `VlmDetector` in `brain/perceive_lab.py`, behind the same
`Detector` Protocol as everything else, reachable as `--detector vlm:<model
id>`. One design decision is load-bearing:

> **The score is `P(yes)` read from the logits of the first generated token,
> not the presence of a grounding box.** A box is present or absent, which
> gives one operating point and no curve -- and a single point is how a model
> gets compared at whatever threshold happens to flatter it. Every
> matched-precision table in 4.11 needs a sweepable score, so the yes/no
> probability is the score and the grounding pass runs only to place a box.

An ungrounded answer is labelled `vlm:ungrounded` rather than given a
full-frame box quietly: a full-frame box puts the bearing dead ahead, and a
wrong bearing is worse than an absent one (1.16 #4, which cost this project a
field that was never once a number).

`--vlm-max-pixels` exposes the tiling budget, because that -- not the parameter
count -- is the variable actually under test.

#### First measurements: the yes/no score is bluffable, and tiling is non-monotonic

**Two findings within an hour of the harness existing, and the first one
changed its design.**

**1. A VLM will answer "yes" to a frame it then refuses to point at.**

| frame | P(yes) | grounding |
|---|---|---|
| basket close | 0.925 | `[918, 287, 1073, 442]` |
| **basket distant** | **0.884** | **`[]`** |
| no basket in the room | 0.097 | `[]` |

0.884 on a frame the same model declines to localise. A yes/no question can be
answered from prior -- a home gym plausibly contains a laundry basket -- and
that is the cloud tier's 23%-precision failure reproduced in a 3B model. **The
score is now `P(it localises)`**, taken at the token where the model commits to
`{` or `]`. Grounding cannot be bluffed: a box is a claim about a location.

**And that score is decisive rather than graded** -- 1.0000 or 0.0000, nothing
between. So a VLM used this way yields **one operating point, not a curve**,
and cannot be put through `recall_at_fp_budget` the way the detectors were. The
constraint is real and worth stating plainly: *a sweepable score that can be
answered from prior, or an honest one that cannot be swept.* Grounding wins;
its row in any comparison carries a footnote instead of a curve.

**2. The tiling budget is non-monotonic, and native resolution is the peak.**

| `max_pixels` | distant basket | close basket |
|---|---|---|
| model default | ungrounded | grounded |
| **921 600 = native 1280x720** | **grounded, P=0.257** | grounded |
| 3 686 400 | ungrounded | grounded |
| 8 847 360 | ungrounded | grounded |

**At exactly native resolution the distant basket becomes groundable**, and
above it fails again -- which is mechanically sensible, because `smart_resize`
upscales to fill the budget and upscaling adds patches without adding detail.
So the lever is *no resampling*, not *more pixels*, and "give it a bigger
budget" is the wrong instruction.

Weak (0.257), n=1 per cell, and the close frame degraded at that setting, so
this is a reason to run the full walk rather than a result. **The full distant
walk at native tiling is the measurement that decides it** -- 18 visible frames
against the shipped pipeline's 1/18.

#### The result: **Qwen3-VL-4B gets 91% where the shipped tier gets 4%** -- 2026-09-09

The distant walk (`...215252`, 86 frames, 23 with the basket visible), every
model at native tiling, scored on grounding:

> **CORRECTION, 2026-09-11 (P7): the two Qwen3-VL rows below were NOT at
> native tiling.** `--vlm-max-pixels` is silently ignored by Qwen3-VL --
> measured at 897 input tokens with the flag and 897 without. It works on
> Qwen2.5-VL (1224 -> 1153), which is the model the tiling sweep above
> actually ran on. The sweep's conclusion was then carried to a model where
> the control does not exist, so these two rows are real numbers at an
> unknown tiling budget.

| config | recall | false pos | s/frame (MPS laptop) |
|---|---|---|---|
| YOLO + floor + CLIP @0.8 (**shipped**) | 1/23 = **4%** | 0 | 1.2 |
| Qwen2.5-VL-3B | 5/23 = 22% | 0 | 16.1 |
| Qwen3-VL-2B | 9/23 = 39% | 2 | 8.4 |
| **Qwen3-VL-4B** | **21/23 = 91%** | **1** | 13.2 |

**A 23x improvement on the exact failure that has blocked this plan**, at 95%
precision -- and not the recall-bought-with-false-positives shape that 4.11
warns about. The 2B model shows that shape (39% for 2 FP, worse than 2.5-VL-3B
at matched precision); the 4B does not.

**One generation and 1B parameters moved 22% to 91%**, which retires the
conclusion drawn from Qwen2.5-VL two sections above. That section generalised
from a single model to the family and was wrong to.

#### And the labels were wrong in the model's favour

Qwen3-VL-4B grounded a **contiguous run** at frames 0081-0085 that the
adjudication had marked absent. Opened: the basket is plainly visible through
the doorway, and the first labelling pass simply missed the second span where
the rig turns back. `labels.json` is corrected, everything is rescored, and the
shipped baseline drops from 6% to **4%** as a result.

**A contiguous run of false positives is almost always a labelling error
rather than a model error**, and it is worth adding to the method: the earlier
walks were adjudicated frame-by-frame from contact sheets, where a *span* is
easy to lose. Reading the model's disagreements as a hypothesis about the
labels -- rather than only as the model's mistakes -- is what caught it.

#### What this does to the hardware decision -- **it re-opens it**

The position taken in 4.9 and reinforced in 4.11 was that a local VLM is *"a
2.5 fallback for when the network is gone, not a deliberation tier"*, and that
a Jetson buys capacity the evidence says is not short. **The first half of that
survives; the second does not.**

- **The 8L still owns the reactive tier.** 92 FPS, obstacles and arrival at
  close range, where YOLO measures 86%. Nothing here displaces it, and no VLM
  runs at 15-30Hz on any edge part.
- **But search proposal is not a tier the 8L can serve at all.** 4% is not a
  weak tier, it is an absent one -- and 2.4's trigger discipline has been
  running on `cold_search`'s timer for exactly this reason.
- **A 4B VLM at an estimated 3-6s/frame on an Orin Nano could serve it**, and
  an 8GB board runs 4B at INT4 comfortably. The 8L runs no VLM at any size.

**The test that could still settle it for the Pi is unrun**: a **1280 HEF**.
The shipped detector letterboxes 1280 captures down to 640, and *not
resampling* was the entire mechanism in the tiling sweep -- so the 4%
baseline is YOLO measured after throwing half the pixels away. If a 1280 HEF
takes it to 40%+, the cheap path survives; if it moves it to 10%, the Jetson
case is made on measurement rather than on the "run arbitrary models" wish.

**That is an EC2 hour against a $400 board, and it is now the highest-value
experiment left in this plan.** Do not order until it has been run.

#### OWLv2 answers it, and the answer is not a Jetson -- **2026-09-09**

The VLM thread pulled attention away from a model that had already won 4.11's
own bench, and running it across three targets settles the hardware question
more cleanly than any VLM did. **68 of 68 visible frames, three targets, three
resolutions, zero false positives:**

| walk | capture | target | OWLv2 @ 0 FP |
|---|---|---|---|
| blue-shoes-...210511 | 1280 | out of vocabulary | **25/25 = 100%** |
| red-backpack-...144856 | VGA | COCO `backpack` | **33/33 = 100%** |
| blue-bottle-...185007 | VGA | the 209-frame search | **10/10 = 100%** |

**And it beats the cloud on the walk that broke 1.11:**

| | recall | precision |
|---|---|---|
| Opus 4.5 | 10/10 | **10/44 = 23%** |
| **OWLv2** | **10/10** | **10/10 = 100%** |

The storage-bin frames top out at **0.035** against true sightings at
**0.73-0.76** -- a 20x gap with nothing in it. On the negative frames OWLv2
mostly emits **no box at all**, which is the cleanest behaviour available: it
declines rather than guessing quietly.

**At 2.1s/frame and ~150M parameters**, against 8-16s and 2-4B for the VLMs.

#### What that does to the part decision

The Jetson case rested on *"only a 2-4B VLM can do open-vocabulary search
proposal, and the 8L runs no VLM."* **A 150M ViT does it better** -- better
than the VLMs, better than the cloud, better than the shipped pipeline, on
every walk tested. So the capability that was going to justify $400 turns out
to cost 150M parameters.

**The decision now rests on one unanswered question, and it is a compile
question rather than an accuracy one:**

> **Can OWLv2 compile to a Hailo HEF?** It is a ViT-B/16 with a text tower,
> which is unusual for a toolchain built around CNNs -- though Hailo ships
> some ViT support. The text side can be precomputed on the Pi's CPU exactly
> as CLIP's already is (4.2), so only the image tower and the detection head
> have to compile.

- **Compiles** -> Pi 5 + Hailo-8L, decisively, and the perception is *better*
  than the Jetson plan would have delivered.
- **Does not** -> a Jetson, but for a 150M model rather than a 4B one, which
  changes the board and the power budget it needs.

> **MEASURED 2026-09-09: it does not.** The second branch is the live one.
> OWLv2 parses and quantizes on DFC 3.34 and then fails allocation on 73
> layernorm and 38 softmax layers -- every per-token reduction in the
> transformer. `conv1` failed first and hid this for three attempts; an exact
> factored patch embedding removed it and exposed the real wall. Full record
> in P6 and `evaluations/hailo/`.

**This retires the experiment 1.10 item 1 was going to be pointed at.** "Test
a 1280 HEF" is answered and dead: PyTorch YOLO11s at 1280 reaches 2/23 on the
distant walk against 640's 1/23, because the limit is *vocabulary* -- a COCO
detector has no class for a laundry basket at any resolution. The compile loop
is still needed, but its first subject is now **OWLv2**, not a wider YOLO and
not the floor mask.

#### P6: the compile loop, and what it decided -- **RUN 2026-09-09**

1.10 item 1 has asked for this since 2026-09-04 and it was never started:
*"Build that loop before the hardware arrives. If the loop exists on day one
the Hailo is a sandbox; if it never gets built, the Hailo is a
fixed-function part and the IMX500 was the cheaper way to get one."* It is
built now, in `tools/hailo/`, and aimed at the model the paragraph above
names rather than at a YOLO that 4.3.1 already established is simply
downloadable.

**The split, and it is the same one CLIP already uses.** OWLv2 is a ViT-B/16
image tower, a CLIP-style text tower and three small MLP heads. Only the
image side is compiled; the text tower stays on the Pi's CPU exactly as
CLIP's does (4.2) -- not for convenience, but because it runs once per
*target string* and not once per frame. The seam is one op inside
`Owlv2ClassPredictionHead`:

```
image_class_embeds = dense0(image_feats)            # image only  -> Hailo
pred_logits = image_class_embeds @ query_embeds.T   # the only text op -> CPU
pred_logits = (pred_logits + logit_shift) * logit_scale
```

So the accelerator returns five per-patch tensors and the Pi does a
`[3600, 512] x [512, Q]` matmul in microseconds.

**The split is verified against the unmodified model, on a real frame, and
this is the part that would otherwise go wrong silently.** A HEF of a subtly
wrong graph compiles perfectly and is worthless, and nothing would surface
until the part was bought. On
`woven-laundry-basket-20260908-215252/frame-0005.jpg`, all four export
variants: max |d score| **1.6e-05**, max |d box| **2.1e-04**, top-1 patch
agrees, top-50 set identical. The tolerance is on *scores*, not logits,
deliberately -- OWLv2's logits span about -39..0, so an absolute tolerance
on them is a tolerance on a number nothing downstream reads, and a 4e-4
relative wobble from fp32 reduction order in a 12-layer ViT read as a
failure while changing no decision.

**Three axes, because "it failed" is not an answer.** Hailo's own table in
1.10 lists OWLv2 under *"does not fit -- anything attention-heavy"*, so
failure is the expected outcome and a perfectly good one. But three failures
mean three different purchases, so the loop walks a matrix and records what
each cell died of:

| axis | why |
|---|---|
| opset **17** vs **14** | at >= 17 torch emits `LayerNormalization` as one op; below it the same maths decomposes. A parser that rejects the fused op may take the decomposition -- and a difference between these two is a toolchain fact, not a statement about the part |
| head **full** vs **minimal** | `minimal` moves the `ReduceL2`, the `Elu` and the box `Sigmoid` to the CPU. Those are the three ops here least likely to exist in a CNN toolchain, and they cost microseconds on `[3600, 512]` and `[3600, 1]`. A test pins that the two heads produce the same numbers, so the fallback is the same model rather than a similar one |
| calibration **normalized** vs **uint8** | the uint8 path adds a `normalization()` layer so the part takes raw camera bytes and does the mean/std itself -- the arrangement worth having on the robot |
| `--image-size` | **the most likely thing to exhaust the part.** Native 960 is 60x60 = **3600 tokens**, against ~196 for the ImageNet ViTs in Hailo's zoo, and attention is quadratic in that. 640 gives 1600. **Any non-native size changes accuracy and must be re-scored** -- P5's own tiling sweep found the peak at native and worse either side, so this is a lever with a known cost |

For the record, what the exporter emits at 960 under opset 17: 575 nodes, 23
distinct ops -- 105 `MatMul`, 27 `LayerNormalization`, 12 `Softmax`, 4 `Erf`,
and exactly **one `Conv`** (the patch embedding). That last number is the
whole risk in one figure: this is not a CNN.

**The calibration set is the corpus, and it is drawn deliberately.** 128
frames stratified across all eleven rig walks and balanced on each walk's
adjudicated `labels.json` -- never on `walk.jsonl`, the same rule P3
enforces. Target-visible frames are a small minority of the corpus (74 of
299 in the four adjudicated walks) and are exactly the frames whose
activations decide recall, so a random draw would under-represent them by
construction. Deterministic given `--seed`, because a calibration set that
changes between runs makes two compiles incomparable. Preprocessing calls
OWLv2's own processor rather than reproducing it: a hand-rolled version was
written here first and came out **2.0 off in normalised units**, most of the
input range, because the processor derives a Gaussian anti-aliasing sigma
from the scale factor before it resizes.

**What the report will decide.** Three stages fail independently, and each
points somewhere different:

| stage | a failure means |
|---|---|
| `translate` | **op coverage**, and the headline names the op. `ReduceL2` or `Elu` -> try the `minimal` head; that is a fix. `Softmax` or the attention block -> a wall |
| `optimize` | **numerics or host memory.** A `MemoryError` at 3600 tokens is the predicted outcome; try 640 and re-score |
| `compile` | **resource allocation on the part.** The graph is understood and does not fit in the 8L. The cleanest possible "buy a Jetson" |

**A HEF is not an accuracy result.** INT8 post-training quantisation changes
scores, and re-scoring a compiled OWLv2 needs real silicon -- so that is a
hardware-day item, under P5's unchanged method rules: score on grounding,
read recall only at a matched false-positive budget, and check `separable`.

**Cost, stated because 1.10 item 1 estimated it at "an EC2 hour per model"
and that is roughly right**: an `r6i.4xlarge` in us-east-2 at ~$1.01/hr plus
~$0.02/hr of gp3, torn down by `tools/hailo/ec2.sh down`. Access is SSM
Session Manager, so there is no key pair and the security group authorises
no inbound rules at all.

#### The answer: **it does not compile, and conv1 was hiding why** -- 2026-09-09

Run against **DFC 3.34.0**, `hw_arch=hailo8l`, on an `r6i.4xlarge` for 3.1
hours (**$3.13** compute + $0.07 EBS). Every record is in
`evaluations/hailo/`; the instance is gone.

| variant | translate | optimize | compile |
|---|---|---|---|
| 960px, stock graph | ok | ok | FAIL -- `conv1` |
| 640px (1600 tokens) | ok | ok | FAIL -- `conv1` |
| 640px + `allocator_param(automatic_reshapes=enabled)` | ok | ok | FAIL -- `conv1` |
| 960px, **patch conv factored** | ok | ok | **FAIL -- the whole transformer body** |

**Translation and quantization are not the problem, and that is the
surprise.** OWLv2's ViT-B/16 image tower parses in 44-107s and quantizes with
no OOM, at 3600 tokens *and* at 1600. DFC 3.34 carries a **LayerNorm
Decomposition** pass, **Matmul Equalization** and **MatmulDecompose**, and
uses all three on this graph. So 1.10's *"no efficient attention path and no
memory for the weights"* is wrong on the second half and imprecise on the
first.

**The first three rows all die on `conv1`** -- the single Conv in a 575-node
graph, the 16x16-stride-16 patch embedding:

```
Reshape is needed for layers: conv1, but adding a reshape has failed
```

Resolution does not move it and neither does the allocator's own reshape
policy, so it is neither capacity nor a missing flag. The SDK carries a
`SPACE_TO_DEPTH` conversion type, which says what the allocator is trying to
do and failing at.

**So the fourth row does that rewrite in the export, exactly**, as
`--factor-patch`: `3->48 k4s4` with one-hot weights (a space-to-depth by 4),
then `48->768 k4s4` carrying the original weights re-indexed. A 16x16 patch
is a 4x4 grid of 4x4 blocks, so the composition has the same receptive field
and the same weights. Verified against the unmodified model on a real frame
at the same bar as the text-tower split -- max |d score| **1.4e-05**, top-50
patch set identical. **An identity, not an approximation.** The form was
chosen deliberately over the other obvious one (host-side `unfold` into a
768-channel 1x1 conv) because it keeps an image-shaped input and two
small-kernel convs, which is where a CNN toolchain is comfortable.

**It works, and what it reveals is the real wall.** `conv1` disappears from
the error. In its place:

| layers named in the failure | count |
|---|---|
| layer normalization (reduce_mean / sub / square / mult) | **73** |
| softmax (reduce_max / sub / sum / mult) | **38** |
| precision change | 36 |
| matmul | 9 |
| conv | 8 |

**Every attention and every layernorm, not a subset.** The Hailo allocator
cannot place the reshapes that per-token reductions require. That is an
architectural property of the dataflow design rather than a size limit --
which is exactly why no smaller input and no flag moved it, and why `conv1`
failing first was actively misleading for three attempts.

#### So the part decision resolves, against the Hailo

P5 set the two branches. This is the second one:

> **Does not** -> a Jetson, but for a 150M model rather than a 4B one, which
> changes the board and the power budget it needs.

That is now the recommendation, and it is a **much cheaper Jetson case than
the one 4.8 costed**. The board was going to be bought to run a 2-4B VLM at
INT4 in 8GB at 2-4s per answer. It is now being bought to run a **150M ViT at
2.1s/frame**, which an Orin Nano does comfortably and which leaves the 8GB
mostly for SLAM and nav2 -- the exact objection 1.10 raised against the
Jetson (*"8GB shared with the GPU that cannot hold SLAM, nav2, a detector and
a local VLM at once"*). Dropping the local VLM removes that objection rather
than paying for it.

**Three things this does NOT change**, and they matter:

- **The reactive tier is untouched.** YOLO11 n/s/m compile for the 8L off the
  shelf (4.3.1) and measure 86% close range at 92 FPS. Nothing here was
  pointed at them. If the platform stays Pi + Hailo for the reactive tier and
  gains a second board for search proposal, that is a real option -- and a
  worse one on power and cost than one Jetson doing both.
- **OWLv2's accuracy stands.** 68/68 visible frames, three targets, zero
  false positives, and 100% precision against Opus 4.5's 23% on the walk that
  broke 1.11. Those are PyTorch numbers and remain the reason this model is
  worth a board at all.
- **The split is already built and still correct.** `owlv2_host_head.py` puts
  the text tower on the CPU and joins five per-patch tensors with one matmul.
  That division is right on a Jetson too; only the accelerator changes.

**And the loop itself was the point.** 1.10 item 1 asked for it so the part
would be a sandbox rather than a fixed function. It answered a $400 question
for $3.20 in about three hours, and it is one `ec2.sh up` away from re-running
against a newer DFC -- which is the only thing that could reverse this
result, since the limit is the allocator rather than the model.


#### P7: the whole corpus, on a rented GPU -- **RUN 2026-09-11/12**

Two A10G instances (g5.xlarge), 3.7 hours, **$3.70**, both torn down. Every
record is in `evaluations/gpu/` with its own README naming the control, the
invalid runs and the model that was deliberately not run.

**It exists because of a scope error, not a hardware need.** Every row in
4.11 and P5 above was scored on **four** walks. The corpus has **eight**
labelled walks -- 610 frames, 159 visible -- and nothing had ever been
scored against all of it. So the comparison those sections rest on mixes a
subset with a corpus, and the four excluded walks are the hard ones,
including `...215140`: **95 frames with nothing to find**, a pure
false-positive test that can never contribute recall and had never been
scored by anything.

**The control is what licenses reading any of it.** CUDA fp32 reproduced the
committed CPU record **exactly** -- 26% / 96% / 100% at gates 0.677 / 0.119
/ 0.067, same true positives, same false positives. A GPU changes nothing
about what a detection is. Without that check every row below would carry an
unfalsifiable "different hardware" caveat, which is the same discipline the
ONNX split verification in P6 exists to provide.

##### The table 4.11 should have had

Recall at matched false-positive budgets, all 8 walks, one A10G:

| config | @0 FP | @3 FP | @16 FP | ms |
|---|---|---|---|---|
| OWLv2-large | 8% | **86%** | **97%** | 717 |
| **OWLv2-base fp16** | 12% | **82%** | 92% | **111** |
| Qwen2.5-VL-3B | 42% | 79% | 79% | 725 |
| InternVL3-2B | 3% | 75% | 89% | 625 |
| YOLO11s + floor + CLIP (**shipped**) | 8% | 58% | 70% | 1210 |
| **Grounding DINO** | **50%** | 55% | 71% | 226 |
| YOLO-World | 47% | 53% | 74% | **16** |
| LLMDet | 11% | 18% | 72% | 276 |
| OmDet-Turbo | 7% | 7% | 13% | 49 |

Latency is PyTorch eager and **includes CPU preprocessing**; it compares
models to each other, never to the robot's budget (2.9).

##### Four findings, in order of how much they change

**1. fp16 costs nothing, and that assumption was load-bearing.** Identical
true positives at every budget, gates matching to three decimals, **1.8x
faster**. P5, P6 and 4.9 all assume a quantised ViT keeps its accuracy and
none of them measured it. It does. **INT8 is still unmeasured** -- ViTs
often need quantisation-aware training to hold it -- and every latency
projection below assumes INT8, so that gap is now the load-bearing one.

**2. OWLv2 reads 82%, not 96% -- and its margin WIDENS.** The gate must
climb 0.119 -> 0.209 to stay inside three false positives, costing 14
points. But the shipped pipeline falls further, 85% -> 58%, so the gap goes
from 11 points to **24** at **11x the speed**. The recommendation is
stronger than the number that was published for it.

**3. 4.11's central claim is false.** It says *"the three-model pipeline
dominates at every operating point."* At zero false positives **Grounding
DINO gets 50% where the shipped pipeline gets 8%**, beating every model in
the bench including OWLv2's 12%. That claim was measured on the subset and
does not generalise -- and it is the claim that justified keeping the
composed pipeline at all.

  The useful corollary is for **1.11a**: a model that finds half the
  sightings with *literally zero* false positives is a better corroborator
  than one finding 82% with three. 1.11a's "lower bar for corroboration" may
  be better served by a different model than by a lower threshold on the
  same one. First evidence either way, and it does not require deciding
  1.11a now.

**4. The field was not as covered as this document assumed.** 4.11 and P5
tested the models this plan happened to name. HuggingFace's own zero-shot
detection list carries three families nobody had tried. Two ran:
**OmDet-Turbo loses decisively** (7% at 3 FP, and its scores barely separate)
and **LLMDet loses** (18%). Neither displaces OWLv2 -- a real result rather
than a null one, for about forty cents. The lesson is procedural: "we tested
the alternatives" meant "we tested the alternatives we had named."

##### And the two VLM rows that were far better than their 4-walk rows

Qwen2.5-VL-3B reads **42% / 79%** and InternVL3-2B **3% / 75%**, against the
much weaker figures the distant walk alone implied. Both still lose to
OWLv2 on every engineering axis -- 6x the latency, 13-20x the parameters --
and both are **non-separable**: their gates sit pinned (1.000 for InternVL3,
0.000 for Qwen2.5) across budgets, so they offer one operating point rather
than a curve, the same shape P5 caught on Qwen3-VL-4B's 4e-07 margin.
OWLv2's gate slides 0.68 -> 0.13 over the same budgets, which is what
ranking looks like.

##### Qwen3-VL was not run, and the reason corrects P5

Three independent problems, all measured:

- **An upstream performance bug.** 5262 ms/token against Qwen2.5-VL-3B's 77
  -- 68x slower on a *smaller* model with *fewer* input tokens (897 vs
  1224) and the same `sdpa` attention. Known and unfixed upstream
  (QwenLM/Qwen3-VL#1811, sgl-project/sglang#14078); a community
  optimisation pass moved it 6%. 4.5 h per model.
- **`--vlm-max-pixels` is silently ignored by Qwen3-VL.** 897 input tokens
  at budget `None` *and* at `921600`. It works on Qwen2.5-VL (1224 ->
  1153). **So P5's "every model at native tiling" is wrong for the two
  Qwen3-VL rows** -- the flag was passed and discarded. P5's tiling sweep,
  which established that native resolution is the peak, ran on
  Qwen2.5-VL-3B (the default), and that conclusion was then carried to a
  model where the control does not exist.
- Its score is **non-separable**, so it cannot be read at a matched budget
  at all.

$9 of GPU time for two rows that are mislabelled, unreadable and losing. The
diagnosis is the more valuable output and it is recorded here instead.

##### Two runs in `evaluations/gpu/` are INVALID

Kept so nobody re-derives them, and labelled in that directory's README.
`a10g-sam-8walk.json` scored every frame exactly 0.0 -- run with `--metric
confidence` when SAM + CLIP yields a probability. `a10g-owlvit-8walk.json`
is 610/610 `unavailable` -- OWL-ViT v1 loaded through Grounding DINO's
post-processing, so the model never ran. Both are harness errors. Neither is
a statement about either model.

#### P7b: what the latency actually says about an Orin -- and what it does not

The A10G measurement was taken to make the Orin extrapolation rest on one
variable instead of three. It does, and **the answer moved by 2.4x once
assumptions were replaced by measurements** -- which is the main thing to
carry forward from it.

**Measured directly**, separating GPU from CPU rather than deriving them:

| | GPU | CPU preprocessing |
|---|---|---|
| fp32 | 123.3 ms | VGA **17.1 ms** |
| fp16 | **38.8 ms** (3.18x) | 1280 **91.5 ms** |

An earlier derivation had assumed a 2.75x fp16 speedup and solved for the
split; it was wrong by 24% on GPU and **65% on CPU at VGA**. The lesson is
small and repeats all through this document: a four-measurement system with
four unknowns is rank-deficient, and the missing input was supplied by
recall rather than by a fifth measurement that took two minutes.

**Projected to an Orin Nano Super** at the measured 20% utilisation of fp16
peak, with CPU scaled 2.5x for an A78AE:

| | GPU | CPU | total |
|---|---|---|---|
| fp16, VGA | 163 | 43 | **205 ms** (4.9 Hz) |
| fp16, 1280 | 163 | 229 | 391 ms (2.6 Hz) |
| INT8, VGA | 81 | 43 | **124 ms** (8.1 Hz) |
| INT8, 1280 | 81 | 229 | 310 ms (3.2 Hz) |

**The bottleneck is not the model.** At INT8 on a 1280 capture the Orin
spends **36 ms detecting and 229 ms resizing a photograph** -- OWLv2's own
anti-aliasing resize, in Python, on the CPU. Three fixes in order of
preference: do the resize on the GPU; capture nearer 960 so there is less
rescaling; or drop the anti-aliasing filter, which needs its accuracy cost
measured first because small distant targets are what it protects. **Do not
simply capture at VGA to dodge it** -- VGA is *upscaled* to 960 and loses
detail, where 1280 is downscaled and keeps it, and the distant-target walks
are the 1280 ones.

**What this measurement structurally cannot see** is Orin's memory
bandwidth. OWLv2's attention score matrices are **3.7 GB/frame** at 3600
tokens: 6 ms on the A10G's 600 GB/s, **37 ms on the Orin's 102 GB/s**, or
half the entire INT8 GPU estimate. The measured run used `sdpa`, so fusion
is probably already reflected in that 20% utilisation -- but that is an
inference from a stack trace. A second GPU with a different compute:bandwidth
ratio (an L4, ~$1) would resolve it; so would the board.

#### P7c: what P7 reopens in the design

Three questions the latency result changes, none of which needs hardware to
think about and all of which need hardware to settle.

**1. YOLO is NOT redundant, and an earlier claim here is withdrawn.** When
OWLv2 looked like 51 ms it appeared able to serve the 15-30 Hz reactive tier
outright, collapsing three models into one. At the corrected **124 ms / 8
Hz** it cannot. The reactive tier keeps a fast detector, and 1.10's two-layer
split stands. **What does change is the justification**: on Pi + Hailo, YOLO
was the only detector that could run at all; on a Jetson it is a deliberate
choice to spend a cheap model on the fast loop, and its 92 FPS figure is a
Hailo-8 number that does not transfer (YOLO11s on an Orin is unmeasured).

**2. The vocabulary hole in the reactive tier, and why 1.10's deleted layer
two stays deleted anyway.** 1.10 removed lidar blob tracking on the grounds
that *"tracking cannot break if detection never stopped."* That held when
the identifying detector ran at 92 FPS. It does not hold for an
out-of-vocabulary target, where **only OWLv2 can identify the target at
all** and it runs at 8 Hz -- YOLO has no COCO class for a woven laundry
basket, so it cannot supply that bearing.

  The resolution is not a tracker. **The target is static**; the only thing
  changing the bearing is the robot's own motion, which encoders and the
  motion board's 9-axis IMU already measure. So a detection becomes a
  **goal pose in the odom frame**, not a per-frame bearing: the reactive
  tier recomputes the bearing to a stored point at 30 Hz from odometry, and
  re-detection corrects drift rather than supplying the answer. This is
  1.8's *"the detector points, the lidar measures"* with a clock added, and
  it is the same rule 2.3 already applies to the cloud tier ("an egocentric
  goal, never a coordinate the reactive tier cannot resolve"), applied one
  layer down.

  At 8 Hz the gap is 125 ms -- 2.5 cm at 0.2 m/s, which odometry covers
  trivially. It also **reinforces the differential-drive choice from a new
  direction**: mecanum slips, and this design spends odometry accuracy.

**3. `target_reached` must move off the cloud.** `brain/navigate.py:169`
reads it from the `/navigate` reply and `control/mission_runner.py:585` ends
the mission on it -- so arrival is declared **~5.6 s late** (2.1 s
perception plus a 3.5 s round trip), which is 1.7 m of overshoot at 0.3 m/s
past the one thing the robot was trying to stop at. Worse,
`brain/tiered.py:475` forces `target_reached: False` on every free step, so
arrival is only *declarable* on a paid one.

  Arrival should be local and needs no YOLO: **OWLv2 supplies a bearing, the
  lidar measures range at that bearing, and a threshold fires.** Both are
  on-board and under 33 ms. This converges with 1.11a's own open
  recommendation -- gate the *commitment* rather than the sighting -- from
  an unrelated direction, which is the strongest kind of agreement.

**None of the three is built.** They are recorded here because the
measurement that provoked them is recorded here, and because two of them
contradict text elsewhere in this document.

  **Item 3 now has live evidence, and it is worse than this argued --
  see P7e (2026-09-13).** A walk physically arrived, the cloud said `STOP`
  and it was held, the local tier read P = 0.998, and the robot drove
  FORWARD into the target until the step cap. Arrival was not merely late;
  Phase G's steer-over-hold precedence **overrode** it. Fixing the latency
  alone would not have stopped that walk.


#### P7d: INT8 destroys OWLv2 -- **MEASURED 2026-09-12**

P7 left one gap and called it load-bearing: fp16 was measured and free, INT8
was assumed and never tested, and **every Orin latency figure in P7b assumed
INT8**. Measured now, on a g5.xlarge with TensorRT 10.13 (JetPack 6.x ships
10.3, same API), all 8 walks:

| budget | fp16 | **INT8** |
|---|---|---|
| @0 FP | 12% | **4%** |
| @3 FP | **82%** | **7%** |
| @16 FP | 92% | **16%** |

**82% to 7% is not degradation, it is collapse.** OWLv2 does not survive
naive post-training quantisation, which is the behaviour ViTs are known for
and the reason quantisation-aware training exists. Nothing here says a
QAT'd OWLv2 would fail; it says the free path does.

##### The first attempt produced a false positive, and the tells are worth keeping

The obvious route -- `IInt8EntropyCalibrator2`, deprecated in TRT >= 10.1 --
built without error and produced an engine whose scores were **bit-identical
to fp16 on all 610 frames**. That reads as a clean "INT8 is free" result and
is the measurement not happening: the build log carried `Missing scale and
zero-point for tensor layer_norm.bias_output, expect fall back to non-int8
implementation`, so every layer ran fp16 behind an INT8 flag.

Three checks caught it, and all three are cheap:

- **Identical to three decimals is suspicious, not clean.** Quantisation
  that changes no score changed nothing.
- **Latency did not move** (150 vs 153 ms).
- **The engine did not shrink.** INT8 weights are half the size; the "INT8"
  engine was 14KB *larger* than the fp16 one.

The real run inverts all three: 0 of 610 scores identical, mean |delta|
0.112, max 0.665, engine **98MB against fp16's 184MB**, quantised ONNX 183MB
against 365MB.

This is the same failure class as P6's `KeyError: 'USER'` -- an environment
problem arriving dressed as an answer to the question being asked. It is the
second time in this document, which makes it a pattern worth naming rather
than an anecdote.

**The supported path on TRT >= 10 is explicit quantisation**: Q/DQ nodes
inserted into the ONNX by `nvidia-modelopt` with real calibration data, then
an ordinary build. Note also that **TensorRT 11 removes implicit
quantisation entirely** -- no calibrator classes, no INT8/FP16 builder flags
-- and `pip install tensorrt` gets 11.x by default, so the calibrator recipe
now fails on two different versions for two different reasons.

##### What it does to P7b's numbers

The INT8 rows are withdrawn. **fp16 is the deployment precision by
elimination**, not by preference:

| | GPU | CPU | total | rate |
|---|---|---|---|---|
| ~~INT8, VGA~~ | ~~81~~ | ~~43~~ | ~~124 ms~~ | **withdrawn -- 7% recall** |
| **fp16, VGA** | 163 | 43 | **205 ms** | **4.9 Hz** |
| fp16, 1280 | 163 | 229 | 391 ms | 2.6 Hz |

**The Orin estimate has now moved three times -- 51 -> 124 -> 205 ms -- and
every move was an assumption being replaced by a measurement, always
downward.** That is the shape to expect when the optimistic path is the
assumed one, and it is the argument for buying the board rather than
extrapolating a fourth time.

A secondary result worth carrying: **INT8 was no faster** on the A10G (151
vs 153 ms), because latency there is dominated by CPU preprocessing rather
than the GPU. On that hardware INT8 bought nothing and cost everything.

##### What survives

The recommendation. OWLv2 at fp16 still reads **82% at 3 FP against the
shipped pipeline's 58%**, at a ninth of the latency. What weakens is the
*speed* case: 4.9 Hz is comfortably a search-proposal tier at 2.1's
0.2-1Hz row, and further than ever from a reactive tier -- so **P7c's
conclusion that YOLO stays is now strongly supported rather than
marginal.**

#### P8: the soft label gate, swept -- **2026-09-12**

Records and full tables in `evaluations/gpu/softgate/`. One A10G, 4.2 h,
**~$4.20**, torn down. The harness is new and committed --
`tools/gpu/ec2.sh` plus `tools/gpu/sweep.py`, the GPU sibling of
`tools/hailo/ec2.sh`. P7 rented two of these boxes and left no runner
behind; this is the third time that script was written and the first time
it survives.

**The question came from 4.2's own history.** The hard `label_gate` lost 11
of 18 true positives on the bottle walk because at 10cm the detector
*relabels* the target, so the default moved to open vocabulary on
2026-09-07. That fixed recall by deleting the gate, and **2.9's per-frame
budget is what now pays for it**: every surviving proposal is one more CLIP
image encode, against 3.1x headroom on an 8L. So: is there a useful point
*between* the two settings already measured?

`brain/perceive.py`'s `soft_gate` is that axis. COCO's 80 class names are
ranked against the target string in CLIP's **text** space, once at mission
start (on the robot: the Pi's CPU, beside the target encode 2.8 step 1
already pays for), and the top `k` become the accept set. k=1 is close to
the hard gate; **k=80 is the open-vocabulary path exactly**.

**Three controls hold.** `soft-k80` reproduces `shipped-lowconf` frame for
frame; `soft-k80+floor` reproduces `shipped-lowconf+floor` likewise; and
`shipped-lowconf+floor` reproduces **P7's committed published row** at
7.5% / 57.9% / 69.8% against 8% / 58% / 70%.

**The answer, in one line: alone the gate strictly costs recall; beside a
class-agnostic proposer it cuts 24-38% of the CLIP budget for accuracy that
is a wash.** Detector only, every k below 80 is worse than shipped at every
budget. With the floor mask unioned in, `soft-k1` reads 3.39 crops/frame
against 5.51 and `soft-k8` 4.18, both at 73.0% @16 FP against 69.8%. It is
worth having as an option. **It is not a new default**, and this document
should not acquire one on the strength of it.

**Why it fails alone, and this is the part that generalises.** The accept
sets were recorded with every run rather than summarised, written that way
before the run because a flat curve cannot distinguish "the gate does not
matter" from "the ordering is nonsense." For `"blue bottle"` CLIP's text
space ranks `bottle`(0.792), `bicycle`(0.736), `cup`(0.735), `apple`,
`bird`. **`vase` is 31st of 80 and `refrigerator` is 61st** -- the two
labels that caused the defect the gate was built to fix.

**The empirical table says it from the other side.** What YOLO11s actually
calls the target, on frames where it is visible: the bottle is `vase` x12
against `bottle` x5; the shoes are `bed` x5 and `couch` x5; the basket is
`handbag` x20; the backpack is `backpack` x27. **The detector's confusions
are geometric at 10cm** -- a bottle from below is a vase, a shoe on the
floor is a large flat thing, a basket is a handbag -- where **CLIP's text
space encodes semantic relatedness**. They coincide twice and miss twice.

So: **any scheme that derives an in-vocabulary noun from the target string
predicts what the target *is*, where the gate needs to predict what the
detector will *say*.** That is 4.3.1's standing-height caveat arriving a
third time, as a statement about label space rather than about mAP, and it
applies to the obvious "extract the head noun and search for that" design
as much as to this one.

**Why it works beside the mask** is mechanism rather than inference. The
mask proposes on geometry and carries no class, so it passes the gate
untouched by construction and supplies the recall the gate would otherwise
cost -- leaving the gate to suppress the detector's junk boxes. The
17th-highest non-target score falls from **0.683** (shipped) to
**0.469-0.502** (gated): a lower false-positive floor lets a lower
threshold fit the same budget. That is the same "they fail on different
frames" result already in `PerceptionPipeline`'s comment (detector 86%,
mask 59%, both 94%), seen from the precision side.

**And a methodological finding that outlasts the result.** A matched-FP
comparison at a *small* budget is hostage to single frames. At 3 FP over
610 frames the gate is set by the 4th-highest non-target score, and
`soft-k1+floor` excludes exactly **one** crop scoring 0.910 -- which moves
the gate 0.883 -> 0.743 and carries **16 true positives** with it. That one
crop is the whole of k=1's apparent +5.6 points and k=2's apparent -4.4.
P7 added `recall_at_fp_budget` because a fixed threshold discarded the best
model in the bench; this is the opposite failure and needs the opposite
guard. **Quote @16 FP**, where the gate sits in a smooth part of the
distribution, and read the per-walk table beside it -- which, at each
walk's own 0-FP point, reads 100 / 105 / 100 / 102 true positives out of
159 across the four floor configs. A wash.

**What stays open.** `max_crops` ranks crops by AREA, which
`PerceptionPipeline` already calls "the weak part"; a better ranking cuts
encodes with no gate and no recall cost. And an **empirical** confusion map
-- which labels the detector puts on a target's box at 10cm -- is the thing
CLIP text space was standing in for. The table above is that map for four
targets; whether it generalises to a target with no labelled walk is the
case a gate actually has to serve, and is untested.

**Also built alongside it, and deliberately inert**: the up-front
out-of-vocabulary verdict. 4.2's quiet failure is that `absent` from a
class-gated pipeline and `absent` from a clear room are the same string --
there is no `unknown` class and no error. `PerceptionPipeline.vocabulary`
says which at mission start instead of leaving it to be discovered when
`cold_search` happens to fire, and `brain/tiered.py` publishes it on every
`_tier` readout, free frames included. **It enforces nothing**, and a test
pins that: `oov_cold_search_after` can shorten the cold-search wait for a
target COCO has no word for, and defaults to `None`, leaving the trigger
policy bit-for-bit unchanged. Same discipline as 1.11a and for the same
reason -- **`in_vocabulary` is a measured-bad predictor of local
visibility.** `bottle` is one of COCO's 80 and the local tier lost 11 of 18
sightings on the bottle walk; the shoes walk has no COCO word at all and
detected 7 of 13.

**A run note.** The first sweep died silently about three hours in -- no
OOM, no traceback, the log stopped mid-walk and read as a slow run rather
than a dead one. `nohup ... &` under SSM RunShellScript, whose agent reaps
the document's process group when the command completes. `ec2.sh` uses
`setsid` now and `sweep.py` grew `--only`. Third time an environment
failure has arrived dressed as an answer here, after P6's `KeyError:
'USER'` and P7d's fp16-behind-an-INT8-flag.

#### Phases A-E: the deliberation call stops blocking, and the timer gets chosen -- **2026-09-12**

Provoked by one observation that turned out to be right and already half
written down: *the Pi+Hailo models never triggered a call to the cloud.*
The trigger record in `recordings/` says so exactly -- see the correction
now inline in 2.8. Five phases followed.

**A -- the deliberation call stops blocking the tick.** 2.5 has specified
this since the tiered architecture was written (*"the reactive tier always
holds a current goal; a new one arrives asynchronously and replaces it"*)
and `control/mission_runner.py:498` did the opposite, calling the policy
through `call_with_timeout`. With `vision_timeout_s` at 20.0 against a
0.25s tick, and 79% of calls fired by a counter, that is a stall roughly
every nine frames. Two halves, and they only work together: dispatch on
one worker thread, and **hold the last cloud goal while the call is out**
-- because the stand-in otherwise emits `SCAN_ACTION` on every waiting
frame, which is the degenerate RIGHT-on-every-frame mode this document has
already recorded once, reached from a different direction.

Three guards came with it, all tested by mutation. An answer landing after
the mission ends is **dropped** (the `guidanceEpoch` rule one layer down --
the same orphaned-in-flight-call class that put a dead session's decision
over a live camera view and filed one walk's frame into the next walk's
directory). A **queued** call is never paid for after the mission ends,
which is a cost guard rather than a correctness one and so needed its own
test. And an async failure is raised on the next frame, so **B3.2's
failure budget counts exactly the events it always did** -- moving the call
off the tick must not quietly disarm a failsafe.

**B -- `RobotInterface.get_odometry()`.** The second method on that
interface with an honest default, for M2's reason: most backends here
cannot measure motion. `distance_m` is **path length, not displacement** --
a robot that drives a metre out and back reports 2.0 -- because the
consumer asks *how much new ground has been covered*, and displacement
would leave a robot searching one small room permanently below any
threshold. A pivot changes heading and adds no distance, which is correct
for 1.1's differential chassis.

**The catch is structural and worth stating plainly: the only real-pixels
backend this project has is a phone on a wheeled rig, and a phone has no
encoders.** So the walks that validate perception are exactly the walks
that cannot report odometry. `TeleopRobot` and `ReplayRobot` answer
`usable: False`, the twin prints it, and Phase C falls back rather than
reading it as "has not moved".

**C -- pace the cloud by ground covered, not frame count.** Frame count is
a proxy for ground covered and the robot now has the real quantity. The
proxy breaks at both ends: a robot parked while the operator reads the log
burns calls for no new view, and one at 0.5 m/s under 1.14's continuous
motion travels a metre between looks at the same frame count. The two
rules are an **OR**, never a replacement, because a scan in place reveals
new view at zero path and distance alone would never re-trigger it.

**D -- the number, measured.** `tools/gpu/pacing_sweep.py`, all 8 labelled
walks, 610 frames, **15 visible spans**, one A10G, cloud stubbed. The
metric is not call count -- cost is not the constraint here -- it is
**look-latency**: frames from a visible span starting to the first cloud
call inside it, with spans nothing looked at counted separately because an
infinite latency has no mean. Full tables in `evaluations/gpu/pacing/`.

| config | calls | spans looked at | median lat | max lat |
|---|---|---|---|---|
| frames-6 (shipped) | 100 | 12/15 | 1.0 | 4 |
| **frames-6 async** | **94** | **14/15** | 1.0 | **2** |
| frames-2 | 266 | 13/15 | 1 | 2 |
| frames-2 async | 135 | 14/15 | 1.0 | 3 |
| cm-40 *(derived odometry)* | 89 | 11/15 | 1 | 4 |

Four findings. **Async dominates at every interval on every axis** -- at
the shipped interval it covers 14 spans of 15 against 12, with fewer calls
and half the worst-case latency; there is no operating point where
blocking is better at anything. **A shorter interval buys nothing once
async is on** -- 2, 3, 4 and 6 all read 14/15, so dropping to 2 costs 41
more calls for zero coverage, and the instinct to poll harder for a
smoother experience is simply wrong here. **Past about 9 the staleness
floor becomes the binding rule** -- `frames-9` and `frames-12` are
identical in every column, which is 6.1's finding arriving from the other
side and bounds this parameter's useful range at 2..9. And **the distance
rule does not help on this corpus, in a knowably weak way**: all three
settings cover 11 of 15 and barely differ across a 4x range, but the
odometry is **derived** from each walk's own recorded action stream at a
nominal 30cm per FORWARD, and on search walks the policy turns far more
than it drives. Unproven, not refuted; the real test needs encoders.

So: **`tier_async_cloud: true`, `tier_cold_search_after: 6` unchanged,
`tier_cold_search_after_cm: 0`** -- each written into `config/robot.yaml`
with its measurement beside it, the way `DEFAULT_MATCH_PROBABILITY` was.

**E -- 2.8 corrected**, inline where the claim lives.

**What the sweep cannot tell you**, and it matters for how hard to read
the Phase A row. The cloud is stubbed, so "looked at" is when the cloud was
**asked**, not when it answered: async does not make answers arrive
sooner. What it does is keep the robot moving on its last goal instead of
freezing, and spread the asking more evenly. The freeze it removes is
structural and follows from the call being non-blocking -- it is not
measured here. Replay is also open loop, so a different pacing cannot
change where the robot went; on a live walk it would. And this is eight
walks in one basement with 15 spans, two of which contribute none at all.

**What each phase owes the twin** (section 7), all built: an odometry line
beside the depth strip reading "no encoders on this backend" where there
are none; a **Cloud pacing** row naming the rule in force and the distance
or frames to the next look; and a **Deliberation** row distinguishing the
three states that must not look alike -- waiting while driving on a held
goal, waiting with no goal yet, and not waiting.

#### A method note worth more than the result

**OWLv2's fixed-gate table reads 3/25 on the shoes walk and its swept table
reads 25/25.** Its confidence scale simply sits low. A single-threshold
comparison would have discarded the best model in the bench -- which is
precisely why `recall_at_fp_budget` exists and why 4.11 insists every recall be
read at a matched false-positive count. The instrument earned itself here.

#### The two win conditions, and they come from the corpus

| test | current best | the VLM wins if |
|---|---|---|
| distant basket, walk 215252 | **1/18 = 6%** (YOLO + floor + CLIP) | recall meaningfully above that |
| storage-bin frames, walk 185007 | **10/44 = 23%** precision (Opus 4.5) | fewer false claims |

**Clearing both** makes it a genuine third option and justifies costing a
Jetson properly. **Clearing neither** ends the question for the price of a
rented GPU hour. **Clearing only the first** says it belongs in the *proposal*
tier and not the identity tier, which is a useful answer and a cheaper one.

**And it must beat the cheap fix, not the current state.** If a tiling encoder
recovers the distant basket, that is evidence the floor mask and lidar
clusters would too -- on a $70 part rather than a $400 one. The comparison that
decides hardware is against those, not against today's 6%.

**Two cautions.** Published benchmark scores do not transfer: a camera at
10-13cm in one basement is out of distribution for every model in this family,
and this corpus is the only eval that means anything about it. And latency on
an Orin Nano for a 2-3B model at INT4 is roughly **2-4s per answer** including
tiled prefill -- deliberation rate (2.1's 0.2-1Hz row), never the reactive
tier's 15-30Hz. It would supplement YOLO, never replace it.

#### What to record next, and why these walks -- 2026-09-08

1.11a is undecided, and the reporting-only variant above exists so the next
walks decide it. This is what those walks have to be. **Two out-of-vocabulary
searches, plus one cheap control.**

**Why out-of-vocabulary, and why searches.** 1.11a's falsifier is *"a target
the local tier cannot see at all"*, and no walk in the corpus is one. The
reason is structural rather than bad luck: `bottle` and `backpack` are both
COCO classes, so the detector was proposing boxes on the target in three of
the four walks even where it mislabelled them. Only the shoes walk is
genuinely out of vocabulary, and it corroborates 10 of 13 -- a near miss that
settles nothing in either direction. And it has to be a **search**: the
approach walks corroborate 18/18 and 33/33, so they cannot discriminate at
all. That also repairs a quieter weakness -- every statement this document
makes about search behaviour currently rests on **one walk**.

**Two targets, chosen to bracket the answer rather than sample it once.**

| | target | why this one |
|---|---|---|
| moderate | **"a woven laundry basket"** | no COCO word, but large and distinctively textured, so the floor mask has a real chance at it. If this corroborates, 1.11a survives its intended case |
| hard | **"a white phone charger cable"** | no COCO word, thin, small, low-contrast on most floors. The local tier almost certainly cannot see it -- which is the falsifier, stated as an object |

**Two nouns to avoid, checked rather than assumed.** `coco_class_for()` maps
*"a grey TV remote"* to COCO's `remote` and *"a bicycle helmet"* to
`bicycle`. Either would quietly reproduce the case the corpus already has,
and the walk would look out-of-vocabulary while not being one.

**Shape.** Second room, through a doorway, 150-250 frames, target visible on
well under 10% of them -- the 209-frame walk's shape, which is the only one
that has ever exercised `cold_search` or the arbitration.

**Drive them via the brain under `policy: "tiered"`**, not Robot view alone.
That is what makes the verdict a live measurement rather than a replay, and
the thing to read is not only the corroborated-of-claimed tally but **whether
the mission still arrives**. A walk whose claims come back mostly `unclear`
*and which still arrives* is 1.11a working; one that steers without ever
committing is 1.11a failing, in precisely the way this section predicted.

**The control, and it is the cheap part.** An *approach* on the same hard
target, 15-20 frames. Without it a failure on the hard search is confounded
between "the local tier cannot see this object" and "the local tier cannot
see this object at that distance", and those have different answers -- the
first is a vocabulary problem the crop sources own, the second is the
small-distant-target limit 4.11 already identified and which no amount of
model capacity fixes.

**And the standing items from 1.16 #10**, which cost nothing now and cannot
be repaired afterwards: lock landscape, write the rig height into the walk's
meta note, and use an **accurate** description -- the shoes walk moved recall
7x on the target string alone, and `"blue shoes"` for navy-and-lime shoes
scored like the poor description it was.

#### P10: YOLO-World COMPILES to a Hailo-8L -- **MEASURED 2026-09-13, $1.05**

1.10 item 1 has asked since 2026-09-04 for the compile loop to exist before
hardware day. P6 built it and pointed it at OWLv2, which failed. This is the
second model through it, and the one the part decision now rests on after the
Jetson was ruled out on cost the same day.

    translate   ok      4.5s
    optimize    ok     70.0s
    compile     ok    723.5s   -> yoloworld-hailo8l.hef, 25.5 MB, 4 contexts

**The reactive tier is therefore worth 72%, not 45%** (P9's numbers at the
shipped P>=0.8 gate), on a $70 part rather than a $480 one. The hardware
order is actionable on evidence.

##### The first attempt failed, and the failure named its own fix

Parsing the whole graph fails on DFC 3.34 with three `UnsupportedEinsumLayerError`
(`bchw,bkc->bkhw`) and two `UnsupportedShuffleLayerError` on the DFL box
decode. **Read the supported list in that error, because it is the result:**

    Currently supporting: ['bmchw,bnmc->bmhwn', 'bchw,cj->bjhw', ...]

`bmchw,bnmc->bmhwn` is **first on it** -- that is the vision-language path
aggregation in the neck, the text-conditioned fusion, and it is the part
one would have bet against. Hailo supports it outright.

The error also recommends the end nodes to parse to, and cutting there does
**three** things at once, which is why it is the shipped split:

1. drops the DFL decode the parser cannot build a shuffle layer for --
   the standard Hailo YOLO recipe puts that on the host anyway;
2. drops the three rejected Einsums; and
3. **un-bakes the vocabulary.**

##### Point 3 retracts a claim made earlier the same day

The naive export has one input, `images[1,3,640,640]`, and no text input:
the class embeddings are folded constants, so it looked like a HEF would be
a **chosen-vocabulary** detector frozen at compile time, which would have
been a real degradation from what P9 measured with per-walk target strings.

**That is false of the deployable split.** Those three Einsums *are* the
text contrast. With them on the CPU the accelerator runs a pure image tower
and the target string is a runtime argument again. The deployed shape is:

    Hailo   the CNN image tower (4 contexts, 25.5 MB HEF)
    Pi CPU  the text einsum, the DFL decode, NMS

which is the same split CLIP's text encoder already uses (4.2), and the
einsum is one matmul against a vector 2.8 step 1 already caches per mission.

##### Why this is not OWLv2, in one line

OWLv2 died at **allocation** -- three attempts, no valid partition, 73
layernorm and 38 softmax layers the dataflow design cannot place. YOLO-World
found a valid partition on **iteration 0** and built a HEF in 34s of kernel
compilation. The op census predicted this (0 layernorm, 1 softmax, 68 conv
against OWLv2's 73 / 38 / 1) and the census was free -- but P6 is the reason
it was tested rather than quoted, and that was the right call given the
first attempt failed.

##### What is NOT established, and the next test is a real one

**This is a compile, not an inference.** Nobody has run the HEF on silicon,
measured its latency, or checked that INT8 quantization preserved P9's
accuracy.

**That last one is not a formality: P7d measured that INT8 DESTROYS
OWLv2.** Assuming YOLO-World survives it is precisely the class of
assumption this loop exists to stop. The DFC's `optimize` stage already
produced the quantized model, so **it can be scored in emulation against the
11-walk corpus without any hardware** -- and it should be, before the part is
ordered, because a 72% that becomes 45% under INT8 changes nothing about the
purchase but everything about what the purchase buys.

Records in `build/yoloworld/` and `tools/hailo/compile_yoloworld.py`.
`ec2.sh` is now parameterised by model (`HAILO_BUILD` picks the build dir and
the sweep module), which is what a second model cost.

#### P11: the INT8 question is OPEN, not answered -- **2026-09-14**

P10 proved YOLO-World compiles. The obvious follow-up -- does INT8 keep
P9's accuracy -- was attempted on 2026-09-13/14 and **the result is
withdrawn**. What was measured is not INT8; it is quantization with every
accuracy-recovery pass disabled.

The DFC said so, in a warning that was read past:

    [warning] Reducing optimization level to 0 (the accuracy won't be
    optimized and compression won't be used) because there's less data
    than the recommended amount (1024), and there's no available GPU

    Finetune encoding skipped / Bias Correction skipped / Adaround
    skipped / Quantization-Aware Fine-Tuning skipped

**Two independent causes, both mine.** The calibration set is 128 frames
where the DFC wants **1024** -- the corpus has 1234 and they were right
there -- and an `r6i.4xlarge` has no GPU. Level 0 is the crudest
quantization the tool can produce.

For the record, and **not to be quoted as an INT8 result**:

| config (201 frames) | detections | recall @3 FP |
|---|---|---|
| fp32 ONNX | 15,334 | 42% |
| INT8, opt-level 0 | 370 | 0% |
| INT8 + `a16_w8_a16` on the embedding convs, opt-level 0 | 404 | 0% |

The precision promotion changing nothing is consistent with the same
cause: fixing three layers' precision while no error compensation runs
anywhere is not a fix.

**The corrected experiment**, which has not been run: calibration on
**1024+ frames** from the 11-walk corpus, on a **GPU instance** so the
optimization passes execute -- P7d already used a `g5.xlarge` for the
TensorRT work, so the precedent and the instance type exist. ~$1.50.

**Until then the only defensible statement is that the INT8 question is
open.** P7d's "INT8 destroys OWLv2" is a separate, properly-run result
and is unaffected by this.

##### Four failed attempts before that, kept because each misdirects

Reaching even the invalid number took four tries, and the SDK's error
messages pointed away from the cause every time:

1. promote every 512-channel layer -> 16 of them -> `Layer conv22 not
   found in model`, because the optimizer **fuses internal tensors away**;
2. promote the `output_layer*` markers -> `Unsupported value`, because a
   pass-through marker has no weights for a `w8` to describe;
3. promote the convs that feed them -> **the same** `Unsupported value`,
   now pointing at a layer that was perfectly valid;
4. enumerate `PrecisionMode` -> the names have **three** parts,
   `a<in>_w<weights>_a<out>`. `a16_w8` never existed.

**Step 3 to 4 is the lesson: the DFC reports a bad MODE as a problem with
the LAYER.** Three rounds went after layer selection while the mode string
was wrong throughout. Enumerate the enum first; it costs one call.

The accepted set on 3.34.0 is `native`, `a8_w8_a8`, `a8_w8_a16`,
`a8_w4_a8`, `a8_w4_a16`, `a16_w16_a8`, `a16_w16_a16`, `a16_w8_a8`,
`a16_w8_a16`, `a16_w4_a8`, `a16_w4_a16`. All INT -- **the 8L has no
floating-point units at all**, which is why there is no fp16 option and
why precision is a compile-time decision baked into the HEF rather than a
runtime dtype.

##### One process note, because it cost two runs

Both inference runs died silently at **exactly frame 201 of 365**, no
traceback, no OOM, 115GB free. Deterministic frame count means a timer,
almost certainly SSM reaping the process group ~8 minutes in. `nohup` is
not enough; `setsid` or a systemd unit is. The runs produced data at all
only because the tool writes incrementally -- the same habit
`tools/hailo/label_prepass.py`'s sibling learned the same week.

#### P12: INT8 degrades YOLO-World's embedding path, and the rig is now PROVEN -- **2026-09-14**

P11 withdrew an INT8 result because the DFC had silently dropped to
optimization level 0. Re-run with the two causes fixed -- **1024
calibration frames** (from 128) and a **forced `optimization_level=1`**,
so Bias Correction actually executes -- plus three controls that P11
lacked. The finding survives, and now it is properly attributed.

##### The controls, which are the reason to believe any of this

| control | result |
|---|---|
| body ONNX + head ONNX vs the full model | **max \|diff\| 0.0** |
| Hailo **native** emulation vs onnxruntime, pre-normalised input | **corr +1.0000**, identical means |
| quantized fed **uint8 0-255** vs fed 0-1 | **0.67-0.82** vs 0.54-0.70 |

The second is the one that matters: native emulation reproduces
onnxruntime **exactly**, so the split, the extracted head, the NHWC->NCHW
transpose and the output ordering are all correct. Any difference under
quantization is quantization.

The third settles the input scale, which was a live doubt: `normalization()`
from the model script **is** applied in `SDK_QUANTIZED` and is **not**
applied in `SDK_NATIVE` -- feeding native raw uint8 gives corr 0.13-0.35
and 30x magnitudes, which is what made the first control look like a
wiring bug. Feed native 0-1 and quantized 0-255.

##### The result

Quantized activations against the fp32 reference, per output tensor:

    cv2 (box branch)        +0.69  +0.74  +0.82
    cv3 (embedding branch)  +0.67  +0.71  +0.74

For an INT8 CNN one expects >0.95. End to end on 365 frames, scoring the
detector's own class score against the adjudicated labels:

| config | detections | recall @0/3/16 FP |
|---|---|---|
| fp32 ONNX | 29,331 | 14% / 34% / 38% |
| INT8, level 0 | 370 | 0% / 0% / 0% |
| INT8, level 0, `a16_w8_a16` on the embedding convs | 404 | 0% / 0% / 0% |
| **INT8, level 1 (Bias Correction)** | **83** | **0% / 0% / 0%** |

**Bias Correction made it worse**, which is not what that pass is for and
is not explained here. Note the DFC also prints *"Reducing compression
level to 0 because requested optimization level equal or less than 1"*,
so level 1 is not simply level 0 plus an accuracy pass.

##### What this does and does not establish

**Establishes:** on a Hailo-8L, default INT8 quantization destroys
YOLO-World's usable detection output, and neither 16-bit promotion of the
embedding convs nor level-1 Bias Correction recovers it. The measurement
rig is verified exact, so this is the model and the tool, not the harness.

**Does not establish:** that the model cannot be made to work. **Levels 2+
are untested and need a GPU** -- AdaRound and Quantization-Aware
Fine-Tuning are training loops, and QAT in particular is the standard
answer for a network that loses accuracy at INT8. P7d's note that ViTs
"often need quantisation-aware training" applies to an embedding head for
the same reason: the head is a **cosine similarity**, so it depends on the
DIRECTION of a 512-d vector, and direction is what a per-tensor scale
scrambles. That is also why promoting three convs did nothing -- the
damage accumulates along the whole embedding path, not at its last layer.

**So the reactive tier is still worth 45% or 72%, and the deciding test is
now a `g5.xlarge` at `optimization_level=2` or higher.** ~$1.50.

##### Process notes

Both earlier runs died silently at **exactly frame 201 of 365**. It was
SSM reaping the process group; `setsid` fixed it and the run completed all
365. `nohup` alone is not enough on an SSM-launched job.

Editing `tools/hailo/ec2.sh` while a background invocation of it was
running corrupted that run -- bash re-reads a script as it executes.

#### P13: INT8 does not preserve YOLO-World, at any optimization level -- **MEASURED 2026-09-14**

P12 found INT8 damaging and named levels 2+ as the untested escape.
They are tested now, on an A10G, and they do not rescue it. **The INT8
evaluation is complete.**

365 frames, the detector's own class score against the adjudicated labels:

| config | detections | @0 FP | @3 FP | @16 FP |
|---|---|---|---|---|
| **fp32 ONNX** | 29,331 | **14%** | **34%** | **38%** |
| INT8 level 0 | 370 | 0% | 0% | 0% |
| INT8 L0 + `a16_w8_a16` embeddings | 404 | 0% | 0% | 0% |
| INT8 L1 (Bias Correction) | 83 | 0% | 0% | 0% |
| INT8 L2 (QAT) | 4,580 | 1% | 2% | 2% |
| **INT8 L2 (QAT) + `a16_w8_a16`** | 4,577 | **3%** | **5%** | **5%** |

**Both mitigations work and they compose** -- QAT is worth 55x the
detections over level 1, and promoting the embedding convs roughly
doubles recall on top of it. **The ceiling is still 5% against 34%.**

##### Why this one is believable where P11's was not

Three controls, all run before the result was read:

* body ONNX + head ONNX vs the full model: **max \|diff\| 0.0**
* Hailo **native** emulation vs onnxruntime: **corr +1.0000**, identical means
* quantized fed uint8 vs fed 0-1: **0.67-0.82 vs 0.54-0.70**, settling the
  input scale empirically rather than by assumption

The second is the load-bearing one: native emulation reproduces
onnxruntime exactly, so the split, the extracted head, the NHWC->NCHW
transpose and the output ordering are all correct, and any difference
under quantization is quantization.

Two traps worth keeping, both of which silently produce a wrong answer:

1. **`normalization()` from the model script is applied in
   `SDK_QUANTIZED` and NOT in `SDK_NATIVE`.** Feed native 0-1 and
   quantized 0-255. Getting this backwards gives corr 0.13-0.35 and 30x
   magnitudes, which reads exactly like a wiring bug.
2. **The DFC's TensorFlow needs its own CUDA wheels.** A Deep Learning AMI
   supplies the driver, but `tf.config.list_physical_devices("GPU")`
   returned `[]` until `pip install "tensorflow[and-cuda]==2.18.0"`.
   Without that it falls back to CPU and **skips the passes again** --
   which is precisely the failure P11 was.

Also: at level 2 the log says "Bias Correction skipped" and "Adaround
skipped" **because QAT supersedes them**, not because anything went
wrong. Level 1 is not level 0 plus an accuracy pass -- it also prints
"Reducing compression level to 0", which is why its 83 detections are
*worse* than level 0's 370.

##### What it does to the part decision

**YOLO-World compiles to a Hailo-8L (P10) but does not survive its
quantization.** So the reactive tier on that part is **not P9's 72%**.
What it actually is depends on a question this evaluation raises and does
not answer:

> **Does YOLO11s + CLIP survive INT8?** It is the shipped pipeline, it is
> a Hailo-native model with published HEFs, and 4.9 lists the family under
> "compiles well". Its fp32 number is 45%. If it quantizes cleanly the
> tier is 45%; if it degrades like YOLO-World the on-board tier is worth
> very little on this part and the whole tiered argument weakens.

**That is now the cheapest decisive test left**, and it is the one to run
before ordering. The rig, the corpus, the head-split method and the
scoring are all built.

**And it strengthens the 10H case.** The failure here is a **cosine
similarity over a 512-d embedding** meeting a per-tensor INT8 scale --
direction is what quantization scrambles, which is also why promoting
three convs helped only at the margin. The 10H has on-module memory and a
transformer-oriented design, and 16-bit is cheaper there. It remains
blocked on the gated DFC v5.x download.

Records in `build/yoloworld/`, tools in `tools/hailo/`. Total spend across
P10-P13: **~$8.50**.

#### P14: the 10H is blocked on its COMPILER, not its silicon -- **OVERTURNED 2026-09-15 by P15**

> **Every conclusion in this section is wrong, and the section is kept
> only for the shape of the mistake.** P15 ran the same DFC version
> (5.4.0) inside Hailo's own AI Software Suite container and parsed
> yolov11s, yolov8s and yolo_world_v2s on `hailo10h` -- and parsed OUR
> exports too, including the exact pre-cut graph this section failed on.
> The compiler was never the problem. **The environment we assembled
> around the wheel was.** Read P15 instead; what follows is the record of
> how a bad installation was written up as a property of a part.

The 10H was costed in "The Hailo-10H option" above as $60 for the model
the 8L cannot host. Tested now, with DFC 5.4.0 (the 8L's 3.34.0 rejects
`hailo10h` outright -- the two lines target **disjoint** hardware).

| model | DFC 5.4.0 -> hailo10h |
|---|---|
| trivial 4-node conv | **OK** (and OK on `hailo15h`) |
| YOLO11s | `ValueError: channels is not in list` |
| YOLO-World | `NetworkXUnfeasible: Graph contains a cycle` |
| OWLv2 (CLS-folded) | `NetworkXUnfeasible: Graph contains a cycle` |

**The compiler runs; it cannot parse real models.** The CLI gives the
diagnosis the Python API hides:

    onnx_graph.py:6090, in is_null_split
        axis = self.input_format.index(Dims.CHANNELS)
    ValueError: channels is not in list

That is a crash handling a **`Split`** node, which every YOLOv8/11 C2f
block contains. A parser bug on one of the most standard detector
families there is -- not a property of our exports, and not something a
model change fixes.

Corroborating, from Hailo's own v5.1.0 Overview page: its architecture
diagram is labelled **"Future support (for Hailo-10H)"**.

##### What was ruled out first, so it is not re-run

Four rounds went into blaming the environment before the trivial-model
control settled it:

* **the model** -- YOLO-World compiles fine on 3.34.0 (P10);
* **the arch** -- `hailo10h`, `hailo15h`, `hailo15l` fail identically;
* **networkx** -- 2.8.8 and 3.4.2 both;
* **system packages** -- all four Hailo lists (`python3.X-dev`,
  `python3-tk`, `graphviz`, `libgraphviz-dev`) present, `pygraphviz`
  built. **These were genuinely missing at first and are a real gap**
  (`setup_host.sh` now installs them) **but they are NOT the cause**;
* **Python version** -- 3.9 is *impossible*: 5.4.0 requires
  `torch==2.9.1`, which has no 3.9 build. The v5.1/5.2 docs saying
  3.8/3.9/3.10 are stale for 5.4.0; **3.10 is the only option.**

**The control that settled it** was a 4-node conv ONNX. It parses. Two
conclusions that were written down before it -- "the missing packages are
the root cause" and "the vendor's distribution is broken, this is
blocked" -- were both wrong, and both would have stood without it. A
trivial-input control costs minutes and is worth running FIRST next time.

##### What it does to the decision

**The 10H is not a $60 upgrade that can be acted on today.** Its
toolchain cannot compile the models it would be bought for. The 8L with
DFC 3.34.0 compiles everything tried and delivers 45% (YOLO11s + CLIP, 93%
box retention at the pipeline's own confidence). **Order the 8L.**

**Tested against DFC 5.2.0 as well (2026-09-15), and it is NOT a
regression.** Identical four outcomes on both releases -- tiny OK,
yolo11s `ValueError`, yoloworld and owlv2 `NetworkXUnfeasible`. Two
independent releases failing the same four ways makes this a property of
the 5.x line as distributed, not a bug in one build. `ec2.sh` gained
`HAILO_DFC` to pin an exact wheel, because 5.4.0 must stay in the bucket
for P13's results to remain reproducible.

**A third undocumented prerequisite turned up doing it:** 5.2.0 imports
`pkg_resources` and fails without `setuptools`, which neither the wheel
nor the System Requirements page mentions -- alongside `python3-tk` and
`libgraphviz-dev`. Three gaps between what the docs list and what the
toolchain needs is itself a signal about its maturity.

One thing still keeps the 10H alive as a later option, and it is not
blocking:

1. ~~An older DFC 5.x~~ -- **tried, same result.**
2. **A Hailo ticket** with the `is_null_split` traceback against a stock
   YOLO11s ONNX -- a two-line reproducer on their own model family,
   which is a far stronger report than anything about our exports.

`build/owlv2/owlv2_op17_minimal_clsfold.onnx` is banked either way: the
CLS-token `Expand` folded to a constant, verified an exact identity
(`max |diff| 0.0` on all five outputs), which removes the one OWLv2 export
blocker found before the compiler's own trouble eclipsed it.

#### P15: the 10H was never blocked -- P14 measured our own installation -- **2026-09-15, $1.30**

P14 concluded, the same day, that DFC 5.x "runs; it cannot parse real
models" and that the 10H was therefore not purchasable. That conclusion
was challenged rather than accepted, and it does not survive.

**Three free findings came first, before any spend.** Hailo's download
page tags 5.1.0, 5.2.0 and 5.3.0 each with Hailo-10H / 15H / 15L, which
contradicts the stale "Future support (for Hailo-10H)" diagram P14 leaned
on as corroboration. The 5.4.0 Model Zoo declares `supported_hw_arch:
[hailo15h, hailo15l, hailo10h]` on yolov11s and on **224 of its 233
networks**. And the zoo's `parser.nodes` for yolov11s names exactly the
six Conv end nodes we had been cutting at -- so our cut POINT was right,
which narrowed the question rather than settling it.

##### The matrix

`hailomz parse` inside Hailo's AI Software Suite container
(`hailo_ai_sw_suite_2026-08`), which carries **DFC 5.4.0 -- the same
version P14 tested**. Parse, not compile, because parse is where P14 died
and it needs no calibration set and no GPU, which is what made the whole
question answerable for a dollar.

| row | model | arch | ONNX | result |
|---|---|---|---|---|
| **A1** | yolov11s | hailo10h | Hailo's | **OK** |
| A2 | yolov11s | hailo15h | Hailo's | OK |
| A3 | yolov8s | hailo10h | Hailo's | OK |
| A4 | yolo_world_v2s | hailo10h | Hailo's | OK |
| **B1** | yolov11s | hailo10h | **ours, opset 13** | **OK** |
| C1 | yolov11s | hailo8l | Hailo's | FAIL -- argparse, see below |

Then row D, which is the load-bearing one: **P14's exact input** -- our
hand-cut `yolo11s-body.onnx` through the raw `ClientRunner API` it used,
not through `hailomz` -- run inside the container. **D1 OK. D2 OK**
(`tools/hailo/zoo_rawparse.py`).

##### What that eliminates

Six candidates, all of them named in P14 or raised while re-opening it,
all dead: the compiler, the 10H, the model family, YOLO11's C2PSA
attention `Split`, our opset-13 export, and our end-node cut. The DFC
version is identical on both sides. **The only variable left is the
environment we built around the wheel**, which makes P14 a measurement of
an installation reported as a property of a part -- the same failure P11
was, where a silently-skipped optimization level was written up as INT8.

`hailo10h` is `parse`'s **default** `--hw-arch`. `hailo8l` is not a valid
choice for the 5.x line at all, which is why C1 fails at argparse: it is
the harness control proving the matrix can report a failure, and it is
NOT evidence about the compiler.

##### What is NOT established, and was tested rather than assumed

**The precise root cause inside our environment is unknown.** The DFC
logs a "parsing retry attempt" via ONNX simplification, so the obvious
theory was that `setup_host.sh` never installed a simplifier. **Tested,
and false**: `onnxsim` is a hard import of `hailo_sdk_common`, so P14's
host had it or nothing would have imported. Rather than name a third
guess, the container's exact pins are banked for a future diff in
`evaluations/hailo/zoo-probe/container_versions.txt` (onnx 1.17.0,
onnxruntime 1.18.0, numpy 1.26.4, protobuf 3.20.3, networkx 2.8.8, torch
2.9.1, Python 3.10.12). The practical answer does not need it: **use the
vendor container**, which `tools/hailo/zoo_probe.sh` now does.

##### What it does, and does not do, to the decision

**Does:** the 10H is no longer blocked, and P14's "order the 8L, the 10H
is not actionable" no longer follows from anything measured.

**Does not:** parse is not compile, and compile is not quantization.
**P12 and P13 stand untouched** -- they were measured on a rig verified
exact (Hailo native emulation reproducing onnxruntime at corr +1.0000)
and are independent of all of this. YOLO-World's INT8 collapse is a
cosine similarity over a 512-d embedding meeting a per-tensor scale, and
nothing here says the 10H handles that better. On-module memory and
cheaper 16-bit are an argument, not a measurement.

**So the open question is now worth paying for: does YOLO-World compile
to a 10H HEF and survive quantization there?** If yes the reactive tier is
72% on a ~$130 part rather than 45% on a ~$70 one, and that is the whole
hardware decision.

##### Process notes

Two harness bugs of the reviewer's own, both caught by the C1 control or
by reading output rather than by luck. A heredoc nested inside a
double-quoted SSM payload had its array expansion eaten by the outer
shell, so the first matrix ran one row and reported a FAIL that meant
only that its output directory did not exist -- **a harness failure
wearing a result's clothes, which is exactly the P11 hazard**; the matrix
is now its own uploaded file (`tools/hailo/zoo_matrix.sh`). And the
container runs as `uid=10642(hailo)`, not root, so a root-owned bind
mount silently blocked every write.

Records in `evaluations/hailo/zoo-probe/`. Tools:
`tools/hailo/zoo_probe.sh`, `zoo_matrix.sh`, `zoo_rawparse.py`. The suite
image and the model-zoo wheel are in `s3://vision-picar-deploy-.../hailo/suite/`.

#### P16: the 10H preserves YOLO-World -- and the floor mask makes it not matter -- **2026-09-15/16, $5.80**

P15 removed the reason not to buy a 10H. This asked what it is worth, and
answered a different question than the one it set out to.

##### Phase 1 -- allocation

Allocation is the stage that kills models on this family: P6 watched
OWLv2 translate AND quantize on an 8L and then die there, on 73 layernorm
and 38 softmax layers. So a HEF was the go/no-go.

**YOLO-World compiles to a Hailo-10H**: 11.9 MB, 5 contexts, translate
5.4s / optimize 62.2s / compile 844s. Same `compile_yoloworld.py`, same
`yoloworld-vocab32.onnx`, same six Conv end nodes, same calibration array
as P10's 8L run -- only `--arch` differs, so the 25.5 MB / 4-context 8L
HEF is a like-for-like comparison.

##### Phase 2 -- the detector, quantized

A10G box, and **TensorFlow saw the GPU inside the vendor container**,
which is the gate that makes the number readable at all: on the CPU box
the same toolchain printed P11's warning verbatim (`Reducing optimization
level to 0 ... no available GPU`) and skipped every accuracy pass.

365 frames, all 11 labelled walks, scored against `labels.json`:

| config | @0 FP | @3 FP | @16 FP |
|---|---|---|---|
| fp32 | 14% | **34%** | 38% |
| **10H, QAT** | **18%** | **22%** | **22%** |
| 8L, QAT (P13) | 1% | 2% | 2% |
| 8L, QAT + `a16_w8_a16` (P13's best) | 3% | 5% | 5% |

Proposal agreement against the same fp32 run:

| confidence | 10H | 8L + a16 |
|---|---|---|
| 0.50 | **99%** | 69% |
| 0.25 | **94%** | 52% |
| 0.05 | **61%** | 23% |

**The 10H retains 65% of fp32's recall where the 8L retained 15%**, and is
near-lossless on confident boxes. P13's mechanism explains it: the head is
a cosine similarity over a 512-d embedding, direction is what a per-tensor
INT8 scale scrambles, and this part does not scramble it.

`a16_w8_a16` could not be tested on the 10H -- it crashes **inside the
DFC**, in `hailo_conv_a16_mercury.py` -> `a_b_factorize` ->
`ValueError: arange: cannot compute length`. Unlike P14, that is the
vendor's own container; and unlike P14 it should be confirmed against the
arch's supported mode list before anyone calls it a vendor bug, because
P13's lesson is that the DFC reports a bad precision MODE as a problem
with the LAYER.

##### Phase 3 -- the TIER, and the finding that outranks the rest

A detector number cannot say what the tier is worth: the tier is detector
-> crops -> CLIP -> match, and CLIP runs fp32 on the Pi either way.
Multiplying a retention percentage by P9's 72% would produce a figure that
looks measured and is not. So the recorded boxes were replayed through the
real pipeline (`brain/perceive_lab.py`'s `ReplayDetector`, a
`replay:<json>` detector spec) and scored by the real
`control/perception_eval.py`.

**YOLO-World + floor mask + CLIP, 195 visible frames, `P>=0.8`, 48 crops:**

| config | TP | recall | precision |
|---|---|---|---|
| fp32 | 98 | **50%** | 99% |
| 10H QAT | 96 | **49%** | 99% |
| 8L QAT + a16 | 95 | **49%** | 99% |

Three detections separate them. **The same three runs with the floor mask
OFF:**

| config | TP | recall |
|---|---|---|
| fp32 | 39 | **20%** |
| 10H QAT | 26 | **13%** |
| 8L QAT + a16 | 5 | **3%** |

Without the mask the ordering reproduces the detector metrics exactly and
the 8L collapses to 3%. With it, a wrecked detector and an intact one
score the same.

**So the floor mask is not an accessory to the detector. It is the load
-bearing crop source, and it fully rescues a destroyed one.** Which means
the 45%-vs-72% gap that drove the entire hardware decision is, in the
shipped configuration, worth about one point of recall -- and the part
question is much less interesting than it looked this morning.

##### The crop cap is load-bearing, and it is not monotonic

At the shipped `max_crops` (8 with a proposer) the SAME data gives the
8L **30%** against fp32's **25%** -- the wrecked detector WINS. Detection
volume is 80.4 boxes/frame fp32, 35.8 on the 10H, 12.5 on the 8L, crops
are ranked by AREA, and a target is usually smaller than the furniture
beside it. So more proposals crowd the target out of a fixed cap.

`perceive.py` already calls area-ranking "the weak part", but the
measurement behind that note varied the number of crop SOURCES (4 vs 8),
not the proposal VOLUME. **A detector that gets better can make the
shipped tier worse.** That is a defect in the tier, it is independent of
any accelerator, and it is a candidate explanation for some of P3's
per-walk variance.

##### What this makes the next question

Not "8L or 10H". **Does SegFormer-B0 survive INT8?** The mask is now the
component the tier rests on, it has never been quantized, and if it
degrades the way YOLO-World does on an 8L then these 49% rows are fp32
numbers that no part can reproduce. It is also a third model per frame
against the Pi's four cores, which is handoff open item 1 -- uncosted, and
now first-order rather than a scheduling detail.

##### Caveats, and three tables that are VOID

These are 365 frames of 11 walks at one operating point, with the mask
running fp32 on a laptop. Three earlier tier tables from this same session
are wrong and are recorded here only so they are not quoted: a scratch
harness that read 4%/8% (a `getattr` default silently zeroed every true
positive, and a class filter pre-empted CLIP); a 15%/14%/18% table (869 of
1234 frames uncovered by the detections file and counted as misses); and
the 25%/23%/30% table above, which is the crop-cap confound. Each was
caught by an fp32 control, which is the argument for always running one.

Records in `evaluations/hailo/zoo-probe/`.

#### P18: the floor mask runs on BOTH parts, and INT8 barely touches it -- **2026-09-17, ~$14**

P16 left one question able to overturn it. The tier reads 49% with the
floor mask and 3% without it on a quantized 8L detector, so the mask is
what the tier rests on -- and the mask had never been compiled, let alone
quantized. If it degraded the way YOLO-World does on an 8L, those 49%
rows were fp32 numbers no part could reproduce.

They are not.

| | translate | optimize | compile | HEF |
|---|---|---|---|---|
| **hailo10h** (DFC 5.4.0) | ok 4.0s | ok 414s | **ok 678s** | **6.4 MB, 13 contexts** |
| **hailo8l** (DFC 3.34.0) | ok 3.6s | ok 466s | **ok 545s** | **20.0 MB** |

And INT8 preserves the mask, measured as floor-mask IoU against the same
model in fp32 over 120 corpus frames:

| | |
|---|---|
| mean IoU | **0.988** |
| median IoU | **0.995** |
| frames below 0.5 IoU | **0** |
| frames below 0.8 IoU | 1 of 120 |
| mean floor pixels | 8358 fp32 vs 8356 INT8 |

**That is at optimization level 0** -- Bias Correction, AdaRound, QAT and
Layer Noise Analysis all explicitly skipped, because the GPU box is capped
at 8 vCPU on this account and that size OOM-killed the run. So it is a
LOWER BOUND, and a lower bound is the one direction in which good news
needs no further spending: the accuracy passes can only improve it. The
GPU row was therefore not run, deliberately, rather than left undone.

##### Why IoU against fp32, and not against labels

The same argument `proposal_agreement.py` makes. The mask's job in the
tier is to propose regions for CLIP; ADE20K's own floor classes are not
this corpus's ground truth; and what quantization can break is agreement
with the model P16 measured the 50% with. A per-frame IoU also degrades
gracefully -- it says how MUCH was lost, where a recall number over a gate
would only say whether a threshold moved.

##### Two export fixes, both exact, and only the second was the real one

The Hailo parser rejected the model outright:

    UnsupportedShuffleLayerError in op node_Reshape_581
    UnsupportedShuffleLayerError in op node_Reshape_654

1. **Decode-head `Linear` -> 1x1 `Conv`.** `SegformerMLP.forward` is
   `flatten(2).transpose(1,2)` -> `Linear`, and the head transposes and
   reshapes back. A Linear applied at every spatial position IS a 1x1 Conv
   over the feature map. Verified at max |diff| 7.6e-06. **This was not
   the fix** -- the named nodes survived it, which a check of the exported
   file showed and which I had asserted otherwise.
2. **`attn_implementation="eager"`.** The offending reshapes were in the
   ENCODER's attention: `[8, 32, 256] -> [1, 8, 32, 256]`, the batch
   dimension folded into the heads by SDPA's export and then restored.
   Eager keeps it explicit and the nodes never appear.

Both are kept: (1) is a real simplification and costs nothing, (2) is what
made it parse. The exporter raises rather than writing anything if the
identity check fails, because a graph that compiles and computes
something else is the worst outcome available here.

##### What it does to the part decision

**It does not make the 10H mandatory.** That was the live possibility --
the mask carries the tier, the 8L rejected OWLv2 over LayerNorm and
Softmax density, and SegFormer is a transformer carrying 30 and 8 of
them. It compiles anyway, at 2.4x smaller counts than the model that
failed.

So the 8L runs the WHOLE tier: detector, floor mask, CLIP. The 10H stays
a **headroom** purchase at +$60 -- worth it for keeping embedding-head
models open (P16: 22% vs 5% on YOLO-World's cosine head), not for
anything in today's pipeline. The 20.0 MB vs 6.4 MB HEF is the visible
cost of the smaller part, and what that does to latency is unmeasured.

##### Process notes, and three harness faults in one measurement

* **32 GB is not enough.** Bias Correction holds activations for 64
  calibration entries at 512x512 and was OOM-killed at 29.9 GB -- the
  SAME mistake made with OWLv2 an hour earlier on the same box size.
* **A poll that only looks for success is not a poll.** One slept three
  hours against a process that died in 27 minutes: ~$3.60 for nothing.
* **Then the liveness fix watched the wrong signal** -- a log piped
  through `tail` inside the container, which buffers to EOF and cannot
  grow -- and declared a healthy job stalled. Poll the CONTAINER, not its
  output.
* **This account's G-instance vCPU quota is 8**, so `g5.2xlarge` is the
  largest GPU box available and it is the size that OOMs. Any future QAT
  row needs a reduced calibration set or a quota increase.

Records in `evaluations/hailo/zoo-probe/`; both HEFs are banked.

#### The Hailo-10H option, costed -- the download BLOCKER is cleared (P15)

Asked for 2026-09-14: is there another Pi-compatible NPU offering a wider
range of models? Within the Hailo family (4.9's table):

| | Hailo-8L | Hailo-8 | **Hailo-10H** |
|---|---|---|---|
| Price | ~$70 | ~$110 | **~$130** |
| Architecture | dataflow, no external memory | same, larger | **on-module 8GB LPDDR4X, built for transformers** |

**The Hailo-8 buys nothing.** Same architecture, so it fails on OWLv2's
layernorms exactly as the 8L did -- that is a dataflow limitation, not a
capacity one, and more TOPS does not create an attention path.

**The 10H is the only part that could change which MODELS are available**,
and for $60 it plausibly unlocks the 82%-accuracy model rather than the
72% one. Note 4.8/4.9 chose the 10H on 2026-09-06 and reverted it the same
day -- **but on its generative throughput** (5.89 tok/s makes a local VLM
pointless). **That reasoning does not apply to running OWLv2 as a
detector**, which is one forward pass and not token generation. The 10H's
case is therefore stronger now than when it was rejected.

**The blocker, found for free 2026-09-14:**

    hailo8l    OK
    hailo8     OK
    hailo10h   not a valid hw arch. Please use Dataflow Compiler v5.x

**DFC 3.34.0 cannot target the 10H at all.** v5.x is a separate gated
Developer Zone download. Everything in `tools/hailo/` carries over -- it is
one `--arch` flag plus a newer wheel in S3 -- so the cost is a login, not
engineering. **Fetch the wheel before costing anything else here.**

Outside Hailo, briefly: the **IMX500** is already ruled out (4.4, nano
ceiling in silicon and cannot be fed a recorded frame, which kills the
replay method); **Coral** is INT8-only with a narrow op set and a static
ecosystem; **RK3588** means a different SBC, not a Pi accessory; and
**MemryX MX3 / DeepX DX-M1 / Kinara Ara** claim broader coverage but are
**unverified here** -- and "the table says it should compile" is exactly
what P6 disproved for OWLv2. The standing asset argues for the 10H over
any of them: `tools/hailo/` took ~$5 and real engineering, it works, and
it is Hailo-specific.

**On YOLO-World + floor mask + CLIP on the 10H**, asked the same day: the
floor mask is **SegFormer, a ViT**, so on the 8-series it should fail as
OWLv2 did and the three-model pipeline cannot be fully on-board there.
Worth knowing -- but P9 measured that combination at **75% against 77%
without the mask, at 9x the latency**, so it is an option to have rather
than one to exercise. OWLv2 is the model worth 10H budget.

#### DECISION 2026-09-13: no Jetson. The part is Pi + Hailo-8L, and P9 becomes the critical path

**The owner has ruled out the Jetson on cost.** At $399 list / ~$480
street after July 2026's repricing (4.8), against ~$70 for a Hailo-8L M.2
plus a ~$30 camera, it roughly doubles a ~$555-620 build. That is a
budget decision, not a technical one, and it is final unless the owner
re-opens it.

**What it settles.** 4.7's "Jetson path, kept for later" is closed. P7c's
and P7d's recommendations, which assumed an Orin, are recorded but not
actionable. Most importantly:

**OWLv2 is OUT, and this is the expensive consequence.** It is the
strongest model this project has measured -- 82% at 3 FP against the
shipped pipeline's 58% on the same corpus -- and P6 established for $3.20
that it does not compile to a Hailo-8L: it translated, it quantised, and
it died at **allocation** on 73 layernorm and 38 softmax layers, which is
an architectural limit of the dataflow design rather than a size limit.
The same reasoning retires Grounding DINO, DINOv2, SAM and every VLM.
**Choosing the cheaper board means choosing against the best model**, and
that trade should be stated in those words rather than discovered later.

**What the on-board tier can therefore be:** a CNN proposer plus CLIP
re-ranking, with the text encoder on the Pi's CPU. Nothing else fits.

##### This promotes P9 from interesting to load-bearing

Within that constraint, P9 measured the only two candidates that exist:

| at the shipped gate, P >= 0.8 | recall | precision | FP |
|---|---|---|---|
| YOLO11s + CLIP RN50 (shipped) | 45% | 94% | 9 |
| **YOLO-World + CLIP RN50** | **72%** | **99%** | **2** |

Better on **both** axes, and twice as fast. So the on-board tier is worth
**45% or 72%** depending entirely on one unrun test:

> **Does YOLO-World compile to a Hailo-8L HEF?**
>
> **ANSWERED 2026-09-13: YES -- see P10.** translate/optimize/compile all
> pass for $1.05, cut at the six Conv end nodes the DFC's own error
> recommends. 25.5 MB HEF, 4 contexts. The split also **un-bakes the
> vocabulary**, so the retraction below about a frozen class list does not
> apply to the deployable form. What is still unmeasured is whether INT8
> preserves P9's accuracy -- P7d found it destroys OWLv2.

It is now the single highest-value open question in this document, and it
is the cheapest: `tools/hailo/` already exists, P6 cost $3.20 and 3.1
hours, and the loop is one `ec2.sh up` from running against a new model.
**Do this before ordering the accelerator**, which is what 1.10 item 1
has asked for since 2026-09-04 and is the reason that item existed.

**The case for optimism, and why it is only a case.** YOLO-World is
YOLOv8 with a text-conditioned head -- the image path is convolutional,
which is 4.9's "compiles well" family, and the text encoder would sit on
the Pi CPU exactly as CLIP's already does. **But OWLv2's table row said
it should compile too.** P6 is the standing warning that an inference
from architecture is not a measurement, and it is the reason this
question gets tested rather than assumed a second time.

**If it does not compile**, the on-board tier is YOLO11s + CLIP at 45%
recall, the reactive tier gets materially weaker, and the tiered
architecture leans harder on the cloud -- which raises cost per mission
and makes 2.9's latency budget the binding constraint again. That is a
worse system, not a broken one, and it is the outcome to plan against.

#### P9: composing YOLO-World instead of replacing with it -- **MEASURED 2026-09-13**

4.11 ran YOLO-World as a **replacement** for all three models and
`brain/perceive_lab.py` wrote the reason into its docstring: *"comparing
it any other way would answer a question nobody asked."* The question was
asked on 2026-09-13. Two things had changed since that comment: P7
falsified 4.11's *"the three-model pipeline dominates at every operating
point"*, so which models to COMPOSE is open again; and the corpus is now
**11 labelled walks / 1234 frames / 323 visible**, where every prior row
was scored on 8 walks or fewer.

**The mechanism this tests.** YOLO11s proposes crops from a fixed COCO
list, and "woven laundry basket" is not on it -- which is why
`crop_source` degrades to `low_confidence` on those walks, and why 4.2's
label gate discards exactly the close-range frames where YOLO calls the
basket a `vase`. YOLO-World proposes from the **target string**, so CLIP
re-ranks crops that are already about the right object. The composition
is a third pipeline, not either measured one, and the adapter is
`tools/yoloworld_crops.py`.

##### The result

Recall at matched false-positive budgets, all 11 walks:

| config | @0 FP | @3 FP | @16 FP | median ms | neg scored |
|---|---|---|---|---|---|
| YOLO11s + CLIP RN50 (**shipped**) | **31%** | 41% | 46% | 161 | 787/911 |
| YOLO11s + CLIP ViT-B/32 | 30% | 34% | 56% | 169 | 787/911 |
| YOLO11s + floor mask + CLIP RN50 | 4% | 49% | 66% | 773 | 866/911 |
| **YOLO-World + CLIP RN50** | 4% | **77%** | 82% | **78** | 32/911 |
| YOLO-World + CLIP ViT-B/32 | 0% | 76% | 81% | 84 | 32/911 |
| YOLO-World + CLIP ViT-L/14 | 3% | 75% | **83%** | 82 | 32/911 |
| YOLO-World + floor mask + CLIP RN50 | 4% | 75% | 80% | 733 | 755/911 |

**Three findings, in order of how much they change.**

**1. The gain is the CROP SOURCE, not the classifier.** Three CLIP
variants spanning an order of magnitude in size -- RN50, ViT-B/32,
ViT-L/14 -- land within **two points of each other at every budget**.
Swapping the classifier moves nothing; swapping the proposer nearly
doubles recall. 4.11 concluded the recall problem was not model capacity
and proposed a threshold fix; this agrees it is not capacity and locates
it one stage earlier, at **what gets proposed at all**. A classifier
cannot rescue a crop the detector never emitted. Note also that **RN50 is
the smallest and the one Hailo has already ported** (4.9), so the
cheapest CLIP is also the right one and no upgrade is owed.

**2. The floor mask helps YOLO11s and HURTS YOLO-World.** 41% -> 49% on
the COCO-limited detector; 77% -> **75%** on YOLO-World, at **9x the
latency**. The last column says why: it takes YOLO-World's scored
negatives from **32 to 755**, re-introducing exactly the
confident-false-positive surface the text conditioning was suppressing.
**The floor mask is a workaround for a vocabulary-limited proposer** --
4.2 added it as a class-agnostic crop source precisely because the COCO
list cannot propose an out-of-vocabulary target. Fix the proposer and the
workaround becomes a liability costing 655 ms/frame to do slightly worse.
That also retires C6's open question of when to schedule segmentation, if
YOLO-World is adopted: the answer becomes "never".

**3. The ranking is the same at 3 FP and at 16 FP**, so it does not
depend on where the budget is drawn -- with the one exception in the
caveats below.

**Nearly double the recall at the same error budget, at half the
latency.** And it is not one walk carrying it -- it wins on **9 of the 10
walks that have visible frames**:

| walk | visible | shipped | YOLO-World |
|---|---|---|---|
| blue-bottle-142454 | 18 | **89%** | 61% |
| blue-bottle-185007 | 10 | 0% | 50% |
| blue-shoes-152528 | 13 | 31% | 85% |
| blue-shoes-210511 | 25 | 32% | 64% |
| red-backpack-144856 | 33 | 88% | **100%** |
| basket-215252 | 23 | 0% | 61% |
| basket-215351 | 37 | 51% | 95% |
| basket-231358 | 37 | 5% | 78% |
| basket-115414 | 30 | 3% | 73% |
| basket-115703 | 97 | 55% | 74% |

The basket walks are the predicted mechanism firing: **5% -> 78%** and
**3% -> 73%** on exactly the out-of-vocabulary target where the COCO list
has nothing to offer.

##### Two caveats, and the second changes how it should be deployed

**It scores only 32 of 911 negative frames.** On 879 it proposes nothing
at all. That is *correct* rather than a gap -- a frame with no proposal
cannot be a false positive, and recall is over all 323 visible frames so
its 55 unscored positives count against it as misses. But it means **the
false-positive curve saturates at 32**: there is no operating point above
that, and the two configs' budgets are not drawn from comparable negative
pools. Read the @0 and @3 columns; @16 is already near the ceiling.

**At 0 FP it is much worse -- 4% against 31% -- and that is a property,
not noise.** Its highest-scoring false frame scores **exactly 1.000**
(blue-shoes 0017), which drags the zero-error gate to 1.000 and admits
almost nothing. The cause is structural: when YOLO-World proposes one
crop, CLIP's softmax has few competitors, so a wrong crop scores as
confidently as a right one. This document has already recorded the same
effect from the other direction -- *"more competitors in a softmax
mechanically lowers every P"* (3661).

So: **YOLO-World rarely fires falsely, but when it does it fires with
total confidence.** That makes it excellent as the standalone sighting
detector at a small error budget and bad as a zero-error corroborator --
which is the exact inverse of Grounding DINO's profile (50% at 0 FP, P7).
**1.11a wants both roles filled, and this says they want different
models.**

**One number not to over-read: the median ms.** On most frames
YOLO-World proposes nothing, so CLIP never runs and the median reflects
the detector alone -- which is why ViT-L/14 reads 82 ms against RN50's
78. It is a fair *per-frame* cost and an unfair *model* comparison. The
floor-mask rows are the ones where the median is doing real work, because
segmentation runs on every frame whether anything is proposed or not.

##### What it does NOT change

The hardware question. YOLO-World is YOLOv8 with a text-conditioned head,
so its image path is CNN and plausibly sits in 4.9's "compiles well"
family with the text encoder on the Pi CPU, exactly as CLIP's already is
-- **but that is an inference from architecture, not a measurement, and
P6 is the standing warning about exactly that inference.** OWLv2's table
row said it should compile too; it translated, quantised, and died at
allocation on 73 layernorm and 38 softmax layers. Until
`tools/hailo/` is pointed at YOLO-World, "it should compile" is worth
what OWLv2's row was worth.

**What makes it worth pointing there:** it is the only composition
measured so far that improves the on-board tier *without* changing the
board, and the loop is one `ec2.sh up` away.

##### The three 2026-09-12/13 walks, adjudicated -- **2026-09-13**

The corpus is **11 labelled walks**. Counts: 231358 **37/180**, 115414
**30/231**, 115703 **97/213**. Method in each file; three things it found
that are worth more than the counts.

**1. Each walk has its own tan distractor, and they are not the same
object.** 231358's is a grey **cable-knit bolster** on a weight bench --
woven texture, basket-sized, on the floor line, and the shipped pipeline
puts **5 of its 9 detections on this walk inside that span**. 115414's is
the beige **exercise ball**: tan, round, floor-standing. Between them the
corpus now offers two distinct woven/tan confusions with adjudicated
labels, which is what a false-positive budget needs to mean anything.

**2. 115414 is the hard walk, and the reason is geometric rather than
semantic.** The target is only ever seen **at long range through a
doorway** -- 24 frames of 231, never approached, never close. That is
4.11's small-distant-target limit, which no amount of model capacity
fixes, and NOT the out-of-vocabulary problem the crop sources own.
Scoring it beside an approach walk without saying so averages two
different problems, exactly as P3's per-walk split found for the
corpus-wide 84%.

**3. `walk.jsonl` was wrong again, and in the same way.** The 2026-09-13
handoff records 115414 as seeing the target on *"only 2 steps"*. It is in
frame on **24**. That number was `walk.jsonl`'s LOCAL-tier status, which
is the mislabelling this document already recorded twice (3925, and the
handoff's own section 7). **`labels.json` is the reference; `walk.jsonl`
is evidence about the model, not about the room** -- stated a third time
because it has now cost three readings.

**On scoring these with OWLv2, read the `method` field first.** 231358's
sheets were read before the prepass finished, so its labels are
independent and scoring OWLv2 against them is fair (36/37 at 16 false
positives). For 115703 and 115414 the OWLv2 spans were in hand while
reading: **recall on those two is optimistic and precision is the
trustworthy half.** Every span was still confirmed or rejected against
the pixels, so the labels are not OWLv2's output -- they are just not
blind to it. A genuinely blind re-read is cheap and has not been done.

#### P7e: the first walk that ARRIVED, and the two reasons nothing noticed -- **2026-09-13**

P7c item 3 argued, from latency alone, that `target_reached` must move off
the cloud, and named the line: *"`brain/tiered.py:475` forces
`target_reached: False` on every free step, so arrival is only declarable on
a paid one."* It was written on 2026-09-12 and marked **not built**.

The next day's walk is the first live evidence for it, and it goes one step
further than the argument did. **`woven-laundry-basket-20260913-115703`
arrived.** Frames 0204-0209 are the basket at touching distance, filling the
frame edge to edge -- the rig was pushed right up to it. The mission ended
`max_steps`.

| seq | step | action | `target_reached` | perception | P | `holding` |
|---|---|---|---|---|---|---|
| 203 | 117 | FORWARD | False | detected | 0.9977 | **STOP** |
| 204 | 117 | FORWARD | False | detected | 0.9977 | **STOP** |
| 206 | 119 | FORWARD | False | detected | 0.9973 | **STOP** |
| 208 | 120 | -- | -- | detected | 0.9558 | **STOP** |

Both tiers were right and neither was heard. The local tier was at
**P = 0.996-0.998**, about as certain as this pipeline ever gets. The cloud
had already returned `STOP` and it was being held. The robot drove FORWARD
into the basket until the step cap.

**This is not the lateness P7c predicted. It is an override.** P7c costed
arrival at ~5.6 s late, 1.7 m of overshoot at 0.3 m/s; fixing only the
latency would not have stopped this walk, because the arrival signal that
did exist was discarded rather than delayed. Two independent defects
compose, and either alone would have ended the mission correctly:

**1. Phase G outranks a stop.** `_local_scene()` computes

    direction = steer or held or SCAN_ACTION

where `steer` is a bearing measured this frame and `held` is the cloud's
last goal. Phase G's precedence is sound **for directions** -- a fresh
bearing really is better evidence than a three-second-old one. The defect is
that `safest_direction` is an **overloaded channel**: it carries both "which
way" and "do not move", and `brain/navigate.py:166` also defaults to `STOP`
for any unrecognised action. Three meanings, one token. Phase G applies
direction-logic to a mode change, and the mode change loses.

**2. Arrival is dropped in transit.** `_held_direction()` returns only
`_last_cloud_scene["safest_direction"]`. The rest of the cloud's reply --
`target_reached` included -- is held in `_last_cloud_scene` and never read,
and `_local_scene()` then hardcodes `target_reached: False`. So
`control/mission_runner.py`'s `if nav.get("target_reached")` cannot fire on
a free frame whatever the cloud said. This is exactly P7c item 3's line,
observed doing the thing it was predicted to do.

##### Why "make a held STOP un-overridable" is the wrong repair

It is the obvious one-line fix and it should not be taken:

- **`STOP` is ambiguous** -- arrived, blocked, and unparseable-action all
  produce it.
- **A held `STOP` is stale by construction.** Holding a goal across the
  round trip is the entire point of Phase A.
- **It can deadlock.** A stopped robot's view does not change, so the next
  call sees the same blocked scene and answers `STOP` again. Today's
  override is at least what keeps the robot moving.

`target_reached: True` has none of these properties. It means one thing, and
the correct response to it -- end the mission -- cannot deadlock, because
there is no mission left to deadlock.

##### The three candidates, and what each needs

1. **Carry `target_reached` on the held goal.** The smallest correct change:
   arrival stops being undeclarable on a free frame. Still ~5.6 s late, and
   still says nothing about the override.
2. **Local arrival -- P7c item 3's own proposal.** OWLv2 for bearing, lidar
   for range at that bearing, a threshold. Both on-board, both under 33 ms.
   **Needs the lidar**, so it is hardware-day work by construction.
3. **An interim local arrival with no lidar**, using detected-box area as a
   range proxy. At touching distance the signal was enormous -- near-full-frame
   box at P = 0.998 -- so it is almost certainly separable on this corpus.

**Recommendation: (3) is REPORTED and NOT ENFORCED when it is built**, on
1.11a's precedent and for 1.11a's reason. A threshold chosen from one walk is
how `DEFAULT_MATCH_MARGIN` came to be ~2x too high, and a panel that lets a
measurement read as a decision corrupts the walks meant to decide it. Compute
the verdict, show it beside the corroboration row, enforce nothing, and let
the next rig walks say where it separates.

##### Status: NOT BUILT, and deliberately deferred

None of the three is built, and none should be built now. The arrival
question resolves differently once a lidar exists -- (2) becomes available
and (3) becomes an interim nobody needs -- so writing (1) or (3) today
optimises against a sensor suite that is about to change. The decision
(2026-09-13) is to record the evidence and settle it when the hardware is
bought, deployed and tested on.

**What this costs in the meantime**, stated plainly so it is not discovered
again: every tiered walk will continue to end `max_steps` rather than
`found`, including walks that physically arrive. Walk outcomes from this
period are evidence about steering and pacing, **not** about arrival, and
`control/walk_eval.py`'s completion score -- 0.25 of the total, keyed on
`target_reached` -- is structurally zero for all of them. Do not read it as a
navigation result.

##### One instrumentation fix was NOT deferred

`_tier.cloud_called` was False on every frame of every async mission from
Phase A until 2026-09-13: the dispatch branch returns `_local_scene()`, which
hardcoded it, and only the synchronous path ever set it True. The twin's
counter was unaffected -- it reads `stats.cloud_calls`, incremented before
dispatch -- but `_tier` is what a recorded walk stores, so **no walk could
say which frame the cloud had been shown**, and `[cloud: <trigger>]` never
appeared in an async mission log. That is what made "was the cloud ever asked
at point-blank range?" unanswerable from the record of three walks in which
`holding` was set on 169/222/204 rows and `cloud_called` on none.

Fixed the same day, with `cloud_landed` added as the separate fact, because
the deferral above depends on the record being readable later. A walk
recorded blind cannot be re-analysed when the hardware lands.

### 4.11 Is the local tier worth the part? -- **measured 2026-09-08**

Asked directly after the search walk, where the on-board tier was silent
for 212 frames: *if it contributes nothing, what is the Hailo for, and
would a Jetson run models good enough to change the answer?* The honest
route to an answer was to try the model a Jetson would be bought for.

#### The open-vocabulary detector loses to the pipeline it would replace

YOLO-World takes the target string **into the detector** -- no crop
proposals, no CLIP, no COCO list, one model instead of three. It is the
shape of thing the "run arbitrary Hugging Face models" argument (4.7) is
really about. Run over all four walks against the adjudicated labels
(74 visible frames of 299):

| | recall | false positives |
|---|---|---|
| YOLO-World @ conf 0.30 | 35/74 | **0** |
| YOLO-World @ conf 0.10 | 41/74 | 5 |
| YOLO-World @ conf 0.02 | 63/74 | 14 |
| **current: YOLO + floor mask + CLIP @ P>=0.8** | **62/74** | **3** |
| current @ P>=0.50 | 69/74 | 16 |

**The three-model pipeline dominates at every operating point.** At ~3
false positives it finds 62 where YOLO-World finds ~36; at ~14-16 it finds
69 where YOLO-World finds 63. The composed pipeline is not a workaround for
lacking a better detector -- on this corpus it *is* the better detector.

> **FALSIFIED on the full corpus, 2026-09-11 -- see P7.** This row and
> the claim above were measured on FOUR of the corpus's eight labelled
> walks. Scored over all 610 frames, the shipped pipeline reads **58%** at
> 3 FP rather than 85%, and **Grounding DINO gets 50% at ZERO false
> positives where the shipped pipeline gets 8%** -- so it does not
> dominate at every operating point. OWLv2's margin over it *widens*
> (11 points -> 24), so the conclusion this section draws about the PART
> survives; the claim about the pipeline does not.

Two further results from the same run, both load-bearing:

- **YOLO-World never falls for the storage bin either** -- 0 of the 34
  frames the VLM claimed. Its 13 false positives on the search walk are a
  **vacuum cleaner**, at frames 0136-0155. So the confabulation is specific
  to the cloud tier, and 1.11a's premise survives a second local model.
- **Its recall is excellent where the target is large** (33/33 on the
  backpack walk, perfect precision) and poor where the target is small.
  Same failure axis as the current pipeline.

> **Composed rather than replaced, YOLO-World WINS -- see P9
> (2026-09-13).** This section ran it as a replacement for all three
> models, which `brain/perceive_lab.py` says was deliberate. As a crop
> source feeding the same CLIP matcher it reads **77% at 3 FP against the
> shipped 41%**, on 11 walks, at half the latency -- and the floor mask,
> which helps YOLO11s, *hurts* it. The conclusion below ("not a model
> problem") survives and is sharpened: it is not a model-capacity
> problem, it is a **proposal** problem, one stage earlier than the
> threshold fix this section proposes.

#### So the recall problem is not a model problem

Sweeping the current pipeline's own gate says where the information is:

| gate | total recall | false pos | **search-walk recall** |
|---|---|---|---|
| 0.80 (shipped) | 62/74 | 3 | **2/10** |
| 0.60 | 68/74 | 13 | 7/10 |
| 0.50 | 69/74 | 16 | **8/10** |

**The signal for six more search sightings is already in the scores** -- it
sits between 0.5 and 0.8, which is exactly the band 1.11a measured for
corroboration. Lowering the gate globally is not the answer (3 -> 16 false
positives), but that is the only thing a bigger model or a faster part would
change, and it is not what is limiting the system.

**The recall fix and the confabulation fix are therefore the same change**:
one threshold for a standalone sighting (0.8, precision-first) and a lower
one for corroborating a claim the cloud has already made (0.5). That is
1.11a, and this is a second, independent argument for it.

#### What this says about the part

The case for on-board perception **shifts rather than weakens**, and the new
version is stronger:

- **It was never mainly about saving calls.** On the search walk the saving
  came from the timer, not the tier. But on approaches the tier does fire,
  and approaches are where collisions and arrival happen.
- **It is the only thing that caught the cloud being wrong** -- 34 of 34,
  and YOLO-World agrees. That is a correctness role, not a cost role, and
  nothing in the cloud tier can perform it by construction.
- **The measured bottleneck is small distant targets and crop proposals**,
  neither of which more compute fixes. A Jetson buys model *capacity*; the
  evidence says capacity is not what is short.

**The honest limit of this argument**: only one open-vocabulary model was
tested. Grounding DINO and OWLv2 are stronger and too heavy for a Hailo, and
SAM would give class-agnostic proposals directly -- the exact weakness
measured. **None of those has been tried**, and a Jetson is the only way to
run them. So this is evidence that the *current* plan is sound, not proof
that a Jetson would not help. 4.7's promotion rule already covers the
follow-up: try them off-robot first, behind the `RegionProposer` and
`Detector` Protocols, which is now a two-line substitution.

> **That limit was closed the next day, and it did not hold. See "The three
> untried models" below: Grounding DINO and YOLO-World lose as this section
> predicts, SAM confirms the crop-proposal diagnosis -- and **OWLv2 beats the
> shipped pipeline at every operating point that admits a single false
> positive**, which reverses the "capacity is not what is short" conclusion
> for one specific job.**

#### Three cautions, so no result here is over-read

**Throughput is not measurable on a laptop.** 2.9 budgets three models against
an 8L's 33ms frame; a Mac is not one. The timings the harness prints compare
*models to each other*, never model to robot.

**A high match rate is not automatically good.** CLIP returns a similarity, not
a probability, so the margin over competing strings is the number that means
something -- which is why `DEFAULT_DISTRACTORS` exists and why every match here
is scored against them rather than against a bare threshold.

**And this measures finding, not navigating.** Stage 0's failure was never
recognition -- *"every cloud model identifies a red backpack"* -- so a perfect
score here would leave the actual gap untouched. That gap is `brain/planner.py`,
and it is C8.

---

## 5. What this invalidates elsewhere

The chassis decision has documentation consequences, and **1.14's move to
continuous driving has more**. Recorded here so they are not discovered on
hardware day.

| Document | What is now wrong |
|---|---|
| `HARDWARE-READINESS.md` | **This row was stale and is corrected 2026-09-06.** It described the file as *"written for the PiCar-X throughout"*, but that document was rewritten 2026-09-04 -- its §1 already lists the Yahboom chassis and the RPLidar C1, its §5.2 is already headed "resolved by the chassis", and its §5.3 is no longer about the ultrasonic at all. **What is still stale in it:** "Hailo-8L **AI HAT+**" (4.9 settled on the M.2 module) and "~$500 of parts" (3.6 now says ~555-620). CLAUDE.md repeats the old version of this row and needs the same fix |
| `PLAN-sim-hardening.md` | **S6's Ackermann half is unnecessary** -- `grid_world.py`'s pivot assumption is now correct, and §3.3's divergence closes by hardware choice rather than by code. **But 1.14 un-retires S6's continuous-pose half (2026-09-06)**, for a different reason than S6 gave and at a much smaller cost: `sim/renderer.py` already takes a float pose, so only `grid_world.py` and a two-line boundary conversion in `mock_robot.py` are discrete |
| `AGENT-HARNESS.md` | **1.14 splits the tick.** Its tick contract, concurrency model and status shape all assume one blocking sense-decide-act step. Continuous driving needs a drive loop and a mission loop at different rates. B3.2 and B3.3 keep their jobs but change what they time |
| `robot/safety.py`, `config/robot.yaml` | **Two numbers are wrong under 1.14, in code, today.** `watchdog_timeout_s: 1.0` is 50cm of travel at 50cm/s, and the fixed `min_distance_cm: 20.0` is a stopping distance good for only ~0.45 m/s -- **~0.15-0.2 m/s once 1.14 item 5's corrected `t_react` and the sensor offset are applied.** Neither is wrong for discrete motion, which is why neither was caught. **A third was found and FIXED 2026-09-06**: the collar had no sensor-to-bumper term, which is exactly zero for a front-mounted ultrasonic and 11-14cm for a deck-centre lidar. `SENSOR_TO_BUMPER_CM` and `safety.sensor_to_bumper_cm` now carry it, `path_clearance()` returns bumper-relative clearance, `GET /depth` publishes the offset, and `tests/test_depth_veto.py` pins it. Fixed while the number is still zero, so a sensor move cannot change the comparison's meaning silently |
| `robot/interface.py` | Has no way to express a held velocity. 1.14 item 3 adds one, and `tests/test_robot_contract.py` has to carry it across all five backends |
| `PLAN-aws-cost-redesign.md` | **Missing from this table until 2026-09-06, and it invalidates two phases.** Nine stacks -- every ECS service, the NLB, the ALB, the VPC and EFS -- were deleted 2026-09-05, *one day before* C1-C9 was written. So C5 and C7's "ALB path patterns" name infrastructure that does not exist, 3.4's "running on ECS Fargate" is wrong, and this document's five references to the corpus being "on EFS" contradict 1.16 #10, which correctly says S3. All corrected in place |
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

**Replay the trigger policy over recorded walks.** Every walk on S3 carries
`walk.jsonl` -- frames plus the `/navigate` reply at the time, including
`room_guess`. Running a candidate trigger policy over that data offline counts
**how many deliberation calls it would have fired** versus the number of frames.
No new inference, no Bedrock charge, no hardware. It turns §2.4's cost claim
into a number, on data already owned -- the same move `control/walk_replay.py`
already makes for prompts.

**Run: `python -m tests.manual_trigger_count`** -- 39 walks, 821 recorded
frames, §2.4's six triggers, `stale_n=8`. *(Six was the table's full set when
this ran; 2.4 gained a seventh, `cold search`, on 2026-09-06. The measurement
predates it and has not been re-run -- the count is not an error.)* Two of them are not witnessable in
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

**Amended 2026-09-06, because the sentence that stood here is no longer true.**
It read *"what remains open is not design but measurement and verification"*,
and three things have since reopened design:

- **1.16 #11** -- whether the floor mask gets a veto -- is a decision, not a
  measurement, and the plan currently contains both halves of it.
- **C4-C9** (1.14) named four things no phase covered: the goal vocabulary as a
  *type*, the feature transport, the two cloud contracts, and `brain/planner.py`.
  That is design and build work, not verification.
- **4.2's open-vocabulary path** is a design choice with no implementation and no
  measurement behind it.

What remains open on the **measurement and verification** side, which is what
that sentence was reaching for: **3.8's seven seller questions** (five when this
was written; 6 and 7 were added 2026-09-06, and 6 is the only one in this
document that cannot be fixed after delivery), the compile loop 1.10 item 1 asks
for before hardware day -- which 4.3's note now points at floor segmentation and
which is therefore **downstream of the corpus** -- and **1.16 #10's re-recorded
Stage 0 corpus**, which four things now wait on.

**Two of these moved on 2026-09-06.** The compile loop now has a phase series
(4.10's P1-P4) rather than being an item with no home, and **its first four
fifths are built**: `brain/perceive.py` and `brain/tiered.py` run the real
detector, CLIP and trigger policy against real photographs, on any machine,
with no accelerator and no robot. What is left of it is P4 -- one EC2 hour and
a Hailo developer account. And the corpus count went from three to four,
because P3 has nothing valid to score against either.
**1.10's ordering-time checks are closed**, not open: 4.9 settled the storage
decision (configuration C or D) and the AI HAT+ 2 question (take the 8L).
**6.1's free trigger-count experiment is done** (2026-09-03) -- it is the one
item on that list that needed neither a seller nor a walk.

Kept as an index into where each landed:

| # | Question | Where it landed |
|---|---|---|
| Q1 | Is there a map, and where does memory live? | **1.5** persistence -- only the map persists, planner is a pure function · **1.6** the visual-edge mechanism is cancelled |
| Q2 | Closed or open goal vocabulary? | **1.7** closed and versioned, **four** verbs (three until 1.15.3 added the report-only goal), unknown verbs refuse by name |
| Q3 | Who owns the stop condition? | **1.8** typed success from the planner, typed failure from the robot, lidar x bearing |
| Q4 | Is there an on-device detector, and who arbitrates? | **1.11a** PROPOSED 2026-09-07: the VLM's identity claims need local corroboration on a search, where its precision measured 23% · **1.10** a Hailo-8L, two layers (the IMX500 for one day -- §4 has the comparison) · **1.11** arbitration split by question, not authority · **1.13** room identity is not one of the detector's jobs · **4.8**/**4.9** re-checked against industry practice and 2026 prices; the part settles as a Hailo-8L in M.2 module form · **1.15** the physical layout it has to live in |
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
  **PARTLY BUILT 2026-09-07** (4.10's "What it owes the twin"): the name, the
  margin and the tri-state are on the Remote brain panel under
  `policy: "tiered"`. The boxes are not, and cannot be until there is a camera
  whose frames a detector may honestly be run against -- over the twin's FPV
  canvas they would be boxes on a raycaster render, which 1.12 forbids.
- **The tiers:** the current goal, when it was set, and what triggered it -- plus
  a deliberation-call counter that **visibly does not climb every step**. That
  single number makes the whole architecture watchable.
  **BUILT 2026-09-07**, as calls *and* frames plus the live ratio, with the
  trigger named on every paid line of the mission log. The *goal* half is still
  owed and belongs to C4/C6 -- nothing holds a goal yet, which is also why
  `UNAVAILABLE_TRIGGERS` lists four of 2.4's seven triggers as unfirable.
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
| **EFS** | Elastic File System. **Deleted 2026-09-05** -- it held the recorded walks and was the only component that required a VPC; the corpus is on S3 now (`PLAN-aws-cost-redesign.md`) |
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
