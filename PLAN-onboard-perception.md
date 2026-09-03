# Plan: perception on the car itself

Status: **decisions recorded, nothing built** · Date: 2026-09-03 · Phase IDs: none assigned yet

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

### 1.9 A live tension, not yet resolved

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

The saving is exactly **reactive steps per goal**, and that number is
**unmeasured**. Sim runs took 40 steps; real walks reached target in 6-22
frames. At ~8 reactive steps per goal it is ~8x -- a real number, but derived,
not measured, and it should not be quoted as one until §6.1's free experiment
has been run.

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
| **Slamtec RPLidar C1** | 100 | 1.2 |
| Differential chassis kit, **encoder motors** | 100 | 60-150; encoders push it up |
| Motor driver (TB6612FNG) | 10 | 0 if the kit includes one |
| Camera Module 3 | 30 | Q4 may change this |
| 2-axis pan/tilt bracket + SG90s | 12 | Replaces what the PiCar-X bundled |
| **3S** Li-ion pack + charger | 35 | Motor rail only (1.3). **Not 2S** -- see the correction below |
| Wiring, connectors, switch, XT60 | 15 | |
| Standoffs, M2.5/M3 hardware | 10 | For stacking decks |
| | **~407** | |

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
| AI Camera (IMX500) instead of Camera Module 3 | +40 | Q4, still open |

**Totals**

| Scenario | ~USD |
|---|---|
| Essential only | 407 |
| **+ recommended** | **465** |
| + NVMe | 510 |
| + AI Camera instead | 550 |
| Already own a Pi 5 | subtract ~100 |

Already owned, 0: the power bank (1.3). Deferred, possibly never: any AI
accelerator (4.5). **Budget ~450-500**, and the two variables that move it are
whether a Pi 5 is already owned and how Q4 resolves.

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

**Not decided.** Recorded because dismissing it required believing something
untrue, and that is the kind of premise worth writing down.

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
| `PLAN-microduck-transplants.md` | **M2/M3 are built (2026-09-03) and the seam holds** -- but `PATH_FRACTION` does not. See §5.1, which is the one concrete defect this decision creates in existing code. **M10** (clearance from a real sensor) is satisfied far better by 360-degree metric returns than by one ultrasonic beam |
| `CLAUDE.md` | The status table and build order reference S6 and the PiCar-X hardware path |

`HARDWARE-READINESS.md` and `PLAN-sim-hardening.md` have had staleness notes
added pointing here. Nothing else has been edited.

### 5.1 `PATH_FRACTION` breaks on a 360-degree sensor

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

---

## 6. Open questions

### 6.1 Measurable today, for free

**Replay the trigger policy over recorded walks.** Every walk on EFS carries
`walk.jsonl` -- frames plus the `/navigate` reply at the time, including
`room_guess`. Running a candidate trigger policy over that data offline counts
**how many deliberation calls it would have fired** versus the number of frames.
No new inference, no Bedrock charge, no hardware. It turns §2.4's cost claim
into a number, on data already owned -- the same move `control/walk_replay.py`
already makes for prompts.

**Caveat with teeth:** the current corpus is the invalid one (target on raised
furniture, camera at standing height). The trigger count is probably robust to
that, being about room transitions rather than depth, but it should be re-run on
the re-recorded corpus before anyone quotes it.

### 6.2 Still to discuss

Q2 (goal vocabulary) and Q3 (stop conditions) are **settled** -- see 1.7 and 1.8.

| # | Question | Why it is not obvious |
|---|---|---|
| Q4 | **Who arbitrates a confident detector against a planner that says the target is not here?** -- and, from 1.9, **is there an on-device detector at all?** | M4's subject, and Microduck's own open #3. The IMX500-embedding sub-question is **withdrawn**: 1.6 cancels the visual-edge design, so Q4 turns on detection quality and on 1.9's stale-bearing problem, not on embeddings |
| Q5 | **Does the sim participate at all?** | `sim/renderer.py` renders flat-shaded walls; a COCO detector finds nothing in them, so a sim backend would have to synthesise detections -- making the sim leg unable to test the detector, only its consumers |

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
