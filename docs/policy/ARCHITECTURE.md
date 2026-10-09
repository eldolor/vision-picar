---
kind: architecture
domain: policy
status: current
verified: 2026-10-02
---

# Policy -- architecture

The policy is what decides each move. Every tick, the mission runner hands it
one camera frame; it returns one action out of a small fixed set, which the
safety layer then vets. Three policies exist -- a free rule-based explorer, a
cloud vision policy, and a tiered policy that asks the cloud only on events
-- plus the arrival rule that lets a mission end `found`: the lidar judges
the distance, and one cloud call on the arrival frame confirms the identity.
Read this for why decisions are split the way they are; the
[engineering spec](../engineering/policy/ENGINEERING.md) has the constants,
precedence code, and the sweeps that pin them.

## Purpose

A mission needs something that turns "what the robot sees and where it is"
into "what to do next", in a way that can be swapped without touching the
mission lifecycle, the safety layer or the body. The policy domain owns that
decision and nothing else. It depends on perception (what is in view), the
world (where the robot is), the body's sensors (how much room there is), and
the cloud vision service (what the target is, when asked). The mission
service depends on it for every step, and the twin's readouts (turns, tier
counters, arrival) are its outputs.

## Components and boundaries

```text
            frame (pixels, pan, odometry, sim detections)
                         |
                         v
  +----------------- vision function -----------------+
  |  frontier: sensed scene (depth/scan clearance)    |
  |  vision:   one cloud navigation call -> scene     |
  |  tiered:   local perception every frame,          |
  |            cloud only on a trigger -> scene       |
  +---------------------------------------------------+
                         |  one scene schema
                         v
  arrival review (local perception + lidar; then one cloud call confirms
  identity) -> may rewrite to STOP/found
                         |
                         v
  decide: frontier preference | trust the model | stuck-breaker
                         |
                         v
           safety layer (robot side) -> body
```

| Part | Owns | Must not |
|---|---|---|
| Constrained agent | the allowed action set, the capture -> scene -> decide -> vet -> execute step, a stuck-breaker | bypass the safety layer; trust a model's distance claim over a sensor |
| Mission agent | recording every step into mission memory, arrival review, room backfill from the cloud's room guess, sighting poses from the world | read simulator state; import a world backend |
| Frontier explorer | rule-based coverage: peek, prefer unvisited directions | grow; it is kept, not extended (`PLAN-sim-hardening.md` 2.2) |
| Explore policy (`explore`) | a search over SLAM's map: frontier, view and approach goals sent to nav2, the retry rule, the escape when wedged | send a verb while a goal is live; choose a goal the robot could not drive away from (3.45) |
| Vision agent | trusting the model's action unless the mission is complete | peek (a pan costs a real move and buys nothing a photograph lacks) |
| Cloud vision step | one frame -> one cloud navigation answer -> the scene schema | retry; the mission's failure budget is the retry policy |
| Tier (trigger discipline) | when to spend a cloud call, and what to do on free frames | become a planner or a reactive goal executor |
| Arrival check | "arrived" from detection + bearing + lidar range | read the detector's own distance, or any simulator fact; call the cloud |
| Identity confirmation (the mission agent) | asking the cloud, once per arrival, whether the arrival frame shows the target; "found" only on its yes | end a mission on a judgement it did not ask the cloud for |
| Goal pose | holding an anchored sighting across rotation | decide anything |
| Mission memory, room matching | rooms visited/searched, sightings, action history | drive control flow beyond completion |

The brain-wide rule applies with full force: nothing here imports a robot or
world backend or the simulator. Simulator-provided detections reach the tier
only as data in the frame (see [perception](../perception/ARCHITECTURE.md)).

## Decisions

### A constrained action set, one short move at a time

**Decision.** The policy picks one of seven actions (forward, left, right,
reverse, stop, look left, look right; the explorer's look-around also
re-centres the camera) each tick, every move short and
re-evaluated next tick. The safety layer re-checks every move against
sensors and can veto it.

**Rejected.** Longer commands or free-form plans from the model: errors here
are physical and not cheaply reversible, which argues for the least agentic
thing that does the job (`AGENT-HARNESS.md` section 11).
**Trade-off.** More ticks, and turn/forward alternation is visible by design.

### One scene schema behind one seam

**Decision.** Every policy produces the same scene -- obstacles, free space,
the important objects, the safest direction -- from a single "frame in, scene
out" function. Swapping the rule-based policy for the model changes nothing
downstream. A target counts as found only when it appears in the scene's
important objects, which the cloud policy fills only on **target reached**,
not on first sight -- otherwise a mission would end in a doorway across the
room from the backpack (`brain/navigate.py`'s note).

**Which paths can end a mission `found`, as the code behaves.** Under the
vision policy, the cloud's target-reached answer does. Under the tiered
policy with the cloud call dispatched asynchronously (the shipped mode), it
cannot: a landed cloud answer only becomes the held goal, and the scene every
frame returns is the local one, which never names the target. There, only
the arrival rule (below) ends a tiered mission `found`. With a blocking cloud
call, the trigger frame's cloud answer can end it as well. This, rather than
the arrival rule alone, is why a tiered phone walk (no range sensor) that
reaches its target ends at the step budget (P7e). That a landed
target-reached does not count was decided by the user on 2026-10-02 (see
"The cloud confirms identity at arrival; the lidar decides distance").

### The cloud-driven policies are the hardware path; the explorer is a test tool

**Decision** (`PLAN-sim-hardening.md` Q1, 2.2). The robot is vision-driven.
The frontier explorer is kept because it is free, deterministic and the
fastest way to exercise the safety layer and mission memory, and since the
ROS alignment it reads only robot sensors and the world's pose, never grid
cells (`PLAN-ros-alignment.md` 3.2). Without a world that can localise it
degrades to the right-hand rule rather than failing. nav2's frontier
exploration is its replacement on the car.

### Deliberation is event-driven, never periodic

**Decision** (`PLAN-onboard-perception.md` 2.4). The tier runs local
perception on every frame for free and calls the cloud only on: mission
start, a confirmed candidate sighting, a run of confirmed absence (cold
search), and a staleness floor so a robot that can see its target is not
silenced by things going well. An edge must hold for consecutive frames
before it is believed.

**Rejected.** A cloud call per frame (the vision policy's cost profile), and
firing on raw edges: 6.1 measured that about 40% of a naive trigger count is
field flicker; hysteresis took the saving from 2.8x to 4.1x. The three other
triggers in 2.4 (goal achieved, goal impossible, room change) need tiers that
do not exist and are named as unavailable rather than faked.

**Scope of the hysteresis.** It gates *when the cloud is called*, and nothing
else. It does not gate steering: the next decision makes that explicit.

### Arbitration split by question, and whoever sees the target steers

**Decision** (1.11, and Phase G of 2026-09-12). Identity belongs to the cloud
model; bearing to the local detector; range to the lidar; collision to the
safety layer, which never votes. On a frame where local perception has a
measured bearing, that bearing steers -- ahead of a cloud direction several
seconds old. The full precedence on a free frame is: a bearing measured this
frame, then a dead-reckoned bearing to an anchored sighting (built, off),
then the last cloud goal, then a search turn.

**Rejected.** Letting the cloud keep both identity and direction: on the rig
walk that motivated the change, the robot turned right 28 times and forward
4 on 37 frames where the target was locally detected.

**Trade-off, stated as the code behaves.** A single detected frame steers --
the consecutive-frame hysteresis gates only the cloud triggers, not steering.
For **steering**, the only bound is the local verifier's per-frame
probability gate: a wrong object steers the robot on every frame it passes
that gate. The cloud's identity does not stop it. With the cloud call
dispatched asynchronously (the shipped mode) the cloud's answer only becomes
the held goal, which a local sighting outranks, so a cloud "not visible"
never overrides a frame where the wrong object is detected. With a blocking
call the cloud's answer decides the frame the call was made on -- unless the
arrival review, which runs after the vision step, rewrites that same frame
to a stop and `found` (it overrides the trigger's direction, never its
identity: `found` still needs the arrival confirmation's yes). For **ending `found`**, the
bound is the arrival rule's conditions on top of the gate (consecutive
frames, centred, a lidar range within the radius, one surface), all of them
local: a wrong object that keeps passing the gate can be driven to and
reported `found` -- until 2026-10-02, when the user decided, and the same
day it was built, that a cloud identity check closes the `found` half of
this. Steering stays as described (see "The cloud confirms identity at
arrival; the lidar decides distance"). The corroboration verdict
([perception](../perception/ARCHITECTURE.md)) still measures the exposure and
enforces nothing.

### Turns are sized; search turns are smaller than the field of view

**Decision** (R1, R1b, R1c -- `PLAN-ros-alignment.md` 3.3, 3.5, 3.6b). A turn
chosen from a measured bearing turns by that bearing (clamped); a turn with no
bearing behind it is a search step smaller than the camera's field of view;
corrections use a finer band than the reporting vocabulary; and the spin
guard that forces a forward after long unproductive turning counts degrees,
not turns.

**Why.** Quarter turns against a 10-degree centre band closed zero distance
and flipped left/right on 52 of 60 steps. Quarter-turn search steps left
blind gaps between views. And when the search step halved, a guard counted in
turns silently halved its meaning. **Lesson recorded:** a threshold counted in
steps changes meaning when the step does.

### Arrival is the rule the car runs, judged by the range sensor

**Decision** (P7e first half, `PLAN-ros-alignment.md` 3.11, decided by the
user). A mission is judged ARRIVED locally when the target is detected, centred
within the steering band, and the lidar reads it within the arrival radius at
that bearing, on consecutive frames. It refuses to judge -- rather than
guess -- with no local perception, no usable scan, or a panned camera, and it
refuses a window of returns that looks like an edge rather than one face
(3.32: a mission had stopped beside a door jamb and declared `found` 1.1 m
short).

**The panned-camera refusal holds on real frames only.** Sim frames carry no
pan angle and their reported bearings are already relative to the camera,
so in the sim arrival cannot tell a panned camera from a centred one. That is
harmless today: the cloud-driven policies never peek, and a mission starts
with the camera centred (`PLAN-ros-alignment.md` 3.20). It stops being
harmless the day a tiered policy pans.

**Rejected.** 3.8's sim-only rule using the simulator's detection distances:
it would pass in the sim while saying nothing about the robot. **Rejected
also:** the detector's box size as range. **Acceptance bars (commitments):**
of missions that physically arrive, at least 95% end `found`, at perfect and
at degraded detection; and no mission ends `found` more than 0.60 m from the
target.

### The cloud confirms identity at arrival; the lidar decides distance

**Decision.** Decided by the user 2026-10-02; built the same day. Two rules,
decided together, that split the end of a tiered mission by question, as the
arbitration above splits steering:

- **Identity: the cloud confirms at arrival.** Before a tiered mission ends
  `found`, one paid cloud call on the arrival frame must agree that what the
  robot has stopped at is the target. If the cloud says no, or the mission's
  call budget is spent, the mission does not end `found`, and a refused
  arrival is not paid for again while the robot stays there. A call that
  FAILS is not an answer: it counts against the vision failure budget like
  any cloud failure, and the arrival is asked again. If the budget runs
  out while the arrival still holds, the mission ends `arrived_unconfirmed`
  (3.47, chosen by the user 2026-10-07): the robot reached what the local
  tier took for the target, and nothing checked what it is. It is never
  `found`, because a local false positive looks exactly like it, and it is
  not `failed`, because the robot did its part. A cloud that answers no is a
  refusal and never ends there.
- **Distance: the arrival rule only.** A cloud target-reached answer that
  lands under the asynchronous tier does not end a tiered mission. The
  lidar-judged arrival rule stays the only way a tiered mission ends `found`
  on the car.

**Rejected, for identity:**

- Enforcing the corroboration verdict ([perception](../perception/ARCHITECTURE.md))
  at arrival. It is free, but it was measured net-negative on the search
  walk and is unproven on out-of-vocabulary targets.
- Extending the hysteresis to steering. It reduces wrong-object chases, but
  a wrong object that keeps passing the gate for consecutive frames still
  ends `found`.
- Leaving it as is: the per-frame probability gate is then the only bound on
  a wrong `found`.

**Rejected, for distance:**

- Applying a landed cloud target-reached. It reintroduces a distance judged
  from a photograph -- uncalibrated, and exactly what the arrival rule was
  built not to trust -- onto the car.
- Applying it only on bodies with no range sensor. It lets phone walks end
  `found`, but makes a mission's ending depend on which body ran it, and the
  phone walk is not the car.

**Trade-off.** One paid cloud call per arrival (a synchronous tier whose
trigger fires on the arrival frame pays twice for that frame), spent on the judgement the
cloud model is good at (identity) and never on the one it is not (range).
Steering is unchanged: a wrong object can still be driven to; it can no
longer be reported `found`. Accepted consequence: a tiered phone walk, which
has no range sensor, cannot end `found`, by design, and ends at its step
budget (P7e stays).

### Hold the cloud's goal; never block on it

**Decision** (2.5, Phases A and F). The cloud call is dispatched without
blocking the tick, at most one in flight, and the last cloud direction is
held on free frames. An answer that lands after the mission ends is dropped.
**One exception, by design:** the identity confirmation at arrival blocks the
tick, because the mission is about to end on its answer. It waits for any
call already in flight before it is made, so "at most one in flight" holds
for it too (`docs-review/SPEC-REVIEW-3.md` fix 8).
**Rejected.** Blocking the tick on a multi-second call (a stall every few
frames), and scanning on every free frame, which on five rig walks outvoted
the cloud five to one.

### A sighting can outlive the frame -- but only when data says so

**Decision** (P25 / P7c item 2). A goal pose anchors a sighting in the
odometry frame so the bearing survives rotation; without a range it holds a
direction, exact under rotation and wrong under translation, and says so. It
is built and wired in, and **off** until a run shows it helps: the problem it
targets is measured, the cure is not, and a 1-in-3 detector needs a
range-anchored point, not a direction (R1).

### A model's distance guess is never a safety input

**Decision.** An optional brain-side veto stops a forward when the model says
the obstacle is within one step, only on a backend with no distance sensor,
and off by default. It exists to exercise the veto path on photograph-driven
backends; models disagree on obstacle presence from about 0% to about 100% on
the same frames, so it must never be copied onto the hardware backend.

### Explore's goals are places the robot can drive away from

**Decision (3.45, 2026-10-07).** A frontier or view goal keeps the chassis'
half-length plus the safety layer's stopping distance from anything the map
shows occupied. Stood there, at any heading, the straight move ahead and
the one behind each have the full bar.

**Why.** The safety layer, not nav2, decides whether the robot may move,
and it is stricter than nav2's inflated costmap. A goal nav2 can reach but
the safety layer will not let the robot leave is a trap: nav2 spends its
120 s goal timeout pushing against refusals, which is the whole parked
stall. On the furnished-home maps explore really had, the old 0.22 m
clearance chose goals that could be left at every heading only 36% of the
time on ground truth; 0.30 m reached 52%; the half-length plus 20 cm
reached 98%.

**What it costs.** Fewer stopping places in tight spots. A frontier is
seen from where the robot stops, and the lidar sees 360 degrees, so a goal
set back from a corner still clears it. Coverage is the progress check.
Approach points toward a seen target keep the old clearance: the arrival
rule judges them at the lidar's 0.40 m, and they are not left again.

## Contracts

| With | Direction | Category | Ownership |
|---|---|---|---|
| Mission runner ([mission](../mission/ARCHITECTURE.md)) | runner calls the agent's step; the agent calls the vision function, and at an arrival the identity confirmation, both through the runner's guard | in-process | runner owns budget, timeout, outcome; policy owns the action |
| Safety layer (safety domain) | agent submits each action | in-process controller over the gated body, re-vetted on the robot | the robot's veto is final; a refusal is a normal outcome |
| Body ([body](../body/ARCHITECTURE.md)) | agent reads frame, distance, depth, scan, odometry | in-process interface, HTTP underneath | body owns all readings; the policy never fabricates one |
| World ([world](../world/ARCHITECTURE.md)) | agent reads pose and map resolution | in-process interface, HTTP underneath | advisory: failure degrades exploration only |
| Perception ([perception](../perception/ARCHITECTURE.md)) | tier calls the pipeline once per frame | in-process: frame -> tri-state with bearing | perception owns what is seen; the policy owns what to do about it |
| Cloud vision ([cloud-vision](../cloud-vision/ARCHITECTURE.md)) | vision step calls the service; the tier also asks it on the arrival frame, reading only its visibility flag | HTTP/JSON | the service owns model, wording and the decision vocabulary |

## Failure modes and resilience targets

| Failure | Response | Target |
|---|---|---|
| Cloud call errors or hangs | raised to the mission's failure budget (an async failure on the next frame) | every blind step is counted, never swallowed |
| Cloud unreachable at arrival | the identity confirmation raises into the same budget and is retried while the arrival holds; when the budget runs out the mission ends `arrived_unconfirmed` | an offline mission that reaches the target says so, and never says `found` (3.47: 24 / 24 offline arrivals, sync and async) |
| Local perception unavailable (camera wedged, model error) | never a trigger, never advances the cold-search count, never read as absent | a dead camera never looks like an empty room |
| Detector misses frames | trigger hysteresis, held goals, degree-counted spin guard | at 90% per-frame detection, at least 95% of missions arrive (3.6b's bar) |
| Target directly on the straight line through a door jamb | refused forwards; the mission ends `blocked` | no policy spends its budget pushing into a wall; going around is nav2's job |
| No range sensor (phone walk, replay) | arrival not judged; under the shipped asynchronous tier nothing else can end the mission `found`, so it runs to its step budget (P7e) | a mission never ends `found` on a guess. By design since 2026-10-02: a landed cloud target-reached does not end it |
| An edge beside the target | arrival refused | zero false arrivals beyond 0.60 m |
| World unreachable | right-hand rule | a mapper outage never ends a mission |
| Local false positive | steers on every frame it passes the probability gate; one frame is enough; the cloud's answer does not override a local sighting | target: a false positive never ends a mission `found`. **Met** (the cloud confirms identity at arrival; decided and built 2026-10-02): a wrong object can be driven to, and the arrival is refused unless the cloud agrees. Steering on a false positive is accepted |
| Spin with no detection | forced forward after a fixed amount of rotation | a held turn cannot repeat forever |

## Open questions

- **The room-level planner** -- an LLM reasoning over mission memory to
  choose which room to search next -- is designed and not built; it is the
  main gap on the hardware path. Decided by the user; it has a slot (the
  vision function) and a prompt shape (mission memory's context block)
  waiting.
- **Dead-reckoned bearing**: stays off until a run both ways shows the run
  length of commands rise without losing arrival.
- **Arrival on real pixels**: needs the real detector to keep recognising the
  target at 40 cm (it relabelled a bottle as a vase); a rig walk on the robot
  answers it.
- **P7e's second half** (a held cloud STOP versus steering) no longer matters
  on the lidar path and is left alone.
- **Steering by verbs versus nav2 goals**: the tier keeps steering by verbs
  for now (R6). Revisit with continuous motion.
