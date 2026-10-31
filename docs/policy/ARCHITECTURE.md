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

### Privacy is a reason for the trigger discipline

**Decision** (the user, 2026-10-09; `PLAN-ros-alignment.md` 3.52). The robot
photographs the inside of a home, so **every frame that does not go to the
cloud is a frame of the house that never leaves it.** That is a reason for
the event-driven tier in its own right, beside cost and latency: a change that
sends more frames out (a new trigger, a lower staleness floor, a per-step
policy on the car) has to argue against it, not only against the bill.

What leaves the house today, traced to the code:

| What | Where it goes | When |
|---|---|---|
| A camera frame, the target string, searched rooms | the cloud `/navigate` route (Bedrock), from `brain/navigate.py` | tiered: only on a trigger (mission start, candidate sighting, cold search, staleness, arrival confirmation); the `vision` policy: every step |
| A phone frame | `/navigate` or `/guidance`, straight from the twin's Guide tab | every guidance step |
| Recorded walk frames and answers | the brain's walk store: local disk, or the private recordings bucket when `recording_backend` is `s3` | only while a walk is being recorded |
| The object inventory (3.46): labels and positions, no pixels | the recordings bucket under `inventory/`, when `INVENTORY_BUCKET` is set | at mission end |
| A metrics row: outcome, counts, timings, config; no pixels | the metrics route, when `metrics_url` is set | at mission end |

Nothing else sends pixels. `brain/vision.py`'s direct Anthropic call is
reached only from a manual test.

**Rejected, for now: blurring people before a frame leaves.** Measured in
3.52 on the shipped detectors and failed: at floor height the people the
corpus holds (one person lying on a sofa, 3-6 m off) were found on at most 7
of 43 frames, while the "person" boxes landed on furniture on 21% of
person-free frames, once squarely on the search target, which the blur then
hid from the cloud. A blur that misses people and erases targets protects
nothing and breaks the search. Revisit with a corpus of people as a floor
robot sees them and a detector that finds them; the section names the walk
that would close the gap.

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

**Amended 2026-10-23 (3.91.C, the user's choice): a "no" about a candidate
is remembered.** The steering half of the trade-off above had a measured
cost: on the 2026-10-21 phone walk a correct cloud "not the target" was
overridden on the next frame by a false local sighting of a lookalike, about
nine times, and with the local VLM first perception still steered at a bag
it had refused on 54 of 54 frames. Now an identity "no" -- the local VLM's,
where its box is a fair view of the object, or the cloud's -- is evidence
about that candidate, not a passing opinion: the candidate is neither
steered at nor asked about again until it changes (the body moved or turned;
on a body without odometry, the candidate was lost and found again, or a
set time passed, and then it is asked about before it is steered at again)
or a later answer says yes. The local VLM's "no" where it cannot tell (the
far view, fabric only) is not remembered; it asks the cloud, once a
candidate -- escalation on the tiers' disagreement, which 3.92 found catches
as many errors as a learned router.

**The tiers' order (the user, 2026-10-24).** Each tier is more capable than
the one below it -- YOLOE + CLIP < the local VLM < the cloud VLM -- and its
answer about a candidate is never undone by a lower tier's: only a cloud
"yes" lifts the cloud's "no", a cloud "yes" stands against a local "no",
and where the two VLMs disagree, or the local one cannot tell, the cloud is
asked. Perception acts alone only where neither VLM has spoken about the
candidate. Only the robot's own safety layer overrides every tier. The
order is a hard rule beside the behaviour trees, not a node in them, so no
reordered or tuned tree can break it.
**Rejected:** leaving the steer to perception and fixing only the paid
re-asks (the wrong bag is still approached); a learned router (3.92: AUROC
0.70). **Trade-off:** a true target refused once by mistake is not
approached until the robot moves or the hold ends; arrival's own checks are
unchanged.

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

**In the simulator, its geometry confirms instead** (3.98, the user
2026-10-24; `docs-review/SPEC-REVIEW-3.md` fix 7). A render is not a
photograph, and a paid call on a flat-shaded billboard tied every sim
outcome to how a model reads it. As the sim reports detections from its
geometry rather than run a detector on a render (1.12), it answers the
identity question from the same ground truth: the nearest object at the
centre of the arrival frame must be the target. A setting
(`sim_arrival_confirm: cloud`) keeps the real call for a run that means to
test it. Frames that are not the simulator's always go to the tiers below.
**Trade-off:** sim missions no longer exercise the real confirmation call
by default, so it is measured on purpose (Stage 1's lit-renderer re-run,
the car), not in passing.

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

### Explore's next goal may be learned; nothing else in explore is

**Decision (3.65 phase 2, 2026-10-11).** Which goal explore sends next may
come from a learned score instead of the hand-written order, behind
`explore_goal_policy` (default `rooms` since 3.125; `rule` before). The learning is reinforcement
learning in its evolutionary form: a linear score over ten facts explore
already has about each candidate, its weights found by the cross-entropy
method on simulated missions, each mission scored +1 if found, minus 0.02
per goal and 0.01 per metre.

**Why this decision and not the driving.** The misses 3.69 fixed were all
decisions; nav2 reaches every goal it is sent; and the simulator is
kinematic, so a learned controller would learn a world without slip. The
choice of goal is where better data changes the outcome, and it is scored
by a fast sweep against ground-truth reachability.

**Why evolutionary, not a gradient method.** Ten weights, a reward mostly
decided at the mission's end, and missions about a second each: scoring
whole missions avoids per-step credit assignment, and the result is ten
numbers anyone can read. A small network (and a gradient method such as
PPO to train it) is allowed only after the linear score fails its
criteria on record.

**What keeps it safe.** The policy only ranks candidates the rules would
send: unleavable, set-aside and twice-reached places are never offered.
Sightings, approaches, escapes, the retry book, the end of the search,
nav2 and `robot/safety.py` stay rules. A bad weight makes a search slower,
never a move unsafe. It is judged on houses it was not trained or selected
on, and live on nav2, before the default could change.

### Where to look is chosen from the map, never from a frame

**Decision (3.80.G and 3.80.I, 2026-10-16/17).** With the target out of
view, the next place to look comes from what the robot knows beyond the
current frame -- its map, its memory and common sense about rooms -- never
from a model asked "which way?" about one picture.

**Why: the answer is not in the picture.** 3.80.I asked every tier, local
and cloud, for a search direction on 42 out-of-view renders whose true
answer is the first metre of the shortest path to the target. The bar was
the majority side's share plus 15 points (58%). Nothing passed: the best
were RoboBrain2.0-3B 41% and Opus 4.5 38%, then GPT-6 Astra 29% and Opus
5.5 21%, all below always naming the more common side (43%); the local
Qwens said FORWARD on all 42. The mirror test shows why: flipped left to
right, the out-of-view answer swaps sides only 36-67% of the time, so it is
mostly a guess. With the target in view the same models steer well (Astra
56 / 60): seeing is solved, searching is not a vision question.

**The layers, each feeding the one below it:**

| Source of the answer | What it adds | State |
|---|---|---|
| **A map of what's unexplored** -- explore's frontier, view and approach goals over SLAM's map | the candidate places to go; coverage that eventually finds anything reachable | **built, the design** (3.80.G: arrivals 2 -> 16, cloud calls 765 -> 6) |
| **A learned choice among those frontiers** -- `explore_goal_policy` (3.65) | the order in which the frontier's candidates are visited | built; default `rooms` since 3.125 (the user, 2026-10-31: without room labels on 588 test houses, median goals -14% in mode R and -22% in U, finds within 1%, mean metres +10% in R; choice 132.9 ms p90 on the Jetson, bar 100). Before it, `rule` (3.65's network saved 11% of median goals on fresh houses, bar 20%; 3.96's room-feature network, `rooms`, saved 43% with room labels but, without them, nothing when the target is in a room type that suits it (20% when it is not), and a find mission has none until a survey's labels reach its map) |
| **Memory of where things were seen** -- the object inventory (3.46) | a candidate's distance to the last place the target, or something like it, was seen | built; empty on real frames until a namer for camera frames passes |
| **Common sense about rooms** -- one language-model call ranks room types for the target ("a charger: office, kitchen, bedroom") | a candidate's room prior | rooms from CLIP (3.51) built, not wired; the ranking not built |
| **What is visible toward a frontier** -- a model rates how promising each unexplored edge looks for this target (published object-goal navigation work) | a candidate's visual promise | not built |

**How reinforcement learning ties to the frontier.** The frontier map is
where the candidates come from; RL only decides their order. 3.65's policy
scores each candidate the rules allow (unleavable, set-aside and
twice-reached places are never offered) and explore sends the best one;
training rewards finding the target in fewer goals and metres. Today its
ten inputs are all geometric -- path length, turn, frontier length, view-gap
area, unknown and unseen share nearby, past failures and reaches nearby
(`brain/goal_policy.py` `FEATURES`) -- which is why it gained so little over the rule: from
geometry alone, one frontier looks much like another. The layers above are
new inputs to that same ranker: a room prior, an inventory distance, a
visual promise become columns beside the geometric facts, and training
learns how much each is worth. So the search improves without a new policy
or a new safety argument: the ranker still only orders places the rules
already allow, and a bad weight makes a search slower, never a move unsafe.

**Next, in order:** (1) the room prior as a ranker input, trained and judged
on the ProcTHOR sweeps (their imports carry room types), against today's
`rule` on goals and time to find, on houses it was not trained on; (2) the
inventory distance, once real frames name objects; (3) visual promise, only
if (1) and (2) leave a gap, since it costs a model call per frontier.

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

## The tiered ladder is a behavior tree

Since 3.94 the tiered policy's decisions are drawn as behavior trees: what
to do this frame, when to ask, and who to ask. Each is a few small nodes,
read in order. Each node has its own settings, and the tree prints itself.

**Why.** The ladder grew as nested code. Every new rule was an edit to a
long function, and its order of precedence could only be read off the
code. As a tree:
* a new behaviour is a new node;
* an order that may be tuned is a setting;
* the scenario simulator and a later learned node act on named nodes,
  not on lines.

**What the tree owns, and what it does not.**
* **It owns the choices among allowed actions:**
  * which evidence steers;
  * which event earns a question;
  * which tier is asked.
* **Hard rules stay outside or fixed:**
  * the safety veto;
  * a person's Stop;
  * the arrival rule;
  * "`found` needs the cloud";
  * B3's failsafes;
  * the order of who-to-ask's guards (the call cap, one call at a time
    and "mission ended" come before any question).

  No setting reaches these, so a later learned node can choose only
  among what they allow (the shield in
  `LEARNING-APPROACHES-2026-10-22.md`).

**Redrawn first, changed later.** 3.94 changed no behaviour. The new tree
was checked against the code it replaced on thousands of random states.
Changes of behaviour, such as 3.92's escalation on disagreement and 3.91.C's
rules, are new nodes in their own sections, each with its own bars.

**On a standard library (3.95).** The trees run on `py_trees`, the common
open-source Python behavior-tree library, rather than code of our own. The
richer trees still to come need what it already has: nodes that span frames,
decorators such as timeouts and retries, and drawings of the tree. It also
reads BehaviorTree.CPP's XML, the format nav2's trees are written in. The
move changed no decision.

## Open questions

- **The room-level planner** -- an LLM reasoning over mission memory to
  choose which room to search next -- is designed and not built; it is the
  main gap on the hardware path. Since 3.80.I it is the second-to-last layer of
  "where to look is chosen from the map" (above): a room prior fed to
  explore's goal ranker, not a policy of its own. Decided by the user; it has a slot (the
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
- **Learning explore's goal choice from a person** (raised 2026-10-20, not
  built). The ideas it rests on:
  * *Reinforcement versus imitation.* Reinforcement learning learns from a
    reward over many tries (3.65's +1 found, minus goals and metres).
    Imitation (behaviour cloning) learns "choose what the expert chose"
    from examples. A person at the controls gives examples, not a reward,
    so teleoperation feeds imitation.
  * *Learn at the level of the decision.* What is learned is the choice
    among the goals the rules allow, so the example worth recording is the
    person's pick among those same candidates (a tap on the map), not
    their D-pad verbs. Wheel commands do not map onto "which candidate",
    and the driving is nav2's.
  * *Warm start.* Cloned choices give a sensible starting policy; RL in
    simulation then improves past the expert instead of being capped at
    the person's skill.
  * *Intervention learning (DAgger and its human-gated form).* The robot
    runs on its own, a person corrects it when it goes wrong, and each
    correction is a training example. Efficient, because the corrections
    land exactly where the policy is weak.
  * *The feature gap.* A person may choose by cues the ten candidate facts
    cannot express (a hallway's look, where bins live); cloning through
    those facts loses exactly that, so the payoff is unknown until
    measured.
  * *How much data.* About 10-30 choices a mission: a few hundred taps can
    fit the ten-weight linear score, not the network.

  **Decided by the user (2026-10-20): corrections are goal taps, never
  takeovers.** A person's Stop and D-pad still end explore and keep the
  brain out (the safety domain), unchanged. A person instead taps another
  candidate while explore runs; explore sends that goal and logs the pick
  as a training example, so the correction needs no exception to the
  safety rule. It must be its own explicit message, not the person's map
  goal, which explore deliberately neither adopts nor cancels (3.76 E).
  Not built; no criteria yet.
- **Learning the tiered ladder** (raised 2026-10-22, after 3.91 B6; not
  built). The ladder's numbers are config (the `tier_*` keys), but its flow
  is code: who is asked, and what an answer does. B6 showed a local "no"
  changes no steering. The user asked whether the rules could be learned.
  [`LEARNING-APPROACHES-2026-10-22.md`](../evaluations/LEARNING-APPROACHES-2026-10-22.md)
  surveys twelve ways and orders them:
  * value of information over calibrated confidences;
  * then the ladder as a behaviour tree whose numbers are tuned black-box,
    against a simulator that emulates perception's reports calibrated on
    real frames;
  * then, on the Rover, conformal bounds on the local tier's unchecked
    errors, and logged randomised choices so that bandits and routers can be
    judged from logs;
  * end-to-end RL last, after a realistic simulator (evaluated and parked by
    the user 2026-10-22).

  The same rule as explore's goal choice holds: a learner only chooses among
  actions the rules allow. Safety, a person's Stop and the arrival guards are
  its shield.
