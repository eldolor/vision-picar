---
kind: architecture
domain: mission
status: current
verified: 2026-10-02
---

# Mission -- architecture

The mission service is "the brain as a service": the process that owns one
mission at a time, drives its decision loop a tick at a time, and guarantees
that a mission which ends -- for any reason -- leaves the robot stopped. It
does not decide moves (that is the [policy](../policy/ARCHITECTURE.md)
domain) and it does not vet them (that is the safety layer on the robot).
Read this to understand why the brain is shaped the way it is; read the
[engineering spec](../engineering/mission/ENGINEERING.md) for routes,
config keys, timeouts and the tests that pin them.

## Purpose

Without this component the autonomy loop would be a blocking script: nothing
outside it could start it, stop it, or ask what it is doing, and a phone tab
going to sleep would halt the robot. The mission service turns the loop
inside out so that:

- a mission can be started, stopped and inspected over the network, and
  survives the request (and the browser tab) that started it;
- the three ways an autonomous loop goes wrong on a real robot -- it goes
  blind, it hangs, or its process dies with the motors energised -- each have
  a guard that ends in the same place, a stopped robot;
- the brain can run on the robot's onboard computer or on a laptop across
  the room with nothing changing but one address.

Its dependants are the twin (which starts and observes missions), the
operations health check (which reads its liveness), and the recordings
tooling (which stores what a mission saw beside what it decided).

## Components and boundaries

```text
   twin / curl / health check
            |  HTTP/JSON
            v
  +---------------------- brain process ----------------------+
  |  service front     one mission slot, start-time validation,|
  |                    the hung-tick dead-man, drills          |
  |        |                                                   |
  |        v                                                   |
  |  mission runner    lifecycle, outcomes, vision-failure     |
  |                    budget, halt gate, status snapshot      |
  |        |  in-process "frame -> scene" seam                 |
  |        v                                                   |
  |  policy (agent + vision function)   -- policy domain       |
  |        |                                                   |
  |  body client  ---- HTTP ---->  robot server (control-api)  |
  |  world client ---- HTTP ---->  world routes (world)        |
  +------------------------------------------------------------+
```

| Part | Owns | May not |
|---|---|---|
| Service front | the single mission slot, request validation, the per-tick deadline, the drill surface | run decision logic; keep a second mission alive |
| Mission runner | one mission's lifecycle and outcome, the vision-failure budget, the halt gate, the status snapshot | decide a move; compute perception; outlive its mission (one runner, one mission) |
| Halt gate | refusing every movement and camera-pan command once the mission is over | block a stop or a sensor read |
| Brain config reader | the brain's block of the shared config file | import the robot runtime to read it |
| Drills | injecting exactly one fault into an otherwise ordinary mission | make the robot move |

**The boundary that matters most:** the mission package may not import a
robot backend, the simulator, the world factory or the robot server. It
reaches the robot only as an HTTP client, through the body contract
([body](../body/ARCHITECTURE.md)) and the world contract
([world](../world/ARCHITECTURE.md)). The rule is enforced by an automated
test, not by review (the mechanism is in the
[engineering spec](../engineering/mission/ENGINEERING.md)). If the brain can
reach a backend directly, "run the brain on the robot" stops being a
configuration change.

## Decisions

### The brain and the robot are two processes

**Decision.** The brain runs as its own service beside the robot server,
even when both run on the robot, and talks to it over localhost HTTP.

**Rejected.** Running the loop as a task inside the robot server: less code.
It was rejected for three reasons (`PLAN-brain-relocation.md`, "Why not one
process"): a synchronous block in the agent would block the event loop the
robot's watchdog polls on, so the one guard for a dead brain would die with
it; it merges the robot runtime with the decision-maker, the separation the
project has kept since Phase 0; and it loses the property that moving the
brain is a one-address change.

**Trade-off accepted.** A localhost round trip per robot call, against a
loop that spends seconds waiting on vision.

### The loop is driven from outside, one tick at a time

**Decision.** A mission exposes start, tick, stop and status; the service
drives ticks in the background. No decision logic moved when this was built
(B1), and a tick-driven mission was pinned to take the same steps as the old
blocking loop.

**Rejected.** Keeping the blocking loop and wrapping it: it cannot answer
"what are you doing" mid-mission or be stopped between steps.

### Three failsafes, kept separate

**Decision.** Three distinct failures get three distinct guards
(`PLAN-brain-relocation.md` B3, `AGENT-HARNESS.md` section 6):

| Guard | Failure it sees | Where it lives |
|---|---|---|
| B3.1 watchdog | motors energised by a process that then died | the robot (safety domain) |
| B3.2 vision budget | the car is blind: vision calls error or hang | the mission runner |
| B3.3 dead-man | the loop is alive but stuck | the service front |

**Rejected.** Collapsing them into one timeout. The watchdog cannot see a
brain that is alive but stuck (it keeps the event loop healthy while a tick
hangs on a worker thread); neither brain-side guard can see motors left
running by a crash. **Rejected also:** retrying a failed vision call inside
the policy. The failure budget *is* the retry policy, and a silent retry
while the robot keeps moving is the behaviour B3.2 exists to prevent.

### Stopping is enforced at the robot boundary, not only in the loop

**Decision.** The agent drives the robot through a halt gate that refuses
movement and pan commands the moment the mission ends, while always passing
stop and sensor reads. A blocking call on another thread cannot be
interrupted, so the gate is what makes "stop stops the car" true for a tick
already in flight. The residue -- a command dispatched microseconds before
the stop -- is the watchdog's job.

**The gate must be transparent to safety.** Whatever the safety layer needs
from the body to vet a move, the gate passes through. A gate that hides it
makes in-process missions run a weaker safety path than the car, and every
sweep measured through it would then overstate the car's safety
(`PLAN-ros-alignment.md` 3.32 is the record of that defect and its fix).

### One mission per brain; a second start is refused, not queued

**Decision.** The service holds one mission slot. A second start while one
runs is refused with a conflict. A person at the D-pad still outranks the
brain, but that is enforced by the robot (safety domain), and a mission that
loses the robot ends **preempted**.

**Rejected.** Queueing or replacing: two loops driving one robot is exactly
the failure the service exists to prevent, and a silent replace hides it.

### Refuse at start what would otherwise fail mid-mission

**Decision.** Everything that can be known before the first move is checked
at start: the policy needs a vision service and an object target; a
requested cloud model or prompt wording must be on the vision service's
published allow-list; a tiered mission must be able to load its local
models. Each is a refusal naming the cause, before a motor turns.

**Rejected.** Discovering these on the first tick, where they are counted as
vision failures, so the mission limps through its budget and dies reporting
"vision unavailable" -- the wrong diagnosis, after a robot has started
moving.

**Trade-off.** An *unreachable* allow-list endpoint is not a refusal: an
unknown value is a caller error, an outage is a soft problem, and refusing
to start over a failed validation call would turn one into the other.

### Outcomes say what happened, not just that it stopped

**Decision.** A mission ends in exactly one of: found, room_reached,
max_steps, blocked, searched, arrived_unconfirmed, stopped, preempted,
failed. The first outcome wins and later stops cannot rewrite it. Three are
deliberately not "failed":

- **preempted** -- a person took the robot. Filing a normal intervention
  beside a dead cloud link would invite a retry where retrying is wrong.
- **blocked** -- the policy kept asking to drive forward and the safety
  layer kept refusing (R1b, `PLAN-ros-alignment.md` 3.4). Nothing broke;
  going around is route planning, and nav2 reports an unreachable goal the
  same way.
- **arrived_unconfirmed** -- the vision budget ran out while the robot was
  parked at an arrival whose identity the cloud never answered (3.47). Not
  "found" either: only a cloud yes makes "found"
  ([policy](../policy/ARCHITECTURE.md), "The cloud confirms identity at
  arrival").

### An unconfirmed arrival is asked again when the cloud is back

**Decision** (3.53, approved by the user 2026-10-09). After an
`arrived_unconfirmed` ending the brain keeps the frame the arrival was
judged on, probes the cloud's free health route, and when it answers asks
the identity question ONCE, recording the answer beside the outcome
(`late_confirmation`), never instead of it. The outcome is not rewritten:
the first outcome wins, and a late yes is evidence for a person to read,
not a `found` the mission earned on its own terms. It is bounded (a probe
every 15 s for ten minutes, two paid attempts at most), it never moves or
stops the robot (the mission is over; a stop now would halt whoever drives
next), a new mission drops it, and a brain restart loses it. A Stop after
the ending does not drop it: it moves nothing, and the phone sends Stop on
every close.

**Rejected:** rewriting the outcome to `found` on a late yes (breaks "first
outcome wins", and the robot may have been moved since); keeping the
mission running until the cloud answers (holds the robot and the mission
slot for an unbounded time); probing with a paid `/navigate` call (a free
route exists).

### A mission starts with the camera centred

**Decision.** The first tick centres the camera through the halt gate,
before the first decision, uncounted as a step (3.20). A mission that ended
mid-peek otherwise left the next mission's first frame, depth grid and scene
cast sideways.

**Rejected.** Centring at the end of the previous mission: by then the brain
may no longer hold authority over the robot.

### The world is advisory; the body is load-bearing

**Decision.** If the world client cannot be built or cannot answer, the
mission proceeds without a map (the frontier policy falls back to its
right-hand rule). If the body cannot answer, the step fails and the mission
ends **failed** with the robot stopped. Odometry for pacing is attached to
each frame on a best-effort basis for the same reason.

### Offline is a mode, not a failure

**Decision** (3.49, asked by the user 2026-10-09). With the cloud
unreachable -- Wi-Fi down, the tunnel gone, a refused or hanging connection
-- these keep working, each pinned by a test with every socket connect
refused (mission start with an unreachable model allow-list is pinned only
by 3.49's live run, criterion 7):

- the safety veto and the watchdog: the robot server imports no cloud
  client;
- the tiered policy's local search, steering and arrival, ending
  **arrived_unconfirmed** at the target rather than **found** or **failed**;
- the twin, served by the robot server on the LAN, loading nothing from the
  internet;
- mission start (an unreachable model allow-list is tolerated) and
  `/health`.

On the car, offline means the tiered policy: `frontier` and `explore` never
call the cloud, but their vision step is the rule-based one, which reads the
simulator. The `vision` policy is cloud-only and ends **failed** at once.

**Rejected.** Falling back to the rule-based explorer when the cloud drops
(`PLAN-onboard-perception.md` 2.5): on the car it has no detector, and the
tiered policy already searches without the cloud.

**Trade-off.** Offline, no mission ends **found**: identity is the cloud's
question. A hanging cloud costs up to the failure budget times the vision
timeout of parked time at arrival (60 s at the shipped 20 s).

### Fault drills exercise the real guards

**Decision.** A mission may be started with one named fault (vision errors,
vision hangs, the loop hangs) on shortened deadlines, so B3.2 and B3.3 can be
watched firing. With no fault the drill path builds the same object as the
production path. Drills are fail-safe by construction -- each can only end a
mission with the robot stopped -- and can be switched off for a brain
reachable beyond the LAN.

### Hand-rolled, not an agent framework

**Decision.** The loop is project code. The hard parts -- a stop that beats
an in-flight action, a veto in another process, a dead-man for a live but
stuck loop -- are physical, and no orchestration framework has opinions
about them (`AGENT-HARNESS.md` section 11). If the policy ever grows real
tools, a tool runner belongs inside the vision function, below the runner.

## Contracts

| With | Direction | Category | Ownership |
|---|---|---|---|
| Twin, scripts | they call the brain | HTTP/JSON: start, stop, status | the brain owns mission state; callers only read it or ask for a transition |
| Robot body ([body](../body/ARCHITECTURE.md), [control-api](../control-api/ARCHITECTURE.md)) | brain calls robot | HTTP/JSON through the body client, naming itself as the brain driver | the robot owns all body state and the final veto; the brain never assumes a move happened |
| World ([world](../world/ARCHITECTURE.md)) | brain calls world | HTTP/JSON through the world client | the world owns pose and map; the mission treats them as advisory |
| Policy ([policy](../policy/ARCHITECTURE.md)) | runner calls agent; agent calls the vision function | in-process: a frame goes in, a scene comes out | the policy owns the decision; the runner owns the budget, timeout and outcome |
| Cloud vision ([cloud-vision](../cloud-vision/ARCHITECTURE.md)) | brain calls service | HTTP/JSON, at start (allow-lists) and through the policy (decisions) | the vision service owns the model and wording allow-lists; the brain never keeps a copy |
| Operations | health check calls brain | HTTP/JSON | the brain reports liveness facts; operations alone turns them into a verdict |
| Recordings | twin calls brain | HTTP/JSON routes the brain mounts but does not own | the recordings domain owns the routes and storage |

## Failure modes and resilience targets

| Failure | What the component does | Target |
|---|---|---|
| A vision call errors or hangs | stops the robot on that step and counts it; a success resets the count | every blind step stops the car; a short run of consecutive failures ends the mission **failed** with the robot stopped, or **arrived_unconfirmed** if the run is an arrival's identity check that never got an answer |
| A tick never returns | the service abandons the tick thread, stops the robot and ends the mission | a stuck loop ends **failed** within one tick deadline, robot stopped, while status and health keep answering |
| The brain process dies mid-move | nothing here can act | the robot's watchdog stops the motors within one watchdog period (safety domain) |
| Stop lands while a tick is in flight | the halt gate refuses that tick's move or pan | after a mission ends, no further movement command from it reaches the robot |
| A person takes the robot | the mission ends **preempted**; it stops the car, not retries | a preempted brain never re-takes the robot |
| The safety layer keeps refusing forward | the mission ends **blocked** after a short run of refusals | no mission spends its budget pushing into a wall |
| A body read or move raises anything else | the mission ends **failed** with the robot stopped | the brain never loops blind on a dead robot link |
| World unreachable | missions run without a map | a mapper outage costs exploration efficiency, never a mission |
| Bad model, wording, missing models or vision service | refused at start with the cause | no misconfiguration is discovered after a motor turns |
| A late async cloud answer arrives after the mission ends | the policy is closed on finish, so the answer is dropped | no decision from an ended mission is applied |
| The stop command itself fails | logged and swallowed | a failsafe never raises |
| Brain restart | the mission is lost; the robot keeps its own state | accepted gap, see below |

## Open questions

- **Boot-time start on the robot (B5).** The units that start the robot
  server, the brain and the ROS container at boot are not written; the brain's
  home moved from a Pi to the Jetson on 2026-09-19. Decided by the user on
  hardware day; settled when a mission starts from a phone after a reboot with
  no laptop on the network.
- **Degraded mode when the cloud is unreachable** is decided for the
  tiered policy (see "Offline is a mode, not a failure"). Still open: whether
  status should say "cloud unreachable" before a mission fails or parks, and
  whether a parked **arrived_unconfirmed** robot re-asks the cloud when the
  link returns.
- **Mission persistence.** A brain restart loses the mission. Worth revisiting
  only if a field run shows restarts happen.
- **Chaos and soak (S7).** Added latency, dropped requests and a 1000-step run
  are planned, not run.
