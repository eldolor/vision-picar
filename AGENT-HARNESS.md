# The agent harness

How `control/` works: the loop that runs a mission, what it guarantees,
where to plug things into it, and what not to break.

Written 2026-08-27, when phases B0-B4 of `PLAN-brain-relocation.md` were
built. That document says *what to build and why*; this one says *how the
built thing works*. `README.md` has the per-phase engineering detail,
`INTRODUCTION.md` the plain-language version.

**What kind of document this is.** Sections 2-11 describe the
*architecture* -- the contracts, the control flow, the concurrency model,
the invariants. That content is **both the current state and the target
state**, deliberately: the harness was built so the vision policy can
drop into it without any of it changing. It should survive S2, S2b and
the hardware swap intact.

**Section 1 holds everything that expires** -- which policy is currently
plugged in, what is stubbed, what is missing. It is the only section that
goes out of date on a normal week, and it carries its own checklist of
what to update when each phase lands. If you are reading this after a
phase landed and section 1 still describes the old world, that section is
wrong and the rest probably is not.

The reason for the split: a reference that describes a target lies about
the system you are actually debugging, for the whole stretch before the
target arrives. The plan documents own target state (`PLAN-*.md`); this
one owns behaviour. Quarantining the perishable facts in one place is
what keeps the other ten sections from rotting with them.

---

## 1. Where this is on the road  *(the perishable section)*

Updated 2026-08-27, when the vision policy landed for recorded walks.

The word "agent" covers two things. Both now exist in Python; what
separates them is which backends they can run against.

| | Status |
|---|---|
| **The harness** -- goal, memory, perceive-decide-act loop, tool surface, guardrails, lifecycle | **Built** (`control/`) |
| **The intelligence** -- a model looking at a frame and choosing the move | **Built** (`brain/navigate.py` + `brain/vision_agent.py`), and runnable **only against frames that carry pixels** |

### The two policies

| `policy=` | Decides with | Runs against | Costs |
|---|---|---|---|
| `"frontier"` (default) | rule-based exploration: frontier-preference where the frame carries grid coordinates, a plain wall-follower where it does not (§3 step 5) | anything | nothing |
| `"vision"` | `/navigate` -- one model call per step, one action back | backends whose `get_camera_frame()` returns `image_base64`: `sim/replay_robot.py` today, hardware later. **Not `MockRobot`**, which has no pixels until phase S2 | one paid call per step |

`POST /mission/start` with `policy: "vision"` needs `brain.vision_url` and
a `target_object`; both are checked at start time rather than failing on
the first paid call.

### What still isn't there

- **The vision policy cannot drive the simulator.** `MockRobot` returns a
  grid description. Phase S2 (the raycaster ported into Python) is what
  changes that; until then the sim runs the rule-based policy and real
  pixels come from a recorded walk.
- **No room-level step memory.** A photograph carries no room label, so
  `MissionMemory.visited_rooms` stays empty under the vision policy and
  nothing stops it re-searching a room but the step cap. This is the open
  half of S2b.
- **`as_context()` still feeds nothing.** §12.
- **A replay is open loop.** It measures memory, lifecycle and cost
  honestly; it does not measure navigation. `sim/replay_robot.py` has the
  full caveat, including why the safety layer is inert there.

### What expires, and when

Everything above is a statement about today. When a phase lands, update
this section first, then the specific paragraphs it names -- nothing else
in this document should need to move.

| Phase | What becomes untrue | Update |
|---|---|---|
| **S2** -- real image bytes | `MockRobot` has no pixels, so the vision policy cannot drive the sim | §1's policy table; §3 step 3 |
| **S2b remainder** -- room memory | "nothing stops it re-searching a room" | §1; §12's room-memory gap |
| **Phase 11** -- hardware | `HardwareRobot` does not exist | §5's body-seam row |
| **B5** -- systemd on the Pi | The brain runs wherever you start it | §2's diagram caption |
| any | A gap in §12 gets closed | §12 -- it is the only other section that dates |

Two facts elsewhere are also perishable and deliberately kept out of the
architecture sections: the line counts in §2's table, and the "83 steps"
in §9's invariant 5. Both are illustrations, not contracts -- if they
drift, correct them, but nothing depends on them.

---

## 2. Anatomy

```
  ┌─────────────────────────────────────────────────────────────┐
  │  :8001  control/brain_server.py        THE MIND             │
  │         FastAPI. Owns one MissionRunner, drives it as an    │
  │         asyncio task. Failsafe B3.3 (hung tick) lives here. │
  │                                                             │
  │    control/mission_runner.py                                │
  │      start() / tick() / stop() / status(), the halt gate,   │
  │      failsafe B3.2 (vision timeout + failure budget)        │
  │         │                                                   │
  │         ├── brain/agent.py:ObjectSearchAgent   decides      │
  │         ├── brain/memory.py:MissionMemory      remembers    │
  │         └── vision_fn(frame) -> scene           perceives   │
  │                                                             │
  │    control/remote_robot.py  RobotInterface over HTTP        │
  └──────────────────────────┬──────────────────────────────────┘
                             │  HTTP -- localhost on the Pi,
                             │  LAN from a MacBook. This is the
                             │  ONLY coupling between the two.
  ┌──────────────────────────▼──────────────────────────────────┐
  │  :8000  robot/server.py                 THE BODY            │
  │         safety veto (authoritative), watchdog (B3.1),       │
  │         backend chosen by config/robot.yaml's `mode`        │
  └─────────────────────────────────────────────────────────────┘
```

| File | Owns | Code lines |
|---|---|---|
| `control/mission_runner.py` | one mission's lifecycle, the halt gate, the vision timeout and failure budget | 257 |
| `control/brain_server.py` | HTTP surface, the asyncio drive loop, the per-tick dead-man | 137 |
| `control/remote_robot.py` | `RobotInterface` over HTTP, transparently | 71 |
| `control/drills.py` | fault injection, so the failsafes can be shown and not only tested | 57 |
| `control/brain_config.py` | the `brain:` block of `config/robot.yaml` | 26 |

548 lines total. `control/` imports **no backend, no simulator, and not
`robot/server.py`** -- only `robot/interface.py` and the
`SafetyViolation` type that is part of that contract.
`tests/test_brain_server.py` asserts this in a subprocess.

---

## 3. One tick, in order

`MissionRunner.tick()` advances the mission by exactly one agent step and
returns `True` while the mission is still running, so a caller can write
`while runner.tick(): pass`. What happens inside:

1. **Refuse if not running.** A stopped runner never steps again.
2. **Check the step budget.** At `max_steps`, finish with outcome
   `max_steps`.
3. **Perceive.** `agent.step()` calls `get_camera_frame()` -- an HTTP
   `GET /frame` when the robot is remote.
4. **Interpret.** The frame goes to `vision_fn`, wrapped in
   `_guarded_vision`: a per-call timeout enforced on a **daemon thread**
   (section 7). A raise or a timeout becomes `VisionUnavailable`. Under
   the vision policy this is the paid `/navigate` call
   (`brain/navigate.py`); under the rule-based one it is an offline
   converter that makes no call at all.
5. **Decide.** The policy runs. *Which* policy is a §1 question; the
   harness calls whatever agent class the `policy` selected.

   `VisionAgent.decide()` is one sentence long -- trust the model's
   action, unless the mission is already over -- because the validation
   and the stuck-breaker are inherited. It does **not** peek: against a
   camera, three pans and three distance reads per decision cost real
   moves and buy nothing the photograph does not already show.

   `ObjectSearchAgent.decide()` picks one action in this order of
   precedence:

   - mission complete -> `STOP`;
   - a queued look-around scan -> its next pan. The scan is queued on
     first entry to a room, and **only when the mission has a
     `target_object`** -- a pure "go to the kitchen" mission has nothing
     to scan for;
   - the step right after an executed `LEFT`/`RIGHT` -> `FORWARD`,
     committing to the cell the turn just confirmed was clear, instead of
     re-peeking and spinning in place;
   - otherwise: peek right, left and centre (three real pan actions and
     three distance reads), then choose.

   That last choice has **two branches, and which one runs depends on
   what the frame carries.** With grid `position` and `facing` -- sim
   only -- it prefers a clear direction leading to an unvisited cell, and
   backtracks when everything nearby is visited. Without them, which is
   what a real camera frame will look like, it degrades to a plain
   right-hand rule: right if clear, else forward, else left. Boxed in on
   all three sides, it falls back to trusting the scene's
   `safest_direction`, with a forced `RIGHT` after three consecutive
   `STOP`s so it cannot freeze against a wall forever.

   **The degradation matters for the hardware path.** The
   frontier-preference behaviour that the demos and tests exercise is the
   grid branch. On real frames this policy is a wall-follower.
   `PLAN-sim-hardening.md` 2.2 is why that is acceptable: the rule-based
   policy is not on the hardware path, it is the free deterministic way
   to test the safety layer and the mission memory.
6. **Check safety, brain-side.** The agent's own `SafetyController`
   re-reads the distance sensor before a `FORWARD`. This is an early out,
   not the authority.
7. **Act, through the gate.** The action goes to `_HaltGate`, which
   refuses movement if the mission has already ended (section 6), then to
   the robot -- `POST /action` when remote.
8. **Check safety, robot-side.** `robot/server.py` runs the same check
   again against its own sensor read, and can veto. Over HTTP a veto
   comes back as `200 {"executed": false}`; `RemoteRobot` re-raises it as
   `SafetyViolation` so the agent cannot tell local from remote.
9. **Record.** The step goes into `MissionMemory` -- rooms visited, rooms
   searched, sightings, action history -- and into the log tail.
10. **Check for completion.** `MissionMemory.is_complete()` is true once
    the target object is sighted or the target room reached.

Steps 6 and 8 are the same check run twice on purpose. The brain's copy
saves a round trip; **the robot's copy is the one that counts**, because
it is the only one a compromised or buggy brain cannot skip.

---

## 4. Lifecycle and outcomes

```
   idle ──start()──> running ──┬── found          target sighted
                               ├── room_reached   target room entered
                               ├── max_steps      budget exhausted
                               ├── stopped        stop() -- an operator
                               └── failed         abort(), a vision budget
                                                  blown, or a step raised
```

Every terminal transition goes through `_finish()`, which does three
things in order: mark the runner not-running (so the halt gate closes),
record the outcome and reason, and **stop the robot**. `_safe_stop()`
swallows and logs an exception from that stop -- a failsafe that raises is
not a failsafe.

`_finish()` is idempotent. The first outcome wins; a later `stop()` or
`abort()` on a finished mission is a no-op, so a race between an operator
stop and a natural completion cannot rewrite history.

**One runner runs one mission.** `start()` on a used runner raises. The
brain server builds a fresh one per `POST /mission/start`.

---

## 5. The seams

Four places designed to have something else plugged into them:

| Seam | Type | What it is for |
|---|---|---|
| `vision_fn(frame) -> scene` | callable | **The policy seam.** Swap in the LLM. Section 10. |
| `RobotInterface` | ABC | **The body seam.** `MockRobot` and `RemoteRobot` today; `HardwareRobot` *(does not exist -- Phase 11)*. The runner never touches a backend. |
| `policy=` | string | **The decision seam.** `"frontier"` or `"vision"`; which backends each can run against is a §1 question. |
| `robot_factory` / `runner_factory` on `create_app()` | callables | **The test seam.** How the drills, the recording stub, and the two-hop tests inject what they need without the production path knowing. |

The `vision_fn` contract is `brain/vision.py`'s schema, and both existing
implementations honour it:

```python
{
  "obstacles_ahead": [str],
  "free_space": "none" | "some" | "clear",
  "doorway_visible": bool,
  "important_objects": [str],     # how a target gets "found"
  "safest_direction": "FORWARD" | "LEFT" | "RIGHT" | "STOP",
}
```

`important_objects` is load-bearing: `MissionMemory.record_observation()`
matches the mission's `target_object` against it, and that match is what
ends a mission successfully.

---

## 6. The failsafes

Three distinct failures, three distinct guards, one shared ending: the
robot is told to stop.

| | Failure | Guard | Lives in |
|---|---|---|---|
| **B3.1** | A move energises the motors, then the process dies before stopping them | Watchdog -- no `/action` or `/stop` for `watchdog_timeout_s`, motors stop | `robot/server.py` |
| **B3.2** | The car goes blind -- vision errors, or never answers | Per-call timeout plus a consecutive-failure budget (default 3). Every blind step stops the car; a run of them ends the mission | `mission_runner.py` |
| **B3.3** | The loop is alive but stuck | Per-tick dead-man. A tick that overruns `tick_timeout_s` triggers `abort()` | `brain_server.py` |

They do not substitute for each other. The watchdog cannot see a loop
that is stuck but alive. The brain-side guards cannot see motors left
running by a crash. And **sensing reads do not feed the watchdog** --
only `/action` and `/stop` do -- so the twin polling `/frame` while it
observes a mission cannot hold B3.1 off.

**The halt gate.** A `tick()` already in flight when `stop()` lands
cannot be interrupted; it is a blocking call on another thread. So the
agent drives the robot through `_HaltGate`, which refuses movement and
pan commands the moment the mission ends, while always passing `stop()`
and sensor reads through. Without it, an operator's stop can be followed
by the move the last tick had already decided on. A command dispatched in
the microseconds *before* the stop is still possible; that residue is
what B3.1 is for.

**Drills** (`control/drills.py`). B3.2 and B3.3 cannot be provoked by
pressing anything, so `POST /mission/start` accepts a `fault`:
`vision_error`, `vision_hang`, `tick_hang`. Each breaks exactly one thing
and leaves every other guard standing, so what gets watched is the real
guard firing. Two rules hold: a drill can only ever end with the robot
stopped, and with no fault the drill path builds the *same object* the
production path does. `brain.allow_drills: false` removes the surface.

---

## 7. Concurrency: what runs where

Three threads matter, and knowing which is which explains most of the
code's shape:

| Thread | Runs | Why |
|---|---|---|
| The asyncio event loop | `brain_server`'s drive loop, all HTTP handling | `POST /mission/status` must answer while a mission runs |
| A worker thread | `runner.tick()`, via `asyncio.to_thread` | The tick is blocking (HTTP calls to the robot). Running it on the loop would freeze `/mission/status` -- and the dead-man that is timing it |
| A daemon thread | one `vision_fn` call, via `call_with_timeout` | A hung call must be abandonable. Daemon, so it cannot block interpreter exit; a fresh thread per call, so it cannot sit in a pool ahead of the next one |

Consequences worth knowing:

- **A timed-out call is abandoned, not killed.** A blocking socket read
  cannot be interrupted from outside. The failure budget bounds how many
  can accumulate: three, then the mission ends.
- **The dead-man cannot stop the tick it is timing** -- same reason. It
  stops the *robot* and ends the mission on its own thread, and the
  abandoned tick finds the halt gate closed when it comes back.
- **`MissionRunner` guards its state with an `RLock`**, taken for status
  snapshots and outcome transitions. It is deliberately *not* held across
  `robot.stop()`, which is a network call when the robot is remote.

---

## 8. The status contract

`GET /mission/status` returns `MissionRunner.status()` plus the requested
`fault`. Every field:

| Field | Meaning |
|---|---|
| `running` | is a mission in flight |
| `outcome` | `idle` / `running` / `found` / `room_reached` / `stopped` / `max_steps` / `failed` |
| `error` | why, when `outcome` is `failed`; `null` otherwise |
| `policy`, `mission`, `target_object`, `target_room` | what was asked for |
| `step`, `max_steps` | progress against the budget |
| `found`, `room_reached`, `complete` | the mission's own completion flags |
| `last_action`, `last_reasoning` | the most recent move and why |
| `rooms_visited`, `rooms_searched` | sorted, from `MissionMemory` |
| `vision_failures` | the consecutive-failure count, reset by any success |
| `sighting` | step, object, room, position -- when found |
| `log_tail` | the last 20 log lines |
| `fault` | which drill, or `none` (added by the server, not the runner) |

`last_reasoning` is an observation today ("hallway, facing N, free space
clear, sees red backpack"), because the rule-based policy has no
rationale to report. An LLM policy should put its actual reasoning there;
the twin already renders that field.

---

## 9. Invariants -- do not break these

1. **Safety is enforced robot-side, always.** Add no movement path that
   reaches the robot without passing `robot/safety.py`. The brain's own
   check is a convenience.
2. **`control/` imports no backend and no simulator.** It talks to
   `robot/interface.py` and `RemoteRobot`. If the brain can reach a
   backend directly, "run the brain on the Pi" stops being a config
   change. There is a test.
3. **Stopping the mission stops the car.** Every terminal path calls
   `robot.stop()`, and the halt gate keeps a late tick from undoing it.
4. **One brain drives at a time.** The server 409s a second
   `/mission/start`; the twin refuses to start its local loop during a
   remote mission.
5. **`RemoteRobot` stays transparent.** A safety veto must raise
   `SafetyViolation` exactly as in-process, and `position` must survive
   JSON as a tuple. `tests/test_remote_robot.py` asserts an identical
   83-step action sequence in-process and over a live socket; if that
   test starts failing, the HTTP boundary has stopped being invisible.
6. **Drills are fail-safe.** A new fault may only ever end a mission with
   the robot stopped. Never add one that makes the robot move.
7. **A capability ships with something to press.** `CLAUDE.md` section 7.

---

## 10. The vision policy, as built (Phase S2b, partial)

It is one function plus a four-line class, because the harness already
provided everything else.

**`brain/navigate.py`** is the `vision_fn`: frame in, `/navigate` call,
scene out. `vision_fn_for(target, vision_url, secret)` binds the target
and endpoint into the one-argument callable the harness wants. One mapping
decision in there is load-bearing and worth knowing:

> **`important_objects` carries the target only when `target_reached` is
> true, not when it is merely visible.** `MissionMemory` treats a match
> there as "found", and found ends the mission -- so completing on first
> sight would stop the robot in a doorway across the room from the
> backpack and call that success. `/navigate` has a separate arrival
> signal precisely so `STOP`-because-blocked and `STOP`-because-arrived
> stay distinguishable; Robot view already pauses on it, and this uses the
> same signal for the same reason.

**`brain/vision_agent.py`** is the policy: trust the model's action unless
the mission is over. Everything else -- memory recording, action
validation, the stuck-breaker -- is inherited from `MissionAgent` and
`ConstrainedAgent`.

What the harness supplies, and what you must therefore *not* add:

- **The retry policy is the failure budget.** Do not retry inside
  `vision_fn`. Retrying a failing vision call while the robot keeps moving
  is the browser autopilot's bug, and §6 exists to prevent it.
- **The step cap is the cost cap**, since every step is a paid call.
  `tick_interval_s` is the throttle.

### Running it

Recorded walks are the backend that works today:

```bash
python -m tests.demo_replay_mission recordings/<walk> "red backpack"
```

Record one from the twin: Guide tab -> Robot view -> "Record this walk"
(needs the brain connected). Each frame is POSTed to
`/recording/frame` and lands in `brain.recording_dir`, with its live
`/navigate` answer appended to `walk.jsonl` beside it -- so a replayed run
can be compared against what the service said at the time, on the same
pixels.

### What is left of S2b

Room-level step memory. A photograph carries no room label, so nothing but
the step cap stops the vision policy re-searching. Closing it needs a room
signal in the frame -- the `/analyze` route's room guess, or
`identify_room()` over a caption -- not a change to the policy.

---

## 11. Why this is hand-rolled

A reasonable question, since agent frameworks exist. The short version:

**The loop is not the hard part.** One model call per step, returning one
action out of eight. No tool selection, no conversation, no branching
plan. As a graph it is a single node with a self-edge. The hard parts --
a stop that beats an in-flight action, a safety veto in another process,
a dead-man for a loop that is alive but stuck, a failure budget on
perception -- are physical, and no orchestration framework has opinions
about them.

**Hosted-agent platforms are structurally wrong here.** The surface where
Anthropic runs the loop *and* hosts a sandbox for tool execution is a
good fit for agents whose tools are bash and files. Our tools are motors
in a hallway: execution has to happen on the Pi, behind the safety layer.
(It is also unavailable on Bedrock, where this project's vision calls
run.)

**Cost of error.** Errors here are physical and not cheaply reversible,
which argues for the least agentic thing that does the job -- a
constrained action set, one move at a time, re-evaluated every step, with
a veto underneath that the model cannot argue with.

**What would change this (none of it true today).** If the policy grows real tools -- consult
memory, ask a human, re-plan the room order -- the right reach is the
Anthropic SDK's tool runner (`client.beta.messages.tool_runner`), which
supplies the loop for tools you define and host, with per-turn hooks that
map cleanly onto an approval gate. It belongs **inside** `vision_fn`,
below the mission runner, not wrapped around it. Durable mission state
across a brain restart, or real tracing once every step is a paid call,
are the other two reasons worth revisiting this.

---

## 12. Known gaps  *(perishable -- see §1)*

- **The vision policy cannot drive the simulator** (no pixels until phase
  S2), and has **no room memory** (§1). Both are the remainder of S2b.
- **Mission state does not survive a brain restart.** A crash loses the
  mission; only the robot's position persists, server-side. Nothing
  resumes.
- **One mission at a time, one robot per brain.** Deliberate, and the
  409 makes it explicit rather than racy.
- **`MissionMemory` is per-mission.** Nothing carries across missions, so
  a second search of the same house starts ignorant.
- **`as_context()` feeds nothing.** Section 10, item 2.
- **No planner.** `brain/planner.py` does not exist -- `VisionAgent`
  decides one move from one frame, which is not the same thing as a
  room-level planner reasoning over `MissionMemory.as_context()`. The
  rule-based policy is not on the hardware path and should not be extended
  (`PLAN-sim-hardening.md` 2.2); it is kept because it is free,
  deterministic, and the fastest way to test the safety layer and the
  mission memory.
