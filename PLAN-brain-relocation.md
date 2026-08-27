# Plan: move the autonomy loop onto the Pi

Status: proposal, nothing built. Written 2026-08-27.
Decision and reasoning: `HARDWARE-READINESS.md` section 7.
Answers Q2/Q3 in `PLAN-sim-hardening.md` section 5.

**Goal.** The robot is self-contained: it runs its own autonomy loop,
calls AWS outbound, and needs no laptop. The phone becomes an observer
that can watch, take manual control, and start/stop missions. The
MacBook keeps the development path unchanged.

**The whole trick.** If the brain is *always* an HTTP client of
`robot/server.py`, then "brain on the Pi" and "brain on the MacBook"
differ only by a base URL:

```
on the Pi        brain -> http://localhost:8000     (robot/server.py)
on the MacBook   brain -> http://192.168.1.50:8000  (same server, over LAN)
```

Same code, same process model, same tests. Where the brain runs stops
being an architectural question and becomes a deployment one. Everything
below exists to make that true.

---

## Target topology

```
  THE PI                                             AWS
  ─────────────────────────────────────              ─────────────────

  ┌──────────────────────────────┐
  │ :8001  control/brain_server  │  ── JPEG ──────►  /navigate
  │        the autonomy loop     │  ◄── action ───   vision service
  │        (RemoteRobot client)  │
  └───────────┬──────────────────┘
              │ localhost HTTP
              ▼
  ┌──────────────────────────────┐
  │ :8000  robot/server.py       │
  │        safety + watchdog     │
  │        hardware_robot.py     │
  └──────────────────────────────┘
              ▲            ▲
              │ manual     │ mission start/stop + status
              │            │
        ┌─────┴────────────┴─────┐
        │  PHONE (web twin)      │   observer + operator console
        └────────────────────────┘
```

Two processes, not one. The separation is load-bearing -- see "Why not
one process" below.

---

## Phase B0 -- Prerequisite: `RemoteRobot`

This is **Phase S3 of `PLAN-sim-hardening.md`**, unchanged; it is listed
here only to mark the dependency. Nothing below can start until
`control/remote_robot.py` exists and passes the identical-action-sequence
test (in-process vs. over HTTP).

---

## Phase B1 -- Extract the mission runner from the agent

**Problem.** `ObjectSearchAgent.run_mission()` is a blocking `for` loop
that returns only when finished. It cannot be started, stopped, or
inspected from outside, so nothing can drive it over HTTP.

**Build.** A `MissionRunner` owning one mission's lifecycle: target,
step budget, the agent, the memory, and a status snapshot. Expose
`start()`, `stop()`, `status()`, and a single-step `tick()`, with the
loop driven from outside rather than owned inside. No decision logic
moves -- this is purely turning a `for` loop inside out.

**Files.** New `control/mission_runner.py`; `brain/agent.py` gains
nothing (leave `run_mission()` in place for the demo scripts and tests).

**Test.** `tests/test_mission_runner.py`: drive a runner to completion by
calling `tick()` in a loop against `MockRobot`, and assert the result
matches `demo_active_search.py`'s (found in the kitchen, comparable step
count). Assert `stop()` mid-mission leaves `status()` reporting a
stopped, non-complete mission.

---

## Phase B2 -- The brain service

**Build.** `control/brain_server.py` -- a small FastAPI app that owns a
`MissionRunner` and drives it as an asyncio background task. It talks to
the robot exclusively through `RemoteRobot`, so it neither imports `sim/`
nor knows what backend is underneath.

Endpoints:

```
POST /mission/start   {"target_object": "red backpack", "max_steps": 80,
                       "policy": "frontier" | "vision"}
POST /mission/stop    always available, always stops the robot too
GET  /mission/status  {running, step, target, found, last_action,
                       last_reasoning, rooms_visited, log_tail}
GET  /health
```

Config: the robot's base URL and the vision service URL/secret, from
`config/robot.yaml`. Reuse `robot/server.py`'s `require_secret()` pattern
verbatim -- this process will be reachable on the LAN too.

**Files.** New `control/brain_server.py`; `config/robot.yaml` (a `brain:`
block -- `robot_url`, `vision_url`, `max_steps`); `requirements.txt`
unchanged (`httpx` is already there).

**Test.** `tests/test_brain_server.py`, FastAPI `TestClient` against a
live `robot/server.py` in `mode: sim`: POST a mission, poll status until
complete, assert the backpack is found. Assert `/mission/stop` halts a
running mission and that a second `/mission/start` while running is
rejected rather than racing.

**Note.** `POST /mission/stop` must call the robot's `/stop` as well as
halting the loop. Stopping the *thinking* is not stopping the *car*.

---

## Phase B3 -- Failsafes

Three distinct failures, three distinct guards. Today only the first
exists.

**B3.1 Motors left running (keep as-is).** `robot/server.py`'s watchdog.
Its meaning narrows once the brain is on localhost, but it keeps the job
that actually matters on hardware: if a movement call energises the
motors and then crashes before `px.stop()`, nothing else catches it.
Update the module docstring to say this, since "detects a dead MacBook"
stops being the primary description.

**B3.2 The AWS link (new).** The brain must treat vision failure as a
reason to stop, not a reason to retry silently. Add to `MissionRunner`: a
per-call timeout, and a consecutive-failure budget (suggest 3) after
which it issues `STOP` and ends the mission with a failure status. This
closes the gap noted in `HARDWARE-READINESS.md` section 7 -- the browser
currently logs a `/navigate` error and schedules the next tick.

**B3.3 A hung brain loop (new).** The brain service's own dead-man: if
`tick()` has not completed within N seconds, stop the robot and mark the
mission failed. Distinct from B3.1, which cannot see a loop that is alive
but stuck.

**Files.** `control/mission_runner.py`, `control/brain_server.py`,
`robot/server.py` (docstring only).

**Test.** `tests/test_failsafes.py` with a stub vision function: (a) three
consecutive failures -> mission ends, robot received a `STOP`; (b) a
vision call that hangs past the timeout -> same; (c) a healthy mission is
untouched by either guard.

---

## Phase B4 -- The twin becomes an observer

**Build.** Add a "Brain" connection field alongside the existing robot
and vision-service ones. Explore / Find / Autopilot POST to
`/mission/start` and render from polled `/mission/status` instead of
running their own JS timers. The manual D-pad keeps talking straight to
`robot/server.py` -- manual control must not depend on the brain being
up.

Keep the existing in-browser JS loop behind a "local brain" toggle. It is
already validated, it is the only thing that works with no Pi present,
and it stays useful for LAN development.

**Files.** `web-twin/index.html`; `web-twin/README.md`.

**Test.** Manual, against a live Pi (or two local uvicorns): start a
mission from the phone, background the tab, confirm the robot keeps
going -- **this is the observable proof that the brain has actually
moved.** Today, backgrounding the tab halts autonomy (there is an
explicit `visibilitychange` handler that stops the autopilot timer).

**Documentation debt.** `CLAUDE.md` section 6 currently defends the JS
duplication as correct, on the grounds that the browser legitimately
plays the brain role. That reasoning was right; this change supersedes
it. Update that bullet rather than leaving the two documents in
contradiction -- the browser becomes an *optional* brain, not the
primary one.

---

## Phase B5 -- Deployment

**Build.** Two `systemd` units on the Pi (`vision-picar-robot.service`,
`vision-picar-brain.service`), the brain unit ordered after the robot
unit, both with `Restart=on-failure`. Document the one-line switch
between brain-on-Pi and brain-on-Mac (point `brain.robot_url` at the Pi's
LAN IP and run `control/brain_server.py` on the Mac -- no code change).

**Files.** New `deploy/` with both unit files and a short README; update
`CLAUDE.md`'s repo map.

**Test.** Reboot the Pi; both services come up; a mission can be started
from a phone with no laptop involved anywhere.

---

## Why not one process

Running the loop as an asyncio task inside `robot/server.py` is less
code, and wrong here for three reasons:

1. **It defeats the watchdog.** A synchronous block in the agent loop
   blocks the event loop the watchdog polls on. As two processes, a hung
   brain still stops sending commands and still trips it.
2. **It merges the two roles the project has kept apart since Phase 0.**
   `robot/server.py` is the robot runtime; deciding what to do next is
   the brain. `CLAUDE.md` section 2 names this the single most important
   constraint not to break.
3. **It loses the base-URL trick.** The one-line move between Pi and
   MacBook only works if the brain is an HTTP client of the robot in
   every deployment, including on the Pi.

The cost is a localhost HTTP round trip per call -- roughly a millisecond
against a loop that spends 1-3 seconds waiting on Bedrock.

---

## Definition of done

1. A mission runs to completion with **no laptop on the network** --
   started from a phone, robot and brain both on the Pi.
2. Backgrounding or closing the phone's browser does not stop an
   in-flight mission.
3. `control/brain_server.py` imports nothing from `sim/` and nothing from
   `robot/` except `RemoteRobot` and the interface.
4. Moving the brain to the MacBook is a `brain.robot_url` change and
   nothing else -- proven by running the same mission both ways and
   comparing outcomes.
5. All three failsafes have tests: motors-left-running, AWS-link-dead,
   brain-loop-hung.
6. `POST /mission/stop` demonstrably stops the car, not just the loop.
7. Manual D-pad control still works with the brain service stopped.
8. `CLAUDE.md`'s "duplication is intentional" bullet reflects the new
   arrangement.

---

## Ordering note

B0 (= S3) is the only hard prerequisite. B1-B3 are all testable in
simulation today, with no hardware and no Pi -- they are the largest
block of pre-purchase work left after `PLAN-sim-hardening.md`'s S1-S4.
B4 is testable with two local uvicorns. Only B5 needs the hardware.
