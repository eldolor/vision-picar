# Plan: hardening the SIM so hardware day is short

Status: proposal, nothing built yet. Written against the repo as of
`183b99f`, with `pytest tests/ -q` at 61 passed.

Goal: exercise the robot <-> brain loop hard enough in simulation that
buying the PiCar-X and flipping `config/robot.yaml`'s `mode` is genuinely
the only change. This doc first corrects the mental model of how the loop
actually works today, then lists the specific places where "config change
only" is currently **not** true, then phases the work.

---

## 0. What already works (don't rebuild this)

The loop is already emulated end to end. Before planning new work, the
honest accounting:

- `robot/server.py` is the real Phase 9 Wi-Fi API and already runs
  against `MockRobot` with zero sim-specific code in it.
- Every movement path -- Python agent, browser D-pad, browser autonomous
  mode, vision autopilot -- routes through `robot/safety.py`. There is no
  second implementation of the robot.
- `web-twin/index.html` is a genuine HTTP client of that server (fixed in
  a prior session; the JS no longer simulates anything).
- **A real image pipeline already exists, in JavaScript**: the twin's
  Vision Autopilot renders a first-person raycaster view to a canvas
  (`renderFPV`, `web-twin/index.html:1853`), JPEGs it
  (`captureFPVFrame`, :1911), and POSTs it to the cloud service's
  `/navigate` route, executing the returned action through the same
  `commitAction()` safety path as every other mode. That is a working
  synthetic-camera emulation. It just isn't reachable from Python.

So the question is not "how do we emulate the loop" -- it's "where does
the emulation stop being faithful, and which of those gaps will bite on
hardware day."

---

## 1. How the loop actually works today

**The prompt's mental model is inverted, and the correction matters for
everything below.**

There is no PiCar -> Server uplink. There is no telemetry push, no
WebSocket, no heartbeat from the robot. The shape is:

```
   BRAIN (MacBook / browser)              ROBOT RUNTIME (the Pi)
   the HTTP *client*                      the HTTP *server*
   -------------------------              -----------------------------
                          GET /frame   ->
                       GET /distance   ->   robot/server.py (FastAPI)
                                              |
                        POST /action    ->    v
                          POST /stop    ->  robot/safety.py  (veto layer)
                                              |
                                              v
                                            robot/factory.py -> get_robot()
                                              |
                                        mode: sim -> sim/mock_robot.py
                                        mode: hardware -> (Phase 11, absent)
```

Concretely:

| Direction | Transport | Endpoint | Handler | Backend call |
|---|---|---|---|---|
| sense | HTTP GET, **pull** | `/frame` | `robot/server.py:frame()` | `RobotInterface.get_camera_frame()` -> `GridWorld.frame_description()` |
| sense | HTTP GET, **pull** | `/distance` | `robot/server.py:distance()` | `get_distance()` -> `GridWorld.distance_ahead()` x 30cm |
| act | HTTP POST | `/action` | `do_action()` | `SafetyController.check_and_execute()` -> dispatch table |
| act | HTTP POST | `/stop` | `stop()` | `RobotInterface.stop()` (bypasses the veto, by design) |
| liveness | HTTP GET | `/health` | `health()` | reports `seconds_since_last_command` |

Message shapes, verbatim from the code:

```jsonc
// POST /action request  (robot/server.py:ActionRequest)
{"action": "FORWARD", "speed": 50, "duration": 0.5, "angle": 90}

// POST /action response -- note a safety veto is HTTP 200, not an error
{"executed": true,  "result": {"action": "drive_forward", "speed": 50,
                               "duration": 0.5, "requested": 1,
                               "moved": 1, "position": [3, 2]}}
{"executed": false, "detail": "Blocked FORWARD: distance=0.0cm < min=20.0cm"}

// GET /distance
{"distance_cm": 270.0}

// GET /frame  (sim/grid_world.py:frame_description)
{"room": "living room", "facing": "E", "free_space_cells": 9,
 "doorway_ahead": false, "objects_visible": ["sofa"], "position": [2, 2]}
```

Two consequences that shape the rest of this plan:

1. **The watchdog runs the opposite way from what you'd expect.** It does
   not detect a dead robot; it detects a dead *brain*. If no request
   arrives for `watchdog_timeout_s` (1.0s), the Pi stops its own motors
   (`robot/server.py:watchdog_loop`). This is the correct design for a
   pull-based link -- but it means the brain must poll continuously, and
   nothing currently tests that it does.

2. **`brain/` does not use HTTP at all.** `brain/agent.py` receives a
   `RobotInterface` and calls it **in-process**. The only HTTP client of
   `robot/server.py` in this repo is the browser. So the Python agents
   that all 61 tests validate are exercising a code path that will not
   exist on hardware.

---

## 2. Where "config change only" is not true today

These are architecture gaps, not fidelity gaps. Ordered by how badly
each breaks the stated goal.

### 2.1 `get_camera_frame()` has no image contract -- BLOCKER

`MockRobot.get_camera_frame()` returns a dict of grid facts
(`room`, `facing`, `position`, `objects_visible`). A real Pi camera
returns JPEG bytes. There is no shape a hardware backend can return that
satisfies the current consumers. So `RobotInterface` -- the one
abstraction the whole design rests on -- **will have to change** when
hardware lands. That is precisely the outcome `robot/factory.py`'s
docstring promises won't happen.

### 2.2 The validated exploration policy cannot run on hardware -- BLOCKER

`MissionAgent.decide()` (`brain/agent.py`) prefers frontier cells by
reading `frame["position"]` and `frame["facing"]` -- grid coordinates no
camera can produce. Its docstring says it "degrades gracefully" to a
right-hand-rule fallback without them. True, but the implication is
under-stated: **the policy proven by `demo_explore.py` /
`demo_active_search.py` is not the policy hardware will run.** The
fallback path has far less coverage, and the same coordinate dependency
exists in the twin's JS (`web-twin/index.html:1592`, indexing
`position[0]`).

### 2.3 No Python HTTP client, and `mode:` doesn't cover the brain side

`control/` is empty. `config/robot.yaml`'s `mode` selects which backend
*the server drives* -- it says nothing about whether the *brain* talks
in-process or over the wire. Both need to be selectable.

### 2.4 No shared conformance suite

`RobotInterface` is an ABC with signatures but no documented return-shape
contract, no units, no ranges, no error semantics. `robot/hardware_robot.py`
will be written from scratch on hardware day with nothing to check it
against but "does it look right."

---

## 3. Where the physics diverges

Fidelity gaps. All are known-risk decisions, not blockers.

### 3.1 There is no time in the sim

`MockRobot._settle()` is `pass` -- an explicit hook, never filled in.
`drive_forward(speed=50, duration=0.5)` returns instantly. Downstream:

- The watchdog's async loop is **never executed by any test** (only the
  pure `watchdog_should_stop` is). The one safety mechanism whose whole
  job is timing has zero timing coverage.
- No command can ever still be in flight when the next arrives, so
  overlapping-command and re-entrancy behavior is undefined and untested.
- `tests/` complete in 0.66s partly *because* nothing takes any time.

### 3.2 The safety threshold is untestable at its boundary -- verified

`get_distance()` returns `cells * 30.0` where `cells` is an int in
0..10. The only values the sim can ever produce are:

```
0.0  30.0  60.0  90.0 ... 300.0
```

`safety.min_distance_cm` is `20`. **The threshold is therefore only ever
crossed at exactly 0.0 cm** -- i.e. the robot's nose already inside the
wall. Any value from 1 to 30 would make every existing test pass
identically. The same constant is duplicated in the twin
(`web-twin/index.html:1318`, `MIN_DISTANCE_CM = 20`). The safety layer is
structurally sound and completely uncalibrated.

Also: the sim caps at `max_range=10` cells = 300cm; an HC-SR04 reads
roughly 2-400cm. And `distance_ahead()` casts a perfect 1-cell-wide ray
along a grid axis; the real sensor has a ~15 degree cone, +/- a few cm of
noise, returns garbage on soft or angled surfaces, and takes ~40ms.

### 3.3 Motion is discrete, instantaneous, and always succeeds

- `_speed_duration_to_cells` is `max(1, round(...))` -- `speed=1,
  duration=0.01` still moves a full 30cm cell. Speed and duration are
  very nearly decorative.
- Turns are exactly 90 degrees, in place, always successful, no drift.
- **A PiCar-X has Ackermann steering and cannot turn in place.** A real
  `turn_left(90)` is a steering-servo angle plus forward motion -- an
  *arc* that consumes forward space. Every "turn in a tight spot"
  decision validated in sim is validated against a maneuver the robot
  cannot perform.
- No wheel slip, so heading error never accumulates. Real dead reckoning
  drifts within a few meters.

### 3.4 The starter map is not to scale -- verified

`sim/maps/starter_house.py` is 13x10 cells. At the code's own 30cm/cell:

- whole house: **3.9m x 3.0m**
- living room (3x3 cells): **90cm x 90cm**
- doorways (1 cell): **30cm wide**

A PiCar-X is roughly 26cm long, 17cm wide, with a turning radius in the
40-50cm range. **It cannot execute a turn inside the 90cm living room**,
and a 30cm doorway leaves ~6cm clearance per side. The map is a fine
logic puzzle and a poor physical proxy.

### 3.5 The synthetic camera is a flat-shaded raycaster

The FPV view is untextured maze geometry. A VLM's accuracy on that tells
you very little about its accuracy on photographs of a real living room.
This is the gap least closable in simulation -- see section 6.

### 3.6 The network is not in the loop

`TestClient` uses an in-process ASGI transport -- no sockets, no latency,
no loss. The deployed twin talks to a container, not to a Pi over
household Wi-Fi. Nothing exercises jitter, packet loss, a mid-command
disconnect, or Wi-Fi roaming.

---

## 4. Phased plan

Each phase: what gets built, files touched, and the test that proves it.
Phases 1-3 close architecture gaps; 4-6 close fidelity gaps; 7 is chaos.
Phases 1-3 are the ones that change what hardware day looks like.

### Phase S1 -- Pin the contract

**Build.** Document `RobotInterface`'s return shapes, units, ranges, and
error semantics as an explicit contract. Add a backend-agnostic
conformance suite that any `RobotInterface` implementation must pass,
parameterized over backends so `MockRobot` runs it now and
`HardwareRobot` runs it unchanged later.

**Files.** `robot/interface.py` (docstrings + a `FrameSchema` /
`ActionResult` typed shape); new `tests/test_robot_contract.py`; new
`tests/conftest.py` fixture parameterizing the backend.

**Test.** `pytest tests/test_robot_contract.py` passes against
`MockRobot`. Every assertion phrased in backend-neutral terms -- no grid
coordinates, no cell counts. Assert units (cm, degrees, seconds),
monotonicity (`stop()` is idempotent), and that every method returns the
declared keys.

**Why first.** It is cheap, it changes no behavior, and every later phase
gets checked against it.

### Phase S2 -- Give `get_camera_frame()` a real image contract

**Build.** Move the twin's raycaster from JS into Python so `MockRobot`
can return actual JPEG bytes. `get_camera_frame()` returns
`{"image_base64": ..., "media_type": "image/jpeg", "metadata": {...}}`,
where `metadata` carries the grid facts as **explicitly sim-only debug
data** that no policy is permitted to read. `GET /frame` returns the
same. Keep `frame_description()` available for the offline/free path.

**Files.** New `sim/renderer.py` (Python port of `renderFPV`);
`sim/mock_robot.py`; `robot/interface.py`; `robot/server.py:frame()`;
`web-twin/index.html` (consume server-rendered frames instead of
rendering its own -- removes the last piece of simulation living in JS);
`brain/vision.py` (accept bytes, not just a file path).

**Test.** (a) Contract suite asserts `get_camera_frame()` returns
decodable JPEG bytes. (b) Golden-image test: renderer output for a known
pose is stable. (c) Parity test: the Python renderer and the JS
raycaster produce the same view for the same pose -- this is what lets
the JS one be deleted. (d) `test_server.py`: `GET /frame` returns a
decodable image.

**Ambiguity to resolve first.** See section 5, Q1 -- whether the
rule-based policy or the vision policy is the one hardware inherits
changes how much this phase matters.

### Phase S3 -- Put the brain on the wire

**Build.** `RemoteRobot(RobotInterface)` -- an HTTP client of
`robot/server.py` implementing the same interface, so `brain/agent.py`
runs unchanged whether the robot is in-process or across the room. Add a
brain-side transport selector distinct from the robot-side `mode`.

**Files.** New `control/remote_robot.py`; `config/robot.yaml` (a
`brain.robot_url` key -- per Q3 there is no transport switch, only a base
URL); optionally `control/manual_control.py`, now nearly free.

**Test.** Run `tests/demo_active_search.py` twice -- once in-process,
once through `RemoteRobot` against a live `uvicorn robot.server:app` --
and assert **identical action sequences**. This is the single most
valuable test in the plan: it proves the HTTP boundary is transparent.
Add to the contract suite so `RemoteRobot` must pass it too.

### Phase S4 -- Put time in the loop

**Build.** Implement `MockRobot._settle()` for real, behind a config flag
(`sim.realtime: true|false`) so the unit suite stays fast. Add a simulated
clock so tests can advance time without sleeping. Make actions occupy
their `duration`.

**Files.** `sim/mock_robot.py` (`_settle`); `config/robot.yaml`; new
`tests/test_watchdog_integration.py`.

**Test.** Start the app with a real event loop (`httpx.ASGITransport` +
`asyncio`, or a live uvicorn on a port), issue one command, stay silent
past `watchdog_timeout_s`, assert `robot.stop()` actually fired and
`/health` reports the stale age. Second test: issue commands faster than
the timeout, assert the watchdog never fires.

### Phase S5 -- Sensor realism

**Build.** A `DistanceSensorModel` wrapping `GridWorld.distance_ahead()`:
sub-cell resolution, Gaussian noise, a cone rather than a ray, a dropout
rate, min/max range clamped to the real sensor's 2-400cm, and read
latency. All parameters in config, all defaulting to today's ideal
behavior so existing tests are unaffected until opted in.

**Files.** New `sim/sensors.py`; `sim/grid_world.py` (sub-cell distance);
`sim/mock_robot.py`; `config/robot.yaml` (a `sim.sensor_noise` block).

**Test.** (a) With noise enabled, `min_distance_cm` is crossed at
realistic distances, not only at 0 -- this is what makes the safety layer
testable at its boundary at all. (b) Sweep `min_distance_cm` across
5/10/20/30 and assert the collision rate over a long run responds; today
it is provably flat. (c) Assert the safety layer never permits FORWARD
into a wall across N seeded noisy runs. (d) Dropout: sensor returns
`None`/max -- assert the system fails safe (STOP), which is **currently
undefined behavior**.

### Phase S6 -- Motion realism

**Build.** Continuous pose (float x, y, heading in degrees) underneath
the grid, with the discrete grid derived from it for room lookup. An
Ackermann turn model where `turn_left(angle)` traces an arc with a
configurable minimum radius and can be blocked mid-arc. Optional
per-move heading drift. Rescale the starter map, or add a to-scale
second map, so the geometry admits a real PiCar-X.

**Files.** `sim/grid_world.py` (largest change in the plan);
`sim/mock_robot.py`; new `sim/maps/scaled_house.py`;
`config/robot.yaml`.

**Test.** (a) With `min_turn_radius` set to a PiCar-X-like value, assert
`demo_active_search.py` still completes on the scaled map -- if it does
not, the exploration policy needs work *before* hardware, which is
exactly the finding worth having now. (b) Assert a turn in a corridor
narrower than the turning circle is refused or arcs into a blocked state
rather than teleporting. (c) With drift enabled, assert the safety layer
still prevents all collisions over a long run.

**Note.** This is the biggest change and the most deferrable, because
safety re-checks distance every step regardless of pose error. Do it
last, and be willing to stop at "scaled map + arc turns" without drift.

### Phase S7 -- Chaos and soak

**Build.** A fault-injection proxy between `RemoteRobot` and the server:
added latency, jitter, dropped requests, connection resets, duplicated
requests.

**Files.** New `tests/fault_proxy.py`; new `tests/test_chaos.py`.

**Test.** (a) 200ms latency + 5% loss: mission still completes, no
collisions. (b) Kill the link mid-mission: the watchdog stops the motors
within `watchdog_timeout_s` (verifies S4 under realistic conditions).
(c) Duplicate a `/action` POST: assert the outcome is sane -- the API has
no idempotency key today, so this may surface a real design question.
(d) Soak: 1000-step run, assert no unbounded growth in `GridWorld.log`
(which currently appends forever) or `agent.history`.

---

## 5. Ambiguities -- answer these before building

**Q1. Which policy does hardware inherit?** There are two working
autonomy implementations: the rule-based frontier explorer
(`brain/agent.py` + its JS twin) and Vision Autopilot (real Claude calls
on rendered frames). The first cannot run on hardware without
coordinates (section 2.2); the second can. If the answer is "vision,"
Phase S2 becomes the top priority and Phase S6's grid fidelity matters
much less. If it is "rule-based," a coordinate source is needed and that
is a much larger project (odometry / SLAM) that this plan does not cover.
**This is the single highest-leverage open question here.**

**Q2. Does `brain/` stay on the MacBook? -- ANSWERED (2026-08-27): no.**
`README.md` says yes, but that predates vision moving to Bedrock. The
brain no longer computes anything -- it is four HTTP calls in a loop, and
`requirements.txt` has no ML dependency at all. Meanwhile the MacBook
split is what forces the brain onto the LAN, which is the root of the NAT
and mixed-content problems. **Decision: the autonomy loop moves onto the
Pi as its own process; the MacBook keeps the development path; the phone
becomes a pure observer.** Reasoning in `HARDWARE-READINESS.md` section
7, implementation in `PLAN-brain-relocation.md`.

This makes **Phase S3 more important, not less** -- `RemoteRobot` becomes
the single path every brain uses, on the Pi and on the MacBook alike.

**Q3. Where does the brain-side transport switch live? -- ANSWERED
(2026-08-27): nowhere. There isn't one.** The earlier framing assumed a
choice between in-process and over-HTTP. Q2's answer removes it: if the
brain is *always* an HTTP client, "on the Pi" versus "on the MacBook" is
a base-URL difference (`http://localhost:8000` versus
`http://<pi-lan-ip>:8000`) and nothing more. No `brain.transport` key, no
third `mode` value, one code path. The in-process path survives only for
tests and the `tests/demo_*.py` scripts, which construct a backend
directly and should keep doing so.

**Q4. How much fidelity is worth buying before just buying the robot?**
A PiCar-X kit is roughly the cost of a few days of this work. Phases
S1-S4 are worth doing regardless (they are correctness work, not
simulation work). Phases S5-S6 approach the point where measuring the
real thing beats modeling it -- consider capping the sim work at S4 and
buying hardware, with S5-S7 as calibration work done *against* real
measurements.

**Q5. `brain/planner.py`.** `CLAUDE.md` lists it as a real gap. It is
out of scope here, but note it would sit exactly where Q1's answer lands
-- worth settling Q1 first so the planner is not built against the wrong
observation shape.

---

## 6. Definition of done

Before trusting a hardware swap-in, all of these:

1. The Phase S1 contract suite passes against `MockRobot` and
   `RemoteRobot`, with no grid-specific assertions in it.
2. `demo_active_search.py` produces an **identical action sequence**
   in-process and over HTTP (Phase S3).
3. The watchdog's real async loop is proven by an integration test to
   stop the motors after silence, and proven not to fire under a normal
   command cadence (Phase S4).
4. `min_distance_cm` is demonstrably load-bearing: changing it changes
   measured behavior (Phase S5). Today it provably does not.
5. Sensor dropout and out-of-range reads have defined, tested fail-safe
   behavior. Today this is undefined.
6. `get_camera_frame()` returns real image bytes on every backend, and no
   policy reads grid coordinates on the path intended for hardware
   (Phase S2 + Q1).
7. A mission completes with no collisions under 200ms latency and 5%
   packet loss (Phase S7).
8. A to-scale map with arc-based turning is navigable by the chosen
   policy (Phase S6) -- or this is consciously waived as known risk.
9. `robot/server.py`, `robot/safety.py`, and `brain/` are unchanged by
   the hardware swap. If any of them needs a change, the abstraction
   leaked and the swap is not a config change.
10. `MIN_DISTANCE_CM` exists in exactly one place. It is currently in two
    (`config/robot.yaml` and `web-twin/index.html:1318`, kept in sync by
    a comment).

---

## 7. What cannot be validated without hardware

State these as accepted risk rather than pretending simulation covers
them:

- **VLM accuracy on real photographs.** A raycast render is not a room.
  Partial mitigation: replay real photos of an actual room through
  `/analyze` and `/navigate` -- no robot required, only a phone. This is
  worth doing early and cheaply, and it is the highest-value
  non-hardware check available.
- **Actuation reality.** Motor deadband, battery-voltage-dependent speed,
  carpet vs. hardwood, wheel slip, servo backlash. Modelable, not
  verifiable.
- **Ultrasonic behavior on real surfaces.** Curtains, sofas, glass, and
  table legs all defeat an HC-SR04 in ways no grid predicts.
- **Camera characteristics.** FOV, rolling shutter, motion blur, exposure
  in a dim hallway, and the real capture-to-decision latency budget.
- **Wi-Fi in a real house.** Roaming between APs, dead spots, contention.
  S7 approximates the failure modes but not their real distribution.
- **The stopping-distance budget.** How far the robot travels between
  "sensor read" and "motors actually stopped" is the number that decides
  whether `min_distance_cm: 20` is safe. It can only be measured. Until
  then, treat 20 as a placeholder, not a validated value.
