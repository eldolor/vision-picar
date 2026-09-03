# Plan: what to take from Microduck

Status: M1 built and measured (see its entry -- the replays settled more than the sim run did), M2 built 2026-09-03, M3-M12 proposed (M7b added 2026-09-03) · Date: 2026-09-02 · Phase IDs: `M1`-`M12`

[Microduck](https://github.com/pollen-robotics/microduck) (Apache-2.0, read
2026-09-02) is Pollen Robotics' open-source brain for a 25cm bipedal robot:
seven Rust daemons on a Rockchip RK3566, a 50Hz control loop driving fifteen
servos from ONNX policies trained in MuJoCo.

**None of its code should be ported.** Different language, different board,
different locomotion problem -- a PiCar-X has two degrees of freedom and no
gait to learn. What transfers is the design record, because it has already
worked through several problems that are still open here.

Phase IDs are `M*`, alongside `S*` (`PLAN-sim-hardening.md`), `B*`
(`PLAN-brain-relocation.md`) and `T*` (`PLAN-teleop-robot.md`).

Section 8 lists every source file and doc section cited, so each claim below
can be checked at source rather than taken from this summary. Section 9 records
how this document was made, because two of its corrections came out of that.

---

## 1. Where Microduck is ahead of us

| Microduck | vision-picar today | Gap |
|---|---|---|
| Camera gives direction, ToF gives distance (`autonomous_behavior.md`) | The prompt is asked for both. Stage 0 shows it cannot answer the second | The Stage 0 gate failure, restated as a sensor split |
| An 8x8 depth matrix at 15Hz, with floor returns and short-range noise filtered out (`kinematics/tof.rs`) | One ultrasonic beam, and a prompt asked what is in the centre third | No metric answer to "is my next step clear" |
| A zone is `Range` / `NoTarget` / `Unusable` -- three outcomes, on purpose (`tof/lib.rs`) | `sim/sensors.py` reads a dropout as `0.0cm` | Correct today; breaks the moment a grid has a consumer (M3) |
| `robotd` owns motors, safety, and a 500ms deadman | `robot/server.py`, `robot/safety.py`, the 1.0s watchdog (B3.1) | Same shape. Refusals carry no reason on the wire |
| Authority arbitration is a decided order, not last-writer-wins (`architecture` §6) | One brain at a time via the 409. Nothing governs the D-pad during a mission | `robot/server.py` has no notion of who is driving. Verified |
| Health is a verdict plus a description, and only what a release can be blamed for reaches the verdict (`robotd-design` §3.4) | Two `/health` routes, no verdict, nothing exits non-zero | Nothing for B5 to gate a deploy on |
| `robotd` never moves the robot because a process started (`robotd-design` §1.5) | Undefined. `Restart=on-failure` will restart mid-mission on the Pi | Testable today, before `HardwareRobot` exists |
| Perception in its own process; a stall degrades perception, never motor control | `picamera2` capture will run in the same process as `/stop` | A wedged capture has no defined behaviour |
| Releases swapped under a symlink, health-gated, rolled back (`updater-design` §8) | B5 plans `Restart=on-failure` | No answer for a bad deploy |
| A method or field this release does not have refuses **by name** (`duck-ipc-proto`) | `/navigate` validates its two allow-lists correctly. Its env-var default does not | Small residue, one step from being armed (M7) |

---

## 2. The argument the first four phases rest on

Microduck states a rule for splitting perception across sensors:

> vision cannot tell identical ducks apart -- camera = direction, ToF =
> distance, BLE beacon = identity + presence

Each question goes to the sensor that can answer it. Its `duck-detect` crate
reduces a bounding box to a single `bearing()` in -1..1 -- "turn towards it"
needs a bearing, not a box.

**This project measured the same thing and has not yet acted on it.** Stage 0
found that every model identifies a red backpack (semantics: fine) and that
depth is where it falls apart:

- `obstacle_ahead` is uncalibrated in both directions -- on identical frames,
  Opus reports it on ~100% and Qwen on ~0%.
- `default-with-distance` answers `within_one_step` on 60% of frames, and
  still 55% on frames where the target is not visible at all, so it is not a
  target-counting artifact.
- Four wordings have been measured. Two produced never-FORWARD, two produced
  always-FORWARD. Both are degenerate; neither is a calibration problem that
  better wording looks likely to fix.
- `center-third-path` (written, unmeasured) is a fifth attempt at the same
  thing by changing the question's shape.

The Microduck reading is not "find a better wording". A single monocular frame
does not contain metric depth, so no wording recovers it. `/navigate` keeps
*what* and *which way*; a sensor owns *how far*.

CLAUDE.md already goes half this distance -- "do not let the vision policy be
the thing relying on this field". M1 and M2 go the rest, and do it in the sim
for no hardware and no new hardware money.

---

## 3. Order of work

Seven of the twelve phases need no hardware.

| # | Phase | When | Press this to prove it |
|---|---|---|---|
| M1 | Settle the gate reading | pre-hardware | **DONE.** The replay table gained two columns; the sim leg is blocked on the renderer |
| M2 | A depth grid on the interface, and in the sim | pre-hardware | **DONE.** A depth strip under the FPV canvas, tracking the view |
| M3 | The tri-state zone, and a centre-zone veto | pre-hardware | Dropout reads grey, not "wall". The collar fires off-centre |
| M4 | Refusals are state, manual preempts autonomous | pre-hardware | Tap the D-pad mid-mission. It ends `preempted`, and says by whom |
| M5 | One health command | pre-hardware | Kill the brain. Settings flips and the command exits non-zero |
| M6 | A process start never moves the robot | pre-hardware | Restart the robot server mid-mission. The map does not twitch |
| M7 | Nothing falls back silently | pre-hardware | A misspelled env variant refuses at boot instead of serving `default` |
| M7b | One shipped wording | pre-hardware | The mission panel names no wording, because there is only one |
| M8 | Floor rejection and the too-close band | hardware day | Tilt the camera down. The strip stays clear |
| M9 | The camera cannot wedge `stop` | hardware day | Pull the ribbon mid-mission. `/frame` errors, `/stop` answers |
| M10 | Clearance from a real sensor | buy list | D-pad at a chair leg the ultrasonic beam misses. The collar flashes |
| M11 | The small updater | hardware day | Install a broken build. The Pi returns to the previous one and says so |
| M12 | Novelty-grid exploration memory | conditional | The log shows the model told what it already tried here |

M1 is first because it is the cheapest test of section 2's thesis and it *is*
Stage 1's outstanding closed-loop run. M2-M3 follow because they are what M1
argues for, and they are testable in the sim. M5 precedes M11 because M11 gates
on it. M6 is separated from M9 because its test can be written today and its
implementation cannot. M12 is last and conditional -- see its entry.

---

## 4. Pre-hardware phases

### M1 -- Settle the gate reading -- **BUILT AND MEASURED 2026-09-02**

**What it showed, in one line.** Deleting the obstacle question is the first
change that leaves no model degenerate; the closed-loop sim leg could not
adjudicate anything, because the renderer's frames are too dark to navigate
from.

**The replay leg answered its question.** All 22 frames of
`red-backpack-20260829-195904`, three models x the two new wordings, 100%
coverage on every cell, deployed against the redeployed vision service. The
full 5x3 table is in `CLAUDE.md`'s Stage 0 notes; three findings:

- **`bearing-only` is the only column with no degenerate cell.** Sonnet goes
  0.000 -> 0.591 without flipping to the always-FORWARD mode that every
  reworded question traded the stall for. No `stalled`, no `degenerate`.
- **Question 5 broke `next-step-obstacle`, not the region change.**
  `center-third-path` moves question 2 alone: Sonnet 0.000 -> 0.091. Moving 2
  and 5 together gave 1.000. That attribution is what the variant existed
  for, and it cost one replay. Neither wording is worth promoting.
- **Qwen was already ignoring the question** -- `obstacle_rate` 0.000 under
  `default`, and its `bearing-only` numbers are identical to its `default`
  ones in every field. Removing a question a model never answered changes
  nothing, which is the cleanest confirmation available that `obstacle_ahead`
  is uncalibrated rather than noisy.

Every `bearing-only` cell is still flagged `collision`, 12 of 12 checked
FORWARDs. **That is this phase's other output, and it is a specification, not
a defect** -- `ReplayRobot` has no sensor, and M10's sensor is what has to
catch those twelve.

**The sim leg did not answer its question, and the reason is worth more than
the answer would have been.** Stage 1's "done when" ran for real -- Opus 4.5,
`policy: "vision"`, the grid world, the deployed `/navigate`, sensor noise
on, 40 paid steps per wording. `default` ended 14 cells away in 142.2s;
`bearing-only` ended 13 cells away in 146.1s having never left its start
cell. Neither claimed arrival.

But nearly every decision's reasoning said "the image is very dark and
unclear" or "a blank gray wall", and the frames bore that out: from the start
cell `sim/renderer.py` rendered mostly black with two grey slabs. **The sim's
frames were too information-poor for a closed-loop run to discriminate
between wordings at all.** M1's design assumed the sim's distance sensor
could stand in for a ToF and settle the gate reading empirically; that
assumption is sound and the instrument was not ready.

**The lighting half of that has since been fixed** (see
`PLAN-sim-hardening.md` 3.5): the render was painting its ceiling and floor
with the twin's two near-black UI colours, and both renderers now carry lit
constants of their own. **The fix is itself unmeasured** -- it makes the
frames legible to a human eye, and whether that is enough for a closed-loop
run to discriminate is the next paid run's question. The other half of the
diagnosis is untouched: the starter house's start pose faces a near wall, so
even a lit first frame shows very little of the room. That is a map question.

Two things the run did prove that no replay can. The safety collar is live
and fired (1 veto on `default`, 2 on `bearing-only`) -- under `bearing-only`
it is the only obstacle logic left, exactly as designed. And
`min_distance_cm: 30.0` is exactly one grid cell, so with S5's 3cm jitter the
veto is near a coin flip at one cell of clearance. A threshold sitting on a
quantization boundary should be moved before anyone reads a FORWARD rate off
a noisy sim run.

Built:

- **`bearing-only`**, in `service/vision_analyze/vision_core.py`. Two lines
  removed from `default` and nothing else -- the question and its schema line
  -- pinned by a test that diffs the two templates and asserts the removal
  set exactly. `obstacle_ahead` is stripped from the reply for any variant
  whose template does not ask for it (`variant_asks_obstacle()`, read off the
  template so it cannot drift), including when a model volunteers the field
  unasked.
- **`brain/navigate.py` tolerates the absence honestly.** `free_space` becomes
  `"unknown"` and `_navigate["obstacle_ahead"]` becomes `None`, never `False`
  -- the same distinction M3 makes for a failed depth zone. The only obstacle
  logic left on the path under this variant is `SafetyController`'s
  `get_distance()` re-check before every FORWARD.
- **The wording reaches a mission at all.** `prompt_variant` now threads
  `POST /mission/start` -> `brain_config` -> `vision_fn_for()` ->
  `/navigate`, validated against the service's published `prompts` in the
  same round trip that already validated `model_id`.
- **The twin can start a vision mission.** The Remote brain panel gained a
  policy picker; `policy: "vision"` was previously unreachable from the UI,
  which by section 7's rule meant it was not shipped.

**A live silent fallback was found and closed on the way**, and it is the
same class of bug M7 is about. `web-twin/app.js` has been sending
`prompt_variant` on "Drive via brain" since the picker shipped;
`MissionStartRequest` had no such field, and pydantic discards an unknown one
by default. Every brain-driven walk ran the service's default wording while
the UI named the operator's pick. The model is now `extra="forbid"`, so a
misspelled field is a 422 naming it.

Also built, to make the runs possible and attributable:
`tests/demo_sim_mission.py` takes `NAVIGATE_MODEL_ID` /
`NAVIGATE_PROMPT_VARIANT` / `ROBOT_CONFIG_PATH`, prints all three plus
whether the sensor is noisy, and builds its backend from `config/robot.yaml`
via `robot/factory.py` so `sim.sensor_noise` actually applies. All five ECS
services were redeployed from this branch; `/navigate/models` serves six
wordings.

Two follow-ups found by the sim runs, both since fixed:

- **The renderer's lighting** (above).
- **`min_distance_cm: 30.0` was exactly one grid cell**, so with S5's 3cm
  jitter the veto fired on roughly half of the legal one-cell moves --
  measured at 90/200 in a test written for it. Both the brain-side default
  and `config/robot.yaml` are now 20.0, matching the robot server's own
  threshold, which changes nothing against the noiseless sensor (an exact
  reading is only ever a multiple of 30, so any threshold in (0, 30] blocks
  the same single case) and takes the veto 3.3 sigma clear of one cell.

Still open after this phase:

- **Re-running the closed loop** now that the frames are lit. The runs above
  measured a renderer, not a policy, and the fix is unmeasured.
- **Whether to promote `bearing-only`.** It is the best column measured, on
  one walk, against a `collision` count that only a real sensor answers. That
  is M10's evidence, not this phase's, and promoting a default off one walk
  is how the `NavigateModelId` mistake happened.

**Why.** Two questions are open at once, and one run each answers both.

The first is `center-third-path`, written and unmeasured. It changes question 2
alone, byte-identical elsewhere by construction, precisely so that a degenerate
result can be attributed -- `next-step-obstacle` moved questions 2 and 5
together and left no way to tell which half caused it. Measuring it is one
replay over frames that already exist.

The second is whether a depth sensor changes what Stage 0 is measuring. Robot
view and `ReplayRobot` have no depth sensor and never will -- a phone walk has
no ToF. So if hardware gets one, Stage 0's dominant failure mode is being
measured on a configuration that will not ship. That reads two ways: either the
gate is partly moot (vision was never going to be the obstacle sensor), or it
still stands (a robot that navigates only because a sensor vetoes its bad
decisions has not been shown to navigate).

It is not resolvable by argument, and it does not need to be. **The sim has a
distance sensor.** `bearing-only` run closed-loop in the grid world, with
`sim.sensor_noise.enabled: true`, is the noisy distance model standing in for
the ToF. If the target is reached with the obstacle question removed and is not
reached with it present, the reading is settled empirically.

**Build.**

- Replay `center-third-path` over all 22 frames of
  `red-backpack-20260829-195904`. Read `coverage` before the score.
- A `/navigate` variant `bearing-only`: the obstacle question removed, every
  other question byte-identical to `default` (string surgery, pinned by a test,
  the way `center-third-path` is). `obstacle_ahead` absent from the reply.
- `brain/vision_agent.py` tolerates an absent `obstacle_ahead`. The only
  obstacle logic left on the path is `SafetyController`'s `get_distance()`
  re-check before every FORWARD.
- Run Stage 1's "done when": a Python agent, `policy: "vision"`, the grid-world
  sim, the real `/navigate`, cost and wall-clock recorded. This is the first
  closed-loop measurement this project has ever had -- unlike a replay, turning
  left really does change the next frame.
- Replay `bearing-only` over the same 22 frames so the FORWARD-rate table gains
  a second column.

**Files.** `service/vision_analyze/vision_core.py` and its tests,
`brain/vision_agent.py`, `tests/demo_replay_mission.py`, the Stage 0 notes in
`CLAUDE.md`.

**Test.** The variant differs from `default` only by the removed question. The
agent accepts a reply with no `obstacle_ahead`. Note that
`control/walk_eval.py`'s collision check will flag the `bearing-only` replay,
because `ReplayRobot` has no sensor -- **record that column as "what the sensor
must catch", not as a defect.** That is the phase's other output: a
specification for M10.

**Press this.** Remote brain, `policy: vision`, variant `bearing-only`, sensor
noise on. The collar vetoes a FORWARD at the wall, and the model stops turning
on the spot with the target centred.

**Done when** the sim backpack hunt completes under the real `/navigate` with
cost and wall-clock in the notes, and the variant table has both new columns.

### M2 -- A depth grid on the interface, and in the sim -- **BUILT 2026-09-03**

**What it is.** `RobotInterface.get_depth_grid()`, the seam a distance
sensor arrives through, exercised end to end in the sim before any sensor
is bought. Nothing in `brain/` reads it yet -- M3 gives it its first
consumer.

Built:

- **The method, non-abstract, with an honest default.** `unusable_grid()`
  in `robot/interface.py`: every zone `ZONE_UNUSABLE`, which is
  `NO_SENSOR_CM`'s rule one sensor later. `ReplayRobot` and `TeleopRobot`
  inherit it -- a photograph has no depth in it and a phone walk has no
  ToF -- so neither can make a walk look as though it exercised collision
  avoidance it never had.
- **All three zone outcomes exist from the start**, because the default
  needs the third one. `ZONE_RANGE` carries a number; `ZONE_NO_TARGET`
  (nothing within range -- information about the room) and `ZONE_UNUSABLE`
  (the absence of information) both carry `None`. M3 is still where the
  tri-state becomes load-bearing: it is what starts producing `UNUSABLE`
  in the sim, from `sim.sensor_noise`'s dropout, and what teaches
  `robot/safety.py` to reduce the centre zones.
- **`MockRobot` synthesises the grid from `renderer.cast_ray()`** -- the
  same raycaster that draws the camera frame, off the *view* heading, so
  `look_left()` swings the strip exactly as it swings the picture.
  `rows: 1`, truthfully: `cast_ray()` has no elevation, and eight
  identical copies of one row would look like a matrix and be a fiction.
- **The grid and the scalar agree by construction.** Half a cell (the
  robot occupies its own cell) and one `FPV_STEP` (the march overshoots
  into the wall) are subtracted, which makes the centre zones equal
  `get_distance()` exactly on an axis-aligned wall and makes the reading
  conservative rather than optimistic everywhere else. This was not
  cosmetic: without it the grid read ~16cm further than the scalar on the
  same wall, and M3's veto has to choose between those two numbers. The
  project has already paid once for a threshold sitting half a cell from
  where it was assumed to be (`min_distance_cm: 30.0`, M1).
- **`GET /depth` on `robot/server.py`, and `RemoteRobot` over it.** Its
  own route, not a field on `/frame`: a wedged camera must not take the
  clearance reading down with it (M9). `RemoteRobot` **overrides** the
  interface default rather than inheriting it -- inheriting would report
  "no sensor" about a robot that has one -- and treats a 404, and only a
  404, as "this server predates the route". A 500 still raises, so a
  broken sensor is never quietly reported as an absent one.
- **The conformance suite gained a fifth backend.** `_HaltGate` wraps the
  robot every mission is actually driven through, delegates method by
  method, and did exactly what that shape does: it inherited the
  all-unusable default while wrapping a `MockRobot` that had a working
  sensor. Nothing else in the suite would have noticed -- a gate that
  forgets a *sensing* method still passes every test about the methods it
  guards. `tests/conftest.py`'s `RecordingRobot` had the same hole.

**Press this.** Sim tab, under the FPV canvas: eight zones, red near, green
far, hatched grey where unmeasurable, with a readout naming the grid's own
shape and the nearest zone. Drive the D-pad at a wall and watch the centre
zones close; tap look-left and the strip swings with the picture. A server
that predates the route says "depth: not reported by this server" rather
than going blank -- blank and "no obstacles" must not look alike.

**Done.** The strip tracks the FPV view and the contract suite passes on
all five backends (75 tests, up from 44).

**Not deployed.** `cloudformation/twin.yaml` and `teleop-robot.yaml` carry
the new path patterns -- `/depth` took the twin's PWA rule to its
five-value limit and needed a third rule on the teleop stack -- but no
stack has been redeployed. Until it is, the deployed twin shows "depth:
not reported by this server", which is the correct reading of a pre-M2
server and is what that message exists for.

**As originally specified**, kept below the way M1's is -- the design is
what the built thing has to be read against.

**Why.** M1 argues the sensor should own clearance. This is the seam that lets
it, and it can be exercised in the sim before any sensor is bought.

**Build.** One method on `RobotInterface`:

```python
def get_depth_grid(self) -> dict:
    """{"rows": int, "cols": int, "zones": [Zone, ...]}  # row-major"""
```

Borrowing Microduck's `Frame` shape, `rows`/`cols` travel in the data rather
than being pinned by the contract. That is what lets the sim be honest: the
grid world is two-dimensional and `sim/renderer.py:cast_ray()` has no
elevation, so **the sim can produce eight columns truthfully and cannot produce
eight rows at all.** It publishes `rows: 1`, and a consumer can see the
vertical dimension is absent instead of reading eight identical copies of one
row.

Non-abstract, with a no-sensor default, following `NO_SENSOR_CM`'s precedent:
`ReplayRobot` and `TeleopRobot` inherit it and answer all-unusable -- the same
honest no-op they already give for `get_distance()`. `MockRobot` synthesises
from `cast_ray()`. Nothing in `brain/` reads it yet.

Extend `tests/test_robot_contract.py` (S1's conformance suite) to pin the shape
across all four backends, the way S2 extended it to pin pixels.

**Press this.** A depth strip under the twin's FPV canvas -- eight zones,
coloured by range, grey where unusable. Drive the D-pad at a wall.

**Done when** the strip tracks the FPV view and the contract suite passes on
all four backends.

### M3 -- The tri-state zone, and a centre-zone veto

**Why.** `tof/src/lib.rs` distinguishes three outcomes and argues for it:

> ST's status byte is the difference between "nothing is there" and "I could
> not tell", and collapsing them loses the distinction a map most needs: empty
> space is information, an unusable measurement is not.

`sim/sensors.py` collapses them -- a dropout reads `0.0`. **That is not a
defect and this phase does not change it.** Its docstring's defence holds:
`0.0` fails toward stop, and threading `Optional[float]` through
`RobotInterface`, `robot/server.py`'s JSON and `RemoteRobot`'s `float()`
coercion would touch every layer for a case `robot/safety.py` already handles.

The defence works *because there is no mapping consumer*. M2 creates one, and
the consequence is concrete: **a "nearest cell" veto over a grid whose failed
zones read `0.0` would stop constantly.** The same trap exists on hardware and
arrives differently -- a VL53L5CX zone failure is a status byte, not a range --
so the rule that generalises is that a failed zone must never enter the range
comparison at all, in either direction. That is what makes the tri-state
load-bearing rather than tidy.

**Build.** The scalar `get_distance()` keeps its fail-safe collapse, unchanged.
The grid carries `Range` / `NoTarget` / `Unusable`, because the grid has a
consumer that needs the difference.

Then `robot/safety.py` reduces the **centre zones to one scalar** and compares
that to the threshold, exactly as Microduck's `Hit.range` is meant to be used.
Side zones are informational. The veto stays a number against a threshold, so
nothing about the safety layer's shape changes; scalar `get_distance()` stays
the veto for any backend with no grid, so nothing regresses.

**Press this.** `sim.sensor_noise.enabled: true` with a dropout rate, then
drive at a wall. Unusable zones read grey and distinct from far zones, the
collar fires on centre-zone hits, and the robot does not stop dead on dropout.

**Done when** a wall approached off-centre stops the robot at the same distance
as one approached head-on, and dropout is visibly not the same thing as clear
floor.

### M4 -- Refusals are state, and manual preempts autonomous

**Why.** Microduck's state stream must report what was *refused*, with a
reason, because a teleop UI showing the stick forward and the robot still is
unusable (`robotd-design` §3.2). Separately, it lists authority priority
between the physical controller, the app and the autonomous layer as something
to decide rather than let emerge (`architecture` §6).

Both gaps exist here. A vetoed move surfaces as a log line. The D-pad talks
straight to the robot server during a remote mission and **nothing on the
server knows two drivers exist** -- verified: `robot/server.py` has no notion
of a driver at all. The only guards are the twin refusing to start a local loop
during a remote mission, and the brain's 409 on a second `/mission/start`.

**Decide**, in writing, in `AGENT-HARNESS.md`: **stop > manual D-pad > remote
mission > local brain.** Physical and manual always preempt autonomous.

**Build.**

- Every command carries a driver name (a header: `twin-dpad`, `brain`,
  `teleop`). `robot/server.py` records the last driver and returns it on
  `/action` and in `/health`.
- Every refusal carries a reason on the wire: `watchdog`, `safety_distance`,
  `mission_ended`, `preempted`. `/health` reports the last refusal and reason.
- `RemoteRobot` raises when a different driver has moved the robot since its
  own last command; `MissionRunner` ends the mission `preempted`, robot
  stopped, log line naming the driver.
- The twin sends its driver name and shows the last refusal beside the watchdog
  readout.

**Files.** `robot/server.py`, `control/remote_robot.py`,
`control/mission_runner.py`, `web-twin/index.html`, `tests/test_failsafes.py`,
`tests/test_ui.py`, `AGENT-HARNESS.md`.

**Test.** A foreign driver's command between two ticks ends the mission
`preempted` with the robot stopped. A veto's reason round-trips to the twin.
Re-introduce silent last-writer-wins and watch the test go red first.

**Press this.** Start a remote mission, tap the D-pad. The mission ends
"preempted by twin-dpad", the car does what the pad said, and the readout says
why the brain's last move was refused.

### M5 -- One health command

**Why.** "What is wrong with this robot" does not divide into hardware and
software until after it is answered, so Microduck answers it with one command
that exits non-zero when the robot is unhealthy or unreachable (`architecture`
§8.4). And only conditions a release can be blamed for reach the verdict: a
robot updated on a low battery must not roll its release back and then judge
the replacement on the same battery (`robotd-design` §3.4, invariant 5).

**Build.**

- `python -m control.health`: asks the robot's `/health` and the brain's
  `/mission/status`, prints one answer, exits non-zero if either is unhealthy
  or unreachable. `--json` for the same content.
- Verdict inputs, and nothing else: each process reachable, the watchdog loop's
  last poll fresh, the brain's tick rate against its target. A loop at 60% of
  target is alive, answers every request, and is broken; the rate is what shows
  it.
- Description, never verdict: distance reading, watchdog age, last driver, last
  refusal, and later battery and motor temperature. Written as a rule next to
  the code.
- Both servers log an identity line first, at warning level: service, git
  revision, executable path, config path. The executable path is what separates
  "the symlink moved" from "the update worked".

**Files.** `control/health.py` (new), `robot/server.py`,
`control/brain_server.py`, a health line in the twin's Settings,
`tests/test_health.py`.

**Test.** Unreachable brain: non-zero. Stale watchdog poll: non-zero. A `0.0cm`
distance reading with everything else fine: zero, reported as description.

**Press this.** Kill the brain server. The Settings health line flips to
unhealthy and names the half that failed; the command's exit code follows.

### M6 -- A process start never moves the robot

**Why.** `robotd`'s invariant 2: an update restart leaves a standing robot
standing. On the Pi, `Restart=on-failure` will restart the robot process
mid-mission.

The point of doing it now is that **the test is writable before the code it
governs exists.** It is the rule `HardwareRobot` inherits on the day it is
written, rather than the rule someone derives after the first restart pulses a
wheel.

**Build.** A contract test across all four backends
(`tests/test_robot_contract.py`): constructing a backend and calling nothing
issues no motion and no camera pan. Pose unchanged on `MockRobot`; no HTTP call
from `RemoteRobot`.

**Press this.** Restart the robot server during a mission. The map does not
twitch.

### M7 -- Nothing falls back silently

**Why.** `duck-ipc-proto` records a lesson worth having. Microduck used to
refuse the handshake on a protocol version mismatch, and it was wrong: it
failed calls that were perfectly serveable, including `update apply`, which is
how a version skew ends. What refuses now is narrower and self-describing -- an
unknown method returns `METHOD_NOT_FOUND` naming the method, and every params
type denies unknown fields, so a moved parameter returns `INVALID_PARAMS`
naming the member.

**`/navigate`'s HTTP path already does this correctly** and an earlier draft of
this plan said otherwise. `service/vision_analyze/app.py:268-280` validates
`model_id` and `prompt_variant` against the server-side allow-lists and returns
a 400 naming the allowed set.

The residue is two narrower things:

- `vision_core.py:561` -- `DEFAULT_PROMPT_VARIANT =
  os.environ.get("NAVIGATE_PROMPT_VARIANT", "default")` is never validated, and
  `vision_core.py:624` falls back to the default template silently.
- Unknown request-body fields are ignored throughout (`body.get(...)`).

**The first is latent rather than live, and M1 arms it.**
`cloudformation/service.yaml:31,269` currently templates `NavigateModelId` and
nothing else, so no stack passes the variant today. The moment one does --
which is what shipping `bearing-only` or promoting `center-third-path` needs --
a typo deploys a service that serves `default` while every recorded walk claims
the variant that was asked for.

That is exactly the `NavigateModelId` trap the Stage 0 notes already record: it
always passed a value, the env var won, and every walk before 2026-08-29 was
Sonnet 4.5 while being diagnosed as Nova. This one is worse, because it needs
no drift between two files -- only a typo in one.

**Build.** Validate `NAVIGATE_PROMPT_VARIANT` against `NAVIGATE_PROMPT_VARIANTS`
at import and fail loudly. Reject unknown body fields by name. Do it *before*
adding the stack parameter that arms it.

**Press this.** Start the service with `NAVIGATE_PROMPT_VARIANT=bearing-onlyy`.
It refuses at boot naming the variant, instead of coming up healthy and
serving `default`.

### M7b -- One shipped wording

**Why.** Five wordings of `/navigate` exist. They were never features; they
were an experiment, and the experiment has finished. Section 2's argument is
that a single monocular frame does not contain metric depth, so no wording
recovers it -- and four attempts to reword the obstacle question, plus one
deletion of it, are what established that. `center-third-path` settled it
structurally rather than by yet another comparison: Opus answers `blocked` on
66% of frames and Qwen on 5%, but the disagreement is perfectly **nested** --
every frame Opus calls `open_floor`, Qwen does too. They read the picture the
same way and cut the threshold in different places, and **a threshold has no
wording.**

Three further facts point the same way:

- **On the model actually shipped, the wording is already a no-op.** In the
  5x3 table, Claude Opus 4.5 reads 0.591 under `default` and 0.591 under
  `bearing-only`. The wording column only separates models this project does
  not run.
- **Every live variant is a way for a walk to become unattributable**, and
  this project has now paid for that three times: the `NavigateModelId` env
  var that made every pre-2026-08-29 walk Sonnet while it was diagnosed as
  Nova; `MissionStartRequest` silently discarding `prompt_variant` for as
  long as the twin had been sending it; and the stray frame found on
  2026-09-03 (below).
- **A wording picker on the phone is a control with no correct setting.**
  Offering an operator five ways to ask an unanswerable question invites the
  reading that one of them is right.

**What this is not.** It is not "delete the variants". Replay is the only
controlled comparison this project has -- two live walks vary the operator's
path as well as the model -- and deleting the wordings deletes the ability to
re-run the experiment. The split is between *shipped* and *experimental*, not
between kept and thrown away.

**Depends on M7**, which is what makes "one shipped wording" true rather than
aspirational: until `NAVIGATE_PROMPT_VARIANT` is validated at import, a typo
deploys a service that serves `default` while every walk claims otherwise.
Do M7 first.

**Build.**

- **`default` is the one wording on the live path.** It is the incumbent, it
  is identical to `bearing-only` under Opus 4.5, and it is the column the
  3x3 established never reaches the always-FORWARD degenerate mode on any
  model. Not `bearing-only`: it is the best replay column measured, on one
  walk, against a `collision` count only a real sensor answers -- and
  promoting a default off one walk is exactly how the `NavigateModelId`
  mistake happened.
- **Remove the wording picker from the live mission and Robot-view paths.**
  The model picker stays; the model is a real operator choice with measured
  consequences, and the wording is not.
- **Keep all five reachable from replay only** (`control/walk_replay.py`'s
  allow-list, and the admin console). They cost nothing there and cannot
  contaminate a live walk.
- **`/navigate` keeps accepting `prompt_variant`** -- replay needs it -- but
  the twin stops sending one, so a live walk records the server's own
  default and can be attributed to it.
- **Fix the stray-frame contamination** found on 2026-09-03: a `/navigate`
  call still in flight when a Robot-view session is stopped gets written
  into the *next* walk's directory, carrying its own wording and its own
  sequence number. Walk `bottle-opus-4-5-center-third-path-20260902-163923`
  holds sixteen `center-third-path` frames and one `bearing-only` frame at
  `seq: 37`. This is the same orphaned-in-flight-call class the Robot-view
  HUD already fixed with `guidanceEpoch`, one layer down: the recorder
  needs the same epoch. **Until it is fixed, no walk that follows another
  walk within one session is safely attributable**, which undercuts every
  measurement this plan makes.

**Files.** `web-twin/app.js` (the picker and the recorder's epoch),
`control/brain_server.py`, `control/walk_replay.py`,
`service/vision_analyze/vision_core.py`'s "Per-route models" note, the Stage
0 notes in `CLAUDE.md`, `tests/test_ui.py`.

**Test.** A live walk records the server's default wording and no other. Two
walks recorded back to back in one session contain no frame from each other
-- re-introduce the straggler and watch it go red first, the way
`guidanceEpoch`'s own fix was checked.

**Press this.** Start a Robot-view walk, stop it, immediately start another.
The second walk's frames are all its own, and neither walk's recorded
wording is something anyone had to choose.

**Done when** the live path offers exactly one wording, replay offers all
five, and two back-to-back walks are cleanly separated.

---

---

## 5. Hardware-day phases

### M8 -- Floor rejection and the too-close band

**Why.** `kinematics/src/tof.rs` is pure geometry turning a raw grid into
usable points, and it handles the two nuisances that appear on day one:

- **Floor returns.** A downward-tilted sensor sees the floor at every range. A
  beam whose slant range times its downward component reaches sensor height
  (times a safety factor, for pose error) hit floor, not obstacle.
- **A too-close noise band.** Sub-10cm returns are cover-glass crosstalk and
  pulse pile-up -- discarded rather than believed.

Its output type is the one M3's veto wants: `Hit { point, range }`, where
`range` is horizontal distance from the vertical axis -- "the number obstacle
avoidance compares against a stop threshold". This is also what defines "the
path cells" that M10 reduces: without it, there is no principled way to say
which zones are in the way.

It lives in `HardwareRobot`, on the hardware side of the abstraction, not on
`RobotInterface`. Ports as Python geometry, with no Rust.

**Most of this cannot be validated in the sim**, and belongs in
`PLAN-sim-hardening.md` section 7 with the other hardware-only items: the grid
world has no floor and no vertical dimension, so there are no floor returns to
reject. Only the too-close band and the horizontal-range projection are
testable before the sensor exists. Say so rather than writing a sim test that
passes because the failure it guards against cannot occur.

**Press this.** Tilt the camera down with the twin's look controls; the depth
strip stays clear instead of reporting an obstacle in every zone. Before
hardware this phase has **no UI proof**, and that is this plan's one written
exemption -- the readout that would show it if it broke is M2's strip going
uniformly red on a downward tilt.

### M9 -- The camera cannot wedge `stop`

**Why.** Microduck keeps perception outside the motor process because bringing
a sensor up takes seconds, shares a bus, and may not be fitted at all; a retry
loop for that does not belong next to the motors. On the Pi, `picamera2`
initialisation and capture will run in the same process as `/stop`.

**Build.**

- `HardwareRobot.__init__` does not centre the camera or pulse the wheels
  (M6's rule, on the real backend).
- `get_camera_frame()` runs capture under a timeout and returns an error dict
  rather than hanging the caller. The brain's B3.2 budget already knows what to
  do with that.
- Whether the camera becomes its own small process, read by `robot/server.py`
  over localhost, is decided on hardware day by measuring init time and stall
  behaviour -- not in advance.

**Files.** `robot/hardware_robot.py`, a new 5.5 item in
`HARDWARE-READINESS.md`.

**Press this.** Cover the lens and pull the ribbon mid-mission. `/frame`
errors, `/stop` still answers, and the mission ends on the vision budget.

### M10 -- Clearance from a real sensor

**Why.** `tofd` publishes an 8x8 depth matrix from a VL53L5CX at 15Hz over
I2C. `center-third-path` asks what is in the bottom half of the centre third of
the frame -- which is a description of a depth matrix's centre columns and
lower rows, answered in hardware.

It also settles `HARDWARE-READINESS.md` 5.3 by construction. That item warns
that the frontier policy's `look_left(); get_distance()` peek silently degrades
to noise if the ultrasonic turns out to be chassis-mounted, and says the fix
would then be "a real design change (turn the chassis to peek, or add a second
distance source)". **Mount the ToF on the pan/tilt with the camera** and the
peek is metric as well as visual. A 45-degree field of view means chassis
mounting would also work, but that is the fallback, not the preference --
gimbal mounting gives both.

**Build.**

- Add the sensor to the buy list next to the kit.
- `HardwareRobot` fills `get_depth_grid()` from the sensor and reduces the path
  cells (M8's geometry) for `get_distance()` when fitted, the ultrasonic
  otherwise. `RobotInterface` does not change again.
- Write the fusion rule into `HARDWARE-READINESS.md` 5.4: camera for bearing
  and room, sensor for clearance, the prompt is never asked for distance.
- Compare against M1's recorded "what the sensor must catch" column.

**Press this.** D-pad toward a chair leg a single ultrasonic beam misses. The
safety collar flashes before contact.

### M11 -- The small updater

**Why.** Three of Microduck's documented failures are what
`git pull && systemctl restart` on a Pi produces in miniature: an update killed
before its health check ran, a board that silently came back on a release
nobody asked for, and a journal that lived in RAM and was empty after a power
cut. Raspberry Pi OS also defaults to a volatile journal.

Take the shape, not the machinery. No signing, no release channel, no boot
counter -- those are built for a fleet in strangers' homes.

**Build.**

- `deploy/`: the two units B5 already plans, plus `APP_SHARED_SECRET` set in
  the environment -- `require_secret()` is a no-op without it, and a server on
  the LAN is Microduck's "wrong interface" bug class.
- Installs land in `releases/<git-rev>/` under a `current` symlink.
  `deploy/install.sh`: unpack, flip, restart, run `control.health` (M5), and on
  failure flip back and restart. Keep one previous release.
- `robot-boot-check.timer`: run the health command a minute after boot and
  write the answer to the journal.
- A journald drop-in for persistent storage with a size cap.
- Config stays outside the release directory, so it survives both an install
  and a rollback.

**Files.** `deploy/` (new), `PLAN-brain-relocation.md`'s B5 entry, the CLAUDE.md
repo map.

**Press this.** B5's own proof stands: reboot the Pi, open the twin on a phone,
start a mission with no laptop on the network. Then install a build that fails
health on purpose -- the Pi comes back on the previous release and the boot
check says so.

---

## 6. Conditional

### M12 -- Novelty-grid exploration memory

**Why.** Microduck's brain drives Wander from a **novelty grid**: where it has
already been, decayed over time. This is a candidate answer to the note in
CLAUDE.md that "the single-step `/navigate` contract has no memory of which way
it already turned", and to the instruction that the next attempt should change
the question's *shape*. It is the other shape-change answer, orthogonal to
depth: M1-M3 fix *how far*, this fixes *where I have been*.

`searched_rooms` is the precedent and the mechanism: client-side memory passed
into the call and backfilled from the reply, with the `vision_fn(frame) ->
scene` contract untouched (`AGENT-HARNESS.md` section 10). A novelty grid is
the same move at finer resolution.

**Conditional on M1.** If removing the obstacle question stops the
turning-on-the-spot, the oscillation this phase addresses may already be gone.
Measure before building.

**Tension to resolve first.** The frontier-preference explorer in
`brain/agent.py` is explicitly "keep, do not extend" (`PLAN-sim-hardening.md`
2.2). This phase lands in the vision policy, not there.

**Press this.** The remote-brain log shows the model being told what it has
already tried from this spot; the twin map shades visited cells.

---

## 7. Not taking, on purpose

- **Rust.** The brain is four HTTP calls in a loop. Python is the right size.
- **JSON-RPC over unix sockets.** It would break the base-URL trick that lets
  the brain run on the Mac, the Pi or Fargate unchanged. Microduck itself
  concedes HTTP for the LLM path (`architecture` §5.3).
- **WebRTC and `mediad`.** A JPEG on demand at 1-2fps is what their design says
  an agent wants, and it is what `/frame` already is. Telepresence is not a
  goal here.
- **Signed releases, channels, a boot counter, BLE provisioning.** Fleet
  machinery. One robot on one LAN gets M11's subset.
- **The ONNX policies** (`policies/`, MuJoCo, PPO). They walk a biped. A
  PiCar-X has nothing to learn.
- **The 16-state mood machine** (Chill, Zoomies, Preen, Nap...). Charm, where
  this robot's job is task completion. M12 is the one input worth extracting.
- **BLE presence, the chorale, the shared beat, the theremin.** Multi-robot
  social behaviour, with one robot.
- **A local target detector on an NPU.** Deferred, not rejected. If it is ever
  wanted, the recorded walks on EFS are already robot-height footage with
  operator labels -- which is the dataset Microduck says is the real project
  ("Data is the project, not the model").

---

## 8. Ambiguities and unverified facts

**Answered by M1, not by argument:** whether a depth sensor changes what the
Stage 0 gate is measuring. See M1's "Why".

**Hardware facts, none verified here:**

- Does the PiCar-X Robot HAT leave an I2C address and bus free for a VL53L5CX?
- Does the pan/tilt have the payload capacity and cable routing for it? M10
  prefers gimbal mounting; chassis mounting is the fallback.
- Current price and availability of a breakout.

**Provenance.** Everything cited above, checkable at source.

| Claim | File or section |
|---|---|
| Camera = direction, ToF = distance; novelty grid; mood model; "data is the project" | `docs/ideas/autonomous_behavior.md` |
| A bearing, not a box | `duck-detect/src/lib.rs:47` |
| `Frame` shape; `Range` / `NoTarget` / `Unusable` | `tof/src/lib.rs` |
| 8x8 at 15Hz over I2C | `tof/src/main.rs:26` |
| Floor rejection, too-close band, `Hit { point, range }` | `kinematics/src/tof.rs` |
| The five invariants; no motion on process start | `docs/design/robotd-design.md` §1.5 |
| State must report refusals, with a reason | `docs/design/robotd-design.md` §3.2 |
| What may reach the health verdict | `docs/design/robotd-design.md` §3.4 |
| Safety and authority arbitration | `docs/design/architecture.md` §6 |
| Health is one question, so it is one command | `docs/design/architecture.md` §8.4 |
| Server-side agents need not go through WebRTC | `docs/design/architecture.md` §5.3 |
| Refuse per-method and per-field, never per-handshake | `duck-ipc-proto/src/lib.rs` |
| Health gate and rollback | `docs/design/updater-design.md` §8 |
| 500ms deadman default | `robotd-params/src/lib.rs:771` |

---

## 9. How this document was made

Two independent plans were written on 2026-09-02 from the same read of the
Microduck repo -- one by Claude Opus 5, one by Claude Fable 5.1 -- and merged
here after each evaluated the other. Recorded because two of the corrections
came out of that and would otherwise look like ordinary edits:

- **M7's premise was wrong in the first draft.** It claimed a misspelled
  `prompt_variant` silently ran `default`. `app.py:268-280` already refuses it
  with a 400. The phase survives, shrunk to the env-var default and unknown
  body fields -- which turned out to be the sharper problem anyway, because M1
  is what arms it.
- **M2's interface question was resolved by combining both answers.** One plan
  proposed `get_depth_grid()` on `RobotInterface`, the other folding the sensor
  into `get_distance()` with no interface change. The first is testable in the
  sim today and the second keeps the veto surface small; M2 plus M3's scalar
  reduction is both.
- **M3's dropout interaction was caught by the plan that did not propose the
  tri-state.** A "nearest cell" veto over `sim/sensors.py`'s `0.0` dropout
  would stop constantly. That is what moved the tri-state from tidy to
  load-bearing.
