# Plan: a teleop robot -- close the loop on Robot view

> **Status note, 2026-09-28.** T1-T3 stand and are how a phone walk drives
> the brain today. **The AWS deployment T4 describes was deleted
> 2026-09-05** (`PLAN-aws-cost-redesign.md` Stage 2), with the shared ALB
> and the main brain the recording proxy forwards to, so **T4 and "Recording
> proxy" below are history**. Run teleop locally instead:
> `ROBOT_MODE=teleop bash service/tunnel/run.sh`, then `ngrok start picar`,
> and point the deployed twin's Settings at `https://<domain>` and
> `https://<domain>/brain` (`service/tunnel/run.sh`'s header, `CLAUDE.md`
> section 6). The `/frame` 503 follow-up in the definition of done is
> fixed. The "watched on a phone" rule quoted in the goal was retired
> 2026-09-25 (`CLAUDE.md` section 7).

Status: **Done, 2026-08-28.** T1-T4 built (`sim/teleop_robot.py`, the
`POST /teleop/frame` route on `robot/server.py`, Robot view's "Drive via
brain" switch in `web-twin/index.html`, and a live AWS deployment -- see
T4), two bugs found and fixed against the real deployment the same day
(see T4's addendum and "Stall detection" below), and the actual walk this
whole plan was for has now happened: a real phone, driving a real
`MissionRunner` mission through `control/brain_server.py`, correctly
found its target (`OUT: FOUND`, `VF: 0`) and reported `REACHED` --
satisfying the definition of done at the bottom of this file. Written
against the repo as of `172770b`.

Goal: today, walking around with a phone in Robot view proves the vision
*model* can read a room. It proves nothing about the *brain* -- the
mission memory, the failure budget, the timeouts, `MissionRunner`, any of
`control/`. This plan wires Robot view into `control/brain_server.py` so
that a walk through a real house exercises the exact same code the car
will run on, live and closed-loop, with the section 7 rule in mind:
**a phase isn't done until someone holding a phone can watch it work.**
Get this right and buying the hardware becomes what `PLAN-brain-relocation.md`
already promises for the robot side -- a config change, not new code --
except now it's true for the *brain* side too: no code currently reads a
live camera through `MissionRunner`; after this, a phone is one working
instance of "live camera," and the car is another.

---

## 0. What already works (don't rebuild this)

- `control/brain_server.py`'s `policy: "vision"` already runs
  `brain/vision_agent.py`'s `VisionAgent` through `MissionRunner`, with
  B3.2's per-call timeout and failure budget wrapped around it
  (`control/mission_runner.py:_guarded_vision`). This is fully built --
  it has just never been driven by a live camera, only recorded walks
  (`sim/replay_robot.py`) and drills (`control/drills.py`).
- `robot/interface.py` + `robot/factory.py` is exactly the seam this
  needs. Adding a robot "body" for this is a new `mode:` branch, the same
  shape as `sim` -- no change to `control/`, `brain/`, or the mission
  loop's shape.
- `sim/replay_robot.py` is 80% of the design already, minus liveness: it
  proves a `RobotInterface` can be "a body made of photographs" and
  documents its own honest limits (open-loop, no distance sensor, no room
  label) -- this plan inherits the same limits, plus closes the open-loop
  one.
- Robot view's frame capture and 2.5s cadence (`web-twin/index.html`,
  `guidanceMode === "robot"`) already produces real JPEGs at a sane rate.
  Nothing about frame capture changes -- only where the frame goes.

## 1. Target topology

Today, two paths exist side by side and never touch:

```
Guide tab, Robot view          Sim tab, Remote brain
  phone camera                   (no camera -- MockRobot's grid world)
    |                              |
    v                              v
  POST /navigate               POST /mission/start
  (service/vision_analyze)       |
    |                            v
    v                          control/brain_server.py -> MissionRunner
  HUD renders the reply          |
  nothing moves,                 v
  no mission exists            RemoteRobot -> robot/server.py -> MockRobot
```

After this plan, Robot view becomes a third *source* for the same mission
loop, not a separate feature:

```
Guide tab, Robot view ("drive via brain")
    |
    | POST /teleop/frame  (new)
    v
robot/server.py  --  mode: teleop  --  sim/teleop_robot.py  (new)
    ^                                        |
    | GET /frame (existing, unmodified)      | get_camera_frame() returns
    |                                        | the last pushed frame
control/brain_server.py -> MissionRunner -> VisionAgent -> vision_fn_for()
    |                                                            |
    v                                                            v
 GET /mission/status  -->  Robot view HUD                 service/vision_analyze
 (polled at the same 2.5s cadence Robot view already uses)  (Bedrock, unchanged)
```

The phone is the camera. **The person holding it is the motor** --
`drive_forward`/`turn_left`/etc. on this backend don't move anything; they
just log what the model decided, the same way `ReplayRobot`'s pan methods
are an honest no-op. The next frame you get is whatever you actually
photograph after reading that instruction and walking -- which is what
makes this closed-loop where `ReplayRobot` is open-loop.

---

## Phase T1 -- `TeleopRobot`, a backend with a live camera and no motor -- BUILT

**Problem.** No existing `RobotInterface` backend accepts a frame pushed
in from outside. `MockRobot` renders its own (synthetic, grid-only, and
not real JPEG bytes until S2 lands); `ReplayRobot` reads a fixed directory
decided in advance.

**Build.** `sim/teleop_robot.py`, a `TeleopRobot(RobotInterface)`:

- `push_frame(image_base64, media_type)` -- not part of `RobotInterface`,
  called only by `robot/server.py`'s new route. Stores the frame and bumps
  an internal sequence number, then notifies anyone waiting on it.
- `get_camera_frame()` -- blocks until a frame *newer than the last one
  this method returned* is available, or `stall_timeout_s` elapses
  (default 15s -- see "Stall detection" below), then returns it in S2's
  `{image_base64, media_type, metadata}` shape. Implemented with a
  `threading.Condition`, not a sleep-poll loop.
- `drive_forward` / `reverse` / `turn_left` / `turn_right` / `look_*` --
  log the call, return an ack dict. No position, no motion.
- `stop()` -- ack.
- `get_distance()` -- returns `NO_SENSOR_CM` (999.0, matching
  `replay_robot.py`'s constant and its documented reasoning: honestly
  represent "no distance sensor here" rather than fake a safe number).

**Files.** New `sim/teleop_robot.py`. `robot/factory.py` gains a
`mode == "teleop"` branch. `config/robot.yaml` documents the new mode
(commented out, same as `hardware` is today before Phase 11).

**Test.** `tests/test_teleop_robot.py`: push a frame, assert
`get_camera_frame()` returns it verbatim; assert every movement method
returns successfully and changes no internal position; assert
`get_distance()` never drops below any realistic `min_distance_cm`, so
`robot/safety.py` never blocks a move against this backend (same
assertion style as the safety-inert claim in `replay_robot.py`'s
docstring). Plus the stall case: push one frame, consume it, call
`get_camera_frame()` again with a short `stall_timeout_s` and no second
push, and assert it raises `TeleopStall` at roughly that deadline --
not immediately, not never.

**Twin proof.** None yet on its own -- this phase is backend-only. The
honest "no UI change" answer section 7 allows once per phase.

**Built.** `sim/teleop_robot.py` (`TeleopRobot`, `TeleopStall`), the
`mode == "teleop"` branch in `robot/factory.py`, and `config/robot.yaml`'s
new `teleop:` block (`stall_timeout_s: 15.0`, documented as needing to
stay under `brain.tick_timeout_s`). `tests/test_teleop_robot.py` -- 7
tests, including one that measures the stall firing at roughly the
configured deadline, not immediately or never. `push_frame()` returns
`{"seq": N}`, used by T2's route to confirm receipt.

---

## Phase T2 -- the ingress route -- BUILT

**Build.** `POST /teleop/frame` on `robot/server.py`, secret-gated like
`/action`/`/stop`/`/distance`/`/frame` already are. Calls
`robot.push_frame(...)`. Returns a clear error if the loaded backend
isn't a `TeleopRobot` (a `mode: teleop` mismatch should fail loudly, not
silently accept a frame nothing will ever read).

**Files.** `robot/server.py` only.

**Test.** Extend `tests/test_server.py`: POST a frame to
`/teleop/frame`, then `GET /frame` and assert it's the same frame back --
proving the existing pull side needs no changes at all.

**Twin proof.** None yet -- still no UI wired to it.

**Built.** `POST /teleop/frame` on `robot/server.py`, secret-gated the
same way as `/action`/`/stop`/`/distance`/`/frame`. Detects a non-teleop
backend by `hasattr(robot, "push_frame")` rather than an isinstance
import, so this file's own docstring claim -- unchanged regardless of
backend -- stays true. Returns 400 with a clear message when
`config/robot.yaml`'s `mode` isn't `"teleop"`. Two new tests in
`tests/test_server.py`: a push-then-pull round trip through the existing
`/frame`, and the 400 on a non-teleop backend; the existing secret-gating
tests were extended to cover the new route too.

---

## Phase T3 -- Robot view drives the brain -- BUILT

**Build.** A mode switch in Robot view, next to "Record this walk" and
gated the same way (`state.brainConnected`): **"Drive via brain."** When
on:

- Start button -> `POST {brainUrl}/mission/start` with
  `policy: "vision"`, `target_object` = the existing guidance-target
  field (no new input needed -- Robot view already collects this).
- Each captured frame -> `POST {serverUrl}/teleop/frame` instead of
  `POST {visionUrl}/navigate`.
- HUD renders from polling `GET {brainUrl}/mission/status` at the same
  2.5s cadence -- `last_action`, `last_reasoning`, `outcome`,
  `vision_failures` -- instead of the raw `/navigate` reply. This is a
  strictly richer readout than today's (it's the same status payload the
  Sim tab's Remote brain panel already renders).
- Stop button -> `POST {brainUrl}/mission/stop`.

**Files.** `web-twin/index.html` only. No change to `control/` or
`brain/` -- this phase is entirely "point an existing client at an
existing endpoint."

**This mode stays serial, and that is now load-bearing.** Robot view's
ordinary `/navigate` calls overlap (up to `GUIDANCE_MAX_IN_FLIGHT` = 2,
see `FEATURES.md` section 1), but driving via brain pins itself to one
call in flight: `guidanceStep()` reads
`driveViaBrainActive() ? 1 : GUIDANCE_MAX_IN_FLIGHT`. A real
`MissionRunner` mission is strictly one action at a time -- the step
budget and every failsafe in `AGENT-HARNESS.md` §6 are built on that --
so overlapping here would have the brain deciding from frames it has
already acted past. Pipelining buys a *person* more readings to integrate
across; it buys a robot executing each decision nothing, and costs it
coherence. If that pin is ever removed, the failure will not look like a
crash: it will look like a mission that turns the wrong way for reasons
that made sense two frames ago.

**Built.** A "Drive via brain" switch next to "Record this walk" in Robot
view, gated by `driveViaBrainActive()` (needs the brain connected and
`guidanceMode === "robot"`, same shape as `recordingActive()`) and
mutually exclusive with it -- recording saves a `/navigate` reply per
frame, which this path never produces. `startGuidance()` latches the
choice into `state.guidanceViaBrain` at Start (so Stop knows what *this*
session did, even if the toggle changes mid-walk) and calls
`POST /mission/start` with `policy: "vision"`; `stopGuidance()` fires
`POST /mission/stop` the same fire-and-forget way `recordWalkFrame()`
already does. `guidanceStep()`'s existing loop is unchanged except for one
branch: when driving via brain, `driveViaBrainStep()` pushes the frame to
`POST /teleop/frame` and polls `GET /mission/status`, then
`missionStatusToRobotResult()` reshapes that into the same
`{action, reasoning, target_reached, ...}` shape `/navigate` replies with
-- so `renderRobotOverlay`/`renderRobotStatus`/`renderRobotTelemetry`/
`announceRobot` and the existing found-streak-to-pause logic all keep
working unchanged, for either source. `renderRobotTelemetry` gained two
extra rows (`OUT`, `VF`) when driving via brain -- `vision_failures` and
`outcome`, fields `/navigate` never had -- and marks `VIS`/`OBS` as `N/A`
rather than a false `CLEAR`/`NO`, since MissionRunner records a decision,
not the per-frame reading that produced it. A mission that ends any way
other than found/room_reached (stopped, failed, max_steps -- including a
live T1 stall or a B3.2 vision failure) surfaces through the same error
path a lost `/navigate` connection would, and stops the tick loop the
same way arrival does.

Verified: the inline script still parses (`node --check`) and the full
backend suite still passes (142) -- this phase touches only
`web-twin/index.html`, so no backend test coverage changed. The phase's
actual proof is the manual one this plan calls for: walking a real room
with a phone, which needs a reachable robot/brain service and hasn't been
run yet.

**Twin proof -- this is the phase that actually answers the question.**
Guide tab -> Robot view -> "Drive via brain" -> Start, then walk a real
room:

- You should see the same step counter / rooms-searched / log-tail the
  Sim tab's Remote brain panel shows, now advancing from your own steps.
- Stand still (or point the phone at the ceiling) and don't submit a new
  frame: within `stall_timeout_s` (15s) the mission should end `failed`
  with a stall message in the log -- a real failsafe firing, with no
  drill involved. Separately, pointing the phone somewhere the model
  can't parse a scene from exercises B3.2's vision-failure budget the
  same way.
- Background the tab per B4's existing proof: the mission is a service
  call, not a page state, so it should still be running when you come
  back.

---

## Phase T4 -- deploy alongside the sim-mode twin on AWS -- BUILT

> **History: deleted 2026-09-05** with the rest of the ECS stacks. The
> templates (`cloudformation/teleop-robot.yaml`, `teleop-brain.yaml`) are
> still in the tree but back nothing. See the status note at the top for
> how teleop runs now.

**Problem.** Robot view's camera needs a secure context (`https://` or
`localhost` -- confirmed in `web-twin/index.html:1055`, which is now
`startGuidanceInner()`'s `window.isSecureContext` check in
`web-twin/app.js`); a LAN IP does
not qualify, so a phone off the same machine as `uvicorn` needs a real
HTTPS deployment to test any of T1-T3 at all. The existing ECS
deployment already solves this (`cloudformation/cdn.yaml`'s CloudFront
front end) -- but it runs `mode: sim`, and `robot/factory.py` picks one
backend per process at boot. Flipping the deployed twin to `mode: teleop`
would work, but takes the Sim tab away while it's flipped; the goal here
was both live at once, the same way `brain.yaml` sits alongside
`twin.yaml` rather than replacing it.

**The wrinkle this surfaced.** `robot/server.py`'s route set
(`/action`, `/frame`, ...) is already exhaustively claimed by
`twin.yaml`'s `ListenerRule`s on the one shared ALB -- a second instance
of the same file can't claim the same literal paths there. And
`control/brain_server.py` picks one `robot_url` at process start, so two
brains are needed too, not just two robots, for sim and teleop to be
independently drivable at the same time.

**Build.**

- `robot/factory.py` -- `ROBOT_MODE` env var overrides `config/robot.yaml`'s
  `mode`, same pattern `control/brain_config.py` already uses for
  `ROBOT_URL`/`VISION_URL`.
- `robot/server.py` and `control/brain_server.py` -- `ROUTE_PREFIX` env
  var, prepended to every route. Empty by default, so the existing twin
  and brain deployments are unaffected. This is what lets a second
  instance of each file share the existing load balancer instead of
  needing one of its own.
- `control/brain_config.py` -- `ALLOW_RECORDING` env var override, so a
  brain with no EFS mount fails loudly (403) on a stray recording call
  instead of silently writing to its own ephemeral disk.
- `cloudformation/teleop-robot.yaml` / `teleop-brain.yaml` -- new sibling
  stacks (new files, not parameterized versions of `twin.yaml`/`brain.yaml`
  -- chosen over the DRY alternative specifically so a mistake here can't
  touch the already-running deployment). Same images as the existing
  services (`service/twin/Dockerfile` / `service/brain/Dockerfile`
  unchanged -- they already `COPY` exactly the code these env vars live
  in), own ECR repo/secret/IAM roles/`ListenerRule` each, sharing only the
  existing NLB/ALB/cluster/CloudFront distribution and the one
  vision-analyze service. Path prefixes `/teleop-robot` and
  `/teleop-brain`; `ListenerRule` priorities 40/41/50 (existing:
  twin 10/11, brain 20, admin 30).

**Deployed and verified live**, not just written: both stacks bootstrapped
(`DesiredCount=0` -> push image -> `DesiredCount=1`, same dance
`brain.yaml` documents), both ECS services reached steady state, and:

- `GET https://<cloudfront-domain>/teleop-robot/health` -> 200, over
  real HTTPS.
- A push to `/teleop-robot/teleop/frame` came back correctly from
  `/teleop-robot/frame` -- confirming `TeleopRobot` running for real, not
  a demo.
- `GET https://<cloudfront-domain>/action` (bare, unprefixed) still
  answers as the *existing* sim-mode twin's own route (405 on a GET,
  since it's POST-only there) -- proving the two services coexist on the
  shared listener with no collision, which was the entire point of T4.
- `POST /teleop-brain/mission/start` (policy `"frontier"`, to avoid a
  Bedrock call in a smoke test) drove a real tick through the deployed
  brain against the deployed teleop robot over the *internal* ALB: the
  log shows `vision failure 1/3: vision call failed: 'free_space_cells'`
  -- the frontier policy's offline `vision_fn` choking on a frame shape
  it doesn't expect (expected and harmless; `policy: "vision"` is the
  real path and only needs an image) -- which is only reachable if
  `RemoteRobot` actually called the real `teleop-robot` service's
  `get_camera_frame()` and got the pushed frame back. B3.2 counted the
  failure and stopped the car; a manual `/mission/stop` ended it cleanly.

**Not yet done:** an actual `policy: "vision"` mission against real
Bedrock calls, and the phone walk-through itself -- this phase proves the
wiring, not the experience. Each new secret's real value was set via
`aws secretsmanager put-secret-value` (the template's own placeholder is
never the real value, matching `twin.yaml`/`brain.yaml`'s documented
pattern) -- retrieve them with `aws secretsmanager get-secret-value
--secret-id vision-picar-teleop-robot-shared-secret` /
`--secret-id vision-picar-teleop-brain-shared-secret` to enter into the
twin's Settings panel.

**Two real bugs found the same day, during actual phone testing, both
fixed and redeployed:**

1. The twin's Settings panel had no field for the brain's own secret at
   all -- `brainApi()` was silently sending the *robot's* secret as every
   brain call's `x-app-secret` header, which 401s the moment the two
   differ (as they always do once each is its own ECS service). Fixed:
   a real `cfg-brain-secret` field, `state.brainSecret`, and a
   `brainAuthHeaders()` distinct from the robot's `authHeaders()`.
2. `teleop-brain.yaml`'s `AlbListenerRule` excluded `/teleop-brain/health`
   "for consistency with `brain.yaml`," without noticing that
   `brain.yaml`'s exclusion has a structural reason (it shares the twin's
   *unprefixed* namespace, where `/health` is already claimed) that
   doesn't apply to a prefixed service. Excluding it meant Settings'
   "Connect" button for this brain could never succeed over the public
   URL. Fixed by adding it to the rule and redeploying the stack. See the
   "Stall detection" section below for the third, more serious bug this
   same testing round found in `TeleopRobot` itself.

---

## Stall detection: a staleness check, not a block -- corrected 2026-08-28

**Decided:** a phone that stops moving should turn into a real, live
failsafe within a fixed number of seconds -- not silently re-decide
against a stale frame forever.

**Originally built as a blocking wait** ("`get_camera_frame()` blocks for
a frame strictly newer than the last one it returned, up to
`stall_timeout_s`, then raises") and **that design shipped a real bug**,
found the same day against the live deployment: `_last_returned_seq` was
a single cursor shared by *every* caller of `GET /frame`, not just an
active mission's tick loop. Settings' own `connect()` reachability check
calls `GET /frame` too (same as the Sim tab's frame-following, or any
other passive read) -- and once any one caller "consumed" the current
frame, the *next* caller, whoever it was, blocked for the full
`stall_timeout_s` and then 500'd. In practice this meant the ordinary
"Connect" button in Settings took 15 seconds and then failed, with no
mission ever having run. Confirmed live: `curl .../teleop-robot/frame`
twice in a row took 15.2s on the second call.

**Fixed: `get_camera_frame()` is now a staleness check, not a wait.**
`TeleopRobot` tracks *when a frame was last pushed*, not *who has read
one*. Every call returns the current frame immediately if it arrived
within `stall_timeout_s` (default **15s**), or raises
`TeleopStall(RuntimeError)` immediately otherwise -- never blocks. This
makes `get_camera_frame()` a cheap, repeatable, idempotent read again,
matching every other backend, and it is *more* responsive for the
mission-stall case too: a stalled tick now fails on the next attempt
almost instantly instead of occupying up to `stall_timeout_s` of that
tick's own wall-clock time.

`ConstrainedAgent.step()` calls `get_camera_frame()` directly
(`brain/agent.py:96`), **outside** `MissionRunner`'s B3.2 timeout wrapper
-- that wrapper only covers the `vision_fn` call, which is why this
backend owns its own deadline at all.

**That exception needs no new handling anywhere.** It surfaces from
`agent.step()` into `MissionRunner.tick()`'s existing catch-all:

```python
except Exception as e:
    logger.exception("mission step failed")
    self._finish(FAILED, f"step failed: {e}")
    return False
```

-- which already stops the robot and ends the mission `failed` with the
stall's own message in the log. **No change to `control/mission_runner.py`
at all.**

**The B3.3 ordering concern from the original design (`stall_timeout_s`
must stay under `tick_timeout_s`, 30s) is now moot, not just satisfied:**
since a stale read fails immediately rather than occupying wall-clock time
inside the call, there is no longer a blocking duration that could compete
with B3.3's deadline in the first place. 15s remains the default -- a
realistic amount of time to read an instruction and take a step -- but
it's no longer load-bearing for that reason.

**Open detail, not a blocker:** a stall ends the mission on the *first*
miss, with no retry budget -- unlike B3.2's vision failures, which get
`max_vision_failures` (default 3) chances before the mission ends. That
asymmetry is deliberate: a missing frame has nothing to decide on, where
a flaky model response might succeed on retry. Revisit only if a single
stall turns out to be too trigger-happy in practice (e.g. the phone
briefly loses focus switching apps).

---

## Recording proxy: teleop-brain forwards to the main brain -- built 2026-08-28

> **History.** Both brains and the internal ALB this proxies across were
> deleted 2026-09-05. Locally one brain serves both missions and
> recordings, so no proxy is involved; `recording_proxy_url` remains a
> config option.

**Bug found the same day, via real use, not a drill.** Robot view's
"Record this walk" was enabled with the twin's Settings pointed at
`teleop-brain` (left over from testing "Drive via brain" earlier in the
same session) and the app never complained beyond one easy-to-miss toast
-- the admin viewer showed **0 walks, 0 frames** afterward. Root cause:
`teleop-brain.yaml` sets `ALLOW_RECORDING=false` and mounts no EFS volume
at all (see Phase T4 above), so `control/brain_server.py`'s
`record_frame()` 403'd on every single frame. The original design
comment for that -- "the twin's UI already makes 'Record this walk' and
'Drive via brain' mutually exclusive, so this brain should never
legitimately receive `POST /recording/frame`" -- was true about the
*toggles* but silently assumed the client would also always point
Settings' one Brain URL field at the *correct* brain for whichever toggle
was active. Nothing enforced that; switching workflows without
remembering to switch the URL back produced exactly this silent
data-loss failure.

**Decided: proxy the recording endpoint specifically, not the brain
connection.** Two endpoints look similar but are not:

- **`/mission/start` \| `/stop` \| `/status` can never be proxied between
  brains.** A brain's `RemoteRobot` is bound to one `robot_url` for its
  entire process lifetime (`control/brain_server.py`'s
  `default_robot_factory()`), and each process allows exactly one running
  mission (`state["runner"]`, 409 on a second `/mission/start`). Forwarding
  a mission call from teleop-brain to the main brain would tick the
  **sim** robot's synthetic camera, not the phone's pushed frames -- not
  merely redundant, actively wrong -- and would collide with a
  concurrently running Sim-tab mission on that same main-brain process,
  defeating the entire reason Phase T4 built teleop-brain as a *separate*
  deployment (sim and teleop usable at once).
- **`/recording/frame` has none of that coupling.** Reading
  `record_frame()` (`control/brain_server.py`) confirms it end to end:
  decode base64, validate the walk name/size, write to
  `config["recording_dir"]`. No reference to `robot`, `runner`, or any
  mission state anywhere in it. It is pure storage, so forwarding it to a
  brain that actually has storage is safe in a way mission control
  structurally is not.

So the fix makes recording work no matter which brain URL a client
happens to have configured, instead of asking the client to always
remember the right one.

**Built.**

- `control/brain_config.py` -- three new keys: `recording_proxy_url`
  (default `""`), `recording_proxy_secret` (default `""`),
  `recording_proxy_timeout_s` (default `10.0`). Env overrides
  `RECORDING_PROXY_URL` / `RECORDING_PROXY_SECRET`, same pattern as
  `ROBOT_URL`/`VISION_URL`/`ALLOW_RECORDING` already use.
- `control/brain_server.py`'s `record_frame()` -- when
  `config["allow_recording"]` is false, it now checks
  `config["recording_proxy_url"]` before giving up: if set, the request is
  forwarded via a new `_proxy_recording_frame()` helper; if empty, the
  original 403 ("Recording is disabled on this brain.") still fires --
  fully backward compatible, this is additive.
- The forward itself is a plain `httpx.Client` (not `AsyncClient`) run
  inside `asyncio.to_thread`, matching `RemoteRobot`/`vision_fn_for`'s
  existing sync-httpx style elsewhere in `control/`/`brain/` -- explicitly
  off the event loop because this fires once per recorded frame (up to
  ~2/s during an active walk), unlike the one-off health checks
  `web-twin/index.html` already does before starting a mission.
- The peer's response is relayed **verbatim** on success, and its error
  **status code and body** are relayed as-is on failure (its own 403, a
  400 on a bad walk name, ...) rather than inventing a new error --
  callers see exactly what the brain that actually owns storage said. An
  unreachable peer (`httpx.HTTPError`, e.g. connection refused) becomes a
  clean `502`, never an unhandled exception.
- Tests: `tests/test_brain_server.py`'s three new
  `test_recording_proxy*` cases fake `control.brain_server.httpx.Client`
  (a stub `.post()`) rather than spinning up a second live server --
  `record_frame()`'s local-storage behavior already has its own coverage
  above this section, so the proxy tests only need to prove the right
  request goes out and the peer's response comes back honestly. All
  three pass, plus the existing 147.

**Deployed.** `cloudformation/teleop-brain.yaml` gained a
`MainBrainSecretArn` parameter (`brain.yaml`'s own
`vision-picar-brain-shared-secret`, same "exact full ARN via parameter,
not `Fn::ImportValue`" pattern the other cross-service secrets already
use here -- get it with `aws secretsmanager describe-secret --secret-id
vision-picar-brain-shared-secret --query ARN`), read access to it added
to `EcsExecutionRole`'s `ReadSharedSecrets` policy, and two new container
settings:

```yaml
- Name: RECORDING_PROXY_URL
  Value: !Sub
    - "http://${AlbDnsName}"      # same bare internal-ALB host VISION_URL
    - AlbDnsName:                  # already uses -- distinguished from
        Fn::ImportValue: ...       # vision-analyze's routes by PATH
                                    # (/recording/frame) at the ALB's own
                                    # ListenerRules, not by host.
# ...
- Name: RECORDING_PROXY_SECRET
  ValueFrom: !Ref MainBrainSecretArn
```

`ALLOW_RECORDING` on teleop-brain stays `"false"` -- it now means "no
*local* storage", not "reject the request." The main brain (`brain.yaml`)
needed no CloudFormation change at all: its own `allow_recording`
defaults to `true`, so `record_frame()`'s proxy branch is never entered
there -- it was only redeployed with the same image for code-consistency
(one source of truth for `control/brain_server.py`), not a behavior
change.

Verified live: both `vision-picar-brain-service` and
`vision-picar-teleop-brain-service` ECS services redeployed from the
same rebuilt image (`service/brain/Dockerfile`, unchanged -- it already
copies exactly `control/` and `brain/`), and the
`vision-picar-teleop-brain` CloudFormation stack updated with the new
parameter/secret/env vars.

---

## What this doesn't change

- The rule-based frontier policy, `MockRobot`, and `ReplayRobot` are
  untouched -- `TeleopRobot` is a fourth `RobotInterface` backend, not a
  replacement.
- Room-level step memory (S2b's known open gap -- a photograph carries no
  room label) is **not** solved here. A teleop mission can still
  re-search a room it already covered; only the step cap stops it, same
  as every other vision-policy mission today.
- Phase 11 (real hardware) is unaffected -- `mode: teleop` is a sibling of
  `mode: sim`, not a stand-in for `mode: hardware`.

## Definition of done

A mission completes by physically walking a real room, started and
watched entirely through the twin's Robot view, driven by
`control/brain_server.py`'s `policy: "vision"` -- same mission-status
contract the Sim tab's Remote brain panel already renders. **Done,
2026-08-28**: a real walk ended `OUT: FOUND`, `VF: 0`, the twin's HUD
showing `REACHED` and pausing on arrival exactly as designed.

**Also done, 2026-08-28, deliberately rather than by accident:** both
live failsafes, triggered directly against the deployed services (no
drill, no phone needed for this part -- just the same API a phone
would call). B3.2: pushed an undecodable frame, started a `policy:
"vision"` mission, and watched `vision_failures` climb 1/3 -> 2/3 -> 3/3
in the log before the mission ended `failed`. T1's stall: started a
mission with no fresh frame pushed, which failed almost immediately with
`TeleopStall` surfacing through `RemoteRobot` as `"GET /frame -> HTTP
500"` -- functionally correct (the mission does stop), but a real
**rough edge, left as a follow-up**: `robot/server.py`'s `/frame` route
doesn't catch `TeleopStall` and turn it into a specific response, so the
mission log gets a generic "Internal Server Error" instead of the actual
"no frame has ever been pushed" message. Small fix, not yet done: catch
`TeleopStall` in that route and return e.g. 503 with its real message.
**Done since** (checked 2026-09-28): `/frame` catches any backend
exception -- deliberately not `TeleopStall` by name, so the route imports
no backend -- and returns 503 with the real message.
