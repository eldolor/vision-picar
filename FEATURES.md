# Features -- what's in the app and how it works

A feature-by-feature reference for `web-twin/index.html`, the mobile-first
web app that is this project's one real UI. Written against the repo as
of `01054a0`, which lands the teleop work described in
`PLAN-teleop-robot.md`. Where `README.md` explains *why* something was
built and `CLAUDE.md` tracks *status*, this file explains *how each
button actually works* -- which process it talks to, which route, and
what comes back.

> **Sections 2 and 3 rewritten 2026-09-25** for the ROS alignment
> (`PLAN-ros-alignment.md`): the Camera tab is gone, and the Sim tab lost its
> hardcoded map, its cell-shaped readouts, the JS local brain and the JS
> Vision Autopilot, and gained R0-R2's turn step, sized turns, spin and
> blocked readouts, and a self-reconnecting brain link. The rest of the
> document predates that and is unchanged.

The app has three tabs: **Guide**, **Sim**, **Settings**. This
doc covers all three, then the AWS deployment they run against, in the
most detail for Guide's two modes since that's where a phone actually
exercises the brain/control loop against real pixels.

---

## 0. The moving parts, once, so the rest of this doc doesn't repeat itself

**Three different surfaces in this app answer "does this work" for three
different layers of the system, not one "does the robot work" question
asked three times.** It's tempting to read the digital twin as existing
purely to emulate the robot, which makes Sim, Guide me, and Robot view
look redundant with each other -- they aren't, and none of them is
actually emulating the robot in the literal sense (that's still blocked
on buying hardware, Phase 11):

- **Sim tab** proves the *control logic* -- mission memory, the safety
  veto, the failure budgets, `MissionRunner`'s failsafes -- against a
  free, deterministic grid world, because this project is
  simulation-first by design: nothing here gets validated against real
  hardware first. It's also the only place `control/drills.py`'s fault
  injection is safe to run at all -- nobody deliberately kills the vision
  link or hangs the brain loop on a real robot to prove the failsafe
  fires.
- **Guide me has nothing to do with the robot.** It's a phone-as-sensor,
  human-as-actuator feature that exists because the sim structurally
  can't answer one question: can the vision *model* read a real room?
  `MockRobot`'s camera renders a flat-shaded raycaster, not real pixels
  (`PLAN-sim-hardening.md`'s point that every "the simulation works"
  result so far is a statement about synthetic geometry, not rooms).
  Guide me answers that with real photographs, and is a genuinely useful
  standalone feature in its own right -- point your phone, find your
  keys.
- **Robot view** is the same real-pixel test as Guide me, but asks "what
  would the robot do" instead of "where should the person go" -- the
  cheapest go/no-go gate for the navigation *policy* before spending
  money on hardware.

So: Sim validates the software that will eventually run the robot; Guide
me and Robot view validate the vision model that software depends on, on
real pixels the sim can never produce. All three matter, and they're
testing different things, not the same thing three ways.

Four backend processes this UI can talk to, all HTTP, all optionally
behind `x-app-secret`:

| Process | Port (local) | What it is | Code |
|---|---|---|---|
| **robot server** | `:8000` | Safety layer + watchdog + one `RobotInterface` backend (sim / teleop / eventually hardware) | `robot/server.py` |
| **brain server** | `:8001` | The autonomy loop (`MissionRunner`) as a service -- start/stop/observe a mission over HTTP | `control/brain_server.py` |
| **vision-analyze service** | (cloud only) | Calls Amazon Bedrock for every vision question -- scene description, navigation, AR guidance | `service/vision_analyze/app.py` |
| **admin service** | (cloud only) | Lists/views/deletes recorded walks on the shared EFS volume | `service/admin/`, not reachable from the twin UI |

The twin (`web-twin/index.html`) is a browser client of the first three.
It never talks to a simulator or a robot backend directly -- everything
it does goes over HTTP to `robot/server.py` and/or `control/brain_server.py`,
the same way a Pi's own client eventually will. `robot/server.py` is also
what *serves* `index.html` in the deployed app (`GET /`), so "load the
app" and "have a robot connection" can be the same request.

Two independent "brains" exist and only one may drive at a time:

- **Rule-based / vision-in-JS** -- logic re-implemented in the page's own
  JavaScript, no `control/` service needed. This is the Sim tab's Local
  brain panel.
- **Real `MissionRunner`** -- `control/brain_server.py` running the actual
  Python mission loop (`brain/agent.py` policies, or `brain/vision_agent.py`
  for the vision policy). This is the Sim tab's Remote brain panel, and
  (new) Robot view's "Drive via brain" switch.

---

## 1. Guide tab

Two modes, picked with the **Guide me / Robot view** toggle
(`btn-mode-guide` / `btn-mode-robot`, `state.guidanceMode`). Both modes
share almost everything -- the full-screen camera takeover, the ~500ms
capture-and-call loop, the found-streak-to-pause logic, the call-budget
meter, the countdown ring, the corner brackets and capture scanline --
because both are really "point the phone at a room and repeat one vision
call on a timer" with a different prompt, a different response schema,
and a different renderer. `PLAN-ar-guidance.md` is the from-scratch spec
and redesign history for this tab; this section is what it does today.

**Shared mechanics, both modes:**

- Requires a secure context (`https://` or `localhost`) -- `getUserMedia`
  is blocked otherwise. The first-run start error explains this rather
  than failing silently.
- `<video autoplay playsinline muted>` sourced from
  `getUserMedia({video:{facingMode:"environment"}})`, full-screen
  (`#guide-fullscreen`, CSS `position:fixed`, not the real Fullscreen
  API -- iOS Safari's `requestFullscreen()` support for arbitrary
  elements is unreliable).
- Every capture downscales to a 960px long edge before upload
  (`GUIDANCE_MAX_CAPTURE_DIM`) -- smaller payload, faster Bedrock
  inference, no visible quality loss for a coarse position/action
  question.
- Loop cadence: `GUIDANCE_THROTTLE_MS` / `ROBOT_THROTTLE_MS` = **500ms**
  (both routes moved to Amazon Nova Lite on 2026-08-28; before that Robot
  view paced itself slower to match Sonnet's round trip). On a
  consecutive error the delay backs off exponentially, capped at 30s, and
  resets to the base rate the moment a call succeeds. The backoff
  *replaces* the tick already scheduled at the base rate rather than
  adding a second one -- otherwise a failing endpoint keeps being hit
  every 500ms while the backoff sits unused.
- **Calls overlap** -- up to `GUIDANCE_MAX_IN_FLIGHT` = **2** at once.
  The throttle above is the gap between *dispatches*; the round trip used
  to be added on top, so the real cadence was throttle + latency (~3s once
  `/navigate` moved to Opus 4.5, not the 500ms the constant suggests). The
  next tick is scheduled at dispatch instead of when the call returns,
  which is the whole mechanism -- it decouples the two, so a decision
  arrives every ~500ms, each still describing a frame from one round trip
  ago. **This raises throughput, not freshness**: no answer is newer than
  it was before, there are simply more of them, which is what a person
  walking can actually use (they integrate across several). A robot
  executing each one would be worse off, which is why "Drive via brain"
  pins itself to 1 -- see section 1.2.c. Two rather than more is measured,
  not guessed: `control/walk_replay.py` records that at 8 concurrent
  workers "Bedrock throttled 10 of 22 frames", and Bedrock quota is
  account-and-region scoped, so this contends with everything else running
  in the account -- including production.
- **An answer that has been overtaken is discarded, not drawn.** Latency
  varies per call, so with two in flight seq 7 can land after seq 9. Every
  dispatch takes a sequence number and only renders if it is newer than
  the last one drawn (`guidanceLastRenderedSeq`); a stale answer is still
  *saved* when the walk is being recorded (section 1.2.b), because the
  frame and the reply it got belong together regardless of arrival order.
- **A call cannot outlive the session that made it.** The sequence number
  above only orders calls *within* a run, and both it and `guidanceRunning`
  reset on Stop -- so a call still in flight when you stopped used to clear
  both checks in the *next* session, flash its answer over the new camera
  view, and then set `guidanceLastRenderedSeq` to its own higher number,
  silently suppressing the new run's first several real decisions. Every
  dispatch now also carries the run it belongs to (`guidanceEpoch`, bumped
  on Stop) and is dropped on return if that run is over -- before it can
  touch the overlay, the error streak, or the frame recorder.
- **The call budget is reserved at dispatch, not counted on success.** A
  call that has been sent has been paid for whether or not its answer is
  fresh enough to render, and counting on completion would let the loop
  dispatch past the cap while calls were still outstanding.
- A capture is skipped (no call made) if the phone's orientation sensor
  reports it's been essentially still since the last tick
  (`GUIDANCE_STILL_SKIP_DEG_PER_SEC`), up to `GUIDANCE_MAX_CONSECUTIVE_SKIPS`
  in a row -- saves a paid call when nothing in frame could have changed.
- A hard call cap per session (`GUIDANCE_MAX_CALLS` / `ROBOT_MAX_CALLS` =
  **120**) auto-pauses with a "Resume searching" button, so a
  forgotten-running tab can't burn money unbounded. At the 500ms dispatch
  rate that is about a minute -- and since calls began overlapping it
  really is about a minute, where the old serial loop took several. The
  number of paid calls is unchanged; the budget is just spent faster.
- Backgrounding the browser tab (`visibilitychange`) pauses the timer;
  foregrounding resumes it. The camera stream itself is only released on
  Stop or navigating away (`track.stop()` on every track).
- A wake lock (`requestGuidanceWakeLock`) keeps the screen on while
  active, released on Stop.
- Rotating the phone re-syncs overlay positions immediately rather than
  waiting for the next tick (`resize`/`orientationchange` listeners).

### 1.1 Guide me -- steer a person to an object

**Question answered:** "where is `{target_object}`, relative to where
I'm standing, and how do I get closer?" The *person* is the actuator;
nothing in the app moves anything.

**Backend:** `POST /guidance` on the vision-analyze service
(`service/vision_analyze/app.py:231` -> `describe_image_bytes_guidance()`
in `vision_core.py`, model `amazon.nova-lite-v1:0` by default). Never
touches `robot/server.py` at all -- this mode has no robot connection
requirement and works with only the "Cloud endpoint settings" filled in.

**Request:** `{image_base64, media_type, target_object}` -- the same
shape `/navigate` and `/analyze` take (`_decode_image()` is shared
server-side), which is what lets "Robot view" be a mode toggle here
rather than a second implementation.

**Response schema** (`GUIDANCE_PROMPT_TEMPLATE`):
```json
{
  "target_visible": true | false,
  "position": "far_left" | "left" | "center" | "right" | "far_right" | "not_visible",
  "proximity": "near" | "medium" | "far" | "unknown",
  "guidance": "short human-readable instruction",
  "bounding_box": {x_min, y_min, x_max, y_max} | null
}
```
`bounding_box` is normalized 0.0-1.0 against the *full captured frame*,
requested only when the model is confident (`_validate_bounding_box()`
rejects malformed ones server-side, falling back to `null`).

**What's on screen:**
- **Not found / searching:** a directional chevron pans the reticle
  toward `position`'s zone; a screen-edge glow brightens on the side to
  turn toward (intensity scaled by how far off-center); chevron pulse
  speed scales with `proximity`. While `target_visible` is `false`
  entirely, a fixed **search-sweep** convention takes over instead (right
  for ~5 ticks, then down for ~5 ticks, repeating) -- the model has zero
  positional information in this state, so this is a UI convention to
  encourage full-room coverage, not a model-informed suggestion.
- **Found:** a bounding-box outline (DOM/SVG, not canvas, so it glides
  via CSS `transition` rather than snapping), smoothed 55%-toward-the-new-
  estimate each tick (`smoothGuidanceBox()`) to damp per-tick LLM
  localization jitter, mapped from normalized coordinates to on-screen
  pixels accounting for `object-fit:cover`'s crop (`mapNormalizedBoxToScreen()`).
  If the model doesn't return a confident box on a "found" tick, a
  generic centered pulse shows instead of a guessed rectangle.
- **Found gate** (`isGuidanceFound()`): `proximity === "near"` (any
  position), OR `proximity === "medium" && position === "center"` (added
  after real-device testing showed a dead-centered object sometimes
  scored "medium" and never advanced).
- Bottom caption pill: small icon + the model's own `guidance` text --
  de-emphasized on purpose; the visual cues (chevron/glow/pulse/outline)
  are the primary channel, confirmed with the user during the redesign.
- **Feedback:** directional Web Audio tones (panned left/right,
  works on iOS Safari, muteable) and `navigator.vibrate()` patterns
  (Android Chrome only -- Apple has never implemented the Vibration API
  in Safari; this is a platform limit, not a bug, and audio is the real
  cross-platform channel).
- **Auto-pause:** once found holds for `GUIDANCE_FOUND_STREAK_TO_PAUSE`
  (1) consecutive tick, polling stops entirely -- no reason to keep
  paying for calls once the answer stopped changing. "Resume searching"
  restarts the loop without restarting the session.
- First tap of Start (per page load) shows a dismissible onboarding card
  explaining chevron/glow/pulse/outline *before* the camera permission
  prompt fires.

**Cost:** a real Bedrock vision call every ~1.7s effective cycle (500ms
throttle + ~1.0-1.6s Nova Lite latency). Never auto-starts.

**Never touches** `RobotInterface`, `robot/server.py`, or `control/`.
Same phone-as-sensor / human-as-actuator design as the rest of the twin's
non-robot features -- see `PLAN-ar-guidance.md` section 1.

### 1.2 Robot view -- what would the robot do, from here

**Question answered:** "if this frame came from the robot's camera right
now, what move would it make?" **Nothing executes.** This is a
read-only preview -- proof the vision *policy* works on real room pixels
rather than the grid-world sim's flat-shaded raycaster render, which
`PLAN-sim-hardening.md` 3.5 flags as the thing the sim can't tell you.

Robot view actually has **three distinct behaviors**, stacked as switches
next to the target-object field, only visible in this mode:

```
Guide tab -> Robot view
  |
  +-- (default) one-off /navigate preview -- nothing persists
  +-- "Record this walk"  -- saves each frame+reply to the brain, for replay
  +-- "Drive via brain"   -- runs a REAL MissionRunner mission, live
```
The last two are gated on `state.brainConnected` (Settings' brain
connection must be live) and are **mutually exclusive** -- checking one
unchecks the other, enforced both directions in the checkbox handlers.

#### 1.2.a Default: one-off `/navigate` preview

**Backend:** `POST /navigate` on the vision-analyze service
(`describe_image_bytes_navigate()`, also Nova Lite by default). Exactly
the same request/response plumbing as Guide me (`callGuidanceEndpoint()`
picks the route: `state.guidanceMode === "robot" ? "/navigate" : "/guidance"`),
different prompt and schema.

**Response schema** (`NAVIGATE_PROMPT_TEMPLATE`):
```json
{
  "target_visible": true | false,
  "target_direction": "left" | "center" | "right" | "not_visible",
  "target_reached": true | false,
  "obstacle_ahead": true | false,
  "room_guess": "short room-type label, or \"unclear\"",
  "action": "FORWARD" | "LEFT" | "RIGHT" | "REVERSE" | "STOP",
  "distance_estimate": "within_one_step" | "a_few_steps" | "far" | "unknown",
  "path_ahead": "open_floor" | "blocked" | "unclear",
  "reasoning": "one short sentence"
}
```
`room_guess` is what feeds room-level step memory -- the client sends
`searched_rooms` and the server answers with a room label, which
`brain/agent.py:MissionAgent.step()` backfills into `frame["room"]`
(`AGENT-HARNESS.md` section 10).

**`distance_estimate` is only asked for under the `default-with-distance`
prompt variant, and is `"unknown"` everywhere else** -- including on a
parse failure, since a caller may veto a move on it and an unrecognised
value has to fail towards "do not act on this". It is an *ordinal*
judgement (how many robot moves of clearance), not a distance: a single
monocular frame cannot give metric depth, so asking for centimetres would
invite a confident guess with no error bar. **It is not calibrated and
should not drive anything** -- measured over 80 frames from five recorded
walks it reports `within_one_step` on 60% and `far` on 5%, which is not
what walking across a house looks like. See `CLAUDE.md`'s Stage 0 notes
for the full measurement and section 5 for the veto that stays off
because of it.

**`path_ahead` is the fourth attempt at the same question, asked about a
different region, and is only populated under the `center-third-path`
variant** (`"unclear"` everywhere else, parse failures included, on the
same fail-towards-inaction contract). The three wordings before it and
`distance_estimate` all failed the same way: "is there an obstacle ahead"
is a question about the whole frame, and indoors the honest answer is
almost always yes. This one asks what is in the bottom half of the centre
third -- the patch of ground the next step actually crosses -- and asks it
*descriptively* (`open_floor` / `blocked` / `unclear`, "what is there")
rather than *evaluatively* ("does that count as an obstacle"). Under this
variant `obstacle_ahead` is a restatement of `path_ahead == "blocked"`,
which is what lets the existing scorer measure the new question unchanged;
the parser deliberately does **not** derive one from the other, because
manufacturing that agreement would hide the disagreement worth seeing.
**Measured 2026-09-02 and it does not work** -- the collision rate is
unchanged and the two models answer it 40% alike against a 35% chance rate,
disagreeing about the threshold rather than about the picture. It stays
served so the negative result can be re-run. Note the corpus it was measured
on was found to be invalid in the same pass (targets on raised furniture,
camera at standing height) -- see `CLAUDE.md`'s Stage 0 notes, which is also
the reason this field is not wired to anything.
The frame is split into three vertical thirds (left/center/right) as the
frame of reference for both `target_direction` and the model's own
reasoning about what's in front of it. **`action` only ever means
"movement"; arrival is `target_reached`, a separate field** -- deliberate,
so a `STOP` caused by an obstacle can never be confused with a `STOP`
caused by success (the empty/fallback schema also defaults `action` to
`STOP`, which is exactly why this split matters: a parse failure must
never look like arrival).

**What's on screen:**
- A persistent three-zone HUD frame (left/center/right brackets) stays up
  for the whole session -- it's the mode's visual identity, not a
  per-tick result, so it's the one thing `hideAllGuidanceOverlays()`
  deliberately does *not* clear each tick.
- The zone matching `target_direction` highlights when `target_visible`
  is true; nothing highlights on `not_visible` or an unrecognized value
  (a lit zone must always mean the model actually reported that zone).
- A chevron shows the `action` itself (up/left/right/down), colored
  green for `FORWARD`, neutral otherwise; suppressed entirely on `STOP`
  or arrival, since a directional arrow for "no direction" would be
  actively misleading.
- **Arrival is its own badge state** (`REACHED`), never the raw `STOP`
  action -- rendered only from `target_reached`, exactly to avoid the
  "was that success or a wall?" confusion the schema was designed to
  prevent.
- A telemetry readout (`TGT`/`ACT`/`VIS`/`OBS`/`SEQ`, plus `STA: REACHED`
  on arrival) and an `OBSTACLE AHEAD` banner when `obstacle_ahead` is
  true.
- **No audio/haptic cues in this mode** -- they encode `proximity`, a
  field `/navigate` doesn't return.
- **No auto-pause on a single arrival tick** -- Robot view pauses only
  after `ROBOT_REACHED_STREAK_TO_PAUSE` (2) consecutive `target_reached`
  ticks, both to avoid pausing on one noisy read and because (unlike
  Guide) the run is meant to be a continuous decision readout, not a
  one-shot "you found it."
- Hold the phone low, ~10cm -- roughly the car's camera height. A
  chest-height view isn't the robot's view, and the whole point of this
  mode is testing the vision policy on the actual geometry it will see.

This sub-mode never writes anything and never touches `robot/server.py`
or `control/` -- it's the cheapest, fastest go/no-go signal for "does the
vision model navigate real rooms," which is Stage 0 of the build plan
(`CLAUDE.md` section 5).

#### 1.2.b "Record this walk"

Saves every captured frame plus its `/navigate` reply to the **brain
service** (not the vision service, and not a browser download -- 60
download prompts on a phone isn't a workflow). Requires the brain
connected in Settings; `brain.allow_recording` gates it server-side too.

**Flow:**
1. Start -> `beginWalkRecording()` generates a walk name from the target
   + timestamp (`newWalkName()`).
2. Each tick's frame+reply -> `POST {brainUrl}/recording/frame`
   (`control/brain_server.py:300`), fire-and-forget (`recordWalkFrame()`)
   -- a failed save must never interrupt the walk or delay the next
   `/navigate` call. A running saved/failed counter renders under the
   switch. **The frame's `seq` is allocated when the vision call is
   dispatched, not when the save resolves.** That matters because calls
   overlap: numbering from the saved/failed counters let two frames read
   the same value before either save returned, and
   `control/brain_server.py` writes `frame-{seq:04d}.jpg` with
   `write_bytes()` -- a silent overwrite, while `walk.jsonl` still gained
   a row for each. Measured against the defect, every other frame was
   lost. Dispatch order is also capture order, which is the order
   `sim/replay_robot.py` replays a walk in.
3. `control/brain_server.py` writes frames to the shared EFS volume
   (`cloudformation/recordings.yaml`) -- the same storage the separate
   `/admin` service (`service/admin/`) lists/views/deletes from later.
4. Stop -> the walk name and save count show in a toast, with the exact
   replay command:
   `python -m tests.demo_replay_mission recordings/<walk> "<target>"`.

**Why it exists:** a phone walk through a real house is the only real-
pixel material this project has produced. Recorded, it becomes
`sim/replay_robot.py`'s input -- an `open-loop` body made of photographs
that lets `brain/vision_agent.py`'s `VisionAgent` be run, repeatedly and
for free (no re-walking the house), against the exact same frames while
a prompt is iterated on. It captures the same `/navigate` reply this
sub-mode already produces, alongside each frame, so a recorded walk
carries "what the model said at each step" too.

**Not the same thing as "Drive via brain" below** -- recording still
does a one-off `/navigate` call per frame and just also saves it;
driving via brain replaces that call entirely with a live mission tick.
That's also why the two toggles are mutually exclusive.

#### 1.2.c "Drive via brain" -- closes the loop for real (PLAN-teleop-robot.md, Phase T3)

This is the one that actually exercises `control/brain_server.py`'s real
`MissionRunner` -- mission memory, the failure budget, the timeouts, the
whole B0-B3 stack -- from a live phone camera instead of a recorded walk
or a drill. Before this existed, nothing in the codebase read a live
camera through `MissionRunner`; a phone walk proved the vision *model*
could read a room but proved nothing about the *brain*. After it, a
phone is one working instance of "live camera" and a PiCar will be
another -- the same config-change promise `PLAN-brain-relocation.md`
already made for the robot side now holds for the brain side too.

**Topology (before vs. after):**

```
Before -- two paths that never touch:

Guide/Robot view (phone camera)      Sim tab, Remote brain (no camera)
    |                                    |
    v                                    v
  POST /navigate (vision-analyze)      POST /mission/start
    |                                    |
    v                                    v
  HUD renders the reply.               control/brain_server.py -> MissionRunner
  Nothing moves, no mission exists.      |
                                          v
                                        RemoteRobot -> robot/server.py -> MockRobot

After -- Robot view is a third SOURCE for the one real mission loop:

Guide tab, Robot view, "Drive via brain"
    |
    | POST /teleop/frame  (new)
    v
robot/server.py  (mode: teleop)  --  sim/teleop_robot.py's TeleopRobot
    ^                                        |
    | GET /frame (existing, unmodified)      | get_camera_frame() returns
    |                                        | the last pushed frame
control/brain_server.py -> MissionRunner -> VisionAgent -> vision_fn_for()
    |                                                            |
    v                                                            v
 GET /mission/status  -->  Robot view HUD                service/vision_analyze
 (polled at the same 500ms cadence)                       (Bedrock, unchanged)
```

**The person holding the phone is the motor.** `TeleopRobot`'s
`drive_forward`/`turn_left`/etc. don't move anything -- they log what the
model decided, the same honest no-op `ReplayRobot`'s pan methods already
are. The next frame is whatever you actually photograph after reading
the instruction and walking, which is what makes this **closed-loop**
where a recorded-walk replay is open-loop (a `LEFT` at frame 12 can't
change what frame 13 shows in a replay; here, it can).

**On Start**, when the switch is on:
1. A pre-flight check: `GET {serverUrl}/health` must report
   `mode: "teleop"` -- if the robot server URL still points at a plain
   `mode: sim` twin, this fails loudly here with a clear message rather
   than 404-ing silently on the first frame push.
2. `POST {brainUrl}/mission/start` with `{policy: "vision", target_object}`.
   `control/brain_server.py`'s runner factory (`brain_server.py:151`)
   resolves `policy: "vision"` into a real `vision_fn` bound to the
   configured `vision_url` (`brain/navigate.py`'s `vision_fn_for()`),
   which is what actually calls `/navigate` on the vision-analyze
   service on the brain's behalf, per tick.
3. The choice is **latched** into `state.guidanceViaBrain` at Start (not
   re-read from the toggle every tick) -- so Stop knows whether *this*
   session actually started a mission even if the toggle or brain
   connection changes mid-walk.

**Each tick** (`driveViaBrainStep()`, replacing `callGuidanceEndpoint()`):
1. `POST {serverUrl}/teleop/frame` with the captured JPEG
   (`robot/server.py:244` -> `TeleopRobot.push_frame()`, secret-gated
   like every other robot route). This bumps an internal sequence number
   and notifies anyone blocked waiting on a fresher frame.
2. Independently, `MissionRunner`'s own tick loop (running inside
   `control/brain_server.py` as an asyncio background task, per B2/B3.3)
   calls `RemoteRobot.get_camera_frame()` -> `GET {serverUrl}/frame` ->
   `TeleopRobot.get_camera_frame()`, which returns whatever frame was
   most recently pushed (a **staleness check**, not a block -- see below),
   feeds it through `vision_fn_for()` to `/navigate`, and applies
   `VisionAgent`'s decision (trust the model's action unless the mission
   is over) the same way it would for a recorded walk or the sim.
3. The page polls `GET {brainUrl}/mission/status` and reshapes it
   (`missionStatusToRobotResult()`) into the same
   `{action, reasoning, target_reached, ...}` shape a raw `/navigate`
   reply has, so every existing renderer
   (`renderRobotOverlay`/`renderRobotStatus`/`renderRobotTelemetry`/
   `announceRobot`) and the found-streak-to-pause logic keep working
   unmodified regardless of which of the three sub-modes produced the
   result.
4. `renderRobotTelemetry` shows two extra rows here that raw `/navigate`
   never had -- `OUT` (the mission's `outcome`) and `VF`
   (`vision_failures`, so a live B3.2 failure budget counting up is
   visible) -- and marks `VIS`/`OBS` as `N/A` rather than a false
   `CLEAR`/`NO`, since a mission status is a *decision*, not the
   per-frame zone/obstacle reading that produced it.
5. A mission ending any way other than found/room_reached -- stopped,
   failed, max_steps, a live stall, a live vision-failure budget hit --
   surfaces through the exact same error path a lost `/navigate`
   connection would, and stops the tick loop the same way arrival does.

**Stop** -> `POST {brainUrl}/mission/stop` (fire-and-forget, since
`stopGuidance()` isn't async), which stops both the mission loop and the
robot -- `MissionRunner`'s stop path is enforced at the robot gate, not
just in the loop, so a tick already in flight can't get a move out after
stop.

**What actually makes this a real failsafe demo, not just a preview:**
- **Standing still (or pointing the phone at the ceiling)** and not
  submitting a new frame: within `stall_timeout_s` (default 15s) the
  mission ends `failed` with a stall message in the log -- a genuine
  failsafe firing, no drill involved.
- **Pointing the phone somewhere the model can't parse a scene** exercises
  B3.2's vision-failure budget (default 3 chances) the same way a drill
  would, but for real.
- **Backgrounding the tab**: the mission is a service call, not page
  state (B4's proof), so it's still running when you come back.

**`TeleopRobot`, precisely** (`sim/teleop_robot.py`, a fourth
`RobotInterface` backend alongside `sim`/`hardware`/`replay`):
- `push_frame(image_base64, media_type)` -- not part of `RobotInterface`;
  called only by `robot/server.py`'s `/teleop/frame` route. Stores the
  frame, bumps a sequence number, records *when* it arrived.
- `get_camera_frame()` -- **a staleness check, not a wait.** Originally
  built as a blocking wait for a frame strictly newer than the last one
  *that specific caller* had seen -- a real bug shipped that way: the
  "last returned" cursor was shared across every caller of `GET /frame`,
  so Settings' own passive reachability check could "consume" the
  current frame and leave the *next* caller (an active mission's tick)
  blocking for the full 15s and then failing, even with a healthy phone
  actively streaming. Fixed the same day, against the live deployment:
  it now tracks *when a frame was last pushed*, returns it immediately if
  within `stall_timeout_s`, or raises `TeleopStall` immediately otherwise
  -- never blocks, matching every other backend's cheap/idempotent read.
- `drive_forward`/`reverse`/`turn_left`/`turn_right`/`look_*` -- log-only
  acks, no position tracked.
- `get_distance()` -- always `999.0` (`NO_SENSOR_CM`), the same "honestly
  no distance sensor here" constant `ReplayRobot` uses, so `robot/safety.py`
  never blocks a move against this backend.
- `TeleopStall` (a `RuntimeError`) needs no special handling in
  `control/mission_runner.py` -- it surfaces from `agent.step()` straight
  into `MissionRunner.tick()`'s existing catch-all, which already stops
  the robot and ends the mission `failed` with the exception's own
  message in the log. One known rough edge, not yet fixed: `robot/server.py`'s
  `/frame` route doesn't catch `TeleopStall` specially, so a live stall
  currently surfaces to the brain as a generic "HTTP 500" rather than a
  named 503 -- functionally correct (the mission does stop), just a
  less legible log line than it could be.

**Config:** `config/robot.yaml`'s `mode: teleop` (a sibling of `sim`, not
a stand-in for `hardware`) plus a `teleop:` block (`stall_timeout_s`,
default 15.0 -- documented as needing to stay under `brain.tick_timeout_s`,
though the staleness-check fix above made that no longer load-bearing,
since a stale read now fails instantly instead of occupying wall-clock
time inside the call).

**What this doesn't solve:** room-level step memory (a photograph still
carries no room label -- a teleop mission can revisit an already-searched
room; only the step cap stops it, same limitation as every vision-policy
mission today). Motion realism is not a concern here since nothing
physically moves under this mode anyway.

---

## 2. Camera tab -- removed 2026-09-25

Kept as a numbered section so the rest of this document's numbers stay put.
It was one panel -- take a photo, get a description -- calling `/describe`
with `/analyze` as a fallback, and the user asked for it to go: Guide's
"Guide me" does the same job live. **Both routes are still deployed and
tested** in `service/vision_analyze/`; nothing in the twin calls them.

---

## 3. Sim tab -- the simulated robot, and the brain that drives it

Everything here talks to `robot/server.py` or `control/brain_server.py` over
HTTP. **The page knows nothing about the house**: no copy of the layout, no
room boundaries, no compass tables, no renderer -- every panel reads a route
the robot serves, which is what lets the same tab show the grid-world sim
today and a real robot with a SLAM map later. (Until 2026-09-25 it carried a
hardcoded copy of the starter house for a top-down view, which drew a
finished map at mission start; N1's discovered map replaced it.)

### Manual control

- **D-pad** -- `POST /action` / `POST /stop`, each through `robot/safety.py`'s
  collar on the server, so a person driving over Wi-Fi gets the same
  collision protection an AI decision does. Every command names its driver
  (`x-driver: twin-dpad`), and M4's authority order means a tap preempts a
  running mission.
- **Turn step (15° / 45° / 90°)** -- how far LEFT and RIGHT turn, sent as
  `/action`'s `angle`. Since R0 the pose is continuous, so this is how a phone
  reaches a heading that is not a compass point. Remembered across reloads;
  defaults to 90, what every tap sent before.
- **Action log and Safety line** -- each move, its angle for turns
  (`RIGHT 45°`), and why a refused one was refused (SAFETY VETO, PREEMPTED,
  WATCHDOG).

### Remote brain

`POST {brainUrl}/mission/start`, then this panel only *observes*: it polls
`GET /mission/status`, and the mission survives the tab closing (B4).

- **Start** is the panel's primary (blue) button and becomes a red **Stop**
  while a mission runs. It is greyed only when the brain is unreachable, and
  the page then says so and **retries every 5 seconds on its own** -- a
  restarted brain used to leave Start dead until Connect was pressed in
  Settings. Only a brain that does not answer (or a 502/503/504 from the
  tunnel's proxy) counts as lost; one that answers with an error stays
  connected so the error is shown.
- **Policy picker** -- the free rule-based explorer (`brain/agent.py`, which
  since the ROS alignment explores by the WORLD's pose in metres and builds
  its scene from the depth grid -- no grid cells), the **vision policy** (one
  paid `/navigate` call per step), or the **tiered policy** (below). The two
  paid ones state the model and wording they will ask with before anything
  is spent, and validate both at start.
- **Drill picker** -- `control/drills.py`, off unless `brain.allow_drills`:
  vision errors, vision hangs, or a hung brain loop, each ending with the
  robot stopped.
- **Telemetry** -- outcome, step, last action, vision failures, rooms, the
  model's (or tier's) reason, the robot watchdog, who is driving, the last
  refusal. Three R-phase additions:
  - **Last action names a sized turn** -- `LEFT 23°` is a correction onto a
    measured bearing (R1); a bare `LEFT` is a turn nothing sized.
  - **Turns** -- `98 of 120 steps, 0 reversed the one before`, and **SPINNING
    IN PLACE** in red when most steps were turns in one direction. Reversals
    alone read a spin as success; the share is the other half (the runner
    owns the rule).
  - **Outcome `blocked`** -- the mission stopped itself after five FORWARDs in
    a row were refused (`brain.stuck_after`), rather than pushing into a wall
    until the step budget ran out. Going around is route planning -- nav2, R6.

**The tiered policy** (`brain/tiered.py` over `brain/perceive.py`) puts a local
perception tier in front of the vision policy; the paid `/navigate` call goes
out only on `mission_start`, `candidate_sighting` or `cold_search`. Against
the **simulator no model runs**: the brain sees the frame is a render and
reads `frame["detections"]` -- the simulator's own report of what its
geometry shows (1.12: no detector on a rendered wall) -- through
`FrameReportedPipeline`, and the hint and the Models line say **sim ground
truth**. Against a real camera (Guide -> Robot view -> Drive via brain) the
YOLOE detector and CLIP scorer run in the brain process. Its readouts,
present only under this policy:

- **Perception** -- `detected` / `absent` / `unavailable`, with label and
  bearing; every frame carries `synthesised`, true in the sim.
- **CLIP margin** -- the margin over the best distractor (real cameras only).
- **Models** -- what is actually perceiving: `sim ground truth`, or the
  detector and encoder by name.
- **Deliberation** -- cloud calls *and* frames, the number the architecture
  is judged on; paid steps are marked `[cloud: <trigger>]` in the log.
- **Corroboration** -- §1.11a's verdict, **reported and not enforced**.
- **Cloud pacing / in flight** -- which rule is pacing the cloud, and what
  the robot does while a call is outstanding.

A turn the tier chooses from a measured bearing turns **by** that bearing
(R1); a search turn goes out at 45° so consecutive views overlap under the
camera's 60-66° field (R1b).

### What the robot senses

Every readout here is a route the robot serves:

- **Camera** -- `GET /frame`'s pixels, rendered server-side by
  `sim/renderer.py` (the JS raycaster was deleted 2026-09-25). The
  **frame source** line says `server`, or `none` if a backend sends no
  pixels -- never a picture the page invented.
- **Depth strip** (M2/M3) -- eight zones, the path zones the collar reads
  outlined, and the clearance it compares.
- **Odometry** -- path length and heading, continuous since R0.
- **World map** (N1) -- `GET /world/map`, discovered as the robot drives:
  free, wall and never-seen in three tones, with the robot drawn at
  `GET /world/pose`.

R2's `/wheels`, `/scan` and `/world/truth` are served but not drawn yet;
they exist for the ROS nodes (R4) and SLAM's error readout (R5).

**One brain drives at a time**: the brain server answers a second
`/mission/start` with 409, and M4's authority order ranks the D-pad above
any mission.

---

## 4. Settings tab

- **Robot server connection** -- `serverUrl` + optional secret. Auto-fills
  to the page's own origin when served by `robot/server.py` itself
  (the deployed case); manual entry still works for pointing at a
  different server (e.g. a teleop deployment, per section 1.2.c).
- **Brain service connection** -- `brainUrl` + its own, independently
  entered secret (`brainSecret`/`brainAuthHeaders()` -- deliberately not
  assumed to match the robot's secret, since the two are independently
  deployed ECS services once out of local dev). Optional; needed for
  Remote brain, recording, and Drive via brain.
- **Cloud endpoint settings** -- the vision-analyze base URL + secret,
  shared by Guide's two modes (the Camera tab and the JS Vision Autopilot
  that also used it were removed 2026-09-25). Enter the base URL only; each
  feature appends its own route (`/navigate`, `/guidance`) via
  `deriveServiceUrl()`.
- **Share setup** -- a QR code encoding all of the above (URLs + secrets)
  so a second phone can be configured with no typing. Explicitly flagged
  in the UI: anyone who scans or photographs it gets the same access.
- **Developer** -- a "show developer readouts" switch (exact API call
  counts, the pan-speed meter) for anyone debugging the loop rather than
  just using it.
- **Debug / support** -- copies the last 20 unexpected browser-side
  errors plus basic context (URL, device) to the clipboard, for pasting
  into a bug report without live-reproducing the issue.

All fields except the debug/onboarding flags persist in `localStorage`
(wrapped in try/catch -- private browsing can throw) so a re-opened PWA
doesn't start from a blank state.

**Endpoint mismatch notices.** Each of the three URL fields above shows a
note when the host it points at is not the host that served the page --
"Robot commands and frames go to **X**, not **Y**, which served this
page." It is not an error: a tunnel or a proxy is a legitimate reason for
them to differ, and local dev routinely splits the twin (:8000) from the
service (:8080), so the note says what will be called rather than
claiming anything is broken. `file://` and same-host are silent.

This exists because **that persistence is exactly the hazard**. A saved
URL outlives the page that saved it, so opening a second deployment on a
phone that has used the first leaves the new page calling the old
service. The first version covered only the vision field, because a 401
was the visible symptom -- and a 401 reads as a wrong secret rather than
a call to the wrong place. The brain field then sent a walk recorded on
one deployment to another deployment's volume, and the robot field can do
the same for the D-pad. One saved endpoint following someone between
deployments is the shape; which field it happens to be is incidental,
so all three are covered.

**Environment banner.** When the server that served the page reports a
non-empty `env_label` on `/health`, the twin renders a sticky
orange banner ("LAB ENVIRONMENT"), puts a coloured rule along the top of
the tab bar, and prefixes the browser tab title. The `/admin` console does
the same for itself. Three things worth knowing about how it decides:

- **The page asks the server that served it**, via a relative `/health`,
  rather than pattern-matching its own hostname -- a CloudFront domain can
  change, and the twin is also opened straight off an NLB, off localhost,
  and off a file server in tests. The one thing always true is that
  `robot/server.py` served this HTML.
- **Production sets no label, so nothing renders.** The same image is
  therefore safe in every environment, and a warning that is always on
  never gets the chance to become invisible.
- **It fails silent.** A page that cannot reach its own origin has bigger
  problems than a missing badge, so "no banner" is both the healthy
  production state and the safe failure state.

The label comes from an `ENV_LABEL` environment variable on
`robot/server.py` and `control/admin_server.py`, set by the `EnvLabel`
CloudFormation parameter (section 6). It is unauthenticated on purpose:
`/health` already is, and someone has to know which environment they are
looking at *before* typing a secret, which is precisely when the
confusion happens. The value is a label, not a credential.

---

## 5. Safety and failsafes (cross-cutting, not a tab)

Applies to every mode above that moves anything (D-pad, both local
brains, Remote brain, Drive via brain) -- Guide me and Robot view's
default/recording sub-modes move nothing, so none of this applies to
them.

| Guard | Where | Catches |
|---|---|---|
| **Safety veto** (`robot/safety.py`) | Every `/action` call, on the server | A move that would collide, regardless of who/what requested it |
| **B3.1 watchdog** | `robot/server.py`, async polling loop | Motors left running because the client (any client) went quiet past `watchdog_timeout_s` |
| **B3.2 vision guard** | `control/mission_runner.py`'s `_guarded_vision` | A vision call that times out or fails repeatedly (timeout + `max_vision_failures` budget) |
| **B3.3 dead-man** | `control/brain_server.py`'s `mission_loop` | A tick that starts and never returns -- the watchdog can't see this; a brain that's alive but stuck |
| **T1 stall** (teleop only) | `TeleopRobot.get_camera_frame()` | No fresh frame pushed within `stall_timeout_s` -- surfaces through B3.3's catch-all, no new handling needed |

All five end the same way: the robot is told to stop. B3.2/B3.3 are the
two that can't be triggered by pressing anything on real hardware, which
is why `control/drills.py`'s fault picker exists in the Sim tab -- and
why the live phone walk in section 1.2.c is the closest thing to
provoking them for real without a drill.

### 5.1 The vision proximity veto -- built, off, and staying off

`brain/agent.py` can also stop a `FORWARD` that the *model* says would hit
something, using the `distance_estimate` field from section 1.2.a. It is
**disabled by default** (`vision_proximity_veto`, on both
`ConstrainedAgent` and `MissionRunner`) and three conditions must all hold
before it can fire: it is explicitly enabled; the backend genuinely has no
distance sensor (`get_distance()` returns `robot/interface.py`'s
`NO_SENSOR_CM`, as `ReplayRobot` and `TeleopRobot` do -- a photograph has
no depth in it); and the model actually said `within_one_step`, never
`"unknown"`.

**It is not a safety layer and must not be mistaken for one.**
`robot/safety.py` is, and its docstring is explicit that it never trusts
the AI's own claims about distance -- which is why this lives in `brain/`
instead. If a real reading exists it wins; a model's guess must never
override or pre-empt a measurement.

**What it is for:** on `ReplayRobot` and `TeleopRobot` the safety veto is
dead code, so a whole Robot-view walk says nothing about collision
avoidance. This makes that path execute against real pixels, which nothing
else does before hardware exists.

**Why it ships off:** measured over 80 frames from five recorded walks,
`within_one_step` comes back on 60% of them. Wired on, that would block
roughly three FORWARDs in five and reproduce the never-FORWARD stall the
3x3 prompt matrix already found. **Do not copy this into
`robot/hardware_robot.py`** -- on the car the lidar is the obstacle
sensor, and on identical frames one model reports `obstacle_ahead` ~100%
of the time and another ~0%.

---

## 6. AWS deployment -- how the pieces above actually run in the cloud

Everything in this project shares **one NLB, one internal ALB, and one
CloudFront distribution** -- new features get a new path prefix and a
new `ListenerRule` on the existing shared listener, not a new load
balancer or port (see `CLAUDE.md` section 6 for why: a second pair would
have cost roughly as much as everything else in this project combined).

```
                         Internet
                            |
                  CloudFront distribution        <- cloudformation/cdn.yaml
             (public HTTPS -- satisfies getUserMedia's
              secure-context requirement; origin is
              plain http:// to the NLB, TLS terminates here)
                            |
                    Network Load Balancer          <- cloudformation/service.yaml
                            |
                    Internal ALB, one shared
                    listener on port 80
                            |
      +----------+----------+----------+----------+-----------+
      |          |          |          |          |           |
  vision-      twin       brain     teleop-    teleop-       admin
  analyze    (Priority   (Priority   robot     brain       (Priority
  (default,   10 / 11)     20)     (Priority  (Priority       30)
  no prefix,             /mission/*  40 / 41)    50)        /admin/*
  path-                  ...          /teleop-   /teleop-
  fallthrough)                        robot/*    brain/*
```

| Stack | Service it runs | Path claimed | Priority | Notes |
|---|---|---|---|---|
| `service.yaml` | vision-analyze (`/analyze`, `/navigate`, `/guidance`, `/describe`) | everything the others don't claim -- the ALB's fallthrough | -- | Owns the NLB, ALB, listener, VPC endpoints; every other stack imports these via `Fn::ImportValue` |
| `twin.yaml` | `robot/server.py`, `mode: sim` | `/`, `/action`, `/stop`, `/distance`, `/frame`, `/teleop/frame` (unprefixed) | 10, 11 (PWA assets) | The deployed "main" twin -- what a phone hits by default |
| `brain.yaml` | `control/brain_server.py` | `/mission/*` (unprefixed) | 20 | Sits alongside `twin.yaml`, not inside it -- see `PLAN-brain-relocation.md`'s "interim" reasoning |
| `admin.yaml` | `service/admin/` | `/admin/*` | 30 | Recorded-walk viewer against the shared EFS volume; its own service so reviewing recordings doesn't depend on the brain being up |
| `teleop-robot.yaml` | `robot/server.py`, `mode: teleop` | `/teleop-robot/*` | 40, 41 | A **second, independent instance** of the same image as `twin.yaml`, via `ROUTE_PREFIX` -- lets sim-mode and teleop-mode be live at the same time (`robot/factory.py` picks one backend per process) |
| `teleop-brain.yaml` | `control/brain_server.py`, pointed at the teleop robot | `/teleop-brain/*` | 50 | Same `ROUTE_PREFIX` trick, own secret, own `robot_url` |
| `recordings.yaml` | EFS volume | -- | -- | No routes of its own; mounted by the brain (writes) and admin (reads) services, survives redeploys unlike Fargate's own ephemeral disk |

**`EnvLabel`.** `twin.yaml`, `teleop-robot.yaml` and `admin.yaml` each
take an `EnvLabel` parameter, defaulting to `""`, which becomes the
`ENV_LABEL` environment variable behind the banner described in section
4. An empty value must add **no variable at all** rather than an empty
one: an empty one still rewrites the task definition, which a change set
against the live production stack showed would churn `TaskDefinition` and
`EcsService` for a deployment meant to change nothing. `AWS::NoValue`
under a `HasEnvLabel` condition is what drops the list element entirely.

Production passes nothing and is therefore unaffected. A second
environment passes its own name.

**The consequence, measured 2026-09-02: the three production stacks do
not list `EnvLabel`, and that drift cannot be closed.** They were last
updated before the parameter existed. Deploying the template does
nothing -- `aws cloudformation deploy` answers "No changes to deploy,"
and a change set created by hand comes back `Status: FAILED`,
`ExecutionStatus: UNAVAILABLE`, `Changes: []`, reason "The submitted
information didn't contain changes." The change set *sees* the new
parameter; it just yields no resource delta, because that is exactly what
`AWS::NoValue` above is for. CloudFormation will not execute a
zero-change change set, so the stack's stored template and parameter list
stay where they are.

This is cosmetic and self-correcting -- every deploy passes
`--template-file`, so git is the source of truth and the stale stored
template is visible only through `describe-stacks`. The first deploy
carrying a real change registers the parameter normally, including the
one that matters: `--parameter-overrides EnvLabel=<name>` produces a
genuine delta and works. **Do not try to force it.** The only mechanism
that would is a two-step churn -- deploy with a non-empty label, then
again with it empty -- which rewrites `TaskDefinition` and restarts
`EcsService` twice in production to arrive back where it started, the
precise churn the condition was written to avoid. Note that a failed
attempt leaves a `FAILED` change set attached to the stack;
`list-change-sets` then `delete-change-set` clears it.

**Why CloudFront exists at all**, specifically: `getUserMedia` (Guide
me and both live-camera Robot view sub-modes) requires a secure context.
A bare NLB/ALB only serves plain HTTP; CloudFront's default
`*.cloudfront.net` domain gives free automatic HTTPS in front of it with
no certificate management, which is what makes any live-camera feature
usable from a phone off the deploying machine's own LAN at all.

**Why teleop got its own sibling stacks** rather than reusing
`twin.yaml`/`brain.yaml` directly: `robot/factory.py` picks exactly one
backend per process at boot, and `control/brain_server.py` picks exactly
one `robot_url` at start -- so running sim-mode and teleop-mode
*simultaneously* needs two robot processes and two brain processes, not
a flag flip on the existing ones (which would take the Sim tab away
while flipped). `ROUTE_PREFIX` (env var, empty by default so the
existing deployments are unaffected) is what lets a second instance of
the same Docker image share the one ALB listener instead of needing its
own. Deliberately new files rather than parameterizing the existing
templates, specifically so a mistake in the new stack can't touch the
already-running one.

**Secrets:** each service gets its own generated `x-app-secret` in
Secrets Manager (`vision-picar-<service>-shared-secret`), entered
manually into the twin's Settings fields -- `serverSecret` for whichever
robot server is connected, `brainSecret` for whichever brain, `visionSecret`
for the shared vision-analyze endpoint. They are never assumed to match
each other once each service is independently deployed (a bug fixed live
during teleop testing: the twin was silently sending the robot's secret
as the brain's too, which 401s the instant they differ).

**Models called, per route** (`service/vision_analyze/vision_core.py`):
`/analyze` -> `us.anthropic.claude-sonnet-4-5-20250929-v1:0` (default,
overridable via `BEDROCK_ANALYZE_MODEL_ID`); `/navigate` and `/guidance`
-> `amazon.nova-lite-v1:0` (default, overridable via
`BEDROCK_NAVIGATE_MODEL_ID` / `BEDROCK_GUIDANCE_MODEL_ID`) -- chosen for
the several-times-a-second loops where latency dominates the experience,
against Sonnet's stronger reasoning for the one-shot "analyze this photo"
case where accuracy matters more than round-trip time.

---

## 7. Where to look next

- **`PLAN-ar-guidance.md`** -- Guide me's full build history: every
  redesign, bug, and UX decision, in order, with dates.
- **`PLAN-teleop-robot.md`** -- Robot view's "Drive via brain" build
  history, including the two real bugs found and fixed against the live
  AWS deployment on 2026-08-28.
- **`AGENT-HARNESS.md`** -- the mission tick's internals: seams,
  concurrency model, status contract, invariants. Read before changing
  anything in `control/`.
- **`CLAUDE.md`** section 7 -- the phase -> UI-action -> expected-result
  table this doc's Guide/Sim sections expand on.
