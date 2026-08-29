# Features -- what's in the app and how it works

A feature-by-feature reference for `web-twin/index.html`, the mobile-first
web app that is this project's one real UI. Written against the repo as
of `01054a0`, which lands the teleop work described in
`PLAN-teleop-robot.md`. Where `README.md` explains *why* something was
built and `CLAUDE.md` tracks *status*, this file explains *how each
button actually works* -- which process it talks to, which route, and
what comes back.

The app has four tabs: **Guide**, **Camera**, **Sim**, **Settings**. This
doc covers all four, then the AWS deployment they run against, in the
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
  money on a PiCar-X.

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
  resets to the base rate the moment a call succeeds.
- A capture is skipped (no call made) if the phone's orientation sensor
  reports it's been essentially still since the last tick
  (`GUIDANCE_STILL_SKIP_DEG_PER_SEC`), up to `GUIDANCE_MAX_CONSECUTIVE_SKIPS`
  in a row -- saves a paid call when nothing in frame could have changed.
- A hard call cap per session (`GUIDANCE_MAX_CALLS` / `ROBOT_MAX_CALLS` =
  **120**, ~1 minute at 500ms) auto-pauses with a "Resume searching"
  button, so a forgotten-running tab can't burn money unbounded.
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
  "action": "FORWARD" | "LEFT" | "RIGHT" | "REVERSE" | "STOP",
  "reasoning": "one short sentence"
}
```
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
- Hold the phone low, ~10cm -- roughly PiCar-X camera height. A
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
   switch.
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
  no ultrasonic here" constant `ReplayRobot` uses, so `robot/safety.py`
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
mission today), and Ackermann-style motion realism (not relevant here
since nothing physically moves under this mode anyway).

---

## 2. Camera tab -- Analyze a photo

The original, one-shot vision feature, unaffected by anything above.

- `<input type="file" accept="image/*" capture="environment">` -- the
  native camera picker, which (unlike Guide's live `getUserMedia`) works
  fine over plain `http://`, no secure-context requirement.
- Optional target-object text field.
- **Backend:** `POST /analyze` on the vision-analyze service
  (`describe_image_bytes()` if no target given, model
  `us.anthropic.claude-sonnet-4-5-20250929-v1:0` by default -- the one
  route still on a Claude model rather than Nova Lite, since it's a
  one-off analysis rather than a several-times-a-second loop and
  benefits from the stronger model). Response includes `room_guess`
  (`identify_room()` against the detected objects).
- Downscaled to 1568px long edge before upload (`PHOTO_MAX_CAPTURE_DIM`)
  -- Anthropic's own server-side downscaling threshold, so anything
  larger just burns upload time and per-pixel Bedrock cost for zero
  quality gain.
- A single request/response; no loop, no camera left open, nothing to
  Stop.

---

## 3. Sim tab -- the grid-world twin

Everything here talks to `robot/server.py` (not the vision-analyze
service directly, except Vision Autopilot's per-tick calls) and, for the
grid rendering, a JS port of `sim/grid_world.py`'s starter-house layout.

- **Top-down canvas + telemetry** -- room, facing, free space, doorway,
  visible objects, live safety status. Rendered from the same `/frame`,
  `/distance`, `/action` calls every other panel here uses.
- **Manual control (D-pad)** -- direct `POST /action` / `POST /stop`
  calls. Every move passes through `robot/safety.py`'s
  `SafetyController` on the server, the same veto path an AI decision
  gets -- a human driving over Wi-Fi has the same collision protection.
  Disabled until Settings' robot connection succeeds.
- **Remote brain** (`control/brain_server.py`'s real `MissionRunner`) --
  `POST {brainUrl}/mission/start` with a target and an optional fault
  drill, then this panel becomes a pure *observer*, polling
  `GET /mission/status` for step count, last action, rooms searched, a
  log tail, and following the robot on the map. Closing the tab or
  reopening it later shows the mission further along or finished -- the
  proof that the mission is a service call, not page state (B4). A
  **failsafe drill picker** (`control/drills.py`, gated off by
  `brain.allow_drills: false`) injects exactly one fault --
  vision-service errors, vision-service hangs, or a hung brain loop --
  so the two guards that can't be provoked by pressing anything
  (B3.2, B3.3) can still be watched firing, always ending with the robot
  stopped. A **watchdog readout** shows `robot/server.py`'s own B3.1
  silence counter.
- **Local brain, rule-based** (`Explore` / `Find backpack` / `Reset
  mission`) -- the frontier-preference exploration algorithm from
  `brain/agent.py`, re-implemented in this page's JavaScript, driving
  the same `/action` calls a human would via the D-pad. Free (no API
  calls), deterministic, kept specifically because it needs no brain
  service and is the fastest way to exercise the safety layer and
  mission memory -- not on the hardware path, and not meant to be
  extended further.
- **Local brain, Vision Autopilot** -- the browser's own vision-in-the-
  loop driver: since the grid-world sim has no real camera, a small
  canvas raycaster (`renderFPV`, against the same map data the top-down
  view uses) renders a synthetic first-person frame, POSTs it to
  `/navigate` every ~2.5s with a target object, and executes whatever
  action comes back through the same `commitAction()`/safety-veto path
  every other mode uses. Never auto-starts; a visible call counter and a
  step cap bound the cost. This is the mode Robot view's
  `renderRobotOverlay` machinery was originally built for, reused
  verbatim for real pixels.

**Remote and Local brains are mutually exclusive**, enforced both
directions: the twin refuses to start a local loop during a remote
mission, and the brain server answers a second `/mission/start` with
`409` if one is already running.

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
  shared by Guide, Robot view's default sub-mode, Vision Autopilot, and
  Camera-tab analysis. Enter the base URL only; each feature appends its
  own route (`/analyze`, `/navigate`, `/guidance`) via `deriveServiceUrl()`.
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
