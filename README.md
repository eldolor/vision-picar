# vision-picar

An indoor robot car you can send to find something, built simulation-first:
the decision-making is built and proven against a simulator before any
hardware is bought. For a non-technical introduction read [`INTRODUCTION.md`](docs/guides/INTRODUCTION.md);
for orientation in the code read `CLAUDE.md`.

**Where things stand (2026-10-02).** The compute board has arrived and the
chassis is on order; nothing has run on real hardware yet.

- **Hardware (bought):** an NVIDIA **Jetson Orin Nano Super** (arrived
  2026-09-30, to run at 15 W to start) on a **Waveshare UGV Rover** (ordered
  2026-09-30, ~$730, due Oct 19 - Nov 11). The Rover is differential drive
  (it pivots in place) and brings its own **ROS Driver** ESP32 board (closed
  loop, encoder odometry to the host, 660 pulses/rev), a **D500 lidar**, an
  **OAK-D Lite** camera and a pan-tilt. Still to buy: a separate Jetson
  battery, and only if the 15 W stress test says so (`JETSON-BOM.md` 9.5).
  The Raspberry Pi 5 + Hailo-8L plan, and the RPLidar C1 / IMX219 / General
  Driver build of 2026-09-19, are history. Record in `JETSON-BOM.md`
  section 9; concepts in `GUIDE-robot-base.md`.
- **Next: the Jetson before the Rover** (`PLAN-ros-alignment.md` 3.33). It
  gets opened while it is still returnable: firmware check, JetPack 6.2.1,
  torch on the GPU, the perception tier's latency (budget 250 ms a frame at
  15 W), and gate G4 -- the whole stack on the board against the fake motor
  board.
- **ROS 2 Humble, in one container** (`service/slam/`): the URDF,
  `ros2_control`, `slam_toolbox` and nav2 run there and nowhere else. The
  rest of the project talks to it over HTTP; a test fails if anything
  outside `service/slam/` imports `rclpy`. Off by default (`drive: ros`,
  `WORLD_MODE=ros` turn it on).
- **The brain is a separate service** (`control/brain_server.py`, :8001)
  from the robot runtime (`robot/server.py`, :8000), and only ever reaches
  the robot through `RobotInterface` over HTTP. Both run locally today; the
  deployed twin reaches them through a tunnel (`service/tunnel/`).
- **The simulator** now has continuous pose and wheel kinematics, solid
  objects, a lidar, several houses (`SIM_MAP`) including a model of the
  owner's own first floor, the Rover's CAD geometry (lidar 4 cm ahead of the
  rotation centre), and people and pets that move (`SIM_MOVERS`). The Rover's
  motor board is faked on a serial line (`sim/fake_esp32.py`), so
  `robot/hardware_robot.py` -- the real motor backend -- already runs
  against it.
- **Our firmware fork** (`firmware/ugv_base_ros/`, GPL-3.0, not flashed)
  adds 0.1 mm odometers and a board timestamp to the Rover's board; with it,
  direct-mode turns settle within +/-1 degree.
- **`drive: ros` is to become the car's default**: gates G1-G3 are met in
  the sim (a reliable live chain, the same safety bars as direct mode, and
  a fallback where only a person drives if ROS dies); G4 needs the Jetson.
- **Every component has two specs** under `docs/` -- an architecture spec
  (what and why) and an engineering spec (how, today) -- checked by
  `tools/spec_lint.py` in the test suite.

`CLAUDE.md` section 3 has the authoritative built-vs-planned table.

**Where to read next**

| Doc | For |
|---|---|
| `CLAUDE.md` | orientation: status table, repo map, gotchas. Start here. |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | the whole system on one page |
| [`docs/README.md`](docs/README.md) | the specifications: an architecture and an engineering spec for each of 15 components, the reading path, and where everything else lives |
| [`docs/guides/INTRODUCTION.md`](docs/guides/INTRODUCTION.md) | what the project is, for a non-technical reader |
| [`docs/plans/PLAN-ros-alignment.md`](docs/plans/PLAN-ros-alignment.md) | the current plan: phases R0-R7 and 3.17-3.33 (continuous pose, ROS 2, SLAM, nav2, safety, the motor board, the Jetson bring-up), each closed on pre-stated data |
| [`docs/handoffs/`](docs/handoffs/) | session handoffs, dated; the newest says what is open |
| [`service/slam/README.md`](service/slam/README.md) | the ROS 2 container: how to build and run it |
| [`docs/guides/AGENT-HARNESS.md`](docs/guides/AGENT-HARNESS.md) | how `control/` works: the mission tick, seams, failsafes, invariants |
| [`docs/guides/FEATURES.md`](docs/guides/FEATURES.md) | every feature of the twin, how it works end to end, and the AWS topology it runs against |
| [`docs/hardware/`](docs/hardware/) | what to buy and what was bought (`JETSON-BOM.md`), part numbers and wiring (`HARDWARE-BOM.md`), pre-flight (`HARDWARE-READINESS.md`), prices, the Pi-vs-Jetson walkthrough, and a guide to robot bases |
| [`docs/plans/`](docs/plans/) | the other phase plans: perception, mapping, sim hardening, Microduck transplants, brain relocation, the Guide tab, teleop, AWS cost |

The Guide tab has two modes: **Guide me** steers a person to an object
(`/guidance`), and **Robot view** shows the move the robot would make from
where you are standing (`/navigate`) -- the same decision the brain's
`policy: "vision"` makes, but on real pixels rather than the simulator's
render.

The phase numbering below comes from the original `picar-x-build-plan.md`,
which lives in the Claude Project this work started in and is **not in
this repo** -- the tables in `CLAUDE.md` are the in-repo source of truth.

The rest of this file is an append-only build journal, oldest first,
last extended 2026-10-02. Entries describe things as they were built; some
of them (the browser's Explore/Find and Vision Autopilot, the ECS
deployment) have since been removed and are marked where they appear.

---

## Phase 0 -- mock robot interface

- `robot/interface.py` — abstract contract brain/ will always code against
- `robot/factory.py` — reads `config/robot.yaml`, returns the right backend
- `robot/safety.py` — local safety layer; can veto AI actions (Phase 3, done early)
- `sim/grid_world.py` — 2D grid-world engine
- `sim/mock_robot.py` — implements `RobotInterface`, backed by grid-world
- `sim/maps/starter_house.py` — living room / hallway / kitchen map with a
  red backpack in the kitchen
- `tests/test_mock_robot.py` — validates Phase 0 success criteria + safety override
- `tests/demo_manual_loop.py` — runnable demo of move/sense/safety-check loop

## Setup

Same as `CLAUDE.md` section 1:

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# once, so the browser UI tests run instead of skipping
python -m playwright install chromium

# only for policy: "tiered" (YOLOE + CLIP in the brain process)
pip install -r requirements-perception.txt
```

## Run tests

```bash
pytest tests/ -q                           # no API key needed
pytest service/vision_analyze/tests/ -q    # that service's own suite, run separately
```

Counts drift quickly; `pytest tests/ --collect-only -q` prints the current
one. The UI tests (`tests/test_ui*.py`) skip without the Playwright browser,
and the live ROS tests skip without the container running.

## Run the demo loop

```bash
python -m tests.demo_manual_loop
```

## Phase 1 -- Vision LLM scene understanding

- `brain/vision.py` — `describe_image()` sends a real/stock photo to the
  Vision LLM and returns a structured scene description (obstacles, free
  space, doorway, objects, safest direction). `describe_grid_frame()`
  converts the grid-world's own frame into the identical schema with no
  LLM call, so `brain/agent.py` (next) never has to care which backend
  produced a given observation.
- `tests/test_vision.py` — grid-frame conversion tested directly (free,
  offline); `describe_image()` tested against a mocked API client so the
  whole suite still runs with no API key and no cost.
- `tests/manual_describe_image.py` — the actual Phase 1 milestone check:
  run this against a real phone/stock photo of a room and eyeball whether
  the description is sensible. Needs `ANTHROPIC_API_KEY` set; not part of
  the automated suite since it costs a real API call.

```bash
export ANTHROPIC_API_KEY=sk-...
python -m tests.manual_describe_image path/to/photo.jpg
```

## Phase 2 -- constrained action loop

- `brain/agent.py` — `ConstrainedAgent`: the full capture → vision →
  decide → safety-check → execute loop, restricted to
  `FORWARD/LEFT/RIGHT/REVERSE/STOP/LOOK_LEFT/LOOK_RIGHT`. Includes a
  stuck-breaker so it doesn't freeze forever facing a wall (a minimal
  placeholder for the real exploration logic Phase 4+ will add).
- `tests/test_agent.py` — proves the loop never lets the robot collide
  (safety vetoes every risky move before `grid_world.move()` runs), and
  that the stuck-breaker actually fires.
- `tests/demo_explore.py` — the first real checkpoint demo: "explore this
  room without hitting anything," fully autonomous, zero hardcoded plan.

```bash
python -m tests.demo_explore
```

Note: decision-making here is intentionally dumb (trust `vision`'s
`safest_direction`, turn if stuck) — it's a scaffold for Phase 4's agent
harness (mission, memory, object sightings), not the final logic.

## Phase 4 -- agent harness / mission memory

- `brain/memory.py` — `MissionMemory`: tracks the mission, rooms
  visited/searched, object sightings, and action history. `as_context()`
  formats it into the harness prompt block from the build plan, ready
  for a real planner/LLM to consume once Phase 5 needs it.
- `brain/agent.py:MissionAgent` — adds mission awareness on top of
  `ConstrainedAgent`: records every step into memory and stops with
  success once the target object is sighted. Navigation upgraded to
  **frontier-preference exploration** (peek forward/right/left via
  camera pan, prefer whichever clear direction leads to an unvisited
  cell) -- Phase 2's simpler "trust vision's safest_direction" only
  loops the boundary of the starting room and never finds anything in
  another room.
- `tests/test_memory.py`, `tests/test_mission_agent.py` — target
  detection, memory summaries, and a full end-to-end "find it" run
  against the starter house.
- `tests/demo_find_backpack.py` — the second checkpoint demo: "find the
  red backpack," fully autonomous, reports rooms searched and where it
  was sighted.

```bash
python -m tests.demo_find_backpack
```

Note: exploration is still blind full-coverage, not goal-directed ("go to
the kitchen") -- that's Phase 5. Object detection is also still passive
(a visited room's frame just reveals what's in it) -- deliberate
look-around scanning is Phase 6.

## Phases 5 & 6 -- semantic navigation and object search

- `brain/rooms.py` — `identify_room()`: matches visible objects against
  per-room landmark features (the JSON-style schema from the build plan)
  to guess the current room. Standalone and independently tested; the
  grid-world's ground-truth room label is still what actually drives
  navigation (identify_room is what a real VLM pipeline would use
  instead, once there's no ground truth to check against).
- `brain/memory.py` — `MissionMemory` now accepts `target_room` alongside
  `target_object`. `is_complete()` is true once *either* goal is met, so
  `MissionAgent` can run "find X", "go to room Y", or both in one mission
  with no code changes -- see `tests/test_semantic_navigation.py`.
- `brain/agent.py:ObjectSearchAgent` — Phase 6: on first entering any
  room, runs a real `LOOK_LEFT → LOOK_RIGHT → LOOK_CENTER` scan (each a
  dispatched, safety-checked action) before continuing exploration, so
  it catches objects to the side instead of only whatever's dead ahead.
  Skips the scan entirely for room-only missions with no object target.
- `tests/demo_go_to_room.py` — Phase 5 checkpoint: "go to the kitchen,"
  a pure room-level goal.
- `tests/demo_active_search.py` — Phase 6 checkpoint: "find the red
  backpack" using active scanning instead of passive detection.

```bash
python -m tests.demo_go_to_room
python -m tests.demo_active_search
```

## Simulation checkpoint -- passed

Both build-plan checkpoint demos run end to end in simulation: "explore
without hitting anything" (Phase 2) and "find the red backpack" (Phase
4/6, with active scanning). Per the plan, this is the gate before Part B
(buying hardware), and it is met.

**But "the checkpoint passed" is a weaker statement than it sounds**, and
that gap is the subject of `PLAN-sim-hardening.md`. In short: the safety
threshold is never exercised at its boundary (simulated distances are
quantised to 30cm, so `min_distance_cm: 20` only ever triggers at 0),
turning is free in a grid world but is an arc that consumes forward space
on a real Ackermann chassis, and the map is 3.9m x 3.0m at its own scale
-- with a 90cm living room a PiCar-X could not turn inside. Read
`HARDWARE-READINESS.md` before treating this checkpoint as clearance to
buy.

## Wi-Fi control API (Phase 9, sim-testable now)

- `robot/server.py` — the real `POST /action`, `POST /stop`,
  `GET /distance`, `GET /frame` API from Phase 9, running against the
  mock backend. Every `/action` call goes through `robot/safety.py` --
  a human driving over Wi-Fi gets the same collision protection an AI
  decision does. Includes the watchdog requirement (stop motors if the
  MacBook goes quiet for ~1s); the pure decision logic
  (`watchdog_should_stop`) is unit-tested, the actual async polling loop
  is exercised by running the server for real.
- `tests/test_server.py` — safety enforcement over HTTP, watchdog logic,
  all endpoints.

```bash
uvicorn robot.server:app --reload   # http://127.0.0.1:8000
```

This file doesn't change at all when Phase 11 swaps in real hardware --
only `config/robot.yaml`'s `mode` does.

## The brain on the wire (phases B0-B3)

`PLAN-brain-relocation.md`'s Stage 2. The autonomy loop stops being a
blocking `for` loop inside an agent and becomes a service that can be
started, stopped, and inspected over HTTP -- the prerequisite for the
robot being self-contained, with the phone as an observer rather than the
brain.

**The whole trick:** the brain is *always* an HTTP client of
`robot/server.py`. Then "brain on the Pi" and "brain on the MacBook"
differ by a base URL and nothing else.

```
  :8001  control/brain_server.py     the autonomy loop
             |  HTTP (localhost on the Pi, LAN from a MacBook)
             v
  :8000  robot/server.py             safety + watchdog + backend
```

**`AGENT-HARNESS.md` is the reference for how this works** -- one tick in
order, the seams, the concurrency model, the status contract, the
invariants, and how the S2b vision policy plugs in. What follows is the
per-phase summary.

- `control/remote_robot.py` (**B0**) -- `RemoteRobot` implements
  `RobotInterface` by calling `robot/server.py`. `brain/agent.py` cannot
  tell it apart from an in-process backend: a safety veto raises the same
  `SafetyViolation` it would locally, and `position` is coerced back to a
  tuple after its round trip through JSON.
- `control/mission_runner.py` (**B1**) -- one mission's lifecycle as
  `start()` / `tick()` / `stop()` / `status()`, with the loop driven from
  outside. No decision logic moved; the frontier-exploration policy is
  the same one the demos validated.
- `control/brain_server.py` (**B2**) -- FastAPI on :8001 owning a
  `MissionRunner` and driving it as an asyncio background task.
  `POST /mission/start`, `POST /mission/stop`, `GET /mission/status`,
  `GET /health`. Imports nothing from `sim/`, and touches `robot/` only
  for the interface -- asserted by a test.
- Three failsafes (**B3**), one per distinct failure:
  **B3.1** motors left running (`robot/server.py`'s watchdog, unchanged);
  **B3.2** the AWS link dead (a vision-call timeout plus a
  consecutive-failure budget -- the browser autopilot just logs the error
  and schedules the next tick); **B3.3** a brain loop alive but stuck (a
  per-tick dead-man the watchdog cannot see). All three end with the same
  thing: the robot is told to stop.

```bash
# two processes, the way they run on the Pi
uvicorn robot.server:app --port 8000
uvicorn control.brain_server:app --port 8001

curl -X POST localhost:8001/mission/start \
     -H 'content-type: application/json' \
     -d '{"target_object": "red backpack"}'
curl localhost:8001/mission/status
curl -X POST localhost:8001/mission/stop     # stops the loop AND the car
```

To run the brain on the MacBook instead, point `brain.robot_url` in
`config/robot.yaml` at the Pi's LAN IP and start `brain_server` there.
That is the only change.

**Proof it is transparent:** `python -m tests.demo_brain_over_http` runs
the backpack hunt three ways -- in-process, through `RemoteRobot` over a
real socket, and through the brain service -- and compares them. All
three take the same 83 steps and produce an identical action sequence.
`tests/test_remote_robot.py` asserts it against a live `uvicorn`,
`tests/test_brain_server.py` runs a whole mission through two HTTP hops,
and `tests/test_failsafes.py` covers B3.2 and B3.3.

### The vision policy on real pixels (phase S2b, partial)

The loop that matters -- a model looking at a photograph and choosing the
move -- now exists in Python, not only in the browser:

- `brain/navigate.py` -- the `vision_fn`: a frame goes to the vision
  service's `/navigate`, one action comes back. One mapping decision is
  load-bearing: the target counts as **found only when the service reports
  `target_reached`**, never on mere visibility, or a mission would end in a
  doorway across the room from the backpack and call it success.
- `brain/vision_agent.py` -- `VisionAgent`: trust the model's action unless
  the mission is over. Four lines, because validation, the stuck-breaker
  and mission memory are all inherited.
- `sim/replay_robot.py` -- a body made of photographs. A walk you recorded,
  played back one frame per move.

```bash
# record a walk in the twin (Guide -> Robot view -> "Record this walk"), then:
export VISION_URL="https://<your vision service>"
python -m tests.demo_replay_mission recordings/<walk> "red backpack"
```

That prints one line per step -- action, frame, and the model's own
reasoning -- then the outcome, the call count, and the wall clock. It is
the first thing in this project to put the *loop* on real pixels rather
than one frame at a time.

**Honest limits.** A replay is open loop: the frames follow the path you
walked, so a LEFT at frame 12 doesn't change what frame 13 shows. It
measures memory, lifecycle and cost; it does not measure navigation. The
safety layer is inert (a photograph has no distance in it). And the vision
policy can't drive the simulator yet -- `MockRobot` has no pixels until
phase S2.

### Watching it from the twin (phase B4)

*(The **Local brain** described below was removed 2026-09-25; the Sim tab
now has only the Remote brain.)*

The Sim tab has two brains side by side. **Remote brain** starts a mission
on `control/brain_server.py` and then only observes it -- polling
`/mission/status` for the step count, last action, rooms searched and log
tail, and following the robot on the map. **Local brain** is the JS loop
that has always been there, kept because it needs no brain service at all.
Only one may drive at a time, enforced from both ends.

Close the tab mid-mission and reopen it: the mission is further along, or
finished. That is the whole point of the phase, and it is a thing you can
watch rather than a thing a test asserts.

Two guards would otherwise be unverifiable by hand, since neither can be
provoked by pressing anything -- so the panel has a **failsafe drill**
picker (`control/drills.py`) that asks the brain to break exactly one
thing, and a **watchdog readout** showing the silence `robot/server.py`
measures:

| Drill | Breaks | What you should see |
|---|---|---|
| Vision service errors | every vision call raises | three failures counted, then `failed`, robot stopped |
| Vision service hangs | every vision call never returns | same, reason says "timed out" |
| Brain loop hangs | the loop stops returning | one step, then `failed` -- "brain loop hung" |

Every drill is fail-safe by construction: it can only end a mission with
the robot stopped. `brain.allow_drills: false` removes the surface
entirely. See `CLAUDE.md` section 7 for the standing rule this comes from
-- every stage ships with something you can press.

## Digital twin / web-based visualization

- `web-twin/index.html` — mobile-first web page: canvas view of the
  starter house, manual D-pad control, autonomous "Explore"/"Find
  backpack" modes (**removed 2026-09-25** -- missions now start from the
  Remote brain panel) (a JS port of the frontier-preference algorithm in
  `brain/agent.py` -- verified to match the Python sim's behavior
  step-for-step), and the "take a photo, find the bag" feature calling
  the cloud vision service below. It's a real HTTP client of
  `robot/server.py` -- every move, sensor read, and safety check goes
  through the actual server, not a duplicated simulation (see "Is this
  deviating from the hardware plan?" below for how that was verified).
- **Deployed to ECS Fargate** (`service/twin/`, `cloudformation/twin.yaml`)
  *(torn down 2026-09-05 -- `PLAN-aws-cost-redesign.md`. The page is now
  served from S3 through CloudFront, and the robot runs locally)*
  so it's reachable from a phone on any network, not just a Mac's LAN --
  same cluster as the vision service, sharing its NLB and internal ALB
  on the *same port 80* instead of provisioning a second port or a
  second pair of load balancers. A `ListenerRule` on the shared ALB
  listener routes `robot/server.py`'s exact route set (`/`, `/action`,
  `/stop`, `/distance`, `/frame`) to the twin's target group; everything
  else on that listener falls through to the vision-analyze service.
  Public URL: `http://<the vision service's NLB DNS name>/` (same DNS
  name and port as the `/analyze` endpoint below -- routed by path, not
  port). `/action`, `/stop`, `/distance`, `/frame` require an
  `x-app-secret` header once deployed publicly (see `require_secret()`
  in that file) -- the secret lives in Secrets Manager as
  `vision-picar-twin-shared-secret`. `/` (which now serves
  `web-twin/index.html` directly) stays open, since the page has to
  load before anyone can enter the secret. (`/health` isn't in the
  ListenerRule's path list -- it falls through to vision-analyze's own
  `/health`; the twin target group's own health checks bypass listener
  routing entirely and hit the container directly, so this doesn't
  affect the twin's health checks.)
- When served by `robot/server.py` itself (this deployment, or local
  `uvicorn robot.server:app`), the twin's "Robot server connection"
  field auto-fills to its own origin -- no more typing LAN IPs. Manual
  entry still works for pointing at a different server.
- Also still runs the original way for local hardware-adjacent dev:
  `uvicorn robot.server:app --host 0.0.0.0` on a Mac, LAN IP in the
  connection field, no secret needed (unset `APP_SHARED_SECRET` makes
  `require_secret()` a no-op) -- see `web-twin/README.md`.
- **Vision autopilot panel** (**removed 2026-09-25**, with the browser's
  raycaster; the brain's `policy: "vision"` replaces it): a third driving mode, alongside manual and
  rule-based Explore/Find, where the twin actually drives itself with real
  Claude Vision calls instead of the JS frontier algorithm. Since the
  grid-world sim has no real camera, `web-twin/index.html` renders a
  synthetic first-person view with a small canvas raycaster (against the
  same `LAYOUT`/`OBJECTS` constants the top-down map already uses), POSTs
  it to the vision service's new `/navigate` route every ~2.5s with a
  target object, and executes whatever action comes back through the same
  `commitAction()`/safety-veto path every other mode uses. Never
  auto-starts (each tick is a real paid API call -- there's a visible call
  counter and a step cap). See "Cloud vision endpoint" below for the new
  route; this reuses that service's existing URL/secret fields, deriving
  `/navigate` from the configured `/analyze` URL.
- **"Guide" tab**: the inverse of Vision Autopilot -- the *person* holding
  the phone walks around while Claude Vision, given the phone's real live
  camera feed, guides them toward a named object. Its own top-level tab
  (not nested in Camera), and tapping Start takes over the full screen
  (CSS-simulated, not the real Fullscreen API -- more reliable on iOS
  Safari) in both portrait and landscape. Built per `PLAN-ar-guidance.md`'s
  spec, then redesigned per that doc's "Redesign" section -- see there for
  the full detail. Uses `getUserMedia`, so it requires this page be loaded
  over `https://` or `localhost` -- a plain `http://<lan-ip>` URL (the
  twin's original LAN-testing setup) fails the browser's secure-context
  check for camera access. Calls the vision service's `/navigate`-sibling
  `/guidance` route roughly every ~1s (tightened from an initial ~2s after
  real-device testing found the slower loop too laggy for panning around a
  room -- see `PLAN-ar-guidance.md`'s changelog for the cost tradeoff);
  never auto-starts the camera, and only releases it on Stop
  (backgrounding the tab pauses analysis but keeps the camera live). The
  captured frame is also downscaled to a 960px max dimension before
  upload -- smaller payload, faster inference, no visible quality loss for
  this coarse a question.
  - Once found, draws a glowing outline around the object from the
    response's `bounding_box` field (a real but approximate LLM-estimated
    rectangle, not pixel segmentation, refreshed each tick), positioned
    with `object-fit: cover`-aware coordinate mapping (the video is
    cropped to fill the screen, so naive percentage math misplaces the
    box -- see `mapNormalizedBoxToScreen()` in `web-twin/index.html`).
  - Visual-first guidance: a directional edge glow (brightens along the
    screen edge to turn toward, intensity scaled by how far off-center)
    and a chevron whose pulse speeds up as proximity increases are the
    primary "which way / how close" signal -- the caption text at the
    bottom is a small, de-emphasized readout, not the main channel.
  - Fires `navigator.vibrate()` patterns (works on Android Chrome) and
    Web Audio directional tones (panned left/right, works everywhere
    including iOS). **iOS Safari has never implemented the Vibration
    API** -- this is a confirmed Apple/WebKit platform limitation, not a
    bug, so the audio cues are the actual cross-platform feedback
    mechanism; don't "fix" the iOS silence.

## Cloud vision endpoint (photo analysis, reachable from anywhere)

- `service/vision_analyze/` — a FastAPI app wrapping the same
  `describe_image()` logic as `brain/vision.py` (adapted to accept
  photo bytes from a browser upload instead of a file path), plus a
  room guess via `identify_room()`. This is deliberately separate from
  `robot/server.py`: photo analysis benefits from being reachable from
  anywhere (cellular, not just home Wi-Fi) and having model access live
  in the cloud instead of on a device, but the actual drive/steer/stop
  control loop stays local once hardware exists -- safety-critical
  control shouldn't depend on a cloud hop being up.
- **`POST /navigate`** — a second route alongside `/analyze`, added for
  the twin's Vision Autopilot panel above: given a camera frame and a
  `target_object`, returns a single next `action`
  (`FORWARD`/`LEFT`/`RIGHT`/`REVERSE`/`STOP`) plus short `reasoning`,
  instead of a full scene description. Same auth/size/decode handling as
  `/analyze` (factored into a shared `_decode_image()` helper in
  `app.py`), same Bedrock Converse plumbing in `vision_core.py`
  (`describe_image_bytes_navigate()`), different prompt/schema. A
  deliberately separate route rather than a flag on `/analyze`, to avoid
  colliding with `/guidance` below, which uses the same `target_object`
  field for a different, human-facing purpose. Not mirrored into
  `brain/vision.py` -- this is a browser-only feature, unlike the rest of
  the vision logic that mirror keeps in sync.
- **`POST /guidance`** — a third route, added for the twin's "Guide me
  to..." panel above: given a camera frame (a real phone photo this time,
  not a synthetic sim render) and a `target_object`, returns
  `target_visible`/`position` (5-zone: `far_left`...`far_right`/
  `not_visible`)/`proximity`/a short human-readable `guidance` string, for
  driving an AR overlay arrow rather than executing a robot action. Same
  `_decode_image()`/Bedrock-plumbing pattern as `/navigate`
  (`describe_image_bytes_guidance()` in `vision_core.py`). This is the
  route `PLAN-ar-guidance.md` originally speced as an addition to
  `/analyze`; built as its own route instead once `/navigate` already
  existed, for the same target-object-field reason above.
- Runs as an **ECS Fargate** service (not Lambda -- see history note
  below) *(superseded 2026-09-05: the ECS stacks were deleted and
  `/navigate`, `/guidance` and friends now run as Lambda behind API
  Gateway in the `serverless` stack -- `PLAN-aws-cost-redesign.md`)* behind an internet-facing **NLB → internal ALB → ECS Fargate**
  chain, provisioned by the CloudFormation templates in `cloudformation/`
  (`network.yaml`: VPC across 2 AZs, no NAT Gateway; `service.yaml`:
  ECR repo, ECS cluster/service/task, both load balancers, IAM roles,
  the shared-secret in Secrets Manager). The private subnets have **no
  internet route at all** -- everything the task needs (ECR image pull,
  CloudWatch Logs, Secrets Manager, and the vision model call itself)
  goes over VPC interface endpoints instead.
- Vision inference calls **Amazon Bedrock** (Claude, via the Converse
  API) rather than the direct Anthropic API `brain/vision.py` and the
  sim use -- Bedrock supports a private VPC endpoint, so the service
  never touches the public internet; auth is the ECS task role's IAM
  permissions, not an API key. See `service/vision_analyze/vision_core.py`
  for the model ID in use and why (not every Claude model is enabled for
  Bedrock on every account -- check with `aws bedrock list-foundation-models`
  before assuming a given model ID works).
- Still has the same shared-secret header check and CORS handling as
  before; `APP_SHARED_SECRET` lives in Secrets Manager, injected into
  the task at launch.

```bash
# local smoke test, no AWS needed except Bedrock credentials:
cd service/vision_analyze
docker build -t vision-picar-analyze:local .
docker run -p 8080:8080 -e APP_SHARED_SECRET=test -e ALLOWED_ORIGINS='*' \
  -e AWS_PROFILE=default -v ~/.aws:/root/.aws:ro vision-picar-analyze:local
```

```bash
# deploy (see cloudformation/ templates for the full resource list)
aws cloudformation deploy --stack-name vision-picar-network \
  --template-file cloudformation/network.yaml
aws cloudformation deploy --stack-name vision-picar-service \
  --template-file cloudformation/service.yaml --capabilities CAPABILITY_NAMED_IAM
```

### History: why not Lambda?

This started as a Lambda Function URL (and, after that, an API Gateway
HTTP API in front of the same Lambda) -- both code-complete and correct
(verified via direct `aws lambda invoke`), but every public entry point
into that specific AWS account was silently rejected before the function
ever ran. ECS Fargate behind a load balancer is a completely different
invocation path, so it isn't subject to the restriction. The Lambda code
and its API Gateway have been deleted; this is documented here rather
than left to be rediscovered from git history, since it explains a real
architectural choice, not just "we changed our minds."

**The original root-cause guess on this line was wrong, and the real one
was measured on 2026-09-04.** It used to blame the account's Lambda
concurrency quota, which was pinned at 10 instead of the default 1000 --
read as AWS having placed the account in a reduced-trust tier blocking
"Lambda-based public ingress". Two things are now known:

- **The quota was never the constraint.** It reads 1000 today, and a cap
  of 10 was always ample for one phone -- Robot view allows two
  `/navigate` calls in flight (`GUIDANCE_MAX_IN_FLIGHT`).
- **It is not about ingress, and not about "public" at all.** A probe
  built and torn down on 2026-09-04 established that **Lambda
  resource-based policies do not grant invocation on this account**,
  while identity-based auth works normally. A Function URL called with a
  SigV4 signature from an IAM user with `AdministratorAccess` returned
  200 from a laptop off the AWS network, and the function logged the
  caller's real public IP -- so HTTPS ingress to Lambda plainly works.
  The same URL returned 403 for anonymous access, for CloudFront with
  Origin Access Control, and for a role holding *only* a resource-policy
  grant, and CloudWatch shows the function was never invoked in any of
  those three cases. The account is not in an AWS Organization, so no SCP
  or RCP explains it.

That is why *both* original attempts failed: a Function URL and an API
Gateway integration each invoke Lambda through a resource-based policy.
It also explains why `aws lambda invoke` worked throughout -- that path
is identity-based.

The consequence for any future redesign is in
`PLAN-aws-cost-redesign.md` section 6. In short: CloudFront in front of a
Function URL cannot work here, and the one Lambda pattern that might is
an API Gateway integration carrying an explicit `credentials` role, which
assumes a role rather than relying on the function's resource policy --
**measured working the same day**, anonymously, from off the AWS network,
with the function carrying no resource policy at all.

## Is this deviating from the hardware integration plan?

**No.** `RobotInterface` + `robot/factory.py` are untouched -- Phase
11's hardware swap-in (change `config/robot.yaml`'s `mode` from `sim`
to `hardware`, add `robot/hardware_robot.py`) is exactly as valid today
as before any of this session's work. *(2026-09-28: `robot/hardware_robot.py`
now exists -- see the last section.)*

**The one real gap from earlier -- the web twin duplicating simulation
logic instead of calling it -- is now fixed.** `web-twin/index.html`
was rewritten to be a genuine HTTP client of `robot/server.py`: every
move, sensor read, and safety check goes through the real server, the
same one Phase 9 always intended to run on the Pi. Verified against an
actually-running `robot/server.py` instance (not just unit tests) --
both the manual D-pad and the autonomous "find backpack" mode drive the
real server and produce the same result as the Python simulation (76
steps to find the backpack, matching `demo_active_search.py`'s ~77-83).
CORS support was added to `robot/server.py` for this
(`config/robot.yaml`'s new `server.allowed_origins`).

*(2026-09-28: the paragraph below is history. The JS decision logic was
deleted 2026-09-25; the only brain is `control/brain_server.py`.)*

The twin's autonomous exploration *decision* logic intentionally still
lives in JavaScript, not on the server -- that's correct, not leftover
duplication: deciding what to do next is the "brain" role in the
MacBook/Pi split, and `robot/server.py` only ever played the "robot
runtime" role (movement execution, sensing, safety veto). The browser
now fills the brain role the same way `brain/agent.py`'s `MissionAgent`
does in Python -- two client implementations of the same role, which is
normal (you'll likely also want a real `brain/planner.py` Python client
eventually), not two implementations of the robot itself.

**The cloud photo-analysis feature remains additive, not a deviation**
-- it's a different feature (analyze an uploaded photo) than Phase 1's
streaming Vision LLM work, fully decoupled from the hardware phases.
Its implementation (Lambda, then ECS Fargate) is an infrastructure
choice, not a change to that boundary.

**Net assessment:** hardware integration is unaffected and still a
config change away, and the codebase no longer has two competing
implementations of the robot's behavior.

## Hardware decided, not bought (2026-09-03 and 2026-09-04)

*Superseded 2026-09-19: the board is a **Jetson Orin Nano Super** with an
IMX219 camera, not a Hailo-8L AI HAT+ with a Camera Module 3, and the build is
~$944 all-in, not ~$500. The chassis and lidar below still stand. See
`JETSON-BOM.md` and `HARDWARE-BOM.md`. Superseded again 2026-09-30: the
chassis is the Waveshare UGV Rover, with its own D500 lidar and OAK-D Lite
camera -- see the last two sections.*

Reading Microduck (`PLAN-microduck-transplants.md`) turned into a rewrite
of the hardware plan (`PLAN-onboard-perception.md`). The PiCar-X is out:
it cannot pivot in place, which the grid world always assumed and which
lidar scan matching wants. In: a differential-drive chassis with encoder
motors, an RPLidar C1 as a 360-degree clearance ring (SLAM later, behind
an HTTP wall), and -- after a three-way comparison against the IMX500 AI
Camera and a Jetson -- a Hailo-8L AI HAT+ with a Camera Module 3 for
on-board detection. About $500 of parts. Retires S6, rewrites
`HARDWARE-READINESS.md`, and gives `brain/planner.py` its job as an
event-triggered deliberation tier.

## Swapping to real hardware (Phase 11, later)

*Updated 2026-10-02.* `robot/hardware_robot.py` exists (R7, 2026-09-26;
reworked for the Rover in 3.25): a `RobotInterface` backend that drives the
Rover's ESP32 **ROS Driver** board over serial, anchoring on the board's
`odl`/`odr` odometers. To use it, set `mode: hardware` in
`config/robot.yaml` and give the robot server the board's port in
`ROBOT_SERIAL` (or `hardware.serial_port`); `robot/factory.py` refuses to
start without one. `SIM_MOTOR_BOARD=fake` runs the same backend against
`sim/fake_esp32.py` instead of a board (`SIM_BOARD_FIRMWARE=fork` fakes our
firmware fork). The Rover's board is closed loop from the factory, so it
needs no reflash to drive; flashing `firmware/ugv_base_ros/` (finer
odometers and a timestamp) comes after the arrival checks. The camera and
lidar drivers are not written yet; until they are, on the car those
readings answer "unusable", and **reverse is refused** while the robot has
wheels but no usable scan (user decision, 2026-10-02) -- turns still work.
Under the fake board, the simulated body stands in for the sensors. Before
first power-on, read `HARDWARE-READINESS.md` section 5 (pre-flight) and
`HARDWARE-BOM.md` section 5 (bring-up), then `PLAN-ros-alignment.md` 3.33
for the Jetson. Nothing in `brain/` should need to change.

## 2026-09-28 to 2026-10-02 -- the Rover, safety on the car's path, and specs

- **The robot base was chosen and ordered.** A Hiwonder ROSOrin was
  cancelled (its board sends no encoder data to the host and its firmware is
  closed); the Waveshare UGV Rover was ordered instead. Its firmware was read
  from source (3.26): it speaks JSON, not ROS, and Waveshare's Jetson nodes
  are not used because they would bypass `robot/safety.py`.
- **The sim became the Rover** (3.25, 3.27): the fake board is the Rover's
  ROS Driver firmware (660 pulses/rev, `odl`/`odr`, a lossy 20 Hz stream,
  reboots), and the URDF and safety layer take the Rover's CAD geometry,
  including the lidar 4 cm off-centre.
- **Firmware fork and settle pass** (3.28, 3.29): finer odometers and a
  board clock, and a correction pass after each verb -- 120/120 turns within
  +/-1 degree.
- **Guarded verbs and `drive: ros` gates** (3.24): direct-mode moves are
  re-vetted every 50 ms and stop at the line; the ROS verb path meets the
  same ground-truth bars; when the ROS stack dies, the mission ends and only
  a person can drive.
- **Things that move** (3.30): `SIM_MOVERS` adds people and pets that walk
  a path; across 2021 runs with a person crossing, no contact.
- **Arrival an edge cannot fake** (3.32): a lidar window that spreads across
  an edge no longer counts as arrival, and the sim's detections respect
  occlusion. It also found that in-process missions had not been using the
  guarded verbs -- fixed.
- **Specs and a spec review** (decision 0001): `docs/` now holds an
  architecture and an engineering spec for each of 15 components,
  `tools/spec_lint.py` checks them, and the review
  (`docs-review/SPEC-REVIEW.md`) led to three hardware-path fixes: a stop
  under `drive: ros` never waits on the container, `/health` names the map
  under the fake board, and no blind reverse.
- **Next** (3.33): bring up the Jetson before the Rover arrives.
