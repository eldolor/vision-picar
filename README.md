# vision-picar

Simulation-first build of the PiCar-X Vision Agent. See
`picar-x-build-plan.md` (in your Claude Project knowledge) for the full
phased plan.

## What's built so far (Phase 0)

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

```bash
pip install -r requirements.txt
```

## Run tests

```bash
pytest tests/test_mock_robot.py -v
```

## Run the demo loop

```bash
python -m tests.demo_manual_loop
```

## What's built so far (Phase 1)

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

## What's built so far (Phase 2)

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

## What's built so far (Phase 4)

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

## What's built so far (Phase 5 & 6)

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

## Next up (Phase 6 continued / checkpoint)

Both build-plan checkpoint demos now run end to end in simulation:
"explore without hitting anything" (Phase 2) and "find the red backpack"
(Phase 4/6, now with active scanning). Per the plan, this is the gate
before Part B (buying hardware) -- worth deciding whether to run these
against a richer/larger map first, or move straight to Phase 7 (Pi
setup) since the reasoning and safety architecture is validated.

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

## Digital twin / web-based visualization

- `web-twin/index.html` — self-contained, mobile-first web page. Opens
  directly in Safari, no server or install needed. Canvas view of the
  starter house, manual D-pad control, autonomous "Explore"/"Find
  backpack" modes (a JS port of the frontier-preference algorithm in
  `brain/agent.py` -- verified to match the Python sim's behavior
  step-for-step), and the "take a photo, find the bag" feature calling
  the Lambda endpoint below.
- **Important caveat** — this is a standalone JS re-implementation of
  `sim/grid_world.py` / `brain/agent.py`, not a client of
  `robot/server.py`. See "Is this deviating from the hardware plan?"
  below and `web-twin/README.md` for what that means going forward.

## Cloud vision endpoint (photo analysis, reachable from anywhere)

- `lambda/vision_analyze/` — an AWS Lambda Function URL wrapping the
  same `describe_image()` logic as `brain/vision.py` (adapted to accept
  photo bytes from a browser upload instead of a file path), plus a
  room guess via `identify_room()`. This is deliberately separate from
  `robot/server.py`: photo analysis benefits from being reachable from
  anywhere (cellular, not just home Wi-Fi) and having the API key live
  in the cloud instead of on a device, but the actual drive/steer/stop
  control loop stays local once hardware exists -- safety-critical
  control shouldn't depend on a cloud hop being up.
- Includes a shared-secret header check and configurable CORS, since a
  public endpoint calling a paid API needs abuse protection -- see
  `lambda/vision_analyze/README.md` for deployment steps and cost
  guardrails (reserved concurrency, billing alarms).

```bash
cd lambda/vision_analyze && pytest test_handler.py -v
```

## Is this deviating from the hardware integration plan?

**No.** `RobotInterface` + `robot/factory.py` are untouched -- Phase
11's hardware swap-in (change `config/robot.yaml`'s `mode` from `sim`
to `hardware`, add `robot/hardware_robot.py`) is exactly as valid today
as before any of this session's work.

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

The twin's autonomous exploration *decision* logic intentionally still
lives in JavaScript, not on the server -- that's correct, not leftover
duplication: deciding what to do next is the "brain" role in the
MacBook/Pi split, and `robot/server.py` only ever played the "robot
runtime" role (movement execution, sensing, safety veto). The browser
now fills the brain role the same way `brain/agent.py`'s `MissionAgent`
does in Python -- two client implementations of the same role, which is
normal (you'll likely also want a real `brain/planner.py` Python client
eventually), not two implementations of the robot itself.

**The Lambda photo-analysis feature remains additive, not a deviation**
-- it's a different feature (analyze an uploaded photo) than Phase 1's
streaming Vision LLM work, fully decoupled from the hardware phases.

**Net assessment:** hardware integration is unaffected and still a
config change away, and the codebase no longer has two competing
implementations of the robot's behavior.

## Swapping to real hardware (Phase 11, later)

Change `mode: sim` to `mode: hardware` in `config/robot.yaml`, and add
`robot/hardware_robot.py` implementing `RobotInterface` for real GPIO/PiCar-X
calls. Nothing in `brain/` should need to change.
