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

- `web-twin/index.html` — mobile-first web page: canvas view of the
  starter house, manual D-pad control, autonomous "Explore"/"Find
  backpack" modes (a JS port of the frontier-preference algorithm in
  `brain/agent.py` -- verified to match the Python sim's behavior
  step-for-step), and the "take a photo, find the bag" feature calling
  the cloud vision service below. It's a real HTTP client of
  `robot/server.py` -- every move, sensor read, and safety check goes
  through the actual server, not a duplicated simulation (see "Is this
  deviating from the hardware plan?" below for how that was verified).
- **Deployed to ECS Fargate** (`service/twin/`, `cloudformation/twin.yaml`)
  so it's reachable from a phone on any network, not just a Mac's LAN --
  same cluster as the vision service, sharing its NLB and internal ALB
  on a second port (8000) instead of provisioning a second pair of load
  balancers. Public URL: `http://<the vision service's NLB DNS
  name>:8000/` (same DNS name as the `/analyze` endpoint below, just a
  different port). `robot/server.py`'s `/action`, `/stop`, `/distance`,
  `/frame` routes require an `x-app-secret` header once deployed
  publicly (see `require_secret()` in that file) -- the secret lives in
  Secrets Manager as `vision-picar-twin-shared-secret`. `/health` and
  `/` (which now serves `web-twin/index.html` directly) stay open, since
  the ALB health check can't send custom headers and the page has to
  load before anyone can enter the secret.
- When served by `robot/server.py` itself (this deployment, or local
  `uvicorn robot.server:app`), the twin's "Robot server connection"
  field auto-fills to its own origin -- no more typing LAN IPs. Manual
  entry still works for pointing at a different server.
- Also still runs the original way for local hardware-adjacent dev:
  `uvicorn robot.server:app --host 0.0.0.0` on a Mac, LAN IP in the
  connection field, no secret needed (unset `APP_SHARED_SECRET` makes
  `require_secret()` a no-op) -- see `web-twin/README.md`.

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
- Runs as an **ECS Fargate** service (not Lambda -- see history note
  below) behind an internet-facing **NLB → internal ALB → ECS Fargate**
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
into that specific AWS account was silently rejected before the
function ever ran. Root cause: this account's Lambda concurrency quota
was pinned at 10 instead of AWS's normal default of 1000, with no
history of anyone requesting that reduction -- i.e. AWS had placed the
account in some reduced-trust tier that blocked Lambda-based public
ingress specifically. ECS Fargate behind a load balancer is a
completely different invocation path (long-running container, not a
Lambda-invoke permission), so it isn't subject to whatever that
restriction was. The Lambda code and its API Gateway have been deleted;
this is documented here rather than left to be rediscovered from git
history, since it explains a real architectural choice, not just
"we changed our minds."

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

**The cloud photo-analysis feature remains additive, not a deviation**
-- it's a different feature (analyze an uploaded photo) than Phase 1's
streaming Vision LLM work, fully decoupled from the hardware phases.
Its implementation (Lambda, then ECS Fargate) is an infrastructure
choice, not a change to that boundary.

**Net assessment:** hardware integration is unaffected and still a
config change away, and the codebase no longer has two competing
implementations of the robot's behavior.

## Swapping to real hardware (Phase 11, later)

Change `mode: sim` to `mode: hardware` in `config/robot.yaml`, and add
`robot/hardware_robot.py` implementing `RobotInterface` for real GPIO/PiCar-X
calls. Nothing in `brain/` should need to change.
