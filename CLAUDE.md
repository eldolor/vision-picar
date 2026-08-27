# vision-picar

Orientation for a Claude Code session. Claude Code loads this file
automatically; `README.md` has the full phase-by-phase build details.

Originally written as `HANDOFF.md`, to carry the project over from a
Claude Project into Claude Code. Renamed 2026-08-27 -- the migration is
long done, but the orientation content it accumulated is permanent.

---

## 1. Setup

To pick up from a clean checkout:

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# Confirm everything still works (should show 61 passed)
pytest tests/ -v
```

`service/vision_analyze/` (the ECS Fargate vision service) has no
automated test suite of its own yet -- see section 5, item 1. It's
verified working by manual `docker run` + curl, and against the live
deployment, not by an automated suite the way `lambda/vision_analyze/`
used to be before it was decommissioned.

**Environment variable needed for anything vision-related in the sim:**
`ANTHROPIC_API_KEY` -- required by `brain/vision.py` (only for
`tests/manual_describe_image.py`, which isn't in the automated suite
since it costs a real API call). Nothing else in the automated test
suite needs it (API calls are mocked in tests). `service/vision_analyze/`
does *not* need this -- it authenticates to Amazon Bedrock via its ECS
task role's IAM permissions, not an API key.

---

## 2. What this project is

An indoor autonomous robot: PiCar-X chassis + Raspberry Pi 5 (robot
runtime) + a MacBook running a Vision LLM (high-level reasoning).
(That MacBook placement dates from before vision moved to Bedrock, and is
re-examined in `HARDWARE-READINESS.md` section 7 -- the brain no longer
computes anything. Nothing in the code has changed yet.)
**Development approach is simulation-first**: all decision-making logic
is built and validated against a grid-world simulator before any
hardware is purchased. See `README.md`'s "Final Architecture" section
for the full diagram and reasoning.

The single most important design constraint, and the one thing not to
break while extending this: **`brain/` and `robot/server.py` only ever
talk to `robot/interface.py`'s `RobotInterface` abstraction, never to
`sim/mock_robot.py` directly.** `robot/factory.py` is the only place
that picks a backend, based on `config/robot.yaml`'s `mode` field. This
is what makes the eventual hardware swap-in (Phase 11) a config change
instead of a rewrite -- do not introduce a new code path that imports
`sim.mock_robot` directly from `brain/`.

---

## 3. Current status (what's actually built vs. what's still planned)

Referencing the phase numbering in `README.md` (which itself mirrors
the original build plan phases, reordered simulation-first):

| Phase | What | Status |
|---|---|---|
| 0 | Mock robot interface (`robot/interface.py`, `sim/mock_robot.py`) | Done, tested |
| 0.5 | Simulator tier chosen (grid-world) | Done (`sim/grid_world.py`) |
| 1 | Vision LLM scene understanding | Done (`brain/vision.py`) |
| 2 | Constrained action loop | Done (`brain/agent.py:ConstrainedAgent`) |
| 3 | Local safety layer | Done, built early (`robot/safety.py`) |
| 4 | Agent harness / mission memory | Done (`brain/memory.py`, `MissionAgent`) |
| 5 | Semantic navigation & room recognition | Done (`brain/rooms.py`, `target_room` in `MissionMemory`) |
| 6 | Object search (active scanning) | Done (`ObjectSearchAgent`) |
| -- | Simulation checkpoint | Done -- both demo scenarios pass reliably |
| 9 (partial) | Wi-Fi control API + safety-over-HTTP + watchdog | Done (`robot/server.py`), CORS added |
| 9 (partial) | Manual WASD control client (`control/manual_control.py`) | NOT BUILT -- `control/` dir exists but is empty. Was planned, then deprioritized in favor of the web twin, which now supersedes this use case (see section 5). |
| -- | LLM-driven planner (`brain/planner.py`) replacing rule-based `decide()` | NOT BUILT. Designed but never written to disk -- a `PlannerAgent` calling Claude with `MissionMemory.as_context()` as the prompt. **This is now the main hardware-path gap:** `PLAN-sim-hardening.md` Q1 settled that the robot is vision-driven, and the vision loop currently exists only in JavaScript (`web-twin/index.html`'s Vision Autopilot) -- no Python file calls `/navigate`. Phase S2b of that plan specifies the port, including the step-memory problem the browser version does not solve. Real gap if you want the actual "high-level planner" from the architecture diagram rather than the current rule-based frontier-exploration policy. |
| 7, 8, 10, 11 | Pi setup, physical assembly, real camera streaming, hardware swap-in | Blocked on buying hardware -- by design, per the simulation-first plan. Nothing to do here yet. |
| extra | Web-based digital twin | Done and deployed (`web-twin/index.html` + `robot/server.py` on ECS Fargate, `service/twin/`, `cloudformation/twin.yaml`) -- reachable from a phone on any network, sharing the vision service's NLB/ALB on port 80 via path-based routing (a ListenerRule matching the twin's exact route set). Verified end-to-end from an actual phone on cellular data, not just curl. |
| extra | Cloud photo-analysis endpoint | Done and deployed (`service/vision_analyze/` on ECS Fargate, behind an NLB -> internal ALB, calling Amazon Bedrock for vision inference). Was originally built on Lambda + API Gateway; both were deleted after an account-level restriction made them permanently unreachable publicly -- see README.md's "History: why not Lambda?" |

---

## 4. Repo map

```
vision-picar/
├── robot/                  "Pi" role -- robot runtime, hardware-agnostic
│   ├── interface.py         RobotInterface -- the ONE abstraction brain/ depends on
│   ├── factory.py            picks sim vs. hardware backend from config/robot.yaml
│   ├── safety.py              local safety layer; can veto any action, sim or real
│   └── server.py              FastAPI Wi-Fi control API (Phase 9), CORS-enabled,
│                               require_secret() gate once deployed publicly,
│                               serves web-twin/index.html at GET /
│
├── brain/                  reasoning, hardware-agnostic. (Labelled the "MacBook"
│                            role by the original build plan -- that placement is
│                            being revisited: see PLAN-brain-relocation.md)
│   ├── vision.py             Vision LLM scene understanding (Claude API)
│   ├── agent.py               ConstrainedAgent / MissionAgent / ObjectSearchAgent
│   ├── memory.py              MissionMemory -- mission, rooms, sightings, actions
│   ├── rooms.py                identify_room() -- landmark-feature room matching
│   └── planner.py             NOT YET BUILT -- see gap table above
│
├── sim/                     grid-world simulator (Phase 0.5)
│   ├── grid_world.py
│   ├── mock_robot.py          implements RobotInterface against grid_world
│   └── maps/starter_house.py  living room / hallway / kitchen + red backpack
│
├── control/                 EMPTY today. PLAN-brain-relocation.md fills it:
│                            remote_robot.py, mission_runner.py, brain_server.py
│
├── config/robot.yaml         mode (sim/hardware), safety thresholds, CORS origins
│
├── tests/                    61 tests + 5 runnable (non-automated) demo scripts
│
├── service/vision_analyze/   ECS Fargate: photo upload -> vision analysis (cloud)
│   ├── app.py                 FastAPI app -- /health, /analyze
│   ├── vision_core.py         calls Amazon Bedrock (Claude, Converse API)
│   ├── rooms_core.py          identify_room() -- same logic as brain/rooms.py
│   ├── requirements.txt, Dockerfile
│   └── NOTE: no automated test suite yet -- see section 6
│
├── service/twin/              ECS Fargate: robot/server.py + web-twin/index.html
│   ├── Dockerfile              built from the REPO ROOT (needs real robot/, sim/,
│   │                           config/ -- not dependency-light copies)
│   └── requirements.txt
│
├── cloudformation/            IaC for both ECS Fargate services
│   ├── network.yaml            VPC, 2 AZs, no NAT -- VPC endpoints instead
│   ├── service.yaml            vision service: ECR, ECS, NLB, ALB, IAM, secret
│   └── twin.yaml               twin service: ECR, ECS, IAM, secret -- reuses
│                               service.yaml's NLB/ALB on the same port 80
│                               (path-based ListenerRule) rather than
│                               provisioning a second pair or a second port
│
├── web-twin/index.html       Mobile-first web UI, real client of robot/server.py.
│                              Deployed via service/twin/ (see above) as well as
│                              usable locally (`uvicorn robot.server:app`)
├── requirements.txt
├── .gitignore
├── README.md                  full build-plan-referenced documentation
├── CLAUDE.md                  this file -- session orientation
│
│   -- planning / explainer docs (no code; read before hardware work) --
├── INTRODUCTION.md            project introduction
├── PLAN-ar-guidance.md        the Guide tab: spec, redesign, changelog (BUILT)
├── PLAN-sim-hardening.md      how the sim diverges from hardware, phased fixes,
│                               definition of done before a hardware swap (PROPOSED)
├── HARDWARE-READINESS.md      what the PiCar-X kit changes: verb-to-motor path,
│                               pre-flight checklist, where the brain should live
└── PLAN-brain-relocation.md   moving the autonomy loop onto the Pi (PROPOSED)
```

---

## 5. Build order -- everything left before hardware

Sequenced across the three plan documents, which hold the detail. Phase
IDs are `S*` = `PLAN-sim-hardening.md`, `B*` = `PLAN-brain-relocation.md`.

**None of stages 0-5 needs the PiCar-X.** Each stage is independently
useful, so stopping at the end of any of them leaves the project in a
coherent state.

### Stage 0 -- Validate the premise (no code)

Photograph real rooms with a phone and replay the JPEGs through the
deployed `/navigate`. Check whether the returned actions are sane.

This is first because it is nearly free and it is a **go/no-go gate**.
Every "the simulation works" result so far is a statement about
flat-shaded raycaster frames, not about rooms
(`PLAN-sim-hardening.md` 3.5). If the model cannot navigate from real
photographs, stages 1-5 are premature and the work is prompt engineering
instead. Record what you find; it is the only evidence available about
real-world accuracy without a robot.

### Stage 1 -- Make the vision path real, in Python

The vision loop is the product (Q1) and today it exists **only in
JavaScript**. This stage is the largest and most important block of work
remaining.

- **S1 -- pin the contract.** Document `RobotInterface`'s return shapes
  and units; add a backend-agnostic conformance suite. Cheap, no behavior
  change, and everything below gets checked against it.
- **S2 -- real image bytes.** Port the twin's raycaster
  (`renderFPV`) into Python so `get_camera_frame()` returns JPEG bytes on
  every backend. The one structural blocker between Vision Autopilot and
  hardware.
- **S2b -- the Python vision agent**, plus room-level step memory in the
  `/navigate` prompt. Nothing currently stops the vision loop revisiting
  a searched room except the step cap.
- **Fold in: a test suite for `service/vision_analyze/app.py`.** Still
  the one real gap from the Lambda -> ECS migration (FastAPI
  `TestClient`, mocking `vision_core.*`). S2b changes that service's
  prompt and schema anyway, so write the tests while you are in there.

**Done when** a Python agent completes a backpack hunt in the sim
against the real `/navigate`, with cost and wall-clock recorded.

### Stage 2 -- Put the brain on the wire

- **B0 (= S3) -- `RemoteRobot`**, an HTTP client implementing
  `RobotInterface`. Proof: identical action sequences in-process vs. over
  HTTP.
- **B1 -- `MissionRunner`**, turning `run_mission()`'s blocking loop
  inside out into `start()`/`stop()`/`tick()`/`status()`.
- **B2 -- `control/brain_server.py`** on :8001.
- **B3 -- the three failsafes**: motors-left-running (existing watchdog,
  kept), AWS-link-dead (new), brain-loop-hung (new).

**Done when** a mission runs over HTTP with the same outcome as
in-process, and a stubbed vision failure ends it with the robot stopped.

### Stage 3 -- Make the sim honest about safety

- **S4 -- put time in the loop.** `MockRobot._settle()` is a no-op, which
  is why the watchdog's async loop has never been executed by a test.
- **S5 -- sensor realism.** Until distances stop being multiples of 30cm,
  `min_distance_cm: 20` only ever triggers at 0 and is provably
  load-bearing on nothing.

**Done when** changing `min_distance_cm` measurably changes behavior, and
sensor dropout has defined fail-safe behavior.

### Stage 4 -- Move the console

- **B4 -- the twin becomes an observer.** Explore/Find POST to the brain
  and render polled status; the manual D-pad still talks straight to the
  robot server.

**Done when** you start a mission from your phone, background the tab,
and the robot keeps going. That single observation is the proof the brain
actually moved.

### Stage 5 -- Harden

- **S7 -- chaos and soak.** Latency, packet loss, a killed link
  mid-mission, a 1000-step run.

### Then buy

Check `HARDWARE-READINESS.md` section 5's pre-flight items first -- in
particular 5.2 (`LEFT`/`RIGHT` skip the distance check, correct for a
pivot and wrong for an arc) and 5.3 (verify whether the ultrasonic pans
with the camera; if it does not, the peek-based logic needs redesign).

Then: `robot/hardware_robot.py`, B5 (systemd units), and the calibration
items in `PLAN-sim-hardening.md` section 7 that can only be measured.

### Not on the critical path

- **S6 (Ackermann turns, continuous pose, scaled map)** -- optional since
  Q1. A vision policy does not reason about grid cells. Revisit only if
  real-world runs fail in ways that trace back to grid geometry.
- **`control/manual_control.py`** -- skip. The twin's D-pad covers it
  better, and B0's `RemoteRobot` makes it nearly free later if wanted.
- **The rule-based agent** -- keep, do not extend. It is the fastest,
  free, deterministic way to test the safety layer and mission memory.
  It is not on the hardware path (`PLAN-sim-hardening.md` 2.2).

---

## 6. Things to know before touching the code

- **`service/vision_analyze/` has its own dependency-light copies of
  `vision_core.py`/`rooms_core.py`, deliberately.** Same pattern the
  Lambda version used, kept for the same reason (self-contained Docker
  build context, no need to drag in the whole repo). If you change the
  vision prompt or room-feature logic in `brain/vision.py` or
  `brain/rooms.py`, mirror the change here manually -- known, accepted
  duplication, not an oversight. Note the model call itself is
  *different* here, not just the copy: `service/vision_analyze/vision_core.py`
  calls Amazon Bedrock's Converse API, not the direct Anthropic API
  `brain/vision.py` uses -- see that file's docstring for why.

- **`service/vision_analyze/` has no automated test suite yet** (see
  section 5, item 1) -- the Lambda version's `test_handler.py` doesn't
  carry over since it's shaped around Lambda's `handler(event, context)`
  signature, not a FastAPI app. Verified manually instead: local
  `docker run` + curl, then the same against the deployed NLB endpoint,
  both with a real Bedrock call and a correct response.

- **The web twin's exploration algorithm duplication is intentional,
  not a bug.** `web-twin/index.html`'s JS re-implements the
  frontier-preference decision logic from `brain/agent.py`. This is
  correct: the browser is playing the "brain" role over HTTP, the same
  way a Python client would. Only robot *runtime* logic (movement,
  safety, sensing) was wrong to duplicate, and that's been fixed -- the
  twin now calls `robot/server.py` for all of that.
  **Revisited 2026-08-27:** the reasoning above is still correct, but
  `PLAN-brain-relocation.md` proposes moving the primary autonomy loop
  onto the Pi, which would demote the browser from *the* brain to *an
  optional* brain (kept for LAN dev and for running with no Pi present).
  Nothing has changed in the code yet; if phase B4 of that plan is built,
  update this bullet rather than leaving the two documents in conflict.

- **`robot/server.py`'s watchdog** stops the robot if no command arrives
  within `watchdog_timeout_s` (config, default 1.0s). The decision logic
  (`watchdog_should_stop`) is unit-tested; the actual async polling loop
  is only exercised by running the server for real (see `README.md`).

- **Safety is enforced server-side, always.** Both the sim agents
  (`brain/agent.py`) and the Wi-Fi API (`robot/server.py`) route every
  movement action through `robot/safety.py`'s `SafetyController`. Don't
  add a new movement path that bypasses it.

- **`vision-picar-twin` (the digital twin) and `vision-picar-service`
  (the vision endpoint) share one NLB and one internal ALB, on the same
  port 80**, routed by path via a `ListenerRule` on the ALB's shared
  listener (the twin's exact route set -- `/`, `/action`, `/stop`,
  `/distance`, `/frame` -- forwards to its target group; everything
  else, including `/health`, falls through to vision-analyze's target
  group) -- rather than each having its own pair, or a second port,
  which would have cost roughly as much as everything else in this
  project combined. `cloudformation/twin.yaml` imports the other
  stack's load balancer/listener ARNs and security group ID via
  `Fn::ImportValue`; it does *not* own those resources. If you ever add
  a third public service, follow the same pattern (new `ListenerRule`
  with its own exclusive path set, new target group, on the existing
  NLB/ALB/listener) rather than defaulting to new load balancers or
  ports. A target group's own health checks bypass listener routing
  entirely (they hit the target IP:port directly), so leaving `/health`
  out of a ListenerRule's paths doesn't affect that service's health
  checks -- only the public reachability of that literal path.

- **`robot/server.py`'s `require_secret()` gate is new** (added
  alongside the ECS deployment) and is a no-op when `APP_SHARED_SECRET`
  is unset -- which is how local dev and the test suite both run, so
  existing usage patterns are unaffected. It only matters once this
  server is reachable from the public internet. `/health` and `/`
  (which now also serves `web-twin/index.html`) are deliberately left
  unauthenticated -- the ALB health check can't send custom headers,
  and a user has to be able to load the page before they can enter the
  secret into it.

- **Model strings differ between the sim and the cloud service, on
  purpose.** `brain/vision.py` (direct Anthropic API) uses
  `claude-sonnet-5`. `service/vision_analyze/vision_core.py` (Amazon
  Bedrock) uses the `us.anthropic.claude-sonnet-4-5-20250929-v1:0`
  inference profile instead -- `claude-sonnet-5` isn't enabled for
  Bedrock on this account yet (confirmed via a real `converse` call
  returning `AccessDeniedException`; Sonnet 4.5 was confirmed working).
  Check `aws bedrock list-foundation-models` / `list-inference-profiles`
  before assuming a given model ID works on Bedrock -- don't guess a
  model string here. If a newer/cheaper model becomes preferable for the
  frequent/throttled calls the AR feature will need, that's a reasonable
  thing to reconsider -- see the cost discussion referenced in section 5.3.
