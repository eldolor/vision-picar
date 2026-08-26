# Handoff: vision-picar → Claude Code

This document exists so a new Claude Code session can pick up this
project with zero re-explanation. Read this first, then `README.md` for
full phase-by-phase build details.

---

## 1. First 5 minutes in Claude Code

Git is already initialized (see `git log`). To pick up from a clean checkout:

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# Confirm everything still works (should show 58 passed)
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
| -- | LLM-driven planner (`brain/planner.py`) replacing rule-based `decide()` | NOT BUILT. Discussed and partially designed in conversation (a `PlannerAgent` calling Claude with `MissionMemory.as_context()` as the prompt) but never written to disk. Real gap if you want the actual "high-level planner" from the architecture diagram rather than the current rule-based frontier-exploration policy. |
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
├── brain/                  "MacBook" role -- reasoning, hardware-agnostic
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
├── control/                 EMPTY -- manual_control.py was planned, not built
│
├── config/robot.yaml         mode (sim/hardware), safety thresholds, CORS origins
│
├── tests/                    58 tests + 5 runnable (non-automated) demo scripts
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
└── HANDOFF.md                 this file
```

---

## 5. Recommended next steps, in priority order

1. **Write a test suite for `service/vision_analyze/app.py`.** The
   Lambda version had one (`test_handler.py`, 10 tests) but its
   Lambda-event-shaped fixtures don't carry over to a FastAPI app --
   use FastAPI's `TestClient` instead, mocking `vision_core.describe_image_bytes`
   the same way the old suite mocked the Anthropic client. This is the
   one real gap left by the Lambda -> ECS Fargate migration.

2. **Build `brain/planner.py`** if you want the real LLM-driven decision
   loop rather than the current rule-based frontier exploration. This
   was discussed and partially designed but never implemented --
   picking it up: a `PlannerAgent(MissionAgent)` whose `decide()` calls
   Claude with `MissionMemory.as_context()` as the prompt, parses the
   response into one of `ALLOWED_ACTIONS`, and falls back to
   `super().decide()` on a bad/unparseable response or API failure
   (same pattern as `brain/vision.py`'s error handling).

3. **The AR-guidance feature** (discussed but not started): user points
   the phone camera at a room, app throttles vision analysis calls on a
   timer, and overlays a directional indicator on the live camera feed
   pointing toward a named object. Design constraints already agreed on
   in conversation: AR-style overlay (not just text), timer-throttled
   analysis (not per-frame, for both cost and latency reasons -- see the
   cost breakdown in conversation history / README), reuses the same
   vision service's schema with a new prompt mode rather than a new
   function.

4. **`control/manual_control.py`** -- probably skip. The web twin's
   manual D-pad now covers this use case better (visual, mobile-friendly,
   already built and verified). Only worth building if you specifically
   want a terminal-based WASD controller for some reason (e.g. scripting,
   no browser available).

5. When ready to buy hardware (Phase 7+): the simulation checkpoint has
   already passed (both required demos work reliably -- see
   `tests/demo_explore.py` and `tests/demo_active_search.py`). Buy the
   hardware list in `README.md`'s "Buy" section, then work through
   Phases 7-11 in order. `robot/hardware_robot.py` is the only new file
   required to implement `RobotInterface` against real GPIO/PiCar-X
   calls -- everything in `brain/` needs zero changes.

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
