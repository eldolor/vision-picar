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

# Confirm everything still works (should show 419 passed)
pytest tests/ -v

# service/vision_analyze/ has its own suite -- see section 5, item 1
pytest service/vision_analyze/tests/ -v

# tests/test_ui.py drives the real twin in a real browser. It SKIPS unless
# a browser is installed, so the count above holds either way -- install it
# once to actually get that coverage:
python -m playwright install chromium
```

**`tests/test_ui.py` is the twin's only automated coverage, and it exists
because the UI was where the bugs actually escaped.** Two shipped on
2026-08-29 and were both found on a phone, not by the (then 283-strong)
Python suite: the model picker silently never populated, and it rendered
as a 40px chevron with no readable text. The second is invisible to any
DOM-only test -- it needs real layout at a phone viewport, which is why
this is Playwright rather than jsdom. Both were re-introduced deliberately
to confirm the new tests fail on them before being trusted.

`service/vision_analyze/` (the ECS Fargate vision service) is still
verified against a real deployment by manual `docker run` + curl too, not
only by its own automated suite -- see section 5, item 1 for what that
suite does and does not cover (it's app.py's own routing/validation/error
mapping, not the real Bedrock call).

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

The second constraint follows from the first: **build it, prove it in the
digital twin's UI, then put it on the PiCar** -- section 7 has the rule
and what each phase owes because of it. The abstraction above is what
makes that possible at all (the twin drives the same API the hardware
will, so the tap that works in the sim is the tap that works on the
robot); the ordering rule is what makes it actually happen.

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
| 9 (partial) | Manual WASD control client (`control/manual_control.py`) | NOT BUILT, and now skipped -- the twin's D-pad supersedes it, and `RemoteRobot` (B0, below) makes it nearly free if ever wanted. |
| B0-B3 | Brain on the wire: `RemoteRobot`, `MissionRunner`, `control/brain_server.py`, the three failsafes | Done, tested (`control/`, `tests/test_remote_robot.py`, `test_mission_runner.py`, `test_brain_server.py`, `test_failsafes.py`). The autonomy loop is a service now: startable, stoppable, and inspectable over HTTP, with the robot reachable only as an HTTP client. |
| S2b (partial) | The Python vision agent | Done for recorded walks (`brain/navigate.py`, `brain/vision_agent.py`, `sim/replay_robot.py`, `policy: "vision"`, and the twin's "Record this walk" switch) and, since T1-T4, for a live phone walk too (`sim/teleop_robot.py`, "Drive via brain"). The model decides every move; the harness supplies the timeout, the failure budget and the step/cost cap. Room-level step memory is done -- `/navigate` exchanges `searched_rooms`/`room_guess` with the client, and `brain/agent.py:MissionAgent.step()` backfills `frame["room"]` from it (`AGENT-HARNESS.md` section 10). Not yet drivable in the grid-world sim itself -- still needs S2's real image bytes. |
| B4 | The twin becomes an observer | Done (`web-twin/index.html`'s "Remote brain" panel + `control/drills.py`). Missions start from the phone and survive the tab; the failsafe drills and watchdog readout make B3's guards watchable. Only B5 (systemd on the Pi) is left in that plan. |
| T1-T4 | Teleop robot: a live phone walk drives the real `MissionRunner` mission, closed loop (`PLAN-teleop-robot.md`) | Done and deployed (2026-08-28) -- `sim/teleop_robot.py`'s `TeleopRobot` (a fourth `RobotInterface` backend: `mode: teleop`, no motor, a live pushed camera frame, no distance sensor), `POST /teleop/frame` on `robot/server.py`, the twin's Robot view "Drive via brain" switch, and sibling `teleop-robot.yaml`/`teleop-brain.yaml` CloudFormation stacks sharing the existing NLB/ALB (see section 6's AWS-topology bullet). Verified end to end: a real phone walk found its target (`OUT: FOUND`), and both B3.2 (vision-failure budget) and T1's stall detection were triggered live, no drill, against the deployed services. One rough edge, since fixed: `/frame` now catches a stall and returns 503 with the real message instead of a generic 500. |
| extra | Interim: brain on ECS Fargate | Done and deployed (`service/brain/`, `cloudformation/brain.yaml`) -- `control/brain_server.py` alongside the twin and vision-analyze on the same shared NLB/ALB, so the remote-brain panel works from a phone off the home LAN with no HTTPS tunnel. Not a build-plan phase and not B5: the brain's real home is still the Pi: see `PLAN-brain-relocation.md`'s "Interim: brain on ECS Fargate" for why this doesn't conflict with that, and for the one real gap it surfaced (separate outbound secrets for the robot vs. the vision service). |
| extra | Recorded-walk evaluation harness | Done and deployed (2026-08-30) -- `control/walk_eval.py` scores a walk (metrics + an LLM judge + a collision check, advisory and kept out of the operator's own label), `control/walk_replay.py` re-asks its frames under another model or prompt, `/navigate` takes `model_id` and `prompt_variant` from server-side allow-lists, and the console shows a per-model summary. **This is the instrument the Stage 0 gate needed:** it turns "did that walk go well" from an afternoon of reading JSON into a button, and replay is the only controlled model comparison available -- two live walks vary the operator's path as well as the model. What it has already established is in the Stage 0 notes below. |
| extra | Recorded-walk storage + admin viewer | Done and deployed (`cloudformation/recordings.yaml`, `service/admin/`, `control/admin_server.py`) -- an EFS volume (survives redeploys, unlike Fargate's own filesystem) holding Robot-view "Record this walk" data, plus a separate `/admin` service to list/view/delete it. Deliberately its own service, not more routes on `brain_server.py`: reviewing recordings has no reason to move to the Pi when B5 lands or to go down when the mission server restarts. See `PLAN-brain-relocation.md`'s Interim section and `control/admin_server.py`'s docstring. Since T1-T4, `teleop-brain.yaml`'s brain has no EFS mount of its own and instead proxies `POST /recording/frame` to the main brain (`control/brain_server.py`'s `recording_proxy_url`) -- see `PLAN-teleop-robot.md`'s "Recording proxy" section for why only that one route, never `/mission/*`, may be proxied between brains. |
| -- | LLM-driven planner (`brain/planner.py`) replacing rule-based `decide()` | NOT BUILT. Designed but never written to disk -- a `PlannerAgent` calling Claude with `MissionMemory.as_context()` as the prompt. **This is now the main hardware-path gap:** `PLAN-sim-hardening.md` Q1 settled that the robot is vision-driven, and the vision loop currently exists only in JavaScript (`web-twin/index.html`'s Vision Autopilot) -- no Python file calls `/navigate`. Phase S2b of that plan specifies the port, including the step-memory problem the browser version does not solve. Real gap if you want the actual "high-level planner" from the architecture diagram rather than the current rule-based frontier-exploration policy. Stage 2's `MissionRunner` is where it plugs in -- `AGENT-HARNESS.md` section 10 is the instruction sheet: it takes a `vision_fn` and already enforces the timeout and failure budget such a policy needs, and `control/brain_server.py` answers `policy: "vision"` with a 501 until it exists. |
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
│   ├── navigate.py            the vision policy's perception step: a frame ->
│   │                           the cloud /navigate route -> one action (S2b)
│   ├── vision_agent.py        VisionAgent -- trusts the model's action; the
│   │                           policy that IS on the hardware path
│   └── planner.py             NOT YET BUILT -- room-level planning over
│                               MissionMemory.as_context(); see gap table above
│
├── sim/                     grid-world simulator (Phase 0.5)
│   ├── grid_world.py
│   ├── mock_robot.py          implements RobotInterface against grid_world
│   ├── replay_robot.py        a body made of photographs -- plays a recorded
│   │                           Robot-view walk back, one frame per move
│   │                           (open loop -- see its own docstring)
│   ├── teleop_robot.py        a body made of a live phone camera and a
│   │                           human -- TeleopRobot, mode: teleop
│   │                           (PLAN-teleop-robot.md). Closed loop, unlike
│   │                           replay_robot.py: the next frame really is
│   │                           whatever the person photographs after
│   │                           reading the model's decision. No motor, no
│   │                           distance sensor -- honest no-ops, same
│   │                           pattern as replay_robot.py's.
│   ├── sensors.py              DistanceSensorModel -- Gaussian noise, dropout,
│   │                           real-range clamping for MockRobot.get_distance(),
│   │                           opt-in via config/robot.yaml's sim.sensor_noise
│   │                           (Phase S5)
│   └── maps/starter_house.py  living room / hallway / kitchen + red backpack
│
├── control/                 the brain as a service (phases B0-B3). Imports no
│   │                        backend and no simulator -- the robot is only ever
│   │                        an HTTP client target
│   ├── remote_robot.py       RemoteRobot -- RobotInterface over HTTP (B0)
│   ├── mission_runner.py     one mission's lifecycle: start/tick/stop/status,
│   │                          plus failsafe B3.2 (vision timeout + budget)
│   ├── brain_server.py       FastAPI on :8001; drives the runner as a
│   │                          background task, plus failsafe B3.3 (hung tick)
│   ├── brain_config.py       the `brain:` block of config/robot.yaml
│   ├── drills.py             fault injection, so the failsafes can be shown
│   │                           from the twin and not only asserted in tests
│   ├── walk_eval.py          scores a recorded walk: deterministic metrics
│   │                           (degenerate / stalled / oscillating /
│   │                           unstable-identity), an LLM judge over sampled
│   │                           frames, and a collision check over the frames
│   │                           that commanded FORWARD. Advisory -- written to
│   │                           its own eval.json, never the operator's label
│   ├── walk_replay.py        re-asks a walk's frames under another model or
│   │                           prompt. The ONLY controlled comparison this
│   │                           project has: two live walks vary the
│   │                           operator's path as well as the model
│   ├── admin.html/.js        the recorded-walk console (see admin_server.py)
│
├── config/robot.yaml         mode (sim/hardware), safety thresholds, CORS origins,
│                            and the `brain:` block (robot_url, failsafe budgets)
│
├── tests/                    419 tests, 99% line coverage of brain/,
│                              control/, robot/ and sim/ (incl. test_robot_contract.py's
│                              backend-agnostic conformance suite [S1],
│                              test_sensors.py [S5],
│                              test_watchdog_integration.py [S4],
│                              test_walk_eval.py + test_admin_server.py
│                              (the recorded-walk scorecard and replay),
│                              test_ui.py + test_ui_admin.py (Playwright,
│                              real browser at a phone viewport), and
│                              test_alb_routes.py -- see section 6)
│                              + 7 runnable (non-automated) demo scripts
│
├── service/vision_analyze/   ECS Fargate: photo upload -> vision analysis (cloud)
│   ├── app.py                 FastAPI app -- /health, /analyze, /describe,
│   │                           /navigate, /guidance
│   ├── vision_core.py         calls Amazon Bedrock (Claude, Converse API)
│   ├── rooms_core.py          identify_room() -- same logic as brain/rooms.py
│   ├── tests/                 app.py's own suite (25 tests) -- routing,
│   │                           validation, decode/size/error handling, all
│   │                           vision_core.* calls mocked. Run separately:
│   │                           `pytest service/vision_analyze/tests/ -v`
│   │                           (see section 6 for why it's not swept into
│   │                           the top-level `tests/` package)
│   └── requirements.txt, Dockerfile
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
├── web-twin/index.html       Mobile-first web UI, real client of robot/server.py
│                              AND of control/brain_server.py (the remote-brain
│                              panel, phase B4).
│                              Deployed via service/twin/ (see above) as well as
│                              usable locally (`uvicorn robot.server:app`)
├── requirements.txt
├── .gitignore
├── README.md                  full build-plan-referenced documentation
├── CLAUDE.md                  this file -- session orientation
├── FEATURES.md                 every UI feature (all four tabs), how each
│                               one works end-to-end, and the AWS topology
│                               it runs against -- start here for "how does
│                               X work" questions about the app itself
│
│   -- planning / explainer docs (no code; read before hardware work) --
├── INTRODUCTION.md            project introduction
├── AGENT-HARNESS.md           how control/ works: the tick, the seams, the
│                               failsafes, the invariants, and where the LLM
│                               policy plugs in (BUILT -- read before editing
│                               control/)
├── PLAN-ar-guidance.md        the Guide tab: spec, redesign, changelog (BUILT)
├── PLAN-sim-hardening.md      how the sim diverges from hardware, phased fixes,
│                               definition of done before a hardware swap
│                               (S1 and S3 BUILT, S2/S2b partial, S4-S7 PROPOSED)
├── HARDWARE-READINESS.md      what the PiCar-X kit changes: verb-to-motor path,
│                               pre-flight checklist, where the brain should live
├── PLAN-brain-relocation.md   moving the autonomy loop onto the Pi (B0-B4 BUILT,
│                               B5 needs the Pi)
└── PLAN-teleop-robot.md       a live phone walk driving the real MissionRunner
                                mission, closed loop -- T1-T4 (BUILT); see the
                                T1-T4 status-table row above
```

---

## 5. Build order -- everything left before hardware

Sequenced across the three plan documents, which hold the detail. Phase
IDs are `S*` = `PLAN-sim-hardening.md`, `B*` = `PLAN-brain-relocation.md`.

**None of stages 0-5 needs the PiCar-X.** Each stage is independently
useful, so stopping at the end of any of them leaves the project in a
coherent state.

**Every stage also has to be verifiable from the twin** -- see section 7,
which is a standing requirement on each phase below, not a nice-to-have.
A phase is not done when its tests pass; it is done when someone holding
a phone can watch the thing it built do its job.

### Stage 0 -- Validate the premise

**Primary tool: the Guide tab's "Robot view" mode.** Point your phone at
a real room and it shows the move the robot would make from where you are
standing -- same `/navigate` route Vision Autopilot uses, real pixels
instead of the raycaster render. Nothing executes. Walk toward the target
and see whether following the actions would actually get you there; that
tests the *loop*, which curated stills cannot.

Hold the phone low, around 10cm -- the PiCar-X camera height. A
chest-height view is not the robot's view.

Robot view pauses when the service reports `target_reached`, so a walk
ends at arrival instead of burning calls. **That field needs an ECS
redeploy of `service/vision_analyze/` to take effect** -- against the
currently deployed service the field is simply absent, the pause never
fires, and everything else behaves as before. The 120-call cap (~3.3 min
at the 500ms cadence) bounds the cost either way.

**`/navigate` runs Claude Opus 4.5 as of 2026-08-29, and the previous
claim on this line was wrong in a way worth knowing about.** It used to
say `/navigate` moved to Nova Lite on 2026-08-28. The *code* default in
`service/vision_analyze/vision_core.py` did say Nova Lite -- but
`cloudformation/service.yaml`'s `NavigateModelId` parameter has always
passed a value, and the env var wins. Every walk recorded before
2026-08-29 was actually produced by **Claude Sonnet 4.5**, and was
diagnosed for a while as if it were Nova. Two lessons, both now enforced
in code: keep the code default and the template parameter in step (each
file says so), and trust a walk's own recorded `model_id` -- echoed on
every `/navigate` reply since the model picker shipped -- over any prose.

Opus 4.5 was chosen by measurement: all 22 frames of walk
`red-backpack-20260829-195904` replayed through every invokable vision
model on the account, same pixels and prompt. See `vision_core.py`'s
"Per-route models" note for the result. `/guidance` remains on Nova Lite.

**What the gate has actually shown, as of 2026-08-30** -- run it yourself
before trusting any of it, but this is where it stands:

- Walks now reach the target, which they never did before. Claude Opus 4.5
  and Qwen3-VL both arrive in 6-14 frames; Sonnet 4.5 stalls (one FORWARD in
  22 frames, turning on the spot with the target centred) and Nova Lite
  wanders without arriving.
- **The failure is obstacle routing, not object recognition.** Every model
  identifies a red backpack; the ones that fail refuse to close distance over
  open floor because "is there an obstacle directly ahead" reads as "is there
  furniture anywhere in front of me".
- **`obstacle_ahead` is not calibrated and should not be trusted.** On the
  same frames Opus reports it on ~100% and Qwen on ~0%. On a frame that is
  nothing but a wall, Opus and Sonnet turn away; Qwen and Nova drive into it.
  On hardware the ultrasonic is the real obstacle sensor -- do not let the
  vision policy be the thing relying on this field.
- **The prompt is at least as strong a lever as the model, and the obvious
  fix is wrong.** Rewording the obstacle question to be about the next step
  takes Sonnet from 0% FORWARD to 100% FORWARD on the same frames -- which is
  the *other* degenerate failure. Three wordings now ship (`default`,
  `next-step-obstacle`, `next-step-and-walls`).
- **The full 3x3 was finally run on 2026-08-30, and the third wording does
  not work.** All 22 frames of `red-backpack-20260829-195904`, every model x
  every variant, replayed at full coverage. FORWARD rate:

  |                   | default | next-step-obstacle | next-step-and-walls |
  |---|---|---|---|
  | Claude Opus 4.5   | 0.591 | 0.318 | 0.455 |
  | Claude Sonnet 4.5 | 0.000 | 1.000 | 0.955 |
  | Qwen3-VL          | 0.773 | 1.000 | 1.000 |

  `next-step-and-walls` was written to keep the next-step framing while
  restoring "a surface filling the frame is a stop condition". It buys one
  frame in 22 on Sonnet and nothing on Qwen -- still the always-FORWARD
  degenerate mode, still colliding. **Do not promote it.** `default` is the
  only column that avoids that mode on all three models, which is the
  evidence for leaving it as the default. No cell reached the target, and
  every cell except Sonnet/`default` was flagged `collision`. Next attempt
  should change the *shape* of the question -- the single-step /navigate
  contract has no memory of which way it already turned -- not its wording.
- **These nine numbers replaced nine that were wrong, and the way they were
  wrong is the cautionary tale.** The same matrix had been run before and
  reported 33/33/33 across the variants, which reads as "the wording makes
  no difference". It was really "the vision service timed out": those cells
  completed 3, 9 and 2 of 22 frames. `control/admin_server.py`'s retry
  branched on a status code while httpx *raises* a timeout, so the backoff
  never ran on the failure that dominated, and the scorer graded whatever
  came back. Fixed 2026-08-30 (`replay_timeout_s`, a retry that catches
  `httpx.TransportError`, and `REPLAY_MIN_COVERAGE`, below which a replay is
  stored but deliberately left unscored). **A replay's `coverage` field is
  now the first thing to read**: a score computed over a third of a walk is
  not a weaker measurement, it is a different one.

Secondary: `python -m tests.manual_replay_navigate <dir> "<target>"`
replays a folder of photos and prints an action-spread summary. Use it to
produce a recordable finding -- the failure that matters (the same action
for every frame) is easier to see in a summary than by walking around.

This is first because it is nearly free and it is a **go/no-go gate**.
Every "the simulation works" result so far is a statement about
flat-shaded raycaster frames, not about rooms
(`PLAN-sim-hardening.md` 3.5). If the model cannot navigate from real
photographs, stages 1-5 are premature and the work is prompt engineering
instead. Record what you find; it is the only evidence available about
real-world accuracy without a robot.

### Stage 1 -- Make the vision path real, in Python -- **PARTLY DONE**

The vision loop is the product (Q1). It now exists in Python
(`brain/navigate.py` + `brain/vision_agent.py`, `policy: "vision"`) and
runs against **recorded walks** -- `sim/replay_robot.py`, fed by the
twin's new "Record this walk" switch in Robot view. What is left is the
sim path (S2) and room memory (the rest of S2b).

- **S1 -- pin the contract -- BUILT.** `tests/test_robot_contract.py`:
  a backend-agnostic conformance suite (36 tests) parameterized over all
  four `RobotInterface` backends that exist today (`MockRobot`,
  `RemoteRobot`, `ReplayRobot`, `TeleopRobot`), asserting return shapes,
  units and `stop()` idempotency with no grid-specific assertions. Not yet
  pinned: pixels in `get_camera_frame()` -- that's S2's job, noted in the
  suite's own docstring as the thing to extend it with once real image
  bytes exist on every backend.
- **S2 -- real image bytes.** Port the twin's raycaster
  (`renderFPV`) into Python so `get_camera_frame()` returns JPEG bytes on
  every backend. The one structural blocker between Vision Autopilot and
  hardware.
- **S2b -- the Python vision agent** -- **BUILT** for recorded walks
  (`python -m tests.demo_replay_mission <walk> "red backpack"`) and, since
  `PLAN-teleop-robot.md`'s T1-T4, for a live phone walk too. **Room-level
  step memory is built**: `/navigate` now exchanges `searched_rooms`
  (client -> server, from `MissionMemory.searched_rooms`) and `room_guess`
  (server -> client, backfilled into `frame["room"]`) -- see
  `AGENT-HARNESS.md` section 10 for the exact mechanism, which
  deliberately doesn't touch the `vision_fn(frame) -> scene` contract.
- **`service/vision_analyze/app.py`'s test suite -- BUILT.**
  `service/vision_analyze/tests/` (25 tests, FastAPI `TestClient`, every
  `vision_core.*`/`identify_room` call mocked) -- closes the one real gap
  left over from the Lambda -> ECS migration. Run separately from the
  root suite: `pytest service/vision_analyze/tests/ -v` (see section 6).

**Done when** a Python agent completes a backpack hunt in the sim
against the real `/navigate`, with cost and wall-clock recorded. The
agent and the cost/wall-clock reporting exist
(`tests/demo_replay_mission.py` prints both); *in the sim* is what S2 is
still owed for.

### Stage 2 -- Put the brain on the wire -- **DONE (2026-08-27)**

Built out of order, ahead of Stage 1: B0-B3 need neither hardware nor the
Python vision policy, and they are what make Stage 1's agent something a
robot can run rather than a script someone babysits.

- **B0 (= S3) -- `RemoteRobot`** (`control/remote_robot.py`). Proof, as
  specified: identical action sequences in-process vs. over a live
  `uvicorn` (`tests/test_remote_robot.py`), 83 steps either way.
- **B1 -- `MissionRunner`** (`control/mission_runner.py`) --
  `start()`/`tick()`/`stop()`/`status()`. No decision logic moved; a
  tick-driven mission matches `run_mission()`'s step count exactly.
- **B2 -- `control/brain_server.py`** on :8001, driving the runner as an
  asyncio background task.
- **B3 -- the three failsafes**, all tested (`tests/test_failsafes.py`).

**Done when** a mission runs over HTTP with the same outcome as
in-process, and a stubbed vision failure ends it with the robot stopped.
Both hold. `python -m tests.demo_brain_over_http` shows the first
directly (three ways, same 83 steps, identical actions).

Two things worth knowing before extending it:

- **`policy: "vision"` answers 501 on purpose.** The runner takes a
  `vision_fn` and already enforces S2b's per-call timeout and failure
  budget around it, so Stage 1's agent plugs in there -- but until it
  exists, a "vision" mission must not quietly run the rule-based policy.
- **A stop is enforced at the robot, not just in the loop.** The agent
  drives through a gate that refuses movement once a mission ends, so a
  tick already in flight when `POST /mission/stop` lands cannot get its
  move out. Blocking calls on another thread cannot be interrupted; the
  gate is what makes "stop stops the car" true anyway.

### Stage 3 -- Make the sim honest about safety -- **DONE (2026-08-28)**

- **S4 -- put time in the loop.** Done. `config/robot.yaml`'s
  `sim.realtime` (default `false`) makes `MockRobot._settle(duration)`
  actually `time.sleep(duration)`; `robot/server.py` gained a
  `ROBOT_CONFIG_PATH` env var so `tests/test_watchdog_integration.py` can
  run a real `uvicorn` subprocess against a temp config with a short
  `watchdog_timeout_s` -- the watchdog's async loop is executed by a test
  now, against real wall-clock time on a real event loop.
- **S5 -- sensor realism.** Done, cone geometry deferred (see
  `PLAN-sim-hardening.md`'s S5 section for why). `sim/sensors.py`'s
  `DistanceSensorModel` adds Gaussian noise, dropout, and real 2-400cm
  range clamping behind `config/robot.yaml`'s `sim.sensor_noise.enabled`
  (default `false`, so `get_distance()`'s old exact-multiple-of-30
  formula is untouched until opted in). Dropout reads as `0.0cm` --
  always trips the safety veto, a deliberate fail-safe choice documented
  in that module.

**Done when** changing `min_distance_cm` measurably changes behavior, and
sensor dropout has defined fail-safe behavior. Both true now --
`tests/test_sensors.py::test_min_distance_cm_is_load_bearing_at_a_non_multiple_of_30`
is the direct proof of the first.

### Stage 4 -- Move the console -- **DONE (2026-08-27)**

Pulled forward, immediately after Stage 2, on the principle in section 7:
Stage 2 was otherwise a stage you could only verify by reading a test
file.

- **B4 -- the twin becomes an observer.** Built as two labelled panels in
  the Sim tab -- "Remote brain" (drives `control/brain_server.py`) and
  "Local brain" (the JS loop, kept for LAN dev and for running with no Pi
  present). Only one may drive at a time, enforced from both ends. The
  manual D-pad still talks straight to the robot server and works with
  the brain service stopped.
- **Built alongside it, because B3 was otherwise unverifiable by hand:**
  a failsafe drill picker (`control/drills.py`) and a watchdog readout.

**Done when** you start a mission from your phone, background the tab,
and the robot keeps going. That single observation is the proof the brain
actually moved. Holds with two local uvicorns; still owed against a Pi,
which is B5.

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
  better, and B0's `RemoteRobot` (built) makes it nearly free if ever
  wanted.
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

- **`service/vision_analyze/` has its own test suite now**
  (`service/vision_analyze/tests/`, see section 5 item 1) -- the Lambda
  version's `test_handler.py` didn't carry over since it was shaped around
  Lambda's `handler(event, context)` signature, not a FastAPI app, so this
  is a fresh suite built against `app.py` directly (FastAPI `TestClient`,
  every `vision_core.*` call mocked). It's deliberately **not** under the
  top-level `tests/` package -- both directories are named `tests`, so a
  bare `pytest` from the repo root (not this project's documented
  invocation, which is always `pytest tests/ -q`) would hit a module-name
  collision without `service/__init__.py` and
  `service/vision_analyze/__init__.py` disambiguating the two; run it with
  `pytest service/vision_analyze/tests/ -v`. It covers app.py's own
  routing, request validation, and error mapping -- not the real Bedrock
  call, which is still verified manually: local `docker run` + curl, then
  the same against the deployed NLB endpoint, both with a real Bedrock
  call and a correct response.

- **The web twin's exploration algorithm duplication is intentional,
  not a bug -- and the browser is now the *optional* brain, not the
  primary one.** `web-twin/index.html`'s JS re-implements the
  frontier-preference decision logic from `brain/agent.py`. That was
  always legitimate (the browser plays the "brain" role over HTTP, the
  same way a Python client would); only robot *runtime* logic (movement,
  safety, sensing) was wrong to duplicate, and that has been server-side
  since the twin became a real client of `robot/server.py`.
  **Updated 2026-08-27, when B4 landed:** the Sim tab now has two brains
  side by side -- "Remote brain" driving `control/brain_server.py`, and
  "Local brain" running the JS loop in the tab. The remote one is the
  arrangement the finished robot uses; the local one is kept because it
  needs no brain service, which makes it the only thing that works with
  no Pi present and the fastest path for LAN development. Only one may
  drive at a time, enforced in both directions (the twin refuses to start
  a local loop during a remote mission; the brain server answers a second
  `/mission/start` with a 409).

- **`robot/server.py`'s watchdog** stops the robot if no command arrives
  within `watchdog_timeout_s` (config, default 1.0s). The decision logic
  (`watchdog_should_stop`) is unit-tested; the actual async polling loop
  is only exercised by running the server for real (see `README.md`).
  It is failsafe **B3.1** of three, and the other two live in `control/`
  and cover different failures -- B3.2 (`MissionRunner`: vision call
  timeout + consecutive-failure budget) and B3.3 (`brain_server`: a tick
  that never returns). Don't collapse them: the watchdog cannot see a
  brain that is alive but stuck, and neither brain-side guard can see
  motors energised by a call that then crashed.

- **`AGENT-HARNESS.md` is the reference for `control/`** -- one tick in
  order, the four seams, the concurrency model (which thread runs what
  and why), the status contract, and a numbered list of invariants not to
  break. Read it before changing the mission loop; the bullets here cover
  only what a session needs to avoid breaking something by accident.

- **The brain and the robot are two processes on purpose**, even when
  both run on the Pi. Running the loop inside `robot/server.py` would be
  less code and would defeat the watchdog (a synchronous block in the
  agent loop blocks the event loop the watchdog polls on), merge the two
  roles the project has kept apart since Phase 0, and lose the base-URL
  trick that makes brain-on-Pi vs. brain-on-MacBook a config change.
  `PLAN-brain-relocation.md`'s "Why not one process" has the full
  argument. The cost is a localhost round trip per call, against a loop
  that spends seconds waiting on vision.

- **`control/` may not import a backend, the simulator, or
  `robot/server.py`** -- only `robot/interface.py` (plus the
  `SafetyViolation` type that is part of that contract) and its own
  `RemoteRobot`. `tests/test_brain_server.py` asserts this by importing
  the brain in a subprocess and inspecting `sys.modules`. It is the same
  constraint as section 2's, one level up: if the brain can reach a
  backend directly, "run the brain on the Pi" stops being a config
  change.

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

- **A public route needs an ALB path pattern, or it 404s.** Both public
  services share one listener and are routed by EXACT path patterns (see the
  NLB/ALB bullet above). Adding a route to a FastAPI app is therefore only
  half the job -- and the failure is quiet, because the page still loads and
  one feature is silently dead. It has shipped five times (`/app.js`,
  `/admin.js`, `/recording/finish`, `/recording/summary`,
  `/teleop-robot/app.js`). `tests/test_alb_routes.py` now walks each app's
  real routes against its template's real patterns and fails if they drift;
  deliberate exceptions are listed there with their reason. A ListenerRule
  condition allows at most 5 path values, so a sixth route means a second
  rule, as `twin.yaml`, `teleop-brain.yaml` and `admin.yaml` all now do.

- **Coverage is 99% of `brain/`, `control/`, `robot/` and `sim/`, and the
  last 1% is deliberate.** Measure it with:

  ```bash
  pytest tests/ --cov=brain --cov=control --cov=robot --cov=sim --cov-report=term-missing
  ```

  Two things are left uncovered on purpose. `robot/server.py`'s watchdog
  loop body is exercised by `tests/test_watchdog_integration.py` against a
  live `uvicorn` **subprocess**, which coverage cannot instrument -- it is
  tested, just not visibly. And `control/brain_server.py`'s second walk-name
  check is unreachable defensive code behind a regex that already validated
  the name; deleting a safety check to make a number go up would be a poor
  trade. `brain/agent.py` sits at 92% because the rule-based agent is
  explicitly not on the hardware path (`PLAN-sim-hardening.md` 2.2) -- keep
  it, don't extend it, and don't chase its branches.

  Chasing the number is not the point, but the exercise paid for itself
  twice: it found `MissionRunner.tick()` carrying an unreachable duplicate of
  the step-budget check (now removed -- a second copy of a rule can drift
  from the real one), and it found that DELETE on a walk, the download zip,
  and the whole outbound `/navigate` retry path had no tests at all.

- **The twin and the admin console have UI tests, and they earned them.**
  Every UI bug in this project so far has been found on a phone rather than
  by the Python suite -- a model picker that rendered empty, one that
  rendered 40px wide, an admin console that could only be used sideways.
  `tests/test_ui.py` and `tests/test_ui_admin.py` drive the real pages in a
  real browser at a 390px viewport. They skip (not fail) without
  `playwright install chromium`. When adding one, re-introduce the bug and
  watch it go red first: two tests written in this project passed against
  the very defect they were written for until that was checked.

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


---

## 7. Twin first, then the PiCar

The project's ordering rule, stated by the user 2026-08-27 and binding on
everything below:

> **Build it, prove it in the digital twin's UI, and only then put it on
> the PiCar.** A phase is not done when its tests pass. It is done when
> someone holding a phone can watch the thing it built do its job.

Nothing gets built for the car that cannot first be watched working in the
twin. This is not a preference about documentation -- it decides what
"finished" means, and therefore what each phase has to ship.

Three reasons this is a rule and not a preference:

1. **The twin is the only surface that survives the hardware swap.** It
   speaks to `robot/server.py`'s real control API, not a mock of it, so
   the tap that starts a mission in the sim is the same tap that will
   start one on the Pi -- only `mode: hardware` and a base URL differ. A
   pytest run proves something about `MockRobot`; that tap proves it about
   the robot. Every phase that lands with a UI affordance lands with its
   own bring-up checklist for the day the hardware arrives.
2. **Tests are written by whoever wrote the code.** They encode what the
   author expected. Watching a mission cross a real room is the only
   check that survives being wrong about that -- Stage 0 exists for
   exactly this reason.
3. **The failures that matter cannot be provoked by pressing anything.**
   That is not a reason to leave them unverifiable; it is the reason
   `control/drills.py` exists.

### The rules

- **Every phase ships with something to press.** Name it in the phase's
  plan entry, next to its test.
- **Where the behavior is a failure nobody can trigger on purpose, ship a
  drill.** Fault injection that breaks exactly one thing and leaves every
  other guard standing, so what gets watched is the real guard firing.
  Drills must be fail-safe by construction -- a drill may only ever end
  with the robot stopped -- and switchable off (`brain.allow_drills`).
- **Where a phase genuinely changes nothing observable, say so, and name
  the readout that would show it if it broke.** "No UI change" is an
  acceptable answer exactly once per phase, in writing.

### What you can verify today

Setup for all of it: `uvicorn robot.server:app --port 8000` and
`uvicorn control.brain_server:app --port 8001`, then open
`http://127.0.0.1:8000/` and connect both in Settings. Restart the robot
server to put the robot back at its start position -- there is no reset
endpoint, on purpose (real hardware has none either).

| Stage | Sub-stage | Press this | You should see |
|---|---|---|---|
| 0 | Validate the premise | Guide tab -> Robot view, phone at ~10cm | The move the robot would make from where you stand; pauses on arrival |
| 2 | B0 `RemoteRobot` | Sim tab -> D-pad, then Remote brain -> Start | Both drive the same robot through the same server; the map follows either one |
| 2 | B1 `MissionRunner` | Remote brain -> Start | Step count, last action, rooms searched and a log tail advancing ~4 steps/second |
| 2 | B2 the brain service | Start a mission, then close the tab and reopen it | The mission is further along, or finished. It never needed the page |
| 2 | B3.1 watchdog | D-pad forward, then stop touching it | "Robot watchdog" counts the silence past the timeout and reports the motors stopped |
| 2 | B3.2 AWS link dead | Drill picker -> vision errors / vision hangs | Three failures counted, mission ends `failed`, robot stopped |
| 2 | B3.3 brain loop hung | Drill picker -> brain loop hangs | One step, then `failed` -- "brain loop hung"; the watchdog readout stays quiet, which is the point |
| 2 | stop stops the car | Start a mission, then Stop | Mission ends `stopped`, the map stops moving, watchdog goes quiet |
| 2 | one brain at a time | Start a remote mission, then tap Explore | Refused with a toast; the reverse is the server's 409 |
| S4 | time in the loop | Set `sim.realtime: true` in `config/robot.yaml`, restart the robot server, then Remote brain -> Start | The watchdog readout climbs mid-move instead of only between moves -- a move now genuinely occupies its duration, off by default so this is opt-in |
| S5 | sensor realism | Set `sim.sensor_noise.enabled: true`, restart the robot server, then D-pad toward a wall | Distance telemetry stops being multiples of 30cm and jitters; the safety collar can flash before you're touching the wall, not only once you are |

### What the remaining phases owe

Each of these is a line in that phase's plan entry, to be built with the
phase rather than bolted on after:

| Phase | UI proof it has to ship with |
|---|---|
| S1 pin the contract | **Suite built, button not.** A **"Check robot contract"** button in Settings: run the interface's methods against whatever robot is connected and report which returned the wrong shape. `tests/test_robot_contract.py` is the suite itself, runnable from a terminal against any backend today; the Settings button that makes it pressable from the twin is still owed |
| S2 real JPEG frames | The FPV canvas shows the **server-rendered** frame, with a "frame source: server / local" readout. The picture should not change; where it comes from should |
| S2b Python vision agent | The remote-brain panel's policy picker stops answering **501**. Run a mission with `policy: "vision"` and watch Claude's own reasoning in the log instead of "free space clear" |
| S6 motion realism | The map shows the robot **arcing** rather than pivoting in place, and refusing a turn that will not fit the corridor |
| S7 chaos and soak | New drills: added latency, dropped requests, a killed link mid-mission. Same picker, same fail-safe rule |
| B5 deployment | Reboot the Pi. Open the twin on a phone. Start a mission with no laptop on the network at all -- this is definition-of-done item 1, and it is a UI test by construction |
