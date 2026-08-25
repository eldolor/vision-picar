# Handoff: vision-picar → Claude Code

This document exists so a new Claude Code session can pick up this
project with zero re-explanation. Read this first, then `README.md` for
full phase-by-phase build details.

---

## 1. First 5 minutes in Claude Code

```bash
# 1. Get the repo under version control
git init
git add .
git commit -m "Initial import from claude.ai session"

# 2. Set up the environment
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# 3. Confirm everything still works (should show 58 passed)
pytest tests/ -v

# 4. The Lambda handler has its own isolated test suite (packaged
#    separately on purpose -- see lambda/vision_analyze/README.md)
cd lambda/vision_analyze && pip install -r requirements.txt && pytest test_handler.py -v && cd ../..
```

If both suites pass (58 + 10 = 68 tests), the handoff is clean and you're
working from the exact state this session ended in.

**Environment variable needed for anything vision-related:**
`ANTHROPIC_API_KEY` -- required by `brain/vision.py` (only for
`tests/manual_describe_image.py`, which isn't in the automated suite
since it costs a real API call) and by `lambda/vision_analyze/` once
deployed. Nothing else in the automated test suite needs it (API calls
are mocked in tests).

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
| extra | Web-based digital twin | Done (`web-twin/index.html`), real HTTP client of `robot/server.py`, verified end-to-end against a live server |
| extra | Cloud photo-analysis endpoint | Done (`lambda/vision_analyze/`), not yet actually deployed to AWS (code is written and tested, but no live Function URL exists yet) |

---

## 4. Repo map

```
vision-picar/
├── robot/                  "Pi" role -- robot runtime, hardware-agnostic
│   ├── interface.py         RobotInterface -- the ONE abstraction brain/ depends on
│   ├── factory.py            picks sim vs. hardware backend from config/robot.yaml
│   ├── safety.py              local safety layer; can veto any action, sim or real
│   └── server.py              FastAPI Wi-Fi control API (Phase 9), CORS-enabled
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
├── lambda/vision_analyze/    AWS Lambda: photo upload -> vision analysis (cloud)
│   ├── handler.py, vision_core.py, rooms_core.py   (self-contained, own deps)
│   ├── test_handler.py        10 tests, run separately from this folder
│   └── README.md              deployment steps, cost guardrails
│
├── web-twin/index.html       Mobile-first web UI, real client of robot/server.py
├── requirements.txt
├── .gitignore
├── README.md                  full build-plan-referenced documentation
└── HANDOFF.md                 this file
```

---

## 5. Recommended next steps, in priority order

1. **Deploy `lambda/vision_analyze/`** so the web twin's "find the bag in
   a photo" feature actually works end-to-end (currently code-complete
   and tested, but has no live Function URL). Steps are in
   `lambda/vision_analyze/README.md`.

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
   Lambda endpoint's schema with a new prompt mode rather than a new
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

- **Two separate test suites, two separate dependency sets.**
  `lambda/vision_analyze/` is deliberately self-contained (its own
  `vision_core.py`/`rooms_core.py` copies, its own `requirements.txt`,
  its own test file) so it can be packaged as a minimal Lambda
  deployment zip without dragging in the whole repo. If you change the
  vision prompt or room-feature logic in `brain/vision.py` or
  `brain/rooms.py`, mirror the change in the lambda/ copies manually --
  this is a known, accepted duplication, not an oversight.

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

- **Model string used throughout:** `claude-sonnet-5` (in
  `brain/vision.py` and `lambda/vision_analyze/vision_core.py`). If a
  newer/cheaper model becomes preferable for the frequent/throttled
  calls the AR feature will need, that's a reasonable thing to
  reconsider -- see the cost discussion referenced in section 5.3.
