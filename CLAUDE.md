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

# Confirm everything still works (should show 928 passed, with a browser
# installed -- see below; fewer without, as the parity and UI tests skip)
pytest tests/ -v

# service/vision_analyze/ has its own suite -- see section 5, item 1
pytest service/vision_analyze/tests/ -v

# tests/test_ui.py and tests/test_renderer_parity.py drive the real twin in
# a real browser. They SKIP rather than fail without one, so the rest of
# the suite still runs -- install it once to actually get that coverage:
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

An indoor autonomous robot: a differential-drive chassis + Raspberry Pi 5
(robot runtime, and since B0-B4 the autonomy loop too) + a vision LLM on
Amazon Bedrock (high-level reasoning). It began as a PiCar-X plus a
MacBook-hosted brain; both halves of that changed before any hardware was
bought -- the brain moved to the Pi in design (`HARDWARE-READINESS.md`
section 7) and the chassis, obstacle sensor and detector were re-decided in
`PLAN-onboard-perception.md` (2026-09-03/04).
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
digital twin's UI, then put it on the car** -- section 7 has the rule
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
| S2 | Real image bytes -- the twin's raycaster ported into Python (`sim/renderer.py`) | Done (2026-08-31). `MockRobot.get_camera_frame()` returns a rendered JPEG like every other backend, so the vision policy drives the simulator and `RobotInterface` no longer has to change when hardware lands (`PLAN-sim-hardening.md` 2.1, now closed). Parity with the JS is proven column by column (`tests/test_renderer_parity.py`), which is what makes `renderFPV` safe to delete -- **not yet deleted**: it is still the fallback for a server predating S2, and nothing has been redeployed. The twin's new "frame source: server / local" readout is how you tell which one drew the picture. Read `sim/renderer.py`'s fidelity note before treating a sim result as a statement about real rooms. |
| S2b (partial) | The Python vision agent | Done for recorded walks (`brain/navigate.py`, `brain/vision_agent.py`, `sim/replay_robot.py`, `policy: "vision"`, and the twin's "Record this walk" switch) and, since T1-T4, for a live phone walk too (`sim/teleop_robot.py`, "Drive via brain"). The model decides every move; the harness supplies the timeout, the failure budget and the step/cost cap. Room-level step memory is done -- `/navigate` exchanges `searched_rooms`/`room_guess` with the client, and `brain/agent.py:MissionAgent.step()` backfills `frame["room"]` from it (`AGENT-HARNESS.md` section 10). Since S2 it drives the grid-world sim too, and since M1 it is startable from the twin: the Remote brain panel has a policy picker, and a vision mission carries a `model_id` and a `prompt_variant` that are validated at start and shown before you spend anything. |
| B4 | The twin becomes an observer | Done (`web-twin/index.html`'s "Remote brain" panel + `control/drills.py`). Missions start from the phone and survive the tab; the failsafe drills and watchdog readout make B3's guards watchable. Only B5 (systemd on the Pi) is left in that plan. |
| T1-T4 | Teleop robot: a live phone walk drives the real `MissionRunner` mission, closed loop (`PLAN-teleop-robot.md`) | Done and deployed (2026-08-28) -- `sim/teleop_robot.py`'s `TeleopRobot` (a fourth `RobotInterface` backend: `mode: teleop`, no motor, a live pushed camera frame, no distance sensor), `POST /teleop/frame` on `robot/server.py`, the twin's Robot view "Drive via brain" switch, and sibling `teleop-robot.yaml`/`teleop-brain.yaml` CloudFormation stacks sharing the existing NLB/ALB (see section 6's AWS-topology bullet). Verified end to end: a real phone walk found its target (`OUT: FOUND`), and both B3.2 (vision-failure budget) and T1's stall detection were triggered live, no drill, against the deployed services. One rough edge, since fixed: `/frame` now catches a stall and returns 503 with the real message instead of a generic 500. |
| extra | Interim: brain on ECS Fargate | Done and deployed (`service/brain/`, `cloudformation/brain.yaml`) -- `control/brain_server.py` alongside the twin and vision-analyze on the same shared NLB/ALB, so the remote-brain panel works from a phone off the home LAN with no HTTPS tunnel. Not a build-plan phase and not B5: the brain's real home is still the Pi: see `PLAN-brain-relocation.md`'s "Interim: brain on ECS Fargate" for why this doesn't conflict with that, and for the one real gap it surfaced (separate outbound secrets for the robot vs. the vision service). |
| extra | Recorded-walk evaluation harness | Done and deployed (2026-08-30) -- `control/walk_eval.py` scores a walk (metrics + an LLM judge + a collision check, advisory and kept out of the operator's own label), `control/walk_replay.py` re-asks its frames under another model or prompt, `/navigate` takes `model_id` and `prompt_variant` from server-side allow-lists, and the console shows a per-model summary. **This is the instrument the Stage 0 gate needed:** it turns "did that walk go well" from an afternoon of reading JSON into a button, and replay is the only controlled model comparison available -- two live walks vary the operator's path as well as the model. What it has already established is in the Stage 0 notes below. |
| extra | Recorded-walk storage + admin viewer | Done and deployed (`cloudformation/recordings.yaml`, `service/admin/`, `control/admin_server.py`) -- an EFS volume (survives redeploys, unlike Fargate's own filesystem) holding Robot-view "Record this walk" data, plus a separate `/admin` service to list/view/delete it. Deliberately its own service, not more routes on `brain_server.py`: reviewing recordings has no reason to move to the Pi when B5 lands or to go down when the mission server restarts. See `PLAN-brain-relocation.md`'s Interim section and `control/admin_server.py`'s docstring. Since T1-T4, `teleop-brain.yaml`'s brain has no EFS mount of its own and instead proxies `POST /recording/frame` to the main brain (`control/brain_server.py`'s `recording_proxy_url`) -- see `PLAN-teleop-robot.md`'s "Recording proxy" section for why only that one route, never `/mission/*`, may be proxied between brains. |
| -- | LLM-driven planner (`brain/planner.py`) replacing rule-based `decide()` | NOT BUILT. Designed but never written to disk -- a `PlannerAgent` calling Claude with `MissionMemory.as_context()` as the prompt. **This is now the main hardware-path gap:** `PLAN-sim-hardening.md` Q1 settled that the robot is vision-driven, and the vision loop's *port* is done (`brain/navigate.py` calls `/navigate`; the JS Vision Autopilot is now the optional one). Phase S2b of that plan specifies the port, including the step-memory problem the browser version does not solve. Real gap if you want the actual "high-level planner" from the architecture diagram rather than the current rule-based frontier-exploration policy. Stage 2's `MissionRunner` is where it plugs in -- `AGENT-HARNESS.md` section 10 is the instruction sheet: it takes a `vision_fn` and already enforces the timeout and failure budget such a policy needs, and `control/brain_server.py` serves `policy: "vision"` with `brain/vision_agent.py` today -- what is still missing is room-level *planning* over `MissionMemory`, not the vision loop. |
| M2 | A depth grid on `RobotInterface`, and in the sim | Done (2026-09-03), not deployed -- `get_depth_grid()` with an honest all-unusable default, `MockRobot` synthesising it from `renderer.cast_ray()`, `GET /depth`, `RemoteRobot` over it, a depth strip under the twin's FPV canvas, and `_HaltGate` added to the conformance suite as a fifth backend. Nothing in `brain/` reads it yet: M3 is the consumer. See `PLAN-microduck-transplants.md`. |
| M3 | The tri-state zone, and a centre-zone veto | Done (2026-09-03), not deployed -- `SafetyController.path_clearance()` reduces the middle half of the grid's columns to one number and compares it to `min_distance_cm`, exactly as it compared `get_distance()` before. A failed zone never enters the comparison in either direction; a wholly blind path falls back to the scalar and keeps its `0.0`-on-dropout stop. `GET /depth` publishes the reduction so the twin never recomputes it. |
| M4 | Refusals are state, manual preempts autonomous | Done (2026-09-03), not deployed -- `robot/server.py` arbitrates `/action` by a decided order (`stop > twin-dpad > brain > twin-local-brain`, `AGENT-HARNESS.md` 4.1) instead of letting the last writer win, every refusal carries a machine-readable `reason`, `RemoteRobot` raises `Preempted` rather than `SafetyViolation`, and a preempted mission ends `preempted` with the robot stopped. |
| M5 | One health command | Done (2026-09-03), not deployed -- `python -m control.health` (and a Settings health line) asks both halves and exits non-zero when either is unhealthy or unreachable. Verdict inputs are reachability, the robot watchdog loop's own poll freshness, and a running mission's tick liveness; everything else is description and never changes the exit code. Both servers now log an identity line at start-up. |
| -- | On-car perception + the hardware chain (`PLAN-onboard-perception.md`) | **DESIGN SETTLED 2026-09-03, DETECTOR REVISED 2026-09-04, NOTHING BUILT.** Started as "what could run on the car itself" after reading Microduck and ended up rewriting the hardware plan. Decided: a **differential-drive chassis** rather than the PiCar-X's Ackermann (which **retires S6** and makes `grid_world.py`'s pivot assumption correct); a **lidar** used first as a 360-degree clearance ring and only later as SLAM behind an HTTP wall; a **Hailo-8L in M.2 module form, with a Camera Module 3** for on-board detection (the AI HAT+ until 4.9 settled on the module, 2026-09-06) -- chosen on 2026-09-04 over the IMX500 AI Camera (its nano-only ceiling is silicon, and it cannot be fed a recorded frame) and over a Jetson (the right board for arbitrary Hugging Face models, ruled out for now on cost, power and the camera stack; its section 4 has the three-way comparison and the conditions for re-opening it); and a **tiered architecture** where the VLM becomes an event-triggered deliberation tier -- which is what finally gives `brain/planner.py` a job. Also settles the goal vocabulary, stop conditions, arbitration and what the sim can test. **Revised 2026-09-06 on three counts** (its 4.8, 4.9 and 1.14): the Jetson was re-checked against what delivery-robot fleets actually run and against a July 2026 NVIDIA repricing that put the Orin Nano Super at $399-480, so Pi-plus-Hailo stands more firmly than before; the part is now a **Hailo-8L in M.2 module form**, because the module survives a Jetson pivot and shares the one PCIe lane with the NVMe, and because the 10H's measured 5.89 tok/s makes a local VLM slower than the cloud call it would replace; and **motion becomes continuous rather than discrete** (1.14); a fourth revision the same day settled the models rather than the parts (4.3.1 the 8L's measured benchmarks, 4.2 the open-vocabulary crop path and the standing-height caveat, 2.8 one mission walked end to end), which makes the tiered architecture mandatory instead of an optimisation, puts the accelerator on the first order, and makes two shipped numbers wrong -- `watchdog_timeout_s: 1.0` and the fixed `min_distance_cm: 20.0`, which is a stopping distance good for only ~0.45 m/s. Its C1-C9 phasing (extended from five 2026-09-06, after walking 2.8's mission against the repo) needs no hardware. **A second series, P1-P4 (its 4.10), is the perception harness: P1 and P2 are BUILT (`brain/perceive.py`, `brain/tiered.py`) and run the real YOLO + CLIP + Opus 4.5 chain against real photographs with no robot and no accelerator -- the twin cannot test the *detector*, by 1.12's design, but a phone on a wheeled rig can. **P2 became startable from the twin on 2026-09-07** (`policy: "tiered"`, and 6.3's four readouts on the Remote brain panel) -- see the row below.** Bill of materials **~$555-620** (3.6, recomputed 2026-09-06 -- the earlier ~$498/~$581 priced the bundled motor driver rather than the recommended Waveshare board, bought an M.2 module with no carrier, and had no servo rail). **Read it before buying anything**, and note its section 5: `HARDWARE-READINESS.md` is now partly wrong. The one thing it asks for *before* hardware day is the Hailo compile loop (its 1.10 item 1): without it the Hailo is a fixed-function part and the IMX500 was cheaper. |
| -- | **The first valid Stage 0 walk** (`blue-bottle-20260907-142454`) | Recorded 2026-09-07 -- 33 frames, camera at floor height on a wheeled rig, target **on the floor**. The re-recording 1.16 #10 has demanded since 2026-09-02, and the first walk not disqualified by its own viewpoint. Two findings, both in `PLAN-onboard-perception.md` 4.10: **(a)** the CLIP threshold is measured -- true positives band +0.025..+0.038, non-target frames top out at +0.004, so **0.02 separates them perfectly and the shipped 0.05 detects none of them** (`brain.perception_match_margin`, now 0.02 in config; the module default is deliberately unchanged because the old corpus had handbags at +0.039 against "red backpack", so one threshold does not serve both targets); **(b) 4.2's label gate is losing 11 of 18 true positives** -- at close range YOLO relabels the bottle as a `vase` (once `refrigerator`), so the gate discards exactly the frames where the target fills the view. Forcing the open-vocabulary path recovers 18/18 with zero false positives. `brain.perception_crop_path` makes that measurable; the default stays `auto` until two more walks say otherwise. The full chain ran on it: `found`, **6 paid calls over 31 frames, 1 per 5.17**. |
| P2 (twin) | `policy: "tiered"`, and the readouts that make the architecture watchable -- on the Sim tab **and on the phone walk**, which is the one that matters | Done (2026-09-07), not deployed -- the brain has been deployed nowhere since 2026-09-05 and these models run *in the brain process*, so this is a local two-uvicorn feature by construction. The Remote brain panel's policy picker gains **Tiered**; `control/brain_server.py` wraps `brain/navigate.py`'s cloud `vision_fn` in `brain/tiered.py`'s `TieredVision` and **validates at mission start** -- `ultralytics`/`torch` stay an optional install (`requirements-perception.txt`) and a missing one is a 400 naming the pip command, never a B3.2 vision failure discovered three ticks in. `GET /health` publishes `perception_available` so the panel warns before Start. Four readouts (`PLAN-onboard-perception.md` 6.3): the tri-state, the CLIP **margin** (not the similarity), the detector and encoder by name, and the deliberation counter as **calls and frames** -- 6.3's "single number that makes the whole architecture watchable", comparable to 6.1's measured 4-6x. The mission log names the paid steps `[cloud: <trigger>]`. The detector's *boxes* are deliberately absent: over the twin's FPV they would be boxes on a raycaster render, which 1.12 forbids. **Guide -> Robot view -> "Drive via brain" also carries the policy now** (it hardcoded `policy: "vision"` before), which is the only path where YOLO and CLIP get real pixels -- the Sim tab's tiered mission exercises the loop and never the detector, by 1.12's design. **Run end to end the same day, YOLO -> CLIP -> Opus 4.5** (`python -m tests.demo_replay_mission <walk> "<target>" --policy tiered`, which is new): on `red-backpack-opus-4-5-20260829-214849`, outcome `found`, **4 paid calls over 18 steps -- 1 per 3.5 distinct frames**, all three implementable triggers fired, nothing tuned. That puts 2.4's cost claim inside 6.1's measured 4-6x band *live* for the first time. Three findings in `PLAN-onboard-perception.md` 4.10: `DEFAULT_MATCH_MARGIN` (0.05) is ~2x too high and **has not been changed** -- the corpus is the invalid one and the negative column overlaps on handbags; **the target STRING is a bigger lever than the walk** (`"red backpack"` 13% detected, `"blue bottle"` 0%, bare `"bottle"` 0% -- a colour+noun is worth ~5x the margin of the bare noun); and two defects the run surfaced, both fixed -- `bearing_deg` had never once been a number (no backend publishes `image_width`; the width is now read off the image) and a detected target logged as `not_visible`. |
| P3 | Corpus-wide perception scoring (`control/perception_eval.py`), and 1.11a **reported** | Done (2026-09-08), not deployed. **P3** is the instrument every finding in `PLAN-onboard-perception.md` 4.10/4.11 rests on and it had been written ad hoc three times and lost each time: it reads each walk's adjudicated `labels.json` (**never** `walk.jsonl`), refuses a walk that has none, scores once and sweeps the gate afterwards, and matches two configs on a false-positive budget before reporting either one's recall. It reproduced 4.11's shipped row exactly on first run -- 62/74 at `P>=0.8`, 3 false positives -- which is the only validation a scorer can have. Its new per-walk split is the finding: **100%/97% recall on the two approach walks and 20% on the search walk**, so the corpus-wide 84% is an average over two different problems. **1.11a** (corroborated identity) is now computed, counted and shown on the Remote brain panel and carried in walk data -- and **enforces nothing**, which a test pins: an `unclear` frame passes `target_visible` and `target_reached` through untouched. It stays that way until the two out-of-vocabulary searches 1.11a asks for exist. |
| P6 | The Hailo compile loop (`tools/hailo/`), run against OWLv2 | **RUN 2026-09-09. OWLv2 does NOT compile to a Hailo-8L HEF, and the part decision resolves against the Hailo.** `PLAN-onboard-perception.md` 1.10 item 1 had asked for this loop since 2026-09-04 and it was the one pre-hardware item never started; it answered a $400 question for **$3.20 in 3.1 hours** on an r6i.4xlarge, now terminated. **What it found is not what Hailo's own table predicts.** OWLv2's ViT-B/16 image tower *translates* (44-107s) and *quantizes* (no OOM at 3600 tokens or 1600) -- DFC 3.34 carries a LayerNorm Decomposition pass, Matmul Equalization and MatmulDecompose and uses all three. It fails at **allocation**. Three attempts died on `conv1`, the single Conv in a 575-node graph (the 16x16-stride-16 patch embedding), unmoved by `--image-size 640` or `allocator_param(automatic_reshapes=enabled)`. `--factor-patch` rewrites that conv as two 4x4 convs -- an exact identity, verified against the unmodified model at max |d score| 1.4e-05 with the top-50 patch set unchanged -- and removes it from the error. **What appears instead is the whole transformer body: 73 layernorm and 38 softmax layers, every per-token reduction.** That is an architectural limit of the dataflow design, not a size limit, which is why no smaller input and no flag moved it. Records in `evaluations/hailo/`. **Untouched by this:** YOLO11 n/s/m still compile for the 8L off the shelf (4.3.1) and OWLv2's accuracy still stands (68/68 visible frames, zero false positives) -- those are PyTorch numbers and are why the model is worth a board at all. The loop is one `ec2.sh up` from re-running against a newer DFC, which is the only thing that could reverse this |
| P7 | The whole corpus, on a rented GPU (`evaluations/gpu/`) | **Run 2026-09-11/12. Two A10G instances, 3.7 h, $3.70, torn down.** It exists because of a **scope error**: every row in `PLAN-onboard-perception.md` 4.11 and P5 was scored on FOUR of the corpus's eight labelled walks, and nothing had ever been scored against all 610 frames. **11 configs now are.** Four findings. **fp16 costs no accuracy** -- identical true positives at every budget, gates matching to three decimals, 1.8x faster; that assumption sat under P5, P6 and 4.9 and had never been measured (INT8 still hasn't, and every latency projection assumes it). **OWLv2 reads 82% at 3 FP, not 96% -- but its margin WIDENS**, because the shipped pipeline falls further (85% -> 58%), so the gap goes from 11 points to 24 at 11x the speed. **4.11's central claim is false**: Grounding DINO gets 50% at ZERO false positives where the shipped pipeline gets 8%, so the three-model pipeline does not dominate at every operating point -- with a useful corollary for 1.11a, since a model that is never wrong is a better corroborator than one that is more often right. And **the field was not as covered as this plan assumed** -- HuggingFace's zero-shot list had three untried families; OmDet-Turbo (7%) and LLMDet (18%) both lose, which is a real result for forty cents. Qwen3-VL was deliberately not run: a known upstream perf bug (5262 ms/token vs 77), a `--vlm-max-pixels` flag it silently ignores (**which corrects P5's "at native tiling"**), and a non-separable score. P7b revises the Orin projection from 51 ms to **124 ms** on measured rather than assumed inputs, and finds the bottleneck is **CPU preprocessing, not the model** (36 ms detecting, 229 ms resizing). P7c withdraws the "YOLO is redundant" claim that 51 ms implied, resolves the out-of-vocabulary tracking gap as an **odometry** problem rather than a perception one, and argues `target_reached` off the cloud |
| 7, 8, 10, 11 | Pi setup, physical assembly, real camera streaming, hardware swap-in | Blocked on buying hardware -- by design, per the simulation-first plan. Nothing to do here yet. **The chassis is no longer a PiCar-X** -- see the row above. |
| extra | Web-based digital twin | Done and deployed (`web-twin/index.html` + `robot/server.py` on ECS Fargate, `service/twin/`, `cloudformation/twin.yaml`) -- reachable from a phone on any network, sharing the vision service's NLB/ALB on port 80 via path-based routing (a ListenerRule matching the twin's exact route set). Verified end-to-end from an actual phone on cellular data, not just curl. |
| extra | Cloud photo-analysis endpoint | Done and deployed (`service/vision_analyze/` on ECS Fargate, behind an NLB -> internal ALB, calling Amazon Bedrock for vision inference). Was originally built on Lambda + API Gateway; both were deleted after an account-level restriction made them permanently unreachable publicly -- see README.md's "History: why not Lambda?" |

---

## 4. Repo map

```
vision-picar/
├── robot/                  "Pi" role -- robot runtime, hardware-agnostic
│   ├── interface.py         RobotInterface -- the ONE abstraction brain/ depends on
│   │                         (incl. get_depth_grid(), M2 -- the only method with a
│   │                          default: all-unusable, so a sensorless backend says so)
│   ├── factory.py            picks sim vs. hardware backend from config/robot.yaml
│   ├── identity.py            one start-up line: service, git revision,
│   │                           executable path, config path (M5). In robot/
│   │                           because BOTH servers log it and robot/ may
│   │                           never import control/
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
│   ├── perceive.py            P1 -- the on-board perception pipeline, run
│   │                           OFF the robot: detector -> crops -> CLIP ->
│   │                           match. Protocols, so a fake drives it in
│   │                           tests and a HEF drives it later. Heavy deps
│   │                           are lazy (requirements-perception.txt) and
│   │                           no test needs them. Real pixels only --
│   │                           never sim frames, see 1.12
│   ├── tiered.py              P2 -- the trigger discipline as a vision_fn:
│   │                           perception is local and free, the cloud is
│   │                           called only on mission_start /
│   │                           candidate_sighting / cold_search, with
│   │                           6.1's two-frame hysteresis. Carries the
│   │                           call counter 6.3 calls the one number that
│   │                           makes the architecture watchable.
│   │                           Reachable from the twin since
│   │                           2026-09-07 as policy: "tiered".
│   │                           Since 2026-09-08 it also computes
│   │                           1.11a's corroboration verdict --
│   │                           REPORTED, never enforced
│   ├── perceive_lab.py       candidate perception backends that are NOT
│   │                           parts: Grounding DINO, OWLv2, YOLO-World and
│   │                           SAM, behind the same Detector /
│   │                           RegionProposer Protocols. They exist to
│   │                           answer "would a Jetson buy anything" off the
│   │                           robot, which 4.11 left open. None can run on
│   │                           a Hailo -- that is the point
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
│   ├── health.py             `python -m control.health` -- one verdict for
│   │                           both halves, non-zero when either is broken
│   │                           or unreachable. Holds the rule about what may
│   │                           reach a verdict at all (M5)
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
│   ├── perception_eval.py    P3 -- scores the ON-BOARD tier over the whole
│   │                           corpus against each walk's adjudicated
│   │                           labels.json (never walk.jsonl), sweeps the
│   │                           gate, and matches two configs on a
│   │                           false-positive budget before reporting
│   │                           either one's recall. walk_eval.py's sibling:
│   │                           that one grades the walk, this one grades
│   │                           the perception
│   ├── recording_routes.py   the two routes that WRITE a walk, mounted by
│   │                           brain_server AND the recordings Lambda --
│   │                           the write path follows the storage, not the
│   │                           brain (which is going back to the Pi)
│   ├── walk_store.py         where recorded walks live -- one abstraction,
│   │                           two backends (a directory, or an S3 bucket).
│   │                           RobotInterface's shape one layer down, and
│   │                           walk_store_from_config() is its factory.
│   │                           Written 2026-09-04 so the walks could leave
│   │                           EFS -- the only component that REQUIRED a VPC
│   ├── admin.html/.js        the recorded-walk console (see admin_server.py)
│
├── tools/hailo/            the Hailo compile loop -- 1.10 item 1, finally
│   │                        built (2026-09-09), and pointed at OWLv2
│   │                        because P5 rests the whole hardware
│   │                        recommendation on one unrun test: can it
│   │                        compile to a HEF at all
│   ├── export_owlv2_onnx.py  OWLv2 -> ONNX, IMAGE SIDE ONLY (the text
│   │                          tower stays on the Pi's CPU, as CLIP's
│   │                          does). Verifies the split reassembles
│   │                          into the real model's own numbers -- a
│   │                          HEF of a wrong graph compiles fine and
│   │                          is worthless
│   ├── owlv2_host_head.py    the half that stays on the CPU: the text
│   │                          einsum, and three ops the `minimal`
│   │                          export moves off the accelerator
│   ├── calibration_set.py    128 frames out of recordings/, stratified
│   │                          by walk and balanced on labels.json
│   ├── compile_owlv2.py      the sweep: translate -> optimize ->
│   │                          compile, per variant, never letting one
│   │                          failure end the run. The REPORT is the
│   │                          deliverable -- three stages fail for
│   │                          three different reasons and imply three
│   │                          different purchases
│   ├── ec2.sh / setup_host.sh  the rented x86 box (no Mac, no ARM
│   │                          path for the DFC) and its teardown
│   └── README.md             read this before running any of it
│
├── config/robot.yaml         mode (sim/hardware), safety thresholds, CORS origins,
│                            and the `brain:` block (robot_url, failsafe budgets)
│
├── tests/                    928 tests, 96% line coverage of brain/,
│                              control/, robot/ and sim/ (incl. test_robot_contract.py's
│                              backend-agnostic conformance suite [S1+S2+M2],
│                              75 tests over five backends,
│                              test_sensors.py [S5],
│                              test_depth_veto.py [M3],
│                              test_authority.py [M4],
│                              test_health.py [M5],
                              test_perceive.py + test_tiered.py
                              + test_perception_eval.py
                              (P1/P2/P3 -- the off-robot
                              perception harness, its trigger
                              discipline and the corpus scorer,
                              all entirely against fakes so the
                              suite never needs torch),
│                              test_watchdog_integration.py [S4],
│                              test_walk_eval.py + test_admin_server.py
│                              (the recorded-walk scorecard and replay),
│                              test_walk_store.py (both storage backends
│                              through one suite, the way
│                              test_robot_contract.py does robot backends),
│                              test_serverless_routes.py + test_static_assets.py
│                              (the VPC-less stack: every route across
│                              CloudFront AND API Gateway, and every file
│                              the pages reference -- the successors to
│                              test_alb_routes.py, same failure mode),
│                              test_ui.py + test_ui_admin.py (Playwright,
│                              real browser at a phone viewport),
│                              test_ui_pipeline.py (Playwright too, but
│                              against a real threaded stub -- the
│                              properties it pins are timing ones, and
│                              page.route() serialises the very requests
│                              it would be measuring), and
│                              test_alb_routes.py -- see section 6)
│                              + 7 runnable (non-automated) demo scripts
│
├── service/vision_analyze/   ECS Fargate: photo upload -> vision analysis (cloud)
│   ├── app.py                 FastAPI app -- /health, /analyze, /describe,
│   │                           /navigate, /guidance
│   ├── vision_core.py         calls Amazon Bedrock (Claude, Converse API)
│   ├── rooms_core.py          identify_room() -- same logic as brain/rooms.py
│   ├── tests/                 app.py's own suite (54 tests) -- routing,
│   │                           validation, decode/size/error handling, all
│   │                           vision_core.* calls mocked. Run separately:
│   │                           `pytest service/vision_analyze/tests/ -v`
│   │                           (see section 6 for why it's not swept into
│   │                           the top-level `tests/` package)
│   └── requirements.txt, Dockerfile
│
├── service/tunnel/            reaching the LOCAL robot + brain from the
│   ├── proxy.py               DEPLOYED twin. One ngrok free-tier domain
│   └── run.sh                  serves both, split by path: /brain/* to the
│                               brain (ROUTE_PREFIX=/brain), the rest to the
│                               robot. Needed because policy: "tiered" loads
│                               YOLO + CLIP into the brain process, which is
│                               why the brain cannot be deployed at all
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
│                               (S1-S5 BUILT, S6 RETIRED except its
│                               continuous-pose half, un-retired 2026-09-06
│                               by PLAN-onboard-perception.md 1.14;
│                               S7 PROPOSED)
├── HARDWARE-READINESS.md      what the real robot changes: verb-to-motor path,
│                               pre-flight checklist, where the brain lives.
│                               Rewritten 2026-09-04 for the chosen hardware
├── PLAN-brain-relocation.md   moving the autonomy loop onto the Pi (B0-B4 BUILT,
│                               B5 needs the Pi)
├── PLAN-teleop-robot.md       a live phone walk driving the real MissionRunner
│                               mission, closed loop -- T1-T4 (BUILT); see the
│                               T1-T4 status-table row above
├── PLAN-microduck-transplants.md
│                               twelve designs borrowed from Pollen Robotics'
│                               Microduck -- a depth sensor instead of asking
│                               the model how far, plus refusal reasons, driver
│                               arbitration, a health verdict and a rollback.
│                               M1-M5 BUILT (2026-09-03, not deployed), M6-M12
│                               proposed; seven need no hardware
├── PLAN-onboard-perception.md  what runs on the car itself -- and the hardware
│                               chain that question turned out to be hiding.
│                               DESIGN SETTLED, NOTHING BUILT. Supersedes parts
│                               of HARDWARE-READINESS.md and retires most of S6;
│                               its section 5 says exactly what. Read it before
│                               any hardware purchase -- the chassis is no
│                               longer a PiCar-X
└── PLAN-aws-cost-redesign.md  the ~$159/month of fixed AWS cost, where it
                                comes from, and the rebuild that removes
                                ~$110 of it. Stage 1 (walks off EFS, onto
                                S3) is DONE; the VPC teardown and the
                                VPC-less rebuild are SPECIFIED, NOT BUILT.
                                Read section 1 before quoting any cost
                                number and section 6 before trusting the
                                design -- its central assumption is still
                                untested
```

---

## 5. Build order -- everything left before hardware

Sequenced across the three plan documents, which hold the detail. Phase
IDs are `S*` = `PLAN-sim-hardening.md`, `B*` = `PLAN-brain-relocation.md`.

**None of stages 0-5 needs the hardware.** Each stage is independently
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

**Put the phone on a wheeled rig at floor height. Do not hold it, and do
not crawl.** Tape or band it upright -- lens forward, not lying flat -- to a
shoebox, a book or a small box on a furniture slider, skateboard, baking
tray or toy truck, and push it with a broom handle while walking upright. A
robot vacuum works too, if there is one.

The rig is better data as well as easier data, and for reasons that map
onto exactly what went wrong before: **height is fixed by construction**, so
it cannot drift with a tiring arm; **tilt is level by construction**, which
is what a pan/tilt at rest actually does, where the invalid corpus looks
*down* onto furniture; and the motion is **wheeled and floor-constrained**,
so it cannot hover over an ottoman -- the impossible viewpoint that made
every previous walk untestable.

**On the height: ~10-13cm, and do not spend effort tuning it.** The "10cm"
this file carried for months was justified as *the PiCar-X camera height*,
and `PLAN-onboard-perception.md` 1.1 replaced that chassis on 2026-09-03
with a differential-drive one whose camera height **no document currently
specifies**. Estimating the new stack -- 65mm wheels, chassis plate ~40mm, a
Pi deck on standoffs, a 2-axis pan/tilt bracket -- lands near 10-13cm, so
the old number happens to be about right while its stated reason is not.
The defect being fixed is **~150cm versus ~12cm**, an order of magnitude;
10 against 15 is a rounding error beside it, and chasing it optimises the
one variable that was never broken. Settle the real number when the chassis
is built -- it is a hardware-day pre-flight item.

Two things to lock before starting, both of which cost nothing now and
cannot be repaired afterwards:

- **Lock landscape orientation.** Three of 33 frames in one 2026-09-02 walk
  were portrait and the model said so itself -- *"blurry and rotated"*,
  *"sideways image"*.
- **Put the rig height in the walk's note** (`PUT
  /recording/walks/{walk}/meta` takes free text), so the corpus describes
  its own viewpoint instead of needing this paragraph to interpret it.

A walk is 10-20 frames and the loop is inherently stop-and-shoot -- capture,
read the action, move about 30cm or turn, capture again. Four to six short
walks with breaks is the whole job; "twenty minutes" below is session
wall-clock, not twenty minutes of crawling.

Robot view pauses when the service reports `target_reached`, so a walk
ends at arrival instead of burning calls. **That field needs an ECS
redeploy of `service/vision_analyze/` to take effect** -- against the
currently deployed service the field is simply absent, the pause never
fires, and everything else behaves as before. The 120-call cap bounds the
cost either way -- and since Robot view's calls started overlapping (up to
two in flight, `GUIDANCE_MAX_IN_FLIGHT`), 120 calls is about a minute of
walking rather than the ~3.3 min a strictly serial loop took. The number
of paid calls is unchanged; the budget is just spent faster, so a walk
that used to fit inside the cap may now hit it.

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

**The corpus every finding below was measured on was DELETED on
2026-09-07** -- all 39 walks, from S3 and from the local backup, on purpose.
They were invalid by their own viewpoint (standing height, target on
furniture), so their numbers were suspect regardless, and an invalid corpus
sitting in the bucket is how it gets scored against by accident. **Treat
every number in this section as a recorded observation that can no longer be
re-run**, and re-derive anything you intend to rely on from the new corpus,
which is currently one walk: `blue-bottle-20260907-142454`. The findings are
kept because the arguments they support are still the best available -- see
`PLAN-onboard-perception.md` 4.10, which names the two claims that are now
assertions rather than measurements.

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
  On hardware the lidar is the real obstacle sensor -- do not let the
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

- **An ordinal distance question was tried and does not yet work
  (2026-08-31, Lab only).** `/navigate`'s `default-with-distance` variant
  asks how many robot moves of clearance there are ahead --
  `within_one_step` / `a_few_steps` / `far` -- ordinal rather than metric,
  because a single monocular frame cannot give metric depth. Replayed over
  80 frames from five recorded walks (two targets, 100% coverage) against
  Lab's vision service: **`within_one_step` 60%, `far` 5%**. Someone
  walking across a house does not spend three frames in five one step from
  a collision. It is the same over-reading `obstacle_ahead` already shows,
  and it is not the target being miscounted -- the skew holds at 55% on
  the frames where the target is not visible at all. The two fields agree
  with each other 85% of the time, so they are wrong together rather than
  independently noisy. `brain/agent.py`'s proximity veto exists but stays
  **off by default**: wired on, this would block three FORWARDs in five
  and reproduce the never-FORWARD stall. Next attempt should change the
  question's shape -- what is in the centre third and in the path, not
  what is nearest anywhere in frame.

- **A fifth wording, `bearing-only`, deletes the obstacle question instead
  of rewording it. MEASURED 2026-09-02, and it is the first wording that is
  degenerate on no model.** Phase M1 of
  `PLAN-microduck-transplants.md`. The argument is that a single monocular
  frame does not contain metric depth, so no wording recovers it: `/navigate`
  keeps *what* and *which way*, and a distance sensor owns *how far*. Two
  lines are removed from `default` and nothing else -- the question and its
  schema line -- so `obstacle_ahead` is **absent from the reply**, not false.
  `brain/navigate.py` maps that absence to `free_space: "unknown"`, and the
  only obstacle logic left on the path is `robot/safety.py`'s
  `get_distance()` re-check before every FORWARD. Under a replay
  (`ReplayRobot` has no sensor) `control/walk_eval.py` will therefore flag
  the run `collision`: **record that column as "what the sensor must catch",
  not as a defect** -- it is M10's specification.

- **The full 5x3, at 100% coverage, replayed 2026-09-02** (same 22 frames of
  `red-backpack-20260829-195904`, every model x every wording). FORWARD rate:

  |                   | default | next-step-obstacle | next-step-and-walls | center-third-path | bearing-only |
  |---|---|---|---|---|---|
  | Claude Opus 4.5   | 0.591 | 0.318 | 0.455 | 0.591 | **0.591** |
  | Claude Sonnet 4.5 | 0.000 | 1.000 | 0.955 | 0.091 | **0.591** |
  | Qwen3-VL          | 0.773 | 1.000 | 1.000 | 0.818 | **0.773** |

  Three things this settles.

  **`bearing-only` is the only column with no degenerate cell.** No `stalled`
  flag, no `degenerate` flag, all three models inside 0.591-0.773. Sonnet's
  never-FORWARD stall -- the failure that started this whole investigation --
  is gone without flipping to the always-FORWARD failure that every previous
  attempt traded it for. Deleting the question did what five rewordings of it
  could not. **This is evidence about the degenerate modes, not about
  navigation:** no cell in the table reaches the target, and every
  `bearing-only` cell is still flagged `collision` -- 12 of 12 checked
  FORWARDs on all three models. That column is M10's specification, not a
  defect: `ReplayRobot` has no distance sensor, and the whole argument is
  that the sensor is what refuses those moves.

  **Question 5 was what broke `next-step-obstacle`, not the region change.**
  `center-third-path` moves question 2 alone and takes Sonnet from 0.000 to
  0.091; `next-step-obstacle` moved questions 2 and 5 together and took it to
  1.000. That attribution is exactly what the variant was built for, and it
  cost one replay to get. Neither is worth promoting -- 0.091 is still the
  stall.

  **Qwen was never answering the question anyway.** Its `obstacle_rate` is
  0.000 under `default`, and its `bearing-only` numbers are identical to its
  `default` ones in every field -- same FORWARD rate, same 12/12 collisions,
  same score, same agreement. Removing a question a model was already
  ignoring changes nothing, which is the cleanest possible confirmation that
  `obstacle_ahead` is uncalibrated rather than merely noisy.

- **The closed-loop sim run was made, and it does NOT settle the gate
  question -- the renderer is the blocker (2026-09-02).** Stage 1's "done
  when" ran for real: `tests/demo_sim_mission.py`, `policy: "vision"`, the
  grid world, the deployed `/navigate`, `sim.sensor_noise.enabled: true`,
  Opus 4.5, 40 paid steps each.

  | wording | steps | wall clock | ended | distance to target | outcome |
  |---|---|---|---|---|---|
  | `default` | 40 | 142.2s (3.6s/step) | (2,1) | 14 cells | `max_steps` |
  | `bearing-only` | 40 | 146.1s (3.7s/step) | (2,2) -- never moved | 13 cells | `max_steps` |

  Neither reached the target; neither ever claimed to. Action spread was 31
  RIGHT / 6 STOP / 2 FORWARD / 1 LEFT and 39 RIGHT / 2 FORWARD. **But the
  reason was not the policy.** Nearly every decision's reasoning said some
  version of "the image is very dark and unclear" or "a blank gray wall", and
  the model was right: the render painted its ceiling and floor with the
  twin's `--wall` / `--floor` CSS variables -- two near-blacks meant for dark
  UI chrome -- so a room came back as a black void with two grey slabs. The
  policy spun looking for a view it never got.

  **Both renderers now carry their own lit ceiling and floor** and no longer
  read the app's theme, so restyling the twin cannot change what the model
  sees (`sim/renderer.py`, `renderFPV` in `web-twin/app.js`, golden
  re-blessed, browser parity test still green). **That fix is unmeasured:**
  it makes the frames legible to a human eye, and whether a closed-loop run
  can now discriminate between wordings is the next paid run's question.
  Until then the replay table above is the instrument for anything about the
  *seeing*, and a sim run measures the *loop* -- cost, wall clock, budgets,
  the veto. One half of the diagnosis is still open and is a map question,
  not a renderer one: the starter house's start pose faces a near wall, so
  even a lit first frame shows little of the room.

  Two things the runs did prove, which no replay can. The **safety collar is
  live and fired** (1 veto on `default`, 2 on `bearing-only`) -- the only
  obstacle logic left under `bearing-only`, exactly as designed. And the
  brain-side `min_distance_cm` was **30.0, exactly one grid cell**, so with
  S5's 3cm jitter the veto was near a coin flip at one cell of clearance
  (`28.0`, `28.9`, `27.7` all blocked live; a test written for it measures
  90/200). **Now 20.0**, matching `safety.min_distance_cm` that
  `robot/server.py` has always used, so the brain-side pre-check and the
  robot-side veto agree on one number. Against the noiseless sensor this
  changes nothing -- an exact reading is only ever a multiple of 30, so any
  threshold in (0, 30] blocks exactly the one case that matters -- which is
  why 30.0 sat there unremarked until noise was switched on.

- **That shape change was built as `center-third-path` and measured on
  2026-09-02. It does not work, and the corpus it was measured on turned
  out to be invalid. Both halves matter.**

  It asks what is in the bottom half of the centre third -- the ground the
  next step crosses -- as `open_floor` / `blocked` / `unclear`, with
  `obstacle_ahead` defined as a restatement of it. Eight walks, 96 frames,
  two targets, replayed against `default` under two models, every cell at
  coverage 1.0. Frame-weighted FORWARD rate:

  |                   | default | center-third-path |
  |---|---|---|
  | Claude Opus 4.5   | 0.323 | 0.365 |
  | Qwen3-VL          | 0.354 | 0.573 |

  The collision flag did not move at all -- 6/8 walks for Opus, 7/8 for
  Qwen, under both wordings -- and the models moved *apart* rather than
  together. **The per-frame field explains why, and it is worth knowing:**
  Opus answers `blocked` on 66%, Qwen on 5%, agreement 40% against a chance
  rate of 35% (kappa 0.075). But the disagreement is perfectly **nested**:
  all 32 frames Opus called `open_floor`, Qwen called `open_floor` too, and
  of the 63 it called `blocked` Qwen called 58 of them open. Neither model
  ever contradicts the other's ordering. They read the picture the same way
  and cut the threshold in different places -- and **a threshold has no
  wording**, which is why four rewordings have now failed and a fifth
  should not be written. The instruction itself was followed exactly:
  `obstacle_ahead` restates `path_ahead` on 99%/100% of frames.

- **The Stage 0 corpus does not test what it claims to, and every number
  above and below inherits the problem (found 2026-09-02, by reading the
  frames instead of the JSON).** In every walk sampled, across both
  targets:

  1. **The target is on raised furniture.** The red backpack sits on an
     ottoman; the blue bottle sits on a console table. The car is a floor
     robot. It cannot arrive at either, so `target_reached` is not merely
     rare in these walks -- it is unachievable, and the collision flags are
     *correct*: the only way to approach the target is to drive into the
     furniture holding it. Opus says so in its own reasoning, answering
     `blocked` and then FORWARD "since reaching the backpack requires
     moving toward the couch". That is not incoherence. It is a real
     dilemma handed to it by an impossible task.
  2. **The camera is at standing height, not 10cm.** The frames look *down*
     onto a ~45cm ottoman and a ~75cm table. Stage 0's own instructions
     above say to hold the phone at ~10cm and that "a chest-height view is
     not the robot's view" -- the recordings did not follow it. From 10cm
     that ottoman is a wall, and none of these scenes resolve the same way.

  **Consequence: the four-wording failure is not established as a prompt
  problem or a policy-shape problem.** It was measured on a task the robot
  cannot perform, from a viewpoint it will never have. The 3x3 matrix, the
  `distance_estimate` skew and the table above are all reproducible and all
  suspect for the same reason. **Nothing further should be spent on prompt
  wording until the corpus is re-recorded:** a target on the floor, the
  camera at robot height on a wheeled rig (see the Stage 0 rig note above --
  do not hold the phone, and the "10cm" figure is a retired PiCar-X number),
  both rooms, several walks. That is a phone and twenty minutes, and it is
  the cheapest high-value item left in Stage 0.

- **Five live walks on 2026-09-02 were evaluated on 2026-09-03. None of them
  is usable as evidence about wording, and the reasons are worth more than
  the numbers.** All Opus 4.5, target "Bottle", all scored `poor`, none
  reached:

  | time | wording | n | FWD | obstacle | target visible | vis-flips | score | judge |
  |---|---|---|---|---|---|---|---|---|
  | 16:35:29 | default | 19 | 0.00 | 1.00 | 0.21 | 5 | 24 | 0.12 |
  | 16:36:32 | default | 21 | 0.00 | 0.95 | 0.62 | 9 | 20 | 0.12 |
  | 16:37:47 | `bearing-only` | 33 | **0.03** | -- | 0.12 | 3 | 18 | 0.14 |
  | 16:39:23 | `center-third-path` (+1 stray) | 17 | 0.18 | 0.65 | 0.59 | 3 | 40 | 0.50 |
  | 16:40:17 | default | 47 | 0.17 | 0.96 | 0.43 | 9 | 26 | 0.12 |

  **`bearing-only` went degenerate live -- 30 of 33 frames RIGHT -- which
  looks like a flat contradiction of the replay table and is not one.** Read
  the reasoning: every spin says "no bottle is visible... turning right to
  scan more of the room". It found the bottle at frame 8 (FORWARD, "centre
  third, still across the room"), turned LEFT twice to centre on it,
  overshot, lost it at frame 11 and resumed spinning. **That is a search and
  memory failure, not an obstacle failure** -- deleting the obstacle
  question cannot help a model that has lost the target and has no record of
  which way it already turned. It is the first live evidence for M12, and it
  says nothing about M1's reading either way.

  **The corpus defects recorded above are fully reproduced.** The frames were
  opened, not just the JSON: the camera is at standing height looking *down*
  onto furniture, and the bottle is on a round cafe table (~75cm). A floor
  robot cannot arrive at it, so `collision` is again the correct flag and
  the walk again tests a task the robot cannot perform. Also 3 of 33 frames
  are portrait against 28 landscape -- the model says so itself ("blurry and
  rotated", "sideways image"). **The re-recording called for above is still
  the cheapest high-value item in Stage 0, and it has not been done.**

- **A recording bug found while reading those walks, and it undercuts
  attribution generally.** Walk `bottle-opus-4-5-center-third-path-20260902-163923`
  contains sixteen `center-third-path` frames and one `bearing-only` frame at
  `seq: 37` -- a `/navigate` call still in flight when the previous walk was
  stopped, written into the *next* walk's directory with its old wording and
  its old sequence number. Same orphaned-in-flight-call class the Robot-view
  HUD already fixed with `guidanceEpoch`, one layer down: the recorder needs
  the same epoch. **Until it is fixed, no walk recorded right after another
  in one session is safely attributable.** Fix is specified in M7b.

- **Decision, 2026-09-03: ship one wording.** `default` on the live path;
  all five kept in replay, where they are the only controlled comparison
  this project has. The argument is that the models' disagreement is a
  *threshold* and a threshold has no wording (see `center-third-path` above),
  that Opus 4.5 reads 0.591 under both `default` and `bearing-only` so the
  choice is already a no-op for the shipped model, and that each live variant
  is one more way for a walk to be unattributable -- three of which have now
  cost real measurements. `bearing-only` is deliberately **not** promoted:
  best-on-one-walk is how the `NavigateModelId` mistake happened. Phase M7b
  of `PLAN-microduck-transplants.md`, gated on M7.

- **The re-recording two bullets above call "the cheapest high-value item
  left in Stage 0" WAS done, on 2026-09-07.** Those bullets are dated
  findings and are left as written; this is the correction. The corpus is
  now **four valid rig walks** -- floor height, target on the floor, each
  with an adjudicated `labels.json` beside it -- and the 39 invalid walks
  were deleted the same day. Everything the new corpus settled is in
  `PLAN-onboard-perception.md` 4.10.

- **What to record next is a different question now, and it has an
  answer**: two out-of-vocabulary **searches** plus one control, driven
  under `policy: "tiered"` so 1.11a's corroboration verdict is measured
  live. `PLAN-onboard-perception.md`'s "What to record next, and why these
  walks" (2026-09-08) has the targets, the shape and the two COCO nouns
  that look out-of-vocabulary and are not. **This is now the cheapest
  high-value item left**, and unlike the last one it is not about the
  viewpoint -- it is the falsifier for an amendment that is otherwise
  going to be decided on one walk.

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

The vision loop is the product (Q1). It exists in Python
(`brain/navigate.py` + `brain/vision_agent.py`, `policy: "vision"`) and,
since S2 landed on 2026-08-31, runs against **every backend**: the
grid-world sim, recorded walks (`sim/replay_robot.py`), a live phone walk
(`sim/teleop_robot.py`) and hardware later. Room memory (the rest of S2b)
is built too. What is left in this stage is the demo that spends real
money -- see **Done when** below.

- **S1 -- pin the contract -- BUILT.** `tests/test_robot_contract.py`:
  a backend-agnostic conformance suite (36 tests) parameterized over all
  four `RobotInterface` backends that exist today (`MockRobot`,
  `RemoteRobot`, `ReplayRobot`, `TeleopRobot`), asserting return shapes,
  units and `stop()` idempotency with no grid-specific assertions. Extended
  by S2 (44 tests now) to pin pixels too: every backend must return a
  decodable image and name its media type.
- **S2 -- real image bytes -- BUILT (2026-08-31).** `sim/renderer.py` is
  the twin's raycaster ported into Python, so `get_camera_frame()` returns
  JPEG bytes on every backend and the one structural blocker between
  Vision Autopilot and hardware is gone. The JS raycaster is still present
  as the pre-S2 fallback and is now safe to delete -- see the status table
  in section 3, and `sim/renderer.py`'s fidelity note, which is the reason
  this does not retire the real-photo gate in Stage 0.
- **S2b -- the Python vision agent** -- **BUILT** for recorded walks
  (`python -m tests.demo_replay_mission <walk> "red backpack"`, which
  takes `--policy tiered` since 2026-09-07 to run the P2 chain over the
  same frames) and, since
  `PLAN-teleop-robot.md`'s T1-T4, for a live phone walk too. **Room-level
  step memory is built**: `/navigate` now exchanges `searched_rooms`
  (client -> server, from `MissionMemory.searched_rooms`) and `room_guess`
  (server -> client, backfilled into `frame["room"]`) -- see
  `AGENT-HARNESS.md` section 10 for the exact mechanism, which
  deliberately doesn't touch the `vision_fn(frame) -> scene` contract.
- **`service/vision_analyze/app.py`'s test suite -- BUILT.**
  `service/vision_analyze/tests/` (54 tests, FastAPI `TestClient`, every
  `vision_core.*`/`identify_room` call mocked) -- closes the one real gap
  left over from the Lambda -> ECS migration. Run separately from the
  root suite: `pytest service/vision_analyze/tests/ -v` (see section 6).

**Done when** a Python agent completes a backpack hunt in the sim
against the real `/navigate`, with cost and wall-clock recorded. Every
piece exists -- the agent, the cost/wall-clock reporting
(`tests/demo_replay_mission.py` prints both) and, since S2, the sim
frames themselves.

**Corrected 2026-09-06: this said "the run itself has not been made",
which contradicts the Stage 0 notes above.** It was made twice on
2026-09-02 (`default` and `bearing-only`, 40 paid steps each) and the
results are tabled there. **Neither reached the target**, so the stage is
not done -- but the reason was diagnosed and fixed: the renderer was
painting the room with the twin's dark UI colours, and the model was
correctly reporting "a blank gray wall". **What is outstanding is a
re-run against the lit renderer**, which is unmeasured. Two things a
re-run gets that no replay can, and which the 2026-09-02 pair already
demonstrated: it is closed loop (turning left really does change the next
frame, unlike `sim/replay_robot.py`), and it exercises the safety collar
live -- which fired 1 and 2 times respectively.

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

- **`policy: "vision"` is live, and reachable from the twin since M1.**
  The runner takes a `vision_fn` and enforces S2b's per-call timeout and
  failure budget around it; `brain/vision_agent.py` is what plugs in there.
  A "vision" mission carries a `model_id` and a `prompt_variant`, both
  validated against the vision service's published allow-lists in one round
  trip at start rather than becoming a 400 inside a tick. `MissionStartRequest`
  forbids unknown fields: it silently dropped `prompt_variant` for as long as
  the twin had been sending it, and a mission that runs a different wording
  from the one named in the UI is unattributable.
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

**Read `PLAN-onboard-perception.md` first -- the chassis is no longer a
PiCar-X.** Its section 1 holds the decided parts list (differential chassis,
RPLidar C1, Hailo-8L M.2 module with a Camera Module 3, one pack and
three rails, ~$555-620)
and section 3.8 the seven questions still to ask the seller. Its section 4 is
the detector decision -- IMX500 vs Hailo vs Jetson -- and its 1.10 lists the
ordering-time checks (storage vs the PCIe lane, the AI HAT+ 2) and the compile
loop to build first. Its section 5 lists what that decision invalidates
elsewhere, including in `HARDWARE-READINESS.md`.

`HARDWARE-READINESS.md` section 5's pre-flight items still apply where they
are chassis-independent, with two now answered by the purchase: **5.2**
(`LEFT`/`RIGHT` skip the distance check, "correct for a pivot and wrong for
an arc") **resolves to the pivot branch**, and **5.3** (where the ultrasonic
is mounted) is **superseded** -- a 360-degree lidar is the obstacle sensor.

Then: `robot/hardware_robot.py`, B5 (systemd units), and the calibration
items in `PLAN-sim-hardening.md` section 7 that can only be measured.

### Not on the critical path

- **S6 (Ackermann turns, continuous pose, scaled map)** -- **RETIRED
  2026-09-03.** It existed because the PiCar-X could not pivot in place and
  `sim/grid_world.py` assumed it could. `PLAN-onboard-perception.md` 1.1
  chooses a **differential-drive chassis**, so the sim's assumption is now
  correct about the hardware and the divergence closes by purchase rather than
  by the largest change in the sim-hardening plan. Continuous pose and a
  to-scale map may still be wanted if a lidar lands and the sim must represent
  metric geometry -- but that is a different phase with a different reason.
  **That reason arrived 2026-09-06: `PLAN-onboard-perception.md` 1.14 makes
  motion continuous, which un-retires the continuous-pose half** (its C2). It is
  much cheaper than S6 costed it -- `sim/renderer.py` already takes a float pose
  and radians, so only `grid_world.py`'s integer cells and cardinal `Heading`,
  plus a two-line conversion in `mock_robot.py`, are actually discrete. The
  Ackermann half stays retired.
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

- **Only conditions a release can be blamed for may reach a health
  verdict** (M5). `control/health.py` holds that rule and is the only place
  that decides what "unhealthy" means -- the twin's Settings line renders
  its per-half answers and must never invent one, or the page and the
  command that gates M11's rollback could disagree about the same robot.
  When adding a `/health` field, put it in the verdict list or the
  description list deliberately: a check that goes red because the robot is
  parked is one everybody learns to ignore.

- **Since M4 the robot server arbitrates who is driving**, by the order in
  `AGENT-HARNESS.md` 4.1 (`stop > twin-dpad > brain > twin-local-brain`).
  Every `/action` names its driver in an `x-driver` header; an unnamed one
  ranks as manual on purpose (see `robot/interface.py`'s reasoning -- the
  callers that don't name themselves are people). Authority lapses on
  silence, on the watchdog's own clock, so there is no release call to
  forget. Don't add a movement route that skips this: it is the only guard
  that can see the D-pad, and the twin's 409 and local-loop refusal cannot.

- **Since M3 the safety re-check prefers the depth grid**, and
  `robot/safety.py`'s `path_clearance()` is the only place in the project
  that decides what "the path" means. Three outcomes in order: a measured
  path zone, then "nothing within range" (never a veto), then the scalar
  `get_distance()` as the fallback for a backend with no grid or a wholly
  blind one. **A zone reported `unusable` never enters the comparison** --
  as a distance it would stop the robot on every dropout, as clear it would
  drive through what the sensor could not see. Don't add a second copy of
  the reduction anywhere: `GET /depth` publishes it (`path`) precisely so
  the twin can draw it without owning it.

- **Which zones are "the path" is chosen by ANGLE, not by fraction of the
  columns** (`PLAN-onboard-perception.md` 5.1, fixed 2026-09-03). The grid
  carries `fov_deg`, and `path_zone_indices()` selects every column whose
  bearing is within ~15.4 degrees of ahead -- `atan(half the chassis width /
  one move's travel)`. It used to take the middle half of the columns, which
  was a fair proxy only because every sensor considered had a narrow forward
  field: on the 360-degree lidar now chosen, the middle half of the columns is
  **the entire forward hemisphere**, which vetoes every corridor. **A backend
  with a wide field must publish `fov_deg`** -- one that doesn't gets the old
  fraction rule and a warning, never silence. On the sim's 60-degree grid both
  rules pick the same four zones, so this changed no behaviour there, and the
  readout that would show it if it broke is the depth strip's outlined path
  zones under the twin's FPV canvas.

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

- **The twin is a deployed artifact -- `web-twin/` changes are not shipped
  until they are synced.** `index.html` and `app.js` go to the serverless
  stack's static bucket and are served by CloudFront:
  `bash service/static/sync.sh <bucket> <distribution-id>`, both values from
  the `vision-picar-serverless` stack outputs. Synced for P2 on 2026-09-07.
  **Check parity rather than assuming it** -- `curl -s
  https://<dist>/app.js` diffed against `git show HEAD:web-twin/app.js` is
  the whole test, and it is worth running before calling a twin change
  shipped. Two related traps:

  1. **`config/robot.yaml` and `control/brain_config.py` must move
     together.** `service/lambda/build.sh` copies both into the walks
     Lambda, and `load_brain_config()` *rejects unknown keys* -- so a new
     yaml beside an old `brain_config.py` is a cold-start `ValueError`, not
     a silently ignored setting. The build script copies both from one tree,
     so the documented path is safe; hand-patching one file in a zip is not.
  2. **CloudFront caches `app.js` with `no-cache`, but sync.sh invalidates
     anyway.** Wait for the invalidation to report `Completed` before
     testing, or you are testing the previous build.

- **The brain cannot be deployed, so the deployed twin reaches it through a
  tunnel** (`service/tunnel/`, 2026-09-07). `policy: "tiered"` loads YOLO and
  CLIP into the brain process, and the brain has not been in AWS since
  2026-09-05 -- its home is the Pi (B5). `bash service/tunnel/run.sh` starts
  the robot, the brain and a fan-out proxy; `ngrok start picar` publishes it.
  Three things that are not obvious and cost an afternoon each:

  1. **One tunnel, two services, split by path.** ngrok's free plan gives a
     single static domain per account; a second endpoint on it is
     `ERR_NGROK_334`. So the brain runs with `ROUTE_PREFIX=/brain` and
     `service/tunnel/proxy.py` fans `/brain/*` to it and everything else to
     the robot -- the same mechanism that let both share one ALB on ECS.
     Settings then wants `https://<domain>` and `https://<domain>/brain`.
  2. **ngrok's free tier serves an HTML interstitial to anything with a
     browser User-Agent**, so every `fetch()` from the twin came back as
     markup and `res.json()` threw on a `<`. `app.js` sends
     `ngrok-skip-browser-warning` -- but only to ngrok hostnames, because it
     is a custom header and would otherwise force a CORS preflight on the
     twice-a-second `/health` poll of every LAN setup. `tests/test_ui.py`
     pins both halves.
  3. **Set `APP_SHARED_SECRET`.** `require_secret()` is inert without it,
     and a tunnel puts the robot and a brain with `allow_drills: true` on the
     public internet. `run.sh` reads it from `~/.vision-picar-local-secrets`
     (mode 600, never in the repo), and sets `VISION_SHARED_SECRET`
     separately because the deployed vision service has its own.

  **Warm the models before a rig walk.** The first tiered mission downloads
  `yolo11s.pt` (18MB) *inside* `POST /mission/start`, so the panel sits on
  "Starting..." for tens of seconds and the first status poll shows step 0.
  `python -c 'from brain.perceive import pipeline_for; pipeline_for("x")'`
  once, and start is a second or two thereafter.

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

- **Coverage is 96% of `brain/`, `control/`, `robot/` and `sim/`, and the
  last 4% is deliberate.** (This line read 99% until 2026-09-07 and 98%
  until 2026-09-08. Quote what the command prints, not this sentence.)
  Measure it with:

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

  **The 98% -> 96% step on 2026-09-08 is `brain/perceive_lab.py` (60%) and
  `brain/perceive.py` (87%), and both are the same uncoverable thing: a
  constructor whose body downloads and loads a model.** The lab module is
  mostly constructors by weight -- four candidate backends and almost no
  logic between them -- so its percentage is low for a module with 17 tests
  over everything a fake can reach. The alternative was `# pragma: no cover`
  on those constructors, which would raise the number and make this module
  describe itself differently from the shipped one beside it doing exactly
  the same thing. Consistency won; `control/perception_eval.py`, which holds
  the arithmetic that actually decides things, is at 97%.

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

  **And run it in its module's real order, not just with `-k`.** The
  session-crossing test in `tests/test_ui_pipeline.py` passed alone three
  times and failed in the suite -- because its stub's answer plan was keyed
  to how many calls the *first* run happened to make, which a warmer browser
  changed. The defect was in the test, but a `-k` run would have shipped it
  either way. A timing test that has never run alongside its neighbours has
  not been run.

- **Every ECS task definition here is ARM64 -- build images with
  `--platform linux/arm64`.** An amd64 image pushes to ECR without
  complaint and then fails at placement with `CannotPullContainerError:
  image Manifest does not contain descriptor matching platform
  'linux/arm64 v8'`, which costs a couple of rollout attempts before it is
  obvious what happened. Confirm with `aws ecs describe-task-definition
  --query taskDefinition.runtimePlatform` if in doubt. The full loop for
  any of these services is: `docker build --platform linux/arm64 -f
  service/<name>/Dockerfile -t vision-picar-<name> .` (from the repo root
  -- see the repo map), tag and push to the matching ECR repo, then `aws
  ecs update-service --cluster vision-picar-cluster --service
  vision-picar-<name>-service --force-new-deployment`.

- **A replay outlives its own HTTP response.** `POST
  /recording/walks/{walk}/replay` is synchronous, and the shared load
  balancer closes the response at 60s -- which any walk past roughly 20
  frames will exceed. The server finishes the job and writes its sidecar
  regardless, so the caller's move is to poll `GET
  /recording/walks/{walk}/replays` for a fresh `replayed_at`, which is
  what `control/admin.js`'s `pollForReplay()` does. **Do not fire the next
  replay when the POST returns**: on a timed-out response the previous one
  is still running, and stacking replays on one vision task is the exact
  load that used to make them lose frames. Making replay a job (POST
  returns an id, poll for the result) would remove the class of problem
  and has not been done.

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

## 7. Twin first, then the car

The project's ordering rule, stated by the user 2026-08-27 and binding on
everything below (the quote predates the chassis change; "the PiCar" means
the real robot, whatever it is built from):

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
| 0 | Validate the premise | Guide tab -> Robot view, phone on a wheeled rig at ~10-13cm | The move the robot would make from where you stand; pauses on arrival |
| 2 | B0 `RemoteRobot` | Sim tab -> D-pad, then Remote brain -> Start | Both drive the same robot through the same server; the map follows either one |
| 2 | B1 `MissionRunner` | Remote brain -> Start | Step count, last action, rooms searched and a log tail advancing ~4 steps/second |
| 2 | B2 the brain service | Start a mission, then close the tab and reopen it | The mission is further along, or finished. It never needed the page |
| 2 | B3.1 watchdog | D-pad forward, then stop touching it | "Robot watchdog" counts the silence past the timeout and reports the motors stopped |
| 2 | B3.2 AWS link dead | Drill picker -> vision errors / vision hangs | Three failures counted, mission ends `failed`, robot stopped |
| 2 | B3.3 brain loop hung | Drill picker -> brain loop hangs | One step, then `failed` -- "brain loop hung"; the watchdog readout stays quiet, which is the point |
| 2 | stop stops the car | Start a mission, then Stop | Mission ends `stopped`, the map stops moving, watchdog goes quiet |
| 2 | one brain at a time | Start a remote mission, then tap Explore | Refused with a toast; the reverse is the server's 409 |
| S4 | time in the loop | Set `sim.realtime: true` in `config/robot.yaml`, restart the robot server, then Remote brain -> Start | The watchdog readout climbs mid-move instead of only between moves -- a move now genuinely occupies its duration, off by default so this is opt-in |
| S5 | sensor realism | Set `sim.sensor_noise.enabled: true`, restart the robot server, then D-pad toward a wall | Distance telemetry stops being multiples of 30cm and jitters. The collar still fires only at the wall: `min_distance_cm` is 20 on both sides now, which is 3.3 sigma clear of one cell -- at the old brain-side 30 the jitter alone vetoed ~45% of legal one-cell moves |
| -- | a lit sim camera | Sim tab, drive the D-pad and watch the FPV canvas (or `GET /frame`) | A room: light ceiling, mid-brown floor, blue-grey walls. It used to be a black void with two grey slabs, because the render borrowed the twin's dark `--wall`/`--floor` UI colours -- which is why the model called every sim frame "very dark and unclear" |
| -- | a session's calls die with it | Guide tab -> Robot view, Start, Stop, Start again | The new session's HUD never shows the previous one's decision. A call still in flight at Stop is orphaned by run (`guidanceEpoch`), not by a boolean -- it used to flash its answer over the new camera view and then suppress the new run's first few real decisions. Everything else was already reset on Stop, so a straggler was the only route |
| M2 | a depth grid the robot reports | Sim tab, look under the FPV canvas, then drive the D-pad at a wall | Eight zones, red near and green far, hatched grey where unmeasurable, with the grid's own shape and the nearest zone named beneath. Tap look-left and the strip swings with the picture -- it is cast off the *view* heading, same as the render. A server predating the route says "not reported by this server" rather than going blank, because blank and "no obstacles" must not look alike |
| M3 | the veto reading the grid | Set `sim.sensor_noise.enabled: true` with `dropout_rate: 0.2`, restart the robot server, then drive the D-pad at a wall | The path zones are outlined on the strip and the readout names the clearance the veto actually reads, plus where it came from. Dropped zones hatch grey and the robot keeps going -- seven others answered, where a single beam reading `0.0` would have stopped it. Against the wall the strip turns red and says FORWARD is vetoed |
| M4 | a person outranks the brain | Sim tab -> Remote brain -> Start, then tap the D-pad | The mission ends `preempted` (not `failed`), the log line names `twin-dpad`, the car does what the pad said, and the Driving readout switches. Stop touching it for a second and it reads "twin-dpad (lapsed)" -- authority rides the same deadman the watchdog does, which is why there is no release button to forget |
| M5 | one health answer | Settings -> Check health, then kill the brain and press it again | OK naming each build's git revision, then UNHEALTHY naming the brain with the robot still ok. Drive into a wall first and it stays OK: a parked robot with a safety veto on record is not a release to blame. `python -m control.health` prints the same verdict and its exit code follows |
| M1 | the vision policy, from a phone | Sim tab -> Remote brain -> policy "Vision policy", then Start | The panel names the model and wording first; then each step is one `/navigate` call and the log shows the model's own reasoning. With `sim.sensor_noise.enabled: true` the safety collar is the only thing vetoing a FORWARD at a wall -- which under `bearing-only` is the entire design |
| -- | overlapped vision calls | Guide tab -> Robot view, Start, with developer readouts on | Decisions land about every 500ms instead of every ~3s. The call counter climbs at the dispatch rate, and an answer overtaken by a newer one is never drawn |
| -- | environment banner | Run the robot server with `ENV_LABEL=Lab`, reload the twin | An orange "LAB ENVIRONMENT" bar at the top, a coloured rule on the tab bar, and `LAB ·` prefixing the tab title. Unset it and everything disappears -- that absence *is* production's state |
| -- | endpoint mismatch notice | Settings -> point any of the three URL fields at a host other than the one serving the page | A note under that field naming both hosts. Not an error: it says what will be called, because a tunnel or split local dev is legitimate |
| P2 | the three tiers, on real pixels | Guide -> Robot view -> "Drive via brain" -> "Tiered policy", phone on the wheeled rig, then Start | The only path where YOLO and CLIP see real pixels. Needs the robot server in `mode: teleop` and `pip install -r requirements-perception.txt` where the **brain** runs -- the models are in that process, and the brain is deployed nowhere since 2026-09-05, so this is laptop-plus-phone-on-one-LAN |
| P3 | 1.11a being measured, and visibly not applied | Sim tab -> Remote brain -> "Tiered policy" -> Start, and watch the Corroboration row | This step's verdict with the local P and the bar it was read against, the running corroborated-of-claimed tally, and the words **"not enforced"** on every line. That phrase is the feature: the amendment is undecided, and a panel that let a measurement read as a decision would corrupt the walks meant to decide it. A step that cost nothing says "no claim this step" rather than holding the last verdict |
| P2 | the three tiers, in the sim | Sim tab -> Remote brain -> policy "Tiered policy", then Start | The hint names the two local models and says the paid call fires only on a trigger. Then, per step: the tri-state, the CLIP margin, the detector by name, and **the deliberation counter -- calls *and* frames.** The counter is the proof: it must visibly NOT climb every step, and 6.1 measured 4-6x. The log marks the paid steps `[cloud: <trigger>]` and the free ones say "no cloud call" in the model's own place. Without `requirements-perception.txt` installed, the panel says so *before* Start and the mission refuses with the pip command rather than dying three ticks in |

### What the remaining phases owe

Each of these is a line in that phase's plan entry, to be built with the
phase rather than bolted on after:

| Phase | UI proof it has to ship with |
|---|---|
| S1 pin the contract | **Suite built, button not.** A **"Check robot contract"** button in Settings: run the interface's methods against whatever robot is connected and report which returned the wrong shape. `tests/test_robot_contract.py` is the suite itself, runnable from a terminal against any backend today; the Settings button that makes it pressable from the twin is still owed |
| S2 real JPEG frames | The FPV canvas shows the **server-rendered** frame, with a "frame source: server / local" readout. The picture should not change; where it comes from should |
| M2 depth grid | **Built 2026-09-03.** The depth strip under the twin's FPV canvas, tracking the view |
| M5 one health command | **Built 2026-09-03.** The Settings health line and its Check health button |
| M4 authority + refusal reasons | **Built 2026-09-03.** "Driving" and "Last refusal" beside the watchdog readout, and the mission log naming the refusal instead of stamping every one of them `[VETOED]` |
| M3 tri-state veto | **Built 2026-09-03.** The path zones outlined on that strip, and a readout naming the clearance the veto reads and its source. The "off-centre approach" half of its done-when is **not** demonstrable here and is deliberately left to M10 -- the grid world's walls are axis-aligned and 30cm apart, so a cone and a single ray hit them at nearly the same distance, and a sim test for it would pass because the failure cannot occur |
| P3 corpus scoring | **Built 2026-09-08.** No new UI of its own -- it is a terminal instrument, and this is that phase's one written "no UI change". **The readout that would show it if it broke** is the per-walk table's own agreement with 4.11's published row: a scorer that has drifted stops reproducing 62/74 at 3 false positives on the four walks in `recordings/`. What P3 *did* put on the panel is 1.11a's corroboration row, next to it in this table |
| P2 tiered policy | **Built 2026-09-07.** The Remote brain panel's Tiered option and 6.3's four readouts, above. The detector's *boxes* on the FPV canvas are the one part of 6.3 deliberately not built: the twin's frames are raycaster renders and 1.12 forbids running a detector against them, so boxes there would be a false positive signal rather than a weak one |
| S2b Python vision agent | **Built 2026-09-02 (M1).** The Remote brain panel has a policy picker; `policy: "vision"` runs the Python agent and the log carries Claude's own reasoning instead of "free space clear". The panel names the model and the wording it would ask with, before you spend anything |
| S7 chaos and soak | New drills: added latency, dropped requests, a killed link mid-mission. Same picker, same fail-safe rule |
| B5 deployment | Reboot the Pi. Open the twin on a phone. Start a mission with no laptop on the network at all -- this is definition-of-done item 1, and it is a UI test by construction |
