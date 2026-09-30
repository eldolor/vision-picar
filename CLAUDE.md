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

# Confirm everything still works (about 1420 passed, 51 skipped as of 2026-09-28, with a browser
# installed -- see below; fewer without, as the parity and UI tests skip)
pytest tests/ -v

# service/vision_analyze/ has its own suite -- see section 5, item 1
pytest service/vision_analyze/tests/ -v

# tests/test_ui.py, tests/test_ui_admin.py, tests/test_ui_pipeline.py and
# tests/test_frame_source.py drive the real twin in a real browser. They
# SKIP rather than fail without one, so the rest of the suite still runs --
# install it once to actually get that coverage:
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

**Since 2026-09-19 that rule has a second half, with the same force.**
`RobotInterface` holds **body** state only; **world** state -- the map,
and the robot's pose on it -- lives on `world/interface.py`'s
`WorldInterface`, picked by `world/factory.py` from the `world:` block.
The line is *whose frame is the answer in*: egocentric is body
(`get_distance`, `get_depth_grid`, `get_odometry` -- "how far am I from
that", "what is ahead of me", "how far have I driven"), allocentric is
world (`get_pose`, `get_map`). The short form, and it is in the
docstring: **odometry is what the body says about itself; pose is what
the world says about the body** -- which is why they are separate methods
on separate interfaces, because a mapper's pose JUMPS on loop closure and
odometry by contract never does. Same prohibition as above: no code path
in `brain/` may import a world backend directly.
`tests/test_world_contract.py` pins both interfaces against each other,
so a `get_pose()` that drifts back onto `RobotInterface` because "where
am I" felt like body state fails a test rather than passing review.
**And one more, guarding the ROS decision: nothing outside
`service/slam/` may import `rclpy`** (`tests/test_ros_containment.py`,
written before any ROS exists). That single rule is the whole difference
between `PLAN-onboard-perception.md` 3.3's (b+) and its (c), which
"swallows the project" -- see `PLAN-mapping.md` section 4.

The second constraint follows from the first: **build it, prove it with
data in the sim, then put it on the car** -- section 7 has the rule. The
abstraction above is what makes the sim's data mean anything about the
robot (the sim drives the same API the hardware will, so a mission that
succeeds through it exercises the path the car will use). **Changed
2026-09-25:** this used to say "prove it in the digital twin's UI", with a
phase done only when someone had watched it on a phone; the user replaced
that with a data-driven definition of done.

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
| S2 | Real image bytes -- the twin's raycaster ported into Python (`sim/renderer.py`) | Done (2026-08-31). `MockRobot.get_camera_frame()` returns a rendered JPEG like every other backend, so the vision policy drives the simulator and `RobotInterface` no longer has to change when hardware lands (`PLAN-sim-hardening.md` 2.1, now closed). Parity with the JS was proven column by column, which is what licensed deleting `renderFPV` -- **deleted 2026-09-25 with the ROS alignment**, along with the parity suite that had nothing left to compare against (`tests/test_frame_source.py` keeps the half that still means something; the golden image in `tests/test_renderer.py` is what pins `sim/renderer.py` now). R0 turned that deletion from tidy-up into a fix: `renderFPV` read a CELL and a CARDINAL heading, so against a continuous pose it drew a direction the robot was not facing, and one call site fed that picture to the model. The "frame source" readout survives with a better meaning -- real pixels vs. NO pixels, which is a state a real camera can be in. Read `sim/renderer.py`'s fidelity note before treating a sim result as a statement about real rooms. |
| S2b (partial) | The Python vision agent | Done for recorded walks (`brain/navigate.py`, `brain/vision_agent.py`, `sim/replay_robot.py`, `policy: "vision"`, and the twin's "Record this walk" switch) and, since T1-T4, for a live phone walk too (`sim/teleop_robot.py`, "Drive via brain"). The model decides every move; the harness supplies the timeout, the failure budget and the step/cost cap. Room-level step memory is done -- `/navigate` exchanges `searched_rooms`/`room_guess` with the client, and `brain/agent.py:MissionAgent.step()` backfills `frame["room"]` from it (`AGENT-HARNESS.md` section 10). Since S2 it drives the grid-world sim too, and since M1 it is startable from the twin: the Remote brain panel has a policy picker, and a vision mission carries a `model_id` and a `prompt_variant` that are validated at start and shown before you spend anything. |
| B4 | The twin becomes an observer | Done (`web-twin/index.html`'s "Remote brain" panel + `control/drills.py`). Missions start from the phone and survive the tab; the failsafe drills and watchdog readout make B3's guards watchable. Only B5 (systemd on the Pi) is left in that plan. |
| T1-T4 | Teleop robot: a live phone walk drives the real `MissionRunner` mission, closed loop (`PLAN-teleop-robot.md`) | Done and deployed (2026-08-28) -- `sim/teleop_robot.py`'s `TeleopRobot` (a fourth `RobotInterface` backend: `mode: teleop`, no motor, a live pushed camera frame, no distance sensor), `POST /teleop/frame` on `robot/server.py`, the twin's Robot view "Drive via brain" switch, and sibling `teleop-robot.yaml`/`teleop-brain.yaml` CloudFormation stacks sharing the existing NLB/ALB (see section 6's AWS-topology bullet). Verified end to end: a real phone walk found its target (`OUT: FOUND`), and both B3.2 (vision-failure budget) and T1's stall detection were triggered live, no drill, against the deployed services. One rough edge, since fixed: `/frame` now catches a stall and returns 503 with the real message instead of a generic 500. |
| extra | Interim: brain on ECS Fargate | **DELETED 2026-09-05** with the rest of the ECS stack (`PLAN-aws-cost-redesign.md`); the brain now runs locally and the deployed twin reaches it through `service/tunnel/`. Was: done and deployed (`service/brain/`, `cloudformation/brain.yaml`) -- `control/brain_server.py` alongside the twin and vision-analyze on the same shared NLB/ALB, so the remote-brain panel works from a phone off the home LAN with no HTTPS tunnel. Not a build-plan phase and not B5: the brain's real home is still the Pi: see `PLAN-brain-relocation.md`'s "Interim: brain on ECS Fargate" for why this doesn't conflict with that, and for the one real gap it surfaced (separate outbound secrets for the robot vs. the vision service). |
| extra | Recorded-walk evaluation harness | Done and deployed (2026-08-30) -- `control/walk_eval.py` scores a walk (metrics + an LLM judge + a collision check, advisory and kept out of the operator's own label), `control/walk_replay.py` re-asks its frames under another model or prompt, `/navigate` takes `model_id` and `prompt_variant` from server-side allow-lists, and the console shows a per-model summary. **This is the instrument the Stage 0 gate needed:** it turns "did that walk go well" from an afternoon of reading JSON into a button, and replay is the only controlled model comparison available -- two live walks vary the operator's path as well as the model. What it has already established is in the Stage 0 notes below. |
| extra | Recorded-walk storage + admin viewer | **Walks moved to S3 (`control/walk_store.py`, `cloudformation/recordings-s3.yaml`) and the EFS volume and admin ECS service were deleted 2026-09-05**; the console is served by the serverless stack. Was: done and deployed (`cloudformation/recordings.yaml`, `service/admin/`, `control/admin_server.py`) -- an EFS volume (survives redeploys, unlike Fargate's own filesystem) holding Robot-view "Record this walk" data, plus a separate `/admin` service to list/view/delete it. Deliberately its own service, not more routes on `brain_server.py`: reviewing recordings has no reason to move to the Pi when B5 lands or to go down when the mission server restarts. See `PLAN-brain-relocation.md`'s Interim section and `control/admin_server.py`'s docstring. Since T1-T4, `teleop-brain.yaml`'s brain has no EFS mount of its own and instead proxies `POST /recording/frame` to the main brain (`control/brain_server.py`'s `recording_proxy_url`) -- see `PLAN-teleop-robot.md`'s "Recording proxy" section for why only that one route, never `/mission/*`, may be proxied between brains. |
| -- | LLM-driven planner (`brain/planner.py`) replacing rule-based `decide()` | NOT BUILT. Designed but never written to disk -- a `PlannerAgent` calling Claude with `MissionMemory.as_context()` as the prompt. **This is now the main hardware-path gap:** `PLAN-sim-hardening.md` Q1 settled that the robot is vision-driven, and the vision loop's *port* is done (`brain/navigate.py` calls `/navigate`; the JS Vision Autopilot that used to be the alternative was deleted 2026-09-25). Phase S2b of that plan specifies the port, including the step-memory problem the browser version does not solve. Real gap if you want the actual "high-level planner" from the architecture diagram rather than the current rule-based frontier-exploration policy. Stage 2's `MissionRunner` is where it plugs in -- `AGENT-HARNESS.md` section 10 is the instruction sheet: it takes a `vision_fn` and already enforces the timeout and failure budget such a policy needs, and `control/brain_server.py` serves `policy: "vision"` with `brain/vision_agent.py` today -- what is still missing is room-level *planning* over `MissionMemory`, not the vision loop. |
| M2 | A depth grid on `RobotInterface`, and in the sim | Done (2026-09-03), not deployed -- `get_depth_grid()` with an honest all-unusable default, `MockRobot` synthesising it from `renderer.cast_ray()`, `GET /depth`, `RemoteRobot` over it, a depth strip under the twin's FPV canvas, and `_HaltGate` added to the conformance suite as a fifth backend. Nothing in `brain/` reads it yet: M3 is the consumer. See `PLAN-microduck-transplants.md`. |
| M3 | The tri-state zone, and a centre-zone veto | Done (2026-09-03), not deployed -- `SafetyController.path_clearance()` reduces the middle half of the grid's columns to one number and compares it to `min_distance_cm`, exactly as it compared `get_distance()` before. A failed zone never enters the comparison in either direction; a wholly blind path falls back to the scalar and keeps its `0.0`-on-dropout stop. `GET /depth` publishes the reduction so the twin never recomputes it. |
| M4 | Refusals are state, manual preempts autonomous | Done (2026-09-03), not deployed -- `robot/server.py` arbitrates `/action` by a decided order (`stop > twin-dpad > brain > twin-local-brain`, `AGENT-HARNESS.md` 4.1) instead of letting the last writer win, every refusal carries a machine-readable `reason`, `RemoteRobot` raises `Preempted` rather than `SafetyViolation`, and a preempted mission ends `preempted` with the robot stopped. |
| M5 | One health command | Done (2026-09-03), not deployed -- `python -m control.health` (and a Settings health line) asks both halves and exits non-zero when either is unhealthy or unreachable. Verdict inputs are reachability, the robot watchdog loop's own poll freshness, and a running mission's tick liveness; everything else is description and never changes the exit code. Both servers now log an identity line at start-up. |
| -- | On-car perception + the hardware chain (`PLAN-onboard-perception.md`) | **Superseded on the board 2026-09-19: it is a Jetson Orin Nano Super, not a Pi + Hailo -- see section 3b.** The rest of this row is the 2026-09-03..06 record. **DESIGN SETTLED 2026-09-03, DETECTOR REVISED 2026-09-04, NOTHING BUILT.** Started as "what could run on the car itself" after reading Microduck and ended up rewriting the hardware plan. Decided: a **differential-drive chassis** rather than the PiCar-X's Ackermann (which **retires S6** and makes `grid_world.py`'s pivot assumption correct); a **lidar** used first as a 360-degree clearance ring and only later as SLAM behind an HTTP wall; a **Hailo-8L in M.2 module form, with a Camera Module 3** for on-board detection (the AI HAT+ until 4.9 settled on the module, 2026-09-06) -- chosen on 2026-09-04 over the IMX500 AI Camera (its nano-only ceiling is silicon, and it cannot be fed a recorded frame) and over a Jetson (the right board for arbitrary Hugging Face models, ruled out for now on cost, power and the camera stack; its section 4 has the three-way comparison and the conditions for re-opening it); and a **tiered architecture** where the VLM becomes an event-triggered deliberation tier -- which is what finally gives `brain/planner.py` a job. Also settles the goal vocabulary, stop conditions, arbitration and what the sim can test. **Revised 2026-09-06 on three counts** (its 4.8, 4.9 and 1.14): the Jetson was re-checked against what delivery-robot fleets actually run and against a July 2026 NVIDIA repricing that put the Orin Nano Super at $399-480, so Pi-plus-Hailo stands more firmly than before; the part is now a **Hailo-8L in M.2 module form**, because the module survives a Jetson pivot and shares the one PCIe lane with the NVMe, and because the 10H's measured 5.89 tok/s makes a local VLM slower than the cloud call it would replace; and **motion becomes continuous rather than discrete** (1.14); a fourth revision the same day settled the models rather than the parts (4.3.1 the 8L's measured benchmarks, 4.2 the open-vocabulary crop path and the standing-height caveat, 2.8 one mission walked end to end), which makes the tiered architecture mandatory instead of an optimisation, puts the accelerator on the first order, and makes two shipped numbers wrong -- `watchdog_timeout_s: 1.0` and the fixed `min_distance_cm: 20.0`, which is a stopping distance good for only ~0.45 m/s. Its C1-C9 phasing (extended from five 2026-09-06, after walking 2.8's mission against the repo) needs no hardware. **A second series, P1-P4 (its 4.10), is the perception harness: P1 and P2 are BUILT (`brain/perceive.py`, `brain/tiered.py`) and run the real YOLO + CLIP + Opus 4.5 chain against real photographs with no robot and no accelerator -- the twin cannot test the *detector*, by 1.12's design, but a phone on a wheeled rig can. **P2 became startable from the twin on 2026-09-07** (`policy: "tiered"`, and 6.3's four readouts on the Remote brain panel) -- see the row below.** Bill of materials **~$565-628** (2026-09-16: pan-only ST3215 servo, and the 10H option at +$60; was ~$555-620) (3.6, recomputed 2026-09-06 -- the earlier ~$498/~$581 priced the bundled motor driver rather than the recommended Waveshare board, bought an M.2 module with no carrier, and had no servo rail). **Read it before buying anything**, and note its section 5: `HARDWARE-READINESS.md` is now partly wrong. The one thing it asks for *before* hardware day is the Hailo compile loop (its 1.10 item 1): without it the Hailo is a fixed-function part and the IMX500 was cheaper. |
| -- | **The first valid Stage 0 walk** (`blue-bottle-20260907-142454`) | Recorded 2026-09-07 -- 33 frames, camera at floor height on a wheeled rig, target **on the floor**. The re-recording 1.16 #10 has demanded since 2026-09-02, and the first walk not disqualified by its own viewpoint. Two findings, both in `PLAN-onboard-perception.md` 4.10: **(a)** the CLIP threshold is measured -- true positives band +0.025..+0.038, non-target frames top out at +0.004, so **0.02 separates them perfectly and the shipped 0.05 detects none of them** (`brain.perception_match_margin`, then 0.02 in config -- **since superseded: config sets 0.0 and the gate is `perception_match_probability` 0.8**; the module default is deliberately unchanged because the old corpus had handbags at +0.039 against "red backpack", so one threshold does not serve both targets); **(b) 4.2's label gate is losing 11 of 18 true positives** -- at close range YOLO relabels the bottle as a `vase` (once `refrigerator`), so the gate discards exactly the frames where the target fills the view. Forcing the open-vocabulary path recovers 18/18 with zero false positives. `brain.perception_crop_path` makes that measurable; the default stays `auto` until two more walks say otherwise. The full chain ran on it: `found`, **6 paid calls over 31 frames, 1 per 5.17**. |
| P2 (twin) | `policy: "tiered"`, and the readouts that make the architecture watchable -- on the Sim tab **and on the phone walk**, which is the one that matters | Done (2026-09-07), not deployed -- the brain has been deployed nowhere since 2026-09-05 and these models run *in the brain process*, so this is a local two-uvicorn feature by construction. The Remote brain panel's policy picker gains **Tiered**; `control/brain_server.py` wraps `brain/navigate.py`'s cloud `vision_fn` in `brain/tiered.py`'s `TieredVision` and **validates at mission start** -- `ultralytics`/`torch` stay an optional install (`requirements-perception.txt`) and a missing one is a 400 naming the pip command, never a B3.2 vision failure discovered three ticks in. `GET /health` publishes `perception_available` so the panel warns before Start. Four readouts (`PLAN-onboard-perception.md` 6.3): the tri-state, the CLIP **margin** (not the similarity), the detector and encoder by name, and the deliberation counter as **calls and frames** -- 6.3's "single number that makes the whole architecture watchable", comparable to 6.1's measured 4-6x. The mission log names the paid steps `[cloud: <trigger>]`. The detector's *boxes* are deliberately absent: over the twin's FPV they would be boxes on a raycaster render, which 1.12 forbids. **Guide -> Robot view -> "Drive via brain" also carries the policy now** (it hardcoded `policy: "vision"` before), which is the only path where YOLO and CLIP get real pixels -- the Sim tab's tiered mission exercises the loop and never the detector, by 1.12's design. **Run end to end the same day, YOLO -> CLIP -> Opus 4.5** (`python -m tests.demo_replay_mission <walk> "<target>" --policy tiered`, which is new): on `red-backpack-opus-4-5-20260829-214849`, outcome `found`, **4 paid calls over 18 steps -- 1 per 3.5 distinct frames**, all three implementable triggers fired, nothing tuned. That puts 2.4's cost claim inside 6.1's measured 4-6x band *live* for the first time. Three findings in `PLAN-onboard-perception.md` 4.10: `DEFAULT_MATCH_MARGIN` (0.05) is ~2x too high and **has not been changed** -- the corpus is the invalid one and the negative column overlaps on handbags; **the target STRING is a bigger lever than the walk** (`"red backpack"` 13% detected, `"blue bottle"` 0%, bare `"bottle"` 0% -- a colour+noun is worth ~5x the margin of the bare noun); and two defects the run surfaced, both fixed -- `bearing_deg` had never once been a number (no backend publishes `image_width`; the width is now read off the image) and a detected target logged as `not_visible`. |
| P3 | Corpus-wide perception scoring (`control/perception_eval.py`), and 1.11a **reported** | Done (2026-09-08), not deployed. **P3** is the instrument every finding in `PLAN-onboard-perception.md` 4.10/4.11 rests on and it had been written ad hoc three times and lost each time: it reads each walk's adjudicated `labels.json` (**never** `walk.jsonl`), refuses a walk that has none, scores once and sweeps the gate afterwards, and matches two configs on a false-positive budget before reporting either one's recall. It reproduced 4.11's shipped row exactly on first run -- 62/74 at `P>=0.8`, 3 false positives -- which is the only validation a scorer can have. Its new per-walk split is the finding: **100%/97% recall on the two approach walks and 20% on the search walk**, so the corpus-wide 84% is an average over two different problems. **1.11a** (corroborated identity) is now computed, counted and shown on the Remote brain panel and carried in walk data -- and **enforces nothing**, which a test pins: an `unclear` frame passes `target_visible` and `target_reached` through untouched. It stays that way until the two out-of-vocabulary searches 1.11a asks for exist. |
| P6 | The Hailo compile loop (`tools/hailo/`), run against OWLv2 | **RUN 2026-09-09. OWLv2 does NOT compile to a Hailo-8L HEF, and the part decision resolves against the Hailo.** `PLAN-onboard-perception.md` 1.10 item 1 had asked for this loop since 2026-09-04 and it was the one pre-hardware item never started; it answered a $400 question for **$3.20 in 3.1 hours** on an r6i.4xlarge, now terminated. **What it found is not what Hailo's own table predicts.** OWLv2's ViT-B/16 image tower *translates* (44-107s) and *quantizes* (no OOM at 3600 tokens or 1600) -- DFC 3.34 carries a LayerNorm Decomposition pass, Matmul Equalization and MatmulDecompose and uses all three. It fails at **allocation**. Three attempts died on `conv1`, the single Conv in a 575-node graph (the 16x16-stride-16 patch embedding), unmoved by `--image-size 640` or `allocator_param(automatic_reshapes=enabled)`. `--factor-patch` rewrites that conv as two 4x4 convs -- an exact identity, verified against the unmodified model at max |d score| 1.4e-05 with the top-50 patch set unchanged -- and removes it from the error. **What appears instead is the whole transformer body: 73 layernorm and 38 softmax layers, every per-token reduction.** That is an architectural limit of the dataflow design, not a size limit, which is why no smaller input and no flag moved it. Records in `evaluations/hailo/`. **Untouched by this:** YOLO11 n/s/m still compile for the 8L off the shelf (4.3.1) and OWLv2's accuracy still stands (68/68 visible frames, zero false positives) -- those are PyTorch numbers and are why the model is worth a board at all. The loop is one `ec2.sh up` from re-running against a newer DFC, which is the only thing that could reverse this |
| P7 | The whole corpus, on a rented GPU (`evaluations/gpu/`) | **Run 2026-09-11/12. Two A10G instances, 3.7 h, $3.70, torn down.** It exists because of a **scope error**: every row in `PLAN-onboard-perception.md` 4.11 and P5 was scored on FOUR of the corpus's eight labelled walks, and nothing had ever been scored against all 610 frames. **11 configs now are.** Four findings. **fp16 costs no accuracy** -- identical true positives at every budget, gates matching to three decimals, 1.8x faster; that assumption sat under P5, P6 and 4.9 and had never been measured (INT8 still hasn't, and every latency projection assumes it). **OWLv2 reads 82% at 3 FP, not 96% -- but its margin WIDENS**, because the shipped pipeline falls further (85% -> 58%), so the gap goes from 11 points to 24 at 11x the speed. **4.11's central claim is false**: Grounding DINO gets 50% at ZERO false positives where the shipped pipeline gets 8%, so the three-model pipeline does not dominate at every operating point -- with a useful corollary for 1.11a, since a model that is never wrong is a better corroborator than one that is more often right. And **the field was not as covered as this plan assumed** -- HuggingFace's zero-shot list had three untried families; OmDet-Turbo (7%) and LLMDet (18%) both lose, which is a real result for forty cents. Qwen3-VL was deliberately not run: a known upstream perf bug (5262 ms/token vs 77), a `--vlm-max-pixels` flag it silently ignores (**which corrects P5's "at native tiling"**), and a non-separable score. P7b revises the Orin projection from 51 ms to **124 ms** on measured rather than assumed inputs, and finds the bottleneck is **CPU preprocessing, not the model** (36 ms detecting, 229 ms resizing). P7c withdraws the "YOLO is redundant" claim that 51 ms implied, resolves the out-of-vocabulary tracking gap as an **odometry** problem rather than a perception one, and argues `target_reached` off the cloud |
| R0 | Continuous pose + differential-drive kinematics in the sim | Done and **watched** (2026-09-25) -- the first phase proven on the Sim tab by the user on a phone, not just signed off. The twin half is deployed; the robot and brain run locally -- `PLAN-ros-alignment.md` R0, which merges C2 with 1.14's continuous motion. `GridWorld` is no longer a dataclass: float `x`/`y`/`theta` are the state of record and `robot_x`/`robot_y`/`heading` are *views* of it (the getters floor, the setters snap to the cell centre), so every caller that thinks in cells still works and nothing outside `sim/` had to change. `MockRobot` gains `set_wheel_velocity()` / `step()` / `get_wheel_state()` -- **left/right wheel angular velocities, not a twist, on purpose**: `diff_drive_controller` owns that conversion on the real robot, so a sim taking a twist would leave it and its two chassis constants unexercised until hardware day. The verbs are now thin wrappers over that one path and still mean what they meant (one cell, one quarter turn), which is what keeps every step budget and safety threshold in the repo meaningful; what changed is that `turn_left(45)` turns 45 degrees. Constants are `HARDWARE-BOM.md` 4.3's: wheel radius 0.0325m, 1760 counts/rev, and **track width 0.172m as a flagged PLACEHOLDER** -- a test pins that a straight line does not depend on it, so a wrong value can make the sim pivot at the wrong rate and never travel the wrong distance. Collision is `renderer.cast_ray()` and deliberately **not** `get_depth_grid()`'s conservative reduction; see 3.1. **N1's written prediction held exactly** -- three lines changed in `sim/mock_world.py` and nothing in `world/interface.py`, `control/remote_world.py` or the twin, which is the evidence the ROS wall was drawn in the right place before anything stood behind it. `tests/test_continuous_pose.py`. **UI proof shipped 2026-09-25:** a 15° / 45° / 90° turn step under the Sim tab's D-pad, so a non-cardinal heading is reachable from a phone rather than only by curl (`tests/test_ui.py`, confirmed red against a twin that drops the angle). Press-and-hold wheel velocity is left for R4, when the D-pad goes through `twist_mux` |
| R1 | Bearing-sized turns, and synthetic perception in the sim | Built (2026-09-25), **not yet watched** -- `PLAN-ros-alignment.md` 3.3. The P25 A/B found the real defect was the turn's SIZE: every LEFT/RIGHT was the executor's default 90 degrees against a 10-degree centre band, so the tier closed zero distance and flipped on 52 of 60 steps. Now a turn chosen from a measured bearing carries `turn_deg` (clamped 5-90) through `ConstrainedAgent` as `angle` -- including the cloud's turns when a local bearing agrees on the side. Also built: 1.12's synthetic detections (`frame["detections"]`, `FrameReportedPipeline`, chosen by `metadata.source`) -- before this a tiered mission on the Sim tab ran YOLO and CLIP on raycaster renders and could never steer. Through the whole mission path: 4.06 cells closed / 0.6 reversals against quarter turns' -1.23 / 4.8; every clear-line start arrives. **Open:** a detector landing 1 frame in 3 still fails (search turns spin the target away; dead-reckoning anchors a direction, not a point, because perception passes no range) -- `tier_hold_bearing` stays OFF. `tests/test_bearing_turns.py`, `python -m tests.demo_hold_bearing_ab` |
| R1b + stuck | Search that cannot miss; missions end `blocked` instead of pushing into walls | Done on data (2026-09-25) -- `PLAN-ros-alignment.md` 3.4-3.5, the first phases closed under the data-driven rule. **Stuck detection:** `stuck_after` (5) refused FORWARDs in a row ends a mission `blocked` -- jamb starts stop in 8-11 steps instead of 120, and one live mission through the brain API made a single cloud call and stopped. **R1b:** search turns go out at `SCAN_TURN_DEG` 45 (a 90-degree step against a 60-degree view left blind gaps: 86% of search starts found the target, now 100%), and measured bearings steer on `STEER_BAND_DEG` 3 rather than the 10-degree reporting band (all 28 remaining failures had driven 5-6 degrees off the doorway line into a jamb). 171 search starts: 100% found, 100% arrive. Live on `536b13f`, the mission that stopped a metre short now reaches the backpack. **Open:** a 1-in-3 detector still closes only ~1.6 cells (needs a range-anchored goal pose); arrival is still not recognised (P7e); and sim objects are not solid, so the robot drives onto the target and then searches for it |
| R1c | An unreliable detector, and a regression R1b introduced | Done on data (2026-09-25) -- `PLAN-ros-alignment.md` 3.6b. At a realistic 80-90% per-frame detection only 73-85% of missions arrived; the cause was R1b's own 45-degree search step, which silently turned the 8-TURN spin guard into one rotation, so one missed frame forced a FORWARD off the doorway's line. The guard now counts DEGREES (8 quarter turns = 720). Arrival ~98.3-98.6% (+/- 1%) at both 90% and 80% detection over 690 missions each (criterion >= 95%; a first three-seed estimate of 95.2% was noise). Lesson: thresholds counted in steps change meaning when the step does |
| Solid objects | Objects are obstacles to everything that senses or moves | Done on data (2026-09-26) -- `PLAN-ros-alignment.md` 3.9, decided by the user ("objects must be treated as solid to emulate the real world"). Collision, `get_distance()`, the depth grid, the lidar scan and `MockWorld`'s map all treat an object's cell as an obstacle (`renderer.cast_ray(..., solid=)`); the camera still draws objects as billboards, so the golden image is unchanged. The starter house's sofa moved from the robot's start cell to (1, 1). The robot now stops in front of the backpack, never on it; no measurable cost to arrival |
| R2 (read-only) | `GET /wheels`, `GET /scan`, `GET /world/truth` | Done on data (2026-09-25) -- `PLAN-ros-alignment.md` 3.6. `RobotInterface.get_wheel_state()` / `get_scan()` and `WorldInterface.get_truth()`, each with an honest `usable: false` default, implemented by `MockRobot` / `MockWorld`, served by `robot/server.py`, read by `RemoteRobot` / `RemoteWorld`. **The scan is at `/scan`, not the plan's `/world/scan`**: it is the robot's own reading, so BODY state by section 2's rule. Every beam equals `renderer.cast_ray()`; truth equals the pose in the sim until R5 parts them. `tests/test_r2_routes.py` |
| R2b | `POST /wheels` -- a standing wheel-velocity command, the first way to move without a verb | Done on data (2026-09-26), not deployed -- `PLAN-ros-alignment.md` 3.10, six criteria written first, all met and each confirmed red against a mutation. `robot/server.py` runs a 20 Hz control loop that re-vets the standing command through `SafetyController.vet_wheel_velocity()` every period: it zeroes forward speed below `min_distance_cm` (stops at 19.5 cm; 0.0 cm without the clamp) and **never clamps rotation**, so a robot facing a wall can pivot away. **Reverse is now checked against the scan's rear beams on every path, D-pad REVERSE included** (user decision). A new driver `ros` ranks with the brain, below the D-pad, and **the autonomous rank is now exclusive while held** -- before this the server let equal ranks interleave. A backend without wheels refuses with `unsupported`. `tests/test_wheels_command.py` |
| P7e (first half) | Arrival recognised: a mission that reaches its target ends `found` | Done on data (2026-09-26), not deployed -- `PLAN-ros-alignment.md` 3.11, decided by the user as the rule the CAR runs, not a sim-only stand-in. `brain/arrival.py`, applied in `MissionAgent._review_scene()` between perception and decision: the target detected, within the 3-degree steering band, and **the lidar** (`get_scan()`, median of five beams at the bearing -- never the detector's distance) within 0.40 m, two frames running; then `STOP`, `target_reached`, `found`. Refuses to judge with no scan (teleop, replay), a panned camera, or no local perception (rule-based, cloud-only vision). 69/69 arrivals end `found` at perfect detection and 676/678 at 90% and 80%, none beyond 0.386 m; **0/69 without it** -- every one used to end `blocked` or `max_steps`. The first version read the NEAREST beam and declared `found` 95 cm out against a door jamb; criterion 2 caught it. `status.arrival` carries the range and streak. The other half of P7e (the steer-over-hold precedence on a held cloud `STOP`) no longer matters on this path and is left alone. `tests/test_arrival.py` |
| R3 | The URDF and TF tree, in the first ROS container | Done on data, one criterion FAILED and recorded (2026-09-26), not deployed -- `PLAN-ros-alignment.md` 3.12. `service/slam/` now holds ROS 2 **Humble** (JetPack 6 is Ubuntu 22.04) in one container; `picar_description`'s xacro puts every dimension in one block, `[BOM]` or flagged `[PLACEHOLDER]`. `check_urdf` passes; wheel radius and separation are one number across the xacro, `controllers.yaml` and `sim/mock_robot.py` (always-run test); 15 tf2 lookups match numpy FK within 1 mm / 0.1 deg. **Criterion 4 failed:** "pan + in-frame bearing" is 4.6 deg out at 1 m with the pan axis 8 cm ahead of `base_link` -- but within 0.75 deg when the camera is centred and the target inside the steering band, which is all the tier and `brain/arrival.py` use. Pinned as a strict xfail. **Nothing may treat a panned bearing as body-relative** until it is composed through TF with a range, or the pan axis moves over the rotation centre. The sim renders from the robot's centre, so it cannot show this error. `tests/test_urdf.py` (live half skips without a container) |
| R4 | `picar_sim_hardware`, `twist_mux`, and ONE writer to the wheels | Done on data (2026-09-26), not deployed, **off by default** -- `PLAN-ros-alignment.md` 3.13, all eight criteria met. Under `drive: ros` (`ROBOT_DRIVE=ros`), `robot/ros_drive.py` turns each `/action` verb into twists closed on the wheel encoders and sends them to the container's bridge (`picar_bridge`, HTTP :8090) -> `twist_mux` (teleop 100 > brain 50) -> `diff_drive_controller` -> `picar_sim_hardware` (C++, `hardware_interface::SystemInterface`) -> `POST /wheels`, which then accepts only driver `ros`. M4's `/action` arbitration is unchanged, so a D-pad tap still ends a mission `preempted`. Verbs land within 4.4 mm / 0.64 deg live; a tiered mission ended `found` in 7 steps with every move through ROS. **Two things learned:** a proportional verb ramp over the chain's 40-150 ms of jitter overshot a 45-degree turn to 59-74 degrees until retuned with a signed settle pass; and the plugin returning ERROR on a robot-server restart silently deactivated it for good -- it now keeps trying. The default stays `direct`, so the twin never depends on Docker. `tests/test_ros_drive.py` (always), `tests/test_ros_chain_live.py` (skips without the stack) |
| R5 | `slam_toolbox`, and the error only a sim can measure | Done on data (2026-09-26), not deployed, **off by default**, criteria 2 and 3 FAILED on their tight bars and recorded -- `PLAN-ros-alignment.md` 3.14, eighteen laps. `world/ros_world.py` (`WORLD_MODE=ros`) is the `WorldInterface` over SLAM, converting ROS's frame on its own side of the wall and anchoring SLAM's frame to the house with the truth at the session's START (sim-only). `GET /world/error` and the twin's map (truth ghost + "SLAM error ... · odometry alone ...") show it. With the right encoder 3% long, **odometry ends up to 99 cm / 54 deg off while SLAM ends within 1-4.5 cm**, and SLAM's map is the house on every lap (96-100% of occupied cells within 10 cm of a true surface). Failed: at-rest position within 5 cm without drift on 2 of 9 laps (4.0-8.2 cm; cause not established), heading within 3 deg with drift on 4 of 9. Two things learned: errors sampled WHILE MOVING measure the pose's ~150 ms latency, not SLAM; and anchoring at first contact read "0.0 cm" on the twin after the robot had driven. `sim.odom_drift` / `SIM_ODOM_DRIFT` make encoders misreport. `tests/test_ros_world.py`, `tests/test_slam_live.py`, `tests/demo_slam_lap.py` |
| R6 | nav2 + `collision_monitor`, goals on the SLAM map | Done on data (2026-09-26), not deployed, **off by default**, on the SCALED house -- `PLAN-ros-alignment.md` 3.15. `POST /world/goal` (house frame, converted by `world/ros_world.py`) -> nav2 (NavFn, Regulated Pure Pursuit) -> `twist_mux` -> `collision_monitor` (APPROACH, not stop) -> the wheels; a D-pad twist cancels the goal. Two runs on the final image: **6/6 and 6/6**, ending 9-13 cm from goal, never nearer than 16.5 cm to a surface, 0.76-0.81 command reversals per metre, an unreachable goal aborting in 19-24 s stopped, a tap cancelling in 0.04-0.05 s, and `robot/safety.py` never clamping. Decided on the way: the collars run in SERIES with `safety.py` last; the tier keeps steering by verbs for now. **Five real findings**, all in 3.15: the starter house's 30 cm doors cannot host this chassis (0/6-5/6), so `sim/maps/scaled_house.py` (`SIM_MAP=scaled_house`, 90 cm doors); map before navigating; `collision_monitor`'s stop polygon froze the robot against a jamb; `slam_toolbox`'s apt release lacks `restamp_tf` (built from a pinned commit); and **an ABBA deadlock inside ROS 2 Humble's tf2** froze nav2's costmap TF listeners -- the controller then "reached" every goal instantly. First blamed on scan stamps (wrong: a debugger on the frozen process showed the lock cycle); fixed upstream in tf2/tf2_ros 0.25.24, which apt does not ship yet, so the image builds it from the tag. The image now runs Cyclone DDS. `tests/demo_nav_goals.py`, `tests/test_nav_live.py` |
| R7 | The ESP32 motor board faked on a serial line; the real robot's motor backend | Done on data (2026-09-26) -- `PLAN-ros-alignment.md` 3.16. Read from the firmware SOURCE first: the heartbeat does stop the motors (was believed, now verified); **`T=1` is open-loop PWM in the mode the BOM's example selects** (closed-loop speed needs `mainType` 3 with this chassis' constants -- a firmware change); the `1001` frame has wheel SPEEDS, not counts. `sim/fake_esp32.py` is that firmware on a pty; `robot/hardware_robot.py` is a `RobotInterface` backend over it (`mode: hardware`, `ROBOT_SERIAL`; `SIM_MOTOR_BOARD=fake` for the sim), passing all 22 contract tests, and R4's live suite passes over the serial line. **The seam moved:** the serial port belongs to this backend and ROS keeps its HTTP plugin, so `safety.py` stays in every path and **hardware day is a config change -- `picar_hardware` is not written**. `tests/test_fake_esp32.py` |
| Home | The user's own house, first floor, as a simulation | Built 2026-09-26 -- `sim/maps/home_first_floor.py` (`SIM_MAP=home_first_floor`). The OUTSIDE walls and the garage are the measured sketch in the home's 2012 appraisal (p. 29), in feet, closing to within 2% of its 1,483 ft^2; **the interior walls, doorways, start and target are INFERRED from where the sketch prints room names and are marked PROVISIONAL** -- to be corrected by the user, then furniture added as solid objects. No address in the file. nav2 toured all eight rooms (58 m, 9-12 cm from each goal, never nearer than 15 cm to a wall) once two fixes landed: the sim's lidar now reaches the RPLidar C1's 12 m (it was the camera's 4.2 m, and SLAM mapped almost nothing in a 16 m house), and the tf2 deadlock fix. The reactive search policies do NOT find the backpack here in 200 steps -- a real house needs a map, which is nav2's job. **Furnished the same day**, at the user's request with typical furniture: 450 solid cells (sofas, counters, the island, appliances, a car), tables modelled as four LEGS because a floor robot drives under them; the user's correction built (kitchen -> hall -> garage, pantry left, laundry right); the staircase is a guess in the foyer. Furnished tour: **8 of 9 rooms**, 62 m, 9-13 cm from each goal, never nearer than 16.8 cm to anything. The dining room failed: NavFn plans for a circle and threaded a gap between chairs that Regulated Pure Pursuit, checking the real rectangle, refused 101 times -- a known planner/controller footprint mismatch, not yet fixed. `tests/test_home_map.py` pins the measured parts and that every room is reachable around the furniture |
| Wall costs | The ROS wall's costs, made visible: duplicate/bridge linters, the brain on ROS topics, HTTP at 20 Hz measured | Done on data (2026-09-27) -- `PLAN-ros-alignment.md` 3.17. `tests/test_wall_linters.py` (static): a registry of the ten concepts defined on BOTH sides of the wall, each checked for agreement, a detector for UNLISTED copied physical constants, and budgets (`MAX_DUPLICATES` 10, `MAX_BRIDGE_ROUTES` 13) plus a ban on generic pass-through routes -- every rule confirmed red against a mutation. `picar_bridge/brain_view.py`: the bridge POLLS the brain's `/mission/status` and publishes `/brain/status`, `/diagnostics`, `/brain/markers`; `foxglove_bridge` is in the image, **read-only** (only `connectionGraph`), on 127.0.0.1:8765. HTTP: a bare app over the same Docker hop holds 200 Hz at p99 1.6-4 ms; the robot server's 20-34 ms p99 tail is the SIMULATOR (scan ray casting ~13 ms, render ~35 ms) sharing its process -- re-measure on the Jetson (`tests/test_http_rate_live.py`). An audit of the ROS tests closed three gaps: `/world/goal` + `/world/error` and RosWorld's goal conversion had no offline tests (`tests/test_ros_goals.py`), the bridge's scan/quaternion conversion was live-only (now `picar_bridge/convert.py`, `tests/test_bridge_convert.py`), and **`test_slam_live.py` ran its starter-house lap in whatever house the server was in** -- `/health` now reports `sim_map` and the live suites ask the server. `picar_sim_hardware` (C++) still has no unit tests of its own. **Two SAFETY findings, recorded here and FIXED the same day in 3.18 (row below):** in the furnished home a standing twist at an oblique approach drove from 42 cm to **3.0 cm** past the 20 cm clamp (the +/-15.4 deg path cone is narrower than the chassis inside ~30 cm, or rays slip a diagonal cell corner -- not yet established); and R4's starter-house wall stop reads 18.0 cm in 3 of 5 runs on today's image AND the previous commit's (suspected: the wheel loop vets once, then integrates the real elapsed `dt`) -- now a non-strict xfail. Also fixed: a clean checkout could not build the ROS image (an empty untracked `config/` in `picar_description`'s install rule) |
| 3.18 | The two safety findings, fixed on data | Done on data (2026-09-27), not deployed -- `PLAN-ros-alignment.md` 3.18, nine criteria written first, all red on `3ab3058`, all met. **Oblique approach:** `robot/safety.py` now also vets a move against the chassis' SWEPT CORRIDOR off the 360-degree scan (URDF rectangle, 3 cm side margin, body frame), in series with the cone, forward and reverse; `sim/grid_world.py` collides with the rectangle. Judged on ground truth (`tests/footprint_sweep.py`): 5760 runs, 0 under 18 cm of travel-to-contact and 0 contacts (unfixed: 25 and 9 of 1440, worst 0.0), progress 98.8%. **The flaky 18.0 cm was NOT a stalled loop** -- a `/health` `wheel_loop` readout showed zero late ticks -- it was **a camera left panned 90 degrees** by a preempted mission, and the depth-grid cone is cast along the camera: the robot drove guarded by a cone looking sideways (unfixed, panned right, it drove to 3.1 cm). The grid now publishes `pan_deg` and the cone picks zones by BODY bearing; no scan and a camera facing away refuses FORWARD. Live in suite order: 5/5 (was 2/5 failing). Also: `get_scan(max_range_m=)` hint + an exact grid-traversal ray (`renderer.cast_ray_exact`) for the safety scan, 13 ms -> 0.4 ms, and it measured the march slipping past diagonal corners on 0.1-0.2% of beams. `tests/test_footprint_safety.py`, `tests/test_pan_safety.py`, `python -m tests.demo_footprint_sweep` |
| 3.19 | Pivots can no longer swing a corner into furniture | Done on data (2026-09-27), not deployed -- `PLAN-ros-alignment.md` 3.19. A rectangle's corners reach 15.1 cm against 9.9 cm sides, so "pivots within its own footprint" was false. `robot/safety.py` now scales a turn that would close within 1.2 cm of a scan return (never one that opens the gap -- a robot against furniture can always turn away), and `GridWorld.rotate()` stops at contact. 360 pivots started with something inside the turning circle: 0 within 1 cm (was 120/120), 83/83 with room turned into it (worst 2.7 degrees short). The free-angle metric was corrected after its first run (it measured room to contact, contradicting the no-contact bar) and zeroing a turn was replaced by scaling it; both recorded. `tests/test_pivot_safety.py` |
| 3.20 | A mission starts with the camera centred | Done on data (2026-09-27), not deployed -- `PLAN-ros-alignment.md` 3.20. A mission that ended mid-peek left the camera panned and the next policy's first frame, depth grid and scene were cast 90 degrees off its heading. `MissionRunner` now centres it through its gate on the first tick, before the first decision, uncounted. Pinned against the pre-change 83-step frontier trace (`tests/data/frontier_trace_centred.json`), unchanged. Live, pan sampled through the first step: -90, 0, -90, +90 (was -90, +90). `tests/test_camera_centred_start.py` |
| 3.23 | A nav2 goal is an autonomous driver | Done on data (2026-09-27), not deployed -- `PLAN-ros-alignment.md` 3.23, found by the doc review (`docs-review/REPORT.md` V4). `POST /world/goal` used to skip M4 arbitration entirely, and nav2 drives on `cmd_vel/nav` at the same twist_mux priority as the brain, so a goal during a mission was refused nowhere and ordered by nothing. Now the goal arbitrates as driver `ros` (refused `preempted` while the brain or a person holds the robot), and while a goal is pending or active every other autonomous `/action` is refused `preempted`; a person never is. `tests/test_goal_arbitration.py` (criteria 1-2 red first). |
| 3.24 | `drive: ros` as the car's default: gates G1-G4 | **G1-G3 met on data (2026-09-30), G4 needs the Jetson** -- `PLAN-ros-alignment.md` 3.24. `direct` stays as the sim/test default and the car's fallback, not a second equal path. **G1:** the live ROS chain 20/20 consecutive (was up to 5/8 failing) -- the bridge polls over kept-open connections (Docker Desktop stalled ~1 new connection in 10), and the chain suite runs in the scaled house (the starter house's 30 cm door refused the UGV chassis on ~0.5 deg of heading error). **G2:** the ROS verb path meets 3.18/3.19's ground-truth bars (`tests/ros_verb_sweep.py`): the wheel vet looks ahead one period (stops AT the line), a verb the vet held is a refusal as in direct mode, the executor settles to its tolerance (clear turns within 0.64 deg: 80% -> 100%). **G3 (user decision: only a person drives on the fallback):** ROS's pulse is the plugin's 20 Hz `/wheels`; 0.5 s silent = down; a person's verbs then run direct mode's guarded path, autonomy is refused `ros_unavailable` and the mission ends `failed` -- live: ended 2.04 s after a container kill, D-pad drove 0.36 s after it, back up at the first post. `tests/test_ros_verb_safety.py`, `tests/test_ros_fallback.py`, `tests/test_bridge_keepalive.py` |
| 7, 8, 10, 11 | Pi setup, physical assembly, real camera streaming, hardware swap-in | Blocked on buying hardware -- by design, per the simulation-first plan. Nothing to do here yet. **The chassis is no longer a PiCar-X** -- see the row above. |
| extra | Web-based digital twin | Done and deployed. **Since 2026-09-05 the page is static on S3 + CloudFront** (`service/static/sync.sh`) and the robot server runs locally behind `service/tunnel/`; the ECS/NLB/ALB arrangement below was deleted. Was: `web-twin/index.html` + `robot/server.py` on ECS Fargate, `service/twin/`, `cloudformation/twin.yaml`) -- reachable from a phone on any network, sharing the vision service's NLB/ALB on port 80 via path-based routing (a ListenerRule matching the twin's exact route set). Verified end-to-end from an actual phone on cellular data, not just curl. |
| extra | Cloud photo-analysis endpoint | Done and deployed -- **since 2026-09-05 as a Lambda behind API Gateway and CloudFront** (`cloudformation/serverless.yaml`, `service/lambda/`); the ECS/NLB/ALB below were deleted. Was: `service/vision_analyze/` on ECS Fargate, behind an NLB -> internal ALB, calling Amazon Bedrock for vision inference). Was originally built on Lambda + API Gateway; both were deleted after an account-level restriction made them permanently unreachable publicly -- see README.md's "History: why not Lambda?" |

---

## 3b. Current state, in one place (2026-09-28)

The dated narrative that used to follow the table -- the P-series, the
Hailo/Jetson reversals, the handoffs -- is in
`docs/archive/CLAUDE-history-2026-09.md`, verbatim. What is still true and
still load-bearing:

* **Latest session handoff: `HANDOFF-2026-09-30.md`** -- the robot base.
  The ROSOrin order was cancelled; the **UGV Rover was ORDERED 2026-09-30**
  (Amazon, ~$730 delivered); the separate Jetson battery is still to buy
  (`JETSON-BOM.md` 9.5). Its section 5 is the open work, in
  order.

* **Hardware: CLOSED 2026-09-19 -- the board is a Jetson Orin Nano Super,
  ~$944 all-in (`JETSON-BOM.md`; parts, wiring and protocol in
  `HARDWARE-BOM.md`, read its editor's note first).** Stated by the user.
  The Hailo path is history: do not re-open it and do not spend on a Hailo
  compile run. The parts below the Jetson now come from the robot base
  (ordered; see below): on the UGV Rover kit, the **ROS
  Driver** board (closed loop from the factory, encoder odometry `odl`/`odr`
  to the host, 660 pulses/rev), a D500 lidar, an OAK-D Lite and a pan-tilt
  -- not the General Driver + RPLidar C1 + IMX219 of the 2026-09-19 build,
  which `HARDWARE-BOM.md` still describes. The software assumes JetPack 6.x /
  Ubuntu 22.04 / ROS 2 Humble. **The dev kit was ORDERED 2026-09-27**
  (Amazon, $399, arriving Oct 14-26) -- it is `JETSON-BOM.md` section 1's
  "buy regardless" line. **The chassis was ORDERED 2026-09-30** -- the
  Waveshare UGV Rover PT Jetson Orin ROS2 Kit Acce, Amazon, ~$730
  delivered, expected Oct 19 - Nov 11, 30-day return. A
  Hiwonder ROSOrin ordered 09-29 was cancelled 09-30: Hiwonder
  confirmed its board sends **no encoder data** to the host, its firmware
  is proprietary, and its Jetson port cannot sustain 25 W. The Rover
  needs **a separate Jetson battery** (not yet bought) -- Waveshare confirmed the ROS Driver
  board (closed loop, encoder odometry to the host), 660 pulses/rev and
  ~5 A continuous. Runner-up: the Cobra Flex (no IMU, 9.6). Record:
  `JETSON-BOM.md` section 9. **Concepts
  (encoders, firmware, vendor protocols vs ROS 2/DDS, power budgets):
  `GUIDE-robot-base.md`.**
* **Perception, shipped:** `brain/perceive.py` defaults to
  `yoloe-11s-seg.pt` -> CLIP, `low_confidence` crops, 16 crops/frame, gate
  P >= 0.8, no floor mask -- 82% at 3 FP, 139 ms on laptop CPU, on the
  1234-frame corpus of P23/P24. The labelled corpus has grown since and
  `perception_eval score` scores every labelled walk, so pin the frame set
  before comparing. **Never merge GPU and CPU rows** (P24). The next
  latency work is preprocessing (P7b), not another model.
  `PLAN-onboard-perception.md` has the record.
* **Tiered phone walks still end `max_steps` when they arrive** (P7e): the
  arrival rule (`brain/arrival.py`) reads the lidar scan, so it works on
  `MockRobot` and refuses to judge on a phone walk. Do not read a tiered
  phone walk's outcome as a navigation result.
* **The ROS stack** (R3-R7, 3.17-3.23) is built, off by default, and run as
  `service/slam/README.md` describes. nav2 is judged on
  `SIM_MAP=scaled_house`. A nav2 goal is an autonomous driver (3.23).
* **The world map is DISCOVERED, not copied** (N1): `MockWorld` casts a
  360-degree ring from wherever the robot stands. `world.mode` must track
  `mode` -- the factory refuses `world: sim` for a robot with no grid, which
  is why `service/tunnel/run.sh` sets `WORLD_MODE`.
* **`brain/goal_pose.py`** (P25) is wired into `brain/tiered.py`;
  `tier_hold_bearing` stays OFF (R1: a 1-in-3 detector needs a
  range-anchored goal pose).
* **Two Pythons disagree about the suite** (`.venv` FastAPI 0.141, system
  Anaconda 0.136). `pytest` from `.venv` is the one to trust.

**Next up:** the open questions in `PLAN-ros-alignment.md` section 6 --
item 6 (a search that uses the map, saving it, and the S3 backup) was
decided by the user on 2026-09-27 -- and the remaining fixes in
`docs-review/REPORT.md` section 7.

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
│   ├── ros_drive.py           R4: under `drive: ros`, verbs become twists sent
│   │                           through the ROS container, closed on the encoders
│   ├── hardware_robot.py      R7: the real robot's motors -- the ESP32 driver
│   │                           board over serial, as a RobotInterface backend
│   ├── safety.py              local safety layer; can veto any action, sim or real
│   └── server.py              FastAPI Wi-Fi control API (Phase 9), CORS-enabled,
│                               require_secret() gate once deployed publicly,
│                               serves web-twin/index.html at GET /
│
├── world/                  WORLD state -- what is true about the HOUSE, as
│   │                        opposed to about the body. RobotInterface's
│   │                        sibling and deliberately the same shape
│   │                        (PLAN-mapping.md N1, 2026-09-19)
│   ├── interface.py         WorldInterface -- get_pose() and get_map(),
│   │                         both with honest all-unusable defaults, so
│   │                         adding this broke no backend. Holds the
│   │                         occupancy grid's TRI-STATE (free / occupied
│   │                         / UNKNOWN), which is M3's argument one level
│   │                         up: unmapped must not look like empty floor.
│   │                         No TF, no quaternions, no ROS message types
│   │                         -- the contract is ours and ROS converts on
│   │                         its own side of the wall
│   ├── ros_world.py         R5/R6: the world from slam_toolbox + nav2 goals,
│   │                         through the ROS container's bridge
│   └── factory.py           picks the world backend from config/robot.yaml's
│                             `world:` block (config ships `sim`; `ros` is
│                             SLAM via ros_world.py; the code default
│                             `none` is NullWorld -- a NAMED configuration
│                             rather than a fallback)
│
├── brain/                  reasoning, hardware-agnostic. (Labelled the "MacBook"
│                            role by the original build plan -- that placement is
│                            being revisited: see PLAN-brain-relocation.md)
│   ├── vision.py             Vision LLM scene understanding (Claude API)
│   ├── agent.py               ConstrainedAgent / MissionAgent / ObjectSearchAgent
│   │                           -- the free rule-based policy. Pose from the
│   │                           world, scene from the depth grid; no cells
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
│   ├── goal_pose.py          P7c item 2 / P25 -- a sighting anchored in
│   │                           the odom frame, so a bearing survives the
│   │                           robot turning. The repair for a command
│   │                           that changes every frame: anchor once,
│   │                           recompute from odometry, let re-detection
│   │                           correct DRIFT rather than supply the
│   │                           answer. Degrades honestly -- with a range
│   │                           it holds a POINT, without one a DIRECTION
│   │                           (exact under rotation, useless under
│   │                           translation), and `is_point` says which.
│   │                           BUILT and wired into brain/tiered.py
│   │                           (tier_hold_bearing stays OFF -- R1)
│   ├── perceive_lab.py       candidate perception backends that are NOT
│   │                           parts: Grounding DINO, OWLv2, YOLO-World and
│   │                           SAM, behind the same Detector /
│   │                           RegionProposer Protocols. They exist to
│   │                           answer "would a Jetson buy anything" off the
│   │                           robot, which 4.11 left open. None can run on
│   │                           a Hailo -- that is the point
│   ├── arrival.py            P7e's first half: `found` when the target is
│   │                           in the steering band and the LIDAR (never the
│   │                           detector) reads it within 0.40 m, two frames
│   │                           running. Refuses to judge with no scan
│   │                           (PLAN-ros-alignment.md 3.11)
│   └── planner.py             NOT YET BUILT -- room-level planning over
│                               MissionMemory.as_context(); see gap table above
│
├── sim/                     grid-world simulator (Phase 0.5)
│   ├── grid_world.py
│   ├── mock_robot.py          implements RobotInterface against grid_world
│   ├── mock_world.py          the WORLD half of the simulator (N1) --
│   │                           WorldInterface against the same GridWorld
│   │                           mock_robot.py drives. The house is
│   │                           DISCOVERED: a 360-degree ring from
│   │                           wherever the robot stands, everything
│   │                           behind a wall left unknown. Copying the
│   │                           layout would have been three lines and
│   │                           would have drawn a finished house at
│   │                           mission start, which no mapper does
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
│   ├── fake_esp32.py          R7: the ESP32 driver board's firmware, on a pty,
│   │                           turning a sim body's wheels (SIM_MOTOR_BOARD=fake)
│   └── maps/                  SIM_MAP picks one (sim/maps/__init__.py):
│       ├── starter_house.py   the original, 30 cm doors -- too narrow for nav2
│       ├── scaled_house.py    R6: real proportions, 90 cm doors
│       └── home_first_floor.py  the user's own house (appraisal sketch; interior PROVISIONAL)
│
├── control/                 the brain as a service (phases B0-B3). Imports no
│   │                        backend and no simulator -- the robot is only ever
│   │                        an HTTP client target
│   ├── remote_robot.py       RemoteRobot -- RobotInterface over HTTP (B0)
│   ├── remote_world.py       RemoteWorld -- WorldInterface over HTTP (N1),
│   │                          its sibling. The CONSUMER side of the ROS
│   │                          wall: it reads JSON and contains no hint
│   │                          that ROS exists. When N6 puts slam_toolbox
│   │                          behind /world/map, nothing here changes --
│   │                          which is the test of whether (b+) was built
│   │                          or whether (c) arrived wearing its clothes
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
│   ├── target_probe.py       pre-flight on a candidate target STRING,
│   │                           before anyone walks a rig. The string is a
│   │                           first-class variable (7x on the shoes walk)
│   │                           and this rejects a bad one in 30 seconds.
│   │                           Read TWO numbers: a high firing rate means
│   │                           it cannot discriminate ("a black dumbbell",
│   │                           22% of random frames at P 0.998), and a
│   │                           ZERO rate with a 0.000 peak means the prompt
│   │                           is INERT -- the detector never grounds it,
│   │                           which looks like specificity and is worth
│   │                           nothing to a falsifier
│   ├── label_assist.py       proposes labels.candidate.json so a recorded
│   │                           walk becomes scorable -- NEVER labels.json,
│   │                           and `adjudicated` stays empty because that
│   │                           field means a human looked. The proposer is
│   │                           deliberately NOT the shipped detector: a
│   │                           labeller sharing the model under test marks
│   │                           its own homework
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
│   ├── admin_server.py       its API: list / view / replay / delete walks
│   ├── metrics_client.py     one metrics row per mission, shipped to the
│   │   metrics_routes.py      walks service; can never fail a mission.
│   │   metrics.html/.js       /metrics is the observability dashboard
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
│   │                          path for the DFC) and its teardown.
│   │                          NOTE setup_host.sh builds the DFC
│   │                          environment BY HAND, and P15 traced
│   │                          P14's wrong conclusion to exactly that
│   │                          -- prefer zoo_probe.sh's container
│   ├── phase2_matrix.sh      P16 -- the tier on a 10H: YOLO-World
│   │   zoo_compile_matrix.sh  QAT, then OWLv2/CLIP/SegFormer
│   │                          allocation. Accuracy rows and
│   │                          allocation rows are kept apart on
│   │                          purpose: a HEF is not a recall number
│   ├── zoo_probe.sh          P15 -- the vendor's AI Software Suite
│   │   zoo_matrix.sh          container, driven over SSM: does DFC
│   │   zoo_rawparse.py        5.x parse real models on hailo10h, or
│   │                          did we fail? Removes US from the
│   │                          experiment one variable at a time.
│   │                          The matrix is its own uploaded FILE
│   │                          because the first version lived in a
│   │                          nested heredoc and reported a harness
│   │                          bug as a result -- the P11 hazard
│   └── README.md             read this before running any of it
│
├── config/robot.yaml         mode (sim/hardware), safety thresholds, CORS origins,
│                            and the `brain:` block (robot_url, failsafe budgets)
│
├── tests/                    ~1450 tests (`pytest --collect-only` for today's
│                              count), 93% line coverage of brain/,
│                              control/, robot/ and sim/ (incl. test_robot_contract.py's
│                              backend-agnostic conformance suite [S1+S2+M2],
│                              six backends,
│                              test_sensors.py [S5],
│                              test_depth_veto.py [M3],
│                              test_authority.py [M4],
│                              test_health.py [M5],
│                              test_world_contract.py + test_ros_containment.py
│                              (N1 -- WorldInterface's shape, the body/world
│                              split pinned against RobotInterface in BOTH
│                              directions, and the rule that nothing outside
│                              service/slam/ may import rclpy, written before
│                              any ROS exists),
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
│                              + runnable (non-automated) demo_*/manual_* scripts
│
├── service/vision_analyze/   photo -> vision analysis (cloud). Deployed as a
│                              Lambda since 2026-09-05 (service/lambda/)
│   ├── app.py                 FastAPI app -- /health, /analyze, /describe,
│   │                           /navigate, /guidance
│   ├── vision_core.py         calls Amazon Bedrock (Claude, Converse API)
│   ├── rooms_core.py          identify_room() -- same logic as brain/rooms.py
│   ├── tests/                 app.py's own suite -- routing,
│   │                           validation, decode/size/error handling, all
│   │                           vision_core.* calls mocked. Run separately:
│   │                           `pytest service/vision_analyze/tests/ -v`
│   │                           (see section 6 for why it's not swept into
│   │                           the top-level `tests/` package)
│   └── requirements.txt, Dockerfile
│
├── service/slam/              the ROS 2 container (Humble) -- the ONLY place
│   │                          rclpy/ROS exists (tests/test_ros_containment.py).
│   │                          R3/R4: picar_description (URDF), picar_sim_hardware
│   │                          (C++ ros2_control plugin over /wheels), picar_bridge
│   │                          (HTTP :8090 -> twist_mux, /scan, tf lookups) and
│   │                          picar_bringup. Used only under drive: ros;
│   │                          service/slam/README.md: how to run it, check it,
│   │                          and what its failures look like.
│   │                          picar_bridge/brain_view.py + convert.py are plain
│   │                          Python (unit-tested on a laptop); foxglove_bridge
│   │                          is read-only on 127.0.0.1:8765 (3.17)
│
├── service/lambda/            build.sh: the two Lambda zips (vision, walks)
│                              for cloudformation/serverless.yaml. Prints the
│                              deploy command -- pass BOTH shared secrets
├── service/static/            sync.sh + assets.json: the twin and console
│                              to the S3 static bucket, CloudFront invalidated
├── service/admin/, service/brain/   ECS images for stacks DELETED 2026-09-05
├── service/tunnel/            reaching the LOCAL robot + brain from the
│   ├── proxy.py               DEPLOYED twin. One ngrok free-tier domain
│   └── run.sh                  serves both, split by path: /brain/* to the
│                               brain (ROUTE_PREFIX=/brain), the rest to the
│                               robot. Needed because policy: "tiered" loads
│                               YOLO + CLIP into the brain process, which is
│                               why the brain cannot be deployed at all
│
├── service/twin/              ECS Fargate image for robot/server.py + the twin
│                               (the stack was DELETED 2026-09-05; kept for B5)
│   ├── Dockerfile              built from the REPO ROOT (needs real robot/, sim/,
│   │                           config/ -- not dependency-light copies)
│   └── requirements.txt
│
├── cloudformation/            IaC. LIVE since 2026-09-05: serverless.yaml
│                               (CloudFront + S3 static + API Gateway + Lambda),
│                               recordings-s3.yaml, deploy-bucket.yaml. The rest
│                               (network, service, twin, brain, admin, teleop-*,
│                               recordings, cdn) are DELETED stacks, kept as history
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
├── docs/archive/CLAUDE-history-2026-09.md
│                              the dated narrative moved out of this file
│                              2026-09-28, verbatim (P-series, Hailo/Jetson
│                              reversals, Stage 0 findings on the deleted corpus)
├── docs-review/REPORT.md       the 2026-09-27 documentation review: scores,
│                              verified mismatches, and the fix list
├── FEATURES.md                 every UI feature (all tabs), how each
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
├── PLAN-ros-alignment.md      **the governing plan since 2026-09-25** -- ROS 2
│                               adopted properly (nav2, slam_toolbox,
│                               ros2_control, twist_mux), R0-R7 and 3.17-3.23
│                               built on data. Supersedes parts of
│                               PLAN-mapping.md
├── PLAN-mapping.md            map the house while searching it. N1 BUILT;
│                               N6 became R5+R6; superseded in part by
│                               PLAN-ros-alignment.md. N1-N7. Mapping being the point is the
│                               stated trigger in PLAN-onboard-perception
│                               3.3, so ROS 2 enters the project as ONE
│                               service behind an HTTP wall ((b+)), never
│                               near brain/ or RobotInterface. N1-N4 need
│                               no hardware. Its section 2 lists what is
│                               already decided (1.5, 1.6, 3.4) and must
│                               be implemented rather than re-argued
├── PLAN-microduck-transplants.md
│                               twelve designs borrowed from Pollen Robotics'
│                               Microduck -- a depth sensor instead of asking
│                               the model how far, plus refusal reasons, driver
│                               arbitration, a health verdict and a rollback.
│                               M1-M5 BUILT (2026-09-03, not deployed), M6-M12
│                               proposed; seven need no hardware
├── BOM-COMPARISON.md          Pi 5 + Hailo-8L vs Jetson Orin Nano Super,
│                               like for like at 2026-09-17 prices. The
│                               delta is ~$86 and has been stable across
│                               four passes. Read section 4 before quoting
│                               it: the Pi's accelerator line is the only
│                               Hailo form still in stock, and it costs the
│                               NVMe. **The decision is made (Jetson,
│                               2026-09-19)**; this is the price record
├── HARDWARE-BOM.md            the Jetson BOM as PRICED, 2026-09-17 -- exact
│                               part numbers, vendor plan, bring-up order,
│                               power budget, and the ESP32 driver board's
│                               JSON protocol (which is RobotInterface's
│                               shape on the hardware side). Researched by
│                               Claude Cowork; filed verbatim under an
│                               editor's note listing four corrections.
│                               Read its note first (corrections 5-6,
│                               2026-09-27: the motor protocol and JetPack)
├── PLAN-onboard-perception.md  what runs on the car itself -- and the hardware
│                               chain that question turned out to be hiding.
│                               DESIGN SETTLED, NOTHING BUILT. Supersedes parts
│                               of HARDWARE-READINESS.md and retires most of S6;
│                               its section 5 says exactly what. Read it before
│                               any hardware purchase -- the chassis is no
│                               longer a PiCar-X
│
│   -- bill of materials. JETSON-BOM.md is the one to read --
├── GUIDE-robot-base.md    **a learning guide**: the four layers from wheel
│                           encoder to ROS 2 node, closed loop vs "reports
│                           to the host", reading a vendor driver, firmware
│                           openness, powering a Jetson from a robot battery,
│                           chassis geometry, lidar and depth cameras, and
│                           reading a kit listing. Written from the 2026-09
│                           chassis search -- read before buying a robot base
├── JETSON-BOM.md          **what to buy** (recommended build, 2026-09-17),
│                           doubling as a brief for a ready-made-kit
│                           search. Carries the constraints that
│                           disqualify most kits -- 3S power above the
│                           Jetson's 9V floor, differential drive,
│                           quadrature encoders, and a serial motor
│                           controller rather than a Pi HAT
├── BOM-COMPARISON.md      verified retailer prices, Pi vs Jetson, like
│                           for like. **Price from here, never from 3.6**
│                           -- 3.6's Pi 5 line reads $80 against $175
├── PI-VS-JETSON.md        the "what if the Pi instead" walkthrough
│                           (2026-09-30): price, gains, losses, and the
│                           one test (YOLOE on a Hailo-8L at INT8) that
│                           could change the answer
├── HARDWARE-BOM.md        part numbers, vendors, wiring and bring-up
│                           order (Cowork's research + editor's note)
├── BOM.md                 SUPERSEDED -- the 2026-09-12 Jetson build, on
│                           estimates. Right argument, wrong prices
└── PLAN-aws-cost-redesign.md  the ~$159/month of fixed AWS cost, where it
                                comes from, and the rebuild that removes
                                ~$110 of it. ALL DONE: walks on S3, the VPC
                                and ECS stacks torn down 2026-09-05, and the
                                serverless stack is the only deployment.
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

**Every stage is done on data** -- see section 7: acceptance metrics and
thresholds written down before the run, measured through the real mission
path, recorded, and pinned in a test. (Until 2026-09-25 this said a phase
was done only when someone holding a phone had watched it; retired by the
user.)

### Stage 0 -- Validate the premise

**Primary tool: the Guide tab's "Robot view" mode.** Point your phone at
a real room and it shows the move the robot would make from where you are
standing -- the same `/navigate` route the vision policy uses, real pixels
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

**The 39-walk corpus the early Stage 0 findings were measured on was
DELETED on 2026-09-07** (standing height, targets on furniture -- invalid by
its own viewpoint). Those findings -- the 3x3 and 5x3 wording matrices, the
`obstacle_ahead` calibration failure, the closed-loop sim runs, the
corpus-invalidity diagnosis -- are in `docs/archive/CLAUDE-history-2026-09.md`,
verbatim. What survives:

- **One wording ships** (`default`, decided 2026-09-03); the other four stay
  replay-only, the only controlled comparison there is. A threshold has no
  wording, so do not write a sixth.
- **`obstacle_ahead` is uncalibrated** -- models disagree on it from ~0% to
  ~100% on the same frames. On hardware the lidar is the obstacle sensor;
  never let the vision policy rely on this field.
- **The corpus is rig walks now** (floor height, target on the floor,
  adjudicated `labels.json` beside each); `PLAN-onboard-perception.md` 4.10
  has what it settled.
- **What to record next:** two out-of-vocabulary searches plus one control,
  under `policy: "tiered"`, so 1.11a's corroboration verdict is measured
  live -- `PLAN-onboard-perception.md` "What to record next, and why these
  walks" (2026-09-08).

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
  a backend-agnostic conformance suite (36 tests when written; six
  backends now -- `BACKENDS` in the file) parameterized over all
  four `RobotInterface` backends that existed then (`MockRobot`,
  `RemoteRobot`, `ReplayRobot`, `TeleopRobot`), asserting return shapes,
  units and `stop()` idempotency with no grid-specific assertions. Extended
  by S2 (44 tests now) to pin pixels too: every backend must return a
  decodable image and name its media type.
- **S2 -- real image bytes -- BUILT (2026-08-31).** `sim/renderer.py` is
  the twin's raycaster ported into Python, so `get_camera_frame()` returns
  JPEG bytes on every backend and the one structural blocker between the
  vision policy and hardware is gone. The JS raycaster it replaced was
  **deleted 2026-09-25** -- see the status table in section 3, and
  `sim/renderer.py`'s fidelity note, which is the reason this does not
  retire the real-photo gate in Stage 0.
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
  `service/vision_analyze/tests/` (FastAPI `TestClient`, every
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
  "Local brain" (the JS loop). **The local one was deleted 2026-09-25**
  with the ROS alignment: R4 puts `twist_mux` between any driver and the
  wheels and allows exactly one writer, and a brain that dies with a
  browser tab was never going to be it. The server's own 409 and M4's
  authority order enforce what the two-panel arrangement used to. The
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

**The Jetson is ordered (2026-09-27) and so is the robot base (2026-09-30,
the Waveshare UGV Rover PT Jetson Orin ROS2 Kit Acce, Amazon). What is
left to buy is the separate Jetson battery and its fused cable:
`JETSON-BOM.md` 9.5.** The Rover's ROS Driver
board ships closed-loop firmware, so **no firmware change** is needed;
`HARDWARE-BOM.md` correction 5 (raw PWM, reflash for closed loop) applies to
the General Driver board of the 2026-09-19 build, not to the Rover.
`HARDWARE-BOM.md` still holds the ESP32 JSON protocol, udev/`dialout` and
the bring-up order. Verified prices are `BOM-COMPARISON.md`. *(Rewritten 2026-09-28: this section used to point at
`PLAN-onboard-perception.md` section 1's Pi + Hailo-8L list at ~$555-620,
which the Jetson decision superseded.)*

Before ordering, the two open risks in `JETSON-BOM.md` section 7 (a torch
wheel for the chosen JetPack, and on-board latency) and the devkit firmware
check in `HARDWARE-BOM.md` 5.1.

On hardware day: `HARDWARE-READINESS.md` section 5's pre-flight list
(re-bannered 2026-09-28 for the Jetson parts). `robot/hardware_robot.py`
already exists (R7, against `sim/fake_esp32.py`); what remains is B5 (the
units that start the ROS container, robot and brain at boot --
`PLAN-brain-relocation.md`), the measurements the xacro marks
`[PLACEHOLDER]`, and the calibration items in `PLAN-sim-hardening.md`
section 7 that can only be measured.

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
  Ackermann half stays retired. **The continuous-pose half was BUILT
  2026-09-25 as R0** (`PLAN-ros-alignment.md`) -- and the estimate above was
  about right on the sim's side and wrong about `mock_robot.py`, where the
  two-line conversion turned out to be a wheel-velocity integrator, because
  R0 took the chance to put `diff_drive_controller`'s own kinematics under
  test rather than only un-quantising the pose.
- **`control/manual_control.py`** -- skip. The twin's D-pad covers it
  better, and B0's `RemoteRobot` (built) makes it nearly free if ever
  wanted.
- **The rule-based agent** -- keep, do not extend. It is the fastest,
  free, deterministic way to test the safety layer and mission memory.
  It is not on the hardware path (`PLAN-sim-hardening.md` 2.2). **Since
  2026-09-25 it no longer thinks in cells** (`PLAN-ros-alignment.md` 3.2):
  its pose comes from `WorldInterface` and its scene from the depth grid
  (`ConstrainedAgent.sensed_scene()`), so it needs a world to explore
  well -- a test that builds a `MissionRunner` directly passes one
  (`tests/conftest.py`'s `fresh_mock_runner()` / `mock_world_for()`), and
  without one it degrades to the right-hand rule rather than failing. nav2's
  frontier exploration replaces it at R6.

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

- **SUPERSEDED 2026-09-25 -- the JS local brain this bullet describes was
  deleted** (`PLAN-ros-alignment.md` 3.2), along with the twin's copy of
  the house and its raycaster. Kept below as the reasoning it was built on.
  **The web twin's exploration algorithm duplication is intentional,
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

- **Since 3.19 rotation is vetted too** (`SafetyController.pivot_scale()`):
  a turn that closes on something within 1.2 cm is slowed to what keeps the
  margin; a turn that opens the gap is never touched. Anything that relies on
  "rotation is never clamped" (R2b's pivot-away) still holds for turning
  AWAY, and only for that.

- **Since 3.18 a forward (or reverse) move is vetted against TWO things in
  series** (`SafetyController.forward_clearance()` / `reverse_clearance()`):
  the depth-grid cone above, and the chassis' swept corridor off the lidar
  scan in the BODY frame. Use those, not `path_clearance()` alone, for any
  "may I move" question -- the cone is sized for one 30 cm move, is narrower
  than the chassis up close, and is cast along the CAMERA, so a peek swings
  it (the grid publishes `pan_deg`, and zones are chosen by body bearing).
  Judge any change to either on ground truth (`tests/footprint_sweep.py`),
  never on the readings the veto itself uses -- 3.17's "flaky 18.0 cm" was a
  reading, and the truth under it was 25 cm.

- **Safety is enforced server-side, always.** Both the sim agents
  (`brain/agent.py`) and the Wi-Fi API (`robot/server.py`) route every
  movement action through `robot/safety.py`'s `SafetyController`. Don't
  add a new movement path that bypasses it.

- **HISTORY -- the NLB/ALB/ECS stacks this bullet and the next two describe
  were DELETED 2026-09-05** (`PLAN-aws-cost-redesign.md`). What is deployed
  now is `cloudformation/serverless.yaml` (CloudFront + S3 + API Gateway +
  Lambda); its route guard is `tests/test_serverless_routes.py` +
  `tests/test_static_assets.py`, the successors to `test_alb_routes.py`.
  Kept for B5 and in case ECS returns. **`vision-picar-twin` (the digital
  twin) and `vision-picar-service`
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
  the detector weights (`yoloe-11s-seg.pt` since P23) *inside* `POST /mission/start`, so the panel sits on
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

- **Coverage is 93% of `brain/`, `control/`, `robot/`, `sim/` and
  `world/`, and the shortfall is deliberate.** (This line read 99% until
  2026-09-07, 98% until 2026-09-08 and 96% until 2026-09-20. Quote what the
  command prints, not this sentence -- it drifts down as the `P*` modules
  whose constructors load models grow, which is the trade recorded below.)
  Measure it with:

  ```bash
  pytest tests/ --cov=brain --cov=control --cov=robot --cov=sim --cov=world --cov-report=term-missing
  ```

  `world/` is 100% and `sim/mock_world.py` 98% -- N1 is small and has no
  model to load, which is the whole reason those numbers mean something
  where `brain/perceive_lab.py`'s 60% does not.

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

- **The /navigate picker gained three models on 2026-09-21, and one of them
  is invoked in a different REGION than this service runs in.** Claude
  Fable 5.1, Claude Opus 5 and GPT-6 Astra joined
  `vision_core._DEFAULT_NAVIGATE_MODEL_CHOICES`, each confirmed the way
  every entry there is -- a real Converse call carrying a real walk frame,
  not a catalog listing. The default is deliberately unchanged (Opus 4.5)
  and the three are labelled "unmeasured": no walk has been replayed
  through them, and `control/walk_replay.py` is the instrument for that.
  The comment above them naming the 5-series as AccessDenied is dated and
  now wrong -- Sonnet 5 and Opus 4.8 are invokable too, just not listed.

  **Fable 5.1 answers only from us-east-1 on this account.** From us-east-2
  (where the service is deployed) and from us-west-2 it is refused with
  Bedrock's `data retention mode 'default' is not available for this
  model`, on the `us.` and `global.` profiles alike. Both regions'
  inference-profile fan-out and the account's
  `get-use-case-for-model-access` form are identical, so this is AWS-side
  per-region enablement with nothing in the account to toggle. So
  `vision_core.MODEL_REGIONS` pins that one model's `bedrock-runtime`
  client to us-east-1 and leaves every other model on the ambient region;
  `_get_client()` now takes the model id and caches one client per region.
  Two things follow. The pin is **expected to be deleted** -- set
  `BEDROCK_MODEL_REGIONS=""` (or a `model=region` list) the day Fable 5.1
  is enabled in us-east-2, or it just buys a slower call. And a model that
  400s only in production is exactly what this list is supposed to prevent,
  so **re-confirm a pinned model from the deployed region, not a laptop**:
  the laptop's ambient region is whatever `aws configure` says, which is
  how this was nearly missed.


---

## 7. Data first, then the car

**The definition of done, as of 2026-09-25.** Stated by the user: *"remove
that rule, use logs and data to make a data-driven decision."* It replaces
the 2026-08-27 rule, which is kept below for the record.

### A phase is done when its acceptance data says so

1. **Write the metric and the threshold down BEFORE measuring**, in the
   phase's plan entry -- so the data decides, not the reading of it
   afterwards. A threshold chosen after seeing the numbers is a
   description, not a test.
2. **Measure through the real path.** Missions run end to end --
   `MissionRunner` -> agent -> `robot/safety.py` -> backend -- never a
   harness that moves the robot itself (R1's first probe did, leaked ground
   truth into its search turns, and reported 4.7 cells where the honest
   number was 0.8). Sweep many starts in process for the numbers, and run at
   least one mission against the live local stack through the brain's HTTP
   API, so the deployed path is covered too.
3. **Record the numbers in the plan entry and pin them in a test**, the way
   `tests/test_bearing_turns.py` pins R1's A/B, so a regression fails the
   suite rather than waiting to be noticed.
4. **Pair every stability metric with a progress metric.** Reversals alone
   reward a spin (the first watched R1 run: 98 turns in 120 steps, "0
   reversed"); `median_command_run` alone rewards the same. Distance closed,
   arrival rate and outcome are the progress half.
5. **Claude runs the missions and reads the logs.** The user is not asked to
   run a simulation or fetch logs; results come back with the numbers.

### What stays, and why

* **The twin**, as the surface a person drives from and the API the hardware
  will answer. It is no longer the gate.
* **UI tests** (`tests/test_ui*.py`, Playwright at a phone viewport) as a
  regression guard, and **a phone-size screenshot as part of the evidence for
  any change to the page**. The last day of the old rule is why: one session
  of watching caught five defects over 1100 tests had not -- a Start button
  that read as a label, a spin that scored as "0 reversed", the robot seeing
  the sofa it stood on, a hint naming models that were not running, and a
  driver printed as "null". Screenshots keep that coverage without making a
  person the gate.
* **Drills** (`control/drills.py`), for failures nothing can provoke on
  purpose -- now judged by what their logs show.

### Retired 2026-09-25 -- the rule this replaced

> **Build it, prove it in the digital twin's UI, and only then put it on
> the PiCar.** A phase is not done when its tests pass. It is done when
> someone holding a phone can watch the thing it built do its job.

It was stated 2026-08-27, reaffirmed as non-negotiable on 2026-09-25, and
retired later the same day. The sections below still list what each phase
put on the page; read them as a reference to the UI, not as gates.

### UI affordances by phase (reference, not a gate)

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
| 2 | one brain at a time | *(Explore and the local brain were deleted 2026-09-25; this row is history)* Start a remote mission, then tap Explore | Refused with a toast; the reverse is the server's 409 |
| S4 | time in the loop | Set `sim.realtime: true` in `config/robot.yaml`, restart the robot server, then Remote brain -> Start | The watchdog readout climbs mid-move instead of only between moves -- a move now genuinely occupies its duration, off by default so this is opt-in |
| S5 | sensor realism | Set `sim.sensor_noise.enabled: true`, restart the robot server, then D-pad toward a wall | Distance telemetry stops being multiples of 30cm and jitters. The collar still fires only at the wall: `min_distance_cm` is 20 on both sides now, which is 3.3 sigma clear of one cell -- at the old brain-side 30 the jitter alone vetoed ~45% of legal one-cell moves |
| -- | a lit sim camera | Sim tab, drive the D-pad and watch the FPV canvas (or `GET /frame`) | A room: light ceiling, mid-brown floor, blue-grey walls. It used to be a black void with two grey slabs, because the render borrowed the twin's dark `--wall`/`--floor` UI colours -- which is why the model called every sim frame "very dark and unclear" |
| -- | a session's calls die with it | Guide tab -> Robot view, Start, Stop, Start again | The new session's HUD never shows the previous one's decision. A call still in flight at Stop is orphaned by run (`guidanceEpoch`), not by a boolean -- it used to flash its answer over the new camera view and then suppress the new run's first few real decisions. Everything else was already reset on Stop, so a straggler was the only route |
| N1 | a map that fills in as you drive | Sim tab, look under the depth strip, then drive the D-pad into a room | The house appearing a room at a time -- seen floor, seen wall, and NEVER SEEN in three distinguishable tones, with the robot drawn on it and a readout naming cells-seen, cell size, map id and version. It starts mostly unknown because the map is **discovered**, not copied: a ring is cast from wherever the robot stands and everything behind a wall stays unknown. A server predating the route says "not reported by this server" rather than going blank, because blank and "no map" must not look alike. The pose is continuous as of R0 (2026-09-25): drive a diagonal and the robot is drawn between cells at a non-multiple-of-90 bearing. It was quantised to cell centres before that, and the contract already took a float -- which is why making it smooth changed three lines in `sim/mock_world.py` and nothing on the consumer side |
| R0 | a pose that is not on a grid line | Sim tab -> under the D-pad, **Turn: 45°**, then tap right, then forward | The robot on the map view at 135 degrees and off both grid lines, with the FPV and the depth strip both swung 45 degrees -- all three are cast from one continuous `view_angle()`. The log names the angle (`RIGHT 45°`). The step (15° / 45° / 90°) is remembered across reloads and defaults to 90, which is what every tap sent before R0 |
| R1 | the robot aims, instead of flickering | Sim tab -> Remote brain -> policy **Tiered**, Start (a few paid cloud calls on triggers) | The Models line reads **sim ground truth** -- no YOLO or CLIP runs on a render. Once the backpack is in view, Last action shows a SIZED turn ("LEFT 23°"), then FORWARD, and the robot drives to it on the map; **Turns** reads a handful made with 0-1 reversed. Before R1 the same mission flipped LEFT/RIGHT and never approached. From a start whose straight line clips the kitchen door jamb it waits at the jamb -- correct, and R6's job |
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

### UI planned for later phases (reference, not a gate)

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
