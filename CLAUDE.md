# vision-picar

Orientation for a Claude Code session; Claude Code loads this file
automatically. It states what is true now. Every per-phase record lives in
the plan that built it (`docs/plans/`), and the long version of this file
-- the full status table, the Stage 0-5 narrative, the deleted-ECS notes,
the UI-affordance tables -- is `docs/archive/CLAUDE-2026-10-06.md`,
verbatim. Older reversals are in `docs/archive/CLAUDE-history-2026-09.md`.
**Where an archive disagrees with this file, this file governs.**

---

## 1. Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

pytest tests/ -q                          # ~1755 tests; `pytest --collect-only` for today's count
pytest service/vision_analyze/tests/ -v   # the cloud vision service's own suite (section 6)
python -m playwright install chromium     # once: the UI tests SKIP without a browser
```

* **Trust `pytest` from `.venv`.** The system Anaconda Python has an older
  FastAPI and disagrees about a few tests.
* **In a Claude Code cloud session, `.claude/hooks/session-start.sh` does
  all of this at start-up** (cloud only): `.venv`, `requirements.txt`, a
  Playwright that can launch the image's preinstalled Chromium (the newest
  one may not, and `playwright install` is not allowed there), and the
  `tools/hooks` push gate.
* **The UI tests (`tests/test_ui*.py`, `tests/test_frame_source.py`) drive
  the real twin in a real browser at a 390 px phone viewport.** They exist
  because the UI is where bugs escaped: two shipped on 2026-08-29 and were
  found on a phone, not by the Python suite. When adding one, re-introduce
  the bug and watch it fail first, and run it in its module's real order,
  not only with `-k` (section 6).
* **`ANTHROPIC_API_KEY`** is needed only by `tests/manual_describe_image.py`
  (a real API call). The automated suite mocks every model call, and the
  cloud vision service authenticates to Bedrock through IAM, not a key.
* **Perception models** are an optional install
  (`pip install -r requirements-perception.txt`). The suite never needs them.

---

## 2. What this project is, and the rules that keep it working

An indoor autonomous robot that searches a house for an object. The
hardware is a **Waveshare UGV Rover** (skid-steer chassis, ESP32 motor
board, D500 lidar, OAK-D Lite, pan-tilt camera) carrying an **NVIDIA Jetson
Orin Nano Super**. The Jetson runs the robot server, the brain (mission
policy plus on-board perception) and a ROS 2 container (SLAM, nav2,
`ros2_control`). A vision LLM on Amazon Bedrock is called only on triggers.
The Jetson is in hand and measured; the Rover is due Oct 19 - Nov 11.
`docs/ARCHITECTURE.md` is the system on one page.

**Development is simulation-first:** build it, prove it with data in the
simulator, then put it on the car. The simulator drives the same API the
hardware will, so a mission that succeeds through it exercises the path the
car uses.

**The rules. Each is enforced by a test, not by memory.**

1. **Body state goes through `RobotInterface` only.** `brain/` and
   `robot/server.py` talk to `robot/interface.py`, never to
   `sim/mock_robot.py`. `robot/factory.py` is the only place that picks a
   backend (`mode` in `config/robot.yaml`), which makes the hardware swap a
   config change. `tests/test_robot_contract.py` runs one conformance suite
   against every backend.
2. **World state goes through `WorldInterface`** (`world/interface.py`,
   picked by `world/factory.py`). The line is whose frame the answer is in.
   Egocentric is body state (`get_distance`, `get_depth_grid`,
   `get_odometry`, `get_scan`); allocentric is world state (`get_pose`,
   `get_map`). **Odometry is what the body says about itself; pose is what
   the world says about the body.** A mapper's pose jumps on loop closure
   and odometry never does. `tests/test_world_contract.py` pins the split
   in both directions.
3. **ROS stays inside `service/slam/`.** Nothing outside it may import
   `rclpy` (`tests/test_ros_containment.py`). The rest of the system talks
   to ROS over HTTP through one bridge. **Plumbing goes in; judgement and
   safety stay out** (decided 2026-10-02, `PLAN-ros-alignment.md` 1.1):
   * Inside ROS: SLAM, nav2 and wheel kinematics.
   * Outside: the brain, `robot/safety.py`, the watchdog, the phone, and
     any sensor that feeds a veto (motor board, lidar, OAK-D floor depth).
4. **Safety is enforced in the robot server, on every path.** Every
   movement goes through `robot/safety.py`'s `SafetyController`. ROS's
   `collision_monitor` sits in series before it, and the robot server has
   the last word. Never add a movement path that bypasses it.
5. **`control/` (the brain service) imports no backend and no simulator.**
   It reaches the robot only as an HTTP client (`RemoteRobot`,
   `RemoteWorld`). `tests/test_brain_server.py` checks `sys.modules`.
6. **Every component has an ARCHITECTURE spec and an ENGINEERING spec**
   (decision 0001, 2026-10-02):
   * `docs/<domain>/ARCHITECTURE.md` says what and why, and outlives a
     rewrite of the code.
   * `docs/engineering/<domain>/ENGINEERING.md` says how, and **changes in
     the same commit as the code it describes**.
   * `docs/README.md` is the index; `tools/spec_lint.py`
     (`tests/test_spec_lint.py`) enforces the mechanical rules;
     `/spec-review` the judgement ones.
   * The `PLAN-*.md` files are the dated history the specs cite.

---

## 3. Current state (2026-10-06)

### 3a. What is built

Phase IDs: `R*`/`3.*` = `PLAN-ros-alignment.md` (the governing plan since
2026-09-25), `P*` = `PLAN-onboard-perception.md`, `S*` =
`PLAN-sim-hardening.md`, `B*` = `PLAN-brain-relocation.md`, `M*` =
`PLAN-microduck-transplants.md`, `T*` = `PLAN-teleop-robot.md`, `N*` =
`PLAN-mapping.md`. The full per-phase rows are in the archive copy.

| Area | State | Where to read |
|---|---|---|
| Simulator | Continuous pose and differential-drive kinematics (R0). Solid objects (3.9). People and pets that move (3.30, `SIM_MOVERS`). Four houses via `SIM_MAP`: `starter_house` (30 cm doors), `scaled_house` (90 cm doors; nav2 is judged here), `home_first_floor` (a realistic furnished test house, not a replica of the user's), `complex_house` (3.46: a loop, dead ends, 16 small things on the floor). Under `SIM_MOTOR_BOARD=fake` the body runs as its own programs (3.36). | `sim/`, R0, 3.30, 3.36 |
| Robot server and safety | Arbitration by driver rank (M4). A depth-grid cone plus the chassis' swept corridor off the 360-degree scan (3.18), a pivot guard (3.19), guarded verbs (3.22), a settle pass (3.29), no motion without fresh wheel feedback (3.34) and stall detection (3.35). The watchdog is B3.1. | `robot/`, `docs/safety/` |
| Brain service | `MissionRunner` behind `control/brain_server.py` on :8001. Three failsafes (B3). Policies: `frontier` (rule-based), `vision` (cloud per step) and `tiered` (local YOLOE + CLIP, cloud only on triggers). | `control/`, `AGENT-HARNESS.md` |
| Perception | YOLOE-11s-seg crops, then CLIP RN50, gate P >= 0.8. On the Jetson at 15 W: median 61 ms, p90 110 ms a frame. | P22-P24, 3.33 |
| Arrival | The target centred and the lidar under 0.40 m for two frames, then one cloud call confirms identity (3.11, 3.32, handoff 1a). Cloud unreachable at arrival: `arrived_unconfirmed`, never `found` (3.47). | `brain/arrival.py` |
| ROS stack | URDF/TF (R3, Rover CAD geometry since 3.27); `ros2_control` + `twist_mux` (R4); `slam_toolbox` (R5); nav2 + `collision_monitor` (R6). Off by default (`ROBOT_DRIVE=ros`, `WORLD_MODE=ros`). `drive: ros` gates G1-G4 all met (G4 on the Jetson, 2026-10-05). | `service/slam/README.md`, `docs/ros/` |
| Lidar | `robot/lidar_ld19.py` reads the D500 (LD19 protocol) in the robot server, not ROS; `sim/fake_lidar.py` fakes it on a pty (`SIM_LIDAR=fake`). Ground-truth safety holds under its real timing after two fixes. On the Jetson: 18/18. Nav smoothness on the laptop sits at its bar (3.42) | `docs/plans/PLAN-ros-alignment.md` 3.42 |
| Motor board | `robot/hardware_robot.py` over the ESP32's JSON serial protocol (R7). `sim/fake_esp32.py` copies the Rover's firmware from source (3.25). Our GPL-3.0 firmware fork adds 0.1 mm odometers and a timestamp; compiled, not flashed (3.28-3.29). | `firmware/`, `docs/motor-board/` |
| Jetson | JetPack 6.2.1 on the NVMe, 15 W, kept 2026-10-05. The suite, G4 and headroom all pass there (0 late wheel-loop ticks in 14,289). | 3.33-3.37, `tools/jetson/README.md` |
| Cloud | The twin and console are static on S3 + CloudFront. `/navigate` (Opus 4.5) and the walks API run on Lambda (`cloudformation/serverless.yaml`). The robot and brain run locally, reached through `service/tunnel/`. | `docs/cloud-vision/`, `docs/operations/` |
| Twin | `web-twin/`: Sim tab (D-pad, map, depth strip, Remote brain panel), Guide tab (Robot view, Drive via brain), Settings. | `docs/guides/FEATURES.md` |
| Recorded walks | S3 storage, scorecard (`walk_eval`), replay under another model (`walk_replay`), corpus scoring (`perception_eval`), labelling help (`label_assist`). | `docs/recordings/` |

### 3b. Open work, in order

* **SLAM in the furnished home: fixed and merged** (3.38-3.40; branch
  `frontier-search` merged into `dev` 2026-10-06). The map bent 2.5-6 deg in
  the open rooms because slam_toolbox's scan matcher blurred each point
  10 cm before scoring (`correlation_search_space_smear_deviation`
  0.10 -> 0.03 m: two tours within 4.2 cm, every room's walls 100% true).
  Scans are also stamped with their capture time (skew 21 -> 7 ms).
* **Offline is a mode (3.49), merged 2026-10-09.** With every connection
  refused: the robot server loads no cloud client, `frontier`/`explore`
  still find, tiered ends `arrived_unconfirmed` at the target, `vision`
  ends `failed` without moving, the twin loads nothing from the internet.
  One criterion failed and is recorded (an async start closes 0.62 cells
  less than with the network). `tests/test_offline_mode.py`.
* **Offline arrival (3.47), merged 2026-10-09.** A tiered mission whose
  arrival holds but whose identity check cannot reach the cloud ends
  `arrived_unconfirmed` (not `found`, not `failed`). Only `CloudUnavailable`
  from `brain/navigate.py` (transport error, 5xx, 408, 429) counts as
  unreachable. Known limits in the section. Twin warn style and re-asking
  the cloud when it returns: approved by the user, not built.
* **Object inventory (3.46), merged 2026-10-08.** Every mission records
  what it saw, placed by the lidar, weighted by looks from new viewpoints;
  `GET /mission/inventory`, and uploaded to the recordings bucket under
  `inventory/` (labels and positions only; the user's decision). Nothing
  reads it. Sim criteria met on fresh missions (sweep 3: recall 96.6%,
  precision 97.2%); sweep 1's recall failure is recorded. **Real frames
  give it nothing yet:** phase C (a prompt-free detector) is not built.
  `HANDOFF-2026-10-08-3.46-object-inventory.md`.
* **Explore's corner stalls: fixed** (3.45, merged 2026-10-09 with the
  tour criterion failed, the user's call; 3.43/3.44 before it). Frontier and
  view goals now keep the chassis' half-length plus the 20 cm stop
  (0.3265 m) from the map's walls, where 0.22 m let nav2 reach goals the
  safety layer would not let the robot leave. On recorded maps the goals
  explore chooses are 95.5% leavable on ground truth (was 24%). Live:
  parked 99 / 93 / 88 s (bar < 120 s; was 119 / 197 / 152), coverage median
  0.88 (was 0.55). 3.44's speed-dependent stop and creep escape stay.
  **The tour now fails on its own:** 4 / 8 / 4 of 9. Goal 6's point lies
  inside the dining furniture and can never succeed, and nav2 wedges itself
  near the garage door on its own costmap while truth is free. Next: move
  goal 6 (instrument only), then the garage wedge. Live runs now keep
  per-run logs with explore's decisions (`PICAR_LOG_LEVEL`,
  `evaluations/slam-345/`). Handoff:
  `HANDOFF-2026-10-07-3.45-explore-goal-choice.md`. 3.31's frontier-search
  batch is unblocked.
* **Raising nav2's speed** (6.9's high end) now has its speed-dependent
  stop, but waits on measured braking (6.9 (d)) and 3.42's relayed-scan
  time stamp; read `HANDOFF-2026-10-07-lidar-and-speed.md` first.
* **Isaac ROS evaluation (3.41).** Isaac ROS 3.2 is the last release for
  JetPack 6 / Humble.
  * **Part A, measured 2026-10-06: TensorRT not adopted; torch stays.**
    The fp16 engines change CLIP probabilities by up to 0.052 (bar 0.01).
    p90 is only 20-25% faster (bar 30%).
  * **Closed by the user the same day.** Revisit TensorRT or Isaac ROS
    only if perception goes over its 250 ms budget on the car: a real
    camera, a higher frame rate or a heavier model. 3.41 lists the order
    to try things in.
  * **Part B (cuVSLAM, nvblox)** needs the Rover's camera.
* **Before the Rover arrives:** check with calipers that the devkit's
  mounting holes match the Rover deck's 86 x 58 mm pattern
  (`UGV-ROVER-MOUNTING.md`); fallback is an adapter plate. Waveshare
  request 258472 (sent 2026-10-06) is open on the same question.
* **On arrival** (`HARDWARE-READINESS.md` section 5;
  `PLAN-ros-alignment.md` 3.26):
  * Disable Waveshare's stock app and its ROS nodes, which would hold the
    serial port and bypass safety.
  * Confirm the serial route (expected `/dev/ttyTHS1`); dump the stock
    firmware; flash the fork after the return-window checks.
  * Set up udev names and `dialout`.
  * Verify the lidar driver on the real D500 (3.42 criterion 7): the
    protocol and baud, the 10 Hz rate, a taped wall, and the mounting yaw.
  * Run a 15 W stress test with motors. It decides the separate Jetson
    battery (`JETSON-BOM.md` 9.5).
  * Measure every `[PLACEHOLDER]` in the xacro, including `track_scrub`.
  * B5: systemd units so everything starts at boot.
* **Decisions still open:** `PLAN-ros-alignment.md` section 6 (map save and
  reload, gyro heading, the floor band below the lidar, speed by
  clearance); `HANDOFF-2026-10-02-spec-review.md` section 3 (3a-3c);
  review 3's fix 7 (arrival confirmation against the real cloud) and fix
  11 (whether a person's verb cancels a goal).
* **Not built:** `brain/planner.py`, room-level planning over
  `MissionMemory`. S7 (chaos and soak). A Stage 1 re-run of a vision
  mission against the lit renderer.

### 3c. Standing facts a session needs

* **Hardware is decided** (2026-09-19, user): a Jetson Orin Nano Super. The
  Hailo / Pi path is closed; do not reopen it or spend on Hailo runs.
  Prices and records: `JETSON-BOM.md`, `BOM-COMPARISON.md`.
* **The Rover's motor board speaks JSON, not ROS** (`T:1001` frames with
  `odl`/`odr`, 660 pulses/rev). Its `T:0` "e-stop" releases the arm, not
  the wheels; the heartbeat (1.5 s) is the board's only stop.
* **Tiered phone walks end `max_steps` even when they arrive.** Arrival
  needs a lidar scan and a phone walk has none, so do not read a phone
  walk's outcome as a navigation result.
* **The world map is discovered, not copied** (N1). `world.mode` must
  track `mode`: the factory refuses `world: sim` for a robot with no grid,
  which is why `service/tunnel/run.sh` sets `WORLD_MODE`.
* **`tier_hold_bearing` stays OFF.** A 1-in-3 detector needs a
  range-anchored goal pose (R1). `brain/goal_pose.py` is wired but holds
  only a direction without a range.
* **`obstacle_ahead` from the vision model is uncalibrated** (0-100%
  across models on the same frames). The lidar is the obstacle sensor;
  never let a policy rely on that field.
* **Perception numbers:** never merge GPU and CPU rows (P24), and pin the
  frame set before comparing configs, because the labelled corpus grows.
  `control/perception_eval.py` is the scorer; do not rewrite it in a
  scratchpad.
* **Parallel sessions** share this repo and the Jetson. Each manages its
  own commits. **Read `docs/guides/PARALLEL-SESSIONS.md` before working on
  the plan** (the user, 2026-10-07). In short:
  * One branch and worktree per plan section:
    `python tools/plan_section.py new <slug> "<title>"` reserves the next
    number.
  * Each section is its own file in `docs/plans/ros-alignment/`.
  * Your own ports, ROS domain, container and image:
    `eval "$(python tools/plan_section.py env)"`. Never rebuild
    `vision-picar-ros:latest` from a branch.
  * `overlap` before editing a shared file.
  * `ready` (rebase check + full suite) before asking to push to `dev`; a
    pre-push hook enforces it.
  * One heavy live run at a time (a lock in `tests/demo_explore.stack()`).
  * CLAUDE.md section 3b and other shared status are updated at merge, not
    on a branch.

---

## 4. Repo map

Files are cited by name across the repo and every doc name is unique, so
search for the name.

```
robot/        the robot server and the body contract
  interface.py      RobotInterface: the one body abstraction (honest "unusable" defaults)
  factory.py        picks the backend from config/robot.yaml
  safety.py         SafetyController: the veto on every path (cone, swept corridor, pivots, verbs)
  server.py         FastAPI on :8000: /action arbitration, /wheels 20 Hz loop, watchdog, serves the twin
  hardware_robot.py R7: the ESP32 motor board over serial
  lidar_ld19.py     3.42: the D500 lidar (LD19 protocol), read here so safety keeps a scan when ROS hangs
  ros_drive.py      R4: under drive: ros, verbs become twists through the ROS container
  identity.py       the start-up identity line both servers log (M5)
world/        WorldInterface (pose, map), its factory, and ros_world.py (SLAM + nav2 goals)
brain/        decisions and perception, hardware-agnostic
  agent.py          ConstrainedAgent / MissionAgent / ObjectSearchAgent (rule-based; keep, do not extend)
  vision_agent.py   VisionAgent: the cloud picks every move
  navigate.py       a frame -> the cloud /navigate route -> one action
  tiered.py         local perception every frame, the cloud only on triggers; 1.11a reported, not enforced
  perceive.py       YOLOE crops -> CLIP -> match gate (heavy deps lazy)
  perceive_lab.py   candidate detectors that are not shipped (Grounding DINO, OWLv2, YOLO-World, SAM)
  arrival.py        the lidar-confirmed arrival rule
  goal_pose.py      a sighting anchored in odometry
  memory.py, rooms.py, vision.py   mission memory, room recognition, direct Anthropic API
sim/          the grid-world simulator
  grid_world.py, mock_robot.py, mock_world.py, renderer.py, sensors.py, movers.py
  replay_robot.py   a recorded walk as a body (open loop)
  teleop_robot.py   a live phone camera as a body (mode: teleop)
  fake_esp32.py     the Rover's motor-board firmware on a pty
  fake_lidar.py     the D500 on a pty, every point cast at its own instant (3.42)
  body_server.py, sensor_server.py, body_state.py, body_client.py   the body in its own processes (3.36)
  maps/             starter_house, scaled_house, home_first_floor
control/      the brain as a service; robot reachable only over HTTP
  brain_server.py, mission_runner.py, brain_config.py, remote_robot.py, remote_world.py
  drills.py         fault injection for the failsafes
  health.py         `python -m control.health`: one verdict for both halves (M5)
  walk_eval.py, walk_replay.py, perception_eval.py, label_assist.py, target_probe.py
  walk_store.py, recording_routes.py, admin_*, metrics_*   recorded walks and the console
service/
  slam/             the ROS 2 Humble container: the only place ROS exists
  vision_analyze/   the cloud vision service (Bedrock), its own copies of vision/rooms code, its own tests
  lambda/, static/  build and sync for the serverless deployment
  tunnel/           run.sh / restart.sh: robot + brain locally, one ngrok domain split by path
  admin/, brain/, twin/   ECS images for stacks deleted 2026-09-05 (kept for B5)
firmware/ugv_base_ros/    our GPL-3.0 patch to the Rover's ESP32 firmware, and build.sh
tools/        spec_lint.py; jetson/ (setup, perception benches); hailo/ (closed path, kept as record)
cloudformation/   serverless.yaml, recordings-s3.yaml, deploy-bucket.yaml are live; the rest are deleted stacks
web-twin/     the phone UI (index.html, app.js), a client of both servers
config/robot.yaml   mode, safety thresholds, the brain: and world: blocks
tests/        the suite, plus demo_*/manual_* scripts that are run by hand
docs/         ARCHITECTURE.md first, then README.md (the index): specs, plans, guides,
              hardware, evaluations, handoffs, archive
```

---

## 5. What is left before, and on, hardware day

Each stage was independently useful; stopping after any of them left the
project coherent. Their full narrative is in the archive copy.

| Stage | State |
|---|---|
| 0. Validate on real photos (the go/no-go gate) | Rig walks exist (floor height, target on the floor, adjudicated `labels.json`). The 39-walk standing-height corpus was deleted 2026-09-07 as invalid. Next to record: two out-of-vocabulary searches plus one control under `policy: "tiered"`. |
| 1. The vision path in Python | Built (S1, S2, S2b). Outstanding: a vision mission re-run against the lit renderer, with cost and wall-clock recorded. |
| 2. The brain on the wire | Done (B0-B3). |
| 3. The simulator honest about safety | Done (S4 time in the loop, S5 sensor noise). |
| 4. The twin as an observer | Done (B4). Still owed against the car: B5. |
| 5. Harden | S7 (chaos and soak) not started. A memory-trend soak on the Jetson is open. |
| Hardware day | Section 3b's arrival list, then `ROBOT_MODE=hardware ROBOT_SERIAL=/dev/picar-board` and the same measurements in the real house. |

**How to record a Stage 0 walk:**
* Put the phone on a wheeled rig at floor height, about 10-13 cm, lens
  forward and level. Push it with a broom handle; never hand-hold it.
* Lock landscape orientation.
* Put the rig height in the walk's note
  (`PUT /recording/walks/{walk}/meta`).
* `/navigate` runs Claude Opus 4.5. Trust a walk's recorded `model_id`
  over any prose.

**Not on the critical path:**
* The rule-based agent: keep it as the free, deterministic way to test
  safety and memory, but do not extend it.
* `control/manual_control.py`: skipped, because the twin's D-pad covers it.
* S6's Ackermann half is retired by the differential-drive chassis.

---

## 6. Things to know before touching the code

**Architecture**

- **The brain and the robot are two processes on purpose**, even on one
  board. A loop inside `robot/server.py` would block the event loop the
  watchdog polls on. `PLAN-brain-relocation.md` has the argument.
- **Three failsafes, each covering a different failure; do not collapse
  them:**
  * B3.1, the robot watchdog: no command within `watchdog_timeout_s`
    stops the motors.
  * B3.2, `MissionRunner`: a vision timeout, plus a consecutive-failure
    budget.
  * B3.3, `brain_server`: a tick that never returns.

  The watchdog cannot see a brain that is alive but stuck. The brain's
  guards cannot see motors energised by a call that then crashed.
  `AGENT-HARNESS.md` is the reference for `control/`; read it before
  changing the mission loop.
- **The robot server arbitrates who drives** (M4):
  `stop > twin-dpad > brain > twin-local-brain`, plus driver `ros` ranked
  with the brain.
  * Every `/action` names its driver in `x-driver`; an unnamed one ranks
    as a person.
  * Authority lapses on silence; there is no release call.
  * A nav2 goal arbitrates as driver `ros` (3.23).
  * Do not add a movement route that skips this.
- **Only conditions a release can be blamed for may reach a health
  verdict** (M5). `control/health.py` decides what "unhealthy" means. When
  you add a `/health` field, put it in the verdict list or the description
  list deliberately.

**Safety**

- **"May I move" means `forward_clearance()` / `reverse_clearance()`**,
  not `path_clearance()` alone. Since 3.18 a move is vetted against two
  things in series:
  * the depth-grid cone, chosen by BODY bearing, because the grid
    publishes `pan_deg` and a peek swings the camera;
  * the chassis' swept corridor off the scan, in the body frame, with the
    lidar 4 cm ahead of centre (`LIDAR_X_M`).
- **A zone reported `unusable` never enters a comparison** (M3). As a
  distance it would stop the robot on every dropout; as clear it would
  drive through what the sensor could not see. `GET /depth` publishes the
  reduction so the twin never recomputes it.
- **The path cone is chosen by angle** (about 15.4 degrees), not by a
  fraction of the columns. A backend with a wide field must publish
  `fov_deg`.
- **Rotation is vetted too** (3.19): a turn that closes within 1.3 cm of a
  return is slowed. A turn that opens the gap is never touched.
- **Judge any safety change on ground truth** (`tests/footprint_sweep.py`),
  never on the readings the veto itself uses. 3.17's "flaky 18.0 cm" was a
  reading; the truth under it was 25 cm.
- **Thresholds counted in steps change meaning when the step does** (R1c).
  Count degrees or metres.

**Cloud and deployment**

- **`service/vision_analyze/` keeps its own copies of
  `vision_core.py`/`rooms_core.py`, deliberately.** It is a self-contained
  build context, and it calls Bedrock's Converse API where `brain/vision.py`
  calls Anthropic directly. Mirror prompt or room-logic changes by hand.
  Its tests live in `service/vision_analyze/tests/`, not `tests/`, because
  the two `tests` packages would collide.
- **Model strings differ between the simulator and the cloud on purpose.**
  * Never guess a Bedrock model id. Confirm it with a real `converse`
    call carrying a real frame.
  * Keep the code default and the template parameter in step; a deployed
    env var once silently overrode the code.
  * **Claude Fable 5.1 answers only from us-east-1** on this account, so
    `vision_core.MODEL_REGIONS` pins its client there. Re-confirm a pinned
    model from the deployed region, not a laptop.
- **The deployed twin is static on S3 + CloudFront.** A `web-twin/`
  change is not shipped until
  `bash service/static/sync.sh <bucket> <distribution-id>` has run and the
  invalidation reports `Completed`. Check parity with
  `curl https://<dist>/app.js` against `git show HEAD:web-twin/app.js`.
- **`config/robot.yaml` and `control/brain_config.py` move together.**
  `load_brain_config()` rejects unknown keys, so a new yaml beside an old
  config module is a cold-start `ValueError` in the walks Lambda.
- **A new public route needs its route-guard entry.** Five features once
  shipped dead because a route had no path pattern; the failure is quiet.
  The guards today are `tests/test_serverless_routes.py` and
  `tests/test_static_assets.py`. `tests/test_alb_routes.py` is kept for the
  deleted ECS stacks.
- **The brain cannot be deployed** (it loads YOLOE and CLIP), so the
  deployed twin reaches the local robot and brain through `service/tunnel/`:
  * one ngrok domain, split by path: `/brain/*` goes to the brain via
    `ROUTE_PREFIX`;
  * `app.js` sends `ngrok-skip-browser-warning`, to ngrok hosts only;
  * **always set `APP_SHARED_SECRET`**, read from
    `~/.vision-picar-local-secrets` (mode 600).

  Warm the models before a rig walk, or the first tiered start downloads
  weights inside `POST /mission/start`.
- **`require_secret()` is inert without `APP_SHARED_SECRET`**, which is
  how local dev and the suite run. `/health` and `/` stay unauthenticated
  on purpose.
- **A replay outlives its HTTP response.** Poll
  `GET /recording/walks/{walk}/replays` for a fresh `replayed_at`, and
  never stack replays.
- **History:** the ECS/NLB/ALB stacks were deleted 2026-09-05
  (`PLAN-aws-cost-redesign.md`). If ECS ever returns, the archive copy has
  the shared-listener pattern and the arm64 build loop.

**ROS**

- **Run the stack as `service/slam/README.md` says.** It is off by
  default. Start the robot server before the container. nav2 is judged on
  `SIM_MAP=scaled_house`. Build images that other sessions share under
  your own tag.
- **The bridge is the only door.** Its routes are capped and generic
  pass-through routes are banned (`tests/test_wall_linters.py`). Facts
  defined on both sides of the wall are registered there and must agree.
- **The image builds tf2 0.25.24 and a pinned `slam_toolbox` from
  source.** Apt's tf2 deadlocks nav2's costmaps (R6). Do not "simplify"
  the Dockerfile back to apt.

**Tests and coverage**

- **Coverage is about 93%** of `brain/`, `control/`, `robot/`, `sim/` and
  `world/`. Quote what the command prints, not this line:
  ```bash
  pytest tests/ --cov=brain --cov=control --cov=robot --cov=sim --cov=world --cov-report=term-missing
  ```
  The shortfall is deliberate:
  * Model-loading constructors cannot be covered without the models.
  * The watchdog loop runs in a live `uvicorn` subprocess that coverage
    cannot see.
  * The rule-based agent is not on the hardware path, so do not chase
    its branches.
- **A timing test that has never run alongside its neighbours has not been
  run.** One passed alone three times and failed in the suite.

**Code review** (user, 2026-10-09)

- **Run `/code-review` on every code change** before it is committed or
  merged.
- **On a risky change, also run the Thermos bug pass** on the same target,
  in parallel: `/thermos-bug-pass` runs it, and
  `docs-review/THERMOS-BUG-PASS.md` is its rubric. A change is
  risky if it touches any of:
  * a loop that can be stopped, paused or restarted with calls in flight
    (the twin's guidance loop, `MissionRunner`, `brain_server`);
  * anything that writes recorded or stored data (recordings, eval and
    replay sidecars, the inventory, maps, S3);
  * the safety or motion path, who-drives arbitration, or the watchdogs;
  * retries, timeouts or paid cloud calls;
  * ALB routes or CloudFormation.
- **Why both:** measured on six commits, `/code-review` found 17 of 18
  real bugs and the bug pass 15. Together they found all 18. The file
  above has the numbers. A finding only one of them raised is a claim to
  verify, not an order. Thermos's code-quality pass is not used; it added
  no bug and much noise.

---

## 7. Data first, then the car

**The definition of done (2026-09-25, stated by the user: "use logs and
data to make a data-driven decision").** A phase is done when its
acceptance data says so:

1. **Write the metric and the threshold down BEFORE measuring**, in the
   phase's plan entry. A threshold chosen after seeing the numbers is a
   description, not a test.
2. **Measure through the real path:** `MissionRunner` -> agent ->
   `robot/safety.py` -> backend, never a harness that moves the robot
   itself. Sweep many starts in process for the numbers. Run at least one
   mission against the live local stack through the brain's HTTP API.
3. **Record the numbers in the plan entry and pin them in a test**, so a
   regression fails the suite.
4. **Pair every stability metric with a progress metric.** Reversals
   alone reward a spin. Distance closed, arrival rate and outcome are the
   progress half.
5. **Claude runs the missions and reads the logs.** The user is not asked
   to run a simulation or fetch logs.

A failed criterion is recorded as failed, not re-scoped. An amendment to a
criterion is written down before the run it affects.

**What stays from the earlier "watched on a phone" rule (retired
2026-09-25):**
* The twin, as the surface a person drives from.
* The UI tests as a regression guard.
* **A phone-size screenshot as evidence for any change to the page.** One
  session of watching caught five defects that 1,100 tests had not.
* Drills (`control/drills.py`), for failures nothing else can provoke,
  judged by their logs.

Setup for watching anything by hand: `uvicorn robot.server:app --port 8000`
and `uvicorn control.brain_server:app --port 8001`. Then open
`http://127.0.0.1:8000/` and connect both in Settings. Restart the robot
server to reset the robot; there is no reset endpoint, on purpose.
`docs/guides/FEATURES.md` describes every UI feature end to end.
