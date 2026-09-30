# Documentation review: "ROS 2 for vision-picar"

**Doc reviewed:** the Claude Doc "ROS 2 for vision-picar"
(`https://claude.ai/artifact/JAyBPg4cDEPQdsp8Gta2r9`, tab `22ad8a18-1299`, read at rev 45). It has one tab, 12 sections and 3 embedded diagrams.
**Checked against:** the repo at `d2b1a0a` (`HEAD`, 2026-09-28, clean tree).
**Method:** read-only. I read the doc through the docs connector and checked its claims with `grep`, `sed` and `pytest --collect-only` (from `.venv`). Nothing was built, launched or moved, and the doc was not edited.

This report is written beside `docs-review/REPORT.md`, not over it. That file is the tracked 2026-09-27 whole-repo review, and CLAUDE.md section 3b points to its fix list.

> **Fixes applied 2026-09-30** (doc rev 56 → 77, repo at `465df4a`).
> - **Before this pass:** another session had already fixed V1, V3, V8 and V9 and added section 13 (the robot base).
> - **Superseded:** since this review, Waveshare confirmed that the UGV Rover kit ships the **ROS Driver** board: closed loop, 660 pulses/rev, no reflash. That replaces this report's own V3 advice (mainType 2, 1650).
> - **Doc: fixed.**
>   - Verification: V2, V4, V6, V7, V10 (the table cell, the diagram caption and the `/cmd_vel/brain` label, plus two label overlaps), V11, V12, V13, V14, V15 (byline dated 2026-09-30, with the commit named) and V16.
>   - Sec. 4: the 3.24 default.
>   - Sec. 10: rows for 3.18–3.24.
>   - Sec. 11: prerequisites, map before the goal, the goal as curl, `SIM_MAP` kept, `docker rm`, debug commands and a README pointer. `demo_slam_lap` is now flagged as starter-house only; it hard-codes that house.
>   - Sec. 12: real sub-headings, the multiplier, `/dev/picar-board`, and the 5,760-run qualifier.
>   - Sec. 13.5: male-to-male cable, and unplug the DC5525 lead.
> - **Repo: fixed.** Item 5: `CLAUDE.md` (hardware line, "Then buy") and `HARDWARE-BOM.md` 4.2's firmware-defaults bullet.
> - **Not done:** fix #13's move of Foxglove, the benchmark and Isaac ROS out of sec. 8 (a restructure, not a correctness fix), the four missing docs, and the `GUIDE-robot-base.md` vs. sec. 13 duplication (HANDOFF-2026-09-30 item 5, the user's call).

---

## 1. Summary

As an explainer this is the best ROS document in the project. It builds the mental model before the mechanics (frames → TF → odometry → SLAM → twist → nav2), maps every idea to a file in this repo, and explains its decisions: why ROS, why Humble, why the HTTP wall, why not Isaac ROS yet. Most of its ROS-stack claims check out exactly against the code: twist_mux ranks and timeouts, bridge ports, nav2 and collision_monitor parameters, SLAM resolution and range, occupancy thresholds, test counts and linter budgets.

**The single biggest problem is that the doc was frozen before the last two days of work, and everything about the physical robot is now wrong.** The chassis became the Waveshare UGV Rover on 2026-09-27/28 (PLAN 3.21). The doc still describes a two-wheel car with 65 mm wheels, 1,760 counts per revolution, a placeholder track width, an RPLidar C1 and a mainType 3 firmware flash. Section 12 also lists two safety bugs as open and "your decision" after PLAN 3.18 fixed both. It says to order the Jetson, which was ordered on 2026-09-27.

The "Run it yourself" section also stalls a fresh reader twice. It never says how to create the secrets file that `run.sh` exits without, and it sends a nav2 goal into a room that has not been mapped yet.

## 2. Inventory and scorecard

Step 1 inventory. The scope is the one doc; the other rows are the repo docs it overlaps. They are listed for the cross-doc review and not scored.

| Doc | Type | Intended reader | Scored |
|---|---|---|---|
| "ROS 2 for vision-picar" (Claude Doc) | Concept/architecture explainer, plus one how-to (sec. 11) and a checklist (sec. 12) | A software engineer new to ROS, or the owner in 6 months | Yes |
| `service/slam/README.md` | How-to + troubleshooting for the container | Someone running the ROS stack | No (overlap) |
| `PLAN-ros-alignment.md` | Decision record + measured results, R0–3.24 | The owner | No (source of truth for numbers) |
| `CLAUDE.md` sec. 3/3b | Status/orientation | A Claude session | No (overlap) |
| `HARDWARE-BOM.md`, `JETSON-BOM.md` | Reference (parts, wiring, udev) | Whoever builds the car | No (overlap) |

Scoring runs 0–2 per criterion; ×2 marks double weight. The maximum is 38.

| # | Criterion | Score | Why |
|---|---|---|---|
| 1 | Audience and prerequisites | 1 | Sec. 3 names its reader ("a software engineer"), and the intro says how to read it. No prerequisites are stated (Docker, HTTP, Python venv), and there are no links to ROS background. |
| 2 | Purpose up front | 2 | The first paragraph says what ROS does here and that the move to hardware is a config change. |
| 3 | Model before mechanics (×2) | 2 → 4 | Sections 1–2 build the concepts in order. Sec. 4 has the system diagram, sec. 5 the node graph and TF tree, and sec. 6 traces one command end to end. |
| 4 | Progressive disclosure | 2 | Concepts, then the system, then one trace, then failures, then phases, then how to run it. |
| 5 | Precise terms | 2 | Odometry vs. pose is defined once and kept (sec. 2.4), and `map` vs. `odom` is explained. The one slip is the node count (V10). |
| 6 | Concrete examples | 1 | Real commands exist, but none shows expected output. `POST /world/goal` has no host, port, curl or secret header. The only failure signature given is `docker logs … grep`. |
| 7 | Rationale and tradeoffs (×2) | 2 → 4 | Why ROS, why Humble (JetPack 6), why the wall, why a C++ plugin, why the bridge pulls, and Isaac ROS weighed and deferred. |
| 8 | Honest gaps | 2 | Failed criteria are recorded as failures, sec. 9 lists what broke, and the Isaac claim is flagged "from memory". |
| 9 | Navigable structure | 1 | Sec. 8, "The walls we kept", also holds the Foxglove how-to, the HTTP benchmark and Isaac ROS. Sec. 7 holds map storage and the S3 design. Sec. 12's sub-headings are plain paragraphs, not headings. |
| 10 | Verifiable and current | 0 | The byline says 2026-09-26, but the doc contains 2026-09-27 material (3.17). There is no commit or version. The chassis, lidar and firmware facts and two safety items are wrong at `HEAD` (section 3 of this report). |
| 11 | Actionable outcome (×2) | 1 → 2 | Sec. 11 stalls on the missing secrets file and on goal-before-map (section 6). Sec. 12's firmware instructions would flash the wrong constants. |
| 12 | Units and conventions | 1 | SI units throughout. It states ROS's x-forward/y-left/CCW and the project's x-east/y-south/clockwise. It never names REP 103, never says which wheel direction or encoder sign is positive, and its encoder count is stale. |
| 13 | Reproducibility | 1 | The image build is pinned (tf2 0.25.24, Cyclone). Missing: the `.venv` requirement, the secrets file, a Linux variant (`--network host`, which README:166 has) and udev/`dialout` for `/dev/picar-board`. |
| 14 | Hardware facts | 0 | The chassis, wheels, encoder, lidar and firmware mode are stale. There is no wiring or power budget, and no link to `HARDWARE-BOM.md`, which has them. |
| 15 | Operational safety | 1 | The silence chain is excellent (0.25 + 0.25 s, 1 s watchdog, 1.5 s heartbeat). The e-stop and battery get one line each. It lists two fixed safety bugs as open and describes the pre-3.18 single-cone check. |
| 16 | Debuggability | 1 | It covers Foxglove, `/world/error` and `docker logs`. There is no `ros2 topic hz/echo`, `view_frames` or `docker exec picar-ros ros2 …`, and it never links README's failure-signature table (README:232–240). |

| Doc | Weighted total | Lowest-scoring criteria |
|---|---|---|
| "ROS 2 for vision-picar" | **25 / 38 (66%)** | 10 Verifiable/current (0), 14 Hardware facts (0), 11 Actionable (1 ×2) |

The explanatory core would score about 34/38. The robot-facing and how-to parts are what pull the total down.

## 3. Verification failures (Step 2)

| # | Doc says | Code or plan says | Doc location | Repo location |
|---|---|---|---|---|
| V1 | "those are 0.0325 m and 0.172 m" (wheel radius, track width) | Wheel radius is **0.040** m | sec. 2.5 | `picar.urdf.xacro:23`, `controllers.yaml:21`, `sim/mock_robot.py:83`, `robot/hardware_robot.py:50` |
| V2 | Track width 0.172 m is "a placeholder until the chassis is built" (also sec. 10 R0, sec. 12 Measure) | 0.172 is the UGV Rover firmware's own value: "It is the right robot now." Only skid steer's effective track (`wheel_separation_multiplier`) is unmeasured | sec. 2.5, 10, 12 | `sim/mock_robot.py:85-92`; PLAN-ros-alignment.md 3.21 table (~l.1800) |
| V3 | "Flash … closed-loop mode (`mainType` 3) … wheel size (65 mm), encoder count (1,760 per turn)" | 80 mm tyres, **1650** pulses/rev, **mainType 2** ("UGV Rover") | sec. 12 Buy and build | `sim/mock_robot.py:78-84`, `robot/hardware_robot.py:50-52`, PLAN 3.21 |
| V4 | "The car has two driven wheels and no steering" | "6 wheels, 4 driven, skid steer" | sec. 2.5 | PLAN-ros-alignment.md:1795-1796 |
| V5 | "the real car will carry an RPLidar C1" | The kit's lidar is a D500 (LDROBOT STL-19P), 12 m, 10 Hz | sec. 2.8 | PLAN 3.21 table; `sim/mock_robot.py:100-103` |
| V6 | "turns 0.3 m/s into two wheel speeds of 9.2 rad/s each" | 0.3 / 0.040 = **7.5 rad/s** (9.2 was the old 0.0325 m wheel) | sec. 6 step 7 | `controllers.yaml:21` |
| V7 | "30 cm doors leave the 20 cm-wide chassis 5 cm a side" | Body is 0.253 × **0.231** m; nav2 footprint ±0.1155 | sec. 9 table | `picar.urdf.xacro:26-27`, `nav2.yaml:74,105` |
| V8 | "Safety: an oblique approach can get past the stop … That's your decision to make" and "the wall stop is 1.5 cm short in 3 runs out of 5" | Both fixed in 3.18. The first was fixed with a swept-corridor check off the 360° scan. The second was a panned camera, not a stalled loop | sec. 12 Still open | PLAN-ros-alignment.md:1487 (3.18); CLAUDE.md sec. 3 "3.18" row |
| V9 | "Order the Jetson build as priced in `JETSON-BOM.md`" | Dev kit ordered 2026-09-27; the chassis is not ordered | sec. 12 | CLAUDE.md sec. 3b, first bullet |
| V10 | "about 20 in the container" (nodes) | The sec. 5 diagram caption says "11 nodes". The launch file has 15 `Node()` entries, 2 of them one-shot spawners, so 13 long-running processes (+2 controllers inside `ros2_control_node`) | sec. 3 table, sec. 5 caption | `picar.launch.py:42-80` |
| V11 | "the bridge and `world/ros_world.py` flip between the two, there and nowhere else" | The bridge converts only inbound (it reverses the robot's scan into ROS order) and says outbound conversion "is the consumer's job". `robot/ros_drive.py` also maps verbs to signed ROS twists | sec. 2.1 | `picar_bridge/bridge.py:24-27, ~395`; `robot/ros_drive.py:167` |
| V12 | nav2 goals: "A tap on the phone's D-pad cancels it" (the only arbitration mentioned) | Since 3.23 a goal arbitrates as driver `ros`: it is refused `preempted` while the brain or a person holds the robot, and blocks other autonomous `/action`s while active | sec. 2.9, sec. 7 | PLAN-ros-alignment.md:2026 (3.23) |
| V13 | Safety: "checks the path ahead first: nothing within 20 cm"; "`vet_wheel_velocity()` clamps forward speed if a wall is within 20 cm" | Since 3.18/3.19 there are two checks in series, the camera cone plus the chassis' swept corridor off the scan, and turns that close on an obstacle are scaled | sec. 6 steps 3 and 9 | CLAUDE.md sec. 6 ("Since 3.18…", "Since 3.19…"); `robot/safety.py` `forward_clearance()`, `pivot_scale()` |
| V14 | "The wheel speed control itself … the ESP32 motor board runs it at about 100 Hz" | "the stock firmware runs mainType 2 OPEN LOOP": no speed control until the firmware change in sec. 12 | sec. 8, "Does HTTP hold" | PLAN 3.21 (~l.1810) |
| V15 | Byline date 2026-09-26 | The doc includes 3.17 material (wall linters, Foxglove, HTTP rate), dated 2026-09-27 | byline | PLAN-ros-alignment.md:1357 |
| V16 | `ROBOT_MODE=hardware ROBOT_SERIAL=/dev/picar-board` | No udev rule creates `/dev/picar-board`. The rule and the `dialout` requirement are documented only elsewhere | sec. 8 code block | HARDWARE-BOM.md:119-120, 297, 430; HARDWARE-READINESS.md:464-467 |

**Checked and correct.** These need no action:

- twist_mux: teleop 100 / brain 50 / nav 50, 0.25 s timeouts (`twist_mux.yaml`), and 0.25 + 0.25 = 0.5 s silence (`controllers.yaml:28`).
- Container ports: bridge `:8090`, Foxglove `127.0.0.1:8765`, the brain polled at 2 Hz (`bridge.py:88`).
- Test and linter counts: 19 offline + 5 live brain-view tests; `MAX_DUPLICATES` 10 and `MAX_BRIDGE_ROUTES` 13 (`test_wall_linters.py:61,63`); 22 contract tests × 6 backends.
- nav2: NavFn, RPP at 0.20 m/s with 0.30 m lookahead, spin/backup/wait behaviours, and collision_monitor in `approach` mode with `time_before_collision` 1.0 (`nav2.yaml:39-44,132-150,181-187`).
- SLAM and the map: resolution 0.05, range 12 m, `restamp_tf` (`slam.yaml`); occupancy thresholds 65/25 and `map_id` `slam-<session>` (`world/ros_world.py:37-38,121`).
- Build: Cyclone DDS and tf2 0.25.24 (`Dockerfile:21,34,75`).
- Frames: every name in sec. 2.2 matches the URDF; laser at 0.10 m and pan axis 8 cm ahead (`xacro:30-31`).
- Timeouts and thresholds: heartbeat 1.5 s (`hardware_robot.py:56`), watchdog 1.0 s and `min_distance_cm` 20 (`config/robot.yaml:93-94`), and D-pad speed 50 → 0.3 m/s (`robot/server.py:134`, `ros_drive.py:192`).
- The kitchen goal (6.45, 1.35) (`tests/demo_nav_goals.py:43`), and `/cmd_vel_mux` in the diagram (`picar.launch.py`, `nav2.yaml:176`).

## 4. Per-doc findings

**Sec. 2.5, 2.8, 6, 9, 10, 12: the chassis.** Every number from the Yahboom build needs to become the UGV Rover's: V1–V7, V14. "Two driven wheels and no steering" also needs a sentence on skid steer. `diff_drive_controller` is still the right controller, but the effective track is wider than the geometric one, which is why `wheel_separation_multiplier` exists.

**Sec. 12 "Still open in software".** Delete the two safety bullets or rewrite them as "fixed in 3.18: here is what changed and what it teaches". Keep the lesson "a stop check must cover the travel until the next check": 3.22's look-ahead is exactly that lesson built. Add 3.22 (guarded verbs) and 3.24 (`drive: ros` as the car default), which change the answer to "what is left for hardware day".

**Sec. 12 "Buy and build".** "Order the Jetson build" should become "Jetson ordered 2026-09-27; chassis waits on Waveshare's 25 W answer". The firmware bullet should say mainType 2 with its stock constants, closed loop being the change.

**Sec. 4.** "ROS is off by default (`drive: direct`)" is still true for the sim. After 3.24 it needs "…and `drive: ros` is planned as the car's default once four gates pass", or a reader will think direct mode is how the car drives.

**Sec. 3 and sec. 5.** "about 20 in the container" and "11 nodes" contradict each other and the launch file (V10). Give one number with a footnote on spawners and lifecycle managers. The diagram label "/cmd_vel/teleop, /brain" reads as a topic called `/brain`; it should be `/cmd_vel/teleop`, `/cmd_vel/brain`.

**Sec. 6.** Steps 3 and 9 describe the single-cone check (V13). One added clause on the swept corridor and pivot scaling fixes it. The "about 1.5 s" per tap and "four boundary crossings" have no source in the repo; either cite one or drop the number.

**Sec. 8.** The heading promises "the walls we kept", but the section also runs the Foxglove how-to, a benchmark and an Isaac ROS evaluation. The `/dev/picar-board` code block needs a link to HARDWARE-BOM's udev/`dialout` lines (V16).

**Sec. 11.** See section 6 of this report: the secrets file, mapping before the goal, curl with host and secret for `/world/goal`, `docker rm -f picar-ros` on "Back to normal", and a Linux line.

**Byline.** Date it to when it was last true, and put the commit it was checked against (`d2b1a0a` after fixes) under the title. The doc is outside git, so nothing else ties it to the code.

## 5. Cross-doc issues and reading path

**Contradictions**

| Topic | This doc | Other doc | Code/plan |
|---|---|---|---|
| Lidar | RPLidar C1 (sec. 2.8) | CLAUDE.md sec. 3b l.190: "RPLidar C1"; sec. 3 "Home" row | PLAN 3.21: D500 |
| Wheel radius / encoder | 0.0325 m, 1760 | CLAUDE.md sec. 3 R0 row (l.153): "0.0325m, 1760 counts/rev, track width … PLACEHOLDER" | 0.040 m, 1650, 0.172 real |
| Firmware defaults | "mainType 3" flash needed (sec. 12) | HARDWARE-BOM.md:334: the defaults 0.080 m / 1650 / 0.172 are "wrong for this chassis" | PLAN 3.21: they are this chassis |
| Safety bugs | Open (sec. 12) | CLAUDE.md 3.18 row: fixed | PLAN 3.18: fixed |
| Node count | "about 20" / "11" | none | 13 processes |

CLAUDE.md and HARDWARE-BOM.md are stale on the same facts. The chassis change was recorded only in PLAN 3.21 and the code.

**Duplication**

- The sec. 11 run commands duplicate `service/slam/README.md:116-131`. The two have already drifted: README adds `--restart unless-stopped`, a Linux variant, `docker rm -f` and a troubleshooting table. **Keep the commands in README and have sec. 11 link to it.**
- The measured-results table (sec. 10) duplicates PLAN-ros-alignment.md 3.1–3.16 and CLAUDE.md sec. 3's rows. That is acceptable as a summary, but it cites "3.1–3.16" and should cite through 3.24.
- The ten-concept duplicate registry (sec. 8) duplicates `tests/test_wall_linters.py`'s own registry. That is fine because the test is the authority, but say so in the doc.

**Coverage gaps (code without a mention in this doc)**

- `picar_bridge/convert.py` and the bridge's route list.
- The `/nav/stats` routes.
- `sim/fake_esp32.py`'s heartbeat default (3 s, overridden to 1.5 s).
- 3.19 (pivots), 3.20 (camera centred at start), 3.22 (guarded verbs), 3.23 (goal arbitration) and 3.24 (`drive: ros` default).

No code was found for anything the doc describes as built. Map saving, S3 backup and frontier search are correctly marked "proposed", and nothing in the repo implements them.

**Proposed reading path.** Today there is no link in either direction between this doc and the repo.

1. `README.md`: what the project is. It should link to this doc as "ROS, explained".
2. **This doc, sections 1–10**: the mental model.
3. `service/slam/README.md`: how to run it, check it and diagnose it. Sec. 11 should shrink to a pointer here.
4. `PLAN-ros-alignment.md`: the measured record and open questions.
5. `HARDWARE-BOM.md` → `JETSON-BOM.md`: hardware day. Sec. 12 should link both.

## 6. Empirical spot check (Step 5)

I dry-ran two how-to parts by inspection, reading as a new user with only the doc.

**A. Sec. 11 "Run it yourself" (Mac, Docker Desktop)**

1. Step 1, `docker build -t vision-picar-ros service/slam`: works; the Dockerfile is there.
2. Step 2, `SIM_MAP=scaled_house ROBOT_DRIVE=ros WORLD_MODE=ros bash service/tunnel/restart.sh`: **stalls.**
   - `restart.sh` launches `run.sh`, which runs `source .venv/bin/activate` (`run.sh:29`). The doc never mentions the venv.
   - `run.sh` then exits with "missing ~/.vision-picar-local-secrets" (`run.sh:32`). The doc never says to create that file or that it must define `LOCAL_SECRET`.
   - `restart.sh` then waits and reports failure. The reason is in `~/.vision-picar-tunnel.log`, which the doc also never mentions.
3. Step 3, `source ~/.vision-picar-local-secrets`: the same missing file.
4. "Watch it map": works once running. The reader has to know the twin is at `http://127.0.0.1:8000/` and needs the secret entered in Settings. Neither is said here.
5. "Send it somewhere: `POST /world/goal {…}`": **stalls twice.**
   - There is no host (`:8000`), no curl line and no `x-app-secret` header, so the call is a 401 or a guess.
   - The kitchen is only reachable if it has been mapped. Sec. 7 says "Mapping comes first", but sec. 11 goes straight from a few D-pad taps to a kitchen goal. README:239 gives the result: the goal aborts "off the global costmap". The step should say to run `python -m tests.demo_slam_lap` first.
6. "Use the motor-board code path": works (`factory.py:111-116`; `HardwareRobot.__getattr__` exposes the sim's `world`). The replacement settings silently drop `SIM_MAP=scaled_house`, so the reader lands in the starter house, whose 30 cm doors defeat nav2 (sec. 2.8's own point).
7. "Back to normal": restarts the servers but leaves `picar-ros` running. Its plugin then logs `POST /wheels … failed` indefinitely. README:156 says `docker rm -f picar-ros`; the doc doesn't.

**B. Sec. 12 "What is left for hardware day"**, read as the person building the car:

1. "Order the Jetson build": already done (V9). A reader might order a second one.
2. "Flash the motor board with … mainType 3 … 65 mm … 1,760": **would program the wrong chassis's constants** into the UGV Rover (V3). This is the most harmful line in the doc.
3. "Measure … track width (0.172 m is a placeholder)": wastes a measurement. The number to measure is the skid-steer multiplier, not the track.
4. "`ROBOT_MODE=hardware ROBOT_SERIAL=/dev/<board>`": the reader doesn't know that two CP210x devices enumerate unpredictably or that `dialout` is needed. The first run fails with permission denied or opens the lidar as the motor board (HARDWARE-BOM.md:430).
5. The "Still open" safety bullets would send the reader to re-investigate two bugs that are already fixed (V8).

## 7. Prioritized fix list

Ordered by impact on someone building, running or debugging the robot. Effort: S = minutes, M = an hour, L = more.

| # | Fix | Effort |
|---|---|---|
| 1 | Sec. 12 firmware bullet: mainType 2, 80 mm / 1650 / 0.172, closed loop as the firmware change. Remove "65 mm, 1,760, mainType 3" (V3). | S |
| 2 | Sec. 12: remove or re-label the two safety bullets as fixed in 3.18 (V8). Mark the Jetson as ordered (V9). | S |
| 3 | Sec. 11: add the prerequisites (`.venv` + `pip install -r requirements.txt`, and creating `~/.vision-picar-local-secrets` with `LOCAL_SECRET=…`). Add `demo_slam_lap` before the goal. Give the goal as a full `curl -H "x-app-secret: …" -X POST localhost:8000/world/goal …`. Add `docker rm -f picar-ros` to "Back to normal". Or replace sec. 11 with a link to `service/slam/README.md` sec. 2 (see #9). | S–M |
| 4 | Chassis sweep: sec. 2.5 (radius 0.040, skid steer, track real), 2.8 (D500), 6 step 7 (7.5 rad/s), 9 (23 cm body), 10 R0 wording, 8 ("100 Hz speed control" only after the firmware change) (V1, V2, V4–V7, V14). | M |
| 5 | Fix the same stale chassis and lidar facts in CLAUDE.md (l.153, l.190) and HARDWARE-BOM.md:334, so the three agree. | S |
| 6 | Sec. 6 steps 3 and 9, and sec. 2.9/7: add 3.18's swept corridor, 3.19's pivot scaling and 3.23's goal arbitration (V12, V13). | S |
| 7 | Sec. 4 / sec. 12: add 3.22 (guarded verbs) and 3.24 (`drive: ros` as the car default, four gates). | M |
| 8 | Sec. 8 code block: link HARDWARE-BOM udev/`dialout` (V16). Sec. 12: link `HARDWARE-BOM.md` and `JETSON-BOM.md` for wiring and power. | S |
| 9 | Make `service/slam/README.md` the one home of run commands and failure signatures. Link it from sec. 11 and a new "When it breaks" line (README:232-240). | S |
| 10 | Add a short debugging block: `docker exec picar-ros bash -lc 'ros2 topic hz /scan'`, `ros2 topic echo /cmd_vel_mux --once`, `ros2 run tf2_tools view_frames`, `ros2 lifecycle get /bt_navigator`, `ros2 doctor`. | S |
| 11 | One node count everywhere (13 processes), and fix the diagram label `/brain` → `/cmd_vel/brain` (V10). | S |
| 12 | Byline: the true as-of date plus "checked against commit `<sha>`". Update the "sections 3.1–3.16" citation (V15). | S |
| 13 | Restructure sec. 8: keep "the walls". Move Foxglove to a "Seeing inside" section next to debugging, and move the HTTP benchmark and Isaac ROS to a "Decisions deferred to hardware" section. Make sec. 12's pseudo-headings real headings. | M |
| 14 | Sec. 2.1: say precisely where the frame conversions happen: bridge for the inbound scan, `world/ros_world.py` for pose/map/goals, `ros_drive.py` for turn sign (V11). Name REP 103 and the positive wheel/encoder direction. | S |

## 8. Missing docs to write

- **Hardware bring-up runbook for the UGV Rover + Jetson**: flash JetPack 6 → udev rules for the two CP210x devices → `dialout` → firmware change and verify → `ROBOT_MODE=hardware` smoke test on blocks → first floor run, each with the expected output.
- **ROS debugging cheat sheet**: `docker exec` recipes for `topic hz/echo`, `view_frames`, lifecycle states and `ros2 doctor`, plus a table of failure signatures. Grow it from README:232-240.
- **Operational safety page**: the e-stop wiring, the silence chain (0.25/0.5/1.0/1.5 s) as one diagram, velocity limits per path (verbs 0.6 m/s, nav2 0.2 m/s, `controllers.yaml` 0.6 m/s / 6 rad/s), battery handling, and what each crash (brain, robot server, container, Wi-Fi) leaves the wheels doing.
- **Calibration procedure**: measuring `wheel_separation_multiplier` for skid steer (a 10-turn spin test), the camera height and tilt, and the lidar-to-bumper offsets, written into the xacro's `[PLACEHOLDER]` block.
