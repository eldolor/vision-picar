# Documentation review — vision-picar

Reviewed 2026-09-27 against the **working tree** on `dev`. That includes uncommitted edits to `robot/safety.py`, `sim/grid_world.py`, `PLAN-ros-alignment.md` 3.18 and the untracked `tests/test_footprint_safety.py`. The review was read-only: no doc was edited, and nothing was built, launched or moved. Claims were checked with `grep`, `pytest --collect-only`, one targeted test run and arithmetic. Each finding carries a file:line reference on both sides. Anything established only by reading code is marked **UNCONFIRMED**.

Scope: all 33 tracked Markdown docs (about 24,000 lines). The four ROS 2 packages in `service/slam/`, whose comment-heavy Dockerfile, launch file, YAML and xacro are the only documentation they have, are scored together as one doc.

> **Batch 1 applied, later on 2026-09-27.**
> - **Done:** fixes #1 (HARDWARE-BOM correction 5), #2 (chassis-width comment in `robot/safety.py` and HARDWARE-READINESS 5.5), #3 (deploy command, plus `service/lambda/build.sh`'s printed command) and V7 (HARDWARE-READINESS REVERSE claim).
> - **#7 and V9:** already resolved before batch 1. The plan now records 3.18's measured results, and CLAUDE.md:167-168 say the findings were fixed.
> - **Batch 2 (same day):** fix #4 (`service/slam/README.md`), #5 (AGENT-HARNESS 4.1/4.2 and the watchdog inputs, rewritten against the code, plus the stale authority comments in `robot/interface.py` and `robot/server.py`), #10 (package manifests; `rosdep check` passes in the image) and #11 (JetPack 6 / Humble recorded as the working choice in the three BOMs).
> - **V4 fixed in code** (`PLAN-ros-alignment.md` 3.21): a nav2 goal is now arbitrated as an autonomous driver.
> - **Batch 3 (2026-09-28, branch `docs-review`):** fixes #6, #12-#19 and most of #15 and #20 applied across 25 docs. CLAUDE.md dropped from 2,133 to about 1,520 lines; its dated history moved verbatim to `docs/archive/CLAUDE-history-2026-09.md`. `web-twin/README.md` rewritten. New `evaluations/tier-decomp/README.md`. **Not done:** a 300-line CLAUDE.md, splitting PLAN-onboard-perception.md, and `INTRODUCTION.html` (hand-built; now out of sync with the .md).
> - **Everything below is the original review, unchanged.**

---

## 1. Summary

The reasoning in these docs is unusually good. Every doc that records a decision scores 2 on "rationale and tradeoffs". Measurements reproduce from the checked-in records exactly: P23/P24 were recomputed frame for frame. Failures, xfails and causes that are "not established" are all recorded honestly.

What fails is **currency**. "Verifiable and current" scores 0 on 15 of 33 docs. Every doc is an append-only journal. When a decision is reversed, the new text is added further down, and the old text is not marked where a reader will actually meet it. The board decision shows how far this goes: it is stated four different ways (Jetson; Pi + Hailo-8L; "not decided"; "no Jetson, final"). CLAUDE.md's own **"Then buy"** section names the rejected part at about $330 below the real total.

**Biggest single problem:** the facts someone needs to *run the robot safely* are the ones that have drifted most, and they are not in the repo at all.
- **Motor board.** HARDWARE-BOM.md's example motor command runs the motors at **full power** in the firmware's default mode.
- **Chassis width.** Two docs and a code comment say the safety cone's chassis width "errs wide". It errs **narrow**.
- **Arbitration.** The written rule ("equal rank passes") is no longer what the server does. Nav2 goals skip the robot server's arbitration completely.
- **Running the ROS stack.** The only complete run command lives in a private Claude memory file outside the repo.

---

## 2. Inventory and scorecard

The weighted total counts criteria 3, 7 and 11 twice. N/A criteria are dropped from the maximum. "Lowest" lists the criteria that scored 0, or 1 where nothing scored 0. Sorted worst first.

| Doc | Type · reader | Weighted | % | Lowest-scoring criteria |
|---|---|---|---|---|
| `web-twin/README.md` | component README · new contributor | 10/34 | **29%** | 3 Model 0, 10 Current 0, 11 Actionable 0, 13 Repro 0, 16 Debug 0 |
| `README.md` | landing page + build journal · visitor | 18/36 | 50% | 10 Current 0, 13 Repro 0, 14 Hardware 0, 16 Debug 0 |
| `HANDOFF-2026-09-15.md` | session snapshot · next session | 18/36 | 50% | 10 Current 0, 11 Actionable 0 |
| `PLAN-onboard-perception.md` (9,012 lines) | plan + lab notebook · owner, future session | 20/38 | 53% | 4 Progressive 0, 9 Navigable 0, 10 Current 0 |
| `PLAN-ar-guidance.md` | feature spec + changelog · frontend dev | 17/32 | 53% | 4 Progressive 0, 10 Current 0 |
| `evaluations/README.md` | reference index · evaluator | 19/32 | 59% | 9 Navigable 0, 10 Current 0 |
| `PLAN-ros-alignment.md` | plan + build log · owner, future session | 23/38 | 61% | 1, 4, 5, 6, 9, 10, 11, 13, 16 all at 1 |
| `PLAN-brain-relocation.md` | plan · whoever does B5 | 22/36 | 61% | 10 Current 0, 13 Repro 0, 14 Hardware 0, 16 Debug 0 |
| `FEATURES.md` | reference · "how does X work" | 21/34 | 62% | 10 Current 0, 13 Repro 0 |
| `PLAN-mapping.md` | decision record · owner | 21/34 | 62% | 10 Current 0, 16 Debug 0 |
| `HARDWARE-READINESS.md` | explainer + pre-flight · hardware day | 23/36 | 64% | 10 Current 0, 14 Hardware 0, 16 Debug 0 |
| `CLAUDE.md` (2,111 lines) | orientation · every session | 25/38 | 66% | 4 Progressive 0, 10 Current 0 |
| `PLAN-sim-hardening.md` | plan + build log · dev | 25/38 | 66% | 10 Current 0 |
| `JETSON-BOM.md` | shopping list · buyer | 23/34 | 68% | 10 Current 0 (three arithmetic errors) |
| `service/slam/` (4 packages, no README) | reference-by-comment · ROS dev | 26/38 | 68% | 1, 4, 6, 9, 10, 11, 13–16 all at 1 |
| `BOM-COMPARISON.md` | decision record · buyer | 23/32 | 72% | 1, 6, 10, 11, 14, 15 at 1 |
| `BOM.md` (superseded) | historical | 23/32 | 72% | 5, 6, 10, 11 at 1 |
| `EDGE-PERCEPTION-BENCH.md` | evaluation report | 26/36 | 72% | 10 Current 0 |
| `HANDOFF-2026-09-13.md` | session snapshot | 26/36 | 72% | 1, 3, 5, 10, 11 at 1 |
| `evaluations/gpu/README.md` | evaluation record | 23/32 | 72% | 10 Current (a GPU/CPU claim later disproved) |
| `AGENT-HARNESS.md` | architecture reference · editor of `control/` | 25/34 | 74% | 10 Current 0 |
| `evaluations/gpu/{softgate,pacing}/README.md` | evaluation records | ~24/32 | ~75% | 10 Current ("shipped" is ambiguous) |
| `HARDWARE-BOM.md` | reference + bring-up · hardware day | 29/38 | 76% | 10, 11, 13, 14, 15, 16 at 1 |
| `PLAN-teleop-robot.md` | plan + log · phone-walk operator | 26/34 | 76% | 10 Current 0 |
| `tools/hailo/README.md` | how-to · compile-loop re-runner | 25/32 | 78% | 10 Current 0 |
| `evaluations/{trt,hailo}/README.md` | evaluation records | 25/32 | 78% | 10 Current (no "Hailo closed" note) |
| `PLAN-microduck-transplants.md` | plan · safety/health/authority | 30/38 | 79% | 1, 4, 10, 11–14 at 1 |
| `evaluations/gpu-yoloe/README.md` | evaluation record | 26/32 | 81% | 13 (corpus not pinned) |
| `PLAN-aws-cost-redesign.md` | decision record + runbook · operator | 28/34 | 82% | 4, 9, 10, 11, 13 at 1 (**deploy command is unsafe**) |
| `INTRODUCTION.md` | concept explainer · non-technical reader | 25/30 | 83% | 10 Current 0, 14 Hardware 0 |
| `EVAL-navigate-models-2026-09-22.md` | evaluation report | 29/34 | 85% | 13 Repro 0 (no script, model ids or result path) |

**How to read the scores.** A high score does not mean a doc is safe to follow. INTRODUCTION.md scores 83% because it is well written, yet it describes a Raspberry Pi 5 with a steering servo. PLAN-aws-cost-redesign.md scores 82%, and its deploy command still ships an unauthenticated endpoint. Read column 10 as a flag on its own.

**Patterns across the whole set:**
- **Criterion 7 (rationale)** scores 2 on every decision doc. This is the strength to keep.
- **Criterion 10 (current)** scores 0 on 15 docs. This is the failure to fix.
- **Criterion 16 (debuggability)** is weak on every ROS-facing doc. None of them contains a `ros2 topic hz`, `ros2 control list_controllers`, `tf2_echo`/`view_frames` or Foxglove recipe, although `tf2-tools` and `foxglove_bridge` are both installed in the image.
- **Criterion 12 (conventions).** The xacro header, the bridge docstring and `world/ros_world.py` handle REP 103 well. But no prose doc states the project's own convention: `bearing_deg` is clockwise-positive, which is the opposite of ROS.

---

## 3. Verification failures (doc claim vs. code)

### 3.1 Safety-relevant

| # | Doc claim | Code reality |
|---|---|---|
| V1 | `HARDWARE-BOM.md:269` `T=1` is "Speed control, left/right `{"T":1,"L":0.5,"R":0.5}`". `:282` selects stock `mainType` 2 via `T=900`. | `sim/fake_esp32.py:161-165`: outside mainType 3, `T=1` is **open-loop PWM = L×512**, clamped to ±255. The doc's own example therefore gives PWM 256 → 255, which is **full power** (~1 m/s no-load), not 0.5 m/s. `robot/hardware_robot.py:22-25` and PLAN-ros-alignment 3.16 both say so. HARDWARE-BOM's header (`:4-5`) promises that corrections will be appended, and none was. |
| V2 | `HARDWARE-READINESS.md:388-390` and `robot/safety.py:141-145`: `CHASSIS_WIDTH_CM = 16.5` "over-states the Yahboom chassis (148mm) and so errs wide". | 148 mm is the deck width. With the wheels on, the chassis is 198 mm: `nav2.yaml:73` footprint ±0.099 m, the xacro's `wheel_separation` + `wheel_width`, and working-tree `robot/safety.py:84` `FOOTPRINT_WIDTH_M = 0.198`. The same file (`safety.py:89`) says "a 19.8cm chassis". So the ±15.4° cone errs **narrow**. This is plausibly related to the 3.0 cm oblique-approach finding in PLAN-ros-alignment 3.17. |
| V3 | `AGENT-HARNESS.md:275` and `PLAN-microduck-transplants.md:519-523`: "**Equal rank passes**". | `robot/server.py:305-311` refuses a second driver at the autonomous rank ("one autonomous driver at a time") until the holder lapses. This changed in R2b. The docs' order (`AGENT-HARNESS.md:260`) also leaves out the `ros`, `teleop` and `teleop-operator` drivers (`robot/interface.py:47-58`), and still ranks the local brain that was deleted 2026-09-25. |
| V4 | `service/slam/.../twist_mux.yaml:3-4` and PLAN-ros-alignment 3.10: "two autonomous sources share a rank, and robot/server.py's M4 arbitration already refuses a second one at /action". | `POST /world/goal` (`robot/server.py:670-674`) calls no `arbitrate()` and takes no `x-driver`. nav2 then publishes on `cmd_vel/nav` at priority 50, the same as `cmd_vel/brain` (`twist_mux.yaml:18-24`). The bridge also maps driver `ros` onto `cmd_vel/nav` (`bridge.py:79-83`), even though 3.13 (`:931`) calls that input "reserved for nav2". What happens when a brain mission and a nav2 goal run together is undocumented (**UNCONFIRMED at runtime**). |
| V5 | `AGENT-HARNESS.md:300-305` lists refusal reasons `safety_distance`, `preempted`, `watchdog` and `mission_ended`. `:466` (invariant 8): "every refusal names a machine-readable reason". | `mission_ended` appears in no `.py` file. `_HaltGate` raises `MissionHalted`, which `tick()` swallows (`control/mission_runner.py:131-155`, `:425-428`). The server also emits `ros_unavailable` (`server.py:464`), `not_the_actuator` (`:498`) and `unsupported` (`:530`), none of which are documented. |
| V6 | `AGENT-HARNESS.md:352`, `:358` and `robot/server.py:50`: the watchdog is fed only by `/action` and `/stop`. | `POST /wheels` also refreshes `last_command_at` (`server.py:501`). A standing wheel command has a different lost-comms profile from a verb, and no doc describes it. |
| V7 | `HARDWARE-READINESS.md:333-335`: "`REVERSE` also skips the check". | This has been false since R2b. `REVERSE_ACTIONS` are vetted against the rear scan beams (`robot/safety.py:58`, `:493`). |
| V8 | `PLAN-aws-cost-redesign.md:462-465`: the deploy command passes only `LambdaCodeBucket VisionCodeKey WalksCodeKey`. | `cloudformation/serverless.yaml:85-86`, `:187` and `:209` set `APP_SHARED_SECRET` only when `VisionSharedSecret` / `WalksSharedSecret` are non-empty. On a **fresh** deploy, the documented command therefore publishes an unauthenticated Bedrock endpoint and an unauthenticated walk-DELETE endpoint. The doc's own `:453` says "Empty means no auth, so always pass both". |
| V9 | `CLAUDE.md:167`: the two 3.17 safety findings are "recorded and NOT fixed". PLAN-ros-alignment `:1485` has the heading "…fixed", while its body (`:1551`) says "to be confirmed RED before the fix is written". | The working tree already contains the fix: `robot/safety.py:70-94` corridor check, `sim/grid_world.py` +86 lines, and `tests/test_footprint_safety.py` (10 tests, untracked). Three sources give three different states. |
| V10 | `CLAUDE.md` §6 and `robot/interface.py`: an unnamed `/action` ranks as manual (driver `unknown`). | Under `drive: ros`, `robot/ros_drive.py:118-120` forwards that name, and `bridge.py:445-447` answers 400 "unknown driver". The drivers the bridge accepts are only `twin-dpad`, `brain` and `ros`, so a bare `curl /action`, `teleop` and `teleop-operator` all fail under ROS (**UNCONFIRMED at runtime**). |

### 3.2 ROS stack

| # | Doc claim | Code reality |
|---|---|---|
| R1 | `PLAN-ros-alignment.md:20-21` ("`robot_localization` … all run") and the `:53` diagram. | Not installed (`service/slam/Dockerfile:13-31`), not launched (`picar.launch.py`), and in no `package.xml`. Odometry comes only from `diff_drive_controller` (`controllers.yaml:25`). |
| R2 | `PLAN-ros-alignment.md:1022-1027` run command: `-p 8090:8090` only. `:1395` and `CLAUDE.md:892` say Foxglove is "on 127.0.0.1:8765". | `picar.launch.py:80` binds `0.0.0.0:8765`, and the command publishes no 8765, so Foxglove is unreachable from the host. The "127.0.0.1" holds only with `-p 127.0.0.1:8765:8765`, which appears in no repo doc. Under `--network host` (the natural choice on a Jetson) it is exposed on every interface. |
| R3 | The same command uses `ROBOT_URL=http://host.docker.internal:8000`. | `host.docker.internal` exists only under Docker Desktop. On Linux, which is what the Jetson runs, it needs `--add-host=host.docker.internal:host-gateway` or `--network host`. `service/tunnel/run.sh:83` also binds the robot server to `127.0.0.1`, which a bridge-networked Linux container cannot reach. |
| R4 | PLAN-ros-alignment 3.17 (`:1393`): "`BRAIN_URL=""` turns it off". | `bridge.py:142` defaults `BRAIN_URL` to `http://host.docker.internal:8001/brain`. `control/brain_server.py:544` defaults `ROUTE_PREFIX` to empty. With the setup CLAUDE.md gives (`:2059`, a plain `uvicorn … --port 8001`), the bridge polls a path that 404s and `/diagnostics` reads STALE forever. It works only under `service/tunnel/run.sh:85` (`ROUTE_PREFIX=/brain`). |
| R5 | `PLAN-ros-alignment.md:113` and `:115` (`GET /world/scan`, a `sim_scan_node`), `:935`, `:120`. | The route is `GET /scan` (`robot/server.py:583`). No `sim_scan_node` exists: the republisher is folded into `picar_bridge` (`bridge.py:39-42`). `ros2 node list` will not show what the table promises. |
| R6 | `PLAN-ros-alignment.md:837`: `service/slam/picar_description/urdf/picar.urdf.xacro`. | The real path is `service/slam/src/picar_description/urdf/…`. |
| R7 | `PLAN-ros-alignment.md:117`: R6 uses "a DWB/MPPI controller". | `nav2.yaml:39` uses `RegulatedPurePursuitController`. |
| R8 | `PLAN-ros-alignment.md:1042`: SLAM is anchored "at first contact". | `:1118-1123` and `world/ros_world.py:16-23` anchor at session **start**. The design paragraph was never amended. |
| R9 | `picar_bringup/package.xml:19` declares `nav2_bringup`. | That package is never launched. The launch file starts `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_lifecycle_manager` and `foxglove_bridge` (`picar.launch.py:59-82`), and none of them is declared. `picar_sim_hardware/package.xml` omits the curl and nlohmann_json dependencies that its `CMakeLists.txt:8-9` requires. Only the Dockerfile's apt line satisfies them, so `rosdep install` on a fresh Jetson would not. |
| R10 | `nav2.yaml:4-7` and `:163`, `slam.yaml:3-5`: settings are justified by "the starter house's 0.30 m doors". | R6 concluded the starter house **cannot** host nav2 and moved to `scaled_house` (`PLAN-ros-alignment.md:1202-1208`). The `inflation_radius: 0.12` rationale refers to a house the config is no longer judged in. |
| R11 | `tests/demo_nav_goals.py:71`, `:111`: defaults to `SIM_MAP` or `starter_house`, read from the **local** env. | The robot server's house is set by the **server's** env. If the two differ, the demo scores against the wrong ground truth. The default is also the house that fails 0/6–5/6. CLAUDE.md's "live suites ask the server" (`/health` `sim_map`) covers the tests, not this demo. |
| R12 | `PLAN-mapping.md:30-33` "Never (c)'s full adoption"; `:103`, `:296`, `:312` "Store is DynamoDB"; `:134`, `:267` continuous pose is a missing prerequisite; `:277` "no drift". | `PLAN-ros-alignment.md:20` says "adopted properly, not minimally". `:1634-1641` records S3 as decided by the user. R0 built continuous pose, and R5 built drift (`robot/factory.py:85-88`). PLAN-mapping never mentions PLAN-ros-alignment. |

### 3.3 Hardware and BOM

| # | Doc claim | Code reality |
|---|---|---|
| H1 | `HARDWARE-BOM.md:283` and `:398`: 1001 feedback "field layout not read". `:285`: heartbeat stop is `[U]`. | PLAN-ros-alignment 3.16 read it from firmware source: `T,L,R,r,p,y,temp,v[,pan,tilt]`. L and R are **speeds, not encoder counts**, and the heartbeat is verified. The host sets 1500 ms (`hardware_robot.py:57`) against the server's 1.0 s watchdog (`config/robot.yaml:94`). No hardware doc records either fact. |
| H2 | `HARDWARE-BOM.md:341` bring-up step 4: "Set wheel diameter, counts per revolution and track width". | This is a **firmware edit** to mainType 3 (3.16), not a runtime command. `HardwareRobot.__init__` never sends `T=900` (`hardware_robot.py:81-83`). The fake defaults to mainType 3 (`fake_esp32.py:54`), so stock mode is never exercised. |
| H3 | `HARDWARE-BOM.md:243` and `:391`, `JETSON-BOM.md:156`, `BOM-COMPARISON.md:167`: JetPack 6 vs 7 / Humble vs Jazzy is "undecided". | `service/slam/Dockerfile:11` is `FROM ros:humble-ros-base`, and `CLAUDE.md:161` says JetPack 6 / 22.04. The code has decided. |
| H4 | The xacro (`picar.urdf.xacro:22-23`) tags the deck 228×148 as `[BOM]`, and its header says `[BOM]` values come from HARDWARE-BOM or JETSON-BOM. | `HARDWARE-BOM.md:294`: deck dimensions are "unpublished: measure". The number actually comes from a kit survey (`PLAN-onboard-perception.md:924`). |
| H5 | `JETSON-BOM.md:38`: "Buy regardless — $464.94". `:58`: "~$290". `:68`: battery $74.97. | The rows `:44-49` sum to **$480.94**, and `:65-77` sum to **~$362**. `HARDWARE-BOM.md:196` prices the battery line at **$50.47**: the 2-pack is counted twice and the buzzer dropped. The kit threshold is built on these wrong numbers. |
| H6 | `BOM-COMPARISON.md:29` "honest range $59-110" vs `:196` "$59-86". `:147-152`: OWLv2 "has never been run as a crop source". | These contradict each other. P19 ran OWLv2 as a crop source on 2026-09-17, as `JETSON-BOM.md:4-6` itself cites. |
| H7 | HARDWARE-READINESS.md passim: Pi 5, Hailo-8L, Camera Module 3, "STM32 driver board", two SG90s. `:26`, `:247`, `:277`: `hardware_robot.py` is "the one file still unwritten". | The actual build is a Jetson, an IMX219, a Waveshare ESP32 and one ST3215 pan servo (`HARDWARE-BOM.md:114-119`, xacro `pan_joint` only). `robot/hardware_robot.py` exists (R7). Eight file:line references are also wrong, e.g. `sendAction` is at `web-twin/app.js:475`, not `index.html:1495`. |
| H8 | `HARDWARE-READINESS.md:243-245`: a pivot runs "until the encoders report 90°". | `hardware_robot.py:195-200` is **open-loop timed** at 1.2 rad/s. Pivots close on the encoders only under `drive: ros`. |
| H9 | No doc names `mode: hardware`, `ROBOT_SERIAL`, `hardware.serial_port`, `SIM_MOTOR_BOARD`, a udev rule, `dialout` or `safety.sensor_to_bumper_cm`. | `robot/factory.py:99-124` requires them. `os.open(port, O_RDWR)` at `hardware_robot.py:62` fails with EACCES if the user is not in `dialout`. |

### 3.4 Entry points, harness, twin and perception

| # | Doc claim | Code reality |
|---|---|---|
| E1 | `CLAUDE.md:1585-1590` ("Then buy"): a Hailo-8L M.2 build for ~$555-620. Also `:147`, `:416-420` ("DECISION 2026-09-13 governs") and `:973` ("decision is NOT made"). | `CLAUDE.md:226` and `:280`: the decision **closed on 2026-09-19** for a Jetson Orin Nano Super at ~$944. Four places in the same file contradict the paragraph that governs. |
| E2 | `CLAUDE.md:664`: `goal_pose.py` "nothing consumes it yet". | `brain/tiered.py:80` imports it and `:605` uses it. |
| E3 | `CLAUDE.md:139`: brain on ECS "Done and deployed". `:169` and `:1779`: twin on ECS behind a shared NLB/ALB. `:1011`: VPC teardown "SPECIFIED, NOT BUILT". | `PLAN-aws-cost-redesign.md:8` and `:262-264`: nine stacks deleted 2026-09-05, with 0 load balancers and 0 ECS clusters. `CLAUDE.md:1798` itself says the twin is served from S3 + CloudFront. |
| E4 | Test counts: `CLAUDE.md:21`/`:828` 1440; `:875` 54; `:831`/`:1448`/`:1452` contract suite 75/36/44. `README.md:63` 471. | `pytest --collect-only` gives **1449** (1439 committed), **58** for vision_analyze, and **134** for `tests/test_robot_contract.py` over **6** backends (`:85`). |
| E5 | `CLAUDE.md:148` and `PLAN-onboard-perception.md:3836-3837`: `perception_match_margin` "now 0.02 in config". `:3873`: `auto` crop path "still the default". | `config/robot.yaml:156` sets `perception_match_margin: 0.0`, and the gate is now `perception_match_probability` → `DEFAULT_MATCH_PROBABILITY = 0.8` (`brain/perceive.py:256`). `DEFAULT_CROP_PATH = CROP_LOW_CONFIDENCE` (`perceive.py:163`). `config/robot.yaml:128` still names `yolo11s.pt` as the default, while `perceive.py:197` uses `yoloe-11s-seg.pt`. |
| E6 | `PLAN-onboard-perception.md:3`: "design settled, nothing built". `:65`: "Pi-plus-Hailo decision stands". `:8077`: "DECISION 2026-09-13: no Jetson … final". | `:7242` records the Hailo path closed and the Jetson chosen (2026-09-19), but nothing in lines 1–110 points there. P1–P3, `goal_pose` and `arrival` are all built. |
| E7 | `AGENT-HARNESS.md:148`: the step budget is checked second. §8 `:414` lists the outcomes without `blocked`. | `control/mission_runner.py:417-421` removed that check, and the budget is now checked last (`:528`). `BLOCKED` exists (`:122`) and ends missions after `stuck_after` (`:522-527`). |
| E8 | `FEATURES.md:117-119` and `:939-944`: `/navigate` runs on Nova Lite. `PLAN-ar-guidance.md:556`: `/guidance` runs on Sonnet 4.5. | `vision_core.py:74-76`: `/navigate` defaults to Opus 4.5 and `/guidance` to Nova Lite. Both docs are wrong, in opposite directions. |
| E9 | `FEATURES.md:113` and `web-twin/README.md:42`: 960 px capture via `GUIDANCE_MAX_CAPTURE_DIM`. `PLAN-ar-guidance.md:177`: `GUIDANCE_FOUND_STREAK_TO_PAUSE` is 2. | `web-twin/app.js:2809` `CAPTURE_MAX_DIM = 1280`. `app.js:3672` sets the streak to **1**, the very value the doc argues against. |
| E10 | `web-twin/README.md:3-5`: "No server". `:148`: `LAYOUT`/`OBJECTS` constants. Explore, Find backpack, Vision Autopilot, the Camera tab, the local brain, two ngrok tunnels. | 0 grep hits in `index.html` or `app.js` for any of them. The page is served by `robot/server.py` or CloudFront. The map comes from `GET /world/map`. The tunnel is `service/tunnel/run.sh`. |
| E11 | `PLAN-teleop-robot.md:578-583`: the `/frame` 503 fix is "not yet done". `:3-11`: stacks are live. | The fix is in `robot/server.py:751-767`. The stacks were deleted 2026-09-05. |
| E12 | `PLAN-sim-hardening.md:3-4`: "proposal, nothing built yet, 61 passed". `:341`: 36 tests over 4 backends. `:667`: continuous pose is "live again as C2". | S1–S5 are built; the suite has 134 tests over 6 backends; continuous pose was built as R0 on 2026-09-25. |
| E13 | `EDGE-PERCEPTION-BENCH.md:417`: "INT8 still has not been measured", projecting 124 ms. `:453`: Jetson costs $320–430 more. | `evaluations/trt/README.md:31-35` (same date): INT8 takes OWLv2 from 82% to 7%, and the real figure is ~205 ms. The verified cost delta is $59–86 (BOM-COMPARISON). |
| E14 | `tools/hailo/README.md:50-54`: DFC needs Python 3.8–3.10. | `tools/hailo/setup_host.sh:22`: "the real window for 3.34.0 is 3.10-3.12". |
| E15 | PLAN-onboard-perception P23/P24 and CLAUDE.md: "11 walks / 1234 frames". | There are now **21 labelled walks / 2261 frames**, 6 of them machine-labelled. `perception_eval score` scores every walk that has `labels.json`, so a re-run today silently scores a different corpus. Only P19's subset is pinned (`evaluations/tier-decomp/p19_frames.json`). |

Several things were checked and found **correct**, so nobody needs to re-check them:
- **Chassis constants.** Wheel radius and separation agree across the xacro, `controllers.yaml`, `sim/mock_robot.py` and `hardware_robot.py`.
- **ROS rates and limits.** twist_mux priorities and timeouts, the 0.25 s `cmd_vel_timeout`, 20 Hz everywhere and the 12 m lidar range all match.
- **nav2 footprint.** It equals the xacro dimensions.
- **Perception defaults.** `yoloe-11s-seg`, 16 crops, P≥0.8, no floor mask.
- **Env vars.** Every env var the ROS plan names is read where the docs say it is.
- **Commands and tests.** Every `python -m tests.*` command and every named test file exists and runs.
- **Failsafe config.** The failsafe values in `config/robot.yaml` match `control/brain_config.py`.

One exception: `brain_config.py:156` still defaults `min_distance_cm` to 30.0, the value CLAUDE.md calls a coin flip.

---

## 4. Empirical spot check: two how-tos, walked as a new reader

### 4.1 CLAUDE.md §1 "Setup", from a clean checkout

| Step | Where a reader stalls or misreads |
|---|---|
| `python -m venv .venv` | No Python version is given. `.venv` is 3.13.9, and there is no `.python-version` or `pyproject.toml`. `requirements.txt` is fully unpinned. §6 (`~:575`) admits two FastAPI versions (0.141 and 0.136) disagree about the suite, yet setup pins nothing. A reader on 3.10 or 3.11 gets whatever pip resolves. |
| `pip install -r requirements.txt` | This works. `requirements-perception.txt`, which `policy: "tiered"` needs, is not mentioned here; it turns up 1,800 lines later. README.md's own setup (`:54`) skips the venv and Playwright entirely, so the two entry points disagree. |
| `pytest tests/ -v`, "1389 passed, 51 skipped" | 1449 are collected in the working tree. A reader who sees a different number can't tell drift from breakage. |
| Then what? | Setup stops at the tests. Getting a robot running requires reaching §7 (`:2057`, `uvicorn robot.server:app --port 8000` + `uvicorn control.brain_server:app --port 8001`). The ROS stack, which R4–R6 are built on, is not mentioned in Setup at all: no Docker, no `ROBOT_DRIVE`, no `WORLD_MODE`, no `SIM_MAP`. |

**Verdict: a new reader can run the unit tests, and not the robot.**

### 4.2 Bringing up the ROS stack (`PLAN-ros-alignment.md` 3.13, `:1022-1027`, the only run instructions in the repo)

| Step | Where a reader stalls or misreads |
|---|---|
| Finding the instructions | They are one paragraph at line 1022 of a 1,644-line build log, headed "How to run it". Nothing points there: not CLAUDE.md, not README, and `service/slam/` has no README. |
| `docker build -t vision-picar-ros service/slam` | This works. It builds tf2 and slam_toolbox from pinned SHAs (Dockerfile `:34-60`), and the doc does not warn that this takes a long time. |
| `docker run … -e APP_SHARED_SECRET …` | If the variable isn't exported, Docker silently forwards an empty value. The robot server then answers 401. The only symptom is `picar_sim_hardware` logging "POST /wheels … failed (N in a row) -- still trying" (`picar_sim_hardware.cpp:135`), and no doc lists that failure signature. Where the secret comes from (`~/.vision-picar-local-secrets`, CLAUDE.md `:1837`) is in a different doc. |
| `ROBOT_URL=http://host.docker.internal:8000` | This works on macOS. On the Jetson (Linux) it fails: see R3. |
| (no `-p 8765`) | 3.17 says Foxglove is on 127.0.0.1:8765, but it is unreachable (R2). |
| "restart the robot server with `ROBOT_DRIVE=ros`" | The doc doesn't say how. The mechanism, `ROBOT_DRIVE=ros bash service/tunnel/restart.sh`, is named only in `tests/test_ros_chain_live.py:10`. The required start order (robot server before container) and the IPv6-first `host.docker.internal` trap appear only in the private memory file `ros_container_how_to_run.md`, outside the repo. |
| Is it working? | The doc gives no health check. There is no `curl :8090/health`, no `ros2 control list_controllers`, no `ros2 topic hz /scan`, no `tf2_echo map base_footprint`. |
| Reproducing R5/R6 | This needs `WORLD_MODE=ros`, `SIM_MAP=scaled_house` on **both** the server and `tests/demo_nav_goals.py` (R11), `SIM_ODOM_DRIFT=1.0,1.03`, and "a fresh container" (the doc gives no `docker rm -f` step). These are scattered across 3.14, 3.15, CLAUDE.md and the demo's docstring. |
| The brain view | With CLAUDE.md's own `uvicorn control.brain_server:app`, `/diagnostics` reads STALE forever (R4). |

**Verdict:** someone on a Mac who already knows the project can get the stack up after about four detours. Someone on the Jetson cannot get it up from the docs.

---

## 5. Per-doc findings (specific and fixable)

Each fix below points back to a §3 row where one exists. Findings already covered there are not repeated.

**CLAUDE.md**
- **"Then buy" (`:1585`) names the wrong part** (E1). Point it at `JETSON-BOM.md`. Mark `:147`, `:416-420` and `:973` superseded where they stand.
- **Status rows contradict each other.** `:139` (brain deployed), `:169` (twin on ECS), `:951` ("PLAN-mapping … nothing built" vs `:175` "N1 BUILT") and `:1011` all disagree with other parts of the file (E3).
- **Stale claims.** `:664` says goal_pose is unconsumed (E2); `:148` gives the margin as 0.02 (E5); `:167` says "NOT fixed" (V9).
- **The repo map is missing live code:** `PLAN-ros-alignment.md`, `brain/arrival.py`, `control/metrics_*`, `service/{admin,brain,lambda,static}` and 9 of the 12 CloudFormation templates.
- **`:439` is a garbled splice:** "Superseded … P12 and With the rig now verified exact".
- **`:2074`'s UI table says "tap Explore",** a button that no longer exists.
- **Structure.** About 800 of the 2,111 lines (`:173-578` and the Stage 0 findings at `~:1035-1435`, about a corpus that was deleted) are history interleaved with live instructions. There is no "current state in 20 lines" and no "next up". The actual next work, the 3.17 safety findings, lives only in a table cell at `:167`.

**README.md**
- **It describes deleted UI as live:** Vision Autopilot and Explore/Find (`:28`, `:345`, `:382`).
- **Its only hardware section** (`:590-601`) is the rejected Hailo build at ~$500.
- **Test counts** are 471 and 47 (`:63-64`).
- **It never mentions ROS.**
- **"Where to read next" (`:15-26`)** omits PLAN-ros-alignment, JETSON-BOM, AGENT-HARNESS and FEATURES.
- **`:603-606`** says to "add `robot/hardware_robot.py`", which already exists.

**INTRODUCTION.md**
- `:9` and `:288` describe a Raspberry Pi 5 and a "small AI chip".
- `:105` gives it a "steering servo", which contradicts its own `:288` explanation of why the PiCar-X was dropped.
- `:200-227` presents the retired "watch it on a phone" rule as the project's discipline.
- The milestone table (`:231-248`) omits R0–R7.
- `INTRODUCTION.html` is a second copy that will drift.

**web-twin/README.md.** Rewrite it: see E10. It should cover `app.js`; how to run it (two uvicorns); how to reach it from a phone (`service/tunnel/run.sh`); how to deploy (`service/static/sync.sh` plus the parity diff); and how to test it (`tests/test_ui*.py`). `:211` "Hold the phone low" contradicts CLAUDE.md's "do not hold it; wheeled rig".

**AGENT-HARNESS.md**
- **Arbitration and refusals.** V3 (rule 3), V5 (reasons) and V6 (watchdog inputs).
- **Tick order and outcomes.** E7 (step order, and `blocked`/`preempted` missing from §8).
- **It never answers "who can move the robot"** for `/wheels` and `/world/goal`.
- **Outdated seams.**
  - `:322` says "HardwareRobot does not exist".
  - §3 step 5 still describes the rule-based policy as "grid position and facing"; it now takes its pose from `WorldInterface` (`brain/agent.py:434`).
  - There is no `WorldInterface` seam in §2 or §5.
- **Small errors.**
  - `:388` has "POST /mission/status"; it is a GET.
  - `:606-611` describes the deleted Vision Autopilot.
  - Invariant 7 (`:465`) cites the retired rule.
- **Maintenance.** The §1 update checklist ("perishable", last run 2026-08-31) was not run for R2b, R4, R6, R7 or P7e.

**FEATURES.md**
- **§6 AWS topology** (`:838-875`) describes the NLB, ALB, ECS and EFS as live. All of it was deleted 2026-09-05, and the current S3, CloudFront, Lambda and tunnel path is described nowhere.
- **The deleted local brain** is still described (`:79-89`, `:784`).
- **Wrong constants.** E8 (models) and E9 (capture size).
- **Tap-to-goal (`POST /world/goal`) moves the robot and is not documented.** Neither are `/odometry` and `/depth`.
- **The §5 safety table** omits `/wheels` vetting and nav2's collision_monitor.

**PLAN-ros-alignment.md**
- **Wrong or stale claims:** R1 (robot_localization), R5–R8.
- **Header and run commands.** The header (`:3`, `:8`) is dated 2026-09-25 at `672a9bb` and omits 3.17 and 3.18. §4 `:1568` ("RViz is never needed") contradicts 3.17's Foxglove addition. The instruments `demo_slam_lap` and `demo_nav_goals` have no invocation lines.
- **Open questions.** §6 is numbered 1, 2, 3, 5, 6, 4, and Q2 and Q3 were already decided in 3.15.
- **Unresolvable references.** `:114` cites "§900", which exists nowhere.
- **Two chassis widths.** 3.17 (`:1453`) cites a 19.8 cm chassis, while the cone's 15.4° comes from `CHASSIS_WIDTH_CM = 16.5`. The plan never explains the two numbers.
- **Conventions.** The map-frame convention (`:1035`, "x-forward") never states compass zero (−y) or the flip `(x, −y, 90 − yaw)` that `world/ros_world.py:41-44` implements.

**PLAN-mapping.md.** Add a "superseded in part" banner listing what changed: "never (c)"; N6 → R5+R6; `/goto` withdrawn; C2 built; drift built; S3 vs DynamoDB; the retired DoD at `:303-305`. §1 and §4 (body vs world) are still the best conceptual writing in the ROS docs, so keep them.

**service/slam/ (no README)**
- R9 (package.xml dependencies) and R10 (house rationale in the YAML).
- The bridge docstring omits:
  - `BRIDGE_PORT` (read at `bridge.py:564`);
  - the three accepted drivers;
  - the `/brain` prefix assumption.
- Dead packages: the Dockerfile installs `python3-requests` (unused; the bridge uses urllib) and `joint-state-publisher` (never launched).

**HARDWARE-BOM.md**
- **Append dated corrections** for V1, H1 and H2.
- **Decisions left open that are already made.** Record JetPack 6.2.1 / Humble as decided (H3); B8's USB installer is then likely unneeded. Correction 1 (`:26-43`) still says "treat as [R], run P10 first". Correction 4 (`:60-68`) still says the board exists "to run OWLv2".
- **Missing items.** There is no hardware e-stop or motor-only kill, and the rocker switch (`:197`) is not DC-rated. `:360` calls the 10.2 V buzzer "the independent backstop", but it trips before the 10.0 V software stop, so it is the first alarm. The software voltage cutoff it specifies is implemented nowhere: `HardwareRobot._on_base_feedback` ignores `v`.

**JETSON-BOM.md**
- H5 (arithmetic).
- `:23` still calls the Pi build "the fallback".
- `:156-161` still says the board's "entire value rests on OWLv2 … ~5 Hz".
- `:75` prices a fuse that the other BOMs say is unpriced.

**BOM-COMPARISON.md**
- It needs one line saying the decision closed 2026-09-19.
- Fix H6.
- `:212` asks for the P10 run, which is moot.
- `:48` and `:50` tag items `[V]` that `:229` calls `[I]`.

**BOM.md.** The banner (`:3`) says only "do not price". The body's power and servo design is also reversed: a 12 V buck-boost where the pack now feeds the Jetson directly; SG90 pan/tilt where there is now one ST3215 pan; "NVMe, not microSD" where microSD is now required. Widen the banner.

**HARDWARE-READINESS.md**
- **Stale or wrong:** H7, H8, V2 and V7.
- **Missing pre-flight items** the code now requires:
  - flash mainType 3 with this chassis's constants;
  - `ROBOT_SERIAL`, a udev rule and `dialout` access;
  - verify the 1500 ms heartbeat;
  - measure `safety.sensor_to_bumper_cm` (0.0 today);
  - measure the xacro's `[PLACEHOLDER]` block (`laser_z`, `pan_x`, `pan_z`, `camera_up`, `camera_pitch`, `wheel_separation`);
  - the battery voltage cutoff.

**PLAN-brain-relocation.md**
- B5 (`:270-285`) plans two systemd units on a **Pi**. It needs three on a Jetson: the ROS container, robot and brain. It should specify `After=`/`Requires=`, an `EnvironmentFile` for secrets and modes, serial device passthrough and udev, and what a mid-mission restart does to the ESP32 heartbeat and the mission.
- `:177` refers to `px.stop()`, a PiCar-X call.
- The Interim ECS section (`:290`) is not marked deleted.
- `:215-223` and `:255-258` still keep the local brain.

**PLAN-teleop-robot.md**
- E11.
- It never mentions the current run path, `ROBOT_MODE=teleop bash service/tunnel/run.sh`.
- `:266` cites `index.html:1055`, which is now CSS.

**PLAN-sim-hardening.md**
- E12.
- `:159` says "`control/` is empty". `:533-535` says "RemoteRobot … does not exist yet".
- `:291-293` still states the retired DoD.
- S5, `sim/sensors.py` and `config/robot.yaml:49-50` all model an HC-SR04, while the sensor is a lidar. `robot/interface.py`'s M2 comment names a third sensor, a VL53L5CX.

**PLAN-microduck-transplants.md**
- V3 and V5 (M4).
- `:415` presents `PATH_FRACTION` as the live rule; zones have been chosen by angle since 5.1 (`safety.py:212`). M10 in the same doc says the opposite.
- M8–M11 target a Pi, Raspberry Pi OS and `picamera2`.
- M7's line refs point at `service.yaml`, which was deleted; the parameter is now at `serverless.yaml:53` and `:186`.

**PLAN-ar-guidance.md**
- Every function it places in `index.html` lives in `app.js`.
- `:17` and `:628` refer to the removed Camera tab.
- E8 and E9.
- `:430` says vision_analyze "has no test suite"; it has 58 tests.
- `:204` says "no localStorage"; `app.js` uses it.
- It never mentions Robot view or Drive via brain, which are now the Guide tab's main modes.
- It opens with 240 lines of changelog before the spec.

**PLAN-aws-cost-redesign.md**
- **V8 is the one to fix first.**
- `:3` is dated 2026-09-04 but reports the 09-05 teardown. `:9` and `:472` say "alongside ECS".
- The $159/month figure is pre-teardown and not labelled as such.
- `:670` says 675 tests.

**PLAN-onboard-perception.md**
- E5, E6 and E15.
- There is no current-state block at the top. The agent's review draft contains a ready-made one, reproduced in §8 below.
- The chronology is out of order: the "no Jetson, final" decision and P9 (`:8027-8434`) sit **after** P25, and §4.11 comes after both.
- `### 1.11a` sits inside §4.10 (`:4393`).
- Superseded callouts are needed at `:439`, `:3009`, `:3408` and `:8077`.
- P4 (`:3497`) is marked "NOT BUILT", but it was done in P18 and is now moot.
- P7e (`:8402`) and the C2 blocker (`:8010`) are built (R0, R1, `arrival.py`), and the doc links nowhere to PLAN-ros-alignment.
- `:6137` gives `tools/hailo/label_prepass.py`; the path is `tools/label_prepass.py`.
- The worked examples (`:3693`, `:3761`) use walks deleted 2026-09-07.
- The bearing convention is never stated. In the code, `bearing_deg` is clockwise-positive and **linear** in pixel column over a 66° HFOV (`perceive.py:1002`, `:732`), which is up to ~1.5° off at a quarter-frame.

**EDGE-PERCEPTION-BENCH.md**
- E13.
- `:17` recommends OWLv2; the shipped model is YOLOE.
- `:128` says 968 frames, while `:185` says 610.
- The reproduce block (`:504-506`) compares 4-walk, 299-frame records, which is the scope error the doc itself says was corrected.
- It needs a superseded banner.

**EVAL-navigate-models-2026-09-22.md**
- There is no path to the "24 JSON files" of raw results, no tool name (was it `control/walk_replay.py`?), and no model ids.
- Fable 5.1 is pinned to us-east-1, so its 4.2 s latency plausibly includes a cross-region hop, and the doc doesn't say so.
- `vision_core.py:137-139` still labels the models "unmeasured".

**tools/hailo/README.md**
- E14.
- It needs a status line: "OWLv2 did not compile; Hailo path closed 2026-09-19".
- The file table covers 6 of 22 files.
- It does not warn that `setup_host.sh` produced P14's wrong conclusion and that `zoo_probe.sh` is the one to use.
- It does not warn that calibration with fewer than 1024 frames silently drops the DFC to optimization level 0 (P11).

**evaluations/ READMEs**
- The root README (`evaluations/README.md`) indexes none of its subdirectories.
- `tier-decomp/` (39 files, the lineage behind the shipped default) has no README.
- No record says which corpus it was scored on (299, 610 or 1234 frames).
- `gpu/README.md:20-23` says "a GPU changes nothing". `gpu-yoloe/README.md:12-18` disproves that for CNNs (251 vs 258 true positives).
- `gpu/README.md:29` still marks YOLO11s as "shipped".

**HANDOFF-2026-09-13 / -09-15.** Add superseded banners. On 09-13:
- open item 5's "RED" test now passes;
- open item 1 (arrival) is built in the sim.

On 09-15:
- the headline and open item 5 push a Hailo 10H test that CLAUDE.md says not to spend on;
- "1036 tests, 0 fail" is lower than 09-13's 1043, and also contradicts the RED test.

---

## 6. Cross-doc issues and proposed reading path

### 6.1 Contradictions

| Topic | Positions and where they appear |
|---|---|
| **Board** | Jetson: CLAUDE.md:226, JETSON-BOM, PLAN-onboard-perception:7242. Pi 5 + Hailo-8L: CLAUDE.md:147/:1585, README:598, INTRODUCTION:9, HARDWARE-READINESS, PLAN-onboard-perception:10-65, PLAN-microduck M8-M11, PLAN-brain-relocation B5, HANDOFF-09-15. "Not decided": CLAUDE.md:973, BOM-COMPARISON. "No Jetson, final": PLAN-onboard-perception:8077. The private memory file `hardware_decision_reopened.md` also says "NO JETSON", and it loads into every session. |
| **AWS topology** | Deleted 2026-09-05: PLAN-aws-cost-redesign:8. Live: CLAUDE.md:139/:169/:1779/:1921, FEATURES §6, PLAN-brain-relocation Interim, PLAN-teleop-robot T4, PLAN-ar-guidance:567. The templates and `tests/test_alb_routes.py` stay in the tree with no "not deployed" marker. |
| **Authority order** | AGENT-HARNESS:260, PLAN-microduck:517, `robot/interface.py:35` (its own comment) and CLAUDE.md §6 each state a different order, and all of them disagree with `interface.py:42-58` + `server.py:305`. |
| **Local brain / Vision Autopilot** | Deleted 2026-09-25, yet described as live in README, web-twin/README, FEATURES:79, AGENT-HARNESS:452/:606 and PLAN-brain-relocation:215. The dead rank `twin-local-brain` (`interface.py:58`) and `DRIVER_LOCAL_BRAIN` (`app.js:469`) keep the confusion alive in code. |
| **Definition of done** | Data-driven: CLAUDE.md §7. "Watched on a phone": INTRODUCTION:200, PLAN-sim-hardening:291, PLAN-mapping:303, PLAN-teleop-robot:19, AGENT-HARNESS invariant 7, EDGE:117, and PLAN-ros-alignment's title and "Proof" column. |
| **Chassis width** | 14.8 cm (PLAN-microduck:417, HARDWARE-READINESS). 16.5 cm (`safety.py:146`). 19.8 cm (nav2.yaml, xacro, `safety.py:84`, PLAN-ros-alignment 3.17). |
| **Obstacle sensor** | HC-SR04 (PLAN-sim-hardening S5, `sim/sensors.py`, `config/robot.yaml:49`). VL53L5CX (`robot/interface.py` M2 comment). RPLidar C1 (everything current). |
| **Map storage** | DynamoDB (PLAN-mapping). S3, decided by the user (PLAN-ros-alignment §6 Q6). |
| **`/navigate` and `/guidance` models** | FEATURES says Nova Lite for both. PLAN-ar-guidance says Sonnet 4.5. The code says Opus 4.5 for `/navigate` and Nova Lite for `/guidance`. |
| **Test counts** | Seven different figures across CLAUDE.md, README, PLAN-sim-hardening, PLAN-microduck, PLAN-aws, PLAN-ar-guidance and the two HANDOFFs. |

### 6.2 Duplication (where the drift comes from)

- **R-phase results.** CLAUDE.md §3's R-phase rows restate PLAN-ros-alignment almost verbatim; the `:167` cell alone is about 350 words. The 3.18 status has already diverged between the two.
- **P-series.** CLAUDE.md `:173-578` duplicates PLAN-onboard-perception's P-series and the HANDOFFs.
- **Drill and failsafe tables.** These appear in AGENT-HARNESS §6, FEATURES §5, web-twin/README and PLAN-brain-relocation B3. They agree today; make AGENT-HARNESS canonical and link to it from the others.
- **The 3S-BMS-below-9V argument and the OWLv2 rationale** appear in all three current BOMs, and the OWLv2 half is stale in every copy.
- **EDGE-PERCEPTION-BENCH §4-§5** copies tables from `evaluations/gpu` and `evaluations/hailo`.
- **Operational ROS knowledge exists only in private memory:** the complete run command, the start order, the IPv6 trap, "don't export APP_SHARED_SECRET into pytest", and "nav2 is judged only on scaled_house". No collaborator and no other machine can see it.

### 6.3 Coverage gaps

These are subsystems that exist in code and have no doc of their own. They are mentioned only in CLAUDE.md's repo map and in plan logs.

- **Tracked:**
  - `service/slam/` (four packages), `robot/ros_drive.py`, `world/ros_world.py`;
  - `robot/hardware_robot.py` + `sim/fake_esp32.py`, i.e. the serial protocol as implemented;
  - `sim/maps/*` (which house to use for what);
  - `service/tunnel/`, `service/lambda/`, `service/static/`;
  - `control/metrics_*`, `control/label_assist.py`, `control/target_probe.py`;
  - `evaluations/tier-decomp/`.
- **Untracked:** `tests/test_footprint_safety.py`, `tests/footprint_sweep.py` (referenced only in PLAN 3.18).

Docs that describe code which no longer exists:
- **web-twin/README.md**, almost entirely;
- **FEATURES.md §6**;
- **PLAN-ar-guidance.md's** Camera-tab, index.html and canvas-overlay spec;
- **every reference to `renderFPV`, Explore and the local brain**;
- **`mission_ended`.**

### 6.4 Proposed reading path

There is no clean entry point today. README is stale and CLAUDE.md is 2,111 lines, a third of it history. Proposed path:

1. **`README.md`**, rewritten to about 80 lines: what the robot is (Jetson, diff drive, RPLidar C1, ROS 2 Humble in one container, brain as a separate service); a three-sentence status; and this reading list.
2. **`docs/ARCHITECTURE.md`** (new). The system picture in one place:
   - the process diagram (twin → robot server ↔ brain; robot server ↔ `picar_bridge` → twist_mux → collision_monitor → diff_drive_controller → `picar_sim_hardware` / ESP32);
   - the two walls (Robot vs World, rclpy containment);
   - the TF tree;
   - the frame and bearing conventions;
   - the driver/authority table.
3. **`docs/SETUP.md`** (new). A fresh machine, from a pinned Python version, to (a) the unit tests, (b) the sim with the twin, (c) the ROS stack, with Mac and Linux/Jetson variants.
4. **Subsystem docs:**
   - `AGENT-HARNESS.md` for `control/`;
   - `service/slam/README.md` (new) for ROS;
   - `HARDWARE-BOM.md` for parts and wiring;
   - `FEATURES.md` for the UI.
5. **`docs/OPERATIONS.md`** (new). Safety model, health checks and failure signatures.
6. **Plans, logs and history.** The `PLAN-*` docs, the `HANDOFF-*`s and `evaluations/`, each with a status block at the top, and superseded ones moved under `docs/archive/`.
7. **`CLAUDE.md`**, cut to about 300 lines: setup pointer, the walls, a one-line-per-phase status table with links, §6 "things to know", §7 definition of done, and a "Next up" block.

---

## 7. Prioritized fix list

Ordered by impact on someone trying to build, run or debug the robot. Effort: **S** is under 30 minutes, **M** is an hour or two, **L** is half a day or more.

| # | Fix | Effort |
|---|---|---|
| 1 | **HARDWARE-BOM.md: append a dated correction for `T=1`** (V1: PWM in stock mode, full power at the doc's own example), the 1001 layout, the verified heartbeat, and mainType 3 needing a firmware edit (H1, H2). Someone on hardware day will send that example first. | S |
| 2 | **Chassis width: correct "errs wide" to "errs narrow"** in HARDWARE-READINESS 5.5 and the `safety.py:141-145` comment (V2). Link it to the 3.17 oblique-approach finding, and reconcile 14.8, 16.5 and 19.8 cm to one source (the xacro). | S |
| 3 | **PLAN-aws-cost-redesign: add `VisionSharedSecret=… WalksSharedSecret=…` to the deploy command** (V8). | S |
| 4 | **Write `service/slam/README.md`**:<br>• the package map;<br>• build;<br>• the full `docker run` (secret sourcing, `-p 127.0.0.1:8765:8765`, `--restart`, a Linux/Jetson variant with `--network host` or `host-gateway`, and the robot server on 0.0.0.0);<br>• start order;<br>• `ROBOT_DRIVE`/`WORLD_MODE`/`SIM_MAP`/`SIM_ODOM_DRIFT` via `service/tunnel/restart.sh`;<br>• the `BRAIN_URL`/`ROUTE_PREFIX` caveat (R4);<br>• the topic/frame graph;<br>• a six-line health check (`curl :8090/health`, `ros2 control list_controllers`, `ros2 topic hz /scan`, `ros2 run tf2_ros tf2_echo map base_footprint`, `view_frames`, Foxglove);<br>• failure signatures ("still trying" = secret or URL wrong; "Transform data too old"; goals "reached" in 0.08 s = TF freeze);<br>• one invocation line per instrument.<br>Move the content of the private memory file here. | M |
| 5 | **Document who can move the robot.** Rewrite AGENT-HARNESS §4.1/§4.2 against `interface.py:42-58` and `server.py:286-312`: the exclusive autonomous rank, all six refusal reasons, `/wheels` feeding the watchdog, and `/world/goal` bypassing arbitration with nav2 at priority 50 alongside the brain. Either implement `mission_ended` or delete that row (V3–V6, V10). Decide the nav2-vs-brain question in code if it is a bug. | M |
| 6 | **One board, everywhere.** Put a one-line "SUPERSEDED 2026-09-19: board is Jetson Orin Nano Super" callout on:<br>• CLAUDE.md `:147`, `:416`, `:973`, `:1585` (rewrite "Then buy" to point at JETSON-BOM);<br>• PLAN-onboard-perception `:3`, `:65`, `:439`, `:3009`, `:3408`, `:8077`;<br>• HARDWARE-READINESS, INTRODUCTION, README `:590`;<br>• PLAN-microduck M8–M11, PLAN-brain-relocation, EDGE-PERCEPTION-BENCH, BOM-COMPARISON, HANDOFF-09-15, tools/hailo and evaluations/hailo.<br>Also correct the memory file. | M |
| 7 | **Settle the 3.18 status in one place** (V9). Measure it, then update the plan heading and body and CLAUDE.md `:167` together. | S |
| 8 | **Write `docs/SETUP.md`**: a pinned Python version (add `.python-version`), pinned or constrained requirements, the perception extras, Playwright, the two uvicorns, the ROS stack (from #4), and hardware-mode prerequisites (`ROBOT_SERIAL`, udev rule text, `dialout`). | M |
| 9 | **Fix ROS-plan drift**: R1 (drop robot_localization or mark it "planned with the IMU"), R5–R8, R10 (YAML comments), and the header status. | S |
| 10 | **Fix the package manifests** to match launch and CMake (R9), so `rosdep install` works on a fresh Jetson. | S |
| 11 | **Record the decided JetPack 6.2.1 / Ubuntu 22.04 / Humble** in the three BOMs (H3). | S |
| 12 | **HARDWARE-READINESS: new banner, and rewrite the parts table and §4** (what `hardware_robot.py` *does*: open-loop timed verbs, encoders only under ROS). Add the missing pre-flight items (§5 above), and fix the eight file:line references. | M |
| 13 | **Replace the AWS topology** in FEATURES §6 and CLAUDE.md `:169`, `:1011`, `:1779`, `:1921` with the current S3 + CloudFront + Lambda + tunnel path. Mark the deleted-stack templates and `tests/test_alb_routes.py` as historical, or delete them. | M |
| 14 | **Rewrite `web-twin/README.md`** at a third of its length (E10). | M |
| 15 | **Add current-state blocks** to PLAN-onboard-perception (the draft is in §8), PLAN-sim-hardening, PLAN-mapping, PLAN-teleop-robot, PLAN-ar-guidance and both HANDOFFs. | M |
| 16 | **Fix perception config claims** (E5): margin 0.0 with the probability gate, the `low_confidence` default, and the `robot.yaml:128` detector comment. **Pin the 1234-frame corpus** as a frame-list file next to `p19_frames.json` (E15). | S |
| 17 | **State the conventions once** (in ARCHITECTURE.md, linked from the perception and ROS plans): `bearing_deg` clockwise-positive, linear over a 66° HFOV; house frame x-east / y-south with compass zero along −y; the flip to REP 103 happens only in `bridge.py` and `world/ros_world.py`. | S |
| 18 | **Fix JETSON-BOM arithmetic** (H5) and BOM-COMPARISON's range (H6). Widen BOM.md's banner. | S |
| 19 | **AGENT-HARNESS**: tick order, `blocked`, the WorldInterface seam, and the GET typo (E7). Run its §1 checklist for R2b, R4, R6, R7 and P7e. | M |
| 20 | **Cut CLAUDE.md to about 300 lines.** Move `:173-578` and the Stage 0 findings to `docs/archive/HISTORY.md`, fix the repo map, and add "Next up". Do this last, once the facts above are corrected, so the move doesn't carry errors with it. | L |
| 21 | **Split PLAN-onboard-perception.md.** Keep §0–§2 as the plan, move P1–P25 to `evaluations/LOG.md`, and move the Hailo material to an archive doc with one "closed" banner. | L |
| 22 | **Update test counts,** or stop quoting them in prose. Say "run `pytest --collect-only`" instead. | S |

---

## 8. Missing docs to write

| Doc | One-line outline |
|---|---|
| `service/slam/README.md` | Four packages; build and run on Mac and on Jetson; env vars; topic/frame graph; health check; failure signatures. (Fix #4.) |
| `docs/ARCHITECTURE.md` | Process and data-flow diagram (sensors → odometry → SLAM → nav2 → twist_mux → collision_monitor → ros2_control → robot server → ESP32); TF tree from the xacro; the two walls; frame and bearing conventions; the driver table. |
| `docs/SETUP.md` | A fresh machine from a pinned Python to the tests, the twin, the ROS stack and hardware mode (serial, udev, `dialout`). |
| `docs/OPERATIONS.md` (safety and runbook) | What stops the robot and in what order (D-pad stop, watchdog 1.0 s, cmd_vel_timeout + mux 0.5 s, ESP32 heartbeat 1.5 s, B3.2/B3.3); velocity caps (0.6 m/s, 6 rad/s); what happens on lost Wi-Fi, a crashed container, a crashed brain or a restarted robot server; the battery cutoff (specified, not implemented); the open safety findings. |
| `docs/HARDWARE-BRINGUP.md` | Ordered hardware-day checklist: flash mainType 3, set `ROBOT_SERIAL` and udev, verify the heartbeat, measure every `[PLACEHOLDER]` in the xacro and `sensor_to_bumper_cm`, calibrate the track width against the encoders, and first power-on with the wheels off the ground. |
| `docs/DEPLOY.md` | Current topology only (S3 static + CloudFront, the serverless Lambdas, `sync.sh` and parity diff, the tunnel for robot and brain) and which templates are live vs historical. |
| `sim/maps/README.md` | The three houses, what each is for (starter = unit tests, scaled = nav2, home = tour), and which one each demo and live suite must be run in. |
| `evaluations/tier-decomp/README.md` | What each record is, which corpus it scored (pin the frame lists), and the INVALID rows. |
| `DECISIONS.md` (index) | One line per settled decision (board, ROS scope, detector, map storage, definition of done), with date and governing section, so reversals have one place to land. |

**Draft status block for the top of PLAN-onboard-perception.md.** Every fact in it was verified in this review.

> **State as of 2026-09-27.** Board: Jetson Orin Nano Super (decided 2026-09-19; every Hailo section below is history). Shipped on-board tier (`brain/perceive.py`): `yoloe-11s-seg.pt` → CLIP RN50, `low_confidence` crops, 16 crops/frame, gate P≥0.8, no floor mask. On the 1234-frame, 11-walk corpus (P23, `evaluations/tier-decomp/F_yoloe-11s.json`) it reads 80% recall with 2 FP at the gate and 82% at 3 FP, at a mean of 139 ms on laptop CPU. The corpus is now 21 labelled walks (6 machine-labelled 2026-09-21); pin the frame list before re-scoring. Built: P1–P3, `goal_pose.py` (off by default), arrival on the lidar (`brain/arrival.py`, sim only). Open: `brain/planner.py`, 1.11a enforcement, P7b preprocessing latency on the Orin. Convention: `bearing_deg` positive = right (clockwise), the opposite of REP 103.
