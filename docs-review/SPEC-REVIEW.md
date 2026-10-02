# Spec review -- all 15 domains

Reviewed 2026-10-02 against the working tree on `dev` (`b35d37d` plus the
uncommitted specs), following `docs/SPEC_REVIEW_PROMPT.md`.

**Method.** Five reviewers read three domains each. None of them wrote the
specs it reviewed. The coordinator did the coverage and duplication passes
and re-checked the two most serious findings itself. Everything was
read-only: `grep`, reading files, `pytest --collect-only`, dry imports and
dry `MockRobot` calls, plus one offline test run. Nothing touched a server,
Docker, the network, AWS or a paid API. Findings established only by
reading are marked **UNCONFIRMED**. About 690 claims were checked across
the 15 engineering specs, plus every contract row and failure-mode row in
the 15 architecture specs.

## 1. Summary

The specs are accurate where it matters most:

- Every parameter table, route list, config key and test count spot-checked
  in the safety, ros, world, motor-board and twin engineering specs held.
- Fewer than one claim in ten failed, and most failures are small (a
  line-number count, a code default confused with a yaml value).

Several architecture specs promise more than the code delivers, and those
are the defects that matter. Writing the specs surfaced real behaviour that
no document had recorded, and three findings are **code defects on the
hardware path**:

- a stop under `drive: ros` waits on the ROS container;
- reversing and turning run unguarded on a body with no lidar scan;
- the Jetson's G4 gate would pass on skipped tests.

**The single most important defect:** body and control-api ARCHITECTURE
promise that "a stop never depends on the container". In fact
`RosDriveRobot.stop()` posts to the bridge three times, with a 2 s timeout
each, *before* it stops the robot directly. The watchdog calls that stop
on the server's event loop. A bridge that accepts connections but hangs
therefore delays the stop by up to ~6 s and freezes every route while it
waits (`robot/ros_drive.py:233-243`, `:91`; `robot/server.py:302`;
re-checked by the coordinator).

## 2. Linter results

`python tools/spec_lint.py` reports 0 errors and 0 warnings over all 31
specs, so there are no warnings to rule on. During authoring the linter
gave two false positives, both fixed in the linter with a test each:

- an `s3://` URI read as a missing repo path;
- a bare `exports/` S3 prefix read as a missing repo directory.

What the linter cannot see, and where most findings below sit:

- tuning numbers in prose;
- mechanisms described in a failure table's Response column;
- file paths used as pointers inside architecture tables;
- claims that are false.

## 3. HOW in architecture specs

These are architecture specs that commit to one way of building something.
None is severe; each is a move to the engineering spec, which in most
cases already holds the fact.

**Patterns seen in several domains:**

- **Mechanisms in the Response column of failure tables.** For example:
  "per sub-step", "compared byte for byte", "skips a tick", "on a
  background thread". The target in each row is fine. The mechanism
  belongs in the engineering spec.
- **Dated status and commercial terms.** Platform ARCH:20-25 has delivery
  windows. Platform ARCH:116-117, :202 and :232 have the Amazon return
  window and buying from Amazon rather than direct. These will be wrong
  within weeks.
- **Version pins presented as decisions.** Ros ARCH:248-257 (D10) says
  the tf2 and slam_toolbox source builds "must be revisited when apt
  catches up". That makes them tuning. Keep "DDS and the distro are chosen,
  with reasons" and move the pins.
- **The sensor named where the principle is sensor-neutral.** Safety
  ARCH:44 and :109-111 say "the 360-degree scan in the body frame". That is
  the exact counter-example decision 0001 uses ("replace the scan-based
  corridor with a depth camera and the first document does not change").
  Rephrase as "everything beside and ahead of the chassis outline".

| Spec:line | Quoted | Why it is HOW | Move to |
|---|---|---|---|
| safety ARCH:230 | "against a 20 cm stopping floor" | 20 is `safety.min_distance_cm` | State the bar relative to the floor ("no more than 2 cm inside it") |
| safety ARCH:240 | "0 runs under 18 cm in 2021 crossings" | A result, not a target | safety ENG §Recorded numbers (already there) |
| safety ARCH:196 / control-api ARCH:152 | "judges ROS alive from the actuator's own regular wheel posts" | A mechanism with no alternative named | Add the rejected alternative, or move it |
| body ARCH:86-87 | "a test that imports the brain service in a subprocess" | How the enforcement works | body ENG §Verification |
| body ARCH:189 | "serial, newline-delimited JSON" | Framing | motor-board ENG |
| control-api ARCH:163-164, :205-209 | the identity-line fields; "at 20 Hz" | Log format; a rate | control-api ENG (already there) |
| motor-board ARCH:55, :72 | "~20 Hz feedback frames" | Set by `T:142`; the firmware default is 50 ms | "a periodic feedback stream" |
| motor-board ARCH:184-185 | "with the stock image dumped first" | A procedure step | motor-board ENG:163 (already there) |
| ros ARCH:168-171 | "projects the footprint; it does not use a stop zone" | A collision_monitor setting | The commitment is "a collision guard never blocks turning away" |
| simulator ARCH:98, :124-126, :143-144 | 0.30 m cells; the ray algorithms; "historic cell rate" | Constants and algorithms | simulator ENG (already there) |
| operations ARCH:48-50, :60, :64, :162-164, :175-178 | route literals; publisher steps; package name; identity fields; thread and per-day layout | Each a HOW | operations ENG |
| platform ARCH:120, :126, :132, :226-228 | "253 x 231 mm"; "JetPack 6.2.1 from the SD-card image"; "Python 3.10"; "step 3", "P26", "20 Hz" | Part dimensions, point releases, procedure pointers, a rate | platform ENG |
| cloud-vision ARCH:176, :69-71, :289 | "Claude Opus 4.5 won"; the four route paths; "Transcoded to JPEG first" | Today's default model; paths; a mechanism | The decision is "chosen by replay" and "one route per question" |
| recordings ARCH:124, :256 | "An empty walk exists on disk but not in the bucket"; "Retries with backoff, then 503" | A backend quirk; a mechanism, and also wrong (§5) | recordings ENG |

**Numbers the reviewers judged legitimate** (acceptance bars, or evidence
for a decision):

- Safety's failure table: 18.0 cm, 1.0 cm, 95%, 3 s / 2 s, 0 late ticks.
- Policy's arrival bars: 95%, 0.60 m.
- Perception's 250 ms at 15 W.
- World's SLAM bars: 5 cm / 2 deg and 10 cm / 3 deg.
- Ros's failure table.
- Motor-board's measured bars: 1.02 / 0.047 cm, 120/120.
- Platform's $86, ~5 A, the 15 W decision and the 9 V floor.
- Every "rejected because it measured X" figure.

## 4. Restatement and thinness in engineering specs

No engineering spec is thin. All 15 supply values with units, where each
value is read, commands, and failure signatures. The defects are copies
and gaps in procedures:

- **Hand-transcribed copies that have already drifted from their source:**
  - cloud-vision ENG:157-163 copies the deploy command but drops
    `--region`, which `service/lambda/build.sh` prints.
  - platform ENG:128-165 transcribes `tools/jetson/README.md` but drops its
    one-time git setup, so step `:142` cannot work on a fresh board.
  - ros ENG:199-208 copies `service/slam/README.md`'s failure table,
    including a wrong row (§5).
- **Procedures without expected output:**
  - body ENG:176 ("Add a backend");
  - cloud-vision ENG:175-190 (the real Converse call, and changing the
    default model);
  - simulator ENG:170-186 (the sweeps and the paid mission). Its
    `demo_mover_sweep 20 0` comment also misreads the arguments, which are
    `n` and `seed`.
- **Copied parameters.** control-api ENG:113-126 repeats safety's
  parameter rows (§6).
- **Known gaps restated from the parent.** operations ENG:247-261
  restates its architecture spec's open questions instead of linking them.
- **A sync flag with no failure signature.** recordings ENG:135 uses
  `--size-only` without saying that it silently skips edits to
  `labels.json`, `meta.json` and `tags.json` that keep the same size.
- **Platform ENG** uses tags `[V]` and `[I]` that are missing from its own
  legend, and lists the bench output without `model_ms`, `handling_ms` and
  `clip_ms_per_crop`.

## 5. Verification failures

### High: would mislead someone building or operating the robot

| Spec claim | Code reality | Kind |
|---|---|---|
| body ARCH:203, control-api ARCH:193: "a stop never depends on the container" | `RosDriveRobot.stop()` posts zeros to the bridge for three drivers, each with a 2.0 s timeout, before `inner.stop()` (`robot/ros_drive.py:233-243`, `:91`). The watchdog calls it synchronously on the event loop (`robot/server.py:302`). A hung bridge delays the stop by up to ~6 s and freezes every route meanwhile. A dead container (connection refused) fails fast. Re-checked by the coordinator; the stall itself is UNCONFIRMED (not run). | **Code defect** |
| safety ARCH:127-131 "backing up is vetted against the rear on every path"; ARCH:86-92 "unobserved is never clear" | With no usable scan, `rear_clearance()` returns `(None, "no_rear_sensor")` and the reverse proceeds (`robot/safety.py:458-473`, `:614-618`). `pivot_blocked()` likewise returns None. `HardwareRobot` without sensors has no scan, so **on the car, until the lidar driver lands, only FORWARD is guarded** (by `get_distance()` = 0.0). The docstring calls this deliberate. Neither spec states the decision or the exposure. Re-checked by the coordinator. | **Undocumented decision** |
| platform ENG:160-163 and `PLAN-ros-alignment.md` 3.33 step 6: G4 is "5 consecutive passes" with `ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake SIM_MAP=scaled_house` | `/health` reports `sim_map` only when `mode == "sim"` (`robot/server.py:991`). Under hardware with the fake board it is `null`, so `tests/test_ros_chain_live.py:62-65` and `tests/test_nav_live.py:34-35` **skip**. As specified, G4 would "pass" on five runs of skips. Re-checked by the coordinator. | **Code defect / false gate** |
| ros ENG:205 and `service/slam/README.md:235`: "`reports no wheels (usable: false)` means the server is still on `drive: direct`" | `GET /wheels` answers in either drive mode, and `MockRobot` is `usable: true`. The real causes are a teleop/replay body, or `HardwareRobot` before its first `T:1001` frame (`robot/hardware_robot.py:345`). In the second case `on_activate()` -> `read()` -> ERROR deactivates the plugin **for good** (`picar_sim_hardware.cpp:90-94`, `:157-160`). That is a start-up race on hardware day, which the spec misattributes. | **Wrong runbook + start-up race** |

| policy ARCH:132-133 and :216, perception ARCH:212: a local false positive is "bounded by the gate and the hysteresis" / costs "at most one wasted cloud call" | `_steer_to()` steers on any single `DETECTED` frame (`brain/tiered.py:1128-1158`). The two-frame hysteresis gates only the *cloud* triggers (`:861-874`). One false detection after absences steers the robot until the next cloud call. The `tiered.py:217-218` comment makes the same claim. Re-checked by the coordinator. | **Spec misstates the bound on wrong-object steering** |

### Medium

| Spec claim | Code reality |
|---|---|
| policy ENG:182: arrival recorded "69/69, 676/678 … farthest 0.386 m" | These are 3.11's starter-house numbers. The test runs in the scaled house now. The current record (PLAN 3.32) is **69/69, 689/689, 677/689 (98.3%), farthest `found` 0.51-0.52 m**: 0.08 m inside the 0.60 m bar, not 0.21 m. |
| policy ENG:181, :159-161: R1 "4.06 cells / 0.6 reversals", R1c "98.3-98.6%"; the A/B demo's quoted output | These are starter-house numbers from 2026-09-25, printed beside "runs in the scaled house since 3.32", which only recorded pass/fail. `tests/demo_hold_bearing_ab.py` still builds the starter house, where the Rover clips the jamb. |
| perception ENG:146-147: the 400 text "ultralytics is not installed" | With the shipped YOLOE default the message is `"YOLOE needs ultralytics. …"` (`brain/perceive.py:1347`), so an operator searching the logs for the documented text won't find it. |
| policy ENG:175: `arrival.state` stays `not_judged` without a scan | `brain/arrival.py:84-85` returns `approaching` on any undetected frame, before the scan check, so on a phone walk most frames read `approaching`. |
| safety ARCH:70-72 "every motion passes the vet, standing wheel commands included" | `vet_wheel_velocity()` passes a command through when `get_wheel_state()` is unusable (`robot/safety.py:365-367`). `HardwareRobot` is unusable until its first frame (`robot/hardware_robot.py:349`), so a `POST /wheels` in that window is unvetted. |
| body ARCH:201 "a stop ends any verb within one control period; the stop is never overwritten" | A race in `carry_out_verb` (`robot/interface.py:248-268`): a `stop()` landing between the `stop_count` check and `set_wheel_velocity()` is overwritten, and the `finally` skips zeroing. The wheels then run until the watchdog stops them (~1 s). `/stop` does not take `motion_lock`. UNCONFIRMED (read only). |
| body ARCH:174-178, :204 "the conformance suite runs every backend and every wrapper" | `BACKENDS` has six entries, and `RosDriveRobot` is not among them (`tests/test_robot_contract.py:85`). body ENG:134 is right. |
| body ENG:48, `robot/interface.py:401`: odometry heading is "clockwise-positive from start" | `MockRobot` returns a compass bearing (90.0 at start, by dry import; `sim/mock_robot.py:710`). `HardwareRobot` is relative to start. The two backends disagree, and the contract suite does not pin it. |
| body ENG:153, motor-board ENG:181-186 (first contact on hardware) | `mode: hardware` without sensors has no `.world`, so the shipped `world.mode: sim` raises at start-up (`world/factory.py:72-86`). The procedure needs `WORLD_MODE=none`; only the world ENG says so. body ENG:153 also uses `/dev/ttyUSB0` against its own udev advice. |
| ros ARCH:175-178 (D6) "people reach the wheels through the verb route" | Under `drive: ros` only `twin-dpad` has a twist_mux input (`picar_bridge/bridge.py:79-83`). `teleop-operator` and unnamed callers, who rank as manual, are refused `ros_unavailable`. This is `docs-review/REPORT.md` V10, still open. |
| ros ENG:137 "`drive.bridge_url` is read in robot/ and world/ factories" | `world/factory.py:102-104` reads `world.bridge_url`, never `drive.bridge_url`. Setting only the yaml key sends verbs and world reads to different bridges. |
| motor-board ARCH:125-126 (D3) states as settled that `T:0` is aimed at the arm | Its own ENG:226-230 says the sources disagree. The fake stops the wheels on `T:0` (`sim/fake_esp32.py:251-253`). The firmware source is not in the repo. |
| recordings ARCH:256 "vision down during replay: retries with backoff, then 503" | Per-frame errors are swallowed and counted. The replay is stored `unusable` and returns 200 (`control/admin_server.py:653-701`). A 503 is returned only when no vision URL is configured. |
| recordings ENG:243-248 known gap (missing `VISION_URL`, judge switch) | The walks function also lacks `VISION_SHARED_SECRET` (`cloudformation/serverless.yaml:203-210`), so it falls back to the walks secret. Deployed replay would 401 on every frame, 4xx is not retried, and every replay ends `unusable`. operations ENG:128, :259 and ARCH:92 assume replay runs. UNCONFIRMED against the live stack. |
| cloud-vision ARCH:145-148, :285 "out-of-vocabulary values become unknown; fields not asked for are removed" | `target_direction` is never coerced and `target_visible` is not type-checked, so `"false"` is truthy and can pass the reached-implies-visible guard (`vision_core.py:850-880`). Only `obstacle_ahead` is stripped. The ENG table is right. |
| cloud-vision ENG:157-163 deploy command | No `--region`. `build.sh` uploads to us-east-2 and prints `--region` (`service/lambda/build.sh:22`, `:77`). UNCONFIRMED. |
| recordings ENG:189-190 deploy of `recordings-s3.yaml` | The template creates a named IAM managed policy, so the deploy needs `--capabilities CAPABILITY_NAMED_IAM`. UNCONFIRMED. |
| operations ENG:202 `python -m control.health`, expect OK | `run.sh` runs the brain under `ROUTE_PREFIX=/brain` (`service/tunnel/run.sh:85`), so the default probe gets a 404 and reports UNREACHABLE. |
| platform ENG:48 "setup.sh checks the shipped pipeline lands on `cuda`" (also 3.33 criterion 2) | Only CLIP is asserted (`tools/jetson/setup.sh:73-74`). Risk 1 could be closed with the detector running on the CPU. |
| simulator ARCH:210, ENG:249, platform ARCH:120 "the starter house cannot hold the Rover" | It fits all 3 of 3 rooms (`PLAN-ros-alignment.md:1954`). The real problems are nav2's costmap sealing the doors, and verbs refused on ~0.5 deg of heading error. |
| simulator ARCH:247, ENG:95 "the server reports which house it built" | It re-reads the `SIM_MAP` env with its own default (`robot/server.py:991`). It does not report what the factory built (the root cause of the G4 skip). |
| world ARCH:130-136 (D4) "the version changes whenever the cells change" | Under SLAM it counts `/map` publications, probably ~1 Hz at rest. UNCONFIRMED. |
| control-api ARCH:126-128 "/health is open because a load balancer cannot send headers" | The ALB was deleted on 2026-09-05. The decision stands, but its reason is stale. |
| safety ARCH:234 "motors stop within one watchdog period" | Under `drive: ros` driver silence is covered by twist_mux's 0.25 s input timeout, not the server watchdog, and the spec never names it. In direct mode detection takes the timeout plus one 0.1 s poll. |
| twin ENG:95-102 persisted preferences | `vp_cfg_brain_secret` and `guidanceMode` are omitted (`web-twin/app.js:5288`, `:4457`). |

### Low (one line each)

- **safety ENG:**
  - :47 lists the `path_clearance` order wrong; `depth_grid_facing_away` is
    checked first.
  - :86-88: a zero `/wheels` from the holder sets nothing, and `ignored`
    applies in direct mode only.
- **control-api:**
  - ENG:33: the ROS-down fallback does hold the lock.
  - ENG:168: hardware with the fake board starts fine under
    `world: sim`.
  - ARCH:62: `/world/error` does compute in the server.
- **body ENG:**
  - :28: `stop()` goes to the bridge first.
  - :99: `stddev_cm` defaults to 0.0 in code (3.0 is the yaml value).
- **ros ENG:**
  - :52: settle is the main pass plus at most two corrections.
  - :184: bridge `ok` is always true, so it is not a signal.
- **motor-board ENG:**
  - :30: the patch adds 13 lines.
  - :129: nine other libraries, not eight.
- **world:**
  - ARCH:263-264: with drift on, position met the bar on 9 of 9 laps;
    only heading failed.
  - ENG:129: `SIM_MAP` is read in `robot/factory.py` and
    `robot/server.py`.
  - ENG:176: missing cause, the bridge's single `/world/truth` read
    failed.
  - ENG:149: `$LOCAL_SECRET` is not sourced.
- **simulator:**
  - ENG:204: `test_bearing_turns` pins relative bars, not 4.06 / 0.6, and
    4.06 is now 3.69.
  - ENG:122: a render costs ~6 ms in the starter house but ~41 ms at home.
  - ENG:107, :111: yaml values are shown as code defaults.
- **operations:**
  - ARCH:219, ENG:225: `control/metrics.html` is never asset-checked, and
    `test_static_assets.py` has 10 tests.
  - ENG:118: `VISION_URL` is a literal.
  - ENG:121: `ENV_LABEL` is unread by the vision function.
  - ENG:46: the proxy's header drops are request vs response.
  - ENG:89: a bad `run_id` is a 400.
  - ENG: the parameters table has no Unit column.
- **cloud-vision:**
  - ENG:27, :213: the vision zip does bundle boto3 (via the service
    requirements).
  - ENG:66: `usage` is a dict.
  - ENG:45: a non-object body is a 500.
  - ARCH:80: the describer's users are wrong.
  - ENG's stale-docstring list misses `vision_core.py:69`, which points at
    the deleted `service.yaml`.
- **recordings:**
  - ENG:100: there is no timeout env override.
  - ENG:109: `REPLAY_MIN_COVERAGE` is env-overridable.
  - ENG:250: the summary skips metrics containers.
  - ARCH:254: the refusal is conditional on `RECORDING_BACKEND=s3`.
  - ARCH:226-228: lazy scoring is the only path.
- **twin:**
  - ARCH:72-73: tap-to-goal sends no `x-driver`, so the server
    arbitrates it as `ros` and a person's tap is refused during a mission.
- **platform ENG:**
  - :103: motor power is ~20 W here against `JETSON-BOM.md` 9.5's
    ~15 W.
- **mission ENG:**
  - :95, :97: `policy` and `fault` are plain strings, and a bad value is
    a 400, not a 422.
  - :171-180: the procedure gives no healthy-start output.
- **policy:**
  - ENG:75: under the frontier policy `important_objects` fills on first
    sight, not on arrival.
  - ARCH:115: it is three unavailable triggers, not four.
  - ARCH:37, :47, :59: the `/navigate` and `agent.decide()` names belong
    in ENG.
  - ENG:163: the demo has no expected output.
- **perception ENG:**
  - :26: the sim-pipeline choice is a probe at start, not the first
    frame.
  - :163-165: `--frames-from` is the replay detector's filter.
  - :150-167: the scorer procedure has no sample output.
- **Code prose not yet in any Known gaps:**
  - `config/robot.yaml:293` gives 83 steps (it is 61).
  - `config/robot.yaml:241` says the spin guard counts "turns" (it counts
    degrees).
  - `control/brain_config.py:97` has a misplaced comment.
  - `YoloDetector`'s default weights are the YOLOE checkpoint (latent).

## 6. Pair, boundary, coverage and duplication

**Pair contradictions.** In each pair, the engineering spec is the one
that matches the code:

| Domain | Architecture says | Engineering / code says |
|---|---|---|
| body | Every wrapper is in the conformance suite | It is not |
| safety | Blind reverse and pivot not stated | Code lets them through |
| motor-board | `T:0` settled | Disputed |
| recordings | Replay 503s | Stored `unusable` |
| cloud-vision | Coercion overstated | ENG table is right |
| ros | People can drive | Only `twin-dpad` can |

**Boundaries.** Each of these decisions is stated in two or three
architecture specs; one should own it and the others link:

| Decision | Stated in | Owner |
|---|---|---|
| The brain is a separate process; the watchdog and the ROS-down fallback | safety, control-api | control-api owns process topology; safety owns what silence means |
| "A nav2 goal is an autonomous driver" | world D9, ros D6 | safety, which owns the driver order |
| Serial port ownership, and why Waveshare's ROS nodes were rejected | motor-board D1, ros D3/D4 | motor-board for the port, ros for the plugin seam |
| Where the lidar driver lives | ros D3, motor-board D9 (which says the board is "only the motors") | ros (or body); drop it from motor-board |
| B5 start order | operations (decided), control-api (still an open question) | operations |
| Metrics-as-walks pollution | Caused by operations' storage layout; only recordings lists it | operations owns the fix; recordings owns the list filter |
| The house the server reports | Served by control-api, computed from the env | simulator should expose it |

**Coverage.** Every Python, JS and HTML file under `robot/`, `world/`,
`brain/`, `control/`, `sim/`, `service/`, `firmware/` and `web-twin/` is
named by at least one spec. The only uncovered code:

- `tools/hailo/` (21 files), the Hailo compile loop, which is closed
  history;
- `tools/gpu/` (4 files);
- one-off scripts: `tools/contact_sheet.py`, `label_prepass.py`,
  `steer_check.py` and `yoloworld_crops.py`;
- ROS package manifests (build metadata).

The perception engineering spec should list the GPU and Hailo instruments
in one row each, as historical tooling.

**Duplication.** The same HOW fact appears in two or more engineering
specs. The values agree today: the coordinator checked wheel radius, track,
660 and `LIDAR_X_M` across all copies. Each should have one home.

| Fact | Copies | Canonical |
|---|---|---|
| Chassis constants (wheel radius, track, 660 counts, footprint, `LIDAR_X_M`) | body, motor-board, ros, simulator, platform, safety, policy | platform (physical facts); safety (margins) |
| Safety parameters (`safety.*`, loop periods, `ROS_SILENCE_S`) | safety, control-api | safety |
| The V10 unnamed-driver gap | safety ENG, control-api ENG, safety ARCH | safety ENG |
| Deploy, publish and tunnel procedures | cloud-vision, twin, operations | operations |
| Bridge route fields | world, ros | ros |
| ROS failure signatures | ros ENG, `service/slam/README.md` | the README |
| Jetson bring-up procedure | platform ENG, `tools/jetson/README.md` | the README |
| Serial device and flash dump step | platform, motor-board | motor-board |
| Metrics storage layout | operations, recordings | operations |
| `brain.min_distance_cm` code default (30.0) vs yaml (20.0) | mission, safety (both correct) | mission |
| `tier_corroboration_bar` | policy ENG:114, perception ENG:104 | perception (the decision); policy points to it |
| How the tiered pipeline is built (`FrameReportedPipeline` in the sim) | mission, policy, perception | mission (the service front builds it) |

Already drifted between copies: `--region` in the deploy command, and the
git setup in the Jetson procedure. Operations' `https://<SiteUrl>/app.js`
is also wrong, because `SiteUrl` already carries the scheme and the
trailing slash.

## 7. Fix list

Ordered by how badly each would mislead someone building or operating the
robot. S/M/L is effort. **Code** marks a code change; the rest are spec
edits.

1. **(M, code)** Make the ROS-drive stop independent of the container.
   Call `inner.stop()` first, then send the bridge zeros with a short
   timeout or off-thread (`robot/ros_drive.py:233-243`). Until then, strike
   "never depends on the container" from body and control-api ARCH.
2. **(S, code)** G4 must not pass on skips:
   - `/health` `sim_map` should report the house the factory built,
     including hardware with the fake board (`robot/server.py:991`);
   - the G4 procedure in platform ENG and PLAN 3.33 should say skips are
     not passes.
3. **(S, decision)** Blind reverse and pivot on a body with no scan:
   - **either** refuse them in code when the body can move but cannot see
     astern;
   - **or** record it as a decision in safety ARCH, with its trade-off, and
     add a safety ENG known gap: "`mode: hardware` without the lidar
     driver guards FORWARD only".

   The user decides which.
4. **(S, code + spec)** The `/wheels` start-up race:
   - document it in ros ENG and `service/slam/README.md`: start the robot
     server, wait for `GET /wheels` to report `usable: true`, then start
     the container;
   - consider letting the plugin's `read()` treat `usable: false` like
     unreachable;
   - fix the wrong "drive: direct" signature.
5. **(S, decision)** Local wrong-object steering:
   - **either** fix policy ARCH:132-133, :216, perception ARCH:212 and
     the `brain/tiered.py:217-218` comment to say "one detected frame
     steers; the gate and cloud identity bound it";
   - **or** extend the hysteresis to `_steer_to()`.

   Then replace policy ENG:181-182's arrival and R1 numbers with 3.32's
   scaled-house record, or re-measure them.
6. **(S, code)** `vet_wheel_velocity()`'s pass-through before the first
   feedback frame: refuse instead, or document it in safety ARCH/ENG.
7. **(S, code)** The stop race in `carry_out_verb`: re-check `stop_count`
   after `set_wheel_velocity()` and zero the wheels if it changed.
   Otherwise soften body ARCH:201.
8. **(S)** Hardware-day procedures: add `WORLD_MODE=none` to body ENG:153
   and motor-board ENG's first-contact procedure, and use the udev symlink.
9. **(S, code)** Assert the detector's device too in
   `tools/jetson/setup.sh`. Otherwise risk 1 can close with YOLOE on the
   CPU inside the return window.
10. **(M, decision)** Under `drive: ros`, give every manual-rank driver a
   twist_mux input. Otherwise narrow ros ARCH D6 to "only the D-pad" (V10).
11. **(S/M, decision)** Pick one odometry heading convention and pin it in
    `tests/test_robot_contract.py`.
12. **(M)** Deployed replay:
    - **either** template `VISION_URL` and `VISION_SHARED_SECRET` on the
      walks function;
    - **or** state in operations and recordings that replay is disabled as
      templated.

    Fix recordings ARCH:256 either way. Verify against the live stack
    first.
13. **(S)** Fix the ros ENG:137 `bridge_url` claim, or make one key serve
    both the robot and world factories.
14. **(S)** motor-board ARCH D3: mark `T:0` disputed and never relied on.
15. **(S)** cloud-vision ARCH: narrow the coercion claim, or coerce
    `target_direction` and `target_visible` in code.
16. **(S)** Correct the starter-house claim in simulator ARCH/ENG and
    platform ARCH: it fits; nav2's costmap and heading tolerance are the
    problem.
17. **(S)** Correct the procedures:
    - operations health command (`/brain`);
    - cloud-vision deploy (link operations, or add `--region`);
    - recordings deploy (`CAPABILITY_NAMED_IAM`);
    - recordings `--size-only`;
    - operations `SiteUrl` parity URL.
18. **(M)** De-duplicate per §6: one canonical copy for each fact, and
    links elsewhere.
19. **(M)** Move the HOW in §3 out of the architecture specs, starting with
    platform (dated and commercial terms), safety (sensor-neutral wording)
    and ros (version pins).
20. **(S)** Add the metrics-as-walks gap to operations ENG. Own the fix as
    a prefix filter in `list_walks()`.
21. **(S)** The low-severity corrections in §5, in one pass per domain.
22. **(S)** Add a row for the GPU and Hailo tooling to perception ENG.

## 8. Checked and found correct

So nobody needs to re-check these:

- **safety ENG (~60 claims), all held:**
  - every constant in §Parameters against `robot/safety.py`;
  - the server's loop constants and late-tick rule;
  - the comparison edges, and the 1/64 binary search;
  - the driver ranks and the `arbitrate()` order, step by step;
  - all six refusal reasons;
  - every test count;
  - every recorded number against `PLAN-ros-alignment.md` 3.10-3.30.
- **body ENG (~45 claims):**
  - factory selection and env precedence;
  - every interface constant;
  - the sim's grid, scan and distance (by dry import);
  - the backend matrix, `carry_out_verb`'s shape, `BACKENDS`, and the
    test counts.
- **control-api ENG (~50 claims):**
  - every route, method and secret dependency;
  - request and response shapes, and `/health` fields;
  - CORS, identity, and the tunnel ports;
  - the stale-docstring list.
- **world ENG (~38), ros ENG (~60), motor-board ENG (~45):**
  - every bridge route, topic, TF frame, and `controllers.yaml` /
    `twist_mux` / `slam` / `nav2` value;
  - wheel constants pinned four ways;
  - the wall-linter budgets;
  - `HardwareRobot` and `FakeEsp32` constants and frame handling;
  - every PLAN 3.13-3.29 number quoted.
- **mission ENG (~48), policy ENG (~55), perception ENG (~45):**
  - every `brain:` key's shipped and code values (by dry import of
    `load_brain_config()`);
  - every env var and route under `ROUTE_PREFIX`, and the status shape;
  - the 9 outcomes, tick order and `_HaltGate`'s forwarding;
  - the 61-step trace;
  - the `TieredVision` signature, the precedence ladder and every tier
    and arrival constant (by dry import);
  - the `perceive()` steps and `as_dict` keys;
  - the P23/P24 and 1.11a numbers, and the Mac bench numbers;
  - the `perception_eval` defaults, and every test count.
- **cloud-vision ENG (~42), recordings ENG (~48), twin ENG (~40):**
  - every route and status code;
  - the reply schema and its coercion table;
  - the model allow-list and region pins;
  - every Lambda and template value, and the packaging;
  - walk store and scoring constants;
  - the bucket lifecycle;
  - all 19 twin parameter rows against `app.js`;
  - `assets.json` and `sync.sh`.
- **simulator ENG (~55), operations ENG (~50), platform ENG (~45):**
  - every sim constant and house layout (by dry run);
  - template, script, health and identity facts;
  - metrics shapes;
  - xacro values and tags;
  - setup and bench facts;
  - the BOM facts against `JETSON-BOM.md` and `HARDWARE-BOM.md`.
- **Architecture specs:** every contract row and failure row in all 15 was
  checked; the exceptions are listed in §5.

## 9. Resolution (2026-10-02, same day)

**Code: fixes 1-3 landed, each with tests that fail against the old code.**

| Fix | Change | Pinned by |
|---|---|---|
| 1. A stop never waits on the container | `RosDriveRobot.stop()` zeroes the robot directly first. It then zeroes the ROS inputs on one single-flight background thread (0.5 s per post), and holds stale non-zero wheel commands from ROS at zero for 0.4 s. The next verb lifts the hold. | `tests/test_ros_drive.py` (5 new tests) |
| 2. G4 cannot pass on skips | `build_world()` names the world it builds, and `/health` `sim_map` reads that name. It is right under the fake motor board, `null` with no sim house, and unaffected by `SIM_MAP` changing later. `PLAN-ros-alignment.md` 3.33 now says a skip is not a pass. | `tests/test_health_sim_map.py` |
| 3. No blind reverse (user decision: refuse reverse only) | `reverse_clearance()` answers `0.0` / `astern_not_observed` when the body reports its wheels and has no usable scan. This covers the reverse verb, a standing reverse command and a backwards settle pass. Turns stay allowed. A body with no wheels keeps the old answer. | `tests/test_blind_reverse.py` |

**Specs.** Every finding in §3-§6 was applied to the 30 specs by five
agents, each of which re-checked its findings against the code before
editing. Code defects not fixed today are stated as Known gaps or Open
questions rather than promised:

- fixes 4, 6, 7, 9-11 and 14;
- arrival on a local match alone (below);
- the plugin's permanent deactivation;
- people other than the D-pad under `drive: ros`.

Duplicated facts now have one canonical home each, per §6. Also changed:

- `service/slam/README.md`: its wrong "no wheels" row is fixed, and the
  start-up order is added.
- `PLAN-ros-alignment.md` 3.33: criterion 2 and the G4 criterion are
  amended.
- `.gitignore`: it no longer hides `docs/recordings/`.

**Found while fixing:**

- **Tiered arrival can declare `found` on a wrong object.** Local steering
  is bounded only by the per-frame probability gate. Under the shipped
  asynchronous cloud call, the cloud's answer never overrides a local
  sighting, and `brain/arrival.py` ends a mission `found` on a local
  detection plus lidar range alone. Confirmed with a fake pipeline (no
  models). It is recorded as an open question in policy and perception,
  next to 1.11a, which is reported but not enforced.
- **`python -m tests.demo_active_search` ends NOT FOUND after 150
  steps.** Most of its turns are clamped by the pivot guard. This predates
  today's changes: the result is the same with fix 3 reverted. Not yet
  diagnosed.
