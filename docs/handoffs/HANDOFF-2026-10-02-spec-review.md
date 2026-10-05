# Handoff 2026-10-02 -- what the spec review left open

The session that wrote `docs/` (decision 0001: an architecture and an
engineering spec for every component) ran an independent review of all 30
specs against the code (`docs-review/SPEC-REVIEW.md`). The review's three
most serious findings were fixed the same day (`fe9508e`), and the specs
were corrected (`7f6ee49`). This file is everything that is still open, in
the order to pick it up.

Every item below is already recorded in the relevant spec, under Known
gaps or Open questions, so the specs do not promise any of it. **When an
item closes, update that spec in the same commit** (decision 0001), and
strike the item here.

Project rule (CLAUDE.md section 7): write each item's acceptance criteria
down BEFORE the run, measure through the real mission path, and pin the
result in a test. Each item says "**Done when**".

## 0. Start here

- `python tools/spec_lint.py` should report 0 errors and 0 warnings.
- `pytest tests/test_spec_lint.py` should pass. Both are fast.
- Read `docs/README.md` for the reading path.
- Read `docs-review/SPEC-REVIEW.md` §5 and §7 for the evidence behind each
  item. The § and fix numbers below refer to that file.

## 1. Decided 2026-10-02 -- build these first

The user decided all four on 2026-10-02. **All four are built or closed
(2026-10-02): 1b, 1d and 1a in separate commits; 1c needed nothing beyond
1a.** Each decision, its rejected alternatives and its trade-off are in
the architecture specs named below, and the mechanism in the matching
engineering spec. Spec review 3 (`docs-review/SPEC-REVIEW-3.md`) then found
the builds closed less than claimed; its fixes are recorded there (§9).

~~**1a. DECIDED: the cloud confirms identity at arrival.**~~ **BUILT
2026-10-02:** `TieredVision.confirm_arrival()`, asked by
`MissionAgent._review_scene()` through `MissionRunner._guarded_confirm()`;
`tests/test_arrival_confirmation.py`; the 3.11 sweep unchanged (69/69,
689/689, 677/689, 0 false). Kept below as the record.

A tiered mission
can end `found` on a wrong object today. Found by the fix agent and
confirmed with a fake pipeline (no models):

- One local `DETECTED` frame steers the robot (`brain/tiered.py`
  `_steer_to()`).
- The two-frame hysteresis gates only cloud triggers.
- Under the shipped `tier_async_cloud: true`, the cloud's answer never
  overrides a local sighting.
- `brain/arrival.py` declares `found` on a local detection plus lidar range
  alone.

**Build:** before a tiered mission ends `found`, one paid cloud call on the
arrival frame must agree it is the target; if it disagrees, no `found`.
Steering is unchanged. Rejected: enforcing 1.11a corroboration at arrival
(free, but measured net-negative and unproven); extending the hysteresis to
steering (reduces wrong-object chases, does not stop a wrong `found`);
leaving it as is. Trade-off: one paid call per arrival, spent on identity,
which the cloud is good at. Specs: `docs/policy/ARCHITECTURE.md` ("The
cloud confirms identity at arrival; the lidar decides distance"),
`docs/perception/ARCHITECTURE.md`, and both engineering Known gaps. The
`brain/tiered.py:217-218` comment still claims hysteresis bounds it, so fix
the comment in the same commit.

**Done when** a test, with the cloud faked, shows a wrong-object arrival is
refused when the cloud disagrees and a right one still ends `found`; a
single false-positive run (absent, absent, detected-wrong, ...) cannot end
`found`; and the sweep in `tests/test_arrival.py` still meets 3.11's bars
in the scaled house: >= 95% of arrivals `found`, none beyond 0.60 m.

~~**1b. DECIDED: a stop ends a nav2 goal**~~ **BUILT 2026-10-02:**
`robot/server.py` `stop()` cancels the goal on a background thread;
`tests/test_stop_cancels_goal.py`. Kept below as the record.

(second review, H1;
`docs-review/SPEC-REVIEW-2.md`). Two reviewers found this independently.

- `POST /stop` only calls `robot.stop()` (`robot/server.py:656-669`).
- The bridge cancels a goal only on a non-zero `twin-dpad` twist
  (`service/slam/src/picar_bridge/picar_bridge/bridge.py:453-455`).
- So nav2 resumes once the stop hold ends. Under `drive: direct` with
  `WORLD_MODE=ros`, it resumes within ~50 ms.
- This predates today's work.

**Build:** `/stop` also cancels any active goal, on a background thread
after `robot.stop()`, so the stop never waits on ROS; a person re-sends a
goal to resume. Rejected: a stop that only pauses, plus a separate cancel
control. Owner: the safety architecture ("Who drives"); ros D6, world D9,
the twin and body link to it.

**Done when** a test with an active goal shows the wheels still at zero
past `STOP_HOLD_S` after `/stop`, and a new goal can be set afterwards.

~~**1c. DECIDED: a landed cloud `target_reached` does not end a tiered
mission; the arrival rule only**~~ **DONE 2026-10-02:** nothing to build
beyond 1a; the policy specs record it. Kept below as the record.

 (second review, M3).

- Under the shipped `tier_async_cloud: true` it never does. Only
  `brain/arrival.py` can end the mission, and that stays so: the lidar's
  distance is measured, the cloud's is not.
- Rejected: applying it (puts an uncalibrated distance back on the car);
  applying it only on bodies with no scan.
- With 1a: the cloud confirms IDENTITY at arrival, the lidar decides
  DISTANCE.
- Accepted consequence: tiered phone walks (no lidar) end `max_steps` when
  they arrive, **by design** (P7e stays).

**Done when** nothing beyond 1a is built: today's code already behaves
this way. The policy specs record it.

~~**1d. DECIDED: bridge failures count toward ROS liveness**~~ **BUILT
2026-10-02:** `RosDriveRobot.bridge_up()` (`robot/ros_drive.py`), ANDed into
`ros_up()`; `tests/test_ros_fallback.py` `test_1d_*`. Kept below as the
record.

(second review,
M2).

- `ros_up()` is judged only from the plugin's `/wheels` posts.
- If the bridge dies but the plugin lives, even a person's `/action` is
  refused `ros_unavailable`.

**Build:** a failed send to the bridge marks ROS down, so 3.24 G3's
fallback applies (a person drives direct, autonomy refused). Rejected:
making the container exit when a node dies (launch-file change only); both.
Owner: the safety architecture ("When ROS dies, only a person drives");
control-api and ros link to it. A dead multiplexer or controller behind a
live bridge is left as is (fails toward stop).

**Done when** a test with a bridge that refuses connections shows
`drive.ros_up` false, a person's verb runs direct, and an autonomous verb
is refused `ros_unavailable`.

## 2. Hardware-path code fixes (before the Rover, ideally during 3.33)

**2-pre. G4 cannot pass as written** (second review, H2). It needs, in
order:

- a brain on :8001 with `ROUTE_PREFIX=/brain`;
- `SIM_MAP=scaled_house` in pytest's own environment;
- the container started only after `GET /wheels` is usable;
- the container named `picar-ros`.

Platform ENG has the full command set, and `tools/jetson/README.md` §4
carries it too (2026-10-03).

~~**2a. The wheel plugin's start-up race (fix 4).**~~ **BUILT 2026-10-03:**
`HardwareRobot` answers `awaiting_feedback: true` before its first frame and
`on_activate()` activates on it; live-tested with `SIM_BOARD_SILENT_S=30`
(`tests/test_startup_race.py`). The start order no longer needs the wait.

- **What goes wrong:** if the ROS container activates before
  `HardwareRobot` has read its first `T:1001` frame, `GET /wheels` answers
  `usable: false`. The plugin's `read()` then returns ERROR and
  `ros2_control` deactivates it for good
  (`service/slam/src/picar_sim_hardware/src/picar_sim_hardware.cpp:90-94`,
  `:157-160`).
- **Today's workaround:** the start order is documented in
  `service/slam/README.md`: wait for `/wheels` to report usable, then start
  the container.
- **Fix:** make `read()` treat `usable: false` as "not yet": keep trying,
  as it already does for an unreachable server.

**Done when** a container started before the board's first frame drives
normally once frames arrive (live test against the fake board with a
delayed first frame).

~~**2b. Unvetted wheel commands before the first feedback frame (fix 6).**~~
**CLOSED by 3.34 (2026-10-03):** `POST /wheels` before the first frame is
refused `no_feedback`; pinned in the handoff's terms in
`tests/test_startup_race.py`.

- **What goes wrong:** `SafetyController.vet_wheel_velocity()` passes a
  command through untouched when `get_wheel_state()` is unusable
  (`robot/safety.py`, the early return). `HardwareRobot` is unusable until
  its first frame (`robot/hardware_robot.py:349`), so a direct
  `POST /wheels` in that window goes to the board unvetted.
- **Fix:** refuse with `unsupported` (or `safety_distance`) on a body that
  has motors but no feedback yet.

**Done when** a test shows `POST /wheels` before the first frame is
refused, and the same command after the first frame is vetted as usual.

~~**2c. The stop race in `carry_out_verb` (fix 7).**~~ **BUILT 2026-10-03:**
re-checked after the re-command; `tests/test_verb_stop_race.py` (red first).

- **What goes wrong:** a `stop()` that lands between the `stop_count` check
  and `set_wheel_velocity()` gets overwritten, and the `finally` then skips
  zeroing (`robot/interface.py:248-268`). `/stop` does not take
  `motion_lock`. The window is small, and the watchdog ends it about 1 s
  later.
- **Fix:** re-check `stop_count` after the re-command and zero the wheels
  if it changed.

**Done when** a test that injects a stop exactly in that window (a hook or
a fake body) shows the wheels at zero within one control period.

~~**2d. One odometry heading convention (fix 11).**~~ **BUILT 2026-10-03:**
start-relative, clockwise-positive (the docstring's); `MockRobot` changed,
pinned on every backend in `tests/test_robot_contract.py`; the frontier
trace is unchanged.

- **What goes wrong:** `MockRobot.get_odometry()` returns a compass bearing
  (90 at start), while `HardwareRobot` returns degrees turned since start.
  `robot/interface.py:401` says "0 = start". The contract suite does not
  pin it.
- **Fix:** pick one convention (start-relative matches the docstring), fix
  the other backend and every reader, and add the check to
  `tests/test_robot_contract.py`.

**Done when** the contract test passes for all six backends and the
frontier trace (`tests/data/frontier_trace_centred.json`) is unchanged, or
re-pinned with the reason given.

~~**2e. `RosDriveRobot` is not in the conformance suite (fix 6 in §5).**~~
**BUILT 2026-10-03:** `ros_drive` is the seventh entry in `BACKENDS`, over
`tests/test_ros_drive.py`'s fake chain; every contract test passes.
Either add it to `BACKENDS` in `tests/test_robot_contract.py` (with a fake
bridge, as `tests/test_ros_drive.py` does), or change body ARCH to say
"every backend and the mission gate".

## 3. Decisions to make, then build

**3a. People other than the D-pad under `drive: ros` (V10, fix 10).** **DECIDED 2026-10-05 (the user, on the recommendation): every person's commands go on the people's input -- `robot/ros_drive.py` `ros_input_for()`. Built.**

- **Today:** only `twin-dpad` has a `twist_mux` input
  (`service/slam/src/picar_bridge/picar_bridge/bridge.py:79-83`).
  `teleop-operator` and unnamed callers, who rank as manual, are refused
  `ros_unavailable`.
- **Options:** give every manual-rank driver an input, or accept that only
  the D-pad drives under ROS and say so as a decision.
- Owner: the safety domain (driver order), with ros.

**3b. Twin tap-to-goal is ranked as autonomous.** **DECIDED 2026-10-05: yes, a person's tap outranks the brain. Built: the twin names the person, and `/world/goal` arbitrates a named person at their rank.**

- **Today:** the page posts `/world/goal` with no `x-driver`, so the server
  arbitrates the goal as `ros` and a person's tap is refused while a
  mission holds the robot.
- **Decide:** should a person's tap outrank the brain, as the D-pad does?
- Specs: twin and safety, open questions.

**3c. Deployed replay (fix 12). Verify against the live stack first.** **VERIFIED 2026-10-05: the walks Lambda had neither value. Template fixed (`ReplayVisionUrl` + the vision secret); NOT deployed.**

- **Probable state:** as templated, the walks Lambda has neither
  `VISION_URL` nor `VISION_SHARED_SECRET`
  (`cloudformation/serverless.yaml:203-210`). Replay would answer 503, and
  with only a URL added, every frame would get a 401.
- **Check:** `aws lambda get-function-configuration` on the walks function.
- **Then:** either template both values or decide replay stays local-only.
- Specs: recordings and operations.

## 4. Smaller code items

- **4a. DONE 2026-10-03.** `tools/jetson/setup.sh` should assert the detector's device on
  `cuda`, not only CLIP's (`:73-74`). PLAN 3.33 criterion 2 says to check
  it by hand until then. **Do this before 3.33 step 3 if you can**: it is a
  one-liner, now that `bench_perception.py` reads the predictor's device
  (`87c92f1`).
- ~~**4b.**~~ **BUILT 2026-10-03** (service-side, plus `/guidance`'s flag; not
  deployed until the Lambda is rebuilt). cloud-vision: coerce `target_direction` to its vocabulary and
  type-check `target_visible` in `service/vision_analyze/vision_core.py`.
  Today the string `"false"` is truthy and can pass the
  reached-implies-visible guard. Or decide every caller coerces.
- ~~**4c.**~~ **BUILT 2026-10-03:** filtered in `list_walks()` and refused by
  every walk route (`tests/test_walk_store.py`, `tests/test_admin_server.py`). Metrics rows show up as zero-frame walks. Filter the metrics
  prefix in `control/walk_store.py` `list_walks()`. Operations owns the
  storage layout and recordings owns the filter.
- ~~**4d.**~~ **BUILT 2026-10-03:** refused by name (`ros_unavailable`; 503 on
  read and cancel), `tests/test_ros_goals.py`. `world/ros_world.py` goal routes return 500 when the bridge is
  down (no `except` around httpx). Refuse them by name instead.
- ~~**4e.**~~ **BUILT 2026-10-03:** `drive.ros_up` false under `drive: ros` is a
  verdict input, named by half (`tests/test_health.py`). `control/health.py`: its verdict ignores `drive.ros_up`, so a
  dead ROS container doesn't show in the health check.
- ~~**4f.**~~ **BUILT 2026-10-03:** wired (`SIM_MAP` overrides it), now a map
  name; `tests/test_config_and_factory.py`. `config/robot.yaml`'s `sim_map:` key is read by nothing. Wire it
  or delete it.
- ~~**4g.**~~ **DONE 2026-10-03** (every item below; the specs' matching
  Known-gap bullets removed). Stale code prose to fix in one pass:
  - `config/robot.yaml` :6 ("PiCar-X"), :86 ("ros not built yet"), :241
    ("turns"; the guard counts degrees), :293 ("83 steps"; it is 61);
  - `control/brain_config.py` :58-59, :66-70, :97, and :156, where
    `min_distance_cm` defaults to 30 against the yaml's 20;
  - `robot/safety.py:46-57`;
  - `robot/interface.py` :6, :387;
  - `robot/server.py` :40-53, :693-698;
  - `sim/mock_world.py:161`;
  - `sim/renderer.py` :74, :410;
  - `brain/perceive.py:125-126`;
  - `brain/tiered.py` :17-31, :831-834;
  - `brain/vision_agent.py:35-42`;
  - the `AGENT-HARNESS.md` §12-vs-§10 citations;
  - `service/vision_analyze/vision_core.py:69`.

- ~~**4h.**~~ **BUILT 2026-10-03:** peak and hits over every candidate
  (`tests/test_target_probe.py`). **`control/target_probe.py:65-69` reads its "peak" from
  `r.best`,** which is set only on DETECTED (P >= 0.8). So "0.000 = inert
  prompt" is wrong, and `--gate` below 0.8 does nothing. Compute the peak
  over `max(c.probability for c in r.candidates)`, then fix `CLAUDE.md`'s
  description of the probe (second review, M5).
- ~~**4i.**~~ **BUILT 2026-10-03:** `build.sh` exits 2 before building when
  either secret is empty (`tests/test_lambda_packaging.py`). **`service/lambda/build.sh` prints a deploy command that expands
  `$VISION_SHARED_SECRET` and `$WALKS_SHARED_SECRET`,** but the secrets
  files define `VISION_SECRET` and `WALKS_SECRET`. Pasted as printed, it
  deploys with **no auth**. Make the script refuse to print the command
  when either is empty (M4). Operations ENG now gives the export lines.
- ~~**4j.**~~ **BUILT 2026-10-03:** scaled into the window, ratio kept
  (`tests/test_board_speed_clamp.py`). **The host never clamps wheel commands to the board's ±2.0 m/s
  window.** The board drops an out-of-range `T:1` and keeps running the
  old setpoint, while the command still feeds the heartbeat
  (`robot/hardware_robot.py:330-336`).
- ~~**4k.**~~ **DONE 2026-10-03.** **`brain/arrival.py:6-9`'s docstring is stale** (see 1c), and so is
  `world/factory.py:14-22`'s docstring.

## 5. To investigate

~~**5a.**~~ **FIXED 2026-10-03:** the diagnosis held (a refused turn re-chosen
forever), and the demo also passed no world; both fixed, `tests/test_refused_turn_loop.py`,
the demo now finds the backpack at step 60. **`python -m tests.demo_active_search` ends NOT FOUND after 150
steps.**

- Of 132 `Blocked` lines, nearly all are LEFT turns clamped by the pivot
  guard (3.19), and the stuck-breaker never fires.
- It predates today: the result is the same with fix 3 reverted.
- **Probable cause** (second review, UNCONFIRMED): two clearance
  definitions disagree. The left ray clears `side_clearance_cm + 1`, but
  the pivot guard refuses the swept corner. A refused LEFT is not
  "executed", so the next `decide()` re-peeks and picks LEFT forever, and
  neither the boxed-in fallback nor the stuck-breaker fires
  (`brain/agent.py:356-411`).
- Recorded in `docs/engineering/policy/ENGINEERING.md`, Known gaps.

~~**5b.**~~ **DONE 2026-10-03:** scaled house, re-measured (sized 6.36 cells /
0.2 reversals vs quarter -0.29 / 12.3; policy ENG). `tests/demo_hold_bearing_ab.py` still builds the starter house,
which the Rover's chassis clips (3.32). Move it to the scaled house and
re-measure R1's A/B there. Policy ENG holds the old numbers as
starter-house history only.

~~**5c.**~~ **CHECKED 2026-10-03:** 10/10 alone and 5/5 full-suite runs passed;
load-only, left as is. `tests/test_settle_pass.py::test_3_a_forward_is_a_cell_on_either_firmware[stock]`
failed once in a full-suite run while five agents loaded the machine: a
28.6 cm move against a bar of 30 ± 1.34. It passed alone (2/2) and in its
file (8/8). If it fails again without load, it is a real flake in a
wall-clock test.

## 6. Housekeeping

- **`/spec-review` is local only.** `.claude/` is gitignored, so
  `.claude/commands/spec-review.md` exists only on this laptop. Un-ignore
  `.claude/commands/` to share it. The prompt itself is committed at
  `docs/SPEC_REVIEW_PROMPT.md`.
- **Run `/spec-review` again after any batch of the fixes above.** The
  linter cannot see false claims; only the review can.
