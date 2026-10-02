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

## 1. Decision needed from the user first

**1a. A tiered mission can end `found` on a wrong object.** Found by the
fix agent and confirmed with a fake pipeline (no models):

- One local `DETECTED` frame steers the robot (`brain/tiered.py`
  `_steer_to()`).
- The two-frame hysteresis gates only cloud triggers.
- Under the shipped `tier_async_cloud: true`, the cloud's answer never
  overrides a local sighting.
- `brain/arrival.py` declares `found` on a local detection plus lidar range
  alone.

So the only bound on a wrong object is the per-frame P >= 0.8 gate. The
options:

- extend the hysteresis to steering;
- enforce 1.11a corroboration at arrival (it is reported today, not
  enforced, see `pending_decision_1_11a` in memory);
- require a cloud identity check before `found`;
- some combination of these.

Specs: `docs/policy/ARCHITECTURE.md` and `docs/perception/ARCHITECTURE.md`
(open questions). The `brain/tiered.py:217-218` comment still claims
hysteresis bounds it, so fix the comment with the decision.

**Done when** the chosen rule is pinned by a test where a single
false-positive run (absent, absent, detected-wrong, ...) cannot end
`found`, and the sweep in `tests/test_arrival.py` still meets 3.11's bars
in the scaled house: >= 95% of arrivals `found`, none beyond 0.60 m.

## 2. Hardware-path code fixes (before the Rover, ideally during 3.33)

**2a. The wheel plugin's start-up race (fix 4).**

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

**2b. Unvetted wheel commands before the first feedback frame (fix 6).**

- **What goes wrong:** `SafetyController.vet_wheel_velocity()` passes a
  command through untouched when `get_wheel_state()` is unusable
  (`robot/safety.py`, the early return). `HardwareRobot` is unusable until
  its first frame (`robot/hardware_robot.py:349`), so a direct
  `POST /wheels` in that window goes to the board unvetted.
- **Fix:** refuse with `unsupported` (or `safety_distance`) on a body that
  has motors but no feedback yet.

**Done when** a test shows `POST /wheels` before the first frame is
refused, and the same command after the first frame is vetted as usual.

**2c. The stop race in `carry_out_verb` (fix 7).**

- **What goes wrong:** a `stop()` that lands between the `stop_count` check
  and `set_wheel_velocity()` gets overwritten, and the `finally` then skips
  zeroing (`robot/interface.py:248-268`). `/stop` does not take
  `motion_lock`. The window is small, and the watchdog ends it about 1 s
  later.
- **Fix:** re-check `stop_count` after the re-command and zero the wheels
  if it changed.

**Done when** a test that injects a stop exactly in that window (a hook or
a fake body) shows the wheels at zero within one control period.

**2d. One odometry heading convention (fix 11).**

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

**2e. `RosDriveRobot` is not in the conformance suite (fix 6 in §5).**
Either add it to `BACKENDS` in `tests/test_robot_contract.py` (with a fake
bridge, as `tests/test_ros_drive.py` does), or change body ARCH to say
"every backend and the mission gate".

## 3. Decisions to make, then build

**3a. People other than the D-pad under `drive: ros` (V10, fix 10).**

- **Today:** only `twin-dpad` has a `twist_mux` input
  (`service/slam/src/picar_bridge/picar_bridge/bridge.py:79-83`).
  `teleop-operator` and unnamed callers, who rank as manual, are refused
  `ros_unavailable`.
- **Options:** give every manual-rank driver an input, or accept that only
  the D-pad drives under ROS and say so as a decision.
- Owner: the safety domain (driver order), with ros.

**3b. Twin tap-to-goal is ranked as autonomous.**

- **Today:** the page posts `/world/goal` with no `x-driver`, so the server
  arbitrates the goal as `ros` and a person's tap is refused while a
  mission holds the robot.
- **Decide:** should a person's tap outrank the brain, as the D-pad does?
- Specs: twin and safety, open questions.

**3c. Deployed replay (fix 12). Verify against the live stack first.**

- **Probable state:** as templated, the walks Lambda has neither
  `VISION_URL` nor `VISION_SHARED_SECRET`
  (`cloudformation/serverless.yaml:203-210`). Replay would answer 503, and
  with only a URL added, every frame would get a 401.
- **Check:** `aws lambda get-function-configuration` on the walks function.
- **Then:** either template both values or decide replay stays local-only.
- Specs: recordings and operations.

## 4. Smaller code items

- **4a.** `tools/jetson/setup.sh` should assert the detector's device on
  `cuda`, not only CLIP's (`:73-74`). PLAN 3.33 criterion 2 says to check
  it by hand until then. **Do this before 3.33 step 3 if you can**: it is a
  one-liner, now that `bench_perception.py` reads the predictor's device
  (`87c92f1`).
- **4b.** cloud-vision: coerce `target_direction` to its vocabulary and
  type-check `target_visible` in `service/vision_analyze/vision_core.py`.
  Today the string `"false"` is truthy and can pass the
  reached-implies-visible guard. Or decide every caller coerces.
- **4c.** Metrics rows show up as zero-frame walks. Filter the metrics
  prefix in `control/walk_store.py` `list_walks()`. Operations owns the
  storage layout and recordings owns the filter.
- **4d.** `world/ros_world.py` goal routes return 500 when the bridge is
  down (no `except` around httpx). Refuse them by name instead.
- **4e.** `control/health.py`: its verdict ignores `drive.ros_up`, so a
  dead ROS container doesn't show in the health check.
- **4f.** `config/robot.yaml`'s `sim_map:` key is read by nothing. Wire it
  or delete it.
- **4g.** Stale code prose to fix in one pass:
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

## 5. To investigate

**5a. `python -m tests.demo_active_search` ends NOT FOUND after 150
steps.**

- Of 132 `Blocked` lines, nearly all are LEFT turns clamped by the pivot
  guard (3.19), and the stuck-breaker never fires.
- It predates today: the result is the same with fix 3 reverted.
- First question: when did it stop finding the backpack? A `git bisect`
  over 3.19 to 3.32 should answer it.
- Recorded in `docs/engineering/policy/ENGINEERING.md`, Known gaps.

**5b.** `tests/demo_hold_bearing_ab.py` still builds the starter house,
which the Rover's chassis clips (3.32). Move it to the scaled house and
re-measure R1's A/B there. Policy ENG holds the old numbers as
starter-house history only.

**5c.** `tests/test_settle_pass.py::test_3_a_forward_is_a_cell_on_either_firmware[stock]`
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
