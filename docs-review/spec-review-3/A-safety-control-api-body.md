# Spec review, third pass -- reviewer A: safety, control-api, body

Scope: `docs/{safety,control-api,body}/ARCHITECTURE.md` and their
`docs/engineering/*/ENGINEERING.md`, against `dev` at `50b2293`. Linter: 0
errors, 0 warnings (re-run). Read-only; probes were in-process pytest files in
the scratchpad (`test_probe_goal.py`, `test_probe_window.py`,
`test_probe_driver.py`, `probe_400.py`), all against fakes, no ports.
`tests/test_stop_cancels_goal.py`, `test_ros_fallback.py`, `test_ros_drive.py`,
`test_arrival_confirmation.py`: 31 passed.

**Most important:** today's 1b makes every safety/body/control-api spec say a
stop now *ends* a nav2 goal and the wheels *stay* stopped ("Met"). Three things
the code does are not in any spec, and the first is a regression of 3.23: the
**loser** of a goal arbitration cancels the **winner's** goal through its
teardown stop (CONFIRMED in-process).

---

## (a) Findings in today's changes

### A1 (high). A mission refused *by* a nav2 goal cancels that goal on its way out -- CONFIRMED (in-process; runner leg by reading)

- 3.23 (safety ARCH:211-213): "While one [goal] is pending or active, every
  other autonomous command is refused". `arbitrate()` does refuse the brain
  `preempted` (`robot/server.py:350-358`).
- The runner then ends the mission and stops the car:
  `control/mission_runner.py:468-477` (`_finish(PREEMPTED)`) -> `_safe_stop()`
  (`:790-796`) -> `RemoteRobot.stop()` (`control/remote_robot.py:106-110`) ->
  `POST /stop`, which since d549878 starts `_cancel_goal_quietly`
  (`robot/server.py:681-693`) for any world with `cancel_goal`.
- Probe (`test_probe_goal.py`): goal accepted; brain `FORWARD` -> `preempted`;
  brain `/stop` -> `bridge.goal` is `None`, `cancelled: 1`.
- The runner's comment (`mission_runner.py:472-476`) reasons that a loser's
  stop is harmless "the server refuses nobody's stop". That was true while a
  stop only paused a goal; it is not now. It is the exact failure safety
  ARCH rule 2 (L189-190) exists to prevent: "Otherwise the loser of an
  arbitration takes the robot back by giving up."
- Not stated anywhere in the specs. Safety ARCH L186-209 and ENG L120-125
  read as if only a person's stop reaches `/stop`.
- Decision needed (safety owns it): e.g. cancel only for a stop from a
  driver that outranks or equals `ros` and is not the one a goal preempted,
  or have `_finish(PREEMPTED)` skip the stop (the robot is already the other
  driver's), or make the cancel conditional on the stopping driver.

### A2 (high). "The wheels stop and the goal is ended" is not guaranteed: a window before the cancel lands, and nothing at all if it fails -- CONFIRMED with a fake; real latency unmeasured

- Under `drive: direct` + `WORLD_MODE=ros` a stop has no hold
  (`RosDriveRobot` is not the body) and claims nothing, so nav2's next
  non-zero `/wheels` post as `ros` (every 50 ms) is arbitrated and allowed
  (`robot/server.py:602-660`). The wheels run again until the background
  `DELETE /goal` lands and nav2's output reaches zero. Probe
  (`test_probe_window.py`, cancel delayed 0.3 s): 50 ms after `/stop` the
  wheels read `(3.0, 3.0)`.
- If the cancel fails (bridge HTTP unreachable or slow past `RosWorld`'s
  2.0 s client timeout, `world/ros_world.py:55-58`), `_cancel_goal_quietly`
  logs and returns (`robot/server.py:688-693`); nav2 still holds its goal and
  drives on. Probe: cancel raises -> 0.7 s after `/stop` the wheels read
  `(3.0, 3.0)`, goal `active`. Under `drive: ros` the same happens after
  `STOP_HOLD_S` (0.6 s), and `/wheels` from `ros` is accepted even while
  `ros_up()` is false (`robot/server.py:615-629`), so 1d's "ROS down" does
  not stop it either.
- The code's justification is false: "an unreachable bridge has no goal to
  resume" (`robot/server.py:689-690`). The bridge's HTTP side is not nav2;
  bt_navigator keeps an accepted goal whether or not its client answers
  (UNCONFIRMED for a dead bridge node: by reading ROS 2 action semantics).
- The test that closed 1b (`tests/test_stop_cancels_goal.py:47-58`) ticks
  the fake nav2 only *after* `STOP_HOLD_S + 0.1` s, by which time the fast
  fake cancel has landed, so it cannot see either case; `test_a_stop_never_waits_on_the_bridge`
  measures only the stop's latency.
- Spec text that over-promises: safety ARCH L187-188, L200-203, L299 ("Met");
  safety ENG L123-124 ("A failure is logged and swallowed: the robot is
  already stopped"); body ARCH L212 ("Met"); control-api ENG L54.
- Fix options: a server-side hold that zeroes non-zero `ros` posts until the
  cancel is acknowledged (in both drives), and on cancel failure keep
  refusing `ros` motion (or mark ROS down and refuse `ros` `/wheels`) until a
  cancel succeeds. At minimum, record both windows in safety ENG Known gaps
  and downgrade the "Met" rows to "met when the cancel succeeds".

### A3 (med). A STOP *verb* is arbitrated and does not cancel a goal -- CONFIRMED

- Safety ARCH L186: "Stop is never arbitrated. Anyone may stop the robot at
  any time." control-api ARCH L59: arbitration is skipped by "a stop".
- `POST /action {"action": "STOP"}` goes through `arbitrate()` first
  (`robot/server.py:538-540`). Probe: brain `/action STOP` during a goal ->
  `{"executed": false, "reason": "preempted"}`; a `twin-dpad` `/action STOP`
  executes but leaves the goal `active`, `cancelled: 0`.
- The twin (`web-twin/app.js:477`) and `RemoteRobot.stop()` both use
  `/stop`, so no shipped client hits this today; curl and any future client
  would. Safety ENG L35 lists `/action` STOP as `_dispatch()`, check "None",
  which is about the vet only.
- Fix: say "the stop route" in both ARCH rules and add a row to safety ENG's
  motion table ("`/action` STOP: arbitrated like any verb; does not cancel a
  goal; use `/stop`"), or route the STOP verb to the stop handler.

### A4 (low-med). 1d marks ROS down on any `httpx.HTTPError`, including an HTTP error *status* from a live bridge -- CONFIRMED at the wrapper; server impact race-dependent

- `RosDriveRobot._send()` (`robot/ros_drive.py:165-175`) catches
  `httpx.HTTPError`, which includes `HTTPStatusError` from
  `raise_for_status()`. The bridge answers **400** for an unknown driver
  (`service/slam/src/picar_bridge/picar_bridge/bridge.py:443-445`), which is
  exactly the open V10 case (unnamed, `teleop`, `teleop-operator`).
  `probe_400.py`: a 400 -> `bridge_up()` false.
- Through the real app (`test_probe_driver.py`) the mark did not persist: the
  refusal's `robot.stop()` posts zeros that succeed and clear it. But the
  zeroing is single-flight (`ros_drive.py:323-325`); if one is already
  running, no fresh zero is sent, and the first `bridge_up()` call after a
  mark always answers false (`ros_drive.py:184-197`), so the next autonomous
  verb can be refused `ros_unavailable` and end its mission `failed`.
  (UNCONFIRMED for the race; by reading.)
- Liveness is about reachability; a 4xx is proof of life. Fix: mark down on
  `httpx.TransportError` (connect, timeout) only. Specs say "a failed send"
  without saying which failures count (safety ARCH L246, ENG L156, ros ENG
  L63).

### A5 (low). Each `/stop` spawns an unbounded cancel thread

`robot/server.py:682-684` starts a new `stop-cancel-goal` thread on every
`/stop` whenever the world has `cancel_goal` (always under `WORLD_MODE=ros`,
goal or not), each a `DELETE /goal` with a 2.0 s timeout. `RosDriveRobot`
made its own zeroing single-flight for exactly this reason (body ENG L41-46).
Not stated; low because stops are rare compared with watchdog stops, which
do not cancel.

### A6 (low). Safety ARCH's rejected alternative now reads against the code

Safety ARCH L240-243 rejects "asking the container whether it is healthy",
but 1d's recovery does exactly that: `_probe()` GETs the bridge's `/health`
(`ros_drive.py:199-208`). Not a contradiction (the probe only clears a
bridge-failure mark; the actuator's posts are still required), but one
clause saying so would stop the next reader flagging it.

### A7 (low). Line citations shifted by today's code

`robot/server.py` gained 8 lines in `ros_up()` and 18 in `stop()`:

- `docs/engineering/ros/ENGINEERING.md:282` cites `robot/server.py:571-575`
  for the `ros_unavailable` mapping; it is now `:579-583`.
- `docs/engineering/world/ENGINEERING.md:237` cites `:837-857` for the goal
  routes; now `:863-883`. `:240` cites `:370-385` for `goal_in_progress()`;
  now `:378-393`.
(Not my domains; reported so the ros/world reviewers need not re-find them.)
All citations inside safety/body/control-api still hold (see (g)).

### 50b2293 (arrival confirmation)

Touches no sentence in my three domains (grep for arrival/confirm/found: none
relevant). Checked only for safety effects: a refused arrival leaves the
robot within 0.40 m and the next FORWARD is vetted at 20 cm as usual; a
confirmation timeout raises `VisionUnavailable` from inside `agent.step()`
and lands in `tick()`'s B3.2 handler (`mission_runner.py:466-467`); success
resets the budget at `:497`. Nothing for safety to record.

## (b) HOW in architecture specs

Prior passes cleaned these specs well; only small items remain.

- **body ARCH L212** ("Stop while a nav2 goal is active", column "What the
  body does"): the body does not do this; body ENG L64-68 says so ("The
  body's stop does not cancel a nav2 goal; the server's does"). It is a
  boundary misplacement as much as HOW: replace with a pointer to safety's
  row, or reword the response cell as "Not the body's: the robot server's
  stop ends the goal".
- **safety ARCH L54, body ARCH L62/L69** use `robot/safety.py`,
  `robot/interface.py`, `robot/factory.py` in the "Part" column. They are
  pointers beside a role name, so they survive a rename. Legitimate; no
  change.
- **safety ARCH L200-203**: "the cancel runs after the wheels are zeroed and
  off the stop's own path" is the decision (a stop never waits on ROS), not
  mechanism. Keep; "on a background thread" correctly stays in ENG.
- Numbers in ARCH failure tables (2 cm / 18.0 cm / 1.0 cm, 3 s / 2 s, 95%)
  are acceptance bars, labelled as such at safety ARCH L283-284. Keep.

## (c) Restatement in engineering specs

- **safety ENG L120-125** ("Stop ends a nav2 goal") is mostly HOW, but its
  last two sentences restate the decision's reasoning ("The thread is why a
  hung bridge costs the stop nothing") and omit the HOW that matters: which
  stops reach it (`/stop` only; not the STOP verb, not the watchdog -- the
  watchdog half is there), that it fires on every `/stop` with or without a
  goal, the 2.0 s client timeout, and what happens on failure (A2).
- **safety ENG has no prose for 1d at all.** Its only trace is the
  parameter row L156 and the test row L239; the mechanism (`ros_up()` =
  fresh actuator post AND `robot.bridge_up()`; `_send()` marks down; probe
  on the first `bridge_up()` call, at most every 0.5 s; the first call after
  a mark always answers false) is written only in ros ENG L63. The commit
  calls safety the owner. Add it under Interfaces beside the watchdog
  paragraph, and have ros ENG link.

## (d) Verification failures (claim vs code)

| # | Spec claim | Code | Sev |
|---|---|---|---|
| D1 | safety ARCH L186 "Stop is never arbitrated"; control-api ARCH L59 "a stop" skips arbitration | `/action` STOP is arbitrated, `robot/server.py:538-540` (A3) | med |
| D2 | safety ARCH L299, body ARCH L212: stop with a goal -> "wheels stop and the goal is ended", **Met** | Window before the cancel lands; nothing on cancel failure, `robot/server.py:602-660, 688-693` (A2) | high |
| D3 | safety ENG L123-124 "A failure is logged and swallowed: the robot is already stopped" | Stopped only until nav2's next post (direct) or the hold's end (ros) | high |
| D4 | safety ARCH L189-190 rule 2's guarantee (the loser cannot take the robot back by giving up) | A preempted mission's teardown `/stop` cancels the goal that preempted it (A1) | high |
| D5 | safety ENG L211 "`ros_unavailable`: the container is down. A person can still drive." | Also any failed `/cmd_vel` send, including a 4xx from a live bridge (`ros_drive.py:172`); and for the verb that found it, a person's too (ENG L280-284 says so; the procedure line does not) | low |
| D6 | control-api ENG L215-216 Known gap: CORS omits DELETE, so a browser "cannot cancel a goal" | Still true of `DELETE /world/goal`, but since d549878 a browser cancels any goal with `POST /stop`. Gap is now cosmetic; say so | low |
| D7 | body ENG L28 / L32-47: `RosDriveRobot` row and "The ROS drive stop" describe the wrapper's full behaviour | `bridge_up()`, `_mark_bridge_down()`, `_probe()` and the down-mark side effect of every `_send()` (including step 3's zero posts, which now set/clear it) are absent from the domain that owns `robot/ros_drive.py` | med |
| D8 | control-api ENG L24: `RosDriveRobot` row lists what the server uses from the wrapper | Omits `bridge_up()`, which `ros_up()` now calls by duck typing (`robot/server.py:330-331`) | low |

## (e) Pair, boundary, coverage, duplication

- **Duplication:** the `BRIDGE_PROBE_S` / `BRIDGE_PROBE_TIMEOUT_S` row is
  maintained word for word in safety ENG L156 and ros ENG L157. The
  constants live in `robot/ros_drive.py`, whose other constants
  (`STOP_HOLD_S`, `STOP_ZERO_TIMEOUT_S`, client timeout) are canonical in
  **body ENG** L160-162. Make body ENG canonical (it owns the module); safety
  ENG and ros ENG link. Same pattern the second pass applied to chassis
  constants.
- **Boundary:** "a stop ends a goal" is a safety rule executed by the
  robot server against the world model. Safety owns the rule (agreed by all
  specs); body ARCH L212 claims the response in its own failure table
  (see (b)). World and ros link correctly.
- **Pair (safety):** ARCH decision "When ROS dies" (L238-261) now has an
  implementation with no ENG prose (see (c)). ARCH "Who drives" rule 1
  carries the 1b addition; ENG has it but not its limits (A2, A3, A1).
- **Pair (control-api):** ARCH failure row L203 (hung bridge when a stop
  arrives) is about zeroing; with 1b the stop also starts a cancel against
  that hung bridge. Worth one clause ("and the goal cancel, likewise off the
  stop's path").
- **Coverage:** `tests/test_stop_cancels_goal.py` is listed in safety ENG
  L238 but not in control-api ENG Verification (L189-202), though it is a
  route test through a real app like its neighbours on L193. Add it there.
- **Pre-existing, not reported before (low):** `goal_in_progress()` runs a
  synchronous `GET /goal` on every autonomous `/action` and `/world/goal`
  (`robot/server.py:378-393`); on a *hung* (not refusing) bridge each costs
  up to `RosWorld`'s 2.0 s timeout before failing open. control-api ARCH
  L204 ("Treated as 'no goal'") is true but silent on the cost. UNCONFIRMED
  (read).

## (f) Proposed fixes

| # | Fix | Sev | Effort |
|---|---|---|---|
| 1 | Decide and build: a preempted mission's teardown stop must not cancel the goal that preempted it (A1). Record the rule in safety ARCH "Who drives"; test: goal active, brain refused `preempted`, runner finishes, goal still `active` | high | M |
| 2 | Close the stop-to-cancel window and the failed-cancel case (A2): hold non-zero `ros` `/wheels` at zero after a `/stop` until the cancel is acknowledged, in both drives; on failure keep holding (or refuse `ros` posts) until a cancel succeeds. Extend `test_stop_cancels_goal.py` with a nav2 tick immediately after `/stop` and one with a failing cancel | high | M |
| 3 | Until 2 lands: change "Met" to "met when the cancel succeeds" in safety ARCH L299 and body ARCH L212; replace safety ENG L123-124's "the robot is already stopped" with the two windows; delete the false comment at `robot/server.py:689-690` | high | S |
| 4 | Say "the stop route" in safety ARCH L186 and control-api ARCH L59; add `/action` STOP (arbitrated, no cancel) to safety ENG L35 (A3) | med | S |
| 5 | Write 1d's mechanism in safety ENG Interfaces; move the `BRIDGE_PROBE_*` row to body ENG constants and link from safety/ros; add `bridge_up()` to body ENG's `RosDriveRobot` row and "The ROS drive stop" step 3 (zero posts set/clear the mark) | med | S |
| 6 | Mark ROS down only on `httpx.TransportError` (A4); state in safety ENG which failures count | low-med | S |
| 7 | Make the stop's cancel single-flight, like the zeroing (A5) | low | S |
| 8 | Update ros ENG L282 and world ENG L237/L240 line citations (A7) | low | S |
| 9 | Add `tests/test_stop_cancels_goal.py` (4) to control-api ENG Verification; amend control-api ENG L215-216 (DELETE gap now cosmetic) and safety ENG L211 (`ros_unavailable` meaning) | low | S |
| 10 | One clause in safety ARCH L240-243 that the bridge `/health` probe only clears a bridge-failure mark (A6); one clause in control-api ARCH L203 that the goal cancel is also off the stop's path | low | S |

## (g) Checked and found correct (about 130 claims)

- **Today's spec edits, true against code:** `/stop` order (stop, then
  daemon thread `stop-cancel-goal`, `robot/server.py:680-693`); only `/stop`
  cancels, the watchdog calls `robot.stop()` directly (`:298`); `RosWorld`
  cancel is `DELETE /goal` (`world/ros_world.py:240-241`); `ros_up()` ANDs
  `bridge_up()` (`server.py:319-331`); `_send()` marks down, success clears
  (`ros_drive.py:165-175`); `bridge_up()` never blocks, single probe via
  non-blocking lock, at most every `BRIDGE_PROBE_S` (`:184-197`); probe
  timeout `BRIDGE_PROBE_TIMEOUT_S` 0.5 and 200 clears (`:199-208`); the verb
  that finds the bridge dead is refused `ros_unavailable` after a direct
  stop and not retried (`server.py:579-583`); dead mux/controller still
  fails toward stop; body ENG L64-68 (body stop does not cancel).
- **Test counts** (`pytest --collect-only`), all match the specs:
  test_stop_cancels_goal 4, test_ros_fallback 8, test_ros_drive 14,
  test_server 23, test_authority 13, test_goal_arbitration 9,
  test_wheels_command 8, test_watchdog_integration 6, test_health 15,
  test_health_sim_map 4, test_bridge_keepalive 7, test_blind_reverse 5,
  test_depth_veto 28, test_footprint_safety 10, test_pan_safety 6,
  test_pivot_safety 4, test_guarded_verbs 8, test_mission_guarded_verbs 3,
  test_ros_verb_safety 7, test_mover_safety 3, test_robot_contract 134,
  test_remote_robot 10, test_teleop_robot 8, test_config_and_factory 22.
- **Constants and config** (all equal to the specs): `WATCHDOG_POLL_INTERVAL_S`
  0.1, `WHEEL_LOOP_INTERVAL_S` 0.05, `ROS_SILENCE_S` 0.5, `VERB_PERIOD_S`
  0.05, `VERB_TURN_STEP_DEG` 5.0, `NO_SENSOR_CM` 999.0, `DEPTH_COLS_DEFAULT`
  8, `FOOTPRINT_SIDE_MARGIN_CM` 3.0, `PIVOT_MARGIN_CM` 1.3,
  `PIVOT_LOOKAHEAD_S` 0.05, `PIVOT_MIN_LOOKAHEAD_DEG` 1.0,
  `SAFETY_SCAN_RANGE_M` 0.6, `CHASSIS_WIDTH_CM` 16.5, `PATH_REFERENCE_CM`
  30.0, `PATH_FRACTION` 0.5, `VERB_MIN_MOVE_M` 0.01, `VERB_MIN_TURN_DEG` 0.5,
  `SETTLE_*` (0.5, 0.003, 0.1, 0.02, 0.15, 3), `STOP_ZERO_TIMEOUT_S` 0.5,
  `STOP_HOLD_S` 0.6, `TURN_RATE_RAD_S` 1.2, `HEARTBEAT_MS` 1500,
  `DEFAULT_TIMEOUT_S` 10.0, `DEFAULT_STALL_TIMEOUT_S` 15.0;
  `safety.min_distance_cm` 20, `watchdog_timeout_s` 1.0,
  `sensor_to_bumper_cm` 0.0, `brain.min_distance_cm` 20.0 in
  `config/robot.yaml`; `twist_mux` timeouts 0.25 and `cmd_vel_timeout` 0.25.
- **Line citations in my domains, still correct:** safety ENG
  `robot/safety.py:46-57` (stale FORWARD-only comment), `:665` (unusable
  wheels pass-through), `robot/hardware_robot.py:349`; body ENG
  `robot/interface.py:241-271` (stop race), `tests/test_robot_contract.py:85`
  (six BACKENDS), `sim/mock_robot.py:710`, `robot/hardware_robot.py:371`.
- **Driver ranks** (`robot/interface.py:44-61`) and arbitration order
  (`server.py:333-376`) match safety ENG L92-106; goal routes arbitrate as
  `ros` and do not claim (`server.py:863-875`); bridge driver table
  (`bridge.py:79-83`) and 400 on unknown driver match the V10 gap text;
  twin and `RemoteRobot` both stop via `/stop`.
- Linter: 0 errors, 0 warnings.
