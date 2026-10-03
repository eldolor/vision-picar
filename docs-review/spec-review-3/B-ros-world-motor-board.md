# Spec review, third pass -- reviewer B: ros, world, motor-board

Scope: `docs/{ros,world,motor-board}/ARCHITECTURE.md` and their
`docs/engineering/.../ENGINEERING.md`, plus `service/slam/README.md` §4,
because ros ENG delegates every failure signature to it. Today's commits
(d549878, 6e4b11e, 50b2293) were read against `robot/server.py`,
`robot/ros_drive.py`, `world/ros_world.py` and
`service/slam/src/picar_bridge/picar_bridge/bridge.py`.

Commands run:
- `tools/spec_lint.py`: 0 errors, 0 warnings.
- `.venv/bin/pytest tests/test_wall_linters.py tests/test_stop_cancels_goal.py tests/test_ros_fallback.py tests/test_ros_goals.py -q`: 56 passed.
- `--collect-only` counts for 22 test files.
- Dry imports of `tests.test_wall_linters`: 11 duplicates, and 13 bridge routes enumerated.

Anything marked UNCONFIRMED was established by reading code only, with no
live ROS stack.

---

## (a) Findings in today's changes

### A1. HIGH: `service/slam/README.md` §4 still says a stop only pauses a goal and that the rule is undecided

`service/slam/README.md:255`:

> "The robot moves again a fraction of a second after Stop, during a nav2
> goal | Expected today: a stop pauses a goal and does not cancel it
> (`docs/ros/ARCHITECTURE.md` D6; the rule is undecided)..."

d549878 built the cancel (`robot/server.py:680-694`). ros ENG:16-18 and
:191-192 say this table is "the one table of failure signatures". The one
place an operator is sent therefore contradicts the code and ros ARCH D6
(`docs/ros/ARCHITECTURE.md:188-195`, "built 2026-10-02").

The symptom is still possible: see A2 and A3. Rewrite the row as follows.
"Moves again after Stop during a goal" now means one of three things:
- the cancel failed (look for the server log line `stop: could not cancel the nav2 goal`);
- the goal was still `pending` (A2);
- the bridge is down (A3).

### A2. MED: a stop (or a D-pad tap) during a `pending` goal does not cancel it. UNCONFIRMED

`bridge.py:303-311`, `cancel_goal()`:
- It cancels only if `self.goal_handle` is set.
- `goal_handle` is set only in `_on_goal_response()` (`:279-290`), after nav2 accepts the goal.
- While the goal is `pending`, the method sets `cancel_reason`, returns `False`, and cancels nothing.

When nav2's acceptance then arrives, `_on_goal_response()` runs:
- Its only guard is `record is not self.goal` (supersession).
- `cancel_goal()` left `self.goal` in place.
- So the goal goes `active` and nav2 drives.

The bridge then answers `GET /goal` with `state: active` and a
`cancel_reason`. `goal_in_progress()` (`robot/server.py:378-394`) keeps
refusing the brain `preempted`.

Specs this makes false, in the pending window:
- ros ARCH:188-191 and :311 ("A nav2 goal is ended too ... Met");
- world ARCH:220-222 ("A stop zeroes the wheels and then cancels the goal, so the planner does not resume");
- ros ARCH:315 (a tap cancels within 1 s);
- the safety ARCH failure row "A stop while a navigation goal is active ... Met".

Why the tests miss it: the in-process fake bridge
(`tests/test_ros_goals.py:41-47`) jumps straight to `active` and always
answers `{"cancelled": true}`. `tests/test_stop_cancels_goal.py` therefore
cannot see the window.

The window is short: nav2 normally accepts in milliseconds. But
`send_goal()` returns 200 `pending` before acceptance, so a tap-to-goal
followed at once by Stop hits it.

Fix (code), either:
- in `_on_goal_response`, cancel if `record.get("cancel_reason")` is set; or
- clear `self.goal` on a pending cancel.

Add a fake-bridge mode that holds `pending`.

### A3. MED: "a hung bridge delays only the cancel" is false past the client timeout, and a dead bridge never ends a goal. UNCONFIRMED

The stop's cancel is one call:
- `RosWorld.cancel_goal()` is `self._http.delete("/goal").json()` (`world/ros_world.py:240-241`), on a 2.0 s client timeout.
- `_cancel_goal_quietly()` (`robot/server.py:688-694`) logs any failure and never retries.
- Its docstring justifies that with "an unreachable bridge has no goal to resume". That is wrong: nav2's goal lives in nav2, not in the bridge's HTTP thread.

So a bridge that hangs for longer than 2 s, or has died, leaves the goal
held, and nav2 drives again when `STOP_HOLD_S` ends. That contradicts
ros ARCH:311 ("A nav2 goal is ended too ... so a hung bridge delays only
the cancel. Met (D6...)").

A3 compounds with 6e4b11e. With the bridge dead and the plugin alive:
- `/wheels` from `ros` is still accepted (`robot/server.py:614-626`, which never consults `ros_up()`);
- so nav2's output still reaches the wheels;
- meanwhile `ros_up()` is false and a person's verbs run `direct-fallback` on `robot.inner` (`:543-565`).

The result is two writers to the wheels, against ros ARCH D6 ("only ROS's
actuator may write the wheels"). It also makes ros ARCH:307's "so the row
above applies" untrue: the row above (container dies) assumes the
actuator's posts have stopped.

`safety.py` still vets both writers, so this is an arbitration defect and
not a collision defect. Whether nav2 keeps driving with the bridge's
`/scan` gone is UNCONFIRMED.

Fix, by severity:
- **Spec:** state the limit in ros ARCH:307 and :311 (the goal survives a stop when the bridge is unreachable), and add a world ARCH failure row.
- **Code:** while `bridge_up()` is false, either refuse `ros` `/wheels` non-zero posts or zero them. Retry the cancel until it succeeds or a new goal is set.

### A4. MED: 6e4b11e makes the "unknown driver" known gap worse, and the specs still describe the old behaviour

`RosDriveRobot._send()` (`robot/ros_drive.py:165-176`) marks the bridge
down on any `httpx.HTTPError`, and `raise_for_status()` is inside the
`try`. So the bridge's 400 `unknown driver` marks ROS down. That 400 is
what every unnamed, `teleop` or `teleop-operator` verb gets (`bridge.py:443-446`).

What follows, by reading:
1. The next manual-rank verb runs `direct-fallback` while ROS is actually up, until a probe (started by any `ros_up()` call, such as the twin's `/health` poll) marks the bridge up again.
2. Under an active nav2 goal, that is a second writer next to nav2 (as in A3).
3. The same driver's verbs alternate between refused and executed, depending on timing.
4. A wrong `APP_SHARED_SECRET` gives the same flapping. Sends get 401 and mark ROS down. The unauthenticated `/health` probe (`bridge.py:500-501`) marks it up again.

Specs now incomplete or wrong:
- ros ENG:140 ("which surfaces as `ros_unavailable`");
- ros ENG Known gaps :278-284;
- safety ENG:301-303 ("are refused `ros_unavailable`");
- `service/slam/README.md:256`.

Code fix: mark down only on transport errors (`httpx.TransportError`) and 5xx, not on 4xx.

### A5. LOW-MED: `/health` cannot tell a dead bridge from a silent plugin

`drive.ros_up` (`robot/server.py:995-1000`) is now an AND of plugin
freshness and `bridge_up()`. Neither part is published separately.

`service/slam/README.md:254` tells the operator that `drive.ros_up`
false with the container up means the plugin's posts have stopped, which
means a deactivated plugin and a container restart. Since 6e4b11e the
same readout can mean "the bridge failed one send", and a restart is the
wrong remedy for that.

Fix:
- publish `drive.bridge_up` and `drive.last_ros_post_age_s`;
- amend README:254;
- say in ros ENG:63 that the probe runs only when `ros_up()` is asked (lazy).

### A6. LOW: line references that today's commits moved

- world ENG:237 `robot/server.py:837-857` (the goal routes) is now `:850-883`.
- world ENG:240 `goal_in_progress()` at `:370-385` is now `:378-394`.
- ros ENG:282 `robot/server.py:571-575` is now `:578-583`.

### A7. LOW: the new stop-cancel thread has no "one at a time" guard

Each `/stop` starts a daemon thread (`robot/server.py:682-684`), and on a
hung bridge each one blocks for up to 2 s. `RosDriveRobot.stop()` got
exactly that guard (`_zeroing` lock, pinned in `tests/test_ros_drive.py`),
and the cancel did not. This is bounded by the timeout, so it is low.

### A8. LOW: `cancel_goal()` ignores the status code

`RosWorld.cancel_goal()` returns `.json()` whatever the status. A 401
(secret mismatch) comes back as `{"error": ...}`:
- `DELETE /world/goal` answers 200 with an error body;
- the stop's cancel "succeeds" and logs nothing.

world ENG:71 says the route answers "the bridge's `{cancelled: bool}`".

### Checked and correct in today's changes

- Bridge `DELETE /goal` exists, is authenticated (`bridge.py:547-552`), and calls `cancel_goal_async()` on an accepted goal.
- `/health` is unauthenticated on the real bridge (`:500-501`). ros ENG:70 is correct.
- `RosWorld.cancel_goal()` with no session sends the DELETE anyway. That is harmless, because the bridge answers `cancelled: false`.
- `hasattr(world_model, "cancel_goal")` is true only for `RosWorld`. `NullWorld` and `MockWorld` stop without a thread (`test_a_world_that_cannot_plan_still_stops`).
- `BRIDGE_PROBE_S` and `BRIDGE_PROBE_TIMEOUT_S` are 0.5 and 0.5 (`robot/ros_drive.py:111-112`).
- `ros_up()` ANDs `bridge_up()` (`robot/server.py:319-331`).
- `tests/test_ros_fallback.py` has 8 tests and `tests/test_ros_drive.py` has 14, as ros ENG says.
- `tests/test_wall_linters.py` still describes reality:
  - 11 duplicates, matching `MAX_DUPLICATES` 11;
  - 13 bridge routes, matching `MAX_BRIDGE_ROUTES` 13, and the route table at ros ENG:74-87 lists exactly those 13;
  - no new wall duplicate was introduced: `BRIDGE_PROBE_*` is one-sided.
- 50b2293 (cloud arrival confirmation) touches no sentence in ros, world or motor-board. Its only nearby dependency, the lidar-read range in `brain/arrival.py`, is described in policy, not here.

---

## (b) HOW in the architecture specs

None of today's sentences adds HOW. "Off the stop's own path" and "never makes
the stop wait" are WHAT. Remaining items, not reported before:

- **world ARCH:46.** The diagram says the server "derives only the sim-only error readout". Since 3.23 and 1b it also derives goal-in-progress for arbitration and issues the stop's cancel. This is a contract inaccuracy, not HOW. Reword to "pass-through, plus the error readout; goals: see D9".
- **motor-board ARCH:141.** "A whole centimetre is 3.3 degrees of heading on this track" is a derived number used as a reason. It is acceptable: it is the why behind rejecting odometers alone. No change.
- **ros ARCH:160.** "about 40-150 ms of jitter (3.13)" is a measured cost of a decision. Acceptable. Note that it is also the trade-off text for the one-extra-hop decision in motor-board D1 (ARCH:100-101), which states it without a number. Fine.

## (c) Restatement in the engineering specs

- **ros ENG:141** re-describes the stop's cancel mechanism ("on a background thread after `robot.stop()`") that safety ENG:121-125 specifies, and links to it. Trim it to the link plus "`DELETE /goal` on the bridge", which is the ROS-side fact. Low.
- **ros ENG:157 and safety ENG:156** carry the identical `BRIDGE_PROBE_S` / `BRIDGE_PROBE_TIMEOUT_S` row, word for word. Pick one: safety is the declared owner in 6e4b11e's message. ros should link to it, as it already does for `STOP_HOLD_S`. Low.
- **ros ENG:63** also re-describes the probe that safety ENG specifies. Acceptable as a one-clause summary with a link.

## (d) Verification failures (spec against code)

| # | Spec | Code | Defect |
|---|---|---|---|
| V1 | `service/slam/README.md:255` | `robot/server.py:680-694` | Says a stop does not cancel a goal, and that the rule is undecided (A1) |
| V2 | ros ARCH:311; world ARCH:220-222 | `bridge.py:279-311`; `world/ros_world.py:240-241`; `robot/server.py:688-694` | "Ended too", "Met". Not met in the pending window, nor past the 2 s timeout or with a dead bridge (A2, A3). UNCONFIRMED |
| V3 | ros ARCH:307 ("the row above applies") | `robot/server.py:614-626` | The actuator still writes the wheels when only the bridge is dead (A3). UNCONFIRMED |
| V4 | ros ENG:140, :278-284; safety ENG:301-303; README:256 | `robot/ros_drive.py:165-176` | Unknown driver: the verb is now not only refused but also marks ROS down (A4) |
| V5 | README:254 | `robot/server.py:319-331` | `ros_up` false no longer implies the plugin went silent (A5) |
| V6 | world ENG:237, :240; ros ENG:282 | see A6 | Stale line numbers |
| V7 | world ENG:71 | `world/ros_world.py:240-241` | Non-200 bridge replies pass through as 200 (A8) |

No other mismatches were found in the motor-board pair. See (g).

## (e) Pair, boundary, coverage and duplication

- **Pair (world).** world ARCH D9 says the stop's cancel is built. world ENG has no line saying where it lives or which test pins it: its Known-gaps entry was deleted, and nothing replaced it. Add one sentence ("`POST /stop` calls `cancel_goal()`; see safety ENG") and `tests/test_stop_cancels_goal.py` to its checklist (:215-217). Low.
- **Pair (ros).** ros ENG's change checklist (:235-237) does not run `tests/test_stop_cancels_goal.py` or `tests/test_ros_goals.py`. Both pin bridge-goal behaviour that a `bridge.py` change could break. Low.
- **Boundary.** The cancel is claimed consistently:
  - safety owns the rule;
  - ros D6 mirrors it;
  - world D9 says only that it is built.

  No conflict. The bridge-liveness rule is likewise owned by safety and mirrored in ros. No conflict.
- **Coverage.** No new unowned code. `BRIDGE_PROBE_*` and `bridge_up()` sit in `robot/ros_drive.py`, which is ros and body. The stop thread sits in `robot/server.py`, which is control-api and safety.
- **Duplication.**
  - the `BRIDGE_PROBE_*` row (c);
  - the unknown-driver known gap is stated in ros ENG:278-284, safety ENG:301-303 and README:256, and all three now need A4's correction. Make ros ENG canonical (it holds the code reference) and have the other two link.

## (f) Proposed fixes

| # | Fix | Severity | Effort |
|---|---|---|---|
| F1 | Rewrite `service/slam/README.md:255` (A1) and :254 and :256 (A4, A5) | high | S |
| F2 | Bridge: honour a cancel that arrives while the goal is `pending` (cancel on acceptance if `cancel_reason` is set). Fake bridge: add a pending phase. Test: a stop between `POST /world/goal` and acceptance (A2) | med | S-M |
| F3 | Spec: ros ARCH:307, :311 and world ARCH D9 and its failure table must state that a stop cannot end a goal while the bridge is unreachable. Code: retry the cancel; while `bridge_up()` is false, refuse non-zero `ros` `/wheels` (A3) | med | S (spec) / M (code) |
| F4 | `RosDriveRobot._send()`: mark down only on transport errors and 5xx, not 4xx. Then correct ros ENG:140 and :278-284 and safety ENG:301-303 (A4) | med | S |
| F5 | Publish `drive.bridge_up` (and the plugin's post age) on `/health`. Note that the probe is lazy in ros ENG:63 (A5) | low-med | S |
| F6 | Update three stale line refs (A6) | low | S |
| F7 | `RosWorld.cancel_goal()` (and `get_goal`): check status, so a 401 or 5xx is not a 200 (A8; joins handoff 4d) | low | S |
| F8 | One-at-a-time guard on the stop-cancel thread (A7) | low | S |
| F9 | De-duplicate the `BRIDGE_PROBE_*` row and the stop-cancel mechanism in ros ENG. Add stop-cancel pointers and tests to world ENG and the ros ENG checklist (c, e) | low | S |
| F10 | world ARCH:46 diagram caption (b) | low | S |

---

## (g) Checked and found correct (about 130 claims)

**ros ENG, constants and configuration:**
- yaml values match the files:
  - `cmd_vel_timeout` 0.25, `update_rate` 20, wheel radius 0.040 / separation 0.172, max velocities 0.6 and 6.0;
  - twist_mux timeouts 0.25 and priorities 100/50/50;
  - slam `resolution` 0.05, `max_laser_range` 12.0, `restamp_tf`, `transform_publish_period` 0.05, `map_update_interval` 1.0;
  - nav2: footprint ±0.1265 x ±0.1155, inflation 0.12 / 8.0, RPP 0.20 / 0.30, goal tolerances 0.10 / 6.28, NavFn `allow_unknown` / 0.10;
  - collision_monitor `approach`, 1.0 s, `slowdown_ratio` 0.5, and no stop polygon.
- Bridge settings:
  - `SCAN_HZ`, `PAN_HZ`, `BRAIN_HZ` are 10, 20, 2;
  - KeepAliveClient timeouts are 0.5 and 1.0;
  - foxglove listens on 0.0.0.0:8765 with `connectionGraph` only.
- `kTimeoutMs` is 40.
- All `robot/ros_drive.py` constants in the table match, as do `ROS_SILENCE_S` 0.5 and `WHEEL_LOOP_INTERVAL_S` 0.05.
- The factory bridge-URL rules (`robot/factory.py:40-41`, `world/factory.py:105-106`) and the `RosDriveRobot` signature match.

**ros ENG, routes, drivers and tests:**
- The route table has all 13 routes and the auth rule.
- `DRIVER_TOPICS` matches, and the twin-dpad cancel is at `bridge.py:453-455`.
- Test counts match for 14 files: containment 3, wall linters 18, urdf 20, cad 13, ros_drive 14, verb_safety 7, fallback 8, convert / keepalive / brain_view 16/7/19, chain_live 13, nav_live 5, slam_live 3, brain_view_live 5, http_rate_live 2.

**ros ENG, known gaps:**
- These line refs hold: `picar_sim_hardware.cpp:90-94` and `:157-160`, `hardware_robot.py:349`, `bridge.py:79-83`.
- The stale header comment exists (`picar_sim_hardware.hpp:8-9`).

**world ENG:**
- `OCCUPIED_AT` 65 and `FREE_BELOW` 25.
- `CELL_M` 0.30, `DEFAULT_RAYS` 360, `DEFAULT_RANGE_CELLS` = `FPV_MAX_DIST`.
- `DEFAULT_TIMEOUT_S` 10.0.
- The `RemoteWorld` 404 rule and `WorldTransportError`.
- The `WORLD_MODE` / `none` default and the `ValueError` text.
- `map_id` values `sim-grid` and `slam-<session>`.
- `set_goal`, `get_goal`, `cancel_goal` and `get_odom_pose` exist only on `RosWorld`.
- Arbitration (`robot/server.py:333-375`) refuses autonomous drivers other than `ros` during a `pending` or `active` goal.
- The goal routes give 501 via `_goals()`.
- These stale items are still stale, as recorded:
  - `world/factory.py:14-22`;
  - `config/robot.yaml:86` ("not built yet");
  - `sim/mock_world.py:161` ("quantised").
- Test counts: world_contract 57, ros_world 13, ros_goals 26, goal_arbitration 9.

**motor-board ENG:**
- Signatures match: `HardwareRobot(port, sensors=None, baud=115200)` and `FakeEsp32(...)`.
- Set-up `T:136` / `T:131` (`hardware_robot.py:156-157`) and `T:1` rounding to 5 places (`:334-335`).
- Constants match:
  - `WHEEL_RADIUS_M`, `TRACK_WIDTH_M`, `COUNTS_PER_REV`;
  - `REBOOT_JUMP_M`, `EXTRAPOLATE_MAX_S`, `HEARTBEAT_MS` 1500;
  - `MOVE_M`, `TENTH_MM`, `CM`, `MILLIS_WRAP`;
  - the fake's `BOARD_LOOP_S`, `NO_LOAD_WHEEL_M_S`, `DEFAULT_HEARTBEAT_MS` 3000, `DEFAULT_FEEDBACK_INTERVAL_MS` 50, `OUT_BUFFER_BYTES` 4096;
  - the `SETTLE_*` constants in `robot/safety.py:169-174`.
- `SIM_BOARD_FIRMWARE` is read in `sim/fake_esp32.py:93`.
- The factory's fake and serial paths and the error text match (`robot/factory.py:123-136`).
- The patch adds exactly 13 lines and removes none.
- `astern_not_observed` (`safety.py:629`) and "settle refused" (`:856`) exist.
- `sim_map` is on `/health` (`server.py:1024`).
- The ±2.0 guard line refs (`fake_esp32.py:254-257`, `:289-295`) and `hardware_robot.py:330-336` hold.
- Test counts: fake_esp32 11, ros_driver_board 24, firmware_fork 21, settle_pass 8.

**Architectures:**
- motor-board ARCH contracts and failure numbers agree with ENG's recorded table: 1.02 / 0.047 cm, 120/120 worst 0.84, 8/8 reboots.
- world ARCH's 3.14 bars agree with world ENG's recorded table.
- ros ARCH D9's two budgets agree with the linter.
