---
kind: engineering
domain: safety
status: current
verified: 2026-10-02
parent: docs/safety/ARCHITECTURE.md
---

# Safety -- engineering

How the safety layer, driver arbitration and the watchdog are built today.
The rules and why they are shaped that way are in the
[architecture spec](../../safety/ARCHITECTURE.md). This document is true only
until the implementation changes, and is updated in the same commit that
changes it.

## Implementation

| Where | What |
|---|---|
| `robot/safety.py` | `SafetyController`: every clearance judgement, the wheel-command vet, the guarded-verb runner and the settle pass. `path_zone_indices()` and `scan_points_cm()` are module functions. `SafetyViolation` is the veto exception |
| `robot/interface.py` | The rank table `DRIVER_PRIORITY`, `driver_priority()` (unknown names rank as manual), `Preempted`, and `carry_out_verb()`, the loop `run_verb()` supplies a limit to |
| `robot/server.py`, inside `create_app()` | `authority_holder()`, `arbitrate()`, `refuse()`, `goal_in_progress()`, `ros_up()`, `watchdog_loop()`, `wheel_loop()`, and the `motion_lock` that serialises everything that moves the robot. Under ROS drive, a second `SafetyController` (`fallback_safety`) runs over the wrapped body |
| `control/remote_robot.py` | Maps refusal reasons to `Preempted` / `RobotTransportError` / `SafetyViolation` |
| `brain/agent.py` | The brain's own `SafetyController` (an early out; the server's is authoritative). Its `min_distance_cm` comes from the `brain:` block |
| `tests/footprint_sweep.py`, `tests/verb_sweep.py`, `tests/ros_verb_sweep.py` | Ground-truth instruments: the URDF rectangle against the house's occupied cells, sharing no geometry code with `robot/safety.py` |

### What each motion is vetted against

| Motion | Path | Check |
|---|---|---|
| `/action` FORWARD | `check_and_execute()` | Pre-check `forward_clearance() < min_distance_cm` raises. Then, if the body has a `verb_plan()`, `run_verb()` re-vets every 50 ms period with look-ahead |
| `/action` REVERSE | same | `reverse_clearance()`, then `run_verb()`. Refused outright (`astern_not_observed`) on a body with wheels and no usable scan |
| `/action` LEFT / RIGHT | same | No pre-check. `run_verb()` limits each step with `pivot_scale()` (only on bodies with a plan). With no usable scan, never limited |
| `/action` LOOK_* / STOP | `_dispatch()` | None |
| `POST /wheels` (direct drive) | `vet_wheel_velocity()` on receipt, then every `wheel_loop()` tick | Forward/reverse look-ahead and clamp; rotation scaled by `pivot_scale()`. Passed through unvetted while the body's wheel state is unusable (Known gaps) |
| ROS drive: verbs | `check_and_execute()` -> `RosDriveRobot` (no plan) | The pre-check, then the actuator's `/wheels` posts are vetted on receipt and every tick. `_refuse_if_nothing_achieved()` turns a verb that covered under 1 cm / 0.5 deg into a `SafetyViolation` |
| ROS drive, ROS down: a person's verbs | `fallback_safety.check_and_execute()` on the wrapped body | Exactly direct drive's path |

### Clearance sources

Each clearance method returns `(cm or None, source)`. `None` means nothing
within range, never "unknown".

| Method | Sources | Meaning |
|---|---|---|
| `path_clearance()` | `depth_grid_facing_away`, `depth_grid`, `depth_grid_no_target`, `distance_sensor` | The cone, tried in that order. A panned grid with no path zones left is no opinion (`None`); otherwise the nearest `ZONE_RANGE` in the path zones; otherwise, if any path zone answered, no target; otherwise (no grid, or every path zone unusable) the scalar `get_distance()`. Unusable zones are dropped. All values are bumper-relative (`sensor_to_bumper_cm` subtracted, floored at 0) |
| `footprint_clearance(+1 / -1)` | `scan_footprint`, `scan_footprint_no_target`, `no_scan` | Returns in the body frame within half-width + 3 cm of the centre-line and beyond the leading edge: distance past the edge. A return inside the outline reads 0.0 |
| `rear_clearance()` | `scan_rear`, `scan_rear_no_target`, `no_rear_sensor` | Beams within `PATH_HALF_ANGLE_DEG` of astern, minus `LIDAR_TO_REAR_BUMPER_CM` |
| `forward_clearance()` | the lesser of cone and corridor, or `path_not_observed` (0.0) | `path_not_observed` when the grid faces away and there is no scan |
| `reverse_clearance()` | `astern_not_observed` (0.0), or the lesser of rear and corridor astern, off one scan | `(0.0, "astern_not_observed")` when the scan is missing or unusable **and** `_has_wheels()` (the body's `get_wheel_state()` reports `usable: true`; a body without the method, or whose call raises, has none). Otherwise, with no scan, `(None, "no_rear_sensor")` and the reverse proceeds. The rule and its reasons: [architecture spec](../../safety/ARCHITECTURE.md), "A body that cannot see astern does not reverse" |
| `pivot_blocked(omega)` / `pivot_scale(omega)` | reason string or None / `(fraction, reason)` | A return whose distance to the rectangle would end under `PIVOT_MARGIN_CM` **and** shrink. Binary search to 1/64 of the turn rate. No usable scan: never blocked |

Comparison edges: the verb pre-check refuses at `< min_distance_cm`; the
wheel vet clamps at `<= min_distance_cm` and otherwise limits speed to
`(clearance - min) / 0.05 s`. `run_verb()`'s limit allows
`clearance - min` metres, so it ends `clamped` at the line.

### Driving without a lidar

The canonical statement of what a body that moves but has no scan may do;
other specs link here. Today that body is the car: `HardwareRobot` with no
`sensors` (no scan, the default all-unusable depth grid, `get_distance()`
0.0). Once the board's first feedback frame has arrived (`get_wheel_state()`
usable):

| Motion | Result | Why |
|---|---|---|
| FORWARD verb, or a forward component of `/wheels` | Refused / clamped, `safety_distance` (source `distance_sensor`) | No usable grid, so the cone falls back to the scalar, which reads 0.0 |
| REVERSE verb, a reverse component of `/wheels`, a settle pass astern | Refused / clamped, `astern_not_observed` | `reverse_clearance()` with `_has_wheels()` and no scan |
| LEFT / RIGHT verb, rotation in `/wheels`, a turning settle pass | Allowed, never limited | `pivot_blocked()` / `pivot_scale()` never refuse with no usable scan |

So on the car only turns move until its lidar driver lands, and a straight
verb that overshoots keeps its overshoot. **Before the first feedback
frame** the table does not hold: `vet_wheel_velocity()` passes a standing
command through unvetted and a REVERSE verb answers `no_rear_sensor` and
proceeds (Known gaps). The decision behind the reverse row:
[architecture spec](../../safety/ARCHITECTURE.md), "A body that cannot see
astern does not reverse".

## Interfaces

**`SafetyController(robot, min_distance_cm=20.0, sensor_to_bumper_cm=0.0)`**

| Method | Returns / raises |
|---|---|
| `check_and_execute(action, speed=, duration=, angle=)` | The body's result dict. Raises `SafetyViolation` on a veto, or when a guarded verb achieved under `VERB_MIN_MOVE_M` / `VERB_MIN_TURN_DEG`; `ValueError` on an unknown action |
| `vet_wheel_velocity(left_rad_s, right_rad_s)` | `(left, right, reason)`. `reason` is None when only slowed. Pass-through if the body's wheels are unusable (`robot/safety.py:665`). A reverse component on a blind body with wheels is clamped to zero with `astern_not_observed` in the reason; rotation is untouched |
| `run_verb(plan)` | `carry_out_verb()`'s `{"done", "ended", "reason"}`, after the settle pass when the plan asks for one |
| `forward_clearance()`, `reverse_clearance()`, `path_clearance()`, `footprint_clearance(direction, scan)`, `rear_clearance(scan)`, `pivot_scale(omega, scan)`, `pivot_blocked(omega, scan, exact)` | See above |

**Driver ranks** (`DRIVER_PRIORITY`): `twin-dpad` 30, `teleop-operator` 30,
any unnamed or unknown name 30 (`DRIVER_UNKNOWN = "unknown"`), `brain` 20,
`teleop` 20, `ros` 20, `twin-local-brain` 10 (a dead rank: the in-browser
brain was deleted 2026-09-25).

**`arbitrate(driver, now)`**, in order:

1. If `driver` is autonomous and not `ros`, and the world reports a nav2 goal
   `pending` or `active`: refuse `preempted`. An unreadable bridge counts as
   no goal.
2. If nobody holds the robot (`now - driver_at > watchdog_timeout_s`), or
   the holder is `driver`: allow.
3. Lower rank than the holder: refuse `preempted`.
4. Both autonomous: refuse `preempted`.
5. Otherwise allow (equal manual rank, or a higher rank taking over).

Allowed `/action` and non-zero direct `/wheels` set `driver`, `driver_at`
and `last_command_at` as soon as `arbitrate()` passes, before the command
runs. The claim stands even if the command is then refused or fails:
`ros_unavailable`, `safety_distance`, a 400 for an unknown action, or
`unsupported` from `/wheels` on a body without motors. `/stop` sets only `last_command_at`. A zero direct
`/wheels` is never arbitrated and sets none of the three: from the holder
it zeroes the wheels, from anyone else it returns `ignored: true` (direct
drive only). Under ROS drive, `/wheels` is accepted only from `ros`, is not
arbitrated, and sets `last_command_at` and `last_ros_post_at`, never
authority. `POST /world/goal` arbitrates as `ros` and does not feed the
watchdog.

**The watchdog** (`watchdog_loop()`): every `WATCHDOG_POLL_INTERVAL_S` it
skips while `verb_active`, else calls `robot.stop()` once
`now - last_command_at > watchdog_timeout_s`. Detection is therefore the
timeout plus up to one poll (1.1 s today). Under ROS drive the actuator's
20 Hz posts feed it, so it fires only when the actuator itself goes silent;
a silent ROS input is dropped first by `twist_mux` (0.25 s,
`service/slam/src/picar_bringup/config/twist_mux.yaml`) and then
`diff_drive_controller`'s `cmd_vel_timeout` (0.25 s,
`service/slam/src/picar_bringup/config/controllers.yaml`). The watchdog's
`robot.stop()` runs on the event loop and must not block; `RosDriveRobot`'s
does not wait on the bridge ([body engineering](../body/ENGINEERING.md)).

**Refusal reasons** (body `{"executed": false, "reason", "detail"}`,
counted in `/health` `refusal_counts`, last one in `last_refusal`):
`safety_distance`, `preempted`, `ros_unavailable`, `not_the_actuator`,
`unsupported`, and `watchdog` (recorded in `last_refusal` only, once per
silence).

## Parameters and configuration

| Key / constant | Value | Unit | Read in | Why |
|---|---|---|---|---|
| `safety.min_distance_cm` | 20 | cm | `robot/server.py` | The stopping floor. 20 on both server and brain since S5: 3.3 sigma clear of one cell under sensor noise; 30 vetoed ~45% of legal moves |
| `brain.min_distance_cm` | see mission | cm | `control/brain_config.py` -> `brain/agent.py` | The brain's early out. Value and code default: [mission engineering](../mission/ENGINEERING.md), the canonical home |
| `safety.watchdog_timeout_s` | 1.0 | s | `robot/server.py` | Watchdog and authority lapse |
| `safety.sensor_to_bumper_cm` | 0.0 | cm | `robot/server.py` | Subtracted from cone ranges. To measure on the car |
| `WATCHDOG_POLL_INTERVAL_S` | 0.1 | s | `robot/server.py` | Watchdog wake rate; published in `/health` for `control/health.py` |
| `WHEEL_LOOP_INTERVAL_S` / `VERB_PERIOD_S` / `PIVOT_LOOKAHEAD_S` | 0.05 | s | `robot/server.py`, `robot/interface.py`, `robot/safety.py` | nav2's controller rate. A tick over 0.1 s counts as late |
| `ROS_SILENCE_S` | 0.5 | s | `robot/server.py` | Ten missed 20 Hz actuator posts = ROS down (3.24 G3) |
| `FOOTPRINT_LENGTH_M`, `FOOTPRINT_WIDTH_M`, `LIDAR_X_M`, `LIDAR_TO_REAR_BUMPER_CM` | see platform | m / cm | `robot/safety.py` (defined here; the footprint is imported by `sim/grid_world.py`, `LIDAR_X_M` is the sim's scan origin) | The chassis geometry every check places returns against. `LIDAR_TO_REAR_BUMPER_CM` is derived: half the length plus `LIDAR_X_M`. Values and sources: [platform engineering](../platform/ENGINEERING.md), the canonical table. Held equal to the xacro and nav2 by `tests/test_wall_linters.py` |
| `FOOTPRINT_SIDE_MARGIN_CM` | 3.0 | cm | `robot/safety.py` | Starter-house doors are 30 cm against a 23.1 cm chassis. 3 cm less the march's 1.5 cm over-read; larger refuses every door (`tests/chassis_fit.py`) |
| `PIVOT_MARGIN_CM` | 1.3 | cm | `robot/safety.py` | 1.2 at 3.19. Raised after one guarded turn read 0.96 cm once the lidar moved 4 cm ahead (3.27). Moved after seeing data, as recorded |
| `PIVOT_MIN_LOOKAHEAD_DEG` | 1.0 | deg | `robot/safety.py` | Minimum look-ahead for a slow turn |
| `SAFETY_SCAN_RANGE_M` | 0.6 | m | `robot/safety.py` | Scan hint. Effective `max(0.6, half-length + 1.5 x min_distance)`. Without it the sim cast 360 rays to 12 m, about 13 ms a period |
| `CHASSIS_WIDTH_CM` / `PATH_REFERENCE_CM` | 16.5 / 30.0 | cm | `robot/safety.py` | Cone half-angle `PATH_HALF_ANGLE_DEG` = atan(8.25/30), about 15.4 deg. Errs narrow against the 23.1 cm Rover; the corridor covers the gap |
| `PATH_FRACTION` | 0.5 | | `robot/safety.py` | Only for a grid with no `fov_deg`. Warns above 8 columns |
| `VERB_MIN_MOVE_M` / `VERB_MIN_TURN_DEG` | 0.01 / 0.5 | m / deg | `robot/safety.py` | Under this, a verb is a refusal (3.22 decision 1) |
| `SETTLE_TOLERANCE_DEG` / `SETTLE_TOLERANCE_M` | 0.5 / 0.003 | deg / m | `robot/safety.py` | Same tolerances as `robot/ros_drive.py` |
| `SETTLE_TURN_RAD_S` / `SETTLE_LINEAR_M_S` | 0.1 / 0.02 | rad/s, m/s | `robot/safety.py` | 10 ms late = 0.06 deg / 0.2 mm |
| `SETTLE_WAIT_S` / `SETTLE_PASSES` | 0.15 / 3 | s / count | `robot/safety.py` | Wheels stopped plus three 50 ms feedback frames. Only for `wall_clock` plans with `settle` (fork firmware, 3.29) |

## Procedures

**Run the safety suites** (from `.venv`; the ground-truth ones take a
minute or two):

```bash
.venv/bin/pytest tests/test_depth_veto.py tests/test_footprint_safety.py \
  tests/test_pivot_safety.py tests/test_pan_safety.py tests/test_guarded_verbs.py \
  tests/test_mission_guarded_verbs.py tests/test_wheels_command.py \
  tests/test_ros_verb_safety.py tests/test_mover_safety.py -q
.venv/bin/pytest tests/test_authority.py tests/test_goal_arbitration.py \
  tests/test_ros_fallback.py tests/test_server.py tests/test_watchdog_integration.py -q
```

A failure names a criterion (`test_criterion_N_...`) and prints the three
worst runs with their house, pose, `T` (travel-to-contact) and `G` (gap).

**Run the full ground-truth sweeps** after any change to `robot/safety.py`,
`sim/grid_world.py`'s collision, or a chassis constant:

```bash
python -m tests.demo_footprint_sweep              # 40 starts/house x 24 headings x 2 directions
python -m tests.demo_footprint_sweep --unclamped  # no clamp: judges the sim's own collision (no run > 0.5 cm deep)
python -m tests.demo_verb_sweep
python -m tests.demo_mover_sweep
```

Each prints `PASS`/`FAIL` per criterion and the closest travel-to-contact.
Judge only on these. Never judge on `/depth`'s `veto_cm` or the log line,
which are the readings the vet itself used.

**Diagnose "why is the robot not moving"** from `GET /health`:
`authority_holder` (who holds it, or null once lapsed), `last_refusal`
(`reason`, `detail`, `driver`, `seconds_ago`), `refusal_counts`,
`seconds_since_last_command`, `drive.ros_up`. Then `GET /depth`'s `path`
gives `blocked`, `veto_cm`, `veto_source` and `clearance_cm`.

- `preempted` with a holder named: wait for `watchdog_timeout_s` of silence,
  or have the holder stop driving.
- `safety_distance` with `veto_source` `path_not_observed`: the camera is
  panned and there is no scan. Send `LOOK_CENTER`.
- `ros_unavailable`: the container is down. A person can still drive.
- `safety_distance` with `astern_not_observed`: a REVERSE (or a negative
  `/wheels`, or a settle pass astern) on a body with wheels and no usable
  scan. Expected on the car until its lidar driver lands; turn instead.
- `wheel_loop.late_ticks` above 0: periods the robot moved unvetted.
  Investigate CPU load before anything else.

**Raise a margin**: change the constant, run the full sweeps, and record the
before/after table in the plan entry. A margin moved after seeing data is
recorded as such (3.27 did this for `PIVOT_MARGIN_CM`).

## Verification

| Test (collected) | Pins |
|---|---|
| `tests/test_depth_veto.py` (28) | Tri-state handling; unusable zones never compared; fallback to the scalar; angle-based zone selection |
| `tests/test_footprint_safety.py` (10) | 3.18: seeded sample of the oblique-approach sweep over three houses |
| `tests/test_pan_safety.py` (6) | 3.18 part 2: zones by body bearing; `path_not_observed` |
| `tests/test_pivot_safety.py` (4) | 3.19: no contact from turning; turns with room complete |
| `tests/test_guarded_verbs.py` (8) | 3.22: verbs stop at the line; a stop ends a verb within 100 ms on the fake board |
| `tests/test_mission_guarded_verbs.py` (3) | 3.32: in-process missions use guarded verbs through `_HaltGate` |
| `tests/test_wheels_command.py` (8) | R2b's six criteria plus the zero-heartbeat rules |
| `tests/test_ros_verb_safety.py` (7) | 3.24 G2: the same bars through the ROS path |
| `tests/test_mover_safety.py` (3) | 3.30: a crossing person |
| `tests/test_blind_reverse.py` (5) | 2026-10-02: `HardwareRobot` over the fake board with no sensors refuses a REVERSE verb without moving and clamps a standing reverse to zero; a standing turn (`vet_wheel_velocity()`) passes; a teleop body still reverses; a body with a scan is judged by it. Not pinned: a turn VERB, and a settle pass astern (Known gaps) |
| `tests/test_authority.py` (13) | M4: ranks, lapse, stop claims nothing, reasons on the wire, preemption ends a mission |
| `tests/test_goal_arbitration.py` (9) | 3.23: a goal is an autonomous driver |
| `tests/test_ros_fallback.py` (6) | 3.24 G3, all four criteria |
| `tests/test_watchdog_integration.py` (6) | The watchdog against a live `uvicorn` subprocess with a 0.3 s timeout |
| `tests/test_server.py` (23) | `watchdog_should_stop()`, sensing does not feed the watchdog, safety over HTTP |
| `tests/test_wall_linters.py` | Footprint and `LIDAR_X_M` equal across the ROS wall; both collars still exist |

**Recorded numbers** (all on ground truth; plan sections in
`PLAN-ros-alignment.md`):

| Phase | Result |
|---|---|
| R2b (3.10) | Standing 0.1 m/s into a wall stops at 19.5 cm; 0.0 cm with the clamp removed |
| 3.18 | 5760 runs: 0 under 18 cm (closest 19.5), 0 contacts, progress 98.8%. Unfixed: 25 of 1440 under 18 cm, 9 contacts. Panned camera: 0 (unfixed 42 / 43) |
| 3.19 | 360 pivots: 0 within 1 cm (closest 1.02); 83 of 83 with room turned (worst 2.7 deg short). Unfixed: 120 of 120 |
| 3.24 G2 | 1440 ROS-path verbs: 0 under 18 cm (closest 19.73); 0 of 252 tiny verbs unrefused; clear turns 100% within 0.64 deg (worst 0.49) |
| 3.24 G3, live | Mission ended 2.04 s after a container kill; a D-pad verb ran 0.36 s after it |
| 3.27 | 2880 runs: 0 under 18 cm; 120 pivots: 0 touching (worst gap 1.03). With a centred-lidar assumption: 177 under 18 cm (worst 15.7, astern), 56 pivots touching |
| 3.29 | Fork-firmware turns: 120 of 120 within 1 deg (worst 0.84), +0.16 s a turn |
| 3.30 | 2021 crossing runs: 0 under 18 cm, 0 contacts. Clamp off: 1522 failures |

Change checklist: confirm each new criterion RED against the old code
first; run the sweeps, not just the pinned samples; run the suite in module
order; record the table in the plan entry.

## Known gaps

- **Stale comment:** `robot/safety.py:46-57` says only FORWARD is checked
  and that turns "can never collide". Since 3.19 and 3.22, REVERSE is
  checked and turns are vetted through `run_verb()` and the wheel vet.
- **The car only turns until its lidar driver lands**: see "Driving
  without a lidar" above. By decision.
- **`tests/test_blind_reverse.py` pins less than the rule.** It covers the
  REVERSE verb, a standing reverse and a standing turn. Nothing pins that a
  turn VERB still runs blind on the car, or that a settle pass astern is
  refused `astern_not_observed` (`_settle()` -> `run_verb()` in
  `robot/safety.py`); both are read from the code, not tested.
- **Before the board's first feedback frame**, `get_wheel_state()` is
  unusable (`robot/hardware_robot.py:349`): `vet_wheel_velocity()` passes a
  standing command through unvetted (`robot/safety.py:665`), and
  `_has_wheels()` is false, so a REVERSE verb in that window answers
  `no_rear_sensor` and proceeds unguarded. Open decision
  (`docs-review/SPEC-REVIEW.md` fix-list 6).
- **A stop pauses a nav2 goal; it does not end it** (decided fix not yet
  built; `docs-review/SPEC-REVIEW-2.md` H1). `POST /stop` only records the command
  time and calls `robot.stop()` (`robot/server.py:655-669`).
  `RosDriveRobot.stop()` (`robot/ros_drive.py:266-278`) zeroes the
  `twist_mux` inputs and holds non-zero commands for `STOP_HOLD_S` (0.6 s)
  but cancels no goal, and the bridge cancels a goal only on a **non-zero**
  `twin-dpad` twist
  (`service/slam/src/picar_bridge/picar_bridge/bridge.py:453-455`). nav2
  keeps publishing on `cmd_vel/nav`, the same input the stop's `ros` zero
  goes to, so its next message overrides the zero; under `drive: ros` the
  wheels resume when the hold lapses; under `drive: direct` with
  `WORLD_MODE=ros` they resume on the plugin's next `/wheels` post, about
  50 ms later, because a stop claims nothing and the plugin's next non-zero
  post is arbitrated and allowed. Pre-existing.
  **Decided by the user 2026-10-02; not yet built** (the rule is in the
  [architecture spec](../../safety/ARCHITECTURE.md), "Who drives"): `/stop`
  also cancels any active goal, on a background thread after
  `robot.stop()` returns, so the stop never waits on the bridge; a person
  re-sends a goal to resume. Done when a test with an active goal shows the
  wheels still at zero past `STOP_HOLD_S` after `/stop`, and a new goal can
  be set afterwards.
- **ROS liveness is judged only from the plugin's `/wheels` posts**
  (UNCONFIRMED: read, not run; `docs-review/SPEC-REVIEW-2.md` M2).
  `ros_up()` (`robot/server.py:319-323`) reads `last_ros_post_at`, set only
  by the actuator's posts (`robot/server.py:620`). If the bridge dies while
  `picar_sim_hardware` keeps posting, `ros_up()` stays true, every verb
  fails at `RosDriveRobot._send()`, and `/action` refuses even a person
  `ros_unavailable` (`robot/server.py:571-575`), so the direct fallback
  never engages. If `twist_mux` or `diff_drive_controller` dies, verbs
  achieve nothing and `_refuse_if_nothing_achieved()` returns
  `safety_distance`. **Decided by the user 2026-10-02; not yet built** (the
  rule is in the [architecture spec](../../safety/ARCHITECTURE.md), "When
  ROS dies, only a person drives"): a failed send to the bridge
  (`RosDriveRobot._send()`) marks ROS down, so `ros_up()` is false and
  3.24 G3's fallback applies. Making the container exit when a node dies
  was rejected. Done when a test with a bridge that refuses connections
  shows `drive.ros_up` false, a person's verb runs direct, and an
  autonomous verb is refused `ros_unavailable`. The multiplexer or
  controller case is unchanged by it.
- **Turns on a body with no `verb_plan()`** (the remote body, the ROS
  wrapper) get no pivot check inside `check_and_execute()`. The remote body
  is vetted again by the server, and the ROS path by the wheel vet, so the
  car is covered. An in-process caller holding such a body directly is not.
- **`CHASSIS_WIDTH_CM` is 16.5** (the PiCar-X), so the cone errs narrow
  against the Rover's 23.1 cm. The corridor check covers the difference.
  Re-measure on the car.
- **`sensor_to_bumper_cm` is 0.0** and applies to the cone only; the
  corridor and rear checks place returns with `LIDAR_X_M` instead.
- **Unnamed, `teleop` and `teleop-operator` drivers under ROS drive** are
  refused `ros_unavailable`: the bridge maps only `twin-dpad`, `brain` and
  `ros` (`docs-review/REPORT.md` V10).
- **Until `ROS_SILENCE_S` (0.5 s) of silence after a container dies**,
  `ros_up()` is still true, so a person's verb goes to the dead bridge and
  is refused `ros_unavailable`. That window is inside G3's 2 s bar. The
  bridge-failure rule above (decided, not yet built) would close it at the
  first failed send.
- **The fixed 20 cm floor** is a stopping distance for about 0.45 m/s
  (`PLAN-onboard-perception.md`); a ROS verb peaks at 0.6 m/s and relies on
  the look-ahead.
- **Late safety ticks on the Jetson** are unmeasured (3.33).
