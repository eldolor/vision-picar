---
kind: engineering
domain: safety
status: current
verified: 2026-10-04
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
| `/action` FORWARD | `check_and_execute()` | Pre-check `forward_clearance() < required_clearance_cm(v, creep)` raises, where `v` is the verb's `speed` as m/s (`verb_speed_m_s()`). Then, if the body has a `verb_plan()`, `run_verb()` re-vets every 50 ms period with look-ahead against the same bar |
| `/action` REVERSE | same | `reverse_clearance()`, then `run_verb()`. Refused outright (`astern_not_observed`) on a body with wheels and no usable scan |
| `/action` LEFT / RIGHT | same | No pre-check. `run_verb()` limits each step with `pivot_scale()` (only on bodies with a plan). With no usable scan, never limited |
| `/action` LOOK_* / STOP | `_dispatch()` | None |
| `POST /wheels` (direct drive) | `vet_wheel_velocity()` on receipt, then every `wheel_loop()` tick | Forward/reverse look-ahead and clamp; rotation scaled by `pivot_scale()`. Its own early return passes a command through while the body's wheel state is unusable; on `HardwareRobot` the body then refuses it (`no_feedback`, 3.34), so nothing reaches the board unvetted. A tick reads `stop_count` before the command and skips its write, or zeroes after it, if a `/stop` (which never takes `motion_lock`) landed in between (3.76 A2) |
| ROS drive: verbs | `check_and_execute()` -> `RosDriveRobot` (no plan) | The pre-check, then the actuator's `/wheels` posts are vetted on receipt and every tick. `_refuse_if_nothing_achieved()` turns a verb that covered under 1 cm / 0.5 deg into a `SafetyViolation`, except one `RosDriveRobot` marks `superseded` (cut off by a newer verb or a stop, 3.76 T1): `/action` answers that one `preempted` when another driver now holds the robot, else as done |
| ROS drive, ROS down: a person's verbs | `fallback_safety.check_and_execute()` on the wrapped body | Exactly direct drive's path |

### Clearance sources

Each clearance method returns `(cm or None, source)`. `None` means nothing
within range, never "unknown".

| Method | Sources | Meaning |
|---|---|---|
| `path_clearance()` | `depth_grid_facing_away`, `depth_grid`, `depth_grid_no_target`, `scan_path`, `scan_path_no_target`, `distance_sensor` | The cone, tried in that order. A panned grid with no path zones left is no opinion (`None`); otherwise the nearest `ZONE_RANGE` in the path zones; otherwise, if any path zone answered, no target; otherwise, with a usable scan, its beams within `PATH_HALF_ANGLE_DEG` of ahead minus `LIDAR_TO_FRONT_BUMPER_CM` (3.42: without it a lidar-only car fell to the scalar's 0.0 and could never drive forward); otherwise the scalar `get_distance()`. Unusable zones are dropped. All values are bumper-relative (`sensor_to_bumper_cm` subtracted, floored at 0) |
| `footprint_clearance(+1 / -1)` | `scan_footprint`, `scan_footprint_no_target`, `no_scan` | Returns in the body frame within half-width + 3 cm of the centre-line and beyond the leading edge: distance past the edge. A return inside the outline reads 0.0 |
| `rear_clearance()` | `scan_rear`, `scan_rear_no_target`, `no_rear_sensor` | Beams within `PATH_HALF_ANGLE_DEG` of astern, minus `LIDAR_TO_REAR_BUMPER_CM` |
| `floor_clearance(grid)` | `floor_band`, `floor_band_no_target`, `floor_band_unusable`, `no_floor_band` | 3.74. The depth grid's `floor` block (the depth camera's floor band: body-fixed, `origin_x_cm` ahead of `base_link`, `fov_deg`, `min_range_cm`, zones of `{status, distance_cm, bearing_deg?}`; fields in `RobotInterface.get_depth_grid()`). Each `ZONE_RANGE` zone is a point in the body frame -- range from the origin along the zone's `bearing_deg` (null or absent: its centre) -- judged by `_corridor_room()`, the one corridor rule the scan's `footprint_clearance()` uses too: ahead of the front edge within half-width + 3 cm, the room past the edge; beside the body only within the half-width (0.0). Inside the band's minimum range nothing is seen or carried (Known gaps). Unusable zones never compared. `no_floor_band` for a grid without the block or with no usable `fov_deg` (every real body today). `floor_band_unusable` (3.76 K1) when no zone could be read (blind is not clear) or the band itself cannot be: a field that does not parse, or zones that disagree with its `cols`; a malformed zone is skipped as unusable, never raised out of the check (a raise made every FORWARD a 400 and skipped the wheel loop's re-vet) |
| `forward_clearance()` | the least of cone, corridor and floor band, or `path_not_observed` (0.0); then aged (below) | The cone and the band read ONE depth grid, the corridor and the band ONE scan. `path_not_observed` when the grid faces away and there is no scan; the band does not lift it |
| `reverse_clearance()` | `astern_not_observed` (0.0), or the lesser of rear and corridor astern, off one scan; then aged (below) | `(0.0, "astern_not_observed")` when the scan is missing or unusable **and** `_has_wheels()` (the body's `get_wheel_state()` reports `usable: true`; a body without the method, or whose call raises, has none). Otherwise, with no scan, `(None, "no_rear_sensor")` and the reverse proceeds. The rule and its reasons: [architecture spec](../../safety/ARCHITECTURE.md), "A body that cannot see astern does not reverse" |
| `pivot_blocked(omega)` / `pivot_scale(omega)` | reason string or None / `(fraction, reason)` | A return whose distance to the rectangle would end under `PIVOT_MARGIN_CM` **and** shrink. Binary search to 1/64 of the turn rate. No usable scan: never blocked. **Aged rotation** (3.42): `_turn_since_scan()` is the turn the scan has missed -- the body's own answer (`turn_since_scan()`, `HardwareRobot` from its encoder heading history) or else yaw rate x `sensor_age_s()` -- and the check runs from `PIVOT_AGE_SAMPLES` + 1 rotations across that arc. Zero for a body whose readings have no age |

**Aged readings** (`_aged(result, direction)`, `PLAN-ros-alignment.md`
3.36). The look-ahead (3.24 G2) stops a move AT the line only if the
clearance is the clearance now. A body that reports how old its readings
are (`sensor_age_s()`: `sim/body_client.py`'s `SimBodyClient`, whose
polled bundle is up to `SENSOR_STALE_S` 0.15 s old, and since 3.42
`HardwareRobot` with a lidar, whose scan is 0-150 ms old) has the way
covered since subtracted: `clearance - max(0, direction x v) x age`, with
`v` the mean wheel speed times `wheel_radius_m`, floored at 0.0 and
rounded to 0.1 cm, and the source gains `, aged <n> ms`. It is skipped,
leaving the result untouched, when the clearance is None or 0, the body
has no `sensor_age_s` or reports no age, the wheel state is unusable or
raises, or the robot is not closing (moving away, or still). Every
in-process backend reads on demand and is unchanged. 3.36's sweep fed
three-period-old readings: worst travel-to-contact 18.2 cm without it,
19.0 cm with it (`tests/test_footprint_safety.py`). On the car a lidar scan
is 0-100 ms old when it is read, which is the rule's reason beyond the
sim.

Comparison edges: the verb pre-check refuses at `< need`; the wheel vet
clamps at `<= need` and otherwise limits speed to `(clearance - need) /
0.05 s`. `run_verb()`'s limit allows `clearance - need` metres, so it ends
`clamped` at the line.

**The bar, `need` (3.44):** `required_clearance_cm(v, creep)` is
`CREEP_MARGIN_CM` (3 cm) when `creep` and `|v| <= CREEP_M_S` (0.03 m/s),
else `max(min_distance_cm, stop_distance_cm(v))` with
`stop_distance_cm(v) = 3 + 100 (|v| x 0.15 + v^2 / (2 x 0.5))` cm -- 20 cm
up to about 0.39 m/s, 25 at 0.4, 35.5 at 0.5, 48 at 0.6. `v` unknown
(None): `min_distance_cm`. `creep` is true only while a straight verb
ASKED at creep speed runs: `check_and_execute()` sets `_asked_v_m_s` and
`_creep_asked` through `_asking()` for the verb's whole run and clears them
in `finally`. The server shares one `SafetyController` between `/action`
and the wheel loop, so under ROS drive the twists of a creep verb are
vetted at the creep bar while it runs; nav2's twists, which no verb asked,
never are. The wheel vet computes `need` from the COMMANDED `v` before
slowing it, and `run_verb()` from the asked speed, so the vet's own slowing
never earns the creep margin.

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
frame** (and whenever feedback is stale) the body refuses every non-zero
command and verb `no_feedback` (3.34), so the table's gaps there are moot
(`tests/test_startup_race.py` pins the first-frame case). The decision
behind the reverse row:
[architecture spec](../../safety/ARCHITECTURE.md), "A body that cannot see
astern does not reverse".

## Interfaces

**`SafetyController(robot, min_distance_cm=20.0, sensor_to_bumper_cm=0.0)`**

| Method | Returns / raises |
|---|---|
| `check_and_execute(action, speed=, duration=, angle=)` | The body's result dict. Raises `SafetyViolation` on a veto, or when a guarded verb achieved under `VERB_MIN_MOVE_M` / `VERB_MIN_TURN_DEG`; `ValueError` on an unknown action. A straight verb's `speed` (default 50) sets its bar (3.44) |
| `required_clearance_cm(v_m_s, creep=False)` | The straight-move bar in cm, as above |
| `scan_hint_m()` | The range the vet's scans ask for: half the chassis length plus 1.5 x the bar at `MAX_LINEAR_M_S` (about 0.85 m) |
| `vet_wheel_velocity(left_rad_s, right_rad_s)` | `(left, right, reason)`. `reason` is None when only slowed. Pass-through if the body's wheels are unusable (`robot/safety.py:665`). A reverse component on a blind body with wheels is clamped to zero with `astern_not_observed` in the reason; rotation is untouched |
| `run_verb(plan)` | `carry_out_verb()`'s `{"done", "ended", "reason"}`, after the settle pass when the plan asks for one |
| `forward_clearance()`, `reverse_clearance()` (both aged), `path_clearance()`, `footprint_clearance(direction, scan)`, `rear_clearance(scan)`, `pivot_scale(omega, scan)`, `pivot_blocked(omega, scan, exact)` | See above |

**Driver ranks** (`DRIVER_PRIORITY`): `twin-dpad` 30, `teleop-operator` 30,
any unnamed or unknown name 30 (`DRIVER_UNKNOWN = "unknown"`), `brain` 20,
`teleop` 20, `ros` 20, `twin-local-brain` 10 (a dead rank: the in-browser
brain was deleted 2026-09-25).

**`arbitrate(driver, now)`**, in order:

1. If `driver` is autonomous and not `ros`, and the world reports a nav2 goal
   `pending` or `active`: refuse `preempted`. An unreadable bridge counts as
   no goal.
2. If nobody holds the robot, or the holder is `driver`: allow. The holder
   is `driver` while any `/action` verb is being carried out, or a goal
   naming a person is with the bridge (`state["verbs_running"]`, every drive
   path, 3.76 A1; the goal since 3.82), else `driver` until
   `watchdog_timeout_s` after `driver_at`. A verb's end re-stamps
   `driver_at` if its driver still holds the robot, before the count drops,
   so a verb longer than the timeout never lapses mid-run; a person's goal re-stamps it only when
   the bridge took the goal, and otherwise gives the slot back to whoever
   held it if nobody has driven since.
3. Lower rank than the holder: refuse `preempted`.
4. Both autonomous: refuse `preempted`.
5. Otherwise allow (equal manual rank, or a higher rank taking over).

Allowed `/action` and non-zero direct `/wheels` set `driver`, `driver_at`
and `last_command_at` as soon as `arbitrate()` passes, before the command
runs. The claim stands even if the command is then refused or fails:
`ros_unavailable`, `safety_distance`, `no_feedback`, a 400 for an unknown
action, or `unsupported` from `/wheels` on a body without motors. `/stop` sets only `last_command_at`. A zero direct
`/wheels` is never arbitrated and sets none of the three: from the holder
it zeroes the wheels, from anyone else it returns `ignored: true` (direct
drive only). Under ROS drive, `/wheels` is accepted only from `ros`, is not
arbitrated, and sets `last_command_at` and `last_ros_post_at`, never
authority. `POST /world/goal` arbitrates as `ros` and does not feed the
watchdog.

**Stop ends a nav2 goal** (handoff 2026-10-02 1b; completed after
`docs-review/SPEC-REVIEW-3.md` V1-V2). After `robot.stop()` returns,
`POST /stop` -- unless its `x-driver` is autonomous and not `ros` (the
brain), which spares the goal -- bumps `goal_stop["gen"]`, sets
`goal_stop["hold"]`, and starts a daemon thread `_end_goal_after_stop(gen)`
(`stop-end-goal`) when the world has `cancel_goal` (`RosWorld`). Every
`GOAL_STOP_POLL_S` the thread, under `goal_stop_lock` (held across the
bridge calls), reads
`world_model.get_goal()` and re-sends `cancel_goal()` (`DELETE /goal`)
while the goal is `pending` or `active`; once it is neither it waits
`GOAL_STOP_SETTLE_S` and clears the hold. An exception (bridge down,
timeout) is logged once and retried, still holding. While the hold is set,
a non-zero `/wheels` from `ros` is treated as zero: under ROS drive it is
applied as zero; under direct drive it takes the zero-command branch. A
newer stop or an accepted `POST /world/goal` bumps the generation, so one
loop runs at a time and never cancels a new goal (the goal route sets it
under the same lock). A stop that ends a goal (every stop but the
brain's) also counts itself in `goal_stop["stops"]`, which only those stops
move. The goal route reads that count
as the request arrives -- before arbitration and any wait on
`goal_stop_lock` -- and, only when `set_goal()`'s answer is `accepted: true`
and no stop has been pressed since, bumps the generation and clears the hold
(3.82). A goal a stop overtook while it waited for the lock is answered
`{"accepted": false, "reason": "stopped"}` without being sent; another goal
accepted meanwhile does not make it stale. A goal the bridge took while a
stop was pressed is answered `stopped` too (the stop's loop ends it, and the
person's claim is given back):
a refused goal (bridge unreachable, its 409, nav2 down, `accepted: false`)
never existed, so the stop's loop keeps ending the goal it was for and
keeps its hold; and a stop that landed while `set_goal()` was in flight owns the
newer generation, so its loop, waiting on `goal_stop_lock`, ends the goal
just taken. `goal_stop`'s two fields have their own lock,
`goal_gen_lock`, never held across a bridge call; `/stop` takes only that
one, so it answers at once while the bridge hangs (3.76 A8). The loop
clears the hold only if its generation is still current (and then sets
`goal_stop["ending"]`, the generation `/stop` gave it, back to None). A verb
allowed through on the ROS drive path lifts the hold, as it lifts
`RosDriveRobot`'s own stop hold (`lift_hold_for_verb()`, 3.84): under
`goal_gen_lock`, and only if `goal_stop["stops"]` is still the count `/action`
read as the request arrived -- a verb a stop overtook is not run and is
refused `stopped`, and the stop keeps its hold. `goal_stop["lifted"]`
counts the verbs running with the hold lifted, keyed by the stop count they
lifted it under; when the LAST one lifted under the newest stop ends
(`rehold_after_verb()`, in a `finally`; one lifted before a newer stop
neither counts nor re-holds), the hold is set again if a stop's
loop is still ending a goal and is the current one (`ending == gen`; an
accepted goal clears `ending`), so nav2's `ros` commands for the stopped goal
stay at zero after a person's move; the loop keeps cancelling and lifts it
when it settles. The `/action` STOP verb and the watchdog do none of this; they
call `robot.stop()`.

**A person's Stop is the last word over autonomy** (3.84, decided with the
user 2026-10-15). A `POST /stop` whose driver ranks as a person
(`driver_priority() == DRIVER_MANUAL`: `twin-dpad`, `teleop-operator`, an
unnamed caller, any unknown name) sets `goal_stop["autonomy_stopped"]`
(`{"by", "at"}`, under `goal_gen_lock`) BEFORE `robot.stop()`, whatever the
world. While it is set, `person_stop_refusal()` refuses every command from
an autonomous driver other than `ros` (`is_autonomy()`: `brain`, `teleop`)
with `reason: "person_stop"`, before arbitration: `/action`, a non-zero
direct `/wheels`, and a `POST /world/goal` naming `brain` (explore names
itself since 3.84; the latch is re-read under the goal lock, and a brain
goal under it answers `person_stop` whether or not it is stale, 3.76 K1).
A brain command refused on arrival lets go of the robot too
(`lapse_refused_claim()` with the claim's stamp read on arrival, 3.76 K1).
`/stop` counts its stop (`stops`, `gen`, the hold) in the same
`goal_gen_lock` step as the latch, before `robot.stop()`, and starts the
goal-ending loop even if `robot.stop()` raises (3.76 K1: counted after it,
a goal landing in between was accepted -- a brain goal under the latch, or
a person's tap the Stop then cancelled). Every `/action STOP`, from anyone,
is answered by `/stop` (never arbitrated). It is read again
at execution, because an autonomous verb's arbitration can wait on the
bridge: a direct verb under `motion_lock`, a ROS verb in
`lift_hold_for_verb()` under `goal_gen_lock`. Just before that read the
server takes the robot's `stop_count` and hands it to the verb as
`robot.interface.VERB_STOP_BASELINE` (a context variable): `carry_out_verb()`,
`SafetyController.run_verb()`'s settle and `RosDriveRobot._begin()` use it
as their stop baseline instead of reading the count later, so a Stop that
lands between the read and the verb's first motion (the Stop latches before
its `robot.stop()`) ends the verb unmoved -- `_begin()` then returns an
already-superseded generation and leaves the Stop's own hold in place
(`RosDriveRobot.stop_count` is its own stops plus the inner robot's, so a
stop made on the inner robot directly counts too). Under a person's
latch set by the time a brain verb ends, every answer but a fault (`FAULTS`:
`no_feedback`, `ros_unavailable`) becomes `person_stop` -- cut by the Stop, ended unmoved by the
baseline, or read as a veto because of the Stop's zeroing; a move that ran
is recorded as that refusal once, a refusal already recorded is not
recorded again -- and any answer that is not a move lets go of the robot. A brain command refused at execution
(`person_stop`, or `stopped`) had been given the robot by arbitration; its
claim lapses at once (`lapse_refused_claim()`), only if `driver_at` is still
that command's own stamp (`verb_running(stamped=...)`), never a newer
one's; `autonomous_at` keeps it (the house swap stays refused: the safe
side). A direct verb sets `verb_active` before taking
the baseline, so a watchdog stop cannot land in between. Nothing is logged
under `goal_gen_lock`. A direct `/wheels` takes
`stop_count` before its vet and, if any stop landed by the end of its write
(`/stop` never takes `motion_lock`), zeroes the wheels again and answers
`person_stop` (the brain, latched) or `stopped`. Never refused:
any `/stop`, `DELETE /world/goal`, a person's verbs and goals, unnamed
goals (ranked `ros`, today's behaviour), and `ros`'s `/wheels` (the hold
governs those). `brain`, `teleop` and `ros` stops set nothing. Only
`POST /autonomy/resume` clears it, which the brain sends when a mission
starts (`RemoteRobot.resume_autonomy()`, first in `start_mission`): a
person's next verb or goal does not, so a mission the person stopped can
never drive again. A start refused after the lift (400, 403, 409, any
error) began no mission, so the brain puts the latch back with
`POST /autonomy/relatch` (3.76 K2), which only latches and keeps a latch
a Stop during the build set since.

**ROS liveness** (handoff 1d; refined by `docs-review/SPEC-REVIEW-3.md`
V7-V9). Under ROS drive, `ros_up(now)` is true when the actuator's last
`/wheels` post is younger than `ROS_SILENCE_S` AND `robot.bridge_up()`.
`RosDriveRobot._send()` marks the bridge down on a transport error
(connect, timeout) or a 5xx; a 4xx is the bridge answering, so it does not
mark it down and clears an earlier mark, as does any successful send.
`bridge_up()` never blocks: while down it answers false and starts at most
one background `GET /health` probe per `BRIDGE_PROBE_S` (the probe's
constants are in [body engineering](../body/ENGINEERING.md)); a 200 marks
the bridge up. So the first `bridge_up()` after a mark always answers false,
and the probe runs only when something asks. While the bridge is down, an
actuator `/wheels` post still stamps `last_ros_post_at` but is answered
`ros_unavailable` (unlogged, at 20 Hz) and moves nothing, so a person on
the fallback is the only writer. `/health` publishes `drive.bridge_up` and
`drive.ros_post_age_s` beside `drive.ros_up`.

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
`unsupported`, `no_feedback` (3.34: the body's wheel feedback is stale, so
it refuses to move them; from driver `ros` on `/wheels` it is answered
unlogged, as `ros_unavailable` is), and `watchdog` (recorded in
`last_refusal` only, once per silence).

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
| `GOAL_STOP_POLL_S`, `GOAL_STOP_SETTLE_S` | 0.25, 0.6 | s | `robot/server.py` | How often a stop re-reads and re-cancels a goal until nav2 reports it over, and how long `ros` wheel commands stay held after that. The settle is `STOP_HOLD_S`'s derivation: twist_mux's input timeout plus the controller's `cmd_vel_timeout` (they add) plus one plugin period |
| `FOOTPRINT_LENGTH_M`, `FOOTPRINT_WIDTH_M`, `LIDAR_X_M`, `LIDAR_TO_REAR_BUMPER_CM`, `LIDAR_TO_FRONT_BUMPER_CM` | see platform | m / cm | `robot/safety.py` (defined here; the footprint is imported by `sim/grid_world.py`, `LIDAR_X_M` is the sim's scan origin) | The chassis geometry every check places returns against. `LIDAR_TO_REAR_BUMPER_CM` is derived: half the length plus `LIDAR_X_M`; `LIDAR_TO_FRONT_BUMPER_CM` half the length minus it (3.42). Values and sources: [platform engineering](../platform/ENGINEERING.md), the canonical table. Held equal to the xacro and nav2 by `tests/test_wall_linters.py` |
| `FOOTPRINT_SIDE_MARGIN_CM` | 3.0 | cm | `robot/safety.py` | Starter-house doors are 30 cm against a 23.1 cm chassis. 3 cm less the march's 1.5 cm over-read; larger refuses every door (`tests/chassis_fit.py`) |
| `PIVOT_MARGIN_CM` | 1.3 | cm | `robot/safety.py` | 1.2 at 3.19. Raised after one guarded turn read 0.96 cm once the lidar moved 4 cm ahead (3.27). Moved after seeing data, as recorded |
| `PIVOT_MIN_LOOKAHEAD_DEG` | 1.0 | deg | `robot/safety.py` | Minimum look-ahead for a slow turn |
| `PIVOT_AGE_SAMPLES` | 4 | | `robot/safety.py` | Rotations checked across the turn a scan has missed (3.42) |
| `SAFETY_SCAN_RANGE_M` | 0.6 | m | `robot/safety.py` | Scan hint floor. Effective `max(0.6, scan_hint_m())`, about 0.85 m since 3.44: the sim reports beams past the hint as nothing there, so a hint under the top speed's bar made the SIMULATOR blind where the car sees. A real lidar ignores it. Without a hint the sim cast 360 rays to 12 m, about 13 ms a period |
| `CREEP_M_S` / `CREEP_MARGIN_CM` | 0.03 / 3.0 | m/s / cm | `robot/safety.py` | 3.44: a verb asked this slowly keeps only the corridor's side margin. 3.43's wedged robot stood 19.3 cm from what was behind it |
| `T_REACT_S` | 0.15 | s | `robot/safety.py` | One control period plus the R4 actuator chain (40-150 ms). Excludes the sensor's age, which `_aged()` already subtracts |
| `DECEL_M_S2` | 0.5 | m/s^2 | `robot/safety.py` | **[PLACEHOLDER]** braking, until 6.9(d) measures it on the car |
| `MAX_LINEAR_M_S` / `VERB_FULL_SPEED_M_S` | 0.6 / 0.6 | m/s | `robot/safety.py` | The chassis cap (controllers.yaml), for the scan hint; a verb's speed 100 (`moves_for()`'s two 0.30 m moves a second) |
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
  tests/test_ros_fallback.py tests/test_stop_cancels_goal.py tests/test_server.py \
  tests/test_watchdog_integration.py -q
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
- `person_stop`: a person pressed Stop; the brain stays out until a mission
  starts (3.84).
- `safety_distance` with `veto_source` `path_not_observed`: the camera is
  panned and there is no scan. Send `LOOK_CENTER`.
- `ros_unavailable`: the container is down. A person can still drive.
- `no_feedback`: the motor board has not reported for
  `FEEDBACK_STALE_S`. Read `/health` `motor_board`: `link_error` set means
  the serial line is gone (cable, USB, udev); `link_error` null with a
  growing `age_s` means the board is heard-but-silent (feedback switched
  off, firmware hung). Nothing moves until frames return.
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
| `tests/test_footprint_safety.py` (12) | 3.18: seeded sample of the oblique-approach sweep over three houses. 3.36: the same bars hold on readings `SENSOR_STALE_S` old, and `_aged()` subtracts `v x age` closing and nothing moving away or with no age |
| `tests/test_pan_safety.py` (6) | 3.18 part 2: zones by body bearing; `path_not_observed` |
| `tests/test_pivot_safety.py` (4) | 3.19: no contact from turning; turns with room complete |
| `tests/test_lidar_safety.py` (4) | 3.42 criterion 4: the 3.18/3.19 samples with the vet reading the D500's timed scan (no depth grid): straight runs pass; pivots 0/120 within 1 cm with the rotation aged, and fail without it |
| `tests/test_guarded_verbs.py` (8) | 3.22: verbs stop at the line; a stop ends a verb within 100 ms on the fake board. 4a judges "clear" against each speed's own bar (3.44 amendment) |
| `tests/test_speed_clearance.py` (15) | 3.44: today's speeds keep 20 cm; the creep margin only when asked; an unasked 0.02 m/s twist and a look-ahead-slowed verb stop at 20 cm; creep sweep samples (verb instant and lidar-timed, wheel loop) never within 2 cm, and the instrument fails with no margin; 0.4/0.5 m/s sweeps; the scan hint covers the top speed's bar |
| `tests/test_mission_guarded_verbs.py` (3) | 3.32: in-process missions use guarded verbs through `_HaltGate` |
| `tests/test_wheels_command.py` (8) | R2b's six criteria plus the zero-heartbeat rules |
| `tests/test_ros_verb_safety.py` (7) | 3.24 G2: the same bars through the ROS path |
| `tests/test_mover_safety.py` (3) | 3.30: a crossing person |
| `tests/test_blind_reverse.py` (5) | 2026-10-02: `HardwareRobot` over the fake board with no sensors refuses a REVERSE verb without moving and clamps a standing reverse to zero; a standing turn (`vet_wheel_velocity()`) passes; a teleop body still reverses; a body with a scan is judged by it. Not pinned: a turn VERB, and a settle pass astern (Known gaps) |
| `tests/test_authority.py` (13) | M4: ranks, lapse, stop claims nothing, reasons on the wire, preemption ends a mission |
| `tests/test_review_376a.py` (5) | 3.76 batch A: a verb longer than the watchdog keeps its driver's authority; the wheel loop never writes over a stop; lost feedback under ROS drive is `no_feedback` and stops the robot; a second stop answers while the bridge hangs; a person taking over a ROS verb is not stopped by it, and the cut-off verb reads `preempted` |
| `tests/test_review_376k1.py` | 3.76 K1: a person's `/action STOP` latches and ends the goal, the brain's is never refused; a brain goal or a person's tap landing inside a Stop is not accepted; a Stop whose `robot.stop()` raises still ends the goal; a brain refused on arrival lets go; the floor band never raises (bad fields, non-dict, NaN, `cols` mismatch or Infinity are `floor_band_unusable`), a blind band says so, an open-floor band reads clear; battery outside 6-30 V is no reading and one glitch frame keeps the last pack reading |
| `tests/test_person_stop.py` | 3.84: a person's Stop (`twin-dpad`, unnamed, `teleop-operator`) during explore: nav2 at 0.0 rad/s for 3 s, explore's next goal never reaches the bridge, the mission ends `cancelled`/`stop`; under `drive: ros` a verb arbitrated before a Stop (held between arbitration and the hold lift) leaves its hold set, and a person's verb during the Stop's loop turns > 30 deg, then nav2 is held at 0.0 until the loop ends the goal and lifts it; a brain verb or `brain`-named goal after it is refused `person_stop`; a mission over `RemoteRobot` ends `cancelled`/`stop`; a `brain` or `ros` stop latches nothing; a person's verb and goal, and an unnamed goal, still drive; `/autonomy/resume` and a new mission lift it |
| `tests/test_goal_arbitration.py` (9) | 3.23: a goal is an autonomous driver |
| `tests/test_stop_cancels_goal.py` (13) | Handoff 2026-10-02 1b: with an active goal and a fake nav2 that drives while it holds one, the wheels are still at zero `STOP_HOLD_S` + 0.1 s after `/stop`; a new goal can be set afterwards; a hung cancel costs the stop nothing. Spec review 3 V1: the brain's `/stop` spares the goal, and a mission preempted by a goal (`MissionRunner` over `RemoteRobot`) ends `preempted` with the goal still active; a person's stop still ends it. V2, nav2 ticking at 20 Hz from the moment of the stop: zero wheels while a 0.3 s cancel is in flight, through a failing cancel (retried once the bridge returns), and for a goal accepted after the stop; a new goal drives again |
| `tests/test_ros_fallback.py` (10) | 3.24 G3, all four criteria; handoff 1d: a dead bridge behind a live plugin reads `drive.ros_up` false, a person's verb runs direct, an autonomous one is refused `ros_unavailable`, and ROS is back once the bridge answers. Spec review 3 V8-V9: with the bridge down and the plugin alive, the plugin's non-zero `/wheels` post is answered `ros_unavailable` and moves nothing, and `/health` names which half is down |
| `tests/test_watchdog_integration.py` (6) | The watchdog against a live `uvicorn` subprocess with a 0.3 s timeout |
| `tests/test_server.py` (23) | `watchdog_should_stop()`, sensing does not feed the watchdog, safety over HTTP |
| `tests/test_wall_linters.py` | Footprint and `LIDAR_X_M` equal across the ROS wall; both collars still exist |
| `tests/test_floor_hazards.py` | 3.74: the band vetoes what the scan passes over; no block, no change; unusable zones ignored; a 30 cm corridor's walls never stop the robot; beside the body only inside its width; a null bearing falls back to the zone centre; band and scan points share one frame |
| `tests/test_floor_band_safety.py` | 3.74 B1 and B2a samples (slow tier): a sock ahead stops every run at >= 18 cm, and without the band runs touch it; with nothing low the band changes no run |

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
| 3.74 | A 5 cm object ahead, 600 runs at 0.1 and 0.2 m/s: 0 under 18 cm (closest 19.76), 0 contacts; band off 600 / 600 under, 437 contacts. Nothing low, 1920 runs: exact, no run changed; with S5 noise, 10.2% stopped > 3 cm earlier (bar 1%, failed; a property of per-zone noise, see the plan), progress -0.87 points |

Change checklist: confirm each new criterion RED against the old code
first; run the sweeps, not just the pinned samples; run the suite in module
order; record the table in the plan entry.

## Known gaps

- **The floor band's blind range (3.74).** Inside the depth camera's
  minimum range (about 14 cm past the bumper) the band sees nothing, and
  nothing carries an earlier sighting across it. A FORWARD at the normal
  bar (20 cm) never gets there; a creep (3 cm bar, 3.44) after the band
  stopped the robot, or a pivot that swings a low object in, can. A hold
  carried by the wheels was built and removed in review: each of the
  robot server's two controllers and the brain's own kept a separate one,
  `/depth` raced it, and it failed open on a feedback blip. Options are in
  `PLAN-ros-alignment.md` 3.74.

- **The car only turns until its lidar driver lands**: see "Driving
  without a lidar" above. By decision.
- **`tests/test_blind_reverse.py` pins less than the rule.** It covers the
  REVERSE verb, a standing reverse and a standing turn. Nothing pins that a
  turn VERB still runs blind on the car, or that a settle pass astern is
  refused `astern_not_observed` (`_settle()` -> `run_verb()` in
  `robot/safety.py`); both are read from the code, not tested.
- **`vet_wheel_velocity()` still passes a command through when the body's
  wheel state is unusable** (`robot/safety.py`, the early return). Harmless
  on every body today: `HardwareRobot` refuses motion without fresh feedback
  (3.34; handoff 2b closed), and the sim's bodies always have wheels. A new
  backend that reports unusable wheels and still moves would reopen it.
- **The verb that finds the bridge dead is refused, a person's too.** The
  first failed `RosDriveRobot._send()` raises inside the verb, which
  `/action` refuses `ros_unavailable` after stopping the robot directly
  (`WheelFeedbackLost`, also a `RuntimeError`, is refused `no_feedback`
  instead, after the same direct stop: 3.76 A5); it
  is not retried on the direct path, because some of it may already have
  gone through ROS. Only the next verb sees `ros_up()` false and falls back.
- **A dead `twist_mux` or `diff_drive_controller` behind a live bridge**
  still reads as alive (decided 2026-10-02 to leave it): verbs achieve
  nothing and `_refuse_if_nothing_achieved()` returns `safety_distance`,
  which fails toward stop.
- **Turns on a body with no `verb_plan()`** (the remote body, the ROS
  wrapper) get no pivot check inside `check_and_execute()`. The remote body
  is vetted again by the server, and the ROS path by the wheel vet, so the
  car is covered. An in-process caller holding such a body directly is not.
- **`CHASSIS_WIDTH_CM` is 16.5** (the PiCar-X), so the cone errs narrow
  against the Rover's 23.1 cm. The corridor check covers the difference.
  Re-measure on the car.
- **`sensor_to_bumper_cm` is 0.0** and applies to the cone only; the
  corridor and rear checks place returns with `LIDAR_X_M` instead.
- **Closed 2026-10-05 (handoff 3a):** unnamed, `teleop` and
  `teleop-operator` drivers under ROS drive used to be refused
  `ros_unavailable` (`docs-review/REPORT.md` V10); `robot/ros_drive.py`'s
  `ros_input_for()` now sends every person on the D-pad's `twist_mux`
  input and every other autonomous driver on the brain's. **And a person's
  goal is a person driving (handoff 3b):** `POST /world/goal` with a
  manual-rank `x-driver` arbitrates as that person -- it takes the robot
  from the brain, whose next command is refused `preempted` -- while a goal
  naming no one keeps 3.23's `ros` rank.
- **Until `ROS_SILENCE_S` (0.5 s) of silence after a container dies**,
  `ros_up()` is still true, so a person's verb goes to the dead bridge and
  is refused `ros_unavailable`. That window is inside G3's 2 s bar. The
  bridge-failure rule (built 2026-10-02) shortens it: the first failed send
  marks ROS down, so only that one verb is refused.
- **`DECEL_M_S2` is a placeholder** (3.44). Above about 0.39 m/s the bar
  is set by it, unmeasured; raising nav2's speed also waits on 3.42's
  relayed-scan time stamp (6.9).
- **A creep verb plans a whole 30 cm cell**, as every verb does
  (`moves_for()`), so it crawls to the 3 cm line: up to 10 s at 3 cm/s,
  the brain's HTTP timeout. The explore escape creeps only toward an end
  the 20 cm bar already refuses (at most 17 cm, ~6 s). Whether the Rover's
  board holds 0.75 rad/s wheels smoothly is a hardware-day check.
- **Late safety ticks on the Jetson** are measured only against the fake
  motor board: 0 in 14,289 under full load (3.33, with 3.37). With the real
  lidar driver and serial board in the process they are unmeasured.
