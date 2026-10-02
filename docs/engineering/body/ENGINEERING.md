---
kind: engineering
domain: body
status: current
verified: 2026-10-02
parent: docs/body/ARCHITECTURE.md
---

# Body -- engineering

How the body contract and its backends are built today. The what and why
(one contract, body vs world, honest defaults, wheel velocities as the
primitive) are in the [architecture spec](../../body/ARCHITECTURE.md). This
document is true only until the implementation changes, and is updated in
the same commit that changes it.

## Implementation

| File | What it holds |
|---|---|
| `robot/interface.py` | `RobotInterface` (ABC), the honest defaults (`unusable_grid()`, `unusable_odometry()`, `unusable_wheels()`, `unusable_scan()`), zone statuses, `NO_SENSOR_CM`, the driver-rank table used by [safety](../../safety/ARCHITECTURE.md) (`DRIVER_PRIORITY`, `driver_priority()`, `Preempted`), and `carry_out_verb()`, the one loop every guarded or unguarded plan-based verb runs through |
| `robot/factory.py` | `get_robot()`: picks the backend (`_backend()`), builds the simulated house (`_sim_world()`), and wraps it for ROS drive if asked (`_with_drive()`) |
| `sim/mock_robot.py` | `MockRobot`: the sim body. Wheel velocities integrated by `step()` with exact differential-drive kinematics, sub-stepped against the grid world's collision. Camera via `sim/renderer.py`, an 8-column depth grid, a 360-beam scan from the lidar's mount point |
| `control/remote_robot.py` | `RemoteRobot`: the contract over `robot/server.py`'s HTTP API, sending `x-driver` (default `brain`) and `x-app-secret`. `RobotTransportError` for "nobody heard it" |
| `sim/replay_robot.py` | `ReplayRobot`: a directory of images, one frame per movement verb; `loop=True` wraps at the end. Open loop. Not selectable from the factory; constructed directly, by `tests/demo_replay_mission.py` and the tests |
| `sim/teleop_robot.py` | `TeleopRobot`: the latest frame pushed through `POST /teleop/frame`; verbs are acks; `TeleopStall` once the last push is older than the stall timeout |
| `robot/hardware_robot.py` | `HardwareRobot`: the motors over the ESP32 board's serial line (protocol: [motor-board engineering](../motor-board/ENGINEERING.md)). Sensing comes from an optional `sensors` body; without one, the defaults (and `get_distance()` 0.0, so a forward move is always vetoed) |
| `robot/ros_drive.py` | `RosDriveRobot`: wraps any backend under `drive: ros`. Verbs become `POST /cmd_vel` twists closed on the wrapped body's encoders; reads go to the wrapped body. Marks itself `drives_by_velocity = True`. `stop()` is described below |
| `control/mission_runner.py` | `_HaltGate`: wraps the mission's body and raises `MissionHalted` on any motion once the mission has ended. Owned by [mission](../../mission/ARCHITECTURE.md) and listed here because the conformance suite runs it |
| `tests/conftest.py` | `RecordingRobot`, a test double, and the `robot_over_asgi` fixture (a `RemoteRobot` over an in-process robot server) |

### The ROS drive stop

`RosDriveRobot.stop()`, since 2026-10-02 (`docs-review/SPEC-REVIEW.md`
finding 1), in order:

1. Bumps the verb generation, so any verb in flight stops streaming twists,
   and arms a **stop hold** until `time.monotonic() + STOP_HOLD_S`.
2. Calls `inner.stop()`, the wrapped body's own stop, and returns its result.
   Nothing before this step touches the network.
3. If no zeroing is already running (a non-blocking `threading.Lock`),
   starts one daemon thread named `ros-stop-zero` that posts a zero
   `/cmd_vel` for `twin-dpad`, `brain` and `ros`, each with
   `STOP_ZERO_TIMEOUT_S`, swallowing every error. The watchdog calls
   `stop()` every 0.1 s poll while the robot is silent, so this
   single-flight rule is what keeps a hung bridge from accumulating threads.

While the hold is in force (same generation as the stop, and before its
deadline), `set_wheel_velocity()` turns any **non-zero** command into a zero
on the wrapped body. That is the stopped verb's last twist still on its way
through `twist_mux`, `diff_drive_controller` and the plugin's `POST /wheels`.
The next verb (`_begin()`, a new generation) lifts the hold at once. Before
this change the three posts ran first, each with the client's 2.0 s timeout,
so a bridge that accepted connections and never answered held a stop for
about 6 s, on the server's event loop when the watchdog called it.

## Interfaces

### Methods

Every backend implements the abstract ones. The non-abstract ones have the
default shown.

| Method | Returns | Default |
|---|---|---|
| `drive_forward(speed, duration)` / `reverse(speed, duration)` | dict with `action` named after the method | abstract |
| `turn_left(angle)` / `turn_right(angle)` | dict with `action` | abstract |
| `look_left()` / `look_right()` / `look_center()` | dict with `action` | abstract |
| `stop()` | dict; zeroes any standing wheel command and increments `stop_count` where the backend keeps one | abstract |
| `get_camera_frame()` | `{"image_base64", "media_type", "room", "metadata", ...}`; may raise | abstract |
| `get_distance()` | float, cm | abstract |
| `get_depth_grid()` | `{"rows", "cols", "fov_deg"?, "pan_deg"?, "zones": [{"status", "distance_cm"}]}`, row-major | `unusable_grid()`: 1 x 8, all `unusable`, no `fov_deg` |
| `get_odometry()` | `{"usable", "distance_m", "heading_deg"}`. Path length, never decreasing. Heading is clockwise-positive on both moving bodies, but its zero differs: `HardwareRobot` reports turn since start (0.0 at start); `MockRobot` reports the grid world's compass bearing (90.0 at start in the starter house). See Known gaps | `unusable_odometry()` |
| `get_wheel_state()` | `{"usable", "left": {"position_rad", "velocity_rad_s", "counts"}, "right": {...}, "wheel_radius_m", "track_width_m", "counts_per_rev"}` | `unusable_wheels()` |
| `get_scan(max_range_m=None)` | `{"usable", "angle_min_deg", "angle_increment_deg", "range_min_m", "range_max_m", "ranges_m"}`. Angles relative to the body, clockwise-positive, 0 ahead; `None` per beam = no return | `unusable_scan()` (`ranges_m: None`) |
| `set_wheel_velocity(left_rad_s, right_rad_s)` | dict; a standing command | raises `NotImplementedError` |
| `advance(dt)` | lets `dt` s of the standing command elapse (the sim integrates; the hardware body re-sends it) | no-op |
| `verb_plan(action, speed, duration, angle)` | `{"kind": "straight"|"turn", "left_rad_s", "right_rad_s", "target", "wall_clock", "settle"?}` or `None` | `None` |
| `verb_done(action, plan, outcome, **kwargs)` | the dict the backend's own verb would have returned | raises |

Zone statuses: `ZONE_RANGE` (`"range"`, a number), `ZONE_NO_TARGET`
(`"no_target"`, `None`), `ZONE_UNUSABLE` (`"unusable"`, `None`). Column `i`
of `cols` points at `-fov/2 + fov*(i+0.5)/cols` degrees.

`carry_out_verb(robot, plan, limit=None)` returns `{"done", "ended",
"reason"}`, where `ended` is one of `complete`, `clamped`, `stopped`,
`stalled` or `timeout`. Each period it checks `stop_count`, reads the
encoders, asks `limit(amount)` (the safety layer's hook), commands the
wheels, then sleeps (`wall_clock`) or calls `advance()`.

### Which backend implements what

| | sim | remote | replay | teleop | hardware | ROS wrapper |
|---|---|---|---|---|---|---|
| pixels | rendered JPEG | server's | file | pushed | `sensors` or raises | wrapped |
| `get_distance()` | free cells x 30 cm (or sensor model) | server's | `NO_SENSOR_CM` | `NO_SENSOR_CM` | `sensors` or 0.0 | wrapped |
| depth grid | 1 x 8, 60 deg, `pan_deg` | `/depth` (404 -> unusable) | default | default | `sensors` or default | wrapped |
| odometry / wheels | integrated | `/odometry`, `/wheels` (404 -> unusable) | default | default | board feedback | wrapped |
| scan | 360 x 1 deg, 12 m | `/scan` (404 -> unusable) | default | default | `sensors` or default | wrapped |
| `set_wheel_velocity` | yes | not overridden (raises) | raises | raises | `T:1` | wrapped |
| `verb_plan` | yes, `wall_clock: False` | `None` | `None` | `None` | yes, `wall_clock: True`, `settle` on fork firmware | `None` |

### Remote body: refusal mapping

`RemoteRobot._action()` reads `{"executed": false, "reason"}` from
`POST /action`: `preempted` raises `Preempted`; `ros_unavailable` raises
`RobotTransportError("ros_unavailable: ...")`, which ends a mission
`failed`; anything else, including no reason (a pre-M4 server), raises
`SafetyViolation`. HTTP 400 raises `ValueError`; any other status of 400 or
above raises `RobotTransportError`. Sensing routes catch only `HTTP 404`.

## Parameters and configuration

### Selection (read by `robot/factory.py`)

| Key / variable | Default | Effect |
|---|---|---|
| `mode` / `ROBOT_MODE` | `sim` | `sim`, `teleop` or `hardware`; anything else is a `ValueError`. The env var wins |
| `drive.mode` / `ROBOT_DRIVE` | `direct` | `ros` wraps the backend in `RosDriveRobot` |
| `drive.bridge_url` / `ROS_BRIDGE_URL` | `http://127.0.0.1:8090` | Where the ROS wrapper posts twists |
| `SIM_MAP` | `starter_house` | Which house `sim/maps/` builds |
| `SIM_MOVERS` | unset | People/pets scenario added to the house |
| `sim.realtime` | `false` | Sim verbs sleep their declared duration (S4) |
| `sim.sensor_noise.*` | `enabled: false` | `DistanceSensorModel`: `stddev_cm` 3.0 in the shipped yaml (code default 0.0 when the key is absent), `dropout_rate` 0.0, range 2-400 cm, `read_latency_s` 0.0 (S5) |
| `sim.odom_drift.*` / `SIM_ODOM_DRIFT` (`"left,right"`) | `enabled: false`, right 1.03 | Encoder scale per wheel, for SLAM tests (R5) |
| `teleop.stall_timeout_s` | 15.0 s | Teleop staleness window |
| `ROBOT_SERIAL` / `hardware.serial_port` | none (required) | The board's serial device under `mode: hardware` |
| `SIM_MOTOR_BOARD=fake` | unset | `mode: hardware` against `sim/fake_esp32.py` on a pty, with a sim body as `sensors` |

`config/robot.yaml` also carries a top-level `sim_map:` key that **no code
reads**; `SIM_MAP` is the only selector.

### Constants

| Constant | Value | Where | Why |
|---|---|---|---|
| `NO_SENSOR_CM` | 999.0 cm | `robot/interface.py` | Far above any `min_distance_cm`, so a photograph body is never vetoed |
| `DEPTH_COLS_DEFAULT` | 8 | `robot/interface.py` | A VL53L5CX row; the twin's strip width |
| `VERB_PERIOD_S` | 0.05 s | `robot/interface.py` | One wheel-loop period (`robot/server.py`) |
| `VERB_TURN_STEP_DEG` | 5.0 deg | `robot/interface.py` | Most a turn rotates between checks; the sim's collision sub-step |
| `WHEEL_RADIUS_M` | 0.040 m | `sim/mock_robot.py`, `robot/hardware_robot.py` | UGV Rover firmware, mainType 2 (3.21/3.25) |
| `TRACK_WIDTH_M` | 0.172 m | same | Rover firmware; effective skid-steer track still to measure (R8) |
| `ENCODER_COUNTS_PER_REV` / `COUNTS_PER_REV` | 660 | same | 11 lines x 2 x 30:1 (3.25) |
| `CELLS_PER_SECOND_AT_FULL_SPEED` / `MOVES_PER_SECOND_AT_FULL_SPEED` | 2.0 | sim, hardware, ROS wrapper | `speed` 0-100 as a fraction of 2 moves/s; `duration` rounds to whole moves, at least one |
| `DEFAULT_CELL_M` / `MOVE_M` | 0.30 m | same | One move. Every step budget is in these |
| `LIDAR_RANGE_M` | 12.0 m | `sim/mock_robot.py` | RPLidar C1 and D500 rating; the 4.2 m camera horizon mapped nothing in the home (3.17) |
| `LIDAR_X_M` | 0.040 m | `robot/safety.py`, read by the sim | Scan origin 4 cm ahead of `base_link` (3.27) |
| `DEFAULT_TIMEOUT_S` | 10.0 s | `control/remote_robot.py` | Per-request timeout |
| `DEFAULT_STALL_TIMEOUT_S` | 15.0 s | `sim/teleop_robot.py` | Teleop staleness |
| Hardware verb turn rate | 1.2 rad/s body | `robot/hardware_robot.py` | Same as `TURN_RATE_RAD_S` in `robot/ros_drive.py` |
| `HEARTBEAT_MS` | 1500 ms | `robot/hardware_robot.py` | Board-side deadman, above the server's 1.0 s watchdog |
| `RosDriveRobot` client timeout | 2.0 s | `robot/ros_drive.py` (`timeout_s`) | Every bridge call except the stop's zeroing posts |
| `STOP_ZERO_TIMEOUT_S` | 0.5 s | `robot/ros_drive.py` | Per zeroing post after a stop; a hung bridge costs the background thread at most 1.5 s and the caller nothing |
| `STOP_HOLD_S` | 0.4 s | `robot/ros_drive.py` | `twist_mux`'s 0.25 s input timeout (`service/slam/src/picar_bringup/config/twist_mux.yaml`) plus one 0.05 s plugin period, with margin: how long the stopped verb's last twist can keep reaching the wheels if the zeros never arrive |

The chassis constants are kept equal across the sim, the hardware body,
the xacro and `controllers.yaml` by `tests/test_wall_linters.py` and
`tests/test_urdf.py`.

## Procedures

**Run the conformance suite** (six bodies: `mock`, `replay`, `teleop`,
`remote`, `halt_gate`, `hardware` over the fake board):

```bash
.venv/bin/pytest tests/test_robot_contract.py -q
```

Expected on 2026-10-02: `130 passed, 4 skipped`. The skips are the two
odometry-motion tests on replay and teleop ("backend reports no odometry,
which this suite allows"). A `hardware` fixture failure that times out
waiting for frames means the pty fake board did not start.

**Start the robot server on a given body:**

```bash
uvicorn robot.server:app --port 8000                                  # sim, starter house
SIM_MAP=scaled_house uvicorn robot.server:app --port 8000             # sim, 90 cm doors
ROBOT_MODE=teleop WORLD_MODE=none uvicorn robot.server:app --port 8000
ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake uvicorn robot.server:app --port 8000
ROBOT_MODE=hardware WORLD_MODE=none ROBOT_SERIAL=/dev/serial/by-id/<the board> \
  uvicorn robot.server:app --port 8000
```

`curl -s localhost:8000/health` reports `mode`, and `sim_map` names the
house that was built for any body standing in a sim house (the sim, and
`mode: hardware` with `SIM_MOTOR_BOARD=fake`); it is null otherwise. A body
with no sim house behind it (`mode: teleop`, or `mode: hardware` on a real
board) fails at start-up under the shipped `world.mode: sim`: the world
factory refuses a sim world for a body with no grid. Set `WORLD_MODE=none`
(see [world](../world/ENGINEERING.md)). On the car, `ROBOT_SERIAL` names a
stable device (a `/dev/serial/by-id/` path or a udev symlink), never
`/dev/ttyUSB0` by enumeration order: the board and the lidar can both
enumerate as CP210x (`HARDWARE-BOM.md` 4.2). `EACCES` from `os.open` means
the user is not in `dialout`.

**Add a sensing method to the contract:**

1. Add it to `RobotInterface` with an `unusable_*()` default. Do not make it
   abstract.
2. Override it in every body that has the sensor, and in every wrapper
   (`RemoteRobot` with the 404 rule, `RosDriveRobot`, `_HaltGate`,
   `HardwareRobot` delegating to `sensors`).
3. Add a route to `robot/server.py` (see
   [control-api](../control-api/ENGINEERING.md)).
4. Add a shape test to `tests/test_robot_contract.py`, plus a "wrapper
   reports what it wraps" test.

**Add a backend:** add a branch to `robot/factory.py`'s `_backend()`, add it
to `BACKENDS` in `tests/test_robot_contract.py` (with a fixture branch that
builds it), and run the suite. Expected: 22 more tests collected (one per
parametrized contract test) and all of them passing, or skipping only with
"backend reports no ..., which this suite allows". A failure naming a shape
key means the new body returns something the contract does not allow.

## Verification

| Test | What it pins |
|---|---|
| `tests/test_robot_contract.py` (134 collected) | Return shapes, units, `stop()` idempotency, decodable pixels plus media type, depth tri-state, odometry monotonic and pivot-invariant, wheels/scan shapes, wrappers reporting what they wrap |
| `tests/test_remote_robot.py` (10) | The same mission gives an identical action sequence in-process and over HTTP; refusal mapping |
| `tests/test_teleop_robot.py` (8) | Staleness, the non-blocking read, push/pull |
| `tests/test_config_and_factory.py` (22) | Mode and drive selection, env overrides, errors on an unknown mode |
| `tests/test_ros_drive.py` (14) | The ROS wrapper's verbs, encoder closure, reads to the wrapped body; the stop: direct and under 0.1 s against a bridge that hangs, all three ROS inputs zeroed in the background, no thread pile-up over 20 stops, a stale ROS command held at zero, and the hold lifted by the next verb or by `STOP_HOLD_S` |
| `tests/test_blind_reverse.py` (5) | `HardwareRobot` with no sensors refuses reverse; a body without wheels still reverses (rule in [safety engineering](../safety/ENGINEERING.md)) |
| `tests/test_health_sim_map.py` (4) | `/health` `sim_map` names the house the factory built, including `mode: hardware` with the fake board |
| `tests/test_fake_esp32.py`, `tests/test_ros_driver_board.py` | The hardware body against the fake board (detail in [motor-board](../motor-board/ENGINEERING.md)) |
| `tests/test_continuous_pose.py` | The sim's wheel kinematics; a straight line independent of the track width |
| `tests/test_brain_server.py` | The brain imports no backend (a subprocess `sys.modules` check) |
| `tests/test_wall_linters.py` | Chassis constants and `LIDAR_X_M` agree across the ROS wall |

Change checklist: the contract suite passes for all six bodies. A new field
is added to the remote body and every wrapper. No body returns a number it
did not measure.

## Known gaps

- **No camera, lidar or depth driver on the car yet.** `HardwareRobot`
  without `sensors` raises on `get_camera_frame()` and answers 0.0 for
  `get_distance()`. The lidar driver is decided for the robot process but
  not ported.
- **Stale comments in code:** `sim/mock_robot.py` around line 186 says the
  wheel methods "are NOT on `RobotInterface`" (they are, since R2).
  `robot/interface.py`'s `get_depth_grid()` docstring says "Nothing in
  `brain/` reads this yet"; `brain/agent.py` reads it through
  `forward_clearance()`. `robot/interface.py`'s module docstring says
  `hardware_robot.py` is "added in Phase 11"; it exists (R7).
- **Dead config key:** `config/robot.yaml`'s `sim_map:` is never read.
- **Wheel slip is not modelled**: sim encoders count what the body
  achieved (`PLAN-ros-alignment.md` section 4).
- **Pan is three positions**, and the sim's depth grid swings with it
  (`pan_deg`). A real pan-tilt's angle is not in the contract.
- **The stop race in `carry_out_verb()`** (`robot/interface.py:241-271`).
  It checks `stop_count`, then calls `set_wheel_velocity()`; a `stop()`
  between the two is overwritten, and on the next pass the loop sees the
  stop and skips its `finally` zeroing. The wheels then run until the
  watchdog stops them (about 1 s). `/stop` does not take `motion_lock`.
  Read, not run (`docs-review/SPEC-REVIEW.md` section 5, fix-list 7). Fix:
  re-check `stop_count` after commanding and zero if it changed.
- **`RosDriveRobot` is not in the conformance suite**: `BACKENDS` has six
  entries (`tests/test_robot_contract.py:85`). Its own tests in
  `tests/test_ros_drive.py` cover verbs, reads and the stop, not every shape.
- **Odometry heading has two zeros**: `MockRobot` reports the compass
  bearing (`sim/mock_robot.py:710`), `HardwareRobot` the turn since start
  (`robot/hardware_robot.py:371`), and the contract suite pins neither.
  Differences agree; absolute headings do not. Pending a decision
  (fix-list 11).
- **Before the board's first feedback frame**, `HardwareRobot` reports its
  wheels unusable (`robot/hardware_robot.py:349`), so the safety layer's
  wheel vet passes commands through and the blind-reverse rule does not
  apply (see [safety engineering](../safety/ENGINEERING.md), Known gaps).
- **`RemoteRobot` does not override `set_wheel_velocity()`**, so the brain
  cannot send a standing command over HTTP. Nothing needs it today.
