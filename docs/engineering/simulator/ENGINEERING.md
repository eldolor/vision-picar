---
kind: engineering
domain: simulator
status: current
verified: 2026-10-04
parent: docs/simulator/ARCHITECTURE.md
---

# Simulator -- engineering

How the simulator is built today. The what and the why are in the
[architecture spec](../../simulator/ARCHITECTURE.md). This document is true
only until the implementation changes, and it is updated in the same commit
that changes it.

## Implementation

Everything lives under `sim/`. Under `mode: sim` it runs in the robot
server's process (or in a test's process). Under `mode: hardware` with
`SIM_MOTOR_BOARD=fake` it runs in **separate programs** and never in the
robot server's process (`PLAN-ros-alignment.md` 3.36): one physics program
(`sim/body_server.py`) and one or more sensor programs
(`sim/sensor_server.py`), joined by a shared-memory snapshot. On the Jetson
the in-process simulator held the robot server at 98% of one core and its
20 Hz wheel loop ran late on 15-20% of ticks; in one separate program the
sensor reads queued behind the board loop. See "The split simulator" below.

| File | What it does |
|---|---|
| `sim/grid_world.py` | `GridWorld`: the house layout (list of strings, `#` wall, `.` floor, `D` door), rooms, objects, movers and the sim clock. Holds the **continuous pose** `x`, `y` (cell units, fractional) and `theta` (radians). `robot_x` / `robot_y` / `heading` are views of it: the getters floor, the setters snap to the cell centre. `translate()` and `rotate()` are the motion primitives and return what the geometry allowed. `move_object()`, `add_mover()` and `advance_time()` are 3.30's moving world. `frame_description()` and `_detections()` produce simulated perception |
| `sim/renderer.py` | The raycaster. `cast_ray()` (fixed-step march), `cast_ray_exact()` (Amanatides and Woo grid traversal, 3.18), `wall_profile()`, `visible_bearing()` (3.32), `render()` / `render_jpeg()` / `render_world_base64()`. Carries the fidelity note at the top of the file |
| `sim/mock_robot.py` | `MockRobot`, the sim backend of the body contract. Wheel primitive `set_wheel_velocity()` / `step()` / `get_wheel_state()`. Verbs `drive_forward()` / `reverse()` / `turn_left()` / `turn_right()` via `verb_plan()` + `carry_out_verb()` + `verb_done()`. Sensors `get_camera_frame()`, `get_depth_grid()`, `get_scan()`, `get_odometry()`, `get_distance()`. Time: `pass_time()` and `advance()` |
| `sim/sensors.py` | `DistanceSensorModel` (S5): Gaussian noise, dropout, range clamp and read latency on `get_distance()`, plus per-zone dropout on the depth grid |
| `sim/movers.py` | `Mover` (a named solid object on a closed path of 4-adjacent cells), `MOVER_KEEPOUT_M`, `point_to_cell_cells()` |
| `sim/body_server.py` | 3.36's physics program (FastAPI app `sim.body_server:app`). Builds the house, `MockRobot` and `FakeEsp32` with `robot.factory.build_fake_body()`, so it reads `SIM_MAP`, `SIM_MOVERS`, `SIM_BOARD_FIRMWARE` and `SIM_BOARD_SILENT_S` as the in-process build did. Owns kinematics, collision, movers, furniture moves, the camera pan and the board's pty. A thread publishes the body state every `PUBLISH_S`, under the board's lock, so a snapshot is one instant |
| `sim/body_state.py` | The shared-memory snapshot, `StateWriter` (the body program, the one writer) and `StateReader` (each sensor program). Pose, pan, sim clock, house name and the objects (re-encoded only when the dict is replaced). A sequence number (odd mid-write) and a CRC32 of the payload: a reader retries until both sides of its copy show the same even sequence and the CRC matches, which is what makes it safe on the Jetson's weakly ordered Arm cores without a cross-process lock. Attaches without the resource tracker (`track=False` on 3.13, an unregister on 3.10), so a sensor program exiting never unlinks the body's block |
| `sim/sensor_server.py` | 3.36's sensor program (`sim.sensor_server:app`). Each process keeps a replica `MockRobot` of the house, moves it to the latest snapshot, and casts with the same methods the in-process sim uses, so a reading is the reading the body would have produced at that pose. Stateless apart from the snapshot, so several run side by side |
| `sim/fake_lidar.py` | 3.42: the D500 on a pty. `Emitter` casts every point at its own instant (`cast_ray_exact()` from the lidar's position, 450 points a revolution, 10 Hz) and encodes LD19 packets; `world_caster()`; `LossyLine` drops and corrupts bytes; the program reads the body state from shared memory and links `--link` (`ROBOT_LIDAR`) to its pty. Shared with `tests/footprint_sweep.py`'s `_LidarTimed` |
| `sim/body_client.py` | The robot server's side: `SimBodyClient`, handed to `HardwareRobot` as `sensors`, and `RemoteGrid` (its `world`: the house's name, its objects, moving one, the truth). Imports nothing of the simulator, so the robot server's process holds no `GridWorld` (`tests/test_sim_body_process.py`) |
| `sim/maps/__init__.py` | `build_world(name)` and `build_movers(house, scenario)`: the only map registry. `build_world()` stamps the name it was given on the world it returns (`world.map_name`), which is how the server reports the house it built |
| `sim/maps/starter_house.py` | 13 x 10 cells, 30 cm doors. Robot at cell (2, 2) facing east, backpack at (10, 7). The default house |
| `sim/maps/scaled_house.py` | 27 x 14 cells, 90 cm doors (R6). Robot at (6, 4) facing east, backpack at (23, 5). The only house with a mover scenario (`hallway_crossing`) |
| `sim/maps/home_first_floor.py` | The user's first floor, rasterised from feet (appraisal sketch, exterior measured, interior provisional), furnished with solid objects. Tables are four legs |

Owned by other domains but built on this one: `sim/mock_world.py` (world),
`sim/fake_esp32.py` (motor-board; since 3.36 it runs only in the body
program), `sim/replay_robot.py` and
`sim/teleop_robot.py` (body backends with no grid).

**How the backend is built.** `robot/factory.py`'s `_sim_world()` reads the
house name from `SIM_MAP` (the only place outside `tests/` that reads it),
passes it to `build_world()` and adds any movers. The same `_sim_world()`
builds the house for `build_fake_body()`, which `sim/body_server.py` calls
under `SIM_MOTOR_BOARD=fake`; the robot server itself then builds only a
`SimBodyClient` (and refuses to start without `SIM_BODY_URL` and
`SIM_SENSORS_URL`). Under `mode: sim`, `_backend()` builds a `MockRobot` with the `sim:` block's `realtime`, `sensor_noise` and
`odom_drift` settings. The house is `SIM_MAP`, else `config/robot.yaml`'s
top-level `sim_map` (a map name; read since handoff 4f), else the starter
house.

**One tick of motion** (`MockRobot.step(dt)`). With wheel speeds
`v_left`, `v_right` (m/s), `v = (v_right + v_left) / 2` and
`omega = (v_right - v_left) / track`, counter-clockwise positive (REP-103).
The grid's `theta` increases clockwise, so `step()` is the one place the
sign flips. The tick is split into sub-steps of at most 0.1 cell and 5
degrees. Each sub-step rotates (`GridWorld.rotate()`), then translates
along the new heading (`GridWorld.translate()`), then advances the
encoders from what was **achieved**. A blocked wheel does not count. After
the motion, `GridWorld.advance_time(dt)` lets due movers hop.

**Collision.** `translate()` caps travel with a head-on ray from the centre
(the half-cell body reach) and checks the chassis rectangle (the URDF
footprint from `robot/safety.py`, less a 0.2 cm skin) at the end of every
stretch of at most half a cell, and `step()` already splits every tick into
sub-steps (`MAX_SUBSTEP_CELLS`, `MAX_SUBSTEP_RAD`), so an arc cannot cut a
corner between checks. `rotate()` stops where a corner would first touch
(3.19). A pose already in contact is exempt, so a robot against something
can still move away. `GridWorld.objects` is replaced as a whole on every
change and never mutated, because the scan is read on another thread.

**Two ray casts.** The picture, the depth grid and the discovered map use
`cast_ray()`, a fixed-step march (`FPV_STEP`). The safety layer's short
scan (`get_scan(max_range_m=...)`) uses `cast_ray_exact()`, an exact grid
traversal: the march slipped past diagonal cell corners on 0.1-0.2% of
beams, and the exact cast took that scan from about 13 ms to 0.4 ms (3.18).
The march stays where it is because the golden image and the pinned
traces depend on it.

**Simulated perception.** `frame_description()` returns `room`,
`objects_visible` (objects within 3 cells, for the rule-based policy) and
`detections`: every object a ray can reach past walls and other objects, at
the bearing of its visible part (`visible_bearing()`, 9 rays across the
object), nearest first, with `label`, `bearing_deg` and `distance_m`.
`get_camera_frame()` adds `image_base64`, `media_type` and
`metadata.source = "sim"`. The source tag is what makes
`brain/perceive.py` choose `FrameReportedPipeline` rather than a real
detector.

**The split simulator** (3.36). Under `SIM_MOTOR_BOARD=fake`:

- The **body program** advances the body with the board loop (~100 Hz)
  and publishes the snapshot every 5 ms. It serves the sim extras the robot
  server's routes forward: the pan, the truth and the object table.
- Each **sensor program** casts from the newest consistent snapshot.
  `GET /safety?max_range_m=` returns the hinted scan, the depth grid and the
  distance cast from ONE snapshot, so the vet never mixes two instants.
- **`SIM_SENSORS_URL` is ordered.** The FIRST program serves the safety
  bundle, polled at the wheel loop's 20 Hz by a background thread in
  `SimBodyClient`; the LAST serves the heavy reads, ROS's unhinted full
  scans and camera frames. The two loads then land on different cores by
  construction: uvicorn workers sharing one socket did not share the load
  (on the Jetson one of three took 61% of a core and two sat idle).
- **The safety layer's reads never touch the network.** `get_scan()` with a
  range hint, `get_depth_grid()` and `get_distance()` answer from the latest
  bundle. The first build made the vet an HTTP round trip inside
  `motion_lock`, and the wheel loop skipped most of its ticks.
- **A bundle older than `SENSOR_STALE_S` (0.15 s) is not a reading:** the
  scan answers `usable: false`, the zones unusable and the distance `0.0`,
  the existing fail-safe path, so a dead sensor program stops forward motion
  within the bound plus one wheel-loop period. The client never raises into
  the wheel loop. `sensor_age_s()` reports the bundle's age, and the safety
  layer takes the way covered since off the clearance (`SafetyController._aged()`,
  safety engineering).
- The robot server opens the board's pty (`board_path` from the body
  program's `/health`) as a serial device, as on the car, and waits up to 30
  s for every program's `/health` before it does.

## Interfaces

The simulator implements the body contract (`robot/interface.py`). What
follows is the sim-specific shape of each answer. The routes are served by
`robot/server.py` (control-api).

| Call | Returns (sim) |
|---|---|
| `get_wheel_state()` | `usable: true`; `left` / `right` each with `position_rad`, `velocity_rad_s` and `counts` (660 per rev); `wheel_radius_m`, `track_width_m`, `counts_per_rev`. Positions are scaled by `encoder_scale` (drift) |
| `step(dt)` | `moved_m`, `moved_cells`, `turned_deg`, `blocked`, plus the wheel state |
| `get_depth_grid()` | `rows: 1`, `cols: 8`, `fov_deg: 60`, `pan_deg`, `zones[]` of `{status, distance_cm}`. Status is range, no-target (ray reached the horizon) or unusable (dropout, only with a sensor model). Cast along the **view** angle. Clearance is reduced by half a cell and one march step so it never overstates |
| `get_scan(max_range_m=None)` | 360 beams, 1 degree apart, `angle_min_deg: -180`, clockwise-positive, zero dead ahead, cast from the **lidar** 4.0 cm ahead of centre along the **body** heading. `range_max_m: 12.0`. A beam with no return is `null`. With a range hint (the safety layer's short scan) the beams use `cast_ray_exact()` and stop at the hint |
| `get_odometry()` | `usable: true`, `distance_m` (path length actually covered, reverse included), `heading_deg` (body, degrees turned since the robot was built, clockwise-positive, continuous; the compass bearing is `MockWorld.get_pose()`'s) |
| `get_distance()` | Free cells ahead times 30.0 cm, exact. Through `DistanceSensorModel.read()` when noise is on; a dropout reads `0.0` (fail-safe: always trips the veto) |
| `get_camera_frame()` | `room`, `objects_visible`, `detections`, `image_base64` (320 x 200 JPEG), `media_type: image/jpeg`, `metadata` |
| `GridWorld.move_object(src, dst)` | Raises `ValueError` when there is no object at `src`, for a mover, onto a cell that is not floor, onto another object, or inside the robot's turning circle |

Sim-only routes on the robot server (501 on a backend with no house):

| Method, path | Body | Response |
|---|---|---|
| `GET /sim/objects` | -- | `{sim_time_s, objects: [{x, y, name, mover}]}` |
| `POST /sim/objects/move` | `{src: [x, y], dst: [x, y]}` | The same listing; 409 naming the reason for an impossible move; 422 for a malformed cell |

Routes of the split simulator's programs (3.36; all but `/health` behind
`APP_SHARED_SECRET` when it is set):

| Program, method, path | Response |
|---|---|
| body, `GET /health` | `{ok, identity, board_path, sim_map, state_shm, board_frames_out, state_publishes}` |
| body, `POST /look/{side}` | Pans the camera (`left`, `right`, `center`; 404 otherwise) and publishes at once, so the next sensor read sees the pan |
| body, `GET /truth` | `MockWorld.get_truth()` over the body's house: what the robot server's `/world/truth` answers with |
| body, `GET /sim/objects`, `POST /sim/objects/move` | As on the robot server (both compute it with `GridWorld.describe_objects()`) |
| sensors, `GET /health` | `{ok, pid, house, seq}`; `ok: false` while no consistent snapshot is readable |
| sensors, `GET /scan`, `/depth`, `/distance`, `/frame` | One reading each, cast from the latest snapshot |
| sensors, `GET /safety?max_range_m=` | `{scan, depth, distance_cm, state_seq, pid}` from one snapshot: the bundle `SimBodyClient` polls |

`GET /health` on the robot server carries `sim_map`: the `map_name` of the
world the factory built, read as `robot.world.map_name` (the ROS-drive
wrapper forwards `world`). It is right for every body standing in a sim
house, including `ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake` (read off
`RemoteGrid`, from the body program's `/health`), and `null` when
no sim house stands behind the robot (teleop, replay, the real board).
Changing `SIM_MAP` after start-up does not change it. Until 2026-10-02 it
re-read `SIM_MAP` only under `mode: sim`, so the fake-board configuration
reported `null` and the live suites skipped. `tests/test_health_sim_map.py`
pins both.

## Parameters and configuration

"Default" is the code default; where the shipped `config/robot.yaml`
differs, both are given.

| Key or constant | Default | Unit | Read in | Why that value |
|---|---|---|---|---|
| `SIM_MAP` (env) | `starter_house` | name | `robot/factory.py` (`_sim_world()`); the live suites and sweeps in `tests/` read it too | Most tests were measured in the starter house. `scaled_house` for nav2 and R1 sweeps; `home_first_floor` for the user's house |
| `SIM_MOVERS` (env) | unset | scenario name | `robot/factory.py` | Off means off (3.30 criterion 4). Only `scaled_house` defines one, `hallway_crossing` |
| `SIM_ODOM_DRIFT` (env) | unset | `left,right` scales | `robot/factory.py` | Overrides the yaml, as `ROBOT_DRIVE` does |
| `sim.realtime` | `false` | bool | `robot/factory.py` -> `MockRobot._settle()` | S4. On, a move sleeps its declared `duration` so the watchdog readout climbs mid-move. Off keeps the suite fast |
| `sim.sensor_noise.enabled` | `false` | bool | `robot/factory.py` | S5. Off keeps `get_distance()` an exact multiple of 30 cm, which existing tests depend on |
| `sim.sensor_noise.stddev_cm` | `0.0` in code, `3.0` in the yaml | cm | `robot/factory.py` -> `DistanceSensorModel` | With the 20 cm collar, 3.3 sigma clear of one cell (`CLAUDE.md` section 7, S5 row) |
| `sim.sensor_noise.dropout_rate` | `0.0` | fraction | `DistanceSensorModel` | Raise it to show M3's tri-state; 0.2 is the twin demo |
| `sim.sensor_noise.min_range_cm` / `max_range_cm` | `2.0` / `400.0` | cm | `DistanceSensorModel` | HC-SR04's datasheet range |
| `sim.sensor_noise.read_latency_s` | `0.0` | s | `MockRobot.get_distance()` | Only slept when `sim.realtime` is also on |
| `sim.odom_drift.enabled` / `left_scale` / `right_scale` | `false` / `1.0` / `1.0` in code; the yaml sets `right_scale: 1.03` | ratio | `robot/factory.py` | R5: a right encoder 3% long. Odometry then ends up to 99 cm / 54 deg off over a lap while SLAM stays within 1-4.5 cm (3.14) |
| `WHEEL_RADIUS_M`, `TRACK_WIDTH_M`, `ENCODER_COUNTS_PER_REV` | see platform | m, m, counts | `sim/mock_robot.py` | Chassis constants. Values and sources are canonical in the platform engineering spec's chassis table (docs/engineering/platform/ENGINEERING.md), which also lists what pins each copy: radius and track by `tests/test_wall_linters.py` and `tests/test_urdf.py`, the encoder count only by `tests/test_ros_driver_board.py`. In the sim, the track scales pivot rate only: a straight line does not depend on it |
| `CELLS_PER_SECOND_AT_FULL_SPEED` | 2.0 | cells/s | `sim/mock_robot.py` | Keeps a default `drive_forward()` at one cell. `WHEEL_MAX_RAD_S` (15 rad/s here) is derived from it, not from motor rpm |
| `DEFAULT_CELL_CM` | 30.0 | cm | `sim/mock_robot.py`, `sim/sensors.py` | One grid cell. Every verb, collar and step budget is measured against it |
| `LIDAR_RANGE_M` | 12.0 | m | `MockRobot.get_scan()` | D500 and RPLidar C1 rated range. The renderer's 4.2 m horizon left SLAM mapping almost nothing in the home |
| `LIDAR_X_M` | see platform | m | imported from `robot/safety.py` | The scan origin, ahead of the rotation centre. Canonical value in the platform chassis table |
| `FPV_FOV` | 60 | deg | `sim/renderer.py` | The twin's original camera. The depth grid publishes it as `fov_deg` |
| `FPV_MAX_DIST` | 14 | cells (4.2 m) | `sim/renderer.py` | Camera horizon. A depth zone past it is no-target |
| `FPV_STEP` | 0.05 | cells (1.5 cm) | `sim/renderer.py` | March step. The depth grid subtracts it; collision does not (subtracting it there would log walls never touched, 3.1) |
| `DEFAULT_WIDTH` x `DEFAULT_HEIGHT`, `JPEG_QUALITY` | 320 x 200, 82 | px, quality | `sim/renderer.py` | The twin's canvas defaults. Fixed so the golden image is reproducible. A whole `get_camera_frame()` costs with the house: about 6 ms in the starter house, 11 ms in the scaled house and 42 ms in the furnished home (laptop, 2026-10-02) |
| `COLOR_CEILING` / `COLOR_FLOOR` | (198, 203, 211) / (128, 120, 110) | RGB | `sim/renderer.py` | Lit room. The old near-black UI colours made the model report "a blank gray wall" (2026-09-02) |
| `VISIBLE_EXTENT_RAYS` | 9 | rays | `sim/renderer.py` | Rays across an object for its visible bearing (3.32) |
| `MAX_SUBSTEP_CELLS` / `MAX_SUBSTEP_RAD` | 0.1 / 5 (stored as radians) | cells / deg | `sim/grid_world.py` | Bounds travel between two collision checks, so an arc cannot cut a corner |
| `FOOTPRINT_SKIN_CM` | 0.2 | cm | `sim/grid_world.py` | Sized for the 2WD chassis pivoting in a 30 cm gap. The Rover's 17.1 cm corner radius cannot pivot there at all |
| `FOOTPRINT_LENGTH_M` x `FOOTPRINT_WIDTH_M` | see platform | m | imported from `robot/safety.py` | The chassis outline the sim collides with. Canonical value in the platform chassis table |
| `PAN_ANGLE_RAD` | 90 (stored as pi/2 radians) | deg | `sim/grid_world.py` | What `look_left()` / `look_right()` swing the camera by |
| `SIM_PERCEPTION_RANGE_CELLS` | 3.0 | cells | `sim/grid_world.py` | Arrival radius for `objects_visible` only. Detections go to the horizon |
| `MOVER_KEEPOUT_M` | 0.20 | m | `sim/movers.py`, `sim/grid_world.py` | Beyond the chassis turning circle. Same as `min_distance_cm`, so a mover never appears inside the stop line |
| `Mover.hop_s` | 1.0 | s of sim time | `sim/movers.py` | One 30 cm hop a second, a walking person |
| `SIM_BODY_URL`, `SIM_SENSORS_URL` (env) | none; `service/tunnel/run.sh` sets `http://127.0.0.1:8002` and `http://127.0.0.1:8003,http://127.0.0.1:8004` | URL | `robot/factory.py` -> `SimBodyClient` | 3.36. Required with `SIM_MOTOR_BOARD=fake`. Sensors comma-separated: the first serves the safety bundle, the last full scans and frames |
| `SIM_BODY_SHM` (env) | `picar_sim_body` | name | `sim/body_server.py`, `sim/sensor_server.py` | The shared-memory block. Every program of one stack uses the same name; tests use a unique one per `SimPrograms` |
| `PUBLISH_S` | 0.005 | s | `sim/body_server.py` | Twice the board loop's rate, so a sensor never casts from a state more than one board step old |
| `SIZE` | 1 MiB | bytes | `sim/body_state.py` | The furnished home's objects fit many times over |
| `SENSOR_STALE_S` | 0.15 | s | `sim/body_client.py` | Three wheel-loop periods; the real lidar's period is 0.1 s. Fed bundles this old, 3.36's sweep (2880 runs) kept the 3.18 bars: worst travel-to-contact 19.0 cm with `_aged()`, 18.2 cm without (`tests/test_footprint_safety.py` pins it, and asserts the bound is three periods) |
| `POLL_S` | 0.05 | s | `sim/body_client.py` | The wheel loop's period |
| `READ_TIMEOUT_S`, `FRAME_TIMEOUT_S`, `START_TIMEOUT_S` | 0.5, 3.0, 30.0 | s | `sim/body_client.py` | A poll (staleness, not this, is the safety bound); a rendered frame; how long the robot server waits for the programs at start-up |

## Procedures

**Run the robot server on the simulator** (local, port 8000):

```bash
SIM_MAP=scaled_house uvicorn robot.server:app --port 8000
curl -s http://127.0.0.1:8000/health | python -m json.tool | grep sim_map
```

Expect `"sim_map": "scaled_house"`. An unknown name fails at start-up
with `ValueError: unknown SIM_MAP '...' (starter_house | scaled_house |
home_first_floor)`. There is no reset route: restart the server to put the
robot back at its start pose. If another session owns port 8000, use the
parallel-session ports in the operations spec
(docs/engineering/operations/ENGINEERING.md).

**With a person crossing the hallway:**

```bash
SIM_MAP=scaled_house SIM_MOVERS=hallway_crossing uvicorn robot.server:app --port 8000
curl -s http://127.0.0.1:8000/sim/objects
```

`sim_time_s` keeps climbing while the robot is parked, because the wheel
loop's idle ticks call `pass_time()`. A scenario the house does not define
fails with `unknown SIM_MOVERS '...' for <house>`.

**Move a piece of furniture live:**

```bash
curl -s -X POST http://127.0.0.1:8000/sim/objects/move \
  -H 'content-type: application/json' -d '{"src": [2, 2], "dst": [3, 2]}'
```

This example moves the scaled house's sofa, so it works only with
`SIM_MAP=scaled_house`; take a `src` from `GET /sim/objects` for any other
house. A 409 names why (`no object at (x, y)`, a mover, not floor,
occupied, or inside the turning circle). Add `-H "x-app-secret: ..."` when `APP_SHARED_SECRET` is set.

**Run the split simulator** (3.36, the fake motor board). `service/tunnel/run.sh`
with `SIM_MOTOR_BOARD=fake` starts the body program, then the sensor
programs, then the robot server, on the defaults above; `restart.sh` stops
ports 8002-8004 too and checks the body program's git revision. By hand:

```bash
export SIM_BODY_SHM=picar_sim_body SIM_MAP=scaled_house
uvicorn sim.body_server:app --host 127.0.0.1 --port 8002 &
uvicorn sim.sensor_server:app --host 127.0.0.1 --port 8003 &
uvicorn sim.sensor_server:app --host 127.0.0.1 --port 8004 &
ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake WORLD_MODE=ros \
  SIM_BODY_URL=http://127.0.0.1:8002 \
  SIM_SENSORS_URL=http://127.0.0.1:8003,http://127.0.0.1:8004 \
  uvicorn robot.server:app --port 8000
```

`SIM_MAP` and `SIM_MOVERS` belong to the body program; the robot server
reports the house it reads from there. `WORLD_MODE=sim` is refused with a
remote body (the world would need the grid in-process); use `ros` or
`none`. In tests, `tests/conftest.py`'s `sim_programs` fixture
(`SimPrograms`) starts the programs, each in its own process group, on free
ports with a unique `SIM_BODY_SHM`.

**Ground-truth sweeps** (the acceptance instruments, not unit tests):

```bash
python -m tests.demo_footprint_sweep [--starts 40] [--seed 0] [--unclamped]   # 3.18
python -m tests.demo_mover_sweep [starts_per_house] [seed]                     # 3.30, defaults 20 and 0
python -m tests.chassis_fit                                                    # 3.21
```

Expected output:

- `demo_footprint_sweep`: a first line `<n> runs in <s>s (<starts> starts x
  24 headings x 2 directions x 3 houses)`, then one `<criterion>: PASS --
  <detail>` line per 3.18 bar, the closest approach after a move, and one
  line per house with mean travel and how many eligible runs covered the
  progress distance. Any `FAIL` is a regression. `--unclamped` prints only
  `4 sim collision: PASS -- 0 runs deeper than ...`, judging the sim's own
  collision with no safety clamp.
- `demo_mover_sweep`: an `== clamp ON: <n> runs ==` block with `PASS` or
  `FAIL` per bar (travel-to-contact after a robot move >= 18 cm, no
  contact, no penetration) and a line per house, then the same block for
  `== clamp OFF ==`. Clamp ON must PASS every bar; clamp OFF must FAIL at
  least one, or the sample never put the mover in the way.
- `chassis_fit`: for each house at 0 cm and 3 cm margin, a table of rooms
  with `old reached`, `new reached` and the floor where each body can turn
  right round (m^2). A room the Rover loses is flagged `<-- LOST`. Today
  none is: 3 of 3 in the starter house, 5 of 5 scaled, 13 of 13 furnished
  home.

**A paid closed-loop vision mission in the sim** (every step is a real
call; see the script's docstring for the environment it needs):

```bash
python -m tests.demo_sim_mission "red backpack" 20
```

Expected: a header naming the target, the model, the wording, whether the
sensor is noisy, the start pose and the target's true room; one line per
paid step with the action and the model's reasoning; then `Outcome:`,
`Steps/calls:`, `Wall clock:`, `Ended at:`, whether the model ever reported
`target_reached`, and `Actually:` judged on ground truth. A run that ends
without a step line failed before its first call: read `Error:`.

**Re-bless the golden image** after a deliberate render change, and look at
the result before committing. The command is in `tests/test_renderer.py`'s
docstring. It renders the starter house from (5.5, 7.5) facing east to
`tests/golden/fpv_hallway_east.png`.

## Verification

| Test | What it pins |
|---|---|
| `tests/test_robot_contract.py` | `MockRobot` against the body contract, alongside every other backend: shapes, units, a decodable image, a pivot covers no ground |
| `tests/test_world_contract.py` | The body/world split, and that the sim body and world agree about one `GridWorld` |
| `tests/test_continuous_pose.py` | R0: a non-cardinal pose exists and survives a read-back; the cell view still answers; a straight line does not depend on track width and a pivot does; encoders agree with the pose; the robot's cell is never a wall |
| `tests/test_renderer.py` | The golden image byte for byte (raw RGB, not JPEG), determinism, doors pass rays, objects behind walls or under the robot are not drawn |
| `tests/test_frame_source.py` | The twin shows real pixels or says it has none (browser test; skips without Chromium) |
| `tests/test_sensors.py` | S5: noise, dropout as 0.0, range clamp; `min_distance_cm` is load-bearing at a non-multiple of 30 |
| `tests/test_solid_objects.py` | 3.9: collision, scan, distance, depth and map all see objects; the forward beam from (8.5, 7.5) reads the backpack face at 0.41 m (the face is 0.45 m from the centre, less `LIDAR_X_M`); the picture is unchanged |
| `tests/test_bearing_turns.py` | R1: sized turns, against quarter turns, on RELATIVE bars: sized turns close more than 2 cells more on average, with fewer than half the reversals. It does not pin the absolute numbers (R1 recorded 4.06 cells / 0.6 reversals against -1.23 / 4.8 in the starter house on 2026-09-25), so those can drift without failing it. Quote a fresh run, not the R1 figures |
| `tests/test_movers.py` | 3.30 criteria 1 and 4: a move is seen at once by scan, collision and map; movers are deterministic and keep out; with no movers the object table is untouched by time |
| `tests/test_mover_safety.py` | 3.30 criterion 2 on a sample (scaled house, 2 starts, 8 headings), and that the same sample FAILS with the clamp off |
| `tests/test_footprint_safety.py`, `tests/test_pivot_safety.py` | 3.18 and 3.19 bars on ground truth, run through the sim's rectangle collision |
| `tests/test_cad_geometry.py` | 3.27: the scan is cast from the lidar 4 cm ahead, judged on beam endpoints against the layout |
| `tests/test_home_map.py` | The home's outline and garage still match the appraisal (1483.05 and 428.24 sq ft); every room is reachable around the furniture |
| `tests/test_r2_routes.py` | R2: every scan beam equals the renderer's ray; truth equals the pose |
| `tests/test_sim_body_process.py` | 3.36: the robot server's process imports no simulator module; readings follow the body across the boundary and equal what the body measured; furniture moves reach physics and sensors; the physics program dying stops the wheels; a dead sensor program blinds the vet in time; the vet never touches the network; safety stream and heavy reads are different programs; the factory refuses an in-process fake board |

Recorded acceptance numbers (from the plan sections, measured through the
real mission path):

- **3.9 solid objects:** 69 of 69 starts stop at (9.5, 7.5), in front of
  the backpack. Arrival 98.3% +/- 1.0 at 90% and at 80% detection, over 690
  missions per rate.
- **3.30 movers:** 2021 runs in three houses, 0 under 18 cm of
  travel-to-contact after a robot move (worst 19.7 cm), 0 contacts, 0
  penetration. With the clamp off: 1522 under the bar. nav2 with a person
  crossing: 6/6 and 6/6.
- **R5 drift:** odometry up to 99 cm / 54 deg off, SLAM within 1-4.5 cm.

**Checklist for a change here:**

1. Write the metric and the bar in the plan entry before measuring.
2. Run `pytest tests/ -q` from `.venv`. With realism off, every pinned
   trace must pass unchanged, including `tests/data/frontier_trace_centred.json`.
3. If a chassis constant changed, `tests/test_wall_linters.py`,
   `tests/test_urdf.py` and `tests/test_ros_driver_board.py` must agree
   across sim, xacro, the fake board and the backend.
4. For anything touching collision or sensing, run the ground-truth sweep
   and record its numbers, never the veto's own readings.
5. If the picture changed, re-bless the golden image and look at it.

## Known gaps

The open design questions (wheel slip, low obstacles in a 2D world, sim
time against ROS time, the home's interior, whether the lit renderer helps
a vision run) are the
[architecture spec's open questions](../../simulator/ARCHITECTURE.md#open-questions)
and are not repeated here. These are the implementation gaps:

- **Movers hop whole cells** and never approach the robot.
- **The D500's +90 degree mounting yaw is not modelled.** The sim publishes
  the scan with zero dead ahead (3.27, a hardware-day item).
- **The starter house's doors are tight for the UGV Rover.** The body
  fits every room (3 of 3, `python -m tests.chassis_fit`), but a 30 cm door
  leaves a few centimetres a side: nav2's costmap seals it whenever SLAM
  draws a jamb one 5 cm cell thick, and the guarded verbs refuse on about
  0.5 deg of heading error (G1, 3.24). Use the scaled house for nav2, the
  ROS chain suite and anything else that must pass a door reliably.
