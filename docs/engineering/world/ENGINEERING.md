---
kind: engineering
domain: world
status: current
verified: 2026-10-02
parent: docs/world/ARCHITECTURE.md
---

# World -- engineering

This is how the world contract is built today. The what and the why are in
the [architecture spec](../../world/ARCHITECTURE.md). This document is true
only until the implementation changes, and it is updated in the same commit
as the code.

## Implementation

| File | What it does |
|---|---|
| `world/interface.py` | `WorldInterface`, an ABC with no abstract methods: `get_pose()`, `get_map()` and `get_truth()`. The tri-state constants `CELL_UNKNOWN` (-1), `CELL_FREE` (0) and `CELL_OCCUPIED` (1). The defaults `unusable_pose()`, `unusable_map()` and `unusable_truth()`. `NullWorld`, the named "no mapper" backend. The docstrings are the contract; `tests/test_world_contract.py` requires every method to have one. |
| `world/factory.py` | `get_world(config_path, robot)` picks the backend from `WORLD_MODE`, or else the `world.mode` key. The code default is `none`; `config/robot.yaml` ships `sim`. |
| `sim/mock_world.py` | `MockWorld(world: GridWorld)`, the simulator world. `observe()` casts a 360-ray ring from the robot's continuous position and marks each cell the ring crosses as free, or as occupied where a wall or a solid object stops it. `get_map()` calls `observe()` first. The pose and the truth are both exact, read off `GridWorld.x`, `.y` and `.heading_deg`. |
| `world/ros_world.py` | `RosWorld(bridge_url, secret, truth, timeout_s, client)`, the SLAM world. An httpx client of the ROS bridge. It converts ROS's frame to ours (`_ros_to_ours()` and `_ours_to_ros()`, kept side by side), anchors SLAM's frame to the house per bridge session, and resamples SLAM's grid into the house frame (`_resample()`, nearest neighbour). It also carries nav2 goals (`set_goal`, `get_goal`, `cancel_goal`) and `get_odom_pose()`. None of those four are on the interface. |
| `control/remote_world.py` | `RemoteWorld(base_url, secret, timeout, client)`, the world over HTTP for the brain. It maps a 404 to the unusable default and raises `WorldTransportError` on any other failure. It does no caching. |
| `robot/server.py` | The `/world/*` routes are pass-throughs to `world_model = get_world(config_path, robot=robot)`. It also computes `/world/error`, and `goal_in_progress()` feeds driver arbitration. |
| `web-twin/app.js` | The consumer that draws the pose, map, error and goal (polls `/world/pose`, `/world/map`, `/world/error` and `/world/goal`, and posts tap-to-goal). Owned by the twin domain. |

`MockWorld` and `RosWorld` never import a body backend. `get_world()` reads
the body's `robot.world` attribute, which is the `GridWorld`, and nothing
else.

## Interfaces

### The world contract

```text
get_pose()  -> {"usable": bool, "map_id": str|None, "x_m": float|None,
                "y_m": float|None, "heading_deg": float|None}
get_map()   -> {"usable": bool, "map_id": str|None, "map_version": int,
                "resolution_m": float|None, "width": int, "height": int,
                "origin_x_m": float|None, "origin_y_m": float|None,
                "cells": list[int]}         # row-major, len == width*height
get_truth() -> {"usable": bool, "source": "sim"|None,
                "x_m": float|None, "y_m": float|None, "heading_deg": float|None}
```

- **Frame.** `x_m` points east and `y_m` points south. `heading_deg` is a
  compass bearing, clockwise-positive: 0 points to -y_m and 90 points to
  +x_m.
- **Unusable answers.** An unusable pose has every number set to `None`. An
  unusable map has `width = height = 0`, `cells = []`, `map_version = 0` and
  `None` for the remaining numbers.
- **The simulator's map.** It reports `map_id` `"sim-grid"`, origin (0.0,
  0.0), and the `GridWorld`'s own width and height.
- **SLAM's map.** It reports `map_id` `"slam-<session>"`, and its origin is
  the minimum corner of the resampled extent.

### The robot server's routes

Every route below requires the secret (`require_secret`, the `x-app-secret`
header) whenever `APP_SHARED_SECRET` is set.

| Method and path | Answer |
|---|---|
| `GET /world/pose` | `get_pose()`, verbatim |
| `GET /world/map` | `get_map()`, verbatim |
| `GET /world/truth` | `get_truth()`, verbatim |
| `GET /world/error` | See the next table. |
| `POST /world/goal` `{x_m, y_m}` (house frame) | **501** if the world has no `set_goal`. Otherwise it arbitrates as driver `ros`. A refusal answers `{"accepted": false, "reason": "preempted", ...}`. On success it returns `RosWorld.set_goal()`'s answer: `{"accepted": bool, ...bridge reply}`, or `{"accepted": false, "reason": "no SLAM session yet"}`. |
| `GET /world/goal` | **501** as above. Otherwise `{"goal": {...state, x_m, y_m} or None, "plan": [[x, y], ...]}`, in the house frame. A goal from an earlier session reads as `None`. |
| `DELETE /world/goal` | **501** as above. Otherwise the bridge's `{"cancelled": bool}`. |

What `GET /world/error` answers:

- **Usable pose and truth:** `{"usable": true, "source": <map_id>,
  "position_error_m", "heading_error_deg", "odom_position_error_m",
  "odom_heading_error_deg", "truth"}`. The `odom_*` fields are filled only
  when the world has `get_odom_pose()`.
- **Otherwise:** `{"usable": false, ...None}`.

### Arbitration

`arbitrate()` refuses any autonomous driver other than `ros` with
`preempted` while `goal_in_progress()` finds a goal whose `state` is
`pending` or `active`. If the bridge cannot be read, that counts as no goal.
The full driver order belongs to the safety domain
([safety architecture](../../safety/ARCHITECTURE.md)); this is only the part
the world feeds.

### What RosWorld reads from the bridge

The bridge's routes and their fields are specified once, in the
[ros engineering spec](../ros/ENGINEERING.md) ("The bridge's HTTP routes").
`RosWorld` uses `GET /slam/pose`, `GET /slam/map`, and `POST`, `GET` and
`DELETE /goal`. Two things are specific to this side of the wall:

- **Goals carry no heading.** `RosWorld` always sends `yaw_rad` 0.0, because
  nav2's yaw tolerance is 6.28: position goals only.
- **`map_version` is the bridge's `version`, passed through.** The bridge
  increments it on every `/map` message it receives
  (`service/slam/src/picar_bridge/picar_bridge/bridge.py`, `_on_map()`), not
  when the cells change. slam_toolbox's `map_update_interval` is 1.0 s, so
  the version probably climbs about once a second even at rest. UNCONFIRMED,
  by reading only. The architecture spec's D4 allows this (a version may
  change without a cell change); it makes "refetch on a new version" cost a
  full map about once a second under SLAM.

### The conversion and the anchor

- **Conversion.** `_ros_to_ours(x, y, yaw)` returns `(x, -y, (90 -
  degrees(yaw)) % 360)`.
- **Anchor.** A pose in SLAM's frame is mapped to the house as `R(alpha) ·
  p + t`, and `alpha` is added to the heading.
- **When the session starts** (`anchored_at = "session_start"`):
  `alpha = start_truth.heading_deg - 90` and `t = (start_truth.x_m,
  start_truth.y_m)`.
- **Bridges older than `start_truth`** (`"first_contact"`): the anchor falls
  back to the truth at the first question.
- **On hardware** (no truth): `alpha = 0`, `t = 0`, `anchored_at = None`.

## Parameters and configuration

| Key or constant | Default | Unit | Where read | Why this value |
|---|---|---|---|---|
| `WORLD_MODE` env, else `world.mode` | `none` in code; `sim` in `config/robot.yaml` | -- | `world/factory.py` | `none` is the honest setting for teleop and for hardware without SLAM. `ros` is the SLAM world. |
| `ROS_BRIDGE_URL` env, else `world.bridge_url` | `http://127.0.0.1:8090` | URL | `world/factory.py` | The bridge's published port. `world.bridge_url` is not in the shipped yaml, so the env var or the code default applies. |
| `APP_SHARED_SECRET` | empty | -- | `world/factory.py` passes it to `RosWorld` | It is sent as `x-app-secret` to the bridge. |
| `CELL_M` | 0.30 | m | `sim/mock_world.py` | One grid cell. It must equal the body's cell size, and `test_the_world_and_the_body_agree_about_cell_size` pins that. |
| `DEFAULT_RAYS` | 360 | rays | `sim/mock_world.py` | One per degree, a 360-degree scanner. A forward cone would never discover the corridor behind the robot. |
| `DEFAULT_RANGE_CELLS` | 14 (`renderer.FPV_MAX_DIST`, 4.2 m) | cells | `sim/mock_world.py` | The renderer's horizon. It is NOT the sim lidar's 12 m (`LIDAR_RANGE_M`); see Known gaps. |
| March step | `renderer.FPV_STEP`, 0.05 | cells | `MockWorld.observe()` | Shared with the camera raycaster. |
| `OCCUPIED_AT` | 65 | ROS occupancy, 0-100 | `world/ros_world.py` | At or above this a cell is occupied. Matches the convention of nav2's map server. `tests/test_wall_linters.py` forbids any other copy of these thresholds. |
| `FREE_BELOW` | 25 | ROS occupancy, 0-100 | `world/ros_world.py` | Below this a cell is free. Values between the two thresholds stay unknown. |
| `RosWorld` `timeout_s` | 2.0 | s | `world/ros_world.py` | A pose request is answered "unusable" quickly when the bridge is slow. |
| `DEFAULT_TIMEOUT_S` | 10.0 | s | `control/remote_world.py` | The same budget as `RemoteRobot`'s requests. |
| `SIM_MAP` | `starter_house` | -- | `robot/factory.py`, which builds the house with `sim/maps/__init__.py`'s `build_world()` | `build_world()` stamps the house's name on the world (`map_name`), and `/health` reports that as `sim_map`: the house actually built, for every body standing in one, `mode: hardware` with `SIM_MOTOR_BOARD=fake` included. `null` when no sim house stands behind the robot. nav2 is judged only on `scaled_house`, because the starter house's 30 cm doors seal in the costmap (3.15). |

## Procedures

**Read the simulator's world** (the default configuration):

```bash
uvicorn robot.server:app --port 8000
curl -s localhost:8000/world/pose   # {"usable": true, "map_id": "sim-grid", "x_m": ..., ...}
curl -s localhost:8000/world/map | python -c 'import json,sys; m=json.load(sys.stdin); print(m["map_version"], m["width"], m["height"], m["cells"].count(-1))'
```

Expected: `map_version` rises as the robot drives into new rooms and stays
put while it stands still. The count of unknown cells falls.

**Read SLAM's world.** This needs the ROS container to be running; see the
ros engineering spec's procedures.

```bash
set -a; source ~/.vision-picar-local-secrets; set +a    # defines LOCAL_SECRET
ROBOT_DRIVE=ros WORLD_MODE=ros SIM_MAP=scaled_house bash service/tunnel/restart.sh
curl -s -H "x-app-secret: $LOCAL_SECRET" localhost:8000/world/error
```

Expected: `"source": "slam-<8 hex>"`, and at rest a `position_error_m` of a
few centimetres. With `SIM_ODOM_DRIFT=1.0,1.03` added to the restart, the
`odom_*` errors grow lap by lap while SLAM's stay small.

**Send and cancel a goal** (SLAM world only, house frame):

```bash
curl -s -X POST -H 'content-type: application/json' -H "x-app-secret: $LOCAL_SECRET" \
     -d '{"x_m": 2.1, "y_m": 1.5}' localhost:8000/world/goal
curl -s -X DELETE -H "x-app-secret: $LOCAL_SECRET" localhost:8000/world/goal
```

Map first. nav2 cannot plan into a room SLAM has not seen, so map with
`python -m tests.demo_slam_lap` or by driving before sending a goal.

**Failure signatures:**

| Symptom | Meaning |
|---|---|
| Start-up `ValueError: world mode 'sim' needs the grid-world robot` | `world.mode: sim` was combined with `mode: teleop` or hardware without the fake board. Set `WORLD_MODE=none`. |
| `POST /world/goal` answers 501 | The world is not `ros`. |
| `{"accepted": false, "reason": "no SLAM session yet"}` | The bridge has not yet published a map-frame pose. |
| `{"accepted": false, "reason": "preempted"}` | The brain or a person holds the robot. Wait for it to lapse, which takes `watchdog_timeout_s`. |

Signatures that depend on the bridge's state (`/world/pose` unusable under
`WORLD_MODE=ros`, and the SLAM error readout stuck at "0.0 cm" because the
bridge has no `start_truth`) are in the ROS chain's one table,
`service/slam/README.md` section 4 ("Failure signatures").

## Verification

The world tests and what each pins (test counts from `pytest
--collect-only`, 2026-10-02):

| Test file | Tests | What it pins |
|---|---|---|
| `tests/test_world_contract.py` | 57 | The contract's shapes over `NullWorld`, `MockWorld` and `RosWorld`, the last against a fake bridge. The body/world split in both directions (`get_pose`, `get_map` and `get_truth` are absent from `RobotInterface`; the body methods are absent from `WorldInterface`). No abstract methods. The discovered map: it starts unknown, what is behind a wall stays unknown, the robot's own cell is free, the version holds when nothing new is seen. The pose is the body heading, not the view heading. Cell sizes agree. `RemoteWorld`'s 404 versus broken-mapper rule. |
| `tests/test_ros_world.py` | 13 | The anchor at the session's start, not at the first question. Left turns are counter-clockwise in ROS and clockwise on the compass. The map is not mirrored. A sideways displacement lands on the correct side. A restart is a new map and a new anchor. No anchor and no truth on hardware. Odometry is in the same frame. |
| `tests/test_ros_goals.py` | 26 | Goal conversion both ways, plans converted point by point, an earlier session's goal hidden, no goal sent before a session exists, cancel, 501 for a world that cannot plan, the secret, and `/world/error` with and without truth. |
| `tests/test_goal_arbitration.py` | 9 | `PLAN-ros-alignment.md` 3.23: a goal is refused while the brain holds the robot; brain and teleop are refused during a goal; a person never is; an ended goal blocks nothing; a lapsed claim does not block a goal. |
| `tests/test_wall_linters.py` | (1 of 18) | The occupancy thresholds exist only in `world/ros_world.py`. |
| `tests/test_slam_live.py` | 3 | The live SLAM lap. It skips without the stack and wants `starter_house` with `WORLD_MODE=ros`. |

The recorded numbers (`PLAN-ros-alignment.md` 3.14, eighteen laps with
`tests/demo_slam_lap.py`; and 3.27):

| Measure | Result |
|---|---|
| SLAM at-rest error, drift off | Worst per lap 4.0-8.2 cm (bar 5 cm, met on 2 of 9). Heading 0.8-2.5 degrees (bar 2, met on 7 of 9). Every lap ended within 0.1-1.4 cm. |
| Drift on (right encoder 3% long) | SLAM ended within 1.0-4.5 cm on 9 of 9. Odometry ended up to 99 cm and 54 degrees off. |
| Map | 95.9-100% of occupied cells within 10 cm of a true surface; 0-0.73% of free cells inside a wall. |
| SLAM final error after 3.27's lidar offset | 2.6-4.7 cm, against 1.5-3.6 cm for the control (four laps each). Not established at n=4. |

Checklist for a change here:

- Run `pytest tests/test_world_contract.py tests/test_ros_world.py
  tests/test_ros_goals.py tests/test_goal_arbitration.py
  tests/test_ros_containment.py tests/test_wall_linters.py`.
- Any change to the frame or the anchor needs a live lap
  (`python -m tests.demo_slam_lap`) with its numbers recorded against the
  table above.
- A new world method goes in `WORLD_STATE_METHODS` in
  `tests/test_world_contract.py`, with a docstring.

## Known gaps

- **Range mismatch.** `MockWorld`'s ring reaches 4.2 m, the camera's
  horizon. The simulator's lidar (`get_scan()`) reaches 12 m. In a large
  house the simulator world discovers less than SLAM would from the same
  scans.
- **Stale docstrings.** `MockWorld.get_pose()`'s docstring still says
  "quantised to cell centres", which has not been true since R0. The pose
  is continuous.
- **No version-only route.** The twin re-fetches the whole map on a timer.
- **Goal routes during a bridge outage.** `RosWorld.set_goal()`,
  `cancel_goal()` and `get_goal()` (`world/ros_world.py:231-245`) do not
  catch `httpx` errors, and the routes that call them
  (`robot/server.py:837-857`) do not either, so `POST`, `GET` and
  `DELETE /world/goal` answer 500 when the bridge is down. A non-JSON bridge
  reply fails the same way. UNCONFIRMED, established by reading the code
  only. (`goal_in_progress()`, `robot/server.py:370-385`, does catch them,
  so arbitration fails open as the architecture spec says.)
- **`map_version` under SLAM counts publications, not cell changes.** See
  "What RosWorld reads from the bridge". UNCONFIRMED.
- **`start_truth` is read once.** A failed first read is never retried; see
  `service/slam/README.md` section 4.
- **Stale factory docstring.** `world/factory.py:14-22` still calls `none`
  "TODAY'S DEFAULT" with "nothing in this project can build a map yet", and
  labels `ros` as N6. `config/robot.yaml` ships `sim`, and `ros` is built
  (R5/R6).
- **Goals are found by duck typing.** `set_goal`, `get_goal`, `cancel_goal`
  and `get_odom_pose` live only on `RosWorld`, and the server discovers them
  with `hasattr`.
- **Map resampling.** `_resample()` is nearest-neighbour. It is exact only
  for the rotations a cardinal start produces; otherwise it is honest, never
  inventing a state, but blocky.
- **No persistence.** The map lives for the container's lifetime
  (`PLAN-ros-alignment.md` section 6, question 6).
- **Stale config comment.** `config/robot.yaml`'s `world:` comment still
  says `ros` is "not built yet".
