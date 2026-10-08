---
kind: engineering
domain: control-api
status: current
verified: 2026-10-04
parent: docs/control-api/ARCHITECTURE.md
---

# Control API (the robot server) -- engineering

How the robot server is built today. What it is responsible for and why it
is a separate process behind plain HTTP is in the
[architecture spec](../../control-api/ARCHITECTURE.md). The vet,
arbitration and watchdog rules it hosts are specified in
[safety engineering](../safety/ENGINEERING.md). This document is true only
until the implementation changes.

## Implementation

| Where | What |
|---|---|
| `robot/server.py` | `create_app(config_path=None)` builds one FastAPI app with its own body (`robot.factory.get_robot`), world (`world.factory.get_world`), `SafetyController`, state dict and two asyncio tasks (`watchdog_loop`, `wheel_loop`) started in the lifespan. The module-level `app = create_app(os.environ.get("ROBOT_CONFIG_PATH"))` is what `uvicorn robot.server:app` serves |
| `robot/identity.py` | `log_identity()` logs one warning-level line at start-up (`<service> starting -- git=... executable=... config=...`) and returns the dict `/health` serves as `identity`. Revision from `GIT_REVISION`, else `git rev-parse --short HEAD`, else `unknown` |
| `robot/ros_drive.py` | Under `drive: ros`, the body is a `RosDriveRobot`. The server detects it by `drives_by_velocity`, names each verb's driver with `robot.driving_as(driver)`, and builds `fallback_safety` over `robot.inner`. Its `stop()` returns once the wrapped body has stopped and zeroes the ROS inputs on a background thread, so neither `/stop` nor the watchdog ever waits on the bridge ([body engineering](../body/ENGINEERING.md), "The ROS drive stop") |
| `web-twin/` | `web-twin/index.html`, `web-twin/app.js`, `web-twin/manifest.json` and `web-twin/icons/`, served from the same origin (deployed copies come from S3/CloudFront; see [twin](../twin/ENGINEERING.md)) |

Concurrency: route handlers are sync and run on the threadpool; the loops run
on the event loop. `motion_lock` (a `threading.Lock`) serialises everything
that moves the body. Direct-mode verbs hold it for their whole duration and
set `state["verb_active"]`. The wheel loop takes it with
`acquire(blocking=False)` and skips the tick if a verb holds it. `/stop`
does not take it: it calls `robot.stop()`, whose `stop_count` the verb loop
checks every period (a stop landing inside one period can be overwritten:
[body engineering](../body/ENGINEERING.md), Known gaps). Under `drive: ros`
the normal verb path does not hold the lock, because its own motion arrives
back as `POST /wheels`, which needs it. The ROS-down fallback (a person's
verb on `fallback_safety`) does hold it and sets `verb_active`, exactly as
direct drive does. The watchdog calls `robot.stop()` synchronously on the
event loop; every body's `stop()` must therefore return without waiting on
the network, which `RosDriveRobot` has done since 2026-10-02.

## Interfaces

All routes take the `ROUTE_PREFIX` prefix. "Secret" means `require_secret`
applies: 401 `Missing or invalid x-app-secret header.` when
`APP_SHARED_SECRET` is set and `x-app-secret` differs.

### Motion

| Route | Secret | Request | Response |
|---|---|---|---|
| `POST /action` | yes | `{"action", "speed": 50, "duration": 0.5, "angle": 90}`, header `x-driver` | `{"executed": true, "result", "driver"}`; refusal `{"executed": false, "reason", "detail"}` (200); 400 on an unknown action. On the ROS fallback `result` gains `"via": "direct-fallback"` |
| `POST /wheels` | yes | `{"left_rad_s", "right_rad_s"}`, `x-driver` | `{"executed": true, "driver", "applied": {...}, "clamped": reason or null}`. In direct drive a zero is never arbitrated and claims nothing: from the holder it zeroes the wheels, from anyone else it gives `{"executed": true, "ignored": true, ...}`. Under ROS drive only `ros` may post, unarbitrated, and a zero is not special; refusals as above (`not_the_actuator` under ROS drive for any driver but `ros`; `unsupported` on a body without motors) |
| `POST /stop` | yes | `x-driver` (echoed only) | `{"executed": true, "result", "driver"}`. Never arbitrated; feeds the watchdog; claims nothing; then, unless the driver is the brain, ends any nav2 goal from a background thread, holding `ros` wheel commands at zero until nav2 reports it over ([safety engineering](../safety/ENGINEERING.md), "Stop ends a nav2 goal") |

Actions: `FORWARD`, `REVERSE`, `LEFT`, `RIGHT`, `STOP`, `LOOK_LEFT`,
`LOOK_RIGHT`, `LOOK_CENTER`.

### Body sensing (all secret, all GET)

| Route | Returns |
|---|---|
| `/distance` | `{"distance_cm"}` |
| `/frame` | the body's frame dict; **503** with the body's message if it raises |
| `/depth` | the depth grid plus `path`: `{"indices", "clearance_cm", "source", "blocked", "veto_cm", "veto_source", "min_distance_cm", "sensor_to_bumper_cm"}` |
| `/odometry` | `{"usable", "distance_m", "heading_deg"}` |
| `/wheels` | the wheel state (read by `picar_sim_hardware`) |
| `/scan?max_range_m=` | the scan plus `stamp_unix` (capture time) |

### World (all secret)

| Route | Returns |
|---|---|
| `GET /world/pose`, `GET /world/map` | pass-through of `get_pose()` / `get_map()` |
| `GET /world/truth` | pass-through; `usable: false` off the sim. No decision may read it |
| `GET /world/error` | `{"usable", "source", "position_error_m", "heading_error_deg", "odom_position_error_m", "odom_heading_error_deg", "truth"}` |
| `POST /world/goal` | `{"x_m", "y_m"}` -> the world's `set_goal()`, or `{"accepted": false, "executed": false, "reason": "preempted", ...}`. Arbitrated as driver `ros` |
| `GET` / `DELETE /world/goal` | `get_goal()` / `cancel_goal()` |

Goal routes answer **501** on a world without `set_goal` (anything but
`WORLD_MODE=ros`).

### Teleop, sim-only, static

| Route | Secret | Behaviour |
|---|---|---|
| `POST /teleop/frame` | yes | `{"image_base64", "media_type"}` -> `{"received": true, "seq"}`; **400** if the body has no `push_frame` |
| `GET /sim/objects` | yes | `{"sim_time_s", "objects": [{"x", "y", "name", "mover"}]}`; **501** with no simulated house (searched through `.inner` up to 3 levels) |
| `POST /sim/objects/move` | yes | `{"src": [x, y], "dst": [x, y]}`; 422 if not pairs, 409 if the move is illegal. Both sim routes answer with `GridWorld.describe_objects()`; under the fake board (3.36) they are forwarded to the body program's own `/sim/objects` routes through `RemoteGrid` |
| `GET /`, `/app.js`, `/manifest.json`, `/icons/icon-192.png`, `/icons/icon-512.png`, `/icons/apple-touch-icon.png` | no | the twin. `app.js` is served `Cache-Control: no-cache` |

### `GET /health` (no secret)

`status`, `seconds_since_last_command`, `drive` (`mode`,
`verbs_through_ros`, `wheel_posts_from_ros`, `ros_up`, `bridge_up`,
`ros_post_age_s`), `watchdog_timeout_s`,
`wheel_loop` (`moving_ticks`, `late_ticks`, `max_dt_s`, `period_s`),
`motor_board` (the body's `feedback_status()`, or null for a body without
a board: `fresh`, `age_s`, `stale_after_s`, `frames`, `board_reboots`,
`stale_zeroed`, `fork_firmware`, `link_error`; 3.34), `min_distance_cm`, `mode`, `sim_map`, `identity`,
`seconds_since_watchdog_poll`, `watchdog_poll_interval_s`, `driver`,
`authority_holder`, `refusal_counts`, `last_refusal` (with `seconds_ago`),
`env_label`. `sim_map` is `robot.world.map_name`, the name
`sim/maps/__init__.py`'s `build_world()` stamps on the house it built, read
through the wrappers' attribute forwarding. It names that house for any body
standing in one: `mode: sim`, and `mode: hardware` with
`SIM_MOTOR_BOARD=fake` (3.33's G4 configuration; since 3.36 the house is in
`sim/body_server.py`'s process and `robot.world` is `sim/body_client.py`'s
`RemoteGrid`, which reports the name the body program's `/health` gave). It is null when no sim
house stands behind the body (`mode: teleop`, a real board). Changing
`SIM_MAP` after start-up does not change it. Until 2026-10-02 it re-read
`SIM_MAP` and only under `mode: sim`, so under the fake board it was null and
the live ROS suites skipped (`docs-review/SPEC-REVIEW.md` finding 2).
`control/health.py` marks the robot UNHEALTHY when
`seconds_since_watchdog_poll > watchdog_poll_interval_s x 10`; every other
field is description.

### Control loops

| Loop | Period | Each period |
|---|---|---|
| `watchdog_loop` | `WATCHDOG_POLL_INTERVAL_S` | Stamp `watchdog_polled_at`; skip while `verb_active`; if `now - last_command_at > watchdog_timeout_s`, call `robot.stop()` and record a `watchdog` refusal once per silence |
| `wheel_loop` | `WHEEL_LOOP_INTERVAL_S` | If the lock is free and the wheels are usable: zero command -> `pass_time(dt)` (sim movers); otherwise count the tick (late past twice the period), `vet_wheel_velocity()`, apply the vetted speeds if they differ, record a `safety_distance` refusal once per command, then `advance(dt)` |

The periods and the late-tick rule are safety parameters, listed in
[safety engineering](../safety/ENGINEERING.md).

`last_command_at` is fed by `/action` (at the start and again at the end of
a direct verb), `/stop`, and `/wheels` (an actuator post under ROS drive, or
an accepted non-zero direct command; a zero direct command never feeds it).
Sensing reads never feed it.

## Parameters and configuration

| Key / variable | Default | Read | Why / effect |
|---|---|---|---|
| `safety.*`, `WATCHDOG_POLL_INTERVAL_S`, `WHEEL_LOOP_INTERVAL_S`, `ROS_SILENCE_S` | see safety | `create_app` | Canonical values and reasons in [safety engineering](../safety/ENGINEERING.md), Parameters. This server reads them, passes the `safety.*` keys to `SafetyController`, and publishes `min_distance_cm`, `watchdog_timeout_s`, `watchdog_poll_interval_s` and the wheel loop's `period_s` on `/health` |
| `server.allowed_origins` | `["*"]` | `create_app` | CORS origins; methods `GET`, `POST`, `OPTIONS`; all headers |
| `mode` / `ROBOT_MODE`, `drive.*` / `ROBOT_DRIVE`, `SIM_MAP`, ... | | `robot/factory.py` | Body selection: see [body engineering](../body/ENGINEERING.md) |
| `world.mode` / `WORLD_MODE` | `sim` in yaml; `none` in code | `world/factory.py` | See [world engineering](../world/ENGINEERING.md) |
| `APP_SHARED_SECRET` | unset (gate inert) | `require_secret`, and passed to the ROS wrapper | Shared secret |
| `ROUTE_PREFIX` | `""` | `create_app` | Prepended to every route |
| `ROBOT_CONFIG_PATH` | unset | module `app` | Alternate `config/robot.yaml`; used by `tests/test_watchdog_integration.py` |
| `ENV_LABEL` | `""` | `/health` | Environment banner in the twin |
| `GIT_REVISION` | unset | `robot/identity.py` | Build revision in images without `.git` |
| `PICAR_LOG_LEVEL` | unset (logging left as found) | `configure_logging()` in `robot/identity.py`, called by both `create_app`s | One stderr handler on the root logger at that level, lines `<epoch> <LEVEL> <logger> <message>`, the `httpx` logger held at WARNING or above; `tests/demo_explore.stack()` sets INFO for the brain and WARNING for the robot server (3.45) |

## Procedures

**Run it locally:**

```bash
uvicorn robot.server:app --port 8000
curl -s localhost:8000/health | python -m json.tool | head
curl -s -X POST localhost:8000/action -H 'x-driver: twin-dpad' \
     -H 'content-type: application/json' -d '{"action": "FORWARD"}'
```

Expected: the log's first line is
`vision-picar robot server starting -- git=<rev> ...`; `/health` shows
`"mode": "sim"`, `"sim_map": "starter_house"`, `"drive": {"mode": "direct", ...}`;
the FORWARD answers `{"executed": true, ...}` or, against a wall,
`{"executed": false, "reason": "safety_distance", ...}`.

**With the brain and the twin:** start `uvicorn control.brain_server:app
--port 8001` too, open `http://127.0.0.1:8000/`, and set both URLs in
Settings. `bash service/tunnel/run.sh` starts the robot (8000), the brain
(8001, `ROUTE_PREFIX=/brain`) and the fan-out proxy (8080) for the deployed
twin; it reads `APP_SHARED_SECRET` from `~/.vision-picar-local-secrets`
(see [operations](../operations/ENGINEERING.md)).

**Under ROS drive:** `ROBOT_DRIVE=ros uvicorn robot.server:app --port 8000`
with the container running (`service/slam/README.md`). `/health`'s
`drive.ros_up` turns true at the actuator's first `/wheels` post. Another
session may already own ports 8000/8090; use 8100/8101/8190 with a separate
`ROS_DOMAIN_ID`.

**Reset the robot:** restart the server. There is no reset route.

**Failure signatures:**

| Symptom | Meaning |
|---|---|
| 401 on every route but `/` and `/health` | `APP_SHARED_SECRET` set; the client is not sending `x-app-secret` |
| 503 on `/frame`, other routes fine | The camera, not the server: a stalled teleop phone, or no camera driver on hardware |
| 400 on `/teleop/frame` | Not `mode: teleop` |
| 501 on `/world/goal` or `/sim/objects` | Not a ROS world, or not a sim body |
| `ValueError` at start-up about world mode `sim` | A body with no sim house (`mode: teleop`, or `mode: hardware` on a real board) under the shipped `world.mode: sim`; set `WORLD_MODE=none`. With `SIM_MOTOR_BOARD=fake` the message names `SIM_BODY_URL`: the house is in another process (`PLAN-ros-alignment.md` 3.36), so `MockWorld` cannot cast over it; use `WORLD_MODE=ros` or `none` |
| `ValueError` at start-up: `SIM_MOTOR_BOARD=fake runs the simulated body as separate programs` | `SIM_BODY_URL` / `SIM_SENSORS_URL` unset. Start `sim/body_server.py` and `sim/sensor_server.py` first, or use `service/tunnel/run.sh`, which does (3.36; [simulator engineering](../simulator/ENGINEERING.md)) |
| `RuntimeError: no sim/body_server.py at ...` (or `sim/sensor_server.py`) at start-up | The robot server waited 30 s for that program's `/health` |
| FORWARD vetoed everywhere under the fake board, the scan `usable: false` | The sensor programs' safety bundle is older than `SENSOR_STALE_S` (0.15 s): the first program in `SIM_SENSORS_URL` is dead or overloaded. Fail-safe by design (3.36) |
| `/health` `sim_map` null where a live suite expects a house | No sim house behind the body. The live ROS suites skip on it; a skip is not a pass |
| `seconds_since_watchdog_poll` growing | The watchdog task is dead; restart the server |
| `wheel_loop.late_ticks` > 0 | The event loop was starved while the wheels turned |
| `/action` or `/wheels` refused `no_feedback`; `motor_board.fresh` false | The motor board has not reported for `stale_after_s`. `link_error` set: the serial line is gone. Null, with `age_s` growing: the board hears commands and no longer reports. Nothing moves until frames return |

## Verification

| Test (collected) | What it proves |
|---|---|
| `tests/test_server.py` (23) | Routes, 400 on unknown action, the secret gate (protected routes 401, `/health` and `/` open), CORS headers, `ROUTE_PREFIX`, teleop push/pull and 503 on a stall, sensing does not feed the watchdog, the twin's script |
| `tests/test_watchdog_integration.py` (6) | The watchdog and its liveness against a live `uvicorn` subprocess, `sim.realtime`, and the health command against it |
| `tests/test_authority.py` (13), `tests/test_wheels_command.py` (8), `tests/test_goal_arbitration.py` (9), `tests/test_ros_fallback.py` (10), `tests/test_stop_cancels_goal.py` (13) | Arbitration, `/wheels`, goals and the ROS fallback through a real app |
| `tests/test_health.py` (18) | `control/health.py`'s verdict rules over `/health`, including ROS down under `drive: ros` (handoff 4e) |
| `tests/test_health_sim_map.py` (4) | `sim_map` names the house built, for `mode: sim` and `mode: hardware` with the fake board; null for teleop; unchanged by a later `SIM_MAP` |
| `tests/test_ros_drive.py` (14) | Among the ROS wrapper's tests, a stop that returns in under 0.1 s against a hung bridge ([body engineering](../body/ENGINEERING.md)) |
| `tests/test_r2_routes.py`, `tests/test_ros_goals.py` | `/wheels`, `/scan`, `/world/truth`, goal conversion |
| `tests/test_bridge_keepalive.py` (7) | The bridge polls this server over kept-open connections (3.24 G1) |
| `tests/test_http_rate_live.py` | Live HTTP rate: a bare app holds 200 Hz at p99 1.6-4 ms over the Docker hop; this server's 20-34 ms p99 tail is the simulator sharing the process (3.17). Skips without the stack |
| `tests/test_serverless_routes.py`, `tests/test_static_assets.py` | The deployed vision and walks Lambda routes, and the static files the S3/CloudFront pages reference. **Not this server's routes**: the deployed twin reaches them through `service/tunnel/`'s proxy, which forwards every path except `/brain/...` and `/vision/...` to this server (`tests/test_tunnel_proxy.py`) |
| `tests/test_ui.py` | The twin in a real browser against this server's routes, including the ngrok header the tunnel needs. Skips without `playwright install chromium` |
| `tests/test_ros_containment.py` | No `rclpy` outside `service/slam/` |

Change checklist for a new route: put it under `/world/` if it is about the
house; protect it with `require_secret` unless a load balancer must reach
it; answer an unsupported body or world by name (400/501/`unsupported`); add
the 404 rule to `RemoteRobot`/`RemoteWorld`. If the deployed twin calls
it, no route list needs editing (`service/tunnel/proxy.py` forwards every
path to this server except `/brain`, `/brain/...`, `/vision` and
`/vision/...`, which a robot route must therefore never use); cover the twin's use of it in
`tests/test_ui.py`.

## Known gaps

- **CORS omits `DELETE`**, so a cross-origin browser cannot cancel a goal
  with `DELETE /world/goal`. The twin does not call it today.
- **No reset route**, deliberately; a test needing a fresh robot builds a
  fresh app.
- **Unnamed and teleop drivers under ROS drive** are refused; the gap is
  recorded once, in [safety engineering](../safety/ENGINEERING.md), Known
  gaps.
