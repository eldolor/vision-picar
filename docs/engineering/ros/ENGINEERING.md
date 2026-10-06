---
kind: engineering
domain: ros
status: current
verified: 2026-10-04
parent: docs/ros/ARCHITECTURE.md
---

# ROS -- engineering

This is how the ROS container and the code on the project's side of the
wall are built today. The what and the why are in the
[architecture spec](../../ros/ARCHITECTURE.md). This document is true only
until the implementation changes. The full operator runbook is kept beside
the code in `service/slam/README.md`: its section 3 is the start-up runbook
and its section 4 the one table of failure signatures for the ROS chain and
its bridge, including the world's SLAM readouts. This spec links to both
rather than copying them.

## Implementation

### Inside the container (`service/slam/`)

The container is one image, `FROM ros:humble-ros-base`, built by
`service/slam/Dockerfile`. `service/slam/entrypoint.sh` sources three
environments in order: `/opt/ros/humble`, then `/underlay/install` (tf2,
tf2_ros and slam_toolbox, built from source), then `/ws/install`. It then
`exec`s the command.

| Package | Language | What it does |
|---|---|---|
| `picar_description` | xacro | `service/slam/src/picar_description/urdf/picar.urdf.xacro`. Every dimension sits in one block at the top, tagged `[BOM]`, `[CAD]` or `[PLACEHOLDER]`. It declares the `ros2_control` system `picar`, whose plugin is `picar_sim_hardware/PicarSimHardware`, with velocity command and position/velocity state interfaces on `left_wheel_joint` and `right_wheel_joint`. |
| `picar_sim_hardware` | C++ | `service/slam/src/picar_sim_hardware/src/picar_sim_hardware.cpp`, a `hardware_interface::SystemInterface`. `write()` sends `POST /wheels {left_rad_s, right_rad_s}` with `x-driver: ros` on every cycle, zeros included, because those posts are ROS's heartbeat to the robot server. `read()` does `GET /wheels` and reads `left` and `right` `position_rad` and `velocity_rad_s`. libcurl keeps one connection open. |
| `picar_bridge` | Python | `bridge.py` is the `Bridge` node plus a `ThreadingHTTPServer` on `BRIDGE_PORT`. `convert.py` holds the wall's conversions (scan, quaternion, and since `PLAN-ros-alignment.md` 3.36 `odometry_zero_in_house()`) and imports no rclpy. The node runs on a `MultiThreadedExecutor`; the 10 Hz scan poll (a blocking HTTP read of the robot's `/scan`) has its own `MutuallyExclusiveCallbackGroup` (3.36), because in the node's default group it starved every subscription on the Jetson: a full scan took about one timer period, the timer was always due, rclpy serves due timers first, and `/odom` went unread for minutes (nav2's odom TF went stale and every goal stalled). The start truth is recorded by its own thread from node start (next paragraph). `brain_view.py` maps brain status to diagnostics and markers, with no rclpy. `keepalive.py` holds one kept-open HTTP connection per thread, with no rclpy. |
| `picar_bringup` | launch + yaml | `service/slam/src/picar_bringup/launch/picar.launch.py` starts everything (next table). Its `config/` directory holds `controllers.yaml`, `twist_mux.yaml`, `slam.yaml` and `nav2.yaml`. |

**The start truth** (`GET /slam/pose`'s `start_truth`, what
`world/ros_world.py` anchors SLAM's frame to). A thread started with the
node waits for the controllers' first `/odom`, then reads the robot's
`/world/truth`, and composes the house pose at odometry zero from that
truth and the odometry held at the same instant
(`convert.odometry_zero_in_house(truth, odom)`, 3.36). It retries a failed
read until `START_TRUTH_WINDOW_S`, stops at the first answer (usable, or
`usable: false` on hardware, which leaves it null), and logs an error if
the window closes with none. It used to be the truth at the first
successful scan poll, on the assumption the robot had not moved; on the
Jetson that read came after motion (the chain suite had begun driving, or
a person drove the fallback through a container restart) and anchored
SLAM's frame about 20 degrees off for the session.

**Built from source** (the architecture's D10 says why source builds exist
at all; these are today's):

| Component | Pinned at | Why | Drop it when |
|---|---|---|---|
| `tf2`, `tf2_ros` | 0.25.24, `GEOMETRY2_SHA` in `service/slam/Dockerfile` | An ABBA deadlock in Humble's packaged tf2 (0.25.23) froze nav2's costmap TF listeners, and the controller then "reached" every goal instantly. A debugger on the frozen process established the lock cycle, after an earlier explanation (scan stamps) was withdrawn (3.15). | apt ships 0.25.24 or later |
| `slam_toolbox` | `SLAM_TOOLBOX_SHA` in `service/slam/Dockerfile` | The apt release lacks `restamp_tf`, so a robot at rest publishes an ever-older `map -> odom` until nav2 refuses it (3.15). | apt ships a release with `restamp_tf` |

The nodes `picar.launch.py` starts:

| Node | Notes |
|---|---|
| `robot_state_publisher` | The xacro expanded with `robot_url:=$ROBOT_URL`. |
| `controller_manager/ros2_control_node`, plus spawners | `joint_state_broadcaster` and `diff_drive_controller`. |
| `twist_mux` | `cmd_vel_out` is remapped to `/cmd_vel_mux`. |
| `picar_bridge/bridge` | The HTTP side of the wall. |
| `slam_toolbox/async_slam_toolbox_node` | Configured by `slam.yaml`. |
| nav2 `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` | `cmd_vel` is remapped to `cmd_vel/nav`. They run under `lifecycle_manager_navigation` with autostart on. |
| `nav2_collision_monitor/collision_monitor` | Reads `/cmd_vel_mux` and writes `/diff_drive_controller/cmd_vel_unstamped`. It has its own lifecycle manager. |
| `foxglove_bridge` | Port 8765, address `0.0.0.0`, capabilities `["connectionGraph"]` only. |

### On the project's side (no ROS)

| File | What it does |
|---|---|
| `robot/ros_drive.py` | `RosDriveRobot(inner, bridge_url, secret="", timeout_s=2.0)`. Under `drive: ros`, `robot/factory.py` wraps the backend in it. Each verb becomes `POST <bridge>/cmd_vel` at `CONTROL_HZ`, closed on `inner.get_wheel_state()`: a main pass, then at most two signed correction passes after the zero lands. A verb ends early if no progress is made for `STALL_S`. `stop()` and its constants are specified once, in the [body engineering spec](../body/ENGINEERING.md) ("The ROS drive stop"). Every read goes straight to `inner`. |
| `robot/server.py` | Under `drive: ros` (`by_velocity`), `POST /wheels` accepts only driver `ros` (otherwise `not_the_actuator`), and each post stamps `last_ros_post_at`. `ros_up()` is true when the last post is younger than `ROS_SILENCE_S` and `RosDriveRobot.bridge_up()` is true; the bridge-liveness rule is specified in [safety engineering](../safety/ENGINEERING.md) ("ROS liveness"). While ROS is down, a person's `/action` runs through `fallback_safety`, a `SafetyController` over `robot.inner`, and an autonomous `/action` is refused `ros_unavailable`; while the bridge alone is down, the plugin's posts are answered `ros_unavailable` and not applied. `/health` reports `drive.mode`, `drive.ros_up`, `drive.bridge_up` and `drive.ros_post_age_s`. |
| `world/ros_world.py` | The world's SLAM backend. See the [world engineering spec](../world/ENGINEERING.md). |

## Interfaces

### The bridge's HTTP routes (port 8090)

All routes except `/health` require `x-app-secret` when `APP_SHARED_SECRET`
is set, and answer 401 otherwise. There are 13 method+path pairs, exactly
the `MAX_BRIDGE_ROUTES` budget.

| Route | Request | Answer |
|---|---|---|
| `GET /health` | -- | `ok`, `robot_url`, `odom_age_s`, `scan_age_s`, `scans_published`, `twists` (per driver), `pan_rad`, `session`, `map_version`, `brain_url`, `brain_polls`, `brain_age_s`, `brain_error` |
| `POST /cmd_vel` | `{driver, linear_m_s, angular_rad_s}` | 200 `{published: <topic>}`, or 400 `unknown driver`. A non-zero `twin-dpad` twist also cancels the active goal. |
| `GET /odom` | -- | The latest `/diff_drive_controller/odom` as `{x_m, y_m, yaw_rad, linear_m_s, angular_rad_s, at}`, or `{usable: false}` |
| `GET /scan` | -- | The last LaserScan published, as `{angle_min_rad, angle_increment_rad, ranges_m (None = no return), at}` |
| `GET /tf?target=&source=` | Defaults `base_link` and `laser` | `{translation_m, quaternion_xyzw, yaw_rad, pitch_rad, roll_rad}`, or 404 |
| `POST /pan` | `{angle_rad}` | Holds the published `pan_joint` angle. A test hook for R3. |
| `GET /slam/pose` | -- | `{session, start_truth, map, odom, at}`. `map` and `odom` are `{x_m, y_m, yaw_rad}` of `base_footprint` in that frame, or null. |
| `GET /slam/map` | -- | `{session, version, resolution_m, width, height, origin_x_m, origin_y_m, origin_yaw_rad, data, at}`, ROS 0-100 / -1 |
| `POST /goal` | `{x_m, y_m, yaw_rad}`, map frame | 200 `{state: "pending"}`, or 503 when nav2's `navigate_to_pose` has no server within 2 s. It replaces any existing goal. |
| `GET /goal` | -- | `{session, goal: {x_m, y_m, yaw_rad, state, since, [ended, cancel_reason]}, plan: [[x, y], ...]}`. `state` is one of `pending`, `active`, `succeeded`, `aborted`, `canceled`, `rejected`, `status_<n>`. |
| `DELETE /goal` | -- | `{cancelled: bool}` |
| `GET /nav/stats`, `POST /nav/stats/reset` | -- | `{applied, reversals, metres, monitor_stops, monitor_slows, since}`, R6's flicker and collar counts |

### Topics and frames

**Driver to `twist_mux` input** (`DRIVER_TOPICS`):

| Driver | Topic | Priority |
|---|---|---|
| `twin-dpad` | `cmd_vel/teleop` | 100 |
| `brain` | `cmd_vel/brain` | 50 |
| `ros` | `cmd_vel/nav` | 50 |

The robot server picks the input by the driver's RANK, not its name
(`robot/ros_drive.py` `ros_input_for()`, handoff 3a, decided 2026-10-05):
every manual-rank driver -- `twin-dpad`, `teleop-operator`, an unnamed
caller -- posts on `twin-dpad`'s input; `ros` on its own; every other
autonomous driver (`brain`, `teleop`) on `brain`'s. M4's `/action`
arbitration has already chosen between people and autonomy by then.

nav2's controller and behaviours also publish on `cmd_vel/nav`.

**Command path.** `twist_mux` publishes `/cmd_vel_mux`. `collision_monitor`
passes it on to `/diff_drive_controller/cmd_vel_unstamped`.
`diff_drive_controller` then commands the wheel joints.

**What the bridge publishes:**

- `/scan` (`sensor_msgs/LaserScan`, frame `laser`, at `SCAN_HZ`).
- `/joint_states` (`pan_joint`, at `PAN_HZ`).
- `/brain/status` (`std_msgs/String`).
- `/diagnostics`.
- `/brain/markers` (frame `base_footprint`).

**What the bridge subscribes to:** `/diff_drive_controller/odom`, `/map`
(transient local), `/plan`, `/cmd_vel_mux` and
`/diff_drive_controller/cmd_vel_unstamped`.

**Scan stamps.** Each scan is stamped with the newest
`odom -> base_footprint` transform time, so it can never arrive ahead of TF
(3.15).

**TF tree.**

- `map -> odom` comes from slam_toolbox.
- `odom -> base_footprint` comes from diff_drive_controller.
- `base_footprint -> base_link` is fixed, raised by `wheel_radius`.
- `base_link` has these children: `left_wheel` and `right_wheel`
  (continuous), `laser`, `front_bumper` and `rear_bumper` (fixed), and
  `pan_link`. `pan_link` is a revolute joint carrying `camera_link`, which
  carries `camera_optical_frame`.

**Conventions.** The ROS side follows REP-103 (counter-clockwise yaw) and
REP-117 (`+inf` means no return). The project side is clockwise, and uses
`None` for no return. `convert.ros_ranges()` maps beam i to beam j.

### The robot server's side

| Item | Behaviour |
|---|---|
| `POST /wheels` under `drive: ros` | Only `x-driver: ros` is accepted. Each post counts toward `wheel_posts` and refreshes `last_command_at`, so it feeds the watchdog. The command is still vetted by `SafetyController.vet_wheel_velocity()` and re-vetted by the 20 Hz wheel loop. |
| `/action` under `drive: ros` | M4 arbitration runs first. With ROS up, the verb runs `RosDriveRobot` through `safety.check_and_execute()`, and an `httpx.HTTPError` or `RuntimeError` gives `robot.stop()` plus `ros_unavailable`. With ROS down, a person runs `fallback_safety` and the result is tagged `via: direct-fallback`; an autonomous driver is refused `ros_unavailable`. The verb's twists go on the `/action`'s driver's twist_mux input; a driver with no input (anything but `twin-dpad`, `brain` and `ros`) gets 400 `unknown driver` from the bridge, which surfaces as `ros_unavailable` (Known gaps) without marking the bridge down. |
| Stop (`RosDriveRobot.stop()`) | Specified in the [body engineering spec](../body/ENGINEERING.md) ("The ROS drive stop"), including `STOP_ZERO_TIMEOUT_S` and `STOP_HOLD_S`. What the ROS side determines: the hold must outlast twist_mux's input `timeout` plus `diff_drive_controller`'s `cmd_vel_timeout` (the two ADD) plus one plugin period, so a change to either yaml timeout below means re-deriving the hold there. A person's stop also ends any nav2 goal ([safety engineering](../safety/ENGINEERING.md), "Stop ends a nav2 goal"). The ROS-side facts: the server's `DELETE /goal` reaches `Bridge.cancel_goal()`, which cancels an accepted handle; for a goal still `pending` it records `cancel_reason`, answers `cancelled: true`, and `_on_goal_response()` cancels the goal the moment nav2 accepts it and marks it `canceled`. |

## Parameters and configuration

| Key or constant | Default | Unit | Where read | Why this value |
|---|---|---|---|---|
| `drive.mode` / `ROBOT_DRIVE` | `direct` | -- | `robot/factory.py` | `ros` becomes the car's default only after G4 (3.24); G4 is met (3.33). |
| `ROS_BRIDGE_URL` env | unset | URL | `robot/factory.py` and `world/factory.py` | Overrides both keys below. Set this, not a yaml key, to move the bridge: it is the only setting both factories read. |
| `drive.bridge_url` | `http://127.0.0.1:8090` (shipped yaml and code default) | URL | `robot/factory.py` only | Where verbs go. |
| `world.bridge_url` | not in the shipped yaml; code default `http://127.0.0.1:8090` | URL | `world/factory.py` only | Where `RosWorld` reads map, pose and goals. Setting `drive.bridge_url` alone sends verbs and world reads to different bridges. |
| `ROBOT_URL` (container) | `http://host.docker.internal:8000` | URL | bridge, launch file (xacro arg), and the plugin, where the env var wins | Docker Desktop's name for the host. On Linux use `--network host` and `127.0.0.1`. |
| `TRACK_SCRUB` (container) | `1.0` | > 0 | `picar.launch.py` (`_controllers_with_scrub()`) | Skid steer's effective/geometric track (3.35). Not 1.0: the launch writes a copy of `controllers.yaml` with `wheel_separation_multiplier` set to it. The robot server reads the same variable (`hardware.track_scrub`); set both on the car. `[PLACEHOLDER]` until measured |
| `BRAIN_URL` (container) | `http://host.docker.internal:8001/brain`; `""` turns it off | URL | bridge | The `/brain` prefix is what `service/tunnel/run.sh` sets. |
| `BRIDGE_PORT` | 8090 | port | bridge | -- |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | -- | image `ENV` | The architecture's D10. |
| `GEOMETRY2_SHA`, `SLAM_TOOLBOX_SHA` | `404b722...`, `1729c0f...` | git SHA | `service/slam/Dockerfile` | The tf2 0.25.24 ABBA deadlock fix; slam_toolbox with `restamp_tf`. |
| `ROS_SILENCE_S` | 0.5 | s | `robot/server.py` | Ten missed 20 Hz posts: well past jitter, and inside the 1 s watchdog (G3). |
| `WHEEL_LOOP_INTERVAL_S` | 0.05 | s | `robot/server.py` | 20 Hz, the rate nav2 emits at. Wall duplicate "control rate". |
| `CONTROL_HZ` | 20.0 | Hz | `robot/ros_drive.py` | The same 20 Hz. Wall duplicate. |
| `TURN_RATE_RAD_S`, `ANGULAR_GAIN_PER_S` | 1.2, 1.5 | rad/s, 1/s | `robot/ros_drive.py` | At 2 rad/s and a gain of 3, a 45-degree turn landed at 59-74 degrees over 40-150 ms of jitter (3.13). |
| `MIN_ANGULAR_RAD_S`, `ANGULAR_TOLERANCE_RAD` | 0.05, 0.5 deg | rad/s, rad | `robot/ros_drive.py` | At 0.10 and 0.8 deg, 80% of clear turns landed within 0.64 deg. At 0.05 and 0.5, 100%, worst 0.49 (3.24 G2). |
| `LINEAR_GAIN_PER_S`, `MIN_LINEAR_M_S`, `LINEAR_TOLERANCE_M` | 2.0, 0.02, 0.004 | 1/s, m/s, m | `robot/ros_drive.py` | Straight verbs land within 4.4 mm (3.13). |
| `STALL_S` | 0.6 | s | `robot/ros_drive.py` | No encoder progress for this long means a wall or the veto. The verb ends. |
| `kTimeoutMs` | 40 | ms | `picar_sim_hardware.cpp` | One 50 ms controller cycle. A slower round trip is stale. |
| `update_rate` | 20 | Hz | `controllers.yaml` | Matches the wheel loop and the nav2 controller. |
| `wheel_radius`, `wheel_separation` | 0.040, 0.172 | m | `controllers.yaml` | What `diff_drive_controller` reads. The physical values, their sources and every other copy are in the [platform engineering spec](../platform/ENGINEERING.md); `tests/test_urdf.py` and the wall linters pin the copies equal. |
| `cmd_vel_timeout` | 0.25 | s | `controllers.yaml` | It adds to twist_mux's 0.25, so silence stops the wheels within 0.5 s (R4 criterion 6). The ROS drive stop's hold is derived from this sum (body engineering spec). |
| `linear.x.max_velocity`, `angular.z.max_velocity` | +/-0.6, +/-6.0 | m/s, rad/s | `controllers.yaml` | 0.6 m/s is the sim's ceiling: 2 cells per second (wall duplicate). |
| twist_mux `timeout` / `priority` | 0.25 s; teleop 100, brain 50, nav 50 | s, -- | `twist_mux.yaml` | The order mirrors `DRIVER_PRIORITY` (wall duplicate). |
| `resolution` | 0.05 | m | `slam.yaml` | -- |
| `max_laser_range` | 12.0 | m | `slam.yaml` | The 12 m the sim's scan reaches (`LIDAR_RANGE_M`; wall duplicate). At 4.2 m, SLAM mapped almost nothing in a 16 m house. |
| `restamp_tf`, `transform_publish_period`, `map_update_interval` | true, 0.05 s, 1.0 s | -- | `slam.yaml` | `restamp_tf` stops `map -> odom` going stale while the robot is at rest (3.15). |
| `footprint` / `FootprintApproach.points` | +/-0.1265 x +/-0.1155 | m | `nav2.yaml` | Half the chassis outline; the outline itself is in the [platform engineering spec](../platform/ENGINEERING.md). A wall duplicate with `robot/safety.py`. |
| `inflation_radius`, `cost_scaling_factor` | 0.12, 8.0 | m, -- | `nav2.yaml` | Kept small for doors. |
| Regulated Pure Pursuit `desired_linear_vel`, `lookahead_dist` | 0.20, 0.30 | m/s, m | `nav2.yaml` | The flat 0.2 m/s cap is what makes the safety layer's fixed 20 cm stop sound (`PLAN-ros-alignment.md` question 9, speed set by clearance). No recorded reason for the lookahead. |
| `xy_goal_tolerance`, `yaw_goal_tolerance` | 0.10, 6.28 | m, rad | `nav2.yaml` | Position goals only. |
| NavFn `allow_unknown`, `tolerance` | true, 0.10 | --, m | `nav2.yaml` | -- |
| collision_monitor `time_before_collision`; `PolygonSlow` `slowdown_ratio` | 1.0 s; 0.5 | s, -- | `nav2.yaml` | The monitor projects the footprint along the command (`approach`) and has no `stop` polygon. A stop polygon froze the robot against a door jamb, because Humble's stop action refuses every command, turning away included (3.15). This is how the architecture's "a collision guard never blocks turning away" is kept. |
| `SCAN_HZ`, `PAN_HZ`, `BRAIN_HZ` | 10, 20, 2 | Hz | `bridge.py` | The scan reaches about 10 Hz (R4 criterion 7 needs at least 5). The brain ticks at about 4 Hz. |
| `START_TRUTH_WINDOW_S` | 30 | s | `bridge.py` | How long the start-truth thread waits for the controllers' first odometry and one answer from the robot's `/world/truth` (3.36). Generous for a slow board; the anchor no longer assumes the robot is at rest, so waiting costs nothing. The live chain and nav suites wait 35 s for `start_truth` before they move the robot. |
| `KeepAliveClient` timeout | 0.5 (robot), 1.0 (brain) | s | `bridge.py` | New connections through Docker Desktop stalled past 0.5 s on 41 of 433 polls. A kept-open connection stalled on 0 of 595 (G1). |
| `MAX_DUPLICATES`, `MAX_BRIDGE_ROUTES` | 11, 13 | count | `tests/test_wall_linters.py` | Raise only in a commit that says why. Eleven since 3.27's lidar offset. |

## Procedures

**Start-up, health, inside the container, fresh map, back to direct:**
`service/slam/README.md` section 3 ("Run it") and section 4 ("Is it
working?"). The start-up order there (robot server first, then the
container) still holds; waiting for `GET /wheels` to report `usable: true`
first is optional since handoff 2a (Known gaps).

**Failure signatures** are kept in one place, `service/slam/README.md`
section 4 ("Failure signatures"). Read them there.

**The live suites** skip without the stack. Export `LOCAL_SECRET`, not
`APP_SHARED_SECRET`, into pytest's environment.

```bash
pytest tests/test_urdf.py tests/test_ros_chain_live.py tests/test_brain_view_live.py -v  # server on scaled_house
pytest tests/test_slam_live.py -v                     # server on starter_house, WORLD_MODE=ros
SIM_MAP=scaled_house pytest tests/test_nav_live.py -v
SIM_MAP=scaled_house python -m tests.demo_nav_goals   # R6's six goals, on ground truth
```

## Verification

Always-run tests (no Docker; counts from `pytest --collect-only`,
2026-10-02):

| Test file | Tests | What it pins |
|---|---|---|
| `tests/test_ros_containment.py` | 3 | No file outside `service/slam/` imports `rclpy`, `tf2_ros`, `nav_msgs`, `geometry_msgs` or the other ROS modules listed (a source scan). |
| `tests/test_wall_linters.py` | 18 | Eleven registered duplicates each agree across the wall. No unlisted non-round physical constant is copied. Both budgets hold. No generic routes. Every bridge route has a consumer outside the container and appears in `bridge.py`'s docstring. |
| `tests/test_urdf.py` | 20 | The always-run half: the xacro's wheel radius and separation equal the other copies. The live half (`check_urdf`, 15 tf2 lookups against numpy forward kinematics within 1 mm and 0.1 deg) skips without a container. |
| `tests/test_cad_geometry.py` | 13 | The `[CAD]` values from Waveshare's URDF (3.27). |
| `tests/test_ros_drive.py` | 20 | The verb executor against a fake chain carrying R4's measured 40-150 ms jitter. The stop (2026-10-02): a hung bridge does not delay it, every ROS input is still zeroed, repeated stops on a hung bridge start no extra threads, a stale ROS command cannot undo it, and the hold ends on its own and on the next verb. A 4xx from the bridge does not mark it down; a 5xx or transport error does (2026-10-03). |
| `tests/test_ros_verb_safety.py` | 7 | G2 through `tests/ros_verb_sweep.py`: travel-to-contact at least 18.0 cm after every move (0/1440 under, closest 19.73), no contact, a verb achieving under 1 cm or 0.5 deg is a refusal (0/252 unrefused), clear turns 100% within 0.64 deg (worst 0.49). |
| `tests/test_ros_fallback.py` | 10 | G3 offline: the pulse, the person-only fallback, autonomy refused, recovery with no restart. Handoff 1d: a dead bridge behind a live plugin marks ROS down, and ROS is back once the bridge answers; the plugin's posts move nothing meanwhile, and `/health` names which half is down (2026-10-03). |
| `tests/test_bridge_convert.py`, `tests/test_bridge_keepalive.py`, `tests/test_brain_view.py` | 16, 7, 19 | The bridge's plain-Python parts. |

Live tests (skip without the stack):

| Test file | Tests | Recorded |
|---|---|---|
| `tests/test_ros_chain_live.py` | 13 | R4: verbs within 4.4 mm / 0.64 deg; stopped at least 19.4 cm from a wall; silence stops within 0.5 s; scan about 10 Hz. G1: 20/20 consecutive runs (2026-09-29). Since 3.36 it waits for the bridge's `start_truth` before moving, and its first test fails if the bridge's `odom_age_s` is not under 0.5 s (the starved-subscription defect). |
| `tests/test_nav_live.py` | 5 | R6, two runs: 6/6 goals, end error 0.090-0.133 m, never nearer than 0.165 m to a surface, 0.76-0.81 reversals per metre, unreachable goal aborted in 19-24 s, tap cancels in 0.04-0.05 s, `safety.py` clamps 0. |
| `tests/test_slam_live.py` | 3 | R5's lap; numbers in the [world engineering spec](../world/ENGINEERING.md). |
| `tests/test_brain_view_live.py` | 5 | The bridge's brain topics. |
| `tests/test_http_rate_live.py` | 2 | HTTP at 20 Hz over the Docker hop. A bare app holds p99 1.6-4 ms. The robot server's 20-34 ms tail is the simulator sharing its process (3.17, under `mode: sim`; the fake-board configuration has run the simulator in separate programs since 3.36). |

G3 live (3.24): the mission ended `failed` 2.04 s after a container kill; a
D-pad move executed 0.36 s after it; ROS was up at the first post on return.

Checklist for a change:

- Run `pytest tests/test_ros_containment.py tests/test_wall_linters.py
  tests/test_ros_drive.py tests/test_ros_verb_safety.py
  tests/test_ros_fallback.py`.
- After changing any yaml or the xacro, rebuild the image and run the chain
  suite in suite order, not with `-k`.
- After changing nav2 or SLAM settings, also run `tests/demo_nav_goals.py`
  on `scaled_house`.

## Known gaps

- **G4 is met only on the fork firmware** (3.33, 2026-10-05: 5 consecutive
  runs of `18 passed` on the Jetson); on the stock firmware the +/-2 deg
  turn test fails about one run in three (3.25). Since 2026-10-02 `/health`'s `sim_map`
  names the house actually built, under `ROBOT_MODE=hardware
  SIM_MOTOR_BOARD=fake` too, so the chain and nav2 suites run there instead
  of skipping. A skip is still not a pass. Since 3.36 that configuration
  runs the simulated body as separate programs (`sim/body_server.py` on
  :8002, `sim/sensor_server.py` on :8003 and :8004), which
  `service/tunnel/run.sh` starts; the robot server alone refuses to start
  without `SIM_BODY_URL` / `SIM_SENSORS_URL`.
- **"No wheels" deactivates the plugin only at activation** (since 3.34).
  `on_activate()` in `service/slam/src/picar_sim_hardware/src/picar_sim_hardware.cpp`
  reads `GET /wheels` itself and refuses to come up on `usable: false` (a
  phone walk or a replay has no wheels to drive). Once active, `read()`
  treats `usable: false` like an unreachable server: `note_failure()`,
  velocities zeroed, positions held, `OK`, keep trying. Before 3.34 it
  returned ERROR and ros2_control deactivated the component for good, so a
  robot server restarted under a running container on `HardwareRobot`, or
  (from 3.34 on) any quarter second without board feedback, would have left
  the chain dead. Measured against a stub robot server flipping
  `usable: false` for 2 s: the previous image went `unconfigured` and posted
  nothing afterwards; this one stayed `active` and resumed at 20 Hz (61
  posts in 3 s). **At activation** (handoff 2a, 2026-10-03) it refuses only
  a body with no wheels: `HardwareRobot` answers `awaiting_feedback: true`
  beside `usable: false` before its first frame (and while feedback is
  stale), and the plugin activates on that and waits. Measured live with
  `SIM_BOARD_SILENT_S=30`: the old image logged "reports no wheels" and ROS
  never came up; this one activated, logged `no fresh wheel feedback ...
  still trying`, recovered at the first frame ("reachable again after 584
  failed cycles"), and a LEFT 45 through ROS landed within 40-50 degrees
  (`tests/test_startup_race.py`, live half).
- `picar_sim_hardware` has no unit tests of its own.
- **Stale header comment.** `picar_sim_hardware.hpp`'s header still says a
  `picar_hardware` (R7) "is the same class against the ESP32". 3.16 decided
  that plugin will not be written.
- **`robot_localization` is not installed.** Odometry comes from
  diff_drive_controller alone.
- **Closed 2026-10-05 (handoff 3a): every person drives under `drive:
  ros`.** `teleop-operator`, unnamed callers and the `teleop` driver used to
  post their own names, get 400 `unknown driver`, and be refused
  `ros_unavailable` (`docs-review/REPORT.md` V10). `ros_input_for()` now
  maps them by rank (see "Topics and frames"); `tests/test_ros_drive.py`
  reads `DRIVER_TOPICS` from `bridge.py` so the two cannot drift.
- **`cmd_vel/brain` and `cmd_vel/nav` share priority 50.** Only the robot
  server's arbitration separates them.
- **Stale nav2 comments.** `nav2.yaml`'s header comments reason from the
  starter house, but the parameters are judged on `scaled_house`.
- **The D500 lidar is not modelled.** It is mounted yawed +90 degrees, and
  the sim's scan has zero ahead. A hardware-day item (3.27).
- **The Linux/Jetson run is exercised only against the fake motor board.**
  G4 ran it on the Jetson with `--network host` (`PLAN-ros-alignment.md`
  3.33, met 2026-10-05); with host networking Foxglove listens on every
  interface.
