---
kind: engineering
domain: ros
status: current
verified: 2026-10-02
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
| `picar_bridge` | Python | `bridge.py` is the `Bridge` node plus a `ThreadingHTTPServer` on `BRIDGE_PORT`. `convert.py` holds the wall's two conversions and imports no rclpy. `brain_view.py` maps brain status to diagnostics and markers, with no rclpy. `keepalive.py` holds one kept-open HTTP connection per thread, with no rclpy. |
| `picar_bringup` | launch + yaml | `service/slam/src/picar_bringup/launch/picar.launch.py` starts everything (next table). Its `config/` directory holds `controllers.yaml`, `twist_mux.yaml`, `slam.yaml` and `nav2.yaml`. |

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
| `robot/server.py` | Under `drive: ros` (`by_velocity`), `POST /wheels` accepts only driver `ros` (otherwise `not_the_actuator`), and each post stamps `last_ros_post_at`. `ros_up()` is true when the last post is younger than `ROS_SILENCE_S`. While ROS is down, a person's `/action` runs through `fallback_safety`, a `SafetyController` over `robot.inner`, and an autonomous `/action` is refused `ros_unavailable`. `/health` reports `drive.mode` and `drive.ros_up`. |
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
| `/action` under `drive: ros` | M4 arbitration runs first. With ROS up, the verb runs `RosDriveRobot` through `safety.check_and_execute()`, and an `httpx.HTTPError` or `RuntimeError` gives `robot.stop()` plus `ros_unavailable`. With ROS down, a person runs `fallback_safety` and the result is tagged `via: direct-fallback`; an autonomous driver is refused `ros_unavailable`. The verb's twists go on the `/action`'s driver's twist_mux input; a driver with no input (anything but `twin-dpad`, `brain` and `ros`) gets 400 `unknown driver` from the bridge, which surfaces as `ros_unavailable` (Known gaps). |
| Stop (`RosDriveRobot.stop()`) | Specified in the [body engineering spec](../body/ENGINEERING.md) ("The ROS drive stop"), including `STOP_ZERO_TIMEOUT_S` and `STOP_HOLD_S`. What the ROS side determines: the hold must outlast twist_mux's input `timeout` plus `diff_drive_controller`'s `cmd_vel_timeout` (the two ADD) plus one plugin period, so a change to either yaml timeout below means re-deriving the hold there. A stop cancels no nav2 goal today: nav2 resumes once the hold ends. That a stop also cancels the goal is decided (architecture D6) and not yet built (Known gaps). |

## Parameters and configuration

| Key or constant | Default | Unit | Where read | Why this value |
|---|---|---|---|---|
| `drive.mode` / `ROBOT_DRIVE` | `direct` | -- | `robot/factory.py` | `ros` becomes the car's default only after G4 (3.24). |
| `ROS_BRIDGE_URL` env | unset | URL | `robot/factory.py` and `world/factory.py` | Overrides both keys below. Set this, not a yaml key, to move the bridge: it is the only setting both factories read. |
| `drive.bridge_url` | `http://127.0.0.1:8090` (shipped yaml and code default) | URL | `robot/factory.py` only | Where verbs go. |
| `world.bridge_url` | not in the shipped yaml; code default `http://127.0.0.1:8090` | URL | `world/factory.py` only | Where `RosWorld` reads map, pose and goals. Setting `drive.bridge_url` alone sends verbs and world reads to different bridges. |
| `ROBOT_URL` (container) | `http://host.docker.internal:8000` | URL | bridge, launch file (xacro arg), and the plugin, where the env var wins | Docker Desktop's name for the host. On Linux use `--network host` and `127.0.0.1`. |
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
| `KeepAliveClient` timeout | 0.5 (robot), 1.0 (brain) | s | `bridge.py` | New connections through Docker Desktop stalled past 0.5 s on 41 of 433 polls. A kept-open connection stalled on 0 of 595 (G1). |
| `MAX_DUPLICATES`, `MAX_BRIDGE_ROUTES` | 11, 13 | count | `tests/test_wall_linters.py` | Raise only in a commit that says why. Eleven since 3.27's lidar offset. |

## Procedures

**Start-up, health, inside the container, fresh map, back to direct:**
`service/slam/README.md` section 3 ("Run it") and section 4 ("Is it
working?"). The start-up order there (robot server first, then the
container only once `GET /wheels` reports `usable: true`) is not optional
on `HardwareRobot`; Known gaps says why.

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
| `tests/test_ros_drive.py` | 14 | The verb executor against a fake chain carrying R4's measured 40-150 ms jitter. The stop (2026-10-02): a hung bridge does not delay it, every ROS input is still zeroed, repeated stops on a hung bridge start no extra threads, a stale ROS command cannot undo it, and the hold ends on its own and on the next verb. |
| `tests/test_ros_verb_safety.py` | 7 | G2 through `tests/ros_verb_sweep.py`: travel-to-contact at least 18.0 cm after every move (0/1440 under, closest 19.73), no contact, a verb achieving under 1 cm or 0.5 deg is a refusal (0/252 unrefused), clear turns 100% within 0.64 deg (worst 0.49). |
| `tests/test_ros_fallback.py` | 6 | G3 offline: the pulse, the person-only fallback, autonomy refused, recovery with no restart. |
| `tests/test_bridge_convert.py`, `tests/test_bridge_keepalive.py`, `tests/test_brain_view.py` | 16, 7, 19 | The bridge's plain-Python parts. |

Live tests (skip without the stack):

| Test file | Tests | Recorded |
|---|---|---|
| `tests/test_ros_chain_live.py` | 13 | R4: verbs within 4.4 mm / 0.64 deg; stopped at least 19.4 cm from a wall; silence stops within 0.5 s; scan about 10 Hz. G1: 20/20 consecutive runs (2026-09-29). |
| `tests/test_nav_live.py` | 5 | R6, two runs: 6/6 goals, end error 0.090-0.133 m, never nearer than 0.165 m to a surface, 0.76-0.81 reversals per metre, unreachable goal aborted in 19-24 s, tap cancels in 0.04-0.05 s, `safety.py` clamps 0. |
| `tests/test_slam_live.py` | 3 | R5's lap; numbers in the [world engineering spec](../world/ENGINEERING.md). |
| `tests/test_brain_view_live.py` | 5 | The bridge's brain topics. |
| `tests/test_http_rate_live.py` | 2 | HTTP at 20 Hz over the Docker hop. A bare app holds p99 1.6-4 ms. The robot server's 20-34 ms tail is the simulator sharing its process (3.17). |

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

- **G4 is not run.** No Jetson build or live runs yet (3.33). Since
  2026-10-02 `/health`'s `sim_map` names the house actually built, under
  `ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake` too, so the chain and nav2
  suites run there instead of skipping. A skip is still not a pass.
- **The plugin deactivates for good on "no wheels".**
  `service/slam/src/picar_sim_hardware/src/picar_sim_hardware.cpp:90-94` (`on_activate()` through `read()`) and
  `:157-160` (`read()` returns ERROR on `usable: false`). `HardwareRobot`
  answers `usable: false` until its first frame
  (`robot/hardware_robot.py:349`). Two symptoms, by when it happens:
  - **At start-up** (the container's `on_activate()` reads "no wheels"):
    activation fails, so the controllers never come up.
    `ros2 control list_hardware_components` does not show `picar` active,
    and the bridge's `odom_age_s` stays `null`. The README's start-up order
    avoids this.
  - **Mid-run** (a robot server restarted under a running container
    answers "no wheels" before the board's first frame): `read()` returns
    ERROR and ros2_control deactivates the component. The controllers can
    still read active in `list_controllers` while commanding nothing; the
    plugin's posts stop, so the robot server's `drive.ros_up` turns false
    with the container still up. Nothing recovers it but a container
    restart. Nothing orders around
    this one: on `HardwareRobot`, restart the container after any robot
    server restart. UNCONFIRMED, by reading; the simulator never shows it,
    because `MockRobot` reports wheels at its first answer.

  Not fixed: the plugin could treat `usable: false` like an unreachable
  server.
- **A stop does not cancel a nav2 goal.** `POST /stop` calls only
  `robot.stop()` (`robot/server.py`, `stop()`). `RosDriveRobot.stop()`
  zeroes the twist_mux inputs and cancels nothing, and the bridge cancels a
  goal only on a non-zero `twin-dpad` twist
  (`service/slam/src/picar_bridge/picar_bridge/bridge.py:453-455`). nav2
  keeps publishing on `cmd_vel/nav`, so the wheels resume when
  `STOP_HOLD_S` ends; under `drive: direct` with `WORLD_MODE=ros`, on the
  plugin's next post. **Decided by the user 2026-10-02; not yet built**
  (safety architecture, "Who drives"; this spec's architecture, D6): `/stop`
  also cancels any active goal, on a background thread after
  `robot.stop()`, and a person re-sends a goal to resume. Until it is
  built, cancel with `DELETE /world/goal`.
- `picar_sim_hardware` has no unit tests of its own.
- **Stale header comment.** `picar_sim_hardware.hpp`'s header still says a
  `picar_hardware` (R7) "is the same class against the ESP32". 3.16 decided
  that plugin will not be written.
- **`robot_localization` is not installed.** Odometry comes from
  diff_drive_controller alone.
- **Only `twin-dpad` can drive as a person under `drive: ros`.**
  `DRIVER_TOPICS` (`service/slam/src/picar_bridge/picar_bridge/bridge.py:79-83`) maps only `twin-dpad`,
  `brain` and `ros`. `teleop-operator` and unnamed callers, which rank as a
  person, and the `teleop` driver get 400 `unknown driver`, which
  `robot/server.py:571-575` reports as `ros_unavailable`. The architecture's
  D6 is narrowed to match (`docs-review/REPORT.md` V10, open).
- **`cmd_vel/brain` and `cmd_vel/nav` share priority 50.** Only the robot
  server's arbitration separates them.
- **Stale nav2 comments.** `nav2.yaml`'s header comments reason from the
  starter house, but the parameters are judged on `scaled_house`.
- **The D500 lidar is not modelled.** It is mounted yawed +90 degrees, and
  the sim's scan has zero ahead. A hardware-day item (3.27).
- **The Linux/Jetson run has not been exercised.** With `--network host`,
  Foxglove listens on every interface.
