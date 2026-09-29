# service/slam -- the ROS 2 container

The only place ROS exists in this project (`tests/test_ros_containment.py`
fails if anything outside this directory imports `rclpy`). One Docker image,
ROS 2 **Humble** on Ubuntu 22.04 (the car is a Jetson Orin Nano Super on
JetPack 6, which is 22.04), running slam_toolbox, nav2, ros2_control and
twist_mux behind an HTTP bridge.

**Read this when** you want to run the robot through ROS (`drive: ros`,
`WORLD_MODE=ros`), run the ROS live tests, or change anything in `src/`.
**It is off by default**: `config/robot.yaml` ships `drive: direct`, so the
twin and the rest of the suite never depend on Docker.

Why ROS is here, and why only here: `PLAN-ros-alignment.md` section 0-1
(the decision) and `PLAN-mapping.md` section 4 (the body/world wall). The
phase records -- what each part was measured to do -- are
`PLAN-ros-alignment.md` 3.12 (R3, URDF), 3.13 (R4, the drive chain), 3.14
(R5, SLAM), 3.15 (R6, nav2), 3.17 (the wall's costs) and 3.18 (the safety
corridor).

---

## 1. The picture

```
 twin / brain (HTTP)                      the container (one process group, DDS stays inside)
 ------------------                      ---------------------------------------------------
 POST /action (verbs)                     picar_bridge :8090  -- HTTP <-> ROS, the wall
   robot/server.py  (M4 arbitration,        |  POST /cmd_vel {driver,...}
   robot/safety.py)                         v
   robot/ros_drive.py: verb -> twists  -->  cmd_vel/teleop (100) | cmd_vel/brain (50) | cmd_vel/nav (50)
                                                   \______________ twist_mux ______________/
 POST /world/goal -> world/ros_world.py              |  /cmd_vel_mux
   -> bridge POST /goal -> nav2 ------------------->|  (nav2 publishes cmd_vel/nav)
                                                     v
                                            collision_monitor  (approach + slowdown, never stop)
                                                     |  /diff_drive_controller/cmd_vel_unstamped
                                                     v
                                            diff_drive_controller  (ros2_control, 20 Hz)
                                                     |  wheel velocity commands
                                                     v
                                            picar_sim_hardware  (C++ SystemInterface plugin)
                                                     |  POST/GET /wheels over HTTP
   robot/server.py  <--------------------------------'   (vetted AGAIN by robot/safety.py)
     -> MockRobot (sim)  or  HardwareRobot -> ESP32 serial (the car)

 robot/server.py GET /scan --> bridge --> /scan (LaserScan) --> slam_toolbox --> /map, map->odom
                                                          \--> nav2 costmaps, collision_monitor
```

Safety runs in **series**: `collision_monitor` can only slow or stop a
command, and `robot/safety.py` vets the result again at the robot server,
last, on every path (3.15). The ROS side cannot move a wheel the robot
server would refuse.

**Seam:** ROS talks to the robot only through `picar_sim_hardware` -> HTTP
`/wheels`. On the car the same plugin talks to the same robot server, whose
backend is `robot/hardware_robot.py` (R7) -- so hardware day is a config
change and no `picar_hardware` plugin is written.

### TF tree (from `src/picar_description/urdf/picar.urdf.xacro`)

```
map --(slam_toolbox)--> odom --(diff_drive_controller)--> base_footprint
  base_footprint --(fixed, +wheel_radius z)--> base_link   (rotation centre)
    base_link --> left_wheel, right_wheel (continuous)
              --> laser                    (fixed, laser_z above)
              --> front_bumper, rear_bumper (fixed)
              --> pan_link (revolute, pan_joint) --> camera_link --> camera_optical_frame
```

Every dimension is in one block at the top of the xacro, tagged `[BOM]` or
`[PLACEHOLDER]` (a hardware-day measurement). Wheel radius and separation
must equal `controllers.yaml` and `sim/mock_robot.py`; `tests/test_urdf.py`
and `tests/test_wall_linters.py` pin them together.

### Conventions -- the one place they flip

ROS side: REP 103 (x forward, y left, z up, yaw **counter-clockwise**
positive, SI units) and REP 105 frames. Project side: bearings and headings
are **clockwise**-positive ("positive = to the robot's right"), the house
frame is x-east / y-south with compass 0 along -y, and a beam with no return
is `None` (ROS: `+inf`, REP 117). The conversion happens in exactly two
places -- `picar_bridge/bridge.py` (+ `convert.py`) and `world/ros_world.py`
-- and nowhere else.

---

## 2. Packages

| Package | What it is |
|---|---|
| `picar_description` | The URDF (xacro). Frames, dimensions, the `ros2_control` block naming the plugin. |
| `picar_sim_hardware` | C++ `hardware_interface::SystemInterface`. Sends wheel velocities to `POST /wheels`, reads `GET /wheels`. An unreachable robot server is NOT an error (it keeps retrying; the robot server's watchdog is what keeps the car safe); a robot that reports no wheels is. No unit tests of its own yet. |
| `picar_bridge` | Python. The HTTP side of the wall (port 8090), `/scan` republisher, pan joint publisher, nav2 goal client, and the read-only brain view (`/brain/status`, `/diagnostics`, `/brain/markers`). `convert.py` and `brain_view.py` import no ROS and are unit-tested on a laptop. |
| `picar_bringup` | `launch/picar.launch.py` (everything, one launch) and `config/` (`controllers.yaml`, `twist_mux.yaml`, `slam.yaml`, `nav2.yaml`). |

Pinned from source in the image, with the reason in the Dockerfile:
`tf2`/`tf2_ros` 0.25.24 (an ABBA deadlock that froze nav2's costmaps) and
`slam_toolbox` at a commit with `restamp_tf`. The middleware is **Cyclone
DDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).

---

## 3. Run it (macOS, Docker Desktop -- the development setup)

Prerequisites: Docker Desktop running; the repo's `.venv` set up
(CLAUDE.md section 1); `~/.vision-picar-local-secrets` defining
`LOCAL_SECRET` (mode 600, never in the repo -- `service/tunnel/run.sh`
reads it and refuses to start without it).

**Order matters: the robot server first, then the container.**

```bash
# 1. Build (long the first time: tf2 and slam_toolbox compile from source)
docker build -t vision-picar-ros service/slam

# 2. Robot + brain, restarted in ROS mode. restart.sh starts run.sh
#    detached and refuses to report success until the NEW processes answer.
#    Add WORLD_MODE=ros for SLAM/nav2 (R5/R6), SIM_MAP to pick the house.
ROBOT_DRIVE=ros WORLD_MODE=ros SIM_MAP=scaled_house bash service/tunnel/restart.sh

# 3. The container. The secret must be EXPORTED in this shell -- a bare
#    `-e APP_SHARED_SECRET` forwards an empty value if it is not.
set -a; source ~/.vision-picar-local-secrets; set +a
export APP_SHARED_SECRET="$LOCAL_SECRET"
docker run -d --name picar-ros --restart unless-stopped \
  -p 8090:8090 -p 127.0.0.1:8765:8765 \
  -e APP_SHARED_SECRET \
  -e ROBOT_URL=http://host.docker.internal:8000 \
  vision-picar-ros ros2 launch picar_bringup picar.launch.py
```

Notes on those choices:

* **`-p 127.0.0.1:8765:8765`** publishes Foxglove to this machine only. The
  launch binds `0.0.0.0` inside the container and offers only the
  `connectionGraph` capability (read-only: no publish, no services, no
  params -- a second door to the wheels would skip `robot/server.py`'s
  arbitration). Keep the 127.0.0.1 bind anyway.
* **The brain view** polls `BRAIN_URL`, default
  `http://host.docker.internal:8001/brain`. The `/brain` prefix is what
  `service/tunnel/run.sh` sets (`ROUTE_PREFIX=/brain`). If you start the
  brain yourself with a plain `uvicorn control.brain_server:app --port
  8001`, pass `-e BRAIN_URL=http://host.docker.internal:8001` or
  `/diagnostics` reads STALE forever. `-e BRAIN_URL=` turns it off.
* **Which house.** nav2 is judged only on `SIM_MAP=scaled_house` (90 cm
  doors). The starter house's 30 cm doors seal in nav2's costmap (0/6 to
  5/6 goals, 3.15). The SLAM lap (`tests/test_slam_live.py`) wants
  `starter_house`. `home_first_floor` is the furnished tour.
* **Fresh map.** SLAM keeps its map for the container's lifetime. For a
  clean run: `docker rm -f picar-ros` and run step 3 again.
* **Odometry drift** (to watch loop closure fix it, R5): add
  `SIM_ODOM_DRIFT=1.0,1.03` (left,right encoder scale -- "right encoder 3%
  long") to step 2. The yaml equivalent is `sim.odom_drift`.
* **Back to normal:** `docker rm -f picar-ros` and a plain
  `bash service/tunnel/restart.sh` (which returns to `drive: direct`).

### On the car (Linux / Jetson) -- NOT yet exercised

`host.docker.internal` does not exist on Linux, and `run.sh` binds the
robot server to `127.0.0.1`, which a bridge-networked container cannot
reach. Use host networking and point both URLs at loopback:

```bash
docker run -d --name picar-ros --restart unless-stopped --network host \
  -e APP_SHARED_SECRET \
  -e ROBOT_URL=http://127.0.0.1:8000 \
  -e BRAIN_URL=http://127.0.0.1:8001/brain \
  vision-picar-ros ros2 launch picar_bringup picar.launch.py
```

With `--network host` the `-p` flags do nothing and **Foxglove listens on
every interface** (the launch's `0.0.0.0`). It is read-only, but on the car
firewall 8765 or change the launch's `address` before this goes on a
network you do not own. Starting this and the robot server at boot is
`PLAN-brain-relocation.md` B5, not built.

### Environment the container reads

| Variable | Default | Read by |
|---|---|---|
| `ROBOT_URL` | `http://host.docker.internal:8000` | bridge, `picar_sim_hardware` (via the xacro arg) |
| `BRAIN_URL` | `http://host.docker.internal:8001/brain` (`""` = off) | bridge (`brain_view`) |
| `APP_SHARED_SECRET` | empty (= no header sent, bridge accepts anything) | bridge (both directions), `picar_sim_hardware` |
| `BRIDGE_PORT` | `8090` | bridge |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` (set in the image) | every node |

And on the robot-server side: `ROBOT_DRIVE=ros` (or `drive: ros`),
`WORLD_MODE=ros`, `ROS_BRIDGE_URL` (default `http://127.0.0.1:8090`),
`SIM_MAP`, `SIM_ODOM_DRIFT` -- `robot/factory.py`, `world/factory.py`.

---

## 4. Is it working?

From the host:

```bash
curl -s localhost:8090/health | python -m json.tool   # no secret needed
```

Healthy: `"ok": true`, `odom_age_s` and `scan_age_s` under ~0.2,
`scans_published` climbing, `brain_error` null (or the brain view off).
`odom_age_s: null` means the controllers never came up; `scan_age_s: null`
means the bridge cannot read the robot server's `/scan`.

Inside the container (`docker exec` bypasses the entrypoint, so go through
it to get the ROS environment):

```bash
docker exec -it picar-ros /entrypoint.sh bash
ros2 control list_hardware_components     # picar: active
ros2 control list_controllers             # joint_state_broadcaster, diff_drive_controller: active
ros2 topic hz /scan                       # ~robot server's scan rate
ros2 topic hz /diff_drive_controller/odom # ~20 Hz
ros2 topic echo /cmd_vel_mux --once       # what twist_mux let through
ros2 run tf2_ros tf2_echo map base_footprint   # needs WORLD_MODE=ros and a map
ros2 run tf2_tools view_frames            # writes frames_*.pdf in the cwd
ros2 lifecycle get /controller_server     # nav2: active
```

To reach the host from inside the container by hand, use IPv4
(`getent ahostsv4 host.docker.internal`): Python's urllib tries an
unreachable IPv6 address first.

Foxglove Studio: open a connection to `ws://localhost:8765`. Every topic is
visible, including the brain's; nothing can be published.

### Failure signatures

| You see | It means |
|---|---|
| Container log: `POST /wheels to ... failed (N in a row) -- still trying` | The robot server is down, unreachable (Linux without host networking), or rejecting the secret (401) -- check `APP_SHARED_SECRET` was exported before `docker run`. The wheels are stopped meanwhile by the robot server's watchdog. |
| Container log: `the robot server reports no wheels (usable: false)` | The robot server is not in a mode with wheels (`drive: direct` still, or a teleop/replay backend). Restart it with `ROBOT_DRIVE=ros`. |
| Robot server refuses `/action` with reason `ros_unavailable` | `drive: ros` and the bridge did not accept the twist. Usually the container is not up -- but ALSO what you get when the `/action` had no `x-driver` header or came from a teleop driver: the bridge maps only `twin-dpad`, `brain` and `ros` onto twist_mux inputs, answers 400 `unknown driver`, and the robot server reports that as unreachable (by code reading; a known gap). Check `curl localhost:8090/health` first. |
| nav2 log: `Transform data too old` and the robot never moves | `map -> odom` is stale: the image was built without the pinned slam_toolbox (`restamp_tf`). Rebuild. |
| nav2 "reaches" every goal in ~0.08 s | TF listeners frozen -- the tf2 deadlock, or Fast DDS instead of Cyclone. Check `echo $RMW_IMPLEMENTATION` in the container and that the image built tf2 from source. |
| Goals abort "off the global costmap" | nav2 cannot plan into a room SLAM has never seen. Map first: `python -m tests.demo_slam_lap`, then goals. |
| `/diagnostics` shows the brain STALE | `BRAIN_URL` prefix mismatch (section 3) or the brain is down. |

---

## 5. Instruments and live tests

All skip (not fail) when the stack is not up. **Export `LOCAL_SECRET`, not
`APP_SHARED_SECRET`, into the pytest environment** -- with
`APP_SHARED_SECRET` set, the in-process `TestClient` tests start demanding
it and fail by the dozen.

```bash
export LOCAL_SECRET=...            # from ~/.vision-picar-local-secrets
pytest tests/test_urdf.py tests/test_ros_chain_live.py tests/test_brain_view_live.py -v   # server on scaled_house (3.24)
pytest tests/test_slam_live.py -v                        # server on starter_house, WORLD_MODE=ros
SIM_MAP=scaled_house pytest tests/test_nav_live.py -v    # server on scaled_house too
python -m tests.demo_slam_lap      # R5's lap: SLAM error vs odometry alone
SIM_MAP=scaled_house python -m tests.demo_nav_goals      # R6: six goals, judged on ground truth
```

`demo_nav_goals` reads `SIM_MAP` from ITS OWN environment for ground truth;
it must match the robot server's (`curl localhost:8000/health` reports
`sim_map`). Both demos talk to `PICAR_ROBOT_URL` (default
`http://127.0.0.1:8000`) and `PICAR_BRIDGE_URL` (default `:8090`).

---

## 6. Known gaps

* **Who drives, inside ROS.** `cmd_vel/nav` and `cmd_vel/brain` share
  twist_mux priority 50. Since `PLAN-ros-alignment.md` 3.23 the robot server
  admits only one of them at a time (a goal is refused while the brain
  drives, and the brain while a goal is active), so the tie does not arise
  through the supported routes. The bridge's `ros` driver also publishes on
  `cmd_vel/nav`.
* `picar_sim_hardware` has no unit tests.
* `robot_localization` is named in `PLAN-ros-alignment.md` section 0 but not
  installed or launched; odometry is `diff_drive_controller`'s alone.
* The Linux/Jetson run above has not been exercised.
* `nav2.yaml` / `slam.yaml` header comments reason from the starter house;
  the parameters are judged on `scaled_house` now.
