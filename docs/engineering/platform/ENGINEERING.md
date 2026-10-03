---
kind: engineering
domain: platform
status: current
verified: 2026-10-02
parent: docs/platform/ARCHITECTURE.md
---

# Platform -- engineering

How the hardware is specified, described in code and brought up today. The
what and the why are in the [architecture spec](../../platform/ARCHITECTURE.md).
This document is true only until the hardware or its description changes.

**Much of this is PLANNED.** On 2026-10-02 the Jetson is in hand and its
bring-up (`PLAN-ros-alignment.md` 3.33) is starting. The Rover is ordered
and not delivered. Every item below is tagged:

- **[built]** exists in the repo and is tested;
- **[in hand]** the part has arrived;
- **[planned]** decided and not done;
- **[unmeasured]** a vendor or CAD number not yet checked on the car;
- **[V]** verified against a primary source (a datasheet, a devkit spec, or
  the vendor's own written answer), but not measured by us;
- **[I]** inferred or estimated (a worked budget, an assumption from
  geometry); treat it as a planning number only;
- **[U]** unverified: reported by a third-party search and not re-checked
  (`JETSON-BOM.md` marks these the same way).

The chassis geometry table also carries the robot description's own tags:
**[BOM]** (a product-page or firmware value), **[CAD]** (Waveshare's
CAD-derived URDF) and **[PLACEHOLDER]** (no source yet).

The motor board's serial protocol, firmware fork and serial backend are
specified by the motor-board domain (docs/engineering/motor-board/ENGINEERING.md).

## Implementation

**Hardware inventory:**

| Item | Part | Status | Source of record |
|---|---|---|---|
| Compute | NVIDIA Jetson Orin Nano Super Developer Kit, 945-13766-0000-000, $399 (Amazon, ordered 2026-09-27) | [in hand] since 2026-09-30, return window to about Oct 30 | `JETSON-BOM.md` section 1, `CLAUDE.md` section 3b |
| Chassis kit | Waveshare UGV Rover PT Jetson Orin ROS2 Kit Acce, SKU 29227, ~$730 delivered (Amazon) | [planned] ordered 2026-09-30, expected Oct 19 - Nov 11, 30-day return. Bought on Amazon rather than from Waveshare direct, whose returns were reported as 15 days, shipped to China [U] (`JETSON-BOM.md` 9.7, a search not re-checked): too short for the on-arrival checks | `JETSON-BOM.md` 9.1, 9.7 |
| Motor board | Waveshare "ROS Driver for Robots" (ESP32, closed loop), firmware `ugv_base_ros` | [planned] with the kit | `JETSON-BOM.md` 9.3, motor-board domain |
| Lidar | D500 (LDROBOT STL-19P), 360 degrees, 12 m, 10 Hz | [planned] with the kit | `PLAN-ros-alignment.md` 3.21 |
| Depth camera | Luxonis OAK-D Lite (passive stereo) | [planned] with the kit; which sensors ship is an arrival check | 3.21, 3.26 |
| Pan-tilt camera | the kit's pan-tilt module | [planned] with the kit | 3.26 |
| IMU | ICM-20948 on the motor board | [planned] with the kit | 3.21, 3.26 |
| Jetson battery | e.g. Wheeltec E351S 3S 5100 mAh plus a fused 5.5 x 2.1 to 5.5 x 2.5 mm cable, ~$100-110 | [planned] CONDITIONAL on the arrival stress test | `JETSON-BOM.md` 9.5 |
| microSD | 128 GB A2 | needed for JetPack 6.2.1 | `tools/jetson/README.md` |

**Files in the repo that describe or serve the platform:**

| File | What it does |
|---|---|
| `tools/jetson/README.md` | The bring-up procedure for 3.33, with every command. **Canonical**: Procedures below links to it and adds only expected outputs and pass rules |
| `tools/jetson/setup.sh` | Run on the Jetson: checks L4T R36, prints the power mode, installs the Python 3.10 venv, the CUDA torch and the project requirements pinned against torch, then builds the shipped pipeline. It ASSERTS only that CLIP's scorer is on `cuda`; it prints the detector's weights but not its device (see Known gaps) [built, not yet run on the board] |
| `tools/jetson/bench_perception.py` | Times the shipped perception pipeline per frame, split into GPU and CPU work [built] |
| `tools/jetson/bench_frames.json` | The 60 pinned frames (+3 warm-up) from 20 walks and 7 targets, so laptop and board time the same frames |
| `service/slam/src/picar_description/urdf/picar.urdf.xacro` | The robot description. Every dimension sits in one block, tagged `[BOM]`, `[CAD]` or `[PLACEHOLDER]` |
| `service/slam/src/picar_bringup/config/controllers.yaml` | `diff_drive_controller`'s wheel radius and separation |
| `robot/safety.py` | `LIDAR_X_M`, `FOOTPRINT_LENGTH_M`, `FOOTPRINT_WIDTH_M`, the margins the chassis is vetted with (safety domain) |
| `sim/mock_robot.py`, `robot/hardware_robot.py` | The same chassis constants in the simulator and in the serial backend |

## Interfaces

Physical and host-side interfaces. Device names are expected, not seen.

| Interface | Value | Status |
|---|---|---|
| Jetson power input | Barrel 5.5 x 2.5 mm, 9-20 V | [V] devkit spec |
| Rover supply to Jetson | DC5525 lead from the kit's UPS board | [unmeasured] |
| Motor board serial | Which device, how it is named (a udev symlink, never enumeration order) and `ROBOT_SERIAL` are owned by the motor-board spec (docs/engineering/motor-board/ENGINEERING.md). Which route the kit wires is an arrival check | [planned] |
| Lidar serial | LD19 protocol at 230400 baud, read by the robot process. Opened by a udev symlink (name to be chosen), never by enumeration order such as `ttyACM0`, the same rule as the motor board's | [planned]; protocol inferred from `ldlidar` |
| Serial permissions | Service user in `dialout`; udev rules for stable names (the motor board's in motor-board ENG §Procedures, docs/engineering/motor-board/ENGINEERING.md; B5 in operations) | [planned] |
| SSH | `Host picar-jetson` -> `picar@picar-jetson.local`, key `~/.ssh/id_ed25519_jetson` | [built] on the Mac |
| Code delivery | The Mac pushes over SSH to a non-bare repo on the board (`receive.denyCurrentBranch updateInstead`, a `jetson` remote on the Mac). No GitHub credentials on the robot. One-time setup and the push command: `tools/jetson/README.md` section 2 | [planned] |
| Power mode | `sudo nvpmodel -q` / `sudo nvpmodel -m <id>`; ids from `/etc/nvpmodel.conf` | [planned] |

**Bench output** (`tools/jetson/bench_perception.py`). Per frame (`rows` in
the `--out` JSON): `total_ms`, `model_ms`, `handling_ms`, `det_pre_ms`,
`det_infer_ms`, `det_post_ms`, `clip_gpu_ms`, `clip_cpu_ms`,
`clip_score_ms`, `clip_ms_per_crop` (null with no crops), `crops`,
`status`, `frame`. `model_ms` is `det_infer_ms + clip_gpu_ms`, the two
networks' own compute; `handling_ms` is the rest of `total_ms`. On the
Jetson both networks run on CUDA, so model is the GPU's share and handling
the CPU's; on a Mac the detector runs on the CPU by default, so read the
devices, not the names. The summary carries `detector_device`,
`clip_device`, `frames`, `warmup`, `frames_with_crops`, `median` and `p90`
of every key, `budget_ms` and `within_budget` (median `total_ms` against
the budget).

## Parameters and configuration

**Chassis and mounting geometry** (`base_link` at the rotation centre,
0.040 m above the floor; x forward). **This table is the canonical home of
the chassis' physical constants.** Other specs (body, motor-board, ros,
simulator, safety, policy) link here rather than restate values; the
safety spec owns the margins the chassis is vetted with.

**Where the copies live in code, and what holds each one equal:**

| Fact | Copies | Pinned by |
|---|---|---|
| Wheel radius | xacro `wheel_radius`, `controllers.yaml`, `sim/mock_robot.py` `WHEEL_RADIUS_M`, `robot/hardware_robot.py` `WHEEL_RADIUS_M`, `sim/fake_esp32.py` `MAIN_TYPES[2]` (as the diameter) | `tests/test_wall_linters.py` (all four named files); `tests/test_urdf.py` (xacro, yaml, sim only); `tests/test_ros_driver_board.py` (the fake board's diameter against the sim) |
| Wheel separation | the same four, plus `MAIN_TYPES[2]`'s track | as above |
| Encoder pulses per rev | `sim/mock_robot.py` `ENCODER_COUNTS_PER_REV`, `robot/hardware_robot.py` `COUNTS_PER_REV`, `MAIN_TYPES[2]` | `tests/test_ros_driver_board.py` only (criterion 2) |
| Footprint | `robot/safety.py` `FOOTPRINT_LENGTH_M` / `FOOTPRINT_WIDTH_M` (imported by `sim/grid_world.py`), the xacro body, and three copies in `nav2.yaml`: the local costmap's and the global costmap's `footprint`, and `collision_monitor`'s `FootprintApproach` points | `tests/test_wall_linters.py` |
| Lidar offset | xacro `laser_x`, `robot/safety.py` `LIDAR_X_M` (the sim's scan origin imports it) | `tests/test_cad_geometry.py`, `tests/test_wall_linters.py` |
| Lidar range | `sim/mock_robot.py` `LIDAR_RANGE_M`, `slam.yaml` `max_laser_range` | `tests/test_wall_linters.py` |

The two YAML files are `service/slam/src/picar_bringup/config/nav2.yaml`
and `slam.yaml` beside it.

| Constant | Value | Tag | Where read | Why / source |
|---|---|---|---|---|
| `wheel_radius` / `WHEEL_RADIUS_M` | 0.040 m | [BOM] | xacro, `controllers.yaml`, `sim/mock_robot.py`, `robot/hardware_robot.py`, `sim/fake_esp32.py` (diameter) | 80 mm tyres; firmware mainType 2 `WHEEL_D`. One number (copies table above) |
| `wheel_separation` / `TRACK_WIDTH_M` | 0.172 m | [BOM] | same five | Firmware `TRACK_WIDTH`. CAD's wheel centres are 0.1745 m apart (1.5%); the effective skid-steer track is measured on the car |
| `wheel_separation_multiplier` | not set (controller default) | [planned] | `controllers.yaml` | R8: set from a measured pivot on the car |
| Encoder pulses per wheel rev (`ENCODER_COUNTS_PER_REV`, `COUNTS_PER_REV`) | 660 | [V] Waveshare support | `sim/mock_robot.py`, `robot/hardware_robot.py`, `sim/fake_esp32.py` | Two channels, left and right (3.25). Pinned only by `tests/test_ros_driver_board.py` |
| `body_length` x `body_width` / `FOOTPRINT_LENGTH_M` x `FOOTPRINT_WIDTH_M` | 0.253 x 0.231 m | [BOM] | xacro; `robot/safety.py` (imported by `sim/grid_world.py`) | Product page |
| Corner radius from the rotation centre | 17.1 cm | derived | not a constant; follows from the footprint | What a pivot sweeps; the old 2WD chassis was 15.1 cm (3.19, 3.21) |
| `LIDAR_TO_REAR_BUMPER_CM` | 16.65 cm | derived | `robot/safety.py` | Half the length plus `LIDAR_X_M`: how far astern the body reaches from the scan origin |
| Wheelbase (driven wheels) | 0.171 m | [CAD] | not in the xacro | Rotation centre assumed at its middle [I] |
| `laser_x` / `LIDAR_X_M` | 0.040 m | [CAD] | xacro, `robot/safety.py`, sim scan origin | One number (`tests/test_cad_geometry.py`, wall-linter registry) |
| `laser_z` | 0.080 m (0.120 m off the floor) | [CAD] | xacro | The lidar's scan plane. Anything lower is invisible to it |
| Lidar range (`LIDAR_RANGE_M`) | 12.0 m | [V] datasheet | `sim/mock_robot.py`, `slam.yaml` `max_laser_range` | D500 rated range; the sim's scan and SLAM use it |
| Lidar mounting yaw | +90 deg (zero faces left) | [CAD] | NOT modelled | Must go into the driver's angle offset or the URDF before the first scan is used |
| `pan_x` / `pan_z` | -0.009 m / 0.128 m (0.168 m off the floor) | [CAD] | xacro | Pan axis almost over the rotation centre, which is why R3 criterion 4 now passes (1.89 deg at 1 m, was 4.59) |
| `camera_x` / `camera_up` | 0.048 m / 0.042 m (lens 0.210 m off the floor) | [CAD] | xacro | About double the Stage 0 rig height of 10-13 cm |
| `camera_pitch` | 0.2618 rad (15 deg down) | [PLACEHOLDER] | xacro | The mount's tilt is unmeasured |
| `pan_limit` | 1.5708 rad (+/-90 deg) | [PLACEHOLDER] | xacro | Cable reach, not servo range |
| `deck_height` | 0.040 m | [PLACEHOLDER] | xacro | Estimate |
| Depth camera (`3d_camera_link`) | x +0.065 m, 0.102 m off the floor, level | [CAD] | not in the xacro | Waveshare URDF (3.26) |

**Power** (worked budget at peak, `GUIDE-robot-base.md` section 6, [I]
unless tagged):

| Load or limit | Value |
|---|---|
| Jetson at 15 W / 25 W from an 11 V pack | ~1.4 A / ~2.3 A |
| Four drive motors at full effort | up to ~20 W (`GUIDE-robot-base.md`; `JETSON-BOM.md` 9.5 uses ~15 W peak. The two have not been reconciled; the arrival stress test measures it) |
| D500 lidar / OAK-D Lite / pan-tilt servos | ~1.5 W / ~3-5 W / a few W |
| Peak total at 25 W | ~50-55 W, ~5 A at 11 V |
| Typical total at 15 W, all loads | ~40 W, ~3.6 A (`PI-VS-JETSON.md`) |
| Rover UPS board continuous output | "up to 5 A"; overcurrent trip ~7.5-12.5 A [V Waveshare] |
| 3S pack | 12.6 V full, ~11.1 V nominal, ~9 V empty; Jetson floor 9 V |
| 18650 cells in the Rover holder | 4C or better [V Waveshare] |

**Performance budgets and pins:**

| Parameter | Value | Where | Why |
|---|---|---|---|
| Power mode | 15 W | `nvpmodel` | User, 2026-10-01. NVIDIA rates ~40 TOPS at 15 W against ~67 at 25 W |
| Perception budget | 250 ms a frame (4 Hz) at 15 W | 3.33 criterion 3 | Confirmed by the user 2026-10-02 |
| Safety-loop headroom | 0 late ticks at 20 Hz over a 10-minute nav2 run | 3.33 criterion 6, `/health` `wheel_loop` | Safety must not depend on load |
| Free memory under full load | at least 1 GB | 3.33 criterion 6 | |
| JetPack | 6.2.1 (L4T R36, Ubuntu 22.04); 6.2.2 optional via `apt upgrade` | `tools/jetson/setup.sh` checks `R36` | Matches the Humble container |
| Devkit firmware | UEFI 36.0 or newer | `HARDWARE-BOM.md` 5.1 | Older cannot boot JetPack 6 |
| Python on the board | 3.10 | `tools/jetson/setup.sh` | NVIDIA's CUDA torch exists only for cp310 |
| torch / torchvision | 2.8.0 / 0.23.0 from `pypi.jetson-ai-lab.io/jp6/cu126`, plus `libcusolver-12-6` | `tools/jetson/setup.sh` | A plain `pip install torch` gets a CPU build on aarch64; PyPI's cu126 wheels fail with "no kernel image is available" |
| First-run weights | `yoloe-11s-seg.pt` 28 MB, `mobileclip_blt.ts` 572 MB, plus CLIP | downloaded into the working directory | Gitignored |

## Procedures

**Bring-up before the Rover** (3.33). The canonical procedure, with every
command, is `tools/jetson/README.md`; it is not copied here, because a copy
had already dropped its one-time git setup. Each step is a stop point: a
failure is recorded and decided on, never worked around. What each step
must show:

1. **Image** (README section 0): the JetPack 6.2.1 SD image, written with
   balenaEtcher, with Etcher's verify finished. NVIDIA publishes no
   checksum, so the verify is the check.
2. **First boot** (section 1), on the stock 19 V adapter: UEFI 36.0 or
   newer at the Esc menu. Below 36.0, stop and take NVIDIA's JetPack 6
   update path on the stock adapter. User `picar`, hostname
   `picar-jetson`, and `ssh picar-jetson` logs in without a password.
3. **Code** (section 2): the one-time setup on BOTH sides first (a non-bare
   repo on the board with `receive.denyCurrentBranch updateInstead`, and a
   `jetson` remote on the Mac), then the push and checkout. Without the
   one-time setup the push has nowhere to go. Then `recordings/` by rsync
   (about 450 MB).
4. **Setup** (section 3, `tools/jetson/setup.sh`). Expected: `torch 2.8.0
   on <GPU name> -- OK`, then `detector yoloe-11s-seg.pt | scorer device
   cuda`. Failure signatures: `STOP: expected L4T R36 (JetPack 6.x)`,
   `torch.cuda.is_available() is False`, `a requirement replaced torch with
   a CPU build`, `CLIP is not on cuda`. The script does not check the
   detector's device; step 5's `devices:` line does.
5. **Latency** at 15 W (`sudo nvpmodel -m <id>`, record `nvpmodel -q`),
   then at 25 W (MAXN SUPER only on the stock adapter), with
   `python -m tools.jetson.bench_perception --recordings recordings --out
   bench-<mode>.json` from the repo root (`--recordings` is required; run it
   as a module, as the README does). Expected:
   a header line naming host, Python, torch and the CUDA device; `devices:
   detector cuda, CLIP cuda` (anything else fails risk 1, whatever setup
   said); one median / p90 line per key; then `budget 250 ms: WITHIN` or
   `OVER`. The first run downloads about 600 MB of weights.
6. **Suite**: `pytest tests/ -q` from `.venv`, with Chromium installed for
   Playwright first, or the browser tests ERROR rather than skip.
7. **G4**: the live chain and nav suites, 5 runs in a row, against the
   fake motor board. The commands are below this list. **A skip is not a
   pass**: a run counts only if pytest's summary reads `18 passed` with 0
   skipped and 0 failed (`pytest --collect-only -q` on the two files
   collects 18 tests: 13 chain, 5 nav). A run with any skip or failure is
   not one of the five; find why and start the count again.
8. **Headroom**: everything at once for 10 minutes, watching `/health`'s
   `wheel_loop` (0 late ticks at the 20 Hz loop), free memory (at least
   1 GB) and `tegrastats` (no throttling).

**G4, the full command set** (on the board, from the repo root). Each
omission below forces a skip or a failure, which is why none is optional:

| Requirement | Without it |
|---|---|
| A brain on :8001 under `ROUTE_PREFIX=/brain` (`service/tunnel/run.sh` starts one) | `test_a_dpad_tap_still_preempts_a_mission_under_drive_ros` skips "no brain server" (`tests/test_ros_chain_live.py` reads `PICAR_BRAIN_URL`, default `:8001/brain`) |
| `SIM_MAP=scaled_house` for the robot server **and** in pytest's own environment | The server's `/health` `sim_map` gates both suites; `tests/test_nav_live.py` also checks `tests/demo_nav_goals.py`'s `HOUSE`, which that module reads from pytest's `SIM_MAP` |
| The container started only after `GET /wheels` reports `usable: true` | The fake board's `HardwareRobot` answers `usable: false` until its first frame, and a container that activates then loses its wheel plugin for good (`service/slam/README.md` section 3, "Order matters"; docs/engineering/ros/ENGINEERING.md) |
| The container named `picar-ros` | `test_killing_the_container_stops_the_wheels_on_the_robots_own_watchdog` runs `docker kill picar-ros` with `check=True` and FAILS (`PICAR_ROS_CONTAINER` overrides the name). The user running pytest must be able to run `docker` without sudo |
| The secret in pytest's environment (`LOCAL_SECRET` or `APP_SHARED_SECRET`) | The chain suite skips "the robot server wants a secret" |
| A fresh server and a fresh container for each run | The nav suite's mapping lap is a script from the house's start pose, and SLAM keeps its map for the container's lifetime |

```bash
cd ~/vision-picar
docker build -t vision-picar-ros service/slam                  # once; long the first time
set -a; source ~/.vision-picar-local-secrets; set +a           # LOCAL_SECRET (and VISION_SECRET, which run.sh requires to be defined)
export APP_SHARED_SECRET="$LOCAL_SECRET"

# one run; repeat this block five times
docker rm -f picar-ros 2>/dev/null
SIM_MAP=scaled_house ROBOT_DRIVE=ros WORLD_MODE=ros ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake \
  bash service/tunnel/restart.sh                               # robot :8000 and brain :8001/brain, both on 127.0.0.1
curl -s localhost:8000/health | grep -o '"sim_map": *"[a-z_]*"'   # expect "sim_map": "scaled_house"
until curl -s -H "x-app-secret: $LOCAL_SECRET" localhost:8000/wheels | grep -q '"usable": *true'; do sleep 0.5; done
docker run -d --name picar-ros --restart unless-stopped --network host \
  -e APP_SHARED_SECRET -e ROBOT_URL=http://127.0.0.1:8000 -e BRAIN_URL=http://127.0.0.1:8001/brain \
  vision-picar-ros ros2 launch picar_bringup picar.launch.py
until curl -s localhost:8090/health >/dev/null; do sleep 1; done
SIM_MAP=scaled_house .venv/bin/python -m pytest tests/test_ros_chain_live.py tests/test_nav_live.py -rs
```

The container line is `service/slam/README.md`'s Linux form (host
networking, because `run.sh` binds the servers to 127.0.0.1; that form is
not yet exercised on a board). `restart.sh` reports `OK: robot and brain
both running <rev>` only once both answer with the checkout's revision.
Expected from pytest: `18 passed`, and the `-rs` summary lists no skips.
`tools/jetson/README.md` section 4 does not yet carry these requirements;
its owner is to update it from this list.

**On arrival of the Rover** [planned] (`JETSON-BOM.md` 9.5), inside the
30-day window:

1. Stress test at 15 W with the motors working hard, logging the Jetson's
   input voltage. Any sag means buy the separate pack, unplug the kit's
   DC5525 Jetson lead, and never join the two supplies.
2. Read the ESP32 module's shield and note which serial route is wired.
   Naming the device, permissions and first contact with the board follow
   the motor-board spec (docs/engineering/motor-board/ENGINEERING.md).
3. Confirm the feedback frame carries `odl`/`odr`, and check odometry over
   a measured distance, following motor-board ENG §Procedures
   (docs/engineering/motor-board/ENGINEERING.md).
4. Disable Waveshare's `ugv_jetson` app and any `ugv_bringup` /
   `ugv_driver` service, so only our backend opens the port.
5. Measure the CAD geometry: lidar offset and height, pan axis, lens height,
   and the lidar's zero direction.
6. Expect no translation under the safety layer until the lidar driver
   exists; only turns move. The rule and its refusals are canonical in the
   safety spec's Known gaps (docs/engineering/safety/ENGINEERING.md). Wheel
   tests before then are on the stand, wheels off the floor.
7. Once the lidar driver lands, run the safety sweep against the real
   lidar.
8. Before any firmware change, dump the stock ESP32 image. The command and
   the restore path are the motor-board spec's flash procedure; do not
   copy them here.

## Verification

| Test or record | What it proves | Status |
|---|---|---|
| `tests/test_urdf.py` | Wheel radius and separation are one number across xacro, `controllers.yaml` and the sim; the xacro is well formed. Live half: tf2 lookups match numpy FK within 1 mm / 0.1 deg (skips with no container) | [built] |
| `tests/test_cad_geometry.py` | Each `[CAD]` value is the Rover's; one lidar offset; every scan beam ends on a surface seen from the laser, on ground truth | [built] |
| `tests/test_wall_linters.py` | Constants defined on both sides of the ROS wall agree; no unlisted copied physical constant | [built] |
| `tests/test_footprint_safety.py`, `tests/test_pivot_safety.py` | 3.18 and 3.19 bars hold for the 253 x 231 mm body with the lidar 4 cm ahead (3.27: 0 of 2880 sweep runs under 18 cm, 0 of 120 close pivots touched) | [built] |
| `tests/chassis_fit.py` | The Rover fits every room the old chassis reached (13 of 13 in the furnished home) | [built] |
| Python 3.10 suite in Docker on Arm Linux | 1436 passed, 0 failed, 5 errors (3 browser tests without a browser, 2 needing `pillow-heif`) | recorded 2026-10-02 |
| Laptop bench (M1 MacBook Air), defaults | 119 / 156 ms median / p90; detector inference 108 ms; CLIP 31 ms a crop | recorded 2026-10-02 |
| Laptop bench, both models on MPS | 36 / 66 ms; detector 22 ms | recorded 2026-10-02 |
| 3.33 criteria 1-7 on the Jetson | Firmware and OS, torch on GPU, latency at 15 W and 25 W, suite, G4, headroom, reversibility | [planned] not run |

## Known gaps

The open design questions (which camera feeds perception, depth below the
lidar's plane, skid-steer slip, the battery cutoff, the 15 W result) are
the [architecture spec's open questions](../../platform/ARCHITECTURE.md#open-questions)
and are not repeated here. These are the implementation gaps:

- **No battery cutoff code.** Nothing reads pack voltage, warns, stops or
  shuts down before 9 V, and the twin has no readout
  (`HARDWARE-BOM.md` editor's caution).
- **The lidar driver in the robot process is not written**, and the
  simulator has no fake lidar on a pty yet (`PLAN-ros-alignment.md`
  section 6, question 5). What the safety layer then refuses is canonical
  in the safety spec's Known gaps (docs/engineering/safety/ENGINEERING.md).
- **`tools/jetson/setup.sh` asserts only CLIP's device.** It prints the
  detector's weights but never asserts the detector runs on `cuda`, so risk
  1 could close with YOLOE on the CPU inside the return window (the same
  gap is in 3.33 criterion 2). Until the script asserts it, read the
  bench's `devices:` line (Procedures step 5) as the check.
- **`tools/jetson/README.md` section 4 lacks G4's requirements** (a brain
  under `/brain`, `SIM_MAP` in pytest's environment, the `/wheels` wait,
  the `picar-ros` name). Procedures above has the full set; the README's
  owner is to bring it in line.
- **The lidar's +90 degree yaw is not modelled** in the xacro or the sim.
- **Every `[CAD]` value is unmeasured**, and three geometry values are
  still `[PLACEHOLDER]`.
- **`JETSON-BOM.md` section 1 still lists an IMX219 as "buy regardless"**,
  although the kit brings two cameras.
- **No on-board latency number exists yet.** The ~205 ms Orin figure in
  `PI-VS-JETSON.md` was projected for OWLv2, not measured for YOLOE.
- **`HARDWARE-READINESS.md` and `HARDWARE-BOM.md` describe the 2026-09-19
  build** (General Driver board, IMX219, a single pan servo). The Rover kit
  replaced those parts. Use `JETSON-BOM.md` section 9 for the kit.
