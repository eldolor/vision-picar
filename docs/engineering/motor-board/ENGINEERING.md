---
kind: engineering
domain: motor-board
status: current
verified: 2026-10-02
parent: docs/motor-board/ARCHITECTURE.md
---

# Motor board -- engineering

This is how the Rover's motor board, its firmware fork, the serial backend
and the fake board are built today. The what and the why are in the
[architecture spec](../../motor-board/ARCHITECTURE.md). The protocol below
was read from the firmware source: `waveshareteam/ugv_base_ros`, directory
ROS_Driver, at commit `2e7df97`. `sim/fake_esp32.py` is the executable
record of that reading, with each rule citing its firmware function.

`HARDWARE-BOM.md` section 4.2 describes a *different* board, the General
Driver (`ugv_base_general`). That board was open-loop in mainType 2 and had
no odometers. Its correction 5 applies to that board only. Do not use its
command table for the Rover.

## Implementation

| File | What it does |
|---|---|
| `robot/hardware_robot.py` | `HardwareRobot(port, sensors=None, baud=115200)`, a `RobotInterface` backend. It opens the tty raw at 115200 and starts a `select()`-polling reader thread, which parses only `T:1001` frames and skips bad lines. It sends its set-up (`T:136`, `T:131`) at start and again after a detected reboot. It turns wheel commands into `T:1`. It estimates each wheel's travel: a trapezoidal speed integral, clamped by `_anchor()`, with `_absorb_reboot()` and `_track_board_clock()` for the fork. `verb_plan()` and `verb_done()` let `SafetyController.run_verb()` carry verbs out on the wall clock. Camera, lidar, depth and pan are delegated to `sensors`; with none, they answer the interface's unusable defaults (`get_camera_frame()` raises). |
| `sim/fake_esp32.py` | `FakeEsp32(body, main_type=2, drop_rate=0.0, seed=0, firmware=None, millis_at_boot=0)`. It opens a pty (`.path`) and runs a firmware loop thread that turns `body`'s wheels through `set_wheel_velocity()` and `advance()`. It keeps integer encoder counts and measures speeds from count deltas. It sends rate-limited `1001` frames, runs the heartbeat, and has `reboot()`. Its wire is lossy (`drop_rate`) and loses only whole lines. `sent_truth` records the body's wheel travel at each frame actually sent, which is what tests judge against. |
| `robot/factory.py` | `mode: hardware`. With `SIM_MOTOR_BOARD=fake`, it builds `MockRobot`, then `FakeEsp32(body)`, then `HardwareRobot(board.path, sensors=body)`. Otherwise it builds `HardwareRobot(ROBOT_SERIAL or hardware.serial_port)`, and raises if neither is set. |
| `firmware/ugv_base_ros/0001-feedback-fine-odometers-and-board-time.patch` | Our GPL-3.0 fork. It adds 13 lines inside `baseInfoFeedback()` (`ROS_Driver/ugv_advance.h`) and removes none. |
| `firmware/ugv_base_ros/build.sh` | Compiles a checkout, stock or patched, with `arduino-cli`. Compile only. |
| `firmware/ugv_base_ros/README.md` | How to apply, build, dump and restore the firmware. |
| `robot/safety.py` | `SafetyController.run_verb()` and `_settle()` carry out and settle wall-clock verbs. Owned by the safety domain; listed here because `HardwareRobot.verb_plan()` turns the settle pass on. A correction runs through `run_verb()` and is vetted like a move: since 2026-10-02 `reverse_clearance()` answers `(0.0, "astern_not_observed")` on a body whose wheels report but whose scan is unusable, so a correction that would back up ends `clamped` ("settle refused") on `HardwareRobot` with no `sensors`, and so does one that would go forward, because every FORWARD is refused there too (`get_distance()` is 0.0). The rule for a body with no sensors is the safety domain's, specified once in the [safety engineering spec](../safety/ENGINEERING.md) (Known gaps). Over the fake board the sim body supplies the scan (`sensors=body`), so the settle behaves as recorded below. |

## Interfaces

### Serial link

The link is newline-delimited JSON at 115200 baud, 8N1, raw. The board
writes compact JSON (ArduinoJson). The host writes `json.dumps()` output,
which has spaces; the firmware accepts either.

**Commands the host sends:**

| Command | When | Meaning |
|---|---|---|
| `{"T":136,"cmd":1500}` | At start, and after a reboot | Heartbeat: zero the wheel speeds after 1500 ms without a `T:1` or `T:11` |
| `{"T":131,"cmd":1}` | At start, and after a reboot | Continuous feedback on (already the stock boot default) |
| `{"T":1,"L":<m/s>,"R":<m/s>}` | Every wheel command; re-sent by `advance()` while non-zero | Closed-loop wheel surface speeds, `rad/s x WHEEL_RADIUS_M`, rounded to 5 places. The host scales both speeds by one factor into the board's +/-`BOARD_MAX_WHEEL_M_S` (2.0) first, keeping their ratio, because the firmware drops an out-of-range `T:1` and keeps the OLD setpoint running (handoff 4j). |

**Other commands the fake models** (the host never sends these):

| Command | Behaviour in the fake |
|---|---|
| `T:0` | `setGoalSpeed(0, 0)`. See Known gaps. |
| `T:11` | Raw PWM +/-255: turns the PID off and feeds the heartbeat. |
| `T:13` | `{X, Z}` twist. Does NOT feed the heartbeat. |
| `T:130` | One frame now, subject to the rate limit. |
| `T:142` | Feedback interval in ms. |
| `T:900` | `{main}` selects a mainType. |

### Feedback frame `T:1001` (board to host)

| Key | Type | Meaning |
|---|---|---|
| `L`, `R` | float m/s | MEASURED wheel speeds: whole encoder edges per board loop, times `pi x WHEEL_D / pulses`, divided by dt |
| `ax ay az gx gy gz mx my mz` | int | IMU. Zeros in the fake. |
| `odl`, `odr` | int, cm | Each wheel's travel since boot, `(long)(en_odom * 100)`, truncated toward zero. So 0 means (-1, 1) cm. |
| `v` | int, 1/100 V | Pack voltage. The fake sends a constant 1200. |
| `odlt`, `odrt` | int, 0.1 mm | **Fork only.** `(long)(en_odom * 10000)`. One encoder edge is 0.38 mm. |
| `ms` | unsigned long, ms | **Fork only.** `last_feedback_time`, the board's `millis()` when the frame was built. Zero at boot; wraps at 2^32. |

Frames are sent at most once every 50 ms (`feedbackFlowExtraDelay`, set by
`T:142`), whether streamed or requested with `T:130`.

**Frame size** (measured on the fake): stock 125 bytes and the fork 160
bytes. At 20 Hz, the fork uses about 28% of the link.

### mainType constants (`movtion_module.h` `mm_settings()`)

| mainType | Robot | `WHEEL_D` (m) | pulses/rev | `TRACK_WIDTH` (m) |
|---|---|---|---|---|
| 1 | RaspRover | 0.0800 | 2100 | 0.125 |
| 2 | **UGV Rover** (ours) | 0.0800 | 660 | 0.172 |
| 3 | UGV Beast | 0.0523 | 1092 | 0.141 |

The host never sends `T:900`. It assumes mainType 2, the Rover's stock
setting. The 660 is 11 lines x 2 (half quadrature) x 30:1.

### What the backend reports

The shapes are owned by the body contract.

- **`get_wheel_state()`** gives `position_rad` from the estimated travel,
  carried forward up to `EXTRAPOLATE_MAX_S`. Its `velocity_rad_s` is the
  last *commanded* speed, not a measured one. It also gives `counts` (from
  the estimate, at 660/rev), `wheel_radius_m`, `track_width_m` and
  `counts_per_rev`. `usable` turns true at the first frame.
- **`get_odometry()`** gives `distance_m`, the body path: the change in the
  wheels' average, so a pivot adds nothing. Its `heading_deg` comes from
  the wheel difference, clockwise, measured from where the process started.
- **`verb_plan()`** returns `{kind, left_rad_s, right_rad_s, target,
  wall_clock: True, settle, [moves]}`. `settle` is true only while fork
  frames are arriving.

## Parameters and configuration

| Key or constant | Default | Unit | Where read | Why this value |
|---|---|---|---|---|
| `mode` / `ROBOT_MODE` | `sim` | -- | `robot/factory.py` | `hardware` selects this backend. |
| `ROBOT_SERIAL` or `hardware.serial_port` | none (required) | path | `robot/factory.py` | Not in the shipped yaml. Use a udev symlink on the car, never `/dev/ttyUSB0` by enumeration order. |
| `SIM_MOTOR_BOARD` | unset | -- | `robot/factory.py` | `fake` runs the backend over `sim/fake_esp32.py` on a pty. |
| `SIM_BOARD_FIRMWARE` | `stock` | -- | `sim/fake_esp32.py` | `fork` adds `odlt`, `odrt` and `ms`. It re-runs any suite over the fork. |
| `SIM_BOARD_SILENT_S` | 0 | s | `sim/fake_esp32.py` (`silent_until`) | The fake board sends no feedback frame for this long after power-up, to reproduce the start-up race a ROS container started first must ride out (handoff 2a; `tests/test_startup_race.py`). |
| `WHEEL_RADIUS_M`, `TRACK_WIDTH_M`, `COUNTS_PER_REV` | 0.040, 0.172, 660 | m, m, edges/rev | `robot/hardware_robot.py` | What the backend converts with: mainType 2's `WHEEL_D` / 2, `TRACK_WIDTH` and pulses (table above). The physical facts, their sources and every other copy are in the [platform engineering spec](../platform/ENGINEERING.md); `tests/test_wall_linters.py` and `tests/test_ros_driver_board.py` pin the copies equal. |
| `HEARTBEAT_MS` | 1500 | ms | `robot/hardware_robot.py` | Longer than the robot server's 1.0 s `watchdog_timeout_s`, so the server acts first. The firmware default is 3000. |
| `REBOOT_JUMP_M` | 0.03 | m | `robot/hardware_robot.py` | On stock firmware, a reboot needs both odometers near zero AND the estimate this far outside their bucket. A rule on the distance alone fired falsely in 1 of 6 runs (3.25). |
| `EXTRAPOLATE_MAX_S` | 0.10 | s | `robot/hardware_robot.py` | Two feedback intervals. Without carry-forward, a FORWARD overshot to 30.6-32.8 cm (3.25). |
| `TENTH_MM`, `CM`, `MILLIS_WRAP` | 1e-4, 0.01, 2^32 | m, m, ms | `robot/hardware_robot.py` | Units. Millimetres left the at-rest estimate up to 1.26 degrees out (3.29). |
| `MOVE_M`, `MOVES_PER_SECOND_AT_FULL_SPEED` | 0.30, 2.0 | m, 1/s | `robot/hardware_robot.py` | A verb means what it means on `MockRobot`. |
| Turn rate in `verb_plan()` | 1.2 | rad/s (body) | `robot/hardware_robot.py` | The same as `robot/ros_drive.py`'s `TURN_RATE_RAD_S`. |
| `SETTLE_TOLERANCE_DEG`, `SETTLE_TOLERANCE_M` | 0.5, 0.003 | deg, m | `robot/safety.py` | The ROS path's tolerances (3.29). |
| `SETTLE_TURN_RAD_S`, `SETTLE_LINEAR_M_S` | 0.1, 0.02 | rad/s, m/s | `robot/safety.py` | A stop arriving 10 ms late then costs 0.06 deg or 0.2 mm. |
| `SETTLE_WAIT_S`, `SETTLE_PASSES` | 0.15, 3 | s, count | `robot/safety.py` | Lets the wheels stop and three 50 ms frames arrive. |
| `BOARD_LOOP_S` | 0.01 | s | `sim/fake_esp32.py` | The firmware's `loop()` runs at about 100 Hz. Real timing is unmeasured. |
| `DEFAULT_FEEDBACK_INTERVAL_MS`, `DEFAULT_HEARTBEAT_MS` | 50, 3000 | ms | `sim/fake_esp32.py` | `ugv_config.h`. |
| `NO_LOAD_WHEEL_M_S` | 1.3 | m/s | `sim/fake_esp32.py` | Full PWM for `T:11` only: the Rover's rated top speed, assumed. |
| `OUT_BUFFER_BYTES` | 4096 | bytes | `sim/fake_esp32.py` | A USB bridge's buffer. Unread lines are lost whole, and the firmware loop never blocks. |
| `build.sh` pins | esp32 core 3.2.1, INA219_WE 1.3.8, plus nine other libraries | -- | `firmware/ugv_base_ros/build.sh` | Stock `2e7df97` does not compile against INA219_WE 1.4 (renamed constants) or esp32 3.3 (the ESP-NOW callback type changed). |

## Procedures

**Run the whole stack over the fake board:**

```bash
ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake uvicorn robot.server:app --port 8000
ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake SIM_BOARD_FIRMWARE=fork uvicorn robot.server:app --port 8000
```

`/health` answers as usual, and its `sim_map` names the house the sim body
stands in (since 2026-10-02; it was `null` under the fake board, which made
the live ROS suites skip). `GET /wheels` turns `usable: true` within one
feedback interval. With `drive: ros` added (`ROBOT_DRIVE=ros`), R4's live
chain suite runs over the serial line (3.16 criterion 5); start the ROS
container only after `/wheels` reports `usable: true` (the start-up order in
the [ros engineering spec](../ros/ENGINEERING.md)).

**Build the fork** (compile only; about 1 GB of toolchain on first run):

```bash
git clone https://github.com/waveshareteam/ugv_base_ros && cd ugv_base_ros && git checkout 2e7df97
git apply /path/to/vision-picar/firmware/ugv_base_ros/0001-feedback-fine-odometers-and-board-time.patch
bash /path/to/vision-picar/firmware/ugv_base_ros/build.sh .
```

- **Success:** `build/*.bin`. The recorded size is 1,223,030 bytes, 93% of
  the app partition and 236 bytes over stock.
- **Missing toolchain:** `arduino-cli` is found on `PATH`, in `ARDUINO_CLI`,
  or at `~/.local/bin/arduino-cli`.
- **Unpinned libraries:** errors naming `PG_320`, `BRNG_16` or
  `BIT_MODE_9`, or an ESP-NOW callback type, mean the build ran without the
  pins.

**Flash.** This happens on hardware day, after the arrival checks in
`JETSON-BOM.md` 9.5. **UNCONFIRMED: not run.** The commands are
`arduino-cli`'s and esptool's documented ones, with the core `build.sh` pins
(esp32 3.2.1, which carries its own esptool). Stop the robot server and
Waveshare's `ugv_jetson` app first, so nothing else holds the port.

1. Dump the stock image: `esptool.py --port <port> read_flash 0 ALL
   stock-ugv_base_ros.bin`. Keep it off the car.
2. Upload the build from the checkout `build.sh` compiled:

   ```bash
   CLI="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
   "$CLI" upload --fqbn esp32:esp32:esp32 --port <port> --input-dir <checkout>/build
   ```

   Board: "ESP32 Dev Module" (`esp32:esp32:esp32`). Expected: esptool's
   `Connecting...`, `Chip is ESP32-D0WD...`, `Writing at 0x...` lines,
   `Hash of data verified.` for each region, then `Hard resetting via RTS
   pin...`.
3. Confirm the fork's keys with the heartbeat check below: frames now carry
   `odlt`, `odrt` and `ms`.
4. To restore: `esptool.py --port <port> write_flash 0 stock-ugv_base_ros.bin`,
   or Waveshare's ESP32 Download Tool with their published image.

| Symptom | Meaning |
|---|---|
| `A fatal error occurred: Failed to connect to ESP32: No serial data received` | The port is held (the robot server, `ugv_jetson`), the wrong port, or the board did not enter its bootloader: hold BOOT while the upload starts. |
| `Permission denied` / `could not open port` | Not in `dialout`, or the udev rule is missing (below). |
| `Hash of data verified` missing, or a `MD5 of file does not match` error | A bad write. Upload again before power-cycling; restore the dump if it repeats. |
| `Error during Upload: ... no such file` | `--input-dir` does not point at `build.sh`'s `<checkout>/build`. |

**A stable device name (udev).** Skip this if the kit wires the Jetson's
header UART (`/dev/ttyTHS1` is already stable). For the board's USB bridge,
fill the values from the car, never from a guess: the board and the lidar
can both be CP210x, so `idVendor` and `idProduct` alone may match both, and
the `serial` attribute is what tells them apart.

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head
```

`/etc/udev/rules.d/99-picar-motor-board.rules` (the values are to be filled
from that output on the car):

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="<vid>", ATTRS{idProduct}=="<pid>", ATTRS{serial}=="<serial>", SYMLINK+="picar-motor-board", GROUP="dialout", MODE="0660"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/picar-motor-board          # -> ttyUSB<n>
```

`ROBOT_SERIAL=/dev/picar-motor-board` from then on. A
`/dev/serial/by-id/` path is an acceptable alternative.

**On first contact with the real board: the heartbeat, with the robot
server STOPPED.** The server cannot do this check: with no sensors it
refuses every FORWARD and clamps a standing forward `/wheels` to zero
(safety domain). Talk to the board directly instead. Wheels off the floor.
pyserial is installed alongside esptool (`pip install pyserial` otherwise).
UNCONFIRMED: not run against a board.

```python
import json, serial, time
s = serial.Serial("/dev/picar-motor-board", 115200, timeout=0.1)  # opening may reset the board
def send(c): s.write((json.dumps(c) + "\n").encode())
def frames(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        line = s.readline().decode(errors="replace").strip()
        if '"T":1001' in line:
            f = json.loads(line); print(round(time.monotonic() - t0, 2), f["L"], f["R"], f["odl"], f["odr"], f.get("ms"))
send({"T": 136, "cmd": 1500}); send({"T": 131, "cmd": 1})
t0 = time.monotonic()
for _ in range(10):                      # 2 s at 0.1 m/s, re-sent every 0.2 s
    send({"T": 1, "L": 0.1, "R": 0.1}); frames(0.2)
print("--- stopped sending T:1 ---")
frames(2.5)
send({"T": 1, "L": 0, "R": 0}); s.close()
```

Expected:

- `T:1001` lines about every 50 ms, carrying `L`, `R`, `odl`, `odr` (plus
  `odlt`, `odrt`, `ms` once flashed with the fork).
- While sending, `L` and `R` near 0.1 and the odometers climbing.
- After "stopped sending", `L` and `R` still near 0.1 until about 1.5 s
  later, then 0: the board's heartbeat (`T:136`) stopped the wheels on its
  own. That is the check.
- If the wheels never stop, the heartbeat is not being honoured: do not run
  the car. If they stop at about 3 s, the `T:136` was lost and the firmware
  default applied.
- **Do not send `T:0` to stop the wheels.** See Known gaps. A `T:1` of zero
  stops them.

**Odometry over a measured metre.** The same session, on the floor, with a
tape along a straight line and the robot server still stopped. UNCONFIRMED:
not run.

1. Mark the chassis' front edge against the tape and note the last frame's
   `odl` and `odr` (and `odlt`, `odrt` on the fork).
2. Send `{"T":1,"L":0.1,"R":0.1}` every 0.2 s for 10 s (about 1 m), then
   zero, and wait for `L` and `R` to read 0.
3. Note the frame's odometers again, and measure the front edge's travel on
   the tape.

Compare each odometer's change with the tape, not with what was commanded:
the board's speed loop and the stop's slip are not the odometry. A ratio
that is consistently off 1.0 on both wheels means `WHEEL_D` is off for this
unit; a difference between the wheels on a straight run is a hardware-day
calibration item. Record both numbers against `PLAN-ros-alignment.md`'s
hardware-day entry; the bars are not yet written.

**Then start the robot server** with no world, because a board with no sim
body has no house to map, and the shipped `world.mode: sim` refuses it at
start-up:

```bash
ROBOT_MODE=hardware ROBOT_SERIAL=/dev/picar-motor-board WORLD_MODE=none uvicorn robot.server:app --port 8000
```

`GET /wheels` should turn `usable: true` within one feedback interval, and
turns run (FORWARD and REVERSE are refused until the lidar driver lands).
**Free the port** first: the stock `ugv_jetson` app must be disabled, or it
holds the port (3.26).

**Failure signatures:**

| Symptom | Meaning |
|---|---|
| Start-up `ValueError: mode: hardware needs ROBOT_SERIAL` | Set the port, or `SIM_MOTOR_BOARD=fake`. |
| Start-up `ValueError: world mode 'sim' needs the grid-world robot` | A real board with the shipped `world.mode: sim`. Add `WORLD_MODE=none`. |
| A FORWARD blocked with `Blocked FORWARD: distance=0.0cm < min=...cm (distance_sensor)`, or a standing forward `/wheels` answered `forward clamped: 0.0cm <= ...cm (distance_sensor)` | The real board with no sensors: `get_distance()` is 0.0, which reads as an obstacle at the bumper. Expected until the lidar driver lands; only turns move (safety domain). |
| A REVERSE blocked with `rear clearance=0.0cm ... (astern_not_observed)`, or a settle ending "settle refused" | The real board with no lidar scan yet: the safety layer will not back up blind, and a straight correction either way is refused (safety domain). Expected until the lidar driver lands. |
| `PermissionError` / EACCES on open | The user is not in `dialout`, or the udev rule is missing. |
| `get_wheel_state()` stays `usable: false` | No `1001` frames are arriving: wrong port or baud, feedback off, or another process holds the port. |
| `board_reboots` climbing | Brownouts. Check the pack and the motor stall current. |

## Verification

Counts are from `pytest --collect-only`, 2026-10-02.

| Test file | Tests | What it pins |
|---|---|---|
| `tests/test_board_speed_clamp.py` | 2 | Handoff 4j: an over-fast command (3.2 m/s) reaches the fake board scaled into +/-2.0 with its ratio kept, not dropped. |
| `tests/test_fake_esp32.py` | 11 | R7's criteria on the ROS Driver fake: `T:1` closed loop and its +/-2.0 guard, `T:13`, `T:11` turning the PID off, `T:130` and `T:131`, the heartbeat in every mode, wheels over the wire, a severed host stopped by the board's own heartbeat. |
| `tests/test_ros_driver_board.py` | 24 | 3.25: frame keys, cm odometers truncated toward zero, integer edges, measured speeds, at most one frame per 50 ms, `T:142`, `T:900`, 660 in one place, the anchor bound and its mutation, reboot absorption with no false reboot, set-up re-sent, a clear FORWARD within `STOCK_STRAIGHT_BAR_CM` (1.34). Stock turns within +/-1 deg is a NON-STRICT XFAIL with a guard: the mean of ten within 2.0, none past 6. |
| `tests/test_firmware_fork.py` | 21 | 3.28 and 3.29: the patch adds only three keys inside `baseInfoFeedback()`; it applies to `2e7df97` and compiles (both skip unless `PICAR_FIRMWARE_SRC` names a checkout, and the compile also needs `arduino-cli`); the fork's keys and units; the board clock and its wrap; frame size; the 0.1 mm bound; reboots on the board clock; partial keys read as stock; fork straights and turns. |
| `tests/test_settle_pass.py` | 8 | 3.29: fork turns within +/-1 deg, a FORWARD is a cell on either firmware, an overshoot corrected when clear, a correction toward furniture clamped, a `stop()` during the settle ends it. |
| `tests/test_robot_contract.py` | (the `hardware` backend) | `HardwareRobot` over the fake passes the backend-agnostic body contract. |
| `tests/test_guarded_verbs.py` | (hardware cases) | 3.22's guarded verbs over the fake board. |

Recorded numbers:

| Measure | Stock (3.25) | Fork, 0.1 mm (3.29) |
|---|---|---|
| Worst wheel-travel error, 30 s stop-go, 5% lines lost | 1.02 cm (1.63 in the zero bucket); speed integral alone 9.57 | 0.047 cm, 0/1028 over the bar |
| Clear turns within +/-1 deg (120) | 60/120 and 51/120 in two identical runs; worst 4.6-5.2 | 120/120 with the settle, worst 0.84; 118/120 without |
| Median turn time | 0.66 s | 0.82 s |
| Clear FORWARD, 30 cm | 117-119 of 120 within 1.0 cm, worst 1.13 | worst 0.13 cm over ten |
| Reboot mid-drive | 8/8 detected, at most 1 cm move | 8/8 from `ms`, at most 0.058 cm move |

R7's heartbeat drill (3.16, criterion 4, `tests/test_fake_esp32.py`): a
host severed mid-motion, and the board's own heartbeat stopped the wheels
1.5 s later, within one loop.

Checklist for a change:

- Run `pytest tests/test_fake_esp32.py tests/test_ros_driver_board.py
  tests/test_firmware_fork.py tests/test_settle_pass.py
  tests/test_robot_contract.py`.
- Run them again with `SIM_BOARD_FIRMWARE=fork`.
- Judge accuracy on `FakeEsp32.sent_truth` or on the body's truth, never on
  what the host reports about itself.
- A change to the patch must keep `odl` and `odr` untouched, and must
  re-record the compile size here.

## Known gaps

- **Not flashed; no real board yet.** Every number above is against the
  fake.
- **`T:0` disagrees between sources.** The fake models `T:0` as
  `setGoalSpeed(0, 0)`, which stops the wheels. `PLAN-ros-alignment.md`
  3.26 and `CLAUDE.md` say the board's `T:0` releases the arm's servo
  torque, not the wheels. The host never sends it. Settle this against the
  firmware source before relying on either.
- **The fake's limits.** Its PID is ideal: no `THRESHOLD_PWM` deadband, no
  dynamics. Its IMU fields are zero and its voltage is constant. Its loop
  and feedback timing are assumed.
- **No straight driving before the lidar.** On the car with no sensors,
  every FORWARD is refused (`get_distance()` 0.0) and every REVERSE too
  (`astern_not_observed`), so no straight verb runs and no straight
  correction either; only turns move and settle. The rule is the safety
  domain's ([safety engineering](../safety/ENGINEERING.md), Known gaps).
- **Stock turns miss the +/-1 degree bar.** Stock frames carry no
  timestamp, so arrival jitter limits heading. On the car, heading wants the
  gyro (`gz`) or SLAM.
- **Not reported by the backend:** the measured wheel speed in
  `velocity_rad_s` (it reports the commanded one), the IMU and the battery
  voltage.
- **Two hardware-day measurements:** `wheel_separation_multiplier` (the
  skid-steer effective track) and the real device path (`/dev/ttyTHS1`
  versus USB).
