# Guide: how a robot base works, and what to check before buying one

**A learning reference, written 2026-09-30** from the chassis search of
2026-09-27 to 09-30 (the decision itself is recorded in `JETSON-BOM.md`
section 9). Everything here was learned by reading vendor source code,
vendor pages and vendor support replies, and the tags say which:

* `[V]` read directly from a primary source (firmware or driver source, a
  vendor page, a vendor's support email)
* `[I]` inferred or calculated
* `[U]` unverified or second-hand

Read it in order the first time. Later, the section headings are the index.

---

## 1. The four layers between a wheel and your software

Every robot base, from a $90 hobby chassis to an industrial AMR, has the same
four layers. Most confusion about "is this robot compatible with ROS 2" comes
from not knowing which layer a claim is about.

```
 Wheel motor + encoder      (hardware: counts wheel rotation)
        │ electrical pulses
 Motor-board firmware       (STM32 on Hiwonder, ESP32 on Waveshare)
        │ USB serial, in the VENDOR'S OWN protocol   ← not ROS; problems live here
 Driver on the Jetson       (translates the vendor protocol into ROS 2 messages)
        │ ROS 2 topics such as /odom, /joint_states, /imu, carried over DDS
 Other ROS 2 nodes          (SLAM, nav2, …)
```

**Layer 1 -- the motor and its encoder.** A geared DC motor turns the wheel.
An **encoder** is a sensor on the motor shaft that emits electrical pulses as
it turns: count the pulses and you know how far the wheel rotated.

**Layer 2 -- the motor-board firmware.** A small microcontroller (an STM32 or
an ESP32) runs a program -- the **firmware** -- that drives the motors with
PWM, reads the encoders, often reads an IMU and the battery voltage, and
talks to the main computer.

**Layer 3 -- the driver on the main computer.** A program on the Jetson opens
the USB serial port, speaks the board's protocol, and republishes what it
learns as standard ROS 2 messages.

**Layer 4 -- everything else.** SLAM, nav2, and this project's own brain
consume those standard messages and never see the board.

### What ROS 2 standardises, and what it does not

**ROS 2 standardises layers 3 and 4:** the **message formats**
(`nav_msgs/Odometry` = "where the robot thinks it is and how fast it is
moving"; `sensor_msgs/JointState` = wheel positions and velocities;
`sensor_msgs/Imu`; `sensor_msgs/LaserScan`) and the **transport** between
programs, **DDS**. Once data is a ROS 2 message, any ROS 2 program can use it.

**ROS 2 does not standardise layer 2's link.** How a motor board talks to the
computer is a **private protocol each vendor designs**:

* Waveshare's boards send **JSON lines** over serial, e.g.
  `{"T":1001,"L":0,"R":0,"odl":0,"odr":0,"v":11.0, ...}` `[V]`
  (`waveshareteam/ugv_base_ros`, `ROS_Driver/json_cmd.h`).
* Hiwonder's board sends **binary frames**: `0xAA 0x55 | function | length |
  data | CRC8`, at 1,000,000 baud on `/dev/rrc` `[V]`
  (`Hiwonder/ROSOrin`, `ros_robot_controller_sdk.py`).

So "ships with a ROS 2 driver" means **the vendor wrote a layer-3 translator
for their private protocol**. It says nothing about what the board actually
sends up the cable -- and the driver can only publish what the board sends.

(There is a way for a microcontroller to speak ROS 2 directly -- **micro-ROS**,
which runs a small DDS client on the board. Neither board considered here
uses it.)

### Layer 2 on the UGV Rover: which chip, and where the JSON comes from

**The chip is the original ESP32** -- dual-core Xtensa LX6, Wi-Fi plus
Bluetooth Classic/BLE -- not an S3, C3 or C6, and almost certainly an
**ESP32-WROOM-32** module `[I]`. Neither Waveshare's wiki nor the firmware
repo names it; the pin map does (`ugv_base_ros` @ `2e7df97`,
`ROS_Driver/ugv_config.h`) `[V]`:

* encoders on **GPIO 34 and 35** -- input-only pins, which only the
  original ESP32 has;
* motor pins on **GPIO 21, 22, 23 and 25** -- the S3 has no GPIO 22-25, and
  the C3/C6 have far fewer pins;
* **GPIO 16 and 17** used as ordinary pins -- the WROVER module reserves them
  for its PSRAM, so WROOM fits.

The older General Driver is documented as an ESP32-WROOM-32
(`HARDWARE-BOM.md` 4.2), Waveshare calls the ROS Driver a variant of it, and
the pin assignments match. **Confirm by reading the module's shield on
arrival.** If you ever flash it, choose **"ESP32 Dev Module"** in the
Arduino IDE, not an S3 profile.

**The JSON is the firmware's, not the chip's.** `uart_ctrl.h` reads `Serial`
a character at a time until a newline, then parses the line with
`deserializeJson`; the same firmware builds the `T:1001` frames and writes
them back on the same port `[V]`. That `Serial` is the ESP32's **UART0**, a
plain TX/RX line -- the original ESP32 has **no USB hardware** -- and the
board carries it two ways:

| route | conversion | host sees |
|---|---|---|
| USB cable | a USB-to-UART bridge chip on the board (two CP2102s on the General Driver `[V]`; not yet checked on the ROS Driver `[U]`) | `/dev/ttyUSB0` |
| 40-pin header | none: raw UART. Waveshare's README says the kit uses it, and both of their Jetson nodes open `/dev/ttyTHS1` `[V]` | a Jetson UART: `/dev/ttyTHS1` |

Same bytes, same JSON lines, either way. `robot/hardware_robot.py` takes the
device from `ROBOT_SERIAL`, so the route is a setting, not code; expect
`/dev/ttyTHS1`, and confirm on arrival (`JETSON-BOM.md` 9.5). And because the
protocol lives in open firmware, `sim/fake_esp32.py` copies it from source
and is pinned to `2e7df97` -- a firmware update can change it.

**The firmware is C++**, written as an Arduino sketch: `ROS_Driver.ino`
(`setup()` / `loop()`) plus about twenty headers (`uart_ctrl.h`,
`movtion_module.h`, `IMU_ctrl.h`, ...), compiled by GCC through Espressif's
Arduino core, which sits on ESP-IDF and FreeRTOS. Its libraries are Arduino
ones: ArduinoJson, ESP32Encoder, PID_v2, Adafruit's IMU and OLED drivers,
LittleFS `[V]`.

### Why the board is called a "ROS Driver" when it speaks no ROS

**"ROS Driver for Robots" is the board's product name.** Waveshare sells two
versions of one ESP32 board: the *General Driver* (firmware
`ugv_base_general`) and the *ROS Driver* (`ugv_base_ros`), the one that ships
in their ROS kits because it runs closed loop and reports measured odometry.
The firmware contains **no ROS at all** -- no micro-ROS, no rosserial -- only
JSON over UART, HTTP and ESP-NOW `[V]`.

The translation into ROS 2 messages is **layer 3**, a separate program on the
Jetson. Waveshare's is two Python nodes in `waveshareteam/ugv_ws`
(`ugv_bringup` package) `[V]`:

| node | direction | does |
|---|---|---|
| `ugv_bringup.py` | board -> ROS | parses `T:1001`, publishes `imu/data_raw`, `imu/mag`, `odom/odom_raw`, `voltage` |
| `ugv_driver.py` | ROS -> board | subscribes `cmd_vel`, writes `{"T":13,"X":<m/s>,"Z":<rad/s>}` |

**This project does not use them.** `robot/hardware_robot.py` plays
`ugv_bringup.py`'s part, and the ros2_control chain (`picar_sim_hardware` ->
`diff_drive_controller`) plays `ugv_driver.py`'s, so every command passes
through `robot/safety.py`. Waveshare's nodes would own the serial port and
send `cmd_vel` to the motors unchecked. Only one program can hold the port,
so their nodes and the kit's stock app must be disabled on the car. What the
audit of their code found, and what is worth reusing, is
`PLAN-ros-alignment.md` 3.26.

---

## 2. Encoders, closed loop, and where the data goes

### What an encoder reports

A **quadrature encoder** has two sensor channels (A and B) a quarter-cycle
apart. The pulse count gives distance; which channel leads gives direction.
The key number is **pulses per wheel revolution**, which combines the
encoder's own resolution, how edges are counted, and the gearbox:

```
 pulses per wheel rev = (encoder lines per motor rev) × (edges counted) × (gear ratio)
 UGV Rover:           = 11 × 2 (half-quadrature) × 30 = 660   [V: firmware + Waveshare support]
```

Wheel distance then follows from the wheel's circumference:

```
 metres per pulse = π × wheel diameter / pulses per rev
 UGV Rover:       = π × 0.080 / 660 ≈ 0.38 mm per pulse   [I]
```

A trap found along the way: Waveshare's **open-loop** firmware carries
`ONE_CIRCLE_PLUSES = 1650` for the UGV Rover, while its **closed-loop**
firmware carries **660**, with a comment noting the change. The arithmetic
(11 × 2 × 30 = 660; 1650 matches nothing) and Waveshare support both say
**660**. A stale constant survives in code that never uses it -- the
open-loop firmware never reads the encoder to control anything, so nobody
noticed. **Trust the constant in the code path that actually uses it.**

### Open loop vs closed loop

* **Open loop:** the board sets a motor **power** (a PWM duty cycle) and
  hopes. The same power gives different speeds on carpet, on tile, uphill, or
  with a low battery.
* **Closed loop:** the board sets a **target speed**, measures the actual
  speed from the encoders, and adjusts the power continuously -- usually with
  a **PID** controller. The wheel holds its speed whatever the load.

Waveshare's two firmwares for the same hardware differ exactly here `[V]`:

| firmware | board | UGV Rover mode | reported "speed" |
|---|---|---|---|
| `ugv_base_general` | General Driver | **open loop** (`usePIDCompute = false` for mainType 2) | the **commanded PWM**, overwritten into the speed field -- synthetic |
| `ugv_base_ros` | ROS Driver | **closed loop** (`usePIDCompute = true`) | measured, plus odometry `odl`/`odr` |

Waveshare support confirmed the UGV Rover PT Jetson Orin ROS2 Kit ships the
**ROS Driver** board (ticket 257427, 2026-09-30) `[V]`.

### The question that matters: does the encoder data reach the host?

**Closed loop and "reports encoders to the host" are different things.** A
board can use its encoders internally to hold wheel speed and still never
tell the computer what the wheels actually did. That is exactly the Hiwonder
ROSOrin:

* Its STM32 **does** read four encoders and runs closed-loop speed control.
* Its protocol has report types for system/battery, buttons, IMU, gamepad,
  RC receiver and servos -- **none for the motors** `[V]`.
* Hiwonder support: *"The STM32 firmware currently does not support
  reporting motor data or encoder feedback back to the host."* (2026-09-30)
  `[V]`

### Measured odometry vs commanded odometry

**Odometry** is the robot's own estimate of how far it has moved and turned.
There are two ways to produce it:

* **Measured (encoder) odometry:** add up what the wheels actually did. Sees
  a wheel slipping, a stall against a chair leg, a push.
* **Commanded (dead-reckoned) odometry:** add up what you **asked** the
  wheels to do. "I commanded 0.2 m/s for 1 s, so I moved 0.2 m." Blind to
  slip, stalls and pushes.

Hiwonder's `odom_publisher.py` does the second `[V]`: it integrates the
commanded linear and angular velocity, then fuses the IMU with
`robot_localization` (an extended Kalman filter). That works only while the
wheels always achieve exactly what was commanded -- which **skid steer never
does**, because every turn scrubs the tyres sideways.

---

## 3. Anatomy of a vendor driver (the Hiwonder ROSOrin, from its source)

There **is** two-way traffic between the Jetson driver and the firmware; it
is just lopsided.

```
 ┌──────────────── Jetson (Hiwonder's driver, Python) ────────────────┐
 │                                                                     │
 │  odom_publisher.py                                                  │
 │    in:  /cmd_vel  ("drive at 0.2 m/s, turn at 0.5 rad/s")           │
 │    → converts to four wheel speeds (revolutions/s)                  │
 │    out: /ros_robot_controller/set_motor                             │
 │    out: /odom  ← integrated from the COMMANDED velocity             │
 │                                                                     │
 │  ros_robot_controller_node.py                                       │
 │    in:  set_motor, servo, LED, buzzer commands                      │
 │    out: /imu, battery voltage, buttons, gamepad                     │
 │                                                                     │
 │  ros_robot_controller_sdk.py  (class Board)                         │
 │    opens /dev/rrc at 1,000,000 baud                                 │
 │    sends frames:  0xAA 0x55 | function | length | data | CRC8       │
 │    a background thread parses incoming frames by function code      │
 └──────────────────────────────┬──────────────────────────────────────┘
                                │ USB serial
 ┌──────────────────────────────┴──────────────────────────────────────┐
 │  STM32 firmware (proprietary)                                       │
 │    receives: "motor 1 at X rev/s …" → PID on its own encoders → PWM │
 │    sends:    IMU, battery, buttons, gamepad, RC receiver, servos    │
 │    never sends: encoder counts or measured wheel speeds             │
 └─────────────────────────────────────────────────────────────────────┘
```

**Going down:** commands. Function 3 is "set motor speeds" -- each wheel's
target in revolutions per second, sub-command `0x01`, packed as
`<Bf` (motor index, float) per wheel `[V]`. Others drive servos, LEDs, the
buzzer and the OLED.

**Coming up:** readings -- function 0 (system: battery voltage in sub-code
`0x04`), 6 (keys), 7 (IMU), 8 (gamepad), 9 (SBUS RC receiver), 4 and 5
(servo read-backs) `[V]`.

**How its differential mode works:** there is no separate differential
kinematics. The mecanum model is reused with sideways motion forced to zero;
each wheel gets `v ± ω·(wheelbase + track)/2`, with wheelbase 0.17706 m,
track 0.17165 m and wheel diameter 0.08 m `[V]`.

**The Waveshare equivalent, for contrast:** the ROS Driver board's `T:1001`
report, several times a second, carries wheel speeds, **left and right
distance travelled from the encoders** (`odl`, `odr`), the IMU (gyro,
accelerometer, magnetometer) and battery voltage `[V]`. The driver gets
**measured** motion.

---

## 4. Why firmware openness matters

"Open source firmware" means **you can change what the board does** --
including adding a missing message. If a board withholds data it already
measures (as the ROSOrin does with its encoders), openness decides whether
that is a one-evening fix or a permanent limit.

| | Waveshare (both boards) | Hiwonder ROSOrin |
|---|---|---|
| Firmware source | on GitHub, GPL-3.0 (`ugv_base_general`, `ugv_base_ros`) `[V]` | **proprietary**; only a compiled `.hex` is published `[V: Hiwonder support]` |
| Could we add an encoder report? | yes | **no** |
| Protocol documented | yes (the JSON command list is in the source) | yes (the SDK source is open), but only for what the firmware already does |

The ROS 2 **driver** being open (both are) does not help here: the driver is
layer 3, and the missing data is withheld at layer 2.

### May you flash your own firmware?

**Yes, on a GPL-3.0 board.** The licence lets you modify the firmware and run
it on your own device; its obligations start only when you *convey* the
binary to someone else (sell or give them the robot, publish an image), and
then your changed source must go with it. Practical rules: dump the stock
image first (`esptool.py read_flash`), keep the vendor's restore tool
(Waveshare's ESP32 Download Tool), and flash only after any return-window
checks are done, so a fault cannot be blamed on your build.

**What is worth changing on the Rover's board** (`PLAN-ros-alignment.md`
3.26): the firmware already holds wheel travel in float metres and truncates
it to whole centimetres on the way out, and sends no timestamp. Two new
fields -- millimetre odometers and `millis()` -- remove the two limits 3.25
measured. Adding fields rather than changing old ones keeps the vendor's
tools working.

**What the board cannot do for you: safety.** The lidar and cameras plug
into the Jetson, so the firmware never sees an obstacle. Its only guard is a
deadman heartbeat (`T:136`, default 3 s) that zeroes the wheel speed when
commands stop. Its `T:0` "emergency stop" releases the robot arm's servos,
not the wheels.

---

## 5. How much this project depends on measured odometry

**Important, not fatal.** No safety function depends on it; several
navigation functions degrade without it.

| where it's used | without real encoder data | how bad |
|---|---|---|
| **Safety** -- the 20 cm stop, the swept corridor, the pivot guard, guarded verbs re-vetting every 50 ms (`PLAN-ros-alignment.md` 3.18, 3.19, 3.22) | Unaffected: these read the **lidar** every period. Only a guarded verb's "am I done" signal falls back to timing. | fine |
| **SLAM mapping** (`slam_toolbox`) | The motion prior between scans becomes "what we asked for". Fine with plenty of walls and furniture in view; weaker in long featureless corridors, and wrong when a wheel slips or the robot is stuck (odometry says it moved, the lidar says it didn't). | degraded |
| **nav2 path following** | No feedback on actual speed: more overshoot and wobble on turns, worst on skid steer. | degraded |
| **The flicker fix** (P25 / P7c item 2: anchor a sighting in the odometry frame, recompute its bearing as the robot moves) | **Rotation** can come from the IMU gyro (good); **distance travelled** becomes a guess -- the half that matters when driving toward a target. | weakened |
| **Noticing the robot is stuck** (pushing a chair, a spinning wheel) | Can't tell from the wheels; only from lidar positions over time. | slower |
| Arrival, detection, cloud calls | Don't use odometry. | fine |

**Workarounds exist:** **lidar odometry** (estimating motion by matching
consecutive scans, e.g. `rf2o_laser_odometry`), **IMU** for rotation, and
**visual odometry** from a depth camera. Many robots run on lidar odometry
alone. It is integration and tuning work done only because a board withholds
data it already has.

---

## 6. Powering the Jetson from a robot battery

### The numbers that matter

* The **Jetson Orin Nano Super dev kit** takes **9-20 V** at its barrel jack
  (5.5 × 2.5 mm) `[V: JETSON-BOM.md]`.
* A **3S lithium pack** is 3 cells in series: **12.6 V full, ~11.1 V
  nominal, ~9 V empty**. The empty end sits **at the Jetson's floor**, so the
  Jetson browns out near the end of every charge unless software shuts down
  first (a planned item).
* **Power = voltage × current.** At the pack's ~11 V, **25 W is about 2.3 A**
  `[I]`.

### Jetson power modes are a software setting

```bash
sudo nvpmodel -q          # show the current mode
sudo nvpmodel -m <id>     # switch mode; persists across reboots
```

The Orin Nano Super offers **7 W, 15 W, 25 W and MAXN SUPER**. MAXN SUPER is
**uncapped** and can draw more than 25 W. Mode IDs differ between JetPack
releases -- read `/etc/nvpmodel.conf` on the board. 15 W costs roughly 40%
of the AI throughput (NVIDIA rates ~40 TOPS at 15 W against ~67 at 25 W)
`[I]`: a working fallback, not a free fix, and a good way to **test for
brownouts** -- run at 15 W, then step up.

### Budgeting a robot's power

A worked budget for the UGV Rover at absolute peak `[I]`:

| load | watts |
|---|---|
| Jetson at 25 W | ~25 |
| four drive motors at full effort | up to ~20 |
| D500 lidar | ~1.5 |
| OAK-D Lite | ~3-5 |
| pan-tilt servos | a few |
| **total peak** | **~50-55 W** (~5 A at 11 V) |

Typical draw is far lower -- motors are rarely all at full effort at once --
but the **peak** is what causes a brownout.

### What limits the current

* **The cells' discharge rating ("C-rating").** A 2,200 mAh cell rated 4C can
  deliver 2.2 × 4 = 8.8 A. Waveshare asks for **4C or better** `[V]`.
  High-capacity cells often have **lower** C-ratings -- choose for both.
* **The power board.** Waveshare: the UGV Rover's UPS board has **no fixed
  continuous rating**; its overcurrent protection trips at **~7.5-12.5 A**,
  and "normal continuous output current can reach up to **5 A**" `[V]`.
  Hiwonder: the ROSOrin's Jetson port **"cannot supply enough current to
  support the board running continuously at full load (25W mode)"** `[V]`.
* **Charger specs are not output specs.** "12.6 V / 2 A" on the UGV Rover's
  page is the **charger**, not the output limit -- a misreading that
  `HARDWARE-BOM.md` once rejected a board over `[V: Waveshare support]`.

### The standard fix: a separate battery for the computer

Power the Jetson from **its own pack**, bypassing the robot's power board:

```
 3S pack with protection board ──► inline fuse (~5 A) ──► barrel adapter ──► Jetson
 robot's own pack ──► robot's power board ──► motors, lidar, servos
```

* Use a pack **with a protection board** (short-circuit, overcurrent,
  over/under-charge) -- e.g. the Wheeltec E351S: 3S, 5100 mAh (55 Wh), 6 A
  continuous / 13 A peak, charger included `[V: OpenELAB listing]`.
* **Mind the plug:** 5.5 × **2.1** mm and 5.5 × **2.5** mm barrels look
  alike and don't mate properly. The Jetson dev kit takes 2.5 mm.
* **Grounds:** the two packs share a ground through the USB cable between the
  Jetson and the motor board. That is normal and expected.
* **A robot's own "external battery" socket is not this fix** if it feeds the
  same power board -- it adds runtime, not headroom.

---

## 7. Chassis geometry

### Drive types

| type | how it steers | turns in place? | fits this project? |
|---|---|---|---|
| **Differential** | two driven wheels (+ caster) at different speeds | yes | yes |
| **Skid steer** | 4+ driven wheels, left side vs right side | yes, but scrubs the tyres | yes |
| **Tracked** | tank tracks, skid steer | yes, scrubs heavily | yes, worst odometry |
| **Mecanum** | angled rollers; can move sideways | yes | **no** -- the stack assumes differential kinematics |
| **Ackermann** | car-style steering | **no** | **no** |

**Filter on drive type first.** It is the cheapest check on any listing and
it rules out most kits before anything else matters: keep differential,
skid steer and tracked; drop mecanum and Ackermann. The simulator, the pivot
guard, the 45-degree search turns and nav2's fit checks all assume a robot
that turns on the spot. (Example: the Yahboom ROSMASTER A1 on Amazon looks
like a match on price, lidar and depth camera, and is Ackermann.)

### The numbers that describe a chassis

* **Track width** -- left-right distance between wheel centres. Sets how fast
  the robot pivots for a given wheel-speed difference.
* **Wheelbase** -- front-rear distance between axles.
* **Effective track** -- on skid steer, wheels slip sideways when turning, so
  the robot turns **less** than the geometric track predicts. ROS's
  `diff_drive_controller` corrects with `wheel_separation_multiplier`,
  measured on the real robot. (UGV Rover: geometric track 174.52 mm from its
  URDF; the firmware uses 0.172 m `[V]`.)
* **Footprint** -- the outer rectangle the safety layer and nav2 plan with.
* **Corner radius** -- centre-to-corner distance: the circle the robot needs
  to turn in place without touching anything. A 253 × 231 mm body has a
  17.1 cm corner radius, so it **cannot pivot** in a 30 cm gap `[I]`.

`tests/chassis_fit.py` checks a footprint against a house on ground truth:
which rooms the rectangle can reach, and where it can turn right round.

---

## 8. Sensors on a robot base

### Lidar

A 2D **360-degree lidar** spins a laser range-finder and reports a ring of
distances -- ~10 scans per second, out to ~12 m for the budget units (Slamtec
RPLidar C1, LDROBOT STL-19P / "D500", Oradar MS200) `[V]`. In this project it
does two jobs: **safety** (the swept corridor and pivot guard read it every
50 ms) and **mapping** (SLAM). It sees one horizontal plane only -- anything
lower than the scan plane is invisible to it, which is why a depth camera is
the planned upgrade.

### Depth cameras: two ways to measure depth

| | structured light (e.g. Aurora930 Pro) | passive stereo (e.g. OAK-D Lite) |
|---|---|---|
| how | projects an infrared pattern and watches how it deforms | two cameras, like eyes; matches features between them |
| near range | good (**from 0.15 m**) | poor (best from ~0.8 m) |
| blank walls and floors | works -- it supplies its own texture | struggles -- nothing to match |
| far range | short (~3 m) | long (~12 m) |
| bright sunlight | can wash out the pattern | fine |

For a floor robot's near-field job -- **low obstacles just ahead, the floor**
-- structured light fits better. For an all-round camera, the OAK-D Lite
wins: a 13 MP colour camera and an on-board AI chip that can do image
resizing (the Jetson's measured bottleneck) before the frame reaches it.

### Does the robot need its own colour camera?

The object detector resizes every frame to about 640 px anyway, so a 640-wide
colour stream is the **same size the detector works at**. What still matters:
resolution for **small distant targets** (the CLIP second stage works on
crops), image quality, and field of view. The answer should be **measured**:
record walks through the candidate camera and score them with
`control/perception_eval.py`.

---

## 9. Reading a robot-kit listing

### Marketing claims vs hardware

* **"Multimodal AI & LLM integration"** = sample programs that send the
  robot's audio and images to an **online AI service** (ChatGPT, Gemini...)
  and act on the reply. No AI chip in the robot; needs internet and your own
  API keys. "Multimodal" = the model takes images as well as text.
* **"Runs YOLO, SLAM and LLMs"** = runs them **on the computer you supply**.
* **"SLAM: Gmapping, Hector, Cartographer"** = well-known free mapping
  programs (the first two ROS 1-era). This project uses `slam_toolbox` + nav2.
* **"4 encoder motors, closed-loop"** -- true and **says nothing** about
  whether the encoder data reaches your computer (section 2).
* **"Supports Jetson"** -- ask whether it supports **your** Jetson at **your**
  power mode, and get the answer in writing.

### Verify at the source

Every decisive fact in the 2026-09 search came from **source code or a
support email**, and three product-page or summary claims were wrong:

| claim (from a page or a summary) | the source said |
|---|---|
| ROSOrin: "4 encoder motors" → usable odometry | the firmware never reports them to the host |
| ROSOrin: sold with an Orin Nano Super fitted → 25 W is fine | Hiwonder: the port can't sustain 25 W |
| UGV Rover: "12.6 V / 2 A" → output limit | Waveshare: that's the charger |
| Cobra Flex: an `IMU_ctrl.h` in the firmware → an IMU | its functions are empty stubs, and the feedback message's IMU fields are commented out |

Also: check the **manufacturer part number**, not the title. A Micro Center
"Hiwonder ROSOrin" at $299.99 was part **21031708** -- the **Starter** tier
(mecanum only), not the Advanced kit (**21031738**).

A file named for a feature is not the feature. Look for the call that
**sends** the value to the host (the feedback message), not the driver
that could read it -- and watch for reused field names: with the Cobra's
arm module fitted, `ax`/`ay`/`az` are the arm's coordinates.

### Buying terms that change the real price

* **FOB Shenzhen** (Hiwonder direct): import duties and fees are **the
  buyer's**, paid on arrival, amount unknown in advance.
* **Returns from China** (Hiwonder, Waveshare direct): typically unused,
  original packaging, **buyer pays shipping to China**. Useless for a
  "test it for 30 days" plan.
* **"Ships from Amazon"**: domestic, duties handled, free 30-day return --
  worth a premium for a first purchase whose fit is unproven.
* **Vendor lithium shipping:** vendors often ship robots **without 18650
  cells** because lithium air-shipping is restricted.

---

## 10. Glossary

| term | meaning |
|---|---|
| **Closed loop** | the controller measures the result and corrects; here, holding wheel speed with encoder feedback |
| **CRC8** | an 8-bit checksum appended to a frame to detect corrupted bytes |
| **C-rating** | a battery's safe discharge current as a multiple of its capacity |
| **DDS** | Data Distribution Service -- ROS 2's transport between programs |
| **Dead reckoning** | estimating position by adding up assumed motion |
| **dToF** | direct time-of-flight -- timing a light pulse's round trip to measure distance |
| **EKF** | extended Kalman filter; fuses noisy sensors (odometry + IMU) into one estimate |
| **Firmware** | the program running on a microcontroller (layer 2) |
| **IMU** | inertial measurement unit: gyroscope (rotation rate), accelerometer, often a magnetometer |
| **micro-ROS** | ROS 2 running on a microcontroller, speaking DDS directly |
| **nav2** | the ROS 2 navigation stack: plans a path and follows it |
| **nvpmodel** | NVIDIA's tool for switching Jetson power modes |
| **Odometry** | the robot's own estimate of how far it has moved and turned |
| **PID** | proportional-integral-derivative control, the usual closed-loop speed controller |
| **Pulses per revolution** | encoder pulses per full wheel turn; converts counts to distance |
| **PWM** | pulse-width modulation; how a board sets motor power |
| **Quadrature** | two encoder channels a quarter-cycle apart, giving direction as well as distance |
| **SLAM** | simultaneous localisation and mapping -- building a map while locating yourself on it |
| **Skid steer** | steering a multi-wheel vehicle by driving the two sides at different speeds |
| **UPS board** | a power board that can charge the battery while powering the robot |
| **URDF** | the XML description of a robot's links, joints and sensors for ROS |
