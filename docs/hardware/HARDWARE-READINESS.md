# Hardware transition: what the real robot changes

> **Status 2026-09-28 -- the parts named below are partly wrong; read this
> first.** The hardware changed again after the 2026-09-04 revision, and the
> code has moved on. What is now true:
>
> - **Board: Jetson Orin Nano Super**, not a Raspberry Pi 5 (decided
>   2026-09-19; the Hailo-8L path is closed and is not being pursued).
> - **Camera: IMX219** CSI module (Arducam B0191), not Camera Module 3.
> - **Motor driver: Waveshare General Driver for Robots (ESP32)**, not the
>   Yahboom kit's STM32 board.
> - **Pan: one ST3215 serial-bus servo, no tilt**, not two SG90s on a
>   pan/tilt bracket.
> - **`robot/hardware_robot.py` exists** (R7, 2026-09-26, `mode: hardware`,
>   port from `ROBOT_SERIAL`). It drives **open-loop timed verbs** over the
>   ESP32's serial protocol; a verb is closed on the encoders only under
>   `drive: ros` (`robot/ros_drive.py`). Where this doc says "the one file
>   still unwritten", read "written".
> - **The JS Vision Autopilot and local brain were deleted 2026-09-25.** The
>   brain is `control/brain_server.py`; section 2's "Autopilot" column and
>   section 7's references to the browser's loops are history.
>
> **For parts, part numbers, wiring and bring-up, use `HARDWARE-BOM.md`**
> (and `JETSON-BOM.md` for the shopping list), not `PLAN-onboard-perception.md`
> 3.6. The architecture arguments below (sections 2, 4's shape, 6 and 7) still
> stand; substitute "the Jetson" wherever it says "the Pi".

> **Revised 2026-09-04 for the hardware actually chosen.** This document was
> written on 2026-08-27 for the SunFounder PiCar-X kit, and
> `PLAN-onboard-perception.md` replaced that kit before purchase: a
> **differential-drive chassis** (its 1.1), an **RPLidar C1** as the obstacle
> sensor (1.2), and a **Hailo-8L M.2 module with a Camera Module 3** for on-board
> detection (1.10). Sections 1, 3, 4 and 5 below are rewritten for that
> hardware; the PiCar-X version is in git history. Sections 2, 6 and 7 were
> chassis-independent and stand as written.

Status: explainer. Nothing built. Companion to `PLAN-sim-hardening.md`,
which covers the simulation work worth doing first, and to
`PLAN-onboard-perception.md`, which holds the parts list, the bill of
materials and the reasoning behind each part; this doc covers what the
physical robot changes about the architecture and what to check on
hardware day.

---

## 1. What you're actually buying

The parts and prices are `PLAN-onboard-perception.md` 3.6; the reasoning
for each is in its section 1. What matters here is how the parts map onto
`robot/interface.py`'s `RobotInterface`, because that mapping is the whole
of `robot/hardware_robot.py`, the one file still unwritten:

*Table corrected 2026-09-28 to the parts in `HARDWARE-BOM.md`; the 2026-09-04
rows it replaced (STM32 board, two SG90s, Camera Module 3, Hailo-8L, Pi 5) are
in git history.*

| Part | What this repo needs it for |
|---|---|
| Yahboom 2WD chassis, two encoder motors, **Waveshare General Driver (ESP32)** over serial (`T=1` wheel command, `1001` feedback) | `drive_forward()`, `reverse()`, and -- because it pivots in place -- `turn_left()` / `turn_right()` as literal turns. `HardwareRobot` in `robot/hardware_robot.py` |
| **One ST3215 pan servo** (no tilt) | `look_left()` / `look_right()` / `look_center()` |
| RPLidar C1, USB | `get_depth_grid()` -- the 360-degree ring -- and `get_distance()` as the path reduction of it (`robot/safety.py`'s `path_clearance()`); `get_scan()` for SLAM |
| **IMX219** CSI camera | `get_camera_frame()` |
| **Jetson Orin Nano Super** | `robot/server.py`, `control/brain_server.py` (including the on-board detector, `brain/perceive.py`), and the ROS 2 container in `service/slam/` -- section 7 |

**Use the vendor's protocol, not the vendor's stack.** Yahboom ships a
Python library and ROS packages for its driver board. What
`hardware_robot.py` needs from it is the serial or I2C protocol for "set
wheel velocities" and "read encoder counts", and nothing else -- this
project already has its own brain, vision service and safety layer, and
running a second stack on top would be two robots fighting for the same
motors. Check the board's protocol documentation before writing against
any library names.

**The one line that matters for the sim is the pivot.** Differential drive
rotates in place, which is what `sim/grid_world.py` has always assumed.
The PiCar-X could not, and the largest phase in `PLAN-sim-hardening.md`
(S6) existed to model that; the purchase retired it.

---

## 2. How the "Guide" tab aligns with the car

**As a feature, it doesn't -- and that's fine.** Guide and the robot are
mirror images:

| | Guide tab | Vision Autopilot |
|---|---|---|
| Camera | the phone's | the robot's |
| Who moves | **the person** | the robot |
| Output | an arrow on screen | `FORWARD` / `LEFT` / `STOP` |
| AWS route | `/guidance` | `/navigate` |

Same vision service, same schema family, opposite actuator. Buying the
car adds nothing to Guide, and Guide adds nothing to the car's driving.

### But they share more backend than "same service" implies

Worth being precise here, because it determines how much Guide's
real-world testing actually transfers to the robot.

`/guidance` and `/navigate` are siblings in the same file
(`service/vision_analyze/vision_core.py`), and the two functions are
near-copies:

| Layer | Shared? |
|---|---|
| ECS container (`service/vision_analyze/`) | same one |
| ALB routing | same -- the twin's `ListenerRule` only claims `/`, `/action`, `/stop`, `/distance`, `/frame`; both of these fall through to vision-analyze |
| Auth + upload | same `_decode_image()` -> same `_check_secret()`, same 5MB cap, same base64 decode (`service/vision_analyze/app.py` `_decode_image()`) |
| Bedrock client | same `_get_client()`, same `MODEL_ID` |
| Model call | same `converse()`, same `maxTokens: 300`, same image-format conversion |
| Browser config | same "Cloud endpoint settings" URL + secret, same `deriveServiceUrl()` |

The **only** differences are the prompt template and the response parser:
`/navigate` returns a discrete verb (`FORWARD`/`LEFT`/`RIGHT`/`STOP`),
`/guidance` returns a 5-zone position + proximity + a human-readable
sentence + an optional `bounding_box`.

### What Guide does *not* touch

Guide never calls `robot/server.py`. Not `/action`, not `/frame`, not
`/distance`, not `/stop`. It never passes through `robot/safety.py`,
never triggers a veto, never resets the watchdog. There is no actuator in
its loop at all -- the person is the actuator.

```
                                   ┌─ shared from here down ─┐
  Guide:   phone camera ──────────►│                          │
                                   │  ALB → vision-analyze    │
                                   │  → Bedrock → Claude      │
  Autopilot: FPV render ──────────►│                          │
              │                    └──────────────────────────┘
              │                              │
              │                       action ▼
              └────────► robot/server.py ────┘   ← Guide never gets here
                         robot/safety.py
                         the motors
```

### Why this makes Guide the most valuable thing built so far

Guide is **the only part of this project ever tested against real
photographs.** Everything proven on the robot side was proven against a
flat-shaded raycaster render of a maze (then the twin's JS `renderFPV`,
deleted 2026-09-25; now `sim/renderer.py`).
Guide has been run on an actual iPhone pointed at actual rooms, over
cellular, and it works.

Because the backend path is shared down to the model call, that testing
transfers concretely:

- **Measured latency and cost per call already exist** for the cloud leg,
  and carry over directly -- same model, same token budget, same image
  pipeline.
- **Ingress, auth, container, Bedrock, encode/decode, JSON parsing,
  error handling, and throttling** have all been exercised end to end
  from a phone on a real network.
- There is even an asymmetry in Guide's favor: today Autopilot sends a
  canvas raycaster render while Guide sends a real camera JPEG. On
  hardware, Autopilot starts sending real camera JPEGs too -- so
  **Guide's payload is a preview of the *hardware* payload, not the sim
  one.**

What none of this tells you is anything below the divergence point:
whether `min_distance_cm: 20` is survivable, how far the car coasts after
a stop, whether the lidar sees a glass door. That half has never run against
anything real -- see section 3 and `PLAN-sim-hardening.md` section 7.

---

## 3. How far to go before buying

**Do these first** (they are correctness work, not simulation work):

- **Phases S1-S5 of `PLAN-sim-hardening.md`** -- pin the interface
  contract, real image bytes, the Python HTTP client, time in the loop,
  sensor noise. **All built** (S1, S3 and S4 on 2026-08-28; S2 on
  2026-08-31; S5 with its cone deferred).
- **The cheap real-world test:** photograph real rooms with a phone and
  replay those JPEGs through `/navigate`. **No robot required.** Done many
  times over -- `CLAUDE.md`'s Stage 0 notes hold the results, and the
  standing finding is that the corpus has to be re-recorded at robot height
  with the target on the floor before it says anything about wording.
- **The Hailo compile loop** (`PLAN-onboard-perception.md` 1.10 item 1):
  one model from Hugging Face to a HEF on an EC2 box, scored over the
  recorded walks. Nothing on the car depends on it, but without it the
  accelerator arrives as a fixed-function part. **Moot 2026-09-19:** the
  board is a Jetson and the Hailo path is not being pursued.

**Then buy.** Past that point the work shifts to modelling things that
could simply be measured: how far the car rolls in one 0.5s move, how far
it coasts after `stop()`, what the lidar returns from glass, mirrors and a
dark sofa, and how many encoder ticks make a 90-degree pivot on carpet.
About $555-620 of parts (`PLAN-onboard-perception.md` 3.6) measures those
better than a week of simulator work does. (**Superseded 2026-09-28:** the
Jetson build is ~$944 all-in -- `JETSON-BOM.md`, `BOM-COMPARISON.md`.)

The genuinely unbuyable-around items -- coasting distance against
`min_distance_cm`, and lidar behaviour on real surfaces -- are in
`PLAN-sim-hardening.md` section 7. Before ordering, the open items are
`PLAN-onboard-perception.md` 3.8's five seller questions and 1.10's
ordering-time checks; none blocks anything else. (**Moot 2026-10-05:** those
were the Pi-era items. The Jetson is bought, brought up and kept, with its
two risks closed on the board, and the Rover is ordered --
`PLAN-ros-alignment.md` 3.33, `JETSON-BOM.md` section 9.)

---

## 4. How the car talks to AWS

**Today, AWS is pretending to be the robot.** `robot/server.py` is
deployed to ECS Fargate with `mode: sim`, so the "robot" currently lives
in us-east-1. (**Superseded:** the ECS robot and twin were torn down
2026-09-05 -- `PLAN-aws-cost-redesign.md`. The robot and brain now run
locally and the deployed twin reaches them through `service/tunnel/`.)

**When the car arrives, `robot/server.py` moves out of AWS and onto the
Pi.** It has to, for two reasons.

**Reason 1 -- the watchdog.** The safety loop is 1 second
(`safety.watchdog_timeout_s`). A round trip to us-east-1 and back is
100-300ms on a good day and *unbounded* when Wi-Fi hiccups. If the safety
decision lives in AWS, a dropped packet means the car keeps driving. The
sensor re-check in `robot/safety.py` has to happen inches from the lidar.

**Reason 2 -- home NAT.** The Pi sits behind a home router. Nothing on
the public internet can open a connection *to* it. And this design is
pull-based (the brain calls the robot -- see `PLAN-sim-hardening.md`
section 1), so the caller must be on the LAN.

The shape after hardware:

```
  YOUR HOUSE                                      AWS
  ─────────────────────────────────────           ──────────────────────

  ┌─────────────────────────────┐
  │ chassis + Raspberry Pi 5    │
  │   robot/server.py           │
  │   mode: hardware            │
  │   robot/safety.py  ◄── veto happens HERE, locally
  │   hardware_robot.py         │
  └──────────▲──────────┬───────┘
             │          │   home Wi-Fi, HTTP, milliseconds
   POST /action    GET /frame
             │          │
  ┌──────────┴──────────▼───────┐         ┌──────────────────────┐
  │ BRAIN (MacBook or browser)  │──JPEG──►│ /navigate            │
  │   decides what to do next   │◄─JSON───│ vision service       │
  └─────────────────────────────┘  ~1-3s  │ Bedrock → Claude     │
                                           └──────────────────────┘
```

(Where the brain runs is settled in section 7: on the Pi, as its own
process. The diagram shows it as a separate box because it is one,
wherever it runs.)

**The car never talks to AWS. The brain does.** The car only ever talks
to the brain, over home Wi-Fi. What crosses the internet is one JPEG up
and one small JSON action down every ~2.5s -- which is exactly why the
Autopilot loop is throttled to 2.5s, and why safety must be local. (The
browser Autopilot was deleted 2026-09-25; the brain in the diagram is now
`control/brain_server.py`, a separate process on the car.)

### What happens when a command arrives

One `FORWARD`, all the way down:

1. Brain POSTs `{"action":"FORWARD","speed":50,"duration":0.5}` to the Pi
   over LAN.
2. FastAPI on the Pi stamps `last_command_at` -- this is what keeps the
   watchdog quiet.
3. `SafetyController` **re-reads the real sensor** -- the lidar's path
   zones, via `path_clearance()`. It never
   trusts what the brain claims (`robot/safety.py`).
4. If under `min_distance_cm`: motors stop, HTTP 200 with
   `{"executed": false}`. The AI's decision is overruled by a sensor
   reading, locally, in milliseconds.
5. If clear: `HardwareRobot.drive_forward(50, 0.5)` -- set both wheel
   velocities on the driver board, wait 0.5s, set them to zero.
6. A turn is a pivot: opposite wheel velocities. **Corrected 2026-09-28:**
   as built, `HardwareRobot._pivot()` is **open-loop and timed** -- a 1.2
   rad/s body rate for `angle / 1.2` seconds -- and does not read the
   encoders. A turn closes on the encoders only under `drive: ros`, where
   `robot/ros_drive.py` sends twists and stops on the wheel encoders. The sim
   pivots in place too.

Steps 1-4 are already written and already tested. Steps 5-6's guts, and
the lidar feed behind step 3, are the only genuinely new code.

### From a verb to the motors

`/navigate` returns a bare verb. Everything that turns that verb into
motor power happens in three files (all three now written -- R7 added the
last):

```
/navigate returns  {"action": "FORWARD"}
   │
   ▼  web-twin/app.js sendAction() (the D-pad), or control/remote_robot.py (the brain)
POST /action  {"action": "FORWARD"}          ← the verb, and nothing else
   │
   ▼  robot/server.py ActionRequest fills in the blanks
{"action":"FORWARD", "speed":50, "duration":0.5, "angle":90}
   │
   ▼  robot/safety.py SafetyController.check_and_execute() -- re-read the real sensor, veto if too close
   │
   ▼  robot/safety.py check_and_execute()'s dispatch_table  ← THE translation point
"FORWARD" → robot.drive_forward(50, 0.5)
"LEFT"    → robot.turn_left(90)
"STOP"    → robot.stop()
   │
   ▼  RobotInterface -- MockRobot in the sim, HardwareRobot on the car
```

The dispatch table in `SafetyController.check_and_execute()` is a plain
dict of lambdas mapping verb -> `RobotInterface` method. It is entirely
backend-agnostic, which is why it doesn't change on hardware day. **All
the reality lands in `robot/hardware_robot.py`** -- written 2026-09-26
(R7) against `sim/fake_esp32.py`, the firmware faked on a pty.

What that file had to do, as planned on 2026-09-04 (see the correction
under the table for what it actually does):

| Verb | Interface call | Real hardware |
|---|---|---|
| `FORWARD` | `drive_forward(50, 0.5)` | both wheels at the mapped velocity -> sleep 0.5 -> both wheels zero |
| `REVERSE` | `reverse(50, 0.5)` | the same, negative |
| `LEFT` | `turn_left(90)` | wheels in opposite directions until the encoder delta (or IMU yaw) reads 90 degrees, then zero |
| `STOP` | `stop()` | both wheels zero (the car then coasts -- 5.1) |
| `LOOK_LEFT` | `look_left()` | pan servo to -30 degrees |
| -- | `get_depth_grid()` | one lidar revolution reduced to zones, published with `fov_deg: 360` so `path_zone_indices()` selects by angle (`PLAN-onboard-perception.md` 5.1) |
| -- | `get_distance()` | `path_clearance()` over that grid; the scalar exists for the contract, not as a second sensor |

**As built (2026-09-28):** `FORWARD`/`REVERSE` run both wheels at a
velocity for a computed time and `LEFT`/`RIGHT` pivot open-loop for
`angle / 1.2` seconds (`HardwareRobot._move()` / `_pivot()`); nothing reads
the encoders to end a verb. Wheel positions are integrated from the `1001`
frame's speeds, because that frame carries no counts. Closing a verb on the
encoders is `drive: ros`'s job (`robot/ros_drive.py`). The paragraph below
is the goal, not the current code.

**A 90-degree turn is real on this chassis, and it is a calibration, not a
timer.** Encoder counts per degree of pivot depend on the wheel base and
wheel diameter and shift a little with the floor surface; the IMU, if
fitted, closes the loop directly. Measure the count on carpet and on hard
floor once, and prefer the IMU where the two disagree.

---

## 5. Pre-flight checklist before hardware day

Things the current simulation structurally cannot surface. Each was found
by tracing the verb-to-motor path above; none is discoverable from a
passing test suite.

### 5.1 Every vision-driven move is hardcoded to speed 50 for 0.5s

The model returns only a verb. The caller (`web-twin/app.js` `sendAction()`,
or the brain's `RemoteRobot`) posts only a verb. `robot/server.py`
`ActionRequest`'s Pydantic defaults (`speed=50`,
`duration=0.5`, `angle=90`) supply everything else. So the entire
autonomy stack moves in one fixed quantum.

In sim that quantum is exactly one 30cm cell. On hardware it is however
far the car actually rolls in half a second at speed 50 -- an unknown
number to be measured once and then treated as the sim's cell size.
Measure it before tuning anything else; several other constants
(`min_distance_cm`, the map scale in `PLAN-sim-hardening.md` section 3.4)
are only meaningful relative to it.

**And the top speed is a safety parameter.** The chosen motors reach about
1 m/s on 65mm wheels (`PLAN-onboard-perception.md` 3.8), at which a 200ms
reaction latency consumes the entire 20cm collar. "Speed 50" has to map to
a wheel velocity chosen against that budget (4.4 there), not to half of
whatever the driver board allows.

### 5.2 `LEFT` and `RIGHT` skip the distance check -- resolved by the chassis

`robot/safety.py` defines `FORWARD_ACTIONS = {"FORWARD"}`, so only
`FORWARD` triggers the sensor re-read before dispatch. On the PiCar-X that
was a real gap -- an Ackermann turn is a forward arc. On a differential
chassis a turn is a pivot, and a pivot does not consume forward space, so
the line is **correct as written**. Leave it. (This said `REVERSE` also
skips the check. **Corrected 2026-09-27:** since R2b, `REVERSE_ACTIONS` are
checked against the scan's rear beams on every path, D-pad included --
`robot/safety.py`, `PLAN-ros-alignment.md` 3.10.)

One residue: a rectangular chassis sweeps a circle wider than itself when
it pivots (`PLAN-onboard-perception.md` 3.7), so a pivot hard against a
wall can clip it. The ring can see that too; whether it is worth a
side-clearance check before a pivot is a measurement, not a sim question.

### 5.3 Validate the lidar in the actual house before wiring the collar to it

The old item here asked whether the ultrasonic panned with the camera,
because the frontier policy's `look_left(); get_distance()` peek only works
if the sensor turns with the view. A 360-degree lidar answers every bearing
at once, so the peek is metric whichever way the camera points, and the
question is gone.

What replaces it is `PLAN-onboard-perception.md` 3.3's opening move: get
scans into Python and check them against the real rooms -- glass, mirrors,
dark matte fabric, mounting vibration -- **before** `path_clearance()`
reads them. Layering a veto onto scans that have not been validated is how
a mounting problem gets debugged as a safety-layer problem. The lidar sees
one plane: chair legs, not seats; the camera still owns everything above
and below it.

### 5.4 Do not carry the vision proximity veto onto the car

`brain/agent.py` has a `vision_proximity_veto` (off by default) that stops
a `FORWARD` when the model's own `distance_estimate` says
`within_one_step`. It exists **only** because `ReplayRobot` and
`TeleopRobot` have no distance sensor -- `get_distance()` returns
`robot/interface.py`'s `NO_SENSOR_CM` -- so on those backends
`robot/safety.py`'s veto is dead code and a whole recorded walk says
nothing about collision avoidance.

`HardwareRobot` will have a real lidar, which makes the veto both
unnecessary and actively wrong there. Two reasons, and the second is the
one that bites:

1. **A measurement must win over a guess.** The veto already returns
   early whenever `get_distance()` reports anything below `NO_SENSOR_CM`,
   so on the car it would be inert -- but leaving it wired reads as
   "vision helps with obstacles", which invites someone to trust it.
2. **The signal is not calibrated and the models disagree wildly.** On
   identical frames one model reports `obstacle_ahead` ~100% of the time
   and another ~0%; `distance_estimate` says `within_one_step` on 60% of
   real walk frames. On hardware the lidar is the obstacle sensor.
   Do not let the vision policy be the thing relying on either field.

Nothing needs doing on hardware day except *not* copying it into
`robot/hardware_robot.py`. It is listed here because the tempting move --
"we already have obstacle logic, reuse it" -- is the wrong one.

### 5.5 Re-measure the chassis width

`robot/safety.py`'s `CHASSIS_WIDTH_CM` is still the PiCar-X's 16.5cm. This
section used to say that over-states the chassis (148mm) and so errs wide.
**Corrected 2026-09-27: it errs NARROW.** 148mm is the deck; across the
wheels the chassis is 19.8cm (the xacro, nav2's footprint,
`FOOTPRINT_WIDTH_M`), so the cone is ~3cm too narrow. Since 3.18
(`PLAN-ros-alignment.md`) a footprint corridor check on the scan runs in
series with the cone and covers the gap. The constant sets the path cone's
half-angle. Measure the real chassis with its
wheels on, set the constant, and re-run `tests/test_depth_veto.py`. M10 in
`PLAN-microduck-transplants.md` owns the rest of that cone.

### 5.6 Items the code added (2026-09-28)

R4-R7 made several hardware-day steps necessary that this checklist did not
have. Detail is in `HARDWARE-BOM.md` (section 4.2 and correction 5 for the
board, section 6 for power) and `PLAN-ros-alignment.md` 3.16.

- **Flash the ESP32 firmware to `mainType` 3 with this chassis' constants.**
  `HardwareRobot` sends `T=1` as wheel speeds in m/s, which is what
  closed-loop `mainType` 3 means. In the stock `mainType` 2, `T=1` is
  open-loop PWM (`L x 512`), so a command of 0.5 is full power. The wheel
  diameter, counts per revolution and track width are compiled into the
  firmware, not sent at run time (`HARDWARE-BOM.md` correction 5). And
  `HardwareRobot` never sends `T=900` itself, so the board must already be
  in `mainType` 3 when the robot server starts.
- **Give the robot server the serial port.** `mode: hardware` refuses to
  start without `ROBOT_SERIAL` (or `hardware.serial_port` in
  `config/robot.yaml`) -- `robot/factory.py`. Pin the device with a udev
  rule (the ESP32 board and the lidar both enumerate as `/dev/ttyUSB*`, in
  no fixed order), and put the user that runs `robot/server.py` in the
  `dialout` group, or opening the port fails with a permission error.
- **Verify the board's heartbeat stops the motors** (read from the firmware
  source in 3.16; not yet seen on a real board). `HardwareRobot` sets it
  to 1500 ms (`HEARTBEAT_MS`, `T=136`), above the robot server's 1.0 s
  watchdog so that the watchdog normally acts first. Pull the serial cable
  with the wheels spinning (wheels off the ground) and time the stop.
- **Measure `safety.sensor_to_bumper_cm`** (`config/robot.yaml`, shipped as
  0.0). It is the lidar-to-bumper offset subtracted from every reading, and
  0.0 is wrong the day a deck-centre lidar is fitted (`robot/safety.py`).
- **Measure the xacro's `[PLACEHOLDER]` block** in
  `service/slam/src/picar_description/urdf/picar.urdf.xacro`:
  `wheel_separation` (track width, 0.172 m, also in `sim/mock_robot.py`,
  `controllers.yaml` and `robot/hardware_robot.py`), `wheel_width`,
  `deck_height`, `axle_x`, `laser_z`, `pan_x`, `pan_z`, `camera_up`,
  `camera_pitch`, `pan_limit`. `tests/test_urdf.py` pins that the xacro,
  `controllers.yaml` and `sim/mock_robot.py` agree; change all copies
  together.
- **Battery voltage cutoff -- specified, not implemented.** `HARDWARE-BOM.md`
  section 6 asks for a filtered warning near 10.5 V and a motor stop plus
  clean shutdown near 10.0 V under load. Nothing in `robot/` reads the pack
  voltage today (`HardwareRobot` ignores the `1001` frame's `v` field).
  Build it before running on battery unattended.

---

## 6. Two gotchas worth knowing now

**Mixed content.** `web-twin/README.md` already documents this for the
Guide tab, but it becomes structural with a real Pi: an HTTPS page (the
CloudFront URL from `cloudformation/cdn.yaml`) is **blocked by the
browser** from calling `http://192.168.1.50:8000`. So it's one or the
other per page load:

- Load the twin **from the Pi** over LAN HTTP -> driving works, Guide's
  camera does not (`getUserMedia` needs a secure context).
- Load the twin **from CloudFront** over HTTPS -> Guide works, driving
  the Pi does not.

Fixable (self-signed cert on the Pi, or Tailscale), but worth deciding
before hardware day rather than while holding a robot.

**The AWS twin deployment doesn't die, it changes job.** It stops being a
fake robot and becomes (a) the page host and (b) the vision service.
Worth keeping a `mode: sim` instance running there permanently as a demo
that works without unpacking the car.

---

## 7. Where should the brain live?

`README.md` and `CLAUDE.md` both describe the split as "Raspberry Pi
(robot runtime) + a MacBook running a Vision LLM (high-level
reasoning)." That split was correct when it was written. It is worth
re-deciding before hardware day, because the premise behind it has since
changed.

### The original reasoning

A Raspberry Pi cannot run a vision model; a MacBook can. So: heavy
thinking on the laptop, motors and sensors on the Pi, Wi-Fi between them.
A sound edge/compute split -- **if the brain is doing real compute.**

### It isn't, any more

Vision moved to Bedrock (`service/vision_analyze/`). What `brain/`
actually does on hardware is four HTTP calls in a loop:

1. `GET /frame` from the Pi
2. POST the image to AWS
3. receive `{"action": "FORWARD"}`
4. `POST /action` to the Pi

That is coordination, not computation. The whole of `requirements.txt` is
`pyyaml pytest anthropic fastapi uvicorn pydantic httpx` -- no torch, no
opencv, not even numpy. `brain/`'s imports are stdlib plus `anthropic`,
which is an HTTP client. A Pi Zero would run this without noticing.
(**No longer true, 2026-09-28:** `requirements.txt` has grown, and
`policy: "tiered"` loads a YOLOE detector and CLIP into the brain process
from `requirements-perception.txt`, so the brain does real compute again --
which is part of why the board is a Jetson. The argument for putting the
brain on the car, below, is unchanged.)

**The reason for the MacBook was "the Pi is too weak to think," and the
Pi no longer has to think -- Bedrock does.**

### What the split costs now

Because the brain *pulls* from the Pi (section 1), it has to sit on the
LAN. That one constraint is the root of most of the friction elsewhere in
this document:

- The image travels Pi -> MacBook (LAN) -> AWS. An extra hop in a loop
  already running at 1-3s.
- The robot only works when the laptop is on, awake, and on the same
  Wi-Fi.
- It is the direct cause of the NAT and mixed-content problems in
  section 6.

Put the brain on the Pi and all three dissolve: the Pi calls AWS
**outbound**, which passes through home NAT without ceremony, and the
robot becomes self-contained.

### The reframe

"Brain on the MacBook" conflates two jobs that the web twin already
separates in practice:

| | Where it *can* live | Where it *should* live |
|---|---|---|
| **Autonomy loop** -- decide the next action | anywhere | the Pi |
| **Operator console** -- watch, drive manually, reset | a screen in your hand | your phone |

Today the browser does both at once. On hardware they want to be apart.

Note the brain's location is not actually settled in the code either --
there are already three implementations of it (`brain/agent.py` in
Python and in-process, the browser's JS frontier explorer, and the
browser's Vision Autopilot), with `brain/planner.py` a possible fourth.

### What this does to the watchdog

`robot/server.py`'s watchdog exists to detect a *dead brain*: if no
command arrives for `watchdog_timeout_s`, stop the motors. Moving the
brain onto the Pi narrows what it can detect, and how much depends on
*how* it is moved:

- **Brain as a separate process on the Pi**, talking to
  `robot/server.py` over localhost HTTP: the watchdog still catches a
  brain crash, hang, or deadlock -- commands genuinely stop arriving. It
  only stops catching *network* failure, since localhost does not drop.
  Most of its value survives.
- **Brain inside the server process** as an asyncio task: largely
  defeated. A synchronous block in the agent loop blocks the event loop
  the watchdog polls on.

This is a strong argument for the separate-process design -- see
`PLAN-brain-relocation.md`.

Independently of brain location, the watchdog keeps one job that matters
on hardware and has no equivalent in sim: **if a movement call sets the
motors and then crashes before `stop()`, the motors keep running.**
(On the ESP32 board the firmware heartbeat is a second backstop for this --
section 5.6.)
Nothing else in the system catches that.

The genuinely new gap is that **nothing guards the AWS link.** The
browser's Vision Autopilot currently catches a `/navigate` failure,
logs it, and schedules the next tick (`web-twin/index.html`, in
`visionAutopilotStep`'s `catch`; deleted 2026-09-25 with the rest of the
browser brain) -- it does not issue a STOP. That is
survivable today only because motion is discrete and brief: each command
moves for 0.5s and stops on its own, so a hung brain leaves a stationary
car. It is worth making explicit rather than leaving as an accident of
timing.

### Recommendation

Move the autonomy loop onto the Pi as its own process; keep the MacBook
path for development, where it is genuinely better (editing, `pytest`,
breakpoints); make the phone a pure observer; keep the motor watchdog and
add a vision-link failsafe.

### How that gets built

Full detail in `PLAN-brain-relocation.md`; the shape of it:

**The whole trick.** If the brain is *always* an HTTP client of
`robot/server.py`, then "brain on the Pi" and "brain on the MacBook"
differ only by a base URL:

```
on the Pi        brain -> http://localhost:8000     (robot/server.py)
on the MacBook   brain -> http://192.168.1.50:8000  (same server, over LAN)
```

Same code, same process model, same tests. Where the brain runs stops
being an architectural question and becomes a deployment one. Everything
else exists to make that true.

**Target topology -- two processes on the Pi, not one:**

```
  THE PI                                             AWS
  ─────────────────────────────────────              ─────────────────
  ┌──────────────────────────────┐
  │ :8001  control/brain_server  │  ── JPEG ──────►  /navigate
  │        the autonomy loop     │  ◄── action ───   vision service
  │        (RemoteRobot client)  │
  └───────────┬──────────────────┘
              │ localhost HTTP
              ▼
  ┌──────────────────────────────┐
  │ :8000  robot/server.py       │
  │        safety + watchdog     │
  │        hardware_robot.py     │
  └──────────────────────────────┘
              ▲            ▲
              │ manual     │ mission start/stop + status
        ┌─────┴────────────┴─────┐
        │  PHONE (web twin)      │   observer + operator console
        └────────────────────────┘
```

**Phases:**

| | What | Proof it worked |
|---|---|---|
| **B0** | `RemoteRobot` -- an HTTP client implementing `RobotInterface`. This is Phase S3 of `PLAN-sim-hardening.md`, and the only hard prerequisite. | Identical action sequences in-process vs. over HTTP |
| **B1** | `MissionRunner` -- turn `ObjectSearchAgent.run_mission()`'s blocking `for` loop inside out into `start()`/`stop()`/`tick()`/`status()`. No decision logic moves. | Driving `tick()` in a loop reproduces `demo_active_search.py`'s result |
| **B2** | `control/brain_server.py` on :8001 -- `POST /mission/start`, `POST /mission/stop`, `GET /mission/status`. Talks to the robot only through `RemoteRobot`. | Start a mission over HTTP, poll to completion, backpack found |
| **B3** | Three failsafes for three distinct failures: motors-left-running (the existing watchdog, kept), AWS-link-dead (new), brain-loop-hung (new). | A stub vision function that fails or hangs ends the mission with the robot stopped |
| **B4** | Twin becomes an observer: Explore/Find POST to the brain and render polled status, instead of running JS timers. Manual D-pad still talks straight to the robot. | Start a mission from the phone, background the tab, robot keeps going |
| **B5** | Two `systemd` units on the Pi, brain ordered after robot. | Reboot the Pi; a mission runs with no laptop on the network |

*Status 2026-09-28: B0-B4 are built (`control/`); only B5 is left, now on the
Jetson rather than a Pi.*

**Why two processes and not one.** Running the loop as an asyncio task
inside `robot/server.py` is less code and wrong here: a synchronous block
in the agent loop would block the event loop the watchdog polls on
(defeating it), it merges the two roles `CLAUDE.md` section 2 names as
the constraint not to break, and it loses the base-URL trick above. The
cost of separating them is a localhost round trip -- about a millisecond,
against a loop that spends 1-3 seconds waiting on Bedrock.

**What is testable now.** B1 through B3 need no hardware and no Pi --
they run against `mode: sim` today. B4 needs two local `uvicorn`
processes. Only B5 needs the robot. Together with
`PLAN-sim-hardening.md`'s S1-S4, this is the bulk of the pre-purchase
work remaining.

**The single clearest test** is B4's: start a mission from your phone,
then background the tab. If the robot keeps going, the brain has moved.
Today it stops -- there is an explicit `visibilitychange` handler that
kills the autopilot timer. (Written before B4. Since B4 the mission runs in
`control/brain_server.py` and survives the tab; the browser loops were
deleted 2026-09-25.)
