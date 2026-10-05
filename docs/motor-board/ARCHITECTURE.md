---
kind: architecture
domain: motor-board
status: current
verified: 2026-10-02
---

# Motor board -- architecture

The motor board is the microcontroller that turns the Rover's wheels. The
car is the Waveshare UGV Rover, and its board is an ESP32 that Waveshare
calls the "ROS Driver". Despite the name it contains no ROS: it speaks
newline-delimited JSON over a serial line. This domain covers four things:

- the board's protocol, as its firmware source defines it;
- this project's fork of that firmware;
- the robot-side backend that makes the board a body backend;
- a fake board that lets the whole stack exercise that backend with no board
  present.

Read this before changing how the car's wheels are commanded or how its
odometry is computed. The body contract the backend implements belongs to
the body domain. The vetting of each move belongs to the safety domain. The
frames, constants, commands and tests are in the
[engineering spec](../engineering/motor-board/ENGINEERING.md).

## Purpose

The board runs the wheel motors' speed loop, counts encoder edges, reports
both to the host, and stops the motors on its own if the host goes quiet.
Everything above it needs two things:

- **velocity in:** a wheel speed it can command;
- **odometry out:** how far each wheel has turned.

Odometry has a strict contract here: it never jumps. Three things depend on
the backend:

- the robot server's safety layer, watchdog and wheel loop;
- the ROS chain above the server, which reaches the board only through the
  server;
- direct-mode verbs, which close on the backend's odometry.

The fake board is what made it possible to write and measure the backend,
and its odometry, before the board arrived.

## Components and boundaries

```text
  robot server (safety layer, watchdog, arbitration)
        |  body contract (in-process)
        v
  hardware backend  ---- serial, newline JSON ---->  board firmware (stock or our fork)
   (owns the port;                                      speed loop, encoders, heartbeat
    odometry estimate;  <--- feedback frames -------    periodic feedback frames
    reboot handling)
        |
        +-- other sensors (camera, lidar, depth): separate devices; until their
            drivers exist, a stand-in supplies them or they answer "unusable"

  In the simulator:  hardware backend  <--- pseudo-terminal --->  fake board  ---> simulated body's wheels
```

- **The board firmware** is Waveshare's stock image, or our fork of it. Our
  fork adds fields to the feedback frame and changes nothing else. The board
  sees no obstacles: the lidar and cameras are wired to the Jetson, not to
  the board.
- **The hardware backend** is the only process that opens the serial port.
  It turns body commands into board commands and board feedback into wheel
  state and odometry. The motors are all it knows about.
- **The fake board** follows the firmware source rule by rule, including the
  board's real limits. It turns a simulated body's wheels. It runs the stock or the forked
  firmware.

Lines not crossed:

- **ROS never opens the port.** No ROS node and no ROS hardware plugin
  touches the board.
- **The robot server never speaks the board's protocol.** It knows only the
  body contract.
- **No Waveshare host software runs on the car.** Waveshare's stock host
  application and its ROS nodes would hold the port.

## Decisions

### D1. The serial port belongs to a body backend in the robot process

**Decision.** The backend that owns the board is one more body backend, a
sibling of the simulator's. It sits under the robot server's safety layer
and watchdog.

**Rejected:**

- **A ROS hardware plugin that owns the port.** It would take the safety
  layer out of nav2's path and give one port two masters (3.16).
- **Waveshare's Jetson ROS nodes.** They pass velocity commands straight to
  the motors. They also block an executor on a serial read, stamp frames on
  receipt, and integrate whole-centimetre odometry (3.26).

**Trade-off.** Hardware day becomes a configuration change. In exchange,
every ROS velocity reaches the board through one extra HTTP hop.

### D2. Command wheel speed through the board's own closed loop

**Decision.** The host sends left and right wheel speeds. The board's speed
loop holds them. On the Rover's board the loop is closed in the Rover's
mode, straight from the factory.

**Rejected:**

- **A host-side speed loop over raw motor power.** The board is where a fast
  encoder loop belongs.
- **The other Waveshare board (the "General Driver").** It is open-loop in
  this mode and needed a firmware change before it could hold a speed. R7
  was first written against it (3.16, superseded by 3.25).
- **The board's twist command.** It does not feed the board's heartbeat.

### D3. The board's heartbeat is a backstop, set longer than the server's watchdog

**Decision.** The host sets the board's heartbeat to a little longer than
the robot server's watchdog. The server always acts first. The board stops
the wheels on its own if the server itself dies.

**Rejected:** relying on the board's "emergency stop" command. What it does
to the wheels is **disputed**: the project's audit of Waveshare's code read
it as aimed at the arm's servos (3.26), while the fake board, which cites the
firmware function behind each rule, models it as stopping the wheels. The
firmware source is not in the repo to settle it. Either way the design never
relies on it: the host never sends it, and stops by commanding zero speed.
Nothing replaces the safety layer: the board sees no obstacle.

### D4. Odometry is a speed integral clamped by the board's odometers

**Decision.** The host integrates the measured wheel speeds. On every frame
it clamps each wheel's estimate into the range that wheel's odometer
reading allows. The odometer anchors the estimate, and the speeds
interpolate between readings.

**Rejected:**

- **Odometers alone.** A whole centimetre is 3.3 degrees of heading on this
  track. That is Waveshare's own approach.
- **The speed integral alone.** It drifts with every lost line: 9.57 cm over
  30 s with 5% loss (3.25).

**Trade-off.** The error is bounded by one odometer unit forever, and
sub-unit motion still reads.

### D5. A board reboot never makes odometry jump

**Decision.** A brownout or USB reset zeroes the board's counters and makes
it forget the host's set-up. The host detects the reboot and moves its
origin, so the travel it reports carries on. It also sends its set-up
again.

**How a reboot is detected.**

- With our fork, the board's clock going backwards.
- With stock firmware, both odometers back near zero while the estimate is
  far from them. A far odometer alone is not enough: lost and bunched frames
  produced that falsely (3.25).

### D6. Fix the frame, not the host: our firmware fork adds fields only

**Decision** (3.28, amended by 3.29). Two limits are in the stock frame
itself:

- odometers in whole centimetres;
- no time of measurement.

No host-side change can remove them. The fork adds finer odometers and the
board's clock to the frame and leaves every stock field alone. A host that
does not know the new fields, and Waveshare's own tools, behave exactly as
on a stock board. The host uses the new fields only when all of them are
present.

**Rejected:**

- **Changing existing fields.** It would break stock tools.
- **Millimetre odometers.** They were measured as the limit on turn accuracy
  (3.29), so the fork reports a finer unit, below one encoder edge.
- **On-board yaw from the IMU.** Larger work, and deferred.

**Licence.** The firmware is GPL-3.0. Running it on our own robot needs
nothing more. Conveying a binary would mean conveying our source too.

**Flashing waits, and is reversible.** The fork is flashed only after the
purchase's arrival checks, and only in a way that lets the stock image be
restored (the procedure is in the
[engineering spec](../engineering/motor-board/ENGINEERING.md)).

### D7. Correct a verb's leftover error only where the estimate is good enough

**Decision** (3.29). After a verb, a slow second look corrects what is left,
and it is vetted like any move. It runs only when the board reports the
fork's fine odometers.

**Rejected:** settling on stock firmware. There the estimate at rest is
itself about 1.5 degrees out, so a settle chases noise and adds about
0.6 s a turn.

**A correction is a move like any other.** It is vetted by the safety
domain's rules for driving with no sensors, which belong to that domain
([safety engineering](../engineering/safety/ENGINEERING.md), Known gaps). On
the car before its lidar driver lands, nothing can see ahead or behind, so
the car cannot drive straight at all: every forward and every reverse is
refused, and only turns, and turn corrections, run.

### D8. The fake follows the firmware source and runs on a real serial device

**Decision.** The fake board cites the firmware function behind each rule.
It reproduces the board's limits, and it runs on a pseudo-terminal, so the
backend's real serial code is exercised.

**Rejected:** mocking the backend at the Python level. A fake that behaves
better than the board hides exactly what it exists to find.

**What this found.** The pseudo-terminal found host defects no mock would
have: a close that hangs while a read is blocked, a reader too slow for the
feedback rate, partial lines (3.16, 3.25).

### D9. The board is only the motors

**Decision.** The camera, lidar and depth camera are separate devices, with
their own drivers, and none of them is this domain's. Until those exist, a
stand-in body supplies them in the simulator. On the car they answer the
body contract's defaults, and those are not all silent: the distance sensor
reads as an obstacle at zero range, which refuses every forward move, and a
request for a camera frame fails rather than answering "unusable". Where the lidar driver lives is decided by the
[ros architecture](../ros/ARCHITECTURE.md) (D3).

## Contracts

| Neighbour | Direction | Protocol | Ownership |
|---|---|---|---|
| Robot server | The server calls the backend | In-process body contract (owned by the body domain) | The backend owns the port and the odometry estimate. The server owns the decision to move (safety domain). |
| Board firmware | Both ways | Serial: newline-delimited JSON, commands in, feedback frames out | The board owns the speed loop, the encoder counts and the heartbeat. The host owns the set-up and must re-send it after a reboot. |
| ROS chain | None directly | -- | It reaches the board only as wheel-velocity posts to the robot server: see [ros](../ros/ARCHITECTURE.md). |
| Simulator | The fake board drives a simulated body's wheels | In-process | The simulator owns the truth that tests judge against. |
| Firmware fork | Applied to Waveshare's source at a pinned commit | A patch, plus a build with pinned toolchain versions | This project owns the patch. Waveshare owns the base. |

## Failure modes and resilience targets

| Failure | What happens | Target |
|---|---|---|
| The robot process dies or the link is cut | The board's heartbeat zeroes the wheel speeds | Wheels stop within the heartbeat interval plus one board loop, with no host involvement |
| Feedback lines are lost or arrive bunched | The odometer clamp corrects the integral | Each wheel within one odometer unit plus one encoder edge of the truth, at every frame (stock: worst 1.02 cm; fork: worst 0.047 cm, 30 s with 5% loss) |
| The board reboots mid-drive | The origin moves and the set-up is re-sent | Reported travel moves by at most one unit; the heartbeat is restored within 0.5 s (8 of 8 runs on both firmwares) |
| The board's clock wraps after 49.7 days | Read as a small step forward | No false reboot, no jump |
| Frames carry only some of the fork's fields | Read as stock | Never a mixed interpretation |
| A garbled or partial line | Skipped | The next frame is read normally |
| Turn accuracy, direct mode | Fork: settle pass. Stock: none | Fork: 120 of 120 clear turns within +/-1 degree (worst 0.84). Stock: unbiased but sd about 1.5 degrees, a measured and accepted shortfall; on the car SLAM or the gyro corrects heading |
| Straight accuracy | -- | A clear move covers its 30 cm within 0.5 cm on the fork and 1.34 cm on stock |

## Open questions

- **Flashing the fork.** After the arrival checks. Who decides: the user,
  on the Rover's arrival (`JETSON-BOM.md` 9.5).
- **Which serial device on the car.** The kit's own code opens the Jetson's
  header UART. The board also has a USB bridge. Settled on arrival.
- **Using the gyro.** The gyro is already in the feedback frame. It would
  improve heading on stock firmware, or past the fork's bar, as an
  on-board or host-side filter (3.26 step 3). Not needed while the fork
  meets its bar.
- **Unmeasurable before arrival:**
  - the speed loop's deadband at low speeds (slow pivots may stick-slip);
  - the board's real loop and feedback timing;
  - the effective skid-steer track width. Where it goes is settled
    (`PLAN-ros-alignment.md` 3.35): one setting, the track scrub, read by
    the robot server and the ROS container; only the value is owed.
- **Battery voltage.** The board reports it. A software cutoff is owed by
  the platform and safety domains, not here.
