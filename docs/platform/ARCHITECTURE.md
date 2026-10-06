---
kind: architecture
domain: platform
status: current
verified: 2026-10-02
---

# Platform -- architecture

The platform is the physical robot: the compute board, the chassis and its
motor board, the sensors, and the power that feeds them. This document
records what was chosen, what was rejected and why, and what the software
may and may not assume about the hardware. Read it before buying,
mounting or powering anything. Part numbers, measured offsets, power
figures and the bring-up procedure are in the
[engineering spec](../engineering/platform/ENGINEERING.md). The motor
board's protocol and firmware belong to the motor-board domain
(docs/motor-board/ARCHITECTURE.md).

**State.** The compute board (a Jetson Orin Nano Super dev kit) is in hand
and is brought up before the chassis (`PLAN-ros-alignment.md` 3.33). The
chassis (the Waveshare UGV Rover kit) is ordered and not delivered. A
separate battery for the Jetson is conditional and not bought. Nothing has
been mounted, wired or measured on a car. Dates, delivery windows and
return terms are in the engineering spec's inventory, because they change
by the week.

## Purpose

The software was written against a simulator so that hardware day would be
a configuration change, not a rewrite. The platform is what that
configuration points at. It must:

- run the robot server, the brain (including the on-board perception tier)
  and the ROS container on the car itself, with no laptop;
- move as a differential (skid-steer) base that can pivot in place, which
  the simulator, the pivot guard, search turns and the nav2 setup assume;
- report measured wheel odometry to the host, because odometry is
  load-bearing for verbs, SLAM and arrival;
- carry a 360-degree lidar for safety and mapping, and a camera for
  perception;
- stay powered through motor peaks without browning out the computer.

## Components and boundaries

```text
   +-------------------- UGV Rover chassis --------------------+
   |  6 wheels, 4 driven, skid steer   pan-tilt camera          |
   |  ESP32 motor board (closed loop)  depth camera (stereo)    |
   |  3S pack + UPS board              360-degree lidar         |
   +------+------------------+------------------+--------------+
          | serial (JSON)    | serial           | USB / CSI
   +------v------------------v------------------v--------------+
   |  Jetson Orin Nano Super (JetPack 6, Ubuntu 22.04)          |
   |   robot server  -- motor board, lidar (veto inputs)        |
   |   brain         -- perception tier on the GPU              |
   |   ROS container -- SLAM, nav2, behind the HTTP wall        |
   +-------------------------------+---------------------------+
                                   | Wi-Fi
                              phone (twin), cloud vision
```

| Part | Owns | Boundary |
|---|---|---|
| Compute board | All on-car software: robot server, brain, ROS container | Runs no vendor robot software that writes to the motors |
| Chassis and motor board | Turning wheel commands into wheel speeds, and reporting measured odometry | Sees no obstacles. Its heartbeat is a backstop, never the safety layer |
| Lidar | The 360-degree range ring for the safety corridor, pivot guard, arrival and SLAM | Read by the robot server, not only by ROS, because it feeds a veto |
| Cameras | Pixels for perception, and (proposed) depth below the lidar's plane | Which camera feeds perception on the car is still to be measured |
| Power | Feeding motors, sensors and the computer through peaks | A battery's protection board does not protect the computer. Software must |

## Decisions

### Compute: Jetson Orin Nano Super, not a Pi 5 with a Hailo accelerator

**Decision.** Closed on 2026-09-19 by the user. Do not re-open it, and do
not spend on another Hailo compile run (`CLAUDE.md` section 3b).

**Alternatives rejected.**

- **Raspberry Pi 5 + Hailo-8L** (the plan from 2026-09-04 to 09-17). OWLv2
  does not compile to a Hailo at all, because its transformer body fails
  allocation (P6, P17). As a crop source in the tier it read 83% recall
  against the Pi tier's 50% on identical frames (P19). The shipped detector
  today (YOLOE) runs in PyTorch and has never been compiled or quantised
  for a Hailo. Its predecessor with the same text head fell from 34% to 5%
  under INT8 (P13). The Hailo M.2 module is also out of stock; only a
  soldered HAT is sold (`BOM-COMPARISON.md` 4.1, `PI-VS-JETSON.md`).
- **The IMX500 AI camera**, rejected 2026-09-04: a nano-model ceiling in
  silicon, and it cannot be fed a recorded frame.

**Trade-off.** About $86 more on a like-for-like build (`BOM-COMPARISON.md`).
Up to about $250 more on the Rover once the Jetson needs its own battery.
About twice the power draw. Two bring-up risks the Pi did not have: a
working CUDA torch for the installed JetPack, and a devkit firmware update
(one public report of a bricked unit). In return: any PyTorch or Hugging
Face model runs with a pip install, there is an NVMe slot, and there are six
CPU cores plus a GPU for ROS and perception. Both risks were settled on the
board on 2026-10-04 -- JetPack 6 booted with no firmware update, torch on
the GPU -- and the user kept it on 2026-10-05 (`PLAN-ros-alignment.md`
3.33).

### Chassis: the Waveshare UGV Rover kit

**Decision.** Ordered on 2026-09-30 (`JETSON-BOM.md` 9.1, 9.7). It is a
skid-steer base with a closed-loop ESP32 motor board that reports measured
odometry to the host, open GPL firmware, a 360-degree lidar, a stereo depth
camera, a pan-tilt camera and an IMU.

**Alternatives rejected** (`JETSON-BOM.md` 9.2-9.6):

| Candidate | Why not |
|---|---|
| SunFounder PiCar-X | Ackermann steering, cannot pivot. Retired 2026-09-03 (`PLAN-onboard-perception.md` 1.1) |
| Hiwonder ROSOrin | Ordered 09-29, cancelled 09-30. The vendor confirmed its board sends no encoder data to the host, its firmware is proprietary, and its Jetson port cannot sustain 25 W |
| Waveshare Cobra Flex | Runner-up. Better power and drivetrain, but no working IMU in its firmware, sensors and brackets are the buyer's work, and it ships only from China |
| Yahboom ROSMASTER A1 and family | Ackermann or mecanum |
| TurtleBot3 Burger | Battery too small for a 25 W Jetson |
| Husarion, TurtleBot 4, AgileX LIMO | Three to eight times the budget |

Bought through a retailer whose return window is long enough for the
on-arrival checks, rather than direct from the maker, whose return terms
are not (the terms are in the engineering spec).

**Trade-off.** Skid steer scrubs on every turn and the simulator models no
slip. The body is wider than the old chassis, so results name their house,
and nav2 and the ROS chain are judged in the simulator's scaled house
(docs/simulator/ARCHITECTURE.md; the margins are in the simulator's
engineering spec). Its power board is rated for about 5 A continuous,
which is tight for a Jetson at 25 W.

### Software baseline: JetPack 6, Ubuntu 22.04, ROS 2 Humble

**Decision.** JetPack 6 (Jetson Linux R36, Ubuntu 22.04), matching the
Humble container the repo already builds (`HARDWARE-BOM.md` correction 6).
The point release and how it is installed are in the engineering spec.

**Alternative rejected.** JetPack 7, which would move the container to ROS 2
Jazzy for no gain and has a less mature torch story.

**Trade-off.** An older OS, and an older Python on the board than on the
laptop, because NVIDIA's CUDA torch is built only for the board's Python.

### Power: 15 W first, a separate battery only if measured necessary

**Decision.** The Jetson runs at 15 W to start (user, 2026-10-01). A
separate battery for the Jetson is bought only if an on-arrival stress test
at 15 W, with the motors working hard and the Jetson's input voltage
logged, shows sag, or before ever moving to 25 W (`JETSON-BOM.md` 9.5).

**Alternatives rejected.** 25 W from the kit's own supply, which Waveshare
calls untested and the earlier ROSOrin vendor said its port could not
sustain. A separate battery bought up front, which may be unnecessary.

**Trade-off.** Perception is slower at 15 W by an amount to be measured.
Safety does not depend on the power mode: the lidar veto stops the robot,
not the camera.

**Also decided: software battery protection is required.** A 3S pack's
protection board cuts off at or below the Jetson's 9 V input floor, so it
protects the cells and not the board. Only a software cutoff can shut the
computer down cleanly first (`HARDWARE-BOM.md` editor's caution). It is not
built.

### The vendor's Jetson software is not used

**Decision.** Waveshare's stock app and ROS nodes are disabled on arrival.
Their CAD geometry is reused (BSD), and the firmware may be forked (GPL)
(`PLAN-ros-alignment.md` 3.26).

**Alternative rejected.** Waveshare's ROS bridge nodes. They own the serial
port and pass velocity commands straight to the motors, which takes the
safety layer out of the path. Their odometry turns one centimetre count into
a 3.3-degree heading step.

**Trade-off.** We maintain our own serial backend and firmware fork
(motor-board domain).

### Geometry comes from CAD until measured, and each offset is one number

**Decision.** Mounting offsets come from Waveshare's CAD-derived URDF and
are tagged as CAD in the robot description. Offsets with no source are
tagged as placeholders. An offset that more than one component uses (the
lidar's position ahead of the rotation centre) is one number shared by the
safety layer, the simulator and the URDF (`PLAN-ros-alignment.md` 3.27).

**Alternative rejected.** Putting the CAD number into the URDF alone. TF and
the safety layer would then disagree, in the unsafe direction astern and on
pivots.

**Trade-off.** Every CAD value is unverified until measured on the car. The
lidar's 90-degree mounting yaw is not modelled yet.

### Sensors that feed a veto are read by the robot process

**Decision.** Decided by the user on 2026-10-02: the lidar driver runs in
the robot server, and ROS receives the scan through the bridge
(`PLAN-ros-alignment.md` section 6, question 5).

**Alternative rejected.** A ROS lidar driver publishing the scan for the
robot server to read. The safety layer would then depend on the ROS
container being up.

**Trade-off.** The lidar's serial protocol must be ported out of the vendor's
ROS driver, and the simulator needs a matching fake lidar first.

**Consequence for bring-up.** Until that driver exists the car has no
usable scan, so the safety layer lets it pivot but not drive forward or
back; the rule, and the window before the board's first feedback frame,
are recorded in the safety engineering spec
(docs/engineering/safety/ENGINEERING.md). Anything but a turn is a bench
test with the wheels off the floor (the motor-board domain's first-contact
procedure).

### Bring the computer up before the chassis arrives

**Decision.** Decided by the user on 2026-10-02 (3.33). The Jetson's two
open risks are settled while it is still returnable. The box is kept and
nothing on the board is modified until the user decides to keep it.

**Alternative rejected.** Keeping it boxed until the Rover arrives, which
could be after the return window closes.

**Outcome.** Both risks closed on the board on 2026-10-04, and gate G4 and
the headroom run passed on it by 2026-10-05; the user kept the board on
2026-10-05, so the return window no longer applies.

## Contracts

| Between | Direction | Category | Ownership |
|---|---|---|---|
| Jetson and motor board | Robot server commands wheel speeds; board streams feedback | Serial, with a text protocol | motor-board owns the protocol and the serial backend |
| Jetson and lidar | Robot process reads the scan | Serial (USB) | Robot process owns the device; ROS gets the scan through the bridge |
| Jetson and cameras | Perception reads frames | USB (depth camera), CSI or USB (colour) | Which camera feeds perception is open |
| Rover power board and loads | Feeds motors, sensors, and (unless a separate pack is fitted) the Jetson | DC | The two supplies are never joined if a separate Jetson pack is added |
| Robot description and code | Safety, simulator and URDF share chassis constants and the lidar offset | Repo constants checked by a linter | One number per fact, enforced across the ROS wall |
| Car and phone or cloud | Twin and cloud vision | Wi-Fi, HTTP | Operations owns the deployment |

## Failure modes and resilience targets

| Failure | Response | Target |
|---|---|---|
| The host stops commanding (process dies, link drops) | Robot server watchdog first; motor board heartbeat if the server itself is gone | Wheels stop within one watchdog period while the server is alive. The board's own heartbeat is set longer than that period, as a backstop |
| Pack voltage sags under motor peaks | 15 W start; stress test with logged input voltage on arrival | Buy the separate Jetson pack if any sag is seen; never move to 25 W without it |
| Pack runs flat | Software cutoff (planned) | Clean shutdown before the Jetson's 9 V floor. Not built: until it is, watch the pack |
| No working CUDA torch for the JetPack, or a model silently falling back to the CPU | Stop the bring-up at the torch check, which must show every network of the shipped pipeline on the GPU | Met on the board 2026-10-04 (3.33); the setup script now asserts it, so a JetPack upgrade that breaks it stops at the same check |
| Perception too slow on the board | Measure on the pinned frames | Budget: 250 ms a frame at 15 W (user, 2026-10-02). Over it, move image handling off the CPU first |
| Everything at once overloads the board | A headroom run with the whole stack loaded | 0 late safety-loop ticks over 10 minutes, at least 1 GB free, no thermal throttling |
| A CAD offset is wrong on the real car | Measure every CAD value on arrival | The lidar offset is measured before the safety sweep runs on the car |
| The lidar's zero faces left, not ahead | Set its angle offset before the first scan is used | No scan is used by safety or SLAM before the yaw is set |
| No usable scan yet (lidar driver not written) | FORWARD and REVERSE are refused; turns are allowed | Once wheel feedback arrives, the car never translates unobserved. Before the board's first feedback frame the reverse refusal does not yet apply (a known gap in the safety spec) |
| Vendor software holds the serial port | Disable it on arrival | Only one process opens the motor board |
| The chassis fails the on-arrival checks | Return it within the retailer's window | The checks finish inside the window |

## Open questions

- **Which camera feeds perception.** The Rover kit brings a pan-tilt 5 MP
  camera and an OAK-D Lite depth camera (`JETSON-BOM.md` section 1); the
  IMX219 of the 2026-09-17 plan was never bought. `GUIDE-robot-base.md` section 8 says to decide
  by recording walks through the candidate and scoring them. Owner: the
  user, on data.
- **Depth below the lidar's plane.** A floor band from the depth camera is
  proposed (`PLAN-ros-alignment.md` section 6, question 8), with criteria to
  confirm.
- **Skid-steer slip and the effective track.** Measured on the car (R8).
- **The battery cutoff.** Required, not designed: thresholds, the voltage
  source on the board, and a readout on the twin.
- **The 15 W result.** Half answered: 15 W meets the 250 ms budget with
  room to spare (3.33, measured on the board 2026-10-04). Whether the kit's
  supply holds at 15 W with the motors working is measured on the Rover's
  arrival.
