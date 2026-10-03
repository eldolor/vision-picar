---
kind: architecture
domain: body
status: current
verified: 2026-10-02
---

# Body -- architecture

The body is the robot's account of itself: what its sensors read from where
it stands, what its wheels have done, and the motions it can be told to make.
It is one abstract contract with several interchangeable implementations,
and it is the only way the brain and the robot server ever touch a robot.
Read this to understand where the contract's edges are and why each method
is shaped the way it is; read the
[engineering spec](../engineering/body/ENGINEERING.md) for the methods,
return shapes, constants and the conformance suite.

## Purpose

The project is built simulation-first. Every policy, the safety layer and
the robot server are written and measured against a simulated robot, and
the same code must then drive the real one. That only holds if nothing above
the body knows which body it has. The body contract is that guarantee: the
hardware swap is a configuration change, not a rewrite (CLAUDE.md section 2).

Who depends on it:

- **The robot server** ([control-api](../control-api/ARCHITECTURE.md)) holds
  exactly one body and serves its readings and motions over HTTP.
- **The safety layer** ([safety](../safety/ARCHITECTURE.md)) reads every
  clearance from the body and carries out every guarded motion through it.
- **The brain** ([mission](../mission/ARCHITECTURE.md),
  [policy](../policy/ARCHITECTURE.md)) drives a body that is, in deployment,
  the robot server reached over HTTP.
- **The world model** ([world](../world/ARCHITECTURE.md)) is the body's
  sibling. In the simulator it shares one simulated house with the body.

Without it, every backend change (sim to car, a new sensor, a recorded walk)
would ripple into the brain, and a result measured in the sim would say
nothing about the car.

## Components and boundaries

```text
   brain / policies          robot server            safety layer
          |                       |                        |
          +-----------+-----------+------------------------+
                      |   the body contract (in-process)
     +-------+--------+--------+-----------+-------------+
     |       |        |        |           |             |
  sim body  remote  replay   teleop    hardware      wrappers:
  (grid     body    body     body      body          ROS drive,
  world)   (HTTP)  (photos) (phone +   (motor        mission
                             person)    board)        halt gate
                      ^
                      +-- the factory: the ONE place a backend is chosen
```

| Part | Owns | May not |
|---|---|---|
| **The contract** (`robot/interface.py`) | The method set, the units, the honest defaults, the driver ranks, the shared verb loop | Know about any backend, any transport, or ROS |
| **Sim body** | A body over the grid-world simulator: wheels, encoders, camera render, depth grid, lidar scan | Be imported by the brain; it is reached only through the contract |
| **Remote body** | The contract over the robot server's HTTP API | Import a backend, the simulator or the robot server |
| **Replay body** | A recorded walk played back one photograph per move | Claim any sensor or motion it does not have |
| **Teleop body** | A live phone camera; a person is the motor | Claim any sensor or motion it does not have |
| **Hardware body** | The real motors over the motor board's serial line; sensors from attached drivers | Hand the serial port to anything else ([motor-board](../motor-board/ARCHITECTURE.md), D1) |
| **Wrappers** | Re-routing motion (through ROS) or gating it (after a mission ends) | Fall behind the contract: every read must reach the wrapped body |
| **The factory** (`robot/factory.py`) | Choosing one backend and wrapper from configuration | Be bypassed: nothing else constructs a backend for the robot server |

The serial protocol to the motor board belongs to the
[motor-board](../motor-board/ARCHITECTURE.md) domain, and the simulator's
internals to [simulator](../simulator/ARCHITECTURE.md). This domain
specifies only what those look like through the contract.

## Decisions

### One contract, chosen in one place

Everything above the body talks to the abstract contract, and one factory
picks the implementation from configuration. **Rejected:** letting the brain
import the simulator directly, or adding a hardware-specific code path
"just for the car". Either makes the swap a rewrite and lets a sim result
pass through code the car never runs. **Accepted cost:** every new sensor or
motion must be expressed in the contract before anything can use it. The
rule is enforced by a test, not by review (CLAUDE.md section 6; the
[engineering spec](../engineering/body/ENGINEERING.md) names it).

### Body state only; the world is a separate contract

The body answers egocentric questions: how far is that, what is ahead of me,
how far have I driven. Where the robot is on the house map is world state,
on a sibling contract. **Odometry is what the body says about itself; pose
is what the world says about the body.** **Rejected:** a pose method on the
body. A mapper's pose jumps on loop closure, and the body's odometry, by
contract, never does. A consumer that cannot tell the two apart will
eventually integrate a jump. The 360-degree scan is body state because it is
the robot's own reading; it was first placed under the world routes and
moved for this reason (`PLAN-ros-alignment.md` 3.6). History:
`PLAN-mapping.md` N1.

### Say "I cannot tell", never a fabricated reading

The sensing methods that most bodies lack (depth grid, odometry, wheel
state, scan) are not abstract. Each has a default that answers "unusable"
with no numbers. A depth zone has three outcomes: a range, nothing within
range, or unmeasurable. Nothing within range is a fact about the room.
Unmeasurable is the absence of a fact. **Rejected:** making them abstract,
which breaks every backend each time a sensor is added; and answering zeros,
which is a claim. A zero means "I have not moved" or "a wall is touching
me". History: `PLAN-microduck-transplants.md` M2.

One deliberate asymmetry: the single scalar distance has no way to say
"unusable". A photograph-driven body cannot move, so it answers a value far
beyond any threshold, and the safety layer is honestly inert there. A real
body with no range sensor attached answers zero. It can move, so it must
fail closed. The same reasoning, decided by the user on 2026-10-02, makes a
body that moves but cannot see astern refuse to back up
([safety](../safety/ARCHITECTURE.md), "A body that cannot see astern does
not reverse"). Whether a body "really moves" is read from the contract
itself: it reports its wheels.

### Pixels are part of the contract

Every body returns a decodable image and names its media type. Anything else
a backend adds to a frame (the sim's room label and simulated detections, a
replay's file name) is provenance. No policy on the hardware path may read
it. **Rejected:** the sim answering with grid facts instead of an image. The
sim's body would then differ from the camera's, and the contract would have
to change on hardware day (`PLAN-sim-hardening.md` 2.1, closed by S2).

### Wheel velocities are the motion primitive

The lowest motion is a standing left/right wheel angular velocity, with
per-wheel positions read back. **Rejected:** a body twist (linear plus
angular velocity). On the car the ROS differential-drive controller owns the
twist-to-wheels conversion and its chassis constants. A sim that took twists
would leave that conversion untested until hardware day
(`PLAN-ros-alignment.md` R0). A body with no motors refuses a velocity
outright. **Rejected:** ignoring it silently, because a controller that
believes it is driving is worse off than one told it cannot.

### Verbs keep their meaning; the body says what a verb means

The discrete verbs (forward one move, reverse, turn by an angle, pan the
camera, stop) stay, because every step budget, recorded walk and safety
threshold was measured against them. A body that can describe a verb as a
wheel command declares it, and the safety layer carries it out, re-checking
every control period. **Rejected:** each backend running its own verbs after
a single check at the start. A move allowed at 20 cm covered 30 cm and could
end up to 10 cm past a wall on the car (`PLAN-ros-alignment.md` 3.22).
Bodies that move nothing, or whose motion is guarded elsewhere (the remote
body, the ROS wrapper), declare no plan and keep their own verbs.

### The remote body is a full body, not a client library

Over HTTP the brain drives exactly the loop it drives in-process, and a test
pins that the same mission produces the same actions both ways
(`PLAN-brain-relocation.md` B0). Refusals come back typed by their
machine-readable reason, not their prose: a safety veto, a preemption, and
"the drive chain is gone" each need a different response from the caller. A
missing sensing route means "this server predates the route" and reads as
unusable. Any other error raises. **Rejected:** treating every error as an
absent sensor, which would let a broken sensor read as a missing one.

### Devices that feed a veto are reached through a body

Where those devices live is decided elsewhere, and this domain only records
what it means for the contract. The motor board's serial port belongs to
the hardware body ([motor-board](../motor-board/ARCHITECTURE.md), D1, with
the alternatives it rejected). Where the car's lidar driver lives is
decided by [ros](../ros/ARCHITECTURE.md) (D3). For the body, both mean the
same thing: anything a veto depends on reaches the safety layer through a
body in the robot process, so it keeps working while ROS is down. A new
sensor or actuator of that kind is added to the contract, never reached
around it.

### The contract is pinned by one suite run against every body

A single backend-agnostic conformance suite is the contract's definition,
and it is meant to run against every backend **and** every wrapper. The
wrappers are included because the mission halt gate once silently inherited
"unusable" for the depth grid while wrapping a body that had one
(`PLAN-sim-hardening.md` S1). Nothing else in the suite would have noticed.
Both wrappers are in it, the ROS drive wrapper since 2026-10-03.

## Contracts

| Between | Direction | Category | Ownership |
|---|---|---|---|
| Brain / policies -> body | caller -> body | in-process interface | The body owns its readings; the caller owns nothing on it |
| Robot server -> body | caller -> body | in-process interface | The robot server holds the only body instance in its process |
| Safety layer -> body | caller -> body | in-process interface | The standing wheel command is written only through safety-vetted paths |
| Brain service -> robot server (remote body) | client -> server | HTTP/JSON | The robot server is authoritative for every refusal |
| ROS drive wrapper -> ROS bridge | client -> container | HTTP/JSON | Motion goes through ROS; every read and the stop go to the wrapped body. Zeroing ROS's inputs after a stop is best-effort and never delays it |
| Hardware body -> motor board | host -> board | serial line, the board's own protocol | The hardware body is the board's only master ([motor-board](../motor-board/ARCHITECTURE.md)) |
| Teleop body <- phone | phone -> robot server -> body | HTTP/JSON push | The body holds only the latest frame |
| Sim body <-> world model | shared object | in-process | One simulated house, two views: the body (egocentric), the world (allocentric) |

## Failure modes and resilience targets

| Failure | What the body does | Target |
|---|---|---|
| Camera unavailable (a stale phone, no driver on the car) | Raises, and the robot server turns that into "service unavailable" with the real message | A missing frame is never a stale picture passed off as new |
| A teleop phone stops sending | The frame read fails immediately after a staleness window. It never blocks | A stalled phone ends the mission at its first read |
| Sensor absent | The honest "unusable" answer | No consumer ever reads an absent sensor as clear, as blocked, or as zero motion |
| Motor board reboots mid-run | The odometer jump is absorbed and the board is set up again | Odometry never decreases and never jumps |
| Someone calls stop during a verb | The shared verb loop ends at its next period | A stop ends any verb within one control period, and is never overwritten. **Not yet met in one window:** a stop landing between the loop's check and its next wheel command is overwritten, and the wheels run until the watchdog stops them (see Open questions) |
| Robot server unreachable | The remote body raises a transport error distinct from a refusal | "Nobody heard it" is never mistaken for "it was refused" |
| ROS chain dead, or hung, under ROS drive | The wrapper stops the wrapped body first and directly; telling ROS its inputs are zero happens afterwards and cannot hold the stop up. For a short hold after the stop, motion from ROS still in flight from the stopped verb is turned into a zero | A stop never depends on the container, dead or merely not answering |
| Stop while a nav2 goal is active | Not the body's: the body's stop zeroes the wheels; ending the goal is the robot server's, after the body has stopped | See [safety](../safety/ARCHITECTURE.md)'s failure row and "Who drives" |
| A wrapper falls behind the contract | The conformance suite fails, for the wrappers it covers | A wrapper reports exactly what it wraps |
| The car's serial port is missing or not permitted at start (a wrong device name, a user outside the serial group) | The hardware body cannot be built, so the robot server does not start | A misnamed or forbidden port fails loudly at start-up. There is never a body that silently drives nothing |
| The robot process dies or hangs while the wheels turn | Nothing in the process can act; the motor board's own heartbeat stops the wheels ([motor-board](../motor-board/ARCHITECTURE.md)) | The wheels stop with no host involvement. The board's interval is longer than the robot server's watchdog, so the server normally acts first |
| The serial link drops mid-run (USB unplugged, a loose cable) | Motion commands are refused, and a stop still returns. The body stops reporting its wheels and says the link failed. The board's heartbeat stops the wheels | The wheels stop within the heartbeat interval, and wheel state and odometry turn unusable within a quarter of a second. The health check names the failed link. Met (`PLAN-ros-alignment.md` 3.34) |
| Feedback stops while commands still reach the board (feedback switched off, firmware partly hung) | **No fresh feedback, no wheel motion.** Wheel state and odometry turn unusable, the body zeroes a standing command itself, a verb in progress ends, and new motion is refused by name. The rule is the body's, not the safety layer's: answering "unusable" alone would have let commands through unvetted, because the safety layer reads usable wheels as "this body really moves" | A frozen encoder reading counts as "cannot tell", never as a robot standing still; the host stops the wheels within 0.4 s, where only the heartbeat (which never fires while commands arrive) did before. A mission ends `failed`, naming the cause. Met (3.34) |
| A wheel stalls with feedback live (snagged on a rug or a cable, or pushing on something the scan cannot see) | A verb that is commanded to move but whose encoders stop advancing ends "stalled" after a short window, on the car as in the simulator. The direct-mode and ROS verb loops share one window. The mission counts a stalled move as a move not made | A stalled move ends within the window, with the wheels zeroed, and five in a row end the mission `blocked`. Met (`PLAN-ros-alignment.md` 3.34, 3.35). **The window is a placeholder** until the board's low-speed behaviour is measured on the car: too short, and a slow pivot that sticks for a moment would be called a stall |
| Skid-steer turns scrub | The wheels slide sideways on every turn, so the body turns less than its encoders say. One correction factor, effective over geometric track, belongs to the real chassis and is configuration. Direct-mode turns, the odometry heading, the track the body publishes, and the ROS controller all take it from one setting. The simulator never takes it, because its body does not scrub | Turns land within the settle band on the real floor. **Unmeasured:** the factor is 1.0 (no correction) until it is measured on the Rover. Setting it then is a one-number change for both the robot server and the ROS container (3.35) |
| A wheel slips on a straight (smooth floor, a rug edge) | The encoders count distance the body did not cover. The body cannot see this, because odometry is by contract its own account of itself | No consumer treats odometry as pose. The world's SLAM pose, which corrects for slip, is the "where am I" answer. That is the body/world split above, and it is accepted as designed |

## Open questions

- **The car's lidar driver** is placed by [ros](../ros/ARCHITECTURE.md)
  (D3) and not built. Until it is, the car's body has no scan; what that
  costs, and the build plan, are in the
  [engineering spec](../engineering/body/ENGINEERING.md), Known gaps.
- **Where the depth camera's driver lives**, and whether its floor band
  reaches the body as a second depth grid (`PLAN-ros-alignment.md` 6,
  question 8).
- **A per-sensor mounting offset on the grid.** Today one sensor-to-bumper
  offset serves both the grid and the scalar. A car with a deck lidar and a
  front depth sensor needs the offset to travel with each reading, as the
  field of view already does. Settled on hardware day, when both are fitted.
- **The stop race inside the shared verb loop** (`docs-review/SPEC-REVIEW.md`
  fix-list 7, not fixed). A stop that lands between the loop's check and its
  next wheel command is overwritten. Until the loop re-checks after
  commanding, the failure table's "never overwritten" is a target, not a
  fact. Mechanism and fix: the
  [engineering spec](../engineering/body/ENGINEERING.md), Known gaps.
- **Two numbers the car must supply** (3.35). The stall window is 0.6 s, a
  placeholder until the board's low-speed deadband is measured. The
  skid-steer scrub factor is 1.0 until the effective track is measured.
  Both mechanisms exist and are tested; only the values are owed.
- **Discrete camera pans.** The pan verbs are three positions. A pan-tilt
  head with a commanded angle may want a contract change; nothing needs it
  yet.
