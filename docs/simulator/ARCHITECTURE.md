---
kind: architecture
domain: simulator
status: current
verified: 2026-10-02
---

# Simulator -- architecture

The simulator is a stand-in for the house and for the robot's body. Every
decision-making, safety and ROS feature in this project was built and
measured against it before any hardware existed. This document says what
the simulator is responsible for, what it deliberately is not, and the
decisions that shaped it. Read it before you trust a result produced in
simulation, or before you add a realism feature. The modules, constants
and commands are in the
[engineering spec](../engineering/simulator/ENGINEERING.md).

## Purpose

The project is simulation-first, and since 2026-09-25 it is also
data-first. A phase is done when acceptance metrics, written down before the
run, are measured through the real mission path and pinned in a test
(`CLAUDE.md` section 7). The simulator is what makes that possible without a
car. It supplies three things:

- **A body.** A backend of the robot's body contract (see
  docs/body/ARCHITECTURE.md) that drives, turns, pans a camera and senses.
  The brain, the robot server, the safety layer and the ROS container talk
  to it through the same interfaces they will use on hardware. A mission
  that succeeds here has exercised the code path the car will run.
- **A house.** Walls, doors, rooms and solid objects, including objects
  that move. The world domain's sim backend maps this house by casting
  rays from wherever the robot stands (docs/world/ARCHITECTURE.md).
- **Ground truth.** Where the robot really is and what is really there.
  Acceptance sweeps judge safety and arrival against this truth. No
  decision may ever read it.

Without the simulator, no phase from R0 onward could have been closed. The
safety fixes of 3.18-3.19 were found and proved here, and so were nav2's
results on a scaled house. The motor-board fake that lets the real serial
backend run with no board also needs a sim body to turn.

## Components and boundaries

```text
            body contract (in-process)          world contract (in-process)
                     |                                    |
        +------------v-------------+         +-------------v-----------+
        |  simulated body          |         |  world domain's sim map |
        |  wheel-velocity primitive|         |  (owned by world)       |
        |  verbs, camera, depth,   |         +-------------+-----------+
        |  scan, distance, odometry|                       |
        +------------+-------------+                       |
                     | reads and moves                     | reads
        +------------v-----------------------------------v-+
        |  the world: house layout, continuous pose,        |
        |  solid objects, movers, sim clock                 |
        +------------+--------------------------------------+
                     | one geometry
        +------------v-------------+     +----------------------------+
        |  the raycaster           |     |  map library (named houses,|
        |  picture, collision,     |     |  mover scenarios)          |
        |  ranges, visible extent  |     +----------------------------+
        +--------------------------+
```

| Part | Owns | May not |
|---|---|---|
| The world | The house layout, the robot's continuous pose, the objects and their positions, the movers, and the sim clock | Decide anything. It moves the robot only as far as the body asks and the geometry allows |
| The raycaster | The single geometry that draws the camera frame, stops a move, and answers every range query | Keep a second collision model. Two models are how a robot drives through a wall it can see |
| The simulated body | Turning wheel commands into motion, the discrete verbs as wrappers over that one path, and every sensor reading | Bypass the safety layer for a guarded verb, or report what the body achieved as what it was commanded |
| Sensor noise model | Opt-in noise, dropout and range limits on distance readings | Be on by default |
| Movers | People and pets as solid objects that walk a closed path on the sim clock | Ever move closer to the robot than the keep-out allows |
| Map library | The named houses and their mover scenarios | Be chosen by decision code. The backend factory picks the house a body stands in; judges (sweeps and live suites) pick their own |

Three boundaries carry the weight:

1. **Nothing in the brain or the control service imports the simulator.**
   The backend factory is the only place that picks it. Tests enforce this
   (`CLAUDE.md` section 2). A code path that reaches the simulator directly
   would make the hardware swap a rewrite.
2. **The simulator never runs a real detector on its own pictures.** Its
   frames are flat-shaded renders. It reports synthesised detections, taken
   from the same geometry the picture is drawn from and marked as
   synthesised (`PLAN-onboard-perception.md` 1.12).
3. **Ground truth is read only by judges.** The truth readouts and the
   object listing exist for sweeps, live tests and the twin's error
   display. No policy or safety check reads them.

The fake motor board (motor-board domain) and the world domain's map
backend are built on this simulator, and each is owned by its own domain.

## Decisions

### A grid-world raycaster, and no second simulator

**Decision.** The house is a grid of cells (wall, floor, door) with a 2D
raycaster. A physics simulator was not adopted, and Gazebo was
rejected again when ROS 2 arrived.

**Alternatives rejected.** Gazebo and similar physics simulators. The
ROS plan rejected Gazebo because "a second one is a second thing to
disagree with the first" (`PLAN-ros-alignment.md` section 1). The ROS
container instead drives this simulator through the same robot-server
wheel route the car will use.

**Trade-off.** It is fast, deterministic and in-process, so thousands of
missions run in a sweep. Walls are axis-aligned, the world is 2D (every
object fills the lidar's plane), and nothing has mass, friction or slip
(`PLAN-ros-alignment.md` section 4).

### One geometry for the picture, collision and every range

**Decision.** Collision, the distance reading, the depth grid, the lidar
scan, the discovered map and synthesised detections all cast rays through
the same geometry that draws the camera frame.

**Alternative rejected.** A separate collision model for the mover. A
robot could then drive through a wall the camera shows.

**Trade-off.** Any change to the ray geometry moves every sensor at once.
That is why the picture is pinned by a golden image and the ranges by
sweeps. One deliberate exception: the scan the safety layer reads is cast
more exactly than the picture and the map, which keep the cast their tests
are pinned to (`PLAN-ros-alignment.md` 3.18). Both still read the same
house.

### Continuous pose and wheel velocities as the primitive (R0)

**Decision.** The pose is continuous (position and heading as real
numbers). The body's primitive is a left and right wheel angular velocity,
integrated with differential-drive kinematics. The discrete verbs are thin
wrappers over it, and they still mean one cell forward or the angle asked.

**Alternatives rejected.** Integer cells and cardinal headings, because
nav2 cannot drive them and a target off a cardinal direction could never be
centred. A body twist as the primitive was also rejected: on the car,
`diff_drive_controller` owns the twist-to-wheels conversion, so a sim that
took a twist would leave that conversion and its chassis constants
untested until hardware day (`PLAN-ros-alignment.md` 3.1).

**Trade-off.** The verbs keep their old extent, so every step budget,
recorded demo and safety threshold stays comparable. The verb speed is
chosen to keep the verbs' old timing, not taken from the motor's rated
speed.

### Synthesised detections, not a detector on renders (1.12)

**Decision.** The sim reports detections from geometry. A detection is
reported at the bearing of the part of the object the camera can see, and
not at all when the object is hidden (3.32).

**Alternatives rejected.** The sim taking no part in perception testing.
Making the renderer realistic enough for a real detector.

**Trade-off.** The sim tests the consumers of perception: triggers,
steering, arrival and arbitration. It never tests the detector. Any claim
about detection accuracy must come from real frames.

### Objects are solid to everything that senses or moves (3.9)

**Decision.** Decided by the user on 2026-09-26: "objects must be treated
as solid to emulate the real world". The camera still draws objects as
billboards.

**Alternative rejected.** Objects as labels on floor cells. The robot drove
onto its target and then searched for it.

**Trade-off.** No measurable cost to arrival. The 3.9 re-measurement read
98.3% at both 90% and 80% detection, against 98.6% and 98.4% before.

### Realism is opt-in and off by default

**Decision.** Real time, sensor noise and dropout (S4, S5), encoder drift
(R5) and movers (3.30) are each switched on explicitly. With all of them
off, the simulator is exact and instantaneous.

**Alternative rejected.** Realism on by default. Every pinned trace and
step count would then be a statistical statement, and the unit suite would
pay for sleeps it does not need.

**Trade-off.** A default run is flatter than reality. A phase that depends
on noise or drift must say so in its criteria and switch it on.

### Movers hop on a sim clock and never close the gap (3.30)

**Decision.** A mover is a solid object that hops cell to cell around a
closed path on the simulator's own clock. It never hops closer to the robot
than a keep-out beyond the chassis' turning circle. It may step past at the
same distance, or move away.

**Alternatives rejected.** Continuous movers, which would need a moving
disc in the ray caster. The first keep-out rule, "wait whenever the next
cell is inside the keep-out", deadlocked a person against a waiting robot
in a doorway.

**Trade-off.** Responsibility is clean: the robot can only come closer than
the stop line by its own motion, which is what the ground-truth metric
measures. The cost is that the sim cannot show a pet darting at the robot.

### Houses at real proportions are added, not substituted

**Decision.** The starter house, with doors one cell wide, stays the
default that most tests were measured on. A scaled house with 0.90 m doors (R6) and
the user's own first floor (from the appraisal sketch) are added beside it.

**Alternative rejected.** Rescaling the starter house, which would have
re-tuned every test written against it.

**Trade-off.** Results must name their house. nav2, the R1 sweeps and the
live ROS chain suite are judged in the scaled house, because the starter
house's doors leave the UGV Rover too little margin
(`PLAN-ros-alignment.md` 3.21, 3.24; the numbers are in the engineering
spec).

### The renderer is Python, and it is the vision policy's real input (S2)

**Decision.** The raycaster was ported from the twin's JavaScript into
Python, so the sim body returns a real image like every other backend.
The JavaScript copy was deleted on 2026-09-25.

**Trade-off.** The render is the model's input on this backend, so it is
lit like a room, not themed like the UI. Whether that makes a vision run
discriminate between models is unmeasured (`sim/renderer.py`'s fidelity
note).

## Contracts

| Neighbour | Direction | Category | Ownership |
|---|---|---|---|
| Body contract (docs/body/ARCHITECTURE.md) | Robot server and in-process missions call the simulated body | In-process interface | The body contract is owned by the body domain. The simulator implements it in full, including wheel state and scan |
| World domain's sim backend | Reads the simulator's world | In-process, shared state | The simulator owns the world state. The world backend only reads it and builds its discovered map |
| Robot server (control-api) | Calls the body. Its wheel loop advances sim time, and its idle ticks let movers walk while the robot waits. Publishes the name of the house that was built | In-process | The server owns time pacing. The simulator owns what happens in that time, and a built house carries its own name |
| Sim-only routes (object listing, moving furniture) | The twin and tests call the robot server, which calls the world | HTTP/JSON at the server, in-process here | Served by control-api. A backend with no house answers "not implemented". Ground truth: no decision may read it |
| ROS container (ros domain) | Its hardware plugin posts wheel velocities to the robot server, which integrates them here | HTTP at the wall, then in-process | ROS never touches the simulator directly |
| Fake motor board (motor-board domain) | Wraps a sim body and turns its wheels from the serial protocol | In-process | Owned by motor-board |
| Acceptance sweeps and live tests | Read ground truth and the house name | In-process or HTTP | Judges only |

## Failure modes and resilience targets

| Failure | What the simulator does | Target (the commitment) |
|---|---|---|
| A sim result is read as a statement about real rooms | The renderer carries a fidelity note. Detections are marked synthesised. The plans list what the sim cannot show (`PLAN-sim-hardening.md` section 7, `PLAN-ros-alignment.md` section 4) | No claim about detector accuracy, VLM accuracy on photographs, slip, lidar on glass, or stopping distance is ever made from sim data |
| The robot's body ends up inside a wall or object | Moves are checked against the chassis outline along the way, not only at the end. Pivots stop where a corner would touch | 0 contacts and 0 penetration in the ground-truth sweeps of 3.18, 3.19 and 3.30 |
| The safety layer lets the robot get too close | Not the simulator's job to prevent. Its job is to measure it honestly on ground truth | Sweeps judge travel-to-contact on truth, never on the readings the veto uses. Accepted bar: 0 runs under 18 cm (3.18, 3.30) |
| The picture changes silently | A golden image pins the picture | Any render change fails the suite until re-blessed after a person looks at it |
| A realism feature changes default behaviour | Every feature is off by default | With features off, pinned traces and output are identical to the pre-change code (3.30 criterion 4) |
| A sensor read sees half-updated objects | Object updates are atomic to readers | Concurrent readers see either the old table or the new one |
| A mover and a waiting robot block each other | The mover may step past at the same distance | No deadlock in the 3.30 nav2 runs |
| A live test runs in the wrong house | The robot server reports the house that was actually built, for every body standing in a sim house (the fake motor board included), never what the environment says now | A live suite refuses to judge a lap in a house it was not written for |
| An unknown house or mover scenario is named | Start-up refuses with the valid names | Never a silent fallback to another house |

## Open questions

- **Low obstacles.** The world is 2D, so every object fills the lidar's
  plane. Testing a floor band from a depth camera needs objects with a
  height (`PLAN-ros-alignment.md` section 6, question 8). The user decides
  after the criteria there are confirmed.
- **Wheel slip.** Not modelled, because the coefficient is unknown. It
  lands at R8 calibration on the car.
- **Sim time vs ROS time.** The sim uses wall-clock pacing. A ROS clock
  publisher would make regression runs deterministic and is not needed yet
  (question 4).
- **The home's interior.** The outside walls are measured. The interior
  walls, doorways and the staircase are inferred and marked provisional
  until the user corrects them.
- **Does the lit renderer help a vision run?** Unmeasured until the next
  paid closed-loop run (`CLAUDE.md` section 5, Stage 1).
