---
kind: architecture
domain: ros
status: current
verified: 2026-10-02
---

# ROS -- architecture

The ros domain is the ROS 2 stack: wheel control, SLAM, the nav2 planner and
controller, a velocity multiplexer and a transform tree. All of it runs as
one isolated service behind an HTTP wall. Nothing else in the project knows
ROS exists. Read this before changing anything that crosses the wall, before
adding a route to the bridge, or before deciding whether a new piece of logic
belongs inside ROS. Packages, topics, launch, parameters and runbooks are in
the [engineering spec](../engineering/ros/ENGINEERING.md).

## Purpose

ROS supplies the metric layer the project would otherwise build by hand:

- **Wheel control** that turns a velocity into wheel speeds.
- **A map with loop closure**, so a lap of the house lines up with itself.
- **Planning around furniture** with the robot's real footprint.
- **One transform tree** that places every sensor on the chassis.

These packages already agree with each other on message types, units and
frames (REP-103 and REP-105). That integration is what was being bought, not
the algorithms (`PLAN-ros-alignment.md` section 0).

**Who depends on it.** The world domain's SLAM backend reads the map and pose
through the bridge. Under the ROS drive mode, the robot server sends every
movement through the multiplexer. The twin draws SLAM's map and nav2's goals.

**What it is not.** It is not the brain, it is not the safety layer, and it is
not the phone's API. If the container dies, those three keep working, and the
wheels stop.

## Components and boundaries

```text
  OUTSIDE (no ROS)                                 INSIDE the one container
  ----------------                                 ------------------------
  robot server --- verbs as twists ---------->  bridge (HTTP, the only door)
   (arbitration,                                    | per-driver inputs
    safety layer,                                   v
    watchdog)                                  velocity mux (person > autonomy)
       ^                                            v
       |                                       collision monitor (can only slow or stop)
       |                                            v
       |                                       diff-drive controller
       |                                            v
       +---- wheel velocity over HTTP -------  actuator plugin (ros2_control)
       |     (vetted again by the safety layer)
       +---- lidar scan over HTTP ---------->  bridge -> SLAM, nav2 costmaps
  world (SLAM backend) -- map/pose/goals --->  bridge -> SLAM, nav2
  brain <--- polled for status (read-only) --  bridge -> brain topics for ROS tools
                                               robot description -> transform tree
                                               read-only viewer port (no publish)
```

Four lines are not crossed.

1. **Nothing outside the container imports ROS.** A source scan enforces it.
   The scan was written before any ROS existed, because this wall degrades
   one reasonable shortcut at a time.
2. **ROS reaches the wheels only through the robot server.** The actuator
   plugin speaks HTTP to the robot server, in the simulator and on the car
   alike. The robot server's safety layer vets every wheel command, and its
   watchdog stops the wheels when the plugin falls silent.
3. **Frame conventions flip in exactly two places.** The bridge converts on
   the ROS side: the scan's beam order and "no return". The world's SLAM
   backend converts on the project side: pose, map and goals.
4. **The bridge exposes the project's vocabulary, never ROS's.** No route
   takes a topic, service, node or parameter name from a caller. Each route
   means one thing to the project.

## Decisions

### D1. ROS lives behind an HTTP wall, not throughout the project

**Decision.** ROS 2 runs as one isolated service that the rest of the system
reaches over HTTP. Of the ways to use a lidar surveyed in
`PLAN-onboard-perception.md` 3.3, the rejected ones are:

- **(a) A clearance ring only.** It cannot map. It survives as the safety
  layer's use of the scan.
- **(b) Hand-rolled scan-matching SLAM.** It has no loop closure, so a lap
  of the house does not line up with itself.
- **(c) Adopting ROS outright.** That "swallows the project": its own IPC,
  build system, node lifecycle and test model. The brain, the safety layer,
  the FastAPI control plane and the offline test suite would all have to
  live inside it.

**Trade-off.** One process boundary, and a few concepts (the duplicates
listed in D9) that must now be defined on both sides. The condition for
choosing this, that mapping becomes the point, was met on 2026-09-19
(`PLAN-mapping.md` section 0).

### D2. Adopt the full stack, and move the wall down to the wheels

**Decision.** The stack inside the wall is SLAM, nav2, ros2_control, the
velocity multiplexer and tf2.

**Rejected.** The first design was a minimal three-route wall: pose, map,
and "go to". Its go-to route was withdrawn. nav2's controller emits
velocities continuously, at about the control rate, and a three-route wall
has no velocity route. So the wall moved down to the hardware interface
(`PLAN-ros-alignment.md` section 0).

**No second simulator.** The project's own simulator is the hardware
underneath. A second one would be a second thing to disagree with the first.

### D3. Plumbing goes inside ROS; judgment and safety stay outside

**Decision** (decided 2026-10-02, `PLAN-ros-alignment.md` 1.1). ROS gets what
is generic to any mobile robot: control, mapping, planning and transforms.
These stay outside:

- the brain, its mission logic and the LLM calls;
- the safety layer and the watchdog, which must keep working when ROS hangs
  (and ROS did hang: the tf2 deadlock in 3.15);
- the phone's HTTP API.

**Refinement.** A sensor that feeds a veto stays outside, even though a
sensor driver is plumbing. The test is whether safety, or the direct-drive
fallback, needs the sensor while ROS is down. So these stay outside too:

- the motor board's serial port (owned by the
  [motor-board domain](../motor-board/ARCHITECTURE.md), D1);
- the car's lidar driver (open question 5, decided 2026-10-02). This domain
  owns that decision: the driver lives in the robot process, outside ROS,
  and the bridge republishes its scan into ROS like the simulator's;
- the depth camera's floor band;
- driver arbitration;
- the direct verb executor.

**Rejected:** Waveshare's own ROS nodes for the Rover, because they would
take the safety layer out of the path. The full reasoning is the motor-board
domain's ([D1](../motor-board/ARCHITECTURE.md)), which owns the port they
would hold.

### D4. The actuator plugin talks HTTP to the robot server, even on the car

**Decision.** The plugin that writes wheel speeds stays HTTP-only, in the
simulator and on the car.

**Rejected:** a hardware plugin that owns the motor board's serial port
directly, which was R9's original plan. It would have taken the safety layer
out of nav2's path. Who owns the port instead is the motor-board domain's
decision ([D1](../motor-board/ARCHITECTURE.md)); this domain owns only the
seam, which is that ROS's actuator writes the wheels through the robot
server and nowhere else.

**So the seam stays at the body interface.** The car swaps the body backend
underneath the robot server, and no hardware plugin is written
(`PLAN-ros-alignment.md` 3.16). Hardware day becomes a configuration change.

**Trade-off.** A wheel command goes plugin, HTTP, robot server, backend:
about 40-150 ms of jitter (3.13). Every velocity that comes from ROS pays it.

### D5. Two collision guards in series, with the safety layer last

**Decision.** nav2's collision monitor sits between the multiplexer and the
controller. The robot server's safety layer re-vets what reaches the wheels.
Each guard can only slow or stop a command, so they cannot disagree about
what is allowed: the more conservative one wins.

**Trade-off.** The tighter guard decides. Both are kept as insurance. Over
about 40 m of goals the safety layer never had to clamp a nav command, and
the collision monitor stopped one once (3.15, open question 3, closed).

**A collision guard never blocks turning away.** A guard that refuses every
command near an obstacle, turning away included, freezes the robot against
the obstacle. That was tried inside ROS and rejected after it froze the robot
against a door jamb (3.15). How the collision monitor is configured to keep
this is in the [engineering spec](../engineering/ros/ENGINEERING.md).

### D6. One writer to the wheels, ordered as people rank drivers

**Decision.** Under the ROS drive mode, only ROS's actuator may write the
wheels. People and programs reach the wheels through the robot server's
verb route, where driver arbitration decides who may drive. The
multiplexer's priorities mirror that order, with a person above every
autonomous source. A person's **non-zero** D-pad command cancels an active
nav2 goal inside ROS.

**A stop ends a goal.** Decided by the user 2026-10-02 as the safety
domain's rule ([safety architecture](../safety/ARCHITECTURE.md), "Who
drives"); built 2026-10-02. The robot server cancels any active goal after
it has zeroed the wheels, and never makes the stop wait on ROS; a person
re-sends a goal to resume. Rejected there: a stop that only pauses the goal
plus a separate cancel control. Before it was built a stop only paused a
goal: nothing cancelled it, so nav2 kept publishing and the wheels resumed
once the stop's hold ended.
The driver order itself, including that a nav2 goal is an autonomous driver, is
owned by the [safety architecture](../safety/ARCHITECTURE.md); this domain
only mirrors it inside ROS.

**Not yet true for every person.** Today only the twin's D-pad has a
multiplexer input. Other callers that rank as a person (a teleop operator,
or a caller that does not name itself) and the teleop driver are refused
under the ROS drive mode. Until every arbitrated driver has an input, "people
reach the wheels through the verb route" holds for the D-pad only (open
questions; `docs-review/REPORT.md` V10).

**The multiplexer's inputs.** The brain and nav2 share a rank inside it.
That tie never arises through the supported routes, because the robot
server admits only one of them at a time.

### D7. Verbs become velocity streams outside ROS, closed on the encoders

**Decision.** The phone and the brain speak verbs: "one move forward", "turn
45 degrees". Every step budget and safety threshold in the repo was measured
against verbs. Under the ROS drive mode, a verb becomes a stream of twists.
Each verb ends when the wheel encoders say it has covered its extent.

**Rejected:** timed, open-loop streams. Over HTTP a timed stream measures
the network: seven runs of one nominal 0.30 m move landed 0.267-0.316 m
(3.13).

**Where the executor lives.** It sits on the robot's side of the wall, so
the verb vocabulary never enters ROS.

### D8. ROS becomes the car's drive mode only after measured gates

**Decision** (3.24). ROS drives the car by default once four gates hold:

- **G1, a reliable chain:** 20 consecutive live runs.
- **G2, the same safety on either path:** the ground-truth sweep's bars.
- **G3, a real fallback.**
- **G4, the same suites on the Jetson.**

All four are met: G1-G3 in the simulator (2026-09-30), G4 on the Jetson
(2026-10-05, 3.33) -- judged on the fork firmware the Rover will be
flashed with; on the stock firmware the turn test fails about one run in
three (3.25).

**Rejected:** dropping the direct drive mode. The direct mode is kept, but
narrowed to two jobs:

- the simulator and test default, so the offline suite and the deployed twin
  run without Docker;
- the car's fallback, because a crashed ROS must not leave the wheels
  running.

**The fallback is for people only** (user decision, 2026-09-29). While ROS is
down, a person may drive through the direct path. Every autonomous command
is refused, and a running mission ends. Autonomy never continues on a path
without nav2 and SLAM that nobody chose.

### D9. The wall's costs are counted and budgeted

**Decision** (3.17). Every concept defined on both sides of the wall sits in
a registry, with a check that its two copies agree. Examples are the wheel
radius, the driver order and the footprint. Two budgets are kept: the number
of duplicated concepts and the number of bridge routes. Raising either is a
decision made in a commit that says why. Generic pass-through routes are
banned outright, because they are how the wall becomes "a thin copy of ROS".

**Trade-off.** New capability on the wall costs a deliberate budget change.

### D10. One container, ROS 2 Humble, Cyclone DDS

**Humble.** The car's JetPack 6 is Ubuntu 22.04, Humble's platform. Jazzy
would have meant JetPack 7.

**One container.** All the nodes run in one container, so the middleware's
discovery stays inside it. Nothing multicasts across Docker's network.

**Cyclone DDS over the default Fast DDS.** Nav2 recommends it for Humble. It
replaced Fast DDS during R6's freezes.

**Distro defects are fixed by building from source, not by working around
them.** Where the distro's packaged release has a defect that breaks the
stack, the fixed upstream source is built into the image. Which components,
at which versions, and why, are in the
[engineering spec](../engineering/ros/ENGINEERING.md) (3.15).

**Trade-off.** Image builds are long, and each source build is temporary: it
is dropped when the distro's packages catch up.

### D11. ROS tools see the brain through a read-only window

**Decision.** The bridge polls the brain's status and republishes it as ROS
topics, so a recording or a viewer shows the mission beside the scans. The
brain stays unchanged and imports nothing of ROS.

**The viewer port is read-only.** Its default capabilities let a client
publish, call services and set parameters. That would be a second door to
the wheels, one that skips arbitration (3.17).

## Contracts

| Neighbour | Direction | Protocol | Ownership |
|---|---|---|---|
| Robot server, wheels | ROS's actuator plugin calls the server | HTTP/JSON: post a standing wheel velocity, read wheel state | The robot server owns the wheels, the veto and the watchdog. ROS is one driver among several. The plugin's steady posts are also ROS's heartbeat. |
| Robot server, scan and truth | The bridge polls the server | HTTP/JSON | The body owns the scan. The bridge converts it to ROS's convention. The simulator's truth only anchors SLAM's frame and is never published into ROS. |
| Robot server, verbs | The server's ROS drive mode calls the bridge | HTTP/JSON: a twist for a named driver | The robot server decides who may drive. The bridge only maps a driver to a multiplexer input. |
| World (SLAM backend) | It calls the bridge | HTTP/JSON in ROS's frame, tagged with a session | ROS owns the estimate, the grid and the goal's state. The world owns the conversion. See [world](../world/ARCHITECTURE.md). |
| Brain | The bridge polls the brain | HTTP/JSON, read-only | The brain owns the mission. ROS only displays it. |
| Motor board | None | -- | Never touched by ROS. It belongs to the body backend: see [motor-board](../motor-board/ARCHITECTURE.md). |

## Failure modes and resilience targets

| Failure | What happens | Target (bar it was accepted against) |
|---|---|---|
| Velocity commands stop arriving inside ROS | The multiplexer's input times out, then the controller's command times out | Wheels at zero within 0.5 s (R4, 3.13) |
| The container dies | The plugin's posts stop. The robot server's watchdog stops the wheels; after a silence it counts ROS as down | A running mission ends `failed`, naming ROS, within 3 s. A person can drive within 2 s, vetted by the safety layer. Autonomy is refused (G3, measured 2.04 s and 0.36 s) |
| The bridge dies while the plugin keeps posting | The first send that finds it dead (unreachable, or a server error) is refused as ROS-unavailable and marks ROS down, so the row above applies until the bridge answers again. The plugin's wheel commands are refused meanwhile, so a person on the fallback is the only writer. A bridge that answers with a client error is alive and marks nothing | Met (built 2026-10-02; the actuator's posts and client errors settled 2026-10-03, `docs-review/SPEC-REVIEW-3.md` V7-V8). The rule is [safety](../safety/ARCHITECTURE.md)'s ("When ROS dies, only a person drives") |
| The container comes back | The plugin's posts resume | ROS counts as up again within 5 s of its first post, with no server restart (G3.4) |
| The robot server restarts | While the server is unreachable the plugin keeps retrying and reports the outage; it does not declare a hardware error for that, because ros2_control would then deactivate it for good. If the restarted server answers "no wheels" before its body reports them, which the car's backend does until the board's first feedback, the plugin takes that as a hardware error (next row) | The ROS chain recovers by itself, but only when the restarted server reports wheels at its first answer (the simulator does). On the car's backend it is not met: see Open questions. UNCONFIRMED, by reading |
| The robot server reports no wheels | The plugin reports a hardware error, and ROS's control framework deactivates it until the container restarts | It fails loudly in the container's log. Not met: the controllers can still read "active" while commanding nothing, and it does not recover by itself (open questions) |
| The bridge is unreachable or hangs during a stop | The robot server stops the robot directly, without waiting on ROS | A stop never waits on the container, and the stopped verb's commands still in flight inside ROS cannot undo it. A person's stop ends a nav2 goal too, off the stop's own path: the robot server keeps cancelling, and holds the actuator's wheel commands at zero, until nav2 reports the goal over -- through a hung or failed cancel, and a goal nav2 had not yet accepted (the bridge cancels it on acceptance). Met (D6; [safety](../safety/ARCHITECTURE.md) "Who drives"; completed 2026-10-03). The bridge's acceptance-time cancel is unverified live |
| The bridge is unreachable or hangs during a verb | The verb waits out its client timeout, then the robot server stops the robot directly and refuses the verb by name | Refused by name, robot stopped. Only the stop avoids the wait |
| An obstacle near the chassis | The collision monitor slows or stops, then the safety layer re-vets | Stops at or beyond the safety layer's line on every path (R4: at least 19.4 cm; G2: travel-to-contact at least 18 cm after every move, no contact) |
| A goal nav2 cannot reach | nav2 aborts, and the wheels are at zero | Aborted and stopped within 60 s (R6: 19-24 s) |
| A person taps during a goal | The goal is cancelled inside ROS | Cancelled within 1 s (R6: 0.04-0.05 s) |
| tf2 or a transform goes stale | Prevented by the distro defect fix (D10) | Not a runtime guard: the symptom is a goal "reached" instantly |

## Open questions

All open questions here come from `PLAN-ros-alignment.md` section 6, except
where noted.

- **Every person, not just the D-pad, under the ROS drive mode** (D6;
  `docs-review/REPORT.md` V10). Either give every manual-rank driver a
  multiplexer input, or narrow D6 for good. The user decides.
- **Start-up order and the actuator's "no wheels" error.** The plugin treats
  a robot server that answers "no wheels" as a hardware error, and on the
  car's backend that is what the server says until the board's first
  feedback frame. A container started too early, or one running when the
  robot server restarts, is then deactivated for good. The runbook orders
  the start-up today; nothing orders a server restart. Whether the plugin
  should treat "no wheels yet" like an unreachable server is open.
- **Sighting geometry inside ROS.** A route would turn a camera bearing plus
  the lidar's range into a point on the map, composed through the transform
  tree. It is decided in principle (1.1), has no criteria yet, and would
  raise the bridge's route budget.
- **Saving and reloading the map**, keeping mapping on top of the reloaded
  map (question 6).
- **Whether the tiered policy steers by verbs or by nav2 goals** (question 2).
- **Smaller decisions:**
  - Rename the service directory now that it holds nav2 (question 1).
  - Simulated time versus wall time (question 4).
  - A localisation filter fusing the gyro, only if question 10's trigger
    fires on the car.
- **Perception moves inside only on a measured trigger.** The trigger is the
  Jetson measuring the perception tier over its budget, with GPU
  preprocessing as the fix (1.1).
