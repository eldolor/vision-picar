---
kind: architecture
domain: world
status: current
verified: 2026-10-02
---

# World -- architecture

The world domain answers what is true about the house and where the robot is
in it: the robot's pose on a map, the map itself, and (in the simulator only)
the ground truth an estimate can be checked against. It is the allocentric
half of the robot's picture of itself. The body domain is the egocentric
half. Read this to learn where a new piece of state belongs, what a pose or a
map promises its consumers, and how SLAM's answers reach the brain without
the brain learning that ROS exists. The modules, routes, payloads and
constants are in the
[engineering spec](../engineering/world/ENGINEERING.md).

## Purpose

Mapping the house while searching it is the project's next capability
(`PLAN-mapping.md`). That needs a place for state that is about the house
rather than about the body: a map that outlives a mission, and a pose that
can jump when SLAM recognises a room it has seen before. Without a separate
world contract, three things go wrong:

- The pose ends up on the body interface, because "where am I" feels like
  body state.
- Odometry and SLAM get averaged into one number, which hides the
  disagreement between them that is worth seeing.
- Whatever map implementation exists first (today, ROS's) gets to define the
  shape every consumer reads.

Who depends on it:

- The robot server passes world answers through to HTTP.
- The brain reads them over HTTP.
- The twin draws the map, the pose, the error readout and nav2 goals.
- The proposed frontier search would read the map and send goals
  (`PLAN-ros-alignment.md` section 6, question 6).

## Components and boundaries

```text
  brain (control/)  ---HTTP--->  robot server  /world/*  (pass-through; derives only the sim-only error readout)
  twin (web page)   ---HTTP--->        |
                                       v
                              world contract  (pose, map, truth; honest "unusable" defaults)
                                       |
            +--------------------------+---------------------------+
            |                          |                           |
       no mapper                 simulator world              SLAM world
   (says it knows nothing)   (map discovered by a ring     (HTTP client of the ROS
                              cast over the sim's house)    container's bridge; converts
                                                             ROS's frame on its own side;
                                                             also carries nav2 goals)
```

- **The contract** defines pose, map and truth, each with a default that
  says "I cannot tell you." It holds no ROS concept: no TF frames, no
  quaternions, no covariance, no message types. The contract is the
  project's, and a ROS backend converts on its own side.
- **The backend chooser** is the only code allowed to know which backend
  exists. It is the world-side twin of the body's chooser. `brain/` and
  `control/` never import a world backend.
- **The simulator world** wraps the same simulated house the body drives
  in. The body and the world are two views of one simulated house, never two
  copies.
- **The SLAM world** is the only consumer of the ROS container's map and
  pose. It reaches them over HTTP, imports nothing from ROS, and is the one
  place on the project's side where ROS's frame becomes the project's.
- **The HTTP client of the world** lives with the brain's other clients. It
  reads JSON and contains no hint of how the map was made.
- **Mission memory is not world state.** Rooms searched and sightings made
  belong to one mission and die with it. The world holds what outlives every
  mission (`PLAN-mapping.md` section 4, "The other axis").

## Decisions

### D1. Body and world are separate interfaces, split by frame

**Decision.** If an answer is in the robot's own frame, it is body state.
That covers distance, the depth grid, odometry, the wheel state and the
lidar scan. If it is in the house's frame, it is world state: pose, map and
truth. In short: *odometry is what the body says about itself; pose is what
the world says about the body.*

**Rejected:**

- Putting pose on the body interface beside odometry.
- Fusing the two into one estimate.

**Trade-off.** Odometry never jumps by contract. A mapper's pose jumps on
loop closure. Keeping them on separate interfaces keeps that disagreement
visible. The cost is a second interface and a second backend chooser. The
scan is body state even though maps are built from it, so it is served by
the body and not under the world's routes (`PLAN-ros-alignment.md` 3.6).
History: `PLAN-mapping.md` section 4, decided 2026-09-19.

### D2. Honest "unusable" answers, not abstract methods and not an origin

**Decision.** Every world method has a default meaning "no answer". A pose's
numbers are empty, not zero, and a map has no extent. A backend that cannot
map is silent, never broken.

**Rejected:**

- Abstract methods, which would have forced every body backend to grow a
  world.
- Reporting the origin when the pose is unknown. The origin is a valid pose,
  indistinguishable from a robot that really is at the origin, and it would
  put every map-less robot at the same believable spot.

**Trade-off.** Every consumer must check usability before it uses a number.

### D3. Occupancy is tri-state: free, occupied, unknown

**Decision.** A map cell is free, occupied or unknown. Unknown means "never
observed."

**Rejected:** a boolean grid. A planner that reads unknown as free drives
confidently through a wall it has not seen. One that reads unknown as
occupied refuses to explore. A third state forces the consumer to decide
which it wants. This is the depth grid's "unusable zone" argument, one level
up (`PLAN-mapping.md` section 4).

### D4. A pose names its map, and a map carries a version

**Decision.** Every pose names the map its coordinates belong to. The map
carries a version that is guaranteed to change whenever its cells change. It
may also change when they have not: a consumer that refetches on a new
version is never stale, only sometimes wasteful.

**Why.** A pose from yesterday's map is byte-identical to one from today's,
so without the map's name a stale pose would plot somewhere plausible and
wrong. The version lets a consumer poll the pose often and fetch the large
map only when it has changed.

**Rejected:** a bare (x, y). A restarted SLAM session is a new map, and the
SLAM world gives it a new name.

### D5. The project's frame convention, converted at the wall

**Decision.** The house frame has x pointing east and y pointing south.
Headings are compass bearings, clockwise-positive, with 0 pointing north.
Angles are in degrees and positions in metres, matching odometry. ROS uses
REP-103 instead: x forward, y left, yaw counter-clockwise. That conversion
happens inside the SLAM world, both directions side by side, and nowhere
else on the project's side. The bridge does the scan's conversion on the
ROS side (see the [ros architecture](../ros/ARCHITECTURE.md)).

**Rejected:** adopting REP-103 throughout. That would have changed every
angle already in the project: the camera pan, the depth grid's columns and
the steering bearings. It would also let ROS's types into the contract.

**Trade-off.** Two conversions must stay exact. They are the most likely
place for a mirrored map, which looks right at 0 and 180 degrees and wrong
at 90.

### D6. The simulator discovers its map; it does not copy the layout

**Decision.** The simulator knows the whole house. It still builds its map
from a 360-degree ring cast from wherever the robot stands, and everything
behind a wall stays unknown.

**Rejected:** returning the layout. It would be three lines of code, but it
would draw a finished house at mission start, which no mapper does, and
"unknown" would go untested in the one place it can be tested without
hardware.

**Trade-off.** A getter mutates: reading the map observes first. Nothing in
the mission loop knows when to tell a world to look, and the body may not
reach across to the world to tell it.

### D7. World state is served by the robot server as a pass-through

**Decision.** The world routes live on the robot server, which returns
exactly what the backend answered. The one thing it derives is the error
readout, from pose and truth, and that exists only in the simulator (D8).

**Rejected:** a separate world service with its own address. The usual
reason to split a service is that it should survive the other's restarts.
That does not apply here, because a map's durability is a storage decision,
not a process-topology one (`PLAN-mapping.md` section 4, "How the twin gets
world state").

### D8. Ground truth is a named, simulator-only answer that decides nothing

**Decision.** The simulator can report where the robot actually is. That
truth uses the pose's frame and units, so the two can be subtracted into an
error readout. Hardware answers "unusable", always.

**Rejected:** letting any policy read the truth. That would be navigating by
an oracle. The truth's only uses are the error readout and laying SLAM's
frame onto the house.

**How SLAM's frame is laid on the house.** It is aligned once, using the
truth recorded at the session's start (odometry zero). An alignment at the
consumer's first question was rejected after it was measured wrong: a twin
that first asked after eight moves folded SLAM's accumulated error into the
frame and read "0.0 cm" (`PLAN-ros-alignment.md` 3.14). After the alignment,
the truth is never read to produce a pose. It is a choice of frame, not a
correction.

### D9. Goals belong to the planning backend, and are arbitrated as drivers

**Decision.** Only a world that can plan accepts a goal. Today that is the
SLAM world, through nav2. Goals are not part of the world contract. Any
other world refuses a goal by name rather than ignoring it. A goal is
arbitrated like any other driver of the robot. Where it ranks, and what it
blocks, is the driver order, which the
[safety architecture](../safety/ARCHITECTURE.md) owns ("a navigation goal is
an autonomous driver too"). The world's part is only to say whether a goal
is in progress.

**A stop ends a goal.** Decided by the user 2026-10-02 as the safety
domain's rule ([safety architecture](../safety/ARCHITECTURE.md), "Who
drives"; see also the [ros architecture](../ros/ARCHITECTURE.md), D6);
built 2026-10-02. A person's stop zeroes the wheels and then ends the
goal -- held until the planner reports it over -- so the planner does not
resume driving afterwards; a person re-sends the goal to resume. The
brain's stop spares a goal (the safety architecture says why).

**Rejected:**

- A go-to route as part of the wall. It was withdrawn because nav2 drives
  by continuous velocity, so the wall moved down to the wheels
  (`PLAN-ros-alignment.md` section 0).
- Goals outside driver arbitration. That was the state before 3.23, and it
  let a goal interleave with a mission.

**Trade-off.** The robot server learns that goals exist. It asks the world
whether one is in progress, and learns nothing about missions.

### D10. The world backend is configured, and refuses a body it cannot share

**Decision.** The world backend is chosen from configuration, separately
from the body backend. That matters because a real robot with no mapper is
a legitimate combination. A simulator world on top of a body with no
simulated house is refused at start-up, by name.

**Rejected:** a map of a house the robot is not in. It would look entirely
plausible.

## Contracts

| Neighbour | Direction | Protocol | Ownership |
|---|---|---|---|
| Robot server | Server calls the world backend | In-process interface | The backend owns pose, map and truth. The server only passes them through, plus the error readout it derives from pose and truth. |
| Brain, twin | They call the robot server | HTTP/JSON, read-only for pose, map, truth and error | Consumers may cache. The world client does not, so the map version stays honest. |
| ROS container (SLAM, nav2) | The SLAM world calls the bridge | HTTP/JSON, in ROS's own frame, tagged with a session | ROS owns the estimate and the grid. The SLAM world owns the conversion and the anchor. See [ros](../ros/ARCHITECTURE.md). |
| Goals | Twin or a future brain policy, through the robot server, to the SLAM world, to nav2 | HTTP/JSON, in the house frame | nav2 owns the goal's state. The robot server owns the arbitration, by the driver order in [safety](../safety/ARCHITECTURE.md). |
| Body | None at run time | -- | Shares one simulated house in the simulator. Body and world never import each other. |
| Simulator | The simulator world reads the simulated house | In-process | The simulator owns the truth. |

## Failure modes and resilience targets

| Failure | What happens | Target |
|---|---|---|
| A server that predates the world routes | The client reads "not found" as "no map" | An old server is never mistaken for a broken one |
| A broken mapper (any other HTTP error) | The client raises its own error type, distinct from the body's transport error | A map outage reads as a degraded mission (the safety layer and lidar still work), never as a runaway robot |
| The ROS bridge is unreachable | The SLAM world answers "unusable" pose and map | No stale or invented pose is ever reported |
| The ROS container restarts | A new session means a new map name and a new anchor | A consumer can always tell yesterday's coordinates from today's |
| The goal state cannot be read during arbitration | It counts as "no goal in progress" (fail-open) | An outage never locks every autonomous driver out |
| A goal is sent, read or cancelled while the bridge is down | Today the route fails with a server error (see Open questions) | Target: refused by name, like a world that cannot plan. Not met |
| A simulator world on a body with no simulated house | Refused at start-up | Never a plausible map of the wrong house |
| Hardware | Truth and error answer "unusable" | Never a plausible number to be proud of |

These bars were set and measured as the SLAM world's acceptance
(`PLAN-ros-alignment.md` 3.14):

- **SLAM must correct what odometry cannot.** It did: odometry ended up to
  99 cm and 54 degrees off, while SLAM ended within 1-4.5 cm.
- **The map must be the house.** At least 90% of occupied cells must lie
  within 10 cm of a true surface. Every lap met it.
- **At-rest error within 5 cm and 2 degrees** with drift off: FAILED (position
  met on 2 of 9 laps, heading on 7 of 9).
- **At-rest error within 10 cm and 3 degrees** with drift on: position met on
  9 of 9 laps; heading FAILED (met on 4 of 9). Recorded as failed.

## Open questions

- **Keeping the map across sessions.** Saving it, reloading the pose graph
  and keeping mapping on top of it, and the S3 backup are all decided in
  principle and none is built (`PLAN-ros-alignment.md` section 6, question
  6). Open within that: where the robot starts on a reloaded map. The user
  decides, on the criteria written there.
- **A frontier search over this map.** It would read the map and send goals
  from the brain. It reopens whether the tiered policy steers by verbs or by
  goals (question 2).
- **Fetching only on a version change.** It needs a cheap route that returns
  the version alone. Deferred until a real house makes the map's size matter
  (`PLAN-mapping.md` section 4).
- **The at-rest error of about one map cell** with drift off: the 5 cm bar
  was met on only 2 of 9 laps (`PLAN-ros-alignment.md` 3.14). Its cause is
  not established.
- **Goals during a bridge outage.** The goal routes answer a server error
  instead of a named refusal. The fix is not scheduled; the
  [engineering spec](../engineering/world/ENGINEERING.md)'s Known gaps has
  the code reference.
