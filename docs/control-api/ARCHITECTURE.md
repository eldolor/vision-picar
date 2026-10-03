---
kind: architecture
domain: control-api
status: current
verified: 2026-10-02
---

# Control API (the robot server) -- architecture

The robot server is the robot's runtime process and its one network
surface. The brain, the web twin, the ROS container's bridge, the phone on
a teleop walk and the health command all reach the robot through it, over
HTTP. Inside it sit the one body, the world model, the safety layer, the
driver arbitration, the watchdog and the control loop that keeps a standing
wheel command vetted. Read this for what the server is responsible for and
why it is a separate process behind plain HTTP. The routes, payloads,
status codes and environment are in the
[engineering spec](../engineering/control-api/ENGINEERING.md).

## Purpose

Two things must hold whoever is driving: every motion is vetted on the
robot, and the motors stop when the driver goes quiet. The robot server is
where both happen. It also lets every client be written once. The brain
drives the server the same way on a laptop, on the Jetson, or through a
tunnel from a phone; only the address changes.

Who depends on it:

- **The brain service** ([mission](../mission/ARCHITECTURE.md)), through the
  remote [body](../body/ARCHITECTURE.md) and remote
  [world](../world/ARCHITECTURE.md) clients.
- **The twin** ([twin](../twin/ARCHITECTURE.md)), which drives the D-pad,
  draws the sensors and the map, and is served from here when run locally.
- **The ROS container** ([ros](../ros/ARCHITECTURE.md)): its actuator plugin
  reads and writes the wheels here, and its bridge reads the scan.
- **The health command and the tunnel**
  ([operations](../operations/ARCHITECTURE.md)).

## Components and boundaries

```text
            brain      twin/phone     ROS bridge     health command
               \           |              |              /
                +----------+---- HTTP/JSON+-------------+
                                 |
   +--------------------------- robot server ---------------------------+
   | access gate (shared secret)     CORS                                |
   | motion routes ----> arbitration ----> safety vet ----> body          |
   | body sensing routes -------------------------------> body          |
   | world routes (pass-through) --------------------> world model       |
   | teleop ingress, sim-only routes, health, the twin's static files     |
   | control loops: watchdog (silence -> stop), wheel loop (re-vet)      |
   +--------------------------------------------------------------------+
```

| Part | Owns | May not |
|---|---|---|
| Motion routes (verbs, stop, standing wheel command) | Entry for anything that moves the robot | Move anything without arbitration and the vet. Three things skip arbitration only: a stop; under ROS drive, the ROS actuator's standing commands, which are still vetted; and a zero standing command, which can only stop the wheels and only for the driver holding them |
| Body sensing routes | One route per sensor, answering exactly what the body said | Compute a reading of their own |
| World routes | The house map, pose, goals and (in the sim) ground truth, under their own namespace | Compute world state. They pass through to the world model. The one exception is the sim-only error readout, which compares two of the world model's own answers (estimate against truth) and is never an input to a decision |
| Teleop ingress | Accepting a phone's frame for a teleop body | Accept a frame no body can read |
| Sim-only routes | Inspecting and rearranging the simulated house | Exist silently off the simulator |
| Control loops | The watchdog and the wheel loop | Block behind a motion in progress (that would freeze every route, stop included) |
| Health and identity | Facts about this process and the robot | Decide a verdict. That belongs to the health command |

The server never imports the brain (`control/`), never imports ROS, and
never names a backend type. It discovers what a body can do by asking, and
answers "unsupported" by name when it cannot.

## Decisions

### The robot server and the brain are two processes

This domain owns the process topology; what a silent driver means, and the
order in which drivers win, belong to [safety](../safety/ARCHITECTURE.md).
Even on one board. **Rejected:** running the mission loop as a task inside
this server. It is less code. But a synchronous block in the brain would
freeze the event loop the watchdog runs on, it merges the robot runtime with
the decision-maker, and it loses the property that moving the brain is an
address change. The cost is one local HTTP round trip per call, against a
loop that waits seconds on vision (`PLAN-brain-relocation.md`, "Why not one
process").

### Plain HTTP/JSON is the wall to everything, ROS included

The ROS stack reaches the wheels the way every other client does: over HTTP,
through arbitration and the vet. **Rejected:** ROS inside this process, or a
ROS plugin owning the wheels. Either takes the safety layer out of the
navigation path, and either makes this process depend on ROS being healthy
(`PLAN-mapping.md` section 4, "(b+)"; `PLAN-ros-alignment.md` 3.15-3.16). A
containment test forbids ROS imports outside the container.

### Backend-agnostic by construction

The server holds whatever body the factory chose and never checks its type.
Optional abilities (accepting pushed frames, having a simulated house,
taking navigation goals) are discovered by asking. When an ability is
missing, the route says so with an error that names it. **Rejected:**
type checks per backend, which would make the hardware swap edit this file.
**Rejected:** silent no-ops, which read as success.

### Body and world are separate namespaces

Every body route answers "about me". The world routes live under their own
prefix and answer "about the house". The prefix is the seam, not decoration:
a reader who has to ask which kind a route is has already lost the
distinction (`PLAN-mapping.md` N1). The world routes are pass-throughs in
this process. **Rejected:** a separate world service with its own address.
The map's durability is a storage decision, not a process-topology one
(`PLAN-mapping.md` section 4).

### One route per sensor

The depth grid, odometry, wheel state, scan and camera each have their own
route. **Rejected:** folding depth into the camera frame. A wedged camera
must not take the clearance reading down with it (M2, M9).

### The server publishes the safety layer's own reductions

Which depth zones are "the path", what they read, whether a forward move
would be vetoed and the stopping floor in force are published by this
server. **Rejected:** letting the twin recompute them. Safety logic
duplicated in a browser drifts from the real one (CLAUDE.md section 6).

### A shared secret is the access control; CORS is permissive

Once reachable from the internet, motion and sensing routes require a
shared-secret header. The page itself and the health route stay open: a
monitoring probe must be able to ask whether the robot is alive without
holding the secret, and a person must load the page before typing the
secret into it. (The health route was first left open for a cloud load
balancer's health check, which cannot send headers. That load balancer was
deleted on 2026-09-05; the reason survives it for any external probe.) **Rejected:** CORS as access control.
It constrains browsers, not attackers. Unset, the gate is inert, which is
how local development and the test suite run.

### Old and new coexist on the wire

A client asking a server that predates a route gets "not found", and the
client reads that as "this server has no such sensor". Every other error is
a real error. Services are redeployed one at a time, so a mixed fleet is a
normal state, not an edge case.

### The control loops live with the motors

The watchdog and the wheel loop run in this process, beside the motors they
guard, not in the brain. A standing wheel command is re-vetted here as the
room changes, not trusted from the caller. A control loop never waits for a
motion in progress, because waiting once froze every route, the stop route
included (`PLAN-ros-alignment.md` 3.22). What silence means, and how long
it may last, is [safety](../safety/ARCHITECTURE.md)'s.

### Under ROS drive, the server stays the arbitration point

People and programs still drive through the verb route, where arbitration
decides between them. The standing-wheel route becomes the ROS actuator's
alone (`PLAN-ros-alignment.md` 3.13). A stop zeroes the robot directly
first; telling ROS its inputs are zero comes afterwards and cannot hold the
stop up. How the server decides ROS is down, and who may drive while it is,
are [safety](../safety/ARCHITECTURE.md)'s ("When ROS dies, only a person
drives"). **Today only the D-pad among the people has a path through ROS**:
other manual drivers are refused under ROS drive (see Open questions).

### Health reports facts; the verdict is elsewhere

The health route reports what this process knows: mode, drive, the
watchdog's own liveness, who holds the robot, the last refusal and which
build answered. Whether that adds up to "healthy" is decided by one health
command, which counts only conditions a release can be blamed for
(`PLAN-microduck-transplants.md` M5). Like every process, this one names
its build at start-up and on health ([operations](../operations/ARCHITECTURE.md),
"Every process says which build it is").

### No reset

There is no route that puts the robot back at its start. Real hardware has
none, and a sim that had one would invite tests the car cannot run.
Restarting the server is the reset.

## Contracts

| Neighbour | Direction | Category | Ownership |
|---|---|---|---|
| Brain service | brain -> server | HTTP/JSON, driver named in a header | The server is authoritative for every refusal. The brain owns its mission state; the server has no notion of a mission |
| Twin | browser -> server | HTTP/JSON polling; the static page from the same origin when local | The twin draws what the server publishes and computes no safety state |
| ROS actuator plugin | container -> server | HTTP/JSON: reads wheel state, writes regular standing wheel commands as driver `ros` | Under ROS drive, the only wheel writer. Its posts are the liveness signal |
| ROS bridge | container -> server | HTTP/JSON reads of the scan and, in the sim, ground truth | Read-only |
| World model | server -> world | in-process | The world model owns map, pose and goals. The server passes them through and arbitrates goals |
| Body | server -> body | in-process | One body per process |
| Phone (teleop) | phone -> server | HTTP/JSON frame push | The body keeps only the latest frame |
| Health command | command -> server | HTTP/JSON | The command owns the verdict |

## Failure modes and resilience targets

| Failure | Behaviour | Target |
|---|---|---|
| Driver goes silent | The watchdog stops the motors and the claim lapses | Motors stop within the watchdog's bound ([safety](../safety/ARCHITECTURE.md)). The next driver starts clean |
| The watchdog task itself dies | Health reports its poll age growing. The health command turns that into unhealthy | A dead guard is visible, never silent |
| One wheel-loop tick throws | Logged. The loop continues | One bad tick never ends the loop |
| Camera fails or a teleop phone stalls | Only the frame route answers "service unavailable", with the body's own message | Other sensors keep answering |
| ROS bridge unreachable during a verb | Stop the robot directly. Refuse with a ROS-unavailable reason | |
| ROS bridge hung (accepts, never answers) when a stop arrives | The stop reaches the motors directly; zeroing ROS's inputs proceeds on its own and may fail | A stop never depends on the container, and never stalls the server's other routes |
| ROS bridge unreadable when checking for a goal | Treated as "no goal" | An outage never becomes a lockout of every autonomous driver |
| Part of the ROS container dies while the actuator lives (the bridge, or the velocity multiplexer or controller) | With the bridge gone, the first failed send is refused as ROS-unavailable and marks ROS down, so a person drives on the direct path and autonomy is refused until the bridge answers again. With the multiplexer or controller gone, ROS still reads as alive and verbs achieve nothing and are refused as unsafe | **Met for the bridge** (built 2026-10-02): a person can drive, as with a dead container. The rule is [safety](../safety/ARCHITECTURE.md)'s ("When ROS dies, only a person drives"); the multiplexer-or-controller case is left failing toward stop |
| Unknown verb, malformed body | Client error, nothing moves | |
| Body without motors gets a wheel command | Refused "unsupported" | Never a silent no-op |
| Sim-only or goal route on a world or body that cannot serve it | "Not implemented", naming why | Never a silent no-op |
| Wrong or missing secret | "Unauthorized". Nothing moves, nothing is read | |
| A client newer than the server | "Not found" on the new route, read by the client as an absent sensor | Mixed versions keep working |

## Open questions

- **Boot units on the car (B5)** are decided in
  [operations](../operations/ARCHITECTURE.md) and not built.
- **CPU sharing with sensor drivers.** The lidar driver is decided to live
  in this process (`PLAN-ros-alignment.md` 6, question 5). Today the
  simulator's own ray casting, sharing this process, already dominates the
  HTTP tail at the control rate (3.17). Measure on the Jetson (3.33) before
  adding a serial reader.
- **Manual drivers other than the D-pad under ROS drive** are refused,
  because the ROS bridge has an input only for the D-pad
  (`docs-review/REPORT.md` V10). Owned by [safety](../safety/ARCHITECTURE.md)
  (driver order) and [ros](../ros/ARCHITECTURE.md) (the bridge).
- **Bridge route budget.** New world features (sighting geometry, map
  save) add bridge routes against a deliberate budget
  (`PLAN-ros-alignment.md` 1.1).
