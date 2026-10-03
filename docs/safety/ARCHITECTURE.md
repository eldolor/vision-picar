---
kind: architecture
domain: safety
status: current
verified: 2026-10-02
---

# Safety -- architecture

The safety domain decides what may move the wheels. It has three parts: a
local layer that vets every motion against what the robot's own sensors
see; an authority order that says who may drive when more than one party
wants to; and a watchdog that stops the motors when commands stop coming.
Read this for the rules and the reasons behind them. The
[engineering spec](../engineering/safety/ENGINEERING.md) has the margins,
the clearance sources, the refusal reasons and the ground-truth sweeps that
pin them.

## Purpose

Every party that can move this robot is fallible. A vision model guesses
distances it cannot measure. A person on a phone has a laggy picture. A ROS
planner can freeze, and once did (`PLAN-ros-alignment.md` 3.15). The safety
domain makes the robot safe regardless of which of them is driving. **The AI
sits at the bottom of the hierarchy**: it proposes, and the layer below
re-checks with its own sensors and can refuse.

Who depends on it: the robot server
([control-api](../control-api/ARCHITECTURE.md)), which hosts the
arbitration and the watchdog and runs every motion through the vet; the
brain ([mission](../mission/ARCHITECTURE.md)), which must react correctly
to each kind of refusal; and the ROS chain ([ros](../ros/ARCHITECTURE.md)),
whose wheel commands the vet re-checks. Without it, a confident model
decision drives into a wall, and two drivers fight over one robot.

## Components and boundaries

```text
  person (D-pad, curl)    brain mission     nav2 goal / ROS chain
          \                    |                    /
           +------ authority order (who may drive) +       <- robot server
                               |
                      the vet (may this move?)             <- local safety layer
              cone ahead | swept corridor | rear | pivot
                               |
                      the body (wheels, sensors)
                               |
                   watchdog: silence -> motors stop        <- robot server
                   board heartbeat: host silence -> stop   <- motor board
```

| Part | Owns | Boundary |
|---|---|---|
| **The vet** (`robot/safety.py`) | The only definition in the project of "the path", "clearance" and "may this move". It carries out guarded verbs itself | Talks only to the body contract. It knows nothing of drivers, HTTP or ROS |
| **The authority order** | Who holds the robot, and the ranks | Lives in the robot server. The rank table lives on the body contract because the brain may import that and little else |
| **The watchdog** | The deadman on command silence | Lives in the robot server process, not the brain |
| **The ROS-down fallback** | What happens to driving when the ROS chain stops responding | The robot server's decision. ROS cannot judge its own death |
| **Ground-truth instruments** | Judging whether the vet worked | Share no geometry code with the vet they judge |

Outside this domain, and in series with it: the brain's own failsafes
(vision timeout and failure budget, hung-tick detection, the halt gate,
in [mission](../mission/ARCHITECTURE.md)); ROS's `twist_mux` and
`collision_monitor` ([ros](../ros/ARCHITECTURE.md)); and the motor board's
heartbeat ([motor-board](../motor-board/ARCHITECTURE.md)).

## Decisions

### Safety is local, and it is the last word on every path

Every motion, from any driver and through any route, passes the vet on the
robot. That covers verbs, standing wheel commands, and the ROS chain's wheel
posts. A person on the D-pad gets the same collision protection as an AI
decision. **Two exceptions today, in one window:** on a body that cannot
yet report its own wheels (a real motor board before its first feedback),
a standing wheel command passes unvetted, and a reverse verb runs unguarded
astern, because until the wheels report the layer cannot tell that the body
really moves. That is a gap, not a decision (see Open questions). **Rejected:** trusting the brain's own check. The brain keeps one
as an early out, but the robot's check is authoritative, and both must be
able to veto. **Rejected:** relying on ROS's `collision_monitor` on the
navigation path. The two collars run in series with this layer last,
because this layer must keep working when ROS hangs
(`PLAN-ros-alignment.md` 3.15, 1.1).

### Re-check the sensor, never the AI's claim

The layer never takes a distance from a policy. Model-reported obstacle
fields disagree from about 0% to about 100% across models on the same frames
(CLAUDE.md section 5, Stage 0), so on the car the range sensors decide.

### What the sensor could not see is neither clear nor blocked

A zone or beam that could not be measured never enters a distance
comparison. Read as a distance, every dropout would stop the robot. Read as
clear, the robot would drive into what the sensor missed. If nothing at all
observes the ground a forward move would cross, the move is refused; the
same holds astern for a body that really moves (below). A single scalar
sensor with no way to say "unmeasured" fails toward stop. Rotation is the
deliberate exception: a turn nothing can see around is allowed, because
turning is how a robot gets away from something.
History: `PLAN-microduck-transplants.md` M3; `PLAN-ros-alignment.md` 3.18
part 2.

### The path is an angle from the body, not a share of the sensor

"Ahead" is the cone the chassis sweeps over one move, chosen by bearing
relative to the body. **Rejected:** the middle half of the sensor's columns.
On a 360-degree sensor that is the whole forward hemisphere, which vetoes
every corridor (`PLAN-onboard-perception.md` 5.1). **Rejected:** the
direction the camera points. A camera left panned by a preempted mission
once guarded a forward move with a cone looking sideways
(`PLAN-ros-alignment.md` 3.18).

### The chassis' swept corridor is checked too, in series with the cone

The cone is sized for one move, so close in it is narrower than the robot.
An obstacle off a front corner sits outside it. A second check looks at
everything beside and ahead of the chassis outline, placed relative to the
body, and asks whether anything lies in the strip the chassis will sweep.
Whichever check reads less decides.
**Rejected:** widening the cone. That refuses every narrow doorway.
**Rejected:** treating returns beside the body as blockers. A straight move
cannot close on them, and nav2's stop polygon froze the robot against a door
jamb for exactly that reason (`PLAN-ros-alignment.md` 3.15, 3.18).

### A turn is refused only when it closes on something

A rectangle's corners swing beyond its sides when it pivots, so rotation is
vetted. A turn that would bring a corner too close and is closing the gap is
slowed to what keeps the margin. A turn that opens the gap is never touched.
**Rejected:** exempting rotation. Under it, all 120 close pivots in the
sweep came within 1.0 cm of something or touched it. **Rejected:** blocking every turn near furniture, because a robot
pinned against a sofa must always be able to turn away
(`PLAN-ros-alignment.md` 3.19).

### Reversing is checked on every path

Decided by the user: backing up is vetted against the rear, for the D-pad's
reverse verb and a negative wheel command alike. A rule that covers one of
two ways to back up is not one (`PLAN-ros-alignment.md` 3.10).

### A body that cannot see astern does not reverse

**Decided by the user, 2026-10-02** (`docs-review/SPEC-REVIEW.md` finding
3). When nothing observes the ground behind the robot, and the body really
moves (it reports its own wheels), every way of backing up is refused: the
reverse verb, a standing reverse command, and a guarded verb's corrective
pass astern. This is "unobserved is not clear", applied astern exactly as it
already applied ahead. On the car it means: no reversing until its rear
sensing lands. A body that moves nothing (a recorded walk), or whose motion
is a person who can see (a phone walk), keeps reversing, because its
reverse cannot hit anything the robot is responsible for. Turns stay
allowed blind.

**Rejected:** refusing turns too when nothing can see. On the car today
that leaves a robot that cannot move at all once forward is also blind, and
a robot pinned against something must always be able to turn away.
**Rejected:** keeping blind reverse, the behaviour until this decision. It
backs the robot into whatever it cannot see; the rear is exactly where a
forward-facing robot has the least idea what is there.

**Trade-off accepted:** until rear sensing is fitted, the car cannot back
out of a dead end, and a guarded move that overshoots forward cannot correct
itself backwards; it stops where it is. A person may still turn it.

### Stop at the line, and keep checking while moving

A move is re-vetted every control period and looks ahead. It may cover only
the room left before the line, so it slows and stops at the line instead of
being caught after crossing it. **Rejected:** vetting a verb once at its
start, which let a move allowed at 20 cm end past the wall
(`PLAN-ros-alignment.md` 3.22). **Rejected:** clamping only once the reading
is already under the line, which put one ROS-path move just under the bar
(3.24 G2). A move cut short is reported as executed, with the distance it
covered. A move that achieved almost nothing is reported as a refusal, so a
pinned robot ends its mission `blocked` rather than pushing forever.

### Who drives: stop, then a person, then one autonomous driver

```text
   stop  >  a person  >  one autonomous driver at a time
```

Ranks are by role, not by client. The five rules, each chosen because its
opposite is a real failure (`AGENT-HARNESS.md` 4.1):

1. **Stop is never arbitrated.** Anyone may stop the robot at any time.
   A stop zeroes the wheels, **and it ends any active navigation goal**
   (decided by the user 2026-10-02, built the same day; see below).
2. **Stop claims nothing.** Otherwise the loser of an arbitration takes the
   robot back by giving up.
3. **People share; autonomy is exclusive.** Two taps of a person pass. At
   the autonomous rank the holder keeps the robot until its claim lapses,
   so the brain and the ROS stack never interleave commands. This is the
   convention `twist_mux` encodes (`PLAN-ros-alignment.md` 3.10).
4. **Authority lapses on silence**, on the watchdog's own clock. There is no
   release call to forget, and one tap does not lock the brain out forever.
5. **An unnamed command ranks as a person.** The callers that do not name
   themselves are a person with curl, a script run by hand, or a test.

**A stop ends a navigation goal; it does not pause it.** Decided by the
user 2026-10-02 and built the same day. A stop also cancels any active goal, and
the cancel runs after the wheels are zeroed and off the stop's own path, so
a stop never waits on ROS. A person who wants the goal back sends it again.
**Rejected:** a stop that only pauses the goal, with a separate cancel
control. That keeps a goal resumable, but it means "stop" does not stop: the
planner keeps commanding and the robot drives on once the stop's hold ends,
which is the opposite of what the top of this order promises. **Trade-off:**
an interrupted goal is lost, and resuming it is one more action for a
person (`docs-review/SPEC-REVIEW-2.md` H1).

A navigation goal is an autonomous driver too. While one is pending or
active, every other autonomous command is refused, and a person is never
refused because of one (`PLAN-ros-alignment.md` 3.23). **Rejected:**
last-writer-wins, the arrangement before M4. The D-pad and a mission each
read the other's moves as the world changing under them.

### A refusal names its reason

A safety veto means "this move was unsafe; the next may be fine". A
preemption means "you are not driving; stop, do not retry". Those are
opposite responses, so every refusal carries a machine-readable reason, and
the brain branches on the reason rather than the prose (M4,
`AGENT-HARNESS.md` 4.2).

### The watchdog lives with the motors, not with the brain

The robot server stops the motors when no driving command has arrived
within the timeout. A sensing read is not a command. Otherwise a passive
observer polling the camera would hold the watchdog off for ever. A guarded
verb in progress is a busy driver, not a silent one. It is bounded and
re-vetted, and a stop still ends it. The watchdog depends on the brain
being a separate process, which is [control-api](../control-api/ARCHITECTURE.md)'s
decision ("The robot server and the brain are two processes"). The three
failsafes stay separate because each sees a failure the others cannot. The watchdog sees a dead link or a crashed caller. The brain's
vision budget sees a dead model service. Its hung-tick guard sees a brain
that is alive but stuck.

### When ROS dies, only a person drives

Under ROS drive, the robot server judges ROS alive from the actuator's own
regular wheel posts. **Rejected:** asking the container whether it is
healthy. The container can answer while the chain behind it no longer
reaches the wheels. The actuator's posts prove the end of the chain is
alive, but not every part before it: if another part of the container dies
while the actuator keeps posting, ROS still reads as alive (see Failure
modes). **So a failed send to the bridge counts too: it marks ROS down**,
and the fallback below applies. Decided by the user 2026-10-02; not yet
built. **Rejected:** making the container exit when one of its nodes dies.
That is only a launch-file change, but it covers only deaths the launcher
sees, and a hung bridge is not one. **Rejected:** both together, which adds
the launch change for no case the bridge-failure rule misses on the
commanding path. **Trade-off:** a dead multiplexer or controller behind a
live bridge still reads as alive; its verbs achieve nothing and come back as
safety refusals, which fails toward stop (`docs-review/SPEC-REVIEW-2.md`
M2). While ROS is down, a person's verbs run through the
direct path, which uses the same vet, re-checked every period. Every
autonomous command is refused with a reason that ends the mission. Recovery
needs no restart (decided by the user, `PLAN-ros-alignment.md` 3.24 G3).
**Rejected:** refusing everyone, which leaves a person unable to move the
robot out of the way. **Rejected:** letting autonomy continue on the direct
path, which would give one mission two sets of motion semantics.

### Judge safety on ground truth, never on the vet's own readings

A sweep measures the chassis rectangle against the house's real geometry,
written independently of the vet. **Rejected:** judging by sensor readings.
The "flaky 18.0 cm" of 3.17 was a reading, and the truth under it was a
camera looking sideways (`PLAN-ros-alignment.md` 3.18).

## Contracts

| Between | Direction | Category | Ownership |
|---|---|---|---|
| Vet -> body | vet reads sensors and writes the wheel command | in-process (body contract) | The body owns readings. Only vetted paths write motion |
| Robot server -> vet | server calls the vet for every motion | in-process | The server owns the authority state and the watchdog clock |
| Robot server -> any driver | refusal with a reason | HTTP/JSON | The server is authoritative. Clients branch on the reason |
| Brain -> robot server | the brain names itself as a driver | HTTP header | The rank table on the body contract is the single source |
| ROS chain -> robot server | the actuator posts wheel commands, vetted every period | HTTP/JSON | The ROS actuator is the only wheel writer under ROS drive |
| Motor board <- hardware body | a heartbeat the board enforces on its own | serial | A deadman independent of the host process |

## Failure modes and resilience targets

The numbers below are acceptance bars that phases were closed against. They
are commitments, not tuning.

| Failure | Response | Target |
|---|---|---|
| A move toward an obstacle, any heading, any driver | Slowed, then stopped at the line | After every move, travel-to-contact no more than **2 cm** inside the stopping floor (**18.0 cm** at today's floor). The gap never falls below the smaller of its starting value and **1.0 cm** (3.18, 3.22, 3.24 G2) |
| A pivot near furniture | A closing turn is slowed or stopped. A turn away proceeds | No corner within **1.0 cm**. At least 95% of turns with room complete (3.19) |
| Camera panned away, no scan | Forward refused (path not observed) | A panned camera never stops the robot later than a centred one (3.18) |
| Nothing sees astern, on a body that moves | Every reverse refused; turns allowed | The robot never backs into what it cannot see |
| Sensor dropout | The zone or beam is dropped from the comparison. A scalar sensor that drops out reads zero, which vetoes: it fails toward stop | Never read as clear |
| Link to the driver lost, or the caller crashes mid-command | The watchdog stops the motors. Under ROS drive, a silent ROS input is dropped sooner by the ROS chain's own input timeout ([ros](../ros/ARCHITECTURE.md)); the watchdog is the backstop for a silent actuator | Motors stop within one watchdog period of the last command, plus at most the watchdog's own wake-up interval |
| Brain alive but stuck, or the model service down | Brain-side failsafes end the mission ([mission](../mission/ARCHITECTURE.md)) | The robot stopped and the mission `failed` |
| A person takes over from a mission | The mission is refused `preempted` and ends | The person's command executes. The mission never retries |
| Two autonomous drivers at once | The second is refused | One autonomous writer at a time |
| ROS container dies under ROS drive | Autonomy refused. A person drives on the direct path | The mission ends within **3 s**. A person drives within **2 s**. ROS is back without a restart (3.24 G3) |
| Part of the container dies while the actuator lives (the bridge, or the velocity multiplexer or controller), under ROS drive. **UNCONFIRMED** (read, not run) | ROS still reads as alive, because liveness is judged only from the actuator's posts. With the bridge gone, every verb, a person's included, is refused as ROS-unavailable. With the multiplexer or controller gone, a verb achieves nothing and comes back as a safety refusal | **Not met.** The target is the dead container's: a person can drive and autonomy is refused. Fix decided by the user 2026-10-02, not yet built: a failed send to the bridge marks ROS down (Decisions, "When ROS dies, only a person drives"). A dead multiplexer or controller behind a live bridge is left as is: it fails toward stop |
| A stop while a navigation goal is active | The wheels stop and the goal is ended; a person can set a new one | **Met** (Decisions, "Who drives"; built 2026-10-02). Before that the goal was only paused and the wheels resumed after the stop's hold |
| The robot server process dies | The board's heartbeat stops the motors | Motors stop without the host |
| A person or pet crosses the path | Same vet, same bars | The static-obstacle bars above hold with something moving (3.30) |

## Open questions

- **Speed-dependent stopping distance.** The stopping floor is a fixed
  distance, which is sound only at the speeds verbs use today. ROS speed
  zones wait on this (`PLAN-ros-alignment.md` 1.1, question 9). Owner: the
  user, on hardware data.
- **Calibration on the real chassis.** The cone's chassis width, the
  sensor-to-bumper offset and the corridor's side margin are hardware-day
  measurements (R8).
- **Driver names under ROS drive.** The ROS bridge maps only the D-pad, the
  brain and ROS. A bare unnamed command, and both teleop drivers, are refused
  under ROS drive (`docs-review/REPORT.md` V10). Decide whether the bridge
  learns them or the server maps them.
- **Motion before a body can report its wheels.** Until a real motor board
  sends its first feedback, a standing command passes unvetted and a
  reverse verb runs unguarded, because the blind-reverse rule cannot tell
  the body moves. Refuse instead, or accept and record
  (`docs-review/SPEC-REVIEW.md` fix-list 6). Owner: the user. Where in the
  code: the [engineering spec](../engineering/safety/ENGINEERING.md),
  Known gaps.
- **G4 on the Jetson.** Safety-loop timing under full perception load
  (`PLAN-ros-alignment.md` 3.33) is unmeasured. Zero late safety ticks is
  the written bar.
