# Teaching a Car to Look

**vision-picar: an introduction**

A small robot you can ask to find something in your house. It looks around,
works out where the thing might be, and drives there without bumping into
anything.

| | |
|---|---|
| **Simulator** | Working |
| **Robot computer** (NVIDIA Jetson Orin Nano Super) | Bought and tested |
| **Robot body** (Waveshare UGV Rover) | Ordered, arriving Oct 19 – Nov 11, 2026 |

**How to read this page.** Each section starts in plain language. The
**More detail** part under it says how it works and what was measured. A
formatted version with expandable sections, for sharing:
[Teaching a Car to Look](https://claude.ai/artifact/GKPq1hkSpEh8TAEG8Te63W).

Contents: [The goal](#the-goal) · [How it looks for things](#how-it-looks-for-things)
· [How it knows it has arrived](#how-it-knows-it-has-arrived)
· [How it stays safe](#how-it-stays-safe) · [How it learns the house](#how-it-learns-the-house)
· [Why it was built in a simulator first](#why-it-was-built-in-a-simulator-first)
· [How "done" is decided](#how-done-is-decided) · [The hardware](#the-hardware)
· [Where it stands](#where-it-stands) · [What is still hard](#what-is-still-hard)
· [Words you will hear](#words-you-will-hear)

---

## The goal

> **"Go find my red backpack."**

You say that sentence. A small robot drives off into your house and finds the
backpack. That is the whole project.

It sounds simple. Three things make it hard:

- **It has never seen your house.** There is no floor plan to start from. The
  robot has to explore.
- **It has never seen your backpack.** It must recognise "a red backpack" from
  the words alone.
- **It must not hit anything.** Chair legs, door frames, the cat. A mistake on
  wheels has consequences.

### More detail: the idea it started with, and why that changed

The project began with a bold bet: no map at all. The robot would take one
photo, ask an AI vision model which way the backpack was, make one move, and
repeat.

That bet was tested on real photos taken from a phone on a small wheeled cart
at floor height. Two things came out of it.

- **Recognition was never the problem.** Every vision model tried could pick
  out a red backpack.
- **Distance was.** No model could reliably say how far away something was
  from a single photo. Five different wordings of the question were measured
  and all failed the same way. A flat photo does not contain that
  information.

So the robot gained a **lidar**, a spinning laser that measures distance
directly. The camera now answers "what is it, and which way?" The lidar
answers "how far?" Once the robot can measure a room, building a map becomes
easy, so the project adopted a standard robot mapping and navigation toolkit
(ROS 2). The original "no map" premise was tested, lost, and was replaced.
`PLAN-onboard-perception.md` has the record.

---

## How it looks for things

The robot looks in three layers. The cheap, fast ones run all the time. The
expensive, clever one is asked only when it matters.

| Layer | What it does |
|---|---|
| **1. Spot** | A small AI model on the robot checks every camera frame for anything that might be the target. Free and fast. |
| **2. Measure** | The lidar says how far away it is. |
| **3. Ask** | A large AI model in the cloud is consulted only at key moments. Each question costs money and takes a second or two. |

Think of a person searching a room. You scan quickly without much thought. You
stop and look carefully only when something catches your eye.

### More detail: the models and when the cloud is called

**The on-board layer.** A detector (`YOLOE-11s`) proposes regions of the
image. A second model (`CLIP`) scores how well each region matches the words
of the target, such as "red backpack". A region counts as a sighting when that
probability reaches 0.8 or more. This all runs on the robot's own computer, so
it costs nothing per frame (`brain/perceive.py`).

**When the cloud is asked.** The cloud model (Claude, on Amazon Bedrock) is
called on a few triggers only: at the start of a mission, when the on-board
layer first sights a likely candidate, and when the search has gone cold for a
while. A sighting must hold for two frames in a row before it triggers a call,
which stops one noisy frame from costing money (`brain/tiered.py`).

| Measure | Result |
|---|---|
| Frames where the target was visible and the on-board layer found it | 82% |
| False alarms allowed when measuring that | 3 frames |
| Size of the labelled photo set | 1,234 frames |
| Time per frame on the robot's computer (median / slow 10%) | 61 / 110 ms |
| Time budget per frame | 250 ms |
| Cloud calls in one end-to-end test mission | 4 over 18 steps |

A lesson from the photo tests: the target's wording matters a lot. "Red
backpack" was detected far more often than a bare noun like "bottle", because
the colour gives the matching model something specific to hold on to.

---

## How it knows it has arrived

The robot declares "found" only when it can see the target straight ahead,
the laser says it is close, and the cloud model agrees it is the right object.

Each check guards against a different mistake. Seeing it guards against
stopping at the wrong place. The laser guards against "close enough" guesses
from a photo. The final question guards against a look-alike.

### More detail: the exact arrival rule

From `brain/arrival.py` (`PLAN-ros-alignment.md` 3.11 and 3.32):

- The target is detected within 3 degrees of straight ahead.
- The lidar, never the camera, reads the target within **0.40 m**. It takes
  the median of five laser beams around that bearing.
- Both hold for **two frames running**.
- The beams must agree with each other. If they spread by more than 10 cm, or
  mix hits with misses, the robot is probably looking at the edge of a door
  frame, and it refuses to judge.
- Then one cloud call on that frame must confirm the identity of the target.

In simulated missions, 676 of 678 arrivals ended "found" when the detector
missed 10–20% of frames on purpose. None were judged from more than 0.386 m
away. Before this rule existed, every one of those missions ended "blocked" or
ran out of steps.

An early version used the single nearest laser beam. It declared "found" 95 cm
away, against a door frame. The test written before the work caught it.

---

## How it stays safe

The AI never drives the wheels directly. It suggests a move, and a separate
safety layer on the robot can refuse it.

If something is in the way, the move is refused, however confident the AI was.
A vision model can be wrong about a glass door. The worst result of this
design is a robot that stops when it did not need to.

| Guard | What it does |
|---|---|
| **The safety veto** | Every move is checked against the laser before a wheel turns. |
| **The watchdog** | If commands stop arriving for one second, the motors stop on their own. |
| **Give up when blind** | If the AI fails to answer three times in a row, the search ends with the robot stopped. |
| **A person wins** | Touch the controls on the phone and the robot obeys you, ending its own search. |

### More detail: what the safety layer actually checks

All of this lives in `robot/safety.py`.

**Moving forward or backward.** A move is checked two ways, one after the
other. First, a cone of depth readings along the direction of travel. Second,
the full width of the robot's body swept along its path, tested against the
360-degree laser scan. Forward speed drops to zero at 20 cm from an obstacle.

**Turning.** A rectangular robot's corners stick out further than its sides
(15.1 cm against 9.9 cm), so turning on the spot can swing a corner into
furniture. A turn that would bring a corner within 1.3 cm of something is
slowed or shortened. A turn away from an obstacle is never limited, so a robot
against a wall can always turn free.

**How it was tested.** Safety is judged against the simulator's ground truth,
not against the sensor readings the safety layer itself uses.

| Test | Runs | Bad outcomes |
|---|---|---|
| Approaches at awkward angles | 5,760 | 0 contacts |
| Turns started with furniture inside the turning circle | 360 | 0 within 1 cm |
| A simulated person walking across the robot's path | 2,021 | 0 contacts |

**Three separate failure guards.** Each guard catches a failure the others
cannot see. The watchdog catches motors left running by a crashed program. The
vision budget catches a cloud service that is down. A third timer catches a
search program that is still running but stuck. Since none of these can be
caused by pressing a button, the phone app has **drills** that break one thing
on purpose so you can watch the right guard catch it (`control/drills.py`).

**Who is driving.** The robot ranks its drivers: a stop beats a person, and a
person beats any automated driver. Authority lapses after a second of silence,
so there is no "release" step to forget.

---

## How it learns the house

As it drives, the robot draws its own floor plan from the laser. It can then
be sent to any spot on that map and plan its own route there.

Wheels alone are a poor guide. They slip and drift, and small errors pile up.
The map lets the robot correct itself by recognising walls it has already
seen.

### More detail: mapping and route planning

Mapping uses `slam_toolbox`. Route planning and driving to a goal use `nav2`.
Both are standard parts of ROS 2, a toolkit most research and commercial
robots use. They run in the container under `service/slam/`.

| Test | Result |
|---|---|
| Position error from wheel counts alone, one wheel sensor 3% wrong on purpose | up to 99 cm |
| Position error with the map correcting it, same fault | 1–4.5 cm |
| Drive-to-goal trials in a realistic house (90 cm doorways) | 6/6, twice |
| Tour of a furnished model of the owner's home | 8 of 9 rooms |

The one failed room was a dining room. The route planner treats the robot as a
circle and found a gap between chairs that the driving controller, which knows
the robot is a rectangle, refused to enter.

The map is discovered, not handed over. The simulator casts the laser from
wherever the robot stands, so rooms appear only once the robot has looked into
them.

---

## Why it was built in a simulator first

Almost everything was built and tested against a simulated house before any
robot was bought. The trick is that the robot's "body" can be swapped without
changing its "brain".

The brain only ever talks to a body through one fixed set of commands: move,
turn, take a photo, read the laser. Anything that answers those commands can be
the body.

| Body | Status | What it is |
|---|---|---|
| A simulated robot | Working | A make-believe house in software, with walls, furniture and a backpack. |
| You, with a phone | Working | Your phone's camera is the robot's eyes. You walk where the arrow points. |
| A software motor board | Working | A copy of the real robot's motor controller, built from its source code. |
| The real robot | Next | Same brain and safety rules, real wheels and laser. |

The phone mode deserves a word. When you use the app's **Guide** tab, you
stand in for the robot. The arrow on screen is the move it would have made.
Your legs are its wheels. This made it possible to test the idea on real rooms
with no hardware at all.

### More detail: the five programs and the rules between them

The system is five cooperating programs (`docs/ARCHITECTURE.md`). A move
travels down this chain:

1. **Phone app** (`web-twin/`). Shows what the robot reports. Starts and stops
   missions. Has a D-pad for driving by hand.
2. **Brain** (`control/brain_server.py`). Decides what to do next. Runs the
   mission and the AI calls. Reaches the robot only over the network.
3. **Robot server** (`robot/server.py`). Decides who may drive, runs the
   watchdog and the safety veto. The only program that touches the body.
4. **ROS 2 container** (`service/slam/`). Turns moves into wheel speeds,
   builds the map, plans routes. Sealed in its own box. Its wheel speeds go
   back through the robot server and are vetted again.
5. **Body.** The simulator today (`sim/mock_robot.py`), the motor board on the
   real robot (`robot/hardware_robot.py`).

Four rules, each enforced by an automated test:

- **One interface for the body** (`robot/interface.py`). The brain and robot
  server never know whether they drive the simulator or real motors. Eight
  different bodies pass the same conformance tests.
- **Body facts and world facts are kept apart** (`world/interface.py`). "How
  far have my wheels turned" belongs to the body. "Where am I on the map"
  belongs to the world. A map's position can jump when it corrects itself,
  and wheel counts never should.
- **ROS stays in its box.** Nothing outside the ROS container may use ROS.
  Everything else talks to it over the network.
- **Safety runs last.** ROS's own collision check runs first, then the robot
  server's. Each can only slow or stop, never speed up.

The result is that switching from the simulator to the real robot is a
settings change. The code that talks to the real motor board already exists
and passes the same tests against a software copy of the board. The
engineering reference for the searching loop is
[`AGENT-HARNESS.md`](AGENT-HARNESS.md).

---

## How "done" is decided

Before building anything, the goal and the passing score are written down.
Then the feature is run hundreds of times the way the robot will run it, and
the numbers decide.

A target chosen after seeing the results is only a description. One good run
proves little, since it can hide the one attempt in ten that drives into a
door frame. Hundreds of logged runs do not hide it.

### More detail: the rules, and what they have caught

- Write the measure and the threshold down before measuring.
- Measure through the real mission path, end to end. Never through a shortcut
  that moves the robot by hand.
- Record the numbers in the plan and pin them in an automated test, so a later
  change that makes things worse fails the test suite.
- Pair every "steady" measure with a "progress" measure. A robot spinning in
  place has very steady steering and gets nowhere.

Failures are recorded as failures. Several phases list a criterion that was
missed, with the reason, instead of quietly moving the bar. The suite has
about 1,700 automated tests, including ones that drive the phone app in a real
browser at phone size.

Until 25 September 2026 a feature counted as done when someone had watched it
work on a phone. That caught real bugs, and watching still happens. It is no
longer the finish line.

---

## The hardware

The robot is two purchases: a small computer that runs the AI on board, and a
robot base with wheels, a laser and a camera.

| Part | Status | What it is |
|---|---|---|
| **NVIDIA Jetson Orin Nano Super** | Bought and tested | A palm-sized computer with a graphics chip. It runs the on-board AI well inside its time budget. |
| **Waveshare UGV Rover** | Ordered, arriving Oct 19 – Nov 11 | A wheeled base that turns on the spot, with a laser scanner, a depth camera and a camera that pans and tilts. |

### More detail: why these parts

**Why a Jetson.** The cheaper plan was a Raspberry Pi 5 with a small AI
accelerator. It lost on a measurement. The accelerator runs only models that
can be converted for it, and the strongest detector tried could not be
converted at any setting. That test cost $3.20 on a rented server and settled
a $400 question. The Jetson runs ordinary AI models as they are.

On the Jetson, at its 15-watt power setting, the on-board AI takes a median of
61 ms per frame against a 250 ms budget. Its full test suite passes on the
board, and the robot's control loop missed zero of 14,289 deadlines with
mapping, route planning and the AI all running together.

**Why the Rover.**

- **It turns on the spot.** The first plan, a PiCar-X kit, steers like a car
  and cannot pivot.
- **It reports wheel turns to the computer.** Without that, the robot cannot
  tell how far it has driven. A competing kit was cancelled when its maker
  confirmed it does not send this data.
- **Its motor board firmware is open source.** The project has a small patch
  (`firmware/ugv_base_ros/`) that makes wheel readings finer and adds a
  timestamp. It improved turn accuracy from 60 of 120 turns within 1 degree to
  120 of 120 in simulation.

**Still to check.** The Rover's mounting plate is designed for a different
Jetson carrier board, so a caliper check of the mounting holes is due before
it arrives. The fallback is an adapter plate. A separate battery for the
Jetson may be needed, depending on a power test with the motors running.
`docs/hardware/JETSON-BOM.md` has the record.

---

## Where it stands

The software is built and measured in simulation. The robot computer is
tested. The robot body is on its way.

| When | Milestone |
|---|---|
| Aug 2026 | **A simulated house and a searching loop.** A brain that searches room by room, a safety layer that can overrule it, and a phone app to drive and watch. |
| Aug–Sep 2026 | **Real photos, real rooms.** Phone walks at floor height showed the AI recognises targets but cannot judge distance. The plan gained a laser. |
| Sep 2026 | **Seeing on board.** Fast on-board AI with the cloud model asked only at key moments. The Jetson chosen over the Pi. |
| Sep 2026 | **A real robot toolkit.** Smooth driving, mapping and route planning with ROS 2. Arrival recognition. Safety tested on thousands of runs. |
| Oct 2026 | **The Jetson in hand.** Everything runs on the robot's own computer, inside its time budgets. |
| **Oct–Nov 2026 (now)** | **The robot body arrives.** Mount the computer, check the motors and sensors, measure the real chassis, then drive the same tests on a real floor. |
| Early 2027 | **Software platform upgrade.** The current robot toolkit version stops getting updates in May 2027. The move to a newer one is planned for after the robot is driving. |

### More detail: what hardware day involves

- Check the mechanical fit of the Jetson on the Rover's deck.
- Flash the patched motor board firmware after the arrival checks pass.
- Measure the numbers the robot description still marks as placeholders, such
  as how much the wheels skid when turning.
- Run the power test with motors to decide whether the Jetson needs its own
  battery.
- Set the robot, brain and toolkit to start at boot, so a mission can be
  started from a phone with no laptop involved.

---

## What is still hard

A simulator is kinder than a real floor. These are the problems the project
expects to meet, and the ones it already knows about.

- **A laser sees one slice.** It catches chair legs and misses a cable on the
  floor or a tabletop above it.
- **Rendered rooms are not real ones.** The on-board AI is tested on real
  photos. In the simulator, sightings are supplied directly, so the simulator
  says nothing about how well the AI sees.
- **Maps drift in open rooms.** In large furnished rooms the map can slide by
  half a metre or more during a tour. This is being worked on now.
- **Looking costs time.** A cloud answer takes a second or two. A robot moving
  while it waits acts on an old photo.

### More detail: known gaps in the numbers

- **Turn accuracy on stock firmware.** Without the firmware patch, turns
  scatter by 1.3–1.8 degrees because the motor board's readings have no
  timestamp. The gyro or the map is the fix on the real robot.
- **Placeholder chassis numbers.** The effective turning width of a skid-steer
  robot and some sensor offsets can only be measured on the real robot.
- **Phone walks cannot finish.** The arrival rule needs the laser. A phone has
  none, so a phone walk never ends "found", and its outcome says nothing about
  navigation.
- **The arrival confirmation has not yet met the real cloud.** It is tested
  against stand-ins only.
- **Map drift.** In the furnished home model, 0.45–0.86 m of drift during
  tours of open rooms is an open item.

---

## Words you will hear

| Term | Meaning |
|---|---|
| **Lidar** | A spinning laser that measures the distance to the nearest thing in every direction, many times a second. |
| **Vision model** | An AI that takes an image and a question and answers in words. "Is there a red backpack, and where?" |
| **Detector** | A small, fast AI on the robot that only asks "is the thing here, and where in the picture?" |
| **Simulator** | A make-believe house in software that the robot's brain drives exactly as it will drive the real robot. |
| **Digital twin** | The phone app. It drives and shows the simulated robot today, and the real one later. |
| **Safety veto** | The rule that lets the robot's body refuse a move the brain asked for. |
| **Watchdog** | A timer that stops the motors if commands stop arriving for one second. |
| **SLAM** | Mapping a place and working out where you are in it, at the same time. |
| **ROS 2** | The Robot Operating System, a widely used toolkit for robot mapping, route planning and motor control. |
| **Mission** | One search, from "go find it" to found, blocked, stopped or out of steps. |

---

*For the full engineering detail: `docs/ARCHITECTURE.md` (the whole system on
one page), `docs/plans/PLAN-ros-alignment.md` (every phase, its criteria and
its measured results), `docs/plans/PLAN-onboard-perception.md` (how the
on-board AI was chosen), and `docs/hardware/JETSON-BOM.md` (what was bought,
and why). Updated 7 October 2026.*
