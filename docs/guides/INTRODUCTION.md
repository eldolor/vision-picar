# Teaching a Car to Look

**vision-picar — project introduction**

A small robot car you can send to find something. Not by giving it a map or a
route — by letting it look at the room, the way you would, and work out where to
go next.

- A small robot car on an NVIDIA Jetson Orin Nano Super (Jetson and robot base ordered, 2026-09)
- Claude vision models on AWS Bedrock
- Simulation built first, hardware next

> A formatted version of this document, for presenting or sharing, is in
> [`INTRODUCTION.html`](INTRODUCTION.html) — open it in any browser.

---

## The intent: "Go find my red backpack."

That sentence is the whole project. You say it, and a small camera-equipped car
drives off into a real house and comes back having found the thing — around the
sofa, through the doorway, past whatever is on the floor that day.

What makes that hard is not the driving. It's that the car has never seen your
house, has no floorplan, and doesn't know what your backpack looks like.
Traditional robot navigation solves this by building a map first and then
planning a route through it. This project takes the other road: **no map at
all**. The car takes a photograph of whatever is in front of it, asks an AI
vision model what it's looking at and which way the backpack is, and makes
exactly one move. Then it does it again.

It's a deliberately simple idea with a lot hiding inside it, which is why most of
the work so far has gone into building somewhere safe to test it.

### And that premise did not survive contact with the evidence

**Added 2026-09-07.** The paragraph above is how this project started and it is
no longer where it is going. Worth saying here rather than only in a planning
document, because a reader who finds this page and stops will otherwise carry
away a description of a different robot.

Testing the loop against real photographs (the "Stage 0" gate in `CLAUDE.md`)
established two things. Every vision model **identifies** a red backpack, so
recognition was never the hard part. And no model can reliably say **how far
away** anything is from a single photograph -- five different phrasings of the
question were tried and measured, and the failure was the same each time,
because a flat image genuinely does not contain that information. Asking harder
was not going to work.

So the plan changed, and `PLAN-onboard-perception.md` is where. The car gets a
**lidar** -- a spinning laser that measures distance directly -- and the
division of labour becomes: the camera says *what* and *which way*, the lidar
says *how far*. Once the car can measure the room, building a map stops being
something to avoid and becomes something it gets nearly for free. The intended
destination is now a real navigation stack (ROS 2), kept behind a wall so it
cannot swallow the rest of the system.

**That is the opposite of "no map at all", and the reversal is the point.** The
original premise was a reasonable bet that a language model's judgement could
substitute for a sensor. It was tested rather than assumed, and it lost. The
loop above still runs -- it is still one photograph, one decision, one move --
but it now runs at three different speeds, with the slow, expensive, clever tier
asked only when something interesting happens.

The rest of this page describes the loop as built. Read
`PLAN-onboard-perception.md` for where it is going.

---

## The mechanism: one loop, about once a second

Everything in this project — the simulation, the phone app, the car that doesn't
exist yet — runs the same four-step loop. The only thing that changes between
them is who or what performs the last step.

```
   ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌ ─ ─ ─ ─ ─ ┐
   │   LOOK   │ ─> │  THINK   │ ─> │  DECIDE  │ ─> │   MOVE     │
   └──────────┘    └──────────┘    └──────────┘    └ ─ ─ ─ ─ ─ ┘
        ▲                                                │
        └────────────────────────────────────────────────┘
          the world has changed slightly — look again
```

| Step | What happens |
|---|---|
| **Look** | Take one photo of whatever is straight ahead. No map, no memory of the room's layout. |
| **Think** | Send that photo to a vision model. It reads the scene and answers in plain terms: is the target visible, and where? |
| **Decide** | Turn that answer into one small instruction — forward, left, right, stop. Never a whole route. |
| **Move** | Something carries the instruction out. **This is the part that swaps** — and the reason the project works the way it does. |

---

## The trick: three bodies, one brain

Because the last step is the only part that changes, you can unplug the body and
put a different one in its place. The eyes and the brain stay exactly the same.
That's what lets the project be tested long before any hardware is finished — and
it's the thing worth understanding before you try the app.

| | **A simulated car** | **You, on foot** | **The real car** |
|---|---|---|---|
| **Status** | Working today | Working today | **Next phase** |
| **Eyes** | A drawn first-person view of a make-believe house | Your phone's actual camera | The camera bolted to the car |
| **Brain** | The vision model, called for real | The vision model, called for real | The vision model, called for real |
| **Body** | A car that exists only as software | Your legs | Two wheel motors -- it steers by turning them at different speeds, so it can spin on the spot |

---

## Guide mode: when you use the app, you are the car

The middle column above is the one people find surprising, so it's worth saying
plainly. **Guide** — the tab that turns on your camera and points an arrow at
what you're looking for — is not a separate consumer feature that happens to
share some code. It is **the robot's first-person view**, handed to a human.

> Hold up the phone and you're standing in for the car. The camera is its camera.
> The arrow on screen is the instruction it would have sent to its motors. Your
> legs are the motors.

The chevron that swings left and right, the glow along the edge of the screen,
the pulse that quickens as you close in, the outline that snaps around the object
when you've arrived — all of that is one decision, rendered for eyes instead of
wheels. The car will receive the same decision as `FORWARD` or `LEFT`.

This is genuinely useful rather than a gimmick. Walking a room with Guide tells
you, in about ninety seconds and with no hardware at all, whether the loop is
good enough to drive something: whether the model keeps up when you turn, whether
its sense of "close" matches yours, whether once-a-second is often enough to feel
responsive. Every one of those questions has to be answered before a car is worth
building, and every one of them is cheaper to answer on foot.

---

## The discipline: the brain proposes, the body can refuse

One rule shapes the whole codebase: **the AI is never allowed to drive
directly.** Every instruction it produces passes through a safety layer that sits
with the body, not the brain. If the distance sensor says there's something
closer than 20 cm ahead, the move is vetoed and the wheels stop — whatever the
model asked for, however confident it sounded.

The same layer runs a watchdog: if the brain goes quiet for more than a second —
crashed, disconnected, waiting on a slow network call — the motors stop on their
own rather than continuing with the last thing they were told.

This matters more than it might sound. A vision model can be confidently wrong
about a glass door or a dark stair edge. Keeping the veto in the hardware means
the worst case is a car that stops for no reason, not one that drives into
something. It's also why the simulation and the real car will share this code
unchanged — the safety rules were written once, tested in software, and are
already the rules the hardware will run.

---

## Whose job is it to keep looking?

For most of this project the searching was done by something *watching* the car:
a program on a laptop, or the phone app itself. That works, and it hides a
problem. Close the browser tab and the searching stops — the car sits in the
hallway waiting for an instruction that will never come. A robot that needs
someone holding a phone for it is not really a robot.

So the loop moved onto the car. It is now a small program of its own, running
alongside the one that turns the wheels, and it is the thing that keeps asking
"what do I see, where do I go next?" The phone's job shrank to three buttons:
**start**, **stop**, and **watch**. You can close the app mid-search, walk into
another room, open it again — the car has kept going, and the app picks the
story back up where it got to.

The two programs stay separate on purpose, even though they run on the same
machine. One is the *body*: wheels, sensor, safety veto. The other is the
*mind*: what to do next. Keeping them apart is what lets the mind be moved —
onto a laptop for development, onto the car for real use — by changing a single
address, with no code change at all.

---

## Three ways to stop, because there are three ways to fail

The safety veto above catches "about to hit something." It cannot catch a car
that has quietly stopped making sense. Three different failures need three
different guards, and each one ends the same way — wheels stopped.

| What goes wrong | What catches it |
|---|---|
| A move starts the motors and then the program crashes before stopping them | The body's own **watchdog**: if no command arrives for a second, it stops the wheels itself |
| The car goes blind — the vision service is down, or an answer never comes back | After three failed looks in a row, the search **ends rather than driving blind**. One bad answer is survivable; a pattern is not |
| The searching program freezes — still running, but stuck | A **dead-man timer** notices the loop hasn't come back and stops the car |

The second and third are the ones that matter most on a real floor, and neither
can be triggered by pressing anything — you would have to unplug the internet at
exactly the right moment. So the app has a **drill** setting: ask the car to
break one thing on purpose, and watch the right guard catch it. Every drill can
only ever end with the car stopped, which is the same thing the guards do
unaided.

---

## The rule: decide it with numbers, then put it in the car

Everything above was built in a particular order, and it is worth stating as a
rule rather than a habit:

> **A capability is finished when the numbers say so.** Before trying it, write
> down what "working" means and the score it has to reach. Then run it the way
> the car will run it -- many times, in the simulated house -- record what
> happened, and keep a test that fails if it ever gets worse. Only then does it
> go anywhere near the car.

Three reasons this is a rule.

**Deciding the bar first keeps everyone honest.** A target chosen after seeing
the results is a description, not a test.

**One good run proves very little.** Watching a single search cross a room can
hide the one start in ten that drives into a door frame. Hundreds of runs, each
logged, do not.

**Some failures cannot be provoked by hand.** That is not an excuse to leave
them unverified; it is why the drills exist.

Until 25 September 2026 the rule was different: a capability counted as
finished when someone holding a phone had watched it work in the twin. That
caught real bugs, and the twin is still how a person drives and watches the
robot -- but watching is now a check on the page, not the finish line.

---

## Where it stands: built backwards, on purpose

The car was the last thing started, not the first. Everything above it — the
contract the hardware will implement, the safety layer, the vision service, the
decision loop, the app — was built and proven against a simulated house first.
Swapping in real hardware is designed to be a configuration change -- the one
new file it needed, the code that talks to the motor board, has now been
written and tested against a software copy of that board.

| Phase | Milestone | What it did |
|---|---|---|
| **00** | A house that isn't real | A grid-world with a living room, a hallway, a kitchen — and a red backpack somewhere in it. |
| **01** | Teaching it to describe a room | Hand a photo to a vision model, get back a structured account of what's in it. |
| **02–03** | Deciding, and refusing | The look–think–decide–move loop, wrapped in a safety layer that can overrule it. |
| **04–06** | Searching rather than wandering | Remembering which rooms it has already been through, and preferring somewhere new. |
| **09** | Splitting brain from body | Thinking happens on one machine, moving on another, with an HTTP link between them — exactly the split the real car needs. |
| **10** | The digital twin, and Guide | A phone app that drives the simulation for real, plus the vision service in the cloud — and the first-person mode you can walk around with. |
| **B** | The car stops needing a laptop | The search itself became a small program that runs on the car. The phone starts it and then only watches — close the app and the car carries on. Three separate ways for it to stop itself, and a way to test each one from the phone. |
| **R0** | Moving like a real car | The simulated car stopped hopping between squares. It now drives and turns smoothly by spinning its two wheels, using the real chassis' measurements. |
| **R1** | Aiming instead of dithering | Turns are sized to where the target actually is, a search sweeps the room without blind spots, and a mission that is stuck against a wall gives up instead of pushing. It also recognises when it has arrived. |
| **R3–R4** | The standard robot toolkit | ROS 2, the toolkit most real robots use, now runs in one sealed-off box beside the project. The car's shape is described to it, and every wheel command can go through it -- with only one thing ever allowed to drive the wheels at a time. |
| **R5** | Drawing the map as it drives | The car builds its own floor plan from the lidar while it moves. In tests with deliberately faulty wheel sensors, the wheels alone ended up to a metre off; the map kept the car within a few centimetres. |
| **R6** | Driving to a spot on the map | Point at a place on the map and the car plans a route and drives there, keeping clear of walls. In a furnished copy of the owner's own house it reached eight of the nine rooms. |
| **R7** | A pretend motor board | The code that will talk to the real motor board was written from that board's own source code and tested against a software copy of it -- so hardware day is a settings change. |
| **11** | **Put it in the car — next** | Same brain, same safety rules, real motors and a real lidar. Say the object out loud; let it go and find it. |

---

## Next phase: what actually gets hard

On paper the hardware swap is small: point the configuration at real hardware
instead of the simulator. The one file that turns "drive forward" into motor
commands is already written. Nothing in the thinking layer has to change — that
was the point of building it this way.

The honest difficulties are elsewhere, and they're all things a simulation is too
kind about.

### A real room is not a grid

The simulated house has tidy square cells and walls in known places. A real floor
has chair legs, a rug edge, a cable, a cat. The lidar sees one flat slice of the
room at its own height: it catches the chair legs and misses a cable on the
floor or a table top above it.

### Looking costs money and time

Every glance is a paid call to a vision model, and each one takes a moment to
come back. A car moving while it waits is a car acting on a photo of where it
used to be. Slower and cheaper, or faster and dearer — that trade has to be
settled with a real chassis on a real floor.

### Being wrong at speed

In Guide, a bad instruction just means a person turns the wrong way and shrugs.
On a car with momentum, the same mistake ends against a skirting board. The
safety layer already exists for this; the next phase is where it stops being
theoretical.

---

## Plain terms: words you'll hear

| Term | What it means |
|---|---|
| **The car** | A small two-wheeled robot built around an NVIDIA Jetson Orin Nano Super, a small computer with a graphics chip that can run the target-spotting model on board: a camera that can pan left and right, and a spinning lidar that measures distance in every direction. The first plan used an off-the-shelf PiCar-X kit; it was swapped before purchase because that kit steers like a car and cannot turn on the spot. A later plan used a Raspberry Pi 5 with a separate AI chip; the Jetson replaced it in September 2026. |
| **Vision model** | An AI model that accepts an image and a question about it, and answers in words. Here it's asked things like "is a red backpack visible, and roughly where in this frame?" |
| **Digital twin** | A working stand-in for the real machine that you can drive and watch. Not a mock-up — it runs the same movement, sensing and safety code the car will. |
| **Lidar** | A spinning laser rangefinder. It measures the distance to the nearest thing at every angle around the car, many times a second — a floor plan's worth of distances from one small puck. It sees one flat slice of the room: chair legs, not chair seats. |
| **Safety veto** | The rule that lets the body overrule the brain. Every proposed move is checked against the distance reading before any wheel turns. |
| **Watchdog** | A timer on the body's side. If no command arrives for about a second, it stops the motors without asking anyone. |
| **Failsafe drill** | Deliberately breaking one thing to check the guard that should catch it. Available from the app, and only ever able to end with the car stopped. |
| **ROS 2** | The Robot Operating System: a widely used toolkit for robot mapping and route-planning. Here it is kept in one sealed-off box, so the rest of the project never depends on it directly. |
| **Grid-world** | The simulated house: rooms laid out on a coarse grid of squares, with doorways between them and objects placed in specific cells. |

---

## In short: a car that looks, then moves

The destination is a small robot you can point at a room and give a sentence to.
Everything built so far is the same loop wearing different bodies — a simulated
car to prove the idea, a phone in your hand to feel it, and next, the car itself.

The loop now runs where the car will be, stops itself three different ways, and
draws its own map as it goes. Every part of it has been measured, run after run,
in the simulated house before any of it touches a motor. That last clause is the
whole method.

---

*vision-picar — simulation-first build of a vision-driven robot car.
The engineering reference for the searching loop itself is
[`AGENT-HARNESS.md`](AGENT-HARNESS.md).
Phase 11 next: real hardware. See [`README.md`](../../README.md) for the engineering
detail behind each phase.*
