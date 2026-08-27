# Teaching a Car to Look

**vision-picar — project introduction**

A small robot car you can send to find something. Not by giving it a map or a
route — by letting it look at the room, the way you would, and work out where to
go next.

- SunFounder PiCar-X
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

| | **A simulated car** | **You, on foot** | **The real PiCar-X** |
|---|---|---|---|
| **Status** | Working today | Working today | **Next phase** |
| **Eyes** | A drawn first-person view of a make-believe house | Your phone's actual camera | The camera bolted to the car |
| **Brain** | The vision model, called for real | The vision model, called for real | The vision model, called for real |
| **Body** | A car that exists only as software | Your legs | Two motors and a steering servo |

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

## Where it stands: built backwards, on purpose

The car was the last thing started, not the first. Everything above it — the
contract the hardware will implement, the safety layer, the vision service, the
decision loop, the app — was built and proven against a simulated house first.
Swapping in real hardware is designed to be a configuration change plus one new
file.

| Phase | Milestone | What it did |
|---|---|---|
| **00** | A house that isn't real | A grid-world with a living room, a hallway, a kitchen — and a red backpack somewhere in it. |
| **01** | Teaching it to describe a room | Hand a photo to a vision model, get back a structured account of what's in it. |
| **02–03** | Deciding, and refusing | The look–think–decide–move loop, wrapped in a safety layer that can overrule it. |
| **04–06** | Searching rather than wandering | Remembering which rooms it has already been through, and preferring somewhere new. |
| **09** | Splitting brain from body | Thinking happens on one machine, moving on another, with an HTTP link between them — exactly the split the real car needs. |
| **10** | The digital twin, and Guide | A phone app that drives the simulation for real, plus the vision service in the cloud — and the first-person mode you can walk around with. |
| **11** | **Put it in the car — next** | Same brain, same safety rules, real motors and a real ultrasonic sensor. Say the object out loud; let it go and find it. |

---

## Next phase: what actually gets hard

On paper the hardware swap is small: point the configuration at real hardware
instead of the simulator, and write the one file that turns "drive forward" into
motor commands. Nothing in the thinking layer has to change — that was the point
of building it this way.

The honest difficulties are elsewhere, and they're all things a simulation is too
kind about.

### A real room is not a grid

The simulated house has tidy square cells and walls in known places. A real floor
has chair legs, a rug edge, a cable, a cat. The distance sensor sees one narrow
cone straight ahead and knows nothing about the table leg to the left.

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
| **PiCar-X** | An off-the-shelf robot car kit built around a Raspberry Pi — two drive motors, steering, a camera on a small pan-tilt mount, and an ultrasonic distance sensor. |
| **Vision model** | An AI model that accepts an image and a question about it, and answers in words. Here it's asked things like "is a red backpack visible, and roughly where in this frame?" |
| **Digital twin** | A working stand-in for the real machine that you can drive and watch. Not a mock-up — it runs the same movement, sensing and safety code the car will. |
| **Ultrasonic sensor** | A small emitter that measures distance by timing an echo, like a bat. Cheap and reliable, but it only sees a narrow cone directly ahead. |
| **Safety veto** | The rule that lets the body overrule the brain. Every proposed move is checked against the distance reading before any wheel turns. |
| **Grid-world** | The simulated house: rooms laid out on a coarse grid of squares, with doorways between them and objects placed in specific cells. |

---

## In short: a car that looks, then moves

The destination is a small robot you can point at a room and give a sentence to.
Everything built so far is the same loop wearing different bodies — a simulated
car to prove the idea, a phone in your hand to feel it, and next, the car itself.

---

*vision-picar — simulation-first build of the PiCar-X vision agent.
Phase 11 next: real hardware. See [`README.md`](README.md) for the engineering
detail behind each phase.*
