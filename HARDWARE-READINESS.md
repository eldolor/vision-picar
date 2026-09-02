# Hardware transition: what the PiCar-X kit actually changes

Status: explainer, written 2026-08-27 while deciding whether to buy the
kit. Nothing built. Companion to `PLAN-sim-hardening.md`, which covers
the simulation work worth doing first; this doc covers what the physical
robot changes about the architecture.

Kit under consideration: SunFounder PiCar-X AI Robot Smart Car Kit
(Raspberry Pi not included).

---

## 1. What you're actually buying

Most of the product listing is SunFounder's own software stack --
Openclaw, ChatGPT/Gemini/Grok integrations, TTS/STT, Scratch. **You will
use almost none of it.** This project already has its own brain
(`brain/`), its own vision service (`service/vision_analyze/`), and its
own safety layer (`robot/safety.py`). Running their AI stack on top would
be two robots fighting for the same actuators.

What's actually being bought is the **body**:

| Part | What this repo needs it for |
|---|---|
| Chassis + 2 drive motors | `drive_forward()`, `reverse()` |
| Front steering servo | `turn_left()` / `turn_right()` |
| Pan/tilt camera mount | `look_left()` / `look_right()` / `look_center()` |
| Ultrasonic sensor | `get_distance()` |
| Camera | `get_camera_frame()` |

That maps one-to-one onto `robot/interface.py`'s `RobotInterface`. The
only SunFounder software involved is their `picarx` Python library --
roughly `px.forward(speed)`, `px.set_dir_servo_angle(angle)`,
`px.stop()`, `px.ultrasonic.read()` -- as the guts of the one file still
to be written, `robot/hardware_robot.py`. Everything else in that listing
is a different project. (Check SunFounder's current API before writing
against those names.)

**One line in the listing genuinely matters: the steering servo.** It
confirms Ackermann steering -- the car steers with its front wheels like
a real car and *cannot spin in place*. `sim/grid_world.py` assumes it
can. See `PLAN-sim-hardening.md` section 3.3.

---

## 2. How the "Guide" tab aligns with the car

**As a feature, it doesn't -- and that's fine.** Guide and the robot are
mirror images:

| | Guide tab | Vision Autopilot |
|---|---|---|
| Camera | the phone's | the robot's |
| Who moves | **the person** | the robot |
| Output | an arrow on screen | `FORWARD` / `LEFT` / `STOP` |
| AWS route | `/guidance` | `/navigate` |

Same vision service, same schema family, opposite actuator. Buying the
car adds nothing to Guide, and Guide adds nothing to the car's driving.

### But they share more backend than "same service" implies

Worth being precise here, because it determines how much Guide's
real-world testing actually transfers to the robot.

`/guidance` and `/navigate` are siblings in the same file
(`service/vision_analyze/vision_core.py`), and the two functions are
near-copies:

| Layer | Shared? |
|---|---|
| ECS container (`service/vision_analyze/`) | same one |
| ALB routing | same -- the twin's `ListenerRule` only claims `/`, `/action`, `/stop`, `/distance`, `/frame`; both of these fall through to vision-analyze |
| Auth + upload | same `_decode_image()` -> same `_check_secret()`, same 5MB cap, same base64 decode (`app.py:138`) |
| Bedrock client | same `_get_client()`, same `MODEL_ID` |
| Model call | same `converse()`, same `maxTokens: 300`, same image-format conversion |
| Browser config | same "Cloud endpoint settings" URL + secret, same `deriveServiceUrl()` |

The **only** differences are the prompt template and the response parser:
`/navigate` returns a discrete verb (`FORWARD`/`LEFT`/`RIGHT`/`STOP`),
`/guidance` returns a 5-zone position + proximity + a human-readable
sentence + an optional `bounding_box`.

### What Guide does *not* touch

Guide never calls `robot/server.py`. Not `/action`, not `/frame`, not
`/distance`, not `/stop`. It never passes through `robot/safety.py`,
never triggers a veto, never resets the watchdog. There is no actuator in
its loop at all -- the person is the actuator.

```
                                   ┌─ shared from here down ─┐
  Guide:   phone camera ──────────►│                          │
                                   │  ALB → vision-analyze    │
                                   │  → Bedrock → Claude      │
  Autopilot: FPV render ──────────►│                          │
              │                    └──────────────────────────┘
              │                              │
              │                       action ▼
              └────────► robot/server.py ────┘   ← Guide never gets here
                         robot/safety.py
                         the motors
```

### Why this makes Guide the most valuable thing built so far

Guide is **the only part of this project ever tested against real
photographs.** Everything proven on the robot side was proven against a
flat-shaded raycaster render of a maze (`web-twin/index.html:1853`).
Guide has been run on an actual iPhone pointed at actual rooms, over
cellular, and it works.

Because the backend path is shared down to the model call, that testing
transfers concretely:

- **Measured latency and cost per call already exist** for the cloud leg,
  and carry over directly -- same model, same token budget, same image
  pipeline.
- **Ingress, auth, container, Bedrock, encode/decode, JSON parsing,
  error handling, and throttling** have all been exercised end to end
  from a phone on a real network.
- There is even an asymmetry in Guide's favor: today Autopilot sends a
  canvas raycaster render while Guide sends a real camera JPEG. On
  hardware, Autopilot starts sending real camera JPEGs too -- so
  **Guide's payload is a preview of the *hardware* payload, not the sim
  one.**

What none of this tells you is anything below the divergence point:
whether `min_distance_cm: 20` is survivable, how the car arcs through a
turn, whether the ultrasonic sees a sofa. That half has never run against
anything real -- see section 3 and `PLAN-sim-hardening.md` section 7.

---

## 3. How far to go before buying

**Do these first** (they are correctness work, not simulation work):

- **Phases S1-S4 of `PLAN-sim-hardening.md`** -- pin the interface
  contract, make `get_camera_frame()` return real image bytes, build the
  Python HTTP client, put time in the loop so the watchdog is actually
  tested. **All four are now built** -- S1 and S3 on 2026-08-28, S4 the
  same day, and S2 on 2026-08-31 (`sim/renderer.py`). S5 is built too,
  which `PLAN-sim-hardening.md` Q4 puts past the point where measuring
  beats modelling.
- **The cheap real-world test:** photograph real rooms with a phone and
  replay those JPEGs through `/navigate`, the same way Guide already
  replays them through `/guidance`. **No robot required.** This answers
  whether Claude can navigate from real photos -- and if it can't, no
  amount of hardware fixes that.

**Then buy.** Past that point the work shifts to modeling things that
could simply be measured: ultrasonic behavior on an actual sofa, actual
turning radius on actual carpet, real stopping distance. A ~$100 kit
measures those better than a week of simulator work does.

**Both prerequisites are met as of 2026-08-31**, so this is now the live
recommendation rather than a future one. Note the kit has shipping
latency that no other item here has, and section 5.3 -- whether the
ultrasonic pans with the camera -- is answerable only by looking at the
assembly diagram, decides whether the peek-based policy works at all, and
blocks nothing else. That makes ordering the highest-value action
available, and it can happen in parallel with everything below.

The genuinely unbuyable-around items: how far the car coasts between
"sensor says 20cm" and "motors stopped," and whether the ultrasonic sees
curtains at all. Both are in `PLAN-sim-hardening.md` section 7.

---

## 4. How the car talks to AWS

**Today, AWS is pretending to be the robot.** `robot/server.py` is
deployed to ECS Fargate with `mode: sim`, so the "robot" currently lives
in us-east-1.

**When the car arrives, `robot/server.py` moves out of AWS and onto the
Pi.** It has to, for two reasons.

**Reason 1 -- the watchdog.** The safety loop is 1 second
(`safety.watchdog_timeout_s`). A round trip to us-east-1 and back is
100-300ms on a good day and *unbounded* when Wi-Fi hiccups. If the safety
decision lives in AWS, a dropped packet means the car keeps driving. The
ultrasonic re-check in `robot/safety.py` has to happen inches from the
sensor.

**Reason 2 -- home NAT.** The Pi sits behind a home router. Nothing on
the public internet can open a connection *to* it. And this design is
pull-based (the brain calls the robot -- see `PLAN-sim-hardening.md`
section 1), so the caller must be on the LAN.

The shape after hardware:

```
  YOUR HOUSE                                      AWS
  ─────────────────────────────────────           ──────────────────────

  ┌─────────────────────────────┐
  │ PiCar-X + Raspberry Pi      │
  │   robot/server.py           │
  │   mode: hardware            │
  │   robot/safety.py  ◄── veto happens HERE, locally
  │   hardware_robot.py         │
  └──────────▲──────────┬───────┘
             │          │   home Wi-Fi, HTTP, milliseconds
   POST /action    GET /frame
             │          │
  ┌──────────┴──────────▼───────┐         ┌──────────────────────┐
  │ BRAIN (MacBook or browser)  │──JPEG──►│ /navigate            │
  │   decides what to do next   │◄─JSON───│ vision service       │
  └─────────────────────────────┘  ~1-3s  │ Bedrock → Claude     │
                                           └──────────────────────┘
```

(Where the brain itself should run -- MacBook or Pi -- is re-examined
in section 7; this section assumes the MacBook split the build plan
originally specified.)

**The car never talks to AWS. The brain does.** The car only ever talks
to the brain, over home Wi-Fi. What crosses the internet is one JPEG up
and one small JSON action down every ~2.5s -- which is exactly why the
Autopilot loop is throttled to 2.5s, and why safety must be local.

### What happens when a command arrives

One `FORWARD`, all the way down:

1. Brain POSTs `{"action":"FORWARD","speed":50,"duration":0.5}` to the Pi
   over LAN.
2. FastAPI on the Pi stamps `last_command_at` -- this is what keeps the
   watchdog quiet.
3. `SafetyController` **re-reads the real ultrasonic sensor.** It never
   trusts what the brain claims (`robot/safety.py`).
4. If under `min_distance_cm`: motors stop, HTTP 200 with
   `{"executed": false}`. The AI's decision is overruled by a sensor
   reading, locally, in milliseconds.
5. If clear: `HardwareRobot.drive_forward(50, 0.5)` -- roughly
   `px.forward(50)`, wait 0.5s, `px.stop()`.
6. A turn is where reality bites: `px.set_dir_servo_angle(-30)` *plus*
   forward motion. The car **arcs**. The sim pivots in place.

Steps 1-5 are already written and already tested. Step 5's guts are the
only genuinely new code.

### From a verb to the motors

`/navigate` returns a bare verb. Everything that turns that verb into
motor power happens in three files, and only the last one is unwritten:

```
/navigate returns  {"action": "FORWARD"}
   │
   ▼  web-twin/index.html:1495 -- sendAction()
POST /action  {"action": "FORWARD"}          ← the verb, and nothing else
   │
   ▼  robot/server.py:83 -- ActionRequest fills in the blanks
{"action":"FORWARD", "speed":50, "duration":0.5, "angle":90}
   │
   ▼  robot/safety.py:46 -- re-read the real sensor, veto if too close
   │
   ▼  robot/safety.py:60 -- dispatch_table  ← THE translation point
"FORWARD" → robot.drive_forward(50, 0.5)
"LEFT"    → robot.turn_left(90)
"STOP"    → robot.stop()
   │
   ▼  RobotInterface -- MockRobot today, HardwareRobot later
```

The dispatch table at `robot/safety.py:60` is a plain dict of lambdas
mapping verb -> `RobotInterface` method. It is entirely
backend-agnostic, which is why it doesn't change on hardware day. **All
the reality lands in `robot/hardware_robot.py`**, the one file still
unwritten.

What that file has to do, roughly, with the `picarx` library (verify the
current SunFounder API before writing against these names):

| Verb | Interface call | Real hardware |
|---|---|---|
| `FORWARD` | `drive_forward(50, 0.5)` | `px.forward(50)` -> sleep 0.5 -> `px.stop()` |
| `REVERSE` | `reverse(50, 0.5)` | `px.backward(50)` -> sleep 0.5 -> `px.stop()` |
| `LEFT` | `turn_left(90)` | **no such primitive** -- see below |
| `STOP` | `stop()` | `px.stop()` (cuts power; the car then coasts) |
| `LOOK_LEFT` | `look_left()` | `px.set_cam_pan_angle(-30)` |
| -- | `get_distance()` | `px.ultrasonic.read()` |

**There is no "turn 90 degrees" on a PiCar-X.** The hardware offers a
steering angle (`px.set_dir_servo_angle`, roughly +/-30 degrees) and
forward motion. So `turn_left(90)` must be implemented as: steer full
left, drive forward for T seconds, straighten. T is calibrated until the
heading change is about 90 degrees -- and T shifts with speed, floor
surface, and battery charge.

---

## 5. Pre-flight checklist before hardware day

Three things the current simulation structurally cannot surface. Each was
found by tracing the verb-to-motor path above; none is discoverable from
a passing test suite.

### 5.1 Every vision-driven move is hardcoded to speed 50 for 0.5s

The model returns only a verb. `web-twin/index.html:1495` posts only a
verb. `robot/server.py:83`'s Pydantic defaults (`speed=50`,
`duration=0.5`, `angle=90`) supply everything else. So the entire
autonomy stack moves in one fixed quantum.

In sim that quantum is exactly one 30cm cell. On hardware it is however
far the car actually rolls in half a second at speed 50 -- an unknown
number to be measured once and then treated as the sim's cell size.
Measure it before tuning anything else; several other constants
(`min_distance_cm`, the map scale in `PLAN-sim-hardening.md` section 3.4)
are only meaningful relative to it.

### 5.2 `LEFT` and `RIGHT` skip the distance check -- real safety gap

`robot/safety.py:27` reads:

```python
FORWARD_ACTIONS = {"FORWARD"}
```

so only `FORWARD` triggers the ultrasonic re-read before dispatch. That
is **correct for the simulation** -- `GridWorld` pivots in place, and a
pivot cannot hit anything.

It is **wrong for the hardware**. A real Ackermann turn *is* a forward
move, arcing perhaps 30-40cm ahead (see section 4). As written, turns
would drive the car forward with no obstacle check at all.

The fix is one line -- `FORWARD_ACTIONS = {"FORWARD", "LEFT", "RIGHT"}`
-- but **nothing in the current simulation can ever surface the need for
it**, because turning is free there. Apply it as part of writing
`hardware_robot.py`, not after the first collision. (`REVERSE` also skips
the check, but that is inherent: there is no rear sensor.)

### 5.3 Verify where the ultrasonic sensor is mounted

The entire frontier-preference exploration policy peeks by calling
`look_left()` and then `get_distance()`. That works in sim because
`GridWorld.distance_ahead()` casts its ray along `_view_heading()`, which
includes camera pan -- panning the camera changes the measured distance.

On the real kit that only holds **if the ultrasonic sensor sits on the
pan/tilt gimbal with the camera.** If it is fixed to the chassis instead,
`look_left(); get_distance()` returns the *forward* distance, every peek
returns the same number, and the exploration algorithm silently runs on
noise while appearing to work.

This could not be confirmed from this repo -- the build plan lives in the
Claude Project, not here. **Check the kit's assembly diagram before
hardware day.** It decides whether the policy works at all, and if the
sensor is chassis-mounted the fix is a real design change (turn the
chassis to peek, or add a second distance source), not a constant.

### 5.4 Do not carry the vision proximity veto onto the car

`brain/agent.py` has a `vision_proximity_veto` (off by default) that stops
a `FORWARD` when the model's own `distance_estimate` says
`within_one_step`. It exists **only** because `ReplayRobot` and
`TeleopRobot` have no distance sensor -- `get_distance()` returns
`robot/interface.py`'s `NO_SENSOR_CM` -- so on those backends
`robot/safety.py`'s veto is dead code and a whole recorded walk says
nothing about collision avoidance.

`HardwareRobot` will have a real ultrasonic, which makes the veto both
unnecessary and actively wrong there. Two reasons, and the second is the
one that bites:

1. **A measurement must win over a guess.** The veto already returns
   early whenever `get_distance()` reports anything below `NO_SENSOR_CM`,
   so on the car it would be inert -- but leaving it wired reads as
   "vision helps with obstacles", which invites someone to trust it.
2. **The signal is not calibrated and the models disagree wildly.** On
   identical frames one model reports `obstacle_ahead` ~100% of the time
   and another ~0%; `distance_estimate` says `within_one_step` on 60% of
   real walk frames. On hardware the ultrasonic is the obstacle sensor.
   Do not let the vision policy be the thing relying on either field.

Nothing needs doing on hardware day except *not* copying it into
`robot/hardware_robot.py`. It is listed here because the tempting move --
"we already have obstacle logic, reuse it" -- is the wrong one.

---

## 6. Two gotchas worth knowing now

**Mixed content.** `web-twin/README.md` already documents this for the
Guide tab, but it becomes structural with a real Pi: an HTTPS page (the
CloudFront URL from `cloudformation/cdn.yaml`) is **blocked by the
browser** from calling `http://192.168.1.50:8000`. So it's one or the
other per page load:

- Load the twin **from the Pi** over LAN HTTP -> driving works, Guide's
  camera does not (`getUserMedia` needs a secure context).
- Load the twin **from CloudFront** over HTTPS -> Guide works, driving
  the Pi does not.

Fixable (self-signed cert on the Pi, or Tailscale), but worth deciding
before hardware day rather than while holding a robot.

**The AWS twin deployment doesn't die, it changes job.** It stops being a
fake robot and becomes (a) the page host and (b) the vision service.
Worth keeping a `mode: sim` instance running there permanently as a demo
that works without unpacking the car.

---

## 7. Where should the brain live?

`README.md` and `CLAUDE.md` both describe the split as "Raspberry Pi
(robot runtime) + a MacBook running a Vision LLM (high-level
reasoning)." That split was correct when it was written. It is worth
re-deciding before hardware day, because the premise behind it has since
changed.

### The original reasoning

A Raspberry Pi cannot run a vision model; a MacBook can. So: heavy
thinking on the laptop, motors and sensors on the Pi, Wi-Fi between them.
A sound edge/compute split -- **if the brain is doing real compute.**

### It isn't, any more

Vision moved to Bedrock (`service/vision_analyze/`). What `brain/`
actually does on hardware is four HTTP calls in a loop:

1. `GET /frame` from the Pi
2. POST the image to AWS
3. receive `{"action": "FORWARD"}`
4. `POST /action` to the Pi

That is coordination, not computation. The whole of `requirements.txt` is
`pyyaml pytest anthropic fastapi uvicorn pydantic httpx` -- no torch, no
opencv, not even numpy. `brain/`'s imports are stdlib plus `anthropic`,
which is an HTTP client. A Pi Zero would run this without noticing.

**The reason for the MacBook was "the Pi is too weak to think," and the
Pi no longer has to think -- Bedrock does.**

### What the split costs now

Because the brain *pulls* from the Pi (section 1), it has to sit on the
LAN. That one constraint is the root of most of the friction elsewhere in
this document:

- The image travels Pi -> MacBook (LAN) -> AWS. An extra hop in a loop
  already running at 1-3s.
- The robot only works when the laptop is on, awake, and on the same
  Wi-Fi.
- It is the direct cause of the NAT and mixed-content problems in
  section 6.

Put the brain on the Pi and all three dissolve: the Pi calls AWS
**outbound**, which passes through home NAT without ceremony, and the
robot becomes self-contained.

### The reframe

"Brain on the MacBook" conflates two jobs that the web twin already
separates in practice:

| | Where it *can* live | Where it *should* live |
|---|---|---|
| **Autonomy loop** -- decide the next action | anywhere | the Pi |
| **Operator console** -- watch, drive manually, reset | a screen in your hand | your phone |

Today the browser does both at once. On hardware they want to be apart.

Note the brain's location is not actually settled in the code either --
there are already three implementations of it (`brain/agent.py` in
Python and in-process, the browser's JS frontier explorer, and the
browser's Vision Autopilot), with `brain/planner.py` a possible fourth.

### What this does to the watchdog

`robot/server.py`'s watchdog exists to detect a *dead brain*: if no
command arrives for `watchdog_timeout_s`, stop the motors. Moving the
brain onto the Pi narrows what it can detect, and how much depends on
*how* it is moved:

- **Brain as a separate process on the Pi**, talking to
  `robot/server.py` over localhost HTTP: the watchdog still catches a
  brain crash, hang, or deadlock -- commands genuinely stop arriving. It
  only stops catching *network* failure, since localhost does not drop.
  Most of its value survives.
- **Brain inside the server process** as an asyncio task: largely
  defeated. A synchronous block in the agent loop blocks the event loop
  the watchdog polls on.

This is a strong argument for the separate-process design -- see
`PLAN-brain-relocation.md`.

Independently of brain location, the watchdog keeps one job that matters
on hardware and has no equivalent in sim: **if a movement call sets the
motors and then crashes before `px.stop()`, the motors keep running.**
Nothing else in the system catches that.

The genuinely new gap is that **nothing guards the AWS link.** The
browser's Vision Autopilot currently catches a `/navigate` failure,
logs it, and schedules the next tick (`web-twin/index.html`, in
`visionAutopilotStep`'s `catch`) -- it does not issue a STOP. That is
survivable today only because motion is discrete and brief: each command
moves for 0.5s and stops on its own, so a hung brain leaves a stationary
car. It is worth making explicit rather than leaving as an accident of
timing.

### Recommendation

Move the autonomy loop onto the Pi as its own process; keep the MacBook
path for development, where it is genuinely better (editing, `pytest`,
breakpoints); make the phone a pure observer; keep the motor watchdog and
add a vision-link failsafe.

### How that gets built

Full detail in `PLAN-brain-relocation.md`; the shape of it:

**The whole trick.** If the brain is *always* an HTTP client of
`robot/server.py`, then "brain on the Pi" and "brain on the MacBook"
differ only by a base URL:

```
on the Pi        brain -> http://localhost:8000     (robot/server.py)
on the MacBook   brain -> http://192.168.1.50:8000  (same server, over LAN)
```

Same code, same process model, same tests. Where the brain runs stops
being an architectural question and becomes a deployment one. Everything
else exists to make that true.

**Target topology -- two processes on the Pi, not one:**

```
  THE PI                                             AWS
  ─────────────────────────────────────              ─────────────────
  ┌──────────────────────────────┐
  │ :8001  control/brain_server  │  ── JPEG ──────►  /navigate
  │        the autonomy loop     │  ◄── action ───   vision service
  │        (RemoteRobot client)  │
  └───────────┬──────────────────┘
              │ localhost HTTP
              ▼
  ┌──────────────────────────────┐
  │ :8000  robot/server.py       │
  │        safety + watchdog     │
  │        hardware_robot.py     │
  └──────────────────────────────┘
              ▲            ▲
              │ manual     │ mission start/stop + status
        ┌─────┴────────────┴─────┐
        │  PHONE (web twin)      │   observer + operator console
        └────────────────────────┘
```

**Phases:**

| | What | Proof it worked |
|---|---|---|
| **B0** | `RemoteRobot` -- an HTTP client implementing `RobotInterface`. This is Phase S3 of `PLAN-sim-hardening.md`, and the only hard prerequisite. | Identical action sequences in-process vs. over HTTP |
| **B1** | `MissionRunner` -- turn `ObjectSearchAgent.run_mission()`'s blocking `for` loop inside out into `start()`/`stop()`/`tick()`/`status()`. No decision logic moves. | Driving `tick()` in a loop reproduces `demo_active_search.py`'s result |
| **B2** | `control/brain_server.py` on :8001 -- `POST /mission/start`, `POST /mission/stop`, `GET /mission/status`. Talks to the robot only through `RemoteRobot`. | Start a mission over HTTP, poll to completion, backpack found |
| **B3** | Three failsafes for three distinct failures: motors-left-running (the existing watchdog, kept), AWS-link-dead (new), brain-loop-hung (new). | A stub vision function that fails or hangs ends the mission with the robot stopped |
| **B4** | Twin becomes an observer: Explore/Find POST to the brain and render polled status, instead of running JS timers. Manual D-pad still talks straight to the robot. | Start a mission from the phone, background the tab, robot keeps going |
| **B5** | Two `systemd` units on the Pi, brain ordered after robot. | Reboot the Pi; a mission runs with no laptop on the network |

**Why two processes and not one.** Running the loop as an asyncio task
inside `robot/server.py` is less code and wrong here: a synchronous block
in the agent loop would block the event loop the watchdog polls on
(defeating it), it merges the two roles `CLAUDE.md` section 2 names as
the constraint not to break, and it loses the base-URL trick above. The
cost of separating them is a localhost round trip -- about a millisecond,
against a loop that spends 1-3 seconds waiting on Bedrock.

**What is testable now.** B1 through B3 need no hardware and no Pi --
they run against `mode: sim` today. B4 needs two local `uvicorn`
processes. Only B5 needs the robot. Together with
`PLAN-sim-hardening.md`'s S1-S4, this is the bulk of the pre-purchase
work remaining.

**The single clearest test** is B4's: start a mission from your phone,
then background the tab. If the robot keeps going, the brain has moved.
Today it stops -- there is an explicit `visibilitychange` handler that
kills the autopilot timer.
