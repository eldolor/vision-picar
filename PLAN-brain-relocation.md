# Plan: move the autonomy loop onto the Pi

Status: **B0-B4 built 2026-08-27** (`control/`, the twin's remote-brain
panel, and the five test files named below). B5 remains a proposal -- it
needs the Pi. Written 2026-08-27.
Decision and reasoning: `HARDWARE-READINESS.md` section 7.
How the built thing actually works: `AGENT-HARNESS.md` (this document is
the plan; that one is the reference).
Answers Q2/Q3 in `PLAN-sim-hardening.md` section 5.

**Goal.** The robot is self-contained: it runs its own autonomy loop,
calls AWS outbound, and needs no laptop. The phone becomes an observer
that can watch, take manual control, and start/stop missions. The
MacBook keeps the development path unchanged.

**The whole trick.** If the brain is *always* an HTTP client of
`robot/server.py`, then "brain on the Pi" and "brain on the MacBook"
differ only by a base URL:

```
on the Pi        brain -> http://localhost:8000     (robot/server.py)
on the MacBook   brain -> http://192.168.1.50:8000  (same server, over LAN)
```

Same code, same process model, same tests. Where the brain runs stops
being an architectural question and becomes a deployment one. Everything
below exists to make that true.

---

## Target topology

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
              │            │
        ┌─────┴────────────┴─────┐
        │  PHONE (web twin)      │   observer + operator console
        └────────────────────────┘
```

Two processes, not one. The separation is load-bearing -- see "Why not
one process" below.

---

## Phase B0 -- Prerequisite: `RemoteRobot` -- BUILT

This is **Phase S3 of `PLAN-sim-hardening.md`**, unchanged; it is listed
here only to mark the dependency. Nothing below can start until
`control/remote_robot.py` exists and passes the identical-action-sequence
test (in-process vs. over HTTP).

**Built.** `control/remote_robot.py`; `tests/test_remote_robot.py` asserts
the identical action sequence against a live `uvicorn` (83 steps, same
actions) and again in-process over ASGI. Two details turned out to be
load-bearing for transparency and are documented in the module: a
server-side safety veto has to re-raise `SafetyViolation` (over HTTP it
arrives as a 200 with `executed: false`), and `position` has to be coerced
back to a tuple after JSON turns it into a list, since `MissionAgent` keys
a set with it.

---

## Phase B1 -- Extract the mission runner from the agent -- BUILT

**Problem.** `ObjectSearchAgent.run_mission()` is a blocking `for` loop
that returns only when finished. It cannot be started, stopped, or
inspected from outside, so nothing can drive it over HTTP.

**Build.** A `MissionRunner` owning one mission's lifecycle: target,
step budget, the agent, the memory, and a status snapshot. Expose
`start()`, `stop()`, `status()`, and a single-step `tick()`, with the
loop driven from outside rather than owned inside. No decision logic
moves -- this is purely turning a `for` loop inside out.

**Files.** New `control/mission_runner.py`; `brain/agent.py` gains
nothing (leave `run_mission()` in place for the demo scripts and tests).

**Test.** `tests/test_mission_runner.py`: drive a runner to completion by
calling `tick()` in a loop against `MockRobot`, and assert the result
matches `demo_active_search.py`'s (found in the kitchen, comparable step
count). Assert `stop()` mid-mission leaves `status()` reporting a
stopped, non-complete mission.

**Built, with two corrections to the above.** The reference hunt finds the
backpack **in the hallway at step 82** (83 steps), not in the kitchen --
the test asserts against the demo's actual result, computed in the test
rather than hard-coded. And the step count is *exactly* equal, not merely
comparable, which is a stronger claim worth keeping: any drift means
decision logic moved when it should not have. Consequence: the default
`max_steps` is 120, since the 80 suggested below ends four steps short of
the only mission we can currently verify.

One thing the phase description missed. A `tick()` already in flight when
`stop()` lands cannot be interrupted -- it is a blocking call on another
thread -- so it would happily dispatch the move it had already decided on
*after* the operator's stop. The agent therefore drives the robot through
a gate (`_HaltGate`) that refuses movement and pan commands the moment the
mission ends, while always letting `stop()` and sensor reads through. This
is what makes item 6 of the definition of done true rather than
approximately true.

---

## Phase B2 -- The brain service -- BUILT

**Build.** `control/brain_server.py` -- a small FastAPI app that owns a
`MissionRunner` and drives it as an asyncio background task. It talks to
the robot exclusively through `RemoteRobot`, so it neither imports `sim/`
nor knows what backend is underneath.

Endpoints:

```
POST /mission/start   {"target_object": "red backpack", "max_steps": 80,
                       "policy": "frontier" | "vision"}
POST /mission/stop    always available, always stops the robot too
GET  /mission/status  {running, step, target, found, last_action,
                       last_reasoning, rooms_visited, log_tail}
GET  /health
```

Config: the robot's base URL and the vision service URL/secret, from
`config/robot.yaml`. Reuse `robot/server.py`'s `require_secret()` pattern
verbatim -- this process will be reachable on the LAN too.

**Files.** New `control/brain_server.py`; `config/robot.yaml` (a `brain:`
block -- `robot_url`, `vision_url`, `max_steps`); `requirements.txt`
unchanged (`httpx` is already there).

**Test.** `tests/test_brain_server.py`, FastAPI `TestClient` against a
live `robot/server.py` in `mode: sim`: POST a mission, poll status until
complete, assert the backpack is found. Assert `/mission/stop` halts a
running mission and that a second `/mission/start` while running is
rejected rather than racing.

**Note.** `POST /mission/stop` must call the robot's `/stop` as well as
halting the loop. Stopping the *thinking* is not stopping the *car*.

**Built.** `control/brain_server.py`, plus `control/brain_config.py` --
the `brain:` block is read without importing `robot/factory.py`, which
would drag the robot runtime and the simulator into the brain process.
`require_secret()` is duplicated rather than imported for the same reason
(importing `robot/server.py` instantiates a backend at module scope).
`tests/test_brain_server.py` runs a whole mission through two HTTP hops
(TestClient -> brain -> `RemoteRobot` -> live `uvicorn`) and asserts the
import surface in a subprocess. `policy: "vision"` answers **501** until
S2b exists -- silently running the rule-based policy under a "vision"
label would be the worst possible outcome of this endpoint.

---

## Phase B3 -- Failsafes -- BUILT

Three distinct failures, three distinct guards. Today only the first
exists.

**B3.1 Motors left running (keep as-is).** `robot/server.py`'s watchdog.
Its meaning narrows once the brain is on localhost, but it keeps the job
that actually matters on hardware: if a movement call energises the
motors and then crashes before `px.stop()`, nothing else catches it.
Update the module docstring to say this, since "detects a dead MacBook"
stops being the primary description.

**B3.2 The AWS link (new).** The brain must treat vision failure as a
reason to stop, not a reason to retry silently. Add to `MissionRunner`: a
per-call timeout, and a consecutive-failure budget (suggest 3) after
which it issues `STOP` and ends the mission with a failure status. This
closes the gap noted in `HARDWARE-READINESS.md` section 7 -- the browser
currently logs a `/navigate` error and schedules the next tick.

**B3.3 A hung brain loop (new).** The brain service's own dead-man: if
`tick()` has not completed within N seconds, stop the robot and mark the
mission failed. Distinct from B3.1, which cannot see a loop that is alive
but stuck.

**Files.** `control/mission_runner.py`, `control/brain_server.py`,
`robot/server.py` (docstring only).

**Test.** `tests/test_failsafes.py` with a stub vision function: (a) three
consecutive failures -> mission ends, robot received a `STOP`; (b) a
vision call that hangs past the timeout -> same; (c) a healthy mission is
untouched by either guard.

**Built**, all three, plus: a single failure is survivable (the car stops
for it, the counter resets on the next success), and no movement command
reaches the robot after a stop. The hang timeout runs the vision call on a
**daemon** thread rather than in a pooled executor -- an abandoned hung
call must not block interpreter exit, and must not sit in a worker queue
ahead of the next call. B3.1 needed only the docstring change; its
decision logic stays tested in `tests/test_server.py`, and exercising the
async polling loop is still Phase S4's job.

---

## Phase B4 -- The twin becomes an observer -- BUILT

**Build.** Add a "Brain" connection field alongside the existing robot
and vision-service ones. Explore / Find / Autopilot POST to
`/mission/start` and render from polled `/mission/status` instead of
running their own JS timers. The manual D-pad keeps talking straight to
`robot/server.py` -- manual control must not depend on the brain being
up.

Keep the existing in-browser JS loop behind a "local brain" toggle. It is
already validated, it is the only thing that works with no Pi present,
and it stays useful for LAN development.

**Built, with one change of shape.** The two brains are *two labelled
panels*, not one set of buttons behind a toggle: "Remote brain -- runs on
the robot" and "Local brain -- runs in this tab". A toggle would have
hidden which one was about to drive, at the exact moment that is the only
thing worth knowing. Only one may drive at a time, enforced from both
ends -- starting a remote mission stops the local loops, starting a local
loop during a remote mission is refused with a toast, and a second
`/mission/start` still gets the server's 409.

Also built here, because the phase would otherwise be unverifiable rather
than merely unbuilt: a **failsafe drill picker** (`control/drills.py`)
and a **watchdog readout**. B3.2 and B3.3 cannot be provoked by pressing
anything -- you would have to unplug the internet at the right moment --
so the twin can now ask the brain to break exactly one thing and watch
the real guard fire. See `CLAUDE.md` section 7 for the standing rule this
established.

**One config change fell out of it.** `brain.tick_interval_s` is 0.25 in
`config/robot.yaml` rather than 0. With no pacing the sim runs a whole
83-step mission in about half a second -- correct, and useless to watch,
because `MockRobot._settle()` is a no-op until Phase S4. The library
default stays 0 so tests are not slowed by a display concern.

**Files.** `web-twin/index.html`; `web-twin/README.md`.

**Test.** Manual, against a live Pi (or two local uvicorns): start a
mission from the phone, background the tab, confirm the robot keeps
going -- **this is the observable proof that the brain has actually
moved.** Today, backgrounding the tab halts autonomy (there is an
explicit `visibilitychange` handler that stops the autopilot timer).

**As built:** the remote-brain poll is deliberately excluded from that
`visibilitychange` handler, which still governs the local brain and
Vision Autopilot (both of which spend money per tick, and both of which
die with the tab anyway). Backgrounding now pauses the *observer* and
nothing else. Verified with two local uvicorns; still to run against a
Pi, which is B5.

**Documentation debt -- paid.** `CLAUDE.md` section 6's bullet now says
the browser is the *optional* brain and describes the two-panel
arrangement; `web-twin/README.md` gained an architecture section on the
two brains and a table of what each drill proves.

---

## Phase B5 -- Deployment

**Build.** Two `systemd` units on the Pi (`vision-picar-robot.service`,
`vision-picar-brain.service`), the brain unit ordered after the robot
unit, both with `Restart=on-failure`. Document the one-line switch
between brain-on-Pi and brain-on-Mac (point `brain.robot_url` at the Pi's
LAN IP and run `control/brain_server.py` on the Mac -- no code change).

**Files.** New `deploy/` with both unit files and a short README; update
`CLAUDE.md`'s repo map.

**Test / UI proof.** Reboot the Pi; both services come up; a mission can
be started from a phone with no laptop involved anywhere. This is
definition-of-done item 1, and it is a UI test by construction -- there
is nothing else it could be.

---

## Interim: brain on ECS Fargate

Status: **built 2026-08-28**, pre-hardware. Not a phase of the plan above
-- it doesn't move the target topology, and B5 (brain-on-Pi) is still the
destination. This exists only because running `control/brain_server.py` on
the MacBook and reaching it from a phone off the home LAN needs an HTTPS
tunnel (the twin is served over HTTPS from the ALB; a browser refuses to
call an `http://` LAN/localhost brain from an HTTPS page as mixed content),
which is tedious enough to be worth a throwaway deployment instead.

**Why this doesn't conflict with "brain on the Pi, not AWS" (§7 of
`HARDWARE-READINESS.md`).** That recommendation is about hardware day: the
brain has to be physically next to the motors for the watchdog and the
failsafes to mean what they claim. Pre-hardware, `robot/server.py` runs in
`mode: sim` -- there is no physical anything to be near. "The brain is
always an HTTP client of `robot/server.py`, differing only by base URL" is
exactly the property that makes a third deployment target (Fargate, next
to MacBook and Pi) a config change, not an architecture change.

**What's deployed.** `service/brain/` (Dockerfile + requirements.txt,
mirroring `service/twin/`'s "build from repo root" pattern -- pulls in
`brain/`, `control/`, `robot/` (interface + safety only, matters for what's
*imported*, not what's copied), `config/`; deliberately not `sim/`) and
`cloudformation/brain.yaml` (mirrors `twin.yaml`: separate ECR repo, IAM
roles, secret; a new `ListenerRule` on the `vision-picar-service` stack's
shared ALB listener, matching the "third public service" guidance in
`CLAUDE.md` section 6 -- new target group, same NLB/ALB, no second load
balancer).

**Two real wiring gaps this surfaced, both hit and fixed during the actual
deploy, not anticipated in advance:**

1. **One shared secret assumed in two places.** `control/brain_server.py`
   reused its own inbound `APP_SHARED_SECRET` as the outbound secret to
   *both* the robot and the vision service, and the twin's own UI does the
   same thing on the frontend (`web-twin/index.html`'s brain panel sends
   the robot connection's secret to the brain, documented in its own
   Settings disclosure text) -- both harmless when there is genuinely one
   shared secret for the whole system (a Pi or a MacBook talking to
   services you also control end to end), both wrong once the twin and
   vision-analyze each have their own independently-generated Secrets
   Manager secret. Fixed on the backend with two new env vars,
   `ROBOT_SHARED_SECRET` / `VISION_SHARED_SECRET`, each defaulting to
   `APP_SHARED_SECRET` when unset -- so local/Pi/MacBook behavior is
   unchanged. Fixed on the frontend side without touching
   `web-twin/index.html` at all: rather than add a third secret field
   (which the UI's existing convention doesn't have and every other
   touchpoint -- URL params, the setup QR, localStorage -- would need
   updating for), `vision-picar-brain-shared-secret`'s value was set to
   match `vision-picar-twin-shared-secret`'s, so the UI's existing
   "reuses the robot connection's secret" behavior is simply true again.
2. **Referencing another stack's secret by ARN is less obvious than it
   looks.** Two things were tried and both failed before this worked:
   a wildcard-suffixed `Resource` pattern in the IAM policy (e.g.
   `...:secret:vision-picar-twin-shared-secret*`) plus a `ValueFrom` built
   as a suffix-less ARN string -- fails with `AccessDeniedException`,
   because ECS's Secrets Manager authorization check runs against the
   *literal* `SecretId` string as given, before any name-to-ARN
   resolution, so the wildcard never gets the chance to match. Swapping
   `ValueFrom` to a bare secret name (no `arn:` prefix) fixed the fetch in
   isolation but not the authorization check, which still evaluates the
   same way. What actually works: the exact full ARN, suffix included,
   for both the IAM policy's `Resource` and the task definition's
   `ValueFrom` -- passed as `cloudformation/brain.yaml` parameters
   (`RobotSecretArn`/`VisionSecretArn`, since neither `twin.yaml` nor
   `service.yaml` exports its secret's ARN as an `Fn::ImportValue`), found
   with `aws secretsmanager describe-secret --secret-id <name> --query
   ARN`. This only goes stale if one of those secrets is ever deleted and
   recreated -- an ordinary `put-secret-value` rotation keeps the same ARN.

**Config.** `control/brain_config.py` gained `ROBOT_URL` / `VISION_URL`
env-var overrides (same pattern as `service/vision_analyze/app.py`'s
`ALLOWED_ORIGINS`) so one image serves every deployment target --
`config/robot.yaml`'s `brain.robot_url`/`brain.vision_url` stay pointed at
`localhost` for Pi/MacBook dev. **Both point at the shared INTERNAL ALB,
not the internet-facing NLB** -- tried the NLB DNS name first, and a
mission call hung indefinitely: the private subnets have no route to the
internet at all (`network.yaml` has no NAT Gateway, only VPC endpoints for
the specific AWS services those tasks need), and an NLB's DNS name
resolves to public IPs, which are simply unreachable from inside those
subnets. The internal ALB sits inside the VPC and needed one small,
additive change to `service.yaml` to expose it -- a new exported output,
`InternalAlbDnsName` (`!GetAtt InternalAlb.DNSName`), since it wasn't
previously needed by anything outside that stack.

**Persistent storage, and a deliberate split.** Fargate's own filesystem is
wiped on every task replacement, so a walk recorded before EFS existed was
destroyed by the next redeploy. `cloudformation/recordings.yaml` (added
2026-08-28) owns one EFS filesystem for `brain.recording_dir`, mounted by
two independently deployed services: `vision-picar-brain` (write path only
-- `POST /recording/frame`, tightly coupled to a live Robot-view session)
and `vision-picar-admin` (`control/admin_server.py` -- list/view/delete,
plus the `/admin` page). They were bundled into brain_server.py at first,
then split out once it was clear reviewing recorded data has no reason to
share the brain's fate: it doesn't need to move to the Pi when B5 lands,
and it shouldn't go down every time the mission server restarts. The EFS
resource itself lives in a third stack rather than either service's,
specifically to avoid the circular cross-stack dependency that owning it
in brain.yaml (or admin.yaml) would create once both need to mount it.
Each service also got its own independent secret in the split (previously
one secret covered both) -- reviewing/deleting recordings is a different
privilege than starting a mission.

**What this does NOT change.** `allow_drills` and `allow_recording` stay
on by default in `config/robot.yaml`, which is now genuinely wrong for an
internet-reachable deployment -- the brain config docstring already flags
this ("turn this off if the brain is ever reachable beyond the LAN"), and
it applies here. Set `BRAIN_ALLOW_DRILLS=false` at deploy time once
someone other than the owner could plausibly reach this URL, or accept the
risk for a single-operator interim deployment and revisit before sharing
the link. (No env-var override for these two exists yet --
`config/robot.yaml`'s values are still what ships in the image. Add one
the same way `ROBOT_URL`/`VISION_URL` were added, if this stops being a
single-operator deployment.)

**Definition of done.** A mission can be started from a phone on cellular
data (no LAN, no tunnel) against the sim-mode twin, driven by this Fargate
brain, with the same outcome as the local two-uvicorn setup. Torn down (or
left at `DesiredCount: 0`) once B5 lands and `ROBOT_URL`/`VISION_URL` point
at the Pi instead.

**`control/admin_server.py` -- built (2026-08-28):** stats (`GET /stats`),
per-walk labels (`good`/`bad`/`training-ready`, `PUT .../tag`), zip export
(`GET .../download`), and the admin page's search/sort/bulk-delete/
slideshow. **Not built, deliberately deferred -- pick up here when the RL
pipeline is actually being built:**

- **S3 export.** A button (or scheduled job) moving a walk from the live
  EFS volume to S3 once it's tagged `training-ready`, so training tooling
  reads from cheap, toolable S3 rather than mounting EFS. See "why EFS and
  why not S3" earlier in this session -- EFS was right for the live write
  path, S3 is right for the training-data side. AWS DataSync or a plain
  scheduled `aws s3 sync` are both reasonable; don't have the write path
  (`POST /recording/frame`) target S3 directly.
- **Face/PII redaction.** An in-browser blur/crop tool on individual
  frames in `control/admin.html`, so a walk that accidentally captured a
  bystander's face (this happened during testing -- see the frame in
  `red-backpack-20260828-065910`, deleted after) can be cleaned up and kept
  rather than the whole walk discarded. Real client-side image editing,
  not a quick add -- estimate before starting rather than bolting it onto
  an unrelated task.

---

## Why not one process

Running the loop as an asyncio task inside `robot/server.py` is less
code, and wrong here for three reasons:

1. **It defeats the watchdog.** A synchronous block in the agent loop
   blocks the event loop the watchdog polls on. As two processes, a hung
   brain still stops sending commands and still trips it.
2. **It merges the two roles the project has kept apart since Phase 0.**
   `robot/server.py` is the robot runtime; deciding what to do next is
   the brain. `CLAUDE.md` section 2 names this the single most important
   constraint not to break.
3. **It loses the base-URL trick.** The one-line move between Pi and
   MacBook only works if the brain is an HTTP client of the robot in
   every deployment, including on the Pi.

The cost is a localhost HTTP round trip per call -- roughly a millisecond
against a loop that spends 1-3 seconds waiting on Bedrock.

---

## Definition of done

Status after B0-B4 (2026-08-27). Only item 1 needs hardware; everything
else is met, in simulation or with two local uvicorns.

1. **Blocked on hardware (B5).** A mission runs to completion with **no
   laptop on the network** -- started from a phone, robot and brain both
   on the Pi.
2. **Met with two local uvicorns; owed against a Pi.** Backgrounding or
   closing the phone's browser does not stop an in-flight mission. The
   remote-brain poll is outside the `visibilitychange` handler, and
   reopening the page picks the running mission back up.
3. **Met.** `control/brain_server.py` imports nothing from `sim/` and
   nothing from `robot/` except `RemoteRobot` and the interface --
   asserted in a subprocess by
   `test_the_brain_process_never_loads_the_simulator`. (`robot.safety`
   comes along with the `SafetyViolation` type, which is part of that
   interface's contract; no backend and no server.)
4. **Met in sim.** Moving the brain to the MacBook is a
   `brain.robot_url` change and nothing else. The brain reaches the robot
   only through that URL, and `tests.demo_brain_over_http` runs the same
   mission in-process, through `RemoteRobot` over a socket, and through
   the brain service: identical actions, 83 steps each. Still to prove on
   two actual machines.
5. **Met.** All three failsafes have tests: motors-left-running
   (`tests/test_server.py`, decision logic; the async loop is Phase S4),
   AWS-link-dead and brain-loop-hung (`tests/test_failsafes.py`).
6. **Met.** `POST /mission/stop` demonstrably stops the car, not just the
   loop -- and `_HaltGate` additionally prevents an in-flight tick from
   moving the robot after the stop.
7. **Met, unchanged.** Manual D-pad control still works with the brain
   service stopped: the twin talks to `robot/server.py` directly and
   nothing on that path was touched.
8. **Met.** `CLAUDE.md`'s "duplication is intentional" bullet now
   describes the two-panel arrangement and calls the browser the optional
   brain.

---

## Ordering note

B0 (= S3) is the only hard prerequisite. B1-B3 are all testable in
simulation today, with no hardware and no Pi -- they are the largest
block of pre-purchase work left after `PLAN-sim-hardening.md`'s S1-S4.
B4 is testable with two local uvicorns. Only B5 needs the hardware.

**In the event** B0-B3 were built before `PLAN-sim-hardening.md`'s S1-S2b,
not after. Nothing in them depended on the vision policy: the runner takes
a `vision_fn` and enforces the timeout and failure budget around whatever
it is handed, so S2b's agent plugs into a loop that is already a service.
The reverse order would have produced a Python vision agent that still
could not be started or stopped from anywhere.
