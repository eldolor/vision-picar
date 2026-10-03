---
kind: architecture
domain: twin
status: current
verified: 2026-10-02
---

# Web twin -- architecture

The web twin is the project's one user interface. It is a phone-first web
page, and it is a real HTTP client of the robot server, the brain service
and the cloud vision service. It holds no movement, safety, decision or
map logic of its own. It shows what those servers publish, and it lets a
person drive, start missions, walk a real house with the phone's camera,
and configure where everything lives. Read this for what the page may and
may not do, and why. Read the
[engineering spec](../engineering/twin/ENGINEERING.md) for files, routes,
constants and the test suite.

## Purpose

The page tests three different layers, and they are not the same question
asked three times:

- **The Sim tab** exercises the control software against the simulator:
  mission memory, the safety collar, the failsafes, driver arbitration,
  and the discovered map. It is also the only safe place to fire the
  failsafe drills.
- **Guide me** has nothing to do with the robot. A person is the actuator
  and the phone is the sensor. It answers the one question the simulator
  cannot: can the vision model read a real room?
- **Robot view** asks the same real-pixel question as Guide me, but in
  the robot's vocabulary: "what move would you make from here". It can
  also record the walk for later replay (the recordings domain). It can
  also drive a real brain mission closed-loop, with the person as the
  motor ("Drive via brain").

People depend on it to operate the system from a phone on any network.
Developers depend on it as a regression surface: every UI bug this project
shipped was found on a phone, not by the Python suite. Without it, the
brain and robot remain reachable only by command line, and the real-pixel
walks that the corpus is made of could not be recorded.

## Components and boundaries

```text
 +----------------------------- web twin (browser) ------------------------------+
 | Guide tab                 | Sim tab                       | Settings          |
 |  Guide me: steer a person |  D-pad, turn step, look       |  robot, brain and |
 |  Robot view: robot's move |  camera frame, depth strip,   |  vision URLs and  |
 |   + Record this walk      |  odometry, discovered map     |  secrets, health, |
 |   + Drive via brain       |  (tap-to-goal on SLAM maps)   |  setup code,      |
 |                           |  Remote brain panel           |  debug            |
 +---------------------------+-------------------------------+-------------------+
        |                              |                              |
        v                              v                              v
  cloud vision service          robot server                   brain service
  (same origin when deployed)   (body, safety, world)          (missions, recording)
```

- **Guide tab.** Opens the phone camera full-screen and runs a timed loop
  of vision calls, drawing an overlay from each answer.
- **Sim tab.** Manual control, readouts of what the robot senses, the
  discovered map, and the Remote brain panel, which starts and observes a
  mission running in the brain service.
- **Settings.** The three endpoints and their secrets, per-endpoint
  connection checks, the combined health line, a setup QR code that
  configures a second phone, and a debug log.

**Boundaries.** The page may not:

- **Move the robot except through the robot server's command routes.**
  Safety, arbitration and the watchdog all live on the server. Direct
  commands (the D-pad, look and stop) name the page as their driver. A
  goal tapped on the map does not: the server treats every goal as an
  autonomous driver whoever sent it, so a person's tap is refused while
  a mission holds the robot (see "A person outranks the brain").
- **Decide anything a policy decides.** Missions run in the brain service
  and survive the tab closing.
- **Hold a copy of the house, the renderer, or any physical constant.**
  The map is the server's discovered map. The camera picture is the
  server's frame. The safety threshold it displays is read from the
  server.
- **Invent a verdict or a value.** It renders what a server reported. It
  says "not reported by this server" where a route is missing, and it
  draws nothing where there are no pixels, because blank and "no
  obstacles" must not look alike.
- **Talk to a simulator or a robot backend directly.**

## Decisions

### A thin client: no logic that a server already owns

**Decision.** The page carries rendering, input and configuration only.
On 2026-09-25 the ROS alignment deleted the in-page "local brain" (a JS
frontier explorer and a JS vision autopilot), the hardcoded copy of the
starter house, and the JS raycaster (`PLAN-ros-alignment.md` 3.2).

**Rejected.** Keeping the JS brain as a no-backend development path. Once
the twist multiplexer puts exactly one writer between any driver and the
wheels, a brain that dies with a browser tab cannot be that writer. Also
rejected: keeping the JS raycaster. It read a cell and a cardinal heading.
Under a continuous pose it drew a direction the robot was not facing, and
one call site fed that picture to the model (`tests/test_frame_source.py`
records the history).

**Trade-off.** The Sim tab now needs the brain service running to show
autonomy. The server's refusal of a second mission and the driver
priority order enforce "one brain at a time", where the page used to.

### Draw what the server publishes, and say so when it publishes nothing

**Decision.** Every readout comes from a route. Each one distinguishes
three states: a value, "this server does not report that", and "no
reading". Examples:

- The frame-source line says the server sent the frame, or that none
  came.
- Odometry says it has no encoders rather than showing zero.
- The depth strip outlines exactly the zones the server says the veto
  reads.
- The health line renders per-half answers using the verdict rule that
  belongs to the operations domain's health command.

**Rejected.** Computing a reading locally where the server's is missing,
or reducing the depth grid in the page. A second copy of a rule drifts,
and the page is the copy people believe.

### Phone-first, and tested in a real browser at phone width

**Decision.** The layout targets a phone viewport. The automated UI tests
drive the real page in a real browser at that width.

**Rejected.** DOM-only tests (jsdom). The two bugs that motivated the
suite were a model picker that never populated and one that rendered as a
40 px chevron. The second exists only under real layout. Each UI test is
confirmed red against its bug before it is trusted.

**Trade-off.** The UI tests skip without an installed browser, so a green
run on a fresh machine may have tested nothing here.

### A live camera with a 2D overlay, not WebXR

**Decision.** Camera access plus DOM and SVG overlays positioned over the
video.

**Rejected.** WebXR. Its support on iPhone was unsettled when this was
designed, and "AR-style" meant a directional arrow, not spatial anchoring
(`PLAN-ar-guidance.md` section 2).

**Consequence.** The page must be served from a secure origin (HTTPS or
localhost), and that drove the hosting decision below.

### Static hosting on the CDN, same origin as the vision service; the robot and brain through a tunnel

**Decision.** The deployed page is static files on a private bucket
behind the same CDN distribution that fronts the vision service. Vision
calls are same-origin and never preflighted. The robot and brain run on
the developer's machine. A single tunnel domain reaches them, split by
path (the operations domain owns the tunnel).

**Rejected:**

- **The page served by the robot server on ECS behind the shared load
  balancers.** This is how it ran until 2026-09-05. It was deleted with
  the VPC (`PLAN-aws-cost-redesign.md`).
- **Third-party static hosts,** which `PLAN-ar-guidance.md` section 2
  listed. Same-origin with the vision service is what makes vision calls
  work without CORS.
- **Opening CORS on the vision service.**

**Trade-off.** Locally the robot server still serves the page, so there
are two ways to load it. A saved endpoint can follow a person between
deployments. The page shows a notice whenever an endpoint's host differs
from the page's, except when the page itself was loaded from the
developer's own machine, where splitting the page and the services across
ports is the normal layout. It is a notice and not an error, because a
tunnel is a legitimate reason for them to differ.

### No content-hashed filenames: revalidate every load, and publish an explicit asset list

**Decision.** On the deployed site, pages and scripts are served
"revalidate every time". An explicit manifest lists every published file
with its type and cache policy. A test checks the manifest against what
the pages reference. Locally, only the script is marked "revalidate"; the
page itself carries no cache instruction, so a browser may reuse a stale
page against a fresh script. The decision's guarantee (page and script
never a version apart) is therefore met on the deployed site only.

**Rejected.** Hashed filenames. They need HTML templating and a build
step, for one page served to one household. Also rejected: a guessing
upload tool. The console's page has no extension, so a guessed type would
download it instead of rendering it.

**Why the test.** A file the page references but nobody published loads
the page with one feature silently dead. That has shipped five times.

### Throughput for a person, one-at-a-time for a robot

**Decision.** In Guide me and Robot view, vision calls overlap (a small
fixed number in flight), so a person walking gets a steady stream of
decisions. An answer overtaken by a newer one is discarded. An answer from
a previous session is discarded. Calls in flight count against the
paid-call budget, so overlap can never overshoot it. When a real mission
is driving (Drive via brain), the page allows one call in flight.

**Rejected.** A strictly serial loop. Its real cadence was the throttle
plus the model's latency, about 3 s on the model in use when this was
decided (dated; not re-measured on today's default). Also rejected:
unbounded overlap, which Bedrock throttles and which competes with
production for the account's quota.

**Trade-off.** Overlap raises throughput, not freshness. Each answer still
describes a frame one round trip old. A robot executing every answer
would be worse off, hence the single in-flight call under a mission.

### A person outranks the brain, and the page says who it is

**Decision.** Every command from the D-pad names the D-pad as its driver.
The server's priority order lets a tap pre-empt a mission (the safety
domain). The page shows who holds the robot and the last refusal with its
reason.

**Rejected.** Unnamed commands where the last writer wins.

**Exception, as built.** A goal tapped on the map is not a direct
command. The server arbitrates every goal as an autonomous driver (the
safety domain's rule that a navigation goal is autonomous), so a tap
during a mission is refused rather than pre-empting it, and the page
shows the refusal. A person who wants the robot back mid-mission uses the
D-pad, which pre-empts the mission, or the Remote brain panel's Stop,
which ends it. The robot STOP button is not the same thing: it holds the
motors and claims no authority; the one thing it ends is a navigation
goal (decided by the user 2026-10-02, built the same day), and a non-zero
D-pad movement cancels one too. To resume, a person sends the goal again.
The rule belongs to the [safety domain](../safety/ARCHITECTURE.md).
Whether a person's goal should rank as a person
is also open.

### Configuration lives on the device, and shares by QR, never through a third party

**Decision.** Endpoints, secrets and preferences persist in the browser's
storage. Every access is guarded, because private browsing can throw.
"Show setup code" draws a QR code of a magic link carrying the secrets.
The code is generated in-page, shown only on a tap, and hidden again
on a short timer or when the person leaves the tab.

**Rejected.** A QR web service or library CDN. Handing the secrets to a
third-party image API would leak them.

### Tunnel workarounds are scoped to the tunnel

**Decision.** The header that skips the free tunnel's HTML interstitial
is sent only to that provider's hostnames.

**Rejected.** Sending it everywhere. It is a custom header, so it would
force a CORS preflight on the frequent health poll of every LAN
setup.

## Contracts

| Neighbour | Direction | Protocol | Ownership |
|---|---|---|---|
| Robot server | page calls server | HTTP/JSON, robot secret header, driver header on direct commands (not on map goals, which the server ranks as autonomous) | Server owns pose, map, frame, depth, odometry, safety, arbitration and the watchdog. The page renders them. |
| Brain service | page calls brain | HTTP/JSON, the brain's own secret | Brain owns the mission and its status. The page starts and stops it and polls the status. The brain owns recording when the page records. |
| Cloud vision service | page calls service | HTTP/JSON, vision secret, same origin when deployed | Service owns model ids, wordings and answer vocabularies. The pickers fill from it. |
| Static hosting and CDN | page is served from it | HTTPS | Operations owns the deploy. The asset manifest owns the file list. |
| Tunnel | page calls through it | HTTPS, one domain split by path | Operations owns it. The page only adds the interstitial header for tunnel hosts. |
| Recordings | page writes frames and a finish marker through the brain URL | HTTP/JSON | Page owns the walk name and frame numbering. Recordings owns storage. |

## Failure modes and resilience targets

| Failure | Behaviour | Target |
|---|---|---|
| The brain stops answering | Marks it lost, disables Start with the reason, retries on its own | A restarted brain reconnects without a person pressing Connect |
| The brain answers with an error | Stays connected and shows the error | "Answered badly" is never shown as "gone" |
| A secret is rejected by a different deployment | The error names the host it went to | A 401 is never misread as a wrong secret when the endpoint is wrong |
| The vision service is healthy but cross-origin | The connection check names the origin as the fault | "Load failed" always has a diagnosis. Not met precisely: the check flags EVERY cross-origin vision URL as publishing no cross-origin headers, including a local service and the tunnel's vision route, which do publish them and work. The diagnosis is right only for the deployed service. |
| The tab is backgrounded mid-mission | The mission continues in the brain, and the page catches up on return | Closing the page never stops or orphans a mission |
| A stale call returns after Stop or a newer answer | Dropped before it can touch the overlay or the recorder | No previous session's answer is ever drawn |
| A forgotten Guide session | Pauses at the per-session call cap | Unattended spend is bounded |
| A server older than a readout's route | "Not reported by this server" | Absence never reads as zero or clear |
| A frame with no pixels | Draws nothing and says so | The page never shows a picture it invented |
| Recording with no brain connected | A toast says recording is off | A walk is never silently unrecorded |
| An insecure origin for the camera | The start error explains the secure-context rule | |
| A stale script after a deploy | No-cache plus a CDN invalidation | A redeploy never runs an old script against new HTML |

## Open questions

- **The environment banner cannot fire on the deployed twin.** It asks
  the origin that served the page. Under the CDN that origin is the vision
  service, which reports no environment label. The banner works only when
  the robot server serves the page. Fix it in the vision service or in
  the page; operations decides.
- **The health line re-implements the verdict thresholds** rather than
  reading a verdict. It agrees with the command today. Whether the
  servers should publish the verdict itself is open.
- **Should a goal a person taps rank as a person?** Today it ranks as
  autonomous and cannot pre-empt a mission. Ranking it as a person would
  need the server to tell a person's goal from the brain's; the safety
  domain owns the driver order and decides.
- **The S1 "Check robot contract" button** is still owed
  (`CLAUDE.md` section 7).
- **Press-and-hold wheel velocity from the D-pad** waits for the twist
  path to be the default drive (`PLAN-ros-alignment.md` R4/3.24).
