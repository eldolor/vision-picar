---
kind: architecture
domain: operations
status: current
verified: 2026-10-02
---

# Operations -- architecture

Operations is how the system is deployed, reached and kept healthy. It
covers the cloud deployment, the tunnel that lets the deployed page reach
the local robot and brain, the shared secrets, the single health verdict,
build identity, mission metrics, and the planned boot-time deployment on
the car. Read this to understand where each process runs and why, and what
"healthy" is allowed to mean. Commands, stack names, routes and variables
are in the [engineering spec](../engineering/operations/ENGINEERING.md).

## Purpose

The system has two halves with different homes.

- **In the cloud:** the twin's static page, the cloud vision service, and
  the recorded-walk and metrics storage. These are small, bursty and cheap
  to run serverless.
- **On a machine you control:** the robot server, the brain, and (under the
  ROS drive) the ROS container. Today that machine is a laptop. On hardware
  day it is the Jetson on the car.

Operations joins the two. Without it, the deployed twin could not reach a
mission. Nobody could tell which build a process is running. A health check
might go red because the robot is parked. Mission outcomes would have no
record to compare across releases.

## Components and boundaries

```text
   phone (twin page, HTTPS)
        |
        v
   CDN ------------- static page and consoles (object storage)
    |  \
    |   +----------- API gateway --> vision function --> model provider
    |                            \-> walks function  --> walk + metrics storage
    |
    | (Settings points the twin at the tunnel URL)
    v
   tunnel (one public domain) --> local fan-out proxy
                                     brain prefix   --> brain service
                                     vision prefix  --> CDN (same-origin shim)
                                     everything else --> robot server
                                                       |
                                              (drive: ros) ROS container
```

| Part | Owns | Boundary |
|---|---|---|
| Serverless stack | The CDN, the private static bucket, the API gateway and the two functions | Holds no robot or brain. Nothing in it can move the car |
| Recordings storage stack | The walk bucket, shared by recordings and metrics | Has no dependency on any other stack, so it can outlive all of them |
| Deploy-artifact bucket | The function packages, which are the only rollback path for the functions | Adopted into infrastructure-as-code so it can be checked against its template |
| Static publisher | Publishing exactly the files the manifest lists, each served as the right type, and making the CDN serve the new copies | The manifest is the single list of what the site consists of |
| Tunnel and proxy | One public domain in front of the local robot and brain, split by path | A proxy, never a merge. Both servers stay separate processes |
| Secrets | One gate per trust boundary: driving the local robot, asking the vision service, reading walks and metrics | Values live outside the repo. A deploy with an empty secret runs with no authentication, and the build prints that warning |
| Health verdict | One answer for both halves, with an exit code | Only conditions a release can be blamed for may change it |
| Build identity | Which build is running, from where, logged first and published on health | Lives on the robot side of the code, because both servers report it and the robot server may never depend on the brain's code |
| Mission metrics | One summary row per mission, shipped at the end and stored with the walks | The shipper can never fail a mission |
| Boot-time deployment (B5, planned) | Starting the ROS container, robot server and brain on the car at boot | Nothing it restarts may move the robot on its own |

The recordings domain owns the walk store, the walk console and the
evaluation tools (docs/recordings/ARCHITECTURE.md). This domain owns where
they are deployed and the mission-metrics view. Walk metrics and mission
metrics share a bucket and a function, and that is the overlap between the
two specs. The vision service's behaviour, models and region pins belong to
cloud-vision.

## Decisions

### Serverless, with no network of its own

**Decision.** The cloud half is a CDN in front of a private static bucket
and an API gateway with two functions. There is no private network, no
container service and no load balancer. Nine stacks were deleted on
2026-09-05 (`PLAN-aws-cost-redesign.md`).

**Alternatives rejected.** Containers in a private network, behind a
network and an application load balancer. That cost about $159 a month in
fixed charges, of which about $110 existed only to get traffic in and let
tasks reach other cloud services. A cheaper private network was also
rejected, because the only part that truly needed one (a shared file
system for walks) moved to object storage first.

**Trade-off.** Cold starts are paid per warm period, small next to a 1-3 s
vision call. A replay would have to finish within one function invocation
instead of being a proper job. Every public route must be declared in two
routing tables, the CDN's and the gateway's.

**Deployed replay is disabled as templated.** The walks function is given
no vision service address and no vision secret, so a replay asked of the
deployed console is refused (no vision service configured). Even with an
address alone it would send the walks secret to the vision service and be
refused on every frame. This is read from the template, UNCONFIRMED
against the live stack. A walks service run locally with both set can
still replay. Enabling it in the cloud is an open question below.

### The gateway invokes functions with an assumed role

**Decision.** The API gateway calls each function with an explicit
credentials role that it assumes.

**Alternatives rejected.** Function URLs, a CDN with origin access control
in front of a function URL, and resource-policy grants. On this account,
function resource policies do not grant invocation: all three returned 403
with nothing in the function's log (measured 2026-09-04, `PLAN-aws-cost-redesign.md`
section 6).

**Trade-off.** More resources than a function URL. The template states
this so nobody "simplifies" it back.

### The robot and the brain are not in the cloud

**Decision.** The robot server and the brain run on the local machine. The
deployed twin reaches them through a tunnel.

**Alternatives rejected.** The brain on a cloud container service (built,
then deleted 2026-09-05). Its home is the car. The tiered policy loads the
detector and CLIP into the brain process, which a small cloud function
cannot host.

**Trade-off.** The deployed twin works only while a laptop is running the
stack and the tunnel. That gap closes with B5.

### One tunnel domain, split by path

**Decision.** One public tunnel domain fronts a small local proxy. It sends
the brain's prefix to the brain, a vision prefix to the deployed vision
service, and everything else to the robot server.

**Alternatives rejected.** Two tunnels: the free tier gives one static
domain per account, and a second endpoint on it is refused. One process
hosting both apps: that puts the agent loop on the event loop the watchdog
polls, the failure `PLAN-brain-relocation.md`'s "Why not one process"
describes.

**Trade-off.** One more local hop. The vision prefix exists only because
the serverless stack answers no cross-origin preflight. It is a shim until
cross-origin support is deployed there.

### One verdict, and only blameable inputs reach it (M5)

**Decision.** Health is one command over both halves. Its verdict is built
from three inputs only: each process answers; the robot's watchdog loop has
polled recently; and when a mission is running, the brain's loop has
completed a tick within the mission's own tick deadline. Everything else is
printed as description and never changes the exit code. The twin's health
line renders this verdict and never invents its own.

**Alternatives rejected.** Two separate health routes, each reporting
itself fine, which leaves a person to notice the brain is talking to the
wrong robot. A verdict that includes situation, such as a parked robot or a
safety veto on record. A check that goes red because the robot is sitting
still teaches everyone to ignore it. Microduck's design rule, "only
conditions a release can be blamed for" (`PLAN-microduck-transplants.md`
M5), is the source.

**Trade-off.** A real hardware problem that a release cannot cause, such as
a low battery, will not show in the verdict. It will show in the
description.

### Every process says which build it is

**Decision.** Each server logs one start-up line that names the service,
the code revision, where the process runs from and which configuration it
read, in a form no log setting suppresses. The same content is published
on health.

**Alternative rejected.** Relying on the deploy record. After a rollback
the revision can match while a release symlink still points elsewhere; the
executable path is what tells them apart.

**Trade-off.** None worth noting. The restart script uses the published
revision to refuse to report success until the new code is answering.

### Metrics follow the storage, and never fail a mission

**Decision.** The brain posts one summary row per mission, off the
mission's own path, to the walks function. The function stores it in the
walk bucket, one record per run. Percentiles arrive computed per run and
are never blended across runs.

**Alternatives rejected.** Cloud credentials on the robot, so it writes
storage directly. A database for append-only rows read a few times a day.
Averaging percentiles across runs, which flatters.

**Trade-off.** A metrics outage loses rows silently, apart from a log line.
That is accepted: a robot that stopped because a dashboard was down would be
worse. Sharing the walk bucket also means every reader of walks must tell a
metrics record from a walk. Today the walk listing does not, so metrics
records appear as empty walks. Operations owns that fix, because its
storage layout causes it; the recordings domain owns the list filter the
fix lands in.

### Boot-time deployment on the car uses the OS service manager (B5, planned)

**Decision.** On the car, the ROS container, the robot server and the brain
start as separate supervised services, in that order. Secrets and modes
come from an environment file, not the unit files or the repo
(`PLAN-brain-relocation.md` B5). Not built.

**Alternative rejected.** One supervisor process hosting everything, which
loses the two-process watchdog guarantee.

**Trade-off.** A restarted brain loses its mission (mission state does not
survive a restart). That is acceptable only because nothing restarted may
move the robot on its own.

## Contracts

| Between | Direction | Category | Ownership |
|---|---|---|---|
| Twin page and CDN | Phone fetches the page and calls API paths | HTTPS; static assets plus HTTP/JSON | The CDN routes. The manifest owns the asset list. The template owns the path list |
| CDN and gateway | CDN forwards declared API paths | HTTPS | Both routing tables must list a route, or it is silently dead |
| Brain and vision service | Brain calls on triggers | HTTP/JSON with the vision secret | cloud-vision owns the service |
| Brain and walks function | Brain posts one metrics row per mission | HTTP/JSON with the walks secret | The walks function owns storage. Unknown fields are refused |
| Twin and local stack | Phone calls the tunnel. The proxy fans out by path | HTTPS to the tunnel, then HTTP on loopback | The proxy owns only routing. The servers own authentication |
| Health command and both servers | The command reads each server's health route | HTTP/JSON. Health stays unauthenticated on both servers | The command owns the verdict rule. The servers own the fields |
| Operator and boot services (B5) | Service manager starts and restarts the processes | OS service manager | Planned |

## Failure modes and resilience targets

| Failure | Response | Target |
|---|---|---|
| A route is added to an app but not to the CDN or the gateway | Route-drift tests compare each app's real routes with the template | No route ships without both entries. This class shipped five times on the old load balancer |
| A page references a file the publisher does not upload | A test compares the pages' references with the manifest. Today it covers the twin and the walk console, not the metrics dashboard | Every referenced file is published |
| The publisher uploads nothing | It refuses an empty manifest and checks the upload count | Never reports success over an unchanged bucket |
| A restart leaves old code answering | The restart script compares each server's published revision with the checkout | Never reports success unless both servers run the expected revision |
| Either half is down, or its guard loop has stopped | Health verdict UNHEALTHY or UNREACHABLE, non-zero exit | A broken release is always visible. A parked robot is never reported unhealthy |
| A mission is alive but no longer ticking | Health verdict UNHEALTHY for the brain | Detected within the mission's own tick deadline |
| Metrics service unreachable | Row dropped, warning logged | Never delays or fails a mission |
| A secret is unset on an exposed process | The gate is inert. The build prints a warning; the run script refuses to start without its secrets file | Nothing reachable through the tunnel runs without a secret |
| The tunnel stops while servers restart | Restart leaves the tunnel alone and warns if it is not running | Restarting servers never takes the tunnel down |
| A deleted stack is "restored" by habit | The old templates are kept as history only | Nothing deployed depends on them |

## Open questions

- **B5 on the Jetson.** The start order is decided above, and this domain
  owns it. Unit layout, environment file, serial device access and restart
  semantics are listed in `PLAN-brain-relocation.md` B5 and not built. It
  needs the board and the Rover.
- **Cross-origin support on the serverless stack.** It would retire the
  proxy's vision shim. It needs a deploy and the route tests.
- **Deployed replay, and replay as a job.** Replay is disabled in the
  deployed stack as templated (see the serverless decision). Enabling it
  means giving the walks function the vision service's address and its own
  secret; it would then rely on the function's long timeout, and a job with
  an id would remove that class of problem. Verify against the live stack
  first.
- **Map backup to object storage.** Decided by the user (a private,
  versioned prefix per robot), not built (`PLAN-ros-alignment.md` section 6,
  question 6). Operations will own its bucket and permissions.
- **Release rollback (M11).** The health exit code is designed to gate it.
  M11 itself is proposed, not built.
