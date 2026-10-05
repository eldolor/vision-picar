---
kind: architecture
domain: cloud-vision
status: current
verified: 2026-10-02
---

# Cloud vision -- architecture

The cloud vision service answers questions about a single photograph: what
the robot should do next, where a person holding a phone should go, and
what is in the picture. It is a stateless HTTP service in the cloud, a
large vision-language model sits behind it, and nothing on the robot
depends on it for safety. Read this for why the service has the shape it
has. Read the [engineering spec](../engineering/cloud-vision/ENGINEERING.md)
for routes, models, prompts, parameters and deploy commands.

## Purpose

A floor robot needs to be told which way to go, and nothing on the robot
can read a room the way a large vision-language model can. This service is
where that judgement is bought, one call per question.

Three callers depend on it:

- **The brain's paid policies.** The vision policy and the tiered policy
  (the policy domain) send a frame and a target and get back one move. The
  tiered policy calls only on a trigger, and once at each arrival, when it
  reads only the visibility flag: that flag is its identity verdict and
  decides `found` (the policy domain's rule, 2026-10-02). The vision policy
  calls on every step.
- **The twin's Guide tab** (the twin domain). "Guide me" steers a person
  to an object and "Robot view" shows the move the robot would make, both
  on the phone's real camera.
- **Walk replay** (the recordings domain) re-asks a recorded walk's frames
  under another model or wording. It goes through this service on purpose,
  so a replay measures the service and not a copy of its prompt.

Without the service, the vision and tiered policies cannot run, both Guide
modes are dead, and no two models can be compared. The robot stays safe
either way, because no move depends on this service being up.

## Components and boundaries

```text
 brain (vision / tiered policy) --+
 twin (Guide me, Robot view) -----+--> edge (CDN, same origin as the twin)
 walk replay (walks service) -----+          |
                                             v
                                   gateway (one route per path)
                                             |
                                             v
            +--------------- vision service (stateless) ---------------+
            | route layer: auth, base64, size cap, allow-lists;        |
            |   a refusal here comes before any model call             |
            | question layer: one prompt + one schema per route        |
            | model layer: managed model API, per-model region         |
            | parsing: merge over a safe default, coerce, strip unasked|
            | room guesser: object list -> room label                  |
            +----------------------------------------------------------+
                                             |
                                             v
                                 managed model provider (Bedrock)

 manual only: brain-side scene describer -> direct Anthropic API
```

- **Route layer.** Owns authentication by shared secret, decoding the
  upload's text encoding, the image size cap, and the two allow-lists
  (models and prompt wordings). A request that fails here never reaches a
  model and costs nothing. It does not check that the bytes are an image:
  that is first discovered by the provider, as a server error.
- **Question layer.** One prompt and one response schema per question.
  The four questions are: what the robot's scene contains, a description
  of the photo for a person, the robot's next move, and which way to
  steer a person. The engineering spec maps each question to its route.
- **Model layer.** Calls a managed model API under the service's own
  cloud identity, with no API key in the service. Each route has its own
  default model.
- **Parsing.** Turns free text into a fixed schema. It fails towards "do
  not act on this", with three fields not yet covered and one only
  partly covered (see Decisions).
- **Room guesser.** A deterministic object-to-room matcher, a copy of the
  brain's.
- **The brain-side scene describer** (`brain/vision.py`). It calls the
  model provider's own API directly. Only one manual script uses it,
  plus unit tests with the API mocked. No mission, simulator run or
  hardware path calls it.

**Boundaries the service may not cross.** It holds no state between
requests. It knows nothing about missions, mission memory, the robot, or
the map. What it remembers of a search arrives in the request, as the
rooms already searched, and it hands back a room guess. It never moves
anything. Its package imports nothing from the rest of the repository, so
the deploy artefact is self-contained. The policy that turns its answer
into a move belongs to the policy domain. Replay belongs to the
recordings domain.

## Decisions

### Vision is a cloud service, and the control loop never waits on it for safety

**Decision.** Model calls live in a separately deployed HTTP service, not
in the robot server and not on the car. Collision avoidance stays on the
robot (the safety domain). A slow or failed vision call is the brain's
problem: its failure budget and timeout (the mission domain) end the
mission with the robot stopped.

**Rejected.** Calling the model from inside the robot server. A blocking
model call would sit next to the watchdog, and the robot would carry model
credentials. Also rejected: running a vision-language model on the car.
`PLAN-onboard-perception.md` measured a local VLM on the accelerator then
considered as slower than the cloud call it would replace, so the car runs
a small local perception tier and calls the cloud only on a trigger (the
perception domain).

**Trade-off.** Every paid decision costs a network round trip of a few
seconds and money. That cost is why the tiered policy exists.

### One route per question, not a mode flag

**Decision.** The robot's move, steering a person, the robot's scene and a
description for a person are four separate routes. Each has its own
prompt, its own schema and its own default model.

**Rejected.** A target-object mode on the scene question, which
`PLAN-ar-guidance.md` section 4.2 first proposed. Two questions that share
a request field mean different things. A flag would couple their prompts
and their models, and a per-route model choice was later measured to
matter.

**Trade-off.** Four routes to route, test and keep in step with the
gateway's route table.

### Arrival is its own field, and it is never inferred from STOP

**Decision.** The move question reports arrival in a dedicated boolean
that only a literal true sets. Arrival implies visibility: the service
forces it false when the model does not report the target visible. The
move field is only ever about movement.

**Rejected.** Treating STOP as success. STOP is also the model's answer
for a blocked path and the parser's default for a malformed reply. A
mission would then end on an obstacle or on garbage.

**Trade-off.** Over-running by a step or two when the model is slow to
say "reached". Over-running is cheap. Stopping short is not.

The arrival field is what the vision policy and Robot view end on. The
tiered policy ignores it -- a range judged from a photograph is not
measured -- and uses the visibility flag on its arrival frame as an identity
verdict instead, with the lidar deciding distance. That flag therefore
carries a `found`, and its accuracy at arrival range (a floor camera at
0.40 m or less) is unmeasured; see Open questions.

### Every answer fails towards "do not act on it"

**Decision.** A reply that does not parse becomes the empty schema, with
STOP, not visible and not reached. The fields a caller may act on fail
safe: a move outside the vocabulary becomes STOP, arrival counts only
when it is a literal true, and the distance and path answers fall back to
their "unknown" or "unclear" value when off-vocabulary. The room guess
falls back to "unclear" only when it is missing, empty or not text; any
other text is passed on as a room label. The person-steering reply gets
the same treatment for its position, proximity and box. The one
question a wording may leave out, the obstacle question, is removed from
the reply rather than defaulted when the wording did not ask it, so a
caller can tell "the model saw no obstacle" from "nobody asked". The
distance and path answers are never removed; they read "unknown" when
not asked.

**Coerced in the service, once, for every caller** (handoff 4b,
2026-10-03). The target's direction is held to its vocabulary, and the
visibility and obstacle flags (and the person-steering reply's
visibility) are made real booleans, a written "true" counting as true.
**Rejected:** leaving each caller to check types -- the brain did and the
twin did not, and since the brain decides `found` on the visibility flag
at arrival, a written "true" read as false would refuse the right object.
**Not yet covered:** the room guess, where an invented room name passes
through.

**Rejected.** Passing model text through as-is, or defaulting absent
fields to false. A false that nobody measured reads exactly like a
measurement.

### Model and wording are chosen from server-side allow-lists, published and echoed

**Decision.** A caller may pick a model and a prompt wording for the
move question, but only from lists the service owns. The service
publishes both lists and their defaults on one read-only route, and it
echoes the model and the wording that actually answered on every reply.

**Rejected.** Accepting any model string, which could run up the bill or
hit a model the account cannot invoke. Also rejected: lists hardcoded in
each client, which drift.

**Why the echo is a commitment.** For about a week, the code default and
the deployment template disagreed about the navigate model. The
environment variable won silently, and every walk recorded before
2026-08-29 was attributed to the wrong model (`CLAUDE.md` section 5,
Stage 0). The only trustworthy answer to "what produced this decision" is
the one carried on the reply.

### The default model is chosen by replay, never by catalogue or reputation

**Decision.** The default model for the move question is chosen by
replaying every frame of a recorded walk through every model the account
can invoke: same pixels, same prompt, only the model varying. The model
that won the last such replay was the only one that both made progress
and stayed aware of obstacles; which model that is, and the replay's
numbers, are in the engineering spec. A model enters the allow-list only
after a real call with a real walk frame, from the deployed region, has
succeeded. It stays labelled "unmeasured" until a replay has ranked it.
The person-steering question runs on a cheaper model. A measurement that
it is faster and matches accuracy for that task is claimed in the
service's own notes but recorded nowhere (UNCONFIRMED); the choice has
not been through a replay.

**Rejected.** Promoting a model because it is newer or appears in the
provider's catalogue. Several catalogue entries returned access-denied on
a real call.

### The vision model is never the obstacle sensor

**Decision.** The move question answers WHAT is visible and WHICH WAY to go. How
far away things are belongs to a range sensor on the robot. The obstacle
field the default wording still asks for is uncalibrated. Nothing may veto
or permit a move on it alone. A wording that removes the obstacle question
entirely exists for that reason.

**Rejected.** Rewording the obstacle question until it calibrates. Four
wordings gave two never-FORWARD results and two always-FORWARD ones, and
models disagree from about 0% to about 100% on identical frames. Also
rejected: an ordinal distance answer, which said "within one step" on 60%
of 80 frames (`service/vision_analyze/vision_core.py` records the
measurement). A single photograph does not contain metric depth.

### A managed model API under the service's own identity

**Decision.** The deployed service calls Amazon Bedrock, authenticated by
its cloud role. The sim-only scene describer keeps the direct Anthropic
API.

**Rejected.** An API key in the deployed service: a secret to store,
rotate and leak. Bedrock was first chosen so that the service could reach
the model over a private network endpoint. That reason went away with the
VPC (below). Role-based authentication is the reason that remains.

**Trade-off.** Model availability on Bedrock lags the provider's own API
and differs between regions. The model strings therefore differ between
the two paths on purpose.

### A per-model region pin, not a service-wide region move

**Decision.** A model that the account can invoke only in another region
is pinned to that region on its own. Every other model keeps the
service's own region.

**Rejected.** Moving the whole service to the other region. That would
add latency to every call in order to serve one model. The pin is meant
to be removed the day the model is enabled locally.

### The prompt and room logic are copied into the service, on purpose

**Decision.** The service carries its own copies of the scene prompt and
of the room matcher, so its deploy package has no imports from the
repository.

**Rejected.** A shared package. It would drag the whole repository into a
small serverless artefact, and the first deploy that hand-listed
dependencies already died at import.

**Trade-off.** Two copies can drift, and the scene prompt already has (see
Open questions). Changes are mirrored by hand.

### Serverless behind a gateway with an explicit invoke role, under one CDN

**Decision.** The service runs as a function. An HTTP gateway invokes it
through an integration that carries its own credentials role. It is
published under the same CDN distribution as the twin's static page, so
browser calls from the twin are same-origin and never preflighted. There
is no VPC.

**Rejected:**

- **Containers on ECS Fargate behind an NLB and an internal ALB, in
  private subnets with interface endpoints.** This ran until it was
  deleted on 2026-09-05. Roughly $110 of $159 a month was networking that
  bought no capability (`PLAN-aws-cost-redesign.md` sections 1-2).
- **A function URL, or any invocation that relies on the function's
  resource policy.** Measured on 2026-09-04: resource-based policies do
  not grant invocation on this account, so the call is refused and the
  function never runs (`README.md`, "History: why not Lambda?", and
  `PLAN-aws-cost-redesign.md` section 6).
- **Opening CORS** so the twin could be served from anywhere. Same-origin
  hosting removes the need, and an open CORS policy is one more thing to
  get wrong.

**Trade-off.** Network-level isolation is gone. The function reaches a
public model endpoint, still signed and encrypted. Cold starts add a
small delay to a call that already takes seconds.

## Contracts

| Neighbour | Direction | Protocol | Ownership |
|---|---|---|---|
| Brain (vision and tiered policies) | brain calls service | HTTP/JSON, shared-secret header | Service owns the answer schema and both allow-lists. The brain owns mission memory and sends the searched rooms in. |
| Twin, Guide tab | browser calls service, same origin | HTTP/JSON | Service owns the vocabularies (actions, positions, proximities). The twin only renders them. |
| Twin, model and wording pickers | browser reads service | HTTP/JSON, unauthenticated read | The service is the only source of model ids and wordings. No client may hardcode them. |
| Walks service (replay) | walks service calls service | HTTP/JSON | Replay asks the live move question with the same prompt, parsing and allow-list. Two inputs differ from a live call: replay labels every frame as JPEG whatever its format, and sends no searched rooms. Recordings owns the comparison and records the gap. |
| Model provider | service calls provider | managed API under the service's cloud role | Provider owns availability per model and region. The service owns which models are allowed. |
| Gateway and CDN | edge routes to service | HTTP | Each public path is listed on purpose. A path missing from the edge is a 404 on one feature while the page still loads. |

## Failure modes and resilience targets

| Failure | What happens | Target |
|---|---|---|
| The provider errors, throttles or times out | The service answers with a server error. The caller counts it against its own budget. | No vision failure can move or strand the robot. Stopping is the brain's guard, not this service's. |
| The model replies with something that is not JSON | Safe default: STOP, not visible, not reached | A malformed reply never reads as arrival. |
| The model volunteers an obstacle answer the wording did not ask for | The field is removed | "Not asked" is never reported as "false". |
| The model returns an off-vocabulary direction, a non-boolean visibility or obstacle flag, or an invented room name | Passed through unchanged (known gap) | Target: no field a caller acts on is trusted as typed. Not met for these four. |
| A model id or wording not on the allow-list | Refused as a client error before any model call | An invalid choice never reaches the provider and never costs money. |
| A missing or wrong shared secret | Refused | When a secret is configured, no unauthenticated call is billed. A deployment with no secret is open, and the deploy instructions say so. |
| An image over the size cap, or an upload whose text encoding does not decode | Refused before any model call | |
| Bytes that decode but are not an image the provider accepts (or a phone format that fails conversion) | Sent on; the provider or the conversion fails, and the service answers with a server error | Costs a call attempt, never a move. Whether to validate the image before the call is open. |
| A phone uploads a format the provider rejects | Converted to an accepted format before the call | Real phone photos work. |
| The page is served from a different origin | The browser blocks the call. The twin's own connection check names that as the fault. | Never misreported as a bad secret. |
| A model the local region refuses | Pinned to a region that accepts it | Every listed model answers from the deployed region. |
| A decision has to be attributed later | Model and wording are on the reply | Every recorded decision names what produced it. |

## Open questions

- **How often does the move route call the target visible at arrival
  range?** Since 2026-10-02 that answer decides a tiered mission's `found`,
  and nothing has measured it: every sweep fakes the cloud from the
  simulator's ground truth. Replaying the rig walks' last frames (the
  recordings domain's replay) would measure it on real pixels.

- **Should the robot-scene and person-description questions stay
  deployed?** Nothing in the twin calls them since the Camera tab was
  removed on 2026-09-25. They are still routed and tested. The user decides. Evidence would be a caller
  that needs them.
- **Coerce the room guess to a known room list?** The flags and the
  direction are coerced since handoff 4b; an invented room name still
  passes.
- **Promote a newer model?** Three models were added on 2026-09-21 and
  are unmeasured. A replay over the rig corpus decides it, through the
  recordings domain's replay.
- **The obstacle question's shape.** The "center third path" wording asks
  about the robot's next patch of floor rather than "anything ahead". It
  has not had a fair test on a valid corpus. On the car, the range sensor
  makes this moot for safety.
- **The scene prompt has drifted between the two copies.** The service
  frames it as a photo taken by the owner and asks for specific object
  names. The brain-side copy frames it as the robot's view. Whether to
  re-unify them is open. Only a manual script uses the brain copy.
