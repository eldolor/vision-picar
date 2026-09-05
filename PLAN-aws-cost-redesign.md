# PLAN: the AWS redesign, and the cost that provoked it

Status as of 2026-09-04:

| Stage | What | Status |
|---|---|---|
| 1 | Recorded walks off EFS, onto S3 | **DONE** (commit `b42bee6`), data migrated and verified |
| 2 | Tear down the VPC and everything that needs one | **SPECIFIED, NOTHING DELETED** |
| 3 | Rebuild without a VPC | **SPECIFIED, NOTHING BUILT.** Section 6's gate ran: Function URLs are unusable here, API Gateway + a credentials role works and is the design |

Written to hand stages 2 and 3 to a session that was not present for the
measuring. Most of the value here is in section 1 and section 7: the
numbers took three attempts to get right, and two of the wrong answers
were wrong in ways that look completely reasonable.

---

## 1. What it actually costs, and the trap in measuring it

**Steady state is ~$159/month of fixed cost**, before a single Bedrock
token. Measured from Cost Explorer, us-east-2, on 2026-09-03 -- a day
with no deployment on it:

| Line | $/day | $/month | What it is |
|---|---|---|---|
| `VpcEndpoint-Hours` | 2.400 | **72.00** | 5 interface endpoints x **2 AZs** = 10 ENIs |
| `Fargate-ARM-vCPU/GB` | 1.422 | **42.66** | 6 tasks x 0.25 vCPU / 0.5 GB |
| `LoadBalancerUsage` | 1.084 | **32.52** | NLB + internal ALB |
| `PublicIPv4:InUseAddress` | 0.240 | **7.20** | 2 NLB addresses |
| `SecretsManager` | 0.097 | **2.90** | 8 secrets, 6 of them this project's |
| ECR, S3, LCU, endpoint bytes | ~0.05 | ~1.50 | endpoint *data processing* is $0.16/mo, nothing |
| | | **~159** | |

**The trap: do not average across a deployment day.** ECS and ELB both
roughly double on a day with a rolling deployment (2026-09-01: ECS
$2.830 vs $1.422, ELB $2.167 vs $1.084). A three-day window that happens
to include one produces ~$188/month, which is wrong by 18%. A window that
includes the *current* partial day is wrong the other way -- Cost
Explorer's last day is incomplete, and projecting from it gave ~$151.
Both of those numbers were reported confidently in this project before
the daily series was looked at. **Pull `--granularity DAILY`, drop the
current day, drop any day with a deploy, and read the flat part.**

Sanity check the Fargate line arithmetic rather than trusting a total:
ARM Fargate in us-east-2 was $0.032384/vCPU-hr and $0.003556/GB-hr as of
2026-09, so 0.25 vCPU + 0.5 GB is $0.009874/hr per task, x 6 x 24 =
$1.422/day. When the measured number matches the arithmetic to three
decimals, that is a day with no deployment and no change in task count.

The deploy-day inflation reaches further than ECS and ELB, which is why
the rule is "drop the day" and not "drop two lines": the Secrets Manager
figure in the table above was first recorded as $3.69/month from the same
contaminated three-day window. Measured on clean days it is $0.093-0.103
per day, i.e. ~$2.90 -- consistent with 8 secrets at $0.40 plus a little
API traffic, where $3.69 was consistent with nothing.

### The one finding worth acting on immediately

`cloudformation/network.yaml` has an `EndpointHighAvailability` parameter
defaulting to `"false"`, and commit `f528126` is titled "Stop paying
twice for endpoints." **The deployed stack has no such parameter** --
`describe-stacks` on `vision-picar-network` does not list it, and all
five interface endpoints are in two subnets. The template edit was never
deployed. That is $36/month sitting in git, unapplied.

**This is an interim hedge, not part of the target state, and its saving
must never be added to stage 3's.** The fix halves 10 ENIs to 5, saving
$36 of the $72. Stage 3 deletes the VPC, so all five interface endpoints
stop existing and the whole $72 goes -- the fix is superseded, not
incorporated. Section 2's "$110 of the $159" already counts the full $72
on that basis, alongside the load balancers and public IPv4.

So: deploy it if stage 2 is more than a few days away, because $36/month
for one `cloudformation deploy` of a template already in git is the best
ratio on this list. Skip it if stage 2 is imminent -- it updates a stack
that is about to be deleted. Either way it changes nothing about stage
3.

---

## 2. The decision

Roughly **$110 of the $159 is the price of having chosen "containers in a
private VPC."** Endpoints, two load balancers and public IPv4 exist only
to get traffic in and let tasks reach ECR, Logs, Secrets Manager and
Bedrock. None of it buys capability.

And one component was the *blocker*, which is a narrower claim than it
first looks and is worth stating precisely, because an earlier draft of
this document got it wrong. Three things impose a VPC on the deployment
as it stands today (the table below), and Fargate is one of them -- so
"EFS is the only thing that requires a VPC" is false about the current
architecture.

What is true, and is the load-bearing point: **EFS is the only one that
would still require a VPC after the compute moves.** Swap Fargate for
Lambda and the load balancers for CloudFront, and Bedrock, S3, Secrets
Manager and CloudWatch are all reachable with no VPC at all -- EFS alone
would have dragged one back. It is the constraint that survives every
other substitution, which is why stage 1 was first and why it was worth
doing even if stages 2 and 3 never happen.

The workload is one phone against a `/navigate` call that already takes
1-3 seconds, and it is bursty: Bedrock spend over the ten days to
2026-09-03 ran $0.61 to $10.65 a day, a 17x spread, with the
infrastructure billed at a flat ~$5.30/day underneath it regardless.

Note what that data does *not* say. An earlier draft claimed the system
was "idle >95% of the time" and used "a few hours a week"; neither was
measured, and the daily series actually shows non-zero Bedrock activity
on **every one of those ten days**. The defensible claim is the one
above -- low-volume and highly variable demand priced against fixed
always-on capacity -- not a specific idle fraction, which nothing here
establishes.

Cold starts are correspondingly cheap **relative to the call they wrap**,
which is the only comparison that matters: a boto3-only Python function
initialises in well under a second, against a vision call that takes one
to three, and only on the first invocation of a warm period. That is a
small proportional penalty, not a free one, and it should be measured
after stage 3 rather than assumed.

### Why no VPC *at all*, rather than a cheaper one

A VPC is not infrastructure you decide to have. It is a requirement
imposed by what you run, and exactly three things imposed it here:

| Component | Why it forced a VPC |
|---|---|
| ECS Fargate | `awsvpc` is its only network mode; a task cannot be defined without subnets |
| EFS | reachable only via mount targets, which are ENIs in subnets -- there is no public EFS endpoint |
| ALB / NLB | load balancers live in subnets by definition |

Stage 3 removes all three: EFS became S3 in stage 1, Fargate becomes
Lambda, and the load balancers become CloudFront plus Function URLs --
AWS-managed edge infrastructure, not resources in anyone's network. And
**Lambda runs outside a VPC by default.** You attach one only when a
function must reach something private (RDS, an internal ALB, an EFS
mount). Nothing in the target state is private.

The key realisation about the $72: **the five interface endpoints do not
provide access, they restore access that the private-subnet choice
removed.** `network.yaml` provisions no NAT gateway, so those subnets
have no internet route at all, and every AWS service the tasks need --
Bedrock, ECR, Logs, Secrets Manager -- then requires its own paid private
door. Outside a VPC those are ordinary public API endpoints, reachable
with SigV4 like any other AWS call. The $72/month is the price of a
self-imposed constraint, not of a capability.

### What that spends, and it is not nothing

This is a deliberate property being traded away, not an accident, and
`cloudformation/network.yaml`'s own description states it:

> No NAT Gateway -- the private subnets have no internet route at all...
> so the tasks that call Claude (via Amazon Bedrock) **never touch the
> public internet**.

A VPC-less Lambda calling Bedrock reaches the public `bedrock-runtime`
endpoint: a publicly resolvable name, still TLS and still SigV4-signed,
but a public endpoint rather than a private ENI -- precisely the
distinction the original design was built around. Whether such traffic
physically leaves AWS's network is not something this project can verify
or control, so do not lean on "it stays on the backbone" as the
reassurance; the honest statement is that authentication and encryption
are unchanged and network-level isolation is gone. Security groups also stop being a
control on those functions, and a static egress IP stops being possible.

For a hobby robot with no regulated data, no database and no allowlisting
requirement, that is a reasonable trade. **State it as a trade.** Stage 3
spends a deliberate security property to save ~$110/month; it does not
discover that the property was never there.

It is genuinely either/or: the property can be kept by putting the
Lambdas *in* a VPC with interface endpoints, and that reinstates the
entire $72, which defeats the exercise.

**Reasons the VPC comes back**, so a later session recognises them: a
database, a required fixed egress IP, anything self-hosted the functions
must reach privately, or a compliance requirement. None apply today.

---

## 3. Stage 1 -- walks off EFS (DONE)

See commit `b42bee6`. In short: `control/walk_store.py` is
`robot/interface.py`'s shape one layer down -- one abstraction, two
backends (`LocalWalkStore`, `S3WalkStore`), with
`walk_store_from_config()` as the only place that picks. On-disk layout
byte-identical on both, because the corpus is the project's only dataset
and `walk_eval.py` / `walk_replay.py` / the download-zip route all read
that layout.

`tests/test_walk_store.py` (74 tests) runs every assertion against both
backends and cannot tell which, the way `tests/test_robot_contract.py`
does for robots.

**The corpus is already in S3 and verified**:
`s3://vision-picar-recordings-303351622021-us-east-2/recordings/` --
39 walks, 821 frames, 58,860,594 bytes, 1016 files including sidecars.
The admin app running on S3 answered `/recording/walks` with **zero
differences** from the live EFS-backed service across all 39 walks on
every field. Bucket has versioning on and `DeletionPolicy: Retain`.

**Not done, deliberately:** the six deployed ECS services still mount
EFS. They were not cut over to S3 because stage 2 deletes them, and
redeploying six services onto S3 in order to delete them an hour later is
waste. If stage 2 is abandoned, this becomes real work.

---

## 4. Stage 2 -- the teardown (SPECIFIED, NOTHING DELETED)

### Order, derived from the real export/import graph

Only four stacks export anything. Deleting out of order fails with
"Export ... cannot be deleted as it is in use."

```
1. vision-picar-cdn            (imports service's NlbDnsName)
2. vision-picar-twin           ┐
   vision-picar-brain          │ leaves: import network + service,
   vision-picar-admin          │ and brain/admin also import recordings
   vision-picar-teleop-robot   │
   vision-picar-teleop-brain   ┘
3. vision-picar-service        (owns the cluster, NLB, ALB, listener)
4. vision-picar-recordings     (EFS -- see the trap below)
5. vision-picar-network        (VPC, subnets, 5 interface endpoints)

NEVER: vision-picar-recordings-s3  <- holds the corpus, imports nothing,
                                      and exists precisely to outlive all
                                      of the above
```

### Traps

- **`delete-stack` on `vision-picar-recordings` does NOT delete the
  filesystem.** It carries `DeletionPolicy: Retain`, on purpose. The EFS
  volume survives as an orphan and must be deleted explicitly, *after*
  its mount targets. Do that as a separate deliberate act, not as part of
  the stack delete -- and not at all until the S3 copy has been
  re-verified on the day.
- **The app goes fully offline** the moment stage 2 starts: no twin, no
  `/navigate`, no Guide tab, no admin console, until stage 3 lands. This
  is expected, and it is why stages 2 and 3 should not be separated by
  days.
- **ENIs hold subnets hostage.** Endpoint and Fargate ENIs can lag behind
  their stacks; a `vision-picar-network` delete that fails on a subnet
  dependency usually means waiting a few minutes, not forcing anything.
- **Do not run this while another session is deploying.** Two sessions
  operating on the same stacks can leave one in `UPDATE_ROLLBACK_FAILED`,
  which is tedious to clear.

### Before deleting anything

Re-verify the corpus. The verification in section 3 was run on
2026-09-04; if EFS has been written to since, S3 is stale. Diff the two
before the EFS copy stops existing.

---

## 5. Stage 3 -- the architecture without a VPC (SUPERSEDED IN PART -- READ SECTION 6 FIRST)

> **The two "Lambda Function URL" boxes below are wrong.** The gate in
> section 6 ran on 2026-09-04: resource-based policies do not grant
> invocation on this account, so CloudFront + Origin Access Control in
> front of a Function URL cannot work. Lambda itself is fine -- reach it
> through an **API Gateway HTTP API whose integration carries a
> `credentials` role**, which is measured working. Read section 6, then
> read this diagram with `Function URL` replaced by `API Gateway -> Lambda`.
> The S3 half, the storage model and the cost target are unaffected.

```
                          Phone
                            |
                     CloudFront            (free tier covers this
                            |               workload comfortably)
   /  /app.js  /admin.js ---+--> S3 origin, static SPA
   /analyze /navigate  -----+--> Lambda Function URL   (vision, stateless)
   /guidance /describe      |
   /admin/*  /recording/* --+--> Lambda Function URL   (walks, S3-backed)
                            |
   sim + mission routes ----+--> NOT IN AWS: local uvicorn now,
                                 the Pi after B5

   Bedrock  <- called by the vision Lambda, no VPC endpoint
   S3       <- corpus + static assets
```

Target fixed cost: **an estimated under $2/month** -- CloudFront ~$0,
Lambda ~$0 idle, S3 ~$0.10, SSM Parameter Store (Standard) free where
Secrets Manager was $0.40 each, and no ECR if the Lambdas are
zip-packaged. Unlike section 1's numbers this one is projected, not
measured, and should be checked against a real bill a month after stage 3
lands.

The CloudFront component is the one part with evidence: the existing
distribution cost **$0.005 in August 2026 and $0.0009 so far in
September** -- it already serves the whole app and rounds to nothing, so
the free-tier assumption is not doing much work.

### What has to be built

- `service/vision_analyze/app.py` and `control/admin_server.py` are
  FastAPI apps; **Mangum** wraps either for Lambda with little change.
- CloudFront behaviours replace the ALB `ListenerRule` path patterns.
  **Carry `tests/test_alb_routes.py`'s discipline across** -- it exists
  because "route added, page still loads, one feature silently dead" has
  shipped five times here. The failure mode is identical with CloudFront
  behaviours; only the mechanism changes.
- Route `/navigate` etc. to the Lambda origin **by behaviour, not CORS**.
  Same-origin, and it is the same path-pattern discipline already in
  place.
- `require_secret()` becomes a CloudFront Function on viewer-request.
- ARM64 (`arm64` Lambda architecture) -- ~20% cheaper, and consistent
  with every task definition here already being ARM64.
- Replay is a good candidate to become a job (POST returns an id, poll
  for the result). It already exceeds the 60s load-balancer timeout and
  is worked around by polling for a sidecar; Lambda makes fixing it
  natural rather than optional.
- **`GET /recording/walks/{walk}/download` will break on a naive Lambda
  port, and this is measured, not predicted.** A Function URL's buffered
  response is capped at 6MB. Zipping the real corpus:
  `red-backpack-20260829-184355` is **8.64MB** and
  `red-backpack-20260829-185029` is **6.68MB** -- **2 of 39 walks already
  exceed the limit**, and both are `red-backpack` walks, i.e. the ones
  most likely to be pulled for analysis. The fix is to redirect to a
  presigned S3 URL instead of proxying bytes through the function, which
  is better than the status quo regardless: it takes the whole corpus off
  the compute path. The JSON routes are nowhere near the ceiling (the
  walk list is 29KB) and the largest single frame is 147KB.

### Things NOT to move to AWS

The simulator, the robot API and the brain. `PLAN-brain-relocation.md`
already calls the ECS brain interim, and stage 4's done-when is a mission
running with no laptop on the network -- on the Pi. Fargate was a bridge
to that, priced like a destination. Locally they are two uvicorns;
remotely, Tailscale or a Cloudflare Tunnel gives them HTTPS without a
load balancer. Note that the twin page served from CloudFront calling a
Pi over a tunnel **is** cross-origin -- `robot/server.py`'s
`server.allowed_origins` already handles it, but the "same-origin, no
CORS" property only covers the S3/Lambda half.

---

## 6. The gate -- RUN 2026-09-04. Lambda survives, by a different door

**Result: Lambda resource-based policies do not grant invocation on this
account. Identity-based auth works normally, including public HTTPS
ingress.** This is not the constraint anyone thought it was, and it
invalidates the CloudFront-in-front-of-a-Function-URL design in section 5.

A throwaway function, a Function URL, a CloudFront distribution with
Origin Access Control and two IAM roles were created, exercised and
deleted. CloudWatch log lines -- not status codes -- were the
discriminator for "did the request reach the function":

| Auth path | Authorised by | Reached the function | Result |
|---|---|---|---|
| `aws lambda invoke` | identity (admin user) | yes | **200** |
| Function URL, SigV4 signed by that user | identity | yes, logged caller IP | **200** |
| Function URL `AuthType: NONE` | resource policy, `Principal: "*"` | **no** | 403 |
| CloudFront + OAC | resource policy, `Service: cloudfront.amazonaws.com` | **no** | 403 |
| Function URL, SigV4 by a role with **no** identity permissions | resource policy only | **no** | 403 |

The last row is the isolating experiment: same signing mechanism as the
row that returned 200, same account, differing only in whether the
authorisation came from an identity policy or the function's resource
policy. Only two `INGRESS-PROBE INVOKED` lines were ever logged, matching
the two identity-authenticated calls.

Ruled out along the way: it is not ingress (a signed call from a laptop
off the AWS network returned 200 and the function saw the real public
source IP); it is not the concurrency quota (1000, and 10 was always
ample for one phone); and it is not an SCP or RCP, because the account is
not a member of an AWS Organization.

### How far it reaches: Lambda only, tested

The obvious follow-up question -- is this resource-based policies in
general, or Lambda's? -- was measured the same day, because the whole
static half of section 5 rests on the answer. A **private** S3 bucket with
public access fully blocked, read by CloudFront through an S3 Origin
Access Control and a bucket policy granting `cloudfront.amazonaws.com`
with a `SourceArn` condition:

| Request | Result |
|---|---|
| Anonymous `GET` through CloudFront | **200**, real object content |
| Direct `GET` on the S3 URL | **403**, still private |

That is the identical *pattern* to the CloudFront-OAC row that fails on
Lambda -- service principal, resource policy, SourceArn condition -- and
it works. **The restriction is specific to Lambda resource policies.** S3
bucket policies are unaffected, so the SPA can be served from a private
bucket in the normal way and no public-read fallback is needed.

### What this kills

- **CloudFront -> Lambda Function URL with OAC** -- dead. OAC authorises
  via the function's resource policy.
- **Anonymous Function URLs** -- dead, same reason.
- **ALB -> Lambda target** -- dead, same reason.

Section 5's diagram cannot be built as drawn. Do not spend time on it
until the item below is settled.

### The pattern that works -- TESTED 2026-09-04, and Lambda survives

**An API Gateway HTTP API whose integration carries an explicit
`credentials` role ARN.** The gateway *assumes that role* and invokes the
function with identity-based auth, never consulting the resource policy
-- the one column in the table above that works.

Measured, in the same session, with the function carrying **no resource
policy at all** (`get-policy` returned `ResourceNotFoundException`), so
the credentials role was the only possible authorisation:

| Request | Result |
|---|---|
| Anonymous `GET` from a laptop off the AWS network | **200**, function invoked, real source IP logged |
| Anonymous `POST` with a JSON body | **200**, function invoked |
| Warm-path latency | ~0.25s |

All three request ids appeared in CloudWatch, so these were real
invocations and not an edge response. Note the `POST` in particular: it
carried a body without trouble, because API Gateway assumes a role rather
than signing the request, so none of the payload-signing problems that
afflict CloudFront OAC apply.

**So Stage 3 stays on Lambda.** The compute tier becomes:

    CloudFront ──> S3 origin            (static SPA)
              └──> API Gateway HTTP API (custom origin, ordinary
                        │                 CloudFront->public-endpoint hop,
                        │                 no OAC and no resource policy)
                        └──> Lambda, invoked via the integration's
                             credentials role

CloudFront is still wanted -- it puts the SPA and the API under one
origin, which keeps the path-behaviour discipline and avoids CORS -- but
it now fronts API Gateway rather than a Function URL, which is an
unremarkable custom-origin hop with none of the authorisation problems
above.

Cost impact is negligible: HTTP APIs are ~$1.00 per million requests,
against a workload measured in thousands per day. Keep the section 5
target of a couple of dollars a month.

### If a later change breaks that path

Fall back to a single always-on container with its own HTTPS endpoint and
no VPC: App Runner, or Lightsail containers. One service, not six. **The
~$5/month and ~$7/month figures usually quoted for those are carried over
from another analysis and have not been verified against current pricing
or us-east-2 availability** -- price them before relying on them, because
cheapness is the entire argument for this fallback.

### Unrelated, and still true

Deleting the NLB and making the ALB internet-facing is *not* gated on any
of this. On 2026-09-04 this project pulled the walk list and 39 walk
archives through the internet-facing NLB from a laptop off the AWS
network, every one HTTP 200 -- ELB ingress demonstrably works. (Do not
cite the ALB's ~21k requests/day as evidence for that: it is dominated by
health checks originating inside AWS.)

---

## 7. Working on this from a session with no AWS credentials

Stages 2 and 3 are AWS control-plane calls end to end, and the local
credentials are an IAM user with `AdministratorAccess` and two static,
non-expiring access keys. Those are not going into a sandbox. So the work
splits:

| Authorable with no credentials | Needs credentials (run locally) |
|---|---|
| Every CloudFormation template for stage 3 | The ~10 destructive calls in stage 2 |
| Mangum wrappers + Lambda handlers | The `cloudformation deploy` calls |
| The static-asset build for twin/admin | `s3 sync` of the SPA |
| A `teardown.sh` with the order in section 4 | Verification queries |
| The CloudFront-behaviour successor to `test_alb_routes.py` | The Function URL gate in section 6 |
| Tests -- the whole suite is AWS-free and mocked | |

That is roughly 90/10 in favour of authoring. **Stage 2 is
irreversible**, so a human watching `delete-stack` land in the right
order is worth more than the convenience of automating it.

`pytest tests/ -q` needs no AWS and no network (675 tests). It is the
correctness signal available to a credential-less session.

---

## 8. Facts that are easy to get wrong

- **Region is us-east-2**, for every stack in this project.
- **Account 303351622021.** The account also holds unrelated work
  (`bedrock-agentcore-*`, `model-orchestrator`, `llama-agent`,
  `qwen-agent`), so ECR and Secrets Manager totals are **not** all this
  project's.
- **The bucket name is `vision-picar-recordings-303351622021-us-east-2`**
  and the prefix is `recordings/`. S3 bucket names are globally unique,
  which is why it carries the account and region.
- **`admin_server.py`'s `_walk_recorded_at` has a pre-existing timezone
  bug**: naive `strptime().timestamp()` resolves in the process's local
  timezone, so the same walk yields different epochs on a Mac and in a
  UTC container. Harmless today (used only for sorting, every walk shifts
  equally) and a one-line fix (`timezone.utc`). It will make any
  local-vs-deployed comparison look broken when it is not.
- **A ListenerRule condition allows at most 5 path values**, which is why
  several stacks have two rules. If any ALB work happens before stage 2,
  that constraint still applies.
- **Every ECS task definition here is ARM64.** An amd64 image pushes to
  ECR without complaint and then fails at placement.
- **Cost Explorer's `Amazon Virtual Private Cloud` service line is
  endpoint hours + public IPv4**, not endpoint hours alone. The ~$8/month
  gap is IP addresses; endpoint *data processing* is $0.16/month.
