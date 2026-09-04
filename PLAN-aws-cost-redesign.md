# PLAN: the AWS redesign, and the cost that provoked it

Status as of 2026-09-04:

| Stage | What | Status |
|---|---|---|
| 1 | Recorded walks off EFS, onto S3 | **DONE** (commit `b42bee6`), data migrated and verified |
| 2 | Tear down the VPC and everything that needs one | **SPECIFIED, NOTHING DELETED** |
| 3 | Rebuild without a VPC | **SPECIFIED, NOTHING BUILT** |

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
| `SecretsManager` | 0.123 | **3.69** | 8 secrets, 6 of them this project's |
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
ARM Fargate in us-east-2 is $0.032384/vCPU-hr and $0.003556/GB-hr, so
0.25 vCPU + 0.5 GB is $0.009874/hr per task, x 6 x 24 = $1.422/day. When
the measured number matches the arithmetic to three decimals, that is a
day with no deployment on it.

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

And one component forces the whole chain: **EFS is the only thing in this
deployment that genuinely requires a VPC.** Bedrock, S3, Secrets Manager
and CloudWatch are all reachable from a Lambda with no VPC at all. EFS is
not. Remove it and the requirement dissolves -- which is why stage 1 was
first, and why it was worth doing even if stages 2 and 3 never happen.

The workload is one phone, a few hours a week, idle >95% of the time,
against a `/navigate` call that already takes 1-3s. Hourly-priced
always-on infrastructure is the wrong shape for it; cold starts are free
in practice.

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

## 5. Stage 3 -- the architecture without a VPC (SPECIFIED, NOTHING BUILT)

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

Target fixed cost: **under $2/month.** CloudFront ~$0, Lambda ~$0 idle,
S3 ~$0.10, SSM Parameter Store (Standard) free where Secrets Manager was
$0.40 each. No ECR if the Lambdas are zip-packaged.

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

## 6. The gate this whole design rests on, and it is untested

`README.md:504` documents why ECS exists: every public entry point into
this account was silently rejected before a Lambda ever ran, with
concurrency pinned at 10 instead of 1000.

**The quota is now 1000** (`aws lambda get-account-settings`, checked
2026-09-04). But a concurrency cap of 10 was always adequate for one
phone -- **the quota was never the real problem, the ingress rejection
was**, and the quota reading is evidence the account left a reduced-trust
tier, not evidence that ingress works.

**Stand up one throwaway Lambda Function URL and curl it from
off-network before committing to stage 3.** Ten minutes. If it fails, the
fallbacks that still avoid a VPC are App Runner (~$5/month, own HTTPS
endpoint) or Lightsail containers (~$7/month) -- one service, not six.

By contrast, deleting the NLB and making the ALB internet-facing is *not*
the same gate: `vision-picar-nlb` is internet-facing today and serving
~21k requests/day, so internet-facing ELB ingress has continuous positive
evidence on this account.

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
