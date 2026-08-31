# The Lab environment

A second, complete copy of every service, on its own ECS cluster and its
own load balancers, for experimental work that must not be able to touch
production. Stood up 2026-08-31.

**Guiding principle: nothing here may affect the production
environment.** That is why Lab has its own load balancers rather than
path-prefixed rules on production's ALB -- a `ListenerRule` on the prod
ALB would have made prod's `service` stack undeletable while Lab existed,
and a bad rule could shadow a route the twin depends on.

---

## What is shared, and why

Only `vision-picar-network` -- the VPC, its four subnets, and the six VPC
endpoints. Shared deliberately:

- It holds **no compute**. Nothing Lab runs can disturb it.
- Lab's imports are read-only; CloudFormation exports cannot be mutated
  by an importing stack.
- The interface endpoints cost ~$36/month. Duplicating them buys nothing:
  the endpoint security group already admits the whole VPC CIDR
  (`CidrIp: !Ref VpcCidr`), so Lab tasks reach ECR, Secrets Manager,
  CloudWatch and Bedrock with **no change to the network stack at all**.

Everything else is Lab's own: cluster, NLB, internal ALB, ALB listener,
security groups, ECR repos, IAM roles, secrets, log groups, EFS volume.

## What is *not* isolated, and cannot be

**Bedrock quota.** Model invocation limits are account-and-region scoped,
so a Lab experiment firing concurrent `/navigate` calls contends with
production no matter which cluster it runs on. This is not theoretical --
`control/walk_replay.py` records that at 8 workers, "Bedrock throttled 10
of 22 frames."

Only a separate AWS account fixes that. Until then the control is a
concurrency cap in the client, and not running load experiments while
production is being demonstrated.

---

## How it was built

`NamePrefix` (added 2026-08-31) is the whole mechanism. Every physical
resource name in `service`/`twin`/`brain`/`admin`/`teleop-*` is
`!Sub "${NamePrefix}-..."`, defaulting to `vision-picar`. Lab passes
`vp-lab` -- deliberately short, because ELB and target-group names cap at
32 characters and `vision-picar-lab-teleop-brain-tg` is exactly 32.

`recordings.yaml` needed no change: it already names its exports from
`AWS::StackName` and tags the filesystem rather than naming resources.

**The default was verified to be a no-op for production two ways**: by
rendering every `!Sub` at the default and diffing against the previous
hardcoded values (52/52 identical), and by submitting the new `twin.yaml`
as a change set against the live production stack, which CloudFormation
refused with *"The submitted information didn't contain changes."*

Note this reverses `PLAN-teleop-robot.md`'s choice to copy templates
rather than parameterize them ("chosen over the DRY alternative
specifically so a mistake here can't touch the already-running
deployment"). The concern was right; the mitigation here is verification
rather than duplication, and the change-set check above is what makes
that trade acceptable.

---

## Stacks, in dependency order

Every stack takes `NamePrefix=vp-lab` and
`NetworkStackName=vision-picar-network`; all but `recordings` also take
`ServiceStackName=vp-lab-service`.

| Stack | Owns | Extra parameters |
|---|---|---|
| `vp-lab-service` | cluster, NLB, ALB, shared secret, IAM roles, vision-analyze | -- |
| `vp-lab-recordings` | EFS volume + access point | (network only) |
| `vp-lab-twin` | robot server (`mode: sim`) + the web twin | -- |
| `vp-lab-brain` | `control/brain_server.py` | `RecordingsStackName`, `RobotSecretArn`, `VisionSecretArn` |
| `vp-lab-admin` | recorded-walk console | `RecordingsStackName`, `VisionSecretArn` |
| `vp-lab-teleop-robot` | robot server (`mode: teleop`) | `RoutePrefix=/teleop-robot` |
| `vp-lab-teleop-brain` | second brain for teleop | `RoutePrefix`, `RobotRoutePrefix`, three secret ARNs |

**The bootstrap dance applies to every stack that creates its own ECR
repo**: deploy with `DesiredCount=0`, push the image, redeploy with
`DesiredCount=1`. Otherwise the `ECS::Service` cannot stabilize and
CloudFormation rolls the stack back, deleting the freshly created repo
with it. `service.yaml` and `twin.yaml` gained that knob for this work;
the others already had it.

## Images

Four Dockerfiles fill six repos -- teleop reuses the twin and brain
images, exactly as production does:

| ECR repo | Built from |
|---|---|
| `vp-lab-analyze` | `service/vision_analyze/Dockerfile` (context: that directory) |
| `vp-lab-twin`, `vp-lab-teleop-robot` | `service/twin/Dockerfile` (context: repo root) |
| `vp-lab-brain`, `vp-lab-teleop-brain` | `service/brain/Dockerfile` (context: repo root) |
| `vp-lab-admin` | `service/admin/Dockerfile` (context: repo root) |

**Build ARM64 or the task will not place**: `docker build --platform
linux/arm64 ...`. An amd64 image pushes to ECR without complaint and then
fails with `CannotPullContainerError: image Manifest does not contain
descriptor matching platform 'linux/arm64 v8'`.

---

## Reaching it

    http://vp-lab-nlb-a7ac2b732997cf4b.elb.us-east-2.amazonaws.com

Same route set as production: `/` and `/health` (twin), `/navigate`,
`/analyze`, `/guidance`, `/navigate/models` (vision), `/mission/*`
(brain), `/admin` (console), `/teleop-robot/*`, `/teleop-brain/*`.

There is **no CloudFront distribution for Lab**, so this is plain HTTP.
That matters for one thing only: `getUserMedia` requires a secure
context, so **the Guide tab's camera will not start against the Lab
URL from a phone.** Options, cheapest first: test camera work at
`http://localhost` (a secure context by definition), or add a
`vp-lab-cdn` stack modeled on `cloudformation/cdn.yaml`. Everything that
doesn't need the camera -- the Sim tab, the brain panel, the admin
console -- works over plain HTTP today.

Each Lab service has its **own** `x-app-secret`, unrelated to
production's. Read one with:

    aws secretsmanager get-secret-value --secret-id \
      arn:aws:secretsmanager:us-east-2:303351622021:secret:vp-lab-app-shared-secret-avAeTr \
      --query SecretString --output text

## Cost

The two load balancers are ~$32/month and **cannot scale to zero** --
that is the price of the isolation. The six Fargate tasks are the rest;
scale them down between sessions:

    for s in service twin brain admin teleop-robot teleop-brain; do
      aws ecs update-service --cluster vp-lab-cluster \
        --service vp-lab-$s-service --desired-count 0
    done

To remove the environment entirely, delete the stacks in reverse
dependency order (`teleop-brain`, `teleop-robot`, `admin`, `brain`,
`twin`, `recordings`, `service`). ECR repos with images in them block
deletion; empty them first.

## Prod, for comparison

Production is entirely untouched by any of this: `vision-picar-cluster`,
`vision-picar-nlb-0b11f5a12277793e...`, and every `vision-picar-*` stack
still at the revision it had before. Verified after the Lab build by
diffing task-definition revisions and stack `LastUpdatedTime`s.
