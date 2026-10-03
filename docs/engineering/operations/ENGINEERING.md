---
kind: engineering
domain: operations
status: current
verified: 2026-10-02
parent: docs/operations/ARCHITECTURE.md
---

# Operations -- engineering

How the system is deployed, reached and checked today. The what and the
why are in the [architecture spec](../../operations/ARCHITECTURE.md). This
document is true only until the implementation changes. No secret values,
account IDs or hostnames appear here: each is named by the stack output or
file that holds it.

## Implementation

**Live infrastructure** (region `us-east-2`, all CloudFormation):

| Stack (default name) | Template | Holds |
|---|---|---|
| `vision-picar-serverless` | `cloudformation/serverless.yaml` | CloudFront distribution; private static S3 bucket read through an Origin Access Control; API Gateway HTTP API (`$default` stage, auto-deploy); `VisionFunction` and `WalksFunction` (Python 3.12, arm64, 1024 MB); `ApiInvokeRole`, the credentials role the gateway assumes |
| `vision-picar-recordings-s3` | `cloudformation/recordings-s3.yaml` | The walk bucket. Exports `<stack>-BucketName`, `-BucketArn`, `-AccessPolicyArn`. Imports nothing |
| deploy bucket | `cloudformation/deploy-bucket.yaml` | The Lambda zips. Adopted by resource import, not recreated. Exports `<stack>-BucketName` |

**Deleted 2026-09-05, kept as history only** (they back nothing):
`cloudformation/network.yaml`, `service.yaml`, `twin.yaml`, `brain.yaml`,
`admin.yaml`, `recordings.yaml`, `cdn.yaml`, `teleop-robot.yaml`,
`teleop-brain.yaml`. Their images are `service/twin/`, `service/brain/` and
`service/admin/`; `service/twin/` is kept for B5.

**Build and publish scripts:**

| File | What it does |
|---|---|
| `service/lambda/build.sh` | Builds two zips with `pip --platform manylinux2014_aarch64 --only-binary=:all:` for cp3.12. The vision zip is flat: `service/vision_analyze/` app files plus `service/lambda/vision_handler.py`. The walks zip carries all of `control/*.py`, `control/admin.html`/`.js`, `config/robot.yaml` and `service/lambda/walks_handler.py`. Uploads to `s3://<bucket>/lambda/<name>-<UTC stamp>.zip` and prints the deploy command |
| `service/static/sync.sh` | Uploads each file in `service/static/assets.json` with `put-object` and an explicit content type and cache policy, then invalidates `/*`. It does not use `aws s3 sync`, which would guess a type for the extensionless `admin` and `metrics` keys |
| `service/static/assets.json` | The site: `index.html`, `app.js`, `manifest.json`, icons, `admin`, `admin.js`, `metrics`, `metrics.js`. Cache `none` = `no-cache, must-revalidate`; `day` = `max-age=86400` |

**The tunnel** (`service/tunnel/`):

| File | What it does |
|---|---|
| `service/tunnel/run.sh` | Sources the secrets files, exports the variables below, sets `WORLD_MODE` from `ROBOT_MODE` (sim -> sim, anything else -> none), then starts three uvicorns on 127.0.0.1: robot :8000, brain :8001 with `ROUTE_PREFIX=/brain`, proxy :8080. Its cleanup kills only its own children, so the ngrok agent survives |
| `service/tunnel/proxy.py` | FastAPI catch-all. `/brain/*` -> brain; `/vision/*` -> the deployed vision service, answering `OPTIONS` itself with 204; everything else -> robot. On the REQUEST it drops `host` and `content-length`; on the RESPONSE it drops `content-encoding`, `content-length`, `transfer-encoding` and `connection`, and adds permissive CORS headers. An upstream error becomes 502 `upstream <url> unreachable`. Timeout 120 s |
| `service/tunnel/restart.sh` | Stops whatever holds :8000/:8001/:8080 (graceful, then `kill -9` after 5 s), restarts `run.sh` detached (log `~/.vision-picar-tunnel.log`), then polls both `/health` routes until each reports the checkout's `git rev-parse --short HEAD` |

ngrok itself is configured outside the repo: `ngrok start picar` uses the
user's own ngrok config, pointing at :8080.

**Health and identity:**

| File | What it does |
|---|---|
| `control/health.py` | `python -m control.health`. `check_robot()`, `check_brain()`, `check()`, `render()`. Exit 0 on OK, 1 otherwise |
| `robot/identity.py` | `identity(service, config_path)` -> `{service, git_revision, executable, config_path}`; `log_identity()` logs it at WARNING. Revision from `GIT_REVISION`, else `git rev-parse --short HEAD`, else `unknown` |

`robot/server.py` and `control/brain_server.py` each call `log_identity()`
at start-up and publish the result as `identity` on `GET /health`.

**Mission metrics:**

| File | What it does |
|---|---|
| `control/metrics_client.py` | `row_for(status, git_revision=, config=, walk=)` builds a row from `MissionRunner.status()`. `ship_run()` POSTs it and never raises. `ship_run_async()` does it on a daemon thread, which is how "off the mission's path" is met |
| `control/metrics_routes.py` | `register_metrics_routes(app, store, require_secret)`, mounted by `control/admin_server.py` and so by the walks Lambda |
| `control/metrics.html`, `control/metrics.js` | The dashboard, served at `/metrics` from the static bucket |

`MissionRunner._ship_metrics()` (`control/mission_runner.py`) calls the
shipper last, after the mission has ended.

## Interfaces

**Public routes** (CloudFront behaviour and API Gateway route both
required):

| Path | Function | Notes |
|---|---|---|
| `POST /analyze`, `POST /describe`, `POST /navigate`, `GET /navigate/models`, `POST /guidance`, `GET /health` | vision | cloud-vision owns their behaviour |
| `ANY /recording/{proxy+}`, `GET /stats` | walks | recordings owns their behaviour |
| `ANY /metrics/{proxy+}` | walks | see below |
| everything else | static bucket | default behaviour, `index.html` at `/` |

**Metrics routes** (`x-app-secret` = the walks secret):

| Method, path | Request | Response |
|---|---|---|
| `POST /metrics/runs` | `MetricsRun`: `run_id` (matches `^[A-Za-z0-9][A-Za-z0-9._-]{0,120}$`), `started_at`, `finished_at`, `git_revision`, `policy`, `config`, `outcome`, `steps`, `target_object`, `walk`, `stats` (the tier's stats verbatim; since `50b2293` `cloud_calls` includes one `arrival_confirmation` call per tiered arrival, so a release comparison across that commit shows the step). Extra fields are refused (422); a `run_id` that fails the pattern is a 400 | `{stored, day}`. Stored in the walk store as container `metrics-YYYY-MM-DD` (one per UTC day, beside the walks), object `<run_id>.json` |
| `GET /metrics/summary?days=14` | `days` clamped to 1..90 | `{days, runs, count}`, newest first, rows whole, never aggregated |

**Health routes** (unauthenticated on both servers):

| Server | Verdict inputs | Description fields |
|---|---|---|
| Robot `GET /health` | `seconds_since_watchdog_poll` against `watchdog_poll_interval_s` x 10; and `drive.ros_up` false when `drive.mode` is `ros`, named by half (`drive.bridge_up` false: the bridge; else the plugin's posts, with `drive.ros_post_age_s`) | `mode`, `seconds_since_last_command`, `watchdog_timeout_s`, `driver`, `authority_holder`, `last_refusal`, `env_label`; also on the route: `drive`, `wheel_loop`, `motor_board`, `min_distance_cm`, `sim_map`, `refusal_counts`, `identity` |
| Brain `GET /health` (under `/brain` behind the tunnel) | `mission_running` and `seconds_since_last_tick` against `tick_timeout_s` | `robot_url`, `tick_rate_hz`, `navigate_model_id`, `navigate_prompt_variant`; also on the route, among others: `drills_allowed`, `identity`, `perception_available`, the `perception_*` settings, the `tier_*` settings (`tier_consecutive_frames` and others), `recording_allowed`, `faults` (`control/brain_server.py`'s `health()` is the full list) |

`control.health` output: `{status: ok|unhealthy, failed: [names],
parts: [{name, url, status: ok|unhealthy|unreachable, problems, identity,
description}]}`.

**Authentication.** `require_secret()` on the robot server, the brain, the
walks service and the vision service compares the `x-app-secret` header
with that process's `APP_SHARED_SECRET`. It is inert when the variable is
unset. `/health` stays open on both servers, and so does `/` (the twin
page) on the robot server.

## Parameters and configuration

| Variable or constant | Default | Unit | Read in | Why |
|---|---|---|---|---|
| `APP_SHARED_SECRET` | unset (gate inert) | secret | every server's `require_secret()`; `robot/factory.py` and `world/factory.py` for the ROS bridge | Gates inbound calls. `run.sh` sets it to `LOCAL_SECRET` |
| `ROBOT_SHARED_SECRET` | `APP_SHARED_SECRET` | secret | `control/brain_server.py` | The brain calling the robot. Same value locally |
| `VISION_SHARED_SECRET` | `APP_SHARED_SECRET` | secret | `control/brain_server.py`, `control/admin_server.py` | The deployed vision service has its own secret; sending the local one would 401. `run.sh` sets it from `VISION_SECRET` |
| `WALKS_SHARED_SECRET` | empty | secret | `control/brain_server.py` (metrics fallback); `build.sh`'s printed deploy command | The walks and metrics secret. Reviewing walks is a separate privilege |
| `METRICS_URL` / `METRICS_SECRET` | empty / empty | URL / secret | `control/brain_config.py` (overrides `brain.metrics_url` / `brain.metrics_secret`) | Empty disables shipping, so tests and laptops post nothing. `run.sh` points it at the vision URL with `WALKS_SECRET` |
| `VISION_URL` | config `brain.vision_url` (empty) | URL | `control/brain_config.py`, `control/admin_server.py` | `run.sh` defaults it to a hard-coded literal: the distribution's address as it was when written, not read from the `SiteUrl` output (see Known gaps) |
| `ROUTE_PREFIX` | empty | path | `robot/server.py`, `control/brain_server.py` | Lets two servers share one front door. The brain runs with `/brain` |
| `PROXY_ROBOT_URL` / `PROXY_BRAIN_URL` / `PROXY_VISION_URL` | `http://127.0.0.1:8000` / `:8001` / a hard-coded literal, as `VISION_URL` | URL | `service/tunnel/proxy.py` | Upstreams of the fan-out |
| `ENV_LABEL` | unset | text | robot server; the walks function (`control/admin_server.py`). Both Lambdas receive it from the `EnvLabel` stack parameter, but the vision function never reads it | Shows a non-production banner on the twin |
| `GIT_REVISION` | unset | text | `robot/identity.py` | Set at image build time; no `.git` in a container |
| `~/.vision-picar-local-secrets` | must exist (mode 600) | file | `run.sh`, `restart.sh` | Holds `LOCAL_SECRET` and `VISION_SECRET`. Never in the repo |
| `~/.vision-picar-serverless-secrets` | optional | file | `run.sh` | Holds `WALKS_SECRET` for metrics |
| `NavigateModelId` | `us.anthropic.claude-opus-4-5-20251101-v1:0` | model id | `serverless.yaml` -> `BEDROCK_NAVIGATE_MODEL_ID` | Chosen by measurement; owned by cloud-vision. Keep in step with the code default |
| `VisionSharedSecret` / `WalksSharedSecret` | `""` | secret | `serverless.yaml` | Empty means the function runs with NO auth. `build.sh` says so |
| `VisionFunction` `Timeout` | 30 | s | `serverless.yaml` | A vision call is 1-3 s and Robot view allows two in flight |
| `WalksFunction` `Timeout` | 900 | s | `serverless.yaml` | Lambda's ceiling, sized for a replay, which re-asks every frame and outlived the old 60 s load-balancer limit. As templated, deployed replay is disabled anyway (see Known gaps) |
| `ExpireAfterDays` (deploy bucket) | 30 | days | `deploy-bucket.yaml` | The zips are the only pre-built rollback path, at about 39 MB a build |
| `WATCHDOG_POLL_SLACK` | 10 | polls | `control/health.py` | The watchdog loop wakes every 0.1 s (`WATCHDOG_POLL_INTERVAL_S`); a stall is ten missed polls, 1 s |
| `DEFAULT_TIMEOUT_S` | 5.0 | s | `control/health.py`, `control/metrics_client.py` | One health or metrics request |
| `MAX_DAYS` | 90 | days | `control/metrics_routes.py` | Bounds a summary to 90 LIST calls |

**Ports.** Default local stack: robot :8000, brain :8001, tunnel proxy
:8080, ROS bridge :8090, foxglove 127.0.0.1:8765. When another session owns
those, run your own stack on robot :8100, brain :8101, bridge :8190 with
`ROS_DOMAIN_ID=73`, bind to 127.0.0.1, and point the live tests at a dead
port so they cannot drive someone else's robot:

```bash
PICAR_ROBOT_URL=http://127.0.0.1:9 PICAR_BRAIN_URL=http://127.0.0.1:9/brain \
PICAR_BRIDGE_URL=http://127.0.0.1:9 PICAR_ROS_CONTAINER=no-such-container \
  pytest tests/ -q
```

## Procedures

**Deploy the functions** (from the repo root, with AWS credentials):

```bash
set -a; source ~/.vision-picar-local-secrets
[ -f ~/.vision-picar-serverless-secrets ] && source ~/.vision-picar-serverless-secrets; set +a
export VISION_SHARED_SECRET="$VISION_SECRET" WALKS_SHARED_SECRET="$WALKS_SECRET"
bash service/lambda/build.sh <deploy-bucket> [region]   # exits 2 before building if either is empty   # bucket: deploy stack's DeployBucketName output; region defaults to us-east-2
# then run the "Deploy with:" command it prints, in THIS shell
```

**The export line is not optional, and `build.sh` enforces it** (handoff
4i): with either variable empty it prints `STOP: <name> is empty ...` on
stderr and exits 2 before building or uploading anything. The printed command expands
`$VISION_SHARED_SECRET` and `$WALKS_SHARED_SECRET`
(`service/lambda/build.sh`), but the secrets files define `VISION_SECRET`
and `WALKS_SECRET`. Pasted without the exports, both parameters deploy
empty, and `cloudformation/serverless.yaml` then omits each function's
`APP_SHARED_SECRET`, which means **no authentication** on either function.
Failure signature: an unauthenticated request to a gated route succeeds
instead of answering 401. Check the deployed functions' variable NAMES
(not values) with `aws lambda get-function-configuration --function-name
<name> --query 'keys(Environment.Variables)'`, taking each name from
`aws cloudformation describe-stack-resource --stack-name
vision-picar-serverless --logical-resource-id VisionFunction` (and
`WalksFunction`) `--query StackResourceDetail.PhysicalResourceId`; each
list must include `APP_SHARED_SECRET`.

Expected: `vision: <n>MB -> s3://...` and `walks: <n>MB -> s3://...`, then
the command. It deploys `cloudformation/serverless.yaml` to the stack
`vision-picar-serverless` with `--capabilities CAPABILITY_NAMED_IAM` and
`--region` set; run it as printed, because the zips were uploaded to that
region. This is the canonical deploy procedure; the cloud-vision and
recordings specs link here. A missing-`.so` import error at cold start means a wheel for
the wrong architecture got in. `build.sh` copies `config/robot.yaml` and
`control/brain_config.py` from one tree on purpose: `load_brain_config()`
rejects unknown keys, so a mismatched pair is a cold-start `ValueError`.

**Publish the twin and consoles:**

```bash
aws cloudformation describe-stacks --stack-name vision-picar-serverless \
  --query 'Stacks[0].Outputs' --output table     # StaticBucketName, DistributionId
bash service/static/sync.sh <StaticBucketName> <DistributionId>
```

Expected: one line per asset, then the invalidation id and `InProgress`.
Wait for `Completed` before testing. `MISSING: <src>` means the manifest
names a file that does not exist. Check parity afterwards by diffing
`curl -s <SiteUrl>app.js` against `git show HEAD:web-twin/app.js`. The
`SiteUrl` output already carries the `https://` scheme and the trailing
slash, so do not add either.

**Run the local half for the deployed twin:**

```bash
bash service/tunnel/run.sh                      # or ROBOT_MODE=teleop bash service/tunnel/run.sh
ngrok start picar                               # in another terminal
```

In the twin's Settings: robot URL `https://<tunnel domain>`, brain URL
`https://<tunnel domain>/brain`, the local secret in both. Before a rig
walk, warm the perception models once, as the perception spec's
Procedures say (docs/engineering/perception/ENGINEERING.md), or the first
`POST /mission/start` sits on "Starting..." while weights download. Failure signatures: `missing ~/.vision-picar-local-secrets` from
`run.sh`; `ERR_NGROK_334` from a second tunnel on the domain; a bare "Load
failed" on the phone when the servers were restarted under it.

**Restart after a code change** (never a plain `kill`):

```bash
bash service/tunnel/restart.sh
```

Expected: `OK: robot and brain both running <rev>`. `FAILED: expected both
servers on <rev>` prints the log's errors.

**Check health:**

```bash
python -m control.health                                   # two bare uvicorns on :8000 / :8001
python -m control.health --robot-url http://127.0.0.1:8000 \
  --brain-url http://127.0.0.1:8001/brain --secret "$LOCAL_SECRET"   # under service/tunnel/run.sh
```

Under `service/tunnel/run.sh` the brain serves under `ROUTE_PREFIX=/brain`,
so the bare command's default brain URL gets a 404 and reports the brain
`unreachable`: pass `--brain-url .../brain` as above. Add `--json` for the
machine-readable form. Expected first line `OK`, then each part with `build git=<rev>
exe=<python>`. `UNHEALTHY  (brain)` with the robot ok when the brain is
down. A robot parked against a wall stays `OK`.

**B5 on the Jetson (planned, not built).** `PLAN-brain-relocation.md` B5
lists what the units must cover: the robot server first, and the ROS
container started only after `GET /wheels` reports `usable: true` (the
order docs/engineering/ros/ENGINEERING.md's Procedures and
`service/slam/README.md` section 3 require until
`HANDOFF-2026-10-02-spec-review.md` item 2a is fixed; a plain `After=` on
the robot server's unit is not enough, the unit must wait on `/wheels`), an
`EnvironmentFile` for `APP_SHARED_SECRET`, `ROBOT_MODE`,
`ROBOT_DRIVE`, `WORLD_MODE` and `ROBOT_SERIAL`, `dialout` group and a udev
rule for the motor board, and `Restart=on-failure` that never brings back
anything that moves on its own. There is no `deploy` directory yet. The
bring-up that precedes it is the platform domain's procedure
(`tools/jetson/README.md`).

## Verification

| Test | What it proves |
|---|---|
| `tests/test_serverless_routes.py` | Every public route crosses both the CloudFront behaviour and the API Gateway route to the right function; the vision zip's flat layout imports (6 tests) |
| `tests/test_static_assets.py` | Every file `web-twin/index.html` and `control/admin.html` reference is in `service/static/assets.json`, plus the PWA manifest's icons, the manifest's keys and cache policies, and that `sync.sh` reads the same manifest (10 tests). `control/metrics.html` is not checked |
| `tests/test_alb_routes.py` | The same check against the deleted ECS templates. History, still green |
| `tests/test_tunnel_proxy.py` | The proxy's prefix routing and header handling, no network (8 tests) |
| `tests/test_health.py` | Non-zero exit when either half is unreachable or broken, and the verdict's silence for a parked robot, an idle robot and a recorded veto (15 tests) |
| `tests/test_metrics.py` | Store and read a run, refuse unknown fields, the client never raises, and percentiles are never blended (14 tests) |
| `tests/test_brain_server.py` | The brain imports no backend or simulator (checked in a subprocess) |
| `tests/test_watchdog_integration.py` | The watchdog loop runs against a real uvicorn subprocess |

Checklist for an operations change:

1. A new public route: add the CloudFront behaviour and the API Gateway
   route, then run `pytest tests/test_serverless_routes.py`.
2. A new page asset: add it to `service/static/assets.json`, then run
   `pytest tests/test_static_assets.py`.
3. A new `/health` field: put it in the verdict list or the description
   list in `control/health.py` deliberately, and pin it in
   `tests/test_health.py`.
4. After any deploy, verify against the deployed thing (bucket, function,
   distribution), not the template.

## Known gaps

The open design questions (B5, CORS on the serverless stack, replay as a
job, map backup, M11 rollback) are in the
[architecture spec's open questions](../../operations/ARCHITECTURE.md#open-questions)
and are not repeated here. These are the implementation gaps:

- **Deployed replay is disabled as templated, and would fail on auth if
  half-enabled.** Both halves (no `VISION_URL`, so a 503; `VISION_URL`
  alone, so the walks secret is sent to the vision service and every frame
  is refused) are recorded in the recordings spec's Known gaps
  (docs/engineering/recordings/ENGINEERING.md), the canonical home. The
  fix touches `WalksFunction` in `cloudformation/serverless.yaml`, which
  this domain deploys.
- **`control/metrics.html` is not asset-checked.** `tests/test_static_assets.py`
  walks the twin and the walk console only, so a file the metrics
  dashboard references could be missing from `service/static/assets.json`
  without a test failing.
- **Hostnames are hard-coded as defaults** in `service/tunnel/run.sh`
  (`VISION_URL`) and `service/tunnel/proxy.py` (`PROXY_VISION_URL`). They
  should come from the `SiteUrl` output; a new distribution would leave both
  pointing at the old one.
- **`run.sh`'s warm-up hint is stale.** It says the first mission downloads
  `yolo11s.pt` (18 MB); the shipped detector is `yoloe-11s-seg.pt`, plus a
  text encoder of about 572 MB (`tools/jetson/README.md`).
- **`control/health.py` reads no wheel-loop field.** Late safety ticks
  (`wheel_loop.late_ticks`) show only on the robot's health route.
- **`control.health`'s default brain URL does not match `run.sh`.** The
  default is `http://127.0.0.1:8001` with no prefix, so against the tunnel
  stack it needs `--brain-url .../brain` (see Procedures).
- **No `deploy` directory for B5 yet**, so the deployed twin needs a laptop
  running `service/tunnel/run.sh` and ngrok.
