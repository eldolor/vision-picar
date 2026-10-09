---
kind: engineering
domain: recordings
status: current
verified: 2026-10-02
parent: docs/recordings/ARCHITECTURE.md
---

# Recordings -- engineering

How recorded walks are written, stored, scored, replayed and labelled
today. The what and the why are in the
[architecture spec](../../recordings/ARCHITECTURE.md). This document is
true only until the implementation changes, and it is updated in the same
commit as the code.

## Implementation

| File | What it does |
|---|---|
| `control/recording_routes.py` | `mount_recording_routes(app, store, *, prefix, require_secret, allow_recording, proxy)` adds `POST /recording/frame` and `POST /recording/finish`. It holds the naming contract (`WALK_NAME`) and the size caps. |
| `control/walk_store.py` | `WalkStore` (abstract), `LocalWalkStore(root)`, `S3WalkStore(bucket, prefix="recordings", client=None, export_prefix="exports")`, and `walk_store_from_config(config)`, the only place that picks a backend. `SAFE_NAME` guards every walk and file name on both backends. `list_walks()` skips `METRICS_PREFIX` (`metrics-`) containers on both backends, and `admin_server._require_walk()` answers 404 for one, so no walk route reads, replays or deletes a day of metrics (handoff 4c). |
| `control/admin_server.py` | The walks service: `create_app(config_path=None, store=None)`. Mounts the write routes and the metrics routes, then the review API and `/admin` + `/admin.js`. Its own `require_secret` reads `APP_SHARED_SECRET`. |
| `control/admin.html`, `control/admin.js` | The review console. Deployed from the static bucket at `/admin` and `/admin.js` (`service/static/assets.json`). |
| `control/walk_eval.py` | `compute_metrics(entries)`, `metric_flags()`, `judge_walk()`, `check_collisions()`, `score_walk(metrics, judge, collisions)`. Pure apart from the injected Bedrock client. |
| `control/walk_replay.py` | `replay_walk(entries, frame_bytes_for, target_object, post_navigate, model_id, prompt_variant, max_workers)`, which returns entries in `walk.jsonl` shape plus a per-frame diff |
| `control/label_assist.py` | CLI and functions: `propose()` (OWLv2 crops + CLIP), `propose_cloud()` (a Bedrock VLM), `assert_absent()` (walk-level negative). Writes `labels.candidate.json`; only `assert_absent()` produces a `labels.json` document. |
| `control/brain_server.py` | Mounts the same write routes locally, with `allow_recording` read per request and `_proxy_recording()` forwarding `/recording/*` to `recording_proxy_url`. `/health` publishes `recording_allowed`. |
| `service/lambda/walks_handler.py` | `Mangum(create_app(...))`, built at module level, then `_assert_configured()`. With `RECORDING_BACKEND=s3` and no `RECORDING_BUCKET`, the import fails first inside `create_app()`: `walk_store_from_config()` constructs `S3WalkStore("")`, which raises `WalkStoreError: S3WalkStore needs a bucket name.`, so `_assert_configured()`'s own `RuntimeError` never runs (checked 2026-10-02 by a dry import with `mangum` stubbed). Either way the function refuses to start. With `RECORDING_BACKEND` unset it starts on the `local` backend and writes to its ephemeral disk (Known gaps). The template sets `s3`. |
| `cloudformation/recordings-s3.yaml` | The bucket and its `RecordingsAccessPolicy`. Exports `<stack>-BucketName`, `-BucketArn`, `-AccessPolicyArn`. |
| `cloudformation/serverless.yaml` | `WalksFunction`, `WalksRole` (the imported access policy plus `bedrock:InvokeModel`), routes `ANY /recording/{proxy+}`, `GET /stats`, `ANY /metrics/{proxy+}` |

Readers owned by other domains: `control/perception_eval.py`
(perception), `sim/replay_robot.py` (playback as a robot),
`tests/demo_replay_mission.py` and `tests/manual_replay_navigate.py`.
`control/metrics_routes.py` and `control/metrics_client.py` (operations)
store one row per mission in the same `WalkStore`, under containers named
`metrics-YYYY-MM-DD`.

## Interfaces

**Write routes** (on the brain under its `ROUTE_PREFIX`, and on the walks
service; secret-gated):

| Route | Request | 200 response | Errors |
|---|---|---|---|
| `POST /recording/frame` | `walk`, `seq` (0-9999), `image_base64`, `media_type` (default `image/jpeg`), `navigate` (dict or null), `teleop_seq` (int or null) | `saved` (`frame-NNNN.jpg`), `walk`, `frames`, `dir` (path or `s3://` URI) | 400 bad name, seq or base64; 403 recording disabled with no proxy; 409 at 500 frames; 413 frame over 4 MiB; 502 proxy unreachable (brain only) |
| `POST /recording/finish` | `walk`, `model_id`, `target_object`, `capture_width`, `capture_height` | `walk`, `meta` | 400; 403; 404 no such walk |

**Review routes** (walks service; all secret-gated except the two health
routes and the page routes):

| Route | Does |
|---|---|
| `GET /health`, `GET /recording/health` | `status`, `recording_dir`, `recording_dir_exists`, `env_label`. Two paths because API Gateway sends `/health` to the vision function. |
| `GET /recording/walks` | Per walk: `frames`, `bytes`, `label`, `model_id`, `finished`, `recorded_at`, `eval` summary, `replays` summaries |
| `GET /recording/walks/{walk}` | `frames[]`, `entries` (`walk.jsonl` parsed), `label`, `model_id`, `meta`, `eval` |
| `GET /recording/walks/{walk}/frames/{file}` | Image bytes. `file` must match `frame-\d{4}\.(jpg\|png\|webp)`. |
| `GET /recording/walks/{walk}/download` | Zip. 307 to a presigned URL on S3, streamed inline locally. |
| `PUT /recording/walks/{walk}/tag` | `{label}` in `good`, `bad`, `training-ready`; empty or null clears it. 400 otherwise. |
| `PUT /recording/walks/{walk}/meta` | Merges `model_id`, `target_object`, `note` into `meta.json` |
| `POST /recording/walks/{walk}/evaluate?judge=true` | Recomputes and overwrites `eval.json` (manual) |
| `GET /recording/walks/{walk}/evaluation` | `eval.json` if its `schema` is current, else computes it (with the judge, when enabled) and stores it. This lazy read is the ONLY automatic scoring path: `POST /recording/finish` writes `meta.json` and never scores. `GET /recording/walks` only reads an existing `eval.json`, and marks one whose `schema` is not current `stale: true` (3.64); the console shows it as "old scorer" and never re-scores it by itself (that can be a paid judge run). The console calls this route on page load for FINISHED walks only (`scorePendingWalks()` in `control/admin.js` filters `w.finished && !w.eval`), one at a time; an unfinished walk (no `meta.json`) is never scored automatically, only by the Evaluate button (`POST .../evaluate`). Any other client that GETs this route on an unfinished walk does score it. |
| `POST /recording/walks/{walk}/replay` | `{model_id, prompt_variant}`, synchronous. Each frame goes to `/navigate` through `post_navigate()`, which always sends `media_type: image/jpeg` (even for a `.png` or `.webp` frame) and never sends `searched_rooms` (Known gaps). 503 only when no vision URL is configured (checked before any frame). Otherwise per-frame failures are swallowed by `replay_walk()`, counted in `errors` and described in `diff[].error`; below `REPLAY_MIN_COVERAGE` the sidecar is stored with `verdict: unusable`, `score: null`, flag `incomplete`, and the route returns **200** with that record. A vision service that is entirely down therefore yields a 200 `unusable` replay, not a 503. **An unusable replay never replaces a scored one of the current schema** (3.64): that sidecar is kept, gains `last_unusable{replayed_at, coverage, frames, errors}`, and is what the route returns. The console snapshots the stored record's stamp (`replayed_at` and `last_unusable.replayed_at`) before the POST and, when the response is lost (API Gateway gives up at 30 s; the function runs up to 900 s), polls `.../replays` every 10 s for up to 900 s for a record whose stamp differs -- only when no response came back or the gateway answered a bare 502/503/504; a refusal the server answered is shown at once. |
| `GET /recording/walks/{walk}/replays` | Every stored `replay-*.json` |
| `GET /recording/models` | Relays the vision service's `/navigate/models`. An empty list (never an error) when unreachable. |
| `GET /recording/summary` | Per (model, prompt, source) rows over scored walks and replays: `mean_score`, `median_score`, `best`, `worst`, `reach_rate`, `collisions`, `flags`, `median_frames`. Rows with collisions sort last. Anything with a null or missing score is skipped, which includes `unusable` replays and every metrics container (no `eval.json`, no replays), and so is any `eval.json` or replay whose `schema` is not current (3.64). |
| `GET /stats` | `walks`, `frames`, `bytes` |
| `DELETE /recording/walks/{walk}` and `.../frames/{file}` | Deletes; a frame delete also rewrites `walk.jsonl` without that frame's row. `metrics-YYYY-MM-DD` passes `WALK_NAME`, so this deletes a whole day of mission rows when pointed at a metrics container (Known gaps). |
| `GET /admin`, `GET /admin.js` | The console (local runs; deployed copies come from S3) |

**The walk layout** (identical on both backends; `<root>` is a directory
or the `recordings/` key prefix):

| File | Writer | Shape |
|---|---|---|
| `frame-NNNN.jpg` (`.png`, `.webp`) | write route | the bytes the cloud was sent |
| `walk.jsonl` | write route, one line per frame | `seq`, `file`, `media_type`, `navigate` (the live `/navigate` reply, or under Drive via brain the twin's reshaped mission status with `_missionStatus`), `teleop_seq` |
| `meta.json` | finish route; meta route | `finished_at`, `frames`, `model_id`, `target_object`, `capture{width,height}`, `note` |
| `tags.json` | tag route | `{label}` |
| `eval.json` | scorer | `schema`, `walk`, `model_id`, `target_object`, `score`, `score_uncapped`, `verdict`, `flags`, `basis`, `components`, `metrics`, `judge`, `collisions` |
| `replay-<model>[__<prompt>].json` | replay | the scorer's fields plus `prompt_variant`, `replayed_at`, `frames` (attempted), `coverage`, `agreement`, `errors`, `diff`; `last_unusable` on a scored replay a later attempt failed to replace. The walk list's summary of it carries `stale` when its `schema` is not current |
| `labels.json` | a person (or `assert_absent()`) | `walk`, `description`, `target_visible_labels{file: bool}`, `adjudicated[]`, `proposed_by`, `note` |
| `labels.candidate.json` | `label_assist` | as `labels.json`, plus `candidate_scores` and `review_bands{visible,review,absent}`; `adjudicated` is always empty |

The twin builds walk names in `newWalkName()` as
`<target>[-<model tag>][-<prompt tag>]-YYYYMMDD-HHMMSS`. The target is
lower-cased, non-alphanumerics become `-`, and it is cut at 24 characters.
The model tag is present only when a model was picked in the twin, and is
the id with its region and provider prefixes, `claude-`, and any date or
version suffix removed (`opus-4-5`, `fable-5-1`, `gpt-6-astra`). The prompt
tag is present only for a non-`default` wording. Walks without a model tag
exist in the corpus (`blue-bottle-20260907-142454`). The walks service
parses `recorded_at` from the timestamp, not from mtime, and, when
`meta.json` has no `target_object`, recovers the target from the name in
`_walk_target()`: it strips the timestamp, then a prompt tag matching only
`default` or `next-step*`, then a model tag starting with only `nova`,
`claude`, `haiku`, `sonnet`, `opus`, `qwen`, `llama` or `pixtral`. Any
other tag stays in the recovered target (Known gaps).

## Parameters and configuration

| Key or constant | Default | Unit | Read in | Why |
|---|---|---|---|---|
| `brain.recording_backend` / `RECORDING_BACKEND` | `local` | `local` or `s3` | `walk_store_from_config()`, `brain_config.py` | A checkout with no AWS still records. The Lambda sets `s3`. |
| `brain.recording_dir` | `recordings` | path | same | Relative to the repo root; gitignored |
| `brain.recording_bucket` / `RECORDING_BUCKET` | empty | name | same | From the recordings stack's `BucketName` export |
| `brain.recording_prefix` / `RECORDING_PREFIX` | `recordings` | key prefix | same | |
| `brain.allow_recording` / `ALLOW_RECORDING` | `true` | bool | brain, per request | Turn off on any internet-reachable brain that should not take writes |
| `brain.recording_proxy_url` / `RECORDING_PROXY_URL`, `brain.recording_proxy_secret` / `RECORDING_PROXY_SECRET`, `brain.recording_proxy_timeout_s` | empty, empty, 10.0 | URL, secret, s | `brain_config.py`, `brain_server.py` | Forward writes to a peer that owns storage. The timeout has NO environment override; change it in `config/robot.yaml`. |
| `brain.vision_url` / `VISION_URL` | empty | URL | `admin_server.py` | Replay and `/recording/models`. Empty makes replay answer 503. |
| `brain.replay_timeout_s` | 60.0 | s | `post_navigate` | The mission's 20 s cost one 22-frame replay 21 frames to read timeouts |
| `VISION_SHARED_SECRET` (else `APP_SHARED_SECRET`) | unset | string | `post_navigate`, `/recording/models` | The vision service's own secret. The fallback is the walks service's OWN secret, which the vision service rejects unless the two happen to match. `cloudformation/serverless.yaml` sets neither this nor `VISION_URL` on `WalksFunction` (Known gaps). |
| `APP_SHARED_SECRET` | unset | string | `admin_server.require_secret` | The walks service's secret, deliberately not the brain's. Template: `WalksSharedSecret`. |
| `WALK_JUDGE_ENABLED` | off | bool | `admin_server.py` | The judge and the collision check run only when on |
| `BEDROCK_JUDGE_MODEL_ID` | `us.anthropic.claude-opus-4-5-20251101-v1:0` | id | same | |
| `JUDGE_SAMPLE_FRAMES` / `COLLISION_MAX_CHECKS` | 8 / 12 | frames | same; both env-overridable | Bound the paid calls per walk |
| `NAVIGATE_ATTEMPTS` | 4 | tries | `post_navigate` | Backoff `0.6 * 2**n + U(0, 0.4)` s. Retries 429, 5xx and transport errors; other 4xx fail at once. At 8 workers Bedrock throttled 10 of 22 frames. |
| `REPLAY_MIN_COVERAGE` | 0.8 | fraction | `admin_server.py` (module constant, env-overridable), used in `_replay()` | Below this a replay is stored as `verdict: unusable`, `score: null`, flag `incomplete` |
| `walk_replay.DEFAULT_WORKERS` | 6 | threads | `replay_walk()` | Throughput against Bedrock throttling. 150 frames once lost were all read timeouts, now retried. |
| `WALK_NAME` | `^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$` | | routes and walks service (copied) | The naming contract |
| `SAFE_NAME` | the same, up to 128 chars | | `walk_store.py` | Intersection of a safe directory name and a safe key segment |
| `MAX_FRAME_BYTES` / `MAX_FRAMES_PER_WALK` | 4 MiB / 500 | bytes / frames | `recording_routes.py` | Checked on the base64 length before decoding |
| `walk_eval.SCHEMA_VERSION` | 9 | | scorer | Bumping it rescores every stored `eval.json` on next read |
| `DEGENERATE_SHARE`, `STALL_RUN`, `OSCILLATION_RATE`, `IDENTITY_FLIPS`, `MIN_TURNS_FOR_OSCILLATION` | 0.90, 8, 0.40, 4, 6 | | scorer flags | Calibrated on the six walks of 2026-08-28/29 |
| Score weights | judge 0.45, behaviour 0.30, completion 0.25 (no judge: 0.55 / 0.45); capped at 40 on any collision; good at 70 or more, mixed at 45 or more | | `score_walk()` | The judge once carried 0.6 and let an 80-frame stall score 65 |
| `label_assist.HIGH` / `LOW` | 0.90 / 0.30 | probability | `propose()` | The band in between is what a person reviews |
| `download_url` `expires_s` | 900 | s | `S3WalkStore` | |
| Bucket lifecycle | to `STANDARD_IA` at 30 d; noncurrent versions expire at 90 d; incomplete uploads abort at 7 d; `exports/` expires at 1 d | days | `recordings-s3.yaml` | Walks never expire |
| `WalksFunction` `Timeout` / `MemorySize` | 900 / 1024 | s / MB | `serverless.yaml` | A replay re-asks every frame. 900 s is Lambda's ceiling. |

## Procedures

**Record a walk locally.** Start both servers, connect the brain in the
twin's Settings, then use Guide, then Robot view, then "Record this
walk". When the walk stops, the toast reads `N frames saved as <walk>`.
The frames are in `recordings/<walk>/` on the brain's machine. With
recording on and no brain connected, the twin toasts "Not recording".

**Sync rig walks to the bucket after every session.** The local brain
writes to disk. `<bucket>` is the recordings stack's `RecordingsBucketName`
output.

```bash
aws s3 sync recordings/ s3://<bucket>/recordings/
for w in recordings/*/; do n=$(basename "$w"); \
  echo "$n $(ls "$w" | wc -l) $(aws s3 ls "s3://<bucket>/recordings/$n/" | wc -l)"; done
```

Check per-walk object counts, not just that the prefix exists. Expect S3
to be a superset: server-side `eval.json` and replays exist only there.
The sync never deletes (no `--delete`), so those survive it.

Do not add `--size-only`. With it, `aws s3 sync` compares sizes only, and
a local edit that keeps a file's size (flipping one label in
`labels.json`, or a same-length change to `meta.json` or `tags.json`) is
silently skipped: the command succeeds, the object counts match, and the
bucket keeps the old ground truth. The default rule (size, or a local
copy newer than the object) uploads those edits, and does not re-upload
unchanged frames, because an uploaded object is always newer than the
local file it came from. If a sync is suspected of skipping, compare one
file directly: `aws s3 cp s3://<bucket>/recordings/<walk>/labels.json - | diff - recordings/<walk>/labels.json`.

**Run the walks service and console locally:**

```bash
VISION_URL=http://127.0.0.1:8080 VISION_SHARED_SECRET=<vision secret> \
  uvicorn control.admin_server:app --port 8002
# open http://127.0.0.1:8002/admin
```

`VISION_URL` names a local vision service on its default port. That port
is also the tunnel proxy's (`service/tunnel/run.sh`): with the tunnel
stack up, point `VISION_URL` at the deployed site (or at the proxy's
forward to it, `http://127.0.0.1:8080/vision`), or at a local vision
service started on another port (see the cloud-vision engineering spec,
"Port 8080 is shared with the tunnel proxy"). Pointed at the bare proxy
by mistake, every replay frame is a 404 from the robot server and the replay
is stored `unusable`. Set `VISION_SHARED_SECRET` whenever the vision
service has a secret; without it the walks service signs with its own
`APP_SHARED_SECRET` and every frame is a 401.

**Score and replay from the command line** (base URL = the site, or the
local service; add `-H "x-app-secret: ..."` when a secret is set):

```bash
curl -X POST "$BASE/recording/walks/<walk>/evaluate?judge=false"
curl -X POST "$BASE/recording/walks/<walk>/replay" -H 'content-type: application/json' \
  -d '{"model_id":"us.anthropic.claude-opus-4-5-20251101-v1:0","prompt_variant":"bearing-only"}'
curl "$BASE/recording/walks/<walk>/replays"    # poll for a fresh replayed_at
```

Do not start a second replay when the POST times out. The first is still
running and writes its sidecar when done. Stacked replays are what used to
lose frames to throttling.

**Propose labels, then adjudicate:**

```bash
python -m control.label_assist recordings/<walk>
# <walk>: <n> frames -> <v> visible, <r> REVIEW, <a> absent  -> labels.candidate.json
```

It needs `requirements-perception.txt`. `--model <bedrock id>` proposes
with a cloud VLM instead. A walk that already has `labels.json` is
skipped unless `--force` is given, and even then only the candidate file
is written (for calibration). Review the REVIEW band and spot-check the
other two, record every looked-at frame in `adjudicated`, then save the
result as `labels.json`. Score it with `control/perception_eval.py`
(perception domain).

**Play a walk as a mission** (paid calls, one per frame under `vision`,
one per trigger under `tiered`). `VISION_URL` is the vision service's base
URL (the deployed site, or a local service); the client reads the vision
secret from `APP_SHARED_SECRET`, not `VISION_SHARED_SECRET`. `--policy
tiered` needs `requirements-perception.txt`; what a tiered replay does and
does not measure is canonical in the
[policy engineering spec](../policy/ENGINEERING.md#procedures).

```bash
VISION_URL=<vision base URL> APP_SHARED_SECRET=<vision secret> \
  python -m tests.demo_replay_mission recordings/<walk> "<target>" --policy vision
python -m tests.demo_replay_mission recordings/<walk> "<target>" --policy tiered   # same env
```

Expected output: a header `=== <n> frames from recordings/<walk>,
target='<target>', policy=<policy> ===`, then one line per step (step,
action, `[frame-NNNN.jpg]`, and under `tiered` a `<status margin | CLOUD
<trigger>>` or `| local>` tag, then the reasoning), then a summary block:
`Outcome:`, `Steps/calls:` (vision) or `Steps:` / `Paid calls: <c> over
<f> frames -- 1 per <r>` / `Triggers:` (tiered), `Wall clock:`, `Frames
used: <i> of <n>`, and `Arrived:`. With `VISION_URL` unset it exits at
once with `Set VISION_URL to the deployed vision service's base URL.`
A replay is open loop: `Outcome` is not a navigation result.

**Deploy or update the bucket stack**, and confirm what is actually
deployed, not what git says:

```bash
aws cloudformation deploy --template-file cloudformation/recordings-s3.yaml \
  --stack-name vision-picar-recordings-s3 --region us-east-2 \
  --capabilities CAPABILITY_NAMED_IAM
aws cloudformation get-template --stack-name vision-picar-recordings-s3 \
  --region us-east-2 --query TemplateBody --output text \
  | grep -E '^\s*(DeletionPolicy|UpdateReplacePolicy):'
```

`--capabilities CAPABILITY_NAMED_IAM` is required because the template
creates a named managed policy (`RecordingsAccessPolicy`, named
`<stack>-access`). Without it the deploy fails with
`InsufficientCapabilitiesException` before changing anything. Expected
from the second command, exactly two lines: `DeletionPolicy: Retain` and
`UpdateReplacePolicy: Retain` (indented, on the bucket). `--output text`
matters: with the default JSON output `TemplateBody` is one escaped string
on a single line, and grep prints the whole template or nothing useful.
Nothing printed means the deployed template is not the one in git.

The walks function is built and deployed together with the vision
function; the procedure is "Deploy the functions" in the
[operations engineering spec](../operations/ENGINEERING.md#procedures).
Pass `WalksSharedSecret`. As templated, that deploy does not enable
replay (Known gaps).

**Recover a deleted object** (within the 90-day noncurrent window). A
delete on the versioned bucket leaves a delete marker on top of the old
version; removing the marker restores it. First see what is there:

```bash
aws s3api list-object-versions --bucket <bucket> --prefix recordings/<walk>/ \
  --query '{deleted: DeleteMarkers[?IsLatest].[Key,VersionId], versions: Versions[].[Key,VersionId,IsLatest,LastModified]}' \
  --output table
```

Expected: under `deleted`, one row per deleted key with its marker's
version id; under `versions`, the earlier versions of those keys with
`IsLatest` `False`. Both empty means nothing was ever under that prefix
(check the walk name) or the window has passed.

Restore a whole deleted walk by removing its latest delete markers:

```bash
aws s3api list-object-versions --bucket <bucket> --prefix recordings/<walk>/ \
  --query 'DeleteMarkers[?IsLatest].[Key,VersionId]' --output text \
  | while read -r key vid; do aws s3api delete-object --bucket <bucket> --key "$key" --version-id "$vid"; done
aws s3 ls s3://<bucket>/recordings/<walk>/ | wc -l
```

Expected: one `{"DeleteMarker": true, "VersionId": "..."}` per key, then a
count equal to the walk's object count before the delete. To roll back a
REWRITTEN object instead (a frame delete rewrites `walk.jsonl`), copy the
wanted older version over the current one:

```bash
aws s3api copy-object --bucket <bucket> --key recordings/<walk>/walk.jsonl \
  --copy-source "<bucket>/recordings/<walk>/walk.jsonl?versionId=<version>"
```

Expected: a JSON reply with `CopyObjectResult.ETag` and a new `VersionId`.
`GET /recording/walks/<walk>` then shows the restored rows.

**Failure signatures.**

| Symptom | Cause |
|---|---|
| The console shows 0 walks after a walk | The twin's brain URL pointed at a brain with recording off |
| Every save fails with 403 | `allow_recording` is false and no proxy is set |
| Replay answers 503 "No vision service configured" | `vision_url` is empty in this process. Expected on the deployed walks function as templated. |
| Replay answers 200 but the sidecar is `unusable` and the failed frames' `diff[].error` read `RuntimeError: HTTP 401: ...` | `VISION_SHARED_SECRET` is unset, so frames were signed with the walks secret |
| Replay answers 200 but the sidecar is `unusable` and the failed frames' `diff[].error` read `RuntimeError: HTTP 404: ...` | `VISION_URL` points at something other than the vision service, typically the tunnel proxy on :8080 |
| `aws s3 sync` reports nothing to upload after a label edit | `--size-only` was used and the edit kept the file's size |
| The walks function fails at import (`Runtime.ImportModuleError` / init error) with `control.walk_store.WalkStoreError: S3WalkStore needs a bucket name.` | `RECORDING_BACKEND=s3` with no `RECORDING_BUCKET`: the stack was deployed without the bucket import. (`_assert_configured()`'s `RECORDING_BACKEND=s3 but RECORDING_BUCKET is unset` cannot appear: `create_app()` fails first.) |
| A replay row shows `unusable` / `incomplete` | Coverage below 0.8. `errors` in the sidecar is a count; the per-frame reasons are in `diff[].error`. |

## Verification

| Test | What it proves |
|---|---|
| `tests/test_walk_store.py` (76) | Both backends through one conformance suite, S3 via an in-memory client double: layout, names refused with separators or `..`, append, delete, and `download_url` (none locally, presigned on S3, keyed outside the walk prefix) |
| `tests/test_admin_server.py` (80) | The review routes, label vocabulary, lazy scoring, the replay coverage guard (`unusable`), replay's own timeout, retry on a read timeout and no retry on a 4xx, download as a zip, frame delete removing its `walk.jsonl` row, an unknown walk deleting nothing |
| `tests/test_walk_eval.py` (41) | Metrics, flags, score weights, the collision cap, judge parsing |
| `tests/test_brain_server.py` | Recording switched off, the proxy forwarding and surfacing the peer's errors (502 when unreachable), names unable to escape the directory, `finish` recording the model and capture size |
| `tests/test_serverless_routes.py` | The write routes are present in the walks service, and every walks route has an API Gateway route and a CloudFront behaviour |
| `tests/test_lambda_packaging.py` | Every third-party import under `control/` is in `requirements-walks.txt` |
| `tests/test_ui_admin.py` (11) | The console at 390 px: replay dropdown fills even when the model list is slow, newest-first by recording time, score and label as separate controls, no sideways scroll |
| `tests/test_ui.py` | The twin side: recording reports when it cannot start, and a double tap does not begin two walks |
| `tests/test_ui_pipeline.py` | Overlapping frames get distinct sequence numbers |

Ran 2026-10-02: `test_walk_store`, `test_walk_eval`, `test_admin_server`,
`test_static_assets`, `test_serverless_routes`, `test_lambda_packaging`
and `test_vision` gave 228 passed.

**Checklist for a change:** layout unchanged, or every reader updated
(scorer, replay, `ReplayRobot`, `perception_eval`, download); new routes
under `/recording/` (routed by the existing wildcard); a new third-party
import is added to `requirements-walks.txt`; a `SCHEMA_VERSION` bump if
`eval.json` changes meaning.

## Known gaps

- **Deployed replay is disabled as templated, and would fail on auth if
  half-enabled.** `serverless.yaml`'s `WalksFunction` sets neither
  `VISION_URL` nor `VISION_SHARED_SECRET` (nor `WALK_JUDGE_ENABLED`), and
  the shipped `config/robot.yaml` has `vision_url: ""`. So:
  - as templated, `POST .../replay` answers 503 "No vision service
    configured", `/recording/models` returns an empty list, and scoring is
    metrics-only, even though `WalksRole` grants Bedrock for the judge;
  - if `VISION_URL` were added alone, `post_navigate()` would fall back to
    the function's own `APP_SHARED_SECRET` (the walks secret), the vision
    function would answer 401 on every frame, 4xx is not retried, and
    every replay would be stored `unusable` with a 200.

  The fix is to template both `VISION_URL` (the site URL) and
  `VISION_SHARED_SECRET` (the `VisionSharedSecret` parameter) on
  `WalksFunction`, or to declare replay local-only. The architecture
  spec carries this as an open question, and the operations specs record
  the same gap. **UNCONFIRMED against the live stack**: no AWS call was
  made, and values set outside the template would change the answer.
  `aws lambda get-function-configuration` on the walks function settles
  it.
- **The bucket-missing refusal keys on `RECORDING_BACKEND=s3`, and its
  own guard is dead code.** With `s3` and no bucket, `create_app()` raises
  `WalkStoreError` before `walks_handler._assert_configured()` runs, so
  the guard's message never appears; the refusal still happens, by
  accident of ordering. No test covers either path. A function deployed
  with `RECORDING_BACKEND` unset starts on the `local` backend and writes
  frames to its ephemeral disk, which is the failure the guard exists to
  stop. The template always sets `s3`, so this needs a hand-edited
  deployment to bite.
- **An empty walk exists locally and not on S3.** `create_walk()` makes a
  directory on `LocalWalkStore` and is a no-op on `S3WalkStore`, because a
  prefix exists only while an object carries it. Every real walk gets a
  frame immediately, so it is invisible in practice, but `walk_exists()`
  after `create_walk()` alone differs between backends. The append to
  `walk.jsonl` also differs: a real `O_APPEND` locally, read-modify-write
  on S3, which is safe only with one writer per walk.
- **Local recording has no cloud copy until someone syncs by hand.**
- **`label_assist` has no tests.**
- **Teleop entries record `obstacle_ahead: false`.** The twin's
  `missionStatusToRobotResult()` hardcodes it, so a Drive-via-brain walk's
  `walk.jsonl` carries a value nobody measured, and the scorer's
  `obstacle_rate` reads it.
- **Stale docstrings.** `admin_server.py`'s header still describes the EFS
  volume and says the write path stays in `brain_server.py`.
  `walks_handler.py` cites "two of the 39 walks", a corpus deleted on
  2026-09-07.
- **Replay is a synchronous request,** not a job.
- **Replay does not ask exactly the live question.** `post_navigate()`
  hardcodes `media_type: image/jpeg`, so a `.png` or `.webp` frame (which
  `FRAME_SUFFIX` allows) is sent to Bedrock labelled as JPEG; and it never
  sends `searched_rooms`, which a live mission's `brain/navigate.py`
  sends from mission memory. Replay therefore measures a model with no
  search memory. Every rig walk so far is JPEG, so the first difference
  has not bitten.
- **The recovered target can carry tags.** `_walk_target()` strips only
  prompt tags `default` / `next-step*` and model tags starting
  `nova|claude|haiku|sonnet|opus|qwen|llama|pixtral` (the model pattern
  also swallows any prompt tag after a recognised model). So with no
  `meta.json` target, a `fable-5-1` or `gpt-6-astra` model tag, and any
  prompt tag other than `next-step*` that follows one of them or no model
  tag at all (`bearing-only`, `center-third-path`,
  `default-with-distance`), stays in the recovered target, which goes into
  the judge's and the collision check's prompts. Checked against the
  regexes: `red-backpack-gpt-6-astra-<ts>` gives "red backpack gpt 6
  astra", `red-backpack-bearing-only-<ts>` gives "red backpack bearing
  only", while `red-backpack-opus-4-5-bearing-only-<ts>` gives "red
  backpack". Setting `target_object` through `PUT .../meta` avoids it for
  one walk.
- **The 307 redirect is tested only at the store level.** No route test
  drives `GET .../download` against an S3-backed store.
