---
kind: engineering
domain: cloud-vision
status: current
verified: 2026-10-02
parent: docs/cloud-vision/ARCHITECTURE.md
---

# Cloud vision -- engineering

How the vision service is built and deployed today. The what and the why
are in the [architecture spec](../../cloud-vision/ARCHITECTURE.md). This
document is true only until the implementation changes, and it is updated
in the same commit as the code.

## Implementation

| File | What it does |
|---|---|
| `service/vision_analyze/app.py` | FastAPI app. `create_app()` builds the routes. `_decode_image()` does the shared auth, JSON parse, base64 decode and size check. `_check_secret()` compares `x-app-secret` in constant time (`hmac.compare_digest`, 3.76 H2-9). `_target_object()` bounds the target and refuses a blank one (3.76 H2-8, round 2). Every route's model call goes through `_vision_call()` (round 3 P5): an `ImageRejected` from `vision_core` maps to its own 400/413; every other failure, Bedrock's `ValidationException` included, is 502 (round 3 P1). The request gate is `MAX_REQUEST_IMAGE_BYTES` (the Lambda payload); Bedrock's `MAX_IMAGE_BYTES` is checked by `vision_core` after conversion (round 3 P3). |
| `service/vision_analyze/vision_core.py` | Prompts, the model allow-list, prompt variants, region pins, one `bedrock-runtime` client per region (`_get_client(model_id)`), the four `describe_image_bytes*()` calls and their `_parse_*_json()` parsers. |
| `service/vision_analyze/rooms_core.py` | `identify_room()` and `ROOM_FEATURES`, a hand-kept copy of `brain/rooms.py`. Since 3.76 H2-5 both copies drop non-string and empty items and refuse a non-list (here the list is the model's unvalidated output; mirrored into `brain/rooms.py` in round 2). |
| `service/vision_analyze/requirements.txt` | `fastapi`, `uvicorn[standard]`, `boto3`, `pillow-heif`, `pillow` |
| `service/vision_analyze/Dockerfile` | `python:3.12-slim`, flat `/app`, `uvicorn app:app` on port 8080. Used for local container runs. There is no ECS deployment since 2026-09-05. |
| `service/vision_analyze/tests/` | The service's own suite (`test_app.py`, `conftest.py` puts the service dir on `sys.path` so the flat imports resolve) |
| `service/lambda/vision_handler.py` | `handler = Mangum(app, lifespan="off")`, with `from app import app` as a FLAT import |
| `service/lambda/requirements-vision.txt` | Only what Lambda adds: `mangum`. Its comment says `boto3` comes from the runtime, but `build.sh` also installs `service/vision_analyze/requirements.txt`, which lists `boto3`, so the vision zip DOES bundle `boto3` (and `botocore`). See Known gaps. |
| `service/lambda/build.sh` | Builds both Lambda zips for `manylinux2014_aarch64` / CPython 3.12 with `--only-binary=:all:`, copies `app.py`, `vision_core.py`, `rooms_core.py` and `vision_handler.py` to the zip ROOT, uploads to the deploy bucket, and prints the deploy command |
| `cloudformation/serverless.yaml` | `VisionFunction`, `VisionRole` (`bedrock:InvokeModel` on `*`), `ApiInvokeRole`, the HTTP API with `CredentialsArn` integrations, the routes, and the CloudFront behaviours |
| `brain/vision.py` | Manual only: no mission, sim run or hardware path calls it. `describe_image()`, `describe_frame()` and `describe_base64()` call the Anthropic Messages API with `MODEL = "claude-sonnet-5"`, `max_tokens=500` and `ANTHROPIC_API_KEY`. Only `tests/manual_describe_image.py` and the mocked `tests/test_vision.py` / `tests/test_coverage_gaps.py` use it. |

The client of `/navigate` in the brain (`brain/navigate.py`) belongs to
the policy domain. The replay client (`control/admin_server.py`'s
`post_navigate`) belongs to recordings.

## Interfaces

All POST bodies are JSON. When `APP_SHARED_SECRET` is set, every POST
route requires header `x-app-secret`. `GET /health` and
`GET /navigate/models` are not gated.

| Method, path | Request | 200 response | Errors |
|---|---|---|---|
| `GET /health` | none | `{"status": "ok", "env_label": <ENV_LABEL, stripped; "" when unset>}` (3.64: behind CloudFront this is what the twin's and the console's banners read) | none |
| `POST /analyze` | `image_base64`, `media_type` (default `image/jpeg`) | `obstacles_ahead[]`, `free_space`, `doorway_visible`, `important_objects[]`, `safest_direction`, `room_guess` (from `identify_room`) | 400 bad JSON, a body that is not a JSON object, a non-string `media_type`, missing `image_base64`, base64 that cannot be decoded at all, or a non-native image (HEIC) Pillow cannot read; 401 secret; 413 a request image over `MAX_REQUEST_IMAGE_BYTES` (4.5 MB), or a native-format image over `MAX_IMAGE_BYTES` (3.75 MB; a converted one is shrunk to fit instead); 502 model failure, any Bedrock refusal included. Native formats are never opened locally: non-image bytes labelled `image/jpeg`, a mismatched `media_type` or a side over 8000 px go to Bedrock and come back 502 (Known gaps). |
| `POST /describe` | same as `/analyze` | `summary`, `room_type`, `objects[]` | same as `/analyze` |
| `GET /navigate/models` | none | `default`, `models[{id,label}]`, `default_prompt`, `prompts[]` (sorted) | none |
| `POST /navigate` | `image_base64`, `media_type`, `target_object` (required), `searched_rooms[]` (optional; non-strings dropped), `model_id` (optional, must be in the list), `prompt_variant` (optional, must be in the list) | see the schema below | 400 missing or blank target, a target that is not a string or is over 200 characters (`MAX_TARGET_CHARS`), `model_id`/`prompt_variant` not a string or off the list (all checked before any Bedrock call); 401; 413; 502 (any Bedrock refusal). `searched_rooms` keeps non-blank strings, each cut to 64 characters, de-duplicated, the first 20 (the brain sends them sorted, so which 20 is arbitrary; 3.76 H2-8). |
| `POST /room` | `image_base64`, `media_type` (no target) | `room` (one lowercase name of at most three words, a leading article dropped, or `unclear`), `model_id`, `usage`; `_raw` (the reply, first 200 characters) only when it was not a name; `_stop` (Bedrock's `stopReason`, logged once as a warning) only when the reply did not end cleanly (not `end_turn` or `stop_sequence`: cut on `max_tokens`, a guardrail); such a reply is always `unclear`, even when its first word would pass as a name, with its text under `_raw` if there was any (3.70 S2b) | 400, 401, 413, 502 (as `/analyze`) |
| `POST /guidance` | `image_base64`, `media_type`, `target_object` (required) | `target_visible`, `position` (`far_left`, `left`, `center`, `right`, `far_right`, `not_visible`), `proximity` (`near`, `medium`, `far`, `unknown`), `guidance`, `bounding_box` `{x_min,y_min,x_max,y_max}` in 0-1 or `null` | 400, 401, 413, 502 |

**`/navigate` reply.** The reply is `_NAVIGATE_EMPTY_SCHEMA` merged under
the model's JSON, then coerced by `_parse_navigate_json()`:

| Field | Values | Coercion |
|---|---|---|
| `target_visible` | bool | coerced to a real bool (handoff 4b): `true` or the string `"true"` (any case, trimmed) is true, anything else false -- before the reached-implies-visible check below. `brain/navigate.py` reads it with `is True`; `brain/tiered.py`'s `confirm_arrival()` reads the normalised `_navigate.target_visible` as the arrival's identity verdict (handoff 1a); `web-twin/app.js` reads it by truthiness (Robot view's zone highlight, `const zone = result.target_visible ? ...`). |
| `target_direction` | `left`, `center`, `right`, `not_visible` | trimmed and lower-cased, then anything off `TARGET_DIRECTIONS` becomes `not_visible` (handoff 4b) |
| `target_reached` | bool | `is True` only. Forced `false` when `target_visible` is false. |
| `obstacle_ahead` | bool | REMOVED when the variant's template does not ask for it (`variant_asks_obstacle()`). Otherwise coerced like `target_visible` (handoff 4b). `brain/navigate.py` reads it with `is True`; in Robot view the OBSTACLE banner is strict (`=== true`) while the HUD's OBS row and the haptic/audio cue read it by truthiness -- harmless since the service coerces it (handoff 4b). |
| `room_guess` | string | non-string, or empty after `strip()`, becomes `unclear`; any other string passes through stripped, so an invented label (`"spaceship"`) reaches the caller as a room |
| `action` | `FORWARD`, `LEFT`, `RIGHT`, `REVERSE`, `STOP` | anything else becomes `STOP` |
| `reasoning` | string | |
| `distance_estimate` | `within_one_step`, `a_few_steps`, `far`, `unknown` | off-list becomes `unknown`; always present |
| `path_ahead` | `open_floor`, `blocked`, `unclear` | off-list becomes `unclear`; always present; never used to recompute `obstacle_ahead` |
| `model_id`, `prompt_variant` | the model and wording that answered | added after parsing |
| `usage` | a dict, `{input_tokens, output_tokens}`; always present | from Bedrock's `usage`; each count may be `null` when Bedrock omits it |
| `_raw` | the unparsed text | only when the reply held no JSON object |

All four parsers read the reply through `_loads_object()` (3.76 H2-1,
H2-2, round 2 S7). It tries the whole text, then the content of the first
```` ``` ```` fence, then the first object `json.JSONDecoder().raw_decode`
reads from the first `{` -- so prose before or after the JSON, or a
```` ``` ```` inside a JSON string, no longer loses the answer. The result
must be a JSON OBJECT. `brain/vision.py` mirrors it (round 2 S5). Valid JSON that is not an object (a
list, `null`, a number) is a parse failure like any other: the empty
schema plus `_raw`, HTTP 200. It used to escape as a `TypeError` and a
502, which the brain reads as the cloud being unreachable. A parse
failure is the model's answer, not an outage: at arrival it reads as "not
confirmed" (`confirm_arrival()` needs an explicit `target_visible: true`),
which is the documented fail-safe direction (3.47's rule: only transport,
5xx, 408 and 429 are unreachable).

**Image handling.** `_bedrock_image_format()` maps `image/jpg` to `jpeg`.
Any format outside `gif`, `jpeg`, `png`, `webp` (in practice iPhone HEIC)
is transcoded to JPEG with Pillow and `pillow_heif` before the call
(`_convert_to_jpeg()`), all through `_prepare_image()`. This is how the
architecture spec's "converted to an accepted format" is met today. A
converted image is fitted, not refused (round 2 S4): sides capped at
`MAX_IMAGE_SIDE_PX` (8000) and encoded at quality 85; if still over
`MAX_IMAGE_BYTES`, scaled once by 0.9 x sqrt(limit / size) and re-encoded,
then by 3/4 per pass in the rare case it is still over (round 3 P5), so a
48 MP iPhone HEIC can be described. A conversion Pillow cannot do raises
`ImageRejected(400)`; a native image over `MAX_IMAGE_BYTES` raises
`ImageRejected(413)` -- both before any Bedrock call. A native image
(JPEG, PNG, GIF, WebP) over `MAX_IMAGE_BYTES` or with a side over 8000 px
(read from the header by `Image.open`, no pixel decode) goes through the
same shrink and is sent as JPEG (3.76 round 4, G2); one Pillow cannot open
is sent, or refused 413, exactly as before. Small native images are sent
untouched. Every open goes through `_open_image()`, which reads the size from the
header with Pillow's own bomb check off (it raised before the size was
known, so a 14000 px flat PNG skipped the shrink); this service's ceiling
is `MAX_DECODE_PIXELS` instead: an image that would decode to more pixels
is a 413 before any pixel is decoded, converted or native. A JPEG is
decoded through `draft()` at the smallest scale that still covers the
target size, so the ceiling applies to that reduced decode (3.76 round 5).

**What is not validated before the model call.** `_decode_image()` only
base64-decodes, with `base64.b64decode()`'s default non-strict mode (it
discards characters outside the alphabet rather than refusing them), and
checks the size. It never opens the bytes as an image. So a body whose
base64 decodes to something that is not an image, or that does not match
its `media_type`, goes to Bedrock and comes back as a 502 ("Vision ...
failed: ... ValidationException ..."). A HEIC that Pillow cannot open is
a 400 since 3.76 H2-3.

**CORS.** `allow_origins` comes from `ALLOWED_ORIGINS`, defaulting to
`http://localhost:5173` and `http://localhost:8000`. Methods are `POST`
and `OPTIONS`; headers are `Content-Type` and `x-app-secret`. The deployed
function sets no `ALLOWED_ORIGINS`. It relies on same-origin through
CloudFront.

**Edge routing** (`cloudformation/serverless.yaml`). API Gateway routes
`POST /analyze`, `POST /describe`, `POST /navigate`,
`GET /navigate/models`, `POST /room`, `POST /guidance` and `GET /health` to the vision
integration. CloudFront has a behaviour for `/analyze`, `/describe`,
`/navigate`, `/navigate/*`, `/room`, `/guidance` and `/health`, each with
`CachingDisabled` and `AllViewerExceptHostHeader` (forwarding `Host` makes
API Gateway answer 403).

## Parameters and configuration

| Key or constant | Default | Unit | Read in | Why |
|---|---|---|---|---|
| `BEDROCK_MODEL_ID` (`MODEL_ID`) | `us.anthropic.claude-sonnet-4-5-20250929-v1:0` | id | `vision_core.py` | Base model for `/describe`, and the fallback for `/analyze`. `claude-sonnet-5` returned AccessDenied on Bedrock for this account. |
| `BEDROCK_ANALYZE_MODEL_ID` | `MODEL_ID` | id | `vision_core.py` | `/analyze` stays on the conservative default |
| `BEDROCK_NAVIGATE_MODEL_ID` (`NAVIGATE_MODEL_ID`) | `us.anthropic.claude-opus-4-5-20251101-v1:0` (Claude Opus 4.5, today's default) | id | `vision_core.py`; set by the template's `NavigateModelId` (same default) | Chosen by replay, as the architecture spec requires: all 22 frames of `red-backpack-20260829-195904` through every invokable model. Opus 4.5 made forward progress on 10/22, stayed aware of obstacles, and was the only model to refuse a red blanket as the target. Keep the code and the template in step. |
| `BEDROCK_GUIDANCE_MODEL_ID` | `amazon.nova-lite-v1:0` | id | `vision_core.py` | Cheaper than Sonnet for steering a person. `vision_core.py`'s header says it was "measured (real Bedrock calls, real photo) at ~3x Sonnet's latency with matching accuracy", but no numbers, frames or date are recorded anywhere in the repo, and "3x Sonnet's latency" literally says slower: UNCONFIRMED on both speed and accuracy. Never replayed. |
| `BEDROCK_NAVIGATE_MODEL_CHOICES` | the 7 below | comma list | `_load_navigate_model_choices()` | Overrides the allow-list without a code change. Labels then equal the ids. `NAVIGATE_MODEL_ID` is always added to the list (3.76 H2-6), so the advertised default is pickable. |
| `BEDROCK_MODEL_REGIONS` | `us.anthropic.claude-fable-5-1` and `global.anthropic.claude-fable-5-1` to `us-east-1` | `model=region` list | `_load_model_regions()` | Fable 5.1 was refused from us-east-2 and us-west-2 ("data retention mode 'default' is not available", 2026-09-21). An empty string drops every pin. |
| `NAVIGATE_PROMPT_VARIANT` (`DEFAULT_PROMPT_VARIANT`) | `default` | name | `vision_core.py` | One shipped wording (decided 2026-09-03). An unknown name logs a warning and becomes `default`, so replies and `/navigate/models` report what actually runs (3.76 H2-6); `describe_image_bytes_navigate()` itself raises `ValueError` on an unknown variant rather than running another (round 2 S8). |
| `APP_SHARED_SECRET` | unset (no auth) | string | `app.py` | Set from the template's `VisionSharedSecret`. Empty means `/navigate` is publicly billable. |
| `ALLOWED_ORIGINS` | localhost:5173, localhost:8000 | comma list | `app.py` | Local dev only |
| `MAX_IMAGE_BYTES` | 3,750,000 | bytes | `vision_core.py` (`_prepare_image()`, after any conversion) | Bedrock's per-image limit, from botocore's `bedrock-runtime` model (`ImageSource`: "no more than 3.75 MB, 8000 px, and 8000 px"); 5 MB was the Anthropic API's. 3.76 H2-4. |
| `MAX_DECODE_PIXELS` | 50,000,000 | pixels | `vision_core._convert_to_jpeg()` | Measured peak RSS of decode + convert + thumbnail + encode (fresh process, laptop): 48.8 MP (an iPhone 48 MP photo) 604 MB, 50 MP 686 MB, 64 MP 540-792 MB by shape, 81 MP 878 MB, 100 MP 1234 MB. The function has 1024 MB and the app ~60 MB; above the ceiling a 413, never an OOM kill with nothing logged (3.76 round 5). |
| `MAX_REQUEST_IMAGE_BYTES` | 4,500,000 | bytes | `app.py` (`_decode_image()`) | Lambda's 6 MB (6,291,456 B) synchronous payload: base64 makes 4.5 MB about 6.0 MB, plus the JSON. A gate at 3.75 MB here refused the HEIC photos conversion would have shrunk (round 3 P3). |
| `MAX_TARGET_CHARS`, `MAX_SEARCHED_ROOMS`, `MAX_ROOM_CHARS` | 200, 20, 64 | chars, entries, chars | `app.py` | Caller text that goes into the prompt is bounded (3.76 H2-8) |
| Bedrock client `connect_timeout` / `read_timeout` / `retries` | 5 / 20 / `total_max_attempts: 1` | s | `vision_core._BEDROCK_CONFIG` | Under the function's 30 s, so a hung call raises and is logged (502) instead of the function being killed; botocore's default read timeout is 60 s (3.76 H2-11). One attempt: botocore's default retried a read timeout 5 times (measured against a local socket), each retry a possible second paid call (round 2 S1). `tests/test_review_376h2.py` pins attempts x (connect + read) below the template's `Timeout`. |
| `_RETRYABLE_CODES`, `_RETRY_BACKOFF_S`, `_CALL_BUDGET_S` | `ThrottlingException`, `ServiceUnavailableException`; 1; 28 | -; s; s | `vision_core._converse()` | One attempt also dropped botocore's retry of a throttle, so `_converse()` retries those two codes (refusals before any work) ONCE, after 1 s, only if elapsed + 1 s + a whole attempt (25 s) still fits 28 s; otherwise the error stands (502). Round 3 P2. Elapsed runs from the request's entry (`mark_request_start()` in `_decode_image()`, a context variable), so decoding and any conversion count (round 4, G1). |
| `maxTokens` | 500 (`/analyze`, `/describe`), 300 (`/navigate`, `/guidance`), 256 (`/room`, `ROOM_MAX_TOKENS`) | tokens | `vision_core.py` | All three models added on 2026-09-21 answered inside the 300 cap. `/room` asks for a name alone (3.70 S2): Opus 5.5 wrote 230-300 tokens of `/navigate` reply a call. Opus 5.5 always thinks and its thinking counts against the cap: at 16 every real `/room` call stopped on `max_tokens` with no text (3.70 S2b), so `/room` sends `output_config.effort` `low` (`ROOM_EFFORT`, through `additionalModelRequestFields`; 57 tokens out on a real call), only to a model whose id is exactly a name in `ROOM_EFFORT_MODELS` (`claude-opus-5-5` today; a region prefix, a date and a `-vN:N` version allowed, never a longer name or an ARN; only the `us.` profile confirmed by a real call; a model it misses shows as cut replies with `_stop`; older Claude models and other vendors refuse the field, which would make every `/room` a 502), with a test pinning `ROOM_MODEL_ID`'s default and the template's `RoomModelId` default to it. The 256 only bounds the cost |
| `BEDROCK_ROOM_MODEL_ID` (`ROOM_MODEL_ID`) | `us.anthropic.claude-opus-5-5` | Bedrock id | `vision_core.py`, `serverless.yaml` `RoomModelId` | `/room`'s model: the user's choice for room labels (2026-10-13), confirmed with two real Converse calls on walk frames from us-east-2 that day. Code default and template parameter kept in step |
| `VisionFunction` `Timeout` / `MemorySize` | 30 / 1024 | s / MB | `serverless.yaml` | A call takes 1-3 s, with up to two in flight from Robot view. 30 s bounds a hung call. |
| `VisionFunction` runtime / architecture | `python3.12` / `arm64` | | `serverless.yaml`, `build.sh` | arm64 is cheaper and must match the wheels `build.sh` pulls |

**The `/navigate` allow-list** (`_DEFAULT_NAVIGATE_MODEL_CHOICES`, in picker order):

| Id | Label | Evidence |
|---|---|---|
| `us.anthropic.claude-opus-4-5-20251101-v1:0` | best judgement (default) | replay, 22 frames |
| `us.anthropic.claude-fable-5-1` | unmeasured | real call on frame 0010 of `blue-bottle-20260907-142454`, 2.7 s; region-pinned |
| `us.anthropic.claude-opus-5` | unmeasured | same frame, 4.9 s |
| `us.openai.gpt-6-astra` | unmeasured | same frame, 5.9 s |
| `us.anthropic.claude-sonnet-4-5-20250929-v1:0` | cautious, the control | replay: FORWARD on 1/22 |
| `qwen.qwen3-vl-235b-a22b` | non-Anthropic axis | replay: good object ID, did not report obstacles |
| `amazon.nova-lite-v1:0` | cheap baseline | replay: FORWARD on 22/22, drove into a couch |

**Prompt variants** (`NAVIGATE_PROMPT_VARIANTS`):

| Name | Differs from `default` by | Status |
|---|---|---|
| `default` | none | shipped |
| `default-with-distance` | adds question 6 and the `distance_estimate` field | measured on 80 frames: `within_one_step` 60%. Must not drive a veto. |
| `next-step-obstacle` | rewrites questions 2 and 5 | replay only; measured on the invalid corpus |
| `next-step-and-walls` | `next-step-obstacle` plus a clause about walls | replay only; on the invalid corpus it gained one frame in 22 on Sonnet and nothing on Qwen |
| `center-third-path` | question 2 asks about the bottom of the centre third; adds `path_ahead` | replay only; no fair test yet |
| `bearing-only` | removes question 2 and `obstacle_ahead` | M1; not yet measured |

Every variant is accepted by `/navigate` from any caller. "Replay only" is
a convention. The service does not enforce it.

## Procedures

**Run the service's suite** (no AWS, every model call mocked):

```bash
pytest service/vision_analyze/tests/ -q      # expect: 58 passed
```

It lives apart from `tests/` because both directories are named `tests`.
A bare `pytest` from the repo root relies on the `__init__.py` files to
tell them apart.

**Run it locally against real Bedrock** (needs AWS credentials with
`bedrock:InvokeModel`; a real call costs money):

```bash
cd service/vision_analyze && APP_SHARED_SECRET=test ALLOWED_ORIGINS='*' \
  uvicorn app:app --port 8080
curl -s localhost:8080/navigate/models | python -m json.tool
# expect: {"default": "us.anthropic.claude-opus-4-5-20251101-v1:0", "models": [...7...], "default_prompt": "default", "prompts": [...6...]}
```

The twin's Settings default the vision URL to port 8080 on localhost.

**Port 8080 is shared with the tunnel proxy.** `service/tunnel/run.sh`
starts its proxy on 127.0.0.1:8080 (operations owns that layout), so the
local vision service and the tunnel stack cannot both use it. While the
tunnel stack is up, the twin's vision URL depends on where the page was
loaded from:

| Page served from | Vision URL |
|---|---|
| CloudFront (the deployed site) | the site itself (same origin) |
| the tunnel's public domain (ngrok) | `https://<domain>/vision` |
| the laptop, through the proxy or the robot server | `http://127.0.0.1:8080/vision` |

`service/tunnel/proxy.py` forwards `/vision/*` to the DEPLOYED vision
service (`PROXY_VISION_URL`) and answers the preflight itself, because the
serverless stack answers no `OPTIONS` and sends no CORS headers; pointing
a tunnel-served page straight at the site fails as "Load failed". The
bare `:8080` (no `/vision`) is the robot server behind the proxy, and
`/navigate` there answers 404.

To use a LOCAL vision service while the tunnel is up, start it on
another port, for example `--port 8085`, and set the twin's vision URL
(or the walks service's `VISION_URL`) to match. Symptom of the collision:
uvicorn exits with `address already in use`, or, if the proxy won,
`/navigate` answers 404 from the robot server behind it.

**Build and deploy the function.** The canonical procedure, including the
region, both secrets and the expected output, is "Deploy the functions" in
the [operations engineering spec](../operations/ENGINEERING.md#procedures).
Do not copy the deploy command here: run the one `service/lambda/build.sh`
prints, which carries `--region` and the zip keys for this build. Both
functions deploy together from `cloudformation/serverless.yaml`; pass
`VisionSharedSecret`, or the vision function runs with no auth.

Vision-specific smoke test, against the stack's `ApiEndpoint` output (the
gateway without CloudFront) and then the `SiteUrl` output (which already
ends in `/`):

```bash
curl -s <ApiEndpoint>/health                 # {"status":"ok"}
curl -s <SiteUrl>navigate/models             # default + 7 models + 6 prompts
```

A 403 on the second line only is the CloudFront `Host` forwarding fault
in Failure signatures below.

**Add a model to the allow-list.**

1. Make one real Converse call with a real walk frame, from the DEPLOYED
   region (`us-east-2`, not the laptop's ambient region). Confirm the reply
   parses inside 300 tokens. The most faithful call is the service's own
   `/navigate`, run locally against Bedrock with the candidate as the only
   allowed model, so the real prompt, parser and token cap are used (a real
   call costs money; stop the tunnel stack first, or pick another port):

   ```bash
   cd service/vision_analyze && AWS_REGION=us-east-2 \
     BEDROCK_NAVIGATE_MODEL_CHOICES=<new id> uvicorn app:app --port 8085 &
   sleep 3   # until uvicorn logs "Application startup complete"
   python -c 'import base64,json,sys; print(json.dumps({"image_base64": base64.b64encode(open(sys.argv[1],"rb").read()).decode(), "media_type":"image/jpeg", "target_object":"blue bottle", "model_id":sys.argv[2]}))' \
     recordings/blue-bottle-20260907-142454/frame-0010.jpg '<new id>' \
     | curl -s localhost:8085/navigate -H 'content-type: application/json' -d @- | python -m json.tool
   kill %1
   ```

   Expected: a 200 whose JSON has `action` in the vocabulary,
   `target_visible`, `reasoning`, `model_id` equal to the new id, a
   `usage.output_tokens` below 300, and NO `_raw` key (`_raw` means the
   reply did not parse as JSON). A 502 whose detail carries
   `ValidationException` or `AccessDeniedException` means the model is not
   usable from that region (Failure signatures). The bare Bedrock call,
   without the service, has this shape (AWS CLI v2 takes the image bytes
   as base64 inside the JSON):

   ```bash
   B64=$(base64 < recordings/blue-bottle-20260907-142454/frame-0010.jpg | tr -d '\n')
   printf '[{"role":"user","content":[{"image":{"format":"jpeg","source":{"bytes":"%s"}}},{"text":"Is a blue bottle visible? Reply with ONLY JSON: {\\"action\\": \\"FORWARD|LEFT|RIGHT|REVERSE|STOP\\", \\"target_visible\\": true|false, \\"reasoning\\": \\"...\\"}"}]}]' "$B64" > /tmp/converse-msg.json
   aws bedrock-runtime converse --region us-east-2 --model-id '<new id>' \
     --messages file:///tmp/converse-msg.json --inference-config maxTokens=300 \
     --query '{text: output.message.content[0].text, stop: stopReason, usage: usage}'
   ```

   Expected: `stop` is `end_turn` (not `max_tokens`) and `text` is a JSON
   object. This proves the model answers from the region; only the
   `/navigate` call above proves the service's prompt parses.
2. Add the model to `_DEFAULT_NAVIGATE_MODEL_CHOICES` labelled
   "unmeasured".
3. If the model only answers elsewhere, add a `_DEFAULT_MODEL_REGIONS`
   pin.
4. Redeploy (operations procedure above).
5. Expected after the deploy: `curl -s <SiteUrl>navigate/models` lists the
   new id with label `unmeasured`.
6. Rank the model only after a replay (recordings domain).

**Change the default navigate model.** Change `NAVIGATE_MODEL_ID`'s
default and the template's `NavigateModelId` default in the same commit,
then redeploy. Confirm it from a live reply, not from the code: expected,
`GET <SiteUrl>navigate/models` reports the new id as `default`, and a
`POST /navigate` with no `model_id` echoes it in `model_id`. The old id in
either place means the env var (the template parameter) still wins.

**Mirror a prompt or room change.** Any change to `brain/vision.py`'s
schema or to `brain/rooms.py` is copied into `vision_core.py` /
`rooms_core.py` by hand. Then check the copies from the repo root:

```bash
diff <(sed -n '/^ROOM_FEATURES/,/^}/p' brain/rooms.py) \
     <(sed -n '/^ROOM_FEATURES/,/^}/p' service/vision_analyze/rooms_core.py) && echo ROOM_FEATURES match
diff <(sed -n '/^def identify_room/,$p' brain/rooms.py) \
     <(sed -n '/^def identify_room/,$p' service/vision_analyze/rooms_core.py)
diff <(sed -n '/^SCENE_PROMPT = /,/"""$/p' brain/vision.py) \
     <(sed -n '/^SCENE_PROMPT = /,/"""$/p' service/vision_analyze/vision_core.py)
```

Expected as of 2026-10-13: the first prints `ROOM_FEATURES match`; the
second differs by `identify_room()`'s docstring (seven `<` lines,
present in `brain/rooms.py` and absent from the copy) and by the two
3.76 comments above the (identical) input filter; the third differs
in exactly two hunks, the opening framing (lines 1-2) and the
object-naming question (line 6), which is the known drift in Known gaps,
and NOT in the JSON schema lines. Any other difference is an unmirrored
change.

**Failure signatures.**

| Symptom | Cause |
|---|---|
| `Runtime.ImportModuleError` at cold start | The zip was not flat, or a dependency is missing (`pillow-heif` was missed once). `tests/test_lambda_packaging.py` guards it. |
| 403 and nothing in the function's logs | Invocation is going through a resource policy (a Function URL, or an integration with no `CredentialsArn`) |
| 403 from API Gateway through CloudFront | The viewer's `Host` header is being forwarded (wrong origin request policy) |
| 502 with `ValidationException ... data retention mode` | The model is not enabled in this region. Pin it. (Kept 502 on purpose, 3.76 round 3 P1: as a 4xx it would end an arrived mission `failed` without 3.47's late re-ask.) |
| 502 with `AccessDeniedException` | The model is in the catalogue but not invokable on the account |
| The twin's Robot view says "Load failed" | The page is cross-origin from the service. The Settings connection check reports it. |

## Verification

| Test | What it proves |
|---|---|
| `service/vision_analyze/tests/test_app.py` (76 collected, passing 2026-10-03; +18 for handoff 4b's coercion) | Each route's validation and status codes (400/401/413/502); searched rooms forwarded and sanitised; `model_id` and `prompt_variant` rejected off-list before any model call; `/navigate/models` publishes both lists; each variant changes only what it claims (string-surgery tests); `distance_estimate` and `path_ahead` always present and coerced; `obstacle_ahead` stripped under `bearing-only` even when volunteered; region pins and per-region client caching; pins emptied by env; `/room` (3.70 S2): the name verbatim, no target needed, 400/401/502 as the others, a reply cut to one short lowercase name or `unclear` (with `_raw`), 16 tokens on `ROOM_MODEL_ID` |
| `service/vision_analyze/tests/test_review_376h2.py` | 3.76 H2: non-object JSON replies and fenced replies with trailing prose parse; unreadable or oversized (3.75 MB, also after conversion) images are 400/413 with no Bedrock call; any `important_objects` survives `/analyze`; `identify_room()` ignores empty and non-string items; non-object bodies and a non-string `media_type` are 400; an unknown default variant and a default model missing from the picker are corrected; target and room lengths bounded; the client's timeouts fit in Lambda's. Round 2: a read timeout is attempted once (local socket); unhashable `model_id`/`prompt_variant` are 400; converted images are shrunk under the byte and 8000 px limits; JSON with prose around it parses; the navigate call refuses an unknown variant; one image limit; a blank target is 400. Round 3: every Bedrock refusal (ValidationException, AccessDenied) is 502 on all four routes; a throttle or ServiceUnavailable is retried once, only once, not for other codes, and not when the budget is spent (stub client); a 4.2 MB BMP declared non-native is shrunk and sent; the request gate is the Lambda bound and fits 6 MB; shrinking takes at most 3 encodes. Round 4: the retry budget counts a slow conversion before the call (a clock stub); a native image over 8000 px or over the byte limit is shrunk to JPEG; a small one is sent untouched; unreadable native bytes are sent as before. Round 5: an image over Pillow's bomb limit is still measured and shrunk; over `MAX_DECODE_PIXELS` is a 413 (native and converted); a JPEG is decoded at reduced size |
| `tests/test_review_376h2.py` | 3.76 H2: the deploy bucket keeps Lambda zips at least 365 days (rollback; the text only -- the live value needs the explicit deploy in the operations spec), `sync.sh` waits for the invalidation to complete, attempts x Bedrock timeouts fit the template's `Timeout`; round 2: the `brain/rooms.py` and `brain/vision.py` mirrors, and (round 3 P4) `/mission/start` refusing a blank target or one over `brain.navigate.MAX_TARGET_CHARS` for the cloud policies (`vision`, `tiered`) with 400, while a 250-character `frontier` target still starts |
| `tests/test_serverless_routes.py` | Every vision route has an API Gateway route and a CloudFront behaviour, and no gateway route points at a path no app serves |
| `tests/test_lambda_packaging.py` | The vision zip installs the service's own requirements, those cover its imports, `mangum` is packaged, and `boto3` is not LISTED in `requirements-vision.txt`. It reads requirement files, not the built zip, so it does not catch that `boto3` is bundled anyway through the service's own list. |
| `tests/test_cfn_templates.py` | cfn-lint over the templates. Skips without cfn-lint. |
| `tests/test_vision.py` (9) | `brain/vision.py`'s parsing and frame entry points, with the API mocked |

**Not automated:** the real Bedrock call. After any change to
`vision_core.py`'s model call, verify it by hand with one live `/navigate`
against the deployed `ApiEndpoint`.

**Checklist for a change:** the service suite is green; the route is
listed in the gateway and in CloudFront; the code default and the
template parameter agree; a live reply echoes the expected `model_id` and
`prompt_variant`.

## Known gaps

- **`/analyze` and `/describe` have no caller in the repo.** The twin
  lists them only to strip a pasted route suffix
  (`VISION_SERVICE_ROUTES`). Both are still routed and tested.
- **Stale docstrings and comments.** `app.py`'s header still calls the
  service "ECS Fargate replacement", says `ANTHROPIC_API_KEY` is required,
  and shows `amazon.nova-lite-v1:0` as the `/navigate` default.
  `vision_core.py`'s header still describes a private subnet and a VPC
  endpoint, and the comment above `NAVIGATE_MODEL_ID` (`vision_core.py:69`)
  says to keep it in step with `cloudformation/service.yaml`'s
  `NavigateModelId`. That template's stack was deleted on 2026-09-05; the
  live parameter is `NavigateModelId` in `cloudformation/serverless.yaml`.
  The code below all of them is current.
- **`boto3` is bundled although the packaging says it is not.**
  `service/vision_analyze/requirements.txt` lists `boto3`, and `build.sh`
  installs that file into the zip, so the vision function carries its own
  `boto3`/`botocore` (about 10 MB) instead of the runtime's. Harmless
  today; the comment in `requirements-vision.txt` and the test name
  `test_boto3_is_not_bundled` overstate what is checked.
- **`room_guess` is coerced only partly**: it falls back to `unclear`
  when non-string or empty, so an invented label passes. The flags and
  `target_direction` (and `/guidance`'s `target_visible`) are coerced in
  the service since handoff 4b; the coercion takes effect on the deployed
  Lambda only after `service/lambda/build.sh` and a stack deploy.
- **Non-image bytes in a native format reach Bedrock.** The route layer
  base64-decodes non-strictly and never opens a JPEG/PNG/GIF/WebP, so a bad
  upload costs a call attempt and comes back 502. **Known limit (3.76
  round 3 P1):** Bedrock answers a malformed frame, a deployed-wrong model
  id, "on-demand throughput not supported" and a model not enabled in the
  region all with `ValidationException`, and the class cannot tell them
  apart. All are 502, so the brain reads a malformed frame Bedrock rejects
  as the cloud being unreachable (an arrival ends `arrived_unconfirmed` and
  re-asks later) -- the safer reading for the misconfigurations.
- **No throttling at the edge.** The HTTP API stage sets no throttling and
  the function no reserved concurrency; anyone holding the shared secret
  can call `/navigate` as fast as they like (3.76 H2-8, a template change
  that needs a deploy). The raw `ApiEndpoint` is as reachable as
  CloudFront, behind the same secret.
- **Error `detail` carries the botocore message** (it can name the
  function's role ARN). Kept: only a secret holder sees it, and the brain
  words its errors by what came back (3.64 D7r).
- **The two scene prompts differ.** `brain/vision.py`'s `SCENE_PROMPT`
  frames the view as the robot's. `vision_core.py`'s frames it as a photo
  taken by the owner and asks for specific object names. The schema is
  the same.
- **The arrival coercion has no direct test.** No test feeds
  `_parse_navigate_json()` a string `"true"`, or `target_reached: true`
  with `target_visible: false`. The rule is pinned only by reading.
- **No automated test of the real model call,** and no measurement of
  the three 2026-09-21 models beyond one frame each.
- **Bedrock throughput is shared** account-wide and per region. Replay
  bursts have been throttled (the recordings domain handles retries).
