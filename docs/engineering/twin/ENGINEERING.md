---
kind: engineering
domain: twin
status: current
verified: 2026-10-02
parent: docs/twin/ARCHITECTURE.md
---

# Web twin -- engineering

How the web twin is built, served, deployed and tested today. The what and
the why are in the [architecture spec](../../twin/ARCHITECTURE.md). This
document is true only until the implementation changes, and it is updated
in the same commit as the code. For a button-by-button walkthrough, see
`FEATURES.md` (sections 1, 3 and 4 were checked against the code on
2026-10-02; see Known gaps for where they drift).

## Implementation

| File | What it is |
|---|---|
| `web-twin/index.html` | Markup and all CSS. Three tab pages (`data-tab="guide"`, `"sim"`, `"settings"`). The tab bar carries Guide and Sim; Settings opens from the header gear (`#btn-settings`). |
| `web-twin/app.js` | All behaviour (about 5,400 lines): one IIFE in `"use strict"`, with no modules and no build step. Sections are marked by `// ---------- <name> ----------` banners, listed below. |
| `web-twin/manifest.json`, `web-twin/icons/` | PWA manifest and the three home-screen icons |
| `web-twin/README.md` | How to run, reach, deploy and test the page |
| `service/static/assets.json` | Every file published to the static bucket, with its S3 key, content type and cache policy (`none` = `no-cache, must-revalidate`, `day` = `public, max-age=86400`, as `service/static/sync.sh` writes them). It also lists the admin and metrics consoles. |
| `service/static/sync.sh` | Uploads exactly that list with `put-object` (never `aws s3 sync`), then invalidates `/*` |
| `robot/server.py` | Serves the page locally: `GET /`, `/app.js`, `/manifest.json`, `/icons/*.png`. Only `/app.js` sends `Cache-Control: no-cache`; `GET /` is a plain `FileResponse` with no `Cache-Control`, so locally the HTML may be reused from cache (heuristically) against a fresh script. |

**`app.js` sections, in file order:** tester error capture (registered
first) · icon system (inline SVG, `data-icon`) · twin state · persisted
preferences · shared UI helpers · robot server client (`apiGet`, `apiPost`,
`sendAction`, `tunnelHeaders`) · vision URL helper (`deriveServiceUrl`,
endpoint mismatch notes) · M2 depth grid · odometry · N1 world model (map,
pose, SLAM error, tap-to-goal) · action execution · R0 turn step ·
connection · rendering · S2 camera frame and frame source · environment
banner · controls · remote brain (`brainApi`, polling, start and stop) · P2
tier readouts · M5 health line · brain liveness · cloud endpoint check ·
walk recording · `/navigate` model and prompt pickers · Guide camera loop
(both modes, Drive via brain, overlays, audio, budget) · tabs · init ·
restore session · QR encoder · setup sharing · debug capture · target
history · magic-link prefill.

## Interfaces

**Calls to the robot server** (`state.serverUrl`; `x-app-secret` from the
robot secret; `tunnelHeaders()` adds `ngrok-skip-browser-warning: 1` only
for hosts matching `NGROK_HOST`):

| Route | Used by | Notes |
|---|---|---|
| `POST /action` | D-pad, look buttons | `x-driver: twin-dpad`; body `{action, angle?}`, where `angle` is the turn step |
| `POST /stop` | STOP | `x-driver: twin-dpad`. Does NOT cancel a nav2 goal: the server only calls `robot.stop()`, and the bridge cancels a goal only on a non-zero `twin-dpad` twist, so a goal resumes after the stop hold (H1 in `docs-review/SPEC-REVIEW-2.md`). Decided by the user 2026-10-02, the safety domain's rule: the server will also cancel the goal; decided fix not yet built, and nothing changes in the page for it. |
| `GET /frame` | Sim camera | `image_base64` drawn as-is; no pixels sets the frame source to `none` |
| `GET /distance`, `GET /depth` | Sim readouts | The depth strip draws the server's zones and outlines the `path` zones it reports |
| `GET /odometry` | odometry line | "no encoders on this backend — pacing falls back to frame count" when unusable; "not reported" when the route is missing |
| `GET /world/map`, `GET /world/pose`, `GET /world/error` | the map | Tri-state cells at the server's size. Under SLAM, the truth is drawn as a ghost and the errors are printed. |
| `GET /world/goal`, `POST /world/goal` | tap-to-goal | `{x_m, y_m}` in the house frame; only when the map is SLAM's; `accepted: false` shows the reason. `onMapTap()` sends it through `apiPost()` with NO `x-driver` header. The header would not matter: `world_goal_set()` in `robot/server.py` always arbitrates a goal as `DRIVER_ROS` (3.23), so during a mission a person's tap comes back `accepted: false`, `reason: preempted`, and the toast says "Could not send the goal: preempted". |
| `GET /health` | watchdog readout, driver and last refusal, `min_distance_cm`, Drive via brain pre-flight (`mode` must be `teleop`) | polled every `WATCHDOG_POLL_MS` |
| `POST /teleop/frame` | Drive via brain | `{image_base64, media_type}`, one push per Guide tick; also primed once before `/mission/start` |

**Calls to the brain** (`state.brainUrl`; the brain's own secret via
`brainAuthHeaders()`):

| Route | Used by | Notes |
|---|---|---|
| `GET /health` | connect, liveness ping, recording pre-check (`recording_allowed`), tier availability (`perception_available`) | |
| `POST /mission/start` | Remote brain Start | `{target_object, fault, policy, model_id?, prompt_variant?}`. The model fields are sent only for `vision` or `tiered`, and only when picked. |
| `POST /mission/start` | Drive via brain | `{target_object, policy: state.drivePolicy, model_id, prompt_variant}` |
| `GET /mission/status` | panel poll (`BRAIN_POLL_MS`); each Drive-via-brain tick | Reshaped by `missionStatusToRobotResult()` for the Guide overlay |
| `POST /mission/stop` | Stop | |
| `POST /recording/frame`, `POST /recording/finish` | Record this walk | Schemas in the recordings engineering spec |

`brainGone(e)` counts only no-response and 502/503/504 as "lost". Any
other status means a brain answered.

**Calls to the vision service** (`cfg-url`, via
`deriveServiceUrl(url, route)`, which strips a pasted `/analyze`,
`/describe`, `/navigate` or `/guidance` suffix):

| Route | Used by |
|---|---|
| `POST /navigate` | Robot view. Body `{image_base64, media_type, target_object, model_id?, prompt_variant?}`. |
| `POST /guidance` | Guide me |
| `GET /navigate/models` | the model and prompt pickers; the prompt picker hides when only one wording is offered |
| `GET /health` | Settings connection check, plus a same-origin comparison |

The page also fetches a relative `GET <page dir>/health` for the
environment banner (`env_label`).

**Magic-link query parameters:** `secret` (vision), `robotSecret`,
`brainSecret`, `visionUrl`, `serverUrl`, `brainUrl`. They are read once
at load by the prefill section.

**Persisted preferences** (in `localStorage`): through `PREF`,
`vp_server_url`, `vp_brain_url`, `vp_cfg_server_secret` (robot secret),
`vp_vision_url`, `vp_cfg_secret` (vision secret), `vp_active_tab`,
`vp_guide_muted`, `vp_guide_onboarded`, `vp_guide_rotate_dismissed`,
`vp_debug_readouts`, `vp_record_walk`, `vp_drive_via_brain`,
`vp_drive_policy`, `vp_turn_step_deg`, `vp_navigate_model_id`,
`vp_navigate_prompt_variant`, `vp_brain_policy`. The two secret keys in
`PREF` are also read and written by literal name, in the prefill section
and the Settings secret inputs. Outside `PREF`: the brain secret
`vp_cfg_brain_secret` (same two places), `guidanceMode` (`guide` or
`robot`, unprefixed, set by `setGuidanceMode()`), `vp_debug_log` and
`vp_target_history`. `PREF` reads and writes go through try/catch
helpers (`prefGet()` and its siblings); the prefill section's direct
writes sit inside its own try block.

**Default endpoints when no saved value exists:** robot = the page's
origin. Vision = `:8080` on localhost, otherwise the page's host. Brain =
the page's host on `:8001`. A saved value always wins. The vision default
is right only for a page served from CloudFront (vision = the site). On a
laptop running `service/tunnel/run.sh`, :8080 is the tunnel proxy, not
the vision service, and the proxy forwards vision only under `/vision`:
set the vision URL by hand to `http://127.0.0.1:8080/vision` for a page
loaded locally, or `https://<domain>/vision` for a page loaded through
the tunnel (the table in the cloud-vision engineering spec, "Port 8080 is
shared with the tunnel proxy").

## Parameters and configuration

| Constant | Value | Unit | Where | Why |
|---|---|---|---|---|
| `GUIDANCE_THROTTLE_MS`, `ROBOT_THROTTLE_MS` | 500 | ms | Guide loop | Gap between dispatches |
| `GUIDANCE_MAX_IN_FLIGHT` | 2 (1 under Drive via brain) | calls | `guidanceStep()` | At 8 concurrent replay workers Bedrock throttled 10 of 22 frames; a mission must decide from the current frame |
| Error backoff | `min(30000, base * 2^streak)` | ms | Guide loop | Replaces the scheduled tick rather than adding to it |
| `GUIDANCE_MAX_CALLS`, `ROBOT_MAX_CALLS` | 120 | calls / session | Guide loop | Caps a forgotten tab's spend; reserved at dispatch |
| `CAPTURE_MAX_DIM` | 1280 | px, long edge | capture | 960 never engaged on a 640 px stream. The recorder saves the same bytes. |
| `GUIDANCE_STILL_SKIP_DEG_PER_SEC` / `GUIDANCE_MAX_CONSECUTIVE_SKIPS` | 3 / 3 | deg/s / ticks | capture | Skip a paid call while the phone is still |
| `GUIDANCE_FOUND_STREAK_TO_PAUSE` / `ROBOT_REACHED_STREAK_TO_PAUSE` | 1 / 2 | answers | pause logic | Robot view ends a run silently, so it takes one extra confirmation |
| `BRAIN_POLL_MS` | 700 | ms | Remote brain panel | |
| `BRAIN_LIVENESS_MS` | 5000 | ms | reconnect loop | A restarted brain used to leave Start dead |
| `WATCHDOG_POLL_MS` | 500 | ms | `/health` poll | |
| `TURN_STEPS_DEG` | 15, 45, 90 (default 90) | deg | D-pad turn step | 90 is what every tap sent before R0 |
| `MIN_DISTANCE_CM_FALLBACK` | 20 | cm | depth readout | Used only until `/health` supplies the server's value |
| `QR_REVEAL_MS` | 60000 | ms | setup code | The QR is the secrets |
| `DEBUG_LOG_MAX` | 20 | entries | debug capture | "Copy debug info" |
| `NGROK_HOST` | `(^\|\.)ngrok(-free)?\.(app\|dev\|io)$` | regex | `tunnelHeaders()` | Header only for tunnel hosts |
| `DRIVER_DPAD` | `twin-dpad` | | `sendAction()` | M4 priority |

There is no server-side configuration. The page reads everything from the
servers at run time.

## Procedures

**Run locally** (from the repo root):

```bash
uvicorn robot.server:app --port 8000
uvicorn control.brain_server:app --port 8001
# open http://127.0.0.1:8000/ ; Settings -> brain http://127.0.0.1:8001 -> Connect
```

Restarting the robot server resets the robot's pose; there is no reset
route. The camera features need HTTPS or localhost, and a plain
`http://<lan-ip>` fails the secure-context check.

**Reach a laptop stack from a phone, and deploy the page.** Both
procedures are operations', and the canonical copies are in the
[operations engineering spec](../operations/ENGINEERING.md#procedures):
"Run the local half for the deployed twin" (the tunnel) and "Publish the
twin and consoles" (`service/static/sync.sh`, the invalidation, and the
parity check). They are not repeated here. What is the twin's own:

- **Settings after the tunnel is up:** robot `https://<domain>`, brain
  `https://<domain>/brain`, and the local secret in both. The vision URL
  depends on where the page was loaded from: the site itself for a page
  from CloudFront; `https://<domain>/vision` for a page loaded through the
  tunnel (the deployed site directly is cross-origin there, and the
  serverless stack answers no preflight, so every call fails as "Load
  failed"). The vision secret is the deployed vision service's in every
  case: the proxy forwards it unchanged. The page sends `ngrok-skip-browser-warning`
  only to hosts matching `NGROK_HOST`; a tunnel on any other provider's
  domain gets the interstitial HTML and fails with a JSON parse error.
- **Publishing a page change:** the change is not shipped until the
  deployed `app.js` matches `HEAD`. The stack's `SiteUrl` output already
  carries the scheme and the trailing slash, so the deployed script is
  `<SiteUrl>app.js`. A diff after the invalidation reports `Completed`
  means the wrong commit was synced.

**Add a file the page references.** Add it to `service/static/assets.json`
(key = the relative URL path, an explicit type and a cache policy), and
add a server route in `robot/server.py` for local runs.
`tests/test_static_assets.py` fails until both pages' references are
published.

**Add a readout.** Read a server route. Render three states: a value,
"not reported by this server" on 404, and an explicit "none". Add a UI
test at 390 px and watch it fail against the bug first.

**Run the UI tests:**

```bash
python -m playwright install chromium      # once
pytest tests/test_ui.py tests/test_ui_admin.py tests/test_ui_pipeline.py tests/test_frame_source.py -v
```

Run them in module order, not only with `-k`. A timing test in
`tests/test_ui_pipeline.py` passed alone and failed beside its
neighbours.

## Verification

| Test | Collected | What it pins |
|---|---|---|
| `tests/test_ui.py` | 110 | At `PHONE = 390x844`: the page loads with no script error; the model picker populates and is readable; endpoint notices; secret and reachability errors; recording refuses without a brain; no double walk on a double tap; the env banner; the policy pickers and hints for vision and tiered; the depth strip (zones, path, blind and unmeasurable states, the missing route); the turn step sent and remembered; driver and refusal readouts; the health line (ok, unhealthy, a robot against a wall still ok); spin and sized turns; brain auto-reconnect; tier readouts and "not enforced" corroboration; the cloud check's cross-origin diagnosis; the ngrok header only to tunnel hosts; odometry; the pacing row; the map at server size, unknown cells, the SLAM ghost, tap-to-goal only on SLAM |
| `tests/test_ui_pipeline.py` | 5 | Against a real threaded stub: calls overlap, concurrency is capped, an overtaken answer is not drawn, overlapping frames get distinct sequence numbers, a previous session's answer never reaches the next |
| `tests/test_frame_source.py` | 3 | The readout says `server` for a real frame, and `none` (with nothing drawn) for a frame with no pixels |
| `tests/test_static_assets.py` | 10 | Every file both pages and the manifest reference is published; keys mirror paths; `admin` stays extensionless; pages and scripts are no-cache; `sync.sh` reads the same manifest |
| `tests/test_ui_admin.py` | 11 | The admin console at phone width (recordings domain) |

The UI tests skip without Chromium. A green run without a browser has
tested nothing here.

**Checklist for a change to the page:** the UI tests run in module order
with a browser; a phone-size screenshot is part of the evidence
(`CLAUDE.md` section 7); new assets are in the manifest; the deployed
`app.js` matches `HEAD` after the sync.

## Known gaps

- **The environment banner cannot show on the CloudFront deployment.**
  `renderEnvBanner()` fetches a relative `/health`. Under CloudFront that
  route goes to the vision function, whose `/health` returns only
  `{"status": "ok"}`. The comment above it (`web-twin/app.js`, "always
  true is that robot/server.py served this HTML") holds only for local
  runs.
- **The health verdict is duplicated.** `robotProblem()` and
  `brainProblem()` re-implement `control/health.py`'s rules (a watchdog
  poll older than 10 intervals; a running mission with no tick past
  `tick_timeout_s`), although the comment says the page "never invents a
  judgement of its own". They agree today and nothing pins them together.
- **`missionStatusToRobotResult()` hardcodes `obstacle_ahead: false`,**
  and that value is recorded into teleop walks (see the recordings spec).
- **Leftovers of the deleted local brain:** `DRIVER_LOCAL_BRAIN` is still
  defined, and comments still mention the Vision Autopilot.
- **`FEATURES.md` drift:** section 1.2.c says the status is polled "at
  the same 500ms cadence" (true per Guide tick under Drive via brain; the
  Sim panel polls at 700 ms). Section 4's banner text inherits the false
  premise above.
- **Tap-to-goal is arbitrated as autonomous.** `onMapTap()` sends no
  `x-driver`, and the server ranks every goal as `ros`, so a person's
  tap cannot pre-empt a mission and is refused `preempted` while one
  holds the robot. The architecture spec records the exception and the
  open question; the driver order is the safety domain's.
- **The vision connection check misdiagnoses a working cross-origin
  setup.** `checkVision()` treats any URL failing `sameOrigin()` as
  "Reachable, but blocked by the browser ... publishes no CORS headers",
  without looking at the reply's headers. That is true of the deployed
  vision service and false of a local one (`ALLOWED_ORIGINS`) and of the
  tunnel's `/vision` route (`service/tunnel/proxy.py` answers the
  preflight and adds the headers), both of which work from a
  cross-origin page.
- **The endpoint mismatch notice is suppressed whenever the page's own
  host is `localhost` or `127.0.0.1`** (`hostDiffersFromPage()`), so a
  locally loaded page pointed at another deployment gets no notice.
- **The S1 contract button is not built.**
