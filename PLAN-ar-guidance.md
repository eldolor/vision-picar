# Plan: AR-style "find the object" camera guidance

**Status:** Built (2026-08-26), then redesigned (2026-08-26) into its own
full-screen tab. Implemented as the "Guide" tab in `web-twin/index.html`,
backed by `POST /guidance` in `service/vision_analyze/app.py` and
`describe_image_bytes_guidance()` in `service/vision_analyze/vision_core.py`.
The design below (AR overlay, timer-throttled analysis, 5-zone position +
proximity schema, secure-context requirement) was followed as originally
spec'd; implementation deviations from this doc are noted inline below
where they occurred (e.g. a dedicated `/guidance` route instead of
overloading `/analyze`, chosen for consistency with a sibling `/navigate`
route added for a separate feature in the interim -- see `README.md`'s
"Digital twin" section for both).

**Redesign (2026-08-26):** the user asked for the feature to feel like a
real app rather than an embedded panel:
- Its own top-level tab (`Guide`, between `Camera` and `Settings` in the
  bottom nav), not nested inside Camera next to photo upload
- **Full-screen takeover** while active -- a CSS-simulated fullscreen
  overlay (`#guide-fullscreen`, `position:fixed; inset:0`), deliberately
  *not* the real Fullscreen API (`element.requestFullscreen()`), which has
  a long history of unreliable support for arbitrary elements in iOS
  Safari. Works in both portrait and landscape (`object-fit:cover` +
  full-viewport sizing reframes automatically; an explicit
  `resize`/`orientationchange` listener re-syncs the found-outline's pixel
  position immediately on rotate rather than waiting for the next ~2s tick)
- The overlay itself moved from hand-drawn `<canvas>` shapes to DOM/SVG
  elements positioned via CSS `transform`/`left`/`top` with real
  `transition`s -- this is what makes state changes glide instead of snap;
  canvas would need a hand-rolled animation loop to get the same
  smoothness
- **Found state is now an outline, not a checkmark**: the `/guidance`
  response gained an optional `bounding_box` field (normalized 0.0-1.0
  coordinates), requested from the model only when it's confident of the
  object's extent (`null` otherwise, validated server-side in
  `_validate_bounding_box()`). This is a bounding box, not real
  segmentation -- a text-generating vision-language model doesn't produce
  pixel-accurate contours -- and it updates on the same ~2s throttle as
  everything else, not live-tracked between ticks. Falls back to a
  generic centered pulse (no box) if the model doesn't return one on a
  given "found" tick, rather than showing a guessed rectangle.
  - **`isGuidanceFound()` triggers on `proximity === "near"` alone**, not
    `position === "center" && proximity === "near"`. The first shipped
    version used that stricter AND-gate (a holdover from the old
    checkmark design, which had no way to show *where* the object was, so
    it needed the object dead-center to mean anything). Real-device
    testing (iPhone 14 Pro Max, Chrome) showed this AND-gate essentially
    never fired -- an object can be close without landing in the exact
    "center" zone on any single ~2s tick. Since the outline now shows the
    object's actual on-screen position via `bounding_box` regardless of
    which zone it's in, requiring dead-center framing was both redundant
    and the direct cause of the outline never appearing.
- **Haptic feedback** via `navigator.vibrate()`, feature-detected and
  fully implemented -- **but Apple has never implemented the Vibration
  API in Safari, on any iOS version.** This is a confirmed platform
  limitation, not a bug in this code, and not something a web page can
  work around (a native wrapper was already ruled out earlier in this
  project). The real, cross-platform substitute is a Web Audio API
  directional tone system (panned left/right oscillator tones, a rising
  chime on found, silence when not visible), which does work in iOS
  Safari and is what actually provides feedback there. **Do not
  "fix" the iOS vibration silence by trying to force it to work** -- it's
  expected; the audio cues are the intended iOS experience. A mute toggle
  in the fullscreen HUD gates the audio path only (vibration is
  independently silent by nature, so nothing to mute there).

**Polish pass (2026-08-26), after real-device feedback (iPhone 14 Pro
Max, Chrome):** the outline didn't fully cover the object, the loop felt
too slow for real-world panning, and the bottom text was the main way to
understand what to do.

- **Outline coordinate-mapping bug, found and fixed.** The video is
  displayed with CSS `object-fit: cover`, which crops it to fill the
  full-screen container -- but the bounding-box math was multiplying
  normalized coordinates directly against the *displayed* (cropped) rect,
  when the model computed them against the *full, uncropped* captured
  frame. Whenever the camera's native aspect ratio differs from the
  screen's (true almost always in portrait), `cover`'s crop silently
  shifted everything. Fixed with a proper scale+crop-offset mapping
  (`mapNormalizedBoxToScreen()` in `web-twin/index.html`) that reproduces
  exactly what `cover` does to the video, so the box lands where the
  object actually is. Fixing this surfaced a second, previously-latent
  bug: `videoWidth`/`videoHeight` are `0` until the camera stream's
  metadata actually loads, and the very first analysis tick could fire
  before that -- `0 * Infinity = NaN` in the new coordinate math, silently
  producing an invalid (and therefore invisible, since the browser
  rejects "NaNpx") outline. Fixed by explicitly awaiting the video's
  `loadedmetadata` event (`waitForGuidanceVideoReady()`) before the first
  tick, rather than assuming `srcObject` being set means the video is
  ready. The old buggy positioning code never touched `videoWidth`/
  `videoHeight` at all, which is why this race was never triggered before.
- **Speed**: `GUIDANCE_THROTTLE_MS` lowered from `2000` to `1000` (~2x API
  cost, confirmed acceptable tradeoff), plus a free win --
  `captureGuidanceFrame()` now downscales to a 960px max dimension before
  encoding (smaller upload, typically faster inference too; Bedrock
  doesn't need full sensor resolution for a coarse position/proximity/bbox
  answer). `maxTokens` was left alone -- already tight, not a meaningful
  lever. Bedrock's own inference latency (roughly 0.8-1.5s) remains a
  floor no client-side change eliminates; 1s is close to the practical
  sweet spot before ticks queue up behind a slow response.
- **Visual-first guidance, confirmed with the user**: a screen-edge glow
  (like a blind-spot indicator, brighter the further off-center the
  target is) plus a chevron pulse whose *speed* scales with proximity
  (independent of exact screen position) are now the primary "which way /
  how close" signal. The bottom caption shrank from a two-line
  sentence-plus-debug-readout into a small icon+text pill -- present for
  clarity, no longer the main channel. `isGuidanceFound()`'s relaxed
  proximity-only gate (from the redesign above) made this practical: since
  the outline already shows exactly where the object is, the chevron only
  ever needs to communicate "which way" and "how close," not "you found
  it, here's a checkmark."
- Also fixed while in the area: `isGuidanceFound()`'s "no confident box"
  fallback and the edge-glow/pulse state are all reset together in
  `hideAllGuidanceOverlays()` now (previously the edge glow had its own
  separate, easy-to-miss reset call), so `stopGuidance()` can't leave a
  glow lingering after Stop.

**Box-jump follow-up (2026-08-26)**, after further real-device feedback:
the outline math above is correct (independently hand-verified against
mismatched-aspect-ratio test cases), but on-device the box still visibly
"jumped" between ticks -- appearing off the object, then on it, then
elsewhere again. Root cause is architectural, not a math bug: each
analysis tick is a completely independent Bedrock call with zero memory
of the previous tick, so the model re-locates the object from scratch
every ~1s and its exact estimate can legitimately vary even for a
stationary object (LLM vision estimation noise) -- compounded by natural
hand movement over that second. Mitigation added:
`smoothGuidanceBox()`/`guidanceLastNormBox` blend consecutive found-state
boxes in normalized (0-1) space at a 55%-toward-the-new-estimate rate
before mapping to screen pixels, damping jitter while staying responsive
to real movement. Reset to `null` (no blending) whenever the found state
is exited (`not_visible`, a non-found position, no confident
`bounding_box` that tick, or Stop) so a stale box never drags a
freshly-reacquired detection toward an unrelated old position. **This
damps the visual jump, it does not and cannot fully eliminate it** -- a
single wildly-wrong estimate is a limit of asking a general
vision-language model to localize an object with no specialized
detection/tracking model underneath, not something client-side smoothing
can fully paper over. If jumpiness is still a problem after this, the
next lever would be a stronger confidence bar in `GUIDANCE_PROMPT_TEMPLATE`
(`service/vision_analyze/vision_core.py`) asking the model to return
`null` more readily rather than a low-confidence guess -- not yet done,
since it trades away legitimate detections too.

**Search-sweep chevron + found-gate regression fix (2026-08-26):**

- **New: systematic search sweep.** While the target hasn't been spotted
  at all (`target_visible: false`), the app now shows a directional
  chevron (reusing the same `.guide-chevron` element the off-center
  "which way" cue uses) instructing the user to physically pan the camera
  -- right for ~5 ticks (~5s at the 1s throttle), then down for ~5 ticks,
  repeating (`GUIDANCE_SCAN_PHASES`/`currentGuidanceScanPhase()` in
  `web-twin/index.html`). The right-phase also lights the right edge-glow;
  there's no top/bottom glow element, so the down-phase has no glow
  assist. Important: the model has zero information about where the
  object actually is in this state (it's not in the photo at all) --
  this is a fixed UI convention to encourage full room coverage, not a
  model-informed suggestion. Replaced (and removed entirely --
  `.guide-reticle`, its CSS, and JS references are gone) the previous
  plain spinning "searching" reticle, which gave no actionable direction.
- **Found-gate regression, fixed.** The proximity-calibration prompt
  tightening above (making "near" conservative, since the model was
  calling almost anything visible "near") had a side effect real-device
  testing caught: a centered, clearly-arrived-at object could get
  classified "medium" and never advance past a chevron to the outline.
  `isGuidanceFound()` now also accepts `proximity === "medium" &&
  position === "center"` as found, alongside the existing
  `proximity === "near"` (any position) rule -- being dead-center already
  signals the user aimed right at it, so "medium-close and centered" is a
  reasonable second definition of "found" without loosening the proximity
  prompt itself back toward over-eager.

**Auto-pause on found (2026-08-26):** the user asked whether it should
stop making API calls once the outline appears -- it didn't; it kept
polling forever at the full throttle rate even after finding the object,
burning real money for no benefit. Fixed: once `isGuidanceFound()` holds
for `GUIDANCE_FOUND_STREAK_TO_PAUSE` (2) consecutive ticks -- not just
one, to avoid pausing on a single noisy detection that flips back to
searching a moment later -- polling stops entirely (`scheduleGuidanceNext()`
now checks `state.guidancePaused`). The camera feed and the last-rendered
outline stay on screen (frozen at that position, since nothing is
updating it anymore -- a known, accepted tradeoff of not paying for
continuous polling), with a "Resume searching" button
(`pauseGuidanceSearch()`/`resumeGuidanceSearch()` in `web-twin/index.html`)
to explicitly restart the loop. `stopGuidance()`/`startGuidance()` both
reset the pause state and streak counter for a clean next session.

**UX review + idle-screen/onboarding polish (2026-08-26):** the user asked
for a UX review; two items from that review were picked to build:

- **Idle-screen preview** (`.guide-preview`): the Guide tab's pre-Start
  state used to be a plain 📷 emoji. Replaced with a small ambient
  "viewfinder" card -- the same chevron SVG the live feature uses,
  gently drifting side to side and pulsing (pure CSS `@keyframes`, no new
  assets) -- so the idle screen previews the real interaction instead of
  a static placeholder.
- **First-run onboarding** (`#guide-onboarding`): the first time Start is
  tapped, a dismissible card explains what chevron / edge glow / pulse /
  outline each mean, *before* the camera permission prompt fires
  (`showGuidanceOnboarding()` intercepts the click; `getUserMedia` isn't
  requested until "Got it, start" is tapped, which calls
  `dismissGuidanceOnboarding()` → `startGuidance()`). Shown once per page
  load via a plain in-memory flag (`guidanceOnboardingShown`), not
  localStorage -- this matches the convention already established
  elsewhere in this file (the Cloud endpoint settings hint says "Not
  saved between sessions (no localStorage in this preview)"); reloading
  the page shows it again, same as those fields losing their value.

Other items surfaced in the review but not yet built: an in-session
target-object editor (currently locked once Start is tapped), unifying
the rest of the app's visual polish to match Guide's (Drive/Autonomous
tabs are still flat button rows by comparison), reframing the call
counter as less of a debug artifact, and distinct visual treatment per
error type (camera-denied vs. network vs. missing config all look
identical today).

**Origin:** Discussed in the claude.ai session that built the rest of
this repo. The user's proposal: point the phone camera at a room, keep
it on continuously, and have the app show live directional guidance
(overlaid on the camera feed) toward a named object -- e.g. "which way
do I turn to find the red backpack." Two design decisions were made
explicitly at that time and should be treated as settled, not
re-litigated:

1. **Guidance surfaces as an AR-style overlay on the live camera feed**
   (not a text panel off to the side).
2. **Analysis is timer-throttled automatically** (not per-frame, not
   purely manual-tap-triggered).

**Updated 2026-08-26:** the vision backend moved from AWS Lambda to ECS
Fargate + Amazon Bedrock (Lambda's public entry points were blocked by
an account-level restriction -- see `README.md`'s "History: why not
Lambda?"). Every reference to `lambda/vision_analyze/` below has been
updated to `service/vision_analyze/`, and the model call is now Bedrock's
Converse API, not the direct Anthropic API. The plan's actual design
(overlay, throttle, schema, UI) is unaffected -- only the backend
integration points changed.

---

## 1. Why this doesn't deviate from the hardware plan

Same reasoning as the rest of the web twin: this feature never touches
`RobotInterface`, `robot/server.py`, or `robot/factory.py`. It's a
phone-as-sensor, human-as-actuator feature living entirely in the
vision service layer -- arguably a *better* validation of the Vision LLM
pipeline than the grid-world sim, since it's real pixels instead of
synthetic ground truth. Build it without worrying about hardware-phase
implications.

---

## 2. Platform constraint to resolve first (read before writing code)

Continuous live camera access on the web requires
`navigator.mediaDevices.getUserMedia()`, which browsers only allow in a
**secure context** -- `https://` or `http://localhost`. It will **not**
work if `web-twin/index.html` is opened as a local `file://` page, and
it will **not** work over a plain `http://<lan-ip>:port` URL either
(the setup currently used to test the twin from a phone on the same
Wi-Fi) -- both fail the secure-context check.

This is a different constraint than the existing "take a photo" feature
(`<input type="file" capture="environment">`), which opens the native
camera picker and works fine over plain HTTP -- don't confuse the two
when testing.

**Before building the frontend half of this feature, resolve hosting.**
This is the same fix needed for reliable phone access to the twin in
general (LAN-based `http://<lan-ip>` testing is fragile -- router client
isolation, firewall state, and network changes all break it, on top of
failing the secure-context check for this feature specifically).
Reasonable options, roughly cheapest/fastest first:
- **GitHub Pages**, since the repo is already on GitHub (`eldolor/vision-picar`)
  -- push `web-twin/index.html` to a `gh-pages` branch or enable Pages
  from a `/docs` or `/web-twin` folder, free, automatic HTTPS, works
  from any network (cellular included), no local-network dependency at
  all. Recommended given the repo's already in place.
- Netlify, Vercel, or Cloudflare Pages -- drag-and-drop deploy, free
  tier is plenty, marginally more setup than GitHub Pages given this
  repo's situation.
- A local dev tunnel (ngrok, Cloudflare Tunnel) pointed at a simple
  local static file server, if iterating locally is preferred over
  redeploying to test each change.

Note: hosting the twin's *static page* doesn't change how it reaches
`robot/server.py` for D-pad/autonomous control -- that still needs your
phone and Mac on the same LAN (or a tunnel to `robot/server.py` too, out
of scope here). This section is specifically about satisfying the
secure-context requirement for the camera; it's orthogonal to the robot
connection.

I checked current Safari/iOS support for WebXR (the "real" AR API) while
writing this plan and got **materially conflicting results across
sources** -- some claim Safari 18 shipped WebXR AR sessions on iOS via
ARKit delegation, others explicitly state handheld WebXR AR still isn't
exposed on iPhone as of 2026. Given that inconsistency, **don't build on
WebXR**. Recommendation below uses `getUserMedia` + a 2D canvas overlay,
which is guaranteed to work in every modern browser including iOS
Safari today, doesn't gamble on an unsettled platform feature, and is
enough to deliver the "AR-style" directional-arrow experience actually
being asked for here (this isn't a request for 3D spatial anchoring). If
WebXR's iOS status is worth rechecking at implementation time, do that
as a fast follow, not a blocker.

---

## 3. Architecture

```
<video> (live camera, getUserMedia)
       |
       v
<canvas> overlay, position:absolute on top of <video>
       |
       | every N seconds (throttle timer):
       |   1. draw current <video> frame to an offscreen canvas
       |   2. toDataURL() -> base64
       |   3. POST to the vision service (same one as the existing
       |      photo-analysis feature) with a target_object field
       |   4. response -> update the overlay's arrow/reticle + text
       v
service/vision_analyze/ (ECS Fargate) -- extended, not replaced
       |
       | same describe_image_bytes()-style call, new prompt when
       | target_object is present in the request
       v
Amazon Bedrock (Claude, Converse API)
```

No new AWS resources, no new CloudFormation -- this extends the
existing `service/vision_analyze/app.py` with a second mode on the same
`/analyze` route (or a new route -- see 4.2), keyed off an optional
`target_object` field in the request body.

---

## 4. Backend changes

### 4.1 `service/vision_analyze/vision_core.py`

Add a second prompt + function alongside the existing
`describe_image_bytes()`. Do not modify the existing function/schema --
this is additive, and the general scene-description mode still needs to
keep working for the current "find the bag in a photo" feature.

```python
GUIDANCE_PROMPT_TEMPLATE = """You are helping someone find a {target_object} using their phone camera.
Look at this photo and determine:
1. Is the {target_object} visible in this image?
2. If visible, where is it positioned horizontally in the frame?
3. How far away does it appear?
4. What direction should the person move or turn to get closer to it?

Respond with ONLY a JSON object, no other text, matching this schema:
{{
  "target_visible": true | false,
  "position": "far_left" | "left" | "center" | "right" | "far_right" | "not_visible",
  "proximity": "near" | "medium" | "far" | "unknown",
  "guidance": "short human-readable instruction, e.g. 'Turn right and walk forward'"
}}"""

_GUIDANCE_EMPTY_SCHEMA = {
    "target_visible": False,
    "position": "not_visible",
    "proximity": "unknown",
    "guidance": "Unable to analyze image.",
}

def describe_image_bytes_guidance(image_bytes: bytes, target_object: str, media_type: str = "image/jpeg") -> dict:
    # Same Bedrock Converse-API call pattern as describe_image_bytes()
    # (see that function for the boto3 client / image-format handling),
    # but with GUIDANCE_PROMPT_TEMPLATE.format(target_object=target_object)
    # as the prompt text block, and _GUIDANCE_EMPTY_SCHEMA as the parse
    # fallback in place of _EMPTY_SCHEMA.
    ...
```

Deliberately uses a **discrete 5-zone horizontal position** rather than
asking the model for pixel coordinates or a bounding box -- LLM vision
output for exact spatial coordinates is unreliable; a coarse zone is
something the model can answer consistently and is enough to drive a
directional arrow.

### 4.2 `service/vision_analyze/app.py`

Add an optional `target_object` field to the existing `/analyze` request
body schema (or add a new `/guidance` route if keeping the two modes
more clearly separated -- either is fine, but pick one and be
consistent; a single route with an optional field is slightly less
frontend/backend surface to keep in sync). When present, call
`describe_image_bytes_guidance()` instead of `describe_image_bytes()`.
When absent, behavior is unchanged (existing "analyze photo" feature
must keep working exactly as-is -- this is backward compatible, not a
breaking change).

```python
body = await request.json()
image_b64 = body["image_base64"]
media_type = body.get("media_type", "image/jpeg")
target_object = body.get("target_object")  # NEW, optional

image_bytes = base64.b64decode(image_b64)

if target_object:
    result = describe_image_bytes_guidance(image_bytes, target_object, media_type)
else:
    result = describe_image_bytes(image_bytes, media_type)
    result["room_guess"] = identify_room(result.get("important_objects", []))
```

(Follow the existing route's decode/size-limit/error-handling structure
already in `app.py`'s `/analyze` handler -- this snippet only shows the
part that changes.)

### 4.3 `brain/vision.py`

Mirror the same `describe_image_bytes_guidance` logic here too (as
`describe_image_guidance(image_path, target_object)` following the
existing file-path-based pattern in this module), for parity with how
`service/vision_analyze/vision_core.py` intentionally duplicates
`brain/vision.py` already -- see `CLAUDE.md` section 6 on why that
duplication is accepted, not a bug. Note `brain/vision.py` stays on the
direct Anthropic API (unchanged by the Lambda->ECS pivot, which only
affected the cloud-deployed copy) -- don't switch this one to Bedrock
without a separate, deliberate decision to do so.

### 4.4 Tests

`service/vision_analyze/` currently has **no automated test suite at
all** (a gap left by the Lambda->ECS migration -- see `CLAUDE.md`
section 5, item 1). Writing that base test suite is a prerequisite for
this section, not something to build alongside it from scratch here.
Once it exists (using FastAPI's `TestClient`, mocking
`vision_core.describe_image_bytes`/`describe_image_bytes_guidance` the
way the old Lambda suite mocked the Anthropic client), add:
- `target_object` present -> `describe_image_bytes_guidance` is called,
  not `describe_image_bytes`
- `target_object` absent -> existing behavior unchanged (regression
  check)
- Guidance response schema round-trips correctly through the route
- Malformed/garbage model response -> falls back to
  `_GUIDANCE_EMPTY_SCHEMA` gracefully (same pattern as
  `_parse_scene_json`'s existing fallback test)

Add equivalent tests to `tests/test_vision.py` for
`describe_image_guidance()`.

---

## 5. Frontend changes (`web-twin/index.html`)

New section, e.g. "Guide me to..." -- additive, doesn't replace the
existing "Find the bag in a photo" panel (that one-shot upload flow
stays as-is; this is a new continuous-camera mode). **Requires the
hosting fix in section 2** -- this section's UI will silently fail to
get camera access (or throw a `getUserMedia` permission/security error)
if tested over `file://` or plain `http://<lan-ip>`.

### 5.1 UI elements

- Text input for the target object name (default prefilled: "red
  backpack", matching the sim's existing target object for consistency)
- Start/Stop toggle button -- **do not auto-start the camera or the
  analysis loop on page load.** The user must explicitly opt in, both
  for the camera permission prompt and because each analysis call costs
  money (see section 6).
- `<video>` element, `autoplay playsinline muted` (playsinline is
  required on iOS Safari or video attempts to go fullscreen), sourced
  from `getUserMedia({video: {facingMode: "environment"}})`
- `<canvas>` overlay, `position: absolute` directly on top of the
  `<video>`, same dimensions, transparent background, non-interactive
  (`pointer-events: none`) so it doesn't block any future tap
  interactions on the video itself
- Status text below the video: current guidance string from the last
  response, plus a subtle "analyzing..." indicator during in-flight
  requests (don't leave the UI silent for the 1-2s round trip)

### 5.2 Overlay rendering (suggested v1 mapping -- adjust as needed)

Draw on the overlay canvas based on the latest response's `position` /
`proximity` / `target_visible`:

| `position` | Arrow |
|---|---|
| `far_left` | large left-pointing chevron |
| `left` | medium left-pointing chevron |
| `center` | up/forward chevron (or a checkmark if `proximity: "near"`) |
| `right` | medium right-pointing chevron |
| `far_right` | large right-pointing chevron |
| `not_visible` | a slowly rotating/pulsing "searching" reticle, plus guidance text like "turn slowly to search" |

Color: reuse the twin's existing design tokens
(`--accent-safe` #3ECF8E when `position: "center"` and
`proximity: "near"` -- i.e. "you've basically found it"; `--text`
#E8ECEF otherwise). Don't introduce new colors outside the existing
palette -- see the `:root` CSS variables at the top of `index.html`.

### 5.3 Throttle loop

```js
const THROTTLE_MS = 1500; // starting point -- see cost/latency tradeoff below
let guidanceTimer = null;
let guidanceInFlight = false;

function startGuidance(targetObject) {
  stopGuidance();
  guidanceTimer = setInterval(async () => {
    if (guidanceInFlight) return; // skip a tick rather than overlap requests
    guidanceInFlight = true;
    try {
      const frameBase64 = captureVideoFrame(); // draw <video> to an offscreen canvas, toDataURL
      const result = await callGuidanceEndpoint(frameBase64, targetObject);
      renderOverlay(result);
    } catch (e) {
      // network/API failure -- show a brief error state, keep the loop running
    } finally {
      guidanceInFlight = false;
    }
  }, THROTTLE_MS);
}
```

Key correctness points, not optional:
- **Guard against overlapping requests** (`guidanceInFlight`) -- a slow
  response shouldn't cause two in-flight calls stacking up.
- **Pause when the tab/page isn't visible** -- listen for
  `visibilitychange` and stop the timer when hidden, restart on
  visible, so backgrounding the browser doesn't keep burning API calls.
- **Stop the camera stream** (`track.stop()` on all tracks) when the
  user taps Stop or navigates away -- don't leave the camera light on.

---

## 6. Cost and latency (already discussed, restated for reference)

Each analysis call is a real Bedrock Claude vision call: roughly the
same order of magnitude as the direct Anthropic API pricing this
project's cost breakdown was originally based on (see `README.md`),
though Bedrock's exact per-token pricing for the model in use
(`us.anthropic.claude-sonnet-4-5-20250929-v1:0` -- see
`service/vision_analyze/vision_core.py`'s docstring for why this model,
not `claude-sonnet-5`) should be double-checked against current AWS
Bedrock pricing before treating the number below as exact. Roughly
1-2s round trip under normal conditions. At a 1.5s throttle interval, a
5-minute active session is roughly 150-200 calls, i.e. **on the order of
$0.50-0.80 per 5-minute session** (approximate, see above). That's fine
for personal testing but worth showing the user -- consider surfacing a
running call/cost counter in the UI so it's not a surprise, and
definitely don't auto-start the loop on page load (see 5.1).

This is separate from, and additive to, the **fixed monthly cost of the
underlying ECS Fargate/ALB/NLB/VPC-endpoint infrastructure**, which now
runs regardless of whether this feature is ever used (unlike the old
Lambda's pay-per-invocation model). This feature doesn't change that
fixed cost -- it only adds marginal per-call cost on top of it.

`THROTTLE_MS = 1500` is a starting point, not a tuned value -- adjust
based on how it feels in practice. Slower (2000-3000ms) trades
responsiveness for cost; there's little point going faster than the
model's own 1-2s response time, since requests would just queue up
behind `guidanceInFlight`.

---

## 7. Testing plan

Backend logic (schema parsing, request routing, fallback behavior) is
straightforward to unit test with mocks, following the pattern that
needs to be established first for `service/vision_analyze/` (see 4.4)
and the existing pattern in `tests/test_vision.py` -- do that.

The frontend camera/overlay loop is not practically unit-testable
(depends on real camera hardware and a real secure-context browser).
Manual QA checklist for whoever tests this on-device (requires the
hosted-over-HTTPS twin from section 2, not a LAN IP or local file):
- [ ] Camera permission prompt appears on Start, feed displays
- [ ] Overlay arrow updates roughly every `THROTTLE_MS`, doesn't flicker
      or stack up requests
- [ ] Panning the phone away from the target correctly shows
      `not_visible` / searching state
- [ ] Panning back toward it recovers `position`/`proximity` sensibly
- [ ] Stop button actually releases the camera (check the browser's
      camera-in-use indicator disappears)
- [ ] Backgrounding the browser pauses the loop; foregrounding resumes it
- [ ] A network failure mid-session (e.g. airplane mode toggle) doesn't
      crash the page -- loop should recover on the next tick

---

## 8. Open questions for the user, not yet decided

Claude Code should either make a reasonable default choice and note it,
or ask -- these weren't settled in the original design discussion:

- Should the target object be free text, or a dropdown of known objects
  (matching `sim/maps/starter_house.py`'s objects, for consistency
  with the sim side of this project)?
- Should there be a "found it" confirmation state (e.g. a persistent
  green checkmark + haptic/sound) once `proximity: "near"` and
  `position: "center"` hold for N consecutive polls, or is per-frame
  guidance enough?
- Should `THROTTLE_MS` be user-adjustable in the UI (a slider trading
  cost for responsiveness), or fixed?

---

## 9. Non-goals for this task

- Not building true 3D/spatial AR (WebXR, ARKit) -- see section 2.
- Not replacing the existing one-shot "find the bag in a photo" upload
  feature -- this is additive.
- Not adding authentication/user accounts -- reuses the same
  `x-app-secret` shared-secret pattern already in
  `service/vision_analyze/app.py`.
- Not deploying new AWS infrastructure -- this plan assumes
  `service/vision_analyze/` is already deployed (it is -- see the live
  NLB endpoint in `README.md`), and only adds a route/field to it.
- Not solving the robot-server LAN-connection problem for D-pad/autonomous
  control -- section 2's hosting fix is specifically about the camera
  feature's secure-context requirement, not about `robot/server.py`
  reachability.
