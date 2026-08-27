# Web-based digital twin

A self-contained, mobile-first web page visualizing and controlling the
grid-world simulation -- open `index.html` directly in Safari on your
iPhone (or any browser). No server, no install, no App Store.

## What it is

- **Canvas view** of the starter house map -- rooms, doors, the sofa/
  fridge/backpack, the robot (with a "safety collar" ring showing the
  live collision-avoidance radius, flashing orange on a veto).
- **Manual D-pad control** and **autonomous "Explore"/"Find backpack"
  modes** -- both now call `robot/server.py` over HTTP for every move,
  sensor read, and safety check. No physics or safety logic is
  duplicated in JS anymore.
- **"Find the bag in a photo"** -- take a real photo with your phone
  camera, send it to the vision-analysis service
  (`service/vision_analyze/`, running on ECS Fargate), and see the same
  structured scene description the sim uses, plus a room guess.
- **Vision autopilot** -- a third driving mode that actually calls Claude
  Vision to decide each move, instead of the rule-based frontier
  algorithm. Since the sim has no real camera, the page renders a
  synthetic first-person "photo" with a small canvas raycaster (against
  the same map data the top-down view draws from), sends it to the
  vision service's `/navigate` route every ~2.5s along with a target
  object, and executes whatever action comes back -- still through
  `robot/server.py`'s safety veto, same as every other mode. Uses the
  same "Cloud endpoint settings" URL/secret as the photo feature above
  (deriving `/navigate` from the configured `/analyze` URL). Never
  auto-starts -- each tick is a real paid API call, so there's a call
  counter and a step cap.
- **"Guide" tab** -- the inverse of Vision Autopilot: the *person* holding
  the phone walks around, using their real live camera, while Claude
  Vision guides them toward a named object. Its own top-level tab; Start
  takes over the full screen (CSS-simulated fullscreen, not the real
  Fullscreen API, for reliability on iOS Safari) and works in both
  portrait and landscape (a dismissible sheet suggests landscape; the
  dismissal sticks between sessions), refreshed roughly every ~1s through the vision
  service's `/guidance` route (tightened from an initial ~2s -- see
  `PLAN-ar-guidance.md`'s changelog for the responsiveness/cost tradeoff),
  with the captured frame downscaled to a 960px max dimension first
  (smaller upload, faster inference). Requires this page to be loaded
  over `https://` or `localhost` -- see "Testing the camera features on a
  phone" below, since a plain `http://<lan-ip>` URL (the setup in
  "Running it" above) fails the browser's secure-context check for camera
  access. Never auto-starts the camera; only releases it on Stop
  (backgrounding the tab pauses analysis calls but keeps the camera
  running, so resuming is instant).
  - Visual-first guidance: a directional chevron (DOM/SVG, real CSS
    transitions, glides between positions) plus a screen-edge glow toward
    the direction to turn and a pulse that speeds up as proximity
    increases -- readable at a glance, not dependent on reading the small
    caption text at the bottom. Both honour `prefers-reduced-motion`,
    which swaps the pulse for a static size/brightness step so proximity
    stays encoded rather than simply frozen. Once found, it's replaced by a glowing
    outline around the object -- drawn from the response's `bounding_box`
    field, an approximate LLM-estimated rectangle (not pixel-accurate
    segmentation), refreshed each tick, not live-tracked, positioned with
    `object-fit: cover`-aware coordinate mapping (`mapNormalizedBoxToScreen()`
    -- the video is cropped to fill the screen, so naive percentage math
    misplaces the box whenever the camera's aspect ratio doesn't match the
    screen's, which is almost always in portrait).
  - Fires real `navigator.vibrate()` patterns and Web Audio directional
    tones (panned left/right, rising chime on found) on each tick.
    **iOS Safari doesn't support the Vibration API at all, on any
    version** -- a platform limitation, not a bug here -- so the audio
    cues are the actual feedback mechanism on iPhone. A mute button in
    the fullscreen HUD silences the audio cues only, and the setting is
    remembered between sessions.
  - The HUD shows a call-budget meter rather than a raw call count. The
    exact figure, and the pan-speed readout, are developer instrumentation
    hidden behind "Show developer readouts" in Settings.
  - Screen-reader support: the overlay is geometry with no accessible
    name, so a hidden `aria-live` region speaks the guidance instead
    (assertively on found). It announces on *transitions* -- a change of
    zone or proximity -- rather than per tick, since the ~1s loop would
    otherwise interrupt itself continuously. The decorative overlay,
    the video, and the visible caption are all `aria-hidden` so nothing
    is said twice. See `announceGuidance()`.

## Tabs

Three: **Guide** and **Camera** (the phone's real camera, pointed at the
real world) and **Sim** (the grid-world simulation -- map, telemetry,
manual D-pad, rule-based Explore/Find, and Vision Autopilot, all driving
the same simulated robot over the same connection). Settings lives behind
the gear in the header rather than in the tab bar, since connection
details persist and the twin reconnects on its own; the gear returns you
to whichever tab you came from.

Drive and Autonomous used to be separate top-level tabs; they were merged
into Sim because they are one robot viewed two ways, and a saved
`vp_active_tab` of either value migrates to `sim` on load.

## Sharing setup with another device

Settings has a **Show setup code**: a QR of the same magic link the
`?secret=`/`?robotSecret=` query params already implemented, plus the
service URLs when they differ from what the receiving page would derive
itself. Scanning it opens the app fully configured.

The QR *is* the secrets in visual form, so it is revealed on an explicit
tap rather than sitting in Settings permanently, takes itself back down
after 60 seconds, and disappears if you leave the tab. Anyone who
photographs the screen has the same access you do.

The encoder is implemented inline (`qrEncode()` and friends) rather than
pulled from a library or a QR web service -- this page is strictly
self-contained, and handing these secrets to a third-party image API
would be a real leak. It covers ISO/IEC 18004 byte mode, versions 1-20,
EC levels M and L. Its block-structure and alignment tables were
generated from a reference implementation, and its output was verified
module-for-module against that reference across both EC levels and all
eight masks, then decoded end-to-end with zxing-cpp.

## Architecture: this is now a real client of robot/server.py

Movement, sensing, and safety enforcement all happen server-side --
exactly mirroring the MacBook(brain)/Pi(robot) split in the main build
plan. The autonomous exploration *decision* logic (frontier-preference:
peek right/left/forward, prefer unvisited cells) intentionally still
lives in this file's JS, not on the server -- that's correct, not a
shortcut: deciding what to do next is the "brain" role, and the browser
is playing that role here the same way `brain/agent.py`'s `MissionAgent`
does in Python. Only the "robot runtime" (movement execution, distance
sensing, safety veto) moved server-side.

What's still hardcoded client-side, and why that's fine: the `LAYOUT`
and `OBJECTS` constants are for **drawing the map only** -- there's no
`GET /map` endpoint (real hardware has no grid to fetch), so this is
inherent to visualizing a simulation, not duplicated decision logic.

## Running it

1. Start the robot server: `uvicorn robot.server:app --host 0.0.0.0`
   (from the repo root). `--host 0.0.0.0` is required for your phone to
   reach it over Wi-Fi -- `127.0.0.1` only accepts connections from the
   same machine.
2. Find your Mac's LAN IP (System Settings -> Wi-Fi -> Details, or
   `ipconfig getifaddr en0` in Terminal).
3. Open `index.html` in Safari (on the same Wi-Fi network).
4. In "Robot server connection," enter `http://<your-mac-ip>:8000` and
   tap Connect.
5. D-pad, Explore, and Find backpack all now drive the real server.

## Testing the camera features on a phone (Vision Autopilot's photo upload
   works fine over plain LAN HTTP; "Guide me to..." does not)

`getUserMedia()` (live camera access, used only by "Guide me to...") is
blocked by browsers unless the page is loaded over `https://` or is
literally `localhost`/`127.0.0.1` on the same machine -- a LAN IP over
plain HTTP (the setup above) fails this check even though it works fine
for every other feature on this page. Two ways around it, cheapest first:

1. **Desktop first, no tunnel needed**: open `http://localhost:8000` in
   Chrome/Safari on the same Mac running `uvicorn` -- `localhost` is
   automatically a secure context, so this works immediately using your
   Mac's built-in webcam. Good for iterating on the throttle loop and
   overlay rendering before testing on an actual phone.
2. **Real phone, via a tunnel**: run `ngrok http 8000` (twin) and
   `ngrok http 8080` (vision-analyze, if testing locally rather than
   against the deployed cloud service) in two terminals, then open the
   twin's `https://...ngrok...` URL on your phone and paste the
   vision-analyze tunnel's URL into "Cloud endpoint settings." Two
   tunnels are needed, not one: once the twin page itself loads over
   `https://`, browsers block it from calling a plain `http://` backend
   at all (mixed-content blocking) -- and the currently-deployed cloud
   vision service is HTTP-only too (no ACM certificate on its load
   balancer), so pointing at it instead doesn't avoid this.

## Known limitations

- Mission bookkeeping (visited cells, search memory, logs) is not
  persisted -- reloading the page clears it. Connection settings *are*
  remembered on the device (server URL, vision service URL, both secrets,
  the last open tab, the mute preference), and the twin re-connects to the
  robot server automatically on load. The *robot's* actual position
  persists server-side since it's not reset on reconnect.
- "Reset mission" clears the twin's local bookkeeping only -- it does
  not (and, matching real hardware, cannot) teleport the robot back to
  a start position. Restart `uvicorn` to reset the sim's position.
- The robot-orientation triangle uses the server's `facing` field, which
  is camera-pan-adjusted (same ambiguity as `sim/grid_world.py`'s
  `frame_description()`) -- during a manual look-left/right tap the
  triangle reflects the camera pan, not strictly chassis heading. Purely
  cosmetic, doesn't affect movement or safety.
