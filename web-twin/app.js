// web-twin/app.js -- the digital twin's whole client.
//
// Extracted verbatim from index.html, where it lived as one inline <script>
// since the twin was first written. It is served as its own asset by
// robot/server.py (GET /app.js), which means a public path, which means
// cloudformation/twin.yaml's ListenerRule had to learn about it -- the twin's
// rules match an EXACT path set (see that file), so a new asset is a
// deployment change, not just a file move. That coupling is the reason this
// stayed inline as long as it did.
//
// Served with Cache-Control: no-cache rather than a hashed filename: the
// browser revalidates against the ETag FileResponse already sends and gets a
// 304 when unchanged, so a redeploy can never leave a stale script running
// against fresh HTML. A content-hashed URL would be faster by one
// conditional request and would need HTML templating to emit; this page is
// one file served to one household.
//
// Still one IIFE, deliberately: splitting it into modules is a separate
// change with its own risk, and this one is meant to be byte-identical
// behaviour. What it buys immediately is a file that an editor, a linter and
// `node --check` can all read.

(function () {
  "use strict";

  // ---------- tester error capture ----------
  // Registered first, before any other statement in this file, so that a
  // failure during init is recorded too. These listeners used to sit near
  // the end, which meant an exception thrown on the way there was captured
  // nowhere -- and an init failure is precisely the kind of "it just looked
  // broken" report this buffer exists to explain. "Copy debug info" in
  // Settings reads it back.
  const DEBUG_LOG_KEY = "vp_debug_log";
  const DEBUG_LOG_MAX = 20;
  function pushDebugLog(entry) {
    try {
      const log = JSON.parse(localStorage.getItem(DEBUG_LOG_KEY) || "[]");
      log.push(Object.assign({ t: new Date().toISOString() }, entry));
      localStorage.setItem(DEBUG_LOG_KEY, JSON.stringify(log.slice(-DEBUG_LOG_MAX)));
    } catch (e) { /* best-effort -- never let logging itself throw */ }
  }
  window.addEventListener("error", function (e) {
    pushDebugLog({
      type: "error",
      message: e.message,
      source: e.filename + ":" + e.lineno + ":" + e.colno,
      stack: e.error && e.error.stack,
    });
  });
  window.addEventListener("unhandledrejection", function (e) {
    const reason = e.reason;
    pushDebugLog({
      type: "unhandledrejection",
      message: String((reason && reason.message) || reason),
      stack: reason && reason.stack,
    });
  });

  // ---------- icon system ----------
  // One inline-SVG icon set (stroke-based, currentColor, 24x24 viewBox)
  // replaces every OS emoji glyph that used to render inconsistently
  // across platforms next to this otherwise monochrome dark UI. Static
  // icons (tab bar, D-pad, onboarding rows, ...) are marked in the HTML
  // with data-icon="name" and filled in by applyStaticIcons() below;
  // dynamic ones (the guide caption icon, the mute button) are set
  // directly via ICON.name at their call sites, same lookup table.
  const ICON_ATTRS = 'viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"';
  const ICON = {
    chevronUp: '<svg ' + ICON_ATTRS + '><path d="M6 15l6-6 6 6"/></svg>',
    chevronDown: '<svg ' + ICON_ATTRS + '><path d="M6 9l6 6 6-6"/></svg>',
    chevronLeft: '<svg ' + ICON_ATTRS + '><path d="M15 6l-6 6 6 6"/></svg>',
    chevronRight: '<svg ' + ICON_ATTRS + '><path d="M9 6l6 6-6 6"/></svg>',
    eye: '<svg ' + ICON_ATTRS + '><path d="M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7-10-7-10-7z"/><circle cx="12" cy="12" r="3"/></svg>',
    close: '<svg ' + ICON_ATTRS + '><path d="M18 6L6 18M6 6l12 12"/></svg>',
    volumeOn: '<svg ' + ICON_ATTRS + '><path d="M4 9v6h4l5 4V5L8 9H4z"/><path d="M16 8a5 5 0 0 1 0 8"/><path d="M19 5a9 9 0 0 1 0 14"/></svg>',
    volumeOff: '<svg ' + ICON_ATTRS + '><path d="M4 9v6h4l5 4V5L8 9H4z"/><path d="M17 9l5 5M22 9l-5 5"/></svg>',
    search: '<svg ' + ICON_ATTRS + '><circle cx="11" cy="11" r="7"/><path d="M21 21l-4.3-4.3"/></svg>',
    target: '<svg ' + ICON_ATTRS + '><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3.5"/></svg>',
    warning: '<svg ' + ICON_ATTRS + '><path d="M12 3l9.5 17H2.5L12 3z"/><path d="M12 10v4"/><path d="M12 17.3v.1"/></svg>',
    clock: '<svg ' + ICON_ATTRS + '><circle cx="12" cy="12" r="9"/><path d="M12 7v5l4 2"/></svg>',
    car: '<svg ' + ICON_ATTRS + '><path d="M4 16v-3l1.8-4.8A2 2 0 0 1 7.7 7h8.6a2 2 0 0 1 1.9 1.2L20 13v3"/><path d="M2 16h20"/><circle cx="7" cy="18.2" r="1.6"/><circle cx="17" cy="18.2" r="1.6"/></svg>',
    bot: '<svg ' + ICON_ATTRS + '><rect x="5" y="8" width="14" height="10" rx="2.5"/><path d="M12 8V4.5"/><circle cx="12" cy="3.2" r="1"/><circle cx="9" cy="13" r="1" fill="currentColor"/><circle cx="15" cy="13" r="1" fill="currentColor"/><path d="M9 16.5h6"/></svg>',
    camera: '<svg ' + ICON_ATTRS + '><path d="M4 8h3l1.6-2h6.8L17 8h3a1 1 0 0 1 1 1v9a1 1 0 0 1-1 1H4a1 1 0 0 1-1-1V9a1 1 0 0 1 1-1z"/><circle cx="12" cy="13.2" r="3.4"/></svg>',
    mapPin: '<svg ' + ICON_ATTRS + '><path d="M12 21s7-6.4 7-11.5A7 7 0 0 0 5 9.5C5 14.6 12 21 12 21z"/><circle cx="12" cy="9.5" r="2.3"/></svg>',
    settings: '<svg ' + ICON_ATTRS + '><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.7 1.7 0 0 0 .3 1.9l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-1.9-.3 1.7 1.7 0 0 0-1 1.6V21a2 2 0 1 1-4 0v-.1a1.7 1.7 0 0 0-1-1.6 1.7 1.7 0 0 0-1.9.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.7 1.7 0 0 0 .3-1.9 1.7 1.7 0 0 0-1.6-1H3a2 2 0 1 1 0-4h.1a1.7 1.7 0 0 0 1.6-1 1.7 1.7 0 0 0-.3-1.9l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.7 1.7 0 0 0 1.9.3H9a1.7 1.7 0 0 0 1-1.6V3a2 2 0 1 1 4 0v.1a1.7 1.7 0 0 0 1 1.6 1.7 1.7 0 0 0 1.9-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.7 1.7 0 0 0-.3 1.9V9a1.7 1.7 0 0 0 1.6 1H21a2 2 0 1 1 0 4h-.1a1.7 1.7 0 0 0-1.6 1z"/></svg>',
    arrowRight: '<svg ' + ICON_ATTRS + '><path d="M5 12h13M13 6l6 6-6 6"/></svg>',
    glow: '<svg ' + ICON_ATTRS + '><circle cx="12" cy="12" r="4" fill="currentColor" stroke="none"/><circle cx="12" cy="12" r="9" opacity="0.45"/></svg>',
    pulse: '<svg ' + ICON_ATTRS + '><path d="M3 12h4l2-7 4 14 2-7h6"/></svg>',
    frame: '<svg ' + ICON_ATTRS + '><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/></svg>',
    phone: '<svg ' + ICON_ATTRS + '><rect x="7" y="2" width="10" height="20" rx="2.2"/><path d="M11 18h2"/></svg>',
  };
  function applyStaticIcons() {
    document.querySelectorAll("[data-icon]").forEach(function (el) {
      const name = el.getAttribute("data-icon");
      if (ICON[name]) el.innerHTML = ICON[name];
    });
  }
  // Called here rather than at the end of this file. Everything below is
  // wiring that assumes a fully parsed document -- 26 top-level
  // getElementById().onclick assignments among it -- and a single one of
  // those throwing (a truncated response, an element that moved) used to
  // abort the whole init before the icons were ever drawn, leaving every
  // data-icon span blank. The tab bar is the most visible casualty, and it
  // comes back on reload, which makes it look intermittent rather than
  // structural. This script tag is at the end of <body>, so the elements
  // it fills already exist.
  applyStaticIcons();

  // Fallback only, used until the connected server's /health reply has
  // actually been read into state.minDistanceCm (renderWatchdog(), below)
  // -- e.g. before a connection exists at all, or against an older
  // deployment that hasn't been redeployed to include the field yet.
  // Every actual usage in this file reads state.minDistanceCm, not this
  // constant, so a running page stays in sync with whatever
  // config/robot.yaml's safety.min_distance_cm really is on the server
  // it's talking to (PLAN-sim-hardening.md definition of done, item 10 --
  // this used to be a second hardcoded copy that could silently drift).
  const MIN_DISTANCE_CM_FALLBACK = 20;
  // ---------- twin state (mission bookkeeping only -- no physics) ----------

  const state = {
    serverUrl: "", serverSecret: "", connected: false, lastFrame: null,
    // Phase M2. `depthUnsupported` latches on the first 404 so a server
    // that predates /depth is asked once and then left alone, rather than
    // producing one failed request per step for the rest of the session.
    lastDepth: null, depthUnsupported: false,
    // N1. The WORLD model, kept apart from every field above it because
    // those are body state and these are not. `lastMap` is the big one --
    // ~10^5 cells on a real house against a handful of numbers everywhere
    // else -- which is why `map_version` exists and why the two are
    // fetched on different clocks (see refreshWorld).
    lastPose: null, lastMap: null, worldUnsupported: false, lastMapFetchAt: 0,
    searchedRooms: new Set(),
    sightings: [], found: false, foundSighting: null,
    log: [], step: 0,
    safetyFlashUntil: 0,
    guidanceMode: "guide", // "guide" -> /guidance (steer a person); "robot" -> /navigate (what would the robot do)
    guidanceRunning: false, guidanceStarting: false, guidanceTimerId: null, guidanceTarget: null,
    guidanceInFlight: 0, guidanceSeq: 0, guidanceLastRenderedSeq: 0,
    // Which run a vision call belongs to. Bumped on every stop, captured at
    // dispatch, compared on return -- see stopGuidance() and guidanceStep().
    guidanceEpoch: 0,
    guidanceCallCount: 0, guidanceStream: null,
    guidanceMuted: false, guidanceNotVisibleStreak: 0,
    guidancePaused: false, guidanceFoundStreak: 0,
    guidancePanSpeed: 0, guidanceLastOrientation: null,
    guidanceConsecutiveSkips: 0, guidanceErrorStreak: 0,
    connecting: false,
    // Remote brain (phase B4): this page observes a mission it does not run.
    brainUrl: "", brainSecret: "", brainConnected: false, brainConnecting: false,
    brainMissionRunning: false, brainPollTimerId: null, brainLogSignature: "",
    // What the running mission is hunting. Read by recordObservation() so a
    // sighting the twin notices can be matched against the mission's target
    // -- which used to come from this page's own local loops, now deleted.
    brainTarget: null,
    // Brain liveness (see startBrainLiveness): the retry timer, and whether
    // a connection was LOST (as opposed to never made), so its return can
    // be announced.
    brainLivenessTimerId: null, brainLost: false,
    // How far a D-pad LEFT/RIGHT turns, in degrees (R0). 90 is what every
    // tap sent before the pose went continuous, so it stays the default.
    turnStepDeg: 90,
    // The robot server's /health `mode` ("sim", "teleop", "hardware") --
    // read by the tiered hint, which must not name models a sim mission
    // will never load.
    robotMode: null,
    watchdogTimerId: null,
    // Recording a Robot-view walk to the brain, for replay (S2b).
    // The robot's own id for the most recently pushed teleop frame, so a
    // recorded frame can name the id the mission will report deciding on.
    lastTeleopSeq: null,
    recordWalk: false, recordWalkName: null, recordSaved: 0, recordFailed: 0,
    // Which WALK a frame belongs to, which is not the same lifetime as
    // guidanceEpoch's run -- see recordWalkFrame(). recordOrphaned counts
    // frames dropped because their walk ended before their answer landed.
    recordEpoch: 0, recordOrphaned: 0,
    recordSeq: 0,
    // /navigate model A/B (empty string = service default, i.e. omit
    // model_id entirely -- see fetchNavigateModels()).
    navigateModelId: "", navigateModelsLoaded: false,
    // The prompt wording axis, served from the same allow-list as the models.
    navigatePromptVariant: "",
    // The vision service's own default, and whatever the connected brain
    // pins -- both only for showing which model a mission would really use.
    navigateServiceDefault: "", brainNavigateModelId: null,
    brainNavigatePromptVariant: null,
    // Which policy the Remote brain panel starts a mission under. "frontier"
    // is the free rule-based explorer; "vision" spends a model call a step
    // and is the one on the hardware path (PLAN-sim-hardening.md 2.2);
    // "tiered" runs YOLO + CLIP inside the brain process on every frame and
    // spends a call only on a trigger (PLAN-onboard-perception.md 2.4, P2).
    brainPolicy: "frontier",
    // What the connected brain says about its own perception models --
    // whether they are installed at all, and which two would load. Null
    // until a brain has been asked: "not known yet" and "not available"
    // are different answers and the hint says so differently.
    brainPerception: null,
    // Driving Robot view through a real MissionRunner mission instead of a
    // one-off /navigate call (PLAN-teleop-robot.md, Phase T3). driveViaBrain
    // is the live toggle; guidanceViaBrain is latched at Start so Stop knows
    // whether *this* running session actually started a brain mission, even
    // if the toggle or brain connection changes mid-walk.
    driveViaBrain: false, guidanceViaBrain: false,
    // Which policy a "Drive via brain" walk runs. Separate from
    // brainPolicy above because they start two different missions -- that
    // one drives the grid world, this one drives a phone on a wheeled rig
    // -- and because "frontier" is not on offer here: a photograph carries
    // no grid coordinates and TeleopRobot has no distance sensor.
    //
    // This is the only path in the app where the tiered policy sees real
    // pixels. PLAN-onboard-perception.md 4.10: the twin cannot test a
    // detector by 1.12's design, and a phone on a rig is the
    // highest-fidelity pre-hardware test available.
    drivePolicy: "vision",
    // Synced from the connected server's own /health reply
    // (renderWatchdog()) -- see MIN_DISTANCE_CM_FALLBACK above.
    minDistanceCm: MIN_DISTANCE_CM_FALLBACK,
  };

  // ---------- persisted preferences ----------
  // Everything a returning user would otherwise have to re-enter or
  // re-choose. This page is installable as a PWA, so it gets reopened cold
  // constantly; previously only the two shared secrets and the target
  // history survived a reload, which meant every launch started
  // disconnected with an empty vision URL on whichever tab happened to be
  // first. localStorage can throw outright in some private-browsing modes,
  // so every access is wrapped -- a storage failure must never take down
  // page init or a click handler.
  const PREF = {
    serverUrl: "vp_server_url",
    brainUrl: "vp_brain_url",
    serverSecret: "vp_cfg_server_secret",   // shared with prefillTesterSecrets below
    visionUrl: "vp_vision_url",
    visionSecret: "vp_cfg_secret",          // shared with prefillTesterSecrets below
    tab: "vp_active_tab",
    muted: "vp_guide_muted",
    onboarded: "vp_guide_onboarded",
    rotateHintDismissed: "vp_guide_rotate_dismissed",
    debugReadouts: "vp_debug_readouts",
    recordWalk: "vp_record_walk",
    driveViaBrain: "vp_drive_via_brain",
    drivePolicy: "vp_drive_policy",
    turnStepDeg: "vp_turn_step_deg",
    navigateModelId: "vp_navigate_model_id",
    navigatePromptVariant: "vp_navigate_prompt_variant",
    brainPolicy: "vp_brain_policy",
  };
  function prefGet(key) {
    try { return localStorage.getItem(key); } catch (e) { return null; }
  }
  function prefSet(key, value) {
    try { localStorage.setItem(key, value); } catch (e) { /* best-effort */ }
  }

  // ---------- shared UI helpers ----------

  // Values rendered here come from the vision service's JSON response, not
  // from this page -- escape them rather than concatenating them straight
  // into innerHTML.
  function escapeHtml(value) {
    return String(value == null ? "" : value)
      .replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  // A blank dark rectangle before anything has happened reads as broken
  // rather than empty, so every log/result surface gets a real "nothing
  // yet" state. Marked with data-empty so the first real entry knows to
  // clear it rather than appending underneath it.
  function setEmptyState(el, icon, text) {
    if (!el) return;
    el.innerHTML = '<div class="empty-state">' +
      '<span class="empty-state-icon">' + (ICON[icon] || "") + '</span>' +
      '<span class="empty-state-text">' + text + '</span></div>';
    el.dataset.empty = "1";
  }
  function clearEmptyState(el) {
    if (el && el.dataset.empty) { el.innerHTML = ""; delete el.dataset.empty; }
  }

  const LOG_EMPTY = ["clock", "No moves yet — start a mission to see the robot's steps."];

  // Transient confirmations for actions whose outcome would otherwise only
  // appear as rewritten text in a panel the user may not be looking at.
  const TOAST_ICON = { ok: "target", err: "warning", warn: "warning", info: "clock" };
  function showToast(message, kind) {
    const stack = document.getElementById("toast-stack");
    if (!stack) return;
    const el = document.createElement("div");
    el.className = "toast " + (kind || "info");
    el.innerHTML = '<span class="toast-icon">' + (ICON[TOAST_ICON[kind || "info"]] || "") + "</span>" +
      "<span>" + escapeHtml(message) + "</span>";
    stack.appendChild(el);
    setTimeout(function () {
      el.classList.add("leaving");
      setTimeout(function () { el.remove(); }, 220);
    }, kind === "err" || kind === "warn" ? 5200 : 3200);
  }

  // Buttons keep their label but gain a spinner and stop accepting input
  // while the request they started is in flight.
  function setButtonBusy(btn, busy, busyLabel) {
    if (!btn) return;
    if (busy) {
      if (!btn.dataset.idleHtml) btn.dataset.idleHtml = btn.innerHTML;
      btn.innerHTML = '<span class="spinner"></span>' + escapeHtml(busyLabel || "Working…");
      btn.classList.add("is-busy");
      btn.setAttribute("aria-busy", "true");
    } else {
      if (btn.dataset.idleHtml) { btn.innerHTML = btn.dataset.idleHtml; delete btn.dataset.idleHtml; }
      btn.classList.remove("is-busy");
      btn.removeAttribute("aria-busy");
    }
  }

  // ---------- robot/server.py client -- the twin's "brain" side of the ----------
  // ---------- MacBook<->Pi split, talking to the robot over HTTP just  ----------
  // ---------- like brain/agent.py's MissionAgent talks to RobotInterface ----------

  // ngrok's free tier answers a request carrying a browser User-Agent with
  // an HTML interstitial instead of proxying it -- so every fetch() from
  // this page to a tunnelled robot or brain comes back as a page, and
  // res.json() throws on markup rather than saying what happened. The
  // documented escape is this header, with any value.
  //
  // Added ONLY for ngrok hosts, on purpose. It is a custom header, so it
  // forces a CORS preflight on requests that would otherwise be simple --
  // and one of those is the /health poll behind the watchdog readout,
  // twice a second. Both servers answer the preflight with
  // access-control-max-age: 600, so the cost on a tunnel is one extra
  // round trip per ten minutes; on a LAN or localhost there is no cost at
  // all because the header is never added.
  //
  // This is a workaround for someone else's free tier, not a protocol.
  // A tunnel with its own domain (a paid plan, Cloudflare, Tailscale) or
  // B5's brain-on-the-Pi needs none of it, and nothing breaks if the
  // header goes out to a host that has never heard of it.
  const NGROK_HOST = /(^|\.)ngrok(-free)?\.(app|dev|io)$/i;

  function tunnelHeaders(url, headers) {
    try {
      if (NGROK_HOST.test(new URL(url, location.href).hostname)) {
        headers["ngrok-skip-browser-warning"] = "1";
      }
    } catch (e) { /* a URL we cannot parse is not an ngrok URL */ }
    return headers;
  }

  function authHeaders(extra) {
    const headers = Object.assign({}, extra);
    if (state.serverSecret) headers["x-app-secret"] = state.serverSecret;
    return tunnelHeaders(state.serverUrl, headers);
  }
  // A server that answered is a different problem from a server that
  // could not be reached, and only the caller knows how to say so. The
  // status is carried on the Error because both helpers otherwise reduce
  // every failure to a message string -- which is how a precise 503 ("no
  // new frame -- is the phone still capturing?") ended up presented as
  // "check the server is running, on the same network, and that CORS
  // permits this origin", three things that were all true at the time.
  function apiError(res, detail) {
    const err = new Error(detail || ("HTTP " + res.status));
    err.status = res.status;
    return err;
  }

  async function apiPost(path, body, extraHeaders) {
    const res = await fetch(state.serverUrl + path, {
      method: "POST",
      headers: authHeaders(Object.assign({ "Content-Type": "application/json" },
                                         extraHeaders || {})),
      body: JSON.stringify(body || {}),
    });
    if (!res.ok) throw apiError(res, (await res.json().catch(() => ({}))).detail);
    return res.json();
  }
  async function apiGet(path) {
    const res = await fetch(state.serverUrl + path, { headers: authHeaders() });
    if (!res.ok) throw apiError(res, (await res.json().catch(() => ({}))).detail);
    return res.json();
  }

  // ---------- shared vision-service URL helper ----------
  // The "Cloud endpoint settings" field is meant to hold the vision
  // service's BASE URL, but it's reasonable for someone to paste in one of
  // its specific route URLs instead (e.g. copied from a curl example, or
  // left over from testing a different feature). Strip any of the three
  // known route suffixes before appending the one this call actually
  // needs, so all three features tolerate whatever shape was pasted in,
  // not just "bare URL" or "URL ending in /analyze".
  const VISION_SERVICE_ROUTES = ["/analyze", "/describe", "/navigate", "/guidance"];
  // Which host will a vision call actually go to, and is it this page's?
  //
  // A saved endpoint outlives the page that saved it: open a second
  // deployment on a phone that has used the first, and localStorage
  // quietly points the new page at the old service. The only symptom is a
  // 401, which reads as a bad secret rather than a call to the wrong
  // place -- an hour of debugging a credential that was never wrong.
  //
  // Deliberately a *notice*, never an error. A mismatch is legitimate and
  // supported: PLAN-ar-guidance.md's hosting section expects tunnels and
  // path-prefixed proxies, where the vision service properly lives
  // somewhere else. This says what will be called; it does not claim
  // anything is broken.
  function fieldHost(id) {
    const raw = (document.getElementById(id) || {}).value;
    if (!raw || !raw.trim()) return null;
    try { return new URL(raw.trim()).host; } catch (e) { return null; }
  }

  // 3.64 B2: the values init derives FROM this page's own host (the brain
  // on :8001, vision on the bare hostname). They are this deployment by
  // construction, whatever their port; only a value someone saved or
  // typed can point at another one.
  const derivedEndpoints = {};

  function hostDiffersFromPage(id) {
    const el = document.getElementById(id);
    if (el && derivedEndpoints[id] && el.value.trim() === derivedEndpoints[id]) return false;
    const host = fieldHost(id);
    // file:// has no meaningful origin to compare against, and localhost
    // dev routinely splits the twin (:8000) from the service (:8080).
    if (!host || location.protocol === "file:") return false;
    if (location.hostname === "localhost" || location.hostname === "127.0.0.1") return false;
    return host !== location.host;
  }

  function visionHost() { return fieldHost("cfg-url"); }
  function visionHostDiffersFromPage() { return hostDiffersFromPage("cfg-url"); }

  // All three endpoints, not just the vision one. The first version covered
  // vision because that was the 401 in front of me; the brain URL then sent
  // a walk recorded on Lab to production's volume, and the robot URL can do
  // the same for the D-pad. One saved endpoint following someone between
  // deployments is the shape -- which field it happens to be is incidental.
  const ENDPOINT_FIELDS = [
    {id: "cfg-url", note: "cfg-url-mismatch", what: "Vision calls"},
    {id: "cfg-server-url", note: "cfg-server-url-mismatch", what: "Robot commands and frames"},
    {id: "cfg-brain-url", note: "cfg-brain-url-mismatch",
     what: "Missions and recorded walks"},
  ];

  function renderEndpointNote() {
    ENDPOINT_FIELDS.forEach(function (f) {
      const el = document.getElementById(f.note);
      if (!el) return;
      if (!hostDiffersFromPage(f.id)) { el.hidden = true; return; }
      const host = escapeHtml(fieldHost(f.id));
      el.innerHTML =
        escapeHtml(f.what) + " go to <b>" + host + "</b>, not <b>"
        + escapeHtml(location.host) + "</b>, which served this page. That is "
        + "fine for a tunnel or a proxy \u2014 but if these are different "
        + "deployments, this points at the other one, and the secret here has "
        + "to be the one " + host + " expects.";
      el.hidden = false;
    });
  }

  function deriveServiceUrl(rawUrl, targetRoute) {
    let trimmed = rawUrl.trim().replace(/\/$/, "");
    for (const route of VISION_SERVICE_ROUTES) {
      const re = new RegExp(route.replace("/", "\\/") + "$", "i");
      if (re.test(trimmed)) {
        trimmed = trimmed.replace(re, "");
        break;
      }
    }
    return trimmed + targetRoute;
  }

  // Phase M4. Every command names who is issuing it, so the robot server
  // can apply the decided priority order (AGENT-HARNESS.md) instead of
  // letting the last writer win. The page has two drivers and they rank
  // differently: a person tapping the pad outranks the JS loop, and both
  // are only meaningful because the server now knows the difference.
  const DRIVER_DPAD = "twin-dpad";
  const DRIVER_LOCAL_BRAIN = "twin-local-brain";

  // `extra` carries the per-action parameters /action accepts beyond the
  // verb -- today only `angle`, for the D-pad's turn step (R0). Omitted
  // keys take the server's defaults, so a caller that passes nothing sends
  // exactly what this function always sent.
  async function sendAction(action, driver, extra) {
    const headers = { "x-driver": driver || DRIVER_DPAD };
    if (action === "STOP") return apiPost("/stop", {}, headers);
    return apiPost("/action", Object.assign({ action: action }, extra || {}), headers);
  }
  async function fetchDistance() {
    const data = await apiGet("/distance");
    return data.distance_cm;
  }
  async function fetchFrame() {
    const raw = await apiGet("/frame");
    // No facing / free-cells / doorway / position any more: the server
    // stopped sending them with the cell layer (PLAN-ros-alignment.md). The
    // pose is GET /world/pose and clearance is GET /depth, both drawn below
    // from the routes a real robot will serve. `objectsVisible` is the sim's
    // stand-in for a detector, and defaults to empty for a backend with none.
    const frame = {
      room: raw.room,
      objectsVisible: raw.objects_visible || [],
      // Phase S2: the sim renders its own camera now, so a frame arrives
      // with pixels. Carried through rather than dropped here -- this
      // remapping is the only place the server's reply becomes the page's
      // frame object, so a key missed here is a key the whole UI lacks.
      imageBase64: raw.image_base64, mediaType: raw.media_type,
    };
    state.lastFrame = frame;
    // Phase M2. Fired alongside the frame rather than awaited with it: the
    // strip is telemetry, and a robot whose depth route is slow or missing
    // must not slow down or break the picture. Its own failure is silent
    // for the same reason -- renderDepth() says "not reported" and the rest
    // of the page carries on.
    refreshDepth();
    refreshOdometry();
    refreshWorld();
    return frame;
  }

  // ---------- phase M2: the depth grid ----------
  // A separate route because it is a separate sensor (robot/server.py's
  // /depth). Folding it into /frame would mean a camera that has wedged
  // takes the clearance reading down with it, which is the coupling M9
  // exists to prevent.
  async function refreshDepth() {
    if (state.depthUnsupported) return;
    try {
      state.lastDepth = await apiGet("/depth");
    } catch (e) {
      if (e.status === 404) {
        // A server older than M2. A real state, not an error: the stacks
        // are redeployed one at a time.
        state.depthUnsupported = true;
        state.lastDepth = null;
      }
      // Any other failure leaves the previous grid up rather than blanking
      // the strip on one dropped request.
    }
    renderDepth();
  }

  // ---------- phase B: odometry ----------
  // Its own route for the same reason /depth is one: a third sensor, and a
  // wedged camera must not take it down with it. A 404 means the server
  // predates the route, which is a real state while stacks are redeployed
  // one at a time -- and is reported as "no route", never as "no motion".
  async function refreshOdometry() {
    if (state.odometryUnsupported) return;
    try {
      state.lastOdometry = await apiGet("/odometry");
    } catch (e) {
      if (e.status === 404) {
        state.odometryUnsupported = true;
        state.lastOdometry = null;
      }
    }
    renderOdometry();
  }

  function renderOdometry() {
    var el = document.getElementById("odometry-readout");
    if (!el) return;
    var o = state.lastOdometry;
    if (state.odometryUnsupported) {
      el.textContent = "odometry: not reported by this server";
      return;
    }
    if (!o) { el.textContent = "odometry: not connected"; return; }
    if (!o.usable) {
      // The honest no-op, shown as one. This is what a teleop rig reads,
      // and it is the state Phase C's distance rule falls back from.
      el.innerHTML = "odometry: <span class=\"src\">no encoders on this backend"
        + " \u2014 pacing falls back to frame count</span>";
      return;
    }
    el.innerHTML = "odometry: <strong>" + o.distance_m.toFixed(2)
      + "m</strong> travelled \u00b7 heading " + Math.round(o.heading_deg)
      + "\u00b0 <span class=\"src\">(path length, not displacement)</span>";
  }

  // Range zones ramp from alert (close) to safe (far), so the strip reads
  // the same way the map's safety collar does. The scale tops out at
  // DEPTH_FAR_CM rather than at the grid's own maximum: a strip that
  // rescaled itself every frame would make a wall look further away as you
  // drove at it, which is exactly backwards.
  var DEPTH_FAR_CM = 200;

  function depthZoneStyle(zone) {
    if (zone.status === "unusable") return { cls: "depth-zone unusable", css: "" };
    if (zone.status === "no_target") {
      return { cls: "depth-zone", css: "background: var(--accent-safe); opacity: 0.35;" };
    }
    var t = Math.max(0, Math.min(1, (zone.distance_cm || 0) / DEPTH_FAR_CM));
    // Close -> alert, far -> safe. Both are read from CSS so the strip
    // follows the theme, unlike sim/renderer.py's frame colours, which
    // deliberately no longer do (they are the model's input; this is not).
    var color = t < 0.5 ? "var(--accent-alert)" : "var(--accent-safe)";
    return { cls: "depth-zone", css: "background: " + color + "; opacity: " +
             (0.35 + 0.65 * (1 - Math.abs(t - 0.5) * 2)).toFixed(2) + ";" };
  }

  // ---------- phase N1: the world model ----------
  // The map and the pose on it. Separate from everything above because it
  // is a different KIND of state: /distance, /depth and /odometry are all
  // egocentric -- how far from ME, what is ahead of ME, how far have I
  // driven -- and these two are allocentric. `world/interface.py` holds
  // the line; this is the consumer end of it.
  //
  // **Two clocks, on purpose.** The pose is a handful of numbers and is
  // fetched every frame; the map is the whole house and is fetched at most
  // once a second. A truly version-gated FETCH needs a cheap route that
  // returns `map_version` alone -- N4's job, when a real house makes the
  // payload matter. In the sim it is 130 cells, so a timer is honest and
  // enough, and saying so beats pretending the version field is already
  // doing work it is not.
  var MAP_FETCH_INTERVAL_MS = 1000;

  async function refreshWorld() {
    if (state.worldUnsupported) return;
    try {
      state.lastPose = await apiGet("/world/pose");
      // R5: estimate against truth. Sim-only, and a server older than R5
      // has no such route -- that is not an error, just nothing to draw.
      if (!state.worldErrorUnsupported) {
        try {
          state.lastWorldError = await apiGet("/world/error");
        } catch (err) {
          if (err.status === 404) state.worldErrorUnsupported = true;
          state.lastWorldError = null;
        }
      }
      // R6: a nav2 goal and its planned route, only where a world can plan
      // (a SLAM map). Anything else answers 501, which just means "no goals
      // here" -- the map is then not tappable.
      if (isSlamMap(state.lastMap) && !state.goalsUnsupported) {
        try {
          state.lastGoal = await apiGet("/world/goal");
        } catch (err) {
          if (err.status === 404 || err.status === 501) state.goalsUnsupported = true;
          state.lastGoal = null;
        }
      }
      var now = Date.now();
      if (now - state.lastMapFetchAt >= MAP_FETCH_INTERVAL_MS) {
        state.lastMapFetchAt = now;
        state.lastMap = await apiGet("/world/map");
      }
    } catch (e) {
      if (e.status === 404) {
        // A server older than N1. A real state, not an error.
        state.worldUnsupported = true;
        state.lastPose = null;
        state.lastMap = null;
      }
      // Anything else leaves the previous map up rather than blanking the
      // canvas on one dropped request -- same rule as the depth strip.
    }
    renderMap();
  }

  function isSlamMap(grid) {
    return !!(grid && grid.usable && typeof grid.map_id === "string" &&
              grid.map_id.indexOf("slam-") === 0);
  }

  // R6: nav2's goal and planned route, in the house frame the map is drawn in.
  function drawGoal(ctx, grid, scale) {
    var g = state.lastGoal;
    if (!g || !g.goal) return;
    var css = getComputedStyle(document.documentElement);
    var colour = (css.getPropertyValue("--map-goal") || "#39d98a").trim();
    var toPx = function (x, y) {
      return [((x - grid.origin_x_m) / grid.resolution_m) * scale,
              ((y - grid.origin_y_m) / grid.resolution_m) * scale];
    };
    if (g.plan && g.plan.length > 1 && (g.goal.state === "active" || g.goal.state === "pending")) {
      ctx.beginPath();
      g.plan.forEach(function (p, k) {
        var q = toPx(p[0], p[1]);
        if (k === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]);
      });
      ctx.strokeStyle = colour;
      ctx.lineWidth = Math.max(2, scale * 0.5);
      ctx.stroke();
    }
    var c = toPx(g.goal.x_m, g.goal.y_m);
    ctx.beginPath();
    ctx.arc(c[0], c[1], Math.max(6, scale * 2), 0, 2 * Math.PI);
    ctx.strokeStyle = colour;
    ctx.lineWidth = Math.max(2, scale * 0.6);
    ctx.stroke();
  }

  function goalText(grid) {
    if (!isSlamMap(grid) || state.goalsUnsupported) return "";
    var g = state.lastGoal && state.lastGoal.goal;
    var words = {pending: "planning", active: "on its way", succeeded: "arrived",
                 aborted: "could not get there", canceled: "cancelled", rejected: "refused"};
    var what = g ? ("goal: " + (words[g.state] || g.state) + " \u00b7 ") : "";
    // SLAM draws only what the lidar has swept, and adds to it as the robot
    // MOVES -- a map that is mostly unknown is not broken, it is unexplored.
    var seen = 0;
    for (var i = 0; i < grid.cells.length; i++) if (grid.cells[i] !== CELL_UNKNOWN) seen++;
    var hint = seen < 0.25 * grid.cells.length
      ? "drive with the D-pad to map more, then tap the map to send the robot there"
      : "tap the map to send the robot there";
    return '<br><span class="goal-line">' + what + hint + "</span>";
  }

  // Tap the SLAM map: send the robot there (nav2, through the robot server).
  function onMapTap(ev) {
    var grid = state.lastMap;
    if (!isSlamMap(grid) || state.goalsUnsupported || !state.connected) return;
    var canvas = ev.currentTarget;
    var rect = canvas.getBoundingClientRect();
    var px = (ev.clientX - rect.left) * (canvas.width / rect.width);
    var py = (ev.clientY - rect.top) * (canvas.height / rect.height);
    var scale = canvas.width / grid.width;
    var x = grid.origin_x_m + (px / scale) * grid.resolution_m;
    var y = grid.origin_y_m + (py / scale) * grid.resolution_m;
    // A person's tap (handoff 3b): named as the D-pad is, so it outranks a
    // running brain mission the way a D-pad press does.
    apiPost("/world/goal", {x_m: x, y_m: y}, {"x-driver": "twin-dpad"}).then(function (r) {
      if (r && r.accepted === false) {
        showToast("Could not send the goal: " + (r.reason || r.error || "refused"), "error");
      } else {
        showToast("Sending the robot there", "info");
      }
    }).catch(function (e) {
      showToast("Could not send the goal (" + (e.message || e) + ")", "error");
    });
  }

  // Cell states, matching world/interface.py's CELL_* constants. Named
  // here rather than inlined because -1/0/1 at a call site is exactly the
  // sort of thing that reads as a boolean at a glance.
  var CELL_UNKNOWN = -1, CELL_FREE = 0, CELL_OCCUPIED = 1;

  function renderMap() {
    var canvas = document.getElementById("world-map");
    var readout = document.getElementById("map-readout");
    if (!canvas || !readout) return;

    var ctx = canvas.getContext("2d");
    var grid = state.lastMap;

    if (state.worldUnsupported) {
      canvas.width = 0; canvas.height = 0;
      readout.textContent = "map: not reported by this server";
      return;
    }
    if (!grid || !grid.usable) {
      canvas.width = 0; canvas.height = 0;
      // Three different nothings, said as three different things. "No
      // mapper" is a fact a planner works with (1.5's bootstrap rule);
      // "not connected" is not, and neither is an empty canvas.
      readout.textContent = !state.connected ? "map: not connected"
        : (grid ? "map: no mapper on this backend" : "map: not reported yet");
      return;
    }

    // One canvas pixel per cell, scaled up by CSS. `image-rendering:
    // pixelated` keeps a cell a cell -- smoothing would interpolate
    // between "seen floor" and "never seen" and draw a confidence the
    // map does not have.
    var SCALE = Math.max(4, Math.min(16, Math.floor(280 / grid.width)));
    canvas.width = grid.width * SCALE;
    canvas.height = grid.height * SCALE;

    var css = getComputedStyle(document.documentElement);
    var COLOR = {};
    // The map's OWN palette, never --wall/--floor: those are UI chrome,
    // and sim/renderer.py already paid for borrowing them once (every
    // sim frame came back to the model as "very dark and unclear").
    COLOR[CELL_UNKNOWN] = (css.getPropertyValue("--map-unknown") || "#171B22").trim();
    COLOR[CELL_FREE] = (css.getPropertyValue("--map-free") || "#35505F").trim();
    COLOR[CELL_OCCUPIED] = (css.getPropertyValue("--map-wall") || "#8FA0B8").trim();

    for (var y = 0; y < grid.height; y++) {
      for (var x = 0; x < grid.width; x++) {
        var cell = grid.cells[y * grid.width + x];
        ctx.fillStyle = COLOR[cell] || COLOR[CELL_UNKNOWN];
        ctx.fillRect(x * SCALE, y * SCALE, SCALE, SCALE);
      }
    }

    // R5: when the map is SLAM's, draw the TRUTH as an outlined ghost
    // under the estimate. The gap between the two is the error; on a sim
    // map the two are the same number and nothing extra is drawn.
    var werr = state.lastWorldError;
    var slam = typeof grid.map_id === "string" && grid.map_id.indexOf("slam-") === 0;
    if (slam && werr && werr.usable && werr.truth) {
      drawPose(ctx, grid, SCALE, werr.truth, "ghost");
    }
    drawGoal(ctx, grid, SCALE);
    drawPose(ctx, grid, SCALE, state.lastPose, "estimate");
    canvas.classList.toggle("tappable", isSlamMap(grid) && !state.goalsUnsupported);

    var seen = 0;
    for (var i = 0; i < grid.cells.length; i++) {
      if (grid.cells[i] !== CELL_UNKNOWN) seen++;
    }
    var pct = Math.round((100 * seen) / grid.cells.length);
    readout.innerHTML = "map: " + seen + "/" + grid.cells.length + " cells seen (" +
      pct + "%) \u00b7 " + Math.round(grid.resolution_m * 100) + "cm cells \u00b7 " +
      '<span class="src">' + grid.map_id + " v" + grid.map_version + "</span>" +
      slamErrorText(slam ? werr : null) + goalText(grid);
  }

  // "SLAM error 4.2 cm / 1.1° · odometry alone 45 cm / 39°" -- the one
  // number PLAN-ros-alignment.md section 2 says the twin can show and a real
  // room cannot. Nothing at all when there is no truth to compare against.
  function slamErrorText(werr) {
    if (!werr || !werr.usable || werr.position_error_m == null) return "";
    var line = '<br><span class="slam-error">SLAM error ' +
      (werr.position_error_m * 100).toFixed(1) + " cm / " +
      Math.abs(werr.heading_error_deg).toFixed(1) + "\u00b0";
    if (werr.odom_position_error_m != null) {
      line += " \u00b7 odometry alone " + (werr.odom_position_error_m * 100).toFixed(0) +
        " cm / " + Math.abs(werr.odom_heading_error_deg).toFixed(0) + "\u00b0";
    }
    return line + " \u00b7 outline = truth</span>";
  }

  function drawPose(ctx, grid, scale, pose, style) {
    // A pose that is not usable draws NOTHING. Not a dot at the origin:
    // (0, 0) is a perfectly valid pose and a map showing the robot
    // confidently in the corner of a house it cannot localise in is worse
    // than a map showing no robot at all.
    if (!pose || !pose.usable) return;
    // Coordinates from another map. The ghost is the TRUTH, which is in the
    // house frame every sim map shares and carries no map_id of its own.
    if (style !== "ghost" && pose.map_id !== grid.map_id) return;

    // Metres -> cells -> canvas pixels, via the map's own origin and
    // resolution. Never assume the grid starts at (0,0) in metres: a
    // mapper extends its grid westward when it finds a room there.
    var cx = ((pose.x_m - grid.origin_x_m) / grid.resolution_m) * scale;
    var cy = ((pose.y_m - grid.origin_y_m) / grid.resolution_m) * scale;

    // heading_deg is a COMPASS bearing -- clockwise, 0 = north = -y here.
    // Converting it wrong looks right at 0 and 180, which is why this
    // conversion is written once and commented rather than inlined twice.
    var rad = (pose.heading_deg - 90) * Math.PI / 180;
    var r = Math.max(3, scale * 0.6);

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(rad);
    ctx.beginPath();
    ctx.moveTo(r, 0);
    ctx.lineTo(-r * 0.7, r * 0.6);
    ctx.lineTo(-r * 0.7, -r * 0.6);
    ctx.closePath();
    var css = getComputedStyle(document.documentElement);
    if (style === "ghost") {
      ctx.lineWidth = Math.max(1.5, scale * 0.15);
      ctx.strokeStyle = (css.getPropertyValue("--map-truth") || "#f5f7fa").trim();
      ctx.stroke();
    } else {
      ctx.fillStyle = (css.getPropertyValue("--accent") || "#4da3ff").trim();
      ctx.fill();
    }
    ctx.restore();
  }

  function renderDepth() {
    var strip = document.getElementById("depth-strip");
    var readout = document.getElementById("depth-readout");
    if (!strip || !readout) return;

    var grid = state.lastDepth;
    if (!grid || !grid.zones || !grid.zones.length) {
      strip.innerHTML = "";
      readout.textContent = state.depthUnsupported
        ? "depth: not reported by this server"
        : (state.connected ? "depth: no zones reported" : "depth: not connected");
      return;
    }

    // Phase M3: `path` is the safety layer's own reduction, computed
    // server-side and rendered here. A pre-M3 server sends no `path`, in
    // which case no zone is marked and the readout falls back to the
    // nearest zone anywhere -- which is a description, not the veto.
    var path = grid.path || null;
    var isPath = {};
    if (path && path.indices) path.indices.forEach(function (i) { isPath[i] = true; });
    strip.classList.toggle("blocked", !!(path && path.blocked));

    strip.innerHTML = grid.zones.map(function (z, i) {
      var s = depthZoneStyle(z);
      var title = z.status === "range" ? Math.round(z.distance_cm) + "cm" : z.status;
      if (isPath[i]) title += " (path)";
      return '<div class="' + s.cls + (isPath[i] ? " path" : "") +
             '" style="' + s.css + '" title="' + title + '"></div>';
    }).join("");

    var measured = grid.zones.filter(function (z) { return z.status === "range"; });
    var unusable = grid.zones.filter(function (z) { return z.status === "unusable"; }).length;
    var shape = grid.rows + "\u00d7" + grid.cols;

    if (!measured.length && !path) {
      // Says which kind of nothing this is. "No depth sensor" and "the
      // sensor is blind right now" are the distinction M3 is built on, and
      // a strip that showed one grey bar for both would erase it.
      readout.textContent = unusable === grid.zones.length
        ? "depth: no sensor (" + grid.zones.length + " zones unusable)"
        : "depth: nothing within range";
      return;
    }

    if (!path) {
      var nearest = Math.min.apply(null, measured.map(function (z) { return z.distance_cm; }));
      readout.textContent = "depth: " + shape + ", nearest " + Math.round(nearest) + "cm" +
        (unusable ? ", " + unusable + " unusable" : "");
      return;
    }

    // What the veto reads, and where it got it. The source matters as much
    // as the number: "the grid answered" and "every path zone was blind so
    // this is the old single beam" are the same centimetres and very
    // different situations.
    var SOURCE_LABEL = {
      depth_grid: "path zones",
      depth_grid_no_target: "path zones",
      distance_sensor: "fallback: single beam",
    };
    var body;
    if (path.clearance_cm === null || path.clearance_cm === undefined) {
      body = "path clear beyond range";
    } else {
      body = "path " + Math.round(path.clearance_cm) + "cm";
      if (path.blocked) {
        body = '<span class="veto">' + body + " \u2014 FORWARD vetoed</span>";
      }
    }
    readout.innerHTML = "depth: " + shape + ", " + body +
      ' <span class="src">(' + (SOURCE_LABEL[path.source] || path.source) +
      (unusable ? ", " + unusable + " unusable" : "") + ")</span>";
  }



  // ---------- action execution -- talks to robot/server.py for movement,   ----------
  // ---------- sensing, and safety; the twin no longer computes any of this ----------

  function logEntry(action, executed, extra, reason) {
    state.step += 1;
    const room = state.lastFrame ? state.lastFrame.room : "unknown";
    state.log.push({ step: state.step, action: action, executed: executed, room: room,
                     extra: extra || "", reason: reason || "" });
    renderLog();
  }

  function recordObservation(frame) {
    if (frame.room !== "unknown") {
      state.searchedRooms.add(frame.room);
    }
    // The target a local loop was hunting used to be read from this page's
    // own state. The only mission now is the brain service's, so the twin
    // records what it SAW and lets the mission decide what "found" means.
    const activeTarget = state.brainTarget;
    frame.objectsVisible.forEach(function (obj) {
      state.sightings.push({ step: state.step, object: obj, room: frame.room });
      if (activeTarget && !state.found && obj.toLowerCase().indexOf(activeTarget) !== -1) {
        state.found = true;
        state.foundSighting = { object: obj, room: frame.room, step: state.step };
      }
    });
  }

  // Used by both the manual D-pad and the autonomous loop -- sends one
  // action to the server, logs the (server-authoritative) executed/veto
  // result, then refreshes the frame for rendering. Mirrors
  // brain/agent.py's ConstrainedAgent.step(): execute, then observe.
  async function commitAction(action, driver, extra) {
    const resp = await sendAction(action, driver, extra);
    const executed = resp.executed !== false;
    if (!executed) state.safetyFlashUntil = Date.now() + 350;
    // Phase M4: the reason is what a person can act on. "SAFETY VETO" was
    // the only thing this ever said, so a move refused because someone
    // else had taken the robot looked identical to one refused for being
    // about to hit a wall -- two situations with opposite responses.
    // A turn names its angle in the log: once a turn can be 15 degrees, a
    // bare "LEFT" no longer says what happened.
    const label = extra && extra.angle ? action + " " + extra.angle + "\u00B0" : action;
    logEntry(label, executed, executed ? "" : refusalText(resp), resp.reason);

    const frame = await fetchFrame();
    recordObservation(frame);
    render(frame);
    return { executed: executed, frame: frame };
  }

  const REFUSAL_LABEL = {
    safety_distance: "SAFETY VETO",
    preempted: "PREEMPTED",
    watchdog: "WATCHDOG",
  };

  function refusalText(resp) {
    const label = REFUSAL_LABEL[resp.reason];
    if (!label) return resp.detail || "SAFETY VETO";
    return resp.detail ? label + " \u2014 " + resp.detail : label;
  }

  async function manualAction(action) {
    if (!state.connected) return;
    const extra = (action === "LEFT" || action === "RIGHT")
      ? { angle: state.turnStepDeg } : undefined;
    await commitAction(action, DRIVER_DPAD, extra);
  }

  // ---------- R0: the D-pad's turn step ----------
  // The pose has been continuous since R0 (PLAN-ros-alignment.md), and the
  // server has always taken an `angle` -- it just rounded it to a quarter
  // turn until then. This is the control that lets a phone reach a heading
  // that is not a compass point, which is R0's own "done when".
  const TURN_STEPS_DEG = [15, 45, 90];

  function setTurnStep(deg) {
    if (TURN_STEPS_DEG.indexOf(deg) === -1) deg = 90;
    state.turnStepDeg = deg;
    document.querySelectorAll(".turn-step-btn").forEach(function (btn) {
      const on = Number(btn.dataset.turnDeg) === deg;
      btn.classList.toggle("active-mode", on);
      btn.setAttribute("aria-pressed", on ? "true" : "false");
    });
    prefSet(PREF.turnStepDeg, String(deg));
  }

  // ---------- connection ----------

  function setControlsEnabled(enabled) {
    ["btn-forward", "btn-reverse", "btn-left", "btn-right", "btn-stop",
     "btn-look-left", "btn-look-right", "btn-look-center",
    ].forEach(function (id) {
      document.getElementById(id).disabled = !enabled;
    });
  }

  // The header status answers "can I use what I'm looking at?", which is a
  // different question on each tab. Guide and Camera never touch
  // robot/server.py at all -- they talk to the vision service -- so showing
  // them a red "Not connected" about the robot was both irrelevant and
  // alarming, on the two tabs meant for people rather than for developing
  // the robot. Sim is the only tab that needs the robot connection.
  function renderHeaderConnStatus() {
    const dot = document.getElementById("header-conn-dot");
    const text = document.getElementById("header-conn-text");
    const tab = document.body.dataset.tab || "guide";

    // Sim's own panel already reports both connections in full.
    dot.classList.toggle("hidden", tab === "settings");

    let label, connected, connecting = false, needsSetup = false, actionable = false;
    if (tab === "sim") {
      connected = state.connected;
      connecting = state.connecting && !state.connected;
      label = connected ? "Connected" : connecting ? "Connecting\u2026" : "Not connected";
      actionable = !connected && !connecting;
    } else {
      // Guide and Camera need one thing: somewhere to send the image.
      const hasVisionUrl = !!document.getElementById("cfg-url").value.trim();
      connected = hasVisionUrl;
      needsSetup = !hasVisionUrl;
      label = hasVisionUrl ? "Ready" : "Setup needed";
      actionable = needsSetup;
    }

    dot.classList.toggle("connected", connected);
    dot.classList.toggle("connecting", connecting);
    dot.classList.toggle("needs-setup", needsSetup);
    dot.classList.toggle("actionable", actionable);
    text.textContent = label;
    dot.setAttribute("aria-label", actionable ? label + " \u2014 open settings" : label);

    const hint = document.getElementById("drive-connect-hint");
    if (hint) hint.style.display = state.connected ? "none" : "";
  }

  // Tapping the status when something is missing goes where it can be fixed.
  document.getElementById("header-conn-dot").onclick = function () {
    if (this.classList.contains("actionable")) switchTab("settings");
  };

  // `silent` marks the automatic attempt made at page load (see
  // autoConnectOnLoad below) rather than a deliberate tap on Connect. The
  // difference is only in how failure reads: an attempt the user didn't
  // ask for shouldn't greet them with a paragraph of CORS troubleshooting
  // on a tab they aren't looking at.
  async function connect(opts) {
    const silent = !!(opts && opts.silent);
    const url = document.getElementById("cfg-server-url").value.trim().replace(/\/$/, "");
    const statusEl = document.getElementById("connection-status");
    const btn = document.getElementById("btn-connect");
    if (!url) {
      if (!silent) setConnStatus(statusEl, "err", "No URL yet", "Enter the robot server's address above, then tap Connect.");
      return;
    }
    state.serverUrl = url;
    // Connecting somewhere else clears the "this server has no /depth"
    // latch -- otherwise one pre-M2 server would silence the strip for
    // every server connected to afterwards in the same session.
    state.depthUnsupported = false;
    state.lastDepth = null;
    // Same latch, same reason (N1): one server with no /world routes must
    // not silence the map for every server connected to afterwards.
    state.worldUnsupported = false;
    state.lastPose = null;
    state.lastMap = null;
    state.lastMapFetchAt = 0;
    state.serverSecret = document.getElementById("cfg-server-secret").value.trim();
    state.connecting = true;
    setButtonBusy(btn, true, "Connecting\u2026");
    setConnStatus(statusEl, "busy", silent ? "Reconnecting\u2026" : "Connecting\u2026", url);
    renderHeaderConnStatus();
    try {
      const frame = await fetchFrame();
      state.connected = true;
      setControlsEnabled(true);
      startWatchdogPolling();
      setConnStatus(statusEl, "ok", "Connected", url);
      prefSet(PREF.serverUrl, url);
      recordObservation(frame);
      render(frame);
      if (!silent) showToast("Connected to the robot server.", "ok");
    } catch (e) {
      state.connected = false;
      setControlsEnabled(false);
      const failure = connectFailure(url, e, silent);
      setConnStatus(statusEl, failure.state, failure.label, failure.detail);
      if (!silent && failure.toast) showToast(failure.toast, "err");
    } finally {
      state.connecting = false;
      setButtonBusy(btn, false);
      renderHeaderConnStatus();
    }
  }

  // ---------- connection status ----------
  // These two readouts are how you find out, standing in the middle of a
  // room with a phone, whether the thing you are about to drive is actually
  // reachable. They used to be dim grey body text that read identically
  // whether the service was live or dead, with a long URL burying the one
  // word that mattered. State goes in the label, colour and dot; the URL is
  // demoted to a detail line that is allowed to wrap or truncate.
  //
  // Writes through the label/detail spans rather than the container's
  // textContent, which would blow away the dot and the structure.
  const CONN_STATE_CLASS = {ok: "is-ok", err: "is-err", busy: "is-busy", idle: ""};

  function setConnStatus(el, stateName, label, detail) {
    if (!el) return;
    el.classList.remove("is-ok", "is-err", "is-busy");
    const cls = CONN_STATE_CLASS[stateName];
    if (cls) el.classList.add(cls);
    const labelEl = el.querySelector(".conn-status-label");
    const detailEl = el.querySelector(".conn-status-detail");
    if (labelEl) labelEl.textContent = label;
    if (detailEl) detailEl.textContent = detail || "";
  }

  // ---------- rendering ----------

  // Both canvases size themselves to the column they're in rather than to
  // fixed pixel constants. The old fixed 26px cell made the map 338 CSS px
  // wide, which does not fit the ~322px column available inside .panel on
  // a 390-393px iPhone (the most common size there is) -- it overflowed and
  // gave the whole page a horizontal scroll. Sizing is driven by a
  // ResizeObserver rather than a one-shot measurement at load because both
  // canvases live inside .tab-page elements that start display:none, where
  // clientWidth is 0; the observer fires with a real width the moment the
  // tab is first shown, and again on rotate.

  const fpvCanvas = document.getElementById("fpv-canvas");
  const fpvCtx = fpvCanvas.getContext("2d");

  let FPV_W = 320, FPV_H = 200;
  const FPV_ASPECT = 200 / 320;

  // clientWidth includes the element's own padding, so subtract it to get
  // the real content box the canvas has to fit inside.
  function availableWidth(el, padding) {
    return el.clientWidth - padding * 2;
  }

  // Returns true when the size actually changed, so callers know whether a
  // redraw is needed (the observer fires on every layout pass, not just
  // real resizes).
  function sizeFpvCanvas() {
    const avail = availableWidth(document.querySelector(".fpv-wrap"), 0);
    if (avail <= 0) return false;
    const next = Math.max(200, Math.min(480, Math.floor(avail)));
    if (next === FPV_W && fpvCanvas.width) return false;
    FPV_W = next;
    FPV_H = Math.round(FPV_W * FPV_ASPECT);
    const dpr = window.devicePixelRatio || 1;
    fpvCanvas.width = FPV_W * dpr;
    fpvCanvas.height = FPV_H * dpr;
    fpvCanvas.style.width = FPV_W + "px";
    fpvCanvas.style.height = FPV_H + "px";
    fpvCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return true;
  }

  function getCss(varName) {
    return getComputedStyle(document.documentElement).getPropertyValue(varName).trim();
  }
  function normalizeAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  // ---------- phase S2: the camera moved into Python ----------
  // sim/renderer.py renders the grid world, so a frame from a connected
  // server arrives with its own pixels and this page has nothing left to
  // simulate. S2 said the JS raycaster could be deleted "when it reads
  // server everywhere"; the ROS alignment did that, and R0 made keeping it
  // unsafe rather than merely redundant -- it could only ever draw a
  // cardinal heading from a cell centre. The readout is still here, and
  // still earns its place: it now distinguishes real pixels from NO pixels,
  // which is a state a real camera can also be in.

  function setFrameSource(kind) {
    const el = document.getElementById("fpv-source");
    if (!el) return;
    const label = kind === "server"
      ? '<span class="server">server (sim/renderer.py)</span>'
      : '<span class="local">none</span>';
    el.innerHTML = "frame source: " + label;
  }

  function drawFPV(frame) {
    frame = frame || state.lastFrame;
    if (!frame) return;
    if (!frame.imageBase64) {
      // There used to be a JS raycaster here for a server predating S2.
      // It read a CELL and a CARDINAL heading off the frame, so after R0's
      // continuous pose it drew a view the robot was not facing -- and at
      // one call site it fed that picture to the model as if it were the
      // camera. Say "no pixels" instead: a wrong picture is worse than none.
      renderFpvPlaceholder("This server sends no camera frames");
      setFrameSource("none");
      return;
    }
    const img = new Image();
    img.onload = function () {
      // The render is a fixed 320x200 (sim/renderer.py's default, which is
      // what makes the golden test reproducible) and the canvas is
      // whatever the viewport allows, so this is usually an upscale.
      // Smoothing off keeps the hard raycaster edges the local renderer
      // draws -- the phase's proof is that the picture does not change.
      fpvCtx.imageSmoothingEnabled = false;
      fpvCtx.drawImage(img, 0, 0, FPV_W, FPV_H);
      fpvCtx.imageSmoothingEnabled = true;
    };
    // Decode failure would otherwise leave the last frame on screen and
    // silently look like nothing had gone wrong.
    img.onerror = function () {
      renderFpvPlaceholder("Camera frame would not decode");
      setFrameSource("none");
    };
    img.src = "data:" + (frame.mediaType || "image/jpeg") + ";base64," + frame.imageBase64;
    setFrameSource("server");
  }

  // ---------- environment banner ----------
  // Which deployment is this? The page asks the server that served it,
  // rather than pattern-matching its own hostname -- a CloudFront domain
  // can change, and the twin is also opened straight off an NLB, off
  // localhost, and (in tests) off a file server. The one thing that is
  // always true is that robot/server.py served this HTML, so a relative
  // /health is the same deployment by construction.
  //
  // Fails silent and shows nothing. A page that cannot reach its own
  // origin has bigger problems than a missing badge, and production
  // reports no label at all -- so "no banner" is both the healthy
  // production state and the safe failure state.
  async function renderEnvBanner() {
    const el = document.getElementById("env-banner");
    if (!el) return;
    try {
      const base = window.location.pathname.replace(/\/[^/]*$/, "");
      const resp = await fetch(base + "/health", { cache: "no-store" });
      if (!resp.ok) return;
      const label = (await resp.json()).env_label;
      if (!label) return;
      el.innerHTML = '<span class="dot"></span>' + escapeHtml(label) + " environment";
      el.classList.add("visible");
      document.body.classList.add("env-flagged");
      document.title = label.toUpperCase() + " \u00b7 " + document.title;
    } catch (e) {
      /* no banner -- see above */
    }
  }

  // The twin used to paint its own top-down view of the house here, from a
  // hardcoded copy of `sim/maps/starter_house.py`'s layout. **Deleted with
  // the ROS alignment** (`PLAN-ros-alignment.md`), for the reason N1 gave
  // when it built the other map: a page that draws a finished floor plan is
  // showing you the answer, not the robot's belief. `renderMap()` above
  // draws `GET /world/map` instead -- discovered, tri-state, and the same
  // route `slam_toolbox` will serve at R5. There is no layout to copy for a
  // real room, which is why this could never have survived the swap anyway.
  function render(frame) {
    frame = frame || state.lastFrame;
    if (!frame) return; // not connected yet -- nothing to draw
    drawFPV(frame);
    renderTelemetry(frame);
  }

  // Room / facing / free-cells / doorway used to be reported here. They were
  // `frame_description()`'s grid facts -- cells and cardinals -- and
  // `robot/interface.py` is explicit that no policy on the hardware path may
  // read them. A readout is a consumer too, so they went with the rest of
  // the cell layer. What is left is what survives a real robot: whether the
  // collar vetoed, and what the mission has searched.
  function renderTelemetry(frame) {
    const safetyEl = document.getElementById("tel-safety");
    const vetoedRecently = Date.now() < state.safetyFlashUntil;
    safetyEl.innerHTML = vetoedRecently
      ? 'Safety: <span class="alert">VETOED</span>'
      : 'Safety: <span class="safe">OK</span>';
    if (vetoedRecently) {
      requestAnimationFrame(function () { renderTelemetry(state.lastFrame); });
    }

    const rooms = Array.from(state.searchedRooms).sort();
    let summary = rooms.length ? rooms.join(", ") + " searched." : "No rooms searched yet.";
    document.getElementById("mission-summary").textContent = summary;
  }

  function renderLog() {
    const el = document.getElementById("log");
    clearEmptyState(el);
    const entry = state.log[state.log.length - 1];
    const div = document.createElement("div");
    div.className = "entry" + (entry.executed === false ? " veto" : "");
    // Phase M4: name the refusal rather than stamping every one of them
    // VETOED. The reason was already being carried here and thrown away,
    // so a preemption and a wall produced identical lines -- which is the
    // surface version of the bug the whole phase is about. VETOED stays as
    // the fallback for a server that sends no reason.
    var tag = "";
    if (entry.executed === false) {
      tag = " [" + (REFUSAL_LABEL[entry.reason] || "VETOED") + "]";
      if (entry.extra) div.title = entry.extra;
    }
    div.textContent = "#" + entry.step + " " + entry.room + " \u2192 " + entry.action + tag;
    el.appendChild(div);
    el.scrollTop = el.scrollHeight;

    if (state.found && !el.dataset.foundLogged) {
      const foundDiv = document.createElement("div");
      foundDiv.className = "entry found";
      foundDiv.textContent = "\u2605 FOUND " + state.foundSighting.object + " in " + state.foundSighting.room + " at step " + state.foundSighting.step;
      el.appendChild(foundDiv);
      el.dataset.foundLogged = "1";
    }
  }

  // ---------- controls ----------

  document.getElementById("btn-connect").onclick = function () { connect(); };
  document.getElementById("btn-health-check").onclick = function () { runHealthCheck(); };

  document.getElementById("btn-forward").onclick = function () { manualAction("FORWARD"); };
  // R6: tap the SLAM map to send the robot there.
  (function () {
    var mapCanvas = document.getElementById("world-map");
    if (mapCanvas) mapCanvas.addEventListener("click", onMapTap);
  })();
  document.getElementById("btn-reverse").onclick = function () { manualAction("REVERSE"); };
  document.getElementById("btn-left").onclick = function () { manualAction("LEFT"); };
  document.getElementById("btn-right").onclick = function () { manualAction("RIGHT"); };
  document.getElementById("btn-stop").onclick = function () { manualAction("STOP"); };
  document.getElementById("btn-look-left").onclick = function () { manualAction("LOOK_LEFT"); };
  document.getElementById("btn-look-right").onclick = function () { manualAction("LOOK_RIGHT"); };
  document.getElementById("btn-look-center").onclick = function () { manualAction("LOOK_CENTER"); };
  document.querySelectorAll(".turn-step-btn").forEach(function (btn) {
    btn.onclick = function () { setTurnStep(Number(btn.dataset.turnDeg)); };
  });
  setTurnStep(Number(prefGet(PREF.turnStepDeg)) || 90);

  // ---------- remote brain: this page as an observer (phase B4) ----------
  //
  // Everything above this line is a brain that lives in this tab: it
  // decides, it drives robot/server.py, and it dies with the page. This
  // section is the other arrangement -- control/brain_server.py owns the
  // loop, and the page only starts it, watches it, and stops it.
  //
  // The observable difference, and the whole reason the phase exists: this
  // polling loop is deliberately NOT paused on visibilitychange the way
  // Vision Autopilot's is. Backgrounding the tab pauses the *observer* (the
  // browser throttles its timers, and that is fine -- polling costs
  // nothing) and does not touch the mission. Come back, or open the page on
  // a second phone, and the robot has kept going. That is the proof the
  // brain actually moved off this device.
  //
  // Manual control stays pointed straight at robot/server.py and never goes
  // through here: the D-pad must keep working with the brain service down.

  const BRAIN_POLL_MS = 700;
  const BRAIN_EMPTY = ["bot", "No mission yet \u2014 Start hands the loop to the robot."];

  // Distinct from authHeaders() (robot/server.py's secret) -- the brain is
  // an independently deployable service with its own APP_SHARED_SECRET
  // (cloudformation/brain.yaml / teleop-brain.yaml each generate their own),
  // not necessarily the robot's. Conflating the two meant every brain call
  // silently sent the *robot's* secret, which 401s the moment the two
  // differ -- as they always do once each is its own ECS service.
  function brainAuthHeaders(extra) {
    const headers = Object.assign({}, extra);
    if (state.brainSecret) headers["x-app-secret"] = state.brainSecret;
    return tunnelHeaders(state.brainUrl, headers);
  }
  // FastAPI's `detail` as text: a 422's is a list, which once read
  // "[object Object]" (3.64). Empty when the body said nothing -- a
  // gateway's or a proxy's page, not one of our services.
  function errorDetail(data) {
    if (!data || data.detail == null) return "";
    return typeof data.detail === "string" ? data.detail : JSON.stringify(data.detail);
  }

  async function brainApi(method, path, body) {
    const res = await fetch(state.brainUrl + path, {
      method: method,
      headers: brainAuthHeaders(body ? { "content-type": "application/json" } : {}),
      body: body ? JSON.stringify(body) : undefined,
    });
    const data = await res.json().catch(function () { return {}; });
    if (!res.ok) {
      const detail = errorDetail(data);
      const err = new Error(detail || ("HTTP " + res.status));
      err.fromServer = detail !== "";
      // Kept so a caller can tell "the brain answered with an error" from
      // "nothing answered" -- see brainGone().
      err.status = res.status;
      throw err;
    }
    return data;
  }

  // Did this failure mean the brain is GONE, rather than that it answered
  // badly? A fetch that never got a response has no status; a 502/503/504 is
  // the tunnel's proxy saying nothing is behind it. A 404, 401 or 500 means
  // a brain answered -- reachable, and disconnecting from it would hide the
  // real error behind "retrying". The first cut treated every failure as
  // lost, and a test whose stub brain 404s on /mission/status caught it.
  function brainGone(e) {
    return !e.status || e.status === 502 || e.status === 503 || e.status === 504;
  }

  function setBrainText(id, value, cls) {
    const el = document.getElementById(id);
    if (!el) return;
    el.textContent = value == null || value === "" ? "\u2013" : String(value);
    el.className = "val" + (cls ? " " + cls : "");
  }

  const BRAIN_OUTCOME_CLASS = {
    found: "safe", room_reached: "safe", failed: "alert", stopped: "alert",
    blocked: "alert",
    // 3.53: neither -- the robot reached what it took for the target and
    // the cloud that must confirm identity could not be asked.
    arrived_unconfirmed: "warn",
  };

  // 3.53: what the late identity check (asked once the cloud is back) has
  // said so far, in words. "" when there is none.
  const LATE_WORDS = {
    waiting: "will ask again when the cloud is back",
    confirmed: "asked later: the cloud sees it",
    refused: "asked later: the cloud says it is not the target",
    failed: "asked later: the cloud did not answer",
    not_asked: "not asked later: the call budget was spent",
    expired: "the cloud did not come back in time",
    dropped: "not asked later: a new mission or Stop came first",
  };
  function lateWords(status) {
    const late = status && status.late_confirmation;
    return late ? (LATE_WORDS[late.state] || late.state) : "";
  }

  // The sentence for an `arrived_unconfirmed` ending: what was checked
  // (the lidar arrival) and what was not (identity), then the late check.
  function unconfirmedText(status) {
    const late = lateWords(status);
    return "Stopped at what looks like " + (status.target_object || "the target")
      + " (lidar " + ((status.arrival && status.arrival.range_m) != null
        ? status.arrival.range_m + " m" : "range unknown")
      + "), but its identity is unconfirmed: the cloud could not be reached."
      + (late ? " " + late.charAt(0).toUpperCase() + late.slice(1) + "." : "");
  }

  // 3.56: the brain's `status.cloud` in words. Null when the brain has no
  // vision service configured, or predates 3.56 -- shown as a dash, never
  // as "reachable".
  // An answer older than two probe intervals (plus slack) is not current:
  // nothing probes between cloud missions. Shown as when it was last seen,
  // uncoloured, never as a live "reachable". `age_s` is the brain's own
  // measure, so the phone's clock never enters it.
  function cloudWords(cloud) {
    if (!cloud || !cloud.state) return { text: null, cls: null };
    if (cloud.state === "unknown") return { text: "unknown (not checked yet)", cls: null };
    const at = (t) => (t ? new Date(t * 1000).toLocaleString([], {
      month: "short", day: "numeric", hour: "2-digit", minute: "2-digit", second: "2-digit" }) : "");
    const stale = cloud.age_s != null && cloud.age_s > 2 * (cloud.probe_s || 15) + 5;
    if (stale) return { text: "last seen " + cloud.state + " at " + at(cloud.checked_at), cls: null };
    if (cloud.state === "reachable") return { text: "reachable", cls: "safe" };
    return { text: "unreachable" + (cloud.since ? " since " + at(cloud.since) : ""), cls: "warn" };
  }

  // The Cloud row alone, from any status (the panel poll, the late-check
  // watcher, a Stop's answer -- the brain sends `cloud` on all three). A
  // status without the field is a brain that predates 3.56: a dash, never
  // "reachable".
  function renderCloudRow(status) {
    if (!status) return;
    const cloud = cloudWords(status.cloud);
    setBrainText("brain-tel-cloud", cloud.text, cloud.cls);
  }

  function renderBrainStatus(status) {
    const outcome = status.outcome || "idle";
    const late = outcome === "arrived_unconfirmed" ? lateWords(status) : "";
    setBrainText("brain-tel-outcome",
      outcome + (status.fault && status.fault !== "none" ? " (" + status.fault + " drill)" : "")
        + (late ? " — " + late : ""),
      BRAIN_OUTCOME_CLASS[outcome]);
    setBrainText("brain-tel-step",
      status.step == null ? null : status.step + (status.max_steps ? " / " + status.max_steps : ""));
    // R1: a turn names its size. "LEFT 23°" is a bearing-sized correction;
    // a bare "LEFT" is the executor's default quarter turn (a scan, or a
    // cloud answer with nothing local to size it by).
    const t = status.turns || null;
    const sized = t && t.last_turn_deg && (status.last_action === "LEFT" || status.last_action === "RIGHT");
    setBrainText("brain-tel-action", sized
      ? status.last_action + " " + t.last_turn_deg + "\u00B0" : status.last_action);
    // Reversals alone reward a spin (the first watched run: 98 turns in 120
    // steps, all one way, "0 reversed"), so the share of steps spent
    // turning rides beside it and the runner's own spin verdict is shown.
    setBrainText("brain-tel-turns", t
      ? t.count + " of " + status.step + " steps, " + t.reversals
        + " reversed the one before" + (t.spinning ? " \u2014 SPINNING IN PLACE" : "")
      : null, t && t.spinning ? "alert" : null);
    setBrainText("brain-tel-vision", status.vision_failures || 0,
      status.vision_failures ? "alert" : null);
    setBrainText("brain-tel-rooms",
      (status.rooms_searched && status.rooms_searched.length) ? status.rooms_searched.join(", ") : null);
    const unconfirmed = status.outcome === "arrived_unconfirmed";
    setBrainText("brain-tel-why",
      status.error || (unconfirmed ? unconfirmedText(status) : status.last_reasoning),
      status.error ? "alert" : unconfirmed ? "warn" : null);
    renderCloudRow(status);
    renderTierReadouts(status);

    // The mission log comes from the brain, so it is rewritten wholesale
    // each poll rather than appended to -- there is no local copy to keep
    // in step, and a reconnecting observer gets the same tail as everyone
    // else. Skipped when unchanged so it does not fight the scroll.
    const tail = status.log_tail || [];
    const signature = tail.join("\n");
    if (signature !== state.brainLogSignature) {
      state.brainLogSignature = signature;
      const el = document.getElementById("brain-log");
      if (tail.length === 0) {
        setEmptyState(el, BRAIN_EMPTY[0], BRAIN_EMPTY[1]);
      } else {
        el.dataset.empty = "";
        el.innerHTML = tail.map(function (line) {
          const cls = /failure|failed|WARNING/.test(line) ? " veto"
            : /found\)/.test(line) ? " found" : "";
          return '<div class="entry' + cls + '">' + escapeHtml(line) + "</div>";
        }).join("");
        el.scrollTop = el.scrollHeight;
      }
    }
  }

  // ---------- phase P2: the three tiers, made watchable ----------
  //
  // PLAN-onboard-perception.md 6.3 asks for four things here, and each one
  // answers a question the panel could not answer before:
  //
  //   * the tri-state, so a wedged capture never reads as a missing target
  //     (1.12 -- the same distinction the depth grid's ZONE_UNUSABLE makes
  //     one sensor over, and for the same reason);
  //   * the CLIP margin, NOT the bare similarity: CLIP returns a similarity
  //     rather than a probability, so the margin over the distractors is
  //     the only number that means anything;
  //   * the detector's own name, because swapping it is the experiment loop
  //     (4.7's promotion rule) and a swap you cannot see is not one;
  //   * the deliberation-call counter -- "that single number makes the whole
  //     architecture watchable" -- shown as calls AND frames, since a count
  //     climbing by one is indistinguishable from a call every step.
  //
  // The whole group hides under a policy with no perception tier. Drawing
  // zeroes there would say the architecture had stopped deliberating, which
  // is a different and much more alarming claim than "not running".

  const PERCEPTION_CLASS = {
    detected: "safe",
    // Neither safe nor alert: the frame was good and the thing is not in
    // it, which is information and often the correct state for most of a
    // mission.
    absent: null,
    // The absence of information, which is a fault -- and the one state
    // that must never look like `absent`.
    unavailable: "alert",
  };

  function renderTierReadouts(status) {
    const rows = document.getElementById("brain-tier-rows");
    if (!rows) return;
    const tier = status.tier;
    const perception = status.perception;
    if (!tier && !perception) {
      rows.style.display = "none";
      return;
    }
    rows.style.display = "";

    const p = perception || {};
    const bits = [p.status || "\u2013"];
    if (p.label) bits.push(p.label);
    if (typeof p.bearing_deg === "number") {
      bits.push((p.bearing_deg >= 0 ? "+" : "") + p.bearing_deg.toFixed(0) + "\u00b0");
    }
    if (p.status === "unavailable" && p.reason) bits.push(p.reason);
    if (p.status === "absent" && p.candidates) bits.push(p.candidates + " candidate(s)");
    setBrainText("brain-tel-perception", bits.join(" \u00b7 "),
      PERCEPTION_CLASS[p.status]);

    setBrainText("brain-tel-margin", typeof p.match_margin === "number"
      ? p.match_margin.toFixed(2) + " over the best distractor"
        + (typeof p.similarity === "number"
          ? " (similarity " + p.similarity.toFixed(2) + ")" : "")
      : "no candidate scored");

    const models = (tier && tier.models) || {};
    setBrainText("brain-tel-detector",
      models.detector
        // No scorer is a real configuration -- the simulator's synthetic
        // detections have none -- so say nothing rather than print "?".
        ? models.detector + (models.scorer ? " + " + models.scorer : "")
          // Named only when it is actually running. Which crop sources were
          // on is the first thing you need to know reading a walk back --
          // the detector alone measured 86% recall on the corpus and the
          // detector plus the mask 94%, so a walk that cannot say which it
          // used cannot be compared with one that can.
          + (models.proposer ? " + " + models.proposer.split("/").pop() : "")
        : null);

    // 1.11a, reported only. Two things have to be on screen together or the
    // row is unreadable: this frame's verdict, and the running
    // corroborated-of-claimed tally -- which is 1.11a's own list of what it
    // still needs ("the counter in 6.3 should show corroborated-versus-
    // claimed, or the twin cannot show this working").
    //
    // "not enforced" is printed every time, deliberately. The whole point of
    // the reporting-only variant is that a person watching this panel can
    // see the amendment being measured and cannot mistake it for the
    // amendment being applied -- the mission still believes the cloud.
    const corroboration = tier && tier.corroboration;
    const cstats = (tier && tier.stats) || {};
    if (!tier || cstats.claims == null) {
      setBrainText("brain-tel-corroboration", null);
    } else {
      let text;
      if (!corroboration) {
        // A free frame made no cloud call, so there is no claim to
        // corroborate. Said rather than left blank: a stale verdict held
        // over from the last paid call would read as this frame's.
        text = "no claim this step";
      } else if (corroboration.verdict === "no_claim") {
        text = "cloud says not visible";
      } else if (corroboration.verdict === "unavailable") {
        text = "local tier could not tell";
      } else {
        const local = typeof corroboration.local_probability === "number"
          ? corroboration.local_probability.toFixed(2) : "\u2013";
        text = corroboration.verdict + " (local P " + local
          + " vs bar " + corroboration.bar + ")";
      }
      // Phase A. An async verdict is computed against the perception the
      // call was MADE on and lands a frame or more later, so it must say
      // so -- otherwise it reads as a verdict about the picture on screen
      // now, which is the stale-readout failure this row already guards
      // against from the other direction.
      if (corroboration && corroboration.landed_late) {
        text = "\u21b5 landed from the " + (corroboration.for_trigger || "earlier")
          + " call \u00b7 " + text;
      }
      text += " \u00b7 " + cstats.corroborated + "/" + cstats.claims
        + " claims corroborated \u00b7 not enforced";
      setBrainText("brain-tel-corroboration", text,
        // `unclear` is the state 1.11a exists to name, so it is worth
        // seeing -- but it is not an error and must not be dressed as one:
        // under the shipped rule the robot believes the claim anyway.
        corroboration && corroboration.verdict === "unclear" ? "alert" : null);
    }

    // Spec review 3, fix 9: arrival, from status.arrival. The lidar judges
    // distance and the cloud confirms identity; a refusal is the state an
    // operator most needs, because without it the mission's end reads as
    // an obstacle.
    const arrival = status.arrival;
    if (!arrival) {
      setBrainText("brain-tel-arrival", null);
    } else {
      const range = typeof arrival.range_m === "number"
        ? arrival.range_m.toFixed(2) + " m" : null;
      const identity = arrival.identity || {};
      let text, cls = null;
      if (arrival.state === "arrived") {
        text = "arrived" + (range ? " at " + range : "")
          + (identity.confirmed ? " \u00b7 identity confirmed by the cloud" : "");
        cls = "safe";
      } else if (arrival.state === "refused") {
        text = "refused" + (range ? " at " + range : "") + " \u00b7 "
          + (identity.reason || arrival.reason || "identity not confirmed");
        cls = "alert";
      } else {
        text = arrival.state.replace("_", " ") + (range ? " (" + range + ")" : "")
          + (arrival.reason ? " \u00b7 " + arrival.reason : "");
      }
      setBrainText("brain-tel-arrival", text, cls);
    }

    // Phase C. The pacing rule, and the distance to the next look. Reads
    // "frames" on every teleop walk, because a phone on a wheeled rig has
    // no encoders -- and that has to be legible rather than inferred from
    // a distance readout that never moves.
    const pacing = tier && tier.pacing;
    if (!pacing) {
      setBrainText("brain-tel-pacing", null);
    } else if (pacing.rule === "distance") {
      const left = Math.max(0, pacing.cm_bar - (pacing.cm_since_call || 0));
      setBrainText("brain-tel-pacing",
        "by distance \u00b7 " + (pacing.cm_since_call || 0).toFixed(0) + "/"
        + pacing.cm_bar + "cm \u00b7 next look in " + left.toFixed(0) + "cm");
    } else {
      setBrainText("brain-tel-pacing",
        "by frame count \u00b7 " + pacing.frames_absent + "/" + pacing.frames_bar
        + (pacing.reason ? " \u00b7 " + pacing.reason : ""));
    }

    // Phase A. Three states, and they must not look alike: waiting on an
    // answer while driving on the last goal, waiting with no goal to hold,
    // and not waiting at all.
    if (!tier) {
      setBrainText("brain-tel-inflight", null);
    } else if (tier.in_flight) {
      setBrainText("brain-tel-inflight",
        tier.in_flight + " call in flight \u00b7 "
        + (tier.holding ? "holding goal " + tier.holding : "scanning, no goal yet"),
        "alert");
    } else {
      setBrainText("brain-tel-inflight", "idle \u00b7 no call outstanding");
    }

    const stats = (tier && tier.stats) || {};
    if (stats.frames == null) {
      setBrainText("brain-tel-calls", null);
    } else {
      // "1 per N frames" rather than "Nx", because the panel is read while a
      // mission is running and the raw pair is what makes the ratio
      // checkable by eye. 6.1 measured 4-6x on recorded walks; this is the
      // same quantity, live.
      const per = stats.frames_per_call;
      setBrainText("brain-tel-calls",
        stats.cloud_calls + (stats.cloud_calls === 1 ? " cloud call / " : " cloud calls / ")
        + stats.frames + (stats.frames === 1 ? " frame" : " frames")
        + (per ? " \u00b7 1 per " + per.toFixed(1) : ""),
        stats.cloud_calls && stats.cloud_calls >= stats.frames ? "alert" : "safe");
    }
  }

  // The robot server's watchdog (failsafe B3.1) has no UI of its own: it
  // fires inside the robot process and stops the motors. /health reports
  // the silence it measures, which is the closest thing to watching it
  // work.
  //
  // It polls on its own timer rather than off the back of the brain poll,
  // because the moment you most want to read it is the moment the brain
  // has just died -- kill the brain service mid-mission and this is what
  // tells you the robot went quiet and its motors were stopped. Hanging it
  // off the brain poll would freeze it exactly then. /health does not
  // update `last_command_at` (only /action and /stop do), so watching the
  // watchdog cannot hold the watchdog off.
  const WATCHDOG_POLL_MS = 500;

  function startWatchdogPolling() {
    stopWatchdogPolling();
    state.watchdogTimerId = setInterval(function () {
      // Only while the Sim tab is actually on screen -- Guide and Camera
      // never touch the robot server, and a hidden readout is not worth a
      // request every half second.
      if ((document.body.dataset.tab || "guide") !== "sim") return;
      renderWatchdog();
    }, WATCHDOG_POLL_MS);
    renderWatchdog();
  }
  function stopWatchdogPolling() {
    if (state.watchdogTimerId) clearInterval(state.watchdogTimerId);
    state.watchdogTimerId = null;
  }

  async function renderWatchdog() {
    if (!state.connected) { setBrainText("brain-tel-watchdog", null); return; }
    try {
      const res = await fetch(state.serverUrl + "/health",
        { headers: tunnelHeaders(state.serverUrl, {}) });
      const health = await res.json();
      const age = health.seconds_since_last_command;
      const timeout = health.watchdog_timeout_s;
      const fired = age > timeout;
      // Keep the local-brain's own clearance checks and the map's safety
      // collar (below) in sync with whatever this server actually
      // enforces -- see MIN_DISTANCE_CM_FALLBACK's own comment. Guarded
      // so an older deployment with no min_distance_cm field just keeps
      // the fallback rather than setting state.minDistanceCm to
      // undefined.
      if (typeof health.min_distance_cm === "number") {
        state.minDistanceCm = health.min_distance_cm;
      }
      // Which body is on the other end. The tiered hint has to know: since
      // R1 a simulated robot gets synthetic detections and NO model runs,
      // so naming the brain's detector there would describe a different
      // mission from the one Start is about to launch.
      const mode = health.mode || null;
      if (mode !== state.robotMode) {
        state.robotMode = mode;
        renderBrainPolicyHint();
      }
      setBrainText("brain-tel-watchdog",
        "quiet " + age.toFixed(1) + "s / " + timeout + "s " + (fired ? "\u2014 motors stopped" : "\u2014 armed"),
        fired ? "alert" : "safe");
      renderAuthority(health);
    } catch (e) {
      setBrainText("brain-tel-watchdog", "unreachable", "alert");
      setBrainText("brain-tel-driver", null);
      setBrainText("brain-tel-refusal", null);
    }
  }

  // ---------- phase M5: one health answer ----------
  // The same verdict `python -m control.health` prints, from the page.
  //
  // The page asks both services directly rather than asking one of them to
  // report on the other: a brain that cannot reach the robot is exactly
  // the case this has to catch, and it would report itself fine.
  //
  // **The rule about what may reach a verdict lives in control/health.py,
  // not here.** This renders `ok` / `unhealthy` / `unreachable` per half
  // and never invents a judgement of its own -- if it did, the page and
  // the command that gates M11's rollback could disagree about the same
  // robot, and the page is the one people would believe.
  async function probeHealth(url, secret) {
    if (!url) return { status: "not configured" };
    try {
      const headers = tunnelHeaders(url, secret ? { "x-app-secret": secret } : {});
      const res = await fetch(url.replace(/\/$/, "") + "/health", { headers: headers });
      if (!res.ok) return { status: "unreachable", problem: "HTTP " + res.status };
      return { status: "ok", body: await res.json() };
    } catch (e) {
      return { status: "unreachable", problem: e.message };
    }
  }

  function robotProblem(body) {
    // Verdict input: the watchdog loop's own liveness. Absent on a server
    // older than M5, which cannot be judged on a field it does not
    // publish -- absent is not stale.
    const age = body.seconds_since_watchdog_poll;
    const interval = body.watchdog_poll_interval_s;
    if (typeof age === "number" && interval && age > interval * 10) {
      return "watchdog loop has not polled for " + age.toFixed(1) + "s";
    }
    return null;
  }

  function brainProblem(body) {
    const since = body.seconds_since_last_tick;
    const deadline = body.tick_timeout_s;
    if (body.mission_running && typeof since === "number" && deadline && since > deadline) {
      return "a mission is running but has not ticked for " + since.toFixed(1) + "s";
    }
    return null;
  }

  async function runHealthCheck() {
    const verdictEl = document.getElementById("system-health-verdict");
    const detailEl = document.getElementById("system-health-detail");
    if (!verdictEl) return;
    verdictEl.className = "health-verdict";
    verdictEl.textContent = "checking\u2026";
    detailEl.textContent = "";

    const robotUrl = document.getElementById("cfg-server-url").value.trim();
    const brainUrl = document.getElementById("cfg-brain-url").value.trim();
    const [robot, brain] = await Promise.all([
      probeHealth(robotUrl, document.getElementById("cfg-server-secret").value.trim()),
      probeHealth(brainUrl, document.getElementById("cfg-brain-secret").value.trim()),
    ]);

    const parts = [];
    const failed = [];
    [["robot", robot, robotProblem], ["brain", brain, brainProblem]].forEach(function (row) {
      const name = row[0], result = row[1], check = row[2];
      if (result.status === "not configured") { parts.push(name + ": not configured"); return; }
      if (result.status !== "ok") {
        failed.push(name);
        parts.push(name + ": unreachable (" + (result.problem || "no answer") + ")");
        return;
      }
      const problem = check(result.body);
      if (problem) { failed.push(name); parts.push(name + ": " + problem); return; }
      const ident = result.body.identity;
      parts.push(name + ": ok" + (ident && ident.git_revision ? " (" + ident.git_revision + ")" : ""));
    });

    verdictEl.textContent = failed.length ? "UNHEALTHY \u2014 " + failed.join(", ") : "OK";
    verdictEl.className = "health-verdict " + (failed.length ? "bad" : "ok");
    detailEl.textContent = parts.join(" \u00b7 ");
  }

  // Phase M4. Two readouts, both off /health, both answering "the robot is
  // not moving -- what stopped it?".
  //
  // `authority_holder` is deliberately separate from `driver`: the first is
  // who holds the robot *now*, the second is who last had it. Authority
  // lapses on silence, so "brain (lapsed)" is a real and different state
  // from "brain is driving" -- it is what you see a second after a mission
  // ends, and it is when a new driver may take over cleanly.
  //
  // Pre-M4 servers report neither field; both readouts then stay blank
  // rather than inventing a driver, which is the same choice the depth
  // strip makes about a server with no /depth.
  function renderAuthority(health) {
    // Key presence, not value: a pre-M4 server sends no `driver` at all,
    // while an M4 server sends null for "nobody has driven yet". Reading
    // null as "no such server field" would blank the readout on exactly
    // the server that supports it.
    if (!health || !("driver" in health)) {
      setBrainText("brain-tel-driver", null);
      setBrainText("brain-tel-refusal", null);
      return;
    }
    const holder = health.authority_holder;
    setBrainText("brain-tel-driver",
      holder ? holder : (health.driver ? health.driver + " (lapsed)" : "nobody yet"),
      holder ? "safe" : null);

    const refusal = health.last_refusal;
    if (!refusal) { setBrainText("brain-tel-refusal", "none"); return; }
    const label = REFUSAL_LABEL[refusal.reason] || refusal.reason;
    // A watchdog stop can happen before anyone has driven, so there may be
    // no driver to name -- it used to print the word "null".
    setBrainText("brain-tel-refusal",
      label + " \u00b7 " + (refusal.driver || "no driver") + " \u00b7 " +
      refusal.seconds_ago.toFixed(1) + "s ago",
      "alert");
  }

  // ---------- brain liveness: reconnect on its own ----------
  // A restarted brain used to leave this page disconnected for good: the
  // connection was attempted once at load and after that only by pressing
  // Connect in Settings, so Start simply sat greyed with no reason given.
  // Found the day R1 shipped, when the brain was restarted twice in a few
  // minutes under a phone that had it open.
  const BRAIN_LIVENESS_MS = 5000;

  function markBrainLost(e) {
    if (!state.brainConnected) return;
    state.brainConnected = false;
    state.brainLost = true;
    stopBrainPolling();
    setBrainText("brain-tel-outcome", "brain unreachable \u2014 retrying", "alert");
    setConnStatus(document.getElementById("brain-connection-status"), "err",
      "Not connected", state.brainUrl + " stopped answering (" + e.message
        + ") \u2014 retrying every few seconds.");
    updateBrainControls();
    showToast("Lost the brain service \u2014 retrying.", "err");
  }

  function startBrainLiveness() {
    if (state.brainLivenessTimerId) return;
    state.brainLivenessTimerId = setInterval(async function () {
      if (document.hidden || !state.brainUrl || state.brainConnecting) return;
      if (!state.brainConnected) {
        await connectBrain({ silent: true });
        return;
      }
      // Connected and idle: a mission's own status poll covers the running
      // case, so only ping when nothing else is asking.
      if (state.brainPollTimerId) return;
      try { await brainApi("GET", "/health"); } catch (e) { if (brainGone(e)) markBrainLost(e); }
    }, BRAIN_LIVENESS_MS);
  }

  async function pollBrainOnce() {
    let status;
    try {
      status = await brainApi("GET", "/mission/status");
    } catch (e) {
      if (brainGone(e)) { markBrainLost(e); return; }
      setBrainText("brain-tel-outcome", "brain error: " + e.message, "alert");
      stopBrainPolling();
      showToast("The brain service answered with an error: " + e.message, "err");
      return;
    }

    const wasRunning = state.brainMissionRunning;
    state.brainMissionRunning = !!status.running;
    renderBrainStatus(status);
    updateBrainControls();

    // Observer, not driver: the map and first-person view follow the
    // robot's own reported frame, exactly as they do when this page is the
    // one driving. Caught separately from the status call above -- a dead
    // *robot* is a different failure from a dead *brain*, and reporting one
    // as the other would send you debugging the wrong process. The
    // watchdog readout is the one that names this case.
    if (state.connected) {
      try {
        const frame = await fetchFrame();
        recordObservation(frame);
        render(frame);
      } catch (e) { /* robot unreachable -- renderWatchdog() reports it */ }
    }

    if (wasRunning && !status.running) {
      const ok = status.outcome === "found" || status.outcome === "room_reached";
      if (status.outcome === "arrived_unconfirmed") {
        showToast("Mission arrived_unconfirmed: " + unconfirmedText(status), "warn");
      } else {
        showToast("Mission " + status.outcome + (status.error ? ": " + status.error : ""),
          ok ? "ok" : "err");
      }
      stopBrainPolling();
      watchLateConfirmation(status);
    } else if (!status.running && !state.lateWatchTimerId) {
      // A page that connects (or reconnects) after the ending never saw
      // the transition; a late check still waiting is watched all the same.
      watchLateConfirmation(status);
    }
  }

  // 3.53: after an `arrived_unconfirmed` ending the brain asks again once
  // the cloud is back (control/reconfirm.py). The mission poll has stopped,
  // so this watches only `late_confirmation`, slowly, until it is final,
  // then shows the verdict on the panel and, if it is the one driving, the
  // Guide tab. One watcher at a time; a new mission ends it.
  const LATE_POLL_MS = 3000;
  // A watch is identified by its token, not its timer: an answer that lands
  // after the watch was stopped -- or replaced by a new mission's -- must
  // draw nothing (3.53, review).
  let lateWatchToken = 0;
  const LATE_MAX_ERRORS = 5;
  function stopLateWatch() {
    lateWatchToken++;
    // Whatever bumps the token also ends any pending resume's claim; a
    // Guide start reads the flag before calling this, and re-sets it if
    // its own resume takes over (tenth review).
    state.lateResumePending = false;
    if (state.lateWatchTimerId) clearInterval(state.lateWatchTimerId);
    state.lateWatchTimerId = null;
  }
  function watchLateConfirmation(status) {
    stopLateWatch();
    const late = status && status.late_confirmation;
    if (status.outcome !== "arrived_unconfirmed" || (late && late.state !== "waiting")) return;
    const token = lateWatchToken;
    // `late_confirmation` is null for a moment at the ending (the brain
    // starts its check just after), and for good when the check is off.
    // One poll of grace, then a null means off and the watch ends.
    let nullPolls = 0;
    let errors = 0;  // a dead brain is not polled forever
    let inFlight = false;  // a slow tunnel must not double the toast
    state.lateWatchTimerId = setInterval(async function () {
      if (inFlight) return;
      inFlight = true;
      let s;
      try { s = await brainApi("GET", "/mission/status"); }
      catch (e) {
        if (token === lateWatchToken && ++errors >= LATE_MAX_ERRORS) stopLateWatch();
        return;
      }
      finally { inFlight = false; }
      if (token !== lateWatchToken) return;  // stopped or replaced while in flight
      errors = 0;
      // 3.56 follow-up: the panel poll has stopped with the mission, and this
      // is the only status the page still reads -- keep the Cloud row honest
      // ("reachable" when /health returns, "last seen ..." as it ages).
      renderCloudRow(s);
      if (s.running || s.outcome !== "arrived_unconfirmed") { stopLateWatch(); return; }
      const l = s.late_confirmation;
      if (!l) { if (++nullPolls > 1) stopLateWatch(); return; }
      if (l.state === "waiting") return;
      stopLateWatch();
      showLateVerdict(s);
    }, LATE_POLL_MS);
  }

  // A final late verdict, drawn once: the panel, the Guide HUD when it is
  // the one driving, and a toast.
  let lateShownAt = null;
  function showLateVerdict(s) {
    // The verdict's own time is its identity: one already shown (by the
    // watch, before a refused start) is not toasted again.
    if (s.late_confirmation.at != null && s.late_confirmation.at === lateShownAt) return;
    lateShownAt = s.late_confirmation.at;
    renderBrainStatus(s);
    if (driveViaBrainActive()) {
      const result = missionStatusToRobotResult(s);
      renderRobotStatus(result, false);
      renderRobotTelemetry(result);
      announceRobot(result);
    }
    showToast("Late identity check: " + lateWords(s) + ".",
      s.late_confirmation.state === "confirmed" ? "ok" : "warn");
  }

  // After a refused Drive-via-brain start: the brain kept the last
  // mission's check, and it may have finished while the start was out --
  // shown now if so, watched again if not.
  function resumeLateConfirmation(s) {
    const l = s && s.late_confirmation;
    if (!s || s.running || s.outcome !== "arrived_unconfirmed" || !l) return;
    if (l.state === "waiting") watchLateConfirmation(s);
    else showLateVerdict(s);
  }

  function startBrainPolling() {
    stopBrainPolling();
    stopLateWatch();
    state.brainPollTimerId = setInterval(pollBrainOnce, BRAIN_POLL_MS);
    pollBrainOnce();
  }
  function stopBrainPolling() {
    if (state.brainPollTimerId) clearInterval(state.brainPollTimerId);
    state.brainPollTimerId = null;
  }

  function updateBrainControls() {
    const btn = document.getElementById("btn-brain-mission");
    const hint = document.getElementById("brain-connect-hint");
    btn.disabled = !state.brainConnected;
    btn.textContent = state.brainMissionRunning ? "Stop" : "Start";
    // Start is the panel's primary action; Stop is a stop. The default grey
    // style made Start read as a label -- see index.html.
    btn.classList.toggle("primary", !state.brainMissionRunning);
    btn.classList.toggle("danger", state.brainMissionRunning);
    document.getElementById("brain-fault").disabled = state.brainMissionRunning;
    document.getElementById("brain-target").disabled = state.brainMissionRunning;
    document.getElementById("brain-policy").disabled = state.brainMissionRunning;
    if (hint) {
      hint.style.display = state.brainConnected ? "none" : "";
      // Say WHY Start is greyed. A configured brain that stopped answering
      // is a different state from no brain at all, and the page now retries
      // the first on its own -- see startBrainLiveness().
      hint.textContent = state.brainUrl
        ? "Brain unreachable at " + fieldHost("cfg-brain-url")
          + " \u2014 retrying every few seconds. Start comes back on its own."
        : "No brain service configured \u2014 open Settings to point this at one.";
    }
    renderBrainPolicyHint();
    updateBrainPickersRow();
  }

  // The two policies that reach the cloud vision service. They differ in
  // how OFTEN they call it, not in whether they can -- so everything that
  // exists because a mission spends money (the pickers, the hint, the
  // model and wording sent at start) applies to both.
  function isCloudPolicy(policy) {
    return policy === "vision" || policy === "tiered";
  }

  // The vision policy spends money and its result is only interpretable if
  // you know which model and which wording produced it -- so say both, in
  // words, before the mission starts rather than leaving them to be inferred
  // from a log afterwards.
  //
  // The tiered policy needs the same sentence and one more: the local half
  // is where its cost story lives, and it is invisible from everything else
  // on this panel. Naming the detector here is also what makes swapping one
  // watchable (PLAN-onboard-perception.md 6.3) -- change the weights the
  // brain loads and this line changes with it.
  function renderBrainPolicyHint() {
    const hint = document.getElementById("brain-policy-hint");
    if (!hint) return;
    if (!isCloudPolicy(state.brainPolicy)) {
      hint.style.display = "none";
      return;
    }
    hint.style.display = "";
    const asked = "Model: <b>" + escapeHtml(brainModelLabel()) + "</b>. "
      + "Wording: <b>" + escapeHtml(brainPromptLabel()) + "</b>.";
    if (state.brainPolicy === "vision") {
      hint.innerHTML = "Every step is a paid <code>/navigate</code> call. " + asked;
      return;
    }
    hint.innerHTML = tieredCostSentence() + " " + asked + tieredWarning();
  }

  function tieredCostSentence() {
    const tail = " A paid <code>/navigate</code> call goes out only on a "
      + "trigger (mission start, a candidate sighting, a cold search, or "
      + "staleness) and once at arrival to confirm the target.";
    // R1 / 1.12: against the simulator nothing runs a detector on a
    // raycaster render -- the simulator reports what its own geometry shows,
    // and the mission's Models line will read "sim ground truth". Saying
    // "yoloe + RN50" here would describe a mission this button does not
    // start.
    if (state.robotMode === "sim") {
      return "Against the simulator no model runs: the simulator reports what "
        + "its own geometry shows (<b>sim ground truth</b>), because a detector "
        + "on a rendered wall measures nothing." + tail;
    }
    const p = state.brainPerception;
    // Not connected yet: the brain is the only thing that knows which
    // models it would load, and guessing them here would be the
    // NavigateModelId trap in a new place -- a name on screen that no
    // process ever agreed to.
    const models = p
      ? "<b>" + escapeHtml(p.detector) + "</b> + <b>" + escapeHtml(p.clip) + "</b>"
      : "its detector and CLIP encoder (connect the brain to see which)";
    return "Perception runs in the brain process on every frame, free \u2014 "
      + models + "." + tail;
  }

  // `ultralytics` and `torch` are a deliberately optional install, so a
  // brain without them is a normal state rather than a broken one. The
  // mission does refuse with a usable message -- but reading it costs a
  // press of Start, and the panel already knows.
  function tieredWarning() {
    const p = state.brainPerception;
    if (!p || p.available !== false) return "";
    return " <span class=\"alert\">This brain has no perception models "
      + "installed, so a tiered mission will refuse to start. Run "
      + "<code>pip install -r requirements-perception.txt</code> where the "
      + "brain runs.</span>";
  }

  function updateBrainPickersRow() {
    const row = document.getElementById("brain-policy-pickers-row");
    if (row) row.style.display = isCloudPolicy(state.brainPolicy) ? "" : "none";
  }

  // The two pickers live on the Guide tab, next to Robot view -- the flow
  // they were built for. The vision policy is their second consumer, and a
  // control the operator cannot find is the same as no control, so this jumps
  // there rather than duplicating the selects in a second tab where the two
  // copies could disagree about what is selected.
  document.getElementById("btn-brain-pickers").onclick = function () {
    switchTab("guide");
    const row = document.getElementById("navigate-model-row");
    if (row && row.scrollIntoView) row.scrollIntoView({ block: "center" });
  };

  async function startBrainMission() {
    // One brain at a time. Two loops driving one robot is the exact
    // failure the brain service exists to prevent, and the twin should not
    // be the thing that creates it.
    // There used to be two local loops to stop here -- a JS frontier
    // explorer and a JS vision autopilot, both driving the robot from this
    // tab. Both are gone (`PLAN-ros-alignment.md`): R4 puts `twist_mux`
    // between any driver and the wheels and allows exactly one writer, and
    // a brain that dies with a browser tab was never going to be it. The
    // server's own 409 and M4's authority order are what enforce it now.
    const target = document.getElementById("brain-target").value.trim() || "red backpack";
    const fault = document.getElementById("brain-fault").value;
    saveTargetToHistory(target);
    const btn = document.getElementById("btn-brain-mission");
    setButtonBusy(btn, true, "Starting\u2026");
    const body = { target_object: target, fault: fault, policy: state.brainPolicy };
    // Sent only on the policy that reads them, and only when actually
    // picked: an absent field means "whatever the brain, and then the vision
    // service, defaults to". Sending null instead would be the same value
    // with a worse story about where it came from.
    if (isCloudPolicy(state.brainPolicy)) {
      if (state.navigateModelId) body.model_id = state.navigateModelId;
      if (state.navigatePromptVariant) body.prompt_variant = state.navigatePromptVariant;
    }
    try {
      await brainApi("POST", "/mission/start", body);
      state.brainMissionRunning = true;
      state.brainTarget = target.toLowerCase();
      state.brainLogSignature = "";
      showToast(fault === "none" ? "Mission started on the robot."
        : "Failsafe drill started: " + fault, fault === "none" ? "ok" : "info");
      startBrainPolling();
    } catch (e) {
      showToast("Could not start: " + e.message, "err");
    } finally {
      setButtonBusy(btn, false);
      updateBrainControls();
    }
  }

  async function stopBrainMission() {
    const btn = document.getElementById("btn-brain-mission");
    setButtonBusy(btn, true, "Stopping\u2026");
    try {
      const res = await brainApi("POST", "/mission/stop");
      state.brainMissionRunning = false;
      if (res.status) renderBrainStatus(res.status);
      showToast("Mission stopped \u2014 and so is the robot.", "ok");
    } catch (e) {
      showToast("Could not stop: " + e.message, "err");
    } finally {
      setButtonBusy(btn, false);
      stopBrainPolling();
      updateBrainControls();
    }
  }

  document.getElementById("btn-brain-mission").onclick = function () {
    if (state.brainMissionRunning) stopBrainMission();
    else startBrainMission();
  };

  document.getElementById("brain-policy").addEventListener("change", function () {
    state.brainPolicy = this.value;
    prefSet(PREF.brainPolicy, this.value);
    // The model and wording pickers live on the Guide tab and were
    // previously shown only for Robot view. This is their second consumer.
    updateModelPickerRow();
    renderBrainPolicyHint();
    updateBrainPickersRow();
  });

  async function connectBrain(opts) {
    const silent = !!(opts && opts.silent);
    const url = document.getElementById("cfg-brain-url").value.trim().replace(/\/$/, "");
    const statusEl = document.getElementById("brain-connection-status");
    const btn = document.getElementById("btn-brain-connect");
    if (!url) {
      if (!silent) setConnStatus(statusEl, "err", "No URL yet", "Enter the brain service's address above, then tap Connect.");
      return;
    }
    state.brainUrl = url;
    state.brainSecret = document.getElementById("cfg-brain-secret").value.trim();
    state.brainConnecting = true;
    setButtonBusy(btn, true, "Connecting\u2026");
    try {
      const health = await brainApi("GET", "/health");
      state.brainConnected = true;
      // null means "this brain pins nothing, so the vision service's own
      // default applies" -- resolved for display in brainModelLabel().
      state.brainNavigateModelId = health.navigate_model_id || null;
      state.brainNavigatePromptVariant = health.navigate_prompt_variant || null;
      // Phase P2. A brain older than this simply omits the field, and the
      // absence has to stay "not known" rather than becoming "not
      // available" -- the stacks are redeployed one at a time, and greying
      // out a policy that would actually have worked is the same class of
      // wrong as offering one that will 400.
      state.brainPerception = ("perception_available" in health) ? {
        available: !!health.perception_available,
        detector: health.perception_detector || "unnamed detector",
        clip: health.perception_clip_model || "unnamed encoder",
      } : null;
      prefSet(PREF.brainUrl, url);
      setConnStatus(statusEl, "ok", "Connected" + (health.drills_allowed ? "" : " (drills disabled)"),
        url + " \u2014 driving the robot at " + health.robot_url);
      if (!silent) showToast("Connected to the brain service.", "ok");
      if (state.brainLost) {
        state.brainLost = false;
        showToast("The brain service is back.", "ok");
      }
      updateBrainControls();
      renderRecordStatus();
      renderDriveViaBrainStatus();
      // A mission may already be running -- started from another device, or
      // before this page was even opened. Pick it up rather than showing
      // "idle" over a robot that is moving.
      await pollBrainOnce();
      if (state.brainMissionRunning) startBrainPolling();
    } catch (e) {
      state.brainConnected = false;
      state.brainPerception = null;
      const said = e.message && e.message !== "HTTP " + e.status ? " (" + e.message + ")" : "";
      if (e.status === 404 || e.status === 405) {
        // 3.64 D7r: no /health here -- most often a wrong tunnel prefix or
        // path in the URL -- so "is it running?" points the wrong way.
        setConnStatus(statusEl, "err", "Not connected",
          url + " answered HTTP " + e.status + said + ", but has no brain service "
          + "there. Check the address and any tunnel path prefix.");
        if (!silent) showToast("The brain address answered, but no brain service is there.", "err");
      } else if (!brainGone(e) && e.fromServer) {
        // Said why, as our services do: the brain itself answered.
        setConnStatus(statusEl, "err", "Not connected",
          "The brain at " + url + " answered with an error: HTTP " + e.status + said + ".");
        if (!silent) showToast("The brain service answered with an error.", "err");
      } else if (!brainGone(e)) {
        // An answer with no explanation -- and the brain's /health needs no
        // secret -- is something in front of it: a proxy, a tunnel's policy.
        setConnStatus(statusEl, "err", "Not connected",
          url + " answered HTTP " + e.status + " without saying why -- most likely a "
          + "proxy or tunnel in front of the brain, not the brain itself.");
        if (!silent) showToast("Something in front of the brain service refused the request.", "err");
      } else {
        setConnStatus(statusEl, "err", "Not connected", silent
          ? url + " didn't respond. Tap Connect to retry."
          : "Could not reach " + url + " (" + e.message + "). Is control/brain_server.py running?");
        if (!silent) showToast("Couldn't reach the brain service.", "err");
      }
    } finally {
      state.brainConnecting = false;
      setButtonBusy(btn, false);
      updateBrainControls();
    }
  }

  document.getElementById("btn-brain-connect").onclick = function () { connectBrain(); };

  // ---------- the cloud endpoint's own connection check ----------
  //
  // The brain has had one of these since B4 and the vision service never
  // did, which meant the single most common misconfiguration in this
  // project produced no diagnosis at all: **the vision service publishes no
  // CORS headers.** It has never needed them -- the deployed twin is served
  // from the same CloudFront distribution it calls, so every vision request
  // is same-origin. Serve the page from anywhere else (an ngrok tunnel, a
  // local uvicorn, a second deployment) and the browser preflights, the
  // service answers OPTIONS with a 404, and the only thing the operator
  // sees is "Load failed" on the Robot view camera -- with five URL fields
  // on this page and no indication which one is wrong.
  //
  // So this check does two things in order, and the second is the one worth
  // having: reach /health, and separately compare origins. A cross-origin
  // vision URL is reported as the fault it is even when the service itself
  // is perfectly healthy, because it IS healthy -- curl proves it, and the
  // browser still cannot use it.
  function sameOrigin(url) {
    try {
      return new URL(url, window.location.href).origin === window.location.origin;
    } catch (e) {
      return false;
    }
  }

  async function checkVision(opts) {
    const silent = !!(opts && opts.silent);
    const url = document.getElementById("cfg-url").value.trim().replace(/\/$/, "");
    const statusEl = document.getElementById("vision-connection-status");
    const btn = document.getElementById("btn-vision-connect");
    if (!url) {
      if (!silent) setConnStatus(statusEl, "err", "No URL yet",
        "Enter the vision service's address above, then tap Connect.");
      return;
    }
    const cross = !sameOrigin(url);
    setButtonBusy(btn, true, "Checking\u2026");
    try {
      // /health is deliberately unauthenticated on this service (the ALB
      // health check cannot send custom headers), so it needs no secret and
      // triggers no preflight -- which is exactly why it can still answer
      // when the calls that DO carry x-app-secret are being blocked.
      // tunnelHeaders only adds a header for ngrok hostnames, so a normal
      // vision URL still sends none -- which keeps /health free of the
      // preflight that a custom header would otherwise force, and is the
      // whole reason this probe can answer when the real calls cannot.
      const res = await fetch(url + "/health", { headers: tunnelHeaders(url, {}) });
      if (!res.ok) throw new Error("HTTP " + res.status);
      const body = await res.json().catch(function () { return {}; });
      if (cross) {
        // Healthy and unusable. The distinction matters: retrying, changing
        // the secret, or restarting the service all do nothing here.
        setConnStatus(statusEl, "err", "Reachable, but blocked by the browser",
          url + " answered (" + (body.status || "ok") + "), but it is a "
          + "different origin from this page (" + window.location.origin
          + ") and publishes no CORS headers, so vision calls will fail as "
          + "\u201cLoad failed\u201d. Open the twin from " + url + " instead.");
        if (!silent) showToast("Vision service is up, but this page must be served from it.", "err");
      } else {
        setConnStatus(statusEl, "ok", "Connected",
          url + " \u2014 same origin as this page, so vision calls are not preflighted.");
        if (!silent) showToast("Vision service reachable.", "ok");
      }
    } catch (e) {
      setConnStatus(statusEl, "err", "Not reachable",
        "Could not reach " + url + " (" + e.message + ")."
        + (cross ? " It is also a different origin from this page, which "
                 + "blocks vision calls even when the service is healthy." : ""));
      if (!silent) showToast("Couldn't reach the vision service.", "err");
    } finally {
      setButtonBusy(btn, false);
    }
  }

  document.getElementById("btn-vision-connect").onclick = function () { checkVision(); };

  // ---------- recording a Robot-view walk, for replay (phase S2b) ----------
  //
  // Robot view discards every frame the moment its /navigate answer is
  // rendered. That throws away the only real-pixel material this project
  // has: a walk through an actual house, in order, with the service's live
  // answer for each frame. Kept, the same walk can be replayed through the
  // Python agent (sim/replay_robot.py) as often as you like -- which is how
  // a prompt gets iterated on without re-walking the house for every change.
  //
  // Frames go to the brain service rather than downloading: 60 download
  // prompts on a phone is not a workflow, and the brain is the process that
  // wants the material. Recording is off unless the brain is connected and
  // has it enabled (brain.allow_recording).

  function newWalkName() {
    const d = new Date();
    const pad = function (n) { return String(n).padStart(2, "0"); };
    const target = (state.guidanceTarget || "walk").toLowerCase()
      .replace(/[^a-z0-9]+/g, "-").replace(/^-+|-+$/g, "").slice(0, 24) || "walk";
    // The model rides along inside walk.jsonl's own entries regardless (see
    // vision_core.describe_image_bytes_navigate's model_id field), but
    // putting a short tag in the name too means the admin viewer's plain
    // walk list -- names only, no entries loaded -- is enough to tell which
    // walks used which model without opening each one.
    const modelTag = state.navigateModelId
      ? "-" + state.navigateModelId
          .replace(/^(us|global)\./, "")   // region prefix
          .split(".").pop()                // provider prefix (e.g. "amazon.", "anthropic.")
          .replace(/^claude-/, "")
          .replace(/-\d{8}.*$/, "")         // trailing release date + version suffix
          .replace(/-v\d.*$/, "")           // bare version suffix (no date), e.g. nova-lite-v1:0
          .slice(0, 20)
      : "";
    // The prompt too. It rides along inside walk.jsonl the same way the model
    // does, but telling five walks apart in the console meant opening each
    // one to read prompt_variant out of its frames -- which is exactly the
    // job the model tag was added to avoid. "default" is left off, so a name
    // only grows when there is something to distinguish.
    const promptTag = (state.navigatePromptVariant && state.navigatePromptVariant !== "default")
      ? "-" + state.navigatePromptVariant.replace(/[^a-z0-9]+/gi, "-").slice(0, 20)
      : "";
    return target + modelTag + promptTag + "-" + d.getFullYear() + pad(d.getMonth() + 1) + pad(d.getDate()) +
      "-" + pad(d.getHours()) + pad(d.getMinutes()) + pad(d.getSeconds());
  }

  function recordingActive() {
    return state.recordWalk && state.brainConnected && state.guidanceMode === "robot";
  }

  // Phase T3 of PLAN-teleop-robot.md: Robot view pushes frames into a real
  // MissionRunner mission instead of calling /navigate itself. Same gating
  // shape as recordingActive() -- both need a brain and Robot view.
  //
  // **They used to be mutually exclusive, and that was removed 2026-09-08.**
  // The reason given was "recording saves a /navigate reply per frame, which
  // driving via brain never produces", which was true when T3 shipped and
  // stopped being true at T4: missionStatusToRobotResult() returns the whole
  // mission status under `_missionStatus`, carrying the action, the model's
  // reasoning, the perception tri-state, the tier counters and (since P2)
  // 1.11a's corroboration verdict. That is strictly MORE than a /navigate
  // reply, not less.
  //
  // Left alone, the exclusion had become exactly backwards: the tiered walk
  // is the only path where YOLO and CLIP ever see real pixels (4.10 -- the
  // sim cannot test the detector by 1.12's design), so it is the one walk
  // most worth keeping, and it was the one walk the twin refused to record.
  // It cost a rig session before anyone noticed.
  function driveViaBrainActive() {
    return state.driveViaBrain && state.brainConnected && state.guidanceMode === "robot";
  }

  function beginWalkRecording() {
    // Any change of walk invalidates every frame still in flight under the
    // previous one, whether or not a new walk starts. Bumped here and in
    // endWalkRecording(), and captured at dispatch by guidanceStep().
    state.recordEpoch++;
    if (!recordingActive()) {
      state.recordWalkName = null;
      // Silence here costs a whole walk. The toggle stays on, every frame
      // is dropped by recordWalkFrame's own early return, and the first
      // sign of trouble is an empty admin console afterwards -- by which
      // point the walk is gone. The Settings row explains the requirement,
      // but nobody is looking at Settings at the moment they press Start.
      if (state.recordWalk && state.guidanceMode === "robot" && !state.brainConnected) {
        showToast("Not recording: the brain service isn't connected.", "err");
      }
      return;
    }
    state.recordWalkName = newWalkName();
    state.recordSaved = 0;
    state.recordFailed = 0;
    state.recordOrphaned = 0;
    state.recordSeq = 0;
    showToast("Recording this walk as " + state.recordWalkName, "info");
    renderRecordStatus();
  }

  // Fire-and-forget: a failed save must never interrupt a walk, and must
  // never delay the next /navigate call. The count is the feedback.
  // `seq` is allocated by the caller at dispatch, NOT derived here from
  // recordSaved/recordFailed. Those increment when this POST resolves, so
  // once vision calls overlap two frames can both read the same value
  // before either save returns -- and control/brain_server.py writes
  // frame-{seq:04d}.jpg with write_bytes(), which overwrites in silence
  // while walk.jsonl still gains two rows for the one seq. The walk then
  // replays a frame short, with a manifest that disagrees with the
  // directory. Dispatch order is also capture order, which is the order
  // sim/replay_robot.py plays a walk back in.
  function recordWalkFrame(base64, navigateResult, seq, epoch, teleopSeq) {
    // Checked before anything else, so an orphan is counted rather than
    // falling out of one of the looser guards below unnoticed.
    // The walk this frame was dispatched under has ended, or been replaced.
    // Writing it now would file it under whatever walk is current, carrying
    // the OLD walk's seq and the old walk's prompt wording -- which is
    // exactly what happened to
    // bottle-opus-4-5-center-third-path-20260902-163923 (16
    // center-third-path frames and one bearing-only at seq 37), and it makes
    // any walk recorded straight after another unattributable.
    //
    // guidanceEpoch cannot cover this: it tracks a Guide RUN, and a walk is
    // a different lifetime -- beginWalkRecording() can rename the walk with
    // the run's epoch unchanged. Same fix one layer down, its own counter.
    //
    // Dropped rather than filed under the old name: endWalkRecording() has
    // already posted /recording/finish for it, so a late frame would leave
    // walk.jsonl disagreeing with a walk the admin console has scored.
    // Counted, because a silently thinned recording is what this whole
    // class of bug looks like from the outside.
    if (epoch !== undefined && epoch !== state.recordEpoch) {
      state.recordOrphaned += 1;
      renderRecordStatus();
      return;
    }
    if (!recordingActive() || !state.recordWalkName) return;
    // Captured before recording began: there is no slot reserved for it.
    if (seq === null || seq === undefined) return;
    brainApi("POST", "/recording/frame", {
      walk: state.recordWalkName,
      seq: seq,
      image_base64: base64,
      media_type: "image/jpeg",
      navigate: navigateResult || null,
      // The robot's id for THIS frame. Pairs with the mission status's
      // last_frame_seq to say which decision actually saw these pixels.
      // Null outside teleop, where there is nothing to align.
      teleop_seq: teleopSeq,
    }).then(function () {
      state.recordSaved += 1;
      renderRecordStatus();
    }).catch(function (e) {
      state.recordFailed += 1;
      if (state.recordFailed === 1) showToast("Frame not saved: " + e.message, "err");
      renderRecordStatus();
    });
  }

  function endWalkRecording() {
    if (!state.recordWalkName) return;
    const name = state.recordWalkName, saved = state.recordSaved;
    state.recordWalkName = null;
    // A frame whose /navigate answer lands after this must not be written
    // into whatever walk is current by then -- see recordWalkFrame().
    state.recordEpoch++;
    renderRecordStatus();
    if (saved > 0) {
      // Tell the brain this walk is complete, and hand over the two facts
      // the frames can't carry on their own: which model answered, and what
      // was being searched for. Without this a walk is only "finished" in
      // the sense that no more frames arrived, and the admin console can't
      // tell a completed walk from one still in progress. Fire-and-forget
      // and best-effort, exactly like recordWalkFrame() -- a closed tab
      // never sends it, which is why admin also scores lazily on first read.
      brainApi("POST", "/recording/finish", {
        walk: name,
        model_id: state.navigateModelId || null,
        target_object: state.guidanceTarget || null,
        // What the frames were actually captured at. Read off the canvas
        // that encoded them rather than off the constraint we asked for --
        // getUserMedia's `ideal` is a request, not a promise, and a phone
        // that could only manage 640 must say so in the walk rather than
        // leave the next reader to infer it from the plan.
        capture_width: guidanceCaptureCanvas.width || null,
        capture_height: guidanceCaptureCanvas.height || null,
      }).catch(function () { /* admin's lazy path covers this */ });

      showToast(saved + " frames saved as " + name + " -- replay it with " +
        "python -m tests.demo_replay_mission", "ok");
    }
  }

  function renderRecordStatus() {
    const sub = document.getElementById("record-walk-sub");
    if (!sub) return;
    if (!state.brainConnected) {
      sub.textContent = "Needs the brain service connected \u2014 set it up in Settings.";
      return;
    }
    if (state.recordWalkName) {
      sub.textContent = "Recording " + state.recordWalkName + " \u2014 " +
        state.recordSaved + " frames saved" +
        (state.recordFailed ? ", " + state.recordFailed + " failed" : "") +
        (state.recordOrphaned ? ", " + state.recordOrphaned + " from a previous walk dropped" : "") + ".";
      return;
    }
    sub.textContent = "Saves each frame to the brain service, so the same walk " +
      "can be replayed through the Python agent as many times as you like.";
  }

  function updateRecordRow() {
    const row = document.getElementById("record-walk-row");
    if (row) row.style.display = state.guidanceMode === "robot" ? "" : "none";
    renderRecordStatus();
  }

  // ---------- /navigate model picker (price/performance A/B) ----------
  // Robot view is the flow that already produces something comparable
  // (a recorded walk, judged the way Stage 0's go/no-go gate judges any
  // walk) -- picking a model here rather than in Settings ties the choice
  // to the specific walk it will be recorded under, not a standing
  // override that's easy to forget is set. The allow-list itself lives on
  // the server (vision_core.NAVIGATE_MODEL_CHOICES) so this page never
  // hardcodes ids that would drift as that list changes.
  function navigatePickersWanted() {
    // Two consumers now: Robot view's own /navigate calls, and the Sim tab's
    // remote brain when it is set to the vision policy. Both send the same
    // two fields to the same allow-list, so they share one pair of pickers
    // rather than growing a second, driftable copy in the Sim tab.
    // The tiered policy is the third consumer. It calls /navigate far less
    // often -- that is its whole point -- but "less often" is not "never",
    // and a walk nobody can attribute to a model and a wording is not a
    // measurement whether it cost one call or forty.
    return state.guidanceMode === "robot" || isCloudPolicy(state.brainPolicy);
  }

  function updateModelPickerRow() {
    const wanted = navigatePickersWanted();
    const row = document.getElementById("navigate-model-row");
    if (row) row.style.display = wanted ? "" : "none";
    const promptRow = document.getElementById("navigate-prompt-row");
    // Only shown once the service offers more than one wording -- a single
    // variant is not a choice.
    if (promptRow) {
      promptRow.style.display =
        (wanted && promptRow.dataset.hasChoices === "1") ? "" : "none";
    }
    if (wanted && !state.navigateModelsLoaded) fetchNavigateModels();
  }

  // Drops every option but "Service default", which is index 0 and is the
  // page's own (it means "send no model_id at all"), never the service's.
  function resetNavigateModelOptions() {
    const select = document.getElementById("cfg-navigate-model");
    if (!select) return;
    while (select.options.length > 1) select.remove(1);
  }

  function populatePromptOptions(prompts, servicedefault) {
    const select = document.getElementById("cfg-navigate-prompt");
    const row = document.getElementById("navigate-prompt-row");
    if (!select || !row) return;
    while (select.options.length > 1) select.remove(1);
    // Same trap as the model picker: "Service default" and the named variant
    // it resolves to are the same prompt, and a walk recorded under each is
    // identical apart from what the log says produced it.
    if (select.options[0] && servicedefault) {
      select.options[0].textContent = "Service default \u2014 " + servicedefault;
    }
    row.dataset.hasChoices = prompts.length > 1 ? "1" : "0";
    for (const name of prompts) {
      const opt = document.createElement("option");
      opt.value = name;
      opt.textContent = name;
      select.appendChild(opt);
    }
    const saved = prefGet(PREF.navigatePromptVariant);
    if (saved && prompts.indexOf(saved) !== -1) {
      select.value = saved;
      state.navigatePromptVariant = saved;
    }
    updateModelPickerRow();
  }

  function fetchNavigateModels() {
    const url = document.getElementById("cfg-url").value.trim();
    if (!url) return;
    if (state.navigateModelsLoaded) return;
    fetch(deriveServiceUrl(url, "/navigate/models"))
      .then(function (resp) {
        if (!resp.ok) throw new Error("HTTP " + resp.status);
        return resp.json();
      })
      .then(function (data) {
        const select = document.getElementById("cfg-navigate-model");
        if (!select || !Array.isArray(data.models)) return;
        // Only latch on success. Setting this before the request meant a
        // fetch that bailed (an empty URL during init, an offline service)
        // permanently left the picker with nothing in it, since the only
        // other caller is a mode switch the user has no reason to make.
        state.navigateModelsLoaded = true;
        state.navigateServiceDefault = data.default || "";
        populatePromptOptions(data.prompts || [], data.default_prompt || "");
        resetNavigateModelOptions();
        for (const m of data.models) {
          const opt = document.createElement("option");
          opt.value = m.id;
          opt.textContent = m.label || m.id;
          select.appendChild(opt);
        }
        // "Service default" and an explicit pick of the SAME model are not
        // distinguishable in the UI otherwise -- they produce identical
        // walks, and the only difference is whether model_id is sent and
        // recorded. Naming what it resolves to makes the choice honest.
        const placeholder = select.options[0];
        if (placeholder && data.default) {
          placeholder.textContent = "Service default \u2014 " + shortModelName(data.default);
        }
        const saved = prefGet(PREF.navigateModelId);
        if (saved && data.models.some(function (m) { return m.id === saved; })) {
          select.value = saved;
          state.navigateModelId = saved;
        }
        // Both readouts name the service's own defaults when nothing is
        // picked, and until this reply lands there is no name to give. They
        // are rendered synchronously by whatever revealed them, so they have
        // to be redrawn once the answer arrives -- otherwise the panel says
        // "the service default" forever.
        renderDriveViaBrainStatus();
        renderBrainPolicyHint();
      })
      .catch(function () { /* leave "Service default" as the only option; a later call retries */ });
  }

  document.getElementById("cfg-navigate-prompt").addEventListener("change", function () {
    state.navigatePromptVariant = this.value;
    prefSet(PREF.navigatePromptVariant, this.value);
    renderBrainPolicyHint();
  });

  document.getElementById("cfg-navigate-model").addEventListener("change", function () {
    state.navigateModelId = this.value;
    prefSet(PREF.navigateModelId, this.value);
    renderDriveViaBrainStatus();
    renderBrainPolicyHint();
  });

  document.getElementById("cfg-record-walk").addEventListener("change", function () {
    state.recordWalk = this.checked;
    prefSet(PREF.recordWalk, this.checked ? "1" : "0");
    renderRecordStatus();
  });

  // Which model a brain-driven mission would actually run, in words.
  // Precedence mirrors the server's: the picker's choice (sent as model_id
  // on /mission/start) beats the brain's pinned config, which beats the
  // vision service default. Shown rather than inferred on purpose -- the
  // one real failure this project hit was a week of recorded walks
  // attributed to a model that was never running.
  // A model id trimmed to something readable in a dropdown.
  function shortModelName(id) {
    return String(id || "")
      .replace(/^(us|global)\./, "")
      .replace(/^anthropic\.claude-/, "")
      .replace(/-\d{8}-v\d+:\d+$/, "")
      .replace(/^(amazon|openai|qwen)\./, "")
      .replace(/-v\d+:\d+$/, "");
  }

  function brainModelLabel() {
    const labelFor = function (id) {
      const select = document.getElementById("cfg-navigate-model");
      if (select) {
        for (const opt of select.options) {
          if (opt.value === id) return opt.textContent;
        }
      }
      return id;
    };
    if (state.navigateModelId) return labelFor(state.navigateModelId) + " (your pick)";
    if (state.brainNavigateModelId) return labelFor(state.brainNavigateModelId) + " (pinned on the brain)";
    if (state.navigateServiceDefault) return labelFor(state.navigateServiceDefault) + " (service default)";
    return "the service default";
  }

  // The wording axis, resolved the same way and for the same reason. The
  // failure this guards against is not hypothetical: a week of walks were
  // attributed to a model that was never running, because a value was passed
  // and something else was served.
  function brainPromptLabel() {
    if (state.navigatePromptVariant) return state.navigatePromptVariant + " (your pick)";
    if (state.brainNavigatePromptVariant) {
      return state.brainNavigatePromptVariant + " (pinned on the brain)";
    }
    const select = document.getElementById("cfg-navigate-prompt");
    const placeholder = select && select.options[0];
    // "Service default -- <name>", set by populatePromptOptions() once the
    // service has told us what that name is.
    if (placeholder && placeholder.textContent.indexOf("\u2014") !== -1) {
      return placeholder.textContent.split("\u2014")[1].trim() + " (service default)";
    }
    return "the service default";
  }

  function renderDriveViaBrainStatus() {
    const sub = document.getElementById("drive-via-brain-sub");
    if (!sub) return;
    if (!state.brainConnected) {
      sub.textContent = "Needs the brain service connected — set it up in Settings.";
      return;
    }
    let text = "Runs a real mission on the brain service (PLAN-teleop-robot.md) "
      + "instead of a one-off /navigate call -- mission memory, the step "
      + "budget and the failsafes all apply to your walk. "
      + "Would run: " + brainModelLabel() + ".";
    if (state.drivePolicy === "tiered") {
      const p = state.brainPerception;
      text += p
        ? " Perception runs in the brain process on your frames: "
          + p.detector + " + " + p.clip + "."
        : " Perception runs in the brain process on your frames.";
      if (p && p.available === false) {
        text += " This brain has no perception models installed, so the walk "
          + "will refuse to start — run pip install -r "
          + "requirements-perception.txt where the brain runs.";
      }
    }
    sub.textContent = text;
  }

  function updateDriveViaBrainRow() {
    const wanted = state.guidanceMode === "robot";
    const row = document.getElementById("drive-via-brain-row");
    if (row) row.style.display = wanted ? "" : "none";
    // The policy only means anything once the walk is actually being driven
    // by a mission -- a one-off /navigate call has no policy at all.
    const policyRow = document.getElementById("drive-policy-row");
    if (policyRow) {
      policyRow.style.display = (wanted && state.driveViaBrain) ? "" : "none";
    }
    renderDriveViaBrainStatus();
  }

  document.getElementById("cfg-drive-via-brain").addEventListener("change", function () {
    state.driveViaBrain = this.checked;
    prefSet(PREF.driveViaBrain, this.checked ? "1" : "0");
    updateDriveViaBrainRow();
    // Recording stays on if it was on: a tiered walk is the most worth
    // keeping, not the least. See driveViaBrainActive().
    renderRecordStatus();
  });

  document.getElementById("cfg-drive-policy").addEventListener("change", function () {
    state.drivePolicy = this.value;
    prefSet(PREF.drivePolicy, this.value);
    // The tiered policy has a second pair of models to name, and the row
    // above is where the walk's operator reads what they are about to run.
    renderDriveViaBrainStatus();
  });

  // ---------- AR camera guidance: real phone camera, human is the actuator ----------
  // The inverse of Vision Autopilot -- here the *person* holding the phone
  // walks around, and Claude Vision (via the vision service's /guidance
  // route, a sibling of /navigate with a 5-zone position + proximity +
  // human-readable instruction + an optional bounding box instead of a
  // discrete robot action) tells them which way to turn. Never touches
  // robot/server.py -- there's nothing to drive, just a live camera feed
  // and an overlay. Requires a secure context (https:// or localhost) for
  // getUserMedia -- see web-twin/README.md.
  //
  // Full-screen is CSS-simulated (#guide-fullscreen, position:fixed over
  // everything) rather than the real Fullscreen API -- see
  // PLAN-ar-guidance.md for why (iOS Safari support for arbitrary-element
  // requestFullscreen() is unreliable). The overlay itself (chevron /
  // reticle / found-outline) is DOM+SVG with CSS transitions, not
  // canvas-drawn, specifically so state changes glide instead of
  // snapping -- canvas would need a hand-rolled animation loop to get the
  // same smoothness.

  // 500ms since /guidance moved to Nova Lite (~1.0-1.6s/call, measured
  // against a real photo) -- at Sonnet's ~2.5-3.4s/call this throttle was
  // noise next to the call itself; at Nova Lite's latency it's now a real
  // fraction of the cycle, so halving it actually tightens the loop
  // (~2.2s -> ~1.7s/cycle at the measured mean). /navigate moved to the
  // same model on 2026-08-28 (see vision_core.py's "Per-route models"
  // note -- a discussed trade-off, not an independently re-measured one),
  // so Robot view shares this throttle now too instead of pacing itself
  // to Sonnet's slower round trip.
  const GUIDANCE_THROTTLE_MS = 500;
  const ROBOT_THROTTLE_MS = 500;
  // How many vision calls may be in flight at once. The throttle above is
  // the gap between *dispatches*; the round trip is added on top, so with
  // a strictly serial loop the real cadence was throttle + latency --
  // ~3s once /navigate moved to Opus 4.5, not the 500ms the constant
  // suggests. Overlapping calls decouples the two: a decision arrives
  // every ~500ms, each still describing a frame from one round trip ago.
  //
  // **Pipelining raises throughput, not freshness.** No answer here is
  // any newer than it was before; there are simply more of them, which is
  // what a person walking can actually use -- they integrate across
  // several. A robot executing each one would be worse off, which is why
  // drive-via-brain is pinned to 1 below.
  //
  // 2, not more, and the number is measured rather than guessed:
  // control/walk_replay.py records that at 8 concurrent workers "Bedrock
  // throttled 10 of 22 frames" against a single vision task. Bedrock
  // quota is account-and-region scoped, so this contends with whatever
  // else is running in the account -- including production.
  const GUIDANCE_MAX_IN_FLIGHT = 2;
  // Anthropic downscales images server-side above ~1568px on the long
  // ONE size for every consumer -- the cloud call, the perception tier and
  // the recorded walk all get the same pixels.
  //
  // **The entire Stage 0 corpus was captured at 640x480 and nobody noticed
  // until 2026-09-08**, because getUserMedia below was called with no
  // resolution constraint and the browser handed back its default. The 960
  // ceiling never even engaged -- min(1, 960/640) is 1 -- so it was capping
  // nothing, and the walk recorder saves the same base64 the cloud call
  // gets, so no larger copy ever existed. Every finding about small distant
  // targets in PLAN-onboard-perception.md 4.10/4.11 rests on VGA frames,
  // including the search walk where the local tier scores 2 of 10.
  //
  // 1280, for three reasons that agree:
  //
  //   * it is the largest input YOLO11s clears camera rate at on a Hailo-8L
  //     (4.3.1's 92 FPS at 640, and compute scales with pixel count), so a
  //     corpus above it would describe a robot this one is not;
  //   * it is under Anthropic's ~1568 downscale threshold, so the cloud
  //     actually uses every pixel rather than resizing them away;
  //   * and it keeps the corpus IDENTICAL to what the cloud was asked, which
  //     is what makes control/walk_replay.py a reproduction rather than a
  //     different experiment. A twin that recorded 1280 while asking the
  //     cloud at 960 would put every replayed walk quietly out of step with
  //     the live one it claims to re-run -- the same class of mismatch that
  //     produced a week of walks attributed to a model that was never
  //     running.
  //
  // The cost of carrying the cloud at 1280 rather than 960 is about +540
  // input tokens per call (images tokenize near w*h/750), or ~19k tokens
  // across a 209-frame search walk's paid calls. That is the price of the
  // corpus and the replay agreeing, and it is worth paying.
  const CAPTURE_MAX_DIM = 1280;
  const guidanceVideo = document.getElementById("guidance-video");
  const guidanceFullscreen = document.getElementById("guide-fullscreen");
  const guidanceChevron = document.getElementById("guide-chevron");
  const guidanceOutline = document.getElementById("guide-outline");
  const guidanceFoundBadge = document.getElementById("guide-found-badge");
  const guidanceEdgeGlowLeft = document.getElementById("guide-edge-glow-left");
  const guidanceEdgeGlowRight = document.getElementById("guide-edge-glow-right");
  const guidanceCaptureCanvas = document.createElement("canvas");

  // Downscales to CAPTURE_MAX_DIM on the long edge before encoding. One
  // frame, one size, every consumer -- see CAPTURE_MAX_DIM above for why the
  // cloud is no longer given a smaller copy than the corpus keeps.
  function captureGuidanceFrame() {
    const nativeW = guidanceVideo.videoWidth, nativeH = guidanceVideo.videoHeight;
    const scale = Math.min(1, CAPTURE_MAX_DIM / Math.max(nativeW, nativeH));
    guidanceCaptureCanvas.width = Math.round(nativeW * scale);
    guidanceCaptureCanvas.height = Math.round(nativeH * scale);
    guidanceCaptureCanvas.getContext("2d").drawImage(
      guidanceVideo, 0, 0,
      guidanceCaptureCanvas.width, guidanceCaptureCanvas.height);
    return guidanceCaptureCanvas.toDataURL("image/jpeg", 0.8).split(",")[1];
  }



  function callGuidanceEndpoint(base64, targetObject) {
    const url = document.getElementById("cfg-url").value.trim();
    const secret = document.getElementById("cfg-secret").value.trim();
    if (!url) throw new Error('Set the vision service URL in "Cloud endpoint settings" above first.');
    const headers = { "Content-Type": "application/json" };
    if (secret) headers["x-app-secret"] = secret;
    // Both routes take the identical request body (same _decode_image() on
    // the server) -- only the prompt, the response schema and the renderer
    // differ. That is what makes robot view a mode here rather than a
    // second camera implementation.
    const route = state.guidanceMode === "robot" ? "/navigate" : "/guidance";
    const payload = { image_base64: base64, media_type: "image/jpeg", target_object: targetObject };
    // Model A/B only applies to /navigate (Robot view) -- /guidance is a
    // person-facing feature, not the evaluation flow this picker is for.
    if (route === "/navigate" && state.navigateModelId) payload.model_id = state.navigateModelId;
    if (route === "/navigate" && state.navigatePromptVariant) {
      payload.prompt_variant = state.navigatePromptVariant;
    }
    return fetch(deriveServiceUrl(url, route), {
      method: "POST",
      headers: headers,
      body: JSON.stringify(payload),
    }).then(function (resp) {
      return resp.json().then(function (data) {
        if (!resp.ok) {
          let message = errorDetail(data) || ("HTTP " + resp.status);
          // The one failure that is routinely misdiagnosed. A rejected
          // secret says nothing about *which* service rejected it, and
          // when the page and the service are different deployments the
          // secret is usually right and pointed at the wrong one.
          if (resp.status === 401 && visionHostDiffersFromPage()) {
            message += " (sent to " + visionHost() + ", but this page came from "
              + location.host + " -- if those are different deployments, "
              + "this is their secret, not a wrong one)";
          }
          throw new Error(message);
        }
        return data;
      });
    });
  }

  // ---- drive via brain (Phase T3, PLAN-teleop-robot.md) ----
  // The alternative to callGuidanceEndpoint() above when
  // driveViaBrainActive(): instead of a one-off /navigate call, each frame
  // is pushed into sim/teleop_robot.py's TeleopRobot (via robot/server.py,
  // the same server the D-pad and Sim tab already use) and the decision is
  // read back from control/brain_server.py's real MissionRunner -- the
  // exact loop a PiCar will run, not a simulation of it.

  // Returns the robot's own sequence number for this frame. sim/teleop_robot.py
  // stamps one and echoes it back, and MissionRunner now reports which id its
  // last decision was made on -- so a recorded walk can align decisions to
  // pixels exactly. Without both halves the pairing is wall-clock coincidence:
  // the mission ticks once per ~2.5 pushed frames (range 1-10), so the status
  // saved beside a frame usually describes an earlier one.
  function pushTeleopFrame(base64) {
    return apiPost("/teleop/frame", { image_base64: base64, media_type: "image/jpeg" })
      .then(function (res) {
        state.lastTeleopSeq = (res && typeof res.seq === "number") ? res.seq : null;
        return res;
      });
  }

  // Reshapes a GET /mission/status payload into the {action, reasoning,
  // target_reached, target_visible, target_direction, obstacle_ahead, error}
  // shape callGuidanceEndpoint()'s /navigate reply has -- so
  // renderRobotOverlay/renderRobotStatus/renderRobotTelemetry/announceRobot
  // and guidanceStep()'s found-streak-to-pause logic all keep working
  // unchanged, whichever source produced the "result". Not a perfect
  // translation: MissionRunner records a *decision*, not the per-frame
  // zone/obstacle facts that produced it, so target_visible/target_direction/
  // obstacle_ahead have nothing honest to report and stay unset. The raw
  // status rides along as _missionStatus for the fields worth showing that
  // /navigate never had (outcome, vision_failures) -- see
  // renderRobotTelemetry.
  function missionStatusToRobotResult(status) {
    const arrived = status.outcome === "found" || status.outcome === "room_reached";
    if (status.outcome === "arrived_unconfirmed") {
      // 3.53: an outcome, not a lost link. No `action` -- nothing decided a
      // move, and a recorded walk files it like an error frame (action
      // null), so walk_eval's counts do not change.
      return { action: null, unconfirmed: true, reasoning: unconfirmedText(status),
               target_reached: false, target_visible: null, target_direction: null,
               obstacle_ahead: false, _missionStatus: status };
    }
    if (!status.running && !arrived) {
      // Mission ended without finding the target (stopped/failed/max_steps).
      // Reported as an error so the existing error branches (caption,
      // telemetry, the screen-reader announcement) show it exactly like a
      // lost /navigate connection would -- including a live B3.2/T1-stall
      // failsafe firing.
      return { error: status.error || ("mission ended: " + status.outcome), _missionStatus: status };
    }
    // `target_visible` was hardcoded `false` here, which was harmless while
    // these results were only drawn on screen and became misleading the
    // moment they started being RECORDED (2026-09-08): a walk.jsonl full of
    // "target_visible: false" reads as a claim the robot made, and this
    // project has twice been burned by trusting a walk's own log. Report the
    // local tier's actual tri-state when there is one, and leave it null --
    // not false -- when there is not.
    const perception = status.perception;
    return {
      action: status.last_action,
      reasoning: status.last_reasoning || "",
      target_reached: arrived,
      target_visible: perception ? perception.status === "detected" : null,
      target_direction: null,
      obstacle_ahead: false,
      _missionStatus: status,
    };
  }

  async function driveViaBrainStep(base64) {
    await pushTeleopFrame(base64);
    const status = await brainApi("GET", "/mission/status");
    if (!status.running) {
      // The mission ended (found, room_reached, stopped, failed or
      // max_steps) -- nothing left to push frames into. Reuses the same
      // pause the found-streak logic already drives, rather than a second
      // stop mechanism; "Resume searching" starts a fresh call loop, not a
      // fresh mission, so this stays a dead end until Stop/Start.
      pauseGuidanceSearch();
      watchLateConfirmation(status);
    }
    return missionStatusToRobotResult(status);
  }

  // ---- overlay rendering: DOM/SVG elements positioned via CSS, not canvas ----

  const GUIDANCE_ZONE_OFFSET = { far_left: -34, left: -18, center: 0, right: 18, far_right: 34 };
  const GUIDANCE_ZONE_ROTATION = { far_left: 180, left: 180, center: -90, right: 0, far_right: 0 };
  const GUIDANCE_ZONE_SCALE = { far_left: 1.3, left: 1.0, center: 1.1, right: 1.0, far_right: 1.3 };

  // The outline's arrival animation must fire on the transition into
  // found, not on every tick that stays found -- .visible is removed and
  // re-added each tick by hideAllGuidanceOverlays(), which would otherwise
  // restart the animation about once a second.
  let guidanceWasFound = false;

  const robotActionBadge = document.getElementById("robot-action-badge");
  const robotZones = document.getElementById("robot-zones");
  const robotZoneEls = Array.prototype.slice.call(robotZones.querySelectorAll(".robot-zone"));

  // The zone frame itself stays up for the whole robot-view session -- it is
  // the mode's identity, not a per-tick result -- so it is deliberately not
  // cleared by hideAllGuidanceOverlays(). Only the highlight changes per tick.
  const robotHud = document.getElementById("robot-hud");
  const robotScanline = document.getElementById("robot-scanline");
  const robotCountdown = document.getElementById("robot-countdown");

  function setRobotZonesVisible(visible) {
    robotZones.classList.toggle("visible", !!visible);
    robotHud.classList.toggle("visible", !!visible);
    if (!visible) {
      highlightRobotZone(null, false);
      document.getElementById("robot-obstacle").classList.remove("visible");
      document.getElementById("robot-telemetry").innerHTML = "";
      robotCountdown.classList.remove("run");
    }
  }

  // Restarting a CSS animation needs the class removed, a reflow, then the
  // class re-added -- doing both in one frame is a no-op. Same trick the
  // found-outline's arrive animation uses.
  function restartRobotAnimation(el, cls) {
    el.classList.remove(cls);
    void el.offsetWidth;
    el.classList.add(cls);
  }

  // Fired at the moment a frame is actually captured and sent, so the sweep
  // marks a real event rather than running on a decorative timer. Runs in
  // both modes -- the scanline element itself lives outside #robot-hud now,
  // so there's no mode gate here to match.
  function robotSignalCapture() {
    restartRobotAnimation(robotScanline, "sweep");
  }

  // Runs for exactly the delay the next call is scheduled at, including the
  // exponential backoff after an error -- so a stalled loop looks stalled.
  function robotStartCountdown(ms) {
    if (state.guidanceMode !== "robot") return;
    robotCountdown.firstElementChild.style.animationDuration = ms + "ms";
    restartRobotAnimation(robotCountdown, "run");
  }

  function renderRobotTelemetry(result) {
    const el = document.getElementById("robot-telemetry");
    if (!result || result.error) {
      el.innerHTML = "<div><b>ERR</b><span class=\"v-alert\">LINK</span></div>";
      return;
    }
    // Driving via brain (Phase T3): missionStatusToRobotResult() has no
    // per-frame zone/obstacle facts to report -- MissionRunner records a
    // decision, not the reading that produced it -- so show that honestly
    // as N/A rather than a false CLEAR/NO that looks like a confirmed read.
    const mission = result._missionStatus;
    const vis = mission ? "N/A" : result.target_visible
      ? "YES &middot; " + String(result.target_direction || "?").replace("_", " ")
      : "NO";
    const obs = mission ? "N/A" : result.obstacle_ahead
      ? '<span class="v-alert">BLOCKED</span>' : '<span class="v-safe">CLEAR</span>';
    const reached = result.target_reached === true
      ? '<div><b>STA</b><span class="v-safe">REACHED</span></div>'
      : result.unconfirmed
        ? '<div><b>STA</b><span class="v-warn">UNCONFIRMED</span></div>' +
          (mission && mission.late_confirmation
            ? '<div><b>LATE</b><span class="' +
              (mission.late_confirmation.state === "confirmed" ? "v-safe" : "v-warn") + '">' +
              escapeHtml(String(mission.late_confirmation.state).toUpperCase()) + "</span></div>"
            : "")
        : "";
    // Fields /navigate never had: MissionRunner's own outcome and the B3.2
    // failure count, so a live failsafe firing is visible here too.
    const brainRow = mission
      ? "<div><b>OUT</b>" + escapeHtml(mission.outcome || "running") + "</div>" +
        "<div><b>VF</b>" + String(mission.vision_failures || 0) +
        (mission.vision_failures ? '<span class="v-alert"> !</span>' : "") + "</div>"
      : "";
    // 3.56 follow-up (the user): the cloud row on the Guide HUD too, in the
    // panel's own words (cloudWords: unknown, stale "last seen ...", live).
    const cw = mission && ("cloud" in mission) ? cloudWords(mission.cloud) : null;
    const cloudRow = cw && cw.text
      ? '<div id="robot-tel-cloud"><b>CLD</b><span class="' +
        (cw.cls === "safe" ? "v-safe" : cw.cls === "warn" ? "v-warn" : "") + '">' +
        escapeHtml(cw.text) + "</span></div>"
      : "";
    el.innerHTML =
      "<div><b>TGT</b>" + escapeHtml(state.guidanceTarget || "--") + "</div>" +
      "<div><b>ACT</b>" + escapeHtml(result.action || "--") + "</div>" +
      "<div><b>VIS</b>" + vis + "</div>" +
      "<div><b>OBS</b>" + obs + "</div>" +
      "<div><b>SEQ</b>" + String(state.guidanceCallCount).padStart(3, "0") +
        "/" + String(guidanceCallCap()).padStart(3, "0") + "</div>" +
      brainRow + cloudRow + reached;
    document.getElementById("robot-obstacle")
      .classList.toggle("visible", result.obstacle_ahead === true);
  }

  function highlightRobotZone(zone, reached) {
    robotZoneEls.forEach(function (el) {
      const isIt = el.dataset.zone === zone;
      el.classList.toggle("active", isIt && !reached);
      el.classList.toggle("reached", isIt && !!reached);
    });
  }

  function hideAllGuidanceOverlays() {
    guidanceChevron.classList.remove("visible");
    guidanceOutline.classList.remove("visible");
    guidanceFoundBadge.classList.remove("visible");
    guidanceEdgeGlowLeft.style.opacity = 0;
    guidanceEdgeGlowRight.style.opacity = 0;
    guidanceChevron.classList.remove("steady");
    guidanceOutline.classList.remove("steady");
    robotActionBadge.classList.remove("visible");
  }

  // "Found" only required a strict position===center AND proximity===near
  // AND-gate originally, matching the old checkmark design where there was
  // no way to show *where* the object was, only a binary yes/no. Now that
  // the outline shows the object's actual position via bounding_box, that
  // strict gate is redundant and, in real-world testing, too narrow to
  // reliably hit on any single ~2s tick -- an object can be close
  // (proximity: near) without being framed dead-center, so "near" alone
  // triggers found regardless of position.
  //
  // Also accept position===center && proximity===medium: after tightening
  // GUIDANCE_PROMPT_TEMPLATE's "near" calibration (it was firing on
  // almost anything visible), real-device testing showed the opposite
  // problem -- a centered, clearly-arrived-at object sometimes gets
  // classified "medium" and got stuck showing a chevron forever, never
  // reaching the outline. Being dead-center already tells us the user
  // aimed the camera right at it; "medium" close plus centered is a
  // reasonable second definition of "found" without loosening the
  // proximity calibration itself back to over-eager.
  function isGuidanceFound(result) {
    if (!result.target_visible) return false;
    if (result.proximity === "near") return true;
    if (result.proximity === "medium" && result.position === "center") return true;
    return false;
  }

  // Each analysis tick is a completely independent vision call with no
  // memory of the previous one -- the model re-locates the object from
  // scratch every ~1s, so its exact bounding-box estimate can legitimately
  // vary tick to tick even for a stationary object (LLM vision estimation
  // noise, not a rendering bug). Blend consecutive found-state boxes
  // (in normalized 0-1 space, so this stays correct across orientation
  // changes) instead of snapping straight to each new raw estimate --
  // damps jitter while staying responsive to genuine movement. Reset
  // whenever the found state is exited so a stale box never drags a
  // freshly-reacquired detection toward an unrelated old position.
  let guidanceLastNormBox = null;
  function smoothGuidanceBox(newBox) {
    if (!guidanceLastNormBox) { guidanceLastNormBox = newBox; return newBox; }
    const alpha = 0.55; // weight toward the new estimate
    const blended = {
      x_min: guidanceLastNormBox.x_min + (newBox.x_min - guidanceLastNormBox.x_min) * alpha,
      y_min: guidanceLastNormBox.y_min + (newBox.y_min - guidanceLastNormBox.y_min) * alpha,
      x_max: guidanceLastNormBox.x_max + (newBox.x_max - guidanceLastNormBox.x_max) * alpha,
      y_max: guidanceLastNormBox.y_max + (newBox.y_max - guidanceLastNormBox.y_max) * alpha,
    };
    guidanceLastNormBox = blended;
    return blended;
  }

  // Maps a (possibly smoothed) bounding_box, normalized 0-1 against the
  // FULL, UNCROPPED captured frame -- see captureGuidanceFrame, which
  // always sends the whole frame -- onto on-screen pixel coordinates
  // within the <video> element's displayed (and CSS
  // object-fit:cover-CROPPED) box.
  // Naively multiplying by the displayed rect's width/height is wrong
  // whenever the camera's native aspect ratio differs from the screen's
  // (true almost all the time in portrait) -- `cover` scales the video up
  // until it fills the container in both dimensions, then crops whichever
  // axis overflows, centered. This reproduces that same scale+crop so the
  // box lands exactly where the object actually is on screen.
  function mapNormalizedBoxToScreen(box) {
    const videoW = guidanceVideo.videoWidth, videoH = guidanceVideo.videoHeight;
    const rect = guidanceVideo.getBoundingClientRect();
    const scale = Math.max(rect.width / videoW, rect.height / videoH);
    const scaledW = videoW * scale, scaledH = videoH * scale;
    const offsetX = (scaledW - rect.width) / 2, offsetY = (scaledH - rect.height) / 2;
    return {
      left: box.x_min * scaledW - offsetX,
      top: box.y_min * scaledH - offsetY,
      width: (box.x_max - box.x_min) * scaledW,
      height: (box.y_max - box.y_min) * scaledH,
    };
  }

  const GUIDANCE_PULSE_DURATION = { far: "1.8s", medium: "1.15s", near: "0.65s", unknown: "1.8s" };
  const GUIDANCE_EDGE_GLOW = { far_left: 0.95, left: 0.45, center: 0, right: 0.45, far_right: 0.95 };

  // Systematic search sweep, shown while the target hasn't been spotted at
  // all (target_visible: false) -- the model has zero information about
  // where the object actually is in this state (it's not in the photo),
  // so any directional suggestion here is a fixed UI convention to
  // encourage full room coverage, not something derived from the vision
  // response. Alternates a horizontal sweep (right, with an edge-glow
  // assist reusing the same element the off-center chevron uses) with a
  // vertical sweep (down -- no edge-glow, no top/bottom glow element
  // exists) every GUIDANCE_SCAN_PHASE_TICKS ticks, so a user stuck
  // scanning one row of the room gets nudged to try a different height.
  const GUIDANCE_SCAN_PHASE_TICKS = 5; // ~5s per phase at the 1s throttle
  const GUIDANCE_SCAN_PHASES = [
    { rotation: 0, icon: "chevronRight", caption: "Pan right to search", edgeGlow: "right" },
    { rotation: 90, icon: "chevronDown", caption: "Tilt down, then keep panning", edgeGlow: null },
  ];

  // Real-device feedback (Esa/Joyce): a target centered in frame, then
  // panned past, made the sweep above keep going the *same* direction it
  // always starts in (right, then down) with no idea the target had just
  // been seen -- for a seated user, "tilt down" means pointing at your
  // own lap. Bridge the gap: remember the last edge zone (not center --
  // that tells us nothing directional) the target was actually seen at,
  // and for a short grace window after it's lost, point back that way
  // instead of restarting the generic room sweep from scratch. Falls
  // through to the normal sweep once the grace window expires, in case
  // the target moved and the last-seen direction is now stale.
  const GUIDANCE_LOST_GRACE_TICKS = 3; // ~5s at the 500ms throttle + Nova Lite's ~1.2s call
  let guidanceLastEdgePosition = null;
  const GUIDANCE_LOST_PHASES = {
    far_left: { rotation: 180, icon: "chevronLeft", caption: "Lost it — go back left", edgeGlow: "left" },
    left: { rotation: 180, icon: "chevronLeft", caption: "Lost it — go back left", edgeGlow: "left" },
    right: { rotation: 0, icon: "chevronRight", caption: "Lost it — go back right", edgeGlow: "right" },
    far_right: { rotation: 0, icon: "chevronRight", caption: "Lost it — go back right", edgeGlow: "right" },
  };
  function currentGuidanceScanPhase() {
    if (guidanceLastEdgePosition && state.guidanceNotVisibleStreak <= GUIDANCE_LOST_GRACE_TICKS) {
      return GUIDANCE_LOST_PHASES[guidanceLastEdgePosition];
    }
    const idx = Math.floor(state.guidanceNotVisibleStreak / GUIDANCE_SCAN_PHASE_TICKS) % GUIDANCE_SCAN_PHASES.length;
    return GUIDANCE_SCAN_PHASES[idx];
  }

  // Renders per the position table: far_left/left/right/far_right get a
  // progressively offset+scaled side chevron (plus a matching edge glow,
  // the primary glanceable "which way" signal), center-but-not-near gets
  // an up chevron, not_visible gets a spinning searching reticle, and
  // found (proximity: near) gets a glowing outline around the model's
  // bounding_box -- falling back to a centered pulse if the model didn't
  // return a confident box that tick (stay graceful rather than show a
  // wrong box). The chevron's pulse speed (CSS custom property, see
  // .guide-chevron-pulse) is the "getting warmer" signal, driven by
  // proximity independent of exact screen position.
  function renderGuidanceOverlay(result) {
    hideAllGuidanceOverlays();

    if (result.position === "not_visible" || !result.target_visible) {
      guidanceLastNormBox = null;
      state.guidanceNotVisibleStreak++;
      const scan = currentGuidanceScanPhase();
      guidanceChevron.style.left = "50%";
      guidanceChevron.style.top = "50%";
      guidanceChevron.style.transform = "translate(-50%, -50%) rotate(" + scan.rotation + "deg) scale(1.15)";
      guidanceChevron.style.color = getCss("--text");
      guidanceChevron.style.setProperty("--pulse-duration", GUIDANCE_PULSE_DURATION.unknown);
      // Read by the reduced-motion rules, which encode proximity as a
      // static size/brightness step instead of a pulse rate.
      guidanceChevron.dataset.proximity = "unknown";
      guidanceChevron.classList.add("visible");
      guidanceWasFound = false;
      if (scan.edgeGlow === "right") guidanceEdgeGlowRight.style.opacity = 0.5;
      else if (scan.edgeGlow === "left") guidanceEdgeGlowLeft.style.opacity = 0.5;
      return;
    }

    state.guidanceNotVisibleStreak = 0;

    if (isGuidanceFound(result)) {
      let box;
      if (result.bounding_box) {
        box = mapNormalizedBoxToScreen(smoothGuidanceBox(result.bounding_box));
      } else {
        // No confident box this tick -- a generic centered pulse rather
        // than a guessed rectangle, and don't let a future real box blend
        // across this gap from whatever the last real box happened to be.
        guidanceLastNormBox = null;
        const rect = guidanceVideo.getBoundingClientRect();
        const size = Math.min(rect.width, rect.height) * 0.35;
        box = { left: rect.width / 2 - size / 2, top: rect.height / 2 - size / 2, width: size, height: size };
      }
      guidanceOutline.style.left = box.left + "px";
      guidanceOutline.style.top = box.top + "px";
      guidanceOutline.style.width = box.width + "px";
      guidanceOutline.style.height = box.height + "px";
      guidanceFoundBadge.textContent = "Found — " + (state.guidanceTarget || "target");
      guidanceOutline.classList.add("visible");
      guidanceFoundBadge.classList.add("visible");
      if (!guidanceWasFound) {
        // Restart the animation from scratch: removing and re-adding the
        // class in one frame is a no-op, so force a reflow between them.
        guidanceOutline.classList.remove("arrive");
        void guidanceOutline.offsetWidth;
        guidanceOutline.classList.add("arrive");
      }
      guidanceWasFound = true;
      return;
    }

    guidanceLastNormBox = null; // left the found state -- don't blend across this gap either
    if (GUIDANCE_LOST_PHASES[result.position]) guidanceLastEdgePosition = result.position;
    const offset = GUIDANCE_ZONE_OFFSET[result.position] || 0;
    const rotation = GUIDANCE_ZONE_ROTATION[result.position] || -90;
    const scale = GUIDANCE_ZONE_SCALE[result.position] || 1;
    guidanceChevron.style.left = (50 + offset) + "%";
    guidanceChevron.style.top = "50%";
    guidanceChevron.style.transform = "translate(-50%, -50%) rotate(" + rotation + "deg) scale(" + scale + ")";
    guidanceChevron.style.color = result.proximity === "medium" ? getCss("--accent-safe") : getCss("--text");
    guidanceChevron.style.setProperty("--pulse-duration", GUIDANCE_PULSE_DURATION[result.proximity] || GUIDANCE_PULSE_DURATION.unknown);
    guidanceChevron.dataset.proximity = result.proximity || "unknown";
    guidanceChevron.classList.add("visible");
    guidanceWasFound = false;

    const glowSide = offset < 0 ? guidanceEdgeGlowLeft : offset > 0 ? guidanceEdgeGlowRight : null;
    if (glowSide) glowSide.style.opacity = GUIDANCE_EDGE_GLOW[result.position] || 0;
  }

  // ---- robot view: render /navigate's decision ----
  // The inverse of Guide's job. Guide answers "where is the object, for a
  // person walking"; this answers "what would the robot do next", which is
  // the same question web-twin's Vision Autopilot asks -- except the pixels
  // are a real room instead of the raycaster render. Nothing executes: a
  // photo of your kitchen has no corresponding grid-world state, and driving
  // the simulated robot from it would be meaningless.
  //
  // This is the only check available, without buying the hardware, that says
  // anything about the vision policy's accuracy on real rooms rather than on
  // flat-shaded maze geometry -- see PLAN-sim-hardening.md 3.5.
  const ROBOT_ACTION_ROTATION = { FORWARD: -90, LEFT: 180, RIGHT: 0, REVERSE: 90 };
  const ROBOT_ACTION_OFFSET = { FORWARD: 0, LEFT: -18, RIGHT: 18, REVERSE: 0 };
  const ROBOT_ACTION_ICON = {
    FORWARD: "chevronUp", LEFT: "chevronLeft", RIGHT: "chevronRight",
    REVERSE: "chevronDown", STOP: "warning",
  };
  // One fixed intensity, not a graded scale like GUIDANCE_EDGE_GLOW's --
  // target_direction only has three zones (no "far_left"-style outer
  // tier), so there is nothing to grade against.
  const ROBOT_EDGE_GLOW_OPACITY = 0.5;

  function renderRobotOverlay(result) {
    hideAllGuidanceOverlays();
    // An errored/ended-mission result (driveViaBrainStep's error branch, or
    // a lost /navigate connection) carries no `action` at all -- falling
    // through to the "unrecognized action" default below would paint a big
    // STOP badge that reads as a deliberate stop decision, when nothing
    // decided anything. The ERR/LINK telemetry chip and the caption text
    // already say what actually happened; the overlay just stays empty.
    if (result.error) return;
    // Same problem, different trigger: a "drive via brain" mission that IS
    // running fine but hasn't completed its first tick yet also has no
    // `action` -- MissionRunner's status starts with last_action unset,
    // not "STOP". Only skip when there's also no arrival to show, so a
    // reached-but-somehow-actionless result (shouldn't happen, but cheap to
    // guard) still gets its REACHED badge below.
    if (!result.action && !result.target_reached) return;
    const action = ROBOT_ACTION_ROTATION.hasOwnProperty(result.action) || result.action === "STOP"
      ? result.action : "STOP";

    // Arrival gets its own badge state rather than showing the raw action.
    // The run pauses here, and "STOP" on screen would leave you unable to
    // tell success from a blocked path -- the exact confusion target_reached
    // exists to remove.
    const reached = result.target_reached === true;
    // "not_visible" (and any unexpected value) highlights nothing rather
    // than defaulting to centre -- a lit zone must always mean the model
    // actually reported that zone.
    const zone = result.target_visible ? result.target_direction : null;
    highlightRobotZone(["left", "center", "right"].indexOf(zone) >= 0 ? zone : null, reached);

    // Same edge-glow elements Guide me uses (already reset to opacity 0 by
    // hideAllGuidanceOverlays() above), driven by /navigate's 3-zone
    // target_direction instead of /guidance's 5-zone position -- there's
    // no "far_left"-style outer tier here, just one fixed intensity for
    // "the target is to that side". Suppressed on arrival, same as the
    // chevron below: the REACHED badge carries that state alone.
    if (!reached && zone === "left") guidanceEdgeGlowLeft.style.opacity = ROBOT_EDGE_GLOW_OPACITY;
    else if (!reached && zone === "right") guidanceEdgeGlowRight.style.opacity = ROBOT_EDGE_GLOW_OPACITY;

    robotActionBadge.dataset.action = reached ? "REACHED" : action;
    document.getElementById("robot-action-verb").textContent = reached ? "REACHED" : action;
    document.getElementById("robot-action-target").textContent = reached
      ? (state.guidanceTarget || "target")
      : (result.target_visible
          ? (state.guidanceTarget || "target") + " visible" + (result.target_direction ? ", " + result.target_direction : "")
          : "target not visible");
    robotActionBadge.classList.add("visible");

    // STOP is the absence of a direction, so showing a chevron for it would
    // be actively misleading -- the badge carries it alone. Same for arrival.
    if (reached || action === "STOP") return;

    guidanceChevron.style.left = (50 + (ROBOT_ACTION_OFFSET[action] || 0)) + "%";
    guidanceChevron.style.top = "50%";
    guidanceChevron.style.transform =
      "translate(-50%, -50%) rotate(" + ROBOT_ACTION_ROTATION[action] + "deg) scale(1.15)";
    guidanceChevron.style.color = action === "FORWARD" ? getCss("--accent-safe") : getCss("--text");
    guidanceChevron.style.setProperty("--pulse-duration", GUIDANCE_PULSE_DURATION.unknown);
    guidanceChevron.dataset.proximity = "unknown";
    guidanceChevron.classList.add("visible");
  }

  function renderRobotStatus(result, analyzing) {
    const iconEl = document.getElementById("guide-caption-icon");
    const textEl = document.getElementById("guide-caption-text");
    if (analyzing && !result) { iconEl.innerHTML = ICON.clock; textEl.textContent = "Deciding…"; return; }
    if (!result) { iconEl.innerHTML = ICON.search; textEl.textContent = "Not running."; return; }
    if (result.error) { iconEl.innerHTML = ICON.warning; textEl.textContent = result.error; return; }
    if (result.unconfirmed) { iconEl.innerHTML = ICON.warning; textEl.textContent = result.reasoning; return; }
    if (result.target_reached === true) {
      iconEl.innerHTML = ICON.target;
      textEl.textContent = "Reached " + (state.guidanceTarget || "the target") +
        " -- paused. " + (result.reasoning || "");
      return;
    }
    iconEl.innerHTML = ICON[ROBOT_ACTION_ICON[result.action] || "search"];
    // The model's stated reason is the thing worth reading here: it is how
    // you tell a correct action from a right answer for a wrong reason.
    textEl.textContent = result.reasoning || "";
  }

  // Same transition-only discipline as announceGuidance(), keyed on the
  // action rather than the zone -- a per-tick live region would interrupt
  // itself continuously.
  let robotLastAnnouncedAction = null;
  function announceRobot(result) {
    const key = result.error ? "error"
      : result.unconfirmed
        ? "unconfirmed|" + ((result._missionStatus.late_confirmation || {}).state || "")
        : (result.target_reached === true ? "reached" : (result.action || "?"));
    if (key === robotLastAnnouncedAction) return;
    robotLastAnnouncedAction = key;
    const msg = result.error
      ? "Decision unavailable. " + (result.error || "")
      : result.unconfirmed ? result.reasoning
      : key === "reached"
        ? "Reached " + (state.guidanceTarget || "the target") + ". Search paused."
        : key + ". " + (result.reasoning || "");
    announce(document.getElementById("guide-announce-polite"), msg);
  }

  const GUIDANCE_CAPTION_ICON = {
    found: "target",
    far_left: "chevronLeft", left: "chevronLeft",
    right: "chevronRight", far_right: "chevronRight",
    center: "chevronUp",
  };

  function renderGuidanceStatus(result, analyzing) {
    const iconEl = document.getElementById("guide-caption-icon");
    const textEl = document.getElementById("guide-caption-text");
    if (analyzing && !result) {
      iconEl.innerHTML = ICON.clock;
      textEl.textContent = "Analyzing…";
      return;
    }
    if (!result) { iconEl.innerHTML = ICON.search; textEl.textContent = "Not running."; return; }
    if (result.error) { iconEl.innerHTML = ICON.warning; textEl.textContent = result.error; return; }
    if (!result.target_visible || result.position === "not_visible") {
      const scan = currentGuidanceScanPhase();
      iconEl.innerHTML = ICON[scan.icon];
      textEl.textContent = scan.caption;
      return;
    }
    iconEl.innerHTML = ICON[isGuidanceFound(result) ? GUIDANCE_CAPTION_ICON.found : (GUIDANCE_CAPTION_ICON[result.position] || "search")];
    textEl.textContent = result.guidance || "";
  }

  // ---- screen-reader announcements ----
  // Guide's entire visible output is geometry: a chevron, two edge glows,
  // a rectangle. None of it has an accessible name, and the caption that
  // paraphrases it is deliberately de-emphasised because the visuals are
  // meant to be the primary channel. For anyone not looking at the screen
  // that ranking inverts completely -- text becomes the only channel --
  // which matters here more than anywhere else in this app, since "point
  // your phone and I'll tell you where the thing is" is close to a
  // canonical assistive use case. The service already returns a
  // human-readable sentence in `guidance`; this speaks it.
  //
  // Announcements fire on TRANSITIONS, never per tick. The analysis loop
  // runs about once a second, and a live region driven off every tick
  // would interrupt itself continuously -- worse than silence. A tick
  // reporting the same zone and proximity as the last announcement says
  // nothing at all.
  const GUIDANCE_ANNOUNCE_MIN_GAP_MS = 1800;
  const ZONE_WORDS = {
    far_left: "far to your left", left: "to your left", center: "straight ahead",
    right: "to your right", far_right: "far to your right",
  };
  const PROXIMITY_WORDS = { near: "close", medium: "nearby", far: "some distance away" };
  let guidanceLastAnnounceKey = null;
  let guidanceLastAnnounceAt = 0;

  // Most screen readers ignore a live-region write that doesn't change the
  // text, so an identical repeated message (found → resume → found) would
  // be silently dropped. Clearing first forces it to register.
  function announce(el, message) {
    el.textContent = "";
    setTimeout(function () { el.textContent = message; }, 50);
  }

  function resetGuidanceAnnouncements() {
    guidanceLastAnnounceKey = null;
    guidanceLastAnnounceAt = 0;
    document.getElementById("guide-announce-polite").textContent = "";
    document.getElementById("guide-announce-assertive").textContent = "";
  }

  // What counts as "the same situation" for announcement purposes. Zone and
  // proximity only -- the wording of `guidance` can vary tick to tick for an
  // unchanged scene (independent LLM calls), and re-reading a reworded
  // sentence describing the same thing is exactly the chatter to avoid.
  function guidanceAnnounceKey(result) {
    if (result.error) return "error";
    if (isGuidanceFound(result)) return "found";
    if (!result.target_visible || result.position === "not_visible") return "searching";
    return (result.position || "?") + "|" + (result.proximity || "?");
  }

  function guidanceAnnounceMessage(result, key) {
    const target = state.guidanceTarget || "the target";
    if (key === "error") return "Guidance unavailable. " + (result.error || "");
    if (key === "found") return "Found " + target + ".";
    if (key === "searching") return "Searching for " + target + ". Keep panning slowly.";
    if (result.guidance) return result.guidance;
    // Fallback for a response with a position but no sentence.
    const zone = ZONE_WORDS[result.position] || "";
    const prox = PROXIMITY_WORDS[result.proximity] || "";
    return target + (zone ? " " + zone : "") + (prox ? ", " + prox : "") + ".";
  }

  function announceGuidance(result) {
    const key = guidanceAnnounceKey(result);
    if (key === guidanceLastAnnounceKey) return; // same situation -- stay quiet
    const now = Date.now();
    // Found is the one state worth interrupting for: it ends the search, so
    // it skips both the pacing floor and the polite queue.
    if (key !== "found" && now - guidanceLastAnnounceAt < GUIDANCE_ANNOUNCE_MIN_GAP_MS) {
      // Deliberately does not record the key. The next eligible tick should
      // still see a change and announce whatever is true by then, rather
      // than swallowing this transition permanently.
      return;
    }
    guidanceLastAnnounceKey = key;
    guidanceLastAnnounceAt = now;
    announce(
      document.getElementById(key === "found" ? "guide-announce-assertive" : "guide-announce-polite"),
      guidanceAnnounceMessage(result, key)
    );
  }

  // ---- haptic (Android-only in practice) + audio (all platforms) cues ----
  // iOS Safari has never implemented the Vibration API (confirmed platform
  // limitation, not a bug) -- navigator.vibrate() below is a real,
  // feature-detected call that will work if this page is ever opened on
  // Android Chrome, but on iOS it silently no-ops. The Web Audio tone
  // system is what actually provides directional feedback on iOS. See
  // PLAN-ar-guidance.md.

  let guidanceAudioCtx = null;
  function ensureGuidanceAudioCtx() {
    if (!guidanceAudioCtx) {
      const Ctx = window.AudioContext || window.webkitAudioContext;
      if (Ctx) guidanceAudioCtx = new Ctx();
    }
    if (guidanceAudioCtx && guidanceAudioCtx.state === "suspended") guidanceAudioCtx.resume();
    return guidanceAudioCtx;
  }

  // Guide is meant to be used walking around holding the phone up to scan
  // a room -- without this, the OS dims/locks the screen mid-search on
  // anything longer than a quick glance, same as it would during a video
  // call with no interaction. Best-effort and silent on unsupported
  // browsers (notably iOS Safari pre-16.4) -- Guide works exactly as
  // before there, just without this specific convenience.
  let guidanceWakeLock = null;
  async function requestGuidanceWakeLock() {
    if (!("wakeLock" in navigator)) return;
    try {
      guidanceWakeLock = await navigator.wakeLock.request("screen");
    } catch (e) { /* best-effort -- e.g. backgrounded before the request resolved */ }
  }
  function releaseGuidanceWakeLock() {
    if (guidanceWakeLock) {
      guidanceWakeLock.release().catch(function () { /* best-effort */ });
      guidanceWakeLock = null;
    }
  }

  // Peak gain for the directional/found tones below, as a fraction of full
  // scale (1.0). Raised from 0.18 -- real-device feedback found the tones
  // too quiet to reliably notice. Roughly doubled rather than maxed out:
  // these fire multiple times a second while searching, so something
  // closer to 1.0 would be fatiguing/jarring rather than a subtle cue.
  // There's no in-app volume control, only the mute toggle (btn-guidance-
  // mute) -- this constant plus the phone's own volume are the only two
  // levers, by design.
  const GUIDANCE_TONE_GAIN = 0.35;

  function playGuidanceTone(freq, pan, durationMs, opts) {
    if (state.guidanceMuted) return;
    const ctx2 = ensureGuidanceAudioCtx();
    if (!ctx2) return;
    try {
      const osc = ctx2.createOscillator();
      const gain = ctx2.createGain();
      const panner = ctx2.createStereoPanner ? ctx2.createStereoPanner() : null;
      osc.type = (opts && opts.type) || "sine";
      osc.frequency.setValueAtTime(freq, ctx2.currentTime);
      if (opts && opts.glideTo) osc.frequency.linearRampToValueAtTime(opts.glideTo, ctx2.currentTime + durationMs / 1000);
      if (panner) panner.pan.setValueAtTime(pan, ctx2.currentTime);
      gain.gain.setValueAtTime(0, ctx2.currentTime);
      gain.gain.linearRampToValueAtTime(GUIDANCE_TONE_GAIN, ctx2.currentTime + 0.02);
      gain.gain.linearRampToValueAtTime(0, ctx2.currentTime + durationMs / 1000);
      osc.connect(gain);
      if (panner) { gain.connect(panner); panner.connect(ctx2.destination); }
      else { gain.connect(ctx2.destination); }
      osc.start();
      osc.stop(ctx2.currentTime + durationMs / 1000 + 0.02);
    } catch (e) { /* best-effort -- never break the guidance loop over audio */ }
  }

  const GUIDANCE_PAN = { far_left: -1, left: -0.5, center: 0, right: 0.5, far_right: 1 };

  // A second, higher confirmation note shortly after the rising glide --
  // reads more unambiguously as a "ta-da" success chime than a single
  // tone, distinct at a glance (well, a listen) from the plain directional
  // tones below. Now that GUIDANCE_FOUND_STREAK_TO_PAUSE is 1, this also
  // only ever plays once per search (polling pauses immediately after),
  // rather than repeating every tick while found.
  function playGuidanceFoundChime() {
    playGuidanceTone(660, 0, 130, { glideTo: 990 });
    setTimeout(function () { playGuidanceTone(990, 0, 160); }, 140);
  }

  function playGuidanceCues(result) {
    try {
      if (navigator.vibrate) {
        if (isGuidanceFound(result)) navigator.vibrate([40, 60, 40, 60, 90]);
        else if (result.position !== "not_visible" && result.target_visible) navigator.vibrate(35);
      }
    } catch (e) { /* best-effort */ }

    if (!result.target_visible || result.position === "not_visible") return;
    const pan = GUIDANCE_PAN[result.position] || 0;
    if (isGuidanceFound(result)) {
      playGuidanceFoundChime();
    } else if (result.proximity === "near") {
      playGuidanceTone(520, pan, 110);
    } else {
      playGuidanceTone(360, pan, 90);
    }
  }

  // ---- robot view: audio/haptic cues keyed to the decision, not proximity ----
  // /navigate has no `proximity` field, so there is nothing to make a
  // Guide-style "getting warmer" pulse/tone mean anything here -- what it
  // does have (an action verb, arrival, an obstacle flag) carries its own
  // real signal instead: a directional tone per action, the same found
  // chime Guide uses on arrival, and a distinct warning buzz on an
  // obstacle. Reuses Guide's existing Web Audio/vibration plumbing
  // (ensureGuidanceAudioCtx/playGuidanceTone/state.guidanceMuted) as-is --
  // only the cue *mapping* is new, not a second audio system.
  const ROBOT_ACTION_TONE = {
    FORWARD: { freq: 440, pan: 0 },
    LEFT: { freq: 440, pan: -0.6 },
    RIGHT: { freq: 440, pan: 0.6 },
    // Descending pitch reads as "moving away" rather than "turning toward" --
    // the one action tone that isn't just a panned copy of the others.
    REVERSE: { freq: 440, pan: 0, glideTo: 260 },
  };

  // A low square-wave double-buzz, deliberately unlike the sine directional
  // tones above and the found chime's rising glide -- an obstacle is a
  // warning, not a direction or a success, and should not be mistaken for
  // either at a glance (well, a listen).
  function playRobotObstacleTone() {
    playGuidanceTone(180, 0, 90, { type: "square" });
    setTimeout(function () { playGuidanceTone(180, 0, 90, { type: "square" }); }, 120);
  }

  function playRobotCues(result) {
    // Same "nothing decided" guard as renderRobotOverlay's -- an errored
    // or not-yet-ticked result has no action to voice, and playing a tone
    // for it would be inventing feedback for a decision that never
    // happened.
    if (result.error || (!result.action && !result.target_reached)) return;

    if (result.target_reached) {
      try { if (navigator.vibrate) navigator.vibrate([40, 60, 40, 60, 90]); } catch (e) { /* best-effort */ }
      playGuidanceFoundChime();
      return;
    }
    if (result.obstacle_ahead) {
      // Haptic reserved for arrival and obstacles only, not every routine
      // tick -- Robot view decides every ~500ms, and buzzing on each one
      // would be a continuous vibration for the length of a walk rather
      // than a meaningful cue. Audio still plays every tick (matching
      // Guide's own while-searching chattiness, muted by the same toggle).
      try { if (navigator.vibrate) navigator.vibrate([80, 40, 80]); } catch (e) { /* best-effort */ }
      playRobotObstacleTone();
      return;
    }
    const tone = ROBOT_ACTION_TONE[result.action];
    if (tone) playGuidanceTone(tone.freq, tone.pan, 90, tone.glideTo ? { glideTo: tone.glideTo } : undefined);
    // No tone for action === "STOP" here (not reached, not obstacle_ahead) --
    // the ambiguous/fallback case (PLAN-ar-guidance.md's schema note: STOP
    // is also the parser's default on a malformed response), same
    // deliberate silence Guide itself falls back to for its own
    // unresolved states.
  }

  // Once "found" holds for this many consecutive ticks, polling pauses
  // entirely (no more paid API calls) -- was 2, requiring a second
  // confirming tick to avoid pausing on a single noisy detection. In
  // practice this backfired: since each ~1s tick is an independent vision
  // call with real per-frame noise (see isGuidanceFound's comment above),
  // a found tick's very next tick would often flicker back to
  // not-found/off-center, resetting the streak to 0 -- so the found
  // badge/outline would flash and immediately get wiped by the next
  // "Analyzing..." with no pause ever happening. Pausing on the first hit
  // trades an occasional false-positive pause (dismissable via "Resume
  // searching") for the outline actually sticking once shown. Camera and
  // the last outline stay on screen either way.
  const GUIDANCE_FOUND_STREAK_TO_PAUSE = 1;
  // Two, not one: Guide's found state is corroborated by a bounding box and
  // a human looking at the screen, while this one silently ends a run, so it
  // is worth one extra tick of confirmation.
  const ROBOT_REACHED_STREAK_TO_PAUSE = 2;

  function pauseGuidanceSearch() {
    state.guidancePaused = true;
    document.getElementById("btn-guidance-resume").classList.add("visible");
    announce(document.getElementById("guide-announce-polite"),
      "Search paused. Activate resume searching to continue.");
  }

  function resumeGuidanceSearch() {
    state.guidancePaused = false;
    state.guidanceFoundStreak = 0;
    // Clear the transition history so the state we resume into is announced
    // fresh, even if it matches whatever was last spoken before the pause.
    resetGuidanceAnnouncements();
    // A cap-triggered pause (see GUIDANCE_MAX_CALLS in guidanceStep) would
    // otherwise re-trigger on the very next tick -- resuming past it means
    // starting a fresh call budget, same as a brand new Guide session.
    if (state.guidanceCallCount >= guidanceCallCap()) {
      state.guidanceCallCount = 0;
      renderGuidanceBudget();
    }
    document.getElementById("btn-guidance-resume").classList.remove("visible");
    guidanceStep();
  }

  // Safety ceiling on paid API calls for a single Guide session, sized to
  // cover a few minutes of active searching rather than running unattended
  // all day. Originally
  // 200, picked for a ~1s throttle-as-cycle-time approximation (200s ~=
  // 3.3min). The real cycle time is throttle + the /guidance call itself;
  // at Nova Lite's measured ~1.0-1.6s/call plus the 500ms throttle above
  // (~1.7s/cycle typical), 200 calls would run ~5.7min instead -- longer
  // than intended now that the cycle time is actually known. Recalibrated
  // to hit that same ~3.3min window: 200s / ~1.7s =~ 118, rounded to 120.
  // A guess, not a measurement of a full session -- adjust if real Guide
  // sessions end up shorter or longer than that in practice. Reuses the
  // existing pause/"Resume searching" flow rather than a hard stop, since
  // that plumbing already does exactly the right thing here.
  const GUIDANCE_MAX_CALLS = 120;
  // Robot view moved to the same model and throttle as Guide on
  // 2026-08-28 (see ROBOT_THROTTLE_MS above), so the same ~3.3min budget
  // this constant was calibrated for above applies here too -- was 60
  // (~2.5min) back when Robot view paced itself to Sonnet's slower
  // round trip instead. Resume extends it, same as Guide.
  const ROBOT_MAX_CALLS = 120;
  function guidanceCallCap() {
    return state.guidanceMode === "robot" ? ROBOT_MAX_CALLS : GUIDANCE_MAX_CALLS;
  }

  // Skip the paid API call entirely when the phone's orientation sensor
  // says it hasn't moved meaningfully since the last analyzed frame --
  // the framing hasn't changed, so a fresh call would very likely just
  // re-confirm the same answer. Capped at a few consecutive skips so a
  // stationary phone still gets a real re-check periodically (e.g. the
  // target itself could move into/out of frame) rather than freezing on
  // a skip forever. No-ops gracefully when the sensor isn't available
  // (state.guidanceLastOrientation stays null) -- every tick just runs
  // as a real call in that case, same as before this existed.
  const GUIDANCE_STILL_SKIP_DEG_PER_SEC = 3;
  const GUIDANCE_MAX_CONSECUTIVE_SKIPS = 3;

  // Single owner of the HUD budget indicator -- previously three separate
  // "N calls" textContent assignments scattered across start/resume/tick.
  function renderGuidanceBudget() {
    const used = state.guidanceCallCount;
    const cap = guidanceCallCap();
    const pct = Math.min(100, Math.round((used / cap) * 100));
    const fill = document.getElementById("guide-budget-fill");
    fill.style.width = pct + "%";
    fill.classList.toggle("warn", pct >= 80);
    document.getElementById("guidance-call-count").textContent = used + "/" + cap;
    document.getElementById("guide-budget").setAttribute(
      "aria-label", used + " of " + cap + " vision calls used this session");
  }

  // The phone hasn't moved, so the previous read still describes what's on
  // screen and no paid call is made. Previously this replaced the caption
  // with skip wording, which read as a stall -- the guidance was still
  // valid, it just hadn't needed refreshing. Now the last answer stays put
  // and the overlay softens to show it's resting rather than live.
  function renderGuidanceSkipped() {
    guidanceChevron.classList.add("steady");
    guidanceOutline.classList.add("steady");
  }

  async function guidanceStep() {
    if (!state.guidanceRunning) return;
    // A tick scheduled at dispatch (below) can fire after the run paused --
    // on arrival, on a mission that ended, at the call cap. It used to
    // dispatch anyway: one more call, a caption reset to "Deciding...", and
    // an answer the paused-at-dispatch guard then dropped, so the HUD sat on
    // "Deciding..." over the reason the run had stopped (found by 3.53's
    // screenshot). resumeGuidanceSearch() clears the pause before calling.
    if (state.guidancePaused) return;
    // Drive-via-brain runs a real MissionRunner mission, which is strictly
    // one action at a time -- the step budget and the failsafes are built
    // on that. Overlapping calls there would have the brain deciding from
    // frames it has already acted past, so that mode stays serial.
    const maxInFlight = driveViaBrainActive() ? 1 : GUIDANCE_MAX_IN_FLIGHT;
    if (state.guidanceInFlight >= maxInFlight) { scheduleGuidanceNext(); return; }

    if (state.guidanceCallCount >= guidanceCallCap()) {
      document.getElementById("guide-caption-icon").innerHTML = ICON.warning;
      document.getElementById("guide-caption-text").textContent = "Call cap reached for this session -- tap Resume to keep going.";
      pauseGuidanceSearch();
      return;
    }

    const isHoldingStill = state.guidanceLastOrientation && state.guidancePanSpeed < GUIDANCE_STILL_SKIP_DEG_PER_SEC;
    if (isHoldingStill && state.guidanceCallCount > 0 && state.guidanceConsecutiveSkips < GUIDANCE_MAX_CONSECUTIVE_SKIPS) {
      state.guidanceConsecutiveSkips++;
      renderGuidanceSkipped();
      scheduleGuidanceNext();
      return;
    }
    state.guidanceConsecutiveSkips = 0;

    const isRobot = state.guidanceMode === "robot";
    // Which run this call belongs to. `guidanceRunning` cannot answer that:
    // it is a boolean, and it is true again the moment the NEXT session
    // starts -- so a call still in flight when you stop would pass the
    // freshness checks below and render the previous session's decision over
    // the new session's camera view. Worse and invisible: it also sets
    // guidanceLastRenderedSeq to its own (higher) seq, which then suppressed
    // the new session's first several real answers. Captured here, in the
    // same synchronous block as the dispatch, and compared on return.
    const epoch = state.guidanceEpoch;
    const seq = ++state.guidanceSeq;
    // Reserved here, in the same synchronous block as the dispatch, so no
    // two overlapping calls can be handed the same frame number. Null
    // when this frame is not being recorded at all.
    const recSeq = (isRobot && state.recordWalkName) ? state.recordSeq++ : null;
    // Which WALK this frame belongs to, captured in the same synchronous
    // block as its seq. Compared on return -- see recordWalkFrame().
    const recEpoch = state.recordEpoch;
    // A pause that begins DURING this call still gets one final render.
    // `driveViaBrainStep()` itself pauses the run when it sees the mission
    // has ended, and a guard that dropped answers landing in a paused run
    // once dropped that terminal answer by the very pause it had caused:
    // the caption stayed on "Deciding..." over the reason the mission ended
    // (a real rig walk, 2026-09-12, "it eventually got stuck at deciding").
    // Nothing is dispatched while paused (the early return above, 3.53), so
    // that guard is gone rather than kept for a case that cannot happen.
    state.guidanceInFlight++;
    // Budget is reserved at dispatch, not on success. A call that is sent
    // has been paid for whether or not its answer is fresh enough to
    // render, and counting on completion would let the loop dispatch past
    // the cap while calls were still outstanding.
    state.guidanceCallCount += 1;
    renderGuidanceBudget();
    (isRobot ? renderRobotStatus : renderGuidanceStatus)(null, true);

    // Scheduled here rather than in the finally below: that is the whole
    // change. Pacing the next dispatch on the throttle instead of on this
    // call returning is what lets calls overlap at all.
    scheduleGuidanceNext();

    try {
      const base64 = captureGuidanceFrame();
      robotSignalCapture();
      const result = (isRobot && driveViaBrainActive())
        ? await driveViaBrainStep(base64)
        : await callGuidanceEndpoint(base64, state.guidanceTarget);
      // Nothing below this line may touch state belonging to a run that is
      // no longer the current one -- not the error streak, not the frame
      // recorder, not the overlay. See `epoch` above.
      if (epoch !== state.guidanceEpoch) return;
      // 3.64 A7: the pending tick was timed at dispatch from the error
      // streak (up to 30 s); the first success after an outage replaces it
      // with the normal throttle instead of sitting out the backoff.
      const recovered = state.guidanceErrorStreak > 0;
      state.guidanceErrorStreak = 0;
      if (recovered) scheduleGuidanceNext({ replace: true });

      // The frame and the answer it got belong together regardless of
      // arrival order, so a recorded walk keeps every pair -- including
      // ones too stale to draw. Dropping them would silently thin a
      // recording that sim/replay_robot.py later plays back frame by
      // frame.
      if (isRobot) recordWalkFrame(base64, result, recSeq, recEpoch,
                                   state.lastTeleopSeq);

      // Everything past here changes what the person sees or the loop
      // believes, so it must not run for an answer that has been overtaken
      // (latency varies per call, so seq 7 can land after seq 9) or for a
      // walk that has since been paused or stopped.
      if (!state.guidanceRunning) return;
      if (seq <= state.guidanceLastRenderedSeq) return;
      state.guidanceLastRenderedSeq = seq;

      if (isRobot) {
        // No found-pause here, deliberately. Guide pauses on arrival
        // because the person has arrived and further calls are waste;
        // robot view is a continuous readout of decisions, and the run
        // ends at the call cap or when you stop it.
        renderRobotOverlay(result);
        renderRobotStatus(result, false);
        renderRobotTelemetry(result);
        announceRobot(result);
        playRobotCues(result);

        // Arrival comes from the service's own target_reached field, never
        // from action === "STOP" -- STOP is also what the model returns for
        // a blocked path, and what the parser defaults to on a malformed
        // response. Same found-streak discipline as Guide so one flaky tick
        // cannot end a walk.
        if (result.target_reached) {
          state.guidanceFoundStreak++;
          if (state.guidanceFoundStreak >= ROBOT_REACHED_STREAK_TO_PAUSE) pauseGuidanceSearch();
        } else {
          state.guidanceFoundStreak = 0;
        }
      } else {
        renderGuidanceOverlay(result);
        renderGuidanceStatus(result, false);
        playGuidanceCues(result);
        announceGuidance(result);

        if (isGuidanceFound(result)) {
          state.guidanceFoundStreak++;
          if (state.guidanceFoundStreak >= GUIDANCE_FOUND_STREAK_TO_PAUSE) pauseGuidanceSearch();
        } else {
          state.guidanceFoundStreak = 0;
        }
      }
    } catch (e) {
      // A previous run's failure must not pace, or discourage, this one --
      // same reasoning as the epoch check on the success path above.
      if (epoch !== state.guidanceEpoch) return;
      state.guidanceErrorStreak++;
      // The next tick was already scheduled at the base throttle before
      // this call failed, so the backoff has to replace that timer rather
      // than add to it -- otherwise a failing endpoint keeps getting
      // hammered at 500ms while the backoff sits unused.
      scheduleGuidanceNext({ replace: true });
      if (!state.guidanceRunning || state.guidancePaused) return;
      (isRobot ? renderRobotStatus : renderGuidanceStatus)({ error: e.message }, false);
      if (isRobot) renderRobotTelemetry({ error: e.message });
      // Keyed as "error", so the exponential backoff's repeated failures
      // announce once rather than on every retry.
      (isRobot ? announceRobot : announceGuidance)({ error: e.message });
    } finally {
      // 3.64 A2: only this run's calls count against this run's cap. Stop
      // zeroes the count and bumps the epoch, so a call from before it
      // must not take the new run's count below zero.
      if (epoch === state.guidanceEpoch) state.guidanceInFlight--;
    }
  }

  // Backs off exponentially (capped at 30s) after consecutive failures
  // instead of hammering a possibly-down endpoint every second on
  // someone's phone battery/data -- resets to the normal throttle the
  // moment a call succeeds again (see guidanceErrorStreak reset above).
  function scheduleGuidanceNext(opts) {
    if (!state.guidanceRunning || state.guidancePaused) return;
    // Hidden: the visibilitychange handler cleared the tick on purpose and
    // reschedules on return. A call that lands meanwhile -- a success after
    // an outage (3.64 A7) or a failure's backoff -- must not re-arm it, or a
    // locked phone keeps making paid calls.
    if (document.hidden) return;
    // With calls overlapping, more than one path can want to schedule the
    // next tick. `replace` is for the error path, which needs to push an
    // already-scheduled tick further out; without it a pending timer wins
    // and the caller's delay is silently ignored.
    if (opts && opts.replace && state.guidanceTimerId) {
      clearTimeout(state.guidanceTimerId);
      state.guidanceTimerId = null;
    } else if (state.guidanceTimerId) {
      return;
    }
    // Robot view and Guide share a throttle value now (both on Nova Lite
    // as of 2026-08-28 -- see ROBOT_THROTTLE_MS above), kept as two named
    // constants rather than one shared one so they can diverge again
    // without hunting down every use site.
    const base = state.guidanceMode === "robot" ? ROBOT_THROTTLE_MS : GUIDANCE_THROTTLE_MS;
    const delay = state.guidanceErrorStreak > 0
      ? Math.min(30000, base * Math.pow(2, state.guidanceErrorStreak))
      : base;
    robotStartCountdown(delay);
    state.guidanceTimerId = setTimeout(function () {
      // Cleared before the step runs, so the idempotence guard above sees
      // "no tick pending" and the step it is about to run can schedule the
      // next one.
      state.guidanceTimerId = null;
      guidanceStep();
    }, delay);
  }

  function showGuideStartError(message) {
    const el = document.getElementById("guide-start-error");
    el.textContent = message;
    el.style.display = "";
  }

  // ---- fast-pan detection ----
  // Panning the phone faster than the vision model can usefully track
  // produces motion-blurred frames and a jumpy overlay, so this nudges
  // the user to slow down using the phone's orientation sensor (free,
  // continuous, no API cost) rather than anything derived from the
  // throttled /guidance calls themselves.
  //
  // History: started at 10deg/s, briefly tried ~6deg/s after /guidance
  // moved to Nova Lite (reverted the same day -- too aggressive, false
  // "slow down" nudges during normal panning). Both were wrong in the
  // same direction: real-device testing on 2026-08-28 measured a calm,
  // deliberate room-scan -- exactly the motion this feature is supposed
  // to welcome, including the app's own search-sweep telling the user to
  // pan right/down -- at 25-30deg/s, well above either number, so normal
  // use was tripping the warning essentially all the time.
  //
  // Recalibrated 2026-08-28 from the capture cadence itself, not just
  // vibes: a typical phone's rear camera has roughly a 65deg horizontal
  // field of view, and keeping at least 50% overlap between two
  // consecutive analyzed frames -- so the target can't drift clean out of
  // frame between them -- caps the drift budget at half that, ~32.5deg.
  // Captures fire every GUIDANCE_THROTTLE_MS/ROBOT_THROTTLE_MS (500ms),
  // so 32.5deg / 0.5s ~= 65deg/s. That lines up with the same day's
  // measurements: comfortable deliberate scanning tops out around
  // 30-40deg/s (clear margin under it), an actual whip-pan measured
  // 300+deg/s (trips it easily). Using the 500ms capture interval here,
  // not the ~1.7s full round trip (throttle + Nova Lite latency) --
  // that larger number would derive a threshold back down near 19deg/s,
  // right in the over-sensitive range that caused the false positives
  // above. Still a tuning knob, not a settled constant -- adjust the FOV/
  // overlap assumptions above if it under- or over-warns in practice.
  const PAN_SPEED_WARN_DEG_PER_SEC = 65;
  let guidanceOrientationHandler = null;
  let guidanceNoSensorTimerId = null;

  function angleDelta(a, b) {
    // Shortest signed distance from b to a on a 0-360 circle (handles the
    // 359deg -> 1deg wraparound as a 2deg move, not a 358deg one).
    let d = a - b;
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
  }

  function updateMotionWarning() {
    const el = document.getElementById("guide-motion-warning");
    el.classList.toggle("visible", state.guidancePanSpeed > PAN_SPEED_WARN_DEG_PER_SEC);
  }

  function handleDeviceOrientation(e) {
    if (e.alpha == null || e.beta == null || e.gamma == null) return;
    if (guidanceNoSensorTimerId) { clearTimeout(guidanceNoSensorTimerId); guidanceNoSensorTimerId = null; }
    const now = performance.now();
    const prev = state.guidanceLastOrientation;
    if (prev) {
      const dt = (now - prev.t) / 1000;
      // Skip stale gaps (tab backgrounded, sensor hiccup) rather than
      // reading them as one huge, meaningless spike in angular speed.
      if (dt > 0 && dt < 1) {
        const dAlpha = angleDelta(e.alpha, prev.alpha);
        const dBeta = e.beta - prev.beta;
        const dGamma = e.gamma - prev.gamma;
        const speed = Math.sqrt(dAlpha * dAlpha + dBeta * dBeta + dGamma * dGamma) / dt;
        // Exponential moving average -- raw sensor deltas are noisy
        // enough to flicker the warning on and off every frame otherwise.
        state.guidancePanSpeed = state.guidancePanSpeed * 0.7 + speed * 0.3;
        updateMotionWarning();
        document.getElementById("guide-pan-readout").textContent = "pan: " + Math.round(state.guidancePanSpeed) + "°/s";
      }
    }
    state.guidanceLastOrientation = { alpha: e.alpha, beta: e.beta, gamma: e.gamma, t: now };
  }

  // iOS 13+ Safari gates DeviceOrientationEvent behind an explicit,
  // gesture-triggered permission prompt; every other browser just fires
  // the event with no ask. Best-effort and silent either way -- this is
  // a nice-to-have nudge, not core functionality, so a denial or an
  // unsupported browser should just mean the warning never shows, never
  // a broken Guide tab.
  async function requestMotionPermissionIfNeeded() {
    const DOE = window.DeviceOrientationEvent;
    if (DOE && typeof DOE.requestPermission === "function") {
      try {
        const result = await DOE.requestPermission();
        return result === "granted";
      } catch (e) {
        return false;
      }
    }
    return !!DOE;
  }

  function startMotionTracking() {
    if (guidanceOrientationHandler) return;
    state.guidancePanSpeed = 0;
    state.guidanceLastOrientation = null;
    document.getElementById("guide-pan-readout").textContent = "pan: waiting…";
    guidanceOrientationHandler = handleDeviceOrientation;
    window.addEventListener("deviceorientation", guidanceOrientationHandler);
    // If not one single event arrives in a few seconds, the browser is
    // granting the permission but never actually delivering data (seen on
    // some Android WebViews/desktop browsers) -- say so explicitly rather
    // than leaving "waiting..." up forever, which looks identical to "just
    // hasn't moved yet."
    guidanceNoSensorTimerId = setTimeout(function () {
      document.getElementById("guide-pan-readout").textContent = "pan: no sensor";
    }, 3000);
  }

  function stopMotionTracking() {
    if (guidanceOrientationHandler) {
      window.removeEventListener("deviceorientation", guidanceOrientationHandler);
      guidanceOrientationHandler = null;
    }
    if (guidanceNoSensorTimerId) { clearTimeout(guidanceNoSensorTimerId); guidanceNoSensorTimerId = null; }
    state.guidancePanSpeed = 0;
    state.guidanceLastOrientation = null;
    document.getElementById("guide-pan-readout").textContent = "pan: --";
    document.getElementById("guide-motion-warning").classList.remove("visible");
  }

  async function startGuidance() {
    // `guidanceRunning` is not set until the bottom of this function, three
    // awaits away (motion permission, getUserMedia, video metadata), so the
    // Start button's own `if (state.guidanceRunning)` guard reads false for
    // both taps of a double-tap and TWO loops start. They share one
    // guidanceEpoch, so neither can orphan the other's calls, and the second
    // beginWalkRecording() renames the walk under the first loop's feet --
    // which is how a frame from the previous walk, with its old wording and
    // its old seq, ended up in the next walk's directory
    // (bottle-opus-4-5-center-third-path-20260902-163923, seq 37). It also
    // doubles the paid call rate against one budget.
    if (state.guidanceRunning || state.guidanceStarting) return;
    state.guidanceStarting = true;
    try {
      await startGuidanceInner();
    } finally {
      state.guidanceStarting = false;
    }
  }

  async function startGuidanceInner() {
    document.getElementById("guide-start-error").style.display = "none";
    if (!window.isSecureContext || !navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      showGuideStartError(
        "Camera access requires this page to be loaded over https:// or localhost -- " +
        "browsers block it otherwise. This page is currently loaded over " +
        location.protocol + "//" + location.host + "."
      );
      return;
    }
    // Requested first and before any other await -- iOS Safari's "sticky
    // activation" window for a gesture-gated permission prompt can expire
    // after just one prior await (e.g. the getUserMedia call below), so
    // asking in this order gives it the best chance of still counting as
    // user-initiated. Best-effort either way: denied/unsupported just
    // means the warning never shows, never a broken Guide tab.
    if (await requestMotionPermissionIfNeeded()) startMotionTracking();

    try {
      // `ideal`, not `exact`: a device that cannot do 1920x1080 returns its
      // best effort rather than failing outright with OverconstrainedError,
      // which is what `exact` would do to anyone on an older phone. Asking
      // for more than PERCEPTION_MAX_CAPTURE_DIM on purpose -- the encoder
      // downscales to it, and starting above the target means the crop a
      // distant object lands in was sampled from real sensor pixels rather
      // than upscaled from a stream that was already too small.
      state.guidanceStream = await navigator.mediaDevices.getUserMedia({
        video: {
          facingMode: "environment",
          width: { ideal: 1920 },
          height: { ideal: 1080 },
        },
      });
    } catch (e) {
      showGuideStartError("Camera access failed: " + e.message);
      return;
    }
    // Must happen inside this user-gesture-triggered handler -- iOS's
    // autoplay policy blocks AudioContext creation/resume outside one.
    ensureGuidanceAudioCtx();
    requestGuidanceWakeLock();

    guidanceVideo.srcObject = state.guidanceStream;
    guidanceFullscreen.classList.add("active");
    setRobotZonesVisible(state.guidanceMode === "robot");
    updateGuideRotateHint();
    state.guidanceTarget = document.getElementById("guidance-target").value.trim() || "red backpack";
    saveTargetToHistory(state.guidanceTarget);

    // videoWidth/videoHeight are 0 until the stream's metadata actually
    // loads -- capturing or mapping coordinates before then produces a
    // blank frame / NaN math (0 * Infinity), so wait for it explicitly
    // rather than assuming srcObject being set means the video is ready.
    // Moved ahead of the "drive via brain" block below (it used to run
    // after /mission/start) specifically so a real frame can be pushed
    // before the mission exists -- see that block's comment.
    await waitForGuidanceVideoReady();

    // Phase T3 (PLAN-teleop-robot.md): hand this walk to a real
    // MissionRunner mission instead of a one-off /navigate call per frame.
    // Latched here rather than re-read from driveViaBrainActive() every
    // tick -- the toggle or the brain connection could change mid-walk, and
    // stopGuidance() needs to know whether *this* session actually started
    // a mission, not what the toggle says right now.
    state.guidanceViaBrain = driveViaBrainActive();
    if (state.guidanceViaBrain) {
      // Checked before /mission/start, not just left to fail on the first
      // frame push: "drive via brain" needs the *robot server* URL (not
      // just the brain URL) pointed at a mode: teleop deployment -- if
      // it's still the plain twin, POST /teleop/frame 404s per frame with
      // no clue why. /health now reports which mode is actually running.
      try {
        const robotHealth = await apiGet("/health");
        if (robotHealth.mode !== "teleop") {
          throw new Error(
            "Robot server URL isn't a teleop deployment (it reports mode: '" +
            (robotHealth.mode || "unknown") + "'). Point Settings' Robot " +
            "server URL at your teleop-robot service instead."
          );
        }
      } catch (e) {
        showGuideStartError("Can't drive via brain: " + e.message);
        state.guidanceViaBrain = false;
        stopGuidance();
        return;
      }
      // Prime a real, fresh frame on the robot server BEFORE the mission
      // exists. Without this, MissionRunner's tick loop starts as soon as
      // /mission/start returns and can call get_camera_frame() before this
      // page's own first guidanceStep() tick has pushed anything -- on a
      // service that's been idle a while, that first tick then reads
      // whatever ancient timestamp TeleopRobot last had (or none at all)
      // and immediately raises TeleopStall, failing the mission before it
      // ever really started. Pushing one frame here guarantees the first
      // tick, whenever it fires, sees something pushed moments ago.
      try {
        await pushTeleopFrame(captureGuidanceFrame());
      } catch (e) {
        showGuideStartError("Could not reach the robot server to prime the first frame: " + e.message);
        state.guidanceViaBrain = false;
        stopGuidance();
        return;
      }
      // A new mission: the last one's late-check watch must never draw
      // its verdict over this session (3.53, review). Whether one WAS
      // watching decides what a refused start resumes: only a check this
      // page was waiting on, never a verdict that finished long before.
      // A resume still in flight counts too: it is about to become a
      // watch, and this start's own resume takes the job over if refused.
      const wasWatchingLate = !!state.lateWatchTimerId || state.lateResumePending;
      stopLateWatch();
      try {
        // The model picker applies here too, not just to a one-off
        // /navigate call: the brain binds it into the mission's vision_fn,
        // so "Drive via brain" runs the model you chose rather than silently
        // falling back to the service default.
        await brainApi("POST", "/mission/start", {
          target_object: state.guidanceTarget,
          // The walk's own policy, not the Sim tab's. "tiered" runs
          // brain/perceive.py's detector and CLIP over these frames inside
          // the brain process and calls /navigate only on a trigger -- the
          // one path in this app where those models see real pixels
          // (PLAN-onboard-perception.md 4.10). A brain without the optional
          // models refuses here, at start, naming the pip command; the
          // catch below surfaces that message rather than letting the walk
          // begin and fail frame by frame.
          policy: state.drivePolicy,
          model_id: state.navigateModelId || null,
          prompt_variant: state.navigatePromptVariant || null,
        });
      } catch (e) {
        showGuideStartError("Could not start the brain-driven mission: " + e.message);
        state.guidanceViaBrain = false;
        stopGuidance();
        // A refused start leaves the last mission's late check running on
        // the brain (3.53, amendment 1), so watch it again from a fresh
        // status rather than leave its verdict unseen.
        if (wasWatchingLate) {
          // Tied to this moment's watch token: a newer Guide start (which
          // stops the watch, bumping it) makes this answer stale on landing.
          const resumeToken = lateWatchToken;
          state.lateResumePending = true;
          brainApi("GET", "/mission/status").then(function (s) {
            if (resumeToken !== lateWatchToken) return;  // a newer start owns it
            state.lateResumePending = false;
            resumeLateConfirmation(s);
          }, function () {
            if (resumeToken === lateWatchToken) state.lateResumePending = false;
          });
        }
        return;
      }
    }

    // A brain deployment that explicitly disables recording (e.g. the
    // teleop-brain stack -- no EFS mount, ALLOW_RECORDING=false, since
    // recording and "drive via brain" are meant to point at different
    // brains -- see cloudformation/teleop-brain.yaml) would otherwise fail
    // every POST /recording/frame with a 403, one per frame, surfaced only
    // as a single easy-to-miss toast on the very first failure and an
    // admin viewer that silently never gains the walk. Checked once up
    // front instead, the same way the "drive via brain" health check above
    // catches a mismatched deployment before wasting a walk on it.
    if (recordingActive()) {
      try {
        const brainHealth = await brainApi("GET", "/health");
        if (!brainHealth.recording_allowed) {
          showGuideStartError(
            "This brain service has recording disabled (no storage attached). " +
            "Point Settings' Brain service URL at a brain deployment with " +
            "recording enabled instead."
          );
          stopGuidance();
          return;
        }
      } catch (e) {
        showGuideStartError("Could not reach the brain service to check recording support: " + e.message);
        stopGuidance();
        return;
      }
    }

    state.guidanceRunning = true;
    state.guidanceCallCount = 0;
    state.guidanceNotVisibleStreak = 0;
    guidanceLastEdgePosition = null;
    state.guidancePaused = false;
    state.guidanceFoundStreak = 0;
    state.guidanceConsecutiveSkips = 0;
    state.guidanceErrorStreak = 0;
    renderGuidanceBudget();
    document.getElementById("btn-guidance-resume").classList.remove("visible");
    updateGuidanceButton();
    resetGuidanceAnnouncements();
    robotLastAnnouncedAction = null;
    announce(document.getElementById("guide-announce-polite"),
      state.guidanceMode === "robot"
        ? "Robot view. Showing the move the robot would make, for " + state.guidanceTarget +
          ". Hold the phone low, about camera height, and walk slowly."
        : "Guiding you to " + state.guidanceTarget + ". Move your phone slowly to scan the room.");
    // Counts against the pacing floor so the first real result doesn't land
    // on top of this one.
    guidanceLastAnnounceAt = Date.now();
    beginWalkRecording();
    guidanceStep();
  }

  function waitForGuidanceVideoReady() {
    if (guidanceVideo.videoWidth > 0 && guidanceVideo.videoHeight > 0) return Promise.resolve();
    return new Promise(function (resolve) {
      guidanceVideo.addEventListener("loadedmetadata", function onReady() {
        guidanceVideo.removeEventListener("loadedmetadata", onReady);
        resolve();
      });
    });
  }

  // Only releases the camera on explicit Stop / page unload -- backgrounding
  // (visibilitychange) pauses the analysis timer below but leaves the
  // camera live, per PLAN-ar-guidance.md.
  function stopGuidance() {
    endWalkRecording();
    if (state.guidanceViaBrain) {
      // Fire-and-forget, same as recordWalkFrame() -- stopGuidance() isn't
      // async (it runs from a plain onclick), and a failed stop here is not
      // a reason to leave the camera/UI half torn down. /mission/stop
      // always stops the robot too, so the mission ending cleanly matters
      // more than this page's confirmation of it.
      brainApi("POST", "/mission/stop").catch(function (e) {
        showToast("Could not stop the brain mission: " + e.message, "err");
      });
      state.guidanceViaBrain = false;
    }
    state.guidanceRunning = false;
    if (state.guidanceTimerId) clearTimeout(state.guidanceTimerId);
    state.guidanceTimerId = null;
    stopMotionTracking();
    releaseGuidanceWakeLock();
    if (state.guidanceStream) {
      state.guidanceStream.getTracks().forEach(function (t) { t.stop(); });
      state.guidanceStream = null;
    }
    guidanceVideo.srcObject = null;
    guidanceFullscreen.classList.remove("active");
    setRobotZonesVisible(false);
    document.getElementById("guide-rotate-hint").classList.remove("visible");
    guidanceLastNormBox = null;
    guidanceLastEdgePosition = null;
    state.guidanceNotVisibleStreak = 0;
    state.guidancePaused = false;
    state.guidanceFoundStreak = 0;
    // Calls already in flight will still resolve after this. Bumping the
    // epoch is what orphans them: guidanceRunning is true again as soon as
    // the next session starts, so it cannot tell "this run" from "the run
    // before it", and a late answer used to be drawn over the new session's
    // camera view -- then set guidanceLastRenderedSeq to its own higher
    // seq, silently suppressing the new run's first few real decisions.
    // guidanceStep() captures this at dispatch and compares it on return.
    state.guidanceEpoch++;
    state.guidanceInFlight = 0;
    state.guidanceSeq = 0;
    state.guidanceLastRenderedSeq = 0;
    document.getElementById("btn-guidance-resume").classList.remove("visible");
    hideAllGuidanceOverlays();
    renderGuidanceStatus(null, false);
    resetGuidanceAnnouncements();
    updateGuidanceButton();
    if (guidanceAudioCtx && guidanceAudioCtx.state === "running") guidanceAudioCtx.suspend();
  }

  function updateGuidanceButton() {
    const btn = document.getElementById("btn-guidance");
    btn.textContent = state.guidanceRunning ? "Stop" : "Start";
    btn.classList.toggle("active-mode", state.guidanceRunning);
  }

  function updateGuidanceMuteButton() {
    document.getElementById("btn-guidance-mute").innerHTML = state.guidanceMuted ? ICON.volumeOff : ICON.volumeOn;
  }

  // See #guide-rotate-hint's CSS comment for why this is a nudge, not a
  // real orientation lock. Dismissing it now persists: it was previously
  // reset on every startGuidance(), so someone who deliberately prefers
  // portrait had to dismiss the same hint every single session.
  let guideRotateHintDismissed = prefGet(PREF.rotateHintDismissed) === "1";
  function updateGuideRotateHint() {
    const isPortrait = window.innerHeight > window.innerWidth;
    const shouldShow = guidanceFullscreen.classList.contains("active") && isPortrait && !guideRotateHintDismissed;
    document.getElementById("guide-rotate-hint").classList.toggle("visible", shouldShow);
  }
  document.getElementById("btn-guide-rotate-dismiss").onclick = function () {
    guideRotateHintDismissed = true;
    prefSet(PREF.rotateHintDismissed, "1");
    updateGuideRotateHint();
  };

  // Re-sync overlay-dependent absolute positioning (the found-outline's
  // px left/top/width/height) immediately on rotate/resize instead of
  // waiting up to GUIDANCE_THROTTLE_MS for the next analysis tick --
  // rotating the phone should feel instant. The chevron/reticle use %
  // positioning so they're already orientation-safe with no code needed.
  function resyncGuidanceOverlayOnResize() {
    updateGuideRotateHint();
    if (!state.guidanceRunning || !guidanceOutline.classList.contains("visible")) return;
    // Re-run the last render with the last-known result would require
    // caching it; simplest robust fix is to just let the next tick (at
    // most GUIDANCE_THROTTLE_MS away) correct it, but hide the outline
    // immediately so a stale/misaligned box is never shown mid-rotation.
    guidanceOutline.classList.remove("visible");
    guidanceFoundBadge.classList.remove("visible");
  }
  window.addEventListener("resize", resyncGuidanceOverlayOnResize);
  window.addEventListener("orientationchange", resyncGuidanceOverlayOnResize);

  document.addEventListener("visibilitychange", function () {
    if (document.hidden) {
      if (state.guidanceTimerId) { clearTimeout(state.guidanceTimerId); state.guidanceTimerId = null; }
    } else if (state.guidanceRunning && !state.guidanceTimerId) {
      // 3.64 A4: not only when nothing is in flight. Hiding cleared the
      // tick and a success never schedules one, so coming back mid-call
      // stopped the loop for good. guidanceStep() itself waits out the
      // in-flight cap.
      scheduleGuidanceNext();
      // The wake lock itself is auto-released by the browser on
      // backgrounding (spec behavior, not a bug) -- re-request it now that
      // we're back, same as the polling timer just above.
      requestGuidanceWakeLock();
    }
  });
  window.addEventListener("beforeunload", function () {
    if (state.guidanceStream) state.guidanceStream.getTracks().forEach(function (t) { t.stop(); });
  });

  // Shown the first time Start is tapped, before the camera prompt fires,
  // so the chevron/glow/pulse/outline language makes sense before it's
  // actually seen live. Persisted: this used to be a per-page-load flag,
  // which meant an installed PWA re-showed the same four-row explainer on
  // every cold launch. "How it works" on the Guide tab re-opens it
  // deliberately for anyone who wants it again.
  let guidanceOnboardingShown = prefGet(PREF.onboarded) === "1";
  // The same card serves two jobs: the first-run explainer that leads
  // straight into starting the camera, and a "How it works" reference
  // someone opens deliberately later. Only the first should start the
  // camera on dismiss.
  let onboardingIsReview = false;

  function showGuidanceOnboarding(asReview) {
    onboardingIsReview = !!asReview;
    document.getElementById("btn-onboarding-start").textContent =
      onboardingIsReview ? "Close" : "Got it, start";
    document.getElementById("guide-onboarding").classList.add("visible");
  }
  function dismissGuidanceOnboarding() {
    document.getElementById("guide-onboarding").classList.remove("visible");
    if (onboardingIsReview) { onboardingIsReview = false; return; }
    guidanceOnboardingShown = true;
    prefSet(PREF.onboarded, "1");
    startGuidance();
  }
  // Robot view reuses every hard part of Guide -- getUserMedia, the
  // permission and secure-context handling, the fullscreen takeover,
  // orientation re-sync, throttling, the call cap, the backgrounded-tab
  // pause -- and swaps only the route and the renderer. That is why it
  // lives here rather than in the Sim tab, which is also the tab you would
  // load over plain LAN http:// once there is a real Pi, where camera
  // access is blocked outright.
  function setGuidanceMode(mode) {
    state.guidanceMode = mode;
    // A failed-Start error (e.g. last turn's "recording disabled" message)
    // belongs to whichever mode produced it -- #guide-start-error is one
    // shared element, so without this it kept showing after switching to
    // the other mode, where it no longer meant anything.
    document.getElementById("guide-start-error").style.display = "none";
    const isRobot = mode === "robot";
    const guideBtn = document.getElementById("btn-mode-guide");
    const robotBtn = document.getElementById("btn-mode-robot");
    guideBtn.classList.toggle("active-mode", !isRobot);
    robotBtn.classList.toggle("active-mode", isRobot);
    guideBtn.setAttribute("aria-pressed", String(!isRobot));
    robotBtn.setAttribute("aria-pressed", String(isRobot));
    document.getElementById("guide-mode-hint").innerHTML = isRobot
      ? "<b>Robot view</b> shows the move the <i>robot</i> would make from where you\u2019re standing \u2014 nothing moves. Hold the phone low, about camera height."
      : "<b>Guide me</b> steers <i>you</i> to the object with an arrow.";
    updateRecordRow();
    updateDriveViaBrainRow();
    updateModelPickerRow();
    try { localStorage.setItem("guidanceMode", mode); } catch (e) { /* private mode */ }
  }
  document.getElementById("btn-mode-guide").onclick = function () { setGuidanceMode("guide"); };
  document.getElementById("btn-mode-robot").onclick = function () { setGuidanceMode("robot"); };
  try {
    const savedMode = localStorage.getItem("guidanceMode");
    if (savedMode === "robot" || savedMode === "guide") setGuidanceMode(savedMode);
  } catch (e) { /* private mode -- default stays "guide" */ }

  document.getElementById("btn-how-it-works").onclick = function () {
    showGuidanceOnboarding(true);
  };

  document.getElementById("btn-guidance").onclick = function () {
    if (state.guidanceRunning) { stopGuidance(); return; }
    if (guidanceOnboardingShown) startGuidance();
    else showGuidanceOnboarding();
  };
  document.getElementById("btn-onboarding-start").onclick = dismissGuidanceOnboarding;
  document.getElementById("btn-guidance-close").onclick = function () { stopGuidance(); };
  document.getElementById("btn-guidance-mute").onclick = function () {
    state.guidanceMuted = !state.guidanceMuted;
    prefSet(PREF.muted, state.guidanceMuted ? "1" : "0");
    updateGuidanceMuteButton();
  };
  document.getElementById("btn-guidance-resume").onclick = function () { resumeGuidanceSearch(); };

  // ---------- tabs ----------
  // Purely presentational -- toggles which .tab-page is visible and which
  // .tab-btn is marked active. No element ids changed when the panels were
  // grouped into tabs, so nothing else in this file needed to change.

  const TAB_NAMES = ["guide", "sim", "settings"];
  // "drive" and "autonomous" were separate tabs before they merged into
  // "sim"; a device that stored either one still has it in localStorage.
  const LEGACY_TAB_ALIASES = { drive: "sim", autonomous: "sim" };

  // Settings is reached from the header gear rather than the tab bar, so
  // it has no tab button to highlight -- the gear lights up instead, and
  // remembers which tab to return to.
  let tabBeforeSettings = "guide";

  // Declared here (used only by hideSetupQr()/showSetupQr() down in the
  // "setup sharing" section, well below) rather than nearer that code: the
  // page-load restore at the bottom of this file calls
  // switchTab(savedTab, {restoring:true}) before this script has executed
  // that far, and switchTab -> hideSetupQr() reads this variable. A `let`
  // is in its temporal dead zone until its declaration line actually runs,
  // so declaring it down there threw "Cannot access 'qrHideTimer' before
  // initialization" on every page load that didn't restore straight into
  // the settings tab -- moved up here, ahead of switchTab's first caller,
  // fixes that regardless of which tab a returning user lands on.
  let qrHideTimer = null, qrCountdownTimer = null;

  function switchTab(name, opts) {
    if (name === "settings") {
      const current = document.querySelector(".tab-page.active");
      if (current && current.dataset.tab !== "settings") tabBeforeSettings = current.dataset.tab;
    }
    document.querySelectorAll(".tab-page").forEach(function (el) {
      el.classList.toggle("active", el.dataset.tab === name);
    });
    document.querySelectorAll(".tab-btn").forEach(function (el) {
      el.classList.toggle("active", el.dataset.tab === name);
      el.setAttribute("aria-selected", el.dataset.tab === name ? "true" : "false");
    });
    document.body.dataset.tab = name;
    if (name !== "settings") hideSetupQr();
    renderHeaderConnStatus();
    const gear = document.getElementById("btn-settings");
    gear.classList.toggle("active", name === "settings");
    gear.setAttribute("aria-expanded", name === "settings" ? "true" : "false");
    if (!(opts && opts.restoring)) prefSet(PREF.tab, name);
    // A tab page that was display:none had zero width, so its canvases
    // couldn't measure themselves. This is a no-op where ResizeObserver
    // exists (it fires on its own); it's the actual fix on browsers
    // without it.
    relayoutCanvases();
  }
  document.querySelectorAll(".tab-btn").forEach(function (btn) {
    btn.onclick = function () { switchTab(btn.dataset.tab); };
  });
  document.getElementById("btn-settings").onclick = function () {
    const onSettings = document.querySelector(".tab-page.active").dataset.tab === "settings";
    switchTab(onSettings ? tabBeforeSettings : "settings");
  };

  // ---------- init ----------
  // No server connection yet at page load. render() no-ops until
  // state.lastFrame exists (set once the user taps Connect), and the map
  // comes from GET /world/map rather than from anything this page knows.

  function renderFpvPlaceholder(message) {
    fpvCtx.fillStyle = getCss("--surface");
    fpvCtx.fillRect(0, 0, FPV_W, FPV_H);
    fpvCtx.fillStyle = getCss("--text-dim");
    fpvCtx.font = "12px " + getCss("--mono");
    fpvCtx.textAlign = "center";
    fpvCtx.fillText(message || "Connect to see the camera view", FPV_W / 2, FPV_H / 2);
  }

  // Re-measures the camera canvas and redraws it if it changed size.
  // Resizing a canvas clears its bitmap, so a redraw here isn't optional.
  // The world map sizes itself in renderMap(), off the map's own dimensions.
  function relayoutCanvases() {
    if (sizeFpvCanvas()) {
      if (state.lastFrame) drawFPV();
      else renderFpvPlaceholder();
    }
  }

  sizeFpvCanvas();
  renderFpvPlaceholder();
  setEmptyState(document.getElementById("brain-log"), BRAIN_EMPTY[0], BRAIN_EMPTY[1]);
  setEmptyState(document.getElementById("log"), LOG_EMPTY[0], LOG_EMPTY[1]);

  if (window.ResizeObserver) {
    const canvasObserver = new ResizeObserver(relayoutCanvases);
    canvasObserver.observe(document.querySelector(".fpv-wrap"));
  } else {
    window.addEventListener("resize", relayoutCanvases);
  }

  // ---------- restore prior session ----------
  // Precedence for both URL fields: whatever the user last used > a
  // convention-based default derived from this page's own origin > empty.
  // A remembered value always wins, since it's the only one that reflects
  // a setup actually known to work on this device.

  const cfgServerUrlEl = document.getElementById("cfg-server-url");
  const cfgUrlEl = document.getElementById("cfg-url");
  const cfgBrainUrlEl = document.getElementById("cfg-brain-url");

  // If this page is being served BY robot/server.py itself (the ECS
  // deployment, or local `uvicorn robot.server:app`), rather than opened
  // as a local file:// page, the robot server IS this page's own origin --
  // prefill the field instead of making the user type an IP.
  if (location.protocol !== "file:") {
    cfgServerUrlEl.value = location.origin;

    // Best-effort default for the vision service URL too, based on this
    // project's own documented conventions (README.md/CLAUDE.md), not a
    // guess: local dev always runs vision-analyze on :8080 alongside the
    // twin's :8000 (see "local smoke test" in README); the real ECS
    // deployment shares one NLB/hostname between the twin (:8000) and the
    // vision service (the shared ALB's default port, no suffix). Still
    // just a starting point -- editable, and won't be right for one-off
    // setups like a path-prefixed local reverse proxy.
    const isLocalHost = location.hostname === "localhost" || location.hostname === "127.0.0.1";
    cfgUrlEl.value = isLocalHost
      ? location.protocol + "//" + location.hostname + ":8080"
      : location.protocol + "//" + location.hostname;
    derivedEndpoints["cfg-url"] = cfgUrlEl.value;

    // Same convention for the brain: it runs alongside the robot server on
    // :8001 (PLAN-brain-relocation.md's topology), so the page it is served
    // by knows where to look. Only a starting point -- the brain is
    // optional, and it may be on another machine entirely.
    cfgBrainUrlEl.value = location.protocol + "//" + location.hostname + ":8001";
    derivedEndpoints["cfg-brain-url"] = cfgBrainUrlEl.value;
  }

  const savedServerUrl = prefGet(PREF.serverUrl);
  const savedVisionUrl = prefGet(PREF.visionUrl);
  const savedBrainUrl = prefGet(PREF.brainUrl);
  if (savedServerUrl) cfgServerUrlEl.value = savedServerUrl;
  if (savedVisionUrl) cfgUrlEl.value = savedVisionUrl;
  if (savedBrainUrl) cfgBrainUrlEl.value = savedBrainUrl;

  // The model picker's options come from the vision service, so it can only
  // be filled once that URL exists. setGuidanceMode() runs during init well
  // before this point, so its own attempt finds an empty #cfg-url and bails
  // -- this is the call that actually populates the picker on a normal page
  // load, and the URL's change handler below covers the picker being pointed
  // somewhere new afterwards.
  updateModelPickerRow();

  // The vision URL was previously the one setting nothing remembered, so
  // it had to be re-typed after every reload even though its secret was
  // already being persisted right next to it.
  cfgUrlEl.addEventListener("change", function () {
    prefSet(PREF.visionUrl, this.value.trim());
    renderHeaderConnStatus();
    renderEndpointNote();
    // A new service may offer a different model set (the allow-list is
    // server-side, deliberately), so re-ask rather than keeping stale options.
    state.navigateModelsLoaded = false;
    resetNavigateModelOptions();
    fetchNavigateModels();
  });
  cfgBrainUrlEl.addEventListener("change", renderEndpointNote);
  cfgServerUrlEl.addEventListener("change", function () {
    renderEndpointNote();
    prefSet(PREF.serverUrl, this.value.trim());
  });
  cfgBrainUrlEl.addEventListener("change", function () {
    prefSet(PREF.brainUrl, this.value.trim());
  });

  const savedTab = LEGACY_TAB_ALIASES[prefGet(PREF.tab)] || prefGet(PREF.tab);
  if (savedTab && TAB_NAMES.indexOf(savedTab) !== -1) {
    switchTab(savedTab, { restoring: true });
  }

  // Developer readouts: the exact call count and the pan-speed meter. Both
  // are instrumentation for working on this page, not information a person
  // using Guide has any use for, so they're off by default and gated by a
  // single body class the CSS keys off.
  const debugToggleEl = document.getElementById("cfg-debug-readouts");
  function applyDebugReadouts(on) {
    document.body.classList.toggle("debug-readouts", on);
    debugToggleEl.checked = on;
  }
  applyDebugReadouts(prefGet(PREF.debugReadouts) === "1");
  debugToggleEl.addEventListener("change", function () {
    prefSet(PREF.debugReadouts, this.checked ? "1" : "0");
    applyDebugReadouts(this.checked);
  });


  // ---------- QR encoder (byte mode, versions 1-20, EC level M or L) ----------
  // Inline rather than a library: this page is strictly self-contained (no
  // CDN, served by robot/server.py), and the payload here is the shared
  // secrets -- handing those to a third-party QR web service would be a
  // genuine leak, not just an inconvenience. Implements ISO/IEC 18004 byte
  // mode; the block-structure and alignment tables below were generated
  // from a reference implementation rather than typed out by hand.

  // Generated from the ISO/IEC 18004 tables, not hand-transcribed.
  // Per version 1-20 and EC level: [ecCodewordsPerBlock, group1Blocks,
  // group1DataCodewords, group2Blocks, group2DataCodewords].
  const QR_BLOCKS = {
    L: [[7,1,19,0,0],[10,1,34,0,0],[15,1,55,0,0],[20,1,80,0,0],[26,1,108,0,0],[18,2,68,0,0],[20,2,78,0,0],[24,2,97,0,0],[30,2,116,0,0],[18,2,68,2,69],[20,4,81,0,0],[24,2,92,2,93],[26,4,107,0,0],[30,3,115,1,116],[22,5,87,1,88],[24,5,98,1,99],[28,1,107,5,108],[30,5,120,1,121],[28,3,113,4,114],[28,3,107,5,108]],
    M: [[10,1,16,0,0],[16,1,28,0,0],[26,1,44,0,0],[18,2,32,0,0],[24,2,43,0,0],[16,4,27,0,0],[18,4,31,0,0],[22,2,38,2,39],[22,3,36,2,37],[26,4,43,1,44],[30,1,50,4,51],[22,6,36,2,37],[22,8,37,1,38],[24,4,40,5,41],[24,5,41,5,42],[28,7,45,3,46],[28,10,46,1,47],[26,9,43,4,44],[26,3,44,11,45],[26,3,41,13,42]],
  };
  // Alignment-pattern centre coordinates per version (empty for v1).
  const QR_ALIGN = [[],[],[6,18],[6,22],[6,26],[6,30],[6,34],[6,22,38],[6,24,42],[6,26,46],[6,28,50],[6,30,54],[6,32,58],[6,34,62],[6,26,46,66],[6,26,48,70],[6,26,50,74],[6,30,54,78],[6,30,56,82],[6,30,58,86],[6,34,62,90]];

  // GF(256) for Reed-Solomon, primitive polynomial 0x11D.
  const GF_EXP = new Uint8Array(512), GF_LOG = new Uint8Array(256);
  (function initGaloisField() {
    let x = 1;
    for (let i = 0; i < 255; i++) {
      GF_EXP[i] = x;
      GF_LOG[x] = i;
      x <<= 1;
      if (x & 0x100) x ^= 0x11D;
    }
    for (let i = 255; i < 512; i++) GF_EXP[i] = GF_EXP[i - 255];
  })();
  function gfMul(a, b) { return (a === 0 || b === 0) ? 0 : GF_EXP[GF_LOG[a] + GF_LOG[b]]; }

  // Generator polynomial: product of (x - alpha^i) for i in [0, degree).
  // Coefficients are highest-degree-first.
  function rsGenerator(degree) {
    let poly = [1];
    for (let i = 0; i < degree; i++) {
      const next = new Array(poly.length + 1).fill(0);
      for (let j = 0; j < poly.length; j++) {
        next[j] ^= poly[j];                        // multiply by x
        next[j + 1] ^= gfMul(poly[j], GF_EXP[i]);  // ...and by alpha^i
      }
      poly = next;
    }
    return poly;
  }

  function rsRemainder(data, ecLen) {
    const gen = rsGenerator(ecLen);
    const buf = new Uint8Array(data.length + ecLen);
    buf.set(data);
    for (let i = 0; i < data.length; i++) {
      const factor = buf[i];
      if (factor === 0) continue;
      for (let j = 0; j < gen.length; j++) buf[i + j] ^= gfMul(gen[j], factor);
    }
    return buf.slice(data.length);
  }

  function qrDataCapacity(version, ecc) {
    const spec = QR_BLOCKS[ecc][version - 1];
    return spec[1] * spec[2] + spec[3] * spec[4];
  }
  // Byte mode's character-count indicator widens at version 10.
  function qrCharCountBits(version) { return version <= 9 ? 8 : 16; }

  function qrChooseVersion(byteLen, ecc) {
    for (let v = 1; v <= QR_BLOCKS[ecc].length; v++) {
      if (4 + qrCharCountBits(v) + byteLen * 8 <= qrDataCapacity(v, ecc) * 8) return v;
    }
    return 0;
  }

  // Data codewords -> padded, split into blocks, RS-encoded, interleaved.
  function qrCodewords(bytes, version, ecc) {
    const spec = QR_BLOCKS[ecc][version - 1];
    const ecLen = spec[0], g1n = spec[1], g1d = spec[2], g2n = spec[3], g2d = spec[4];
    const totalData = g1n * g1d + g2n * g2d;

    const bits = [];
    const put = function (value, length) {
      for (let i = length - 1; i >= 0; i--) bits.push((value >>> i) & 1);
    };
    put(4, 4);                                  // byte mode
    put(bytes.length, qrCharCountBits(version));
    for (let i = 0; i < bytes.length; i++) put(bytes[i], 8);

    const capacity = totalData * 8;
    for (let i = 0; i < 4 && bits.length < capacity; i++) bits.push(0); // terminator
    while (bits.length % 8 !== 0) bits.push(0);
    for (let i = 0; bits.length < capacity; i++) put(i % 2 === 0 ? 0xEC : 0x11, 8);

    const dataCw = new Uint8Array(totalData);
    for (let i = 0; i < totalData; i++) {
      let v = 0;
      for (let j = 0; j < 8; j++) v = (v << 1) | bits[i * 8 + j];
      dataCw[i] = v;
    }

    const blocks = [];
    let off = 0;
    for (let i = 0; i < g1n; i++) { blocks.push(dataCw.slice(off, off + g1d)); off += g1d; }
    for (let i = 0; i < g2n; i++) { blocks.push(dataCw.slice(off, off + g2d)); off += g2d; }
    const ecBlocks = blocks.map(function (b) { return rsRemainder(b, ecLen); });

    // Interleave: column-wise across blocks, data first then EC.
    const out = [];
    const longest = Math.max(g1d, g2d);
    for (let i = 0; i < longest; i++) {
      for (let b = 0; b < blocks.length; b++) if (i < blocks[b].length) out.push(blocks[b][i]);
    }
    for (let i = 0; i < ecLen; i++) {
      for (let b = 0; b < ecBlocks.length; b++) out.push(ecBlocks[b][i]);
    }
    return out;
  }

  const QR_MASKS = [
    function (x, y) { return (x + y) % 2 === 0; },
    function (x, y) { return y % 2 === 0; },
    function (x, y) { return x % 3 === 0; },
    function (x, y) { return (x + y) % 3 === 0; },
    function (x, y) { return (Math.floor(y / 2) + Math.floor(x / 3)) % 2 === 0; },
    function (x, y) { return (x * y) % 2 + (x * y) % 3 === 0; },
    function (x, y) { return ((x * y) % 2 + (x * y) % 3) % 2 === 0; },
    function (x, y) { return ((x + y) % 2 + (x * y) % 3) % 2 === 0; },
  ];

  // BCH(15,5), then XOR with the spec's fixed mask so an all-zero format
  // never produces an all-zero pattern.
  function qrFormatBits(ecc, mask) {
    const data = ((ecc === "L" ? 1 : 0) << 3) | mask;
    let rem = data;
    for (let i = 0; i < 10; i++) rem = (rem << 1) ^ ((rem >>> 9) * 0x537);
    return ((data << 10) | rem) ^ 0x5412;
  }
  function qrVersionBits(version) {
    let rem = version;
    for (let i = 0; i < 12; i++) rem = (rem << 1) ^ ((rem >>> 11) * 0x1F25);
    return (version << 12) | rem;
  }

  function qrBuildMatrix(version, ecc, codewords, mask) {
    const size = version * 4 + 17;
    const m = [], fixed = [];
    for (let i = 0; i < size; i++) {
      m.push(new Array(size).fill(0));
      fixed.push(new Array(size).fill(false));
    }
    const setFixed = function (x, y, v) {
      if (x < 0 || y < 0 || x >= size || y >= size) return;
      m[y][x] = v ? 1 : 0;
      fixed[y][x] = true;
    };

    // Finder patterns plus their separators (the ring of light modules).
    [[0, 0], [size - 7, 0], [0, size - 7]].forEach(function (o) {
      for (let dy = -1; dy <= 7; dy++) {
        for (let dx = -1; dx <= 7; dx++) {
          const inside = dx >= 0 && dx <= 6 && dy >= 0 && dy <= 6;
          const d = Math.max(Math.abs(dx - 3), Math.abs(dy - 3));
          setFixed(o[0] + dx, o[1] + dy, inside && d !== 2);
        }
      }
    });

    // Timing patterns.
    for (let i = 8; i < size - 8; i++) {
      setFixed(i, 6, i % 2 === 0);
      setFixed(6, i, i % 2 === 0);
    }

    // Alignment patterns, minus the three that would sit on a finder.
    const centres = QR_ALIGN[version];
    for (let a = 0; a < centres.length; a++) {
      for (let b = 0; b < centres.length; b++) {
        const cx = centres[a], cy = centres[b];
        const onFinder = (a === 0 && b === 0) ||
                         (a === 0 && b === centres.length - 1) ||
                         (a === centres.length - 1 && b === 0);
        if (onFinder) continue;
        for (let dy = -2; dy <= 2; dy++) {
          for (let dx = -2; dx <= 2; dx++) {
            setFixed(cx + dx, cy + dy, Math.max(Math.abs(dx), Math.abs(dy)) !== 1);
          }
        }
      }
    }

    // Reserve the format-info strips (written properly further down) and
    // the always-dark module, so data placement skips them.
    // Index 6 is skipped: the format strip runs alongside the timing
    // patterns but does not include them, and writing here would erase the
    // timing modules at (8,6) and (6,8).
    for (let i = 0; i <= 8; i++) {
      if (i === 6) continue;
      setFixed(8, i, false);
      setFixed(i, 8, false);
    }
    for (let i = 0; i < 8; i++) { setFixed(size - 1 - i, 8, false); setFixed(8, size - 1 - i, false); }
    setFixed(8, size - 8, true);

    if (version >= 7) {
      const vbits = qrVersionBits(version);
      for (let i = 0; i < 18; i++) {
        const bit = (vbits >>> i) & 1;
        setFixed(size - 11 + (i % 3), Math.floor(i / 3), bit);
        setFixed(Math.floor(i / 3), size - 11 + (i % 3), bit);
      }
    }

    // Data, in two-module columns snaking up and down, skipping column 6.
    let bitIdx = 0, upward = true;
    for (let right = size - 1; right >= 1; right -= 2) {
      if (right === 6) right = 5;
      for (let vert = 0; vert < size; vert++) {
        const y = upward ? size - 1 - vert : vert;
        for (let k = 0; k < 2; k++) {
          const x = right - k;
          if (fixed[y][x]) continue;
          let bit = 0;
          if (bitIdx < codewords.length * 8) {
            bit = (codewords[bitIdx >> 3] >>> (7 - (bitIdx & 7))) & 1;
          }
          bitIdx++;
          m[y][x] = bit ^ (QR_MASKS[mask](x, y) ? 1 : 0);
        }
      }
      upward = !upward;
    }

    const fbits = qrFormatBits(ecc, mask);
    const fbit = function (i) { return (fbits >>> i) & 1; };
    for (let i = 0; i <= 5; i++) setFixed(8, i, fbit(i));
    setFixed(8, 7, fbit(6));
    setFixed(8, 8, fbit(7));
    setFixed(7, 8, fbit(8));
    for (let i = 9; i < 15; i++) setFixed(14 - i, 8, fbit(i));
    for (let i = 0; i < 8; i++) setFixed(size - 1 - i, 8, fbit(i));
    for (let i = 8; i < 15; i++) setFixed(8, size - 15 + i, fbit(i));
    setFixed(8, size - 8, 1);

    return m;
  }

  // The four penalty rules from the spec; the lowest-scoring mask wins.
  function qrPenalty(m) {
    const size = m.length;
    let score = 0;

    for (let pass = 0; pass < 2; pass++) {
      for (let i = 0; i < size; i++) {
        let run = 1;
        for (let j = 1; j < size; j++) {
          const cur = pass === 0 ? m[i][j] : m[j][i];
          const prev = pass === 0 ? m[i][j - 1] : m[j - 1][i];
          if (cur === prev) {
            run++;
            if (run === 5) score += 3;
            else if (run > 5) score += 1;
          } else run = 1;
        }
      }
    }

    for (let y = 0; y < size - 1; y++) {
      for (let x = 0; x < size - 1; x++) {
        const v = m[y][x];
        if (v === m[y][x + 1] && v === m[y + 1][x] && v === m[y + 1][x + 1]) score += 3;
      }
    }

    // 1:1:3:1:1 finder-like sequences, with four light modules either side.
    const PATTERNS = [
      [1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0],
      [0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1],
    ];
    for (let pass = 0; pass < 2; pass++) {
      for (let i = 0; i < size; i++) {
        for (let j = 0; j + 11 <= size; j++) {
          for (let p = 0; p < 2; p++) {
            let hit = true;
            for (let k = 0; k < 11; k++) {
              const v = pass === 0 ? m[i][j + k] : m[j + k][i];
              if (v !== PATTERNS[p][k]) { hit = false; break; }
            }
            if (hit) score += 40;
          }
        }
      }
    }

    let dark = 0;
    for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) dark += m[y][x];
    const percent = (dark * 100) / (size * size);
    score += Math.floor(Math.abs(percent - 50) / 5) * 10;
    return score;
  }

  // Returns { matrix, version, ecc, mask } or throws if the text cannot fit.
  // forcedMask exists so the encoder can be checked module-for-module
  // against a reference implementation at a known mask.
  function qrEncode(text, forcedMask) {
    const bytes = Array.from(new TextEncoder().encode(text));
    // Prefer M (roughly 15% recoverable) since this is scanned off a
    // screen; fall back to L only if the payload will not otherwise fit.
    let ecc = "M", version = qrChooseVersion(bytes.length, ecc);
    if (!version) { ecc = "L"; version = qrChooseVersion(bytes.length, ecc); }
    if (!version) throw new Error("Too much data for a QR code (" + bytes.length + " bytes).");

    const codewords = qrCodewords(bytes, version, ecc);
    if (typeof forcedMask === "number") {
      return { matrix: qrBuildMatrix(version, ecc, codewords, forcedMask), version: version, ecc: ecc, mask: forcedMask };
    }
    let best = null;
    for (let mask = 0; mask < 8; mask++) {
      const matrix = qrBuildMatrix(version, ecc, codewords, mask);
      const penalty = qrPenalty(matrix);
      if (!best || penalty < best.penalty) best = { matrix: matrix, penalty: penalty, mask: mask };
    }
    return { matrix: best.matrix, version: version, ecc: ecc, mask: best.mask };
  }

  // Drawn light-on-dark-agnostic: always true black on true white, never
  // the page's theme colours -- scanners need the contrast, and a dark-mode
  // QR in muted greys is measurably worse to read.
  function qrRenderToCanvas(canvas, matrix, targetPx) {
    const QUIET = 4;
    const size = matrix.length + QUIET * 2;
    const scale = Math.max(2, Math.floor(targetPx / size));
    const px = size * scale;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = px * dpr;
    canvas.height = px * dpr;
    canvas.style.width = px + "px";
    canvas.style.height = px + "px";
    const c = canvas.getContext("2d");
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.fillStyle = "#FFFFFF";
    c.fillRect(0, 0, px, px);
    c.fillStyle = "#000000";
    for (let y = 0; y < matrix.length; y++) {
      for (let x = 0; x < matrix.length; x++) {
        if (matrix[y][x]) c.fillRect((x + QUIET) * scale, (y + QUIET) * scale, scale, scale);
      }
    }
  }


  // ---------- setup sharing ----------
  // A QR of the existing magic link (see prefillTesterSecrets below), which
  // is a better delivery vehicle than pasting that URL into a chat: it
  // doesn't persist in someone's message history, and there's nothing to
  // retype. It is still the secrets in visual form, though, so it is
  // revealed on an explicit tap rather than sitting in Settings
  // permanently, and it takes itself back down after a minute.
  const QR_REVEAL_MS = 60000;

  // Why the connection failed, in the words of whatever actually failed.
  //
  // Two genuinely different situations were being reported identically:
  // the request never arrived (wrong URL, server down, CORS), and the
  // server answered with a precise complaint. Only the first is worth
  // advice about networks and origins; repeating it for the second sends
  // someone to check three things that are already fine while the real
  // answer sits in the message they were told to ignore.
  // What failed, how it should look, and whether it is worth interrupting
  // someone about.
  //
  // Everything here used to be one red "Not connected" plus a toast
  // reading "Couldn't reach the robot server" -- which was shown, at one
  // point, directly above a panel explaining that the server was running.
  // A state that needs no action should not be styled as a fault or
  // announce itself, and a state that does need action should say which.
  function connectFailure(url, e, silent) {
    const host = url.replace(/^https?:\/\//, "").split("/")[0];

    if (!e.status) {
      return {
        state: "err", label: "Not connected",
        detail: silent
          ? url + " didn't respond. Tap Connect to retry."
          : "Could not reach " + url + " (" + e.message + "). Check the server is "
            + "running, on the same network, and that CORS/allowed_origins in "
            + "config/robot.yaml permits this page's origin.",
        toast: "Couldn't reach the robot server.",
      };
    }

    // A teleop server has no camera of its own -- it serves whatever frame
    // a phone last pushed, so before a walk it has nothing to give. That is
    // the normal resting state of a correctly configured deployment, not a
    // fault, and there is nothing for anyone to do about it here. Neutral
    // styling, no toast: Robot view connects this itself when it starts
    // pushing frames.
    if (e.status === 503 && /frame/i.test(e.message)) {
      return {
        state: "idle", label: "Waiting for a walk",
        detail: host + " is running and simply has no camera frame yet, which "
          + "is normal before a walk. Nothing to do here \u2014 start Robot "
          + "view with \u201cDrive via brain\u201d and this connects itself.",
        toast: null,
      };
    }

    if (e.status === 401 || e.status === 403) {
      return {
        state: "err", label: "Secret rejected",
        detail: host + " is reachable, so this is the wrong secret for THIS "
          + "deployment rather than a network problem. Check it is the one "
          + host + " expects.",
        toast: "The robot server rejected that secret.",
      };
    }

    return {
      state: "err", label: "Not connected",
      detail: host + " answered with an error (HTTP " + e.status + "): " + e.message,
      toast: "The robot server returned an error.",
    };
  }

  function setupLinkParts() {
    return {
      visionSecret: document.getElementById("cfg-secret").value.trim(),
      robotSecret: document.getElementById("cfg-server-secret").value.trim(),
      brainSecret: document.getElementById("cfg-brain-secret").value.trim(),
      visionUrl: document.getElementById("cfg-url").value.trim(),
      serverUrl: document.getElementById("cfg-server-url").value.trim().replace(/\/$/, ""),
      brainUrl: document.getElementById("cfg-brain-url").value.trim().replace(/\/$/, ""),
    };
  }

  // Mirrors the default computed during init, so the two stay in step.
  function conventionalVisionUrl() {
    if (location.protocol === "file:") return "";
    const isLocalHost = location.hostname === "localhost" || location.hostname === "127.0.0.1";
    return isLocalHost
      ? location.protocol + "//" + location.hostname + ":8080"
      : location.protocol + "//" + location.hostname;
  }

  function buildSetupUrl() {
    const p = setupLinkParts();
    const params = new URLSearchParams();
    if (p.visionSecret) params.set("secret", p.visionSecret);
    if (p.robotSecret) params.set("robotSecret", p.robotSecret);
    if (p.brainSecret) params.set("brainSecret", p.brainSecret);
    // The vision URL is worth carrying when it differs from what the
    // recipient's own page would derive (a tunnel, a path-prefixed proxy).
    // When it matches, omitting it keeps the payload short, which is what
    // decides the QR's version and therefore how big its modules are.
    if (p.visionUrl && p.visionUrl !== conventionalVisionUrl()) {
      params.set("visionUrl", p.visionUrl);
    }
    // The robot server is this page's own origin in every normal
    // deployment and the recipient derives that themselves, so it only
    // earns its payload length when pointed somewhere else.
    if (p.serverUrl && p.serverUrl !== location.origin) params.set("serverUrl", p.serverUrl);
    // Only worth carrying once someone has actually connected to a brain:
    // an unused derived default would cost QR payload for nothing.
    if (p.brainUrl && prefGet(PREF.brainUrl)) params.set("brainUrl", p.brainUrl);
    const qs = params.toString();
    return location.origin + location.pathname + (qs ? "?" + qs : "");
  }

  function hideSetupQr() {
    document.getElementById("qr-reveal").hidden = true;
    if (qrHideTimer) { clearTimeout(qrHideTimer); qrHideTimer = null; }
    if (qrCountdownTimer) { clearInterval(qrCountdownTimer); qrCountdownTimer = null; }
  }

  function showSetupQr() {
    if (location.protocol === "file:") {
      showToast("Open this page from a server first \u2014 a file:// path can't be shared.", "err");
      return;
    }
    const url = buildSetupUrl();
    let code;
    try {
      code = qrEncode(url);
    } catch (e) {
      showToast("Setup link is too long to fit in a QR code.", "err");
      return;
    }
    const reveal = document.getElementById("qr-reveal");
    // Unhide before measuring: a hidden container reports clientWidth 0,
    // which would silently pin every screen to the fallback size.
    reveal.hidden = false;
    const frame = document.querySelector(".qr-frame");
    const avail = Math.max(220, frame.clientWidth - 24); // minus .qr-frame padding
    qrRenderToCanvas(document.getElementById("qr-canvas"), code.matrix, avail);

    const p = setupLinkParts();
    const carried = [];
    if (p.visionSecret) carried.push("vision secret");
    if (p.robotSecret) carried.push("robot secret");
    if (p.brainSecret) carried.push("brain secret");
    const what = carried.length
      ? "Carries the " + carried.join(" and ") + "."
      : "No secrets are configured, so this only opens the app.";

    const meta = document.getElementById("qr-meta");

    let remaining = Math.round(QR_REVEAL_MS / 1000);
    const tick = function () {
      meta.innerHTML = escapeHtml(what) +
        '<br><span class="qr-countdown">Hides in ' + remaining + 's</span>';
    };
    tick();
    qrCountdownTimer = setInterval(function () {
      remaining -= 1;
      if (remaining >= 0) tick();
    }, 1000);
    qrHideTimer = setTimeout(hideSetupQr, QR_REVEAL_MS);
  }

  document.getElementById("btn-show-qr").onclick = showSetupQr;
  document.getElementById("btn-hide-qr").onclick = hideSetupQr;

  // ---------- tester debug capture ----------
  // For a build handed to a group of remote testers, "it broke" with no
  // further detail is hard to act on. Every uncaught error/rejection gets
  // appended to a small ring buffer in localStorage; "Copy debug info" in
  // Settings bundles the last 20 of those plus basic context into one
  // paste-able block for a bug report, no screen recording required.
  document.getElementById("btn-copy-debug").onclick = function () {
    const statusEl = document.getElementById("copy-debug-status");
    let errors = [];
    try { errors = JSON.parse(localStorage.getItem(DEBUG_LOG_KEY) || "[]"); } catch (e) { /* best-effort */ }
    const text = JSON.stringify({
      url: location.href,
      userAgent: navigator.userAgent,
      timestamp: new Date().toISOString(),
      errors: errors,
    }, null, 2);

    function reportCopied() {
      statusEl.textContent = errors.length
        ? "Copied (" + errors.length + " recent error(s) included)."
        : "Copied -- no errors logged since this browser last cleared its storage.";
      showToast(errors.length ? "Debug info copied (" + errors.length + " errors)." : "Debug info copied.", "ok");
    }
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(text).then(reportCopied).catch(function () {
        console.log(text);
        statusEl.textContent = "Couldn't copy automatically -- logged to the browser console instead.";
      });
    } else {
      console.log(text);
      statusEl.textContent = "Clipboard unavailable -- logged to the browser console instead.";
    }
  };

  // ---------- recent search-target history ----------
  // Shared between the Guide and Vision Autopilot target fields (same
  // <datalist>, same localStorage key) since they're conceptually the
  // same "what am I looking for" input -- typing "red backpack" fresh
  // every session was pure friction once you've searched for the same
  // handful of things more than once.
  const TARGET_HISTORY_KEY = "vp_target_history";
  const TARGET_HISTORY_MAX = 8;
  function loadTargetHistory() {
    try { return JSON.parse(localStorage.getItem(TARGET_HISTORY_KEY) || "[]"); }
    catch (e) { return []; }
  }
  function renderTargetHistory(list) {
    const dl = document.getElementById("target-history-list");
    if (!dl) return;
    dl.innerHTML = "";
    (list || loadTargetHistory()).forEach(function (t) {
      const opt = document.createElement("option");
      opt.value = t;
      dl.appendChild(opt);
    });
  }
  function saveTargetToHistory(target) {
    if (!target) return;
    try {
      let list = loadTargetHistory().filter(function (t) { return t.toLowerCase() !== target.toLowerCase(); });
      list.unshift(target);
      list = list.slice(0, TARGET_HISTORY_MAX);
      localStorage.setItem(TARGET_HISTORY_KEY, JSON.stringify(list));
      renderTargetHistory(list);
    } catch (e) { /* best-effort -- localStorage unavailable, just skip persisting */ }
  }

  // ---------- tester onboarding: magic-link secret prefill ----------
  // For handing this build to a group of testers, a shareable link that
  // carries the shared secret(s) as query params means nobody has to be
  // told "go copy this value into Settings." Saved to localStorage after
  // the first load, so the link only needs to be opened once -- revisiting
  // the plain URL (or a home-screen bookmark) keeps working after that.
  // The query string is stripped from the address bar immediately after
  // reading it, so the secret doesn't linger visibly there or get carried
  // along if someone re-shares the link straight from their address bar.
  (function prefillTesterSecrets() {
    // localStorage can throw in some private-browsing modes -- never let
    // that take down the rest of page init (icons, header status) below.
    try {
      const params = new URLSearchParams(location.search);
      const urlSecret = params.get("secret");
      const urlRobotSecret = params.get("robotSecret");
      const urlBrainSecret = params.get("brainSecret");
      // The setup QR also carries the service URLs, since the
      // convention-derived defaults computed above are wrong for plenty of
      // real setups (a tunnel, a path-prefixed proxy) -- and the point of
      // scanning is to have nothing left to type.
      const urlVisionUrl = params.get("visionUrl");
      const urlServerUrl = params.get("serverUrl");
      const urlBrainUrl = params.get("brainUrl");
      if (urlSecret) localStorage.setItem("vp_cfg_secret", urlSecret);
      if (urlRobotSecret) localStorage.setItem("vp_cfg_server_secret", urlRobotSecret);
      if (urlBrainSecret) localStorage.setItem("vp_cfg_brain_secret", urlBrainSecret);
      if (urlVisionUrl) localStorage.setItem(PREF.visionUrl, urlVisionUrl);
      if (urlServerUrl) localStorage.setItem(PREF.serverUrl, urlServerUrl);
      if (urlBrainUrl) localStorage.setItem(PREF.brainUrl, urlBrainUrl);
      if (urlSecret || urlRobotSecret || urlBrainSecret || urlVisionUrl || urlServerUrl || urlBrainUrl) {
        history.replaceState(null, "", location.pathname + location.hash);
      }

      const cfgSecretEl = document.getElementById("cfg-secret");
      const cfgServerSecretEl = document.getElementById("cfg-server-secret");
      const cfgBrainSecretEl = document.getElementById("cfg-brain-secret");
      const savedSecret = localStorage.getItem("vp_cfg_secret");
      const savedRobotSecret = localStorage.getItem("vp_cfg_server_secret");
      const savedBrainSecret = localStorage.getItem("vp_cfg_brain_secret");
      if (savedSecret) cfgSecretEl.value = savedSecret;
      if (savedRobotSecret) cfgServerSecretEl.value = savedRobotSecret;
      if (savedBrainSecret) cfgBrainSecretEl.value = savedBrainSecret;
      // Applied to the fields directly: the restore block above already ran
      // and read localStorage before these query params were stored.
      if (urlVisionUrl) document.getElementById("cfg-url").value = urlVisionUrl;
      if (urlServerUrl) document.getElementById("cfg-server-url").value = urlServerUrl;
      if (urlBrainUrl) document.getElementById("cfg-brain-url").value = urlBrainUrl;

      // Also remember anything a tester types in by hand, not just what
      // arrived via the link, so a manual correction sticks too.
      cfgSecretEl.addEventListener("change", function () {
        try { localStorage.setItem("vp_cfg_secret", this.value.trim()); } catch (e) { /* best-effort */ }
      });
      cfgServerSecretEl.addEventListener("change", function () {
        try { localStorage.setItem("vp_cfg_server_secret", this.value.trim()); } catch (e) { /* best-effort */ }
      });
      cfgBrainSecretEl.addEventListener("change", function () {
        try { localStorage.setItem("vp_cfg_brain_secret", this.value.trim()); } catch (e) { /* best-effort */ }
      });
    } catch (e) { /* best-effort -- localStorage unavailable, just skip prefill */ }
  })();

  renderTargetHistory();
  renderHeaderConnStatus();

  // After applyStaticIcons() (called near the top), which stamps the
  // button's data-icon="volumeOn" default and would otherwise overwrite a
  // restored muted state.
  state.guidanceMuted = prefGet(PREF.muted) === "1";
  updateGuidanceMuteButton();

  // Last, so it runs after prefillTesterSecrets() above has populated the
  // secret fields -- connect() reads cfg-server-secret, and a deployment
  // that requires one would otherwise auto-connect without it and fail.
  // Reconnecting here rather than leaving every control disabled behind a
  // Connect button on a tab the user has to go find: the URL is either
  // remembered or derived from this page's own origin, so in the common
  // case there is nothing left to ask. Failure is quiet and non-blocking
  // (see connect()'s `silent`); the manual button remains the retry path.
  if (document.getElementById("cfg-server-url").value.trim()) {
    connect({ silent: true });
  }

  // The brain is optional, so its silent attempt is only made when a URL
  // was actually remembered from a previous session -- the derived
  // localhost:8001 default should not produce a failure message on every
  // load for the many setups that run no brain service at all.
  const brainPolicyEl = document.getElementById("brain-policy");
  const savedPolicy = prefGet(PREF.brainPolicy);
  if (savedPolicy === "vision" || savedPolicy === "frontier" || savedPolicy === "tiered") {
    state.brainPolicy = savedPolicy;
    brainPolicyEl.value = savedPolicy;
    // A remembered "vision" has to bring the Guide tab's pickers back with
    // it, or the panel names a model nobody can see or change.
    updateModelPickerRow();
  }
  updateBrainControls();
  const recordToggleEl = document.getElementById("cfg-record-walk");
  state.recordWalk = prefGet(PREF.recordWalk) === "1";
  recordToggleEl.checked = state.recordWalk;
  updateRecordRow();
  const driveViaBrainToggleEl = document.getElementById("cfg-drive-via-brain");
  state.driveViaBrain = prefGet(PREF.driveViaBrain) === "1";
  driveViaBrainToggleEl.checked = state.driveViaBrain;
  const drivePolicyEl = document.getElementById("cfg-drive-policy");
  const savedDrivePolicy = prefGet(PREF.drivePolicy);
  if (savedDrivePolicy === "vision" || savedDrivePolicy === "tiered") {
    state.drivePolicy = savedDrivePolicy;
    drivePolicyEl.value = savedDrivePolicy;
  }
  updateDriveViaBrainRow();
  if (prefGet(PREF.brainUrl)) {
    connectBrain({ silent: true });
  }
  startBrainLiveness();
  renderEnvBanner();
  renderEndpointNote();
})();
