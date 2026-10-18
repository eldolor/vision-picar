// control/admin.js -- the recorded-walk admin console's client.
//
// Extracted verbatim from admin.html; served by control/admin_server.py
// (GET /admin.js). Same reasoning as web-twin/app.js -- see its banner.

(function () {
  const SECRET_KEY = "vp_admin_secret";
  const secretEl = document.getElementById("secret");
  const connStatusEl = document.getElementById("conn-status");
  const statsEl = document.getElementById("stats-line");
  const walksEl = document.getElementById("walks");
  const searchEl = document.getElementById("search");
  const sortEl = document.getElementById("sort");
  const bulkBarEl = document.getElementById("bulk-bar");
  const bulkCountEl = document.getElementById("bulk-count");
  const btnBulkDelete = document.getElementById("btn-bulk-delete");
  const btnConnect = document.getElementById("btn-connect");
  const btnRefresh = document.getElementById("btn-refresh");

  let secret = "";
  try { secret = localStorage.getItem(SECRET_KEY) || ""; } catch (e) { /* private mode */ }
  secretEl.value = secret;

  // All walks currently known (from the last /recording/walks fetch), plus
  // UI state layered on top -- selection for bulk delete, cached per-walk
  // detail (frames + navigate entries) so View and the slideshow don't
  // re-fetch, and search/sort applied purely client-side since the walk
  // count here is small enough that a server-side index would be
  // premature.
  let allWalks = [];
  // walk name -> the repaint hooks for its row, so a score arriving from the
  // background scoring queue can update one row in place. Rebuilt by
  // renderWalksList on every render, so it never outlives the DOM it points at.
  let walkRenderers = {};
  const selected = new Set();
  const detailCache = new Map(); // walk name -> {frames, entries, label}
  // walk name -> what is still out for it (3.76 N2). A row is rebuilt on
  // every search keystroke, sort and Refresh, and a rebuilt row started with
  // every control enabled -- a second paid replay or judge run, or a second
  // DELETE, one click away while the first was out.
  const inFlight = new Map();
  function flight(name) {
    let f = inFlight.get(name);
    if (!f) { f = { replays: 0, scoring: false, queued: false, deleting: false }; inFlight.set(name, f); }
    return f;
  }
  // The walk as the page knows it NOW: a request started before a Refresh
  // writes its answer onto the object the list holds today, not the one it
  // started from (3.76 N2: Refresh during auto-scoring lost every score).
  function live(w) {
    return allWalks.find(function (x) { return x.walk === w.walk; }) || w;
  }
  function repaint(name) {
    const r = walkRenderers[name];
    if (r) r.repaint();
  }

  function headers(extra, trying) {
    const h = Object.assign({}, extra);
    const s = trying === undefined ? secret : trying;
    if (s) h["x-app-secret"] = s;
    return h;
  }

  // `trying`: a secret being tried by Connect, sent on this request only.
  async function api(method, path, body, trying) {
    const opts = { method: method, headers: headers(body ? { "content-type": "application/json" } : {}, trying) };
    if (body !== undefined) opts.body = JSON.stringify(body);
    const res = await fetch(path, opts);
    if (res.status === 204) return null;
    const data = await res.json().catch(function () { return {}; });
    if (!res.ok) {
      // FastAPI's `detail` as text (a 422's is a list; 3.64).
      const detail = data.detail == null ? ""
        : (typeof data.detail === "string" ? data.detail : JSON.stringify(data.detail));
      const err = new Error(detail || ("HTTP " + res.status));
      err.status = res.status;            // 3.64: a refusal is not a lost response
      // Our own servers always say why; a gateway's timeout page does not.
      // Judged from the body, not the message text.
      err.fromServer = detail !== "";
      throw err;
    }
    return data;
  }

  async function frameBlobUrl(walk, file) {
    const res = await fetch("/recording/walks/" + encodeURIComponent(walk) + "/frames/" + encodeURIComponent(file), {
      headers: headers(),
    });
    if (!res.ok) throw new Error("HTTP " + res.status);
    const blob = await res.blob();
    return URL.createObjectURL(blob);
  }

  function setStatus(el, text, cls) {
    el.textContent = text;
    el.className = "status" + (cls ? " " + cls : "");
  }

  // Same reasoning against alert(): an inline, self-clearing message next
  // to the button rather than a blocking dialog.
  function showInlineError(btn, msg) {
    let span = btn.nextElementSibling;
    if (!span || !span.classList || !span.classList.contains("inline-err")) {
      span = document.createElement("span");
      span.className = "status err inline-err";
      btn.insertAdjacentElement("afterend", span);
    }
    span.textContent = msg;
    clearTimeout(span._timer);
    span._timer = setTimeout(function () { span.remove(); }, 6000);
  }

  // Two-tap confirm rather than a native confirm() dialog -- keeps this
  // page usable from automated browser control (a blocking native dialog
  // there stalls everything), and reads just as clearly for a human.
  function armConfirm(btn, label, onConfirm) {
    let armed = false;
    let timer = null;
    btn.textContent = label;
    btn.onclick = function () {
      if (!armed) {
        armed = true;
        btn.classList.add("confirming");
        btn.textContent = "Really delete? Click again";
        timer = setTimeout(function () {
          armed = false;
          btn.classList.remove("confirming");
          btn.textContent = label;
        }, 4000);
        return;
      }
      clearTimeout(timer);
      // Disarmed before the action (3.76 N2): left armed, bulk delete's next
      // selection went with ONE click.
      armed = false;
      btn.classList.remove("confirming");
      btn.textContent = label;
      onConfirm();
    };
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, function (c) {
      return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c];
    });
  }

  function fmtBytes(n) {
    if (n >= 1024 * 1024) return (n / (1024 * 1024)).toFixed(1) + " MB";
    if (n >= 1024) return (n / 1024).toFixed(0) + " KB";
    return n + " B";
  }

  // The per-model table that was being assembled by hand after every batch
  // of walks -- differently each time, and wrong twice.
  let summarySeq = 0;
  async function loadSummary() {
    const panel = document.getElementById("summary-panel");
    const el = document.getElementById("summary");
    const seq = ++summarySeq;               // only the newest ask paints (3.76 N2)
    try {
      const data = await api("GET", "/recording/summary");
      if (seq !== summarySeq) return;
      const rows = data.rows || [];
      if (!rows.length) { panel.style.display = "none"; return; }
      el.innerHTML = rows.map(function (r) {
        const variant = r.prompt_variant && r.prompt_variant !== "default"
          ? " · " + escapeHtml(r.prompt_variant) : "";
        const hit = r.collisions
          ? ' <span class="summary-hit">· ' + r.collisions + " hit something</span>" : "";
        const flags = Object.keys(r.flags || {})
          .filter(function (f) { return f !== "collision"; })
          .map(function (f) { return f + " x" + r.flags[f]; }).join(", ");
        return '<div class="summary-row">' +
          '<span class="summary-model">' + escapeHtml(shortModel(r.model_id)) + variant +
            // 3.76 F2: a judged mean and a metrics-only one are separate rows
            // (two formulas); the label says which this is.
            ' <span class="status">' + escapeHtml(r.source +
              (r.basis === "metrics-only" && r.source === "recorded" ? " · no judge" : "")) +
            "</span></span>" +
          '<span class="summary-num">' + r.mean_score + " avg</span>" +
          '<span class="summary-num">' + Math.round(r.reach_rate * 100) + "% reached</span>" +
          '<span class="summary-num">' + r.walks + (r.walks === 1 ? " walk" : " walks") + "</span>" +
          '<span class="summary-sub">best ' + r.best + " · worst " + r.worst +
            (r.median_frames ? " · median " + r.median_frames + " frames" : "") +
            (flags ? " · " + escapeHtml(flags) : "") + hit + "</span>" +
          "</div>";
      }).join("");
      panel.style.display = "";
    } catch (e) {
      if (seq === summarySeq) panel.style.display = "none";
    }
  }

  async function loadStats() {
    try {
      const s = await api("GET", "/stats");
      statsEl.textContent = s.walks + " walk" + (s.walks === 1 ? "" : "s") +
        " · " + s.frames + " frame" + (s.frames === 1 ? "" : "s") +
        " · " + fmtBytes(s.bytes);
    } catch (e) {
      statsEl.textContent = "";
    }
  }

  // Models offered for replay. Fetched from the admin server, which relays
  // the vision service's allow-list -- the console must not carry its own
  // copy of model ids, for the same reason the twin's picker doesn't.
  let replayModels = [];
  let replayPrompts = [];

  async function loadReplayModels() {
    try {
      const data = await api("GET", "/recording/models");
      replayModels = Array.isArray(data.models) ? data.models : [];
      // Prompt wording is the other axis, and the one the original stall
      // turned out to live on. Only offered when there is more than one, so
      // a single-variant deployment shows no pointless control.
      replayPrompts = Array.isArray(data.prompts) && data.prompts.length > 1
        ? data.prompts : [];
    } catch (e) {
      replayModels = [];
      replayPrompts = [];
    }
  }

  // A replay outlives its HTTP response: API Gateway gives up at 30 s, the
  // walks Lambda runs up to 900 s and writes its sidecar when done. So a
  // lost response is not a failure; look for the result. 3.64 C1: only a
  // record NEWER than the one stored before the request counts -- matching
  // on model and prompt alone took an earlier replay for this one. Stamps
  // are the server's own (`replayed_at`), never this browser's clock.
  const REPLAY_POLL_MS = window.ADMIN_REPLAY_POLL_MS || 10000;
  const REPLAY_POLL_FOR_MS = 900000;

  function sameReplay(r, modelId, promptVariant) {
    return r.model_id === modelId &&
      (r.prompt_variant || "default") === (promptVariant || "default");
  }

  // "stamp" of the stored replay for this model and prompt: when it was
  // last written, by a scored result or an unusable attempt kept beside it.
  function replayStamp(r) {
    if (!r) return "none";
    return String(r.replayed_at) + "|" +
      String(r.last_unusable ? r.last_unusable.replayed_at : "");
  }

  async function storedReplay(w, modelId, promptVariant) {
    const data = await api("GET", "/recording/walks/" + encodeURIComponent(w.walk) + "/replays");
    return (data.replays || []).find(function (r) { return sameReplay(r, modelId, promptVariant); });
  }

  async function pollForReplay(w, modelId, promptVariant, before) {
    const tries = Math.ceil(REPLAY_POLL_FOR_MS / REPLAY_POLL_MS);
    for (let i = 0; i < tries; i++) {
      await new Promise(function (r) { setTimeout(r, REPLAY_POLL_MS); });
      try {
        const hit = await storedReplay(w, modelId, promptVariant);
        if (hit && replayStamp(hit) !== before) {
          const cur = live(w);
          cur.replays = (cur.replays || []).filter(function (x) {
            return !sameReplay(x, modelId, promptVariant);
          });
          cur.replays.push(hit);
          return hit;
        }
      } catch (err) {
        if (err.status === 404) return null;   // the walk is gone: stop asking
      }
    }
    return null;
  }

  function applyFilterSort() {
    const q = searchEl.value.trim().toLowerCase();
    let list = allWalks.filter(function (w) { return !q || w.walk.toLowerCase().includes(q); });
    const [key, dir] = sortEl.value.split("-");
    list = list.slice().sort(function (a, b) {
      let cmp;
      if (key === "frames") {
        cmp = a.frames - b.frames;
      } else if ((a.recorded_at == null) !== (b.recorded_at == null)) {
        // One order for a mixed list (3.76 N2): a walk with no time falls
        // to the bottom whichever way the list runs.
        return a.recorded_at == null ? 1 : -1;
      } else if (a.recorded_at != null && b.recorded_at != null) {
        // By recording time, NOT by name. The name carries the model now
        // ("red-backpack-opus-4-5-<timestamp>"), so sorting on it groups by
        // model and buries a walk between two runs on a different one --
        // which reads in the console as the walk having gone missing.
        cmp = a.recorded_at - b.recorded_at;
      } else {
        cmp = a.walk < b.walk ? -1 : a.walk > b.walk ? 1 : 0;
      }
      return dir === "desc" ? -cmp : cmp;
    });
    renderWalksList(list);
  }

  function renderWalksList(list) {
    if (!allWalks.length) {
      walksEl.innerHTML = '<div class="empty">No recorded walks yet.</div>';
      updateBulkBar();
      return;
    }
    if (!list.length) {
      walksEl.innerHTML = '<div class="empty">No walks match "' + escapeHtml(searchEl.value) + '".</div>';
      return;
    }
    walksEl.innerHTML = "";
    walkRenderers = {};
    list.forEach(function (w) { walksEl.appendChild(renderWalk(w)); });
  }

  // Everything that has to happen once a walk list arrives, in ONE place.
  // Connect and Refresh each used to carry their own copy, and only
  // Refresh's fetched the replay models -- so every dropdown was empty on
  // the path a page load actually takes. Two copies of a sequence is how
  // that survives being "fixed".
  function onWalksLoaded(data) {
    allWalks = data.walks;
    detailCache.clear();                  // a walk still recording has grown (3.76 N2)
    applyFilterSort();
    loadStats();
    loadSummary();
    // Re-render once the model list lands. renderWalk() builds each
    // "Replay with..." dropdown from replayModels, so rendering before the
    // fetch resolves leaves every dropdown empty. Rendering first and
    // repainting keeps the list appearing immediately rather than waiting
    // on a second request.
    loadReplayModels().then(function () {
      if (replayModels.length) applyFilterSort();
    });
    scorePendingWalks();
  }

  async function loadWalks() {
    walksEl.innerHTML = '<div class="empty">Loading…</div>';
    try {
      onWalksLoaded(await api("GET", "/recording/walks"));
    } catch (e) {
      walksEl.innerHTML = '<div class="empty">Failed to load: ' + escapeHtml(e.message) + "</div>";
    }
  }

  // Score whatever hasn't been scored yet, without being asked.
  //
  // This is the half of the trigger design that makes the feature feel
  // automatic: the twin marks a walk finished (POST /recording/finish, which
  // writes meta.json but deliberately scores nothing), and opening this page
  // is what actually turns that marker into a score. GET .../evaluation is
  // the lazy route -- it returns a stored scorecard if there is one and
  // computes it if there isn't -- so this is safe to call for every pending
  // walk and costs nothing for walks already done.
  //
  // Only *finished* walks qualify. A walk with no meta.json is either still
  // being recorded (scoring it would judge a fragment) or predates the finish
  // signal, and both cases stay on the manual Evaluate button.
  //
  // Serialised rather than fired in parallel: each judge call is several
  // Bedrock requests, and a page load that kicked off eight walks at once
  // would throttle itself and land the user in retry storms.
  let scoringPending = false;
  let scoreAgain = false;

  async function scorePendingWalks() {
    // A list that arrives while the queue runs is looked at once it ends
    // (3.76 N2: a Refresh mid-queue left its new walks unscored).
    if (scoringPending) { scoreAgain = true; return; }
    const pending = allWalks.filter(function (w) {
      const f = flight(w.walk);
      return w.finished && !w.eval && !w.evalFailed && !f.scoring && !f.queued;
    });
    if (!pending.length) return;
    scoringPending = true;
    pending.forEach(function (w) { flight(w.walk).queued = true; repaint(w.walk); });
    try {
      for (const w of pending) {
        const f = flight(w.walk);
        f.queued = false;
        const cur = live(w);
        // Scored meanwhile (by hand, or a Refresh brought one), or deleted.
        if (cur.eval || !allWalks.includes(cur) || f.scoring) { repaint(w.walk); continue; }
        f.scoring = true;
        repaint(w.walk);
        try {
          const ev = await api("GET", "/recording/walks/" + encodeURIComponent(w.walk) + "/evaluation");
          // Onto the walk the list holds now, then just this row -- so scores
          // appear as they land, and a walk the user has expanded to look at
          // frames doesn't collapse under them every time another walk in
          // the queue finishes scoring.
          live(w).eval = ev;
          loadSummary();
        } catch (e) {
          // A walk that won't score (Bedrock down, malformed log) must not
          // stop the queue -- its Evaluate button still works by hand, and
          // now says so (3.76 N2: it read "Scoring…" until a reload).
          live(w).evalFailed = true;
        } finally {
          f.scoring = false;
          repaint(w.walk);
        }
      }
    } finally {
      scoringPending = false;
      pending.forEach(function (w) { flight(w.walk).queued = false; });
      if (scoreAgain) { scoreAgain = false; scorePendingWalks(); }
    }
  }

  function updateBulkBar() {
    const n = selected.size;
    bulkBarEl.classList.toggle("show", n > 0);
    bulkCountEl.textContent = n + " selected";
    if (!n) btnBulkDelete.textContent = "Delete selected";
  }

  function labelBadge(label) {
    if (!label) return "";
    return '<span class="label-badge ' + escapeHtml(label) + '">' + escapeHtml(label) + "</span>";
  }

  // The machine scorecard, deliberately rendered as its OWN badge next to
  // the operator's label rather than merged into it -- the score is
  // advisory (control/walk_eval.py's thresholds are calibrated against six
  // walks) and must never be mistaken for a human judgement.
  function scoreBadge(ev) {
    if (!ev || ev.score == null) return "";
    if (ev.stale) {
      // 3.64 E2: an older scorer's number, not comparable with today's;
      // Re-score is the person's call (it can be a paid judge run).
      // 3.76 F4: or scored before frames were deleted from the walk.
      if (ev.stale_reason === "frames deleted") {
        return '<span class="score-badge stale" title="Scored before frames were deleted -- Re-score to update">' +
          "frames deleted</span>";
      }
      return '<span class="score-badge stale" title="Scored by an older scorer -- Re-score to compare">' +
        "old scorer</span>";
    }
    const verdict = ev.verdict || "";
    const flags = (ev.flags || []).length ? " · " + ev.flags.join(", ") : "";
    return '<span class="score-badge ' + escapeHtml(verdict) + '" title="' +
      escapeHtml((ev.basis || "") + flags) + '">' + ev.score + flags + "</span>";
  }

  function shortModel(modelId) {
    if (!modelId) return "";
    return modelId.replace(/^(us|global)\./, "").replace(/^anthropic\.claude-/, "")
      .replace(/-\d{8}-v\d+:\d+$/, "").replace(/^amazon\./, "").replace(/-v\d+:\d+$/, "");
  }

  function renderWalk(w) {
    const el = document.createElement("div");
    el.className = "walk";

    const head = document.createElement("div");
    head.className = "walk-head";

    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = selected.has(w.walk);
    checkbox.onchange = function () {
      if (checkbox.checked) selected.add(w.walk); else selected.delete(w.walk);
      updateBulkBar();
    };
    head.appendChild(checkbox);

    const main = document.createElement("div");
    main.className = "walk-head-main";
    function renderHead() {
      main.innerHTML =
        '<div class="walk-name">' + escapeHtml(w.walk) + labelBadge(w.label) +
        scoreBadge(w.eval) + "</div>" +
        '<div class="walk-meta">' + w.frames + " frame" + (w.frames === 1 ? "" : "s") +
        (w.bytes ? " · " + fmtBytes(w.bytes) : "") +
        (w.model_id ? " · " + escapeHtml(shortModel(w.model_id)) : "") + "</div>";
    }
    renderHead();
    head.appendChild(main);

    const labelSelect = document.createElement("select");
    labelSelect.className = "label-select";
    labelSelect.innerHTML =
      '<option value="">No label</option>' +
      '<option value="good">good</option>' +
      '<option value="bad">bad</option>' +
      '<option value="training-ready">training-ready</option>';
    labelSelect.value = w.label || "";
    labelSelect.onchange = async function () {
      try {
        const label = labelSelect.value || null;
        await api("PUT", "/recording/walks/" + encodeURIComponent(w.walk) + "/tag", { label: label });
        live(w).label = label;
        repaint(w.walk);
      } catch (e) {
        labelSelect.value = live(w).label || "";   // what is stored (3.76 N2)
        showInlineError(labelSelect, "Tag failed: " + e.message);
      }
    };
    head.appendChild(labelSelect);

    const actions = document.createElement("div");
    actions.className = "walk-actions";
    const viewBtn = document.createElement("button");
    viewBtn.textContent = "View";
    const dlLink = document.createElement("a");
    dlLink.className = "dl-link";
    dlLink.textContent = "Download";
    dlLink.href = "#";
    dlLink.onclick = function (e) {
      e.preventDefault();
      downloadWalk(w.walk, dlLink);
    };
    const delBtn = document.createElement("button");
    delBtn.className = "danger";
    armConfirm(delBtn, "Delete walk", async function () {
      const f = flight(w.walk);
      if (f.deleting) return;
      f.deleting = true;
      syncControls();
      try {
        await api("DELETE", "/recording/walks/" + encodeURIComponent(w.walk));
        selected.delete(w.walk);
        allWalks = allWalks.filter(function (x) { return x.walk !== w.walk; });
        applyFilterSort();
        loadStats();
        updateBulkBar();
      } catch (e) {
        const r = walkRenderers[w.walk];
        showInlineError(r ? r.delBtn : delBtn, "Delete failed: " + e.message);
      } finally {
        f.deleting = false;
        repaint(w.walk);
      }
    });
    const evalBtn = document.createElement("button");
    // A finished walk with no score yet is already queued by
    // scorePendingWalks() -- say so, rather than inviting a click that would
    // duplicate the work already in flight.
    // Its label and state come from syncControls() below: "Scoring…" and
    // disabled while the queue or a click has a score out for this walk
    // (3.76 N2: it looked busy and was not, so a click paid a second judge
    // run), "Re-score" once there is one, else "Evaluate".
    evalBtn.onclick = async function () {
      const f = flight(w.walk);
      if (f.scoring || f.queued) return;
      f.scoring = true;                    // the judge tier is several Bedrock calls
      syncControls();
      try {
        live(w).eval = await api("POST", "/recording/walks/" + encodeURIComponent(w.walk) + "/evaluate");
        loadSummary();
      } catch (e) {
        live(w).evalFailed = true;           // not retried by the queue on its own
        const r = walkRenderers[w.walk];
        showInlineError(r ? r.evalBtn : evalBtn, "Evaluate failed: " + e.message);
      } finally {
        f.scoring = false;
        repaint(w.walk);
      }
    };

    // ---- replay: same pixels, another model ----
    // The only controlled comparison available. Two live walks by two models
    // are two different physical paths, so their score gap mixes model
    // quality with where the phone was pointed; this holds the frames fixed.
    const replaySelect = document.createElement("select");
    replaySelect.className = "replay-select";
    replaySelect.innerHTML = '<option value="">Replay with…</option>';
    // Values are "<model>|<prompt>" so one control covers both axes without
    // a second dropdown crowding the row on a phone.
    for (const m of replayModels) {
      for (const prompt of (replayPrompts.length ? replayPrompts : ["default"])) {
        const opt = document.createElement("option");
        opt.value = m.id + "|" + prompt;
        opt.textContent = shortModel(m.id) + (prompt === "default" ? "" : " · " + prompt);
        replaySelect.appendChild(opt);
      }
    }
    // 3.64 C3: an unusable attempt does not replace a scored replay; the
    // server keeps the score and notes the attempt, and this says so.
    function noteKept(r) {
      if (r && r.score != null && r.last_unusable) {
        const pct = Math.round((r.last_unusable.coverage || 0) * 100);
        const row = walkRenderers[w.walk];
        showInlineError(row ? row.replaySelect : replaySelect,
          "Only " + pct + "% of frames came back -- kept the earlier "
          + (r.stale ? "(no longer current) result" : "score"));
      }
    }
    replaySelect.onchange = async function () {
      if (!replaySelect.value) return;
      const parts = replaySelect.value.split("|");
      const modelId = parts[0];
      const promptVariant = parts[1] || "default";
      const f = flight(w.walk);
      replaySelect.selectedIndex = 0;
      if (f.replays) return;               // one out already (a rebuilt row)
      f.replays += 1;
      syncControls();
      // The row on screen when an answer lands -- a search or Refresh may
      // have rebuilt it since (3.76 N2: results and errors went to a
      // detached row).
      function row() { return walkRenderers[w.walk]; }
      function note(msg) { const r = row(); showInlineError(r ? r.replaySelect : replaySelect, msg); }
      // What is stored now, so a result that lands after a lost response
      // can be told from an older one (3.64 C1). If even this read fails,
      // no older record can be told apart, so nothing is recovered.
      let before = null;
      try {
        before = replayStamp(await storedReplay(w, modelId, promptVariant));
      } catch (err) { before = null; }
      try {
        const r = await api("POST", "/recording/walks/" + encodeURIComponent(w.walk) + "/replay",
          { model_id: modelId, prompt_variant: promptVariant });
        const cur = live(w);
        cur.replays = (cur.replays || []).filter(function (x) {
          return !(x.model_id === r.model_id && (x.prompt_variant || "default") === (r.prompt_variant || "default"));
        });
        cur.replays.push(r);
        loadSummary();
        noteKept(r);
      } catch (e) {
        // A lost response, not a failure: the server finishes the job and
        // writes its sidecar. Look for THIS request's result first.
        // Only a response that never arrived, or the gateway's own
        // 502/503/504 with no detail of ours, is worth waiting out. A
        // refusal the server answered (401, 404, 503 "no vision service")
        // is shown at once -- it was polled for 15 minutes (3.64 review).
        const lost = !e.status || (e.status >= 502 && e.status <= 504 && !e.fromServer);
        const recovered = (before === null || !lost) ? null
          : await pollForReplay(w, modelId, promptVariant, before);
        if (recovered) {
          loadSummary();
          noteKept(recovered);
        } else {
          note("Replay failed: " + e.message);
        }
      } finally {
        f.replays -= 1;
        repaint(w.walk);
      }
    };
    actions.appendChild(replaySelect);
    actions.appendChild(evalBtn);
    actions.appendChild(viewBtn);
    actions.appendChild(dlLink);
    actions.appendChild(delBtn);
    head.appendChild(actions);
    el.appendChild(head);

    // The scorecard's own row: why the number is what it is. Collapsed to
    // nothing until a walk has been scored.
    const evalEl = document.createElement("div");
    evalEl.className = "walk-eval";
    el.appendChild(evalEl);

    function renderEvalDetail() {
      const ev = w.eval;
      if (!ev || ev.score == null) { evalEl.innerHTML = ""; return; }
      const m = ev.metrics || {};
      const j = ev.judge || {};
      const bits = [];
      if (m.forward_rate != null) bits.push("forward " + Math.round(m.forward_rate * 100) + "%");
      if (m.longest_no_forward_run != null) bits.push("longest stall " + m.longest_no_forward_run);
      if (m.oscillation_rate != null) bits.push("oscillation " + Math.round(m.oscillation_rate * 100) + "%");
      if (m.target_visible_rate != null) bits.push("target seen " + Math.round(m.target_visible_rate * 100) + "%");
      if (j.sensible_rate != null) {
        bits.push("judge " + Math.round(j.sensible_rate * 100) + "% sensible (" + j.judged + " frames)");
      } else if (j.error) {
        bits.push("judge unavailable");
      }
      let html = '<div class="eval-line">' + escapeHtml(bits.join(" · ")) + "</div>";
      // The score's own arithmetic, so a number can be argued with rather
      // than just believed -- and so a low score points at which part of the
      // walk earned it.
      const c = ev.components;
      if (c) {
        const pct = function (v) { return v == null ? "--" : Math.round(v * 100) + "%"; };
        html += '<div class="eval-line dim">score = judge ' + pct(c.judge) +
          " · behaviour " + pct(c.behaviour) +
          " (progress " + pct(c.progress) + ", smoothness " + pct(c.smoothness) +
          ", identity " + pct(c.identity) + ", spread " + pct(c.non_degeneracy) + ")" +
          " · reached " + (c.completion ? "yes" : "no") + "</div>";
      }
      // One concrete example of a move the judge disagreed with is more
      // use than the aggregate on its own.
      const bad = (j.frames || []).filter(function (f) { return f.sensible === false && f.why; });
      if (bad.length) {
        html += '<div class="eval-line dim">' + escapeHtml(bad[0].file + ": chose " +
          bad[0].action + " — " + bad[0].why) + "</div>";
      }
      evalEl.innerHTML = html;
    }
    renderEvalDetail();

    const replayEl = document.createElement("div");
    replayEl.className = "walk-eval";
    el.appendChild(replayEl);

    function renderReplays() {
      const rs = w.replays || [];
      if (!rs.length) { replayEl.innerHTML = ""; return; }
      // Recorded score first, then each replay, so the comparison reads as a
      // column rather than something to hold in your head.
      // Unscored replays last, whatever their frame count: they are not a
      // worse result, they are an absent one.
      const rows = rs.slice().sort(function (a, b) {
        // Comparable scores first, best first; then an older scorer's (not
        // comparable, so in storage order); then the unscored.
        function rank(r) { return r.score == null ? 2 : (r.stale ? 1 : 0); }
        if (rank(a) !== rank(b)) return rank(a) - rank(b);
        return rank(a) === 0 ? b.score - a.score : 0;
      })
        .map(function (r) {
          const agree = r.agreement == null ? "--" : Math.round(r.agreement * 100) + "%";
          const variant = (r.prompt_variant && r.prompt_variant !== "default")
            ? " · " + escapeHtml(r.prompt_variant) : "";
          const head = '<div class="eval-line">replay · <b>' +
            escapeHtml(shortModel(r.model_id)) + variant + "</b> ";
          // A replay that lost too many frames says nothing about the model,
          // so it gets no number to compare -- reporting one is exactly how
          // three timed-out prompt variants came to look equivalent.
          if (r.stale && r.score != null) {
            // 3.64 E2: scored by an older scorer, so not comparable.
            // 3.76 F4: or before frames were deleted from the walk.
            return head + (r.stale_reason === "frames deleted"
              ? "scored before frames were deleted" : "scored by an older scorer") +
              " · replay it again</div>";
          }
          if (r.score == null) {
            const got = r.coverage == null ? "" :
              " -- only " + Math.round(r.coverage * 100) + "% of frames came back";
            return head + "not scored (" + escapeHtml(r.verdict || "unusable") + ")" +
              escapeHtml(got) +
              (r.errors ? " · " + r.errors + " frame(s) failed" : "") +
              " · replay it again</div>";
          }
          return head + "score " + r.score + " (" + escapeHtml(r.verdict) + ") · agreed with the " +
            "recording on " + agree + " of frames" +
            (r.errors ? " · " + r.errors + " frame(s) failed" : "") +
            (r.flags && r.flags.length ? " · " + escapeHtml(r.flags.join(", ")) : "") + "</div>";
        });
      replayEl.innerHTML = rows.join("");
    }
    renderReplays();

    // Every control's state from what is out for this walk, so a rebuilt
    // row starts as busy as the one it replaced.
    function syncControls() {
      const f = flight(w.walk);
      const busyScoring = f.scoring || f.queued;
      evalBtn.disabled = busyScoring;
      evalBtn.textContent = busyScoring ? "Scoring…" : (w.eval ? "Re-score" : "Evaluate");
      replaySelect.disabled = f.replays > 0;
      replaySelect.options[0].textContent = f.replays > 0 ? "Replaying…" : "Replay with…";
      delBtn.disabled = f.deleting;
    }
    syncControls();

    walkRenderers[w.walk] = {
      renderHead: renderHead,
      renderEvalDetail: renderEvalDetail,
      renderReplays: renderReplays,
      repaint: function () { renderHead(); renderEvalDetail(); renderReplays(); syncControls(); },
      evalBtn: evalBtn, replaySelect: replaySelect, delBtn: delBtn,
    };

    const framesEl = document.createElement("div");
    framesEl.className = "frames";
    framesEl.style.display = "none";
    el.appendChild(framesEl);

    async function ensureDetail() {
      let detail = detailCache.get(w.walk);
      if (detail) return detail;
      const raw = await api("GET", "/recording/walks/" + encodeURIComponent(w.walk));
      const byFile = {};
      raw.entries.forEach(function (e) { byFile[e.file] = e; });
      detail = { frames: raw.frames, byFile: byFile };
      detailCache.set(w.walk, detail);
      return detail;
    }

    async function toggle() {
      const showing = framesEl.style.display !== "none";
      if (showing) { framesEl.style.display = "none"; return; }
      framesEl.style.display = "grid";
      framesEl.innerHTML = '<div class="empty">Loading frames…</div>';
      try {
        const detail = await ensureDetail();
        framesEl.innerHTML = "";
        detail.frames.forEach(function (f) {
          framesEl.appendChild(renderFrame(w, detail, f, el));
        });
      } catch (e) {
        framesEl.innerHTML = '<div class="empty">Failed to load frames: ' + escapeHtml(e.message) + "</div>";
      }
    }
    main.onclick = toggle;
    viewBtn.onclick = toggle;

    return el;
  }

  function renderFrame(w, detail, f, walkEl) {
    const entry = detail.byFile[f.file];
    const card = document.createElement("div");
    card.className = "frame";

    const img = document.createElement("img");
    img.alt = f.file;
    frameBlobUrl(w.walk, f.file).then(function (url) {
      img.src = url;
      img._blobUrl = url;
    }).catch(function () {});
    // Its place looked up at the tap, not kept from when it was drawn (3.76
    // N2: after a frame delete every later card opened the next frame, and
    // the last one threw).
    img.onclick = function () {
      const index = detail.frames.findIndex(function (x) { return x.file === f.file; });
      if (index >= 0) openSlideshow(w, detail, index);
    };
    card.appendChild(img);

    const body = document.createElement("div");
    body.className = "frame-body";
    let html = '<div class="frame-file">' + escapeHtml(f.file) + " &middot; " + (f.bytes / 1024).toFixed(0) + " KB</div>";
    if (entry && entry.navigate) {
      html += '<div class="frame-action">' + escapeHtml(entry.navigate.action || "?") + "</div>";
      if (entry.navigate.model_id) {
        const usage = entry.navigate.usage || {};
        // Escaped like every other stored field (3.76 N2: a frame posted under
        // the brain's secret ran script here, in the page holding this one's).
        const tokens = (usage.input_tokens != null && usage.output_tokens != null)
          ? " &middot; " + escapeHtml(usage.input_tokens) + " in / " + escapeHtml(usage.output_tokens) + " out"
          : "";
        html += '<div class="frame-model">' + escapeHtml(entry.navigate.model_id) + tokens + "</div>";
      }
      html += '<div class="frame-reasoning">' + escapeHtml(entry.navigate.reasoning || "") + "</div>";
    }
    body.innerHTML = html;

    const delBtn = document.createElement("button");
    delBtn.className = "danger small";
    armConfirm(delBtn, "Delete frame", async function () {
      delBtn.disabled = true;
      try {
        const gone = await api("DELETE", "/recording/walks/" + encodeURIComponent(w.walk) + "/frames/" + encodeURIComponent(f.file));
        card.remove();
        w.frames -= 1;
        detail.frames = detail.frames.filter(function (x) { return x.file !== f.file; });
        const meta = walkEl.querySelector(".walk-meta");
        if (meta) meta.textContent = w.frames + " frame" + (w.frames === 1 ? "" : "s");
        // 3.76 F4: the server marked every stored score of this walk; say so
        // now rather than on the next reload.
        // A score already stale keeps its reason, as the server's
        // _stale_reason() does ("older scorer" first). Only a listed frame
        // marks anything (an unlisted image was in no score).
        function mark(x) {
          return x.stale ? x : Object.assign({}, x, { stale: true, stale_reason: "frames deleted" });
        }
        const cur = live(w);
        if (!gone || gone.marked !== false) {
          if (cur.eval) cur.eval = mark(cur.eval);
          cur.replays = (cur.replays || []).map(mark);
        }
        const r = walkRenderers[w.walk];
        if (r) { r.renderHead(); r.renderReplays(); }
        loadSummary();
        loadStats();
      } catch (e) {
        showInlineError(delBtn, "Delete failed: " + e.message);
        delBtn.disabled = false;
      }
    });
    body.appendChild(delBtn);
    card.appendChild(body);
    return card;
  }

  function saveFrom(href, name) {
    const a = document.createElement("a");
    a.href = href;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    a.remove();
  }

  // `?link=1` (3.76 N2): on S3 the server answered a 307 to a presigned URL,
  // which a fetch() carrying the secret header follows as a cross-origin
  // CORS request -- the bucket has no CORS rules, so it failed and said
  // nothing. Now the server hands the link back and the browser just opens
  // it (the URL is its own credential); locally the zip comes inline.
  async function downloadWalk(walk, link) {
    try {
      const res = await fetch("/recording/walks/" + encodeURIComponent(walk) + "/download?link=1",
        { headers: headers() });
      if (!res.ok) {
        const body = await res.json().catch(function () { return {}; });
        throw new Error(typeof body.detail === "string" ? body.detail : "HTTP " + res.status);
      }
      if ((res.headers.get("content-type") || "").includes("application/json")) {
        const body = await res.json();
        saveFrom(body.url, walk + ".zip");
        return;
      }
      const url = URL.createObjectURL(await res.blob());
      saveFrom(url, walk + ".zip");
      setTimeout(function () { URL.revokeObjectURL(url); }, 5000);
    } catch (e) {
      showInlineError(link, "Download failed: " + e.message);
    }
  }

  // --- Slideshow -------------------------------------------------------
  const slideshowEl = document.getElementById("slideshow");
  const slideshowImg = document.getElementById("slideshow-img");
  const slideshowAction = document.getElementById("slideshow-action");
  const slideshowModel = document.getElementById("slideshow-model");
  const slideshowReasoning = document.getElementById("slideshow-reasoning");
  const slideshowPos = document.getElementById("slideshow-pos");
  const slideshowPlayBtn = document.getElementById("slideshow-play");
  let slideshowState = null; // {walk, detail, index, timer}

  let slideSeq = 0;
  function showSlideImage(url) {
    if (slideshowImg._blobUrl) URL.revokeObjectURL(slideshowImg._blobUrl);
    slideshowImg._blobUrl = url;
    slideshowImg.src = url || "";
  }

  function renderSlide() {
    const s = slideshowState;
    const f = s.detail.frames[s.index];
    const entry = s.detail.byFile[f.file];
    // Only the newest step's picture (3.76 N2: a slow frame landed under the
    // next frame's caption).
    // Its own picture or none -- never the last frame's under this caption
    // -- and Play waits for it (round 1: a frame slower than the 1.2 s tick
    // was always stale by the time it landed, so Play showed nothing).
    const seq = ++slideSeq;
    showSlideImage(null);
    s.loading = true;
    frameBlobUrl(s.walk.walk, f.file).then(function (url) {
      if (seq === slideSeq && slideshowState === s) { showSlideImage(url); s.loading = false; }
      else URL.revokeObjectURL(url);
    }).catch(function () { if (seq === slideSeq) s.loading = false; });
    slideshowAction.textContent = entry && entry.navigate ? (entry.navigate.action || "?") : "";
    if (entry && entry.navigate && entry.navigate.model_id) {
      const usage = entry.navigate.usage || {};
      const tokens = (usage.input_tokens != null && usage.output_tokens != null)
        ? " · " + usage.input_tokens + " in / " + usage.output_tokens + " out" : "";
      slideshowModel.textContent = entry.navigate.model_id + tokens;
    } else {
      slideshowModel.textContent = "";
    }
    slideshowReasoning.textContent = entry && entry.navigate ? (entry.navigate.reasoning || "") : "";
    slideshowPos.textContent = (s.index + 1) + " / " + s.detail.frames.length + " — " + f.file;
  }

  function openSlideshow(w, detail, index) {
    slideshowState = { walk: w, detail: detail, index: index, timer: null };
    slideshowEl.classList.add("show");
    renderSlide();
  }

  function closeSlideshow() {
    if (slideshowState && slideshowState.timer) clearInterval(slideshowState.timer);
    slideshowState = null;
    slideSeq += 1;
    showSlideImage(null);
    slideshowEl.classList.remove("show");
    slideshowPlayBtn.textContent = "Play";
  }

  function slideshowStep(delta) {
    const s = slideshowState;
    if (!s) return;
    s.index = (s.index + delta + s.detail.frames.length) % s.detail.frames.length;
    renderSlide();
  }

  document.getElementById("slideshow-close").onclick = closeSlideshow;
  // Swipe to step through frames. On a phone this is how you actually read a
  // walk -- reaching for Prev/Next between every frame of a 39-frame walk is
  // not a review, it is data entry. Horizontal-only, and ignored unless the
  // gesture is clearly sideways, so it never fights the overlay's own
  // vertical scrolling.
  (function () {
    const el = document.getElementById("slideshow");
    let x0 = null, y0 = null;
    el.addEventListener("touchstart", function (e) {
      if (e.touches.length !== 1) { x0 = null; return; }
      x0 = e.touches[0].clientX;
      y0 = e.touches[0].clientY;
    }, { passive: true });
    el.addEventListener("touchend", function (e) {
      if (x0 === null || !e.changedTouches.length) return;
      const dx = e.changedTouches[0].clientX - x0;
      const dy = e.changedTouches[0].clientY - y0;
      x0 = null;
      if (Math.abs(dx) < 45 || Math.abs(dx) < Math.abs(dy) * 1.5) return;
      slideshowStep(dx < 0 ? 1 : -1);
    }, { passive: true });
  })();

  document.getElementById("slideshow-prev").onclick = function () { slideshowStep(-1); };
  document.getElementById("slideshow-next").onclick = function () { slideshowStep(1); };
  document.getElementById("slideshow-play").onclick = function () {
    const s = slideshowState;
    if (!s) return;
    if (s.timer) {
      clearInterval(s.timer);
      s.timer = null;
      slideshowPlayBtn.textContent = "Play";
    } else {
      s.timer = setInterval(function () { if (!s.loading) slideshowStep(1); }, 1200);
      slideshowPlayBtn.textContent = "Pause";
    }
  };
  slideshowEl.addEventListener("click", function (e) {
    if (e.target === slideshowEl) closeSlideshow();
  });
  document.addEventListener("keydown", function (e) {
    if (!slideshowState) return;
    if (e.key === "Escape") closeSlideshow();
    else if (e.key === "ArrowLeft") slideshowStep(-1);
    else if (e.key === "ArrowRight") slideshowStep(1);
  });

  // --- Bulk delete -------------------------------------------------------
  armConfirm(btnBulkDelete, "Delete selected", async function () {
    btnBulkDelete.disabled = true;
    // Not a walk whose own Delete is out (3.76 N2: a second DELETE, and a
    // gone walk left "selected").
    const names = Array.from(selected).filter(function (name) { return !flight(name).deleting; });
    names.forEach(function (name) { flight(name).deleting = true; repaint(name); });
    const results = await Promise.allSettled(
      names.map(function (name) { return api("DELETE", "/recording/walks/" + encodeURIComponent(name)); })
    );
    names.forEach(function (name) { flight(name).deleting = false; });
    // Gone already is gone (a 404), not a failure.
    const failed = names.filter(function (_, i) {
      return results[i].status === "rejected" && results[i].reason.status !== 404;
    });
    allWalks = allWalks.filter(function (w) { return !names.includes(w.walk) || failed.includes(w.walk); });
    selected.clear();
    failed.forEach(function (name) { selected.add(name); });
    applyFilterSort();
    loadStats();
    updateBulkBar();
    btnBulkDelete.disabled = false;
    if (failed.length) showInlineError(btnBulkDelete, failed.length + " failed to delete.");
  });

  // --- Wiring -------------------------------------------------------
  searchEl.oninput = applyFilterSort;
  sortEl.onchange = applyFilterSort;

  btnConnect.onclick = function () {
    const tried = secretEl.value.trim();
    setStatus(connStatusEl, "Connecting…");
    // /recording/walks, not /health -- /health is deliberately unauthenticated
    // (same reasoning as robot/server.py's), so it can't tell a right secret
    // from a wrong one. This is also the first real data load, not just a check.
    // Tried on this request alone, and kept only once it worked (3.76 N2),
    // as metrics.js does: a mistyped secret replaced the saved one, and the
    // scoring queue's requests went out under it meanwhile.
    api("GET", "/recording/walks", undefined, tried).then(function (data) {
      secret = tried;
      try { localStorage.setItem(SECRET_KEY, tried); } catch (e) { /* private mode */ }
      setStatus(connStatusEl, "Connected.", "ok");
      btnRefresh.disabled = false;
      searchEl.disabled = false;
      sortEl.disabled = false;
      onWalksLoaded(data);
    }).catch(function (e) {
      setStatus(connStatusEl, "Failed: " + e.message, "err");
    });
  };
  btnRefresh.onclick = loadWalks;

  // Which deployment is this? Asked of the server that served the page,
  // for the same reason the twin does it: a hostname is not something
  // anyone reads off an address bar before typing a secret, and this
  // console is byte-identical between environments. Unauthenticated and
  // fails silent -- no banner is both production's healthy state and the
  // safe outcome if the fetch throws.
  fetch("health", { cache: "no-store" })
    .then(function (r) { return r.ok ? r.json() : null; })
    .then(function (body) {
      if (!body || !body.env_label) return;
      const el = document.getElementById("env-banner");
      if (!el) return;
      el.textContent = body.env_label + " environment";
      el.classList.add("visible");
      document.title = body.env_label.toUpperCase() + " \u00b7 " + document.title;
    })
    .catch(function () { /* no banner -- see above */ });

  if (secret) btnConnect.onclick();
})();
