/* control/metrics.js -- the tier-metrics dashboard.
 *
 * Reads GET /metrics/summary and draws it. Deliberately dependency-free
 * and inline-SVG: the page is served from the same CloudFront origin as
 * the twin, and a chart library would be the only third-party script in
 * the whole deployment for a job that is two polylines.
 *
 * THE RULE THIS PAGE ENFORCES, and it is the reason several obvious
 * simplifications are absent: a percentile belongs to the run that
 * produced it. Averaging p90s across runs does not give the p90 of the
 * pair, so nothing here does it. Cross-run figures are a MEDIAN OF
 * PER-RUN VALUES and are labelled as such, or they are a range.
 */
(function () {
  "use strict";

  var SECRET_KEY = "vp_metrics_secret";
  // The admin console is the SAME service behind the SAME secret
  // (WALKS_SECRET guards /recording/*, /stats and /metrics/*), served from
  // the same CloudFront origin -- so its stored value is this page's
  // value, and localStorage is shared between them. Falling back to it
  // means anyone already using /admin never types a 43-character secret
  // on a phone. It is a convenience, not a second source of truth: a
  // secret typed here still wins and is still only kept once it works.
  var ADMIN_SECRET_KEY = "vp_admin_secret";
  var $ = function (id) { return document.getElementById(id); };

  function fmtMs(v) { return v == null ? "–" : Math.round(v) + "ms"; }
  function pct(v) { return v == null ? "–" : Math.round(v * 100) + "%"; }

  /* Median of a list of per-run values. Used everywhere a headline number
   * spans runs -- see the module rule above. */
  function median(xs) {
    var v = xs.filter(function (x) { return typeof x === "number"; }).sort(function (a, b) { return a - b; });
    if (!v.length) return null;
    var m = Math.floor(v.length / 2);
    return v.length % 2 ? v[m] : (v[m - 1] + v[m]) / 2;
  }

  function card(label, dotClass, value, unit, foot) {
    return '<div class="card"><div class="label">'
      + (dotClass ? '<i class="dot ' + dotClass + '"></i>' : "")
      + label + '</div><div class="value">' + value
      + (unit ? '<small>' + unit + '</small>' : "")
      + '</div><div class="foot">' + (foot || "") + '</div></div>';
  }

  function renderKpis(runs) {
    var cloud = runs.map(function (r) { return (r.stats || {}).cloud_ms || null; });
    var local = runs.map(function (r) { return (r.stats || {}).perception_ms || null; });
    var calls = runs.map(function (r) { return (r.stats || {}).cloud_calls || 0; });
    var frames = runs.map(function (r) { return (r.stats || {}).frames || 0; });
    var detected = runs.map(function (r) {
      var p = (r.stats || {}).perception || {};
      var tot = Object.keys(p).reduce(function (a, k) { return a + p[k]; }, 0);
      return tot ? (p.detected || 0) / tot : null;
    });
    var totalCalls = calls.reduce(function (a, b) { return a + b; }, 0);
    var totalFrames = frames.reduce(function (a, b) { return a + b; }, 0);
    var pick = function (arr, k) {
      return arr.map(function (x) { return x ? x[k] : null; });
    };
    var found = runs.filter(function (r) { return r.outcome === "found"; }).length;

    $("kpis").innerHTML = [
      card("Cloud p50", "cloud", fmtMs(median(pick(cloud, "p50"))), "",
           "median of " + runs.length + " runs"),
      card("Cloud p90", "cloud", fmtMs(median(pick(cloud, "p90"))), "",
           "median of per-run p90"),
      card("Cloud p99", "cloud", fmtMs(median(pick(cloud, "p99"))), "",
           "median of per-run p99 · read with n"),
      card("Perception p50", "local", fmtMs(median(pick(local, "p50"))), "",
           "per frame, on-board tier"),
      card("Frames per call", "", totalCalls ? (totalFrames / totalCalls).toFixed(1) : "–", "",
           totalFrames + " frames / " + totalCalls + " calls"),
      card("Target detected", "local", pct(median(detected)), "",
           "median share of frames"),
      card("Missions", "", String(runs.length), "",
           found + " reached the target")
    ].join("");
  }

  /* ---- charts: inline SVG, no library ---- */

  function lineChart(el, series, opts) {
    opts = opts || {};
    var W = 1000, H = 210, P = { l: 46, r: 12, t: 12, b: 26 };
    var all = [];
    series.forEach(function (s) {
      s.values.forEach(function (v) { if (v != null) all.push(v); });
    });
    if (!all.length) {
      el.innerHTML = '<div class="empty">No data in this window.</div>';
      return;
    }
    var max = Math.max.apply(null, all) * 1.12;
    var min = opts.zero === false ? Math.min.apply(null, all) * 0.9 : 0;
    var n = Math.max(1, series[0].values.length - 1);
    var x = function (i) { return P.l + (i / n) * (W - P.l - P.r); };
    var y = function (v) { return P.t + (1 - (v - min) / (max - min || 1)) * (H - P.t - P.b); };

    var parts = [];
    for (var g = 0; g <= 4; g++) {
      var gy = P.t + (g / 4) * (H - P.t - P.b);
      var gv = max - (g / 4) * (max - min);
      parts.push('<line class="gl" x1="' + P.l + '" y1="' + gy + '" x2="' + (W - P.r) + '" y2="' + gy + '"/>');
      parts.push('<text class="ax" x="' + (P.l - 8) + '" y="' + (gy + 3.5) + '" text-anchor="end">'
        + Math.round(gv) + (opts.unit || "") + '</text>');
    }
    series.forEach(function (s) {
      // Gaps are BREAKS, not interpolations: a run with no cloud call has
      // no latency, and drawing a line through it would invent one.
      var seg = [], segs = [];
      s.values.forEach(function (v, i) {
        if (v == null) { if (seg.length) segs.push(seg); seg = []; }
        else seg.push(x(i) + "," + y(v));
      });
      if (seg.length) segs.push(seg);
      segs.forEach(function (pts) {
        if (pts.length === 1) {
          var c = pts[0].split(",");
          parts.push('<circle cx="' + c[0] + '" cy="' + c[1] + '" r="2.6" fill="' + s.color + '"/>');
        } else {
          parts.push('<polyline fill="none" stroke="' + s.color + '" stroke-width="2"'
            + ' stroke-linejoin="round" stroke-linecap="round" points="' + pts.join(" ") + '"/>');
        }
      });
    });
    parts.push('<text class="ax" x="' + P.l + '" y="' + (H - 6) + '">oldest</text>');
    parts.push('<text class="ax" x="' + (W - P.r) + '" y="' + (H - 6) + '" text-anchor="end">newest</text>');
    el.innerHTML = '<svg viewBox="0 0 ' + W + ' ' + H + '" preserveAspectRatio="none" '
      + 'style="height:' + H + 'px">' + parts.join("") + '</svg>';
  }

  function renderCharts(runs) {
    var old = runs.slice().reverse();   // oldest -> newest, which is how a trend reads
    var get = function (k, f) { return old.map(function (r) { var s = (r.stats || {})[k]; return s ? s[f] : null; }); };
    lineChart($("chart-cloud"), [
      { color: "#60a5fa", values: get("cloud_ms", "p50") },
      { color: "#fbbf24", values: get("cloud_ms", "p90") },
      { color: "#f87171", values: get("cloud_ms", "p99") }
    ], { unit: "ms" });

    lineChart($("chart-local"), [
      { color: "#4ade80", values: get("perception_ms", "p50") },
      { color: "#fbbf24", values: old.map(function (r) {
          var p = (r.stats || {}).perception || {};
          var tot = Object.keys(p).reduce(function (a, k) { return a + p[k]; }, 0);
          return tot ? Math.round((p.detected || 0) / tot * 100) : null;
        }) }
    ], {});
  }

  function renderRuns(runs) {
    if (!runs.length) {
      $("runs").innerHTML = '<tr><td class="empty">No runs stored yet.</td></tr>';
      return;
    }
    var head = "<tr><th>When</th><th>Outcome</th><th>Steps</th><th>Target</th>"
      + "<th>Cloud p50/p90/p99</th><th>n</th><th>Perc p50</th><th>Detected</th>"
      + "<th>Calls</th><th>Build</th><th>Config</th></tr>";
    var rows = runs.map(function (r) {
      var s = r.stats || {}, c = s.cloud_ms, l = s.perception_ms, p = s.perception || {};
      var tot = Object.keys(p).reduce(function (a, k) { return a + p[k]; }, 0);
      var when = r.finished_at
        ? new Date(r.finished_at * 1000).toLocaleString(undefined,
            { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" })
        : (r.day || "–");
      var cls = r.outcome === "found" ? "found"
        : (r.outcome === "failed" ? "bad" : "");
      var cfg = Object.keys(r.config || {}).filter(function (k) {
        return r.config[k] !== null && r.config[k] !== "";
      }).map(function (k) {
        return k.replace(/^tier_|^perception_|^navigate_/, "") + "=" + r.config[k];
      }).join(" · ");
      return "<tr><td>" + when + "</td>"
        + '<td><span class="pill ' + cls + '">' + (r.outcome || "–") + "</span></td>"
        + "<td>" + (r.steps == null ? "–" : r.steps) + "</td>"
        + "<td>" + (r.target_object || "–") + "</td>"
        + "<td>" + (c ? c.p50 + " / " + c.p90 + " / " + c.p99 : "–") + "</td>"
        // n beside every percentile, always: a percentile over a handful
        // of samples is one sample with a fancy name.
        + '<td class="mono">' + (c ? c.n : "0") + "</td>"
        + "<td>" + (l ? l.p50 + "ms" : "–") + "</td>"
        + "<td>" + (tot ? Math.round((p.detected || 0) / tot * 100) + "%" : "–") + "</td>"
        + "<td>" + (s.cloud_calls == null ? "–" : s.cloud_calls) + "</td>"
        + '<td class="mono">' + (r.git_revision || "–").slice(0, 7) + "</td>"
        + '<td class="mono">' + (cfg || "–") + "</td></tr>";
    });
    $("runs").innerHTML = head + rows.join("");
  }

  /* Tolerate what a phone paste actually contains.
   *
   * The documented way to find this value is `grep WALKS_SECRET
   * ~/.vision-picar-serverless-secrets`, whose output is the whole LINE.
   * Pasting that sends "WALKS_SECRET=abc..." as the header and 401s, with
   * nothing on screen to suggest why -- observed on an iPhone, 2026-09-13.
   * Shell quoting and a trailing newline arrive the same way.
   *
   * Stripping them is safe: none of `KEY=`, quotes or whitespace can be
   * part of a real secret here, because the value has to survive being an
   * HTTP header and a shell variable.
   */
  function cleanSecret(raw) {
    var v = (raw || "").trim().replace(/[\r\n]+$/, "");
    v = v.replace(/^[A-Z_][A-Z0-9_]*\s*=\s*/, "");   // WALKS_SECRET=...
    v = v.replace(/^["']|["']$/g, "");                // "..." or '...'
    return v.trim();
  }

  function load() {
    var secret = cleanSecret($("secret").value);
    if (secret !== $("secret").value) $("secret").value = secret;
    $("error").hidden = true;
    fetch("metrics/summary?days=" + $("days").value,
          { headers: secret ? { "x-app-secret": secret } : {} })
      .then(function (r) {
        if (!r.ok) {
          // 401 and 403 both mean the secret, and saying so beats a bare
          // status code on a phone where the fix is "check what you
          // pasted".
          throw new Error("HTTP " + r.status
            + (r.status === 401 || r.status === 403
               ? " -- the secret was rejected. Paste only the VALUE of "
                 + "WALKS_SECRET, not the whole WALKS_SECRET=... line."
               : ""));
        }
        return r.json();
      })
      .then(function (d) {
        // Remembered only once it has WORKED. Saving on every attempt
        // means a wrong secret is what comes back next time, which is how
        // a one-off paste error becomes a permanent one.
        try { localStorage.setItem(SECRET_KEY, secret); } catch (e) {}
        var runs = d.runs || [];
        renderKpis(runs);
        renderCharts(runs);
        renderRuns(runs);
        $("footnote").textContent = runs.length + " run(s) over " + d.days
          + " days. Percentiles are per run and are never blended: a cross-run"
          + " figure here is the median of per-run values, and n is shown so a"
          + " percentile over a handful of samples reads as what it is.";
      })
      .catch(function (e) {
        $("error").hidden = false;
        $("error").textContent = "Could not load metrics: " + e.message;
      });
  }

  try {
    $("secret").value = localStorage.getItem(SECRET_KEY)
      || localStorage.getItem(ADMIN_SECRET_KEY) || "";
  } catch (e) {}
  $("reveal").onclick = function () {
    var f = $("secret");
    var hidden = f.type === "password";
    f.type = hidden ? "text" : "password";
    this.textContent = hidden ? "hide" : "show";
  };
  $("reload").onclick = load;
  $("days").onchange = load;
  $("secret").addEventListener("keydown", function (e) { if (e.key === "Enter") load(); });
  load();
})();
