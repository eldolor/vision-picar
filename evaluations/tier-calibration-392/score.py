"""
evaluations/tier-calibration-392/score.py

3.92.B: criteria 1-5 from `qwen-VE.jsonl` and `perception.jsonl`. Every fit
is leave one object out (a fit on five objects scores the sixth). numpy
only. Writes `scores.json` and prints a summary.

    .venv/bin/python evaluations/tier-calibration-392/score.py
"""

import json
import os
import statistics
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from brain.local_vlm import model_size     # noqa: E402

HERE = Path(os.environ.get("PICAR_392_DATA", Path(__file__).parent))
YES_AT, NO_BELOW, GATE = 0.5, 0.1, 0.8      # LOCAL_YES_AT, tier_local_no_below, the match gate
FLAG_RATE = 0.10                             # criterion 4b
EPS = 1e-4

# "burgundy backpack" and "red backpack" are one bag (CLAUDE.md's rig walks).
OBJECT = {"burgundy backpack": "red backpack"}


def obj(target: str) -> str:
    t = target.strip().lower()
    return OBJECT.get(t, t)


def load(name):
    return {r["key"]: r for r in (json.loads(l) for l in (HERE / name).read_text().splitlines() if l.strip())}


def logit(p):
    p = np.clip(np.asarray(p, float), EPS, 1 - EPS)
    return np.log(p / (1 - p))


def fit_logistic(X, y, l2=1e-2, iters=50):
    """Newton's method on the L2-penalised log loss; X gets a bias column."""
    X = np.column_stack([np.ones(len(X)), X])
    w = np.zeros(X.shape[1])
    for _ in range(iters):
        p = 1 / (1 + np.exp(-X @ w))
        g = X.T @ (p - y) + l2 * np.r_[0, w[1:]]
        H = (X * (p * (1 - p))[:, None]).T @ X + l2 * np.diag(np.r_[0, np.ones(len(w) - 1)])
        step = np.linalg.solve(H, g)
        w -= step
        if np.abs(step).max() < 1e-8:
            break
    return w


def predict_logistic(w, X):
    X = np.column_stack([np.ones(len(X)), X])
    return 1 / (1 + np.exp(-X @ w))


def isotonic_fit(x, y):
    """Pool adjacent violators: a step function, increasing in x."""
    order = np.argsort(x, kind="stable")
    xs, ys = np.asarray(x)[order], np.asarray(y, float)[order]
    blocks = [[ys[i], 1.0, xs[i]] for i in range(len(ys))]   # mean, weight, right edge
    out = []
    for b in blocks:
        out.append(b)
        while len(out) > 1 and out[-2][0] > out[-1][0]:
            m2, w2, e2 = out.pop()
            m1, w1, _ = out.pop()
            out.append([(m1 * w1 + m2 * w2) / (w1 + w2), w1 + w2, e2])
    return np.array([e for _, _, e in out]), np.array([m for m, _, _ in out])


def isotonic_predict(model, x):
    edges, means = model
    i = np.searchsorted(edges, np.asarray(x), side="left")
    return means[np.clip(i, 0, len(means) - 1)]


def ece(p, y, bins=10):
    """Expected calibration error over equal-count bins."""
    order = np.argsort(p, kind="stable")
    total = 0.0
    for chunk in np.array_split(order, bins):
        if len(chunk):
            total += len(chunk) * abs(p[chunk].mean() - y[chunk].mean())
    return total / len(p)


def brier(p, y):
    return float(np.mean((p - y) ** 2))


def auroc(score, y):
    """The chance a random positive outranks a random negative (ties half)."""
    score, y = np.asarray(score, float), np.asarray(y, bool)
    pos, neg = score[y], score[~y]
    if not len(pos) or not len(neg):
        return None
    ranks = np.argsort(np.argsort(np.r_[pos, neg], kind="stable"), kind="stable").astype(float)
    # average ranks over ties
    allv = np.r_[pos, neg]
    for v in np.unique(allv):
        m = allv == v
        ranks[m] = ranks[m].mean()
    r_pos = ranks[: len(pos)].sum()
    return float((r_pos - len(pos) * (len(pos) - 1) / 2) / (len(pos) * len(neg)))


def box_share(box, w, h):
    if not box or not w or not h:
        return 0.0
    x1, y1, x2, y2 = box
    return max(0.0, x2 - x1) * max(0.0, y2 - y1) / (w * h)


def vlm_box_share(box, w, h):
    """The service's box is in the model's resized frame (`model_size`, as
    `brain.local_vlm._box_frac` reads it)."""
    if not box:
        return 0.0
    mw, mh = model_size(w, h)
    return min(1.0, box_share([float(v) for v in box[:4]], mw, mh))


def held_out(objects, fit, predict):
    """Leave one object out: returns one prediction per row."""
    out = np.full(len(objects), np.nan)
    for o in sorted(set(objects)):
        test = objects == o
        model = fit(~test)
        out[test] = predict(model, test)
    return out


def main():
    q, pc = load("qwen-VE.jsonl"), load("perception.jsonl")
    keys = sorted(q)
    missing = [k for k in keys if k not in pc] + [k for k in pc if k not in q]
    rows = [(q[k], pc[k]) for k in keys if k in pc]
    s = {}

    # ---- criterion 1
    answered = [a for a, _ in rows if a["status"] == "answered" and isinstance(a["score"], (int, float))]
    rtts = [a["rtt_ms"] for a, _ in rows if a["status"] == "answered"]
    kinds = {"yes": 0, "no": 0, "unable": 0}
    for a, _ in rows:
        sc = a["score"]
        if a["status"] == "answered" and isinstance(sc, (int, float)):
            kinds["yes" if sc >= YES_AT else "no" if sc <= NO_BELOW else "unable"] += 1
        else:
            kinds["unable"] += 1
    s["c1"] = {"frames": len(rows), "missing": len(missing), "answered": len(answered),
               "not_answered": Counter_of([a["status"] for a, _ in rows if a["status"] != "answered"]),
               "rtt_p50_ms": statistics.median(rtts) if rtts else None,
               "rtt_p90_ms": float(np.percentile(rtts, 90)) if rtts else None,
               "decisions": kinds, "met": len(missing) == 0 and len(rows) == 2261}

    y = np.array([bool(a["visible"]) for a, _ in rows], float)
    objects = np.array([obj(a["target"]) for a, _ in rows])
    ve = np.array([a["score"] if a["status"] == "answered" and isinstance(a["score"], (int, float))
                   else np.nan for a, _ in rows])
    clip = np.array([p["p"] if p["p"] is not None else 0.0 for _, p in rows])
    detected = np.array([p["status"] == "detected" for _, p in rows])

    # ---- criterion 2: Platt, held out (VE on answered rows only)
    def calibrate(raw, mask, name):
        r, yy, oo = raw[mask], y[mask], objects[mask]
        X = logit(r)[:, None]
        platt = held_out(oo, lambda tr: fit_logistic(X[tr], yy[tr]),
                         lambda w, te: predict_logistic(w, X[te]))
        iso = held_out(oo, lambda tr: isotonic_fit(r[tr], yy[tr]),
                       lambda m, te: isotonic_predict(m, r[te]))
        per = {}
        for o in sorted(set(oo)):
            m = oo == o
            per[o] = {"n": int(m.sum()), "in_view": int(yy[m].sum()),
                      "ece_raw": round(ece(r[m], yy[m]), 4), "ece_platt": round(ece(platt[m], yy[m]), 4)}
        out = {"rows": int(mask.sum()),
               "ece_raw": round(ece(r, yy), 4), "ece_platt": round(ece(platt, yy), 4),
               "ece_isotonic": round(ece(iso, yy), 4),
               "brier_raw": round(brier(r, yy), 4), "brier_platt": round(brier(platt, yy), 4),
               "brier_isotonic": round(brier(iso, yy), 4), "per_object": per}
        out["met"] = out["ece_platt"] <= 0.05 and out["ece_platt"] < out["ece_raw"]
        full = np.full(len(raw), np.nan); full[mask] = platt
        return out, full

    s["c2a_ve"], ve_cal = calibrate(ve, ~np.isnan(ve), "ve")
    s["c2b_clip"], clip_cal = calibrate(clip, np.ones(len(clip), bool), "clip")

    # ---- criterion 3: today's thresholds, per object
    per = {}
    for o in sorted(set(objects)):
        m = objects == o
        ans = m & ~np.isnan(ve)
        yes = ans & (ve >= YES_AT)
        no = ans & (ve <= NO_BELOW)
        band = m & ~(yes | no)       # the band, or not answered: both go on to the cloud
        inview = m & (y == 1)
        per[o] = {"frames": int(m.sum()), "in_view": int(inview.sum()),
                  "yes": int(yes.sum()), "yes_right": int((yes & (y == 1)).sum()),
                  "yes_precision": rnd((yes & (y == 1)).sum() / yes.sum()) if yes.sum() else None,
                  "yes_recall": rnd((yes & (y == 1)).sum() / inview.sum()) if inview.sum() else None,
                  "missed_by_no": rnd((no & (y == 1)).sum() / inview.sum()) if inview.sum() else None,
                  "unable_share": rnd(band.sum() / m.sum())}
    ans = ~np.isnan(ve)
    yes, no = ans & (ve >= YES_AT), ans & (ve <= NO_BELOW)
    s["c3"] = {"per_object": per,
               "all": {"yes": int(yes.sum()),
                       "yes_precision": rnd((yes & (y == 1)).sum() / yes.sum()) if yes.sum() else None,
                       "yes_recall": rnd((yes & (y == 1)).sum() / y.sum()),
                       "missed_by_no": rnd((no & (y == 1)).sum() / y.sum()),
                       "unable_share": rnd((~(yes | no)).sum() / len(y))}}

    # ---- criterion 4: the router
    decided = yes | no
    wrong = (yes & (y == 0)) | (no & (y == 1))
    vb = np.array([vlm_box_share(a.get("box"), p["w"], p["h"]) for a, p in rows])
    pb = np.array([box_share(p["box"], p["w"], p["h"]) for _, p in rows])
    disagree = (detected & no) | (~detected & yes)
    X = np.column_stack([logit(np.nan_to_num(ve, nan=0.5)), np.sqrt(vb), (vb > 0).astype(float),
                         logit(clip), detected.astype(float), np.sqrt(pb), disagree.astype(float),
                         yes.astype(float)])
    d = decided
    Xd, wd, od = X[d], wrong[d].astype(float), objects[d]
    mu_sd = lambda tr: (Xd[tr].mean(0), Xd[tr].std(0) + 1e-9)

    def fit(tr):
        mu, sd = mu_sd(tr)
        return fit_logistic((Xd[tr] - mu) / sd, wd[tr], l2=1.0), mu, sd

    risk = held_out(od, fit, lambda m, te: predict_logistic(m[0], (Xd[te] - m[1]) / m[2]))
    n_wrong = int(wd.sum())
    c4 = {"decided": int(d.sum()), "wrong": n_wrong,
          "wrong_false_yes": int((yes & (y == 0)).sum()), "wrong_false_no": int((no & (y == 1)).sum())}
    if n_wrong < 20:
        c4["measurable"] = False
        c4["met"] = None
    else:
        k = int(np.floor(FLAG_RATE * d.sum()))
        top = np.argsort(-risk, kind="stable")[:k]
        caught = int(wd[top].sum())
        dis = disagree[d]
        # the hand rule at the same flag rate: disagreements first; if there
        # are more than k, the k flagged are taken in frame order (no ranking)
        dis_idx = np.flatnonzero(dis)[:k]
        caught_rule = int(wd[dis_idx].sum())
        c4.update({"measurable": True, "auroc": rnd(auroc(risk, wd)),
                   "flag_k": k, "caught": caught, "caught_share": rnd(caught / n_wrong),
                   "rule_flags": int(dis.sum()), "rule_flags_used": len(dis_idx),
                   "rule_caught": caught_rule, "rule_caught_share": rnd(caught_rule / n_wrong),
                   "rule_all_caught": int(wd[dis].sum()),
                   "per_object_wrong": {o: int(wd[od == o].sum()) for o in sorted(set(od))}})
        c4["met_4a"] = c4["auroc"] >= 0.80
        c4["met_4b"] = c4["caught_share"] >= 0.50
        c4["met_4c"] = caught >= caught_rule
        c4["met"] = c4["met_4a"] and c4["met_4b"] and c4["met_4c"]
    s["c4"] = c4

    # ---- criterion 5: the gap
    inview = y == 1
    silent = inview & ~detected
    ve_yes_box = yes & (vb > 0)
    s["c5"] = {"in_view": int(inview.sum()), "perception_silent": int(silent.sum()),
               "silent_share": rnd(silent.sum() / inview.sum()),
               "silent_ve_yes_box": int((silent & ve_yes_box).sum()),
               "silent_covered_share": rnd((silent & ve_yes_box).sum() / silent.sum()) if silent.sum() else None,
               "per_object": {o: {"in_view": int((inview & (objects == o)).sum()),
                                  "silent": int((silent & (objects == o)).sum()),
                                  "covered": int((silent & ve_yes_box & (objects == o)).sum())}
                              for o in sorted(set(objects))}}
    c5 = s["c5"]
    c5["astra_replay_proposed"] = bool(c5["silent_share"] >= 0.20 and
                                       (c5["silent_covered_share"] or 0) < 0.5)

    (HERE / "scores.json").write_text(json.dumps(s, indent=1, default=plain) + "\n")
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "per_object"} for k, v in s.items()},
                     indent=1, default=plain))


def plain(x):
    """numpy scalars for json."""
    return x.item()


def rnd(x):
    return None if x is None else round(float(x), 4)


def Counter_of(xs):
    out = {}
    for x in xs:
        out[str(x)] = out.get(str(x), 0) + 1
    return out


if __name__ == "__main__":
    main()
