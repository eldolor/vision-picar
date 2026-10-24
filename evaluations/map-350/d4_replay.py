"""3.50.D4 amendment 4: candidate contradiction rules replayed on the sessions
d4_record.py kept, against the map as reloaded (the first /map recorded).

    python evaluations/map-350/d4_replay.py RECDIR [NAME=VALUE ...]

NAME=VALUE sets a remap.py constant (MIN_RUN_M=0.3 REQUIRE_REST=1 ...). Each
fire is placed in the house frame (a rigid fit of the robot's map pose to
its truth) and labelled TRUE if it lies within NEAR_M of a cell a move
changed (an object's place in the map run or in the session), else FALSE.
Prints one line per session and a JSON summary on the last line.
"""
import glob
import json
import math
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "service/slam/src/picar_bridge"))

from picar_bridge import remap  # noqa: E402

CELL = 0.3
NEAR_M = 0.6
LIDAR_X_M = 0.04
# The bridge's REST_LINEAR_M_S, REST_ANGULAR_RAD_S, REMAP_REST_S, SCAN_HZ and
# REST_ODOM_MAX_AGE_S, REMAP_SCAN_MAX_AGE_S (bridge.py needs rclpy, so they
# are repeated here; tests/test_map_persist.py checks they agree).
REST_LIN, REST_ANG, REST_S, SCAN_HZ, ODOM_MAX_AGE_S = 0.01, 0.02, 0.5, 10.0, 0.5
SCAN_MAX_AGE_S = 0.5


def grid(m):
    data = np.frombuffer(m["data"], dtype=np.uint8).astype(np.int16)
    data[data > 127] -= 256
    return {"resolution_m": m["res"], "width": m["w"], "height": m["h"],
            "origin_x_m": m["ox"], "origin_y_m": m["oy"], "data": data.tolist()}


def fit_house(rec):
    """Map -> house rigid transform, least squares over (centre in map, truth)."""
    tr = np.array([r[:3] for r in rec["truth"]])
    if len(tr) < 10:
        return None
    a, b = [], []
    for s in rec["scans"][::5]:
        k = np.searchsorted(tr[:, 0], s["at"])
        if 0 < k < len(tr) and tr[k, 0] - tr[k - 1, 0] < 0.3:
            x, y, yaw = s["laser"]
            a.append((x - LIDAR_X_M * math.cos(yaw), y - LIDAR_X_M * math.sin(yaw)))
            b.append(tr[k, 1:3])
    a, b = np.array(a), np.array(b)
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    best = None
    for d in (1, -1):                 # the house frame's y runs down: a mirror
        rot = vt.T @ np.diag([1, d]) @ u.T
        resid = float(np.median(np.linalg.norm((rot @ (a - ca).T).T + cb - b, axis=1)))
        if best is None or resid < best[3]:
            best = (rot, ca, cb, resid)
    return best


def still_flags(rec):
    """Per scan: the bridge's own rest rule (remap.RestClock, with its
    constants), fed the recorded odometry, with the bridge's deadline: a
    scan odometry has not passed within SCAN_MAX_AGE_S of its arrival is
    not at rest (review round 3), nor one odometry never passes."""
    rc = remap.RestClock(REST_LIN, REST_ANG, REST_S, 1.0 / SCAN_HZ, ODOM_MAX_AGE_S)
    # The bridge times odometry by its own stamp; recordings before review
    # round 2 kept only the receipt time (the same host's clock, ms apart).
    od = sorted((o[3] if len(o) > 3 and o[3] > 0 else o[0], o[1], o[2], o[0])
                for o in rec.get("odom") or [])
    out, k, got = {}, 0, -math.inf
    for s in sorted(rec["scans"], key=lambda s: s["stamp_ns"]):
        cap = s["stamp_ns"] * 1e-9
        while k < len(od) and rc.still_for(cap) is None:
            rc.on_odom(*od[k][:3])
            got = od[k][3]                       # when that message reached the bridge
            k += 1
        decided = rc.still_for(cap)
        # The deciding message had to arrive before the bridge gave up.
        out[s["stamp_ns"]] = bool(decided) and got <= s["at"] + SCAN_MAX_AGE_S
    return [out[s["stamp_ns"]] for s in rec["scans"]]


def changed_cells(rec_dir, name):
    base = os.path.join(rec_dir, name.replace(".pkl", ".base.json"))
    if not os.path.exists(base):
        return []
    cells = []
    for src, dst in json.load(open(base)).get("moves", []):
        cells += [tuple(src), tuple(dst)]
    return cells


def replay(path, rec_dir):
    rec = pickle.load(open(path, "rb"))
    name = os.path.basename(path)
    saved = remap.SavedMap(grid(rec["maps"][0]))
    fit = fit_house(rec)
    changed = [((c + 0.5) * CELL, (r + 0.5) * CELL) for c, r in changed_cells(rec_dir, name)]
    still = still_flags(rec)
    tr = remap.Contradictions(saved)
    # The session proper: from the truth's first sample (after the reload)
    # for its recorded length; the recorder runs on after it.
    w0 = rec["truth"][0][0] if rec.get("truth") else -math.inf
    w1 = w0 + rec.get("seconds", math.inf)
    fires = []
    for s, ok in zip(rec["scans"], still):
        f = tr.add(s, s["laser"], s["stamp_ns"] * 1e-9, still=ok)
        if f is None:
            continue
        hx, hy = (fit[0] @ (np.array(f["centre_m"]) - fit[1]) + fit[2]) if fit else (math.nan,) * 2
        near = min((math.dist((hx, hy), c) for c in changed), default=math.inf)
        fires.append({"t": round(s["at"] - rec["scans"][0]["at"], 1), "cells": f["cells"],
                      "house_m": [round(float(hx), 2), round(float(hy), 2)],
                      "true": near <= NEAR_M, "agreement": f["agreement"],
                      "in_session": w0 <= s["at"] <= w1,
                      "range_m": round(math.dist(f["centre_m"], s["laser"][:2]), 2)})
        tr = remap.Contradictions(saved)        # as the live trigger: one fire, then start over
    return {"session": name, "scans": len(rec["scans"]), "fires": fires,
            "true": sum(f["true"] for f in fires), "false": sum(not f["true"] for f in fires),
            "false_in_session": sum(not f["true"] and f["in_session"] for f in fires),
            "still_share": round(sum(still) / max(len(still), 1), 2),
            "fit_resid_m": round(fit[3], 3) if fit else None,
            "lost": tr.lost, "seconds": round(rec["scans"][-1]["at"] - rec["scans"][0]["at"])}


def main(argv):
    rec_dir = argv[0]
    for kv in argv[1:]:
        k, v = kv.split("=")
        old = getattr(remap, k)
        setattr(remap, k, type(old)(float(v)) if not isinstance(old, bool) else v not in ("0", ""))
    rows = [replay(p, rec_dir) for p in sorted(glob.glob(os.path.join(rec_dir, "*.pkl")))]
    for r in rows:
        print(f"{r['session']:28s} {r['seconds']:4d}s still {r['still_share']:.2f} "
              f"fit {r['fit_resid_m']} TRUE {r['true']} FALSE {r['false']} "
              + " ".join(f"{f['t']}s:{f['cells']}@{f['house_m']}a{f['agreement']}r{f['range_m']}{'*' if f['true'] else ''}"
                         for f in r["fires"][:12]))
    print(json.dumps({"params": argv[1:], "true": sum(r["true"] for r in rows),
                      "false": sum(r["false"] for r in rows),
                      "false_in_session": sum(r["false_in_session"] for r in rows),
                      "by_session": {r["session"]: [r["true"], r["false"], r["false_in_session"]]
                                     for r in rows}}))


if __name__ == "__main__":
    main(sys.argv[1:])
