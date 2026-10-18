"""
tools/jetson/bench_local_vlm.py

3.80.B's acceptance B1, B2 and B6 (`docs/plans/ros-alignment/3.80-tiered-identity.md`):
the 299 pinned frames asked through the local VLM service's HTTP API, so the
answers can be compared with 3.78's VE records taken straight from the driver.

    python -m tools.jetson.bench_local_vlm frames --recordings recordings \\
        --walk <w> ... --out B1.json
    python -m tools.jetson.bench_local_vlm deadline --recordings recordings \\
        --walk <w> --out B6.json
    python -m tools.jetson.bench_local_vlm compare B1.json \\
        evaluations/identity-378/V/TVE-299.json

`ms` is the client's whole round trip (base64, HTTP, the service's file
write, the driver, the reply). Records have `bench_identity_trt`'s shape.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import statistics
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional, Sequence

from tools.jetson.bench_identity_llama import WARMUP, percentile

URL = "http://127.0.0.1:8005"


def post(url: str, jpeg: bytes, target: str, deadline_s: float, secret: str) -> tuple:
    body = json.dumps({"image_b64": base64.b64encode(jpeg).decode(), "target": target,
                       "deadline_s": deadline_s}).encode()
    req = urllib.request.Request(f"{url}/identity", data=body, headers={
        "content-type": "application/json", "x-app-secret": secret})
    t0 = time.perf_counter()
    try:
        with urllib.request.urlopen(req, timeout=deadline_s + 30) as r:
            got = json.loads(r.read())
    except urllib.error.HTTPError as e:
        got = {"status": f"http {e.code}", "score": None}
    except (urllib.error.URLError, OSError, ValueError) as e:
        # One refused, timed-out or garbled call is one failed frame, not the
        # end of a 299-frame run with nothing written (3.76 P1).
        got = {"status": f"error {type(e).__name__}", "score": None}
    if not isinstance(got, dict) or "status" not in got:
        got = {"status": "error no status", "score": None}
    return got, (time.perf_counter() - t0) * 1000


def compare(new: Sequence[dict], old: Sequence[dict], tol: float = 0.01) -> dict:
    """B1: per frame, the score within `tol` of the old record's and the
    decision at 0.5 the same. A frame either side has no score for is a
    mismatch."""
    old_by = {f"{r['walk']}/{r['frame']}": r for r in old}
    off, flips, missing = [], [], []
    for r in new:
        k = f"{r['walk']}/{r['frame']}"
        o = old_by.get(k)
        if o is None or r["score"] is None or o["score"] is None:
            missing.append(k)
            continue
        if abs(r["score"] - o["score"]) > tol:
            off.append({"frame": k, "new": r["score"], "old": o["score"]})
        if (r["score"] >= 0.5) != (o["score"] >= 0.5):
            flips.append(k)
    return {"frames": len(new), "within_tol": len(new) - len(off) - len(missing),
            "off": off, "flips": flips, "missing": missing,
            "met": not off and not flips and not missing and len(new) == len(old)}


def _frames(args):  # pragma: no cover -- needs the corpus
    from control.perception_eval import load_corpus
    walks = load_corpus(Path(args.recordings), only=args.walk)
    return [(w, p, vis, adj) for w in walks for p, vis, adj in w.frames]


def main(argv: Optional[Sequence[str]] = None) -> int:  # pragma: no cover -- needs the board
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("frames", "deadline"):
        s = sub.add_parser(name)
        s.add_argument("--url", default=URL)
        s.add_argument("--recordings", default="recordings")
        s.add_argument("--walk", action="append", required=True)
        s.add_argument("--out", required=True)
    c = sub.add_parser("compare")
    c.add_argument("new")
    c.add_argument("old")
    args = ap.parse_args(argv)

    if args.cmd == "compare":
        res = compare(json.load(open(args.new))["records"], json.load(open(args.old))["records"])
        print(json.dumps({k: v for k, v in res.items() if k != "off"} | {"off": res["off"][:10]}))
        return 0 if res["met"] else 1

    secret = os.environ.get("APP_SHARED_SECRET", "")
    frames = _frames(args)
    if args.cmd == "deadline":
        # B6: a deadline far under the model's ~1 s, then the same frame
        # with room, which must answer as an undisturbed request does.
        w, p, vis, adj = frames[0]
        jpeg = p.read_bytes()
        short, short_ms = post(args.url, jpeg, w.target, 0.3, secret)
        after, after_ms = post(args.url, jpeg, w.target, 10.0, secret)
        doc = {"frame": f"{w.name}/{p.name}", "short": short, "short_ms": short_ms,
               "after": after, "after_ms": after_ms,
               "met": short.get("status") == "timeout" and short_ms <= 800
               and after.get("status") == "answered"}
        Path(args.out).write_text(json.dumps(doc, indent=1))
        print(json.dumps(doc))
        return 0

    for w, p, _, _ in frames[:WARMUP]:
        post(args.url, p.read_bytes(), w.target, 10.0, secret)
    records, ms, statuses = [], [], {}
    for w, p, vis, adj in frames:
        got, took = post(args.url, p.read_bytes(), w.target, 10.0, secret)
        statuses[got["status"]] = statuses.get(got["status"], 0) + 1
        score = got.get("score")
        if got["status"] == "unparsed":
            score = 0.0  # 3.78 scored a reply that never opened a list as 0
        ms.append(took)
        records.append({"walk": w.name, "frame": p.name, "visible": vis,
                        "status": ("unavailable" if score is None
                                   else "detected" if score >= 0.5 else "absent"),
                        "score": score, "metric": "confidence", "label": None,
                        "candidates": 1 if score is not None else 0,
                        "adjudicated": adj, "ms": took,
                        "service_status": got["status"], "driver_ms": got.get("driver_ms")})
        print(f"{w.name}/{p.name} vis={vis} {got['status']} p={score} {took:.0f}ms", flush=True)
    doc = {"config": {"runtime": "service/local_vlm (3.80.B) over HTTP", "url": args.url},
           "records": records, "statuses": statuses,
           "latency_ms": {"n": len(ms), "p50": statistics.median(ms),
                          "p90": percentile(ms, 90), "max": max(ms)}}
    Path(args.out).write_text(json.dumps(doc, indent=1))
    print(json.dumps({"statuses": statuses, "latency_ms": doc["latency_ms"]}))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
