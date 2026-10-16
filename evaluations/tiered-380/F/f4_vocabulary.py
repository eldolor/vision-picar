"""
evaluations/tiered-380/F/f4_vocabulary.py

3.80.F criterion F4: does VE's reduced vocabulary carry `left`, `right` and
`forward`? The direction question on 5 simulator renders (3.80.E's
render-check, target in view at 1-3 m) and 5 real frames (labelled walks,
target in view or not), through the board's service. Met if `mass` >= 0.5
on at least 8 of 10 (amendment, 2026-10-16, before the run: and each of
the three words is answered at least once -- the first pass had mass on
10/10 and said `right` every time, VE's vocabulary having lost bare `left`
and `forward`). Free (local only).

    .venv/bin/python evaluations/tiered-380/F/f4_vocabulary.py [service-url]
"""

import base64
import json
import sys
import time
from pathlib import Path

import httpx

HERE = Path(__file__).parent
RENDERS = HERE.parent / "E" / "render-check"
RECORDINGS = Path("/Users/anshugaind/vision-picar/recordings")
RENDER_FILES = ["home_first_floor-1.0m.jpg", "home_first_floor-3.0m.jpg",
                "procthor-val-18-2.0m.jpg", "procthor-val-5-1.5m.jpg",
                "procthor-val-5-1.0m.jpg"]
REAL = [("red-backpack-20260907-144856", "frame-0005.jpg"),
        ("red-backpack-20260907-144856", "frame-0030.jpg"),
        ("blue-bottle-20260907-142454", "frame-0010.jpg"),
        ("blue-shoes-20260907-152528", "frame-0015.jpg"),
        ("grey-backpack-20260921-203627", "frame-0015.jpg")]


def target_of(walk: str) -> str:
    return json.loads((RECORDINGS / walk / "labels.json").read_text())["description"]


def main(url="http://127.0.0.1:18005"):
    items = [("render", RENDERS / f, "red backpack", None) for f in RENDER_FILES]
    for walk, frame in REAL:
        labels = json.loads((RECORDINGS / walk / "labels.json").read_text())
        items.append(("real", RECORDINGS / walk / frame, labels["description"],
                      labels["target_visible_labels"].get(frame)))
    rows = []
    with httpx.Client(timeout=10) as c:
        for kind, path, target, visible in items:
            body = {"image_b64": base64.b64encode(path.read_bytes()).decode(),
                    "target": target, "deadline_s": 3.0}
            t0 = time.perf_counter()
            got = c.post(f"{url}/direction", json=body).json()
            rows.append({"kind": kind, "image": str(path.name), "target": target,
                         "visible": visible, "rtt_ms": round((time.perf_counter() - t0) * 1000),
                         **{k: got.get(k) for k in ("status", "direction", "p", "mass",
                                                    "probs", "ms")}})
            print(json.dumps(rows[-1]), flush=True)
    ok = sum(1 for r in rows if (r["mass"] or 0) >= 0.5)
    words = sorted({r["direction"] for r in rows if r["direction"]})
    met = ok >= 8 and len(words) == 3
    out = {"rows": rows, "mass_ok": ok, "of": len(rows), "words": words, "met": met}
    (HERE / "f4.json").write_text(json.dumps(out, indent=1) + "\n")
    print(f"F4: mass >= 0.5 on {ok}/{len(rows)}, words {words} -> "
          f"{'met' if met else 'FAILED'}")


if __name__ == "__main__":
    main(*sys.argv[1:])
