"""
evaluations/tiered-380/I/run_opus.py

3.80.I, Opus 4.5's steer through the deployed /navigate (fresh calls; PAID:
run only with the user's yes). S1, S2, S3, each as is and mirrored. `action`
(FORWARD / LEFT / RIGHT, else wrong), and on every frame `target_direction`
(left / center / right -> the third). Stops at CAP_USD.

    .venv/bin/python evaluations/tiered-380/I/run_opus.py [--cap 3.0]
"""

import argparse
import json
import sys
import time
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "E"))
from common import HERE, b64, items, mirrored                         # noqa: E402
from pilot import DEFAULT_VISION_URL, cost, secrets       # noqa: E402

THIRD = {"left": "LEFT", "center": "FORWARD", "right": "RIGHT"}


def ask(c, url, secret, jpeg, target):
    t0 = time.monotonic()
    r = c.post(f"{url}/navigate", headers={"x-app-secret": secret},
               json={"image_base64": b64(jpeg), "media_type": "image/jpeg",
                     "target_object": target, "searched_rooms": []})
    got = r.json() if r.status_code == 200 else {}
    act = got.get("action")
    return {"status": r.status_code, "action": act if act in ("FORWARD", "LEFT", "RIGHT") else
            (f"other:{act}" if act else None),
            "target_visible": got.get("target_visible"),
            "target_third": THIRD.get(str(got.get("target_direction")).lower()),
            "target_direction": got.get("target_direction"),
            "model_id": got.get("model_id"), "usage": got.get("usage"),
            "cost_usd": cost(got.get("usage") or {}), "s": round(time.monotonic() - t0, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cap", type=float, default=3.0)
    args = ap.parse_args()
    s = secrets()
    url = (s.get("VISION_URL") or DEFAULT_VISION_URL).rstrip("/")
    secret = s.get("VISION_SECRET", "")
    out = HERE / "opus.jsonl"
    spent, done = 0.0, set()
    if out.exists():
        for line in out.read_text().splitlines():
            r = json.loads(line)
            done.add(r["key"])
            spent += r["cost_usd"] + r["mirror"]["cost_usd"]
    with httpx.Client(timeout=60) as c, out.open("a") as f:
        for set_, key, path, target, truth in items():
            if key in done:
                continue
            if spent >= args.cap:
                print(f"STOP: cap ${args.cap} reached (${spent:.2f})")
                break
            jpeg = path.read_bytes()
            a = ask(c, url, secret, jpeg, target)
            m = ask(c, url, secret, mirrored(jpeg), target)
            spent += a["cost_usd"] + m["cost_usd"]
            row = {"set": set_, "key": key, "target": target, "truth": truth, **a,
                   "cost_usd": a["cost_usd"], "mirror": m}
            f.write(json.dumps(row) + "\n"); f.flush()
            print(set_, key, truth, a["action"], a["target_third"], m["action"],
                  f"${spent:.3f}", flush=True)
    print(f"spent ${spent:.3f}")


if __name__ == "__main__":
    main()
