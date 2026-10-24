"""3.80.E D10's latency part (run D): the frame question with its box
(`/identity`, `box: true`) to VE on the board, for the visible frames of
3.48's pinned 299, pass after pass until <seconds> run out -- while the
loaded stack runs beside it (`headroom_mission.sh`). The bar is read on the
answers that say yes (the long ones: the box is generated after the
decision). `deadline_s` 6, twice the brain's 3.0 s, so the time is seen, not
cut.

    APP_SHARED_SECRET=... PICAR_RECORDINGS=~/vision-picar/recordings \\
      python evaluations/tiered-380/E/d10_latency.py <seconds> <out.jsonl> [url]
"""

import json
import os
import sys
import time
from pathlib import Path

import httpx

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "tier-calibration-392"))
from common import b64, items  # noqa: E402

WALKS = {r["walk"] for r in json.load(open(HERE.parent / "C/driver/B1.json"))["records"]}


def main(seconds, out, url="http://127.0.0.1:8005"):
    frames = [(k, p, t) for k, p, t, v in items() if v and k.split("/")[0] in WALKS]
    secret = os.environ.get("APP_SHARED_SECRET")
    headers = {"x-app-secret": secret} if secret else {}
    end, n = time.time() + float(seconds), 0
    with httpx.Client(timeout=30, headers=headers) as c, open(out, "a") as f:
        while time.time() < end:
            for key, path, target in frames:
                if time.time() >= end:
                    break
                t0 = time.perf_counter()
                r = c.post(f"{url}/identity", json={"image_b64": b64(path.read_bytes()),
                                                     "target": target, "box": True,
                                                     "deadline_s": 6.0})
                got = r.json() if r.status_code == 200 else {"status": f"HTTP {r.status_code}"}
                f.write(json.dumps({"t": round(time.time(), 2), "key": key,
                                    "status": got.get("status"), "score": got.get("score"),
                                    "box": got.get("box"),
                                    "rtt_ms": round((time.perf_counter() - t0) * 1000)}) + "\n")
                f.flush()
                n += 1
    print(f"{len(frames)} visible frames, {n} requests", flush=True)


if __name__ == "__main__":
    main(*sys.argv[1:])
