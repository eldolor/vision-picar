"""
evaluations/tier-calibration-392/run_qwen.py

3.92.A: the live frame question (`/identity`, `box: true`) to the board's
local VLM service for every labelled frame. Resumable. Free.

    APP_SHARED_SECRET=... .venv/bin/python evaluations/tier-calibration-392/run_qwen.py [service-url]
"""

import json
import os
import sys
import time

import httpx

from common import HERE, b64, done, items


def main(url="http://127.0.0.1:18005"):
    out = HERE / "qwen-VE.jsonl"
    seen = done(out)
    secret = os.environ.get("APP_SHARED_SECRET")
    headers = {"x-app-secret": secret} if secret else {}
    with httpx.Client(timeout=30, headers=headers) as c, out.open("a") as f:
        for key, path, target, visible in items():
            if key in seen:
                continue
            t0 = time.perf_counter()
            r = c.post(f"{url}/identity", json={"image_b64": b64(path.read_bytes()),
                                                 "target": target, "box": True,
                                                 "deadline_s": 6.0})
            got = r.json() if r.status_code == 200 else {"status": f"HTTP {r.status_code}"}
            row = {"key": key, "target": target, "visible": visible,
                   "status": got.get("status"), "score": got.get("score"),
                   "box": got.get("box"), "rtt_ms": round((time.perf_counter() - t0) * 1000)}
            f.write(json.dumps(row) + "\n"); f.flush()
            print(key, visible, row["status"], row["score"], row["rtt_ms"], flush=True)


if __name__ == "__main__":
    main(*sys.argv[1:])
