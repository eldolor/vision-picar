"""
evaluations/tiered-380/I/run_qwen.py

3.80.I, local Qwen's steer on S1, S2 and S3 (each as is and mirrored),
through the board's service (`service/local_vlm/`):
* `/direction` -> the word (left / right / forward), with its probabilities;
* `/identity` with `box: true` -> a yes (score >= 0.5, tiered's LOCAL_YES_AT)
  with a box -> the box centre's third; else no answer.
Round trips are timed here (the tunnel adds ~0.05 s). Free.

    .venv/bin/python evaluations/tiered-380/I/run_qwen.py <engine-name> [service-url]
"""

import json
import sys
import time
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).parent))
from common import HERE, b64, items, mirrored, third       # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from brain.local_vlm import _box_frac                     # noqa: E402

YES_AT = 0.5


def post(c, url, route, jpeg, target, **extra):
    t0 = time.perf_counter()
    r = c.post(f"{url}/{route}", json={"image_b64": b64(jpeg), "target": target,
                                         "deadline_s": 6.0, **extra})
    got = r.json() if r.status_code == 200 else {"status": f"HTTP {r.status_code}"}
    got["rtt_ms"] = round((time.perf_counter() - t0) * 1000)
    return got


def answer(c, url, jpeg, target):
    d = post(c, url, "direction", jpeg, target)
    i = post(c, url, "identity", jpeg, target, box=True)
    word = d.get("direction") if d.get("status") == "answered" else None
    frac = None
    score = i.get("score")
    if i.get("status") == "answered" and isinstance(score, (int, float)) and score >= YES_AT \
            and i.get("box") is not None:
        bf = _box_frac(i["box"], jpeg)
        frac = round((bf[0] + bf[2]) / 2, 4) if bf else None
    return {"direction": word.upper() if word else None,
            "dir_status": d.get("status"), "probs": d.get("probs"), "dir_rtt_ms": d["rtt_ms"],
            "id_status": i.get("status"), "id_score": score, "box": i.get("box"), "box_frac_x": frac,
            "box_answer": third(frac), "id_rtt_ms": i["rtt_ms"]}


def main(engine, url="http://127.0.0.1:18005"):
    out = HERE / f"qwen-{engine}.jsonl"
    done = set()
    if out.exists():
        done = {json.loads(l)["key"] for l in out.read_text().splitlines() if l.strip()}
    with httpx.Client(timeout=30) as c, out.open("a") as f:
        for set_, key, path, target, truth in items():
            if key in done:
                continue
            jpeg = path.read_bytes()
            row = {"set": set_, "key": key, "target": target, "truth": truth,
                   **answer(c, url, jpeg, target),
                   "mirror": answer(c, url, mirrored(jpeg), target)}
            f.write(json.dumps(row) + "\n"); f.flush()
            print(set_, key, truth, row["direction"], row["box_answer"],
                  row["mirror"]["direction"], row["mirror"]["box_answer"], flush=True)


if __name__ == "__main__":
    main(*sys.argv[1:])
