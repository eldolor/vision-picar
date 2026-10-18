"""
evaluations/tiered-380/J/run_llama.py

3.80.J: one model under the board's llama-server (reached through an ssh
tunnel), asked the local VLM service's two questions on 3.80.I's sets, each
frame as is and mirrored:
* identity: 3.48's box prompt; score = P(it localises) (`p_localised`); a yes
  (score >= 0.5) with a parsed box -> the box centre's third, the box read
  in the pixels the model saw (`model_size`: the server runs with
  --image-min-tokens 128 --image-max-tokens 512, the engines' bounds);
* direction: 3.80.F's DIRECTION_PROMPT; the answer read off the first word
  token's top-20 alternatives (`direction_of`, the service's own reader).
Writes `<name>.jsonl` in I/'s row shape, so I/score.py scores it. Free.

    .venv/bin/python evaluations/tiered-380/J/run_llama.py <name> [server-url]
"""

import io
import json
import sys
import time
from pathlib import Path

import httpx
from PIL import Image

HERE = Path(__file__).parent
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE.parent / "I"))
from brain.local_vlm import model_size                                  # noqa: E402
from common import b64, items, mirrored, third                         # noqa: E402
from service.local_vlm.driver import DIRECTION_PROMPT, direction_of, parse_box   # noqa: E402
from tools.jetson.bench_identity_llama import PROMPT, p_localised        # noqa: E402

YES_AT = 0.5


def chat(c, url, jpeg, text, max_tokens):
    body = {"messages": [{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + b64(jpeg)}},
                {"type": "text", "text": text}]}],
            "max_tokens": max_tokens, "temperature": 0, "top_k": 1,
            "logprobs": True, "top_logprobs": 20}
    t0 = time.perf_counter()
    r = c.post(f"{url}/v1/chat/completions", json=body)
    r.raise_for_status()
    ch = r.json()["choices"][0]
    return (ch["message"]["content"], (ch.get("logprobs") or {}).get("content") or [],
            round((time.perf_counter() - t0) * 1000))


def answer(c, url, jpeg, target):
    text, content, id_ms = chat(c, url, jpeg, PROMPT.format(target=target), 64)
    score, parsed, _ = p_localised(content)
    box = parse_box(text)
    frac = None
    if parsed and score >= YES_AT and box is not None:
        w, _h = model_size(*Image.open(io.BytesIO(jpeg)).size)
        frac = round(min(1.0, max(0.0, (box[0] + box[2]) / 2 / w)), 4)
    _t, dcontent, dir_ms = chat(c, url, jpeg, DIRECTION_PROMPT.format(target=target), 8)
    read = None
    for step in dcontent:
        read = direction_of(step)
        if read is not None:
            break
    word = read["direction"] if read and read["mass"] >= 0.5 else None
    return {"direction": word.upper() if word else None,
            "dir_status": "answered" if word else "unparsed",
            "probs": read["probs"] if read else None, "dir_rtt_ms": dir_ms,
            "id_status": "answered" if parsed else "unparsed", "id_score": round(score, 4),
            "box": box, "box_frac_x": frac, "box_answer": third(frac), "id_rtt_ms": id_ms,
            "id_text": text[:200]}


def main(name, url="http://127.0.0.1:18006"):
    out = HERE / f"{name}.jsonl"
    done = set()
    if out.exists():
        done = {json.loads(l)["key"] for l in out.read_text().splitlines() if l.strip()}
    with httpx.Client(timeout=120) as c, out.open("a") as f:
        for set_, key, path, target, truth in items():
            if key in done:
                continue
            jpeg = path.read_bytes()
            row = {"set": set_, "key": key, "target": target, "truth": truth,
                   **answer(c, url, jpeg, target), "mirror": answer(c, url, mirrored(jpeg), target)}
            f.write(json.dumps(row) + "\n"); f.flush()
            print(set_, key, truth, row["direction"], row["box_answer"],
                  row["mirror"]["direction"], row["mirror"]["box_answer"], flush=True)


if __name__ == "__main__":
    main(*sys.argv[1:])
