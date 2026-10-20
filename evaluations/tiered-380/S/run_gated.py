"""
evaluations/tiered-380/S/run_gated.py

3.80.S on the board, one resolution: YOLOE + CLIP (3.80.I's
`run_perception.answer`) on every S1/S2 frame, each also mirrored, and on
the 3 frames before it in its walk (the gate's window); Florence-2-base's
`<CAPTION_TO_PHRASE_GROUNDING>` box (`R/native_server.py`, already serving)
on every S1/S2 frame and its mirror. Writes one row a frame; the pipelines
(A, G, F, FS) are composed from the rows by `S/score.py`, so every
pipeline sees the same answers.

Amendment 1: `--second ve` asks VE's service (`service/local_vlm/`,
already serving) `/identity` with `box: true` in Florence's place, the
question the tier asks; its rows carry `ve` where Florence's carry
`florence`.

    python evaluations/tiered-380/S/run_gated.py orig|640|320 \
        --recordings ~/vision-picar/recordings --out ~/out-380S/orig.jsonl
"""

import argparse
import io
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE.parent / "I"))
import common                                                  # noqa: E402
from common import b64, mirrored                               # noqa: E402
from run_perception import answer                              # noqa: E402

WINDOW = 3
WARMUP = 3
PROMPT = "<CAPTION_TO_PHRASE_GROUNDING>{target}"


def at_res(path: Path, res: str) -> bytes:
    """The frame as recorded (`orig`), or its long side downscaled to `res`
    pixels (Lanczos, JPEG quality 92). Never upscales."""
    raw = path.read_bytes()
    if res == "orig":
        return raw
    from PIL import Image
    im = Image.open(io.BytesIO(raw)).convert("RGB")
    n = int(res)
    s = n / max(im.size)
    if s < 1:
        im = im.resize((round(im.width * s), round(im.height * s)), Image.LANCZOS)
    out = io.BytesIO()
    im.save(out, "JPEG", quality=92)
    return out.getvalue()


def width_of(jpeg: bytes) -> tuple:
    from PIL import Image
    return Image.open(io.BytesIO(jpeg)).size


def florence(url: str, jpeg: bytes, target: str, width: int) -> dict:
    body = {"messages": [{"role": "user", "content": [
        {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + b64(jpeg)}},
        {"type": "text", "text": PROMPT.format(target=target)}]}],
        "max_tokens": 64}
    req = urllib.request.Request(url + "/v1/chat/completions", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    t0 = time.perf_counter()
    with urllib.request.urlopen(req, timeout=120) as r:
        reply = json.loads(r.read())
    ms = round((time.perf_counter() - t0) * 1000)
    boxes = [b["bbox_2d"] for b in json.loads(reply["choices"][0]["message"]["content"] or "[]")]
    # The first box, as 3.80.R's reader takes it; its centre's third.
    frac = round((boxes[0][0] + boxes[0][2]) / 2 / width, 4) if boxes else None
    return {"boxes": len(boxes), "box": boxes[0] if boxes else None, "frac": frac,
            "answer": common.third(frac), "ms": ms}


def ve(url: str, jpeg: bytes, target: str, width: int) -> dict:
    """VE's yes (score >= 0.5, tiered's LOCAL_YES_AT) with a box -> the box
    centre's third, as 3.80.I's `run_qwen.answer` reads it."""
    from brain.local_vlm import _box_frac
    body = {"image_b64": b64(jpeg), "target": target, "deadline_s": 6.0, "box": True}
    req = urllib.request.Request(url + "/identity", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    t0 = time.perf_counter()
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            got = json.loads(r.read())
    except urllib.error.HTTPError as e:   # busy or unable: no answer, as 3.80.I's `post`
        got = {"status": f"HTTP {e.code}"}
    except (urllib.error.URLError, TimeoutError) as e:   # no answer, recorded
        got = {"status": f"error {type(e).__name__}"}
    ms = round((time.perf_counter() - t0) * 1000)
    score, frac = got.get("score"), None
    if got.get("status") == "answered" and isinstance(score, (int, float)) and score >= 0.5 \
            and got.get("box") is not None:
        bf = _box_frac(got["box"], jpeg)
        frac = round((bf[0] + bf[2]) / 2, 4) if bf else None
    return {"status": got.get("status"), "score": score, "box": got.get("box"), "frac": frac,
            "answer": common.third(frac), "ms": ms}


def previous(path: Path) -> list:
    """The up-to-WINDOW frames before `path` in its walk, nearest first."""
    n = int(path.stem.split("-")[1])
    out = []
    for i in range(n - 1, max(n - 1 - WINDOW, -1), -1):
        p = path.with_name(f"frame-{i:04d}.jpg")
        if p.exists():
            out.append(p)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("res", choices=("orig", "640", "320"))
    ap.add_argument("--recordings", required=True)
    ap.add_argument("--florence", default="http://127.0.0.1:8006")
    ap.add_argument("--second", choices=("florence", "ve"), default="florence")
    ap.add_argument("--ve", default="http://127.0.0.1:8007")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    common.RECORDINGS = Path(args.recordings).expanduser()

    from brain.perceive import pipeline_for
    # One pipeline at a time, as the brain holds one a mission:
    # `pipeline_for` frees the previous one only once nothing refers to it.
    held, seen = {}, {}

    def perceive(path, target, jpeg=None):
        jpeg = jpeg if jpeg is not None else at_res(path, args.res)
        w, _h = width_of(jpeg)
        if held.get("target") != target:
            held.clear()
            held.update(target=target, pipe=pipeline_for(target))
        pipe = held["pipe"]
        t0 = time.perf_counter()
        a = answer(pipe, jpeg, w)
        a["ms"] = round((time.perf_counter() - t0) * 1000)
        return a

    name = args.second
    ask = (lambda jpeg, t, w: florence(args.florence, jpeg, t, w)) if name == "florence" \
        else (lambda jpeg, t, w: ve(args.ve, jpeg, t, w))

    frames = [(s, k, p, t, tr) for s, k, p, t, tr in common.items() if s != "S3"]
    for s, k, p, t, tr in frames[:WARMUP]:
        jpeg = at_res(p, args.res)
        perceive(p, t, jpeg)
        ask(jpeg, t, width_of(jpeg)[0])

    with open(Path(args.out).expanduser(), "w") as f:
        for s, k, p, t, tr in frames:
            jpeg = at_res(p, args.res)
            w, h = width_of(jpeg)
            prev = []
            for q in previous(p):
                key = (str(q), t)
                if key not in seen:
                    seen[key] = perceive(q, t)
                prev.append({"frame": q.name, "status": seen[key]["status"], "p": seen[key]["p"]})
            m = mirrored(jpeg)
            row = {"set": s, "key": k, "target": t, "truth": tr, "res": args.res, "img": [w, h],
                   "perc": perceive(p, t, jpeg), "prev": prev,
                   name: ask(jpeg, t, w),
                   "mirror": {"perc": perceive(p, t, m), name: ask(m, t, w)}}
            f.write(json.dumps(row) + "\n")
            f.flush()
            print(s, k, tr, row["perc"]["status"], row["perc"]["answer"],
                  [x["status"] for x in prev], row[name]["answer"], flush=True)


if __name__ == "__main__":
    main()
