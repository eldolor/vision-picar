"""
evaluations/tiered-380/N/trt_steer.py

3.80.N amendment 1: the identity-box question on 3.80.I's 142 frames (each
also mirrored), asked through the Edge-LLM driver with the reply read in full
(64 tokens, no stopping rule), so 3.80.M's reader reads InternVL's box
(`[["x1, y1, x2, y2"]]`). Runs on the board, one driver resident (nothing
else loaded). Writes rows in I/'s shape (no direction: that is the service's
run) to `--out`; `M/decide_scale.py` then reads the box's scale.

    python evaluations/tiered-380/N/trt_steer.py --engines ~/bench-380N/NVE/V512 \\
        --recordings ~/vp-378/recordings --out ~/out-380N/NVE-V512-box.jsonl
"""

import argparse
import io
import json
import sys
import tempfile
from pathlib import Path

from PIL import Image

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "M"))
sys.path.insert(0, str(HERE.parent / "I"))
sys.path.insert(0, str(HERE.resolve().parents[2]))
import common                                                # noqa: E402
import readers                                               # noqa: E402
import tools.jetson.bench_identity_trt as trt                # noqa: E402

trt.decision_reached = lambda content: False                 # read the whole reply


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engines", required=True)
    ap.add_argument("--recordings", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--edgellm", default="~/bench-378/ell91")
    a = ap.parse_args()
    from tokenizers import Tokenizer
    common.RECORDINGS = Path(a.recordings).expanduser()
    e = Path(a.engines).expanduser()
    ell = Path(a.edgellm).expanduser()
    tok = Tokenizer.from_file(str(e / "llm" / "tokenizer.json"))
    cache = {}

    def decode(i):
        if i not in cache:
            cache[i] = tok.decode([i], skip_special_tokens=False)
        return cache[i]

    out = Path(a.out).expanduser()
    done = set()
    if out.exists():
        done = {json.loads(l)["key"] for l in out.read_text().splitlines() if l.strip()}
    tmp = Path(tempfile.mkdtemp(prefix="trt380n-"))
    log = open(f"{out}.driver.log", "a")
    drv = trt.Driver([str(ell / "build/examples/llm/edgellm_identity"), str(e / "llm"),
                      str(e / "visual")], log, str(ell / "build/libNvInfer_edgellm_plugin.so"))
    drv.wait_ready()

    def ask(jpeg_path, target, n):
        req = tmp / f"{n}.json"
        req.write_text(json.dumps(trt.request_doc(jpeg_path, target)))
        got = drv.ask(req, decode)
        text = "".join(c["token"] for c in got["content"])
        score, parsed, _ = readers.p_box(got["content"])
        return {"id_status": "answered" if parsed else "unparsed", "id_score": round(score, 4),
                "box": readers.parse_any_box(text), "id_text": text[:200],
                "id_ms": round(got["ms"]), "img": list(Image.open(jpeg_path).size)}

    n = 0
    try:
        with out.open("a") as f:
            for set_, key, path, target, truth in common.items():
                if key in done:
                    continue
                n += 1
                mirror = tmp / f"m{n}.jpg"
                mirror.write_bytes(common.mirrored(Path(path).read_bytes()))
                row = {"set": set_, "key": key, "target": target, "truth": truth,
                       **ask(Path(path), target, f"a{n}"), "mirror": ask(mirror, target, f"b{n}")}
                f.write(json.dumps(row) + "\n")
                f.flush()
                print(set_, key, truth, row["box"], row["mirror"]["box"], flush=True)
    finally:
        drv.close()
        log.close()


if __name__ == "__main__":
    main()
