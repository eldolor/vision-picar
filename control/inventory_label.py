"""3.46 criterion 6: does a detector that names everything name things right?

    python -m control.inventory_label vocab              # once: which of the 4,585 names are objects
    python -m control.inventory_label detect             # the detector over every walk (free, CPU)
    python -m control.inventory_label judge --limit 5    # the cloud judge; prints the cost so far
    python -m control.inventory_label judge              # every frame, stops at --budget dollars
    python -m control.inventory_label sample             # the user's 150 frames, as a local page
    python -m control.inventory_label score [--human F]  # threshold, precision, agreement

The method is `PLAN-ros-alignment.md` 3.46 amendment 3, written before any
run. In short:

* **Frames** are every frame of every walk under `recordings/`. Each
  walk's own `labels.json` is never read or written here: it says only
  whether that walk's TARGET is visible, and it stays the target's
  reference (`control/perception_eval.py`).
* **The detector** is YOLOE's prompt-free checkpoint, whose 4,585 names
  include rooms, surfaces and ideas as well as things. `vocab` asks the
  judge model once which names are physical objects with a place; only
  those boxes are ever judged or scored.
* **The judge** (Claude Opus 5.5 on Bedrock) sees a frame with the boxes
  drawn and numbered, and says per box whether the label is right. It is a
  REFERENCE, not the truth -- the same rule `control/label_assist.py`
  keeps: a model's claim never becomes a label by itself.
* **The truth** is the user's verdict on 150 sampled frames, made on a
  local page (`sample`) that never leaves the laptop.

Everything this writes lives under `recordings/inventory_eval/` (git-ignored
with the rest of `recordings/`).
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import os
import random
import re
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Optional

# A worktree has no walks of its own: point these at the main checkout's.
ROOT = Path(os.environ.get("RECORDINGS_DIR", "recordings"))
OUT = ROOT / "inventory_eval"
WEIGHTS = os.environ.get("INVENTORY_PF_WEIGHTS", "yoloe-11s-seg-pf.pt")
JUDGE_MODEL = "anthropic.claude-opus-5-5"
# Opus 5.5 answers on Bedrock from us-east-1 (a real call, 2026-10-09);
# us-east-2 returned 404 for the same id.
JUDGE_REGION = "us-east-1"
# Dollars per million tokens. The first-party list price, used as the
# estimate: Bedrock bills separately and its rate is not read here, so the
# running total is an ESTIMATE and the budget is enforced against it.
PRICE_IN, PRICE_OUT = 4.00, 20.00
DETECT_CONF = 0.05          # kept low so the threshold can be chosen later
JUDGE_FLOOR = 0.10          # boxes below this are not sent to the judge
MAX_BOXES = 20              # the most confident object boxes per frame
SAMPLE_N = 150
SAMPLE_SEED = 3460
TARGET_JUDGE_PRECISION = 0.75


# ---------------------------------------------------------------- frames

def walks(root: Path = ROOT) -> list:
    return sorted(p for p in root.iterdir()
                  if p.is_dir() and p.name != OUT.name and any(p.glob("frame-*.jpg")))


def frames(walk: Path) -> list:
    return sorted(walk.glob("frame-*.jpg"))


def frame_id(path: Path) -> str:
    return f"{path.parent.name}/{path.name}"


# ---------------------------------------------------------------- the judge

def _client():
    from anthropic import AnthropicBedrockMantle
    return AnthropicBedrockMantle(aws_region=JUDGE_REGION)


class Spend:
    """Running cost, from each response's own token counts."""

    def __init__(self, budget: float):
        self.budget, self.dollars, self.calls = budget, 0.0, 0
        self.tokens_in = self.tokens_out = 0
        self._lock = threading.Lock()

    def add(self, usage) -> None:
        with self._lock:
            self.calls += 1
            self.tokens_in += usage.input_tokens
            self.tokens_out += usage.output_tokens
            self.dollars += (usage.input_tokens * PRICE_IN
                             + usage.output_tokens * PRICE_OUT) / 1e6

    def over(self) -> bool:
        return self.dollars >= self.budget

    def line(self) -> str:
        return (f"{self.calls} calls, {self.tokens_in} in / {self.tokens_out} out, "
                f"~${self.dollars:.2f} (estimate at ${PRICE_IN}/${PRICE_OUT} per M)")


def _text(response) -> str:
    if response.stop_reason == "refusal":
        raise RuntimeError("the judge declined")
    return "".join(b.text for b in response.content if b.type == "text")


def _json_from(text: str):
    m = re.search(r"(\{.*\}|\[.*\])", text, re.S)
    if not m:
        raise ValueError(f"no JSON in answer: {text[:200]!r}")
    return json.loads(m.group(1))


VOCAB_PROMPT = """Below is a numbered list of labels from an object detector.
For a home robot's inventory, a label counts as an OBJECT only if it names
a physical thing that sits somewhere in a home and could be pointed at:
furniture, appliances, containers, clothing, tools, toys, devices, plants,
food items, decorations. NOT objects: rooms and places ("kitchen",
"playroom"), building surfaces and structure ("wall", "floor", "ceiling",
"molding"), materials, patterns, colours, activities, events, ideas,
styles, photo genres, people, body parts and animals.

Answer with only a JSON array of the NUMBERS of the labels that are objects.

"""


def cmd_vocab(args) -> None:
    from ultralytics import YOLOE
    names = list(YOLOE(WEIGHTS).names.values())
    client, spend = _client(), Spend(args.budget)
    chunk = 600
    keep = set()
    for start in range(0, len(names), chunk):
        part = names[start:start + chunk]
        listing = "\n".join(f"{start + i}: {n}" for i, n in enumerate(part))
        r = client.messages.create(model=JUDGE_MODEL, max_tokens=16000,
                                   output_config={"effort": "low"},
                                   messages=[{"role": "user", "content": VOCAB_PROMPT + listing}])
        spend.add(r.usage)
        got = [int(i) for i in _json_from(_text(r)) if start <= int(i) < start + len(part)]
        keep.update(got)
        print(f"{start + len(part)}/{len(names)}: {len(got)} objects  {spend.line()}",
              file=sys.stderr)
    OUT.mkdir(parents=True, exist_ok=True)
    vocab = {n: (i in keep) for i, n in enumerate(names)}
    (OUT / "vocab.json").write_text(json.dumps(vocab, indent=0, sort_keys=True))
    print(f"{sum(vocab.values())} of {len(vocab)} names are objects -> {OUT / 'vocab.json'}")


def load_vocab() -> dict:
    return json.loads((OUT / "vocab.json").read_text())


# ---------------------------------------------------------------- detect

def cmd_detect(args) -> None:
    from ultralytics import YOLOE
    model = YOLOE(WEIGHTS)
    names = model.names
    OUT.mkdir(parents=True, exist_ok=True)
    out_path = OUT / "detections.json"
    done = json.loads(out_path.read_text()) if out_path.exists() else {}
    t0, n = time.perf_counter(), 0
    for walk in walks():
        for f in frames(walk):
            fid = frame_id(f)
            if fid in done:
                continue
            r = model.predict(str(f), conf=DETECT_CONF, verbose=False)[0]
            done[fid] = {"w": int(r.orig_shape[1]), "h": int(r.orig_shape[0]), "boxes": [
                {"label": names[int(b.cls[0])], "conf": round(float(b.conf[0]), 4),
                 "xyxy": [round(float(v), 1) for v in b.xyxy[0].tolist()]}
                for b in r.boxes]}
            n += 1
            if n % 200 == 0:
                out_path.write_text(json.dumps(done))
                print(f"{n} frames, {(time.perf_counter() - t0) / n * 1000:.0f} ms each",
                      file=sys.stderr)
    out_path.write_text(json.dumps(done))
    print(f"{len(done)} frames -> {out_path}")


def object_boxes(det: dict, vocab: dict, floor: float = JUDGE_FLOOR,
                 cap: int = MAX_BOXES) -> list:
    """The boxes the judge sees: object-vocabulary labels at or above
    `floor`, most confident first, at most `cap` -- numbered from 1."""
    boxes = [b for b in det["boxes"] if vocab.get(b["label"]) and b["conf"] >= floor]
    boxes.sort(key=lambda b: -b["conf"])
    return [{**b, "n": i + 1} for i, b in enumerate(boxes[:cap])]


# ---------------------------------------------------------------- judge

JUDGE_PROMPT = """This frame is from a camera on a small floor robot. A detector
drew the numbered boxes and named each one:

{listing}

For EACH number, say whether the name is right for what is inside that box:
"correct" if the box mostly covers a thing that the name fairly describes
(a reasonable synonym or a slightly broader name is fine), otherwise
"wrong". Then list up to 10 clearly visible physical objects (furniture,
appliances, containers, clothing, devices, decorations...) that NO box
covers -- not walls, floors or other surfaces.

Answer with only JSON:
{{"boxes": {{"1": "correct", "2": "wrong", ...}}, "missed": ["name", ...]}}"""


def draw(path: Path, boxes: list) -> bytes:
    from PIL import Image, ImageDraw
    img = Image.open(path).convert("RGB")
    d = ImageDraw.Draw(img)
    for b in boxes:
        x1, y1, x2, y2 = b["xyxy"]
        d.rectangle([x1, y1, x2, y2], outline=(255, 40, 40), width=3)
        d.rectangle([x1, y1, x1 + 22, y1 + 16], fill=(255, 40, 40))
        d.text((x1 + 4, y1 + 2), str(b["n"]), fill=(255, 255, 255))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=88)
    return buf.getvalue()


def judge_one(client, path: Path, boxes: list, spend: Spend) -> dict:
    listing = "\n".join(f"{b['n']}: {b['label']}" for b in boxes)
    img = base64.standard_b64encode(draw(path, boxes)).decode()
    r = client.messages.create(
        model=JUDGE_MODEL, max_tokens=4000, output_config={"effort": "low"},
        messages=[{"role": "user", "content": [
            {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                         "data": img}},
            {"type": "text", "text": JUDGE_PROMPT.format(listing=listing)}]}])
    spend.add(r.usage)
    got = _json_from(_text(r))
    verdicts = {int(k): v for k, v in (got.get("boxes") or {}).items()}
    return {"verdicts": {str(b["n"]): verdicts.get(b["n"], "unanswered") for b in boxes},
            "missed": [str(m) for m in (got.get("missed") or [])][:10]}


def cmd_judge(args) -> None:
    vocab = load_vocab()
    dets = json.loads((OUT / "detections.json").read_text())
    out_path = OUT / "judge.json"
    done = json.loads(out_path.read_text()) if out_path.exists() else {}
    todo = [fid for fid in sorted(dets) if fid not in done
            and object_boxes(dets[fid], vocab)]
    if args.limit:
        todo = todo[:args.limit]
    client, spend = _client(), Spend(args.budget)
    lock = threading.Lock()

    def work(fid):
        if spend.over():
            return fid, None
        boxes = object_boxes(dets[fid], vocab)
        return fid, judge_one(client, ROOT / fid, boxes, spend)

    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(work, fid) for fid in todo]
        for k, fut in enumerate(as_completed(futures), 1):
            try:
                fid, res = fut.result()
            except Exception as exc:  # noqa: BLE001 -- one bad frame is not a bad run
                print(f"frame failed: {exc}", file=sys.stderr)
                continue
            if res is None:
                continue
            with lock:
                done[fid] = res
                if k % 50 == 0:
                    out_path.write_text(json.dumps(done))
                    print(f"{len(done)} judged  {spend.line()}", file=sys.stderr)
    out_path.write_text(json.dumps(done))
    stopped = " -- STOPPED at the budget" if spend.over() else ""
    print(f"{len(done)} frames judged -> {out_path}; this run: {spend.line()}{stopped}")


# ---------------------------------------------------------------- sample

def sample_ids(judged: dict, n: int = SAMPLE_N, seed: int = SAMPLE_SEED) -> list:
    """Seeded and spread across walks: frames are drawn walk by walk in
    turn, so one long walk cannot fill the sample."""
    rng = random.Random(seed)
    by_walk: dict = {}
    for fid in sorted(judged):
        by_walk.setdefault(fid.split("/")[0], []).append(fid)
    for ids in by_walk.values():
        rng.shuffle(ids)
    out, order = [], sorted(by_walk)
    while len(out) < n and any(by_walk.values()):
        for w in order:
            if by_walk[w] and len(out) < n:
                out.append(by_walk[w].pop())
    return sorted(out)


def cmd_sample(args) -> None:
    vocab = load_vocab()
    dets = json.loads((OUT / "detections.json").read_text())
    judged = json.loads((OUT / "judge.json").read_text())
    ids = sample_ids(judged)
    items = []
    for fid in ids:
        boxes = object_boxes(dets[fid], vocab)
        items.append({"id": fid, "img": base64.standard_b64encode(
            draw(ROOT / fid, boxes)).decode(),
            "boxes": [{"n": b["n"], "label": b["label"],
                       "judge": judged[fid]["verdicts"].get(str(b["n"]))} for b in boxes]})
    (OUT / "sample.json").write_text(json.dumps(ids))
    page = OUT / "adjudicate.html"
    page.write_text(_PAGE.replace("__DATA__", json.dumps(items)))
    print(f"{len(ids)} frames -> {page}")


_PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Inventory labels</title><style>
body{font:15px system-ui;margin:0 auto;max-width:900px;padding:16px;background:#fafafa;color:#111}
.f{background:#fff;border:1px solid #ddd;border-radius:8px;margin:16px 0;padding:12px}
img{width:100%;border-radius:4px}.row{display:flex;gap:8px;align-items:center;margin:4px 0}
.row span{flex:1}button{padding:6px 12px;border:1px solid #888;border-radius:6px;background:#fff}
button.on{background:#111;color:#fff}#bar{position:sticky;top:0;background:#fafafa;padding:8px 0}
</style></head><body><div id="bar"><b>Is each label right for what is in its box?</b>
<span id="count"></span> <button onclick="save()">Download answers</button></div>
<div id="list"></div><script>
const items=__DATA__;let ans={};try{ans=JSON.parse(localStorage.getItem('inv-ans')||'{}')}catch(e){}
function key(f,n){return f+'#'+n}
function set(f,n,v){ans[key(f,n)]=v;try{localStorage.setItem('inv-ans',JSON.stringify(ans))}catch(e){};draw()}
function draw(){const L=document.getElementById('list');L.innerHTML='';let done=0,total=0;
for(const it of items){const d=document.createElement('div');d.className='f';
d.innerHTML='<div>'+it.id+'</div><img src="data:image/jpeg;base64,'+it.img+'">';
for(const b of it.boxes){total++;const v=ans[key(it.id,b.n)];if(v)done++;
const r=document.createElement('div');r.className='row';
r.innerHTML='<span>'+b.n+': <b>'+b.label+'</b></span>';
for(const o of ['right','wrong']){const x=document.createElement('button');x.textContent=o;
if(v===o)x.className='on';x.onclick=()=>set(it.id,b.n,o);r.appendChild(x)}d.appendChild(r)}
L.appendChild(d)}document.getElementById('count').textContent=done+' / '+total+' answered'}
function save(){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob(
[JSON.stringify(ans)],{type:'application/json'}));a.download='inventory_human.json';a.click()}
draw()</script></body></html>"""


# ---------------------------------------------------------------- score

def _correct(verdict: Optional[str]) -> Optional[bool]:
    if verdict in ("correct", "right"):
        return True
    if verdict == "wrong":
        return False
    return None


def precision_at(rows: list, conf: float) -> tuple:
    """`rows` = [(conf, ok)]; precision and count at or above `conf`."""
    kept = [ok for c, ok in rows if c >= conf and ok is not None]
    return (sum(kept) / len(kept) if kept else float("nan")), len(kept)


def choose_threshold(rows: list, target: float = TARGET_JUDGE_PRECISION) -> Optional[float]:
    """The lowest confidence at which precision reaches `target`."""
    for c in sorted({round(c, 2) for c, _ in rows}):
        p, n = precision_at(rows, c)
        if n and p >= target:
            return c
    return None


def cmd_score(args) -> None:
    vocab = load_vocab()
    dets = json.loads((OUT / "detections.json").read_text())
    judged = json.loads((OUT / "judge.json").read_text())
    sample = set(json.loads((OUT / "sample.json").read_text())) if (OUT / "sample.json").exists() else set()
    tune = []
    for fid, j in judged.items():
        if fid in sample:
            continue
        for b in object_boxes(dets[fid], vocab):
            tune.append((b["conf"], _correct(j["verdicts"].get(str(b["n"])))))
    c = choose_threshold(tune)
    p_all, n_all = precision_at(tune, JUDGE_FLOOR)
    p_c, n_c = precision_at(tune, c) if c is not None else (float("nan"), 0)
    report = {"frames_judged": len(judged), "tuning_boxes": len(tune),
              "judge_precision_at_floor": round(p_all, 3),
              "threshold": c, "judge_precision_at_threshold": round(p_c, 3),
              "boxes_at_threshold": n_c}
    if args.human:
        human = json.loads(Path(args.human).read_text())
        rows, agree, both = [], 0, 0
        for fid in sorted(sample):
            for b in object_boxes(dets[fid], vocab):
                h = _correct(human.get(f"{fid}#{b['n']}"))
                jv = _correct(judged[fid]["verdicts"].get(str(b["n"])))
                if h is None:
                    continue
                rows.append((b["conf"], h))
                if jv is not None:
                    both += 1
                    agree += (h == jv)
        p_h, n_h = precision_at(rows, c) if c is not None else (float("nan"), 0)
        report.update({"human_boxes": len(rows),
                       "criterion_6_precision": round(p_h, 3), "criterion_6_boxes": n_h,
                       "criterion_6_met": bool(p_h >= 0.70),
                       "judge_agrees_with_user": round(agree / both, 3) if both else None})
    print(json.dumps(report, indent=1))


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="python -m control.inventory_label")
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("vocab")
    v.add_argument("--budget", type=float, default=5.0)
    sub.add_parser("detect")
    j = sub.add_parser("judge")
    j.add_argument("--limit", type=int, default=0)
    j.add_argument("--budget", type=float, default=80.0)
    j.add_argument("--workers", type=int, default=6)
    sub.add_parser("sample")
    s = sub.add_parser("score")
    s.add_argument("--human")
    args = ap.parse_args(argv)
    {"vocab": cmd_vocab, "detect": cmd_detect, "judge": cmd_judge,
     "sample": cmd_sample, "score": cmd_score}[args.cmd](args)


if __name__ == "__main__":
    main()
