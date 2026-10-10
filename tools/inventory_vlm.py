"""3.46 amendment 6: Qwen2.5-VL-3B as the inventory's namer.

    python -m tools.inventory_vlm name           # the VLM over the frames (~19 s each, resumable)
    python -m tools.inventory_vlm judge          # the same judge over its boxes (~$5)
    python -m tools.inventory_vlm score          # precision and right boxes a frame, tuning frames

3.48 found Qwen2.5-VL-3B the one small VLM accurate enough to confirm a
target locally; if a 4-bit build fits the Jetson, the same model could name
objects for the inventory. This measures the naming half at bf16 on the
Mac, against the reference `tools/inventory_label.py` built (same judge,
same prompt, same verdict rule). The frames are 300 tuning frames outside
the user's sample plus the user's 150, so the user's page can follow.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
import sys
import time

from tools.inventory_label import (OUT, ROOT, Spend, _client, _correct, judge_one,
                                   precision_at)

MODEL = "Qwen/Qwen2.5-VL-3B-Instruct"
MAX_PIXELS = 640 * 480
TUNING_FRAMES = 300
TUNING_SEED = 3466
PROMPT = (
    "Find EVERY separate object in this photo -- furniture, bags, shoes, boxes, devices, "
    "decorations, anything that sits somewhere -- and give each one a specific everyday name "
    "such as \"office chair\", \"backpack\", \"sneaker\", \"chest of drawers\", \"suitcase\", "
    "\"lamp\". Each object once. Skip walls, floors, ceilings and carpet. "
    "Output JSON: [{\"bbox_2d\": [x1, y1, x2, y2], \"label\": \"name\"}]")


def frame_set() -> list:
    """The user's 150 plus `TUNING_FRAMES` seeded frames from the rest."""
    judged = json.loads((OUT / "judge.json").read_text())
    sample = json.loads((OUT / "sample.json").read_text())
    rest = sorted(f for f in judged if f not in set(sample))
    return sorted(random.Random(TUNING_SEED).sample(rest, TUNING_FRAMES)) + sorted(sample)


def parse_boxes(text: str, scale_x: float, scale_y: float, order: str = "xyxy") -> list:
    """The model's JSON, in the RESIZED image's pixels, back to the frame's.
    Identical repeated boxes are dropped (the model sometimes repeats one).
    Anything unparseable is skipped, never guessed. `order` "yxyx" reads
    Gemma's [y1, x1, y2, x2] (amendment 9); either key, `bbox_2d` or Gemma's
    own `box_2d`, is read."""
    m = re.search(r"\[.*\]", text, re.S)
    try:
        items = json.loads(m.group(0)) if m else []
    except json.JSONDecodeError:
        # A list cut off by the token cap: keep every COMPLETE object in the
        # whole answer (the last "]" may belong to a box, not the list).
        items = [json.loads(o) for o in re.findall(r"\{[^{}]*\}", text) if _loads_ok(o)]
    out, seen = [], set()
    for it in items:
        if not isinstance(it, dict):
            continue
        box = it.get("bbox_2d") or it.get("box_2d")
        label = str(it.get("label", "")).strip().lower()
        if not label or not isinstance(box, list) or len(box) != 4:
            continue
        try:
            a, b, c, d = (float(v) for v in box)
        except (TypeError, ValueError):
            continue
        if not all(math.isfinite(v) for v in (a, b, c, d)):
            continue
        key = (label, tuple(box))
        if key in seen:
            continue
        seen.add(key)
        x1, y1, x2, y2 = (b, a, d, c) if order == "yxyx" else (a, b, c, d)
        out.append({"label": label, "xyxy": [round(x1 * scale_x, 1), round(y1 * scale_y, 1),
                                             round(x2 * scale_x, 1), round(y2 * scale_y, 1)]})
    return [{**b, "n": i + 1} for i, b in enumerate(out)]


def _loads_ok(s: str) -> bool:
    try:
        json.loads(s)
        return True
    except json.JSONDecodeError:
        return False


def _paths(tag: str):
    suffix = f"_{tag}" if tag else ""
    return OUT / f"vlm_boxes{suffix}.json", OUT / f"vlm_judge{suffix}.json"


def name_llama(fid: str, url: str) -> dict:
    """Amendment 7: a GGUF model behind llama.cpp's server. Qwen3.5 answers
    on a 0-1000 scale, so boxes are scaled by the frame's size / 1000."""
    import base64
    import urllib.request

    from PIL import Image
    path = ROOT / fid
    w, h = Image.open(path).size
    img = base64.b64encode(path.read_bytes()).decode()
    body = {"messages": [{"role": "user", "content": [
        {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + img}},
        {"type": "text", "text": PROMPT}]}],
        "temperature": 0, "max_tokens": 500, "repeat_penalty": 1.05,
        "chat_template_kwargs": {"enable_thinking": False}}
    req = urllib.request.Request(url + "/v1/chat/completions", json.dumps(body).encode(),
                                 {"Content-Type": "application/json"})
    text = json.load(urllib.request.urlopen(req, timeout=300))["choices"][0]["message"]["content"]
    return {"boxes": parse_boxes(text, w / 1000, h / 1000), "raw": text[:2000]}


class LiteRTNamer:
    """Amendment 9: Gemma 4 under Google's LiteRT-LM (`litert-lm-api`), run on
    the Jetson in its own venv. One engine, a fresh conversation a frame;
    greedy, 500 new tokens, repetition penalty 1.05, thinking off. Gemma
    answers [y1, x1, y2, x2] on 0-1000."""

    def __init__(self, model: str, device: str):
        import litert_lm
        self.lm = litert_lm
        backend = litert_lm.Backend.GPU() if device == "gpu" else litert_lm.Backend.CPU()
        self.engine = litert_lm.Engine(model, backend=backend, vision_backend=backend,
                                       max_num_images=1)

    def __call__(self, fid: str) -> dict:
        from PIL import Image
        lm, path = self.lm, ROOT / fid
        with Image.open(path) as im:
            w, h = im.size
        with self.engine.create_conversation(
                sampler_config=lm.SamplerConfig(top_k=1),
                thinking_config=lm.ThinkingConfig(enable_thinking=False),
                max_output_tokens=500) as conv:
            msg = conv.send_message(
                lm.Contents.of(lm.Content.ImageFile(absolute_path=str(path.resolve())), PROMPT),
                repetition_penalty_config=lm.RepetitionPenaltyConfig(repetition_penalty=1.05))
        text = message_text(msg)
        return {"boxes": parse_boxes(text, w / 1000, h / 1000, order="yxyx"), "raw": text[:2000]}


def message_text(msg) -> str:
    """The text parts of a LiteRT-LM reply, joined."""
    return "".join(getattr(c, "text", "") or "" for c in msg.contents)


def _rss_mb() -> tuple:
    """This process's (VmRSS, VmHWM) in MB, for amendment 9's memory record."""
    vals = {}
    if not os.path.exists("/proc/self/status"):  # not Linux: nothing to record
        return None, None
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith(("VmRSS:", "VmHWM:")):
                vals[line.split(":")[0]] = int(line.split()[1]) // 1024
    return vals.get("VmRSS"), vals.get("VmHWM")


def cmd_name(args) -> None:
    out_path, _ = _paths(args.tag)
    frames = json.loads(open(args.frames).read()) if args.frames else frame_set()
    if args.limit > 0:
        frames = frames[:args.limit]
    if args.backend == "litert":
        if not args.tag or not args.model:
            sys.exit("--backend litert needs --tag and --model (the default tag is amendment 6's file)")
        run = {"model": os.path.basename(args.model), "device": args.device}
        done = json.loads(out_path.read_text()) if out_path.exists() else {}
        other = {(r.get("model"), r.get("device")) for r in done.values()} - {(run["model"], run["device"])}
        if other:
            sys.exit(f"{out_path} holds another run's frames {sorted(map(str, other))}; use a new --tag")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        namer = LiteRTNamer(args.model, args.device)
        todo = [f for f in frames if f not in done]
        print(f"loaded {args.model} on {args.device}; rss/hwm MB {_rss_mb()}", file=sys.stderr)
        t0, errors = time.perf_counter(), 0
        for k, fid in enumerate(todo, 1):
            t = time.perf_counter()
            try:
                r = namer(fid)
            except Exception as exc:  # noqa: BLE001 -- not saved, so a resume retries it
                errors += 1
                print(f"{fid}: {exc}", file=sys.stderr)
                continue
            r["s"] = round(time.perf_counter() - t, 2)
            done[fid] = {**r, **run}
            print(f"{time.strftime('%T')} {fid} {r['s']} s {len(r['boxes'])} boxes "
                  f"rss/hwm MB {_rss_mb()}", file=sys.stderr, flush=True)
            if k % 10 == 0 or k == len(todo):
                out_path.write_text(json.dumps(done))
                rate = (time.perf_counter() - t0) / k
                print(f"{len(done)} frames, {rate:.1f} s each, "
                      f"~{rate * (len(todo) - k) / 60:.0f} min left", file=sys.stderr)
        out_path.write_text(json.dumps(done))
        print(f"{len(done)} frames -> {out_path}; {errors} failed calls (not saved)")
        return
    if args.backend == "llama":
        done = json.loads(out_path.read_text()) if out_path.exists() else {}
        todo = [f for f in frames if f not in done]
        t0 = time.perf_counter()
        for k, fid in enumerate(todo, 1):
            done[fid] = name_llama(fid, args.url)
            if k % 10 == 0 or k == len(todo):
                out_path.write_text(json.dumps(done))
                rate = (time.perf_counter() - t0) / k
                print(f"{len(done)} frames, {rate:.1f} s each, "
                      f"~{rate * (len(todo) - k) / 60:.0f} min left", file=sys.stderr)
        print(f"{len(done)} frames -> {out_path}")
        return
    import torch
    from PIL import Image
    from transformers import AutoModelForImageTextToText, AutoProcessor

    proc = AutoProcessor.from_pretrained(MODEL, max_pixels=MAX_PIXELS)
    model = AutoModelForImageTextToText.from_pretrained(
        MODEL, dtype=torch.bfloat16).to("mps" if torch.backends.mps.is_available() else "cpu").eval()
    done = json.loads(out_path.read_text()) if out_path.exists() else {}
    todo = [f for f in frames if f not in done]
    t0 = time.perf_counter()
    for k, fid in enumerate(todo, 1):
        img = Image.open(ROOT / fid).convert("RGB")
        msgs = [{"role": "user", "content": [{"type": "image", "image": img},
                                             {"type": "text", "text": PROMPT}]}]
        inp = proc.apply_chat_template(msgs, add_generation_prompt=True, tokenize=True,
                                       return_dict=True, return_tensors="pt").to(model.device)
        with torch.no_grad():
            gen = model.generate(**inp, max_new_tokens=500, do_sample=False,
                                 repetition_penalty=1.05)
        text = proc.batch_decode(gen[:, inp["input_ids"].shape[1]:], skip_special_tokens=True)[0]
        _, gh, gw = inp["image_grid_thw"][0].tolist()
        boxes = parse_boxes(text, img.width / (gw * 14), img.height / (gh * 14))
        done[fid] = {"boxes": boxes, "raw": text[:2000]}
        if k % 10 == 0 or k == len(todo):
            out_path.write_text(json.dumps(done))
            rate = (time.perf_counter() - t0) / k
            print(f"{len(done)} frames, {rate:.1f} s each, ~{rate * (len(todo) - k) / 60:.0f} min left",
                  file=sys.stderr)
    print(f"{len(done)} frames -> {out_path}")


def cmd_judge(args) -> None:
    boxes_path, out_path = _paths(args.tag)
    vlm = json.loads(boxes_path.read_text())
    done = json.loads(out_path.read_text()) if out_path.exists() else {}
    client, spend = _client(), Spend(args.budget)
    for fid in sorted(vlm):
        if fid in done or not vlm[fid]["boxes"] or spend.over():
            continue
        try:
            done[fid] = judge_one(client, ROOT / fid, vlm[fid]["boxes"], spend)
        except Exception as exc:  # noqa: BLE001 -- one bad frame is not a bad run
            print(f"{fid}: {exc}", file=sys.stderr)
        if len(done) % 25 == 0:
            out_path.write_text(json.dumps(done))
    out_path.write_text(json.dumps(done))
    print(f"{len(done)} frames judged; {spend.line()}")


def cmd_score(args) -> None:
    boxes_path, judge_path = _paths(args.tag)
    vlm = json.loads(boxes_path.read_text())
    judged = json.loads(judge_path.read_text())
    sample = set(json.loads((OUT / "sample.json").read_text()))
    tune = [f for f in vlm if f not in sample]
    rows, empty = [], 0
    for fid in tune:
        if not vlm[fid]["boxes"]:
            empty += 1
            continue
        for b in vlm[fid]["boxes"]:
            ok = _correct((judged.get(fid) or {}).get("verdicts", {}).get(str(b["n"])))
            if ok is not None:
                rows.append((1.0, ok))
    p, n = precision_at(rows, 0.0)
    right = sum(ok for _, ok in rows)
    print(json.dumps({"tuning_frames": len(tune), "frames_with_no_boxes": empty,
                      "judged_boxes": n, "judge_precision": round(p, 3),
                      "right_boxes_a_frame": round(right / len(tune), 2),
                      "earns_the_users_page": bool(p >= 0.75)}, indent=1))


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="python -m tools.inventory_vlm")
    sub = ap.add_subparsers(dest="cmd", required=True)
    n = sub.add_parser("name")
    n.add_argument("--backend", choices=("transformers", "llama", "litert"), default="transformers")
    n.add_argument("--url", default="http://localhost:9467")
    n.add_argument("--model", help="litert: the .litertlm file")
    n.add_argument("--device", choices=("gpu", "cpu"), default="gpu", help="litert backend")
    n.add_argument("--frames", help="a JSON list of frame ids instead of frame_set() "
                                    "(the Jetson has no judge.json)")
    n.add_argument("--limit", type=int, default=0, help="first N frames only (feasibility)")
    j = sub.add_parser("judge")
    j.add_argument("--budget", type=float, default=15.0)
    sc = sub.add_parser("score")
    for p in (n, j, sc):
        p.add_argument("--tag", default="", help="'' = Qwen2.5-VL-3B (amendment 6); "
                                                 "'qwen35' = Qwen3.5-4B IQ3_XXS (amendment 7); "
                                                 "'gemma4e2b' / 'gemma4e4b' = LiteRT-LM (amendment 9)")
    args = ap.parse_args(argv)
    {"name": cmd_name, "judge": cmd_judge, "score": cmd_score}[args.cmd](args)


if __name__ == "__main__":
    main()
