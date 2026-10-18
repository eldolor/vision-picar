"""
evaluations/tiered-380/Q/run_identity.py

3.80.Q: a Bedrock model's `target_visible` on 3.48's 299 pinned frames,
through the deployed /navigate (PAID: run only with the user's yes), scored
beside Opus 4.5's recorded answers by control.identity_eval's own labels and
teal-bin subset. A failed call is not written, so a re-run retries it; the cap
charges at least --floor a call (a model may leave the image out of its
reported usage).

    .venv/bin/python evaluations/tiered-380/Q/run_identity.py run \
        --model us.openai.gpt-6-astra --out astra.jsonl \
        --price-in 5e-6 --price-out 25e-6 --floor 0.006 --cap 4 --max-calls 320
    .venv/bin/python evaluations/tiered-380/Q/run_identity.py score astra.jsonl
"""

import argparse
import base64
import json
import sys
import time
from pathlib import Path

import httpx

HERE = Path(__file__).parent
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "evaluations" / "tiered-380" / "E"))
from control.identity_eval import cloud_records, confabulations, current_labels  # noqa: E402
from control.perception_eval import load_corpus                                # noqa: E402
from pilot import DEFAULT_VISION_URL, secrets                                   # noqa: E402

RECORDINGS = Path("/Users/anshugaind/vision-picar/recordings")
WALKS = ["blue-bottle-20260907-142454", "blue-bottle-20260907-185007",
         "blue-shoes-20260907-152528", "red-backpack-20260907-144856"]   # 3.48's pin


def walks():
    return load_corpus(RECORDINGS, only=WALKS)


def target(w):
    return json.loads((w.path / "meta.json").read_text())["target_object"]


def run(args):
    out = HERE / args.out
    failed = out.with_suffix(".failed.jsonl")   # failed calls: never retried from, always charged
    done, spent, calls = set(), 0.0, 0
    if out.exists():
        for line in out.read_text().splitlines():
            r = json.loads(line)
            if r.get("model_id") != args.model:
                sys.exit(f"{out} holds {r.get('model_id')!r}, not {args.model!r}: use another --out")
            done.add(r["key"])
            spent += max(r["cost_usd"], args.floor)
            calls += 1
    if failed.exists():
        for line in failed.read_text().splitlines():
            spent += max(json.loads(line)["cost_usd"], args.floor)
            calls += 1
    s = secrets()
    url = (s.get("VISION_URL") or DEFAULT_VISION_URL).rstrip("/")
    with httpx.Client(timeout=60) as c, out.open("a") as f:
        for w in walks():
            obj = target(w)
            for p, vis, _ in w.frames:
                key = f"{w.name}/{p.name}"
                if key in done:
                    continue
                if spent >= args.cap or calls + 1 > args.max_calls:
                    print(f"STOP: cap ${args.cap} or {args.max_calls} calls "
                          f"(${spent:.2f}, {calls} calls)")
                    return
                t0 = time.monotonic()
                try:
                    r = c.post(f"{url}/navigate", headers={"x-app-secret": s.get("VISION_SECRET", "")},
                               json={"image_base64": base64.b64encode(p.read_bytes()).decode(),
                                     "media_type": "image/jpeg", "target_object": obj,
                                     "searched_rooms": [], "model_id": args.model})
                    status, got = r.status_code, (r.json() if r.status_code == 200 else {})
                except httpx.HTTPError as e:
                    status, got = 0, {"error": f"{type(e).__name__}: {e}"}
                usage = got.get("usage") or {}
                cost = ((usage.get("input_tokens") or 0) * args.price_in
                        + (usage.get("output_tokens") or 0) * args.price_out)
                spent += max(cost, args.floor)
                calls += 1
                tv = got.get("target_visible")
                if status != 200 or not isinstance(tv, bool):
                    err = got.get("error") or str(got)[:200]
                    print("FAILED, not scored (a re-run retries it):", key, status, err, flush=True)
                    with failed.open("a") as g:
                        g.write(json.dumps({"key": key, "status": status, "error": err,
                                            "cost_usd": cost}) + "\n")
                    continue
                row = {"key": key, "target": obj, "visible": vis, "target_visible": tv,
                       "model_id": got.get("model_id"), "usage": usage, "cost_usd": cost,
                       "s": round(time.monotonic() - t0, 2)}
                f.write(json.dumps(row) + "\n"); f.flush()
                print(key, vis, tv, f"${spent:.3f}", flush=True)
    print(f"spent ${spent:.3f} (at least ${args.floor} a call), {calls} calls")


def stats(answers, labels, teal):
    """answers: {key: bool}. Precision, recall and teal-bin YES, by today's labels."""
    tp = sum(1 for k, a in answers.items() if a and labels[k])
    fp = sum(1 for k, a in answers.items() if a and not labels[k])
    vis = sum(1 for k in answers if labels[k])
    return {"frames": len(answers), "yes": tp + fp, "tp": tp, "fp": fp, "visible": vis,
            "precision": round(tp / (tp + fp), 3) if tp + fp else None,
            "recall": round(tp / vis, 3) if vis else None,
            "teal_yes": sum(1 for k in teal if answers.get(k)), "teal_n": len(teal)}


def score(args):
    ws = walks()
    labels = current_labels(ws)
    cloud = cloud_records(ws)
    teal = confabulations(cloud)        # 3.48's teal-bin subset, keyed walk/frame
    opus = {f"{r.walk}/{r.frame}": r.score == 1.0 for r in cloud if r.score is not None}
    rows = [json.loads(l) for l in (HERE / args.file).read_text().splitlines() if l.strip()]
    model = {r["key"]: r["target_visible"] for r in rows}
    shared = set(model) & set(opus)
    missing = sorted(set(labels) - set(model))
    if missing:
        print(f"WARNING: {len(missing)} labelled frames have no answer from {args.file}; "
              "both rows are scored on the shared frames only")
        model = {k: v for k, v in model.items() if k in shared}
        opus = {k: v for k, v in opus.items() if k in shared}
    lat = sorted(r["s"] for r in rows)
    out = {"labelled": len(labels), "missing": missing, "opus_recorded": stats(opus, labels, teal),
           args.file: {**stats(model, labels, teal),
                       "models": sorted({str(r["model_id"]) for r in rows}),
                       "p50_s": lat[len(lat) // 2], "p90_s": lat[int(len(lat) * 0.9)],
                       "cost_reported_usd": round(sum(r["cost_usd"] for r in rows), 3)}}
    print(json.dumps(out, indent=1))
    (HERE / "scores.json").write_text(json.dumps(out, indent=1) + "\n")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--model", required=True)
    r.add_argument("--out", required=True)
    r.add_argument("--price-in", type=float, required=True)
    r.add_argument("--price-out", type=float, required=True)
    r.add_argument("--floor", type=float, required=True)
    r.add_argument("--cap", type=float, required=True)
    r.add_argument("--max-calls", type=int, required=True)
    s = sub.add_parser("score")
    s.add_argument("file")
    args = ap.parse_args()
    (run if args.cmd == "run" else score)(args)


if __name__ == "__main__":
    main()
