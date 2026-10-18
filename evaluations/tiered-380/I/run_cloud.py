"""
evaluations/tiered-380/I/run_cloud.py

3.80.I, run_opus.py's arm for any Bedrock model (PAID: run only with the
user's yes). The same frames (S1, S2, S3, each as is and mirrored), the same
/navigate prompt and the same scoring fields; only the model varies.

  --via route   through the deployed /navigate with `model_id` (the model must
                be in its allow-list: GET /navigate/models)
  --via direct  the same vision_core.describe_image_bytes_navigate() the
                Lambda runs, called from this laptop -- for a model the
                deployed allow-list does not carry yet (no deploy)

Prices are per token and must be given. The cap stops the run on the
larger of the reported cost and --floor a call: a model may leave the image
out of its reported usage (GPT-6 Astra reports ~144 input tokens), and a
call that times out may still be billed, so each is charged at least the
floor. --max-calls is a second, count-based stop. A pair with a failed call
is not written, so a re-run retries it.

    .venv/bin/python evaluations/tiered-380/I/run_cloud.py \
        --model us.openai.gpt-6-astra --via route --out astra.jsonl \
        --price-in 5e-6 --price-out 25e-6 --cap 5 [--limit 1]
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
from pilot import DEFAULT_VISION_URL, secrets                         # noqa: E402

THIRD = {"left": "LEFT", "center": "FORWARD", "right": "RIGHT"}


def via_route(c, url, secret, model):
    def call(jpeg, target):
        r = c.post(f"{url}/navigate", headers={"x-app-secret": secret},
                   json={"image_base64": b64(jpeg), "media_type": "image/jpeg",
                         "target_object": target, "searched_rooms": [],
                         "model_id": model})
        return r.status_code, (r.json() if r.status_code == 200 else {"error": r.text[:300]})
    return call


def via_direct(model):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "service" / "vision_analyze"))
    import vision_core                                                # noqa: E402

    def call(jpeg, target):
        try:
            return 200, vision_core.describe_image_bytes_navigate(
                jpeg, target, "image/jpeg", searched_rooms=[], model_id=model)
        except Exception as e:                                        # recorded, not hidden
            return 0, {"error": f"{type(e).__name__}: {e}"[:300]}
    return call


def ask(call, jpeg, target, price_in, price_out, floor):
    t0 = time.monotonic()
    status, got = call(jpeg, target)
    act = got.get("action")
    usage = got.get("usage") or {}
    return {"status": status, "action": act if act in ("FORWARD", "LEFT", "RIGHT") else
            (f"other:{act}" if act else None),
            "target_visible": got.get("target_visible"),
            "target_third": THIRD.get(str(got.get("target_direction")).lower()),
            "target_direction": got.get("target_direction"),
            "model_id": got.get("model_id"), "usage": usage, "error": got.get("error"),
            "cost_usd": ((usage.get("input_tokens") or 0) * price_in
                         + (usage.get("output_tokens") or 0) * price_out),
            "floor_usd": floor,
            "s": round(time.monotonic() - t0, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--via", choices=("route", "direct"), required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--price-in", type=float, required=True)
    ap.add_argument("--price-out", type=float, required=True)
    ap.add_argument("--cap", type=float, default=5.0)
    ap.add_argument("--floor", type=float, required=True,
                    help="USD a call counted against the cap at least (see above)")
    ap.add_argument("--max-calls", type=int, required=True)
    ap.add_argument("--limit", type=int, default=0, help="frames this run (0 = all)")
    args = ap.parse_args()
    out = HERE / args.out
    charge = lambda c: max(c["cost_usd"], args.floor)              # noqa: E731
    spent, calls, done = 0.0, 0, set()
    if out.exists():
        for line in out.read_text().splitlines():
            r = json.loads(line)
            done.add(r["key"])
            spent += charge(r) + charge(r["mirror"])
            calls += 2
    with httpx.Client(timeout=60) as c, out.open("a") as f:
        if args.via == "route":
            s = secrets()
            call = via_route(c, (s.get("VISION_URL") or DEFAULT_VISION_URL).rstrip("/"),
                             s.get("VISION_SECRET", ""), args.model)
        else:
            call = via_direct(args.model)
        n = 0
        for set_, key, path, target, truth in items():
            if key in done:
                continue
            if spent >= args.cap or calls + 2 > args.max_calls:
                print(f"STOP: cap ${args.cap} or {args.max_calls} calls reached "
                      f"(${spent:.2f}, {calls} calls)")
                break
            if args.limit and n >= args.limit:
                break
            jpeg = path.read_bytes()
            a = ask(call, jpeg, target, args.price_in, args.price_out, args.floor)
            m = ask(call, mirrored(jpeg), target, args.price_in, args.price_out, args.floor)
            spent += charge(a) + charge(m)
            calls += 2
            n += 1
            if a["status"] != 200 or m["status"] != 200:
                print("FAILED, not written (a re-run retries it):", key,
                      a["status"], a["error"], m["status"], m["error"], flush=True)
                continue
            row = {"set": set_, "key": key, "target": target, "truth": truth,
                   "via": args.via, **a, "mirror": m}
            f.write(json.dumps(row) + "\n"); f.flush()
            print(set_, key, truth, a["status"], a["action"], a["target_third"],
                  m["action"], a["usage"], a["error"] or "", f"${spent:.3f}", flush=True)
    print(f"spent ${spent:.3f} (at least ${args.floor} a call), {calls} calls")


if __name__ == "__main__":
    main()
