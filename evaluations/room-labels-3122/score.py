"""3.122: /room (Opus 5.5) on the recorded walks' room-labelled frames.

    python score.py plan                      # frames chosen, cost estimate; no calls
    AWS_PROFILE=mac-admin python score.py run --max-dollars 8
    python score.py report                    # writes run/report.json

The prompt, model, effort and reply parsing are the deployed vision
service's own (`service/vision_analyze/vision_core.py`
`describe_image_bytes_room`), called directly on Bedrock. Frames are read
from the recordings bucket into `run/frames/` (git-ignored: photos of the
user's house); replies are appended to `run/replies.jsonl`, one fsynced
line a call, so a re-run pays only for frames without a reply.
"""

import argparse
import json
import os
import random
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
RUN = HERE / "run"
FRAMES = RUN / "frames"
REPLIES = RUN / "replies.jsonl"
LABELS = [ROOT / "docs/evaluations/room-labels-3.51.json",
          ROOT / "docs/evaluations/room-labels-3.54.json"]
BUCKET = "vision-picar-recordings-303351622021-us-east-2"

# 3.70 S2b's measured cost basis ($4.40/M in, $22/M out; unconfirmed list
# price), and the most one call can cost (input ~1,360 tokens, cap 256 out).
PRICE_IN, PRICE_OUT = 4.40e-6, 22e-6
WORST_CALL = 1400 * PRICE_IN + 256 * PRICE_OUT

# Written before the run (3.122 amendment 1). An answer counts right when it
# is in its labelled room's set; anything else, `unclear` included, is wrong.
ACCEPT = {
    "home gym": {"gym", "home gym", "exercise room", "workout room",
                 "fitness room", "weight room"},
    "basement": {"basement", "family room", "rec room", "recreation room",
                 "game room", "den", "media room", "lounge", "playroom",
                 "basement family room", "basement living room"},
    "living room": {"living room", "family room", "lounge", "den",
                    "sitting room"},
    "hallway": {"hallway", "foyer", "entryway", "entry", "hall", "stairwell",
                "stairway", "staircase", "landing", "stair hall"},
    "home office": {"office", "home office", "study"},
}
# Reported beside the strict score, never judged: a basement's living area
# called "living room".
LENIENT_EXTRA = {"basement": {"living room"}}
JUDGED_MIN = 30
DRAWS, SEED = 10_000, 3122


def labelled():
    """[(walk, frame, room)] for every decided frame, in walk then frame order."""
    rows = []
    for path in LABELS:
        for walk, frames in json.load(open(path))["walks"].items():
            for frame in sorted(frames):
                room = frames[frame]
                if room != "unsure":
                    rows.append((walk, frame, room))
    return rows


def chosen():
    """Every second decided frame of each walk (its 1st, 3rd, ...); every
    hallway (stair hall) frame."""
    out, seen = [], {}
    for walk, frame, room in labelled():
        i = seen.get(walk, 0)
        seen[walk] = i + 1
        if i % 2 == 0 or room == "hallway":
            out.append((walk, frame, room))
    return out


def load_replies():
    got = {}
    if REPLIES.exists():
        for line in REPLIES.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                got[(r["walk"], r["frame"])] = r
    return got


def cost(usage):
    return ((usage.get("input_tokens") or 0) * PRICE_IN
            + (usage.get("output_tokens") or 0) * PRICE_OUT)


def frame_bytes(s3, walk, frame):
    path = FRAMES / walk / frame
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        body = s3.get_object(Bucket=BUCKET,
                             Key=f"recordings/{walk}/{frame}")["Body"].read()
        tmp = path.with_suffix(".part")
        tmp.write_bytes(body)
        tmp.replace(path)
    return path.read_bytes()


def cmd_plan(_):
    rows, todo = chosen(), None
    got = load_replies()
    todo = [r for r in rows if (r[0], r[1]) not in got]
    by_room = {}
    for _, _, room in rows:
        by_room[room] = by_room.get(room, 0) + 1
    print(json.dumps({"decided": len(labelled()), "chosen": len(rows),
                      "by_room": by_room, "walks": len({r[0] for r in rows}),
                      "already_answered": len(rows) - len(todo),
                      "to_call": len(todo),
                      "estimate_usd": [round(len(todo) * 0.0061, 2),
                                       round(len(todo) * 0.0092, 2)],
                      "worst_case_usd": round(len(todo) * WORST_CALL, 2)},
                     indent=1))


def cmd_run(args):
    import boto3
    sys.path.insert(0, str(ROOT / "service/vision_analyze"))
    import vision_core as V

    s3 = boto3.client("s3")
    got = load_replies()
    spent = sum(cost(r.get("usage") or {}) for r in got.values())
    todo = [r for r in chosen() if (r[0], r[1]) not in got]
    print(f"{len(todo)} to call; ${spent:.3f} spent before this run", flush=True)
    lock = threading.Lock()
    state = {"spent": spent, "reserved": 0.0, "errors": 0, "calls": 0,
             "stop": None}
    REPLIES.parent.mkdir(parents=True, exist_ok=True)
    out = open(REPLIES, "a")

    def one(row):
        walk, frame, room = row
        with lock:
            if state["stop"]:
                return
            # The budget holds with calls in flight: each reserves its worst
            # case before it is sent, and is settled at its real cost.
            if state["spent"] + state["reserved"] + WORST_CALL > args.max_dollars:
                state["stop"] = f"budget ${args.max_dollars} reached"
                return
            state["reserved"] += WORST_CALL
        try:
            reply = V.describe_image_bytes_room(frame_bytes(s3, walk, frame))
        except Exception as e:  # noqa: BLE001 -- recorded, never scored
            with lock:
                state["reserved"] -= WORST_CALL
                state["errors"] += 1
                print(f"error {walk}/{frame}: {type(e).__name__}: {e}",
                      flush=True)
                if state["errors"] >= 3:
                    state["stop"] = "3 errors"
            return
        line = {"walk": walk, "frame": frame, "label": room, **reply}
        with lock:
            state["reserved"] -= WORST_CALL
            state["spent"] += cost(reply.get("usage") or {})
            state["calls"] += 1
            out.write(json.dumps(line) + "\n")
            out.flush()
            os.fsync(out.fileno())
            if state["calls"] % 50 == 0:
                print(f"{state['calls']} calls, ${state['spent']:.3f}",
                      flush=True)

    try:
        with ThreadPoolExecutor(args.workers) as pool:
            list(pool.map(one, todo))
    finally:
        out.close()
    print(f"done: {state['calls']} calls, {state['errors']} errors, "
          f"${state['spent']:.3f} in all; stop: {state['stop']}", flush=True)


def right(label, answer, lenient=False):
    ok = ACCEPT[label] | (LENIENT_EXTRA.get(label, set()) if lenient else set())
    return answer in ok


def boot(rows, key):
    """Error and its 95% interval, resampling walks (frames in a walk are
    not independent)."""
    walks = sorted({r["walk"] for r in rows})
    per = {w: [0, 0] for w in walks}
    for r in rows:
        per[r["walk"]][0] += 0 if key(r) else 1
        per[r["walk"]][1] += 1
    wrong = sum(v[0] for v in per.values())
    n = sum(v[1] for v in per.values())
    rng = random.Random(SEED)
    stats = []
    for _ in range(DRAWS):
        pick = [per[rng.choice(walks)] for _ in walks]
        stats.append(sum(p[0] for p in pick) / sum(p[1] for p in pick))
    stats.sort()
    return {"n": n, "walks": len(walks), "error": round(wrong / n, 4),
            "ci95": [round(stats[int(0.025 * DRAWS)], 4),
                     round(stats[int(0.975 * DRAWS) - 1], 4)]}


def cmd_report(_):
    got = load_replies()
    rows = [got[(w, f)] for w, f, _ in chosen() if (w, f) in got]
    strict = lambda r: right(r["label"], r["room"])  # noqa: E731
    lenient = lambda r: right(r["label"], r["room"], True)  # noqa: E731
    rooms = sorted({r["label"] for r in rows})
    answers = {}
    for r in rows:
        a = answers.setdefault(r["label"], {})
        a[r["room"]] = a.get(r["room"], 0) + 1
    rep = {
        "calls": len(rows), "chosen": len(chosen()),
        "usd": round(sum(cost(r.get("usage") or {}) for r in rows), 3),
        "cut_replies": sum(1 for r in rows if r.get("_stop")),
        "frame_error": boot(rows, strict),
        "frame_error_lenient": boot(rows, lenient),
        "per_room": {room: {**boot([r for r in rows if r["label"] == room],
                                   strict),
                            "judged": sum(r["label"] == room for r in rows)
                            >= JUDGED_MIN}
                     for room in rooms},
        "answers": {k: dict(sorted(v.items(), key=lambda kv: -kv[1]))
                    for k, v in answers.items()},
    }
    e = rep["frame_error"]["error"]
    rep["decision"] = ("build the noise model" if e <= 0.05 else
                       "re-run 3.116.A in the sim at this rate" if e <= 0.15
                       else "wait for the Rover")
    (RUN / "report.json").write_text(json.dumps(rep, indent=1) + "\n")
    print(json.dumps(rep, indent=1))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("plan")
    r = sub.add_parser("run")
    r.add_argument("--max-dollars", type=float, required=True)
    r.add_argument("--workers", type=int, default=4)
    sub.add_parser("report")
    args = ap.parse_args()
    {"plan": cmd_plan, "run": cmd_run, "report": cmd_report}[args.cmd](args)


if __name__ == "__main__":
    main()
