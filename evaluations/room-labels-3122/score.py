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
ERRORS = RUN / "errors.jsonl"
LABELS = [ROOT / "docs/evaluations/room-labels-3.51.json",
          ROOT / "docs/evaluations/room-labels-3.54.json"]
BUCKET = "vision-picar-recordings-303351622021-us-east-2"

# 3.70 S2b's measured cost basis ($4.40/M in, $22/M out; unconfirmed list
# price). S2b's calls took ~1,360 tokens in and 6-145 out; a call's worst
# case allows 2,000 in (a larger image) and the 256-token cap, and grows
# with the largest input seen.
PRICE_IN, PRICE_OUT = 4.40e-6, 22e-6
TYPICAL_IN, TYPICAL_OUT = 1360, (6, 145)
WORST_IN, CAP_OUT = 2000, 256

# Written before the run (3.122 amendment 2): 3.54's table (3.51's plus
# gym and basement), and the names below it. An answer counts right when it
# maps to its labelled room; anything else, `unclear` included, is wrong.
sys.path.insert(0, str(ROOT))
from tools.clip_rooms_fewshot import CLOUD_SYNONYMS  # noqa: E402

SYNONYMS = {
    **CLOUD_SYNONYMS,
    "fitness room": "home gym", "weight room": "home gym",
    "game room": "basement", "media room": "basement",
    "playroom": "basement", "basement family room": "basement",
    "basement living room": "basement",
    "stairwell": "hallway", "stairway": "hallway", "staircase": "hallway",
    "landing": "hallway", "stair hall": "hallway",
}
# Reported beside the strict score, never judged: a basement's living area
# called a living room (the strict table maps these to living room only).
LENIENT_EXTRA = {"basement": {"living room", "family room", "den", "lounge"}}
JUDGED_MIN, JUDGED_WALKS = 30, 2
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
    """Every second decided frame of each walk (its 1st, 3rd, ...)."""
    out, seen = [], {}
    for walk, frame, room in labelled():
        i = seen.get(walk, 0)
        seen[walk] = i + 1
        if i % 2 == 0:
            out.append((walk, frame, room))
    return out


def load_jsonl(path):
    """Every whole line; a torn one (a write cut by a crash) is skipped, so
    its frame is called again."""
    rows = []
    if path.exists():
        for line in path.read_text().splitlines():
            if line.strip():
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    print(f"skipping a torn line in {path.name}: {line[:80]!r}",
                          file=sys.stderr)
    return rows


def load_replies():
    return {(r["walk"], r["frame"]): r for r in load_jsonl(REPLIES)}


def worst_call(max_in=0):
    return max(WORST_IN, int(max_in * 1.2)) * PRICE_IN + CAP_OUT * PRICE_OUT


def open_append(path):
    """Append mode, starting on a fresh line if a crash left one torn."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size:
        with open(path, "rb") as f:
            f.seek(-1, os.SEEK_END)
            torn = f.read(1) != b"\n"
    else:
        torn = False
    out = open(path, "a")
    if torn:
        out.write("\n")
    return out


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
    rows = chosen()
    got = load_replies()
    todo = [r for r in rows if (r[0], r[1]) not in got]
    by_room = {}
    for _, _, room in rows:
        by_room[room] = by_room.get(room, 0) + 1
    print(json.dumps({"decided": len(labelled()), "chosen": len(rows),
                      "by_room": by_room, "walks": len({r[0] for r in rows}),
                      "already_answered": len(rows) - len(todo),
                      "to_call": len(todo),
                      "estimate_usd": [round(len(todo) * cost(
                          {"input_tokens": TYPICAL_IN, "output_tokens": o}), 2)
                          for o in TYPICAL_OUT],
                      "worst_case_usd": round(len(todo) * worst_call(), 2)},
                     indent=1))


def cmd_run(args):
    import boto3
    sys.path.insert(0, str(ROOT / "service/vision_analyze"))
    import vision_core as V

    s3 = boto3.client("s3")
    got = load_replies()
    errors = load_jsonl(ERRORS)
    # A call that failed may still have been billed (a read timeout on a
    # call Bedrock ran): each counts at its worst case, in this run and
    # every later one.
    spent = (sum(cost(r.get("usage") or {}) for r in got.values())
             + sum(e["charged"] for e in errors))
    max_in = max([(r.get("usage") or {}).get("input_tokens") or 0
                  for r in got.values()] or [0])
    todo = [r for r in chosen() if (r[0], r[1]) not in got]
    print(f"{len(todo)} to call; ${spent:.3f} counted before this run "
          f"({len(errors)} earlier errors)", flush=True)
    lock = threading.Lock()
    state = {"spent": spent, "reserved": 0.0, "errors": 0, "calls": 0,
             "stop": None, "max_in": max_in, "closed": False}
    out = open_append(REPLIES)
    err = open_append(ERRORS)

    def write(f, line):
        # Under the lock. After the files close (a second Ctrl-C while the
        # calls in flight finish) a paid reply goes to stderr, not lost.
        if state["closed"]:
            print(f"unrecorded: {json.dumps(line)}", file=sys.stderr,
                  flush=True)
            return
        f.write(json.dumps(line) + "\n")
        f.flush()
        os.fsync(f.fileno())

    def one(row):
        walk, frame, room = row
        with lock:
            if state["stop"]:
                return
            # The budget holds with calls in flight: each reserves its worst
            # case before it is sent, and is settled at its real cost.
            worst = worst_call(state["max_in"])
            if state["spent"] + state["reserved"] + worst > args.max_dollars:
                state["stop"] = f"budget ${args.max_dollars} reached"
                return
            state["reserved"] += worst
        try:
            reply = V.describe_image_bytes_room(frame_bytes(s3, walk, frame))
        except Exception as e:  # noqa: BLE001 -- recorded, never scored
            with lock:
                state["reserved"] -= worst
                state["spent"] += worst
                state["errors"] += 1
                write(err, {"walk": walk, "frame": frame, "charged": worst,
                            "error": f"{type(e).__name__}: {e}"[:300]})
                print(f"error {walk}/{frame}: {type(e).__name__}: {e}",
                      flush=True)
                if state["errors"] >= 3:
                    state["stop"] = "3 errors"
            return
        line = {"walk": walk, "frame": frame, "label": room, **reply}
        usage = reply.get("usage") or {}
        with lock:
            state["reserved"] -= worst
            state["spent"] += cost(usage)
            state["max_in"] = max(state["max_in"],
                                  usage.get("input_tokens") or 0)
            state["calls"] += 1
            write(out, line)
            if state["calls"] % 50 == 0:
                print(f"{state['calls']} calls, ${state['spent']:.3f}",
                      flush=True)

    try:
        with ThreadPoolExecutor(args.workers) as pool:
            list(pool.map(one, todo))
    finally:
        with lock:
            state["closed"] = True
            out.close()
            err.close()
        print(f"done: {state['calls']} calls, {state['errors']} errors, "
              f"${state['spent']:.3f} counted in all; stop: {state['stop']}",
              flush=True)


def right(label, answer, lenient=False):
    if SYNONYMS.get(answer) == label:
        return True
    return lenient and answer in LENIENT_EXTRA.get(label, set())


def boot(rows, key):
    """Error and its 95% interval, resampling walks (frames in a walk are
    not independent)."""
    walks = sorted({r["walk"] for r in rows})
    if not rows:
        return {"n": 0, "walks": 0, "error": None, "ci95": None}
    per = {w: [0, 0] for w in walks}
    for r in rows:
        per[r["walk"]][0] += 0 if key(r) else 1
        per[r["walk"]][1] += 1
    wrong = sum(v[0] for v in per.values())
    n = sum(v[1] for v in per.values())
    rng = random.Random(SEED)
    if len(walks) < JUDGED_WALKS:
        # One walk: resampling walks cannot give an interval.
        return {"n": n, "walks": len(walks), "error": round(wrong / n, 4),
                "ci95": None}
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
        "errors": len(load_jsonl(ERRORS)),
        "frame_error": boot(rows, strict),
        "frame_error_lenient": boot(rows, lenient),
        "per_room": {room: {**boot([r for r in rows if r["label"] == room],
                                   strict),
                            "judged": sum(r["label"] == room for r in rows)
                            >= JUDGED_MIN and len({r["walk"] for r in rows
                                                   if r["label"] == room})
                            >= JUDGED_WALKS}
                     for room in rooms},
        "answers": {k: dict(sorted(v.items(), key=lambda kv: -kv[1]))
                    for k, v in answers.items()},
    }
    e = rep["frame_error"]["error"]
    rep["decision"] = (None if e is None else
                       "build the noise model" if e <= 0.05 else
                       "re-run 3.116.A in the sim at this rate" if e <= 0.15
                       else "wait for the Rover")
    RUN.mkdir(exist_ok=True)
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
