"""3.91 B6 (amendment 1): a recorded walk replayed through the full tier
ladder, offline -- YOLOE + CLIP on every frame, each question to the local
VLM first, the cloud only after `tier_local_max_unable` unable answers.

The policy is the real `TieredVision`, built by the brain's own
`_tiered_vision_fn` from the default config with two changes: the local tier
on (`--local-url`) and `tier_async_cloud` off, so every question is answered
on the frame that asked it and the replay is deterministic. The frames are
the ones the live brain decided on (`navigate` set), in order.

The cloud is a stub and makes no call: asked about a frame the walk's cloud
was asked about (`cloud_answers`, matched by `asked_seq`, the robot's seq),
it returns that recorded answer through the real `to_scene`; otherwise it
records "would have asked" and returns not visible with the live row's
action.

Open loop: the frames were taken under the live decisions and cannot change
with the replay's.

    python evaluations/tier-fusion-391/replay_tiers.py <walk dir> <out.json> \
        --local-url http://127.0.0.1:8005 [--probe-seqs 3,19,...]

`--probe-seqs` also puts those frames (robot seqs) to the local VLM
directly, the frame question with `box=True`, outside the policy.
"""

import argparse
import base64
import json
import os
import statistics
import threading
import time

from brain.local_vlm import LocalVLMClient
from brain.navigate import to_scene
from control.brain_config import load_brain_config
from control.brain_server import _tiered_vision_fn


def mem_available_mb():
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) // 1024
    return None


class MemWatch(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.low = None
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            mb = mem_available_mb()
            if mb is not None and (self.low is None or mb < self.low):
                self.low = mb
            self.stop.wait(0.5)


def frame_of(walk_dir, row):
    raw = open(os.path.join(walk_dir, row["file"]), "rb").read()
    return {"image_base64": base64.b64encode(raw).decode(),
            "media_type": row.get("media_type", "image/jpeg"),
            "captured_at": row.get("captured_at"),
            "metadata": {"seq": row.get("teleop_seq"), "source": "teleop"}}


def recorded_answers(rows):
    """asked_seq -> the first recorded cloud answer about that frame."""
    out = {}
    for row in rows:
        for entry in ((row.get("navigate") or {}).get("cloud_answers") or []):
            if "error" not in entry and entry.get("asked_seq") not in out:
                out[entry["asked_seq"]] = entry
    return out


def reversals(actions):
    """FORWARD -> REVERSE changes, reading only the two drive actions."""
    last, n = None, 0
    for a in actions:
        if a in ("FORWARD", "REVERSE"):
            if last == "FORWARD" and a == "REVERSE":
                n += 1
            last = a
    return n


def p90(xs):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(0.9 * (len(xs) - 1))))] if xs else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("walk_dir")
    ap.add_argument("out")
    ap.add_argument("--local-url", required=True)
    ap.add_argument("--secret", default=os.environ.get("LOCAL_VLM_SECRET"))
    ap.add_argument("--probe-seqs", default="")
    args = ap.parse_args()

    meta = json.load(open(os.path.join(args.walk_dir, "meta.json")))
    target = meta["target_object"]
    rows = [json.loads(line) for line in open(os.path.join(args.walk_dir, "walk.jsonl"))]
    decided = [r for r in rows if r.get("navigate")]
    by_seq = {r.get("teleop_seq"): r for r in rows}
    recorded = recorded_answers(rows)

    config = load_brain_config()
    config["tier_local_vlm_url"] = args.local_url
    config["tier_async_cloud"] = False

    current = {"row": None}
    cloud_log = []

    def stub_cloud(frame):
        seq = frame["metadata"]["seq"]
        entry = recorded.get(seq)
        if entry is not None:
            reply = dict(entry.get("reply") or {})
            reply["action"] = reply.get("reply_action") or entry.get("action")
            if "target_visible" not in reply:
                # -200025 was recorded before A's fields were fixed: the verdict only.
                reply["target_visible"] = entry.get("verdict") == "corroborated"
            cloud_log.append({"seq": seq, "kind": "recorded", "verdict": entry.get("verdict"),
                              "action": reply["action"]})
            return to_scene(reply, target)
        live_action = (current["row"]["navigate"] or {}).get("action") or "FORWARD"
        cloud_log.append({"seq": seq, "kind": "would_have_asked", "action": live_action})
        return to_scene({"action": live_action, "target_visible": False,
                         "target_direction": "not_visible",
                         "reasoning": "replay stub: no recorded answer for this frame"}, target)

    local_log = []
    watch = MemWatch()
    watch.start()
    policy = _tiered_vision_fn(target, stub_cloud, config, local_secret=args.secret)
    real_ask = policy.local_sighting_fn

    def logged_ask(frame, box=False):
        started = time.perf_counter()
        got = real_ask(frame, box=box)
        local_log.append({"seq": frame["metadata"]["seq"], "status": got.get("status"),
                          "score": got.get("score"), "box_frac": got.get("box_frac"),
                          "ms": round((time.perf_counter() - started) * 1000, 1)})
        return got

    policy.local_sighting_fn = logged_ask
    out_frames = []
    try:
        for row in decided:
            current["row"] = row
            n_local, n_cloud = len(local_log), len(cloud_log)
            scene = policy(frame_of(args.walk_dir, row))
            perception = scene.get("_perception") or {}
            tier = scene.get("_tier") or {}
            live = row["navigate"]
            out_frames.append({
                "seq": row["seq"], "teleop_seq": row.get("teleop_seq"),
                "perception": perception.get("status"),
                "probability": perception.get("match_probability"),
                "trigger": tier.get("trigger") or tier.get("local_asked"),
                "local": local_log[n_local:] or None,
                "cloud": cloud_log[n_cloud:] or None,
                "action": scene.get("safest_direction"),
                "turn_deg": scene.get("turn_deg"),
                "reasoning": (scene.get("reasoning") or "")[:300],
                "live_action": live.get("action"),
                "live_perception": ((live.get("_missionStatus") or {}).get("perception")
                                    or {}).get("status"),
            })
        probes = []
        if args.probe_seqs:
            client = LocalVLMClient(args.local_url, target, secret=args.secret,
                                    deadline_s=config["tier_local_timeout_s"])
            for seq in (int(s) for s in args.probe_seqs.split(",")):
                started = time.perf_counter()
                got = client.ask(frame_of(args.walk_dir, by_seq[seq]), box=True)
                probes.append({"seq": seq, "status": got.get("status"), "score": got.get("score"),
                               "box_frac": got.get("box_frac"),
                               "ms": round((time.perf_counter() - started) * 1000, 1)})
            client.close()
        stats = policy.stats.as_dict()
    finally:
        policy.close()
        watch.stop.set()

    def kind(entry):
        s, score = entry["status"], entry["score"]
        if s == "answered" and isinstance(score, (int, float)):
            if score >= 0.5:
                return "yes"
            if score <= config["tier_local_no_below"]:
                return "no"
            return "unable_band"
        return f"unable_{s}"

    on_detected = [kind(e) for f in out_frames if f["perception"] == "detected"
                   for e in (f["local"] or [])]
    summary = {
        "frames": len(out_frames),
        "local_answers": dict(sorted(
            {k: sum(kind(e) == k for f in out_frames for e in (f["local"] or []))
             for k in {kind(e) for f in out_frames for e in (f["local"] or [])}}.items())),
        "local_on_detected": {k: on_detected.count(k) for k in sorted(set(on_detected))},
        "cloud_recorded": sum(c["kind"] == "recorded" for c in cloud_log),
        "cloud_would_have_asked": sum(c["kind"] == "would_have_asked" for c in cloud_log),
        "live_dispatched": sum("cloud call dispatched" in (r["navigate"].get("reasoning") or "")
                               for r in decided),
        "reversals": reversals(f["action"] for f in out_frames),
        "live_reversals": reversals(f["live_action"] for f in out_frames),
        "local_ms_median": (statistics.median(e["ms"] for e in local_log) if local_log else None),
        "local_ms_p90": p90([e["ms"] for e in local_log]),
        "mem_available_min_mb": watch.low,
        "probes": {k: sum(kind(p) == k for p in probes) for k in {kind(p) for p in probes}},
    }
    json.dump({"walk": os.path.basename(os.path.normpath(args.walk_dir)), "target": target,
               "config": {k: config[k] for k in sorted(config) if k.startswith("tier_")},
               "summary": summary, "stats": stats, "probes": probes, "frames": out_frames},
              open(args.out, "w"), indent=1, default=str)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
