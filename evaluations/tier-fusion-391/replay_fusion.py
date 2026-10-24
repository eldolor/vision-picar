"""3.91.C (amendment 2): criteria 6-7 replayed offline on the records B made.

The policy is the real `TieredVision`, built by the brain's own
`_tiered_vision_fn` from the default config with `tier_async_cloud` off (as
B6). Three stand-ins, so this runs on the laptop with no model and no call:

* perception: the B3 / B5 record of each decided frame (YOLOE + CLIP on the
  Jetson), rebuilt as a `Perception` -- status, best P, bearing, box share;
* the local VLM: B6's recorded answer about the frame (VE on the Jetson); a
  frame B6 did not ask about borrows the nearest asked frame's (by robot
  seq), counted;
* the cloud: not visible, REVERSE, on every call (criterion 6's stub).

Two arms (cloud only; local first), each with 3.91.C on and off.

    python evaluations/tier-fusion-391/replay_fusion.py <walk> <perception.json> \
        <B6-replay.json> <out.json>

Open loop: the frames were taken under the live decisions.
"""

import json
import math
import sys
from collections import Counter

from brain.navigate import to_scene
from brain.perceive import (ABSENT, CLIP_LOGIT_SCALE, DETECTED, UNAVAILABLE, Box, Candidate,
                            Detection, Perception)
from control.brain_config import load_brain_config
from control.brain_server import _tiered_vision_fn

SIDE = 1000.0      # a frame of SIDE x SIDE "pixels": box shares carry over exactly
HFOV_DEG = 66.0    # perceive.py's default, the live pipeline's


def perception_of(rec):
    """A recorded frame as the pipeline would have returned it."""
    status = rec["status"]
    if status not in (DETECTED, ABSENT):
        return Perception(status=UNAVAILABLE, reason=rec.get("reason") or "recorded")
    p = rec.get("probability")
    if p is None:
        return Perception(status=ABSENT, reason=rec.get("reason") or "recorded")
    p = min(max(float(p), 1e-6), 1 - 1e-6)
    w = (rec.get("box_w") or 0.0) * SIDE
    h = (rec.get("box_h") or 0.0) * SIDE
    scores = (math.log(p) / CLIP_LOGIT_SCALE, math.log(1 - p) / CLIP_LOGIT_SCALE)
    cand = Candidate(detection=Detection(Box(0, 0, w, h), "recorded", 1.0),
                     similarity=scores[0], scores=scores, bearing_deg=rec.get("bearing_deg"))
    if status == DETECTED:
        return Perception(status=DETECTED, candidates=[cand], best=cand,
                          reason=rec.get("reason") or "recorded", image_size=(SIDE, SIDE))
    return Perception(status=ABSENT, candidates=[cand], reason=rec.get("reason") or "recorded")


class Pipeline:
    hfov_deg = HFOV_DEG
    match_probability = None
    vocabulary = None

    def __init__(self, by_seq):
        self.by_seq = by_seq

    def perceive(self, frame):
        return perception_of(self.by_seq[frame["metadata"]["seq"]])


def reversals(actions):
    last, n = None, 0
    for a in actions:
        if a in ("FORWARD", "REVERSE"):
            n += last == "FORWARD" and a == "REVERSE"
            last = a
    return n


def replay(target, frames, perception, answers, local_first, c_on, cloud_visible=False):
    config = load_brain_config()
    config["tier_async_cloud"] = False
    # A URL wires the local tier's settings as live (its pace, the candidate
    # steer); nothing is ever sent to it -- the client is replaced below, and
    # building one makes no request.
    config["tier_local_vlm_url"] = "http://127.0.0.1:9" if local_first else ""
    if not c_on:
        config["tier_remember_no"] = False
    log = {"cloud": 0, "local": 0, "borrowed": 0}
    asked_seqs = sorted(answers)

    def cloud(frame):
        log["cloud"] += 1
        if cloud_visible:
            # Recorded, no bar (2026-10-24): the red walk's target is real, and
            # its cloud (GPT-6 Astra, recall 100% in 3.80.Q) says so.
            return to_scene({"action": "FORWARD", "target_visible": True,
                             "target_direction": "center",
                             "reasoning": "replay stub: visible"}, target)
        return to_scene({"action": "REVERSE", "target_visible": False,
                         "target_direction": "not_visible",
                         "reasoning": "replay stub: not visible"}, target)

    def local(frame, box=False):
        seq = frame["metadata"]["seq"]
        log["local"] += 1
        if seq not in answers:
            log["borrowed"] += 1
            seq = min(asked_seqs, key=lambda s: (abs(s - seq), s))
        a = answers[seq]
        out = {"status": a["status"], "score": a["score"]}
        if a.get("box_frac"):
            out["box_frac"] = a["box_frac"]
        return out

    # simulated=True only to skip loading YOLOE + CLIP; the pipeline and
    # the local tier are replaced below by the records.
    policy = _tiered_vision_fn(target, cloud, config, simulated=True)
    policy.pipeline = Pipeline(perception)
    if local_first:
        policy.local_identity_fn.close()
        policy.local_sighting_fn = local
    policy.local_identity_fn = None
    rows = []
    # C5b's "first local no" as written: any local no (review: the
    # narrower count, a no landing on a DETECTED frame, started late when
    # the first no was about a frame under the gate). Both are recorded.
    yes_seen = no_seen = no_seen_detected = False
    try:
        for seq in frames:
            rec = perception[seq]
            n_cloud, n_local = log["cloud"], log["local"]
            scene = policy({"image_base64": "x", "captured_at": rec.get("captured_at"),
                            "metadata": {"seq": seq, "source": "teleop"}})
            tier = scene.get("_tier") or {}
            why = (scene.get("_navigate") or {}).get("reasoning") or ""
            landed = tier.get("local_landed") or {}
            # At the bag: perception's steer, or its uncertain-candidate rung
            # (a crop under the gate, the same bag; the Thermos pass).
            steered = ("steering on local sighting" in why
                       or "steering on an uncertain candidate" in why)
            rows.append({"seq": seq, "perception": rec["status"],
                         "action": scene.get("safest_direction"), "steered": steered,
                         # The masked rung's words only, not probation's (review).
                         "masked": "said is not the target" in why,
                         "probation": "(probation)" in why,
                         "after_yes": yes_seen, "after_no": no_seen,
                         "after_no_detected": no_seen_detected,
                         "rejected": (tier.get("rejected") or {}).get("by"),
                         "cloud": log["cloud"] - n_cloud, "local": log["local"] - n_local,
                         "local_answer": landed.get("answer"),
                         "why": why[:200]})
            yes_seen = yes_seen or landed.get("answer") == "yes"
            # local_landed is shown only on the frame it landed, which in
            # sync mode is the frame it was asked about.
            no_seen = no_seen or landed.get("answer") == "no"
            no_seen_detected = no_seen_detected or (landed.get("answer") == "no"
                                                    and rec["status"] == DETECTED)
        stats = policy.stats.as_dict()
    finally:
        policy.close()
    det = [r for r in rows if r["perception"] == DETECTED]
    return {
        "reversals": reversals(r["action"] for r in rows),
        "cloud_calls": log["cloud"],
        "local_answers": log["local"],
        "borrowed": log["borrowed"],
        "steered_after_first_no": sum(r["steered"] for r in rows if r["after_no"]),
        "steered_after_first_no_on_a_detection": sum(
            r["steered"] for r in rows if r["after_no_detected"]),
        "probation_frames": sum(r["probation"] for r in det),
        "masked_after_a_yes": sum(r["masked"] for r in det if r["after_yes"]),
        "detected_frames": len(det),
        "actions": dict(Counter(r["action"] for r in rows)),
        "rejections": stats["rejections"], "releases": stats["releases"],
        "cannot_tell": stats["cannot_tell"], "masked_frames": stats["masked_frames"],
        "triggers": stats["triggers"], "local_triggers": stats["local_triggers"],
        "rows": rows,
    }


def main(walk, perception_path, b6_path, out_path, *flags):
    rec = json.load(open(perception_path))
    b6 = json.load(open(b6_path))
    perception = {f["teleop_seq"]: f for f in rec["frames"]}
    frames = [f["teleop_seq"] for f in b6["frames"]]
    answers = {}
    for f in b6["frames"]:
        for a in f.get("local") or []:
            answers.setdefault(a["seq"], a)
    out = {"walk": walk, "target": b6["target"], "frames": len(frames),
           "recorded_local_answers": len(answers), "arms": {}}
    for arm, local_first in (("cloud_only", False), ("local_first", True)):
        for c in ("on", "off"):
            out["arms"][f"{arm}/{c}"] = replay(b6["target"], frames, perception, answers,
                                              local_first, c == "on")
    if "--cloud-visible" in flags:
        out["arms"]["local_first/on/cloud_visible"] = replay(
            b6["target"], frames, perception, answers, True, True, cloud_visible=True)
    json.dump(out, open(out_path, "w"), indent=1)
    for name, r in out["arms"].items():
        print(f"{name:18s} reversals {r['reversals']:2d}  cloud {r['cloud_calls']:2d}  "
              f"local {r['local_answers']:2d} (borrowed {r['borrowed']:2d})  "
              f"steered after first no {r['steered_after_first_no']:2d}  "
              f"masked after a yes {r['masked_after_a_yes']:2d}  "
              f"rejections {r['rejections']} releases {r['releases']} "
              f"cannot_tell {r['cannot_tell']}")


if __name__ == "__main__":
    main(*sys.argv[1:])
