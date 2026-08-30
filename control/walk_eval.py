"""
walk_eval.py

Scores a recorded Robot-view walk (control/brain_server.py's
POST /recording/frame writes them; control/admin_server.py serves them)
so a walk can be judged without a person reading 22 frames of JSON by
hand -- which is exactly how the 2026-08-29 findings were produced, and
is not a repeatable process.

**This is a diagnostic score, not a quality score, and the distinction is
load-bearing.** A walk's log carries no ground truth: a STOP in front of
a wall and a STOP in front of nothing at all produce byte-identical
entries. Nothing here can tell you "the robot navigated well". What the
metrics below *can* tell you, cheaply and deterministically, is that a
walk exhibits one of the failure shapes this project has actually hit:

  degenerate        one action for every frame -- the failure
                    tests/manual_replay_navigate.py's docstring warns
                    about, where a clean-looking run means the model is
                    not reading the scene at all (Nova Lite answered
                    FORWARD on 22/22 frames of walk 195904).
  stalled           a long run with no FORWARD. This is the 2026-08-29
                    ottoman walk: the target stayed visible and centred
                    while the policy turned back and forth for 21
                    consecutive frames and never closed the distance.
  oscillating       LEFT/RIGHT alternation, the signature of a policy
                    re-deciding from scratch every frame with no memory
                    of which way it already turned.
  unstable-identity target_visible flipping on and off, which is what a
                    red blanket being intermittently mistaken for a red
                    backpack looks like from the log side.

The judge tier (judge_walk) is the only part with anything resembling
ground truth, because it looks at the pixels. It is a real Bedrock call
per sampled frame, so it is sampled and capped rather than run over every
frame.

Deliberately free of FastAPI, boto3 client construction and filesystem
layout: compute_metrics/score_walk are pure functions over the parsed
walk.jsonl entries, so the whole scorecard is unit-testable with no AWS
and no temp directories (tests/test_walk_eval.py). The caller supplies a
bedrock client and the frame bytes.
"""

import json
import logging
import re
from concurrent.futures import ThreadPoolExecutor

logger = logging.getLogger(__name__)

# 3: the judge was penalising arrival. It never saw target_reached, and the
# /navigate prompt tells a model that "action" means movement only and that
# arrival belongs in target_reached -- so a model doing exactly that
# (target_reached: true, action: FORWARD) was marked wrong for obeying its
# instructions. Bumping rescoes every stored eval.json rather than leaving
# walks ranked by a judge that disagreed with the policy prompt.
SCHEMA_VERSION = 4

MOVE_ACTIONS = ("FORWARD", "LEFT", "RIGHT", "REVERSE", "STOP")

# Thresholds. Chosen against the six walks recorded 2026-08-28/29, which
# are the only real-world material this project has -- they are calibrated
# to flag those specific failures, not derived from anything more
# principled, and should be revisited once there are walks that a person
# has judged good.
DEGENERATE_SHARE = 0.90      # >=90% one action == not reading the scene
STALL_RUN = 8                # >=8 frames without a FORWARD == not closing distance
OSCILLATION_RATE = 0.40      # >=40% of turns immediately reversing direction
IDENTITY_FLIPS = 4           # target_visible changing this many times


def _entry_actions(entries: list) -> list:
    return [(e.get("navigate") or {}).get("action") for e in entries]


def compute_metrics(entries: list) -> dict:
    """Pure, deterministic, no model calls. `entries` is walk.jsonl parsed,
    in seq order -- the same shape admin_server's get_walk() returns."""
    navs = [(e.get("navigate") or {}) for e in entries]
    n = len(navs)
    if n == 0:
        return {"frames": 0, "empty": True}

    actions = [nav.get("action") for nav in navs]
    counted = {a: actions.count(a) for a in sorted(set(a for a in actions if a))}
    dominant = max(counted.values()) if counted else 0

    # Longest run of consecutive frames with no FORWARD -- "how long did it
    # go without making progress", which is the ottoman failure directly.
    longest_no_forward = run = 0
    for a in actions:
        run = 0 if a == "FORWARD" else run + 1
        longest_no_forward = max(longest_no_forward, run)

    # Turn reversals: RIGHT immediately after LEFT (or vice versa), as a
    # share of all turns. A policy routing deliberately around an obstacle
    # keeps turning the same way; one re-deciding blind flips.
    turns = [a for a in actions if a in ("LEFT", "RIGHT")]
    reversals = sum(1 for x, y in zip(turns, turns[1:]) if x != y)
    oscillation = (reversals / (len(turns) - 1)) if len(turns) > 1 else 0.0

    visible = [nav.get("target_visible") is True for nav in navs]
    flips = sum(1 for x, y in zip(visible, visible[1:]) if x != y)

    models = sorted({nav.get("model_id") for nav in navs if nav.get("model_id")})

    return {
        "frames": n,
        "action_spread": counted,
        "dominant_action_share": round(dominant / n, 3),
        "forward_rate": round(actions.count("FORWARD") / n, 3),
        "longest_no_forward_run": longest_no_forward,
        "oscillation_rate": round(oscillation, 3),
        "target_visible_rate": round(sum(visible) / n, 3),
        "visibility_flips": flips,
        "target_reached": any(nav.get("target_reached") is True for nav in navs),
        "obstacle_rate": round(
            sum(1 for nav in navs if nav.get("obstacle_ahead") is True) / n, 3),
        "model_ids": models,
    }


def metric_flags(m: dict) -> list:
    """The named failure shapes, in the order they matter for diagnosis.

    Reads every field defensively: an eval.json written by an older
    SCHEMA_VERSION is still on the volume after a redeploy, and a missing
    metric should mean "this shape wasn't measured", never a 500 in the
    admin console.
    """
    if m.get("empty"):
        return ["empty"]
    flags = []
    if m.get("dominant_action_share", 0) >= DEGENERATE_SHARE:
        flags.append("degenerate")
    if m.get("longest_no_forward_run", 0) >= STALL_RUN:
        flags.append("stalled")
    if m.get("oscillation_rate", 0) >= OSCILLATION_RATE:
        flags.append("oscillating")
    if m.get("visibility_flips", 0) >= IDENTITY_FLIPS:
        flags.append("unstable-identity")
    return flags


JUDGE_PROMPT = """A small indoor robot is searching for a {target_object}. Its camera
took this photo, and its navigation policy chose the action: {action}
(it reported: target_visible={target_visible}, target_reached={target_reached},
obstacle_ahead={obstacle_ahead}, reasoning: "{reasoning}").

{history_note}
The robot's available actions are FORWARD, LEFT, RIGHT, REVERSE and STOP.
It moves roughly 30cm per FORWARD step. Turning costs a step and does not
close distance.

Important: in this system "action" describes MOVEMENT ONLY. It never means
"the search is over" -- arrival is reported separately, in target_reached.
So do NOT mark an action wrong merely because the robot has arrived:
- If target_reached is true, the mission ends on this frame regardless of
  the action, so the action is moot. Judge it sensible unless it would
  drive into something OTHER than the {target_object}.
- The {target_object} is the goal, not an obstacle. Closing the last of the
  distance to it is correct behaviour, not a collision risk.

Judge whether the chosen action was sensible for this photo AND for where
the robot is in its walk. The mistake to be strictest about is the one that
wastes the robot's time:
- Refusing to move FORWARD when there is clearly open floor ahead, merely
  because furniture is visible somewhere further away.
- Repeating a turn that the recent moves show is not working. If the robot
  has already turned several times without moving forward, another turn is
  NOT sensible -- it is the same mistake again, and the robot is stuck.
  Judge that harshly even if the turn looks defensible in isolation.
Also wrong, in the other direction:
- Choosing FORWARD when a piece of furniture, a wall or a person -- not the
  {target_object} itself -- is within about one step.

Respond with ONLY a JSON object:
{{
  "sensible": true | false,
  "better_action": "FORWARD" | "LEFT" | "RIGHT" | "REVERSE" | "STOP" | null,
  "why": "one short sentence"
}}"""


def _parse_json_reply(text: str) -> dict:
    text = text.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:]
    match = re.search(r"\{.*\}", text, re.S)
    if not match:
        raise ValueError(f"no JSON object in judge reply: {text[:120]!r}")
    return json.loads(match.group(0))


HISTORY_WINDOW = 6


def _history_note(prior_actions: list) -> str:
    """The recent moves, in words, so a frame can be judged in the context
    of the walk rather than in isolation.

    Without this the judge is structurally blind to the failure that
    actually happens here: one RIGHT looks perfectly defensible on its own,
    and so does the sixtieth. A walk that turned in place for 80 consecutive
    frames was rated 6-of-8 sensible, because every frame really did look
    fine by itself. Trajectory failure only exists across frames.
    """
    if not prior_actions:
        return "This is the robot's first move of the walk.\n"
    window = prior_actions[-HISTORY_WINDOW:]
    note = "The robot's previous moves, oldest first, were: " + ", ".join(window) + ".\n"
    if window and "FORWARD" not in window:
        note += (f"Note it has NOT moved forward once in its last {len(window)} moves -- "
                 "it may be turning on the spot instead of making progress.\n")
    return note


def judge_frame(client, model_id: str, image_bytes: bytes, nav: dict,
                target_object: str, media_format: str = "jpeg",
                prior_actions: list | None = None) -> dict:
    prompt = JUDGE_PROMPT.format(
        target_object=target_object,
        action=nav.get("action"),
        target_visible=nav.get("target_visible"),
        target_reached=nav.get("target_reached"),
        obstacle_ahead=nav.get("obstacle_ahead"),
        reasoning=str(nav.get("reasoning", ""))[:200].replace('"', "'"),
        history_note=_history_note(prior_actions or []),
    )
    resp = client.converse(
        modelId=model_id,
        messages=[{"role": "user", "content": [
            {"image": {"format": media_format, "source": {"bytes": image_bytes}}},
            {"text": prompt},
        ]}],
        inferenceConfig={"maxTokens": 200},
    )
    text = "".join(b.get("text", "") for b in resp["output"]["message"]["content"])
    verdict = _parse_json_reply(text)
    return {
        "sensible": verdict.get("sensible") is True,
        "better_action": verdict.get("better_action"),
        "why": str(verdict.get("why", ""))[:300],
    }


def _sample_indices(n: int, k: int) -> list:
    """Evenly spaced, always including the first and last frame -- a walk's
    ending is where arrival or a terminal stall shows up."""
    if n <= k:
        return list(range(n))
    return sorted({round(i * (n - 1) / (k - 1)) for i in range(k)})


def judge_walk(client, model_id: str, entries: list, frame_bytes_for,
               target_object: str, sample: int = 8, max_workers: int = 4) -> dict:
    """Judge up to `sample` evenly spaced frames. `frame_bytes_for` maps an
    entry to its image bytes (admin reads them off EFS); a frame it cannot
    supply is skipped rather than failing the walk."""
    if not entries:
        return {"judged": 0, "sensible_rate": None, "frames": [], "error": "no entries"}

    indices = _sample_indices(len(entries), sample)
    # Actions leading up to each sampled frame come from the WHOLE walk, not
    # from the sample -- the frames between two samples are exactly where a
    # stall hides.
    all_actions = [(e.get("navigate") or {}).get("action") or "?" for e in entries]
    picks = [(i, entries[i]) for i in indices]

    def one(pick):
        i, entry = pick
        nav = entry.get("navigate") or {}
        try:
            image = frame_bytes_for(entry)
            if not image:
                return None
            out = judge_frame(client, model_id, image, nav, target_object,
                              prior_actions=all_actions[:i])
        except Exception as e:  # noqa: BLE001 -- a judge failure must not fail the walk
            logger.warning("judge failed on %s: %s", entry.get("file"), e)
            return {"file": entry.get("file"), "seq": entry.get("seq"),
                    "action": nav.get("action"), "error": f"{type(e).__name__}"}
        out.update({"file": entry.get("file"), "seq": entry.get("seq"),
                    "action": nav.get("action")})
        return out

    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        results = [r for r in pool.map(one, picks) if r is not None]

    scored = [r for r in results if "sensible" in r]
    rate = round(sum(1 for r in scored if r["sensible"]) / len(scored), 3) if scored else None
    return {
        "judged": len(scored),
        "attempted": len(results),
        "sensible_rate": rate,
        "model_id": model_id,
        "frames": results,
    }


def score_walk(metrics: dict, judge: dict | None) -> dict:
    """Blend into one advisory 0-100 number.

    Three parts, and the split is the result of the scorer being wrong twice:

    completion (0.25) -- did the walk ever report target_reached. Reaching
        the object is the entire task, and for a while nothing in this score
        noticed whether it happened: a walk that turned in place for 80
        frames and never arrived was rated the same as one that arrived in
        11, all else equal. It is weighted but not absolute, because a walk
        can legitimately end without arriving (the target really isn't in
        the room, or the operator simply stopped).

    behaviour (0.30) -- the four measured shapes, averaged. Every flag
        metric_flags() can raise now moves the number. Previously only
        degeneracy and stalling did, so `oscillating` and
        `unstable-identity` were computed, displayed, and silently ignored
        -- four walks oscillating at 40-60% paid nothing for it.

    judge (0.45) -- the only part that looks at pixels, so it keeps the
        largest single share, but no longer a majority. It used to carry
        0.6, which let a walk the metrics knew had stalled for 80
        consecutive frames still score 65 because each sampled frame looked
        defensible on its own.

    With no judge the other two are rescaled to the full 100, so a
    metrics-only run can still say "this looks fine".
    """
    if metrics.get("empty"):
        return {"score": 0, "verdict": "poor", "flags": ["empty"]}

    flags = metric_flags(metrics)

    # Non-degeneracy: full marks until one action passes half the walk,
    # zero once it is the only action.
    share = metrics.get("dominant_action_share", 0.0)
    non_degeneracy = 1.0 - max(0.0, (share - 0.5) / 0.5)

    # Progress: full marks for a walk that never goes STALL_RUN frames
    # without a FORWARD, decaying to zero at three times that.
    stall = metrics.get("longest_no_forward_run", 0)
    progress = 1.0 if stall < STALL_RUN else \
        1.0 - min(1.0, (stall - STALL_RUN) / (2 * STALL_RUN))

    # Smoothness: turning back and forth wastes steps even when each turn is
    # individually defensible. Zero once every turn reverses the last one.
    smoothness = 1.0 - min(1.0, metrics.get("oscillation_rate", 0.0))

    # Identity stability: how often the target blinked in and out of being
    # "visible" -- what a red blanket being mistaken for a red backpack
    # looks like from the log. Scaled against the flag threshold.
    flips = metrics.get("visibility_flips", 0)
    identity = 1.0 - min(1.0, flips / (2 * IDENTITY_FLIPS))

    # Non-degeneracy counts double. A walk that emits one action for every
    # frame is not navigating at all -- it scores perfectly on smoothness
    # (no turns to reverse) and progress (never stalls), and averaging the
    # four equally let a blind always-FORWARD walk outscore one that was
    # genuinely reading the scene.
    behaviour = (2 * non_degeneracy + progress + smoothness + identity) / 5
    completion = 1.0 if metrics.get("target_reached") else 0.0
    rate = (judge or {}).get("sensible_rate")

    if rate is None:
        score = 100 * (0.55 * behaviour + 0.45 * completion)
        basis = "metrics-only"
    else:
        score = 100 * (0.45 * rate + 0.30 * behaviour + 0.25 * completion)
        basis = "judge+metrics"

    score = int(round(score))
    verdict = "good" if score >= 70 else ("mixed" if score >= 45 else "poor")
    return {
        "score": score, "verdict": verdict, "flags": flags, "basis": basis,
        # Shown in the console so a number can be argued with rather than
        # just believed.
        "components": {
            "judge": round(rate, 3) if rate is not None else None,
            "behaviour": round(behaviour, 3),
            "completion": completion,
            "non_degeneracy": round(non_degeneracy, 3),
            "progress": round(progress, 3),
            "smoothness": round(smoothness, 3),
            "identity": round(identity, 3),
        },
    }
