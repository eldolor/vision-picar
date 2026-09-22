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
SCHEMA_VERSION = 9

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
MIN_TURNS_FOR_OSCILLATION = 6  # fewer turns than this and the reversal rate is noise
MIN_STUCK_WINDOW = 3         # moves of history before "it may be stuck" is fair


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
    # Reversal rate over too few turns is noise, not a pattern: two turns
    # that happen to differ read as 100% oscillation. Every short successful
    # walk was being flagged `oscillating` on 3-5 turns, which said nothing
    # about them. Below the floor, report 0 rather than a number that will
    # be over-read.
    turns = [a for a in actions if a in ("LEFT", "RIGHT")]
    reversals = sum(1 for x, y in zip(turns, turns[1:]) if x != y)
    oscillation = (reversals / (len(turns) - 1)) if len(turns) >= MIN_TURNS_FOR_OSCILLATION else 0.0

    visible = [nav.get("target_visible") is True for nav in navs]
    flips = sum(1 for x, y in zip(visible, visible[1:]) if x != y)

    # P25 -- how long a single command SURVIVES, and how often one is
    # replaced and immediately restored.
    #
    # `oscillation_rate` above looks only at turns and only at adjacent
    # pairs, so it reports 0.00 for a walk alternating FORWARD, LEFT,
    # FORWARD, LEFT -- which is the exact pattern an operator watching the
    # phone described as "confusing", and which measured a median run of
    # ONE frame on three of six rig walks. A command that changes every
    # frame is not a turn pattern; it is the absence of one.
    #
    # Reported rather than flagged. The repair is P7c item 2 -- a goal pose
    # in the odom frame, with the bearing dead-reckoned between detections
    # -- and until that exists every walk would carry the flag, which is how
    # a flag stops being read.
    decided = [a for a in actions if a]
    runs, run = [], 1
    for x, y in zip(decided, decided[1:]):
        if x == y:
            run += 1
        else:
            runs.append(run)
            run = 1
    runs.append(run) if decided else None
    median_run = (sorted(runs)[len(runs) // 2] if runs else 0)
    # A -> B -> A: the command that came back. Distinct from a run boundary,
    # which is any change at all.
    restored = sum(1 for i in range(2, len(decided))
                   if decided[i] == decided[i - 2] and decided[i] != decided[i - 1])

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
        "command_changes": max(len(runs) - 1, 0),
        "median_command_run": median_run,
        "longest_command_run": max(runs) if runs else 0,
        "command_restored": restored,
        "target_reached": any(nav.get("target_reached") is True for nav in navs),
        "obstacle_rate": round(
            sum(1 for nav in navs if nav.get("obstacle_ahead") is True) / n, 3),
        "model_ids": models,
    }


def collision_flag(collisions: dict | None) -> list:
    return ["collision"] if (collisions or {}).get("collisions") else []


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
- NEVER mark FORWARD wrong on the grounds that the robot is close to the
  {target_object}, or that it "should have stopped" because it has arrived.
  Whether it has arrived is the target_reached field, which you are not
  being asked to grade. If you disagree with target_reached, that is not an
  error in the ACTION -- answer sensible.
- The same goes for obstacle_ahead. You are grading the MOVE, not the
  robot's description of the scene. If you think obstacle_ahead was reported
  wrongly but the move itself was the right one to make, answer sensible.
  Only "better_action" decides this: if the action you would have chosen is
  the action it chose, then it was sensible, whatever else it got wrong.

Judge whether the chosen action was sensible for this photo AND for where
the robot is in its walk. Two kinds of mistake matter and they are NOT
equally bad.

The serious one, because the robot cannot undo it -- driving into something:
- Choosing FORWARD when a wall, a door, furniture or a person is within
  about one step. If a flat surface fills most of the frame and you cannot
  see floor between the camera and it, the robot is already up against
  something. FORWARD is wrong there no matter what it is hunting for, and
  ESPECIALLY when the {target_object} is not visible -- "keep going to
  continue the search" is not a reason to drive into a wall. Mark it not
  sensible and say STOP or a turn.

The wasteful one, which costs a step but breaks nothing:
- Refusing to move FORWARD when there is clearly open floor ahead, merely
  because furniture is visible somewhere further away.
- Repeating a turn that the recent moves show is not working. If the robot
  has already turned several times without moving forward, another turn is
  NOT sensible -- it is the same mistake again, and the robot is stuck.
  Judge that harshly even if the turn looks defensible in isolation.

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
    # Only claim it is stuck once there is enough history to mean it. Two
    # turns at the start of a walk is orientation, not a stall, and telling
    # the judge otherwise made it reject the opening moves of walks that
    # went on to reach the target in 13 frames.
    if len(window) >= MIN_STUCK_WINDOW and "FORWARD" not in window:
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
    sensible = verdict.get("sensible") is True
    better = verdict.get("better_action")
    # A judge that says "not sensible" and then names the SAME action has
    # contradicted itself -- it is really objecting to something other than
    # the move (it did this steadily over obstacle_ahead, marking FORWARD
    # wrong while recommending FORWARD). The action is the only thing being
    # graded, so the recommendation wins over the label.
    if not sensible and better and better == nav.get("action"):
        sensible = True
    return {
        "sensible": sensible,
        "better_action": better,
        "why": str(verdict.get("why", ""))[:300],
    }


COLLISION_PROMPT = """A small indoor robot searching for a {target_object} commanded
FORWARD on this frame.

Answer one question only: would moving forward about 30cm from here run the
robot into something it should not hit?

Say true if a wall, door, furniture, an appliance or a person occupies the
space immediately in front of the camera -- in particular if a flat surface
fills most of the frame with no floor visible between the camera and it. Say
false if there is clear floor ahead for at least a step, however much
furniture is further away.

The {target_object} is the ONE thing that does not count. Reaching it is the
robot's entire purpose, so a {target_object} close to the camera -- even
filling the frame -- is an arrival, not a collision. Answer false for that.
Answer true only if something ELSE is in the way.

You are not judging whether the move was clever or whether the robot found
what it was looking for. Only whether it was about to hit something it
should not.

Respond with ONLY a JSON object:
{{"would_collide": true | false, "why": "one short sentence"}}"""


def collision_candidates(entries: list) -> list:
    """Indices worth checking for an imminent collision.

    Every FORWARD, because that is the only action that can drive into
    anything. Deliberately NOT filtered by the walk's own obstacle_ahead: the
    log contains the model's CLAIM about obstacles, and the failure being
    looked for is precisely a model that claims "clear" while facing a wall.
    Filtering on it would hide exactly the frames that matter.
    """
    out = []
    for i, e in enumerate(entries):
        if ((e.get("navigate") or {}).get("action")) == "FORWARD":
            out.append(i)
    return out


def check_collisions(client, model_id: str, entries: list, frame_bytes_for,
                     target_object: str = "the target object",
                     max_checks: int = 12, max_workers: int = 4) -> dict:
    """Look for frames where the robot was told to drive into something.

    This is the one failure with physical consequences, and nothing else in
    the scorecard can see it. A walk that ended with three consecutive
    commands into a wall scored 56 -- entirely for not reaching the target --
    and the wall went unmentioned. It cannot be computed from the log for the
    reason in collision_candidates() above: it needs the pixels.

    Sampled from the END backwards. A walk that drives into something tends
    to do it once it is lost, which is late; and the last frames are where a
    recording stops precisely because the operator saw it happen.
    """
    candidates = collision_candidates(entries)
    if not candidates:
        return {"checked": 0, "collisions": [], "frames": []}
    picks = sorted(candidates[-max_checks:])

    def one(i):
        entry = entries[i]
        try:
            image = frame_bytes_for(entry)
            if not image:
                return None
            resp = client.converse(
                modelId=model_id,
                messages=[{"role": "user", "content": [
                    {"image": {"format": "jpeg", "source": {"bytes": image}}},
                    {"text": COLLISION_PROMPT.format(target_object=target_object)}]}],
                inferenceConfig={"maxTokens": 150},
            )
            text = "".join(b.get("text", "") for b in resp["output"]["message"]["content"])
            verdict = _parse_json_reply(text)
        except Exception as e:  # noqa: BLE001 -- one bad frame must not lose the check
            logger.warning("collision check failed on %s: %s", entry.get("file"), e)
            return None
        return {"seq": entry.get("seq"), "file": entry.get("file"),
                "would_collide": verdict.get("would_collide") is True,
                "why": str(verdict.get("why", ""))[:200]}

    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        results = [r for r in pool.map(one, picks) if r is not None]

    return {
        "checked": len(results),
        "forward_frames": len(candidates),
        "collisions": [r for r in results if r["would_collide"]],
        "frames": results,
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


def score_walk(metrics: dict, judge: dict | None, collisions: dict | None = None) -> dict:
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

    flags = metric_flags(metrics) + collision_flag(collisions)

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

    # A walk that drove into something is not a good walk, whatever else it
    # did. This is the only failure here with physical consequences, so it
    # caps the score rather than nudging it -- the alternative is a walk that
    # hit a wall still reading "good" because it was efficient about it.
    hits = len((collisions or {}).get("collisions", []))
    uncapped = int(round(score))
    if hits:
        # The cap is deliberate, but on its own it flattens the top of the
        # range: three walks that failed for quite different reasons all
        # scored exactly 40, which makes the number useless for the
        # comparison it exists to support. The capped score stays the
        # headline, and the uncapped one is reported beside it so walks
        # remain orderable.
        score = min(score, 40)

    score = int(round(score))
    verdict = "good" if score >= 70 else ("mixed" if score >= 45 else "poor")
    return {
        "score": score, "score_uncapped": uncapped,
        "verdict": verdict, "flags": flags, "basis": basis,
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
            "collisions": hits,
        },
    }
