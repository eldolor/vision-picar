"""
walk_replay.py

Re-ask a recorded walk's frames, under a different model, and compare the
answers to what was recorded at the time.

**Why this is the only honest way to compare models.** A live walk is a
physical path a person took, and the path was itself steered by whichever
model was answering -- so two walks by two models are two different
experiments, and the gap between their scores mixes model quality with
where the operator happened to point the phone. Replaying holds the pixels
fixed and varies exactly one thing. Every model comparison in this project
that turned out to be trustworthy was done this way; every one done by
comparing live walks had to be retracted at least once.

The frames already sit on the recordings volume that control/admin_server.py
mounts, so a replay is a few HTTP calls and no walking.

**It calls the deployed /navigate rather than Bedrock directly**, which is
worth stating plainly because control/admin_server.py's docstring otherwise
says this process never talks to the vision service:

  * The question a replay asks is "what would the SERVICE say about this
    frame". That includes its prompt, its JSON parsing, its allow-list and
    its model resolution. Calling Bedrock straight from here would answer a
    different question -- what a copy of the prompt says -- and would quietly
    stop tracking the real route the moment either changed.
  * The prompt already exists twice on purpose (brain/vision.py and
    service/vision_analyze/vision_core.py, see CLAUDE.md section 6). A third
    copy inside control/ would be a mistake, not a trade-off.

The cost is that replay needs the vision service up. That is inherent --
there is nothing to replay against otherwise -- and it fails to just this
feature rather than to the console.

`post_navigate` is injected so the whole module is testable with no network
and no AWS.
"""

import logging
from concurrent.futures import ThreadPoolExecutor

logger = logging.getLogger(__name__)

# The vision service is the bottleneck (~2-3s per frame), so this sets how
# long a replay takes: frames / workers * per-call. It matters because the
# load balancer in front of admin gives up at 60s, and a 39-frame walk at 4
# workers went over. Not unbounded, though: at 8 workers Bedrock throttled
# 10 of 22 frames, so the caller retries with backoff and this stays modest.
DEFAULT_WORKERS = 6


def replay_walk(entries, frame_bytes_for, target_object, post_navigate,
                model_id=None, max_workers=DEFAULT_WORKERS):
    """Re-ask every frame of a walk.

    `post_navigate(image_bytes, target_object, model_id) -> dict` performs one
    /navigate call and returns its JSON body.

    Returns the replayed answers shaped as walk.jsonl entries, so the very
    same control/walk_eval.py scoring runs over a replay and over the
    original with no special-casing, plus a per-frame diff against what was
    recorded at the time.
    """
    if not entries:
        return {"model_id": model_id, "frames": 0, "entries": [], "diff": [],
                "errors": 0, "agreement": None}

    def one(entry):
        nav = entry.get("navigate") or {}
        image = frame_bytes_for(entry)
        if not image:
            return entry, None, "no image bytes"
        try:
            return entry, post_navigate(image, target_object, model_id), None
        except Exception as e:  # noqa: BLE001 -- one bad frame must not lose the replay
            logger.warning("replay failed on %s: %s", entry.get("file"), e)
            return entry, None, f"{type(e).__name__}: {e}"[:160]

    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        results = list(pool.map(one, entries))

    replay_entries, diff, errors, agreed, compared = [], [], 0, 0, 0
    for entry, answer, err in results:
        original = (entry.get("navigate") or {}).get("action")
        if err or not answer:
            errors += 1
            diff.append({"seq": entry.get("seq"), "file": entry.get("file"),
                         "original": original, "replayed": None, "error": err})
            continue
        # Carry the model actually used, exactly as a live recording does, so
        # a stored replay is self-describing.
        answer.setdefault("model_id", model_id)
        replay_entries.append({
            "seq": entry.get("seq"),
            "file": entry.get("file"),
            "media_type": entry.get("media_type", "image/jpeg"),
            "navigate": answer,
        })
        replayed = answer.get("action")
        compared += 1
        if replayed == original:
            agreed += 1
        if replayed != original:
            diff.append({"seq": entry.get("seq"), "file": entry.get("file"),
                         "original": original, "replayed": replayed,
                         "why": str(answer.get("reasoning", ""))[:200]})

    return {
        "model_id": model_id,
        "frames": len(entries),
        "errors": errors,
        # How often the replay chose the same move as the recording. Low
        # agreement is not itself bad -- it is the whole point of trying
        # another model -- but it says how much of the walk actually differed.
        "agreement": round(agreed / compared, 3) if compared else None,
        "entries": replay_entries,
        "diff": diff,
    }
