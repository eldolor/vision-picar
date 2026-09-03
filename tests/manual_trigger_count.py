"""
manual_trigger_count.py

PLAN-onboard-perception.md 6.1 -- the free experiment.

That document's 2.4 argues the tiered architecture pays off because
deliberation becomes event-driven rather than periodic, and estimates the
saving at "~8x" from "sim runs took 40 steps, real walks reached target in
6-22 frames". It says in the same breath that the number is derived rather
than measured and should not be quoted until it is. This measures it.

Not an automated test, but unlike tests/manual_replay_navigate.py it is
also **free**: every field it reads was recorded at the time, so it makes
no model calls at all. The only network it does is fetching walk.jsonl
from the admin service.

What one frame means here. A recorded /navigate reply is one step the
robot took, and in the tiered design that step becomes local and free --
only a *trigger* costs a cloud call. So triggers/frames is the fraction of
today's spend that survives tiering, and its reciprocal is the saving.

The triggers are 2.4's, restricted to the ones a recorded walk can
actually witness:

    mission start        frame 0 -- there is no goal yet
    candidate sighting   the target becomes visible
    lost target          it goes away and stays away
    room change          room_guess changes; memory needs updating
    goal achieved        target_reached
    staleness            no trigger in the last --stale-n frames

Two of 2.4's are NOT witnessable and this therefore UNDERCOUNTS triggers
(so it over-states the saving, which is the safe direction for a cost
claim): "goal impossible / boxed in" needs the lidar this corpus predates,
and any goal the reactive tier would have completed early leaves no trace
in a walk the model drove step by step.

**Hysteresis is the whole finding, so it is a flag rather than a
constant.** A trigger policy is only as stable as the fields it reads, and
target_visible flips on roughly a quarter of frames in the corpus as of
2026-09-03 -- control/walk_eval.py raises `unstable-identity` for exactly
this. Firing on every raw edge counts noise as events. --sight-h and
--room-h require that many consecutive frames of agreement before an edge
is believed, the same way lost_target has always needed --lost-h.

Usage:
    export ADMIN_URL="http://<the admin service>"
    export ADMIN_SHARED_SECRET="..."
    python -m tests.manual_trigger_count
    python -m tests.manual_trigger_count --sight-h 2 --room-h 2
    python -m tests.manual_trigger_count --dir ./walks   # cached dumps

Prints a staleness sweep, a per-trigger-kind breakdown, the two fields'
own stability, and a per-walk table -- enough to paste into a findings
note, which is what 6.1 asks for.
"""

import argparse
import collections
import glob
import json
import os
import sys
import urllib.request


def fetch_walks(base: str, secret: str) -> list:
    """Every walk with its entries, from the admin service."""
    def get(path):
        req = urllib.request.Request(base.rstrip("/") + path,
                                     headers={"x-app-secret": secret})
        with urllib.request.urlopen(req, timeout=90) as f:
            return json.loads(f.read())

    names = [w["walk"] for w in get("/recording/walks")["walks"]]
    out = []
    for i, name in enumerate(names, 1):
        print(f"  fetching {i}/{len(names)} {name}", file=sys.stderr)
        try:
            out.append(get("/recording/walks/" + name))
        except Exception as e:  # noqa: BLE001 -- one bad walk must not lose the run
            print(f"    skipped: {e}", file=sys.stderr)
    return out


def load_walks(directory: str) -> list:
    return [json.load(open(p)) for p in sorted(glob.glob(os.path.join(directory, "*.json")))]


def triggers_for(entries, stale_n, lost_h, sight_h, room_h) -> list:
    """The frame indices that would have fired a deliberation call, each with
    the reasons that fired it."""
    fired = []
    last_room = None
    absent_run = seen_run = 0
    believed_visible = lost_pending = False
    room_cand, room_run = None, 0

    for i, e in enumerate(entries):
        nav = e.get("navigate") or {}
        why = []

        if i == 0:
            why.append("start")

        if nav.get("target_visible") is True:
            seen_run += 1
            absent_run = 0
        else:
            absent_run += 1
            seen_run = 0
        if not believed_visible and seen_run >= sight_h:
            believed_visible = True
            lost_pending = True
            why.append("sighting")
        if believed_visible and lost_pending and absent_run >= lost_h:
            believed_visible = lost_pending = False
            why.append("lost_target")

        room = nav.get("room_guess")
        if room:
            room_run = room_run + 1 if room == room_cand else 1
            room_cand = room
            if room_run >= room_h and room_cand != last_room:
                if last_room is not None:
                    why.append("room_change")
                last_room = room_cand

        if nav.get("target_reached") is True:
            why.append("goal_achieved")

        # Staleness is a floor, not an extra: it only fires when nothing else
        # did, so a busy walk never pays for it.
        if not why and (not fired or i - fired[-1][0] >= stale_n):
            why.append("staleness")

        if why:
            fired.append((i, why))
    return fired


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[3])
    ap.add_argument("--dir", help="directory of cached walk JSON dumps")
    ap.add_argument("--stale-n", type=int, default=8)
    ap.add_argument("--lost-h", type=int, default=2)
    ap.add_argument("--sight-h", type=int, default=1)
    ap.add_argument("--room-h", type=int, default=1)
    args = ap.parse_args()

    if args.dir:
        walks = load_walks(args.dir)
    else:
        base = os.environ.get("ADMIN_URL")
        secret = os.environ.get("ADMIN_SHARED_SECRET", "")
        if not base:
            print("Set ADMIN_URL (and ADMIN_SHARED_SECRET), or pass --dir.", file=sys.stderr)
            return 2
        walks = fetch_walks(base, secret)

    walks = [w for w in walks if w.get("entries")]
    if not walks:
        print("No walks with entries found.", file=sys.stderr)
        return 1
    total = sum(len(w["entries"]) for w in walks)
    print(f"\n{len(walks)} walks, {total} recorded frames "
          f"(lost_h={args.lost_h} sight_h={args.sight_h} room_h={args.room_h})\n")

    print("Staleness sweep -- deliberation calls as a fraction of reactive steps")
    print(f"{'stale_n':>8}{'triggers':>10}{'ratio':>8}{'saving':>9}")
    for stale_n in (4, 6, 8, 10, 12, 16, 20):
        t = sum(len(triggers_for(w["entries"], stale_n, args.lost_h,
                                 args.sight_h, args.room_h)) for w in walks)
        print(f"{stale_n:>8}{t:>10}{t / total:>8.3f}{total / t:>8.1f}x")

    kinds = collections.Counter()
    fired_total = 0
    rows = []
    for w in walks:
        fired = triggers_for(w["entries"], args.stale_n, args.lost_h,
                             args.sight_h, args.room_h)
        fired_total += len(fired)
        for _, why in fired:
            kinds.update(why)
        rows.append((w["walk"], len(w["entries"]), len(fired)))

    print(f"\nTrigger kinds at stale_n={args.stale_n}")
    for kind, count in kinds.most_common():
        print(f"  {kind:<14}{count:>5}  ({count / fired_total:.1%} of triggers)")
    print(f"  {'TOTAL':<14}{fired_total:>5}  over {total} frames "
          f"-> {total / fired_total:.1f}x fewer cloud calls")

    # The stability of the two fields the policy triggers on. If these are
    # noisy, the trigger count above is measuring noise -- which is the
    # reason --sight-h and --room-h exist.
    flips_v = flips_r = 0
    for w in walks:
        prev_v = prev_r = None
        for e in w["entries"]:
            nav = e.get("navigate") or {}
            visible = nav.get("target_visible") is True
            room = nav.get("room_guess")
            if prev_v is not None and visible != prev_v:
                flips_v += 1
            if prev_r is not None and room and room != prev_r:
                flips_r += 1
            prev_v = visible
            if room:
                prev_r = room
    print("\nStability of the fields the policy reads")
    print(f"  target_visible flips : {flips_v} ({flips_v / total:.1%} of frames)")
    print(f"  room_guess   changes : {flips_r} ({flips_r / total:.1%} of frames)")

    print(f"\n{'walk':<58}{'n':>5}{'trig':>6}{'saving':>8}")
    for name, n, tr in sorted(rows, key=lambda r: -r[1]):
        print(f"{name:<58}{n:>5}{tr:>6}{n / tr:>7.1f}x")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
