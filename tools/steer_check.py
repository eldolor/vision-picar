"""
tools/steer_check.py

Does the policy ACT on what the local tier sees?

Phase G's claim, checked against recorded frames rather than a live walk.
The defect it was built for is a measurement: on
`woven-laundry-basket-20260912-231358`, across the frames where the local
tier reported the target DETECTED, the robot went **RIGHT 28, FORWARD 4,
LEFT 5**. The bearing was computed, written to
`_navigate.target_direction`, and never reached `safest_direction`.

So the number to move is the action distribution **conditioned on
perception status**, and that is what this prints. Aggregate action counts
cannot show it: a walk is mostly frames where nothing is detected, and
those swamp the ones that matter.

The cloud is **stubbed**. The question is what the stand-in does on a free
frame when perception has a bearing, and no VLM is involved in that. It
also keeps a diagnostic from costing money per run.

**What this cannot tell you.** Replay is open loop: the frames are fixed,
so steering differently cannot change what the robot would have seen next.
It answers "does a sighting steer" and not "does steering arrive". Only a
live walk answers the second.

    python -m tools.steer_check recordings/<walk> "woven laundry basket"
"""

from __future__ import annotations

import collections
import sys
from pathlib import Path

from brain.perceive import DETECTED
from brain.tiered import TieredVision, _direction_for
from control.perception_eval import frame_dict


class StubCloud:
    """A fixed answer, so the only thing that varies is the stand-in."""

    def __init__(self, direction="RIGHT"):
        self.direction = direction
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        return {"obstacles_ahead": [], "free_space": "clear",
                "doorway_visible": False, "important_objects": [],
                "safest_direction": self.direction,
                "_navigate": {"reasoning": "stub", "target_visible": False,
                              "target_reached": False}}


def run(walk: Path, target: str, *, steer: bool, device=None):
    from brain.perceive import pipeline_for

    pipeline = pipeline_for(target, floor_mask=True)
    cloud = StubCloud()
    tier = TieredVision(pipeline, cloud, steer_on_sight=steer,
                        cold_search_after=6, stale_after=0)
    seen = collections.Counter()
    acts = collections.Counter()
    agree = 0
    frames = sorted(walk.glob("frame-*.jpg"))
    for i, path in enumerate(frames):
        out = tier(frame_dict(path))
        perc = out["_perception"]["status"]
        seen[perc] += 1
        if perc == DETECTED:
            acts[out["safest_direction"]] += 1
            # Derived from the perception's own bearing rather than read
            # off `_navigate`: on a PAID frame that block is the cloud's
            # reply, which carries no bearing at all. The question here is
            # about the measured one.
            b = out["_perception"].get("bearing_deg")
            want = ("unknown" if b is None
                    else "left" if b < -10 else "right" if b > 10 else "center")
            got = out["safest_direction"]
            if (want == "left" and got == "LEFT") or \
               (want == "right" and got == "RIGHT") or \
               (want == "center" and got == "FORWARD"):
                agree += 1
        if (i + 1) % 40 == 0:
            print(f"    {i+1}/{len(frames)}", flush=True)
    tier.close()
    return seen, acts, agree


def main(argv=None) -> int:
    argv = argv or sys.argv[1:]
    if len(argv) < 2:
        print(__doc__)
        return 2
    walk, target = Path(argv[0]), argv[1]
    print(f"walk: {walk.name}\ntarget: {target!r}\n")
    for steer in (False, True):
        label = "steer_on_sight=" + ("ON " if steer else "OFF")
        print(f"=== {label} " + "=" * 40, flush=True)
        seen, acts, agree = run(walk, target, steer=steer)
        det = seen.get(DETECTED, 0)
        print(f"  perception: {dict(seen)}")
        print(f"  actions ON DETECTED frames: {dict(acts)}")
        # The one number. "The action matched the measured bearing" is the
        # whole of Phase G, and the pre-change value on this walk was 4 of
        # 37 -- and those four were a coincidence, not a decision.
        print(f"  action matched the bearing: {agree}/{det}"
              f"  ({(agree / det * 100) if det else 0:.0f}%)\n", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
