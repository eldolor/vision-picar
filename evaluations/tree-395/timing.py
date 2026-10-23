"""
evaluations/tree-395/timing.py

3.95 criterion 4: what each of the tiered policy's three trees costs a frame,
on py_trees, measured inside the policy's own frame loop. The pipeline is
3.94's scripted stand-in (`tests/test_tier_tree._ScriptPipe`: random
perceptions, Stops landing mid-frame) and the cloud answers at once, so the
time is the trees', not a model's. Prints JSON: p50 / p90 / max in ms per
tree, and for the three together per frame.

    .venv/bin/python evaluations/tree-395/timing.py [--frames 20000]
"""

import argparse
import json
import platform
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from brain.tiered import TieredVision          # noqa: E402
from tests.test_tier_tree import _ScriptPipe   # noqa: E402


def _q(v):
    v = sorted(v)
    return {"p50": round(statistics.median(v), 4),
            "p90": round(statistics.quantiles(v, n=10)[-1], 4),
            "max": round(v[-1], 4), "n": len(v)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=20_000)
    a = ap.parse_args()
    times = {"what": [], "when": [], "who": []}
    frame_total = []
    for seed in range(a.frames // 500):
        tier = TieredVision(_ScriptPipe(seed), lambda f: {})
        tier.pipeline.tier = tier
        this_frame = {}
        for name in times:
            tree = getattr(tier, f"_{name}_tree")
            tick = tree.tick

            def timed(bb=..., _tick=tick, _name=name):
                if bb is ...:          # py_trees' own call (Composite.stop): its generator
                    return _tick()
                t0 = time.perf_counter()
                try:
                    return _tick(bb)
                finally:
                    this_frame[_name] = this_frame.get(_name, 0.0) + (time.perf_counter() - t0) * 1000
            tree.tick = timed
        for seq in range(500):
            this_frame.clear()
            try:
                tier({"seq": seq})
            except Exception:  # noqa: BLE001 -- a stand-in Stop mid-frame may end a call; timing only
                pass
            for name, ms in this_frame.items():
                times[name].append(ms)
            if this_frame:
                frame_total.append(this_frame.get("when", 0.0) + this_frame.get("who", 0.0))
    # The what tree is ticked INSIDE the who tree (its routes build the
    # scene through `_local_scene`), so a frame's total is when + who, and
    # the what tree's own row is part of the who tree's.
    print(json.dumps({"host": platform.node(), "python": platform.python_version(),
                      **{k: _q(v) for k, v in times.items()},
                      "frame_all_trees": _q(frame_total)}, indent=1))


if __name__ == "__main__":
    main()
