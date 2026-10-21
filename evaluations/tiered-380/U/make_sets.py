"""
evaluations/tiered-380/U/make_sets.py

3.80.U: the steering sets for another camera (the Rover's), drawn by 3.80.I's
own rule (`I/make_sets.draw`, seed 380) from the walks named, before any model
runs:

* S1: up to 60 frames labelled target-visible, at most 8 a walk;
* S2: up to 40 frames labelled not visible, at most 4 a walk;
* no S3 (the renders are the simulator's and were scored in 3.80.I).

Writes `<out>/sets.json` and `<out>/labels-s1.json`, whose `truth` (the third
holding the target's centre, 3.80.I's labelling) is left null to be filled by
viewing each frame. `U/score.py` refuses to score while any is null. Point the
harnesses at the set with `PICAR_380_SETS=<out>` (and `PICAR_380_RECORDINGS`).

    .venv/bin/python evaluations/tiered-380/U/make_sets.py --out evaluations/tiered-380/U/rover \\
        --recordings recordings --walk <w> [--walk <w> ...] [--n1 60 --n2 40]
"""

import argparse
import json
import random
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent / "I"))
from make_sets import SEED, draw                            # noqa: E402


def pools(recordings: Path, walks):
    vis, nov = [], []
    for name in walks:
        walk = recordings / name
        L = json.loads((walk / "labels.json").read_text())
        for f, v in sorted(L["target_visible_labels"].items()):
            if (walk / f).is_file():
                (vis if v else nov).append((name, f, L["description"]))
    return vis, nov


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--recordings", required=True)
    ap.add_argument("--walk", action="append", required=True)
    ap.add_argument("--n1", type=int, default=60)
    ap.add_argument("--n2", type=int, default=40)
    a = ap.parse_args(argv)
    out = Path(a.out)
    if (out / "labels-s1.json").exists():
        sys.exit(f"{out}/labels-s1.json exists: a drawn set is never redrawn over its labels")
    rng = random.Random(SEED)
    vis, nov = pools(Path(a.recordings), sorted(a.walk))
    s1, s2 = draw(vis, a.n1, 8, rng), draw(nov, a.n2, 4, rng)
    out.mkdir(parents=True, exist_ok=True)
    doc = {"seed": SEED, "walks": sorted(a.walk), "pool": {"visible": len(vis), "not_visible": len(nov)},
           "S1": s1, "S2": s2}
    (out / "sets.json").write_text(json.dumps(doc, indent=1) + "\n")
    labels = {"how": "To be labelled by viewing each frame with the thirds drawn, before any model "
                     "runs (3.80.I's rule): truth = the third holding the target's centre.",
              "rows": [{**r, "truth": None} for r in s1]}
    (out / "labels-s1.json").write_text(json.dumps(labels, indent=1) + "\n")
    print(json.dumps({"S1": len(s1), "S1_walks": len({r["walk"] for r in s1}),
                      "S2": len(s2), "S2_walks": len({r["walk"] for r in s2}), "pool": doc["pool"]}))


if __name__ == "__main__":
    main()
