"""
tests/demo_verb_sweep.py -- `PLAN-ros-alignment.md` 3.22's full sweep.

Every straight and turn verb from seeded starts in all three houses, judged
on ground truth (`tests/verb_sweep.py`). `tests/test_guarded_verbs.py` pins a
smaller seeded sample of the same thing.

    python -m tests.demo_verb_sweep [starts_per_house] [headings]
"""

import logging
import sys

from tests import verb_sweep as vs


def main():
    logging.disable(logging.WARNING)     # every refused verb logs a line
    starts = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    headings = int(sys.argv[2]) if len(sys.argv) > 2 else 24
    straight = vs.straight_sweep(starts_per_house=starts, headings=headings)
    turns = vs.turn_sweep(starts_per_house=starts * 2)
    verbs = [v for r in straight for v in r["verbs"]]
    print(f"{len(straight)} straight runs ({len(verbs)} verbs), {len(turns)} turns")
    print(f"closest travel-to-contact after any verb: "
          f"{min(v['T_after'] for v in verbs):.1f} cm")
    print(f"closest gap, any run: {min(r['min_G'] for r in straight + turns):.2f} cm")
    for name, (ok, detail) in vs.verdicts(straight, turns).items():
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")


if __name__ == "__main__":
    main()
