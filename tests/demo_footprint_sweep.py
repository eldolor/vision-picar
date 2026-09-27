"""
tests/demo_footprint_sweep.py -- 3.18 part 1's instrument, in full
(PLAN-ros-alignment.md 3.18).

The seeded sweep `tests/test_footprint_safety.py` pins a sample of: every
house, `--starts` random starts per house near something, 24 headings,
forward and reverse, a standing 0.1 m/s command through the wheel loop's own
two calls, judged on ground truth. Prints each criterion's verdict and the
worst runs. No server, no ROS, no model.

    python -m tests.demo_footprint_sweep [--starts 40] [--seed 0] [--unclamped]
"""

import argparse
import time

from tests import footprint_sweep as fs

HOUSES = ("starter_house", "scaled_house", "home_first_floor")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--starts", type=int, default=40)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--unclamped", action="store_true",
                    help="criterion 4: no safety clamp, judge the sim's own collision")
    args = ap.parse_args()
    t0 = time.time()
    results = fs.sweep(HOUSES, args.starts, args.seed, clamp=not args.unclamped)
    print(f"{len(results)} runs in {time.time() - t0:.0f}s "
          f"({args.starts} starts x {fs.HEADINGS} headings x 2 directions x {len(HOUSES)} houses)")
    if args.unclamped:
        bad = [r for r in results if r["max_P"] > fs.PENETRATION_BAR_CM]
        worst = max(r["max_P"] for r in results)
        print(f"4 sim collision: {'PASS' if not bad else 'FAIL'} -- {len(bad)} runs deeper than "
              f"{fs.PENETRATION_BAR_CM}cm; deepest {worst:.2f}cm")
        return
    for name, (ok, detail) in fs.verdicts(results).items():
        print(f"{name}: {'PASS' if ok else 'FAIL'} -- {detail}")
    moved = [r for r in results if r["T_after_move"] != float("inf")]
    if moved:
        print(f"closest approach after a move: T {min(r['T_after_move'] for r in moved):.1f}cm; "
              f"closest gap in any run {min(r['min_G'] for r in results):.2f}cm")
    for house in HOUSES:
        rs = [r for r in results if r["house"] == house]
        eligible = [r for r in rs if r["T0"] >= fs.PROGRESS_START_CM]
        print(f"  {house}: {len(rs)} runs, travel {sum(r['travel'] for r in rs) / len(rs):.1f}cm mean, "
              f"{sum(r['travel'] >= fs.PROGRESS_MIN_CM for r in eligible)}/{len(eligible)} eligible covered "
              f"{fs.PROGRESS_MIN_CM}cm")


if __name__ == "__main__":
    main()
