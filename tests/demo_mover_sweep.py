"""
python -m tests.demo_mover_sweep [starts_per_house] [seed]

PLAN-ros-alignment.md 3.30, criterion 2: the full ground-truth sweep with a
mover crossing the robot's path, in all three houses, at two speeds and 24
headings, clamped -- and, for contrast, the same runs with the clamp off.
"""

import sys
import time

from tests.mover_sweep import sweep, verdicts

HOUSES = ["starter_house", "scaled_house", "home_first_floor"]


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    for clamp in (True, False):
        t0 = time.time()
        results = sweep(HOUSES, n, seed=seed, clamp=clamp)
        print(f"\n== clamp {'ON' if clamp else 'OFF'}: {len(results)} runs, "
              f"{time.time() - t0:.0f} s ==")
        for bar, (ok, detail) in verdicts(results).items():
            print(f"  {'PASS' if ok else 'FAIL'}  {bar}: {detail}")
        for house in HOUSES:
            rs = [r for r in results if r["house"] == house]
            if rs:
                print(f"    {house}: {len(rs)} runs, mover within 30 cm in "
                      f"{sum(r['nearest_mover_cm'] <= 30 for r in rs)}, worst T after move "
                      f"{min(r['T_after_move'] for r in rs):.1f} cm, min gap "
                      f"{min(r['min_G'] for r in rs):.2f} cm")


if __name__ == "__main__":
    main()
