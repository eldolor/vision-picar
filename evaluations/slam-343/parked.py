"""
python evaluations/slam-343/parked.py run.json [...]

PLAN-ros-alignment.md 3.43 criterion 2: the longest stretch a run's robot
stayed within 5 cm of one spot (ground truth, demo_slam_home's 1 Hz series),
leaving out the last 15 s (the instrument's own wait at rest). Over 120 s
fails. Also the tour's goals and 3.40's SLAM bars (judge.py).
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/../slam-340")
from judge import judge  # noqa: E402

PARKED_M, LIMIT_S = 0.05, 120.0


def longest_parked(series):
    s = [r for r in series if r[4] is not None]
    s = [r for r in s if r[0] <= s[-1][0] - 15]
    best, i = (0.0, None), 0
    for j in range(len(s)):
        while math.hypot(s[j][4] - s[i][4], s[j][5] - s[i][5]) > PARKED_M:
            i += 1
        if s[j][0] - s[i][0] > best[0]:
            best = (s[j][0] - s[i][0], (round(s[i][4], 2), round(s[i][5], 2)))
    return round(best[0]), best[1]


if __name__ == "__main__":
    for p in sys.argv[1:]:
        d = json.load(open(p))
        dur, where = longest_parked(d["series"])
        j = judge(p)
        ok_goals = sum(g == "succeeded" for g in (d.get("goal_states") or []))
        print(json.dumps({"run": os.path.basename(p), "longest_parked_s": dur, "at": where,
                          "parked_ok": dur < LIMIT_S, "goals_succeeded": ok_goals,
                          "slam_east_max_m": j["east_max_m"], "slam_final_m": j["final_m"],
                          "jumps": j["jumps"], "slam_pass": j["pass"]}))
