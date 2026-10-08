"""
python evaluations/slam-345/judge.py run.json [run.json ...]

PLAN-ros-alignment.md 3.45's live judge for demo_slam_home's explore-then-tour
runs. It keeps 3.43's bar unchanged -- the longest stretch the robot stayed
within 5 cm of one spot over the WHOLE run (`slam-343/parked.py`) -- and adds
what 3.44 could not say: in which phase each stall happened (explore, or the
tour and which of its goals), and what explore did there, read off the run's
own logs (3.45's per-run directory; older runs have none).

A stall that begins in explore and runs on into the tour is explore's: the
tour inherited a wedged robot (3.44's run 3: 0 of 9 goals).
"""

import glob
import gzip
import json
import math
import os
import re
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "slam-343"))
sys.path.insert(0, os.path.join(HERE, "..", "slam-340"))
from judge import judge as slam_judge  # noqa: E402
from parked import LIMIT_S, PARKED_M, longest_parked  # noqa: E402

TAIL_S = 15                         # the instrument's own wait at rest (parked.py)


def stalls(series, min_s=30.0):
    """Every maximal stretch within PARKED_M of where it began, >= min_s:
    (start_t, end_t, (x, y)). The longest of them is parked.py's number."""
    s = [r for r in series if r[4] is not None]
    if not s:
        return []
    s = [r for r in s if r[0] <= s[-1][0] - TAIL_S]
    out, i = [], 0
    while i < len(s):
        j = i
        while j + 1 < len(s) and math.hypot(s[j + 1][4] - s[i][4], s[j + 1][5] - s[i][5]) <= PARKED_M:
            j += 1
        if s[j][0] - s[i][0] >= min_s:
            out.append((s[i][0], s[j][0], (round(s[i][4], 2), round(s[i][5], 2))))
        i = j + 1
    return out


def longest_window(series):
    """parked.py's own window (the bar), with when it began."""
    s = [r for r in series if r[4] is not None]
    s = [r for r in s if r[0] <= s[-1][0] - TAIL_S]
    best, i = (0.0, None, None), 0
    for j in range(len(s)):
        while math.hypot(s[j][4] - s[i][4], s[j][5] - s[i][5]) > PARKED_M:
            i += 1
        if s[j][0] - s[i][0] > best[0]:
            best = (s[j][0] - s[i][0], (round(s[i][4], 2), round(s[i][5], 2)), s[i][0], s[j][0])
    return best


def phase_at(d, t):
    ex = (d.get("explore") or {})
    t0 = d.get("t0") or d["series"][0][0]
    tour_at = d.get("tour_started_at") or (t0 + d["tour_started_s"] if d.get("tour_started_s") else None)
    if tour_at is None or t < tour_at:
        return "explore"
    for k, g in enumerate(d.get("goals") or []):
        if g["started_at"] <= t <= g["ended_at"]:
            return f"tour goal {k + 1} ({g['name']})"
    return "tour"


def explore_events(run_dir):
    """The brain's `explore {json}` lines, with their wall times."""
    out = []
    for p in glob.glob(os.path.join(run_dir or "", "explore_brain.log")):
        for line in open(p, errors="replace"):
            m = re.match(r"(\d+\.\d+) INFO explore explore (\{.*\})$", line.strip())
            if m:
                out.append((float(m.group(1)), json.loads(m.group(2))))
    return out


def refusals(run_dir, t_from, t_to):
    """Robot-server refusals in a window, by driver and reason."""
    c = Counter()
    for p in glob.glob(os.path.join(run_dir or "", "explore_robot.log*")):
        opener = gzip.open if p.endswith(".gz") else open
        for line in opener(p, "rt", errors="replace"):
            m = re.match(r"(\d+\.\d+) WARNING server refused \((\w+)\) for (\S+): (\w+)", line)
            if m and t_from <= float(m.group(1)) <= t_to:
                c[f"{m.group(3)} {m.group(4)} {m.group(2)}"] += 1
    return dict(c.most_common(6))


def judge(path):
    d = json.load(open(path))
    run_dir = d.get("run_dir")
    ev = explore_events(run_dir)
    out_stalls = []
    for a, b, where in stalls(d["series"]):
        rec = {"seconds": round(b - a), "at": where, "phase": phase_at(d, a),
               "ends_in": phase_at(d, b)}
        if ev:
            inside = [e for t, e in ev if a - 5 <= t <= b]
            rec["explore"] = dict(Counter(e["event"] + (":" + e.get("action", "") if e["event"] == "verb" else "")
                                          for e in inside))
            before = [e for t, e in ev if t < a and e["event"] == "goal_sent"]
            rec["goal_before"] = before[-1] if before else None
        if run_dir:
            rec["refused"] = refusals(run_dir, a, b)
        out_stalls.append(rec)
    dur, where, a, b = longest_window(d["series"])
    assert (round(dur), where) == longest_parked(d["series"])   # the bar is 3.43's, unchanged
    j = slam_judge(path)
    goals = d.get("goal_states") or []
    ex = d.get("explore") or {}
    by_kind = Counter()
    for _t, e in ev:
        if e["event"] == "goal_ended":
            by_kind[f"{e['kind']} {e['state']}"] += 1
    return {"run": os.path.basename(path),
            "longest_parked_s": round(dur), "at": where, "parked_ok": dur < LIMIT_S,
            "longest_phase": phase_at(d, a) if a else None,
            "longest_ends_in": phase_at(d, b) if b else None,
            "stalls_over_60s": [s for s in out_stalls if s["seconds"] >= 60],
            "explore_coverage": ex.get("coverage"), "explore_outcome": ex.get("outcome"),
            "explore_goal_ends": dict(by_kind),
            "escapes": sum(e["event"] == "escape" for _t, e in ev),
            "goals_succeeded": sum(g == "succeeded" for g in goals), "goal_states": goals,
            "slam_east_max_m": j["east_max_m"], "slam_final_m": j["final_m"],
            "jumps": j["jumps"], "slam_pass": j["pass"]}


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(json.dumps(judge(p)))
