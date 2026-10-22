"""3.76 K5.3's live tour: 3.85's two 13-room batches and R6, re-run on `dev`
with a mission's keepout marks cleared at its end
(docs/plans/ros-alignment/3.76-legacy-review.md, row K5.3).

    python evaluations/keepout-clear-k53/score.py [LOGDIR]   # default /tmp/vision-picar-3.76

Reads rooms-batch{1,2}.jsonl and r6.out, and each run's robot-server log
(`explore_robot.log`, and the brain's `explore_brain.log` for the end; one run directory per stack, in start order: the 26
rooms, then R6). Per room: the outcome, contact samples, marks placed, how
many were still live at the mission's end (a mark lives SPOT_TTL_S, 300 s)
and the server's "N mark(s) cleared at a mission's end" line. The clear
line is owed after every mission whose marks were live at its end; a
mission whose marks all expired first has nothing to clear and the server
logs nothing (`world_keepout_clear` returns before logging)."""
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOGDIR = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/vision-picar-3.76")
TTL_S = 300.0
MARK = re.compile(r"^(\d+\.\d+) .*keepout: marked")
ENDED = re.compile(r"^(\d+\.\d+) .*mission_runner mission ended")
CLEAR = re.compile(r"^(\d+\.\d+) .*keepout: (\d+) mark\(s\) cleared at a mission's end(.*)")


def run_log(d):
    marks, clears = [], []
    for line in (d / "explore_robot.log").read_text(errors="replace").splitlines():
        if m := MARK.match(line):
            marks.append(float(m.group(1)))
        elif m := CLEAR.match(line):
            clears.append((float(m.group(1)), int(m.group(2)), m.group(3).strip()))
    ended = [float(m.group(1)) for line in (d / "explore_brain.log").read_text(errors="replace").splitlines()
             if (m := ENDED.match(line))]
    return marks, clears, (ended[0] if ended else None)


def main():
    rows = []
    for b in (1, 2):
        p = HERE / f"rooms-batch{b}.jsonl"
        rows += [dict(json.loads(l), batch=b) for l in p.read_text().splitlines() if l.strip()]
    runs = sorted(LOGDIR.glob("run-*"))
    found = contacts = owed = shown = 0
    for row, d in zip(rows, runs):
        marks, clears, end = run_log(d)
        # Live at the end: placed within TTL_S of the brain's "mission
        # ended" line (no such line: every mark counts as owed).
        live = [t for t in marks if end is None or end - t < TTL_S]
        need = bool(live)
        found += row.get("outcome") == "found"
        contacts += row.get("contact_samples") or 0
        owed += need
        shown += need and bool(clears)
        print(f"b{row['batch']} {row.get('room', '?'):12s} {row.get('outcome')!s:14s} "
              f"{row.get('seconds')!s:>5}s contacts {row.get('contact_samples')} marks {len(marks)} live@end {len(live)} "
              f"clear {[(n, x) for _, n, x in clears] or '-'}  {d.name}"
              f"{'  INVALID ' + row['invalid'] if row.get('invalid') else ''}")
    print(f"found {found}/{len(rows)}; contact samples {contacts}; "
          f"clear line after {shown} of {owed} missions with live marks at the end")
    r6 = HERE / "r6.out"
    if r6.exists():
        txt = r6.read_text()
        ok = len(re.findall(r'"state": "succeeded"', txt))
        km = re.search(r"keepout marks (\d+)", txt)
        print(f"R6: {ok}/6 succeeded; keepout marks {km.group(1) if km else '?'}")


if __name__ == "__main__":
    main()
