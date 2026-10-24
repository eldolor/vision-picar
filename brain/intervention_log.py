"""
intervention_log.py -- 3.99: did a person step in, and how often per hour?

**The record.** One JSON line per mission end, interventions or not -- the
missions nobody touched are the rate's denominator -- appended to one file
(`<intervention_dir>/interventions.jsonl`), durably, by 3.88's appender. A
mission holds at most one intervention, because every one ends it today: a
person's Stop (`stop`, through the brain's `POST /mission/stop` or 3.84's
latch on the robot server) or a person taking the wheel (`dpad`, the D-pad
or the phone's tap-to-goal preempting the brain). A 3.88 goal pick answers
a question the robot asked, so it is counted beside the intervention
(`person_picks`), never as one; an autonomous preemption and B3.3's abort
are faults, never a person's.

The context kept is what a learner would need to ask "what was it doing
when the person stepped in": the step, the last action, explore's last
event, the mission second, and the pose in the house frame -- read after
the robot's stop, so under a D-pad the person may already have moved it
(`pose_after_stop`). Positions only, never an image or a map, and the file
stays on the machine.

**The report.** `python -m brain.intervention_log [dir]`: missions,
autonomous hours (elapsed minus 3.88's pauses), interventions by kind, and
interventions per hour, overall and per policy. A last line cut by a power
loss is skipped (`read_choices`' rule).
"""

import argparse
import sys
from pathlib import Path
from typing import Optional

from brain.choice_log import ChoiceLog, read_choices

FILE_NAME = "interventions.jsonl"
# The kinds, as `cancelled_by` names them (3.70 E5).
KINDS = ("stop", "dpad")
# How an intervention reached the brain.
VIA_BRAIN_STOP = "brain_stop"      # POST /mission/stop
VIA_ROBOT_STOP = "robot_stop"      # 3.84: the robot server's latch refused the brain
VIA_PREEMPTED = "preempted"        # a person outranked the brain (M4)


class InterventionLog(ChoiceLog):
    """The one file every mission's end line is appended to. Same durable
    append as a goal choice's: created on the first line, each line fsynced."""


def read_missions(path) -> list:
    path = Path(path)
    return read_choices(path) if path.exists() else []


def _tally(lines: list) -> dict:
    seconds = sum(float(x.get("autonomous_s") or 0.0) for x in lines)
    kinds = [(x.get("intervention") or {}).get("kind") for x in lines]
    count = sum(1 for k in kinds if k is not None)
    hours = seconds / 3600.0
    return {"missions": len(lines), "hours": round(hours, 3), "interventions": count,
            **{k: kinds.count(k) for k in KINDS},
            "per_hour": round(count / hours, 3) if hours > 0 else None,
            "person_picks": sum(int(x.get("person_picks") or 0) for x in lines)}


def summarise(lines: list) -> dict:
    """Counts and the rate over every line, and per policy."""
    policies = sorted({str(x.get("policy")) for x in lines})
    return {"all": _tally(lines),
            "by_policy": {p: _tally([x for x in lines if str(x.get("policy")) == p])
                          for p in policies}}


def _row(name: str, t: dict) -> str:
    rate = f"{t['per_hour']:.2f} per hour" if t["per_hour"] is not None else "no time recorded"
    return (f"{name}: {t['missions']} missions, {t['hours']:.2f} h of autonomy, "
            f"{t['interventions']} interventions (stop {t['stop']}, dpad {t['dpad']}), "
            f"{rate}; {t['person_picks']} goal picks asked for")


def report(directory) -> str:
    lines = read_missions(Path(directory) / FILE_NAME)
    if not lines:
        return "no missions recorded"
    s = summarise(lines)
    return "\n".join([_row("all", s["all"])]
                     + [_row(f"  {p}", t) for p, t in s["by_policy"].items()])


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("directory", nargs="?", default="recordings/interventions")
    args = parser.parse_args(argv)
    print(report(args.directory))


if __name__ == "__main__":
    main(sys.argv[1:])
