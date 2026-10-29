#!/usr/bin/env bash
# evaluations/room-prior-3116/live.sh -- 3.116 criterion 6: three live find
# missions on home_first_floor (nav2, the live local stack, through the
# brain's HTTP API) with find_room_labels: frame and explore_goal_policy:
# rooms. Run on the workstation from the repo root with the .venv active;
# needs Docker and vision-picar-ros:latest (or ROS_IMAGE). One JSON line per
# run, tagged with this batch, is appended to
# evaluations/room-prior-3116/live/runs.jsonl; a run that crashes or prints
# no result is recorded as one that failed, never skipped.
# Bars: 3/3 found, 0 contact samples, room_labels.label_calls <= 20 each.
set -uo pipefail
cd "$(git rev-parse --show-toplevel)"
eval "$(python tools/plan_section.py env 2>/dev/null || true)"
OUT=evaluations/room-prior-3116/live/runs.jsonl
mkdir -p "$(dirname "$OUT")"
BATCH="$(date +%Y%m%dT%H%M%S)-$(git rev-parse --short HEAD)"
for i in 1 2 3; do
  line="$(FIND_ROOM_LABELS=frame EXPLORE_GOAL_POLICY=rooms PYTHONPATH="$PWD" \
          python tests/demo_explore.py one | grep '^{' | tail -1)"
  code=$?
  LINE="$line" CODE="$code" RUN="$i" BATCH="$BATCH" python -c '
import json, os, time
try:
    r = json.loads(os.environ["LINE"])
except ValueError:
    r = {"outcome": "error", "error": "no result line", "contact_samples": None}
r.update(run=int(os.environ["RUN"]), batch=os.environ["BATCH"], exit=int(os.environ["CODE"]),
         at=time.strftime("%Y-%m-%dT%H:%M:%S"))
print(json.dumps(r))' | tee -a "$OUT"
done
BATCH="$BATCH" OUT="$OUT" python - <<'PY'
import json, os
rows = [r for r in map(json.loads, open(os.environ["OUT"])) if r.get("batch") == os.environ["BATCH"]]
for r in rows:
    rl = r.get("room_labels") or {}
    print(r["run"], r["outcome"], "contacts", r.get("contact_samples"),
          "asks", rl.get("label_calls"), "seconds", r.get("seconds"))
ok = len(rows) == 3 and all(
    r["outcome"] == "found" and r.get("contact_samples") == 0
    and (r.get("room_labels") or {}).get("label_calls") is not None
    and r["room_labels"]["label_calls"] <= 20 for r in rows)
print("criterion 6:", "met" if ok else "FAILED", "(batch", os.environ["BATCH"] + ")")
PY
