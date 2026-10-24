#!/usr/bin/env bash
# 3.80.E's board runs (the plan's "3.80.E on the board", amendment 1), one
# after another, each a fresh tools/jetson/headroom_mission.sh stack. Run on
# the Jetson from a worktree of this branch, under the board lock:
#
#   PICAR_BOARD_TOKEN=... bash evaluations/tiered-380/E/board_runs.sh <out-root> [run ...]
#
# Runs: A1 (home_first_floor, the loaded window), A2-A6 (the five houses that
# arrive soonest), B (kill drill), C (OOM drill), D (VE's box latency under
# load). Default: all, in that order. A run's directory that exists is kept
# and the run skipped.
set -uo pipefail
ROOT="${1:?out root}"; shift
RUNS=("$@"); [ ${#RUNS[@]} -eq 0 ] && RUNS=(A1 A2 A3 A4 A5 A6 B C D)
export IMAGE=vision-picar-ros:plan-3.80 NAME=picar-ros-380e
export ENGINES="$HOME/bench-378/ws91/q25VE/engines" VLM=service PERCEPTION_LOOP=1
export TIER_SEARCH=explore
declare -A HOUSES=([A2]=procthor:val:21 [A3]=procthor:val:5 [A4]=procthor:val:27
                   [A5]=procthor:val:0 [A6]=procthor:val:26)
for run in "${RUNS[@]}"; do
  out="$ROOT/$run"
  [ -e "$out" ] && { echo "== $run: $out exists, skipped"; continue; }
  echo "== $run start $(date +%T)"
  rc=0
  case "$run" in
    A1) HOUSE=home_first_floor POLICY=tiered CLOUD=off bash tools/jetson/headroom_mission.sh "$out";;
    A[2-6]) HOUSE="${HOUSES[$run]}" POLICY=tiered CLOUD=off bash tools/jetson/headroom_mission.sh "$out";;
    B) HOUSE=home_first_floor POLICY=tiered CLOUD=standin DRILL=kill:120 bash tools/jetson/headroom_mission.sh "$out";;
    C) HOUSE=home_first_floor POLICY=tiered CLOUD=standin DRILL=oom:120 bash tools/jetson/headroom_mission.sh "$out";;
    D)
      # The box question under load: the mission is plain explore (it asks
      # VE nothing), and the client starts once the mission has.
      ( until grep -q "== mission start" "$out/run.log" 2>/dev/null; do sleep 2; done
        set -a; source ~/.vision-picar-local-secrets; set +a
        APP_SHARED_SECRET="$LOCAL_SECRET" PICAR_RECORDINGS="$PWD/recordings" \
          .venv/bin/python evaluations/tiered-380/E/d10_latency.py 900 "$out/d10_latency.jsonl" \
          > "$out/d10_latency.log" 2>&1 ) &
      CLIENT=$!
      HOUSE=home_first_floor POLICY=explore CLOUD=off LIMIT=1200 bash tools/jetson/headroom_mission.sh "$out"
      kill $CLIENT 2>/dev/null; pkill -f "E/d10_latency.py" 2>/dev/null;;
    *) echo "== $run: unknown run"; continue;;
  esac
  echo "== $run end $(date +%T)"
done
