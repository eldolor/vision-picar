#!/bin/bash
# 3.80.R: one model end to end, from the Mac. $1 name, $2 the board
# environment (e.g. 'NATIVE_SPEC=florence:/path NSRV_PY=...' or
# 'MODEL=.. MMPROJ=.. LLBIN=.. LLEXTRA=--special'), $3 identity arms
# ("A", "N:<backend>:<skill>", space-separated), $4 steering runs
# ("A" or "N:<backend>:<skill>"; each a separate R/<name>[-<skill>].jsonl).
set -u
NAME=$1; ENVS=$2; IDS=$3; STEERS=$4
HERE="$(cd "$(dirname "$0")" && pwd)"; ROOT="$HERE/../../.."
B='cd ~/vp-380R/evaluations/tiered-380/R/board'
ssh picar-jetson "pgrep -f '^[^ ]*bash [^ ]*/board/activity\.sh\$' >/dev/null || (setsid nohup ~/vp-380R/evaluations/tiered-380/R/board/activity.sh >/dev/null 2>&1 </dev/null &)"
for a in $IDS; do
  arm=${a%%:*}; nat=""; [ "$arm" = N ] && nat=${a#N:}
  echo "$(date +%T) $NAME identity $a"
  ssh picar-jetson "$B && env $ENVS ./identity.sh $NAME${nat:+-${nat#*:}} $arm $nat"
done
for s in $STEERS; do
  arm=${s%%:*}; nat=""; [ "$arm" = N ] && nat=${s#N:}
  out=$NAME; [ -n "$nat" ] && out=$NAME-${nat#*:}
  echo "$(date +%T) $NAME steering $s -> $out"
  ssh picar-jetson "$B && env $ENVS ./serve.sh $out" || { echo "serve failed"; continue; }
  ssh -f -N -o ExitOnForwardFailure=yes -L 18006:127.0.0.1:8006 picar-jetson
  (cd "$ROOT" && .venv/bin/python evaluations/tiered-380/R/steer.py "$out" http://127.0.0.1:18006 ${nat:+--native $nat} \
     > "$HERE/$out.run.log" 2>&1)
  rc=$?
  echo "$(date +%T) $out steer exit $rc"
  pkill -f "18006:127.0.0.1:8006"
  ssh picar-jetson "pkill -x llama-server; pkill -f native_server.py; true"
done
echo "$(date +%T) $NAME DONE"
