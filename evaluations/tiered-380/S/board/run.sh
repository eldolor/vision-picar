#!/bin/bash
# 3.80.S: one resolution on the board. $1 orig|640|320. MemAvailable every
# second from before Florence's server starts (and before the runner loads
# YOLOE + CLIP) to the end, in ~/out-380S/$1.mem.
set -u
RES=$1; OUT=~/out-380S; V=~/vp-380R; mkdir -p $OUT
# Only this bench's own server (:8006, Florence): the board is shared.
MINE='native_server\.py -m florence:.* --port 8006'
pkill -f "$MINE" 2>/dev/null; sleep 3
( while :; do echo "$(date +%s) $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > $OUT/$RES.mem &
SAMP=$!
sleep 3
(setsid nohup $V/evaluations/tiered-380/R/nsrv.sh -m florence:$HOME/models-380R/fc-Florence-2-base --port 8006 > $OUT/$RES-server.log 2>&1 < /dev/null &)
ok=; for i in $(seq 150); do curl -s localhost:8006/health | grep -q '"status":"ok"' && grep -q "native_server: florence" $OUT/$RES-server.log && ok=1 && break; sleep 2; done
if [ -z "$ok" ]; then
  echo "$(date +%T) $RES server never healthy"; kill $SAMP; pkill -f "$MINE"; exit 1
fi
cd $V && .venv/bin/python evaluations/tiered-380/S/run_gated.py $RES --recordings ~/vision-picar/recordings \
  --out $OUT/$RES.jsonl > $OUT/$RES.run.log 2>&1
rc=$?
kill $SAMP; pkill -f "$MINE"
echo "$(date +%T) $RES exit $rc"
exit $rc
