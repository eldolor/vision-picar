#!/bin/bash
# 3.80.S: one resolution on the board. $1 orig|640|320. MemAvailable every
# second from before Florence's server starts (and before the runner loads
# YOLOE + CLIP) to the end, in ~/out-380S/$1.mem.
set -u
RES=$1; OUT=~/out-380S; V=~/vp-380R; mkdir -p $OUT
pkill -f native_server.py 2>/dev/null; sleep 3
( while :; do echo "$(date +%s) $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > $OUT/$RES.mem &
SAMP=$!
sleep 3
(setsid nohup $V/evaluations/tiered-380/R/nsrv.sh -m florence:$HOME/models-380R/fc-Florence-2-base --port 8006 > $OUT/$RES-server.log 2>&1 < /dev/null &)
ok=; for i in $(seq 150); do curl -s localhost:8006/health | grep -q ok && ok=1 && break; sleep 2; done
if [ -z "$ok" ]; then
  echo "$(date +%T) $RES server never healthy"; kill $SAMP; pkill -f native_server.py; exit 1
fi
cd $V && .venv/bin/python evaluations/tiered-380/S/run_gated.py $RES --recordings ~/vision-picar/recordings \
  --out $OUT/$RES.jsonl > $OUT/$RES.run.log 2>&1
rc=$?
kill $SAMP; pkill -f native_server.py
echo "$(date +%T) $RES exit $rc"
exit $rc
