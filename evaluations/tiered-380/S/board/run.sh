#!/bin/bash
# 3.80.S: one resolution on the board. $1 orig|640|320; $2 florence (default)
# or ve (amendment 1); $3 the output tag (default $1, or $1-ve). MemAvailable
# every second from before the second model's server starts (and before the
# runner loads YOLOE + CLIP) to the end, in ~/out-380S/$TAG.mem.
set -u
RES=$1; SECOND=${2:-florence}; OUT=~/out-380S; V=~/vp-380R; mkdir -p $OUT
if [ "$SECOND" = ve ]; then TAG=${3:-$RES-ve}; else TAG=${3:-$RES}; fi
# Only this bench's own servers (:8006 Florence, :8007 VE): the board is shared.
# VE is stopped by its own pid, never by pattern: the production unit
# (picar-local-vlm) runs the same command line. It must be stopped too, or
# its memory would be in the "before" figure.
if [ "$SECOND" = ve ]; then
  ss -ltnH "sport = :8007" | grep -q . && { echo "STOP: :8007 is already in use"; exit 2; }
  systemctl is-active -q picar-local-vlm && { echo "STOP: picar-local-vlm is running"; exit 2; }
  stop() { [ -n "${VEP:-}" ] && kill $VEP 2>/dev/null && for _ in $(seq 40); do kill -0 $VEP 2>/dev/null || break; sleep 0.5; done; }
else
  MINE='native_server\.py -m florence:.* --port 8006'
  stop() { pkill -f "$MINE" 2>/dev/null; }
  stop; sleep 3
fi
( while :; do echo "$(date +%s) $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > $OUT/$TAG.mem &
SAMP=$!
sleep 3
if [ "$SECOND" = ve ]; then
  # The unit's settings by hand (headroom_mission.sh's VLM=service), on its own
  # port and with no secret, so the bench's requests need none.
  (cd $V && setsid nohup env -u APP_SHARED_SECRET LOCAL_VLM_PORT=8007 \
     LOCAL_VLM_ENGINES=$HOME/bench-378/ws91/q25VE/engines LOCAL_VLM_EDGELLM=$HOME/bench-378/ell91 \
     .venv/bin/python -m service.local_vlm.server > $OUT/$TAG-server.log 2>&1 < /dev/null &)
  for i in $(seq 20); do VEP=$(lsof -nP -tiTCP:8007 -sTCP:LISTEN 2>/dev/null | head -1); [ -n "$VEP" ] && break; sleep 1; done
  [ -n "$VEP" ] && echo 1000 | sudo tee /proc/$VEP/oom_score_adj >/dev/null
  ok=; for i in $(seq 150); do curl -s localhost:8007/health | grep -q '"ready": *true' && ok=1 && break; sleep 2; done
else
  (setsid nohup $V/evaluations/tiered-380/R/nsrv.sh -m florence:$HOME/models-380R/fc-Florence-2-base --port 8006 > $OUT/$TAG-server.log 2>&1 < /dev/null &)
  ok=; for i in $(seq 150); do curl -s localhost:8006/health | grep -q '"status":"ok"' && grep -q "native_server: florence" $OUT/$TAG-server.log && ok=1 && break; sleep 2; done
fi
if [ -z "$ok" ]; then
  echo "$(date +%T) $TAG server never healthy"; kill $SAMP; stop; exit 1
fi
cd $V && .venv/bin/python evaluations/tiered-380/S/run_gated.py $RES --second $SECOND --recordings ~/vision-picar/recordings \
  --out $OUT/$TAG.jsonl > $OUT/$TAG.run.log 2>&1
rc=$?
kill $SAMP; stop
echo "$(date +%T) $TAG exit $rc"
exit $rc
