#!/usr/bin/env bash
# 3.80.B acceptance B1-B6 (docs/plans/ros-alignment/3.80-tiered-identity.md)
# on the board, from a checkout of this branch:
#
#   bash tools/jetson/bench_local_vlm.sh <out-dir>
#
# Desktop and the twin's units off, 15 W, page cache dropped. Installs
# picar-local-vlm from THIS checkout (WorkingDirectory and python pointed
# here; not enabled), runs the frames, the deadline and the death checks,
# then removes the unit and restores the board. picar-stack's unit is not
# touched from a branch: B4's -900 is checked after the merge installs it.
set -uo pipefail
OUT="${1:?out dir}"; mkdir -p "$OUT"
HERE="$(pwd)"
ENGINES="${ENGINES:-$HOME/bench-378/ws91/q25VE/engines}"
EDGELLM="${EDGELLM:-$HOME/bench-378/ell91}"
WALKS=(--walk blue-bottle-20260907-142454 --walk blue-bottle-20260907-185007
       --walk blue-shoes-20260907-152528 --walk red-backpack-20260907-144856)
source .venv/bin/activate
set -a; source ~/.vision-picar-local-secrets; set +a
export APP_SHARED_SECRET="$LOCAL_SECRET"
UNIT=/etc/systemd/system/picar-local-vlm.service
memkb() { awk '/MemAvailable/{print $2}' /proc/meminfo; }
log() { echo "== $*" | tee -a "$OUT/run.log"; }
health() { curl -s -m 2 localhost:8005/health; }
ready() { health | grep -q '"ready": *true\|"ready":true'; }
waitready() {  # seconds
  local end=$(( $(date +%s) + $1 ))
  until ready; do [ "$(date +%s)" -ge "$end" ] && return 1; sleep 1; done
}
SAMPLER=
teardown() {
  [ -n "$SAMPLER" ] && kill $SAMPLER 2>/dev/null
  sudo systemctl stop picar-local-vlm 2>/dev/null
  sudo systemctl reset-failed picar-local-vlm 2>/dev/null
  sudo rm -f "$UNIT"; sudo systemctl daemon-reload
  sudo systemctl isolate graphical.target; sleep 5
  sudo systemctl start picar-stack picar-ngrok
  log "torn down; desktop $(systemctl is-active gdm); twin units $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok)"
}
trap teardown EXIT

sudo systemctl isolate multi-user.target; sleep 5
sudo systemctl stop picar-stack picar-ngrok 2>/dev/null; sleep 3
log "rev $(git rev-parse --short HEAD) $(sudo nvpmodel -q | head -1); desktop $(systemctl is-active gdm); twin $(systemctl is-active picar-stack)"

ENVF=$HOME/.vision-picar-local-vlm.env
( umask 077; printf 'LOCAL_VLM_ENGINES=%s\nLOCAL_VLM_EDGELLM=%s\nAPP_SHARED_SECRET=%s\nLOCAL_VLM_DRIVER_LOG=%s\n' \
    "$ENGINES" "$EDGELLM" "$LOCAL_SECRET" "$OUT/driver.log" > "$ENVF" )
sed -e "s#/home/picar/vision-picar-service/.venv/bin/python#$HERE/.venv/bin/python#" \
    -e "s#^WorkingDirectory=.*#WorkingDirectory=$HERE#" \
    -e "s#append:/home/picar/.vision-picar-local-vlm.log#append:$OUT/service.log#g" \
    tools/jetson/systemd/picar-local-vlm.service | sudo tee "$UNIT" >/dev/null
sudo systemctl daemon-reload

sync; echo 3 | sudo tee /proc/sys/vm/drop_caches >/dev/null; sleep 3
( while true; do echo "$(date +%s.%N) $(memkb)"; sleep 0.5; done ) > "$OUT/mem.log" &
SAMPLER=$!
sleep 5
echo "LOAD $(date +%s.%N)" > "$OUT/marks"
sudo systemctl start picar-local-vlm
waitready 300 || { log "STOP: service not ready"; exit 2; }
log "ready: $(health)"

# B4: the service and its driver.
for pid in $(systemctl show -p MainPID --value picar-local-vlm) $(pgrep -f edgellm_identity); do
  echo "pid $pid $(cat /proc/$pid/comm) oom_score_adj $(cat /proc/$pid/oom_score_adj)"
done | tee "$OUT/B4.txt"

# B1, B2 (B3 reads mem.log between these marks).
echo "START $(date +%s.%N)" >> "$OUT/marks"
python -m tools.jetson.bench_local_vlm frames --recordings recordings "${WALKS[@]}" \
  --out "$OUT/B1.json" > "$OUT/B1.log" 2>&1
echo "END $(date +%s.%N)" >> "$OUT/marks"
log "B1/B2: $(tail -1 "$OUT/B1.log")"

# B6.
python -m tools.jetson.bench_local_vlm deadline --recordings recordings \
  --walk blue-bottle-20260907-185007 --out "$OUT/B6.json" > "$OUT/B6.log" 2>&1
log "B6: $(tail -1 "$OUT/B6.log")"
kill $SAMPLER; SAMPLER=

# B5: kill the driver; /health must stop answering within 2 s, then the
# unit restarts (RestartSec 30) and is ready again. Then three more deaths
# inside 15 min: the start limit must leave it failed.
for n in 1 2 3 4; do
  drv=$(pgrep -f edgellm_identity | head -1)
  [ -z "$drv" ] && { log "B5 death $n: no driver running ($(systemctl is-active picar-local-vlm))"; break; }
  t0=$(date +%s.%N); sudo kill -9 "$drv"
  until ! health >/dev/null; do sleep 0.1; done
  gone=$(python3 -c "import time; print(round(time.time()-$t0,2))")
  log "B5 death $n: health gone after ${gone}s"
  sleep 35
  if waitready 300; then log "B5 death $n: restarted, ready"
  else log "B5 death $n: not restarted; unit $(systemctl is-active picar-local-vlm) $(systemctl show -p Result --value picar-local-vlm)"; fi
done
log "B5 end: unit $(systemctl is-active picar-local-vlm), NRestarts $(systemctl show -p NRestarts --value picar-local-vlm)"

python3 - "$OUT" <<'PY' | tee -a "$OUT/run.log"
import json, statistics as st, sys
out = sys.argv[1]
mk = {}
for l in open(f"{out}/marks"):
    k, v = l.split(); mk[k] = float(v)
m = [(float(a), int(b) / 1024) for a, b in (l.split() for l in open(f"{out}/mem.log") if len(l.split()) == 2)]
pre = [v for t, v in m if t < mk["LOAD"]]
dur = [v for t, v in m if mk["START"] <= t <= mk["END"]]
b1 = json.load(open(f"{out}/B1.json"))
print("== summary:", json.dumps({"mem_before_mb": st.median(pre), "mem_during_median_mb": st.median(dur),
      "steady_use_mb": st.median(pre) - st.median(dur), "peak_use_mb": st.median(pre) - min(v for _, v in m),
      "latency_ms": b1["latency_ms"], "statuses": b1["statuses"]}))
PY
