#!/bin/bash
# 3.80.R: one server on :8006 for the steering run (driven from the Mac over
# an ssh tunnel). $1 name; then the same environment as identity.sh.
pkill -x llama-server 2>/dev/null; pkill -f native_server.py 2>/dev/null; sleep 3
echo "$(date +%T) MemAvailable before $1: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)" >> ~/out-380R/$1.mem
if [ -n "$NATIVE_SPEC" ]; then
  (setsid nohup ~/vp-380R/evaluations/tiered-380/R/nsrv.sh -m "$NATIVE_SPEC" --port 8006 >> ~/out-380R/$1-server.log 2>&1 < /dev/null &)
else
  (setsid nohup ~/vp-380R/evaluations/tiered-380/R/board/srv.sh -m "$MODEL" --mmproj "$MMPROJ" -ngl 99 -c 4096 -np 1 --cache-ram 0 --port 8006 --host 127.0.0.1 >> ~/out-380R/$1-server.log 2>&1 < /dev/null &)
fi
ok=; for i in $(seq 300); do curl -s localhost:8006/health | grep -q ok && ok=1 && break; sleep 2; done
[ -n "$ok" ] || { echo "$(date +%T) $1 server never healthy" | tee -a ~/out-380R/$1.mem; exit 1; }
echo "$(date +%T) MemAvailable with $1: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)" | tee -a ~/out-380R/$1.mem
