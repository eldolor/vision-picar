#!/usr/bin/env bash
# 3.58 (docs/plans/ros-alignment/3.58-local-identity-decision.md), run L:
# 3.33's headroom conditions (~/headroom20.sh on the board: the split
# simulator, the robot server's 20 Hz wheel loop on the fake ESP32, the
# brain, the ROS container with SLAM + nav2, nav2 goals round the scaled
# house, ONE perception pipeline at 4 Hz, desktop off, 15 W) PLUS
# llama-server with Qwen2.5-VL-3B resident, answering the 299 frames under
# the stopping rule for the whole window. Run on the board from a checkout
# of this branch:
#
#   bash tools/jetson/headroom_identity.sh <out-dir>
#
# Env: IMAGE (default vision-picar-ros:plan-3.58), NAME (picar-ros-358),
# BENCH (~/bench-348). Tears the stack down afterwards and restores the
# desktop, so the board is left as found.
set -uo pipefail
OUT="${1:?out dir}"; mkdir -p "$OUT"
IMAGE="${IMAGE:-vision-picar-ros:plan-3.58}"
NAME="${NAME:-picar-ros-358}"
BENCH="${BENCH:-$HOME/bench-348}"
source .venv/bin/activate
set -a; source ~/.vision-picar-local-secrets; set +a
export APP_SHARED_SECRET="$LOCAL_SECRET"
H=(-H "x-app-secret: $LOCAL_SECRET")
wheel() { curl -s "${H[@]}" localhost:8000/health | python3 -c 'import sys,json; w=json.load(sys.stdin)["wheel_loop"]; print(w["moving_ticks"], w["late_ticks"], w["max_dt_s"])'; }
oom() { sudo dmesg | grep -ciE "out of memory|oom-kill|killed process" || true; }
# Every wait is bounded, so a stack that never comes up reaches teardown.
waitfor() {  # seconds what cmd...
  local s=$1 what=$2; shift 2
  local end=$(( $(date +%s) + s ))
  until "$@" 2>/dev/null; do
    [ "$(date +%s)" -ge "$end" ] && { echo "STOP: $what not reached in ${s}s" | tee -a "$OUT/run.log"; exit 3; }
    sleep 1
  done
}

teardown() {
  kill $PERC $MEM $WHL $NAV 2>/dev/null; sudo pkill tegrastats 2>/dev/null
  docker rm -f "$NAME" >/dev/null 2>&1
  pids="$(pgrep -f 'bash service/tunnel/run.sh'; for p in 8000 8001 8002 8003 8004 8080; do lsof -nP -tiTCP:$p -sTCP:LISTEN; done)"
  [ -n "$pids" ] && kill $pids 2>/dev/null; sleep 3
  pids="$(for p in 8000 8001 8002 8003 8004 8080; do lsof -nP -tiTCP:$p -sTCP:LISTEN; done)"
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  sudo systemctl isolate graphical.target
  echo "== torn down; desktop: $(systemctl is-active gdm)" | tee -a "$OUT/run.log"
}
PERC=; MEM=; WHL=; NAV=
trap teardown EXIT

sudo systemctl isolate multi-user.target; sleep 5
{ echo "== desktop: $(systemctl is-active gdm)"
  echo "== run L $(date +%T) rev $(git rev-parse --short HEAD) image $IMAGE $(sudo nvpmodel -q | head -1)"
  echo "== MemAvailable before anything: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB"; } | tee "$OUT/run.log"
OOM0=$(oom)

docker rm -f "$NAME" >/dev/null 2>&1
SIM_MAP=scaled_house ROBOT_DRIVE=ros WORLD_MODE=ros ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake \
  bash service/tunnel/restart.sh >> "$OUT/run.log" 2>&1 || { echo "STOP: restart.sh failed" | tee -a "$OUT/run.log"; exit 2; }
waitfor 60 "wheels usable" sh -c "curl -s -H 'x-app-secret: $LOCAL_SECRET' localhost:8000/wheels | grep -q '\"usable\": *true'"
docker run -d --name "$NAME" --network host \
  -e APP_SHARED_SECRET -e ROBOT_URL=http://127.0.0.1:8000 -e BRAIN_URL=http://127.0.0.1:8001/brain \
  "$IMAGE" ros2 launch picar_bringup picar.launch.py >/dev/null
waitfor 180 "bridge up" curl -sf localhost:8090/health -o /dev/null
waitfor 120 "start_truth" sh -c "curl -s -H 'x-app-secret: $LOCAL_SECRET' localhost:8090/slam/pose | grep -q '\"start_truth\": {'"
echo "== stack up $(date +%T)" | tee -a "$OUT/run.log"
SIM_MAP=scaled_house python - <<'EOF' >> "$OUT/run.log" 2>&1
from tests import demo_nav_goals as nav
nav.map_first(nav.client())
EOF
echo "== mapped $(date +%T); MemAvailable with the stack: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"

sudo tegrastats --interval 1000 --logfile "$OUT/tegrastats.log" &
( while true; do echo "$(date +%s) $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > "$OUT/mem.log" &
MEM=$!
PYTHONPATH=. python tools/jetson/perception_loop.py 3600 > "$OUT/perception.log" 2> "$OUT/perception.err" &
PERC=$!
waitfor 180 "perception loaded" grep -q "loaded 1 pipeline" "$OUT/perception.log"
( while true; do echo "$(date +%T) $(wheel)"; sleep 5; done ) > "$OUT/wheel.log" &
WHL=$!
touch "$OUT/nav.run"
SIM_MAP=scaled_house python - "$OUT/nav.run" <<'EOF' > "$OUT/nav.log" 2>&1 &
import os, sys, time
from tests import demo_nav_goals as nav
flag, robot, n, ok = sys.argv[1], nav.client(), 0, 0
while os.path.exists(flag):
    for name, x, y in nav.GOALS:
        if not os.path.exists(flag):
            break
        # A goal that is not accepted (no end_error_m) or a client timeout
        # under load is a failed goal, never the end of the nav load.
        try:
            r = nav.run_goal(robot, x, y)
        except Exception as e:  # noqa: BLE001
            r = {"state": f"error {type(e).__name__}"}
        n += 1; ok += r["state"] == "succeeded"
        if r["state"] == "not_sent" or r["state"].startswith("error"):
            # A refused or failed request returns at once; pause so a dead
            # server never turns the nav load into a tight request loop.
            time.sleep(5)
        err = r.get("end_error_m")
        print(f"{time.strftime('%T')} {name}: {r['state']} "
              f"{'' if err is None else f'{err:.2f} m'}", flush=True)
print(f"goals {ok}/{n} succeeded", flush=True)
EOF
NAV=$!

W0=$(wheel); T0=$(date +%s)
echo "== window start $(date +%T); wheel (moving late max_dt): $W0" | tee -a "$OUT/run.log"
python -m tools.jetson.bench_identity_llama --stop-at-decision \
  --server "$BENCH/llama.cpp/build/bin/llama-server" \
  --model "$BENCH/models/Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf" \
  --mmproj "$BENCH/models/mmproj-Qwen2.5-VL-3B-Instruct-Q8_0.gguf" \
  --recordings recordings \
  --walk blue-bottle-20260907-142454 --walk blue-bottle-20260907-185007 \
  --walk blue-shoes-20260907-152528 --walk red-backpack-20260907-144856 \
  --out "$OUT/loaded.json" > "$OUT/bench.log" 2>&1
BENCH_EXIT=$?
W1=$(wheel); T1=$(date +%s)
rm -f "$OUT/nav.run"; wait $NAV 2>/dev/null; NAV=
echo "== window end $(date +%T) bench exit $BENCH_EXIT; wheel: $W1" | tee -a "$OUT/run.log"
kill $PERC $MEM $WHL 2>/dev/null; PERC=; MEM=; WHL=; sudo pkill tegrastats
OOM1=$(oom)

python3 - "$OUT" "$T0" "$T1" "$W0" "$W1" "$OOM0" "$OOM1" <<'EOF' | tee -a "$OUT/run.log"
import json, re, sys
out, t0, t1 = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
w0, w1 = sys.argv[4].split(), sys.argv[5].split()
mem = [int(l.split()[1]) for l in open(f"{out}/mem.log")
       if len(l.split()) == 2 and t0 <= int(l.split()[0]) <= t1]
# Frame lines only: "HH:MM:SS <ms> <status>" (the loaded/done lines are not).
perc = [int(m.group(1)) for m in
        (re.fullmatch(r"\d\d:\d\d:\d\d (\d+) \w+", l.strip()) for l in open(f"{out}/perception.log"))
        if m]
s = sorted(perc); p90 = s[max(0, -(-len(s) * 9 // 10) - 1)] if s else None
nav = open(f"{out}/nav.log").read().strip().splitlines()
temps, throttle = [], 0
for line in open(f"{out}/tegrastats.log"):
    t = [float(x) for x in re.findall(r"(?:cpu|gpu|tj)@([\d.]+)C", line)]
    if t: temps.append(max(t))
    throttle += bool(re.search(r"throttl", line, re.I))
summary = {
    "window_s": t1 - t0,
    "mem_available_min_mb": min(mem) if mem else None, "mem_samples": len(mem),
    "oom_events": int(sys.argv[7]) - int(sys.argv[6]),
    "wheel_moving_ticks": int(w1[0]) - int(w0[0]), "wheel_late_ticks": int(w1[1]) - int(w0[1]),
    # max_dt_s is the server's running maximum since it started (mapping
    # lap included), not the window's; criterion 7 reads the late-tick
    # difference above.
    "wheel_max_dt_since_start_s": float(w1[2]),
    "perception_frames": len(perc), "perception_p90_ms": p90,
    "nav": nav[-1] if nav else None,
    "max_temp_c": max(temps) if temps else None, "throttle_lines": throttle,
}
json.dump(summary, open(f"{out}/summary.json", "w"), indent=1)
print("== summary:", json.dumps(summary))
EOF
