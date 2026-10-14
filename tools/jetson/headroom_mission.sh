#!/usr/bin/env bash
# 3.78 amendment 5, C (docs/plans/ros-alignment/3.78-trt-vs-llamacpp.md):
# memory through a whole mission with no VLM -- the transients a real
# mission adds on top of 3.58's window. Built from headroom_identity.sh
# (same stack, same teardown); see that file's header for the conditions.
# Differences: SIM_MAP (default home_first_floor), an `explore` mission for
# the red backpack instead of the scaled house's nav goals, and no VLM.
#
#   bash tools/jetson/headroom_mission.sh <out-dir>
#
# --- the shared harness header follows ---
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
#
# 3.78 amendment 4: RUNTIME=trt answers with TensorRT Edge-LLM through
# bench_identity_trt (ENGINES, the model's engines dir; EDGELLM, the built
# Edge-LLM tree). The twin's systemd units (3.71) are stopped after the
# multi-user isolate, which would otherwise start them (3.78's deviation).
set -uo pipefail
OUT="${1:?out dir}"; mkdir -p "$OUT"
IMAGE="${IMAGE:-vision-picar-ros:plan-3.58}"
NAME="${NAME:-picar-ros-358}"
BENCH="${BENCH:-$HOME/bench-348}"
RUNTIME=none; HOUSE="${HOUSE:-home_first_floor}"; LIMIT="${LIMIT:-1500}"
ENGINES="${ENGINES:-$HOME/bench-378/ws91/Qwen2.5-VL-3B-Instruct/engines}"
EDGELLM="${EDGELLM:-$HOME/bench-378/ell91}"
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
sudo systemctl stop picar-stack picar-ngrok 2>/dev/null; sleep 3
{ echo "== desktop: $(systemctl is-active gdm); twin units: $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); runtime $RUNTIME"
  echo "== run L $(date +%T) rev $(git rev-parse --short HEAD) image $IMAGE $(sudo nvpmodel -q | head -1)"
  echo "== MemAvailable before anything: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB"; } | tee "$OUT/run.log"
OOM0=$(oom)

docker rm -f "$NAME" >/dev/null 2>&1
SIM_MAP="$HOUSE" ROBOT_DRIVE=ros WORLD_MODE=ros ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake \
  bash service/tunnel/restart.sh >> "$OUT/run.log" 2>&1 || { echo "STOP: restart.sh failed" | tee -a "$OUT/run.log"; exit 2; }
waitfor 60 "wheels usable" sh -c "curl -s -H 'x-app-secret: $LOCAL_SECRET' localhost:8000/wheels | grep -q '\"usable\": *true'"
docker run -d --name "$NAME" --network host \
  -e APP_SHARED_SECRET -e ROBOT_URL=http://127.0.0.1:8000 -e BRAIN_URL=http://127.0.0.1:8001/brain \
  "$IMAGE" ros2 launch picar_bringup picar.launch.py >/dev/null
waitfor 180 "bridge up" curl -sf localhost:8090/health -o /dev/null
waitfor 120 "start_truth" sh -c "curl -s -H 'x-app-secret: $LOCAL_SECRET' localhost:8090/slam/pose | grep -q '\"start_truth\": {'"
echo "== stack up $(date +%T); MemAvailable with the stack: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"

sudo tegrastats --interval 1000 --logfile "$OUT/tegrastats.log" &
( while true; do echo "$(date +%s) $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > "$OUT/mem.log" &
MEM=$!
PYTHONPATH=. python tools/jetson/perception_loop.py 3600 > "$OUT/perception.log" 2> "$OUT/perception.err" &
PERC=$!
waitfor 300 "perception loaded" grep -q "loaded 1 pipeline" "$OUT/perception.log"
sleep 20   # perception warm before the mission (3.78.F's load-picture rule)
( while true; do echo "$(date +%T) $(wheel)"; sleep 5; done ) > "$OUT/wheel.log" &
WHL=$!
W0=$(wheel); T0=$(date +%s)
echo "== mission start $(date +%T); wheel: $W0" | tee -a "$OUT/run.log"
python - "$OUT" "$LIMIT" <<'PY' >> "$OUT/run.log" 2>&1
import json, sys, time, httpx, os
out, limit = sys.argv[1], int(sys.argv[2])
h = {"x-app-secret": os.environ["APP_SHARED_SECRET"]}
b = httpx.Client(base_url="http://127.0.0.1:8001/brain", timeout=60, headers=h)
# The config's own mission_time_limit_s (1200 s) bounds the mission; older
# brains reject a per-mission limit.
r = b.post("/mission/start", json={"target_object": "red backpack", "policy": "explore"})
print("start", r.status_code, r.text[:200], flush=True)
if r.status_code != 200:
    raise SystemExit("mission did not start")
t0, last = time.time(), None
with open(f"{out}/status.log", "w") as f:
    while time.time() - t0 < limit + 120:
        s = b.get("/mission/status").json()
        line = {"t": round(time.time()), "running": s.get("running"), "outcome": s.get("outcome"),
                "step": s.get("step")}
        f.write(json.dumps(line) + "\n"); f.flush()
        if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(2)
end = {k: s.get(k) for k in ("running", "outcome", "step")}
json.dump(end, open(f"{out}/mission_end.json", "w"))
print("mission end", json.dumps({k: s.get(k) for k in ("outcome", "step", "report")})[:600], flush=True)
if s.get("running"):
    # Cut off by the poll bound, not ended by the mission: not a whole mission.
    raise SystemExit("mission still running at the poll bound")
PY
MISSION_EXIT=$?
W1=$(wheel); T1=$(date +%s)
echo "== mission end $(date +%T); wheel: $W1" | tee -a "$OUT/run.log"
kill $PERC $MEM $WHL 2>/dev/null; PERC=; MEM=; WHL=; sudo pkill tegrastats
OOM1=$(oom)
python3 - "$OUT" "$T0" "$T1" "$W0" "$W1" "$OOM0" "$OOM1" "$MISSION_EXIT" <<'PY' | tee -a "$OUT/run.log"
import json, os, statistics as st, sys
out, t0, t1 = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
w0, w1 = sys.argv[4].split(), sys.argv[5].split()
mem = [tuple(map(int, l.split())) for l in open(f"{out}/mem.log") if len(l.split()) == 2]
win = [m for t, m in mem if t0 <= t <= t1]
end_path = f"{out}/mission_end.json"
end = json.load(open(end_path)) if os.path.exists(end_path) else {}
# A run is a whole-mission measurement only if the mission started, ended on
# its own, and the window holds at least two minutes of samples (review, 3.78).
valid = sys.argv[8] == "0" and len(win) >= 120 and not end.get("running")
if not win:
    win = [0]
first = win[:60]; last = win[-60:]
s = {"valid": valid, "mission_exit": int(sys.argv[8]), "mission_end": end,
     "window_s": t1 - t0, "mem_min_mb": min(win), "mem_median_mb": st.median(win),
     "mem_first_min_median_mb": st.median(first), "mem_last_min_median_mb": st.median(last),
     "drift_mb": st.median(first) - st.median(last),
     "oom_events": int(sys.argv[7]) - int(sys.argv[6]),
     "wheel_moving_ticks": int(w1[0]) - int(w0[0]), "wheel_late_ticks": int(w1[1]) - int(w0[1])}
json.dump(s, open(f"{out}/summary.json", "w"), indent=1)
print("== summary:", json.dumps(s))
if not valid:
    print("== NOT A VALID MISSION MEASUREMENT (see mission_exit / mission_end / window_s)")
PY
