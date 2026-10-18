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
# 3.81 amendment 6 (criterion 4): POLICY (default explore) is the mission's
# policy; under `tiered` the brain loads perception itself, so the stand-in
# perception loop is not run. VLM=service hand-starts the local VLM
# (`service.local_vlm`, VE; ENGINES, EDGELLM as below) on :8005, first in
# line for the OOM killer as its unit is, and points the brain at it
# (LOCAL_VLM_URL). CLOUD=off points the brain's cloud at a dead port, so no
# paid call is made. ROS_FROM_TREE=1 mounts this checkout's bridge and
# launch file over the image's (installed in develop mode), so an older
# image runs this checkout's ROS side without a rebuild. TEXT_CACHE=empty
# encodes the target's texts from scratch (the new-target worst case).
#
# 3.80.P: PERCEPTION_LOOP=1 runs the stand-in perception loop under `tiered`
# too (the simulator's tiered perception is ground truth and loads no model),
# and the summary reports its latency over the mission window and the local
# VLM's calls. SIM_PERCEPTION_BLIND and TIER_COLD_SEARCH_AFTER, when set,
# reach the brain through the environment (3.80.O's switch, the pace).
# Since the default change (the user's option a, after P's runs) a mission
# with the local VLM is paced by TIER_LOCAL_COLD_SEARCH_AFTER (default 1);
# P@6 is now reproduced with TIER_LOCAL_COLD_SEARCH_AFTER=6.
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
POLICY="${POLICY:-explore}"; VLM="${VLM:-none}"; CLOUD="${CLOUD:-on}"
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
  # Only this run's VLM: its server closes its own driver on the way out.
  kill $PERC $MEM $WHL $NAV $VLMP 2>/dev/null; sudo pkill tegrastats 2>/dev/null
  docker rm -f "$NAME" >/dev/null 2>&1
  pids="$(pgrep -f 'bash service/tunnel/run.sh'; for p in 8000 8001 8002 8003 8004 8080; do lsof -nP -tiTCP:$p -sTCP:LISTEN; done)"
  [ -n "$pids" ] && kill $pids 2>/dev/null; sleep 3
  pids="$(for p in 8000 8001 8002 8003 8004 8080; do lsof -nP -tiTCP:$p -sTCP:LISTEN; done)"
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  sudo systemctl isolate graphical.target
  echo "== torn down; desktop: $(systemctl is-active gdm)" | tee -a "$OUT/run.log"
}
PERC=; MEM=; WHL=; NAV=; VLMP=
trap teardown EXIT

sudo systemctl isolate multi-user.target; sleep 5
sudo systemctl stop picar-stack picar-ngrok picar-local-vlm 2>/dev/null; sleep 3
{ echo "== desktop: $(systemctl is-active gdm); twin units: $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); runtime $RUNTIME"
  echo "== run L $(date +%T) rev $(git rev-parse --short HEAD) image $IMAGE $(sudo nvpmodel -q | head -1)"
  echo "== policy $POLICY; vlm $VLM; cloud $CLOUD; ros from tree ${ROS_FROM_TREE:-0}; text cache ${TEXT_CACHE:-default}"
  echo "== MemAvailable before anything: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB"; } | tee "$OUT/run.log"
OOM0=$(oom)

docker rm -f "$NAME" >/dev/null 2>&1
[ "$CLOUD" = off ] && export VISION_URL=http://127.0.0.1:9
if [ "${TEXT_CACHE:-}" = empty ]; then
  export PICAR_TEXT_CACHE="$OUT/text-cache"; rm -rf "$PICAR_TEXT_CACHE"
fi
if [ "$VLM" = service ]; then
  # :8005 must be free, or the ready check below would pass on someone
  # else's VLM (another session's hand-started one).
  ss -ltn | grep -q ':8005 ' && { echo "STOP: :8005 is already in use" | tee -a "$OUT/run.log"; exit 2; }
  # The unit's own settings, by hand: same engines, same secret, OOM first.
  LOCAL_VLM_ENGINES="$ENGINES" LOCAL_VLM_EDGELLM="$EDGELLM" \
    python -m service.local_vlm.server > "$OUT/local_vlm.log" 2>&1 &
  VLMP=$!
  echo 1000 | sudo tee /proc/$VLMP/oom_score_adj >/dev/null
  waitfor 300 "local VLM ready" sh -c "kill -0 $VLMP && curl -s localhost:8005/health | grep -q '\"ready\": *true'"
  export LOCAL_VLM_URL=http://127.0.0.1:8005
  echo "== local VLM ready $(date +%T); MemAvailable: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"
fi
SIM_MAP="$HOUSE" ROBOT_DRIVE=ros WORLD_MODE=ros ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake \
  bash service/tunnel/restart.sh >> "$OUT/run.log" 2>&1 || { echo "STOP: restart.sh failed" | tee -a "$OUT/run.log"; exit 2; }
waitfor 60 "wheels usable" sh -c "curl -s -H 'x-app-secret: $LOCAL_SECRET' localhost:8000/wheels | grep -q '\"usable\": *true'"
ROS_MOUNT=()
if [ "${ROS_FROM_TREE:-0}" = 1 ]; then
  for f in picar_bridge/picar_bridge/bridge.py picar_bridge/picar_bridge/convert.py \
           picar_bringup/launch/picar.launch.py; do
    ROS_MOUNT+=(-v "$PWD/service/slam/src/$f:/ws/src/$f:ro")
  done
fi
docker run -d --name "$NAME" --network host "${ROS_MOUNT[@]}" \
  -e APP_SHARED_SECRET -e ROBOT_URL=http://127.0.0.1:8000 -e BRAIN_URL=http://127.0.0.1:8001/brain \
  "$IMAGE" ros2 launch picar_bringup picar.launch.py >/dev/null
waitfor 180 "bridge up" curl -sf localhost:8090/health -o /dev/null
waitfor 120 "start_truth" sh -c "curl -s -H 'x-app-secret: $LOCAL_SECRET' localhost:8090/slam/pose | grep -q '\"start_truth\": {'"
echo "== stack up $(date +%T); MemAvailable with the stack: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"

sudo tegrastats --interval 1000 --logfile "$OUT/tegrastats.log" &
( while true; do echo "$(date +%s) $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > "$OUT/mem.log" &
MEM=$!
if [ "$POLICY" != tiered ] || [ "${PERCEPTION_LOOP:-0}" = 1 ]; then
  PYTHONPATH=. python tools/jetson/perception_loop.py 3600 > "$OUT/perception.log" 2> "$OUT/perception.err" &
  PERC=$!
  waitfor 300 "perception loaded" grep -q "loaded 1 pipeline" "$OUT/perception.log"
  sleep 20   # perception warm before the mission (3.78.F's load-picture rule)
fi
( while true; do echo "$(date +%T) $(wheel)"; sleep 5; done ) > "$OUT/wheel.log" &
WHL=$!
W0=$(wheel); T0=$(date +%s)
echo "== mission start $(date +%T); wheel: $W0" | tee -a "$OUT/run.log"
POLICY="$POLICY" python - "$OUT" "$LIMIT" <<'PY' >> "$OUT/run.log" 2>&1
import json, sys, time, httpx, os
out, limit = sys.argv[1], int(sys.argv[2])
h = {"x-app-secret": os.environ["APP_SHARED_SECRET"]}
b = httpx.Client(base_url="http://127.0.0.1:8001/brain", timeout=60, headers=h)
# The config's own mission_time_limit_s (1200 s) bounds the mission; older
# brains reject a per-mission limit.
r = b.post("/mission/start", json={"target_object": "red backpack", "policy": os.environ["POLICY"]})
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
end = {k: s.get(k) for k in ("running", "outcome", "step", "end_reason", "arrival", "tier", "error")}
json.dump(end, open(f"{out}/mission_end.json", "w"))
print("mission end", json.dumps({k: s.get(k) for k in ("outcome", "step", "report")})[:600], flush=True)
if s.get("running"):
    # Cut off by the poll bound, not ended by the mission: not a whole mission.
    raise SystemExit("mission still running at the poll bound")
PY
MISSION_EXIT=$?
W1=$(wheel); T1=$(date +%s)
echo "== mission end $(date +%T); wheel: $W1" | tee -a "$OUT/run.log"
VLM_ALIVE=none
[ -n "$VLMP" ] && { kill -0 $VLMP 2>/dev/null && VLM_ALIVE=alive || VLM_ALIVE=dead; }
echo "== local VLM at the end: $VLM_ALIVE $(curl -s localhost:8005/health 2>/dev/null)" | tee -a "$OUT/run.log"
kill $PERC $MEM $WHL $VLMP 2>/dev/null; PERC=; MEM=; WHL=; VLMP=; sudo pkill tegrastats
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
# 3.80.P: perception's latency inside the mission window, and VE's calls.
import time as _t
day = _t.strftime("%Y-%m-%d", _t.localtime(t0))
ms = []
if os.path.exists(f"{out}/perception.log"):
    for l in open(f"{out}/perception.log"):
        f = l.split()
        if len(f) == 3 and f[1].isdigit() and f[0].count(":") == 2:
            at = _t.mktime(_t.strptime(f"{day} {f[0]}", "%Y-%m-%d %H:%M:%S"))
            if t0 <= at <= t1:
                ms.append(int(f[1]))
if ms:
    ms.sort()
    q = lambda p: ms[min(len(ms) - 1, int(p * len(ms)))]
    s.update(perception_frames=len(ms), perception_p50_ms=q(0.5),
             perception_p90_ms=q(0.9), perception_p99_ms=q(0.99))
if os.path.exists(f"{out}/local_vlm.log"):
    calls = sum('"POST /' in l for l in open(f"{out}/local_vlm.log"))
    s.update(vlm_calls=calls, vlm_calls_per_min=round(calls * 60 / max(1, t1 - t0), 1))
json.dump(s, open(f"{out}/summary.json", "w"), indent=1)
print("== summary:", json.dumps(s))
if not valid:
    print("== NOT A VALID MISSION MEASUREMENT (see mission_exit / mission_end / window_s)")
PY
