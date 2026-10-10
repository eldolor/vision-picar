#!/usr/bin/env bash
# 3.46 amendment 9: the namer's memory with the 3.33 headroom stack running.
# usage: bash mem346.sh <tag> <model.litertlm> <gpu|cpu> [window_s=720]
# Baseline = mean MemAvailable over the 60 s before the namer starts (stack up,
# perception at 4 Hz, nav2 goals running). Resident = max(baseline - min
# MemAvailable while the namer runs, namer's peak VmRSS).
set -uo pipefail
TAG=$1; MODEL=$2; DEV=$3; WIN=${4:-720}
L=~/litert-346/mem-$TAG; mkdir -p $L
cd ~/vision-picar
source .venv/bin/activate
set -a; source ~/.vision-picar-local-secrets; set +a
export APP_SHARED_SECRET="$LOCAL_SECRET"
H=(-H "x-app-secret: $LOCAL_SECRET")
sudo systemctl isolate multi-user.target; sleep 5
echo "== $TAG $(date +%T) rev $(git rev-parse --short HEAD) $(sudo nvpmodel -q | head -1) desktop $(systemctl is-active gdm)"
docker rm -f picar-ros >/dev/null 2>&1
SIM_MAP=scaled_house ROBOT_DRIVE=ros WORLD_MODE=ros ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake \
  bash service/tunnel/restart.sh || { echo "STOP: restart.sh failed"; sudo systemctl isolate graphical.target; exit 2; }
until curl -s "${H[@]}" localhost:8000/wheels | grep -q '"usable": *true'; do sleep 0.5; done
docker run -d --name picar-ros --restart unless-stopped --network host \
  -e APP_SHARED_SECRET -e ROBOT_URL=http://127.0.0.1:8000 -e BRAIN_URL=http://127.0.0.1:8001/brain \
  vision-picar-ros ros2 launch picar_bringup picar.launch.py >/dev/null
until curl -s localhost:8090/health >/dev/null; do sleep 1; done
until curl -s "${H[@]}" localhost:8090/slam/pose | grep -q '"start_truth": {'; do sleep 0.5; done
SIM_MAP=scaled_house python - <<'PY'
from tests import demo_nav_goals as nav
nav.map_first(nav.client())
PY
base=$(curl -s "${H[@]}" localhost:8000/health | python3 -c 'import sys,json; w=json.load(sys.stdin)["wheel_loop"]; print(w["moving_ticks"], w["late_ticks"])')
echo "== stack up and mapped $(date +%T); wheel_loop (moving late) $base"

( while true; do echo "$(date +%s) $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > $L/memavail.log &
MEM=$!
sudo tegrastats --interval 1000 --logfile $L/tegrastats.log &
PYTHONPATH=. python ~/perception_loop.py $((WIN + 400)) > $L/perception.log 2>&1 &
PERC=$!
SIM_MAP=scaled_house python - $((WIN + 300)) <<'PY' > $L/nav.log 2>&1 &
import sys, time
from tests import demo_nav_goals as nav
robot = nav.client()
end, n, ok = time.time() + float(sys.argv[1]), 0, 0
while time.time() < end:
    for name, x, y in nav.GOALS:
        if time.time() >= end:
            break
        r = nav.run_goal(robot, x, y)
        n += 1; ok += r["state"] == "succeeded"
        print(f"{time.strftime('%T')} {name}: {r['state']} {r['end_error_m']:.2f} m", flush=True)
print(f"goals {ok}/{n} succeeded")
PY
NAV=$!
sleep 90                                   # perception warm, nav moving
T_START=$(date +%s)
echo "== namer start $(date +%T)"
cd ~/litert-346/vp
RECORDINGS_DIR=~/litert-346/rec ~/litert-346/venv/bin/python -m tools.inventory_vlm name --backend litert \
  --model "$MODEL" --device "$DEV" --tag "${TAG}_stack" --frames ~/litert-346/frames450.json > $L/namer.out 2> $L/namer.log &
NAMER=$!
( while kill -0 $NAMER 2>/dev/null; do echo "$(date +%s) $(awk '/VmRSS|VmHWM/{printf "%d ", $2/1024}' /proc/$NAMER/status 2>/dev/null)"; sleep 1; done ) > $L/namer_rss.log &
sleep $WIN
kill $NAMER 2>/dev/null; sleep 3
T_END=$(date +%s)
echo "== namer stopped $(date +%T)"
wait $NAV
kill $PERC $MEM 2>/dev/null; sudo pkill tegrastats
cd ~/vision-picar
echo "== nav: $(tail -1 $L/nav.log)"
echo "== wheel_loop now: $(curl -s "${H[@]}" localhost:8000/health | python3 -c 'import sys,json; w=json.load(sys.stdin)["wheel_loop"]; print(w["moving_ticks"], w["late_ticks"], w["max_dt_s"])') (baseline $base)"
python3 - $L $T_START $T_END <<'PY'
import sys, re, statistics as st
L, t0, t1 = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
av = [tuple(map(int, l.split())) for l in open(f"{L}/memavail.log") if len(l.split()) == 2]
base = st.mean(m for t, m in av if t0 - 60 <= t < t0)
during = [m for t, m in av if t0 <= t <= t1]
rss = [list(map(int, l.split()[1:])) for l in open(f"{L}/namer_rss.log") if len(l.split()) == 3]
peak_rss = max(r[1] for r in rss) if rss else 0
drop = base - min(during)
frames = [l for l in open(f"{L}/namer.log") if " boxes rss/hwm" in l]
secs = sorted(float(l.split()[2]) for l in frames)
print(f"== baseline MemAvailable {base:.0f} MB; min during namer {min(during)} MB; drop {drop:.0f} MB")
print(f"== namer peak VmHWM {peak_rss} MB; resident = max(drop, VmHWM) = {max(drop, peak_rss)} MB (bar 3328 MB = 3.25 GB); floor 1 GB {'held' if min(during) >= 1024 else 'BROKEN'}")
if secs:
    print(f"== frames under the stack: {len(secs)}; median {secs[len(secs)//2]:.1f} s, p90 {secs[int(len(secs)*0.9)]:.1f} s")
temps, thr = [], 0
for line in open(f"{L}/tegrastats.log"):
    t = [float(x) for x in re.findall(r"(?:cpu|gpu|tj)@([\d.]+)C", line)]
    if t: temps.append(max(t))
    thr += bool(re.search(r"throttl", line, re.I))
print(f"== temperature max {max(temps):.1f} C; throttling lines {thr}")
p = sorted(int(l.split()[1]) for l in open(f"{L}/perception.log") if len(l.split()) == 3)
print(f"== perception median {p[len(p)//2]} ms p90 {p[int(len(p)*0.9)]} ms over {len(p)} frames")
PY
docker rm -f picar-ros >/dev/null 2>&1
pkill -f "bash service/tunnel/run.sh" 2>/dev/null
for p in 8000 8001 8002 8003 8004 8080; do kill $(lsof -nP -tiTCP:$p -sTCP:LISTEN 2>/dev/null) 2>/dev/null; done
sleep 3; echo "== stack stopped; ports still held: $(for p in 8000 8001 8002 8003 8004 8080; do lsof -nP -tiTCP:$p -sTCP:LISTEN 2>/dev/null; done | wc -l)"
sudo systemctl isolate graphical.target
echo "== desktop restored: $(systemctl is-active gdm)"
