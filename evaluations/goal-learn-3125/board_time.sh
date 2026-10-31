#!/usr/bin/env bash
# 3.125.B: 3.109's board script (3.103's, its arms trimmed). Runs ON the board
# from ~/bench-3125-time (a checkout of the branch):
#   1. quiet: the board as found (picar-stack serving the twin, sim mode), all arms
#   2. full:  headroom_identity.sh's window (split sim, robot server + safety on
#      the fake ESP32, brain, ROS container with SLAM + nav2 driving goals,
#      YOLOE + CLIP at 4 Hz, Qwen VE answering frames, 15 W, desktop off);
#      the choice bench starts when the window starts and must end inside it
#   3. the twin's units restarted (the harness stops them and leaves them down)
set -uo pipefail
cd ~/bench-3125-time
OUT=out; mkdir -p "$OUT"
source .venv/bin/activate
log() { echo "$(date +%T) $*" | tee -a "$OUT/driver.log"; }
log "== quiet: $(systemctl is-active picar-stack) picar-stack, $(sudo nvpmodel -q | head -1)"
python evaluations/goal-learn-3103/jetson_time.py quiet "$OUT/quiet.json" > "$OUT/quiet.log" 2>&1
log "quiet exit $?"

log "== full stack"
RUNTIME=trt ENGINES="$HOME/bench-378/ws91/q25VE/engines" IMAGE=vision-picar-ros:latest \
  NAME=picar-ros-3125 bash tools/jetson/headroom_identity.sh "$OUT/full" > "$OUT/harness.log" 2>&1 &
H=$!
until grep -q "== window start" "$OUT/full/run.log" 2>/dev/null; do
  kill -0 $H 2>/dev/null || { log "harness ended before its window"; break; }
  sleep 2
done
if grep -q "== window start" "$OUT/full/run.log" 2>/dev/null; then
  log "window started; choice bench"
  ARMS=F-large,F-pool-W32 python evaluations/goal-learn-3103/jetson_time.py full "$OUT/full.json" > "$OUT/full.log" 2>&1
  log "full exit $?; window ended yet: $(grep -c '== window end' "$OUT/full/run.log")"
fi
wait $H; log "harness exit $?"
sudo systemctl start picar-stack picar-ngrok
sleep 15; log "twin units: $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); local VLM $(systemctl is-active picar-local-vlm)"
log "DONE"
