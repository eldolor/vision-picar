#!/usr/bin/env bash
#
# The local half of the system, reachable from the DEPLOYED twin.
#
# The brain cannot be deployed: `policy: "tiered"` loads YOLO and CLIP into
# that process (PLAN-onboard-perception.md 4.10), and the brain has not been
# in AWS since 2026-09-05 anyway -- its home is the Pi (B5). So the twin runs
# from CloudFront over HTTPS and reaches back here through a tunnel.
#
#   bash service/tunnel/run.sh            # grid world (Sim tab)
#   ROBOT_MODE=teleop bash service/tunnel/run.sh   # phone on the rig
#
# Then, in another terminal:   ngrok start picar
# and put the URL it prints into the twin's Settings:
#
#   Robot server URL   https://<domain>
#   Brain service URL  https://<domain>/brain
#   Secret (both)      LOCAL_SECRET from ~/.vision-picar-local-secrets
#
# ONE tunnel, two services. ngrok's free plan gives a single static domain
# per account -- a second endpoint on it fails with ERR_NGROK_334 -- so the
# two share it by path behind proxy.py, using the same ROUTE_PREFIX
# mechanism that let them share one ALB on ECS.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1090
source .venv/bin/activate

SECRETS="$HOME/.vision-picar-local-secrets"
[ -f "$SECRETS" ] || { echo "missing $SECRETS -- see the header of this file"; exit 1; }
# shellcheck disable=SC1090
set -a; source "$SECRETS"; set +a
# The DEPLOYED services have their own secrets, in their own file --
# reviewing walks and metrics is a different privilege from driving the
# robot. Sourced when present so METRICS_SECRET below can be the walks
# one rather than the local one, which would 401.
SERVERLESS_SECRETS="$HOME/.vision-picar-serverless-secrets"
# shellcheck disable=SC1090
[ -f "$SERVERLESS_SECRETS" ] && { set -a; source "$SERVERLESS_SECRETS"; set +a; }

# Gates inbound calls to BOTH local servers. require_secret() is inert
# without it, and these are on the public internet through the tunnel.
export APP_SHARED_SECRET="$LOCAL_SECRET"
# The brain calling the robot. Same value: both are yours.
export ROBOT_SHARED_SECRET="$LOCAL_SECRET"
# The brain calling the deployed vision service, which has its OWN secret --
# set separately or the brain would send the one above and get a 401.
export VISION_URL="${VISION_URL:-https://d114x92g7i4syl.cloudfront.net}"
export VISION_SHARED_SECRET="$VISION_SECRET"
# One metrics row per mission, to the walks service (which owns the
# storage). Its own secret, like the vision one -- reviewing metrics is a
# different privilege from asking the model a question. The shipper can
# never fail a mission; see control/metrics_client.py.
export METRICS_URL="${METRICS_URL:-$VISION_URL}"
export METRICS_SECRET="${WALKS_SECRET:-}"
# 3.46: each mission's object inventory (labels and map positions, never an
# image) is uploaded to the private recordings bucket, read from the
# recordings stack's export. 3.55: `INVENTORY_BUCKET=` (set but empty)
# means local only and is never looked up. A failed lookup leaves the
# variable UNSET, not empty, so the brain falls back to config/robot.yaml
# rather than reading a failed lookup as the opt-out.
# --- inventory bucket (tests/test_inventory_mission.py runs this block) ---
if [ -z "${INVENTORY_BUCKET+set}" ]; then
  _bucket="$(aws cloudformation list-exports \
    --query "Exports[?Name=='vision-picar-recordings-s3-BucketName'].Value" \
    --output text 2>/dev/null || true)"
  if [ -n "$_bucket" ] && [ "$_bucket" != "None" ]; then
    export INVENTORY_BUCKET="$_bucket"
  fi
  unset _bucket
else
  export INVENTORY_BUCKET
fi
# --- end inventory bucket ---

export ROBOT_MODE="${ROBOT_MODE:-sim}"
# The world model must follow the body (N1). config/robot.yaml ships
# `world: sim` alongside `mode: sim`, and the sim world model maps a
# GridWorld -- so a teleop robot (a phone on a wheeled rig, no grid, no
# map) needs `none`. world/factory.py REFUSES the mismatch at start-up
# rather than returning a plausible map of a house the robot is not in,
# so without this line a teleop run dies on a ValueError.
if [ -z "${WORLD_MODE:-}" ]; then
  case "$ROBOT_MODE" in
    sim) export WORLD_MODE=sim ;;
    *)   export WORLD_MODE=none ;;
  esac
fi
echo "robot mode: $ROBOT_MODE   world: $WORLD_MODE   vision: $VISION_URL"

# Kill only what THIS script started. `kill 0` would have been shorter and
# signals the whole process group -- which on a normal setup includes the
# ngrok agent running in the same shell session, so stopping the servers
# would silently take the tunnel down with them and the phone would report
# a bare "Load failed" with nothing to point at.
pids=()
cleanup() { for pid in "${pids[@]:-}"; do kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

# 3.36: under SIM_MOTOR_BOARD=fake the simulated body runs as its own
# programs, started first -- physics + the fake board (sim/body_server.py)
# and the sensors (sim/sensor_server.py, SIM_SENSOR_WORKERS processes, so
# ray casting and rendering use the free cores). The robot server waits for
# both, then opens the board's pty as a serial device.
if [ "${SIM_MOTOR_BOARD:-}" = "fake" ]; then
  export SIM_BODY_URL="${SIM_BODY_URL:-http://127.0.0.1:8002}"
  # Two sensor programs by default: the robot server's safety stream on the
  # first, ROS's full scans and camera frames on the second (sim/body_client.py).
  export SIM_SENSORS_URL="${SIM_SENSORS_URL:-http://127.0.0.1:8003,http://127.0.0.1:8004}"
  export SIM_BODY_SHM="${SIM_BODY_SHM:-picar_sim_body}"
  body_port="${SIM_BODY_URL##*:}"; body_port="${body_port%%/*}"
  python -m uvicorn sim.body_server:app --port "$body_port" --host 127.0.0.1 --log-level warning &
  pids+=($!)
  echo "  body    $SIM_BODY_URL   (physics + fake board)"
  for u in ${SIM_SENSORS_URL//,/ }; do
    sensor_port="${u##*:}"; sensor_port="${sensor_port%%/*}"
    python -m uvicorn sim.sensor_server:app --port "$sensor_port" --host 127.0.0.1 --log-level warning &
    pids+=($!)
    echo "  sensors $u"
  done
  # 3.42: SIM_LIDAR=fake -- the D500 faked on a pty (sim/fake_lidar.py), read
  # by the robot server through the car's own driver (robot/lidar_ld19.py).
  # ROBOT_LIDAR is the link, as the udev name is on the car.
  if [ "${SIM_LIDAR:-}" = "fake" ]; then
    export ROBOT_LIDAR="${ROBOT_LIDAR:-/tmp/picar-lidar-$(id -u)}"
    python -m sim.fake_lidar --link "$ROBOT_LIDAR" &
    pids+=($!)
    for _ in $(seq 1 50); do [ -e "$ROBOT_LIDAR" ] && break; sleep 0.1; done
    echo "  lidar   $ROBOT_LIDAR   (fake D500)"
  fi
fi
python -m uvicorn robot.server:app --port 8000 --host 127.0.0.1 --log-level warning &
pids+=($!)
ROUTE_PREFIX=/brain python -m uvicorn control.brain_server:app \
  --port 8001 --host 127.0.0.1 --log-level warning &
pids+=($!)
python -m uvicorn service.tunnel.proxy:app --port 8080 --host 127.0.0.1 --log-level warning &
pids+=($!)

sleep 3
echo
echo "  robot  http://127.0.0.1:8000"
echo "  brain  http://127.0.0.1:8001/brain"
echo "  proxy  http://127.0.0.1:8080   <- point ngrok at this one"
echo
echo "Restarting this kills any mission in flight, and the phone reports it"
echo "as a bare \"Load failed\" -- Safari's words for a fetch that never"
echo "reached anything. Press Start again once this line reappears."
echo
echo "Warm the perception models once before a rig walk -- the first tiered"
echo "mission downloads yolo11s.pt (18MB) inside POST /mission/start:"
echo "  python -c 'from brain.perceive import pipeline_for; pipeline_for(\"red backpack\")'"
wait
