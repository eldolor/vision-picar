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

# Gates inbound calls to BOTH local servers. require_secret() is inert
# without it, and these are on the public internet through the tunnel.
export APP_SHARED_SECRET="$LOCAL_SECRET"
# The brain calling the robot. Same value: both are yours.
export ROBOT_SHARED_SECRET="$LOCAL_SECRET"
# The brain calling the deployed vision service, which has its OWN secret --
# set separately or the brain would send the one above and get a 401.
export VISION_URL="${VISION_URL:-https://d114x92g7i4syl.cloudfront.net}"
export VISION_SHARED_SECRET="$VISION_SECRET"

export ROBOT_MODE="${ROBOT_MODE:-sim}"
echo "robot mode: $ROBOT_MODE   vision: $VISION_URL"

# Kill only what THIS script started. `kill 0` would have been shorter and
# signals the whole process group -- which on a normal setup includes the
# ngrok agent running in the same shell session, so stopping the servers
# would silently take the tunnel down with them and the phone would report
# a bare "Load failed" with nothing to point at.
pids=()
cleanup() { for pid in "${pids[@]:-}"; do kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

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
