#!/usr/bin/env bash
#
# Restart the local robot + brain + proxy (service/tunnel/run.sh), and
# REFUSE TO REPORT SUCCESS until the new processes are the ones answering.
#
# Run from anywhere:
#   bash service/tunnel/restart.sh
#
# Why this exists: stopping run.sh with a plain `kill` fails quietly while a
# phone has the twin open. uvicorn's graceful shutdown waits for open
# connections to close, and the twin polls /health twice a second, so the
# old servers keep the ports; the new ones then die on "address already in
# use", and the OLD CODE carries on serving the phone as if nothing happened.
# That shipped twice on 2026-09-25 -- once after a force-kill in a combined
# script that was assumed to have worked. So this script checks the only
# thing that settles it: the git revision each server reports on /health.
#
# Leaves ngrok alone. It forwards to :8080 and survives a restart of what is
# behind it; this only warns if it is not running.
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
LOG="${TUNNEL_LOG:-$HOME/.vision-picar-tunnel.log}"
PORTS="8000 8001 8080"
# 3.36: the simulator's two programs, under SIM_MOTOR_BOARD=fake, are
# stopped too (run.sh's default ports unless SIM_BODY_URL/SIM_SENSORS_URL say).
if [ "${SIM_MOTOR_BOARD:-}" = "fake" ]; then
  SIM_BODY_URL="${SIM_BODY_URL:-http://127.0.0.1:8002}"
  SIM_SENSORS_URL="${SIM_SENSORS_URL:-http://127.0.0.1:8003,http://127.0.0.1:8004}"
  for u in "$SIM_BODY_URL" ${SIM_SENSORS_URL//,/ }; do
    p="${u##*:}"; PORTS="$PORTS ${p%%/*}"
  done
fi
EXPECT="$(git rev-parse --short HEAD)"
# 3.71: on the Jetson run.sh is the `picar-stack` unit. Killing it from here
# exits it 0 (its TERM trap), so systemd does not restart it, and the copy
# started below runs outside systemd: never restarted after a crash, and not
# stopped by `systemctl stop` before a bench (3.76 H3-13).
# Any state but stopped counts: a crash-looping unit reads `activating`
# between restarts, and would start a second run.sh over this one.
if command -v systemctl >/dev/null 2>&1; then
  case "$(systemctl is-active picar-stack 2>/dev/null || true)" in
    active|activating|deactivating|reloading)
      echo "FAILED: run.sh is the picar-stack unit here. For a bench, stop both units first: sudo systemctl stop picar-stack picar-ngrok (tools/jetson/README.md sections 4-5); to restart the served stack: sudo systemctl restart picar-stack" >&2
      exit 1 ;;
  esac
fi

listeners() {
  local p
  for p in $PORTS; do lsof -nP -tiTCP:"$p" -sTCP:LISTEN 2>/dev/null || true; done
}
runners() { pgrep -f "bash service/tunnel/run.sh" 2>/dev/null || true; }

# ---- stop: ask, then insist, then check ----------------------------------
old="$(printf '%s\n%s\n' "$(runners)" "$(listeners)" | sort -u | grep -v '^$' || true)"
if [ -n "$old" ]; then
  echo "stopping: $(echo $old)"
  kill $old 2>/dev/null || true
  for _ in $(seq 1 10); do [ -z "$(listeners)" ] && break; sleep 0.5; done
  # Still there after 5s: a connection is holding graceful shutdown open.
  left="$(printf '%s\n%s\n' "$(runners)" "$(listeners)" | sort -u | grep -v '^$' || true)"
  if [ -n "$left" ]; then
    echo "still holding the ports after 5s (graceful shutdown waiting on open connections) -- forcing: $(echo $left)"
    kill -9 $left 2>/dev/null || true
    for _ in $(seq 1 10); do [ -z "$(listeners)" ] && break; sleep 0.5; done
  fi
fi
# 3.42: a fake lidar holds no port, so a forced stop above can orphan it.
if [ "${SIM_LIDAR:-}" = "fake" ]; then
  pkill -f "sim.fake_lidar --link ${ROBOT_LIDAR:-/tmp/picar-lidar-$(id -u)}" 2>/dev/null || true
fi
if [ -n "$(listeners)" ]; then
  echo "FAILED: ports $PORTS are still held by: $(echo $(listeners)). Nothing was started." >&2
  exit 1
fi

# ---- start, detached, so it outlives the terminal that ran this ----------
nohup bash service/tunnel/run.sh >"$LOG" 2>&1 </dev/null &
disown || true

# ---- verify: the NEW code is what answers ---------------------------------
revision() {  # $1 = health URL; prints the git revision it reports, or nothing
  # The secret goes in on stdin (`-H @-`), not the command line, where `ps`
  # shows it to every user on the machine (3.76 H3-9).
  printf '%s\n' "${LOCAL_SECRET:+x-app-secret: $LOCAL_SECRET}" \
    | curl -s -m 2 -H @- "$1" 2>/dev/null \
    | python3 -c "import sys,json; print(json.load(sys.stdin)['identity']['git_revision'])" 2>/dev/null || true
}
SECRETS="$HOME/.vision-picar-local-secrets"
# shellcheck disable=SC1090
[ -f "$SECRETS" ] && { set -a; source "$SECRETS"; set +a; }

# Through the proxy (:8080, what ngrok points at), so a proxy that never
# came up is not reported OK (3.76 H3-8).
robot=""; brain=""; body="$EXPECT"
for _ in $(seq 1 40); do
  robot="$(revision http://127.0.0.1:8080/health)"
  brain="$(revision http://127.0.0.1:8080/brain/health)"
  [ "${SIM_MOTOR_BOARD:-}" = "fake" ] && body="$(revision "$SIM_BODY_URL/health")"
  [ "$robot" = "$EXPECT" ] && [ "$brain" = "$EXPECT" ] && [ "$body" = "$EXPECT" ] && break
  sleep 0.5
done

if [ "$robot" != "$EXPECT" ] || [ "$brain" != "$EXPECT" ] || [ "$body" != "$EXPECT" ]; then
  echo "FAILED: expected every server on $EXPECT; robot reports '${robot:-nothing}', brain '${brain:-nothing}', sim body '${body:-nothing}'." >&2
  echo "Log: $LOG" >&2
  grep -E "ERROR|Traceback" "$LOG" >&2 || true
  exit 1
fi

# run.sh stops everything when any one of its programs ends, first checked
# 3 s after it starts and every 2 s after: a sensor server that died at
# import can still be taking the rest down. Look once more after that.
# Asked through the proxy again, not by pgrep: a run.sh already in its
# (bounded) cleanup still matches pgrep for up to 8 s.
sleep 5
if [ "$(revision http://127.0.0.1:8080/health)" != "$EXPECT" ] \
   || [ "$(revision http://127.0.0.1:8080/brain/health)" != "$EXPECT" ]; then
  echo "FAILED: the stack stopped answering after it came up (one of run.sh's programs ended)." >&2
  echo "Log: $LOG" >&2
  grep -E "ERROR|Traceback|ended" "$LOG" >&2 || true
  exit 1
fi
echo "OK: robot and brain both running $EXPECT (log: $LOG)"
if ! pgrep -x ngrok >/dev/null 2>&1; then
  echo "WARNING: ngrok is not running -- the deployed twin cannot reach this machine. Start it with: ngrok start picar"
fi
