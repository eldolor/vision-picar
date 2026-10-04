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
# 3.36: the sim body process, when there is one, is stopped and checked too.
if [ -n "${SIM_BODY_URL:-}" ]; then
  BODY_PORT="${SIM_BODY_URL##*:}"; BODY_PORT="${BODY_PORT%%/*}"
  PORTS="$PORTS $BODY_PORT"
fi
EXPECT="$(git rev-parse --short HEAD)"

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
if [ -n "$(listeners)" ]; then
  echo "FAILED: ports $PORTS are still held by: $(echo $(listeners)). Nothing was started." >&2
  exit 1
fi

# ---- start, detached, so it outlives the terminal that ran this ----------
nohup bash service/tunnel/run.sh >"$LOG" 2>&1 </dev/null &
disown || true

# ---- verify: the NEW code is what answers ---------------------------------
revision() {  # $1 = health URL; prints the git revision it reports, or nothing
  curl -s -m 2 ${LOCAL_SECRET:+-H "x-app-secret: $LOCAL_SECRET"} "$1" 2>/dev/null \
    | python3 -c "import sys,json; print(json.load(sys.stdin)['identity']['git_revision'])" 2>/dev/null || true
}
SECRETS="$HOME/.vision-picar-local-secrets"
# shellcheck disable=SC1090
[ -f "$SECRETS" ] && { set -a; source "$SECRETS"; set +a; }

robot=""; brain=""; body="$EXPECT"
for _ in $(seq 1 40); do
  robot="$(revision http://127.0.0.1:8000/health)"
  brain="$(revision http://127.0.0.1:8001/brain/health)"
  [ -n "${SIM_BODY_URL:-}" ] && body="$(revision "$SIM_BODY_URL/health")"
  [ "$robot" = "$EXPECT" ] && [ "$brain" = "$EXPECT" ] && [ "$body" = "$EXPECT" ] && break
  sleep 0.5
done

if [ "$robot" != "$EXPECT" ] || [ "$brain" != "$EXPECT" ] || [ "$body" != "$EXPECT" ]; then
  echo "FAILED: expected every server on $EXPECT; robot reports '${robot:-nothing}', brain '${brain:-nothing}', sim body '${body:-nothing}'." >&2
  echo "Log: $LOG" >&2
  grep -E "ERROR|Traceback" "$LOG" >&2 || true
  exit 1
fi

echo "OK: robot and brain both running $EXPECT (log: $LOG)"
if ! pgrep -x ngrok >/dev/null 2>&1; then
  echo "WARNING: ngrok is not running -- the deployed twin cannot reach this machine. Start it with: ngrok start picar"
fi
