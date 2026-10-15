#!/bin/sh
# The full suite, once a night, on the latest origin/dev (user, 2026-10-09:
# pushes run only the fast tier, so the browser and live-stack tests run
# here instead). Reports; never blocks a push.
#
# Runs in its own worktree so it never touches a session's checkout, and
# writes to ~/Library/Logs/vision-picar/:
#   nightly-<date>.log   the refresh's and the whole pytest output
#   nightly-latest.txt   one line: date, commit tested ("-" if none), PASS, FAIL,
#                        STOPPED or SKIPPED (another run held the lock), and
#                        pytest's summary or the script's own
#                        ("fetch failed", "could not refresh", "nightly: ...")
# and, on a FAIL only, emails the failing tests (see the end). STOPPED (the
# job was stopped mid-run) is not emailed.
#
# Install the schedule (03:00 daily, macOS) with:
#   sh tools/nightly_suite.sh --install
# Remove it with --uninstall.
#
# ProcessType Interactive: with no ProcessType, launchd throttles the job's
# CPU and I/O, and the timing tests that drive the fake motor board and the
# split sim body fail under it (2026-10-10/11: 5 and 8 failures that pass in
# a terminal; `taskpolicy -b` reproduces all of them). Re-run --install
# after changing the plist.
set -u
repo="$(cd "$(dirname "$0")/.." && pwd)"
common="$(git -C "$repo" rev-parse --git-common-dir)" && [ -n "$common" ] \
  || { echo "nightly: $repo is not a git checkout" >&2; exit 1; }
case "$common" in /*) ;; *) common="$repo/$common" ;; esac
main="$(dirname "$common")"
logs="$HOME/Library/Logs/vision-picar"
label="com.vision-picar.nightly-suite"
plist="$HOME/Library/LaunchAgents/$label.plist"

if [ "${1:-}" = "--install" ]; then
  mkdir -p "$HOME/Library/LaunchAgents" "$logs"
  cat > "$plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>Label</key><string>$label</string>
  <key>ProgramArguments</key><array><string>/bin/sh</string><string>$main/tools/nightly_suite.sh</string></array>
  <key>StartCalendarInterval</key><dict><key>Hour</key><integer>3</integer><key>Minute</key><integer>0</integer></dict>
  <key>ProcessType</key><string>Interactive</string>
  <key>StandardOutPath</key><string>$logs/launchd.log</string>
  <key>StandardErrorPath</key><string>$logs/launchd.log</string>
</dict></plist>
EOF
  launchctl bootout "gui/$(id -u)/$label" 2>/dev/null
  launchctl bootstrap "gui/$(id -u)" "$plist" && echo "installed: $plist (03:00 daily)"
  exit $?
fi
if [ "${1:-}" = "--uninstall" ]; then
  launchctl bootout "gui/$(id -u)/$label" 2>/dev/null
  rm -f "$plist" && echo "removed $plist"
  exit 0
fi

# launchd's PATH is /usr/bin:/bin:/usr/sbin:/sbin; aws may be Homebrew's.
PATH="$PATH:/opt/homebrew/bin:/usr/local/bin"; export PATH
mkdir -p "$logs"
# One run at a time (rounds 10-11): every checkout of this repo shares $wt,
# and a second run's refresh would rewrite the tree under the first's
# pytest. A kernel lock (flock, through the venv's python: macOS has no
# flock(1)) that dies with its holders -- no pid file to go stale or be
# reused. The script runs again under it; a refused run says so in the report.
# With no venv python the run goes on unlocked and reports its own FAIL.
if [ -z "${NIGHTLY_LOCKED:-}" ] && [ -x "$main/.venv/bin/python" ]; then
  exec "$main/.venv/bin/python" - "$common/nightly.lock" "$logs/nightly-latest.txt" \
    "${NIGHTLY_SH:-/bin/sh}" "$0" ${1+"$@"} <<'PY'
import fcntl, os, sys, time
lock, latest, sh, script, args = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5:]
fd = os.open(lock, os.O_CREAT | os.O_RDWR, 0o644)
try:
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError:
    line = f"{time.strftime('%Y-%m-%d')} - SKIPPED  another nightly run holds {lock}"
    print(f"nightly: {line}", file=sys.stderr)
    with open(latest, "w") as fh:            # the running night's report replaces it
        fh.write(line + "\n")
    sys.exit(1)
os.set_inheritable(fd, True)
os.environ["NIGHTLY_LOCKED"] = "1"
os.execv(sh, [sh, script, *args])
PY
fi
unset NIGHTLY_LOCKED NIGHTLY_SH               # not into pytest, nor any script a test runs
wt="$common/nightly-worktree"
day="$(date +%Y-%m-%d)"
log="$logs/nightly-$day.log"
rc=0; sha="-"; was_stopped=0       # sha: the commit tested; none until the refresh holds
# A stalled fetch must not hold the night either (round 11): git's own
# low-speed limit over https. Known limit: an ssh remote has no such limit
# (a default ssh command would override the user's core.sshCommand).
if ! git -C "$main" -c http.lowSpeedLimit=1000 -c http.lowSpeedTime=120 \
    fetch -q origin dev > "$log" 2>&1; then
  rc=1; summary="fetch failed: $(tail -n 1 "$log")"     # names no commit: none was tested
else
  # Detached, forced and cleaned to the commit: a file a test left changed
  # cannot stop the refresh, and nothing carries over (3.76 J1). If that
  # fails (a directory deleted by hand and still registered, a bare one, a
  # `.git` pointing at a moved repo, a lock), the worktree is made again:
  # $wt is this script's own, inside .git, and `add -f -f` takes back a
  # registration even a locked one (J3). A failure of both is the night's
  # FAIL, reported and emailed like any other.
  {
    { [ -e "$wt/.git" ] && git -C "$wt" checkout -q -f --detach origin/dev \
        && git -C "$wt" clean -q -ffdx; } \
      || { rm -rf "$wt" && git -C "$main" worktree add -q -f -f --detach "$wt" origin/dev; }
  } >> "$log" 2>&1 || { rc=1; summary="could not refresh $wt: $(tail -n 1 "$log")"; }
fi
if [ $rc -eq 0 ]; then
  sha="$(git -C "$wt" rev-parse --short HEAD)"
  # A time limit (J2): launchd starts no second copy while one runs, so a
  # hung test would end every later night's report. pytest runs in its own
  # process group: on the limit it gets SIGINT (its teardown runs), then the
  # group SIGKILL a minute later. Stopping the job (TERM, HUP, INT) reaches
  # the driver through the trap: SIGINT, 10 s (launchd's ExitTimeOut is 20),
  # then SIGKILL. A stopped run reads STOPPED and sends no email (someone
  # stopped it; launchd allows 20 s). Known limit: a driver killed by SIGKILL
  # itself leaves the group running.
  drv=""; stopped=0
  trap 'stopped=1; was_stopped=1; [ -n "$drv" ] && kill -TERM "$drv" 2>/dev/null' TERM HUP INT
  "$main/.venv/bin/python" - "$wt" "${NIGHTLY_TIMEOUT_S:-7000}" "$main/.venv/bin/python" \
    >> "$log" 2>&1 <<'PY' &
import os, signal, subprocess, sys, time
wt, limit, py = sys.argv[1], float(sys.argv[2]), sys.argv[3]

# A stop only records itself; the driver polls. Nothing is raised from the
# handler, so no stop can land inside Popen, inside a wait that has reaped
# pytest but not kept its status, or into the cleanup (3.76 J2, rounds 2-7).
stop = {"sig": None, "at": None}

def stopped(sig, frame):
    if stop["sig"] is None:
        stop["sig"], stop["at"] = sig, time.monotonic()

# Handlers, not "ignore": a background job starts with SIGINT ignored, and an
# ignored signal would stay ignored in pytest; a handled one resets on exec.
for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
    signal.signal(s, stopped)

def kill(sig):
    try:
        os.killpg(p.pid, sig)
    except OSError:                      # gone, or a member we may not signal
        pass

p = subprocess.Popen([py, "-m", "pytest", "tests/", "-q", "-p", "no:cacheprovider"], cwd=wt,
                     stdin=subprocess.DEVNULL, start_new_session=True)
why, deadline = None, time.monotonic() + limit
while p.poll() is None and stop["sig"] is None and time.monotonic() < deadline:
    time.sleep(0.2)
if p.poll() is None:                     # the limit, or a stop: SIGINT (pytest's teardown
    if stop["sig"] is None:              # stops what tests started), then SIGKILL
        why, end = f"timed out after {limit:.0f} s", time.monotonic() + 60
    else:
        why, end = f"stopped by signal {stop['sig']}", time.monotonic() + 10
    kill(signal.SIGINT)
    while p.poll() is None and time.monotonic() < end:
        if stop["at"] is not None:       # a stop in the limit's grace: 10 s from it
            end = min(end, stop["at"] + 10)  # (launchd's ExitTimeOut is 20)
        time.sleep(0.2)
kill(signal.SIGKILL)                     # whatever the run left behind in its group
rc = p.wait()
# Its own line: pytest -q killed mid-line leaves no newline (round 7).
if why:
    print(f"\nnightly: {why}")
    sys.exit(124)
if rc < 0:                               # pytest itself killed (OOM, a crash): say so
    print(f"\nnightly: pytest killed by signal {-rc}")
    sys.exit(125)
sys.exit(rc)
PY
  drv=$!
  [ $was_stopped -eq 1 ] && kill -TERM "$drv" 2>/dev/null     # stopped before $drv was known
  wait "$drv"; rc=$?
  while [ $stopped -eq 1 ]; do        # a trap ends a wait early: wait for the driver's own code
    stopped=0; wait "$drv"; r=$?; [ $r -ne 127 ] && rc=$r
  done
  drv=""                              # reaped: a later stop must not signal a reused pid
  # pytest's own summary ("N passed in Xs") or the driver's line: output a
  # leftover server writes after it is not the night's summary (round 8).
  summary="$(grep -E '^nightly: | in [0-9.]+s( \(.*\))?( =+)?$' "$log" | tail -n 1)"
  [ -n "$summary" ] || summary="$(tail -n 1 "$log")"
fi
# STOPPED only when the stop ended the run: the driver's own line decides
# when it wrote one; otherwise it died of the signal. A FAIL or a timeout a
# late stop landed on is still a FAIL.
case "$summary" in
  "nightly: stopped by signal"*) by_stop=1 ;;
  "nightly: "*) by_stop=0 ;;
  *) by_stop=0; [ $was_stopped -eq 1 ] && [ $rc -gt 128 ] && by_stop=1 ;;
esac
if [ $rc -eq 0 ]; then verdict=PASS; elif [ $by_stop -eq 1 ]; then verdict=STOPPED
else verdict=FAIL; fi
echo "$day $sha $verdict  $summary" > "$logs/nightly-latest.txt"
cat "$logs/nightly-latest.txt"

# On failure only, an email through Amazon SES (user, 2026-10-09). SES is in
# sandbox mode on this account, so the address must be a verified identity.
# NIGHTLY_EMAIL overrides it; NIGHTLY_EMAIL="" turns the email off.
to="${NIGHTLY_EMAIL-anshu.gaind@gmail.com}"
if [ "$verdict" = FAIL ] && [ -n "$to" ]; then
  aws="$(command -v aws || echo /usr/local/bin/aws)"
  failed="$(grep -E '^(FAILED|ERROR) ' "$log" | head -20)"
  body="Nightly full suite FAILED on origin/dev $sha ($day).

$summary

$failed

Full log: $log"
  "$aws" sesv2 send-email --region us-east-2 \
    --from-email-address "$to" \
    --destination "ToAddresses=$to" \
    --content "$(python3 -c 'import json,sys; print(json.dumps({"Simple": {"Subject": {"Data": sys.argv[1]}, "Body": {"Text": {"Data": sys.argv[2]}}}}))' \
      "vision-picar nightly suite FAILED ($sha)" "$body")" \
    >/dev/null 2>>"$logs/launchd.log" || echo "$day email failed (see launchd.log)" >> "$logs/nightly-latest.txt"
fi
exit $rc
