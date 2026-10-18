#!/bin/sh
# The full suite, once a night, on the latest origin/dev (user, 2026-10-09:
# pushes run only the fast tier, so the browser and live-stack tests run
# here instead). Reports; never blocks a push.
#
# Runs in its own worktree so it never touches a session's checkout, and
# writes to ~/Library/Logs/vision-picar/:
#   nightly-<date>.log   the refresh's and the whole pytest output
#   nightly-latest.txt   the night's line: date, commit tested ("-" if none),
#                        RUNNING (written at the start, so a run that dies
#                        leaves that, never yesterday's verdict), then PASS,
#                        FAIL or STOPPED, and pytest's summary or the
#                        script's own ("fetch failed", "could not refresh",
#                        "nightly: ..."), with "(unlocked: why)" when the run
#                        had no lock. Lines may be ADDED after it: a refused
#                        run's "- SKIPPED" (another run held the lock) and
#                        "email failed". The first line is the night's verdict
#                        -- or, when no run got the lock tonight (a holder
#                        that is not a nightly run), tonight's "- SKIPPED".
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
# Where a FAIL is emailed (see the end); the wrapper mails its own FAILs too.
# SES is in sandbox mode on this account, so the address must be a verified
# identity. NIGHTLY_EMAIL overrides it; NIGHTLY_EMAIL="" turns the email off.
to="${NIGHTLY_EMAIL-anshu.gaind@gmail.com}"
# One run at a time (rounds 10-11): every checkout of this repo shares $wt,
# and a second run's refresh would rewrite the tree under the first's
# pytest. A kernel lock (flock, through the venv's python: macOS has no
# flock(1)) that dies with its holders -- no pid file to go stale or be
# reused. The script runs again as this python's CHILD, the lock kept here
# and never inherited (round 14: an exec'd sh passed it to git, and a
# daemon git starts, fsmonitor or maintenance, held it for good; and an
# exec from python left SIGPIPE ignored in sh, which no sh can undo -- a
# child gets it back by Popen's restore_signals). A stop is passed on to the
# child, whose trap writes the report. A refused run ADDS its line: it never
# replaces a verdict the holder may already have written. A lock error that
# is not "held" (a filesystem with no flock) runs unlocked, said in
# launchd.log. With no venv python the run goes on unlocked too.
# NIGHTLY_LOCKED names the wrapper ("wrapper:<pid>", never a bare number
# such as launchd's pid 1), so a value left exported in the caller's
# environment never skips the lock (rounds 17-18). The sh must be the
# wrapper's own child: NIGHTLY_SH (a test hook) must exec its shell.
[ "${NIGHTLY_LOCKED:-}" = "wrapper:$PPID" ] || unset NIGHTLY_LOCKED NIGHTLY_UNLOCKED
if [ -z "${NIGHTLY_LOCKED:-}" ] && [ -x "$main/.venv/bin/python" ]; then
  NIGHTLY_TO="$to" exec "$main/.venv/bin/python" - "$common/nightly.lock" "$logs/nightly-latest.txt" \
    "${NIGHTLY_SH:-/bin/sh}" "$0" ${1+"$@"} <<'PY'
import errno, fcntl, json, os, shutil, signal, subprocess, sys, time
lock, latest, sh, script, args = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5:]
day = time.strftime('%Y-%m-%d')
to = os.environ.pop("NIGHTLY_TO", "")

def mail(line):
    """The sh's email, for a FAIL only this wrapper saw (round 21: a broken
    night was reported but never mailed)."""
    if not to:
        return
    content = {"Simple": {"Subject": {"Data": "vision-picar nightly suite FAILED (-)"},
                          "Body": {"Text": {"Data": f"Nightly full suite FAILED ({day}).\n\n{line}\n"}}}}
    cmd = [shutil.which("aws") or "/usr/local/bin/aws", "sesv2", "send-email", "--region", "us-east-2",
           "--from-email-address", to, "--destination", f"ToAddresses={to}",
           "--content", json.dumps(content)]
    try:
        ok = subprocess.run(cmd, stdout=subprocess.DEVNULL, timeout=120).returncode == 0
    except (OSError, subprocess.SubprocessError):
        ok = False
    if not ok:
        try:
            with open(latest, "a") as fh:
                fh.write(f"{day} email failed (see launchd.log)\n")
        except OSError:
            pass

def lines():
    try:
        return open(latest).read().splitlines()
    except OSError:
        return []

def report(line):
    """Tonight's line, with tonight's refused runs kept under it (round 18;
    any line, the first too: round 19). A failed write is said and never
    stops the run (round 19: a full disk ended the night before the suite)."""
    keep = [l for l in lines() if l.startswith(f"{day} - SKIPPED ")]
    tmp = f"{latest}.new.{os.getpid()}"              # its own name (round 21)
    try:
        with open(tmp, "w") as fh:
            fh.write("\n".join([line, *keep]) + "\n")
        os.replace(tmp, latest)
    except OSError as exc:
        print(f"nightly: could not write {latest}: {exc}", file=sys.stderr)

locked = True
try:
    fd = os.open(lock, os.O_CREAT | os.O_RDWR, 0o644)
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError as exc:
    if exc.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
        print(f"nightly: no lock ({exc}); running unlocked", file=sys.stderr)
        os.environ["NIGHTLY_UNLOCKED"] = str(exc)     # the report says so (round 15)
        locked = False
    else:
        line = f"{day} - SKIPPED  another nightly run holds {lock}"
        print(f"nightly: {line}", file=sys.stderr)
        first = (lines() or [""])[0]
        # Added under tonight's line (a nightly run holds it and will write
        # its verdict); otherwise it IS tonight's line (round 16: a holder
        # that is no nightly run, a daemon from before round 14, left an old
        # PASS on top for good).
        try:
            with open(latest, "a" if first.startswith(day + " ") else "w") as fh:
                fh.write(line + "\n")
        except OSError as werr:              # said, never a traceback (round 20)
            print(f"nightly: could not write {latest}: {werr}", file=sys.stderr)
        sys.exit(1)
os.environ["NIGHTLY_LOCKED"] = f"wrapper:{os.getpid()}"
child, pending = None, []

def forward(sig, frame):
    if child is None:                    # before Popen returned: passed on after it
        pending.append(sig)
        return
    try:
        child.send_signal(sig)
    except OSError:
        pass

# Before Popen (round 15): a stop in between killed this process, and with
# it the lock, while the sh ran on unsignalled.
for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
    if signal.getsignal(s) is not signal.SIG_IGN:   # nohup's HUP stays ignored (round 19)
        signal.signal(s, forward)
# Tonight's line, before the sh can die (round 17): an earlier run today
# must not stand in for this one.
report(f"{day} - RUNNING  {'locked' if locked else 'unlocked'} {time.strftime('%H:%M')}")  # the sh's: "started"
try:
    child = subprocess.Popen([sh, script, *args], stdin=subprocess.DEVNULL)   # close_fds: no lock
except OSError as exc:                   # no fork, no sh (round 20): a FAIL, said
    report(f"{day} - FAIL  nightly: could not start {sh}: {exc}")
    mail(f"nightly: could not start {sh}: {exc}")
    sys.exit(1)
for s in pending:
    forward(s, None)
rc = child.wait()                        # PEP 475: retried after a handler
# One rule (round 21): if the sh ended with tonight's line still RUNNING (its
# own or this wrapper's) or not tonight's, it wrote no verdict, and this
# writes one from how it ended -- never a run that looks hung, never
# yesterday's. A signal it does not trap (SIGQUIT, an OOM kill, one before
# its trap) is STOPPED; an exit before its report (a set -u error, a check
# that failed) is a FAIL, and mailed; a 0 whose report failed to write
# (a full disk) is a PASS. Known limit: what a dead sh left running (the
# driver, and through it pytest) runs on unlocked -- under launchd the job's
# group is killed with it; a killpg here would hit a caller's own group.
first = (lines() or [""])[0]
if not first.startswith(day + " ") or first.startswith(f"{day} - RUNNING"):
    if rc < 0:
        report(f"{day} - STOPPED  nightly: the script died of signal {-rc}")
    elif rc:
        report(f"{day} - FAIL  nightly: the script exited {rc} before its report")
        mail(f"nightly: the script exited {rc} before its report")
    else:
        report(f"{day} - PASS  nightly: the script's own report could not be written")
sys.exit(128 - rc if rc < 0 else rc)
PY
fi
unlocked="${NIGHTLY_UNLOCKED:-}"
[ -z "${NIGHTLY_LOCKED:-}" ] && unlocked="no venv python"
unset NIGHTLY_LOCKED NIGHTLY_SH NIGHTLY_UNLOCKED   # not into pytest, nor any script a test runs
wt="$common/nightly-worktree"
day="$(date +%Y-%m-%d)"
log="$logs/nightly-$day.log"
rc=0; sha="-"; was_stopped=0       # sha: the commit tested; none until the refresh holds
drv=""; job=""; stopped=0
# TERM now and once more a second later (round 16): a TERM that lands in a
# job between its fork and its exec meets the sh's own trap there and is
# lost with the exec, and the job runs on unstopped. The driver keeps only
# the first stop; a git step dies of either.
stop_job() {
  kill -TERM "$1" 2>/dev/null
  ( sleep 1; kill -TERM "$1" 2>/dev/null ) > /dev/null 2>&1 &
}
trap 'stopped=1; was_stopped=1; [ -n "$drv" ] && stop_job "$drv"
      [ -n "$job" ] && stop_job "$job"' TERM HUP INT
# The night's line before anything can hang (round 14), after the trap (15):
# a run that dies in the fetch or the refresh leaves RUNNING, never
# yesterday's verdict.
# Tonight's line, with tonight's refused runs (their SKIPPED lines) kept
# under it (rounds 17-18). Known limit: one refused in the milliseconds
# between the grep and the mv is lost.
report() {
  skipped="$(grep "^$day - SKIPPED " "$logs/nightly-latest.txt" 2>/dev/null)"
  # Moved only if every write succeeded (round 20: a full disk wrote nothing,
  # the group still succeeded, and an empty file replaced the report).
  { echo "$1" && { [ -z "$skipped" ] || echo "$skipped"; }; } > "$logs/nightly-latest.txt.new.$$" \
    && mv "$logs/nightly-latest.txt.new.$$" "$logs/nightly-latest.txt" \
    || { rm -f "$logs/nightly-latest.txt.new.$$"; echo "nightly: could not write the report" >&2; }
}
report "$day - RUNNING  started $(date +%H:%M)${unlocked:+  (unlocked: $unlocked)}"
: > "$log"
# One step of the refresh, which a stop can end (round 15): sh runs a trap
# only after its foreground command, and a stalled fetch outlasts launchd's
# 20 s, so each step runs in the background and the trap's TERM reaches it.
# After a stop no further step runs.
# Until a background job is reaped, keeping its own code in $r: a trap ends
# a wait early, and a stop that reaches the sh twice can leave bash's wait
# answering before the job is gone (round 15). Capped (round 16): a reused
# pid must not hold the night.
reap() {
  wait "$1"; r=$?; _n=0
  while { [ $stopped -eq 1 ] || kill -0 "$1" 2>/dev/null; } && [ $_n -lt 90 ]; do
    stopped=0; _n=$((_n + 1)); wait "$1"; _w=$?
    if [ $_w -ne 127 ]; then r=$_w; elif kill -0 "$1" 2>/dev/null; then sleep 1; fi
  done
}
step() {
  [ $was_stopped -eq 1 ] && { r=130; return 130; }
  "$@" >> "$log" 2>&1 &
  job=$!
  [ $was_stopped -eq 1 ] && stop_job "$job"     # stopped before $job was known
  reap "$job"
  job=""
  return $r
}
# A stalled fetch must not hold the night either (round 11): git's own
# low-speed limit over https. Known limit: an ssh remote has no such limit
# (a default ssh command would override the user's core.sshCommand).
if ! step git -C "$main" -c http.lowSpeedLimit=1000 -c http.lowSpeedTime=120 \
    fetch -q origin dev; then
  rc=1; summary="fetch failed: $(tail -n 1 "$log")"     # names no commit: none was tested
else
  # Detached, forced and cleaned to the commit: a file a test left changed
  # cannot stop the refresh, and nothing carries over (3.76 J1). If that
  # fails (a directory deleted by hand and still registered, a bare one, a
  # `.git` pointing at a moved repo, a lock), the worktree is made again:
  # $wt is this script's own, inside .git, and `add -f -f` takes back a
  # registration even a locked one (J3). A failure of both is the night's
  # FAIL, reported and emailed like any other.
  { [ -e "$wt/.git" ] && step git -C "$wt" checkout -q -f --detach origin/dev \
      && step git -C "$wt" clean -q -ffdx; } \
    || { step rm -rf "$wt" && step git -C "$main" worktree add -q -f -f --detach "$wt" origin/dev; } \
    || { rc=1; summary="could not refresh $wt: $(tail -n 1 "$log")"; }
fi
# Stopped before the suite: no step after it ran. A refresh that failed on
# its own before the stop is still a FAIL (round 16): only a step the stop
# ended (> 128) or no failure at all reads STOPPED.
r=${r:-0}
if [ $was_stopped -eq 1 ] && { [ $rc -eq 0 ] || [ $r -gt 128 ]; }; then
  rc=130; summary="nightly: stopped before the suite ran"
fi
if [ $rc -eq 0 ]; then
  sha="$(git -C "$wt" rev-parse --short HEAD)"; [ -n "$sha" ] || sha="?"   # killed by a stop
  # A time limit (J2): launchd starts no second copy while one runs, so a
  # hung test would end every later night's report. pytest runs in its own
  # process group: on the limit it gets SIGINT (its teardown runs), then the
  # group SIGKILL a minute later. Stopping the job (TERM, HUP, INT) reaches
  # the driver through the trap: SIGINT, 10 s (launchd's ExitTimeOut is 20),
  # then SIGKILL. A stopped run reads STOPPED and sends no email (someone
  # stopped it; launchd allows 20 s). Known limit: a driver killed by SIGKILL
  # itself leaves the group running.
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
    if s == signal.SIGHUP and signal.getsignal(s) is signal.SIG_IGN:
        continue                         # nohup's, kept (round 19); INT is the sh's own
    signal.signal(s, stopped)

def kill(sig):
    try:
        os.killpg(p.pid, sig)
    except OSError:                      # gone, or a member we may not signal
        pass

if stop["sig"] is not None:              # stopped before pytest started (round 18)
    print(f"\nnightly: stopped by signal {stop['sig']}")
    sys.exit(124)
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
  [ $was_stopped -eq 1 ] && stop_job "$drv"     # stopped before $drv was known
  reap "$drv"; rc=$r                  # the driver's own code, once it is gone
  drv=""                              # reaped: a later stop must not signal a reused pid
  # pytest's own summary ("N passed, M skipped in Xs") or the driver's line:
  # output a leftover server writes after it is not the night's summary
  # (round 8), nor its own "... in 0.03s" (round 14).
  w='(subtests )?(passed|failed|errors?|skipped|deselected|xfailed|xpassed|warnings?|rerun)'
  summary="$(grep -E "^nightly: |^(=+ )?[0-9]+ $w(, [0-9]+ $w)* in [0-9.]+s( \(.*\))?( =+)?\$|^(=+ )?no tests ran in " "$log" | tail -n 1)"
  # A driver killed before it wrote a line of its own: by a stop (round 20)
  # or by anything else, an OOM kill (round 21).
  if [ -z "$summary" ] && [ $rc -gt 128 ]; then
    if [ $was_stopped -eq 1 ]; then summary="nightly: stopped by signal $((rc - 128))"
    else summary="nightly: the driver died of signal $((rc - 128))"; fi
  fi
  [ -n "$summary" ] || summary="$(tail -n 1 "$log")"
fi
# STOPPED only when the stop ended the run: the driver's own line decides
# when it wrote one; otherwise it died of the signal. A FAIL or a timeout a
# late stop landed on is still a FAIL.
case "$summary" in
  "nightly: stopped "*) by_stop=1 ;;      # by a signal, or before the suite ran
  "nightly: "*) by_stop=0 ;;
  *) by_stop=0; [ $was_stopped -eq 1 ] && [ $rc -gt 128 ] && by_stop=1 ;;
esac
if [ $rc -eq 0 ]; then verdict=PASS; elif [ $by_stop -eq 1 ]; then verdict=STOPPED
else verdict=FAIL; fi
report "$day $sha $verdict  $summary${unlocked:+  (unlocked: $unlocked)}"
head -n 1 "$logs/nightly-latest.txt"

# On failure only, an email through Amazon SES (user, 2026-10-09) to $to.
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
