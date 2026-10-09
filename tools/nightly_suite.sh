#!/bin/sh
# The full suite, once a night, on the latest origin/dev (user, 2026-10-09:
# pushes run only the fast tier, so the browser and live-stack tests run
# here instead). Reports; never blocks a push.
#
# Runs in its own worktree so it never touches a session's checkout, and
# writes to ~/Library/Logs/vision-picar/:
#   nightly-<date>.log   the whole pytest output
#   nightly-latest.txt   one line: date, commit, PASS/FAIL, pytest's summary
# and, on a failure only, emails the failing tests (see the end).
#
# Install the schedule (03:00 daily, macOS) with:
#   sh tools/nightly_suite.sh --install
# Remove it with --uninstall.
set -u
repo="$(cd "$(dirname "$0")/.." && pwd)"
common="$(git -C "$repo" rev-parse --git-common-dir)"
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

mkdir -p "$logs"
wt="$common/nightly-worktree"
day="$(date +%Y-%m-%d)"
log="$logs/nightly-$day.log"
git -C "$main" fetch -q origin dev || { echo "$day fetch failed" > "$logs/nightly-latest.txt"; exit 1; }
if [ -d "$wt" ]; then
  git -C "$wt" checkout -q --detach origin/dev && git -C "$wt" reset -q --hard origin/dev
else
  git -C "$main" worktree add -q --detach "$wt" origin/dev
fi
sha="$(git -C "$wt" rev-parse --short HEAD)"
(cd "$wt" && "$main/.venv/bin/python" -m pytest tests/ -q -p no:cacheprovider) > "$log" 2>&1
rc=$?
summary="$(tail -n 1 "$log")"
[ $rc -eq 0 ] && verdict=PASS || verdict=FAIL
echo "$day $sha $verdict  $summary" > "$logs/nightly-latest.txt"
cat "$logs/nightly-latest.txt"

# On failure only, an email through Amazon SES (user, 2026-10-09). SES is in
# sandbox mode on this account, so the address must be a verified identity.
# NIGHTLY_EMAIL overrides it; NIGHTLY_EMAIL="" turns the email off.
to="${NIGHTLY_EMAIL-anshu.gaind@gmail.com}"
if [ $rc -ne 0 ] && [ -n "$to" ]; then
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
