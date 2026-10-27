# 3.108: sourced by board-side benches. Since 3.108 picar-local-vlm is
# enabled at boot, so a bench's `systemctl isolate` would start it (a 4 GB
# load inside the measurement) and its teardown's isolate would start it
# again whatever it was before. The unit carries
# `ConditionPathExists=!/run/picar-vlm-hold`: while that flag exists no
# start of it runs (a skipped start). /run is cleared at every boot, so a
# bench killed mid-run never leaves the VLM off past a reboot, and the
# unit's enabled state is never touched. The flag holds its bench's pid and
# that process's start time, so a reused pid never reads as the owner.
#
#   vlm_hold [log]  after the bench's trap, before its first isolate: note
#                   whether the VLM was running, set the flag, stop it.
#                   Returns 2 (having changed nothing it does not undo) if
#                   it cannot; the caller exits WITHOUT its teardown:
#                     vlm_hold "$log" || { vlm_restore; trap - EXIT; exit 2; }
#                   A flag whose owner is alive is the bench that called
#                   this one (top5.sh runs S/board/run.sh): left to it. A
#                   flag whose owner is gone is stale (a killed bench):
#                   taken over, and the VLM put back running at the end if
#                   it is enabled (the stale flag had held it off).
#                   Callers whose EXIT trap only calls vlm_restore may use
#                   `vlm_hold "$log" || exit 2`.
#   vlm_restore     in the teardown, after its last isolate: clear the flag
#                   this bench holds, start the VLM only if it was running
#                   (its start limit reset first: the isolates' skipped
#                   starts may have counted against it)
#
# Benches run under the board lock (3.93, `board_lock.py run`): two at once
# would see each other's flag as their own caller's.
#   source "$(dirname "${BASH_SOURCE[0]}")/vlm_hold.sh" || { echo "STOP: no vlm_hold.sh"; exit 2; }
VLM_HOLD_FLAG=/run/picar-vlm-hold
VLM_WAS=; VLM_HELD=
# "<pid> <start time>" of a process, from /proc (readable for any user's
# process, unlike `kill -0`); empty if there is none.
vlm_stamp() { [ -r "/proc/$1/stat" ] && echo "$1 $(cut -d')' -f2 "/proc/$1/stat" | awk '{print $20}')"; }
# 0 if the stamp in the flag is a live process's.
vlm_owner_alive() {
  local pid start
  read -r pid start < "$VLM_HOLD_FLAG" 2>/dev/null || [ -n "$pid" ] || return 1   # no newline: read fails, yet reads
  [ -n "$pid" ] && [ -n "$start" ] && [ "$(vlm_stamp "$pid")" = "$pid $start" ]
}
vlm_hold() {
  local log="${1:-/dev/null}" now unit
  VLM_WAS=$(systemctl is-active picar-local-vlm 2>/dev/null)
  if [ "$(systemctl is-enabled picar-local-vlm 2>/dev/null)" = enabled ]; then
    unit=$(systemctl cat picar-local-vlm 2>/dev/null)          # not piped: pipefail
    if ! grep -q '^ConditionPathExists=!/run/picar-vlm-hold' <<< "$unit" \
       || [ "$(systemctl show -p NeedDaemonReload --value picar-local-vlm 2>/dev/null)" = yes ]; then
      echo "STOP: the loaded picar-local-vlm unit has no hold condition -- re-run install_services.sh" | tee -a "$log"
      return 2
    fi
  fi
  if [ -e "$VLM_HOLD_FLAG" ] && vlm_owner_alive; then
    echo "vlm_hold $(date +%T): held by $(cat "$VLM_HOLD_FLAG") (the bench that called this one)" | tee -a "$log"
  elif [ -e "$VLM_HOLD_FLAG" ]; then
    echo "vlm_hold $(date +%T): taking over a stale hold ($(cat "$VLM_HOLD_FLAG" 2>/dev/null) is gone)" | tee -a "$log"
    vlm_stamp $$ | sudo tee "$VLM_HOLD_FLAG" >/dev/null \
      || { echo "STOP: could not take over $VLM_HOLD_FLAG" | tee -a "$log"; return 2; }
    VLM_HELD=1
    # The stale flag held an enabled VLM off: it is to be running again.
    [ "$(systemctl is-enabled picar-local-vlm 2>/dev/null)" = enabled ] && VLM_WAS=active
  elif vlm_stamp $$ | sudo tee "$VLM_HOLD_FLAG" >/dev/null; then
    VLM_HELD=1
  else
    echo "STOP: could not set $VLM_HOLD_FLAG" | tee -a "$log"; return 2
  fi
  sudo systemctl stop picar-local-vlm 2>/dev/null
  now=$(systemctl is-active picar-local-vlm 2>/dev/null)
  echo "vlm_hold $(date +%T): was ${VLM_WAS:-unknown}; now ${now:-unknown}" | tee -a "$log"
  if [ "$now" = active ] || [ "$now" = activating ]; then
    echo "STOP: could not stop picar-local-vlm" | tee -a "$log"; return 2
  fi
}
vlm_restore() {
  [ -n "$VLM_HELD" ] || return 0        # not this bench's hold: nothing to put back
  sudo rm -f "$VLM_HOLD_FLAG"; VLM_HELD=
  case "$VLM_WAS" in
    active|activating) sudo systemctl reset-failed picar-local-vlm 2>/dev/null
                       sudo systemctl start picar-local-vlm;;
  esac
  echo "vlm_restore $(date +%T): $(systemctl is-active picar-local-vlm 2>/dev/null) (was ${VLM_WAS:-unknown})"
}
