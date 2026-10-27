# 3.108: sourced by board-side benches. Since 3.108 picar-local-vlm is
# enabled at boot, so a bench's `systemctl isolate` would start it (a 4 GB
# load inside the measurement) and its teardown's isolate would start it
# again whatever it was before. The unit carries
# `ConditionPathExists=!/run/picar-vlm-hold`: while that flag exists no
# start of it runs (a skipped start). /run is cleared at every boot, so a
# bench killed mid-run never leaves the VLM off past a reboot, and the
# unit's enabled state is never touched. The flag holds its bench's pid.
#
#   vlm_hold [log]  after the bench's trap, before its first isolate: note
#                   whether the VLM was running, set the flag, stop it.
#                   Returns 2 (having changed nothing it does not undo) if
#                   it cannot; the caller exits WITHOUT its teardown:
#                     vlm_hold "$log" || { vlm_restore; trap - EXIT; exit 2; }
#                   A flag whose pid is alive is the bench that called this
#                   one (top5.sh runs S/board/run.sh): left to it. A flag
#                   whose pid is gone is stale (a killed bench): taken over.
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
vlm_hold() {
  local log="${1:-/dev/null}" now unit owner
  VLM_WAS=$(systemctl is-active picar-local-vlm 2>/dev/null)
  if [ "$(systemctl is-enabled picar-local-vlm 2>/dev/null)" = enabled ]; then
    unit=$(systemctl cat picar-local-vlm 2>/dev/null)          # not piped: pipefail
    if ! grep -q '^ConditionPathExists=!/run/picar-vlm-hold' <<< "$unit" \
       || [ "$(systemctl show -p NeedDaemonReload --value picar-local-vlm 2>/dev/null)" = yes ]; then
      echo "STOP: the loaded picar-local-vlm unit has no hold condition -- re-run install_services.sh" | tee -a "$log"
      return 2
    fi
  fi
  if [ -e "$VLM_HOLD_FLAG" ]; then
    owner=$(cat "$VLM_HOLD_FLAG" 2>/dev/null)
    if [ -n "$owner" ] && kill -0 "$owner" 2>/dev/null; then
      echo "vlm_hold $(date +%T): held by pid $owner (the bench that called this one)" | tee -a "$log"
    else
      echo "vlm_hold $(date +%T): taking over a stale hold (pid ${owner:-none} is gone)" | tee -a "$log"
      echo $$ | sudo tee "$VLM_HOLD_FLAG" >/dev/null && VLM_HELD=1
    fi
  elif echo $$ | sudo tee "$VLM_HOLD_FLAG" >/dev/null; then
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
