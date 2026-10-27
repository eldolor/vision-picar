# 3.108: sourced by board-side benches. Since 3.108 picar-local-vlm is
# enabled at boot, so a bench's `systemctl isolate` would start it (a 4 GB
# load inside the measurement) and its teardown's isolate would start it
# again whatever it was before. The unit carries
# `ConditionPathExists=!/run/picar-vlm-hold`: while that flag exists no
# start of it runs (a skipped start). /run is cleared at every boot, so a
# bench killed mid-run never leaves the VLM off past a reboot, and the
# unit's enabled state is never touched.
#
#   vlm_hold [log]  after the bench's trap, before its first isolate: note
#                   whether the VLM was running, set the flag, stop it;
#                   refuses (exit 2) if that cannot be done. A flag already
#                   set is another bench's (one calling this one): it is
#                   left to that bench, set and cleared there.
#   vlm_restore     in the teardown, after its last isolate: clear the
#                   flag this bench set, and start the VLM only if it was
#                   running (its start limit reset first: the isolates'
#                   skipped starts may have counted against it)
#
#   source "$(dirname "${BASH_SOURCE[0]}")/vlm_hold.sh" || { echo "STOP: no vlm_hold.sh"; exit 2; }
VLM_HOLD_FLAG=/run/picar-vlm-hold
VLM_WAS=; VLM_HELD=
vlm_hold() {
  local log="${1:-/dev/null}" now
  VLM_WAS=$(systemctl is-active picar-local-vlm 2>/dev/null)
  if [ "$(systemctl is-enabled picar-local-vlm 2>/dev/null)" = enabled ] \
     && ! systemctl cat picar-local-vlm 2>/dev/null | grep -q '^ConditionPathExists=!/run/picar-vlm-hold'; then
    echo "STOP: the installed picar-local-vlm unit has no hold condition -- re-run install_services.sh" | tee -a "$log"; exit 2
  fi
  if [ -e "$VLM_HOLD_FLAG" ]; then
    echo "vlm_hold $(date +%T): already held by the bench that called this one" | tee -a "$log"
  elif sudo touch "$VLM_HOLD_FLAG"; then
    VLM_HELD=1
  else
    echo "STOP: could not set $VLM_HOLD_FLAG" | tee -a "$log"; exit 2
  fi
  sudo systemctl stop picar-local-vlm 2>/dev/null
  now=$(systemctl is-active picar-local-vlm 2>/dev/null)
  echo "vlm_hold $(date +%T): was ${VLM_WAS:-unknown}; now ${now:-unknown}" | tee -a "$log"
  if [ "$now" = active ] || [ "$now" = activating ]; then
    echo "STOP: could not stop picar-local-vlm" | tee -a "$log"; exit 2
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
