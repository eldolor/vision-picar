# 3.108: sourced by board-side benches. Since 3.108 picar-local-vlm is
# enabled at boot, so a bench's `systemctl isolate` would start it (a 4 GB
# load inside the measurement) and its teardown's isolate would start it
# again whatever it was before. The unit carries
# `ConditionPathExists=!/run/picar-vlm-hold`: while that flag exists no
# start of it runs (a skipped start, not a failure). /run is cleared at
# every boot, so a bench killed mid-run never leaves the VLM off for good,
# and its enabled state is never touched.
#
#   vlm_hold [log]  after the bench's trap, before its first isolate: note
#                   whether the VLM was running, set the flag, stop it;
#                   refuses (exit 2) if it is still running after
#   vlm_restore     in the teardown, after its last isolate: clear the
#                   flag, start the VLM only if it was running
#
#   source "$(dirname "${BASH_SOURCE[0]}")/vlm_hold.sh"
VLM_HOLD_FLAG=/run/picar-vlm-hold
VLM_WAS=; VLM_HELD=
vlm_hold() {
  local log="${1:-/dev/null}"
  VLM_WAS=$(systemctl is-active picar-local-vlm 2>/dev/null)
  sudo touch "$VLM_HOLD_FLAG" && VLM_HELD=1
  sudo systemctl stop picar-local-vlm 2>/dev/null
  local now; now=$(systemctl is-active picar-local-vlm 2>/dev/null)
  echo "vlm_hold $(date +%T): was ${VLM_WAS:-unknown}; now ${now:-unknown}, flag $([ -e "$VLM_HOLD_FLAG" ] && echo set || echo MISSING)" | tee -a "$log"
  if [ -z "$VLM_HELD" ] || [ "$now" = active ] || [ "$now" = activating ]; then
    echo "STOP: could not hold picar-local-vlm" | tee -a "$log"; exit 2
  fi
}
vlm_restore() {
  [ -n "$VLM_HELD" ] || return 0        # never held: nothing to put back
  sudo rm -f "$VLM_HOLD_FLAG"; VLM_HELD=
  case "$VLM_WAS" in active|activating) sudo systemctl start picar-local-vlm;; esac
  echo "vlm_restore $(date +%T): $(systemctl is-active picar-local-vlm 2>/dev/null) (was ${VLM_WAS:-unknown})"
}
