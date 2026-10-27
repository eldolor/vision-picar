# 3.108: sourced by board-side benches. Since 3.108 picar-local-vlm is
# enabled at boot, so a bench's `systemctl isolate` would start it (a 4 GB
# load, one start of its limit of 4 in 15 min) and its teardown's isolate
# would start it again whatever it was before. A runtime mask cannot stop
# that: the unit file in /etc/systemd/system shadows /run's mask. So:
#
#   vlm_hold      before the first isolate: remember whether the VLM was
#                 enabled and running, stop it, and disable it for the run
#   vlm_restore   in the teardown, after the last isolate: enable it again
#                 if it was, start it only if it was running
#
# A bench that dies before its teardown leaves the VLM disabled (the safe
# side: identity goes to the cloud tier); `vlm_restore` logs what it did.
#   source "$(dirname "${BASH_SOURCE[0]}")/vlm_hold.sh"
VLM_WAS=; VLM_ENABLED=
vlm_hold() {
  VLM_WAS=$(systemctl is-active picar-local-vlm 2>/dev/null)
  VLM_ENABLED=$(systemctl is-enabled picar-local-vlm 2>/dev/null)
  sudo systemctl stop picar-local-vlm 2>/dev/null
  [ "$VLM_ENABLED" = enabled ] && sudo systemctl disable -q picar-local-vlm
  echo "vlm_hold: was $VLM_WAS, $VLM_ENABLED; now $(systemctl is-active picar-local-vlm 2>/dev/null), $(systemctl is-enabled picar-local-vlm 2>/dev/null)"
}
vlm_restore() {
  [ "$VLM_ENABLED" = enabled ] && sudo systemctl enable -q picar-local-vlm
  case "$VLM_WAS" in active|activating) sudo systemctl start picar-local-vlm;; esac
  echo "vlm_restore: $(systemctl is-active picar-local-vlm 2>/dev/null), $(systemctl is-enabled picar-local-vlm 2>/dev/null) (was $VLM_WAS, $VLM_ENABLED)"
}
