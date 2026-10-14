#!/usr/bin/env bash
#
# Install (or re-install) the Jetson's two services from the repo
# (PLAN-ros-alignment.md 3.71; B5's first half):
#
#   picar-stack   service/tunnel/run.sh -- robot server, brain, proxy (:8080)
#   picar-ngrok   `ngrok start picar`, the public URL the deployed twin uses
#   picar-local-vlm   service/local_vlm (3.80.B) -- installed, NOT enabled:
#                 `sudo systemctl start picar-local-vlm` when it is wanted.
#                 Needs ~/.vision-picar-local-vlm.env (README.md section 5).
#
# Run ON the Jetson, from the service worktree:
#   cd ~/vision-picar-service && bash tools/jetson/install_services.sh
#
# Needs, once (README.md section 7): ngrok in /usr/local/bin with the
# `picar` tunnel in ~/.config/ngrok/ngrok.yml, and the MAC's secret files
# copied over -- the twin holds the Mac's LOCAL_SECRET, so a Jetson with its
# own answers "Secret rejected".
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

[ -f "$HOME/.vision-picar-local-secrets" ] || { echo "missing ~/.vision-picar-local-secrets"; exit 1; }
command -v ngrok >/dev/null || { echo "ngrok is not installed"; exit 1; }
ngrok config check >/dev/null

for unit in picar-stack picar-ngrok picar-local-vlm; do
  sudo install -m 644 "$HERE/systemd/$unit.service" "/etc/systemd/system/$unit.service"
done
sudo systemctl daemon-reload
sudo systemctl enable picar-stack.service picar-ngrok.service
sudo systemctl restart picar-stack.service picar-ngrok.service
for _ in $(seq 1 60); do
  curl -sf -m 3 http://127.0.0.1:8080/brain/health >/dev/null && break
  sleep 1
done
systemctl is-active picar-stack.service picar-ngrok.service
curl -s -m 5 http://127.0.0.1:8080/brain/health | head -c 160; echo
