#!/usr/bin/env bash
#
# Set the project up on the Jetson -- PLAN-ros-alignment.md 3.33, steps 3 and 5.
# Run ON the Jetson, from the repo root, after JetPack 6.2.x is installed:
#
#   bash tools/jetson/setup.sh
#
# Idempotent: safe to run again. It CHECKS before it installs and stops at
# the first thing that is wrong, rather than working around it -- every
# step here is one of 3.33's stop points.
#
# Why each choice (researched 2026-10-02):
# * Python 3.10: JetPack 6 is Ubuntu 22.04, and NVIDIA's CUDA builds of
#   torch exist only for cp310. The project's laptop .venv is 3.13; the
#   code compiles and its suite runs under 3.10 (checked in Docker).
# * torch 2.8.0 / torchvision 0.23.0 from the Jetson AI Lab index
#   (pypi.jetson-ai-lab.io/jp6/cu126 -- the old .dev domain is gone). A
#   plain `pip install torch` gets a CPU-only build on aarch64; PyPI's
#   cu126 wheels fail here with "no kernel image is available".
# * libcusolver: the forum fix for torch 2.8 on JetPack 6.2.1.
set -euo pipefail

INDEX="https://pypi.jetson-ai-lab.io/jp6/cu126"
TORCH="torch==2.8.0"
VISION="torchvision==0.23.0"

say() { printf '\n== %s\n' "$*"; }
die() { printf '\nSTOP: %s\n' "$*" >&2; exit 1; }

[ "$(uname -m)" = "aarch64" ] || die "not a Jetson (uname -m is $(uname -m))"
[ -f requirements.txt ] && [ -d brain ] || die "run from the repo root"

say "JetPack / L4T"
cat /etc/nv_tegra_release 2>/dev/null || die "no /etc/nv_tegra_release -- is this JetPack?"
grep -q "R36" /etc/nv_tegra_release || die "expected L4T R36 (JetPack 6.x)"

say "power mode"
sudo nvpmodel -q || true
echo "(3.33 runs at 15 W; set it with: sudo nvpmodel -m <id> -- read the ids in /etc/nvpmodel.conf)"

say "system packages"
sudo apt-get update -qq
sudo apt-get install -y -qq python3.10-venv python3-pip git curl libopenblas-dev \
    libcusolver-12-6 libcusolver-dev-12-6 >/dev/null

say "venv (.venv, python3.10)"
[ -d .venv ] || python3.10 -m venv .venv
. .venv/bin/activate
python -m pip install -q --upgrade pip

say "torch for JetPack 6 (CUDA 12.6)"
pip install -q "$TORCH" "$VISION" --index-url "$INDEX"
python - <<'EOF'
import torch
assert torch.cuda.is_available(), "torch.cuda.is_available() is False -- see 3.33 step 3"
x = torch.randn(1024, 1024, device="cuda")
torch.cuda.synchronize(); (x @ x).sum().item()
print("torch", torch.__version__, "on", torch.cuda.get_device_name(0), "-- OK")
EOF

say "project requirements (torch pinned so pip cannot replace it with a CPU build)"
printf '%s\n%s\n' "$TORCH" "$VISION" > /tmp/jetson-constraints.txt
pip install -q -r requirements.txt numpy pytest -c /tmp/jetson-constraints.txt \
    --extra-index-url "$INDEX"
pip install -q -r requirements-perception.txt -c /tmp/jetson-constraints.txt \
    --extra-index-url "$INDEX"
python -c "import torch; assert torch.cuda.is_available(), 'a requirement replaced torch with a CPU build'"

say "the shipped pipeline on the GPU"
python - <<'EOF'
from brain.perceive import pipeline_for
p = pipeline_for("red backpack")
print("detector", p.detector.weights, "| scorer device", p.scorer.device)
assert p.scorer.device == "cuda", "CLIP is not on cuda"
EOF

say "docker"
if ! command -v docker >/dev/null; then
    echo "docker is not installed -- 3.33 step 5 needs it (and the NVIDIA runtime)"
else
    docker --version
    docker info 2>/dev/null | grep -i -E "runtimes|default runtime" || true
fi

say "done -- next: pytest tests/ -q, then tools/jetson/bench_perception.py"
