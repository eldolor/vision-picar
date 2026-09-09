#!/usr/bin/env bash
# Install the Hailo Dataflow Compiler and its dependencies on the EC2 host.
#
# Run through `tools/hailo/ec2.sh setup`, which finds the wheel in S3 and
# passes its URI. Re-runnable: every step is guarded, so a failed dependency
# can be fixed and the script run again rather than the box rebuilt.
#
#   sudo bash setup_host.sh s3://bucket/hailo/dfc/hailo_dataflow_compiler-*.whl
#
# Ubuntu 22.04 and Python 3.10, and NOT as a preference: Hailo documents the
# Dataflow Compiler as supporting Ubuntu 20.04/22.04 and Python 3.8, 3.9 or
# 3.10 only. 3.11 and 3.12 are not on that list, which is why this does not
# fall back to a newer interpreter and why ec2.sh launches Jammy rather than
# Noble (24.04 ships 3.12). The wheel is tagged py3-none, so pip will install
# it under an unsupported Python and fail later, further in, for reasons that
# look like a model problem.
set -euo pipefail

WHEEL_URI="${1:?usage: setup_host.sh s3://.../hailo_dataflow_compiler-*.whl}"
ROOT=/opt/hailo
VENV=$ROOT/venv

echo "== apt dependencies"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
# graphviz and its dev headers are real DFC requirements (it draws the
# dataflow graph); build-essential and python3-dev are for the packages it
# builds from source.
apt-get install -y -qq \
  build-essential python3.10 python3.10-dev python3.10-venv python3.10-distutils \
  graphviz libgraphviz-dev pkg-config unzip curl \
  python3-pip >/dev/null

if ! command -v aws >/dev/null 2>&1; then
  echo "== awscli"
  curl -sS "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o /tmp/awscli.zip
  unzip -q -o /tmp/awscli.zip -d /tmp && /tmp/aws/install --update >/dev/null
fi

mkdir -p $ROOT/dfc
echo "== fetching the wheel"
aws s3 cp "$WHEEL_URI" $ROOT/dfc/ --only-show-errors
WHEEL="$(ls $ROOT/dfc/*.whl | head -1)"
echo "   $WHEEL"

PY=python3.10
command -v $PY >/dev/null || { echo "!! $PY missing -- this host is not Ubuntu 22.04"; exit 1; }
echo "== installing into a $PY venv ($($PY --version 2>&1))"
rm -rf "$VENV"
$PY -m venv "$VENV"
"$VENV/bin/pip" install -q --upgrade pip setuptools wheel
# pygraphviz is a frequent DFC dependency and needs the headers installed
# above; doing it first makes a headers failure name itself.
"$VENV/bin/pip" install -q pygraphviz >/dev/null 2>&1 || true
"$VENV/bin/pip" install "$WHEEL"

echo "== supporting packages"
# numpy/pillow are almost certainly pulled in by the DFC already; onnx and
# onnxruntime are for inspecting the graph and for a CPU reference run on
# the same host. Pinned to nothing: whatever satisfies the DFC's own pins.
"$VENV/bin/pip" install -q onnx onnxruntime pillow || true

echo "== versions"
"$VENV/bin/python" - <<'PY'
import importlib, platform
print("python      ", platform.python_version())
for mod in ("numpy", "onnx", "onnxruntime", "hailo_sdk_client", "hailo_sdk_common"):
    try:
        m = importlib.import_module(mod)
        print(f"{mod:<12}", getattr(m, "__version__", "(no __version__)"))
    except Exception as exc:
        print(f"{mod:<12} MISSING: {type(exc).__name__}: {exc}")
PY

echo "== hailo cli"
"$VENV/bin/hailo" --version 2>&1 | head -3 || echo "   (no hailo cli on PATH -- the Python API is what the loop uses)"

chown -R ubuntu:ubuntu $ROOT
echo "== ready: $VENV"
