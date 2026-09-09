#!/usr/bin/env bash
# Install the Hailo Dataflow Compiler and its dependencies on the EC2 host.
#
# Run through `tools/hailo/ec2.sh setup`, which finds the wheel in S3 and
# passes its URI. Re-runnable: every step is guarded, so a failed dependency
# can be fixed and the script run again rather than the box rebuilt.
#
#   sudo bash setup_host.sh s3://bucket/hailo/dfc/hailo_dataflow_compiler-*.whl
#
# Ubuntu 22.04 and Python 3.10, and NOT as a preference. Read out of the
# 3.34.0 wheel itself rather than from the docs, which are stale on this:
#
#   Requires-Python  NOT DECLARED   -- pip will not gate the interpreter at
#                                      all, so a wrong one fails much later
#                                      and looks like a model problem
#   jax==0.5.3, networkx==3.4.2     -- both need Python >= 3.10, so the
#                                      3.8/3.9 in Hailo's published docs are
#                                      no longer true for 3.34
#   tensorflow==2.18.0, numpy==1.26.4, onnxruntime==1.18.0
#                                   -- ceilings around 3.12
#
# So the real window for 3.34.0 is 3.10-3.12 and Jammy's 3.10 sits inside it.
# ec2.sh launches Jammy rather than Noble for this reason (24.04 ships 3.12,
# which is at the top edge rather than the middle of that window).
#
# No ABI-tagged .so in the wheel -- the 47 native libraries are plain shared
# objects (HSim, or-tools), not cpython-3XX extensions -- so nothing binds it
# to one interpreter beyond those pins.
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

echo "== dependency check"
# Deliberately NOT installing onnx/onnxruntime/numpy here. The DFC hard-pins
# onnx==1.16.0, onnxruntime==1.18.0, numpy==1.26.4 and protobuf==3.20.3, and
# a bare `pip install onnx onnxruntime` after the wheel silently upgrades
# them and breaks the compiler in ways that surface as a parse failure --
# i.e. as an answer to the question this loop is asking. `pip check` is the
# guard: if it complains, the environment is not the one Hailo tested.
"$VENV/bin/pip" check || echo "!! pip check reported conflicts -- see above"

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
