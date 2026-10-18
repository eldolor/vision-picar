#!/bin/bash
# 3.80.N, on the rented export host (g6.2xlarge, DL base AMI, Ubuntu 22.04):
# InternVL3.5-4B-HF -> Edge-LLM v0.9.1 ONNX, as 3.80.K made RoboBrain's. NT: INT4
# AWQ LLM, full vocabulary. NVE: reduced vocabulary + FP8 embedding (VE's recipe).
# No IAM role: inputs arrive in $W (bundle.tar, via a presigned GET in the user
# data); results leave through the presigned PUT links in $W/put.env. The host
# shuts down (terminate-on-shutdown) at DONE, on error, or at 3 h.
set -euo pipefail
W=/opt/n; cd $W
. $W/put.env
exec > >(tee -a $W/export.log) 2>&1
put() { curl -sS --retry 3 -X PUT -T "$1" "$2" -o /dev/null || true; }
step() { echo "== $(date +%T) $*"; put $W/export.log "$LOG_PUT"; }
trap 'echo "== $(date +%T) FAILED line $LINENO"; put $W/export.log "$LOG_PUT"; sudo shutdown -h now' ERR
step "setup"
dpkg -s python3-venv >/dev/null 2>&1 || { sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq; sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-venv git-lfs; }
test -d ell || git clone -q --depth 1 -b v0.9.1 https://github.com/NVIDIA/TensorRT-Edge-LLM ell
(cd ell && git submodule update --init --recursive -q)
test -x $HOME/.local/bin/uv || curl -LsSf https://astral.sh/uv/install.sh | sh
U=$HOME/.local/bin/uv
test -x venv/bin/python || $U venv -q -p 3.12 venv
. venv/bin/activate
(cd ell && $U pip install -q . && $U pip install -q ".[tools]")
export PYTHONPATH=$W/ell
grep -rl "\"cnn_dailymail\"" ell/tensorrt_edgellm | xargs -r sed -i 's/"cnn_dailymail"/"abisee\/cnn_dailymail"/g'
python -c "import torch; print('torch', torch.__version__, torch.cuda.get_device_name(0))"
step "weights"
M=InternVL3_5-4B-HF
python - <<'PY'
from huggingface_hub import snapshot_download
snapshot_download("OpenGVLab/InternVL3_5-4B-HF", revision="6bd4487402110ef9889ba50eb7aefeb302526fed",
                  local_dir="InternVL3_5-4B-HF", allow_patterns=["*.json", "*.jinja", "*.txt", "*.safetensors"])
PY
(cd $M && sha256sum -c ../SHA256SUMS)
step "quantize int4_awq"
tensorrt-edgellm-quantize llm --model_dir $M --quantization int4_awq --output_dir q
step "export NT"
tensorrt-edgellm-export q onnx-NT
step "upload NT"
tar -cf onnx-NT.tar onnx-NT && ls -la onnx-NT.tar && put onnx-NT.tar "$NT_PUT"
step "reduce vocab"
tensorrt-edgellm-reduce-vocab --model_dir $M --output_dir rv16k --reduced_vocab_size 16384 --method input_aware --max_samples 50000
tensorrt-edgellm-reduce-vocab --model_dir $M --output_dir rv32k --reduced_vocab_size 32768 --method input_aware --max_samples 50000
python forced_internvl.py $M forced_ids.json ivl4b.jsonl forced.json
python edgellm_vocab_map.py --base rv16k --pad rv32k --forced forced.json --vocab-size 151679 --out reduced_vocab
cat reduced_vocab/reduced_vocab.json
put forced.json "$FORCED_PUT"
step "export NVE"
tensorrt-edgellm-export q onnx-NVE --reduced-vocab-dir reduced_vocab/ --fp8-embedding
step "upload NVE"
tar -cf onnx-NVE.tar onnx-NVE && ls -la onnx-NVE.tar && put onnx-NVE.tar "$NVE_PUT"
step "DONE"
sudo shutdown -h now
