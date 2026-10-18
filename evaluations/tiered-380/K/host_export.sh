#!/bin/bash
# 3.80.K, on the rented export host (g6.2xlarge, DL base AMI, Ubuntu 22.04):
# RoboBrain2.0-3B -> Edge-LLM v0.9.1 ONNX, as 3.78 made VE. RT: INT4 AWQ LLM,
# full vocabulary. RVE: the same checkpoint, reduced vocabulary + FP8 embedding.
# Inputs in $W: urls.txt (signed links), SHA256SUMS, model/ (small files),
# forced_ids.json, robobrain-q4.jsonl, forced_extra.py, edgellm_vocab_map.py.
# Results -> s3://$BUCKET/$PREFIX/result/. Self-terminates at 3 h (set at up).
set -euo pipefail
W=/opt/k; cd $W
S3="$1"
exec > >(tee -a $W/export.log) 2>&1
step() { echo "== $(date +%T) $*"; aws s3 cp $W/export.log "$S3/result/export.log" --only-show-errors || true; }
step "setup"
dpkg -s python3-venv >/dev/null 2>&1 || { sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq; sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-venv git-lfs; }
test -d ell || git clone -q --depth 1 -b v0.9.1 https://github.com/NVIDIA/TensorRT-Edge-LLM ell
(cd ell && git submodule update --init --recursive -q)
# Edge-LLM 0.9.1 needs Python >= 3.11 (numpy 2.4.6); the AMI's is 3.10.
test -x $HOME/.local/bin/uv || curl -LsSf https://astral.sh/uv/install.sh | sh
U=$HOME/.local/bin/uv
test -x venv/bin/python || $U venv -q -p 3.12 venv
. venv/bin/activate
(cd ell && $U pip install -q . && $U pip install -q ".[tools]")
export PYTHONPATH=$W/ell
# 3.78's patch: this huggingface_hub rejects the bare dataset id.
grep -rl "\"cnn_dailymail\"" ell/tensorrt_edgellm | xargs -r sed -i 's/"cnn_dailymail"/"abisee\/cnn_dailymail"/g'
python -c "import torch; print('torch', torch.__version__, torch.cuda.get_device_name(0))"
step "weights"
mkdir -p RoboBrain2.0-3B && cp -r model/. RoboBrain2.0-3B/
while read f u; do curl -sSL --retry 5 -o "RoboBrain2.0-3B/$f" "$u"; done < urls.txt
(cd RoboBrain2.0-3B && sha256sum -c ../SHA256SUMS)
step "quantize int4_awq"
tensorrt-edgellm-quantize llm --model_dir RoboBrain2.0-3B --quantization int4_awq --output_dir rb-q
step "export RT"
tensorrt-edgellm-export rb-q onnx-RT
step "reduce vocab"
tensorrt-edgellm-reduce-vocab --model_dir RoboBrain2.0-3B --output_dir rv16k --reduced_vocab_size 16384 --method input_aware --max_samples 50000
tensorrt-edgellm-reduce-vocab --model_dir RoboBrain2.0-3B --output_dir rv32k --reduced_vocab_size 32768 --method input_aware --max_samples 50000
python forced_extra.py RoboBrain2.0-3B forced_ids.json robobrain-q4.jsonl forced.json
python edgellm_vocab_map.py --base rv16k --pad rv32k --forced forced.json --vocab-size 151665 --out reduced_vocab
cat reduced_vocab/reduced_vocab.json
step "export RVE"
tensorrt-edgellm-export rb-q onnx-RVE --reduced-vocab-dir reduced_vocab/ --fp8-embedding
step "upload"
for d in onnx-RT onnx-RVE; do tar -cf $d.tar $d; aws s3 cp $d.tar "$S3/result/$d.tar" --only-show-errors; done
cp forced.json reduced_vocab/reduced_vocab.json $W/ 2>/dev/null || true
aws s3 cp forced.json "$S3/result/forced.json" --only-show-errors
step "DONE"
