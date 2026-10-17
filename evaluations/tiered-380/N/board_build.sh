#!/bin/bash
# 3.80.N, on the board: NT and NVE LLM engines from the export host's ONNX
# (KV and input 1280, room for 1024 image tokens), and two visual engines each
# (V512: 256-512 image tokens; V1024: 256-1024). Engine sets are
# ~/bench-380N/<NT|NVE>/<V512|V1024>/{llm -> ../llm, visual}. The LLM build OOMs
# at 8 GB (3.78), so the twin's units stop and a temporary 8 GB swapfile is
# added for the build only; both are restored after.
set -u
O=~/out-380N; mkdir -p $O
B=~/bench-378/ell91/build/examples
export EDGELLM_PLUGIN_PATH=$HOME/bench-378/ell91/build/libNvInfer_edgellm_plugin.so
sudo systemctl stop picar-stack picar-ngrok; sleep 3
sudo fallocate -l 8G /swap-380n && sudo chmod 600 /swap-380n && sudo mkswap /swap-380n >/dev/null && sudo swapon /swap-380n
for v in NT NVE; do
  W=~/bench-380N/$v
  s=$(date +%s); $B/llm/llm_build --onnxDir $W/onnx/llm --engineDir $W/llm --maxBatchSize 1 --maxInputLen 1280 --maxKVCacheCapacity 1280 > $O/$v-llm_build.log 2>&1; echo "$v llm_build exit $? $(( $(date +%s)-s ))s" | tee -a $O/build.log
  for t in 512 1024; do
    E=$W/V$t; mkdir -p $E; ln -sfn ../llm $E/llm
    s=$(date +%s); $B/multimodal/visual_build --onnxDir $W/onnx/visual --engineDir $E --minImageTokens 256 --maxImageTokens $t --maxImageTokensPerImage $t > $O/$v-V$t-visual_build.log 2>&1; echo "$v V$t visual_build exit $? $(( $(date +%s)-s ))s" | tee -a $O/build.log
  done
  du -shL $W/llm $W/V512/visual $W/V1024/visual | tee -a $O/build.log
done
sudo swapoff /swap-380n && sudo rm -f /swap-380n
sudo systemctl start picar-stack picar-ngrok
echo "BUILD DONE; units $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); swap $(swapon --show=NAME --noheadings | grep -c swap-380n)" | tee -a $O/build.log
