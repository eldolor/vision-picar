#!/bin/bash
# 3.80.K, on the board: RT and RVE engines from the export host's ONNX, with
# VE's flags (KV 1024, input 1024, 128-512 image tokens). The LLM build OOMs
# at 8 GB (3.78), so the twin's units stop and a temporary 8 GB swapfile is
# added for the build only; both are restored after.
set -u
O=~/out-380K; mkdir -p $O
B=~/bench-378/ell91/build/examples
export EDGELLM_PLUGIN_PATH=$HOME/bench-378/ell91/build/libNvInfer_edgellm_plugin.so
sudo systemctl stop picar-stack picar-ngrok; sleep 3
sudo fallocate -l 8G /swap-380k && sudo chmod 600 /swap-380k && sudo mkswap /swap-380k >/dev/null && sudo swapon /swap-380k
for v in RT RVE; do
  W=~/bench-380K/$v
  s=$(date +%s); $B/llm/llm_build --onnxDir $W/onnx/llm --engineDir $W/engines/llm --maxBatchSize 1 --maxInputLen 1024 --maxKVCacheCapacity 1024 > $O/$v-llm_build.log 2>&1; echo "$v llm_build exit $? $(( $(date +%s)-s ))s" | tee -a $O/build.log
  s=$(date +%s); $B/multimodal/visual_build --onnxDir $W/onnx/visual --engineDir $W/engines --minImageTokens 128 --maxImageTokens 512 --maxImageTokensPerImage 512 > $O/$v-visual_build.log 2>&1; echo "$v visual_build exit $? $(( $(date +%s)-s ))s" | tee -a $O/build.log
  du -sh $W/engines/* | tee -a $O/build.log
done
sudo swapoff /swap-380k && sudo rm -f /swap-380k
sudo systemctl start picar-stack picar-ngrok
echo "BUILD DONE; units $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); swap $(swapon --show=NAME --noheadings | grep -c swap-380k)" | tee -a $O/build.log
