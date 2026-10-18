#!/bin/bash
# 3.80.M: llama-server for the identity bench, with 3.80.J's image-token
# bounds and the OOM killer's first pick (as srv.sh starts it for steering).
echo 1000 > /proc/self/oom_score_adj
exec ~/bench-348/llama.cpp/build/bin/llama-server "$@" --image-min-tokens 128 --image-max-tokens 512
