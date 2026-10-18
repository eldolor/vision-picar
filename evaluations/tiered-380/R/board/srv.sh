#!/bin/bash
# 3.80.R: the bench's server for a GGUF. $LLBIN is the llama-server binary
# (a fork or 3.78's Ln), the bench's argv is passed through, and every run
# gets 3.80.M's image-token bounds plus $LLEXTRA (e.g. --special).
echo 1000 > /proc/self/oom_score_adj
exec "$LLBIN" "$@" --image-min-tokens 128 --image-max-tokens 512 $LLEXTRA
