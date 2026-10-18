#!/bin/bash
# 3.80.J: one llama-server on :8006. $1 name, $2 model gguf, $3 mmproj gguf.
pkill -x llama-server 2>/dev/null; sleep 2
echo "$(date +%T) MemAvailable before $1: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)" >> ~/out-380M/$1.mem
(echo 1000 > /proc/self/oom_score_adj; exec setsid nohup ~/bench-348/llama.cpp/build/bin/llama-server -m "$2" --mmproj "$3" -ngl 99 -c 4096 -np 1 --cache-ram 0 --port 8006 --host 127.0.0.1 --image-min-tokens 128 --image-max-tokens 512 >> ~/out-380M/$1-server.log 2>&1 < /dev/null) &
for i in $(seq 120); do curl -s localhost:8006/health | grep -q ok && break; sleep 2; done
echo "$(date +%T) MemAvailable with $1: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)" | tee -a ~/out-380M/$1.mem
