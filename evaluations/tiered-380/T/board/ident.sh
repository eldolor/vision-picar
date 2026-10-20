#!/bin/bash
# 3.80.T run 1 on the board: the pinned 299 frames through R/board/identity.sh
# (arm N, `detect {target}`), PaliGemma 2 at NF4 under ~/bench-380T's venv,
# with MemAvailable every second from before the server starts to the end,
# in ~/out-380R/$TAG.mem. $1 the backend (paligemma-nf4 | paligemma-nf4lm),
# $2 the tag.
set -u
SPEC=$1; TAG=$2; OUT=~/out-380R
pgrep -f native_server.py >/dev/null && { echo "STOP: a native_server is running"; exit 2; }
( while :; do echo "$(date +%s) $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > $OUT/$TAG.mem &
SAMP=$!
sleep 3
cd ~/vp-380R/evaluations/tiered-380/R/board && NATIVE_SPEC=$SPEC:$HOME/models-380R/paligemma2-3b-mix-448 \
  NSRV_PY=$HOME/bench-380T/venv/bin/python ./identity.sh $TAG N paligemma:detect
kill $SAMP
echo "$(date +%T) $TAG done; OOM kills since boot: $(sudo dmesg | grep -c 'Out of memory')"
