#!/bin/bash
# 3.80.R: one identity arm on the pinned 299 frames. $1 name, $2 arm (A|N),
# $3 native backend:skill (arm N). Environment: NATIVE_SPEC (a native_server
# model, with NSRV_PY) or MODEL + MMPROJ + LLBIN (+ LLEXTRA) for a GGUF.
pkill -x llama-server 2>/dev/null; pkill -f native_server.py 2>/dev/null; sleep 3
cd ~/vp-380R
R=evaluations/tiered-380/R
if [ -n "$NATIVE_SPEC" ]; then SRV=$R/nsrv.sh; M="$NATIVE_SPEC"; MM=none; else SRV=$R/board/srv.sh; M="$MODEL"; MM="$MMPROJ"; fi
N=(); [ "$2" = N ] && N=(--native "$3")
.venv/bin/python $R/identity.py --arm $2 "${N[@]}" --server $SRV --model "$M" --mmproj "$MM" \
  --recordings ~/vp-378/recordings \
  --walk blue-bottle-20260907-142454 --walk blue-bottle-20260907-185007 \
  --walk blue-shoes-20260907-152528 --walk red-backpack-20260907-144856 \
  --stop-at-decision --out ~/out-380R/$1-$2.json > ~/out-380R/$1-$2.log 2>&1
rc=$?
echo "$(date +%T) $1 arm $2 exit $rc" >> ~/out-380R/ident.done
