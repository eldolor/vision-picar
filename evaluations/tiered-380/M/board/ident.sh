#!/bin/bash
# 3.80.M: both identity arms for one model. $1 name, $2 model, $3 mmproj.
pkill -x llama-server; sleep 3
cd ~/vp-380
for arm in A B; do
  ~/vp-380/.venv/bin/python evaluations/tiered-380/M/llama_identity.py --arm $arm \
    --server ~/out-380M/srvwrap.sh --model "$2" --mmproj "$3" --recordings ~/vp-378/recordings \
    --walk blue-bottle-20260907-142454 --walk blue-bottle-20260907-185007 \
    --walk blue-shoes-20260907-152528 --walk red-backpack-20260907-144856 \
    --stop-at-decision --out ~/out-380M/$1-$arm.json > ~/out-380M/$1-$arm.log 2>&1
  echo "$1 arm $arm exit $?" >> ~/out-380M/ident.done
done
