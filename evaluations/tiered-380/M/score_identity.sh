#!/bin/bash
# 3.80.M: score the identity records in M/board/ beside VE (3.48's four walks, gate 0.5).
cd "$(dirname "$0")/../../.."
args=()
for f in evaluations/tiered-380/M/board/*-[AB].json; do n=$(basename "$f" .json); args+=(--records "$n=$f:0.5"); done
.venv/bin/python -m control.identity_eval --recordings /Users/anshugaind/vision-picar/recordings \
  --walk blue-bottle-20260907-142454 --walk blue-bottle-20260907-185007 \
  --walk blue-shoes-20260907-152528 --walk red-backpack-20260907-144856 \
  --records VE=evaluations/identity-378/V/TVE-299.json:0.5 "${args[@]}"
