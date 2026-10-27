#!/bin/bash
# 3.80.U: one of the shortlist's top five on another frame set (the Rover's),
# all on the board, so no tunnel. $1 ve|bonsai|minicpm|ivl4b|florence.
# Environment: WALKS (the identity walks, space-separated), PICAR_380_SETS
# (the steering set's directory, U/make_sets.py's), REC (the walks, default
# ~/vision-picar/recordings), TAG (default rover).
# Identity first (not Florence: it boxes every frame), then steering.
# MemAvailable every second through both, in ~/out-380U/<tag>-<model>.mem,
# with each phase's start in <tag>-<model>.marks. VE and Florence steer
# through S/board/run.sh (YOLOE + CLIP alone, the second model alone and the
# gate from one run; its rows and memory go to ~/out-380S/<tag>-<model>-gate.*).
# The GGUF rows run on ~/bench-378/llama.cpp (7f51953d7): 3.80.M's e60eff9
# build was deleted 2026-10-19.
set -u
M=$1; TAG=${TAG:-rover}; N=$TAG-$M
REC=${REC:-$HOME/vision-picar/recordings}
: "${WALKS:?WALKS: the identity walks}"; : "${PICAR_380_SETS:?PICAR_380_SETS: the steering set}"
export PICAR_380_SETS PICAR_380_RECORDINGS=$REC REC
OUT=~/out-380U; V=~/vp-380R; E=$V/evaluations/tiered-380; PY=$V/.venv/bin/python
mkdir -p $OUT ~/out-380R ~/out-380S
W=(); for w in $WALKS; do W+=(--walk "$w"); done
MINE='native_server\.py -m [a-z0-9-]*:.* --port 8006'
if pgrep -x llama-server >/dev/null || pgrep -f "$MINE" >/dev/null; then
  echo "STOP: a bench server is running"; exit 2
fi
# 3.108: the production unit is enabled at boot; held off for the run (after
# the refusals above, so they cost no reload) and put back as found at exit.
source "$(dirname "${BASH_SOURCE[0]}")/../../../../tools/jetson/vlm_hold.sh" || { echo "STOP: no vlm_hold.sh"; exit 2; }
trap vlm_restore EXIT
vlm_hold "$OUT/$N.vlm_hold.log" || exit 2
( while :; do echo "$(date +%s) $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > $OUT/$N.mem &
SAMP=$!
trap 'kill $SAMP 2>/dev/null; vlm_restore' EXIT
sleep 3
mark() { echo "$(date +%s) $1" >> $OUT/$N.marks; }
log() { echo "$(date +%T) $N $*" | tee -a $OUT/done.log; }
: > $OUT/$N.marks

# A GGUF row: identity (arm A, 3.48's JSON prompt), then steering on :8006.
# $1 the identity client (R/identity.py or M/llama_identity.py), $2 the
# steering client (R/steer.py or M/run_steer.py), whose rows land in its own
# directory as <tag>-<model>.jsonl.
gguf() {
  export MODEL MMPROJ LLBIN LLEXTRA=${LLEXTRA:-}
  mark identity
  (cd $V && $PY $E/$1 --arm A --server $E/R/board/srv.sh --model "$MODEL" --mmproj "$MMPROJ" \
     --recordings $REC "${W[@]}" --stop-at-decision --out $OUT/$N-A.json > $OUT/$N-A.log 2>&1)
  log "identity exit $?"
  pkill -x llama-server; sleep 3
  mark steering
  $E/R/board/serve.sh $N || { log "steering server never healthy"; return 1; }
  rm -f $E/$(dirname $2)/$N.jsonl   # the clients resume from rows already there
  (cd $V && $PY $E/$2 $N http://127.0.0.1:8006 > $OUT/$N-steer.log 2>&1)
  log "steering exit $?"
  pkill -x llama-server
  cp $E/$(dirname $2)/$N.jsonl $OUT/ 2>/dev/null
  mark end
}

case $M in
  ve)
    ENG=$HOME/bench-378/ws91/q25VE/engines; ELL=$HOME/bench-378/ell91
    mark identity
    (cd $V && $PY -m tools.jetson.bench_identity_trt --driver $ELL/build/examples/llm/edgellm_identity \
       --engine-dir $ENG/llm --visual-engine-dir $ENG/visual --tokenizer $ENG/llm/tokenizer.json \
       --plugin $ELL/build/libNvInfer_edgellm_plugin.so --recordings $REC "${W[@]}" \
       --out $OUT/$N-A.json > $OUT/$N-A.log 2>&1)
    log "identity exit $?"
    mark steering
    $E/S/board/run.sh orig ve $N-gate; log "steering exit $?"
    mark end;;
  florence)
    mark steering
    $E/S/board/run.sh orig florence $N-gate; log "steering exit $?"
    mark end;;
  bonsai)
    B=$HOME/models-380R/Ternary-Bonsai-2-2B-gguf
    MODEL=$B/Ternary-Bonsai-2-2B-PQ2_0.gguf; MMPROJ=$B/Ternary-Bonsai-2-2B-mmproj-Q8_0.gguf
    LLBIN=$HOME/bench-380R/prism/build/bin/llama-server
    export LLAMA_ARG_REASONING=off
    gguf R/identity.py R/steer.py;;
  minicpm)
    MODEL=$HOME/models-380M/ggml-model-Q4_K_M.gguf; MMPROJ=$HOME/models-380M/mmproj-model-f16.gguf
    LLBIN=$HOME/bench-378/llama.cpp/build/bin/llama-server
    gguf M/llama_identity.py M/run_steer.py;;
  ivl4b)
    MODEL=$HOME/models-380M/OpenGVLab_InternVL3_5-4B-Q4_K_M.gguf
    MMPROJ=$HOME/models-380M/mmproj-OpenGVLab_InternVL3_5-4B-f16.gguf
    LLBIN=$HOME/bench-378/llama.cpp/build/bin/llama-server
    gguf M/llama_identity.py M/run_steer.py;;
  *) echo "bad model $M"; exit 2;;
esac
log "done; OOM kills since boot: $(sudo dmesg | grep -c 'Out of memory')"
