#!/bin/bash
# 3.80.U from the Mac: copy the harness, the steering set and its walks to the
# board, run the top five in turn there (U/board/top5.sh), and copy the outputs
# back to U/board/out/<tag>/. $1 the tag. Environment: WALKS (identity walks,
# space-separated), SET (the steering set's directory, U/make_sets.py's),
# STEER_WALKS (the walks the set draws from; default WALKS), MODELS (default
# all five), REC (the Mac's walks, default ./recordings).
set -u
TAG=$1; : "${WALKS:?}"; : "${SET:?}"
MODELS=${MODELS:-ve bonsai minicpm ivl4b florence}
HERE="$(cd "$(dirname "$0")" && pwd)"; ROOT="$HERE/../../.."
REC=${REC:-$ROOT/recordings}
BREC=vision-picar/recordings; BSET=vp-380R/evaluations/tiered-380/U/sets/$TAG
rsync -a --exclude __pycache__ --exclude '*.jsonl' --exclude 'board/out' \
  "$ROOT/evaluations/tiered-380/" picar-jetson:vp-380R/evaluations/tiered-380/
rsync -a "$ROOT/tools/jetson/bench_identity_trt.py" "$ROOT/tools/jetson/bench_identity_llama.py" \
  picar-jetson:vp-380R/tools/jetson/
ssh picar-jetson "mkdir -p $BSET"
rsync -a "$SET/" picar-jetson:$BSET/
for w in $(echo $WALKS ${STEER_WALKS:-} | tr ' ' '\n' | sort -u); do
  rsync -a "$REC/$w" picar-jetson:$BREC/
done
for m in $MODELS; do
  echo "$(date +%T) $TAG $m"
  ssh picar-jetson "TAG=$TAG WALKS='$WALKS' PICAR_380_SETS=\$HOME/$BSET REC=\$HOME/$BREC \
    bash vp-380R/evaluations/tiered-380/U/board/top5.sh $m" || echo "$(date +%T) $TAG $m exit $?"
done
mkdir -p "$HERE/board/out/$TAG"
rsync -a "picar-jetson:out-380U/$TAG-*" "picar-jetson:out-380U/done.log" "$HERE/board/out/$TAG/"
rsync -a "picar-jetson:out-380S/$TAG-*" "$HERE/board/out/$TAG/" 2>/dev/null
echo "$(date +%T) $TAG DONE"
