#!/bin/bash
# P16 phase 1 -- does YOLO-World ALLOCATE on a Hailo-10H?
#
# P15 settled that DFC 5.4.0 parses everything on `hailo10h`. Parse is not
# the stage that kills models on this family: P6 watched OWLv2 translate
# AND quantize on the 8L and then die at **allocation**, on 73 layernorm
# and 38 softmax layers. So the go/no-go for the 10H is a full compile to
# a HEF, and that is what this does.
#
# Runs inside the vendor's AI Software Suite container, because P15 traced
# P14's wrong conclusion to the environment we built by hand.
#
#   E1  yolov11s        -- the control. A Hailo-native CNN that must
#                          produce a HEF; if it does not, the box or the
#                          calibration path is wrong and E2 means nothing.
#   E2  yolo_world_v2s  -- the decisive one. Hailo's OWN YOLO-World.
#
# NOTE E2 is NOT the graph P9 measured at 72% -- that was our YOLOv8-based
# export with a 32-word vocabulary. If the vendor's compiles and ours does
# not (or the reverse), the difference IS the finding, which is why the
# comparable run against our own ONNX is a separate track and not folded
# in here as a proxy.
set -uo pipefail

SHARE=/local/shared_with_docker
OUT="$SHARE/out"
CALIB="$SHARE/calib"
mkdir -p "$OUT" || { echo "cannot create $OUT" >&2; exit 1; }
REPORT="$OUT/compile_report.tsv"
: > "$REPORT"

CASES=(
  'E1|yolov11s'
  'E2|yolo_world_v2s'
)

printf 'row\tmodel\tarch\tstatus\thef_mb\tdetail\n' | tee -a "$REPORT"

for c in "${CASES[@]}"; do
  IFS='|' read -r row model <<< "$c"
  log="$OUT/$row.$model.compile.log"
  echo "=== $row: hailomz compile $model --hw-arch hailo10h ==="

  # 4h ceiling: compile is the long stage, and a wedged job on a $1/hr box
  # is how the $12 overnight burn happened.
  if timeout 14400 hailomz compile "$model" \
        --hw-arch hailo10h --calib-path "$CALIB" > "$log" 2>&1; then
    status=OK
    hef="$(find /local/workspace -maxdepth 2 -name "$model*.hef" -newermt '-4 hours' | head -1)"
    if [ -n "$hef" ]; then
      mb="$(du -m "$hef" | cut -f1)"
      cp "$hef" "$OUT/" 2>/dev/null
    else
      mb="?"
    fi
    detail="$(grep -iE 'hef|context' "$log" | tail -1)"
  else
    status=FAIL
    mb="-"
    # Allocation failures are the ones worth reading verbatim -- P6's whole
    # result was the LIST of layers the allocator refused.
    detail="$(grep -iE 'error|exception|allocat|resources|unfeasible' "$log" | head -1)"
    [ -n "$detail" ] || detail="$(tail -2 "$log" | tr '\n' ' ')"
  fi
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$row" "$model" hailo10h "$status" "$mb" "${detail:0:200}" | tee -a "$REPORT"
done

echo
echo "=== compile matrix complete ==="
column -t -s $'\t' "$REPORT" 2>/dev/null || cat "$REPORT"
