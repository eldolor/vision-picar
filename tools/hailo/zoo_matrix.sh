#!/bin/bash
# The P15 parse matrix, run INSIDE the Hailo AI Software Suite container.
#
# A separate file on purpose. The first version of this lived in a heredoc
# nested inside a double-quoted string inside an SSM payload, and the outer
# shell ate the `"${CASES[@]}"` expansion before the heredoc was ever
# written -- so the matrix ran one row, wrote nothing, and reported a FAIL
# that meant only that its own output directory did not exist. A run that
# fails for a harness reason and LOOKS like a result is the exact hazard
# P11 was, so the matrix gets to be a file that is uploaded verbatim.
#
# Reads nothing from the environment. Writes out/report.tsv and one log per
# row, both under the bind-mounted share.
set -uo pipefail

SHARE=/local/shared_with_docker
OUT="$SHARE/out"
mkdir -p "$OUT" || { echo "cannot create $OUT" >&2; exit 1; }
REPORT="$OUT/report.tsv"
: > "$REPORT"

# row | model | hw_arch | optional ONNX override (--ckpt)
#
# A* are Hailo's own ONNX, Hailo's own end-node cut, Hailo's own container.
# B1 changes ONE variable: our opset-13 / pytorch-2.14 export, same config.
# C1 is the harness control -- `hailo8l` is not a valid --hw-arch for the
#    5.x line at all, so it must come back FAIL. If it does not, the matrix
#    cannot report a failure and no other row means anything.
CASES=(
  'A1|yolov11s|hailo10h|'
  'A2|yolov11s|hailo15h|'
  'A3|yolov8s|hailo10h|'
  'A4|yolo_world_v2s|hailo10h|'
  "B1|yolov11s|hailo10h|$SHARE/ours_yolo11s_op13.onnx"
  'C1|yolov11s|hailo8l|'
)

printf 'row\tmodel\tarch\tonnx\tstatus\tdetail\n' | tee -a "$REPORT"

for c in "${CASES[@]}"; do
  IFS='|' read -r row model arch ckpt <<< "$c"
  log="$OUT/$row.$model.$arch.log"
  args=(parse "$model" --hw-arch "$arch")
  src=hailo
  if [ -n "$ckpt" ]; then
    args+=(--ckpt "$ckpt")
    src=ours-op13
  fi

  echo "=== $row: hailomz ${args[*]} ==="
  if timeout 1800 hailomz "${args[@]}" > "$log" 2>&1; then
    status=OK
    detail="$(grep -iE 'saved|\.har' "$log" | tail -1)"
  else
    status=FAIL
    # The LAST error line is usually the argparse/wrapper complaint; the
    # informative one is the first real exception. Take both ends.
    detail="$(grep -iE 'error|exception|unfeasible|invalid choice' "$log" | head -1)"
    [ -n "$detail" ] || detail="$(tail -2 "$log" | tr '\n' ' ')"
  fi
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$row" "$model" "$arch" "$src" "$status" "${detail:0:200}" | tee -a "$REPORT"
done

echo
echo "=== matrix complete ==="
column -t -s $'\t' "$REPORT" 2>/dev/null || cat "$REPORT"
