#!/bin/bash
# P16 phase 2 -- the whole reactive tier, on a Hailo-10H, with a real GPU.
#
# Phase 1 proved YOLO-World ALLOCATES on the 10H (11.9 MB HEF, 5 contexts).
# It could not say anything about accuracy: that box had no GPU, so the DFC
# printed P11's exact warning -- "Reducing optimization level to 0 ...
# there's no available GPU" -- and skipped every accuracy pass.
#
# This box has an A10G, and TensorFlow sees it INSIDE the container (the
# check that must pass before any number here is worth reading; P13's host
# needed tensorflow[and-cuda] before tf saw the card, and without it the
# run still reports success while silently skipping the passes).
#
# Two different questions, deliberately not mixed in the report:
#
#   Q*  ACCURACY. YOLO-World at optimization level 2 (QAT), scored the same
#       way P13 scored it on the 8L, against the SAME fp32 baseline. This is
#       the 45%-vs-72% decision.
#   A*  ALLOCATION. Does the model place on the chip at all. A HEF is not a
#       recall number and is never reported as one.
#
# A* covers the whole tier, not just the detector, because the real
# question on hardware day is what is left running on the Pi's four cores
# beside lidar at 10Hz and P7b's 229 ms/frame resize (handoff open item 1,
# which it calls a bigger risk than the accelerator choice):
#
#   A1 OWLv2      -- the 82% model. P6 killed it on the 8L at ALLOCATION,
#                    on 73 layernorm and 38 softmax layers. That was a
#                    different chip on a different compiler line. Not in
#                    the zoo, so this is our own banked clsfold export,
#                    verified an exact identity at max |diff| 0.0.
#   A2 CLIP RN50  -- the classifier the shipped pipeline actually uses.
#   A3 SegFormer  -- the floor mask (brain/perceive.py CROP_FLOOR_MASK).
#                    NOTE the zoo's is the CITYSCAPES variant; ours is
#                    ADE20K, which is what carries floor/rug/earth. A pass
#                    here proves the ARCHITECTURE places, not that their
#                    weights do our job.
#
# One heavy DFC job at a time, in sequence: P13 had two on one host
# OOM-kill each other and cost $12 overnight for nothing.
set -uo pipefail

SHARE=/local/shared_with_docker
OUT="$SHARE/out"
CW="$SHARE/cw"
mkdir -p "$OUT" || { echo "cannot create $OUT" >&2; exit 1; }
REPORT="$OUT/phase2_report.tsv"
: > "$REPORT"
printf 'row\tkind\tmodel\tconfig\tstatus\tdetail\n' | tee -a "$REPORT"

cd "$CW" || exit 1

emit() { printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$1" "$2" "$3" "$4" "$5" "${6:0:200}" | tee -a "$REPORT"; }

# ---------------------------------------------------------------- Q1/Q2 ---
# quantized_detect.py is P13's own tool. It already encodes the two traps
# that silently produce a wrong answer: normalization() applies in
# SDK_QUANTIZED and NOT in SDK_NATIVE, and --optimization-level must be
# FORCED or the DFC quietly drops to 0.
for spec in "Q1|2|" "Q2|2|a16_w8_a16"; do
  IFS='|' read -r row lvl promote <<< "$spec"
  log="$OUT/$row.yoloworld.log"
  args=(--arch hailo10h
        --onnx build/yoloworld/yoloworld-vocab32.onnx
        --calib build/yoloworld/calib_uint8.npy
        --head build/yoloworld/yoloworld-head.onnx
        --frames build/yoloworld/frames.txt
        --optimization-level "$lvl"
        --out "$OUT/det_${row}.json")
  [ -n "$promote" ] && args+=(--promote "$promote")

  echo "=== $row: quantized_detect ${args[*]} ==="
  if timeout 21600 python -m tools.hailo.quantized_detect "${args[@]}" > "$log" 2>&1; then
    n=$(python -c "import json;d=json.load(open('$OUT/det_${row}.json'));print(sum(len(v) for v in d.values()))" 2>/dev/null || echo '?')
    emit "$row" accuracy yolo-world "L$lvl${promote:+ +$promote}" OK "detections=$n"
  else
    emit "$row" accuracy yolo-world "L$lvl${promote:+ +$promote}" FAIL \
      "$(grep -iE 'error|exception|traceback' "$log" | head -1)"
  fi
done

# ------------------------------------------------------------------- A1 ---
# Our own tool and our own calibration set, so no TFDS dependency and a
# direct comparison with P6's 8L attempt.
log="$OUT/A1.owlv2.log"
echo "=== A1: OWLv2 -> hailo10h ==="
if timeout 21600 python -m tools.hailo.compile_owlv2 \
      --build build/owlv2 --arch hailo10h \
      --report "$OUT/owlv2_10h_report.json" > "$log" 2>&1; then
  hef=$(find build/owlv2 -name '*.hef' | head -1)
  emit A1 allocation owlv2 hailo10h "${hef:+OK}${hef:-FAIL}" "${hef:-no HEF written}"
else
  emit A1 allocation owlv2 hailo10h FAIL \
    "$(grep -iE 'error|exception|allocat|resources' "$log" | head -1)"
fi

# --------------------------------------------------------------- A2/A3 ---
# Zoo models, so hailomz resolves their own calibration sets. If that
# resolution needs a dataset the box cannot fetch, the row reports the
# parse result instead and SAYS so -- a parse is not an allocation.
for spec in "A2|clip_resnet_50_image_encoder" "A3|segformer_b0_bn"; do
  IFS='|' read -r row model <<< "$spec"
  log="$OUT/$row.$model.log"
  echo "=== $row: hailomz compile $model --hw-arch hailo10h ==="
  if timeout 21600 hailomz compile "$model" --hw-arch hailo10h > "$log" 2>&1; then
    mb=$(find /local/workspace -maxdepth 2 -name "$model*.hef" -newermt '-6 hours' -exec du -m {} \; | cut -f1 | head -1)
    find /local/workspace -maxdepth 2 -name "$model*.hef" -newermt '-6 hours' -exec cp {} "$OUT/" \; 2>/dev/null
    emit "$row" allocation "$model" hailo10h OK "hef_mb=${mb:-?}"
  else
    detail="$(grep -iE 'error|exception|allocat|resources|dataset' "$log" | head -1)"
    if timeout 3600 hailomz parse "$model" --hw-arch hailo10h >> "$log" 2>&1; then
      emit "$row" parse-only "$model" hailo10h PARSED "compile failed: $detail"
    else
      emit "$row" allocation "$model" hailo10h FAIL "$detail"
    fi
  fi
done

echo
echo "=== phase 2 complete ==="
column -t -s $'\t' "$REPORT" 2>/dev/null || cat "$REPORT"
