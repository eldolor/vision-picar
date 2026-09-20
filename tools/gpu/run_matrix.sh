#!/usr/bin/env bash
# The sweep the A10G was rented for: every YOLOE variant, WITH and WITHOUT
# the floor mask, on all 1234 labelled frames.
#
# Both axes exist because of specific gaps, not for completeness:
#
#   * **The mask.** P20 dropped it on OWLv2 evidence -- median 4 crops,
#     silent on 25% of frames, worth one true positive. YOLOE proposes a
#     median of ONE and is silent on ~40%, which is the exact condition a
#     region proposer exists for. No YOLOE row had ever been run with it,
#     so "the mask is droppable" may be an OWLv2 fact rather than a tier one.
#   * **Every variant.** P22 ranked the family on 365 frames and P23 found
#     the ranking INVERTS at 1234 -- the winner became the smallest
#     checkpoint of its generation. Only four variants have full-corpus rows.
#
# OWLv2 is included as the control: it is the model YOLOE has to beat, and
# its full-corpus number is the one still missing.
set -euo pipefail
REMOTE=/opt/eval
EC2=tools/hailo/ec2.sh

VARIANTS="yoloe-11s-seg yoloe-11m-seg yoloe-11l-seg \
          yoloe-26n-seg yoloe-26s-seg yoloe-26m-seg yoloe-26l-seg yoloe-26x-seg \
          yoloe-v8s-seg yoloe-v8m-seg yoloe-v8l-seg"

bash "$EC2" run "set -e
cd $REMOTE && . /opt/pytorch/bin/activate
mkdir -p out
export PYTHONPATH=$REMOTE
COMMON='--recordings $REMOTE/recordings --clip RN50 --crop-path low_confidence \
        --affinity-k 8 --max-crops 16 --device cuda --dtype fp32 \
        --metric probability --gate 0.8 --quiet'
for ck in $VARIANTS; do
  for prop in none floor; do
    out=out/\${ck}_\${prop}.json
    [ -f \"\\\$out\" ] && continue
    echo \"=== \$ck \$prop \$(date +%H:%M:%S) ===\"
    python -m control.perception_eval score \\\$COMMON --proposer \$prop \\
      --detector crops:yoloe:\$ck.pt --save \\\$out 2>&1 | grep -E 'TOTAL|unavailable|per frame' || true
  done
done
for prop in none floor; do
  echo \"=== owlv2 \$prop \$(date +%H:%M:%S) ===\"
  python -m control.perception_eval score \\\$COMMON --proposer \$prop \\
    --detector crops:owlv2 --save out/owlv2_\${prop}.json 2>&1 \\
    | grep -E 'TOTAL|unavailable|per frame' || true
done
echo MATRIX-DONE"
