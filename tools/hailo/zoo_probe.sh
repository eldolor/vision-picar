#!/usr/bin/env bash
# P15 -- does the DFC 5.x line actually fail on real models, or did WE fail?
#
# P14 (2026-09-15) concluded "the 10H is blocked on its COMPILER": DFC 5.2.0
# and 5.4.0 parsed a trivial 4-node ONNX on `hailo10h` and failed every real
# model. That conclusion rested on three graphs we had all modified
# ourselves, run in an environment we assembled ourselves, and on a diagram
# in the v5.1.0 Overview labelled "Future support (for Hailo-10H)".
#
# Three things found on 2026-09-15 undercut all of that:
#
#   * Hailo's download page tags 5.1.0, 5.2.0 and 5.3.0 each with
#     Hailo-10H / 15H / 15L. The "future support" diagram is stale.
#   * The 5.4.0 Model Zoo declares `supported_hw_arch: [hailo15h, hailo15l,
#     hailo10h]` on yolov11s -- and on 224 of its 233 networks.
#   * Hailo's OWN yolo11s.onnx is **opset 19 / IR 10 / pytorch 2.4.1**.
#     Ours is **opset 13 / IR 7 / pytorch 2.14.0**, and we handed the parser
#     a pre-cut graph where the zoo hands it the full model plus an
#     end-node spec. Either difference is a credible cause of P14's
#     `is_null_split: channels is not in list`.
#
# So this probe removes US from the experiment, one variable at a time, and
# it is deliberately built around `parse` rather than `compile`: P14 died at
# parse, and parse needs no calibration set, no GPU and no optimisation
# pass. The whole matrix is minutes, which is what makes it worth running
# before spending anything on a part.
#
# The three rows that decide it:
#
#   A. Hailo's yolo11s.onnx, Hailo's cut, Hailo's container, hailo10h
#        -> OK  = P14's headline is refuted and the 10H is back on the table
#        -> FAIL = a reproducer made entirely of THEIR artifacts, which is a
#                  far stronger ticket than anything P14 could have filed
#   B. OUR opset-13 export, through that same zoo flow
#        -> FAIL against A's OK pins the cause on the export, not the part
#   C. hailo8l, which 5.x rejects outright -- the control that proves the
#      matrix can report a failure at all
#
# Usage, from the repo root:
#   tools/hailo/ec2.sh up
#   tools/hailo/zoo_probe.sh push      # our ONNX + this script -> S3
#   tools/hailo/zoo_probe.sh setup     # docker + the 8GB suite image
#   tools/hailo/zoo_probe.sh run       # the matrix
#   tools/hailo/zoo_probe.sh pull      # report + logs -> evaluations/hailo/
#   tools/hailo/ec2.sh down            # <- the important one
set -euo pipefail

REGION="${AWS_REGION:-us-east-2}"
BUCKET="${HAILO_BUCKET:-vision-picar-deploy-303351622021-us-east-2}"
PREFIX="${HAILO_PREFIX:-hailo}"
REMOTE_DIR="/opt/hailo"
PROBE_DIR="$REMOTE_DIR/zoo_probe"
SUITE_ZIP="hailo_ai_sw_suite_2026-08_docker.zip"
IMAGE="hailo_ai_sw_suite_2026-08:1"
LOCAL_OUT="${ZOO_PROBE_OUT:-evaluations/hailo/zoo-probe}"

S3="s3://$BUCKET/$PREFIX"
EC2="$(dirname "$0")/ec2.sh"

say() { printf '\033[1m[probe]\033[0m %s\n' "$*"; }
die() { printf '\033[31m[probe] %s\033[0m\n' "$*" >&2; exit 1; }

# ec2.sh already owns the instance, SSM plumbing and the cost readout. This
# script never launches or terminates anything -- `down` stays in one place.
remote() { "$EC2" run "$@"; }

# ------------------------------------------------------------------ push ---
cmd_push() {
  [ -f build/yolo11s/yolo11s.onnx ] || die "build/yolo11s/yolo11s.onnx missing"
  say "uploading our opset-13 export (row B) and this script"
  aws --region "$REGION" s3 cp build/yolo11s/yolo11s.onnx \
      "$S3/zoo_probe/ours_yolo11s_op13.onnx" --only-show-errors
  aws --region "$REGION" s3 cp "$0" "$S3/zoo_probe/zoo_probe.sh" --only-show-errors
  say "-> $S3/zoo_probe/"
}

# ----------------------------------------------------------------- setup ---
# Docker, then the suite image. The vendor's own
# hailo_ai_sw_suite_docker_run.sh is interactive (`docker run -ti`), which
# SSM cannot drive, so this loads the image and runs its own commands
# against it. That is the only place this deviates from the vendor path,
# and it does not touch the toolchain inside the image.
cmd_setup() {
  say "installing docker and loading the suite image (8GB down, ~20GB unpacked)"
  remote "set -e
    export DEBIAN_FRONTEND=noninteractive
    command -v docker >/dev/null || {
      sudo apt-get update -qq
      sudo apt-get install -y -qq docker.io unzip >/dev/null
      sudo systemctl enable --now docker
    }
    sudo mkdir -p $PROBE_DIR && sudo chown -R \$(id -u):\$(id -g) $REMOTE_DIR
    cd $PROBE_DIR
    if ! sudo docker images -q $IMAGE | grep -q .; then
      [ -f $SUITE_ZIP ] || aws s3 cp $S3/suite/$SUITE_ZIP . --only-show-errors
      [ -f hailo_ai_sw_suite_2026-08.tar.gz ] || unzip -o -q $SUITE_ZIP
      sudo docker load -i hailo_ai_sw_suite_2026-08.tar.gz
    fi
    aws s3 cp $S3/zoo_probe/ours_yolo11s_op13.onnx . --only-show-errors
    sudo docker images | head -5
    df -h / | tail -1"
  say "image loaded"
}

# ------------------------------------------------------------------- run ---
# Every cell is `hailomz parse`, which is where P14 died. `--ckpt` points the
# zoo's own config at a different ONNX, which is what makes row B an
# apples-to-apples comparison: same config, same end nodes, same container,
# only the export changes.
cmd_run() {
  say "uploading the matrix and running it -- parse only, no calibration, no GPU"
  aws --region "$REGION" s3 cp tools/hailo/zoo_matrix.sh "$S3/zoo_probe/zoo_matrix.sh" --only-show-errors
  remote "set -e
    cd $PROBE_DIR
    aws s3 cp $S3/zoo_probe/zoo_matrix.sh . --only-show-errors
    chmod +x zoo_matrix.sh
    sudo rm -rf out
    # The container runs as uid 10642(hailo), not root, and the bind mount
    # is root-owned -- so the matrix cannot create its own output directory.
    # a+rwX rather than a chown to a hardcoded uid, which would rot the
    # first time the vendor renumbers its image user.
    sudo chmod -R a+rwX $PROBE_DIR
    nohup setsid sudo docker run --rm \
      -v $PROBE_DIR:/local/shared_with_docker \
      $IMAGE bash /local/shared_with_docker/zoo_matrix.sh \
      > $PROBE_DIR/probe.log 2>&1 &
    echo started; sleep 15; tail -5 $PROBE_DIR/probe.log || true"
  say "follow with: tools/hailo/zoo_probe.sh tail"
}

cmd_tail() { remote "tail -40 $PROBE_DIR/probe.log; echo '--- report ---'; cat $PROBE_DIR/out/report.tsv 2>/dev/null || echo '(not yet)'"; }

# ------------------------------------------------------------------ pull ---
cmd_pull() {
  say "collecting the report and logs"
  remote "aws s3 sync $PROBE_DIR/out/ $S3/zoo_probe/out/ --only-show-errors && echo synced"
  mkdir -p "$LOCAL_OUT"
  aws --region "$REGION" s3 sync "$S3/zoo_probe/out/" "$LOCAL_OUT/" --only-show-errors
  say "-> $LOCAL_OUT"
  [ -f "$LOCAL_OUT/report.tsv" ] && column -t -s $'\t' "$LOCAL_OUT/report.tsv"
}

case "${1:-}" in
  push)  cmd_push ;;
  setup) cmd_setup ;;
  run)   cmd_run ;;
  tail)  cmd_tail ;;
  pull)  cmd_pull ;;
  *) die "usage: $0 {push|setup|run|tail|pull}   (up/down stay in ec2.sh)" ;;
esac
