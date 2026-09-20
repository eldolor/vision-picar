#!/usr/bin/env bash
# A rented A10G for corpus scoring -- launch, run the matrix, tear down.
#
# Why this exists. `control/perception_eval.py` defaults to CPU fp32 because
# every published record was scored that way and stays reproducible. That is
# right for a record and wrong for a sweep: OWLv2 is 2.7s a frame here and
# 111ms on an A10G (P7), so a 1234-frame corpus is 55 minutes locally and
# two on the box. P23 showed the corpus SIZE deciding conclusions -- the
# detector ranking inverted between 365 and 1234 frames -- so the ability to
# re-run everything at full corpus cheaply is the point.
#
# **Accuracy is not at risk and that is measured, not assumed**: P7's control
# had CUDA fp32 reproduce the committed CPU record exactly, same true
# positives, same false positives, gates to three decimals.
#
# It delegates the host lifecycle to tools/hailo/ec2.sh -- SSM instead of
# SSH, a security group that opens nothing, an instance profile scoped to
# one S3 prefix, and a `down` that deletes the role and the group as well as
# the instance. That script is hardened and this one should not grow a
# second copy of it.
#
# Usage:
#   tools/gpu/eval_host.sh up       launch (idempotent)
#   tools/gpu/eval_host.sh setup    deps + the corpus from S3
#   tools/gpu/eval_host.sh matrix   the sweep (streams to a local log)
#   tools/gpu/eval_host.sh pull     records -> evaluations/gpu-yoloe/
#   tools/gpu/eval_host.sh status   state, uptime, running cost
#   tools/gpu/eval_host.sh down     TERMINATE and delete role + SG + policy
set -euo pipefail

REGION="${AWS_REGION:-us-east-2}"
export HAILO_HOST_NAME="${GPU_HOST_NAME:-vision-picar-gpu-eval}"
export HAILO_INSTANCE_TYPE="${GPU_INSTANCE_TYPE:-g5.2xlarge}"
export HAILO_VOLUME_GB="${GPU_VOLUME_GB:-120}"
export HAILO_BUCKET="${GPU_BUCKET:-vision-picar-deploy-303351622021-us-east-2}"
export HAILO_PREFIX="${GPU_PREFIX:-gpu-eval}"
# us-east-2 on-demand g5.2xlarge. Cost readout only -- change with TYPE or
# the number printed is a lie.
export HAILO_HOURLY="${GPU_HOURLY:-1.212}"
# The Deep Learning AMI: CUDA, the driver and a torch already present.
# Installing the driver on bare Ubuntu is 20 minutes of the rental.
export HAILO_AMI="${GPU_AMI:-$(aws --region "$REGION" ec2 describe-images \
  --owners amazon \
  --filters 'Name=name,Values=Deep Learning OSS Nvidia Driver AMI GPU PyTorch 2.*(Ubuntu 22.04)*' \
            Name=state,Values=available \
  --query 'sort_by(Images,&CreationDate)[-1].ImageId' --output text)}"

CORPUS_BUCKET="${CORPUS_BUCKET:-vision-picar-recordings-303351622021-us-east-2}"
EC2="tools/hailo/ec2.sh"
REMOTE=/opt/eval
OUT_LOCAL="${GPU_OUT:-evaluations/gpu-yoloe}"

say() { printf '\033[1m[gpu]\033[0m %s\n' "$*"; }

# The corpus lives in a DIFFERENT bucket from ec2.sh's build prefix, so the
# host needs one more read-only grant. It is deleted again in `down`:
# ec2.sh's own delete_role only knows about its policy, and a leftover
# inline policy makes the role undeletable -- which is exactly the kind of
# quiet residue `down` exists to prevent.
ensure_corpus_read() {
  aws iam put-role-policy --role-name "$HAILO_HOST_NAME" \
    --policy-name corpus-read --policy-document "$(cat <<JSON
{"Version":"2012-10-17","Statement":[
 {"Effect":"Allow","Action":["s3:GetObject"],
  "Resource":"arn:aws:s3:::$CORPUS_BUCKET/recordings/*"},
 {"Effect":"Allow","Action":["s3:ListBucket"],
  "Resource":"arn:aws:s3:::$CORPUS_BUCKET",
  "Condition":{"StringLike":{"s3:prefix":["recordings/*"]}}}]}
JSON
)"
}

case "${1:-}" in
  up)
    bash "$EC2" up
    ensure_corpus_read
    # A dead-man switch. If this session dies, the box still goes away:
    # the user asked for teardown and an unattended GPU is $29 a day.
    bash "$EC2" run "sudo shutdown -h +240" >/dev/null 2>&1 || true
    aws --region "$REGION" ec2 modify-instance-attribute \
      --instance-id "$(aws --region "$REGION" ec2 describe-instances \
        --filters "Name=tag:Name,Values=$HAILO_HOST_NAME" \
                  Name=instance-state-name,Values=running \
        --query 'Reservations[0].Instances[0].InstanceId' --output text)" \
      --instance-initiated-shutdown-behavior terminate
    say "dead-man switch armed: self-terminates in 4h if nothing stops it"
    ;;
  setup)
    bash "$EC2" run "set -e
      sudo mkdir -p $REMOTE && sudo chown ubuntu:ubuntu $REMOTE
      cd $REMOTE
      # The DLAMI's own venv, NOT a fresh one: it carries the matched
      # torch/CUDA/driver build, and a venv beside it reinstalls torch from
      # PyPI against a driver it was not built for. System python3 has no
      # torch at all, which is what the first attempt found.
      . /opt/pytorch/bin/activate
      pip -q install ultralytics open_clip_torch transformers pillow numpy
      aws s3 sync s3://$CORPUS_BUCKET/recordings/ $REMOTE/recordings/ \
        --exclude '*' --include '*.jpg' --include 'labels.json' --only-show-errors
      echo READY; ls $REMOTE/recordings | wc -l" 2>&1 | tail -20
    ;;
  push)
    # The repo's own scorer, so the box runs the same code as the laptop.
    tar czf /tmp/eval-src.tgz brain control
    aws --region "$REGION" s3 cp /tmp/eval-src.tgz \
      "s3://$HAILO_BUCKET/$HAILO_PREFIX/eval-src.tgz" --only-show-errors
    bash "$EC2" run "cd $REMOTE && aws s3 cp s3://$HAILO_BUCKET/$HAILO_PREFIX/eval-src.tgz . \
      && tar xzf eval-src.tgz && echo PUSHED && ls"
    ;;
  matrix)   bash tools/gpu/run_matrix.sh ;;
  pull)
    mkdir -p "$OUT_LOCAL"
    bash "$EC2" run "cd $REMOTE && aws s3 sync out/ s3://$HAILO_BUCKET/$HAILO_PREFIX/out/ --only-show-errors && echo SYNCED"
    aws --region "$REGION" s3 sync "s3://$HAILO_BUCKET/$HAILO_PREFIX/out/" "$OUT_LOCAL/" --only-show-errors
    say "records -> $OUT_LOCAL"; ls "$OUT_LOCAL"
    ;;
  status)   bash "$EC2" status ;;
  down)
    aws iam delete-role-policy --role-name "$HAILO_HOST_NAME" \
      --policy-name corpus-read 2>/dev/null || true
    bash "$EC2" down
    say "verifying nothing is left"
    aws --region "$REGION" ec2 describe-instances \
      --filters "Name=tag:Name,Values=$HAILO_HOST_NAME" \
      --query 'Reservations[].Instances[].[InstanceId,State.Name]' --output text
    aws iam get-role --role-name "$HAILO_HOST_NAME" >/dev/null 2>&1 \
      && say "WARNING: role $HAILO_HOST_NAME still exists" || say "role gone"
    ;;
  *) sed -n '/^# Usage:/,/^set -e/p' "$0"; exit 1 ;;
esac
