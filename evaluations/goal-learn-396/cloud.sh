#!/usr/bin/env bash
# evaluations/goal-learn-396/cloud.sh -- 3.96 B2-B4 on one spot CPU instance
# (the user, 2026-10-23: spot, checkpoints, and tear everything down after).
# Wraps tools/gpu/ec2.sh with its own name, prefix, type and spot flag.
#
#   evaluations/goal-learn-396/cloud.sh start    push code, launch (spot), start the runs
#   evaluations/goal-learn-396/cloud.sh resume   after an interruption: launch again, resume
#   evaluations/goal-learn-396/cloud.sh status   instance, cost so far, the logs' tails
#   evaluations/goal-learn-396/cloud.sh done     0 when every arm and the rule have finished
#   evaluations/goal-learn-396/cloud.sh pull     results -> evaluations/goal-learn-396/cloud/
#   evaluations/goal-learn-396/cloud.sh down     terminate, delete SG + role, AND the S3 prefix
set -euo pipefail
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
cd "$(git rev-parse --show-toplevel)"
export GPU_HOST_NAME=vision-picar-cpu-396
export GPU_PREFIX=train-396
export GPU_INSTANCE_TYPE="${CPU_TYPE:-c7i.16xlarge}"
export GPU_HOURLY="${CPU_HOURLY:-0.79}"      # the spot price seen 2026-10-23 (us-east-2)
export GPU_SPOT=1
export GPU_VOLUME_GB=60
export GPU_PURPOSE=goal-learn-396
REGION="${AWS_REGION:-us-east-2}"
BUCKET="${GPU_BUCKET:-vision-picar-deploy-303351622021-us-east-2}"
S3="s3://$BUCKET/$GPU_PREFIX"
MAX_HOURS="${MAX_HOURS:-6}"   # the instance powers itself off (terminate) by then
EC2=tools/gpu/ec2.sh
HERE=evaluations/goal-learn-396

ami() {
  # Ubuntu 24.04 (python 3.12). A failed lookup must stop here: ec2.sh would
  # otherwise fall back to its GPU image.
  if [ -z "${GPU_AMI:-}" ]; then
    GPU_AMI="$(aws --region "$REGION" ssm get-parameter --name \
      /aws/service/canonical/ubuntu/server/24.04/stable/current/amd64/hvm/ebs-gp3/ami-id \
      --query Parameter.Value --output text)" || { echo "AMI lookup failed" >&2; exit 1; }
  fi
  [ -n "$GPU_AMI" ] || { echo "AMI lookup returned nothing" >&2; exit 1; }
  export GPU_AMI
}

push_code() {
  # The instance gets HEAD, never the working tree: refuse uncommitted changes
  # to tracked files, or the run would train on something else.
  # Untracked files count too: committed code may import one (3.76 batch I).
  # (cloud/ holds pulled results, never code.)
  if [ -n "$(git status --porcelain -- . ":!$HERE/cloud")" ]; then
    echo "uncommitted or untracked files -- commit first (the instance gets HEAD):" >&2
    git status --short -- . ":!$HERE/cloud" >&2
    exit 1
  fi
  local tmp; tmp="$(mktemp -d)"
  git archive --format=tar.gz -o "$tmp/code.tar.gz" HEAD
  aws --region "$REGION" s3 cp --only-show-errors "$tmp/code.tar.gz" "$S3/code.tar.gz"
  aws --region "$REGION" s3 cp --only-show-errors "$HERE/remote.sh" "$S3/remote.sh"
  rm -rf "$tmp"
}

run_remote() {
  # Canonical's Ubuntu image has the SSM agent but no AWS CLI.
  bash "$EC2" run "export PATH=\$PATH:/snap/bin; command -v aws >/dev/null || snap install aws-cli --classic >/dev/null; aws s3 cp --only-show-errors $S3/remote.sh /tmp/remote.sh && sudo -u ubuntu env PATH=\$PATH bash /tmp/remote.sh $S3 $MAX_HOURS"
}

case "${1:-}" in
  # A new launch makes any earlier pull stale, so `down` must see a new one.
  start)  # A fresh run must not resume an earlier run's checkpoints.
          if aws --region "$REGION" s3 ls "$S3/ckpt/" >/dev/null 2>&1; then
            echo "$S3/ckpt/ holds an earlier run -- pull + down first (or resume it)" >&2; exit 1
          fi
          ami; push_code; rm -f "$HERE/cloud/.pulled"; bash "$EC2" up; run_remote ;;
  resume) ami; rm -f "$HERE/cloud/.pulled"; bash "$EC2" up; run_remote ;;
  status) bash "$EC2" status
          for a in oracle F-small F-large MAP-large PONI-large rule; do
            echo "== $a"; aws --region "$REGION" s3 cp --only-show-errors "$S3/logs/$a.log" - 2>/dev/null \
              | grep -v Blocked | tail -n 2 || echo "(no log yet)"
          done ;;
  done)   for f in F-small/best.json F-large/best.json MAP-large/best.json PONI-large/best.json rule/rule-select.json; do
            aws --region "$REGION" s3 ls "$S3/ckpt/$f" >/dev/null 2>&1 || exit 1
          done ;;
  pull)   mkdir -p "$HERE/cloud"
          # Checked BEFORE the download, so nothing can land in S3 unseen
          # between the copy and the mark.
          complete=0; bash "$SELF" done && complete=1
          # Everything under ckpt/ except the big oracle shards (regenerable).
          aws --region "$REGION" s3 sync --only-show-errors --exclude "oracle-*" \
            "$S3/ckpt/" "$HERE/cloud/ckpt/"
          aws --region "$REGION" s3 sync --only-show-errors "$S3/logs/" "$HERE/cloud/logs/"
          # Marked pulled only when every result is in (`down` keys off it).
          if [ $complete = 1 ]; then touch "$HERE/cloud/.pulled"; else echo "not all results in yet -- pull again when done"; fi
          ls -la "$HERE/cloud" "$HERE/cloud/ckpt" ;;
  down)   bash "$EC2" down
          # The results live only in S3 until pulled: keep them unless pulled
          # (or FORCE=1, to throw them away).
          if [ -f "$HERE/cloud/.pulled" ] || [ "${FORCE:-0}" = 1 ]; then
            aws --region "$REGION" s3 rm --recursive --only-show-errors "$S3/" && echo "deleted $S3/"
          else
            echo "NOT deleting $S3/ -- run pull first (or FORCE=1 down to discard it)"
          fi
          aws --region "$REGION" ec2 describe-instances --filters "Name=tag:Name,Values=$GPU_HOST_NAME" \
            "Name=instance-state-name,Values=pending,running,stopping,stopped" \
            --query 'Reservations[].Instances[].InstanceId' --output text ;;
  *) sed -n '2,13p' "$0"; exit 1 ;;
esac
