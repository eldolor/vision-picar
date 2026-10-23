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
export GPU_AMI="${GPU_AMI:-$(aws --region "$REGION" ssm get-parameter --name \
  /aws/service/canonical/ubuntu/server/24.04/stable/current/amd64/hvm/ebs-gp3/ami-id \
  --query Parameter.Value --output text)}"
EC2=tools/gpu/ec2.sh
HERE=evaluations/goal-learn-396

push_code() {
  local tmp; tmp="$(mktemp -d)"
  git archive --format=tar.gz -o "$tmp/code.tar.gz" HEAD
  aws --region "$REGION" s3 cp --only-show-errors "$tmp/code.tar.gz" "$S3/code.tar.gz"
  aws --region "$REGION" s3 cp --only-show-errors "$HERE/remote.sh" "$S3/remote.sh"
  rm -rf "$tmp"
}

run_remote() {
  # Canonical's Ubuntu image has the SSM agent but no AWS CLI.
  bash "$EC2" run "export PATH=\$PATH:/snap/bin; command -v aws >/dev/null || snap install aws-cli --classic >/dev/null; aws s3 cp --only-show-errors $S3/remote.sh /tmp/remote.sh && sudo -u ubuntu env PATH=\$PATH bash /tmp/remote.sh $S3"
}

case "${1:-}" in
  start)  push_code; bash "$EC2" up; run_remote ;;
  resume) bash "$EC2" up; run_remote ;;
  status) bash "$EC2" status
          for a in F-small F-large MAP-large PONI-large rule; do
            echo "== $a"; aws --region "$REGION" s3 cp --only-show-errors "$S3/logs/$a.log" - 2>/dev/null \
              | grep -v Blocked | tail -n 2 || echo "(no log yet)"
          done ;;
  done)   for f in F-small/best.json F-large/best.json MAP-large/best.json PONI-large/best.json rule/rule-select.json; do
            aws --region "$REGION" s3 ls "$S3/ckpt/$f" >/dev/null 2>&1 || exit 1
          done ;;
  pull)   mkdir -p "$HERE/cloud"
          for a in F-small F-large MAP-large PONI-large; do
            aws --region "$REGION" s3 cp --only-show-errors "$S3/ckpt/$a/best.json" "$HERE/cloud/$a.json" || true
            aws --region "$REGION" s3 cp --only-show-errors "$S3/ckpt/$a/clone.pt" "$HERE/cloud/$a-clone.pt" || true
          done
          aws --region "$REGION" s3 cp --only-show-errors "$S3/ckpt/rule/rule-select.json" "$HERE/cloud/rule-select.json" || true
          aws --region "$REGION" s3 sync --only-show-errors "$S3/logs/" "$HERE/cloud/logs/"
          ls -la "$HERE/cloud" ;;
  down)   bash "$EC2" down
          aws --region "$REGION" s3 rm --recursive --only-show-errors "$S3/" && echo "deleted $S3/"
          aws --region "$REGION" ec2 describe-instances --filters "Name=tag:Name,Values=$GPU_HOST_NAME" \
            "Name=instance-state-name,Values=pending,running,stopping,stopped" \
            --query 'Reservations[].Instances[].InstanceId' --output text ;;
  *) sed -n '2,13p' "$0"; exit 1 ;;
esac
