#!/usr/bin/env bash
# evaluations/tier-fusion-391/cloud.sh -- 3.91.C criterion 8's sweeps on one
# spot CPU instance (the user, 2026-10-24: heavy jobs leave the shared
# workstation). The pattern is 3.96's cloud.sh: code from HEAD via S3, a
# dead-man switch, `down` deletes the instance, its role and the S3 prefix.
#
#   evaluations/tier-fusion-391/cloud.sh start    push HEAD, launch (spot), start the 7 arms
#   evaluations/tier-fusion-391/cloud.sh status   instance, cost so far, the logs' tails
#   evaluations/tier-fusion-391/cloud.sh done     0 when every arm has written its result
#   evaluations/tier-fusion-391/cloud.sh pull     results -> evaluations/tier-fusion-391/C8/
#   evaluations/tier-fusion-391/cloud.sh down     terminate, delete SG + role, AND the S3 prefix
set -euo pipefail
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
cd "$(git rev-parse --show-toplevel)"
export GPU_HOST_NAME=vision-picar-cpu-391
export GPU_PREFIX=sweep-391
export GPU_INSTANCE_TYPE="${CPU_TYPE:-c7i.2xlarge}"
export GPU_HOURLY="${CPU_HOURLY:-0.15}"      # an estimate of the spot price (us-east-2)
export GPU_SPOT=1
export GPU_VOLUME_GB=30
export GPU_PURPOSE=tier-fusion-391
REGION="${AWS_REGION:-us-east-2}"
BUCKET="${GPU_BUCKET:-vision-picar-deploy-303351622021-us-east-2}"
S3="s3://$BUCKET/$GPU_PREFIX"
MAX_HOURS="${MAX_HOURS:-2}"   # the instance powers itself off (terminate) by then
case "$MAX_HOURS" in [1-9]|[1-9][0-9]) ;; *) echo "MAX_HOURS must be a whole number of hours, 1-99" >&2; exit 1 ;; esac
EC2=tools/gpu/ec2.sh
HERE=evaluations/tier-fusion-391
ARMS="G-0.9 G-0.0 O1-blind-1 O1-blind-6 O2-decoy-1 O2-decoy-6 sighted-6"

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
  # The instance gets HEAD, never the working tree: refuse uncommitted or
  # untracked files (committed code may import one). C8/ holds results only.
  if [ -n "$(git status --porcelain -- . ":!$HERE/C8")" ]; then
    echo "uncommitted or untracked files -- commit first (the instance gets HEAD):" >&2
    git status --short -- . ":!$HERE/C8" >&2
    exit 1
  fi
  local tmp; tmp="$(mktemp -d)"
  git archive --format=tar.gz -o "$tmp/code.tar.gz" HEAD
  aws --region "$REGION" s3 cp --only-show-errors "$tmp/code.tar.gz" "$S3/code.tar.gz"
  aws --region "$REGION" s3 cp --only-show-errors "$HERE/remote.sh" "$S3/remote.sh"
  git rev-parse HEAD | aws --region "$REGION" s3 cp --only-show-errors - "$S3/HEAD"
  rm -rf "$tmp"
}

run_remote() {
  # Canonical's Ubuntu image has the SSM agent but no AWS CLI.
  # The dead-man switch first, as root (SSM): a failure in anything after it
  # (snap, the copy) must not leave the instance billing (round 5).
  bash "$EC2" run "shutdown -h +$((MAX_HOURS * 60)) '3.91 dead-man switch'; export PATH=\$PATH:/snap/bin; command -v aws >/dev/null || snap install aws-cli --classic >/dev/null; aws s3 cp --only-show-errors $S3/remote.sh /tmp/remote.sh && sudo -u ubuntu env PATH=\$PATH bash /tmp/remote.sh $S3 $MAX_HOURS"
}

case "${1:-}" in
  start)  # Any file under the prefix is a run (out/ fills only as arms finish;
          # a second start would relabel the first run's results).
          if [ -n "$(aws --region "$REGION" s3 ls "$S3/" 2>/dev/null)" ]; then
            echo "$S3/ holds an earlier or running run -- pull + down first" >&2; exit 1
          fi
          ami; push_code; rm -f "$HERE/C8/.pulled"; bash "$EC2" up; run_remote ;;
  status) bash "$EC2" status
          for a in $ARMS; do
            echo "== $a"; aws --region "$REGION" s3 cp --only-show-errors "$S3/logs/$a.log" - 2>/dev/null \
              | tail -n 1 | cut -c1-200 || echo "(no log yet)"
          done ;;
  done)   for a in $ARMS; do
            aws --region "$REGION" s3 ls "$S3/out/$a.json" >/dev/null 2>&1 || exit 1
          done ;;
  pull)   mkdir -p "$HERE/C8"
          complete=0; bash "$SELF" done && complete=1
          # This run's files only: an arm that failed must not leave an
          # earlier run's result beside this run's HEAD -- cleared only once
          # S3 is known to hold this run's results (round 6: a pull after
          # `down`, or a failed listing, emptied C8/).
          if [ -z "$(aws --region "$REGION" s3 ls "$S3/out/")" ]; then
            echo "no results under $S3/out/ -- nothing pulled" >&2; exit 1
          fi
          # Into a fresh folder first: a sync that fails leaves C8/ as it was
          # (review round 7).
          tmp="$(mktemp -d)"
          aws --region "$REGION" s3 sync --only-show-errors "$S3/out/" "$tmp/"
          aws --region "$REGION" s3 sync --only-show-errors "$S3/logs/" "$tmp/"
          aws --region "$REGION" s3 cp --only-show-errors "$S3/HEAD" "$tmp/HEAD" || true
          rm -f "$HERE/C8/"*.json "$HERE/C8/"*.log "$HERE/C8/HEAD"
          mv "$tmp/"* "$HERE/C8/" && rmdir "$tmp"
          if [ $complete = 1 ]; then touch "$HERE/C8/.pulled"; else echo "not all results in yet -- pull again when done"; fi
          ls -la "$HERE/C8" ;;
  down)   bash "$EC2" down
          if [ -f "$HERE/C8/.pulled" ] || [ "${FORCE:-0}" = 1 ]; then
            aws --region "$REGION" s3 rm --recursive --only-show-errors "$S3/" && echo "deleted $S3/"
          else
            echo "NOT deleting $S3/ -- run pull first (or FORCE=1 down to discard it)"
          fi
          aws --region "$REGION" ec2 describe-instances --filters "Name=tag:Name,Values=$GPU_HOST_NAME" \
            "Name=instance-state-name,Values=pending,running,stopping,stopped" \
            --query 'Reservations[].Instances[].InstanceId' --output text ;;
  *) sed -n '2,12p' "$0"; exit 1 ;;
esac
