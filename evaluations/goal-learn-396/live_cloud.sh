#!/usr/bin/env bash
# evaluations/goal-learn-396/live_cloud.sh -- 3.96 criterion 7 (live.py's six
# missions) on one spot CPU instance, off the shared workstation (the user,
# 2026-10-24: heavy jobs go to AWS, on spot). Wraps tools/gpu/ec2.sh with its
# own name, prefix, type and spot flag, as cloud.sh does for B2-B4.
#
#   AWS_PROFILE=mac-admin evaluations/goal-learn-396/live_cloud.sh start    push HEAD, launch, build the image, run
#   AWS_PROFILE=mac-admin evaluations/goal-learn-396/live_cloud.sh status   instance, cost so far, the runs so far
#   AWS_PROFILE=mac-admin evaluations/goal-learn-396/live_cloud.sh pull     results -> evaluations/goal-learn-396/live/<batch>/
#   AWS_PROFILE=mac-admin evaluations/goal-learn-396/live_cloud.sh down     terminate, delete SG + role, AND the S3 prefix
#
# A spot interruption (or the dead-man) terminates the instance and voids the
# batch: `pull` keeps what reached S3 in live/<batch>/, then `FORCE=1 ... down`
# discards the prefix (no DONE, so plain `down` keeps it), then `start` re-runs
# all six. A batch is only ever one instance's six runs.
set -euo pipefail
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
cd "$(git rev-parse --show-toplevel)"
export GPU_HOST_NAME=vision-picar-cpu-live-396
export GPU_PREFIX=live-396
export GPU_INSTANCE_TYPE="${CPU_TYPE:-c7i.2xlarge}"
export GPU_HOURLY="${CPU_HOURLY:-0.12}"     # the top of 2026-10-24's spot range (0.10-0.12, us-east-2)
export GPU_SPOT=1
export GPU_VOLUME_GB=60
export GPU_PURPOSE=live-396
export GPU_AZ="${GPU_AZ:-us-east-2c}"       # the cheapest zone that day
REGION="${AWS_REGION:-us-east-2}"
BUCKET="${GPU_BUCKET:-vision-picar-deploy-303351622021-us-east-2}"
S3="s3://$BUCKET/$GPU_PREFIX"
MAX_HOURS="${MAX_HOURS:-6}"   # the instance powers itself off (terminate) by then:
                              # the build plus six runs capped at 1,800 s each fits
EC2=tools/gpu/ec2.sh
HERE=evaluations/goal-learn-396
OUT="$HERE/live"

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
  # The instance gets HEAD, never the working tree (cloud.sh's rule).
  if [ -n "$(git status --porcelain -- . ":!$OUT")" ]; then
    echo "uncommitted or untracked files -- commit first (the instance gets HEAD):" >&2
    git status --short -- . ":!$OUT" >&2
    exit 1
  fi
  local tmp; tmp="$(mktemp -d)"
  git archive --format=tar.gz -o "$tmp/code.tar.gz" HEAD
  aws --region "$REGION" s3 cp --only-show-errors "$tmp/code.tar.gz" "$S3/code.tar.gz"
  aws --region "$REGION" s3 cp --only-show-errors "$HERE/live_remote.sh" "$S3/live_remote.sh"
  rm -rf "$tmp"
  echo "pushed $(git rev-parse --short HEAD)"
}

run_remote() {
  # Canonical's Ubuntu image has the SSM agent but no AWS CLI.
  # The dead-man is armed FIRST, before anything here can fail and leave the
  # instance billing with no timer (live_remote.sh arms its own as well).
  bash "$EC2" run "sudo shutdown -h +$((MAX_HOURS * 60)) '3.96 live dead-man (SSM)'; export PATH=\$PATH:/snap/bin; command -v aws >/dev/null || snap install aws-cli --classic >/dev/null; aws s3 cp --only-show-errors $S3/live_remote.sh /tmp/live_remote.sh && sudo -u ubuntu env PATH=\$PATH bash /tmp/live_remote.sh $S3 $MAX_HOURS"
}

case "${1:-}" in
  start)  if aws --region "$REGION" s3 ls "$S3/out/" >/dev/null 2>&1; then
            echo "$S3/out/ holds an earlier batch -- pull + down first" >&2; exit 1
          fi
          rm -f "$OUT/.pulled"; ami; push_code
          # The batch's name, so its pull lands in a directory of its own.
          printf '%s %s\n' "$(date -u +%Y%m%dT%H%M%SZ)" "$(git rev-parse --short HEAD)" \
            | aws --region "$REGION" s3 cp --only-show-errors - "$S3/BATCH"
          bash "$EC2" up; run_remote ;;
  status) bash "$EC2" status
          echo "== build"; aws --region "$REGION" s3 cp --only-show-errors "$S3/logs/build.log" - 2>/dev/null \
            | tail -n 2 || echo "(no build log yet)"
          echo "== runs"; aws --region "$REGION" s3 cp --only-show-errors "$S3/logs/drive.log" - 2>/dev/null \
            | tail -n 8 || echo "(no runs yet)" ;;
  done)   aws --region "$REGION" s3 ls "$S3/out/DONE" >/dev/null 2>&1 ;;
  pull)   batch="$(aws --region "$REGION" s3 cp --only-show-errors "$S3/BATCH" - | cut -d' ' -f1)"
          [ -n "$batch" ] || { echo "no batch at $S3" >&2; exit 1; }
          dest="$OUT/$batch"; mkdir -p "$dest"
          # Checked BEFORE the download, so nothing lands unseen in between.
          complete=0; bash "$SELF" done && complete=1
          aws --region "$REGION" s3 sync --only-show-errors "$S3/out/" "$dest/"
          aws --region "$REGION" s3 sync --only-show-errors "$S3/logs/" "$dest/logs/"
          # Which commit ran: the stamp and HEAD that `start` pushed.
          aws --region "$REGION" s3 cp --only-show-errors "$S3/BATCH" "$dest/BATCH"
          if [ $complete = 1 ]; then touch "$OUT/.pulled"; else echo "not finished yet -- pull again when done"; fi
          ls -la "$dest" ;;
  down)   bash "$EC2" down
          # The results live only in S3 until pulled: keep them unless pulled
          # (or FORCE=1, to throw them away).
          if [ -f "$OUT/.pulled" ] || [ "${FORCE:-0}" = 1 ]; then
            aws --region "$REGION" s3 rm --recursive --only-show-errors "$S3/" && echo "deleted $S3/"
          else
            echo "NOT deleting $S3/ -- run pull first (or FORCE=1 down to discard it)"
          fi ;;
  *) sed -n '2,14p' "$0"; exit 1 ;;
esac
