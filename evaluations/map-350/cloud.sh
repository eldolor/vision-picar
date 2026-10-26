#!/usr/bin/env bash
# evaluations/map-350/cloud.sh -- 3.50.D4-a, D4-b and D4-d (amendment 4's
# held-out test) on one spot CPU instance (the user, 2026-10-24: heavy live
# runs go to AWS). Wraps tools/gpu/ec2.sh with its own name, prefix, type and
# spot flag, as evaluations/goal-learn-396/cloud.sh does.
#
#   evaluations/map-350/cloud.sh start    push HEAD, launch (spot), build the image, start the runs
#   evaluations/map-350/cloud.sh resume   after an interruption: launch again, run what is not done
#   evaluations/map-350/cloud.sh status   instance, cost so far, the logs' tails
#   evaluations/map-350/cloud.sh done     0 when every run has its result
#   evaluations/map-350/cloud.sh pull     results -> evaluations/map-350/D4/ (+ logs to D4/cloud/)
#   evaluations/map-350/cloud.sh down     terminate, delete SG + role, AND the S3 prefix
#
# Only results and text logs go to S3: the run directories' maps (a SIM
# house's floor plan) stay on the instance and die with it.
set -euo pipefail
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
cd "$(git rev-parse --show-toplevel)"
export GPU_HOST_NAME=vision-picar-cpu-350
export GPU_PREFIX=d4-350
export GPU_INSTANCE_TYPE="${CPU_TYPE:-c7i.4xlarge}"
export GPU_HOURLY="${CPU_HOURLY:-0.25}"      # an estimate (3.96's 16xlarge spot at 0.79/4, rounded up)
export GPU_SPOT=1
export GPU_VOLUME_GB=60
export GPU_PURPOSE=map-350-d4
REGION="${AWS_REGION:-us-east-2}"
BUCKET="${GPU_BUCKET:-vision-picar-deploy-303351622021-us-east-2}"
S3="s3://$BUCKET/$GPU_PREFIX"
MAX_HOURS="${MAX_HOURS:-6}"   # the instance powers itself off (terminate) by then
EC2=tools/gpu/ec2.sh
HERE=evaluations/map-350
RUNS="a-scaled_house a-home_first_floor b-scaled_house b-home_first_floor d-scaled_house d-home_first_floor"

ami() {
  # Ubuntu 24.04. A failed lookup must stop here: ec2.sh would otherwise fall
  # back to its GPU image.
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
  # untracked files (committed code may import one; 3.76 batch I).
  # (D4/cloud/ holds pulled logs, never code.)
  if [ -n "$(git status --porcelain -- . ":!$HERE/D4/cloud")" ]; then
    echo "uncommitted or untracked files -- commit first (the instance gets HEAD):" >&2
    git status --short -- . ":!$HERE/D4/cloud" >&2
    exit 1
  fi
  local tmp; tmp="$(mktemp -d)"
  git archive --format=tar.gz -o "$tmp/code.tar.gz" HEAD
  git rev-parse HEAD > "$tmp/HEAD"
  echo "$RUNS" > "$tmp/runs"
  aws --region "$REGION" s3 cp --only-show-errors "$tmp/code.tar.gz" "$S3/code.tar.gz"
  aws --region "$REGION" s3 cp --only-show-errors "$tmp/HEAD" "$S3/HEAD"
  aws --region "$REGION" s3 cp --only-show-errors "$tmp/runs" "$S3/runs"
  aws --region "$REGION" s3 cp --only-show-errors "$HERE/remote.sh" "$S3/remote.sh"
  rm -rf "$tmp"
}

run_remote() {
  # Canonical's Ubuntu image has the SSM agent but no AWS CLI.
  bash "$EC2" run "export PATH=\$PATH:/snap/bin; command -v aws >/dev/null || snap install aws-cli --classic >/dev/null; aws s3 cp --only-show-errors $S3/remote.sh /tmp/remote.sh && bash /tmp/remote.sh $S3 $MAX_HOURS"
}

case "${1:-}" in
  # A new launch makes any earlier pull stale, so `down` must see a new one.
  start)  # A fresh run must not mix with an earlier one: anything under the
          # prefix (results, logs, code) stops it. A listing that fails (no
          # access, no session) stops it too -- exit 1 is "nothing there".
          rc=0; aws --region "$REGION" s3 ls "$S3/" >/dev/null 2>&1 || rc=$?
          if [ $rc = 0 ]; then
            echo "$S3/ holds an earlier run -- pull + down first (or resume it)" >&2; exit 1
          elif [ $rc != 1 ]; then
            echo "cannot list $S3/ (exit $rc) -- check the AWS profile" >&2; exit 1
          fi
          ami; push_code; rm -f "$HERE/D4/cloud/.pulled"; bash "$EC2" up; run_remote ;;
  resume) # The rest must run on the code the finished runs ran on.
          pushed="$(aws --region "$REGION" s3 cp --only-show-errors "$S3/HEAD" -)" \
            || { echo "no $S3/HEAD -- nothing to resume (start instead)" >&2; exit 1; }
          if [ "$pushed" != "$(git rev-parse HEAD)" ]; then
            echo "$S3 holds $pushed, HEAD is $(git rev-parse HEAD) -- resume runs the pushed code;" \
                 "check it out, or pull + down and start again" >&2; exit 1
          fi
          ami; rm -f "$HERE/D4/cloud/.pulled"; bash "$EC2" up; run_remote ;;
  status) bash "$EC2" status
          for f in build drive; do
            echo "== $f"; aws --region "$REGION" s3 cp --only-show-errors "$S3/logs/$f.log" - 2>/dev/null \
              | tail -n 4 || echo "(no log yet)"
          done
          aws --region "$REGION" s3 ls "$S3/out/" 2>/dev/null || true ;;
  done)   for r in $RUNS; do
            aws --region "$REGION" s3 ls "$S3/out/$r.json" >/dev/null 2>&1 || exit 1
          done ;;
  pull)   mkdir -p "$HERE/D4/cloud"
          # Pulled logs (a SIM house's poses among them) never reach a commit.
          echo "*" > "$HERE/D4/cloud/.gitignore"
          # Checked BEFORE the download, so nothing can land in S3 unseen
          # between the copy and the mark.
          complete=0; bash "$SELF" done && complete=1
          # cp, not sync: D4/ holds HEAD's results under the same names, and
          # sync keeps a local file of the same size with a newer mtime.
          aws --region "$REGION" s3 cp --recursive --only-show-errors "$S3/out/" "$HERE/D4/"
          aws --region "$REGION" s3 sync --only-show-errors "$S3/logs/" "$HERE/D4/cloud/"
          if [ $complete = 1 ]; then touch "$HERE/D4/cloud/.pulled"; else echo "not all results in yet -- pull again when done"; fi
          ls -la "$HERE/D4" ;;
  down)   bash "$EC2" down
          # The results live only in S3 until pulled: keep them unless pulled
          # (or FORCE=1, to throw them away).
          if [ -f "$HERE/D4/cloud/.pulled" ] || [ "${FORCE:-0}" = 1 ]; then
            aws --region "$REGION" s3 rm --recursive --only-show-errors "$S3/" && echo "deleted $S3/"
          else
            echo "NOT deleting $S3/ -- run pull first (or FORCE=1 down to discard it)"
          fi
          aws --region "$REGION" ec2 describe-instances --filters "Name=tag:Name,Values=$GPU_HOST_NAME" \
            "Name=instance-state-name,Values=pending,running,stopping,stopped" \
            --query 'Reservations[].Instances[].InstanceId' --output text ;;
  *) sed -n '2,15p' "$0"; exit 1 ;;
esac
