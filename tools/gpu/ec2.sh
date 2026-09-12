#!/usr/bin/env bash
# The corpus-scoring host for control/perception_eval.py -- launch, drive,
# tear down. Sibling of tools/hailo/ec2.sh, same two access rules.
#
# **Why a rented GPU at all.** P7 established the shape: the corpus is 610
# labelled frames across eight walks, the shipped tier is three models deep,
# and a laptop pass is tens of minutes per config. A sweep is a dozen
# configs. On an A10G it is an hour, and `down` is still the most important
# subcommand in the file.
#
# **What it is NOT for.** Throughput. 2.9's budget is an 8L's, and an A10G
# says nothing about it -- every millisecond this prints compares configs to
# each other, exactly as perception_eval's own docstring insists.
#
# Two deliberate choices about access, inherited from tools/hailo/ec2.sh:
#
#   * **No key pair, no inbound rules.** Access is SSM Session Manager over
#     the instance's own outbound HTTPS, so the security group authorises no
#     ingress at all.
#   * **The instance profile can read one S3 prefix**, not the account.
#
# Usage:
#   tools/gpu/ec2.sh up        launch (idempotent -- reuses a running one)
#   tools/gpu/ec2.sh status    instance, state, uptime, running cost
#   tools/gpu/ec2.sh push      corpus + code -> S3 -> the instance
#   tools/gpu/ec2.sh setup     venv + requirements-perception.txt, warm models
#   tools/gpu/ec2.sh sweep     the soft-gate sweep (nohup; ^C is safe)
#   tools/gpu/ec2.sh pull      records + logs back into evaluations/gpu/
#   tools/gpu/ec2.sh run "cmd" one command via SSM
#   tools/gpu/ec2.sh shell     interactive SSM session
#   tools/gpu/ec2.sh down      terminate, and delete the SG and role
set -euo pipefail

REGION="${AWS_REGION:-us-east-2}"
NAME="${GPU_HOST_NAME:-vision-picar-gpu-eval}"
TYPE="${GPU_INSTANCE_TYPE:-g5.xlarge}"
VOLUME_GB="${GPU_VOLUME_GB:-150}"
BUCKET="${GPU_BUCKET:-vision-picar-deploy-303351622021-us-east-2}"
PREFIX="${GPU_PREFIX:-gpu-eval}"
OUT_DIR="${GPU_OUT:-evaluations/gpu/softgate}"
REMOTE_DIR="/opt/eval"
# us-east-2 on-demand, g5.xlarge. Used only for the cost readout; if you
# change TYPE, change this or the number printed is a lie.
HOURLY="${GPU_HOURLY:-1.006}"

say() { printf '\033[1m[gpu]\033[0m %s\n' "$*"; }
die() { printf '\033[31m[gpu] %s\033[0m\n' "$*" >&2; exit 1; }
aws_() { aws --region "$REGION" "$@"; }
s3_uri() { printf 's3://%s/%s' "$BUCKET" "$PREFIX"; }

instance_id() {
  aws_ ec2 describe-instances \
    --filters "Name=tag:Name,Values=$NAME" \
              "Name=instance-state-name,Values=pending,running,stopping,stopped" \
    --query 'Reservations[].Instances[0].InstanceId' --output text 2>/dev/null \
    | grep -v '^None$' | head -1 || true
  # `|| true` is load-bearing under `set -euo pipefail` -- see the same note
  # in tools/hailo/ec2.sh. With no instance, grep exits 1 and pipefail takes
  # the whole script down on the one path that has to work first.
}

require_instance() {
  local id; id="$(instance_id)"
  [ -n "$id" ] || die "no instance -- run: tools/gpu/ec2.sh up"
  printf '%s' "$id"
}

ensure_role() {
  local role="$NAME"
  if ! aws iam get-role --role-name "$role" >/dev/null 2>&1; then
    say "creating IAM role $role"
    aws iam create-role --role-name "$role" \
      --description "Corpus scoring host (tools/gpu). Delete with ec2.sh down." \
      --assume-role-policy-document '{"Version":"2012-10-17","Statement":[{"Effect":"Allow","Principal":{"Service":"ec2.amazonaws.com"},"Action":"sts:AssumeRole"}]}' \
      >/dev/null
    aws iam attach-role-policy --role-name "$role" \
      --policy-arn arn:aws:iam::aws:policy/AmazonSSMManagedInstanceCore
    aws iam put-role-policy --role-name "$role" --policy-name s3-eval-prefix \
      --policy-document "$(cat <<JSON
{"Version":"2012-10-17","Statement":[
 {"Effect":"Allow","Action":["s3:GetObject","s3:PutObject","s3:DeleteObject"],
  "Resource":"arn:aws:s3:::$BUCKET/$PREFIX/*"},
 {"Effect":"Allow","Action":["s3:ListBucket"],"Resource":"arn:aws:s3:::$BUCKET",
  "Condition":{"StringLike":{"s3:prefix":["$PREFIX/*"]}}}]}
JSON
)"
  fi
  if ! aws iam get-instance-profile --instance-profile-name "$role" >/dev/null 2>&1; then
    aws iam create-instance-profile --instance-profile-name "$role" >/dev/null
    aws iam add-role-to-instance-profile --instance-profile-name "$role" --role-name "$role"
    say "waiting for the instance profile to propagate"
    sleep 12
  fi
}

delete_role() {
  local role="$NAME"
  aws iam remove-role-from-instance-profile --instance-profile-name "$role" \
    --role-name "$role" 2>/dev/null || true
  aws iam delete-instance-profile --instance-profile-name "$role" 2>/dev/null || true
  aws iam delete-role-policy --role-name "$role" --policy-name s3-eval-prefix 2>/dev/null || true
  aws iam detach-role-policy --role-name "$role" \
    --policy-arn arn:aws:iam::aws:policy/AmazonSSMManagedInstanceCore 2>/dev/null || true
  aws iam delete-role --role-name "$role" 2>/dev/null || true
}

ensure_sg() {
  local vpc sg
  vpc="$(aws_ ec2 describe-vpcs --filters Name=isDefault,Values=true \
        --query 'Vpcs[0].VpcId' --output text)"
  sg="$(aws_ ec2 describe-security-groups \
        --filters "Name=group-name,Values=$NAME" "Name=vpc-id,Values=$vpc" \
        --query 'SecurityGroups[0].GroupId' --output text 2>/dev/null \
        | grep -v '^None$' || true)"
  if [ -z "$sg" ]; then
    # No ingress is authorised, ever. SSM needs egress only.
    sg="$(aws_ ec2 create-security-group --group-name "$NAME" --vpc-id "$vpc" \
          --description "Corpus scoring host: egress only, SSM access" \
          --query GroupId --output text)"
  fi
  printf '%s' "$sg"
}

# ---------------------------------------------------------------- up -------
cmd_up() {
  local id; id="$(instance_id)"
  if [ -n "$id" ]; then say "already up: $id"; cmd_status; return; fi

  ensure_role
  local sg subnet ami
  sg="$(ensure_sg)"
  subnet="$(aws_ ec2 describe-subnets --filters Name=default-for-az,Values=true \
            --query 'Subnets[0].SubnetId' --output text)"
  # The Deep Learning base AMI, resolved through SSM rather than pinned: it
  # carries the NVIDIA driver and CUDA, which is the part that is slow and
  # error-prone to install by hand. Torch itself comes from pip, so the
  # versions this scores under are the repo's own, not the AMI's.
  ami="${GPU_AMI:-$(aws_ ssm get-parameter --name \
      /aws/service/deeplearning/ami/x86_64/base-oss-nvidia-driver-gpu-ubuntu-22.04/latest/ami-id \
      --query 'Parameter.Value' --output text)}"

  say "launching $TYPE from $ami in $subnet (${VOLUME_GB}GB gp3)"
  id="$(aws_ ec2 run-instances \
    --image-id "$ami" --instance-type "$TYPE" --subnet-id "$subnet" \
    --security-group-ids "$sg" \
    --iam-instance-profile "Name=$NAME" \
    --metadata-options 'HttpTokens=required,HttpEndpoint=enabled' \
    --block-device-mappings "[{\"DeviceName\":\"/dev/sda1\",\"Ebs\":{\"VolumeSize\":$VOLUME_GB,\"VolumeType\":\"gp3\",\"DeleteOnTermination\":true,\"Encrypted\":true}}]" \
    --tag-specifications "ResourceType=instance,Tags=[{Key=Name,Value=$NAME},{Key=Project,Value=vision-picar},{Key=Purpose,Value=perception-eval}]" \
    --query 'Instances[0].InstanceId' --output text)"
  say "instance $id -- waiting for SSM to come online (about two minutes)"
  aws_ ec2 wait instance-running --instance-ids "$id"
  for _ in $(seq 60); do
    if aws_ ssm describe-instance-information \
        --filters "Key=InstanceIds,Values=$id" \
        --query 'InstanceInformationList[0].PingStatus' --output text 2>/dev/null \
        | grep -q Online; then
      say "SSM online"; cmd_status; return
    fi
    sleep 10
  done
  die "SSM never came online for $id -- check the instance in the console"
}

# ------------------------------------------------------------- status ------
cmd_status() {
  local id; id="$(instance_id)" || true
  if [ -z "$id" ]; then say "no instance ($NAME)"; return; fi
  local state itype launched
  # Read the type from AWS, not from $TYPE: the env var describes what the
  # NEXT `up` would launch, and a cost figure derived from the wrong one is
  # a readout lying about money.
  state="$(aws_ ec2 describe-instances --instance-ids "$id" \
    --query 'Reservations[0].Instances[0].State.Name' --output text)"
  itype="$(aws_ ec2 describe-instances --instance-ids "$id" \
    --query 'Reservations[0].Instances[0].InstanceType' --output text)"
  launched="$(aws_ ec2 describe-instances --instance-ids "$id" \
    --query 'Reservations[0].Instances[0].LaunchTime' --output text)"
  python3 - "$id" "$state" "$itype" "$launched" "$HOURLY" <<'PY'
import sys, datetime
iid, state, itype, launched, hourly = sys.argv[1:6]
t = datetime.datetime.fromisoformat(launched.replace("Z", "+00:00"))
hrs = (datetime.datetime.now(datetime.timezone.utc) - t).total_seconds() / 3600
print(f"[gpu] {iid}  {state}  {itype}  up {hrs:.2f}h  "
      f"~${hrs * float(hourly):.2f} so far")
PY
}

# --------------------------------------------------------------- run -------
cmd_run() {
  # The script is base64'd into a ONE-LINE command rather than passed as
  # multi-line text. SSM's RunShellScript mangles an embedded newline --
  # the far side reported `set: Illegal option -c` on a plain `set -e\n...`
  # payload, and a push whose body never ran still reported Success, which
  # is the worst of both. One line of base64 has no newline, no quote and
  # no comma for any layer between here and dash to reinterpret.
  local id cmd out b64
  id="$(require_instance)"
  cmd="$*"
  b64="$(printf '%s' "$cmd" | base64 | tr -d '\n')"
  out="$(aws_ ssm send-command --instance-ids "$id" \
      --document-name AWS-RunShellScript \
      --parameters "commands=[\"echo $b64 | base64 -d > /tmp/ec2-run.sh; bash /tmp/ec2-run.sh\"]" \
      --timeout-seconds 3600 \
      --query 'Command.CommandId' --output text)"
  for _ in $(seq 360); do
    local st
    st="$(aws_ ssm get-command-invocation --command-id "$out" --instance-id "$id" \
          --query Status --output text 2>/dev/null || echo Pending)"
    case "$st" in
      Success|Failed|Cancelled|TimedOut)
        aws_ ssm get-command-invocation --command-id "$out" --instance-id "$id" \
          --query StandardOutputContent --output text
        aws_ ssm get-command-invocation --command-id "$out" --instance-id "$id" \
          --query StandardErrorContent --output text >&2
        [ "$st" = Success ] || die "command $st"
        return 0 ;;
    esac
    sleep 5
  done
  die "command never finished"
}

cmd_shell() { aws_ ssm start-session --target "$(require_instance)"; }

# -------------------------------------------------------------- push -------
cmd_push() {
  [ -d recordings ] || die "no recordings/ -- the corpus is the input"
  say "uploading corpus + code -> $(s3_uri)/"
  # The corpus is ~119MB of JPEG. Everything else is text.
  aws_ s3 sync recordings "$(s3_uri)/recordings/" --only-show-errors
  for d in brain control sim robot config; do
    aws_ s3 sync "$d" "$(s3_uri)/code/$d/" --exclude '__pycache__/*' --only-show-errors
  done
  aws_ s3 cp requirements-perception.txt "$(s3_uri)/code/" --only-show-errors
  aws_ s3 cp requirements.txt "$(s3_uri)/code/" --only-show-errors
  aws_ s3 cp tools/gpu/sweep.py "$(s3_uri)/code/" --only-show-errors
  cmd_run "set -e
    sudo mkdir -p $REMOTE_DIR && sudo chown -R ubuntu:ubuntu $REMOTE_DIR
    cd $REMOTE_DIR
    aws s3 sync $(s3_uri)/recordings/ recordings/ --only-show-errors
    aws s3 sync $(s3_uri)/code/ . --only-show-errors
    du -sh recordings; ls"
}

# ------------------------------------------------------------- setup -------
cmd_setup() {
  say "installing requirements-perception.txt (several minutes -- torch is big)"
  cmd_run "set -e
    cd $REMOTE_DIR
    # The DL base AMI ships python3 without ensurepip, so `-m venv` fails
    # with a message about python3.10-venv. Install it rather than fall
    # back to the system interpreter: the versions this scores under have
    # to be the repo's requirements files, not whatever the AMI pinned.
    dpkg -s python3-venv >/dev/null 2>&1 || {
      sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
      sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-venv
    }
    # Test for the interpreter, not the directory: a venv that failed
    # half-way leaves the directory behind, and `test -d` then skips the
    # retry that would have fixed it.
    test -x venv/bin/pip || { rm -rf venv && python3 -m venv venv; }
    ./venv/bin/pip install -q --upgrade pip
    ./venv/bin/pip install -q -r requirements.txt
    ./venv/bin/pip install -q -r requirements-perception.txt
    ./venv/bin/python -c \"import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '')\"
  "
  say "warming the model cache (yolo11s + CLIP RN50 + segformer)"
  # Warm them under `setup` rather than inside the first scored walk, so no
  # config's latency column silently includes a download.
  cmd_run "cd $REMOTE_DIR && ./venv/bin/python -c \"
from brain.perceive_lab import pipeline_for_spec
p = pipeline_for_spec('warm up', device='cuda', proposer='floor')
print('warm:', type(p).__name__)
\" 2>&1 | tail -5"
}

# ------------------------------------------------------------- sweep -------
cmd_sweep() {
  say "starting the sweep under nohup -- ^C is safe, follow it with \`logs\`"
  # `setsid`, not just `nohup`. A plain `nohup ... &` under SSM
  # RunShellScript died silently ~3h in -- no traceback, no OOM, the log
  # simply stopped mid-walk -- because the agent reaps the document's
  # process GROUP when the command completes. setsid leaves the session,
  # so the sweep outlives the shell that started it. The failure mode is
  # expensive and invisible: the log looks like a slow run, not a dead one.
  cmd_run "cd $REMOTE_DIR && setsid ./venv/bin/python sweep.py \
      --out out --device cuda $* > sweep.log 2>&1 < /dev/null &
    disown; echo started; sleep 10; tail -20 sweep.log"
  say "follow with: tools/gpu/ec2.sh logs"
}

cmd_logs() { cmd_run "tail -${1:-40} $REMOTE_DIR/sweep.log"; }

# -------------------------------------------------------------- pull -------
cmd_pull() {
  say "collecting records from the instance"
  cmd_run "cd $REMOTE_DIR && aws s3 sync out/ $(s3_uri)/result/ --only-show-errors \
    && aws s3 cp sweep.log $(s3_uri)/result/sweep.log --only-show-errors && echo synced"
  mkdir -p "$OUT_DIR"
  aws_ s3 sync "$(s3_uri)/result/" "$OUT_DIR/"
  say "-> $OUT_DIR"
  ls -la "$OUT_DIR"
}

# -------------------------------------------------------------- down -------
cmd_down() {
  local id; id="$(instance_id)" || true
  if [ -n "$id" ]; then
    say "terminating $id"
    aws_ ec2 terminate-instances --instance-ids "$id" >/dev/null
    aws_ ec2 wait instance-terminated --instance-ids "$id"
    say "terminated"
  else
    say "no instance to terminate"
  fi
  aws_ ec2 delete-security-group --group-name "$NAME" 2>/dev/null \
    && say "security group deleted" || true
  delete_role
  say "role and instance profile deleted"
  say "NOTE: s3://$BUCKET/$PREFIX/ is left in place -- delete it by hand if you want it gone"
}

case "${1:-}" in
  up)     shift; cmd_up "$@" ;;
  status) shift; cmd_status "$@" ;;
  push)   shift; cmd_push "$@" ;;
  setup)  shift; cmd_setup "$@" ;;
  sweep)  shift; cmd_sweep "$@" ;;
  logs)   shift; cmd_logs "$@" ;;
  pull)   shift; cmd_pull "$@" ;;
  run)    shift; cmd_run "$@" ;;
  shell)  shift; cmd_shell "$@" ;;
  down)   shift; cmd_down "$@" ;;
  *) sed -n '1,31p' "$0"; exit 1 ;;
esac
