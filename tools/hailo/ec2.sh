#!/usr/bin/env bash
# The compile host for tools/hailo -- launch, drive, and tear down.
#
# `PLAN-onboard-perception.md` 1.10 item 1: the Hailo Dataflow Compiler is a
# linux_x86_64 wheel with no Mac and no ARM path, so the loop needs a rented
# x86 box for an hour or two per model. This is that box, and `down` is the
# most important subcommand in the file.
#
# Two deliberate choices about access:
#
#   * **No key pair, no inbound rules.** Access is SSM Session Manager over
#     the instance's own outbound HTTPS, so the security group opens nothing
#     at all. There is no SSH port to leave open on an instance holding a
#     gated vendor wheel.
#   * **The instance profile can read one S3 prefix**, not the account. It
#     needs the wheel and the exports; it has no reason to see anything else.
#
# Usage:
#   tools/hailo/ec2.sh up          launch (idempotent -- reuses a running one)
#   tools/hailo/ec2.sh status      instance, state, uptime, running cost
#   tools/hailo/ec2.sh push        build/owlv2 -> S3 -> the instance
#   tools/hailo/ec2.sh setup       install the DFC and deps on the instance
#   tools/hailo/ec2.sh compile     run the sweep (streams to a local log)
#   tools/hailo/ec2.sh pull        artifacts + report back to build/owlv2
#   tools/hailo/ec2.sh shell       interactive SSM session
#   tools/hailo/ec2.sh run "cmd"   one command via SSM
#   tools/hailo/ec2.sh down        terminate, and delete the SG and role
set -euo pipefail

REGION="${AWS_REGION:-us-east-2}"
NAME="${HAILO_HOST_NAME:-vision-picar-hailo-compile}"
TYPE="${HAILO_INSTANCE_TYPE:-r6i.4xlarge}"
VOLUME_GB="${HAILO_VOLUME_GB:-200}"
BUCKET="${HAILO_BUCKET:-vision-picar-deploy-303351622021-us-east-2}"
PREFIX="${HAILO_PREFIX:-hailo}"
LOCAL_BUILD="${HAILO_BUILD:-build/owlv2}"
REMOTE_DIR="/opt/hailo"
# us-east-2 on-demand, r6i.4xlarge. Used only for the cost readout; if you
# change TYPE, change this or the number printed is a lie.
HOURLY="${HAILO_HOURLY:-1.008}"

say() { printf '\033[1m[hailo]\033[0m %s\n' "$*"; }
die() { printf '\033[31m[hailo] %s\033[0m\n' "$*" >&2; exit 1; }

aws_() { aws --region "$REGION" "$@"; }

instance_id() {
  aws_ ec2 describe-instances \
    --filters "Name=tag:Name,Values=$NAME" \
              "Name=instance-state-name,Values=pending,running,stopping,stopped" \
    --query 'Reservations[].Instances[0].InstanceId' --output text 2>/dev/null \
    | grep -v '^None$' | head -1 || true
  # `|| true` is load-bearing under `set -euo pipefail`: with no instance,
  # grep matches nothing and exits 1, pipefail promotes that to the
  # pipeline's status, and the caller's `id="$(instance_id)"` then takes the
  # whole script down -- silently, on the ONE path that has to work first,
  # which is `up` with nothing running yet.
}

require_instance() {
  local id; id="$(instance_id)"
  [ -n "$id" ] || die "no instance named $NAME -- run 'up' first"
  printf '%s' "$id"
}

s3_uri() { printf 's3://%s/%s' "$BUCKET" "$PREFIX"; }

# ---------------------------------------------------------------- iam ------
ensure_role() {
  local role="$NAME" profile="$NAME"
  if ! aws iam get-role --role-name "$role" >/dev/null 2>&1; then
    say "creating IAM role $role"
    aws iam create-role --role-name "$role" \
      --description "Hailo DFC compile host (tools/hailo). Delete with ec2.sh down." \
      --assume-role-policy-document '{"Version":"2012-10-17","Statement":[{"Effect":"Allow","Principal":{"Service":"ec2.amazonaws.com"},"Action":"sts:AssumeRole"}]}' \
      >/dev/null
    aws iam attach-role-policy --role-name "$role" \
      --policy-arn arn:aws:iam::aws:policy/AmazonSSMManagedInstanceCore
    aws iam put-role-policy --role-name "$role" --policy-name s3-build-prefix \
      --policy-document "$(cat <<JSON
{"Version":"2012-10-17","Statement":[
 {"Effect":"Allow","Action":["s3:GetObject","s3:PutObject","s3:DeleteObject"],
  "Resource":"arn:aws:s3:::$BUCKET/$PREFIX/*"},
 {"Effect":"Allow","Action":["s3:ListBucket"],"Resource":"arn:aws:s3:::$BUCKET",
  "Condition":{"StringLike":{"s3:prefix":["$PREFIX/*"]}}}]}
JSON
)"
  fi
  if ! aws iam get-instance-profile --instance-profile-name "$profile" >/dev/null 2>&1; then
    aws iam create-instance-profile --instance-profile-name "$profile" >/dev/null
    aws iam add-role-to-instance-profile --instance-profile-name "$profile" \
      --role-name "$role"
    say "waiting for the instance profile to propagate"
    sleep 12
  fi
}

delete_role() {
  local role="$NAME"
  aws iam remove-role-from-instance-profile --instance-profile-name "$role" \
    --role-name "$role" 2>/dev/null || true
  aws iam delete-instance-profile --instance-profile-name "$role" 2>/dev/null || true
  aws iam delete-role-policy --role-name "$role" --policy-name s3-build-prefix 2>/dev/null || true
  aws iam detach-role-policy --role-name "$role" \
    --policy-arn arn:aws:iam::aws:policy/AmazonSSMManagedInstanceCore 2>/dev/null || true
  aws iam delete-role --role-name "$role" 2>/dev/null || true
}

# ----------------------------------------------------------------- sg ------
ensure_sg() {
  local vpc sg
  vpc="$(aws_ ec2 describe-vpcs --filters Name=isDefault,Values=true \
        --query 'Vpcs[0].VpcId' --output text)"
  sg="$(aws_ ec2 describe-security-groups \
        --filters "Name=group-name,Values=$NAME" "Name=vpc-id,Values=$vpc" \
        --query 'SecurityGroups[0].GroupId' --output text 2>/dev/null | grep -v '^None$' || true)"
  if [ -z "$sg" ]; then
    # No ingress is authorised, ever. SSM needs egress only.
    sg="$(aws_ ec2 create-security-group --group-name "$NAME" --vpc-id "$vpc" \
          --description "Hailo compile host: egress only, SSM access" \
          --query GroupId --output text)"
  fi
  printf '%s' "$sg"
}

# --------------------------------------------------------------- cli ------
ensure_cli() {
  # The Ubuntu AMI ships no AWS CLI. `push` needs it to pull from S3, and it
  # runs BEFORE `setup` -- which is where the install used to live, so the
  # remote sync failed silently and left the instance empty. A property of
  # the host belongs at the host, not inside one step.
  cmd_run 'command -v aws >/dev/null && { aws --version; exit 0; }
    export DEBIAN_FRONTEND=noninteractive
    sudo apt-get update -qq
    sudo apt-get install -y -qq unzip curl >/dev/null
    curl -sS https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip -o /tmp/awscli.zip
    unzip -q -o /tmp/awscli.zip -d /tmp
    sudo /tmp/aws/install --update >/dev/null
    aws --version' >/dev/null
  say "aws cli present on the host"
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
  # Jammy, not Noble: the DFC targets Ubuntu 20.04/22.04 and Python 3.10,
  # and 24.04 ships 3.12 -- which is a dependency fight this box does not
  # need to have.
  ami="${HAILO_AMI:-$(aws_ ec2 describe-images --owners 099720109477 \
        --filters 'Name=name,Values=ubuntu/images/hvm-ssd/ubuntu-jammy-22.04-amd64-server-*' \
                  Name=state,Values=available \
        --query 'sort_by(Images,&CreationDate)[-1].ImageId' --output text)}"

  say "launching $TYPE from $ami in $subnet (${VOLUME_GB}GB gp3)"
  id="$(aws_ ec2 run-instances \
    --image-id "$ami" --instance-type "$TYPE" --subnet-id "$subnet" \
    --security-group-ids "$sg" \
    --iam-instance-profile "Name=$NAME" \
    --metadata-options 'HttpTokens=required,HttpEndpoint=enabled' \
    --block-device-mappings "[{\"DeviceName\":\"/dev/sda1\",\"Ebs\":{\"VolumeSize\":$VOLUME_GB,\"VolumeType\":\"gp3\",\"DeleteOnTermination\":true,\"Encrypted\":true}}]" \
    --tag-specifications "ResourceType=instance,Tags=[{Key=Name,Value=$NAME},{Key=Project,Value=vision-picar},{Key=Purpose,Value=hailo-dfc-owlv2}]" \
    --query 'Instances[0].InstanceId' --output text)"
  say "instance $id -- waiting for SSM to come online (about two minutes)"
  aws_ ec2 wait instance-running --instance-ids "$id"
  for _ in $(seq 60); do
    if aws_ ssm describe-instance-information \
        --filters "Key=InstanceIds,Values=$id" \
        --query 'InstanceInformationList[0].PingStatus' --output text 2>/dev/null \
        | grep -q Online; then
      say "SSM online"; ensure_cli; cmd_status; return
    fi
    sleep 10
  done
  die "SSM never came online for $id -- check the instance in the console"
}

# ------------------------------------------------------------- status ------
cmd_status() {
  local id; id="$(instance_id)" || true
  if [ -z "$id" ]; then say "no instance ($NAME)"; return; fi
  local launched state itype
  state="$(aws_ ec2 describe-instances --instance-ids "$id" \
    --query 'Reservations[0].Instances[0].State.Name' --output text)"
  # Read the type from AWS, not from $TYPE. The env var describes what the
  # NEXT `up` would launch; a status call made without the same exports
  # reported the wrong instance type beside a cost figure derived from
  # $HOURLY -- which is how a readout lies about money rather than about a
  # label.
  itype="$(aws_ ec2 describe-instances --instance-ids "$id" \
    --query 'Reservations[0].Instances[0].InstanceType' --output text)"
  launched="$(aws_ ec2 describe-instances --instance-ids "$id" \
    --query 'Reservations[0].Instances[0].LaunchTime' --output text)"
  # Args, not string interpolation: an f-string with nested quotes is a
  # syntax error on the Pythons this might run against.
  python3 - "$id" "$itype" "$state" "$launched" "$HOURLY" "$VOLUME_GB" <<'PY'
import datetime, sys
iid, itype, state, launched, hourly, gb = sys.argv[1:7]
up = (datetime.datetime.now(datetime.timezone.utc)
      - datetime.datetime.fromisoformat(launched.replace("Z", "+00:00")))
hours = up.total_seconds() / 3600
ebs = int(gb) * 0.08 / 730
print(f"[hailo] {iid}  {itype}  {state}  up {hours:.2f}h")
if itype not in ("r6i.4xlarge",) and abs(float(hourly) - 1.008) < 1e-6:
    print("[hailo] NOTE: $/hr is the default for r6i.4xlarge; "
          f"this is a {itype}. Export HAILO_HOURLY for a true figure.")
print(f"[hailo] compute so far: ${hours * float(hourly):.2f} "
      f"at ${hourly}/hr, plus EBS ~${ebs:.3f}/hr "
      f"({hours * ebs:.2f} so far)")
PY
}

# --------------------------------------------------------------- ssm -------
cmd_run() {
  local id script b64 params cid status
  id="$(require_instance)"
  [ $# -gt 0 ] || die "run needs a command"
  # SSM's shell carries almost no environment, and the DFC reads several
  # variables unconditionally: HOME in hailo_model_optimization's logger
  # (import time), and USER inside the COMPILE stage. Each surfaces as a
  # bare KeyError at a different point, and the second one is the dangerous
  # kind -- `KeyError: 'USER'` raised from runner.compile() is recorded by
  # this loop as "compile failed", which is an ANSWER to the question the
  # whole exercise is asking. It is not one. VIRTUAL_ENV is set so the DFC
  # keeps its working folder in the venv rather than scattering it.
  script="export HOME=\"\${HOME:-/root}\"
export USER=\"\${USER:-root}\"
export LOGNAME=\"\${LOGNAME:-root}\"
export VIRTUAL_ENV=\"\${VIRTUAL_ENV:-/opt/hailo/venv}\"
$*"

  # Base64, not `--parameters commands=[...]`. The CLI's shorthand syntax
  # treats { } [ ] , = as structure, so any real shell script sent that way
  # comes out mangled -- which showed up here as a `Syntax error: end of
  # file unexpected` on line 2 of a script that is fine.
  #
  # And `| bash` is the other half: AWS-RunShellScript executes its payload
  # with /bin/sh, which on Ubuntu is dash. Everything sent through here is
  # written as bash.
  b64="$(printf '%s' "$script" | base64 | tr -d '\n')"
  params="$(mktemp)"
  # No `trap ... RETURN` here: a RETURN trap stays installed after this
  # function returns and fires again on the NEXT function's return, where
  # $params is out of scope -- which under `set -u` aborts the script with
  # "params: unbound variable" somewhere unrelated. Clean up inline instead.
  python3 -c 'import json,sys; print(json.dumps({"commands": ["echo " + sys.argv[1] + " | base64 -d | bash"]}))' "$b64" > "$params"

  cid="$(aws_ ssm send-command --instance-ids "$id" \
    --document-name AWS-RunShellScript \
    --comment "tools/hailo" \
    --timeout-seconds 3600 \
    --cloud-watch-output-config CloudWatchOutputEnabled=false \
    --parameters "file://$params" \
    --query 'Command.CommandId' --output text)"
  rm -f "$params"

  # Poll rather than `wait`: the DFC prints for minutes and a silent wait is
  # indistinguishable from a hang.
  while true; do
    status="$(aws_ ssm get-command-invocation --command-id "$cid" \
      --instance-id "$id" --query Status --output text 2>/dev/null || echo Pending)"
    case "$status" in
      Success|Failed|Cancelled|TimedOut) break ;;
    esac
    sleep 10
  done
  aws_ ssm get-command-invocation --command-id "$cid" --instance-id "$id" \
    --query StandardOutputContent --output text
  if [ "$status" != Success ]; then
    aws_ ssm get-command-invocation --command-id "$cid" --instance-id "$id" \
      --query StandardErrorContent --output text >&2
    die "command $status (id $cid)"
  fi
}

cmd_shell() {
  local id; id="$(require_instance)"
  say "aws ssm start-session --region $REGION --target $id"
  exec aws_ ssm start-session --target "$id"
}

# -------------------------------------------------------------- files ------
cmd_push() {
  [ -d "$LOCAL_BUILD" ] || die "$LOCAL_BUILD does not exist -- export first"
  ensure_cli
  say "uploading $LOCAL_BUILD -> $(s3_uri)/build/"
  aws_ s3 sync "$LOCAL_BUILD" "$(s3_uri)/build/" --exclude '*.hef' --exclude 'logs/*'
  say "uploading tools/hailo -> $(s3_uri)/tools/"
  aws_ s3 sync tools/hailo "$(s3_uri)/tools/" --exclude '__pycache__/*'
  cmd_run "set -e
    mkdir -p $REMOTE_DIR/build/owlv2 $REMOTE_DIR/tools/hailo
    touch $REMOTE_DIR/tools/__init__.py
    aws s3 sync $(s3_uri)/build/ $REMOTE_DIR/build/owlv2/ --only-show-errors
    aws s3 sync $(s3_uri)/tools/ $REMOTE_DIR/tools/hailo/ --only-show-errors
    du -sh $REMOTE_DIR/build/owlv2; ls -la $REMOTE_DIR/build/owlv2"
}

cmd_pull() {
  say "collecting artifacts from the instance"
  cmd_run "cd $REMOTE_DIR && aws s3 sync build/owlv2/ $(s3_uri)/result/ \
    --exclude '*.npy' --exclude '*.onnx' --only-show-errors && echo synced"
  mkdir -p "$LOCAL_BUILD/result"
  aws_ s3 sync "$(s3_uri)/result/" "$LOCAL_BUILD/result/"
  say "-> $LOCAL_BUILD/result"
  ls -la "$LOCAL_BUILD/result" || true
}

# -------------------------------------------------------------- work -------
cmd_setup() {
  local wheel
  wheel="$(aws_ s3 ls "$(s3_uri)/dfc/" 2>/dev/null | awk '{print $4}' \
           | grep -i '\.whl$' | head -1 || true)"
  [ -n "$wheel" ] || die "no .whl under $(s3_uri)/dfc/ -- upload the Dataflow Compiler wheel there first"
  say "found wheel: $wheel"
  cmd_run "sudo bash $REMOTE_DIR/tools/hailo/setup_host.sh '$(s3_uri)/dfc/$wheel' 2>&1 | tail -60"
}

cmd_compile() {
  local id; id="$(require_instance)"
  say "starting the sweep -- this is the long one; ^C is safe, it runs under nohup"
  cmd_run "cd $REMOTE_DIR && nohup $REMOTE_DIR/venv/bin/python -m tools.hailo.compile_owlv2 \
      --build build/owlv2 --arch hailo8l $* > $REMOTE_DIR/compile.log 2>&1 &
    echo started; sleep 5; tail -5 $REMOTE_DIR/compile.log"
  say "follow it with: tools/hailo/ec2.sh run 'tail -40 $REMOTE_DIR/compile.log'"
}

# -------------------------------------------------------------- down -------
cmd_down() {
  local id; id="$(instance_id)" || true
  if [ -n "$id" ]; then
    say "terminating $id"
    aws_ ec2 terminate-instances --instance-ids "$id" >/dev/null
    aws_ ec2 wait instance-terminated --instance-ids "$id"
    say "terminated (its EBS volume went with it -- DeleteOnTermination)"
  else
    say "no instance to terminate"
  fi
  # The SG cannot be deleted until the ENI is really gone; retry briefly.
  local sg
  sg="$(aws_ ec2 describe-security-groups --filters "Name=group-name,Values=$NAME" \
        --query 'SecurityGroups[0].GroupId' --output text 2>/dev/null | grep -v '^None$' || true)"
  if [ -n "$sg" ]; then
    for _ in $(seq 12); do
      if aws_ ec2 delete-security-group --group-id "$sg" 2>/dev/null; then
        say "deleted security group $sg"; break
      fi
      sleep 10
    done
  fi
  delete_role
  say "IAM role and instance profile deleted"
  say "S3 under $(s3_uri) is LEFT IN PLACE -- it holds the wheel and the report."
  say "  remove it with: aws s3 rm $(s3_uri)/ --recursive"
}

case "${1:-}" in
  up)      shift; cmd_up "$@" ;;
  status)  shift; cmd_status "$@" ;;
  push)    shift; cmd_push "$@" ;;
  setup)   shift; cmd_setup "$@" ;;
  compile) shift; cmd_compile "$@" ;;
  pull)    shift; cmd_pull "$@" ;;
  shell)   shift; cmd_shell "$@" ;;
  run)     shift; cmd_run "$@" ;;
  down)    shift; cmd_down "$@" ;;
  *) sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
