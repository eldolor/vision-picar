#!/usr/bin/env bash
# evaluations/goal-learn-396/remote.sh -- runs ON the spot instance (sent by
# cloud.sh through `tools/gpu/ec2.sh run`). Fetches the code, sets up a venv
# once, pulls every stage's checkpoints and the logs back from S3 (a relaunch
# after a spot interruption resumes where the last one stopped), makes the
# shared oracle sets, then starts the four arms and the rule baseline side
# by side under nohup. Logs are synced to S3 every minute.
#
# The instance ends itself (it was launched with shutdown = terminate):
# a watcher powers off once every result is in S3, and a dead-man timer
# powers off at MAX_HOURS whatever happens.
#
#   bash remote.sh s3://BUCKET/PREFIX [MAX_HOURS]
set -euo pipefail
export PATH="$PATH:/snap/bin"
S3="$1"
MAX_HOURS="${2:-6}"
DIR=/opt/train
ARMS="F-small F-large MAP-large PONI-large"
# The dead-man switch first, before anything that can fail or hang.
sudo shutdown -h "+$((MAX_HOURS * 60))" "3.96 dead-man switch" >/dev/null 2>&1 || true
if pgrep -f "goal-learn-396/train.py" >/dev/null; then
  echo "trainers already running here -- not starting a second set"
  pgrep -af "goal-learn-396/train.py" | head
  exit 0
fi
sudo mkdir -p "$DIR" && sudo chown -R ubuntu:ubuntu "$DIR"
cd "$DIR"
aws s3 cp --only-show-errors "$S3/code.tar.gz" code.tar.gz
rm -rf repo && mkdir repo && tar -xzf code.tar.gz -C repo
# venv/.ok is written only after every install succeeded: a half-made venv
# (a pip blip) is rebuilt, never reused.
if [ ! -f venv/.ok ]; then
  rm -rf venv
  dpkg -s python3-venv >/dev/null 2>&1 || {
    sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-venv
  }
  python3 -m venv venv
  ./venv/bin/pip install -q --upgrade pip
  ./venv/bin/pip install -q -r repo/requirements.txt
  ./venv/bin/pip install -q torch --index-url https://download.pytorch.org/whl/cpu
  touch venv/.ok
fi
mkdir -p out logs
# Resume state: a failed sync must stop here, or an arm would start over and
# overwrite its good checkpoint in S3.
aws s3 sync --only-show-errors "$S3/ckpt/" out/
aws s3 sync --only-show-errors "$S3/logs/" logs/
N=$(nproc)
cd repo
# The shared oracle sets, with every core, before any arm reads them.
CKPT_S3="$S3/ckpt" WORKERS=$((N - 2)) PYTHONPATH=. ../venv/bin/python \
  evaluations/goal-learn-396/train.py oracle ../out >> ../logs/oracle.log 2>&1
aws s3 sync --only-show-errors ../logs "$S3/logs"
W=$(( (N - 4) / 4 )); [ "$W" -ge 1 ] || W=1
for arm in $ARMS; do
  if [ -f "../out/$arm/best.json" ]; then echo "$arm already done"; continue; fi
  CKPT_S3="$S3/ckpt" WORKERS=$W PYTHONPATH=. nohup setsid ../venv/bin/python \
    evaluations/goal-learn-396/train.py "$arm" "../out/$arm" \
    >> "../logs/$arm.log" 2>&1 < /dev/null &
done
if [ ! -f ../out/rule/rule-select.json ]; then
  CKPT_S3="$S3/ckpt" WORKERS=3 PYTHONPATH=. nohup setsid ../venv/bin/python \
    evaluations/goal-learn-396/train.py rule ../out/rule >> ../logs/rule.log 2>&1 < /dev/null &
fi
cd ..
# Logs every minute; power off once every result is in S3.
cat > watch.sh <<EOF
#!/usr/bin/env bash
export PATH="\$PATH:/snap/bin"
while sleep 60; do
  aws s3 sync --only-show-errors $DIR/logs $S3/logs
  all=1
  for f in F-small/best.json F-large/best.json MAP-large/best.json PONI-large/best.json rule/rule-select.json; do
    aws s3 ls "$S3/ckpt/\$f" >/dev/null 2>&1 || all=0
  done
  if [ \$all = 1 ] && ! pgrep -f "goal-learn-396/train.py" >/dev/null; then
    aws s3 sync --only-show-errors $DIR/logs $S3/logs
    sudo shutdown -h now "3.96 finished"
  fi
done
EOF
pgrep -f "$DIR/watch.sh" >/dev/null || nohup setsid bash "$DIR/watch.sh" >/dev/null 2>&1 < /dev/null &
sleep 20
tail -n 3 logs/*.log
echo "started: $N cores, $W workers an arm; dead-man at ${MAX_HOURS} h"
