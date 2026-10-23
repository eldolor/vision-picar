#!/usr/bin/env bash
# evaluations/goal-learn-396/remote.sh -- runs ON the spot instance (sent by
# cloud.sh through `tools/gpu/ec2.sh run`). Fetches the code, sets up a venv
# once, pulls every arm's checkpoints back from S3 (a relaunch after a spot
# interruption resumes where the last one stopped), then starts the four
# arms and the rule baseline side by side under nohup, each with its share
# of the cores. Logs are synced to S3 every minute.
#
#   bash remote.sh s3://BUCKET/PREFIX
set -euo pipefail
export PATH="$PATH:/snap/bin"
S3="$1"
DIR=/opt/train
sudo mkdir -p "$DIR" && sudo chown -R ubuntu:ubuntu "$DIR"
cd "$DIR"
aws s3 cp --only-show-errors "$S3/code.tar.gz" code.tar.gz
rm -rf repo && mkdir repo && tar -xzf code.tar.gz -C repo
if [ ! -x venv/bin/python ]; then
  dpkg -s python3-venv >/dev/null 2>&1 || {
    sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-venv
  }
  python3 -m venv venv
  ./venv/bin/pip install -q --upgrade pip
  ./venv/bin/pip install -q -r repo/requirements.txt
  ./venv/bin/pip install -q torch --index-url https://download.pytorch.org/whl/cpu
fi
mkdir -p out logs
for arm in F-small F-large MAP-large PONI-large rule; do
  mkdir -p "out/$arm"
  aws s3 sync --only-show-errors "$S3/ckpt/$arm/" "out/$arm/" || true
done
# Workers per arm: the cores split four ways, the rule baseline borrows a few.
N=$(nproc)
W=$(( (N - 4) / 4 ))
cd repo
for arm in F-small F-large MAP-large PONI-large; do
  if aws s3 ls "$S3/ckpt/$arm/best.json" >/dev/null 2>&1; then
    echo "$arm already done"
    continue
  fi
  CKPT_S3="$S3/ckpt" WORKERS=$W PYTHONPATH=. nohup setsid ../venv/bin/python \
    evaluations/goal-learn-396/train.py "$arm" "../out/$arm" \
    >> "../logs/$arm.log" 2>&1 < /dev/null &
done
if ! aws s3 ls "$S3/ckpt/rule/rule-select.json" >/dev/null 2>&1; then
  CKPT_S3="$S3/ckpt" WORKERS=3 PYTHONPATH=. nohup setsid ../venv/bin/python \
    evaluations/goal-learn-396/train.py rule ../out/rule >> ../logs/rule.log 2>&1 < /dev/null &
fi
cd ..
pgrep -f logsync >/dev/null || nohup setsid bash -c \
  "exec -a logsync bash -c 'while sleep 60; do aws s3 sync --only-show-errors $DIR/logs $S3/logs; done'" \
  >/dev/null 2>&1 < /dev/null &
sleep 20
tail -n 3 logs/*.log
echo "started: $(nproc) cores, $W workers an arm"
