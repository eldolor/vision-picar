#!/usr/bin/env bash
# evaluations/goal-learn-396/remote.sh -- runs ON the spot instance (sent by
# cloud.sh through `tools/gpu/ec2.sh run`). Setup only, inside the SSM
# command: fetch the code, make the venv once, pull every stage's checkpoints
# and the logs back from S3 (a relaunch after a spot interruption resumes
# where the last one stopped). Then it starts two detached scripts and
# returns:
#
# * drive.sh -- the shared oracle sets, then the four arms and the rule
#   baseline side by side.
# * watch.sh -- every minute, syncs logs and results (out/ -> ckpt/) to S3;
#   once no trainer has run for two checks in a row it syncs one last time
#   and powers off. The instance was launched with shutdown = terminate, and
#   a dead-man timer powers it off at MAX_HOURS whatever happens.
#
#   bash remote.sh s3://BUCKET/PREFIX [MAX_HOURS]
set -euo pipefail
export PATH="$PATH:/snap/bin"
S3="$1"
MAX_HOURS="${2:-6}"
DIR=/opt/train
if pgrep -f "goal-learn-396/train.py|$DIR/drive.sh|$DIR/watch.sh" >/dev/null; then
  # A watcher alone means a run is finishing (it may power off any minute).
  echo "a run is going or finishing here -- not starting another; resume after it ends"
  pgrep -af "train.py|drive.sh|watch.sh" | head
  exit 0
fi
# The dead-man switch, armed only for a run that actually starts.
sudo shutdown -h "+$((MAX_HOURS * 60))" "3.96 dead-man switch" \
  || { echo "could not arm the dead-man switch -- not starting" >&2; exit 1; }
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

cat > drive.sh <<EOF
#!/usr/bin/env bash
# Not -e: one arm's failure must not stop the others.
export PATH="\$PATH:/snap/bin"
cd $DIR/repo
N=\$(nproc)
export CKPT_S3="$S3/ckpt" PYTHONPATH=.
# One maths thread per process: the workers already fill the cores, and
# numpy's own thread pools on top made the load ~100 on 64 cores (3.96 run 1).
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
# The shared oracle sets, with every core, before any arm reads them. If the
# large set fails (B1 missing), F-small and the rule still run; the large
# arms then fail on their own, each with its reason in its log.
WORKERS=\$((N - 2)) ../venv/bin/python evaluations/goal-learn-396/train.py oracle ../out \\
  >> ../logs/oracle.log 2>&1 || echo "oracle stage FAILED (exit \$?)" >> ../logs/oracle.log
W=\$(( (N - 4) / 4 )); [ "\$W" -ge 1 ] || W=1
for arm in F-small F-large MAP-large PONI-large; do
  [ -f "../out/\$arm/best.json" ] && { echo "\$arm already done" >> ../logs/\$arm.log; continue; }
  WORKERS=\$W ../venv/bin/python evaluations/goal-learn-396/train.py "\$arm" "../out/\$arm" \\
    >> "../logs/\$arm.log" 2>&1 &
done
if [ ! -f ../out/rule/rule-select.json ]; then
  WORKERS=3 ../venv/bin/python evaluations/goal-learn-396/train.py rule ../out/rule \\
    >> ../logs/rule.log 2>&1 &
fi
wait
echo "drive.sh finished \$(date)" >> ../logs/oracle.log
EOF

cat > watch.sh <<EOF
#!/usr/bin/env bash
export PATH="\$PATH:/snap/bin"
idle=0
stuck=0
while sleep 60; do
  aws s3 sync --only-show-errors $DIR/logs $S3/logs
  # Results too, every minute: an upload save_atomic missed is retried here.
  aws s3 sync --only-show-errors --exclude "*.tmp" $DIR/out $S3/ckpt
  if pgrep -f "$DIR/drive.sh" >/dev/null || pgrep -f "goal-learn-396/train.py" >/dev/null; then
    idle=0
  else
    idle=\$((idle + 1))
  fi
  if [ \$idle -ge 2 ]; then
    aws s3 sync --only-show-errors $DIR/logs $S3/logs
    if aws s3 sync --only-show-errors --exclude "*.tmp" $DIR/out $S3/ckpt; then
      sudo shutdown -h now "3.96: no trainer left"
    fi
    # The final sync keeps failing: stop paying after 15 minutes of it.
    stuck=\$((stuck + 1))
    [ \$stuck -ge 15 ] && sudo shutdown -h now "3.96: final sync failing"
  fi
done
EOF
nohup setsid bash "$DIR/drive.sh" >/dev/null 2>&1 < /dev/null &
pgrep -f "$DIR/watch.sh" >/dev/null || nohup setsid bash "$DIR/watch.sh" >/dev/null 2>&1 < /dev/null &
sleep 15
pgrep -af "drive.sh|watch.sh|train.py" | head -8
echo "started on $(nproc) cores; dead-man at ${MAX_HOURS} h"
