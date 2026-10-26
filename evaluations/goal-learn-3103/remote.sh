#!/usr/bin/env bash
# evaluations/goal-learn-3103/remote.sh -- runs ON the spot instance (sent by
# cloud.sh through `tools/gpu/ec2.sh run`); 3.96's remote.sh with 3.103.C
# amendment 2's stages. Setup only, inside the SSM command: fetch the code,
# make the venv once, pull every stage's checkpoints and the logs back from S3
# (a relaunch after a spot interruption resumes where the last one stopped).
# Then it starts two detached scripts and returns:
#
# * drive.sh -- 3.101's oracle sets (42 columns) with every core; then F-pool
#   widths 32 and 128, clone then PPO, side by side; then, alone: the choice
#   between the three widths (64 was trained in the cloud container and comes
#   from the repo), the chosen width on the 588 test houses, the score, and
#   criterion 5's timing with nothing else running.
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
if pgrep -f "goal-learn-3103/pool_run.py|$DIR/drive.sh|$DIR/watch.sh" >/dev/null; then
  # A watcher alone means a run is finishing (it may power off any minute).
  echo "a run is going or finishing here -- not starting another; resume after it ends"
  pgrep -af "pool_run.py|drive.sh|watch.sh" | head
  exit 0
fi
# The dead-man switch, armed only for a run that actually starts.
sudo shutdown -h "+$((MAX_HOURS * 60))" "3.103 dead-man switch" \
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
# Resume state: a failed sync must stop here, or a width would start over and
# overwrite its good checkpoint in S3.
aws s3 sync --only-show-errors "$S3/ckpt/" out/
aws s3 sync --only-show-errors "$S3/logs/" logs/

# The oracle sets: 3.101's harness, exactly as the cloud container ran it.
cat > oracle.py <<'PY'
import multiprocessing
import sys
multiprocessing.set_start_method("fork", force=True)
sys.argv = ["x"]
sys.path.insert(0, "evaluations/goal-learn-3101")
import run as R  # noqa: E402  (3.101's run.py)
R.use_rich(True)
from concurrent.futures import ProcessPoolExecutor  # noqa: E402
with ProcessPoolExecutor(R.T.WORKERS) as ex:
    R.T.stage_oracle("large", "../out", ex)
    R.T._select_oracle("../out", ex)
print("ORACLE DONE", flush=True)
PY

cat > drive.sh <<EOF
#!/usr/bin/env bash
# Not -e: one width's failure must not stop the other.
export PATH="\$PATH:/snap/bin"
cd $DIR/repo
N=\$(nproc)
export CKPT_S3="$S3/ckpt" PYTHONPATH=.
# One maths thread per process: the workers already fill the cores (3.96 run 1).
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
R=../out
P="../venv/bin/python evaluations/goal-learn-3103/pool_run.py"
# From the cloud container, via the repo: B's F-lean (C's comparison and cost
# base) and its test rows, and the width-64 F-pool. Never over a resumed copy.
mkdir -p \$R/test
for d in F-lean-S64 F-pool-W64; do
  [ -f \$R/\$d/best.json ] || { mkdir -p \$R/\$d; cp evaluations/goal-learn-3103/run/\$d/best.json \$R/\$d/; }
done
for m in R U; do
  [ -f \$R/test/F-lean-\$m.json ] || cp evaluations/goal-learn-3103/run/test/F-lean-\$m.json \$R/test/
done
WORKERS=\$((N - 2)) ../venv/bin/python $DIR/oracle.py >> ../logs/oracle.log 2>&1 \\
  || { echo "oracle stage FAILED (exit \$?)" >> ../logs/oracle.log; exit 1; }
W=\$(( (N - 4) / 2 )); [ "\$W" -ge 1 ] || W=1
for w in 32 128; do
  ( export POOL_WIDTH=\$w WORKERS=\$W
    [ -f \$R/F-pool-W\$w/clone.pt ] || \$P clone \$R
    \$P ppo \$R ) >> ../logs/F-pool-W\$w.log 2>&1 &
done
wait
# Alone from here: the choice, the one test, the score, then the timing.
( export WORKERS=\$((N - 2))
  \$P choose \$R && \$P test \$R && \$P score \$R && \$P time \$R ) >> ../logs/final.log 2>&1 \\
  || echo "final stages FAILED (exit \$?)" >> ../logs/final.log
echo "drive.sh finished \$(date)" >> ../logs/final.log
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
  if pgrep -f "$DIR/drive.sh" >/dev/null || pgrep -f "goal-learn-3103/pool_run.py" >/dev/null; then
    idle=0
  else
    idle=\$((idle + 1))
  fi
  if [ \$idle -ge 2 ]; then
    aws s3 sync --only-show-errors $DIR/logs $S3/logs
    if aws s3 sync --only-show-errors --exclude "*.tmp" $DIR/out $S3/ckpt; then
      sudo shutdown -h now "3.103: no trainer left"
    fi
    # The final sync keeps failing: stop paying after 15 minutes of it.
    stuck=\$((stuck + 1))
    [ \$stuck -ge 15 ] && sudo shutdown -h now "3.103: final sync failing"
  fi
done
EOF
nohup setsid bash "$DIR/drive.sh" >/dev/null 2>&1 < /dev/null &
pgrep -f "$DIR/watch.sh" >/dev/null || nohup setsid bash "$DIR/watch.sh" >/dev/null 2>&1 < /dev/null &
sleep 15
pgrep -af "drive.sh|watch.sh|pool_run.py|oracle.py" | head -8
echo "started on $(nproc) cores; dead-man at ${MAX_HOURS} h"
