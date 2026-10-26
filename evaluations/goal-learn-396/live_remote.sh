#!/usr/bin/env bash
# evaluations/goal-learn-396/live_remote.sh -- runs ON the spot instance (sent
# by live_cloud.sh through `tools/gpu/ec2.sh run`). Setup inside the SSM
# command: Docker, the venv, the code. Then two detached scripts:
#
# * drive.sh -- builds this branch's ROS image, then live.py's six missions
#   (R N R N R N) with the container on the host's network.
# * watch.sh -- every minute syncs the logs and results to S3; once drive.sh
#   has ended for two checks it syncs once more and powers off. The instance
#   was launched with shutdown = terminate, and a dead-man timer powers it
#   off at MAX_HOURS whatever happens.
#
#   bash live_remote.sh s3://BUCKET/PREFIX [MAX_HOURS]
set -euo pipefail
export PATH="$PATH:/snap/bin"
S3="$1"
MAX_HOURS="${2:-4}"
DIR=/opt/live
if pgrep -f "$DIR/drive.sh|$DIR/watch.sh" >/dev/null; then
  echo "a batch is going or finishing here -- not starting another"
  pgrep -af "drive.sh|watch.sh|live.py|demo_explore" | head
  exit 0
fi
# The dead-man switch, armed only for a batch that actually starts.
sudo shutdown -h "+$((MAX_HOURS * 60))" "3.96 live dead-man switch" \
  || { echo "could not arm the dead-man switch -- not starting" >&2; exit 1; }
sudo DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=600 update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=600 install -y -qq \
  docker.io python3-venv lsof git >/dev/null
sudo systemctl start docker
sudo mkdir -p "$DIR" && sudo chown -R ubuntu:ubuntu "$DIR"
cd "$DIR"
aws s3 cp --only-show-errors "$S3/code.tar.gz" code.tar.gz
rm -rf repo && mkdir repo && tar -xzf code.tar.gz -C repo
# venv/.ok only after every install succeeded: a half-made venv is rebuilt.
if [ ! -f venv/.ok ]; then
  rm -rf venv
  python3 -m venv venv
  ./venv/bin/pip install -q --upgrade pip
  ./venv/bin/pip install -q -r repo/requirements.txt
  touch venv/.ok
fi
mkdir -p out logs runs

cat > drive.sh <<EOF
#!/usr/bin/env bash
# Run as root: docker without a group change. One batch per instance.
export PATH="\$PATH:/snap/bin"
cd $DIR/repo
{ echo "build start \$(date -u +%FT%TZ) on \$(nproc) cores"
  docker build -t vision-picar-ros:plan-3.96 service/slam && echo "build ok \$(date -u +%FT%TZ)"; } \\
  >> ../logs/build.log 2>&1 || { echo "build FAILED" >> ../logs/drive.log; touch ../out/DONE; exit 1; }
PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.96 EXPLORE_LOGDIR=$DIR/runs \\
  ../venv/bin/python evaluations/goal-learn-396/live.py RNRNRN >> ../logs/drive.log 2>&1
echo "live.py exit \$? \$(date -u +%FT%TZ)" >> ../logs/drive.log
cp evaluations/goal-learn-396/live/runs.jsonl ../out/runs.jsonl 2>/dev/null || true
touch ../out/DONE
EOF

cat > watch.sh <<EOF
#!/usr/bin/env bash
export PATH="\$PATH:/snap/bin"
idle=0
stuck=0
while sleep 60; do
  cp $DIR/repo/evaluations/goal-learn-396/live/runs.jsonl $DIR/out/runs.jsonl 2>/dev/null || true
  aws s3 sync --only-show-errors $DIR/logs $S3/logs
  aws s3 sync --only-show-errors $DIR/runs $S3/out/runs
  [ -f $DIR/out/runs.jsonl ] && aws s3 cp --only-show-errors $DIR/out/runs.jsonl $S3/out/runs.jsonl
  if pgrep -f "$DIR/drive.sh" >/dev/null; then idle=0; else idle=\$((idle + 1)); fi
  if [ \$idle -ge 2 ]; then
    aws s3 sync --only-show-errors $DIR/logs $S3/logs
    aws s3 sync --only-show-errors $DIR/runs $S3/out/runs
    # DONE goes up last and alone, so a pull that sees it has everything.
    if aws s3 sync --only-show-errors --exclude DONE $DIR/out $S3/out \
        && { [ ! -f $DIR/out/DONE ] || aws s3 cp --only-show-errors $DIR/out/DONE $S3/out/DONE; }; then
      sudo shutdown -h now "3.96 live: batch over"
    fi
    # The final sync keeps failing: stop paying after 15 minutes of it.
    stuck=\$((stuck + 1))
    [ \$stuck -ge 15 ] && sudo shutdown -h now "3.96 live: final sync failing"
  fi
done
EOF
sudo nohup setsid bash "$DIR/drive.sh" >/dev/null 2>&1 < /dev/null &
nohup setsid bash "$DIR/watch.sh" >/dev/null 2>&1 < /dev/null &
sleep 10
pgrep -af "drive.sh|watch.sh|docker build" | head -5
echo "started on $(nproc) cores; dead-man at ${MAX_HOURS} h"
