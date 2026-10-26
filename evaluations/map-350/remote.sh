#!/usr/bin/env bash
# evaluations/map-350/remote.sh -- runs ON the spot instance, as root (sent
# by cloud.sh through `tools/gpu/ec2.sh run`). Setup only, inside the SSM
# command: Docker, the code, the venv once, and what is already done pulled
# back from S3 (a relaunch after a spot interruption runs only the rest).
# Then it starts two detached scripts and returns:
#
# * drive.sh -- builds this branch's ROS image (its own tag, never :latest),
#   then each D4 run in turn (one live stack at a time), copying each
#   result to out/ as it lands.
# * watch.sh -- every minute, syncs out/ and the text logs to S3 (never a
#   run directory's maps); once drive.sh has gone for two checks in a row it
#   syncs one last time and powers off. The instance was launched with
#   shutdown = terminate, and a dead-man timer powers it off at MAX_HOURS
#   whatever happens.
#
#   bash remote.sh s3://BUCKET/PREFIX [MAX_HOURS]
#
# The run list comes from cloud.sh (S3 `runs`), so the two never drift.
set -euo pipefail
export PATH="$PATH:/snap/bin"
S3="$1"
MAX_HOURS="${2:-6}"
DIR=/opt/d4
if pgrep -f "map-350/d4_runs.py|$DIR/drive.sh|$DIR/watch.sh" >/dev/null; then
  # A watcher alone means a run is finishing (it may power off any minute).
  echo "a run is going or finishing here -- not starting another; resume after it ends"
  pgrep -af "d4_runs.py|drive.sh|watch.sh" | head
  exit 0
fi
# The dead-man switch, armed only for a run that actually starts.
shutdown -h "+$((MAX_HOURS * 60))" "3.50.D4 dead-man switch" \
  || { echo "could not arm the dead-man switch -- not starting" >&2; exit 1; }
# Until watch.sh is up, nothing else powers off early: a failed setup step
# (apt, pip, a sync) must not leave the instance idle until the dead-man.
trap 'echo "setup failed -- powering off" >&2; shutdown -h +1 "3.50.D4: setup failed"' EXIT
mkdir -p "$DIR" && cd "$DIR"
# git: the heavy-run lock asks it where .git is (none here: the lock goes to /tmp).
if ! command -v docker >/dev/null || ! command -v git >/dev/null || ! dpkg -s python3-venv >/dev/null 2>&1; then
  DEBIAN_FRONTEND=noninteractive apt-get update -qq
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq docker.io git python3-venv
fi
systemctl start docker
aws s3 cp --only-show-errors "$S3/code.tar.gz" code.tar.gz
RUNS="$(aws s3 cp --only-show-errors "$S3/runs" -)"
[ -n "$RUNS" ] || { echo "no run list in $S3/runs" >&2; exit 1; }
rm -rf repo && mkdir repo && tar -xzf code.tar.gz -C repo
# venv/.ok is written only after every install succeeded: a half-made venv
# (a pip blip) is rebuilt, never reused.
if [ ! -f venv/.ok ]; then
  rm -rf venv
  python3 -m venv venv
  ./venv/bin/pip install -q --upgrade pip
  ./venv/bin/pip install -q -r repo/requirements.txt
  touch venv/.ok
fi
# The harness's contact check (tests/chassis_fit.py) needs scipy, which only
# requirements-perception.txt names (3.50.D4's first cloud run). Outside the
# .ok guard, so a venv made before this line gets it too (review).
./venv/bin/python -c "import scipy" 2>/dev/null \
  || ./venv/bin/pip install -q "$(grep '^scipy' repo/requirements-perception.txt)"
mkdir -p out logs logdir
# What is done: a failed sync must stop here, or a finished run would run again.
aws s3 sync --only-show-errors "$S3/out/" out/
aws s3 sync --only-show-errors "$S3/logs/" logs/

cat > drive.sh <<EOF
#!/usr/bin/env bash
# Not -e: one run's failure must not stop the others (d4_runs.py records an
# error as that run's result).
export PATH="\$PATH:/snap/bin"
cd $DIR/repo
# The section's env (tools/plan_section.py env; the instance has no .git).
export PICAR_ROBOT_PORT=9500 PICAR_BRAIN_PORT=9501 PICAR_BODY_PORT=9502 PICAR_SENSOR_PORT=9503
export PICAR_SENSOR2_PORT=9504 PICAR_BRIDGE_PORT=9505 ROS_DOMAIN_ID=50
export PICAR_ROS_CONTAINER=picar-ros-350 ROS_IMAGE=vision-picar-ros:plan-3.50
export EXPLORE_LOGDIR=$DIR/logdir PICAR_ROBOT_URL=http://127.0.0.1:9500
export PICAR_BRAIN_URL=http://127.0.0.1:9501 PICAR_BRIDGE_URL=http://127.0.0.1:9505
echo "drive.sh started \$(date) at \$(aws s3 cp --only-show-errors $S3/HEAD - 2>/dev/null)" >> ../logs/drive.log
if ! docker image inspect "\$ROS_IMAGE" >/dev/null 2>&1; then
  echo "building \$ROS_IMAGE \$(date)" >> ../logs/drive.log
  if ! docker build -t "\$ROS_IMAGE" service/slam >> ../logs/build.log 2>&1; then
    echo "image build FAILED \$(date) -- no run started" >> ../logs/drive.log
    exit 1
  fi
  echo "built \$(date)" >> ../logs/drive.log
fi
for r in $RUNS; do
  [ -f "../out/\$r.json" ] && { echo "\$r already done" >> ../logs/drive.log; continue; }
  # Timing counts in these runs (D4-a's 10 s): wait for quiet first (3.79's
  # rule: a live run at load above ~4 is invalid), up to 15 min, after the
  # build and between runs. Recorded either way.
  for _ in \$(seq 90); do
    awk '{exit !(\$1 < 4)}' /proc/loadavg && break
    sleep 10
  done
  echo "--- \$r \$(date) load \$(cut -d' ' -f1-3 /proc/loadavg)" >> ../logs/drive.log
  # The harness's own D4/ holds HEAD's old results: only a fresh one counts.
  rm -f "evaluations/map-350/D4/\$r.json"
  ../venv/bin/python evaluations/map-350/d4_runs.py "\${r%%-*}" "\${r#*-}" >> "../logs/\$r.log" 2>&1
  echo "    exit \$? \$(date)" >> ../logs/drive.log
  # A run that raised (a stack that never came up) is not a result: kept
  # as evidence in logs/, and run again on a resume.
  f="evaluations/map-350/D4/\$r.json"
  if [ -f "\$f" ] && ../venv/bin/python -c "import json,sys; sys.exit('error' in json.load(open(sys.argv[1])))" "\$f"; then
    cp "\$f" "../out/\$r.json"
  elif [ -f "\$f" ]; then
    cp "\$f" "../logs/\$r.error.json"; echo "    \$r raised -- not counted done" >> ../logs/drive.log
  fi
done
echo "drive.sh finished \$(date)" >> ../logs/drive.log
EOF

cat > watch.sh <<EOF
#!/usr/bin/env bash
export PATH="\$PATH:/snap/bin"
idle=0
stuck=0
push_s3() {
  aws s3 sync --only-show-errors $DIR/out $S3/out || return 1
  aws s3 sync --only-show-errors $DIR/logs $S3/logs || return 1
  # The run directories' text logs only (each holds its house's maps too).
  aws s3 sync --only-show-errors --exclude "*" --include "*.log" $DIR/logdir $S3/logs/rundirs
}
while sleep 60; do
  push_s3
  if pgrep -f "$DIR/drive.sh" >/dev/null; then idle=0; else idle=\$((idle + 1)); fi
  if [ \$idle -ge 2 ]; then
    push_s3 && { shutdown -h now "3.50.D4: drive.sh finished"; exit 0; }
    # The final sync keeps failing: stop paying after 15 minutes of it.
    stuck=\$((stuck + 1))
    [ \$stuck -ge 15 ] && shutdown -h now "3.50.D4: final sync failing"
  fi
done
EOF
nohup setsid bash "$DIR/drive.sh" >/dev/null 2>&1 < /dev/null &
pgrep -f "$DIR/watch.sh" >/dev/null || nohup setsid bash "$DIR/watch.sh" >/dev/null 2>&1 < /dev/null &
sleep 15
trap - EXIT
pgrep -af "drive.sh|watch.sh|docker build" | head -8
echo "started on $(nproc) cores; dead-man at ${MAX_HOURS} h"
