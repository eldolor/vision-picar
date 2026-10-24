#!/usr/bin/env bash
# evaluations/tier-fusion-391/remote.sh -- runs ON the spot instance (sent by
# cloud.sh through `tools/gpu/ec2.sh run`). Fetches HEAD's code, makes the
# venv (requirements.txt only: the sweeps load no model), then starts two
# detached scripts and returns:
#
# * drive.sh -- criterion 8's seven arms side by side, one core each
#   (3.80.G's 21 houses at scores 0.9 and 0.0; 3.80.O's blind, decoy and
#   sighted arms over the criterion houses).
# * watch.sh -- every minute, syncs logs and results to S3; once no arm has
#   run for two checks it syncs one last time and powers off (shutdown =
#   terminate). A dead-man timer powers it off at MAX_HOURS whatever happens.
#
#   bash remote.sh s3://BUCKET/PREFIX [MAX_HOURS]
set -euo pipefail
export PATH="$PATH:/snap/bin"
S3="$1"
MAX_HOURS="${2:-2}"
DIR=/opt/sweep
if pgrep -f "$DIR/drive.sh|$DIR/watch.sh|sweep_inprocess.py" >/dev/null; then
  echo "a run is going or finishing here -- not starting another"
  exit 0
fi
sudo shutdown -h "+$((MAX_HOURS * 60))" "3.91 dead-man switch" \
  || { echo "could not arm the dead-man switch -- not starting" >&2; exit 1; }
sudo mkdir -p "$DIR" && sudo chown -R ubuntu:ubuntu "$DIR"
cd "$DIR"
aws s3 cp --only-show-errors "$S3/code.tar.gz" code.tar.gz
rm -rf repo && mkdir repo && tar -xzf code.tar.gz -C repo
if [ ! -f venv/.ok ]; then
  rm -rf venv
  dpkg -s python3-venv >/dev/null 2>&1 || {
    sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-venv
  }
  python3 -m venv venv
  ./venv/bin/pip install -q --upgrade pip
  ./venv/bin/pip install -q -r repo/requirements.txt
  touch venv/.ok
fi
mkdir -p out logs

cat > drive.sh <<EOF
#!/usr/bin/env bash
# Not -e: one arm's failure must not stop the others.
cd $DIR/repo
mkdir -p evaluations/tier-fusion-391/C8
export PYTHONPATH=. OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=$DIR/venv/bin/python
for s in 0.9 0.0; do
  ( \$PY evaluations/tiered-380/G/sweep_inprocess.py \$s > $DIR/logs/G-\$s.log 2>&1 \\
      && cp evaluations/tiered-380/G/inprocess-\$s.json $DIR/out/G-\$s.json ) &
done
run_o() {
  name=\$1; shift
  \$PY evaluations/tiered-380/O/sweep_inprocess.py --name ../../tier-fusion-391/C8/\$name \\
    --houses criterion "\$@" > $DIR/logs/\$name.log 2>&1 \\
    && cp evaluations/tier-fusion-391/C8/\$name.json $DIR/out/\$name.json
}
run_o O1-blind-1 --cold-every 1 &
run_o O1-blind-6 --cold-every 6 &
run_o O2-decoy-1 --decoy 0.05 --cold-every 1 &
run_o O2-decoy-6 --decoy 0.05 --cold-every 6 &
run_o sighted-6 --sighted --cold-every 6 &
wait
echo "drive.sh finished \$(date)" >> $DIR/logs/drive.log
EOF

cat > watch.sh <<EOF
#!/usr/bin/env bash
export PATH="\$PATH:/snap/bin"
idle=0
stuck=0
while sleep 60; do
  aws s3 sync --only-show-errors $DIR/logs $S3/logs
  aws s3 sync --only-show-errors $DIR/out $S3/out
  if pgrep -f "$DIR/drive.sh" >/dev/null || pgrep -f "sweep_inprocess.py" >/dev/null; then
    idle=0
  else
    idle=\$((idle + 1))
  fi
  if [ \$idle -ge 2 ]; then
    aws s3 sync --only-show-errors $DIR/logs $S3/logs
    if aws s3 sync --only-show-errors $DIR/out $S3/out; then
      sudo shutdown -h now "3.91: no arm left"
    fi
    stuck=\$((stuck + 1))
    [ \$stuck -ge 15 ] && sudo shutdown -h now "3.91: final sync failing"
  fi
done
EOF
nohup setsid bash "$DIR/drive.sh" >/dev/null 2>&1 < /dev/null &
pgrep -f "$DIR/watch.sh" >/dev/null || nohup setsid bash "$DIR/watch.sh" >/dev/null 2>&1 < /dev/null &
sleep 15
pgrep -af "drive.sh|watch.sh|sweep_inprocess" | head -10
echo "started on $(nproc) cores; dead-man at ${MAX_HOURS} h"
