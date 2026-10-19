#!/bin/bash
# 3.80.R: one Mac row (amendment 1): identity (arm $3, native $4) then
# steering, both against native_server.py on this machine. $1 name, $2 spec,
# NSRV_PY the venv's python.
set -u
NAME=$1; SPEC=$2; ARM=$3; NAT=${4:-}
cd "$(dirname "$0")/../../.."
R=evaluations/tiered-380/R
N=(); [ "$ARM" = N ] && N=(--native "$NAT")
.venv/bin/python $R/identity.py --arm $ARM ${N[@]+"${N[@]}"} --server $R/nsrv.sh --model "$SPEC" --mmproj none \
  --recordings /Users/anshugaind/vision-picar/recordings --port 9599 \
  --walk blue-bottle-20260907-142454 --walk blue-bottle-20260907-185007 \
  --walk blue-shoes-20260907-152528 --walk red-backpack-20260907-144856 \
  --stop-at-decision --out $R/mac/$NAME-$ARM.json > $R/mac/$NAME-$ARM.log 2>&1
rc=$?
echo "$(date +%T) $NAME identity $ARM exit $rc" >> $R/mac/done.log
$R/nsrv.sh -m "$SPEC" --port 9598 > $R/mac/$NAME-server.log 2>&1 &
ok=; for i in $(seq 300); do curl -s localhost:9598/health | grep -q ok && ok=1 && break; sleep 2; done
[ -n "$ok" ] || { echo "$(date +%T) $NAME server never healthy" >> $R/mac/done.log; pkill -f "native_server.py.*9598"; exit 1; }
.venv/bin/python $R/steer.py mac/$NAME http://127.0.0.1:9598 ${NAT:+--native $NAT} > $R/mac/$NAME.run.log 2>&1
rc=$?
echo "$(date +%T) $NAME steer exit $rc" >> $R/mac/done.log
pkill -f "native_server.py.*9598"
