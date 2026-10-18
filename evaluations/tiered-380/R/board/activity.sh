#!/bin/bash
# 3.80.R: every 30 s, who else is on the board (as 3.80.M's activity.log).
while true; do
  echo "$(date +%T) users=$(who | wc -l) ssh=$(pgrep -c sshd) docker=$(docker ps -q 2>/dev/null | wc -l) py=$(pgrep -c -f 'python.*(bench|pilot|demo|train)') llama=$(pgrep -c llama-server) native=$(pgrep -c -f native_server.py) mem=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)" >> ~/out-380R/activity.log
  sleep 30
done
