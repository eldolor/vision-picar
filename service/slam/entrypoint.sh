#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /underlay/install/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
exec "$@"
