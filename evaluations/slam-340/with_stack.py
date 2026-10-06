"""
ROS_IMAGE=<tag> python evaluations/slam-340/with_stack.py HOUSE DRIFT -- cmd ...

A fresh stack (tests/demo_explore.stack, ports 8100/8101/8190) in HOUSE with
SIM_ODOM_DRIFT=DRIFT ("" for none), then `cmd` with PICAR_ROBOT_URL /
PICAR_BRIDGE_URL / SIM_MAP pointing at it, then the stack is torn down. So
R5's lap and R6's goals (which expect a running stack) can be judged on a
candidate image without touching anyone's default ports.
"""

import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tests import demo_explore as dx  # noqa: E402


def main():
    house, drift = sys.argv[1], sys.argv[2]
    cmd = sys.argv[sys.argv.index("--") + 1:]
    dx.stack(house, odom_drift=drift)
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(PICAR_ROBOT_URL=f"http://127.0.0.1:{dx.ROBOT}",
               PICAR_BRIDGE_URL=f"http://127.0.0.1:{dx.BRIDGE}", SIM_MAP=house)
    try:
        rc = subprocess.run(cmd, env=env, cwd=dx.ROOT).returncode
    finally:
        dx._kill_ports()
    sys.exit(rc)


if __name__ == "__main__":
    main()
