# 3.81.G G1 (VE resident)

Ran 2026-10-16 on the Jetson (15 W) from `~/vp-381g` at `cb9d3b0`
(`dev` 6a0d612 plus amendment 6's harness), image
`vision-picar-ros:plan-3.78` with `dev`'s `bridge.py`, `convert.py` and
`picar.launch.py` mounted over it (`ROS_FROM_TREE=1`).

    POLICY=tiered VLM=service CLOUD=off TEXT_CACHE=empty ROS_FROM_TREE=1 \
      ENGINES=$HOME/bench-378/ws91/q25VE/engines IMAGE=vision-picar-ros:plan-3.78 NAME=picar-ros-381 \
      bash tools/jetson/headroom_mission.sh ~/out-381/G1

Files: the harness's `run.log`, `summary.json`, `mission_end.json`,
`status.log` (every 2 s), `mem.log` (MemAvailable, 1 s), `wheel.log`, `local_vlm.log`.
