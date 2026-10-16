# 3.81.G G3 (amendment 7: VE resident, 3.80.G's cloud-free search)

Ran 2026-10-16 10:25-10:45 CDT on the Jetson (15 W), from `~/vp-381g3`: a
checkout of `5a3fe53` (`plan/3.80-tiered-identity`, 3.80.G; pushed to the
board as `plan/3.81-g3-base`) with `dev`'s `tools/jetson/headroom_mission.sh`
(`c220c89`, md5 2d6761cf73eb659611f72a7d307bbfeb) copied over it. Image
`vision-picar-ros:plan-3.78` with the checkout's `bridge.py`, `convert.py` and
`picar.launch.py` mounted (`ROS_FROM_TREE=1`).

    TIER_SEARCH=explore POLICY=tiered VLM=service CLOUD=off TEXT_CACHE=empty ROS_FROM_TREE=1 \
      ENGINES=$HOME/bench-378/ws91/q25VE/engines IMAGE=vision-picar-ros:plan-3.78 NAME=picar-ros-381 \
      bash tools/jetson/headroom_mission.sh ~/out-381/G3

Files as in G1: `run.log`, `summary.json`, `mission_end.json`, `status.log`,
`mem.log`, `wheel.log`, `local_vlm.log`.
