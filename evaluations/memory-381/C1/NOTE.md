C1 (3.81.C): A1's loaded window with T1-T3, 2026-10-14 19:47-20:01 CDT.
Board worktree ~/vp-381 at 91c10c1 plus these files (sha256, first 12):
9ca2d3be02e2 brain/perceive.py, 9459414c25b2 brain/text_encoder.py,
bf8408d0ad3a tools/jetson/headroom_identity.sh,
41dab32cd0ab service/slam/src/picar_bringup/launch/picar.launch.py.
The committed versions add the second review's fixes and drop
torch.cuda.empty_cache() from YoloE._release (9 MB, under its bar).
tables.md: `memory_account.py table --phase window --torch-log torch.jsonl account.jsonl`.
