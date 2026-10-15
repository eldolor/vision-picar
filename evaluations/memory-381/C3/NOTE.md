C3 (3.81.E, amendment 3): C2's window with SWITCH=release (the previous
pipeline's TieredVision closed before pipeline_for), 2026-10-15 10:46-10:55
CDT. Board worktree ~/vp-381 at 91c10c1 plus: brain/perceive.py 75a6d1d93f73,
brain/tiered.py d9306b28051d, brain/text_encoder.py f6fc0607d926,
tools/jetson/perception_loop.py ceab7a0314fb, headroom_identity.sh
cc5fb71d1200, the launch file 41dab32cd0ab. The committed perceive.py (9d852bf)
differs only in how frames given up on after a 30 s wait are tracked (by
identity, not a count); no frame hung in this run, so the two behave alike.
