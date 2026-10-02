# Bringing up the Jetson

`PLAN-ros-alignment.md` 3.33 is the plan and its criteria; this is the
procedure. Every step is a stop point: a failure is recorded and decided
on, not worked around.

## 0. Before the first boot (on the Mac)

* **The image:** `jetson-orin-nano-devkit-super-SD-image_JP6.2.1.zip`
  (11,725,610,175 bytes, from NVIDIA's JetPack 6.2.1 page). Write it with
  **balenaEtcher** (Flash from file -> the zip, unzipped not needed) to a
  128 GB A2 microSD, and let Etcher's verify finish -- NVIDIA publishes no
  checksum, so the verify is the check.
* **SSH key:** `~/.ssh/id_ed25519_jetson`, and `~/.ssh/config` already has
  `Host picar-jetson` -> `picar@picar-jetson.local`.

## 1. First boot (screen, keyboard, mouse, Ethernet, stock 19 V adapter)

1. **Firmware:** press Esc repeatedly at the NVIDIA logo and read the UEFI
   version. **36.0 or newer:** go on. **Older:** stop -- NVIDIA's JetPack 6
   update path first, on the stock adapter (`HARDWARE-BOM.md` 5.1).
2. Ubuntu's first-boot setup: user **`picar`**, hostname **`picar-jetson`**
   (the Mac's SSH config expects both).
3. From the Mac: `ssh-copy-id -i ~/.ssh/id_ed25519_jetson.pub picar-jetson`,
   then `ssh picar-jetson` must log in without a password.
4. Optional, after everything below works: `sudo apt update && sudo apt
   upgrade` takes it to JetPack 6.2.2 (Jetson Linux 36.5, still Ubuntu
   22.04 -- a maintenance release).

## 2. The code (no GitHub credentials on the robot)

The repo is private, so the Jetson does not clone from GitHub. The Mac
pushes to it over SSH:

```bash
# on the Jetson, once
git init vision-picar && cd vision-picar && git config receive.denyCurrentBranch updateInstead
# on the Mac, once
git remote add jetson picar-jetson:vision-picar
# every time
git push jetson dev && ssh picar-jetson 'cd vision-picar && git checkout -f dev'
```

The recorded walks are not in git: `rsync -a recordings/ picar-jetson:vision-picar/recordings/`
(~450 MB).

## 3. Set up, and the two risks

```bash
ssh picar-jetson
cd vision-picar && bash tools/jetson/setup.sh     # torch on cuda -- risk 1
python -m tools.jetson.bench_perception --recordings recordings --out bench-15w.json   # risk 2
```

`bench_frames.json` pins the 60 (+3 warm-up) frames, so the laptop and the
board time the same frames. The budget is **250 ms a frame at 15 W**. The
laptop's numbers to compare against (M1 MacBook Air, 2026-10-02): median
119 ms, detector 108 ms, handling 6 ms, CLIP 31 ms a crop.

**The first run downloads ~600 MB of weights** into the directory it runs
from: `yoloe-11s-seg.pt` (28 MB) and YOLOE's text encoder
`mobileclip_blt.ts` (572 MB), plus CLIP's from Hugging Face. Both files are
gitignored.

The suite: `pytest tests/ -q`. Its browser tests need
`python -m playwright install chromium` first -- installed without the
browser, they ERROR rather than skip (seen in the Python 3.10 run).

## 4. G4 and headroom

The ROS image built natively (`docker build -t vision-picar-ros service/slam`),
then 3.33 steps 6-7: the live chain and nav suites 5 times in a row against
`SIM_MOTOR_BOARD=fake`, and the whole stack at once with the wheel loop's
late ticks, memory and `tegrastats` watched.
