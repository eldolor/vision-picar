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

`setup.sh` runs the pipeline once on a blank frame and asserts that BOTH
networks ran on `cuda`. It should end with
`detector crops:yoloe-11s-seg.pt on cuda | CLIP on cuda`. The detector picks
its device on its first run, which is why the script runs it.

`bench_frames.json` pins the 60 (+3 warm-up) frames, so the laptop and the
board time the same frames. The budget is **250 ms a frame at 15 W**. The
laptop's numbers to compare against (M1 MacBook Air, 7-core GPU,
2026-10-02):

| devices | total (median / p90) | detector inference | handling | CLIP a crop |
|---|---|---|---|---|
| detector CPU, CLIP MPS (the defaults) | 119 / 156 ms | 108 ms | 6 ms | 31 ms |
| both on MPS (`--device mps`) | **36 / 66 ms** | **22 ms** | 14 ms | 26 ms |

Ultralytics does not choose the Mac's GPU by itself; `brain/perceive.py`'s
CLIP does.

**The first run downloads ~600 MB of weights** into the directory it runs
from: `yoloe-11s-seg.pt` (28 MB) and YOLOE's text encoder
`mobileclip_blt.ts` (572 MB), plus CLIP's from Hugging Face. Both files are
gitignored.

The suite: `pytest tests/ -q`. Its browser tests need
`python -m playwright install chromium` first -- installed without the
browser, they ERROR rather than skip (seen in the Python 3.10 run).

## 4. G4 and headroom

**G4 is 5 consecutive runs of the live chain and nav suites against the
fake motor board, and a skip is not a pass.** A run counts only if pytest's
summary reads `18 passed` (13 chain, 5 nav) with 0 skipped and 0 failed. Any
skip or failure: find why, and start the count again.

Each requirement below is there because leaving it out forces a skip or a
failure:

| Requirement | Without it |
|---|---|
| A brain on :8001 under `ROUTE_PREFIX=/brain` (`run.sh` starts one) | the D-pad preemption test skips "no brain server" |
| `SIM_MAP=scaled_house` for the robot server **and** in pytest's own environment | the suites gate on `/health`'s `sim_map`; the nav suite also reads pytest's `SIM_MAP` |
| The container started only after `GET /wheels` reports `usable: true` | the fake board answers `usable: false` until its first frame, and a container that activates then loses its wheel plugin for good (`service/slam/README.md` section 3) |
| The container named `picar-ros`, and `docker` usable without sudo | the kill test runs `docker kill picar-ros` and FAILS (`PICAR_ROS_CONTAINER` overrides the name) |
| The secret in pytest's environment | the chain suite skips "the robot server wants a secret" |
| A fresh server and container for every run | the nav suite's mapping lap starts from the house's start pose, and SLAM keeps its map for the container's lifetime |

```bash
cd ~/vision-picar
docker build -t vision-picar-ros service/slam                  # once; long the first time
set -a; source ~/.vision-picar-local-secrets; set +a           # LOCAL_SECRET, and VISION_SECRET (run.sh needs it defined)
export APP_SHARED_SECRET="$LOCAL_SECRET"

# one run -- repeat this block five times
docker rm -f picar-ros 2>/dev/null
SIM_MAP=scaled_house ROBOT_DRIVE=ros WORLD_MODE=ros ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake \
  bash service/tunnel/restart.sh                               # robot :8000, brain :8001/brain, both on 127.0.0.1
curl -s localhost:8000/health | grep -o '"sim_map": *"[a-z_]*"'   # expect "sim_map": "scaled_house"
until curl -s -H "x-app-secret: $LOCAL_SECRET" localhost:8000/wheels | grep -q '"usable": *true'; do sleep 0.5; done
docker run -d --name picar-ros --restart unless-stopped --network host \
  -e APP_SHARED_SECRET -e ROBOT_URL=http://127.0.0.1:8000 -e BRAIN_URL=http://127.0.0.1:8001/brain \
  vision-picar-ros ros2 launch picar_bringup picar.launch.py
until curl -s localhost:8090/health >/dev/null; do sleep 1; done
SIM_MAP=scaled_house .venv/bin/python -m pytest tests/test_ros_chain_live.py tests/test_nav_live.py -rs
```

`restart.sh` prints `OK: robot and brain both running <rev>` only once both
answer with the checkout's revision. The container line is
`service/slam/README.md`'s Linux form: host networking, because `run.sh`
binds the servers to 127.0.0.1. **It has not yet run on a board**, so the
first run here is also its first test. The authority for this list is
`docs/engineering/platform/ENGINEERING.md`; if the two disagree, fix both.

**Headroom** (3.33 step 7): a 10-minute nav2 run with the perception tier
processing frames. It passes when all three hold:

* 0 late ticks on the wheel loop (`/health`'s `wheel_loop`);
* at least 1 GB of memory free;
* no thermal throttling in `tegrastats`.
