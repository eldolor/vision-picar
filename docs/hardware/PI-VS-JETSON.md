# Pi 5 vs Jetson for vision-picar

As of **2026-09-30**. A walkthrough of the two compute paths, drawn from the
repo's own records (sources at the end). Shareable copy:
https://claude.ai/code/artifact/3e73c6a1-a25d-46c8-ac7e-8f04b66133c5

> The decision is **made** (Jetson, 2026-09-19, `CLAUDE.md` section 3b). This
> file answers "what if we went the Pi way instead" and does not re-open it.
>
> **Since 2026-10-05 the Jetson is opened, measured and KEPT**
> (`PLAN-ros-alignment.md` 3.33): torch on the GPU, the shipped perception
> at a median 60.6 ms / p90 109.9 ms a frame at 15 W. The timing advice and
> the projected latencies below are as of 2026-09-30; dated notes mark what
> the board has since answered.

**Recommendation: stay with the Jetson Orin Nano Super.** A Raspberry Pi 5 +
Hailo-8L saves roughly $86-250. What it costs is the detector the project
actually ships. One ~$1 test could change that (see "What would change the
answer").

---

## Price

On a like-for-like build the Jetson costs **$86 more**. Prices were read from
retailer pages on 2026-09-17 and are all-in: tax at 8.75% plus shipping
(`BOM-COMPARISON.md` section 1).

| Build | Pi 5 + Hailo-8L | Jetson Orin Nano Super | Jetson extra |
|---|---|---|---|
| Essential | ~$781 | ~$867 | +$86 |
| Recommended | ~$858 | ~$944 | +$86 |
| With 256 GB NVMe | not possible | ~$1,019 | -- |

| Compute line | Pi 5 + Hailo-8L | Jetson |
|---|---|---|
| Board | $175 (Pi 5 8GB; was $80 before three memory-price rises) | $399 (dev kit; cooler and Wi-Fi included) |
| Accelerator | $76.95 (AI HAT+, soldered) | not needed |
| Cooler | $10.95 | included |
| Camera | $29.25 + $3.95 cable | $19.95 |
| 5V buck converter | $39.95 | not needed (takes 9-20 V) |
| Part B subtotal | $384.04 | $470.94 |

**On the UGV Rover the gap is probably wider.** The Rover's power supply is
good for about 5 A, so a 25 W Jetson needs its own battery (~$100-110,
`JETSON-BOM.md` 9.5). A Pi draws ~10-15 W and might run off the Rover's supply
alone. That would put the compute-only gap at roughly **$200-250 in the Pi's
favour**.

Open question: whether Waveshare sells a Pi version of the Rover kit, and at
what price, has not been checked.

## What the Pi gains

- **Money:** at least $86, and perhaps ~$250 on the Rover.
- **Power:** about half the draw. That fixes the Rover's weakest point, and you
  no longer need the software battery cutoff the Jetson requires. A 3S pack's
  empty voltage (9.9 V) sits only 0.9 V above the Jetson's 9 V floor
  (`BOM-COMPARISON.md` 4.5).
- **Fewer bring-up risks:**
    - there is no open question about torch wheels on JetPack
    - there is no devkit firmware update to get through (one Micro Center
      reviewer bricked a unit doing it)
    - the Pi community is larger
- **Speed on fixed models:** a Hailo-8L runs YOLO11 at ~92 FPS, well above the
  15-30 Hz perception target.

## What the Pi loses

1. **The detector you ship today.** `yoloe-11s-seg` runs in PyTorch. On a
   Hailo it would have to be compiled to Hailo's format and then converted to
   8-bit integer precision (INT8). **Neither step has ever been tried.** The
   last model with the same text-matching head, YOLO-World, compiled fine and
   then fell from 34% to 5% under INT8 (P13). If YOLOE fails the same way, the
   Pi falls back to YOLO11s + CLIP. That scored **45% recall where the
   open-vocabulary models score 72-91%**.
2. **Big models, permanently.** OWLv2, Grounding DINO, DINOv2 and SAM cannot
   run on a Hailo at all (P6/P17). That is a limit of the chip's design, so no
   future compiler flag fixes it. The Pi's own CPU manages only 5-13 FPS.
3. **Quick model experiments.** On the Jetson you `pip install` a Hugging Face
   model and run it. On the Pi, every new model means a compile on a rented
   x86 machine ($1-5 each, `tools/hailo/`), and it may still fail.
4. **An NVMe drive.** The Hailo takes the Pi's only PCIe lane, so the Pi boots
   from an SD card, and SD cards corrupt when the battery dips
   (`BOM-COMPARISON.md` 4.2).
5. **Spare CPU.** Four A76 cores cover the perception preprocessing (~0.6-0.8
   of a core, C6) plus ROS/nav2. nav2's cost is the biggest unknown, at
   0.5-1.5 cores. The Jetson has six cores plus a GPU.
6. **Being able to buy the specified part.** The plan called for the removable
   Hailo M.2 module, which is sold out everywhere. Only the soldered AI HAT+ is
   in stock (`BOM-COMPARISON.md` 4.1).

## Side by side

| | Pi 5 + Hailo-8L | Jetson Orin Nano Super |
|---|---|---|
| Perception, measured | ~45-50% (fallback), or unknown if YOLOE compiles and survives INT8 | 82-91%, the shipped setup |
| New models | compile each one, may fail | pip install |
| Power on the Rover | ~10-15 W, probably fits the kit's supply | up to 25 W, needs a second battery |
| Storage | SD card only | NVMe available |
| ROS container | should run as-is (arm64 Docker) | what the software already assumes |
| Availability | only the soldered AI HAT+ in stock | in hand; opened 2026-10-02, kept 2026-10-05 |

## What would change the answer

**Compile YOLOE for the Hailo-8L, then convert it to INT8.** It is one run of
the existing loop in `tools/hailo/` and costs about $1
(`PLAN-onboard-perception.md` P21, "What to do next" item 2).

- **If YOLOE survives INT8**, the best model measured runs on a $77 part, and
  the Pi becomes a real contender.
- **If it collapses the way YOLO-World did**, the Pi's only advantage is
  $86-250.

The Hailo path was closed on 2026-09-19, so this test runs only on an explicit
decision to re-open it.

**Timing:** the Jetson arrived 2026-09-30 and is still unopened, so it can be
returned until about Oct 30. The Rover (Jetson kit) was ordered the evening
of 2026-09-30. Run the test before the Jetson box is opened.
*(Moot 2026-10-05: the box was opened 2026-10-02 to settle its risks while
returnable, and the user kept the board on 2026-10-05.)*

## Jetson power modes

**Run the Jetson at 15 W by default.** It keeps every project goal, costs some
perception speed, and may remove the separate ~$100-110 battery on the Rover.

The Orin Nano Super has four modes: **7 W, 15 W, 25 W and MAXN SUPER**
(uncapped, can exceed 25 W). You switch with `sudo nvpmodel -m <id>`
(`GUIDE-robot-base.md`). The clock figures below are recalled rather than
checked on a board. Confirm them against `/etc/nvpmodel.conf` once the box is
opened. *(Confirmed 2026-10-04 from the board's `/etc/nvpmodel.conf`: 15 W
caps the CPU at 1.498 GHz, 25 W at **1.344 GHz -- lower than 15 W**, and MAXN
SUPER is uncapped, ~1.73 GHz. `PLAN-ros-alignment.md` 3.33.)*

| Mode | CPU | GPU | Memory bandwidth | AI rating |
|---|---|---|---|---|
| MAXN SUPER | 6 cores, ~1.7 GHz | ~1.0 GHz | ~102 GB/s | ~67 TOPS |
| 25 W | 6 cores, a bit slower | ~0.9 GHz | ~102 GB/s | a little under 67 TOPS |
| 15 W | 6 cores, ~1.5 GHz | ~0.6 GHz | ~68 GB/s | ~40 TOPS |
| 7 W | 4 cores, ~1 GHz | ~0.4 GHz | ~68 GB/s | ~20 TOPS |

A lower mode does not break anything. It slows things down, and how much
depends on the job. The slowdown figures are estimates from clock scaling, not
measurements.

| Job | What limits it | 15 W | 7 W |
|---|---|---|---|
| Safety: lidar veto, 20 Hz wheel loop, watchdog | tiny CPU load | no change | probably fine, but it now competes for 4 slow cores |
| Seeing the target: YOLOE + CLIP | mostly CPU (image resizing), partly GPU | ~1.3-1.5x slower (about 3-4 frames/s) | ~2-2.5x slower (about 2 frames/s) |
| Mapping and goals: slam_toolbox + nav2 | CPU, 0.5-1.5 cores (never measured on the board) | fine (2026-10-05: 29/29 nav2 goals at 15 W with perception running, 3.33) | at risk: nav2's control loop may miss deadlines |
| Thinking: the Opus call on triggers | the cloud (~3.6 s per call) | no change | no change |

The baseline for "seeing the target" is the best current Orin guess of ~205 ms
per frame (`PLAN-onboard-perception.md` P7b/P7d). That was projected for
OWLv2 and not yet remeasured for YOLOE, and most of it is CPU image resizing.
*(Measured 2026-10-04 on the board, the shipped YOLOE + CLIP pipeline: median
60.6 ms, p90 109.9 ms a frame at 15 W -- GPU 44.4 ms, CPU handling 18.6 ms --
and 64.2 / 120.4 ms at 25 W, so 15 W costs nothing here. The table's
slowdown estimates and the resizing cost are superseded by that;
`PLAN-ros-alignment.md` 3.33.)*

**What this means for the project:**

- **Missions take a bit longer; outcomes should not change at 15 W.** The robot
  moves one verb at a time (a 30 cm step or a turn) and needs only a few frames
  per decision. Arrival (`brain/arrival.py`) needs two good frames in a row, so
  it comes a fraction of a second later. At 7 W, the continuous driving the plan
  eventually wants (1.14 in the perception plan) is out of reach.
- **Safety does not depend on the mode.** The lidar veto (`robot/safety.py`)
  stops the robot, not the camera, so a slower camera makes the robot slower to
  find things, not slower to stop. The one risk is 7 W: the 20 Hz wheel loop
  could run late, and `/health`'s `wheel_loop` readout counts late ticks.
- **Power is where it matters most.** At 15 W the Jetson draws ~1.4 A from an
  11 V pack, not ~2.3 A. With motors, lidar, camera and servos, the total is
  roughly 40 W (~3.6 A), under the ~5 A Waveshare quoted for the Rover's supply
  (`JETSON-BOM.md` 9.3). If that holds, most of the Pi's power advantage goes
  away. Waveshare called only 25 W "untested", so 15 W is unverified too, but
  the margin is much better.
- **The 9 V floor does not change.** A dying 3S pack can still brown out the
  board. Lower current means less voltage sag, which helps but does not fix it,
  so the software battery cutoff is still needed.
- **Battery life goes up.** An older estimate (`BOM.md`) put one 3S pack at
  ~35-50 min in 15 W mode.

Step up to 25 W or MAXN SUPER only when a measurement shows perception is the
bottleneck. Fix the image resizing first (P7b); it is the biggest cost at every
power level. *(2026-10-04: on the board it is not -- CPU handling is ~19 ms of
a 61 ms frame, and 25 W bought nothing.)* Treat 7 W as a battery-saver, not an operating mode for
autonomy.

**Before buying the separate Jetson battery** (`JETSON-BOM.md` 9.5, still to
buy): when the Rover and Jetson arrive, run the Jetson at 15 W off the Rover's
own supply under a mission load, logging input voltage. If it holds, the
battery may not be needed. If it sags, buy the battery as planned.

## Sources

- `BOM-COMPARISON.md` sections 1-4: prices, availability, the NVMe constraint
- `JETSON-BOM.md` sections 9.3 and 9.5: the Rover's power supply and the
  separate Jetson battery
- `PLAN-onboard-perception.md`:
    - 4.9: Pi CPU and Hailo rates
    - C6: host CPU budget
    - P13: YOLO-World at INT8
    - P6/P17: OWLv2 compile failures
    - P21-P24: YOLOE results and the untried compile
