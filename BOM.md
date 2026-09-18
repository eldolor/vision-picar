# Bill of materials -- the Jetson build (SUPERSEDED)

> **SUPERSEDED 2026-09-17. Do not price anything from this file.**
>
> Its argument was right and arrived early: OWLv2 solves search proposal,
> does not compile to a Hailo, and does not survive INT8, so it needs a
> GPU. P17 and P19 have since confirmed all three by measurement. But its
> **prices are estimates** -- it says so itself below -- and estimates
> mixed with verified figures produced a $220-300 error that stood for
> several hours on 2026-09-17.
>
> Use instead:
>
> | for | read |
> |---|---|
> | what to buy, Jetson build | **`JETSON-BOM.md`** |
> | verified prices, Pi vs Jetson | **`BOM-COMPARISON.md`** |
> | part numbers, vendors, bring-up, wiring | **`HARDWARE-BOM.md`** |
> | why the parts are what they are | `PLAN-onboard-perception.md` 1, 3.6, P17-P19 |
>
> Kept because its reasoning is the earliest correct statement of the
> case, and because deleting the record of a call that turned out right
> would lose the evidence that it was made before the measurements landed.

**Written 2026-09-12, after P6 and P7d.** This superseded
`PLAN-onboard-perception.md` 3.6 for the compute half; everything else in
3.6 stands and its reasoning is not repeated here.

**Why the board changed.** OWLv2 is the only model that solves search
proposal (82% at 3 false positives over 610 frames, against the shipped
pipeline's 58%, at a ninth of the latency). It does **not** compile to a
Hailo-8L -- measured, P6 -- and it does not survive INT8 quantisation either
(82% -> 7%, P7d), so it runs at **fp16 on a GPU**. That is a Jetson.

**Prices are estimates and MUST be checked.** 3.6 says its own figures were
"not verified against a retailer"; the same applies here. The one figure
actually checked is the Jetson, and it is the one that moves most.

---

## The compute block -- what changed

| removed (Pi plan) | $ | added (Jetson plan) | $ |
|---|---|---|---|
| Raspberry Pi 5 8GB | 80 | **Jetson Orin Nano Super Dev Kit** | **249-385** |
| Active cooler | 8 | *(included -- kit ships with a thermal solution)* | 0 |
| microSD 64GB | 12 | **NVMe SSD, M.2 2280** | 35 |
| Hailo-8L M.2 module | 70 | *(not needed)* | -- |
| Hailo carrier board | 20-48 | *(kit has 2x M.2 Key M + 1x Key E)* | 0 |
| Buck #1, 5V/5A | 8 | **Buck-BOOST, 12V 4-5A** | 15 |
| **removed** | **198-226** | **added** | **299-435** |

**Net: +$100 to +$210** over the Pi 5 + Hailo plan.

### The Jetson price is the biggest variable in this document

NVIDIA's own datasheet says **$249**. Newegg (first-party, "limit 1") lists
it at **$384.90**. That is not a repricing, it is scarcity -- and it means
3.6's "$399-480 since the July 2026 repricing" described the *acquirable*
price more honestly than MSRP does.

**Try in this order before paying the markup:** NVIDIA's own store, then
Arrow / SparkFun / Seeed / RobotShop (the named fulfilment channels, filled
in order of receipt), then Amazon or Newegg.

**Also search "Jetson Orin Nano Developer Kit" without "Super".** Super is a
*software* mode -- NVIDIA's datasheet says existing kits get the 67 TOPS
"with just a software upgrade". Same silicon, sometimes a very different
price.

### What is in the box, confirmed from NVIDIA's docs

- The dev kit, a **19V power supply**, a quick-start card
- **WiFi/Bluetooth is already fitted** -- an RTL8822CE in the M.2 Key E slot.
  Nothing to buy. If you ever replace it, stay on the RTL8822CE: Intel AX210
  has driver trouble on this board
- **No storage.** This is the one thing the kit forces you to buy

---

## Buy list

### Compute and storage

| # | item | ~$ | note |
|---|---|---|---|
| 1 | **Jetson Orin Nano Super Developer Kit**, 8GB, PN 945-13766-0000-000 | 249-385 | see above |
| 2 | **NVMe SSD, M.2 2280, 500GB** | 35 | **Not a microSD.** 1.3 is written about brownout and "SD cards corrupt on brownout" is the exact failure. The kit's M.2 is full-length 2280, so any ordinary drive fits |

### Chassis and drive

| # | item | ~$ | note |
|---|---|---|---|
| 3 | **Differential-drive chassis with encoder motors** (Yahboom 2WD class) | 69 | **Differential, not mecanum.** 1.1 chose it; mecanum slips, and P7c's design spends odometry accuracy. Search "2WD robot chassis encoder motors aluminium" |
| 4 | **Waveshare General Driver for Robots** | 30 | ESP32 runs the velocity PID off Linux's scheduler (1.14), TB6612FNG, encoder inputs, **9-axis IMU absorbed** (QMI8658C + AK09918), 7-13V input direct from the pack. **Verify it drives from a Jetson** -- it is a serial device so it should, but its docs are Pi-facing |

### Sensing

| # | item | ~$ | note |
|---|---|---|---|
| 5 | **Slamtec RPLidar C1** | 99 | 360-degree, 12m. The obstacle sensor AND, per P7c, the thing that measures arrival |
| 6 | **Camera -- decide first, see below** | 20-30 | |
| 7 | **2x VL53L1X ToF, forward-down** | 12 | The under-plane volume 1.15 sizes at bumper-to-32cm below 12cm, crossed in 0.64s at 0.5 m/s. Also the only sensor that sees a descending step |
| 8 | **Compliant bumper + microswitches** | 5 | Everything else in the safety chain is an inference; this is the only measurement |

### Power

| # | item | ~$ | note |
|---|---|---|---|
| 9 | **3S Li-ion pack + charger, x2** | 70 | **3S, not 2S.** 11.1V nominal sits inside the Jetson's 9-20V window and inside the driver board's 7-13V. **Not 4S** -- 16.8V exceeds the driver board |
| 10 | **Buck-boost, 12V out, 4-5A** | 15 | **Boost, not plain buck.** A 3S pack falls to 9.0V, which IS the Jetson's floor -- zero margin, and motor stalls sag the same pack. 12V leaves ~3V both sides. 25W at 12V is 2.1A; size 4-5A for inrush |
| 11 | **Buck, 5V 2-3A** | 8 | Servo rail only. Two SG90s repositioning is a ~1.5A transient and 1.15.3 makes that routine, not a fault |
| 12 | **Bulk electrolytic >=470uF + TVS** | 2 | Motor rail. Regenerative braking against a 3% margin |

### Mechanical and wiring

| # | item | ~$ | note |
|---|---|---|---|
| 13 | 2-axis pan/tilt bracket + servos | 12 | Both axes kept (1.15.3) |
| 14 | Wiring, connectors, switch, XT60 | 15 | |
| 15 | Standoffs, M2.5/M3 | 10 | **Check deck clearance** -- the kit is 103 x 90.5 x 34.8mm, notably bigger and taller than a Pi 5 |
| 16 | Powered USB hub | 15 | |
| 17 | Lidar pedestal | 0-15 | 3D printed if you can. Not a bracket -- it has a job (1.15.2): scan plane as low as possible while clearing the camera's swept envelope |
| 18 | Jumper wires, misc | 10 | |

### Totals

| | ~$ |
|---|---|
| Essentials (1-6, 9-15) | **619-755** |
| + recommended (7, 8, 16-18) | **661-812** |
| *(the Pi 5 + Hailo plan, for comparison)* | *555-620* |

---

## Decide before ordering

### The camera -- this one is a real fork

JetPack has **in-tree drivers for IMX219 and IMX477**, and **not** for the
IMX708 in Raspberry Pi Camera Module 3.

- **IMX219 board (~$20)** -- works on first boot, no kernel patch, loses
  autofocus. For a floor robot looking 1-3m ahead at fixed geometry,
  autofocus may cost nothing at all.
- **Camera Module 3 (~$30)** -- keeps autofocus, needs RidgeRun's open-access
  IMX708 driver, which is version-coupled to JetPack.

**Recommendation: IMX219**, unless you specifically want autofocus. An
out-of-tree camera driver is a recurring tax on every JetPack upgrade, and
1.16 #10's viewpoint discipline cares about *height and levelness*, not
focus.

### Two things to measure, not assume

**The barrel jack.** The kit ships a 19V adapter; NVIDIA's quick-start does
not publish the plug dimensions. Measure the adapter that arrives before
ordering the buck-boost, or buy a pigtail set.

**Deck clearance.** 103 x 90.5 x 34.8mm including feet and thermal solution.
Confirm the chassis deck takes it before ordering the chassis.

---

## What this buys you, and the numbers it rests on

| | measured |
|---|---|
| OWLv2 recall, 610 frames, 3 FP | **82%** vs shipped 58% |
| Precision | fp16 (INT8 collapses to 7%, P7d) |
| Latency, projected on Orin | **~205 ms / 4.9 Hz** at fp16, VGA |
| Runtime, one 3S pack | ~35-50 min at 15W mode |

**A caution carried from P7b.** The Orin latency estimate has moved three
times -- 51 -> 124 -> 205 ms -- each time an assumption became a
measurement, always downward. The board is the measurement; treat 4.9 Hz as
the current best estimate rather than a specification.

**And the preprocessing warning.** At 1280 capture the projected split is
163ms GPU and **229ms CPU** -- OWLv2's own anti-aliased resize, in Python.
Plan to move that resize onto the GPU. Do **not** "fix" it by capturing at
VGA: VGA is upscaled to 960 and loses the detail the distant targets need.

---

## An integrated kit instead?

Worth considering -- roughly $300 of this list is chassis, motors, driver,
lidar, camera mount and battery, and a kit removes assembly risk. The
dual-controller split such kits use (host for AI, separate board for motion
PID) is exactly what 1.14 converged on independently.

**Hiwonder ROSOrin** is the one to price: it takes **Jetson Orin Nano** and
offers a **swappable chassis including differential drive**. **Do not buy the
MentorPi M1** -- it is a Raspberry Pi 5 platform with mecanum wheels, which
is the wrong board and the wrong drive.

Verify on any kit: **camera height** (the valid corpus is 10-13cm, and a
viewpoint mismatch is what invalidated 39 walks), and that its vendor ROS2
stack can sit behind `robot/hardware_robot.py`'s `RobotInterface` seam.
