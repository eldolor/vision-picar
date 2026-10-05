<!--
Provenance: researched and written by Claude Cowork on 2026-09-17 at Anshu's
request, from the prompt in that session. Filed here verbatim below the
editor's note. Do not edit the body to "fix" a fact -- append a dated
correction instead, the way PLAN-onboard-perception.md does.
-->

# Editor's note -- read before acting on this file

Filed 2026-09-17, **updated the same day** once P17-P19 landed.

**This file is now the reference for part numbers, vendors, wiring and
bring-up on the Jetson build** -- which is the recommended build as of
2026-09-17. For what to actually buy (including a kit-search brief) read
**`JETSON-BOM.md`**; for verified prices read **`BOM-COMPARISON.md`**;
`BOM.md` is superseded and `PLAN-onboard-perception.md` 3.6 is the Pi
build's parts list and is no longer the recommended path.

**The body below is Cowork's, unedited.** Its arithmetic was
re-checked here and is correct: Part A+B sums to $751.34 and A+B+C+NVMe to
$891.07 exactly as stated. Its tag discipline (`[V]`/`[I]`/`[U]`/`[D]`/`[R]`)
is the right shape for this project and should be preserved.

Four corrections and one caution, from the repo it is meant to serve:

1. ~~**Section 1 records "Compute is the Jetson... The Raspberry Pi 5 + Hailo-8L
   option is dropped" as `[D]` -- decided by Anshu. That decision has not been
   made.**~~ **RESOLVED 2026-09-17 -- the tag was premature when written and is
   now correct.** P19 ran the test this correction asked for: OWLv2 as a crop
   source *inside* the tier reads **83% against the Pi tier's 50%** on identical
   frames, mask and gate, seven walks better and one tied. P17 had already shown
   OWLv2 compiles to no Hailo, and `BOM-COMPARISON.md` priced the gap at $59-86
   rather than the assumed premium. **The Jetson is the recommended build.** The
   original objection stands as written -- a priced option is not a decision --
   and is kept here because it was right at the time. What follows is the
   correction's own record: What was asked for was a *priced Jetson BOM*, which is the input to
   the decision, not the decision. `PLAN-onboard-perception.md`'s standing
   recommendation is still to run **P10's YOLO-World INT8 test (~$1, one EC2
   hour)** first, because it is the only thing that can put the Pi path back at
   72% recall and make the ~$100 premium arguable. Treat that line as `[R]`
   until the owner says otherwise. This repo is careful elsewhere about not
   letting a measurement read as a decision (1.11a is REPORTED and explicitly
   never enforced); the same care applies to a BOM.

2. **Section 7's PiCar-X warning is stale, and it is the second time this
   premise has appeared.** *"Code written for an Ackermann platform such as a
   PiCar-X needs its motion layer replaced, not tuned."* The PiCar-X was
   retired as a design on 2026-09-03 (1.1), **no hardware has ever been
   ordered**, and `sim/grid_world.py` has always assumed pivot-in-place -- which
   is why S6 (Ackermann turns) was retired rather than built. The motion layer
   is already differential. The rest of section 7 is correct and useful,
   particularly the `nvarguscamerasrc` and udev points.

3. **The TVS part does not protect what it is specified to protect, and the
   file says so.** A 1.5KE18A clamps at ~25.2V against the TB6612FNG's 15V
   absolute maximum. No standard TVS fits the window (stand off >12.6V, clamp
   <15V), so this is an open engineering item, not a shopping one. The bulk
   capacitor and low-ESR routing do the work; size them deliberately.

4. **The JetPack 6 vs 7 decision (open item 2) is the highest-risk unknown in
   this file for THIS project, and it is not really an OS choice.** The entire
   reason for buying this board is to run **OWLv2 in PyTorch fp16** (P7: 82% @3
   FP, against the shipped pipeline's 58%). That depends on `torch` +
   `transformers` wheels existing for the installed L4T. JetPack 6.x / Ubuntu
   22.04 has a mature wheel and `jetson-containers` ecosystem and matches ROS 2
   Humble, which is what 3.3's (b+) path assumed. **Confirm a working
   torch wheel for whichever JetPack before committing an SD card to it** --
   and prefer the one that has one today over the newer one.

**Caution:** section 6's software voltage cutoff is not a note, it is a feature
this repo does not have. Reading the INA219 at >=1 Hz, warning at 10.5V,
stopping the motors and starting a clean shutdown at 10.0V, filtering motor
sag, and surfacing an "unplug the battery" prompt are all new work in
`robot/safety.py`, `RobotInterface` and the twin. Under CLAUDE.md section 7 it
also owes a **readout someone can watch on a phone** -- pack voltage beside the
watchdog line is the obvious one. The finding that forces it is section 6's
best: **a 3S BMS cuts off at 8.4-9.0V, at or below the Jetson's own 9V floor,
so a BMS protects the cells and not the board.** Software is the only guard.

Also worth carrying forward: **section 4.2's ESP32 JSON protocol table is the
hardware-side shape of `RobotInterface`**, and section 4.2's note that **no
explicit emergency-stop command exists** (stop is zero-speed plus a heartbeat,
tagged `[U]`) lands directly on failsafe B3.1 and M4's arbitration. Verify it
on arrival before trusting `stop()`.

**Correction 5, 2026-09-27 -- section 4.2's motor protocol, as the firmware
source actually defines it.** Read from `waveshareteam/ugv_base_general`
(`General_Driver`) for R7; the full record is `PLAN-ros-alignment.md` 3.16,
and `sim/fake_esp32.py` encodes it. Four lines of 4.2 are wrong or
unverified, and the first one can hurt the car:

* **`T=1` is NOT a speed in the mode 4.2 selects.** In `mainType` 1 and 2
  (4.2's own example is `{"T":900,"main":2}`) `T=1` is OPEN-LOOP PWM:
  `PWM = L x 512`, clamped to +/-255. **4.2's example `{"T":1,"L":0.5,"R":0.5}`
  is therefore PWM 256 -> 255: FULL POWER** (~1 m/s no-load, 4.3), not
  0.5 m/s. Do not send it to a board on the stand with the wheels on the
  floor. Closed-loop speed in m/s exists only in `mainType` 3.
* **`mainType` 3 needs a firmware change, not a command.** Its wheel
  constants are hard-coded for another robot (0.0523 m wheels, 1092
  pulses/rev, 0.141 m track). 5.2 step 4 ("Set wheel diameter, counts per
  revolution and track width") means *rebuild and flash the firmware with
  4.3's values*. `robot/hardware_robot.py` assumes that has been done; it
  never sends `T=900` itself.
* **The `1001` frame layout is known:**
  `{"T":1001,"L","R","r","p","y","temp","v"[,"pan","tilt"]}`. `L`/`R` are
  wheel SPEEDS in m/s -- **there are no encoder counts**, so odometry
  integrates speed x time on the host (`hardware_robot.py`). Open item 9 is
  answered; a firmware change that also reports counts is the upgrade.
* **The heartbeat stop is VERIFIED, no longer `[U]`.** `heartBeatCtrl()`
  zeroes the motors once `T=136`'s interval passes with no `T=1`/`11`/`13`
  (firmware default 3000 ms). The host sets **1500 ms**
  (`hardware_robot.py` `HEARTBEAT_MS`), deliberately longer than the robot
  server's 1.0 s watchdog, so the server acts first and the board acts if
  the server itself dies.

**How the repo addresses this board:** `mode: hardware` in
`config/robot.yaml` (or `ROBOT_MODE=hardware`), with the serial device in
`ROBOT_SERIAL` or `hardware.serial_port` (`robot/factory.py`). The process
needs read/write on that device -- on Ubuntu, a udev rule binding the
CP210x by serial number (section 7) and the user in the `dialout` group,
or `os.open()` fails with EACCES. `SIM_MOTOR_BOARD=fake` runs the same
backend against `sim/fake_esp32.py` on a pty.

**Correction 6, 2026-09-27 -- JetPack (4.1, open item 2, correction 4).**
The repo already builds for **JetPack 6.x / Ubuntu 22.04 / ROS 2 Humble**:
`service/slam/Dockerfile` is `FROM ros:humble-ros-base` (R3,
`PLAN-ros-alignment.md` 3.12). JetPack 7.x would mean moving that container
to Jazzy. So treat **JetPack 6.2.1 (the SD-card image)** as the working
choice, subject only to correction 4's torch-wheel check -- and B8's USB
installer, which is for 7.2.1, is then probably unnecessary. Correction 4's
premise has also moved: the shipped detector is `yoloe-11s-seg`, not OWLv2
(`PLAN-onboard-perception.md` P22-P24), though it still needs torch.

---

# Indoor autonomous robot — Jetson BOM and hardware reference

Prepared 2026-09-17 for Anshu, to hand to Claude Code as hardware context.
Ship to / pick up near St. Charles, IL 60174. Sales tax 8.75%. New parts only.

**Nothing on this list has been purchased yet.** Prices and stock were read on 2026-09-17 and will drift.

## How to read this file

Every fact carries one of these tags. Treat anything not tagged `[V]` as something to confirm before relying on it in code.

| Tag | Meaning |
|---|---|
| `[V]` | Verified: read from the vendor's or manufacturer's own page on 2026-09-17 |
| `[I]` | Inferred or computed from verified facts |
| `[U]` | Unverified: general knowledge, or a page that could not be read. Confirm on hardware. |
| `[D]` | Decided by Anshu |
| `[R]` | Recommended in research, **not yet confirmed by Anshu** |

Amazon and Micro Center product pages could not be read by the research tools. The one exception is the Micro Center Jetson page, which Anshu supplied as a PDF capture.

---

## 1. Decisions

### Decided `[D]`
- Compute is the **NVIDIA Jetson Orin Nano Super Developer Kit** (8GB). The Raspberry Pi 5 + Hailo-8L option is dropped.
- Differential drive only. No mecanum, no Ackermann, no tracks.
- Lidar is the **Slamtec RPLidar C1**, no substitutes (chosen for driver quality).
- Motor/IO controller is the **Waveshare General Driver for Robots** (ESP32).
- Camera is an **IMX219** CSI module. Not Raspberry Pi Camera Module 3 (IMX708): poor JetPack driver support.
- Camera mount: **one pan axis** on an ST3215 bus servo, at a **fixed downward pitch**. No tilt servo.
- The Jetson is powered **straight from the battery** through its 9–20V barrel jack. No 5V buck converters anywhere in the build.
- Prefer the Waveshare 7–36V USB hub, which runs straight off the pack.
- Minimize vendor count; paying about $5 more beats adding a shipping charge.

### Recommended, not yet confirmed `[R]`
- Power: keep 3S LiPo packs, add a low-voltage buzzer, and implement a **software voltage cutoff** using the driver board's INA219 (section 6).
- microSD: Team PRO+ 128GB A2. NVMe (optional): Team MP33 256GB.
- Camera mount: 3D-print at St. Charles Public Library from a community ST3215 bracket design plus a custom camera wedge.
- Buy the $69 no-battery chassis, because the bundled battery's spec is unknown.

### Rejected — do not propose these
Bundled ROS 2 robot kits (Yahboom ROSMASTER M3/X3/M1, Hiwonder LanderPi), Jetson Orin NX, Orin Nano non-Super modules, Raspberry Pi AI Camera (IMX500), Google Coral, Hailo AI HAT+, anything Pi 4, SG90/MG996R or any open-loop servo, L298N drivers, LDROBOT LD06 or unbranded lidar, Intel AX210 Wi-Fi card (needs a kernel rebuild on JetPack 6.2).

---

## 2. System architecture

### Power topology `[I]`
```
3S LiPo 2200mAh (9.9–12.6V usable), XT60
  └─ inline 10A fuse ─ main switch ─┬─ Jetson barrel jack 5.5×2.5mm (spec 9–20V)
                                     ├─ Waveshare General Driver, XH2.54 power port (spec 7–13V)
                                     │     ├─ TB6612FNG ─ 2× 12V 520 encoder motors
                                     │     └─ ST3215 servo port (servo sees raw pack voltage)
                                     └─ Waveshare USB hub, screw terminal (spec 7–36V)
                                           └─ 5V to USB devices (RPLidar C1, etc.)
LiPo balance lead ─ low-voltage buzzer (set 3.4V/cell)
Motor rail: ≥470µF electrolytic across the driver board input
```
There is **no BMS and no hardware low-voltage disconnect**. Over-discharge protection is the buzzer plus software.

### Data topology `[I]`
```
Jetson Orin Nano Super devkit
  ├─ CSI CAM0 (22-pin, 0.5mm) ── IMX219 camera (Arducam B0191)
  ├─ USB ── Waveshare 4-port hub
  │         ├─ RPLidar C1 (its own USB-UART adapter, 460800 baud)
  │         └─ General Driver board, Type-C "USB" port (CP2102, 115200 baud, JSON)
  ├─ M.2 Key E ── RTL8822CE Wi-Fi/BT (pre-installed)
  ├─ M.2 Key M 2280 ── NVMe SSD (optional)
  └─ microSD slot

General Driver board (ESP32-WROOM-32)
  ├─ TB6612FNG ── left/right motors + quadrature encoders
  ├─ Bus servo UART ── ST3215 (ID 1, 1 Mbps) = camera pan
  ├─ I2C ── INA219 (pack voltage/current), 9-axis IMU, optional OLED
  └─ I2C expansion / spare GPIO ── candidates for VL53L1X ×2 and bumper switches (OPEN, see section 8)
```

---

## 3. Bill of materials

### Part B — compute

| # | Item | Exact part | Qty | Price | Source | Stock / status |
|---|---|---|---|---|---|---|
| B1 | Jetson Orin Nano Super Developer Kit | NVIDIA 945-13766-0000-000, Micro Center SKU 812057, UPC 812674025261 | 1 | **$399.00** `[V]` | Micro Center Westmont, in-store pickup only ("Shipping Not Available") | 7 in stock at capture (about 2 AM CDT 2026-09-17), Aisle 1 Maker & STEM. 30-day return, 1-year warranty. $399 is NVIDIA's list price since July 2026 (was $249); Newegg also $399. |
| B2 | Wi-Fi + BT card | Pre-installed in the devkit | 0 | **$0** `[V]` | — | See section 5.1. Contingency only: Waveshare RTL8822CE card $5.99 with antennas. |
| B3 | CSI camera | Arducam **B0191** (IMX219, fixed focus, 25×24mm board, 62.2°×48.8° FOV) | 1 | **$19.95** `[V]` | SparkFun SEN-28170, in stock | Arducam's own page lists "15cm 15-22pin FPC cable" and "15cm 22-22pin FPC cable" in the box `[V]`; SparkFun's page does not list contents. Arducam direct is $15.99 promo, shipping unknown. |
| B3b | Spare camera cable (optional insurance) | Adafruit #5818, 22-pin to 15-pin, 200mm | 1 | $2.70 `[V]` | Adafruit, in stock | Not in totals. |
| B4 | microSD 128GB, A2 | Team PRO+ TPPMSDX128GIA2V3003 | 1 | **$34.99** `[V]` | Newegg, sold by Newegg, free shipping, in stock | Alternative: SanDisk Extreme 128GB A2 $40.25 at B&H `[V]`. **Required even with an NVMe** (firmware update medium, section 5.1). NVIDIA minimum is 64GB UHS-1. |
| B5 | NVMe SSD, M.2 2280 (optional) | Team MP33 256GB | 1 | **$68.99** `[V]` | Newegg, sold by Newegg, free shipping, limit 2 | 5-year warranty. 512GB is $93.99 `[V]`. ADATA Legend 700 256GB is $54.99 + $4.99 shipping `[V]`. Earlier Kingston NV3 500GB baseline is stale: now $137.99. Avoid Newegg marketplace sellers shipping from Hong Kong. |
| B6 | DisplayPort cable | 6ft DP 1.2 | 1 | about $10 `[I]` | Micro Center SKU 625860 (Inland, price unverified) or StarTech DISPLPORT6L $7.95 at B&H `[V]` | Devkit has DisplayPort only, no HDMI. For an HDMI-only monitor: Plugable active DP→HDMI $18.95 `[V]` or Micro Center QVS active adapter SKU 442151. |
| B7 | Barrel pigtail 5.5×2.5mm, 18AWG | Tensility 10-02220 (6ft, rated 8A) | 1 | about $7 `[I]` | DigiKey (price not readable) | Micro Center alternative: Philmore 2.5mm DC plug to terminal, SKU 662088. **A 5.5×2.1mm plug will not seat.** Avoid 24AWG pigtails (Tensility 10-02227). |
| B8 | USB flash drive, 16GB or larger | any | 1 | about $10 `[I]` | Micro Center | Needed for NVIDIA's current JetPack 7.2.1 installer. Skip if already owned. Not in totals. |

### Part A — chassis, sensing, power

| # | Item | Exact part | Qty | Price | Source | Stock / status |
|---|---|---|---|---|---|---|
| A1 | 360° lidar | Slamtec **RPLidar C1** (C1M1-R2) | 1 | **$69.00** `[V]` | DFRobot or Seeed, in stock, ships from China. DFRobot: free shipping over $50, duties prepaid. | $71.92 at RobotShop US `[V]`. $79.99 at Waveshare `[V]`. |
| A2 | 2WD encoder chassis | Yahboom Smart Robot Car Chassis Kit, 2WD, aluminum. Amazon ASIN **B0F3CZ3WYB** (no battery) / **B0F3CYDQ21** (with battery). Yahboom SKU 6000200702 / 6000200712. | 1 | **$69** no battery, $79 with `[V]` | Yahboom store, in stock. Amazon price unreadable. | Kit also bundles an MSPM0 control board and a 4-channel motor driver, which this build does not use. Bundled battery spec is **unknown** (section 8). |
| A3 | Motor/IO controller | Waveshare **General Driver for Robots** | 1 | **$27.99** `[V]` | Waveshare, stock not shown | $34.77 at RobotShop `[V]`. |
| A4 | Pan servo | Waveshare **ST3215**, **12V variant** (6–12.6V, 30 kg·cm) | 1 | **$21.99** `[V]` | Waveshare, stock not shown | $23.99 at Seeed (US warehouse) `[V]`, $28.60 at RobotShop `[V]`. **Avoid the 7.4V variant**: Seeed C046, Amazon B0G2576KKN. |
| A5 | Pan mount | 3D-printed: servo bracket + custom camera wedge | 1 | about $5 `[I]` | St. Charles Public Library: $0.10/g, cardholders, Bambu P1S/X1C, STL or OBJ, no guaranteed turnaround `[V]` | Base design: makerforgetech "Waveshare ST3215 Servo Mount Brackets", Thingiverse 7074577, CC BY-SA. A 10–20° wedge with the camera's hole pattern **still has to be modelled**. Fallback: Waveshare 2-Axis Pan-Tilt Camera Module $109.99. |
| A6 | Battery + charger | 2× Ovonic 3S 2200mAh 50C LiPo, XT60 ($29.99 per 2-pack, free US shipping) + iMAX B3 charger ($14.99) + 1–8S LiPo buzzer ($5.49, GetFPV) | 1 set | **$50.47** `[V]` | us.ovonicshop.com, supergdrift.com, getfpv.com; all in stock | **No BMS.** Better charger for +$21: ToolkitRC C6, 1–6S, $35.99 at RaceDayQuads `[V]`. |
| A9 | Wiring, switch, XT60 pigtails | 4× XT60 pigtail 14AWG ($1.99 each, RaceDayQuads), rocker switch (Pololu #1406, $1.75), 20AWG silicone wire 50ft (BNTECHGO, $12.29) | 1 set | about $22 `[V]` | three shops | The Pololu rocker is rated 10A at 125VAC only, no DC rating. **Inline 10A fuse + holder not yet priced.** |
| A10 | M2.5 standoff kit | brass, 180–220 pieces | 1 | $12.95 `[V]` | PiShop, in stock | Better: Micro Center 52Pi 220-piece kit SKU 632040 (price unverified) to avoid a one-item shipping charge. No M3 kit found at Micro Center. |
| A11 | Motor-rail cap + TVS | Nichicon UVR1E102MPD 1000µF/25V; Littelfuse 1.5KE18A | 1 each | about $2 `[V]` | Newark / DigiKey | TVS clamps at 25.2V, above the TB6612FNG's 15V absolute maximum `[U]`, so the bulk capacitor does the real work. |

Not needed `[D]`: A7 (5V/5A buck) and A8 (5V/2–3A buck). The Waveshare board powers the ST3215 from its own 7–13V input `[V]`.

### Part C — recommended

| # | Item | Exact part | Qty | Price | Source | Stock / status |
|---|---|---|---|---|---|---|
| C1 | ToF distance sensors | Adafruit **#3967 VL53L1X** (4 m), exposes XSHUT | 2 | **$29.90** `[V]` | Adafruit, in stock | Pololu #3415 is $22.95 each. Micro Center stocks only the VL53L**0**X. |
| C2 | Bumper switches + springs | 4× Omron SS-5GL SPDT lever ($1.58 each, Arrow) + spring assortment (Harbor Freight #67562, $4.99) | 1 set | about $11 `[V]` | Arrow, Harbor Freight | |
| C3 | Powered USB hub | Waveshare **USB3.2-Gen1-HUB-4U**, 4-port, **DC 7–36V** input | 1 | **$17.99** `[V]` | Waveshare, stock not shown | Power via screw terminal or a **5.5×2.1mm** jack (not the Jetson's 5.5×2.5). 2A max per port, 5A total. Includes a USB 3.2 dual-plug host cable, no power adapter. |
| C4 | Jumper wires | Adafruit #826 (F/M), #266 (F/F), #758 (M/M) | 1 set | $11.85 `[V]` | Adafruit; M/M out of stock | Micro Center Inland 3-pack SKU 613879 (price unverified). |

### Totals (computed)

| Build | Subtotal | With 8.75% tax | Shipping estimate | All-in |
|---|---|---|---|---|
| Essential: Part A + Part B, no NVMe | $751.34 | $817.08 | $40–60 | **about $857–877** |
| Full: A + B + C + MP33 256GB NVMe | $891.07 | $969.04 | $50–70 | **about $1,019–1,039** |

B6, B7, A5 and A9 are estimates. B3b and B8 are excluded (B8 adds about $10.88 with tax). Waveshare shows shipping only at checkout; $15–25 is assumed. The original $500–750 budget is exceeded, mainly because the devkit list price rose from $249 to $399.

### Vendor plan `[R]`
- **Micro Center Westmont (pickup):** B1, and if the store prices are reasonable B4, B5, B6, B8, A10, C4 and the barrel plug. SKUs to check with Westmont selected: microSD 659096 (house-brand Performance 128GB A2) or 651841 (SanDisk Extreme PLUS 128GB A2); NVMe 661858 (Inland TN320 256GB) or 661860 (512GB); 625860; 442151; 613879; 632040; 662088; capacitor kit 632685; rocker switch 614933. Not stocked there: XT60 parts, 18–20AWG silicone wire, VL53L1X, 3S/4S LiPo packs, balance chargers.
- **Newegg (one order, free shipping):** B4 + B5 if not bought at Micro Center.
- **Waveshare (one order):** A3 + A4 + C3. Choose a delivered-duty-paid (DDP) shipping method; reviews from mid-2026 report surprise fees after ordering.
- **DFRobot:** A1. Alternative: RobotShop US carries A1 + A3 + A4 for $135.29 with free shipping (+$16 versus the split, one US vendor, stock not shown).
- **Adafruit (one order):** C1 + B3b + C4. **SparkFun:** B3.
- **Yahboom store or Amazon:** A2. **Ovonic + hobby shop:** A6. **DigiKey (one order):** B7 + A11 + C2 switches.

---

## 4. Component reference for software

### 4.1 Jetson Orin Nano Super Developer Kit
- 6-core Cortex-A78AE at 1.7 GHz, 1024-core Ampere GPU with 32 Tensor cores, 8GB LPDDR5 at 102 GB/s, 67 TOPS (sparse INT8), 7–25W `[V]`.
- I/O: 4× USB 3.2 Gen 2 Type-A, 1× USB-C (**debug/data only, does not power the board**), Gigabit Ethernet, DisplayPort 1.2 (no HDMI), 2× MIPI CSI-2 22-pin 0.5mm connectors, microSD, M.2 Key M 2280 (PCIe 3.0 x4), M.2 Key M 2230 (PCIe 3.0 x2), M.2 Key E 2230 populated with the wireless module, 40-pin header `[V]`.
- Size: 100 × 79 × 21mm per NVIDIA; Micro Center lists 103 × 90.5 × 34.77mm overall `[V]`.
- Power input: barrel jack 5.5mm × 2.5mm `[V]`. Range 9–20V per NVIDIA forum staff, who cite a hard under-voltage lockout near 5.5V `[V]`. Stock adapter is 19V `[V]`. Center-positive per JetsonHacks only, not an NVIDIA document `[U]`: **meter the stock adapter before first power-up.**
- Wi-Fi/BT: Realtek **RTL8822CE** (AzureWave AW-CB375NF), two PCB antennas in the plastic base, MHF4 leads that are easy to detach and hard to reattach `[V]`. Works on JetPack 6.x; one forum thread reports slow throughput, moderator suggested forcing 5 GHz `[V]`.
- Display: NVIDIA's hardware page says both passive and active DP→HDMI adapters are supported; its supported-hardware page lists only active `[V]`.
- Power modes: default 25W. `sudo /usr/sbin/nvpmodel -q` to query, `sudo /usr/sbin/nvpmodel -m <id>` to set. MAXN SUPER is selectable `[V]`. Mode IDs were not read: query on the device.
- CSI camera: use **CAM0**, 22-contact end to the Jetson, **gold contacts facing down** `[V]`. Check with `v4l2-ctl --list-devices`; capture with a `gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM), ...'` pipeline `[V]`. IMX219 typically needs enabling through `jetson-io` `[U]`. Use the Arducam-supplied cable: a forum user's generic 22-22 cable had to be flipped and caused shorts `[V]`.
- 40-pin header `[U]`: UART on pins 8/10 is usually `/dev/ttyTHS1` on JetPack 6; I2C on pins 3/5 is usually bus 7, pins 27/28 bus 1. 3.3V logic. Confirm on the device.
- **JetPack version is an open decision** (section 8). NVIDIA's current quick start installs **JetPack 7.2.1** from a bootable USB drive onto microSD or NVMe; any Windows, Mac or Linux laptop can prepare the drive with Balena Etcher `[V]`. JetPack 6.2.1 remains available as an SD-card image `[V]`. The Wi-Fi and camera checks above were made against JetPack 6.x. JetPack 6.x is Ubuntu 22.04 (ROS 2 Humble); JetPack 7.x is believed to be Ubuntu 24.04 (ROS 2 Jazzy) `[U]`. Confirm with `cat /etc/nv_tegra_release` and `lsb_release -a`.

### 4.2 Waveshare General Driver for Robots (ESP32)
- ESP32-WROOM-32; TB6612FNG; INA219 voltage/current monitor; 9-axis IMU (product page: QMI8658C + AK09918C); ST3215 bus-servo interface; lidar interface; TF card slot; two CP2102 USB-UART bridges; two 40-pin headers; onboard 5V DC-DC meant to power a Raspberry Pi or Jetson Nano through the header `[V]`.
- Power input: XH2.54 port, **DC 7–13V**, "directly powers the serial bus servo and motor". Long-term current limit **5A**, set by the power switch `[V]`.
- Two Type-C ports: **"USB"** = ESP32 UART (host link and firmware upload); **"LIDAR"** = lidar data through the second CP2102 `[V]`.
- **Host link for this build: USB**, which appears as `/dev/ttyUSB*` `[I]`. Create udev rules so the board and the lidar get stable names; both can enumerate as CP210x `[I]`.
- **Do not stack this board on the Jetson's 40-pin header** without checking: its 5V regulator would back-feed the Orin Nano's 5V pins, and the Orin Nano cannot be powered from 5V `[I]`.
- Stock firmware: `github.com/waveshareteam/ugv_base_general`, directory `General_Driver`, **GPL-3.0**, Arduino/ESP32 `[V]`. Libraries: ArduinoJson, LittleFS, Adafruit_SSD1306, INA219_WE, ESP32Encoder, PID_v2, SimpleKalmanFilter, Adafruit_ICM20X, Adafruit_ICM20948, SCServo `[V]`.
- **Discrepancy to resolve:** the firmware depends on ICM-20948 libraries while the product page lists QMI8658C + AK09918C. Check which IMU the delivered board revision carries before writing IMU code `[V]`.
- ESP32 pin map from `ugv_config.h` `[V]`:

| Function | GPIO |
|---|---|
| Motor A: PWMA / AIN1 / AIN2 | 25 / 21 / 17 |
| Motor B: PWMB / BIN1 / BIN2 | 26 / 22 / 23 |
| Encoder A: A / B | 35 / 34 |
| Encoder B: A / B | 27 / 16 |
| I2C: SDA / SCL | 32 / 33 |
| Bus servo UART: RX / TX | 18 / 19 |
| Spare IO4 / IO5 | 4 / 5 |

- Protocol: **newline-delimited JSON over UART/USB at 115200 baud** `[V]`. Commands from `json_cmd.h` `[V]`:

| T | Name | Example |
|---|---|---|
| 1 | Speed control, left/right | `{"T":1,"L":0.5,"R":0.5}` |
| 11 | Raw PWM, ±255 | `{"T":11,"L":164,"R":164}` |
| 13 | ROS-style, linear X m/s + angular Z rad/s | `{"T":13,"X":0.1,"Z":0.3}` |
| 2 | Motor PID | `{"T":2,"P":200,"I":2500,"D":0,"L":255}` |
| 3 | OLED text | `{"T":3,"lineNum":0,"Text":"..."}` |
| 126 / 127 | Get IMU data / calibrate IMU (about 5 s) | `{"T":126}` |
| 130 | Request base feedback once | `{"T":130}` |
| 131 | Continuous feedback on/off (default off) | `{"T":131,"cmd":1}` |
| 142 | Extra feedback delay, ms | `{"T":142,"cmd":0}` |
| 136 | Heartbeat interval, ms | `{"T":136,"cmd":3000}` |
| 133 / 134 / 135 | Gimbal simple / move / stop | `{"T":133,"X":45,"Y":45,"SPD":0,"ACC":0}` |
| 132 / 113 | IO4/IO5 PWM / 12V switch outputs | `{"T":132,"IO4":0,"IO5":0}` |
| 501 / 502 / 503 | Servo: set ID / set middle / set PID | `{"T":501,"raw":1,"new":11}` |
| 143 / 600 / 900 | UART echo / reboot / robot-type config | `{"T":900,"main":2,"module":2}` |
| 1001 / 1002 | Feedback **from** board: base info / IMU data | field layout not read: capture live |

- No explicit emergency-stop command exists. Use zero speed plus the heartbeat: the firmware is believed to stop the motors when no command arrives within the heartbeat interval `[U]`.
- **Firmware defaults are for Waveshare's own UGV** `[V]`: wheel diameter 0.080 m, track width 0.172 m, and 1650 encoder pulses per revolution in the open-loop General Driver firmware. They are wrong for the Yahboom chassis this BOM was written for (replace them with 4.3's values), and right for the UGV Rover ordered 2026-09-30 (`JETSON-BOM.md` section 9), except the pulse count: the Rover's ROS Driver firmware, and Waveshare support, give **660**.
- The gimbal commands assume a 2-servo pan-tilt. This build has one pan servo (default ID 1), so either drive it via T=133 and ignore Y, or address the servo directly `[I]`.

### 4.3 Motors, encoders, wheels (Yahboom L-type 520)
- 12V rated, starts at 6V, gear ratio **1:40**, no-load 300 rpm ±5%, rated 150 rpm, stall 4A, stall torque 10 kg·cm, rated torque 4.4 kg·cm `[V]`.
- Encoder: Hall, **11 lines** on the motor shaft, PH2.0 6-pin cable `[V]`. Treated as quadrature AB; no page says so outright `[I]`. The 6-pin pin order was not documented: **check it against the Waveshare motor port before plugging in**. Yahboom notes that motor "A" maps to phase B on its own driver, so direction sign may need inverting `[V]`.
- Counts per wheel revolution: 440 (1×), 880 (2×), **1760 (4× quadrature)** `[I]`.
- Wheels: 65mm rubber `[V]`. Distance per 4× count: about 0.116 mm. Top speed about 1.02 m/s no-load, 0.51 m/s at rated rpm `[I]`.
- **Track width, deck dimensions and payload are unpublished: measure on the chassis.**
- TB6612FNG continuous current is about 1.2A per channel (3.2A peak) `[U]`, well below the 4A motor stall: avoid sustained stalls in software.

### 4.4 ST3215 pan servo
- Feetech STS-series protocol, half-duplex TTL, **1 Mbps**, default **ID 1** `[V]`.
- 12-bit magnetic encoder: **4096 steps per 360°** (0.088° per step), range 0–4095, middle 2047. Servo mode 360°; continuous-rotation motor mode available `[V]`.
- Feedback: position, speed, load, voltage, temperature, current `[V]`.
- 6–12.6V, 30 kg·cm at 12V, 45 rpm no-load (0.222 s per 60°), stall 2.7A, idle 200mA `[V]`. On this robot it sees raw pack voltage (9.9–12.6V).
- Body about 45.2 × 24.7 × 35mm, 25T spline. It appears to have no standard servo mounting ears; it mounts through case screw points `[U]`.
- Library: **SCServo**, class `SMS_STS`, on the ESP32's `Serial1` `[V]`.
- The servo position is also the camera's yaw relative to the chassis: publish it as a TF joint `[I]`.

### 4.5 RPLidar C1
- DTOF, 360°, range 0.05–12 m (white, 70% reflectivity) and 0.05–6 m (black, 10%), 5 kHz sample rate, 8–12 Hz scan (10 Hz typical), 0.72° angular resolution, ±30mm accuracy, 15mm resolution `[V]`.
- TTL UART at **460800 baud**; ships with a USB adapter. 55.6 × 55.6 × 41.3mm, 110g, IP54, Class 1 laser `[V]`.
- Mount level: operating pitch angle 0–1.5° `[V]`. Top-mounted with a clear 360° view.
- Drivers: Slamtec `rplidar_ros` (ros2 branch) or `sllidar_ros2`, both believed to ship a C1 launch file with `serial_baudrate:=460800` `[U]` (the GitHub page could not be read).

### 4.6 Camera (Arducam B0191, IMX219)
- 8MP Sony IMX219, fixed focus, FOV 62.2° H × 48.8° V, board 25 × 24mm `[V]`. Mounting holes believed to follow the Raspberry Pi Camera v2 pattern, about 21 × 12.5mm, M2 `[U]`.
- Arducam states it is designed for the official NVIDIA devkit and lists Orin Nano support `[V]`.

### 4.7 VL53L1X ×2 (Adafruit #3967)
- I2C ToF, up to 4 m. **Both boards power up at address 0x29** `[V]`. Hold one in reset with **XSHUT**, re-address the other at boot, then release; or use an I2C mux. Re-addressing is volatile and must run on every power-up `[U]`.
- Planned orientation: forward and downward (obstacle and drop-off detection) `[D]`.

### 4.8 Bumper
- 4× Omron SS-5GL SPDT lever microswitches behind a spring-loaded compliant bumper `[D]`. Digital inputs, debounce in software.

### 4.9 USB hub (Waveshare USB3.2-Gen1-HUB-4U)
- Powered from the pack, so USB devices do not draw from the Jetson's USB rail `[D]`. 2A per port, 5A total `[V]`.

---

## 5. Bring-up

### 5.1 Jetson firmware check — do this first `[V]`
1. Some devkits ship with factory firmware **older than 36.0**, which cannot boot JetPack 6.x or 7.2.1. A Micro Center review from August 2026 reports a unit bricked during this update; an older review reports a unit that shipped current.
2. With a DisplayPort monitor and USB keyboard attached, power on and press **Esc** repeatedly after the NVIDIA splash to read the firmware version in the UEFI menu. If JetPack media drops to a UEFI shell instead of booting, the firmware is too old.
3. If older than 36.0, follow NVIDIA's **JetPack 6.x Update Path**: write JetPack 5.1.3 to a 64GB+ microSD and boot; let it schedule the firmware update and reboot; install the QSPI updater with apt; reboot; then swap in the target JetPack image.
4. **Run every firmware step on the stock 19V adapter, never the battery.** NVIDIA: "Do not remove power while a firmware update is in progress."
5. Do this inside Micro Center's 30-day return window.

### 5.2 Order of operations `[R]`
1. Meter the stock adapter's polarity, then build the battery barrel pigtail to match.
2. Firmware check, install JetPack, confirm Wi-Fi, then enable and test the camera on CAM0.
3. Bench-test the driver board over USB at 115200: send `{"T":130}` and read pack voltage before connecting motors.
4. Identify the delivered board's IMU chip. Set wheel diameter, counts per revolution and track width for this chassis.
5. Verify the motor connector pin order, then test each motor with low PWM (`T=11`) and check encoder sign.
6. Re-address the VL53L1X pair. Test the servo at ID 1 and set its middle (`T=502`) with the camera facing forward.
7. Only then run from the battery, with the buzzer attached and the voltage cutoff active.

---

## 6. Power budget and battery policy

- Load assumption: Jetson about 25W peak, motors and sensors about 15W peak, so **40W peak**; about **21W average** (15W + 6W) `[I]`.
- One 3S 2200mAh pack is 24.4Wh nominal, about 23.2Wh usable when stopping at 3.3V per cell `[I]`. Stopping at 3.3V instead of 3.0V per cell strands only about 2–5% on LiPo (from a published 3S discharge curve) `[I]`.
- Runtime per pack: about **66 min at 21W**, about **35 min at 40W**. Two packs swapped: about 2.2 hours of average use `[I]`.
- The margin is thin: 9.9V at cutoff is only 0.9V above the Jetson's 9V floor, before wiring and connector drop.
- Generic 3S BMS boards cut off at 2.3–3.0V per cell (pack 8.4–9.0V), at or below the Jetson's floor, so a BMS would protect the cells but not the Jetson `[V]`.

### Software requirements that follow `[R]`
- Read pack voltage from the driver board's INA219 (base feedback) at 1 Hz or faster.
- **Warn at about 10.5V. Stop the motors and start a clean Jetson shutdown at about 10.0V under load.** Filter the reading (for example, several seconds below threshold) so motor-start sag does not trigger a false shutdown.
- A software shutdown does not disconnect the pack: the ESP32 board and hub keep draining it. Surface an "unplug the battery" prompt in the UI; the pack must be physically unplugged after every run.
- Set the hardware buzzer to 3.4V per cell as the independent backstop.
- Log input voltage alongside Jetson power mode, so brownouts can be diagnosed. If brownouts occur at 25W, first drop the power mode; the hardware fallback is 4S below.

### Fallback: 4S, only if brownouts are logged `[R]`
2× CNHL Black 4S 2200mAh 40C XT60 ($17.99 each, US warehouse, in stock; 106 × 34 × 28mm, 211g) + ToolkitRC C6 charger ($35.99) + Pololu D36V28F12 12V 2.4A buck ($24.95) feeding the driver board, which cannot take 16.8V. About +$57 over the baseline and three new vendors; about 88 min per pack at 21W. The Jetson (≤20V) and the hub (≤36V) accept 16.8V directly. Still no hardware disconnect.

### Evaluated and rejected
- Waveshare UPS Module 3S ($28.95): battery output is 12.6V at 2A, too little for 25W plus motors; its 5V/5A rail is useless to the Jetson `[V]`.
- Waveshare UPS Power Module (C) for Orin Nano ($24.99, 3× 21700, pogo pins, outputs raw 9–12.6V): no output for the motors `[V]`. It does show Waveshare itself runs the Orin Nano on raw 3S.
- 12V buck-boost in front of the Jetson (Pololu S13V25F12, 2.5A): about a 30W ceiling, marginal `[V]`.
- XH-M609 low-voltage-disconnect relay: rated for a 12–36V supply, so a 10V setpoint is out of spec `[V]`.

### Safety
Inline 10A fuse at the pack. Balance-charge at 1C or less (2.2A) in a LiPo bag, attended. Never recharge a pack that fell below about 3.0V per cell or is puffed. Store at about 3.8V per cell.

---

## 7. Notes for porting existing robot software `[I]`
- The drive interface is **differential**: command left/right wheel speeds (`T=1`) or linear X + angular Z (`T=13`). There is no steering angle. Code written for an Ackermann platform such as a PiCar-X needs its motion layer replaced, not tuned.
- Wheel odometry is available from encoders; an IMU and a 360° lidar are present, so `slam_toolbox` + Nav2 are the intended direction `[D]`.
- The camera has one controllable axis (pan) with position feedback; pitch is fixed. Search behaviours that assumed tilt must be reworked.
- Compute moves from a Raspberry Pi to a Jetson: CSI capture goes through `nvarguscamerasrc` / Argus rather than libcamera or picamera2, and GPU inference (TensorRT, CUDA) replaces any NPU path.
- Two CP210x serial devices will be present (driver board and lidar): bind by udev attributes, never by `/dev/ttyUSB0` order.

---

## 8. Open items

| # | Item | Status |
|---|---|---|
| 1 | Yahboom bundled battery (ASIN B0F3CYDQ21) | **Unknown.** Spec exists only in listing images. Probably 12.6V 3S (about 75% confidence): Yahboom says 520 motors want a 12.6V supply and sells only 12.6V chargers. Capacity, connector, protection, charger model all unknown. |
| 2 | JetPack 6.2.1 vs 7.2.1, and therefore ROS 2 Humble vs Jazzy | Undecided. Driver checks in this file were against JetPack 6.x. |
| 3 | Where the VL53L1X pair and bumper switches connect | Undecided: ESP32 I2C expansion and spare GPIO (shares the bus with INA219/IMU/OLED; needs firmware changes, firmware is GPL-3.0) or the Jetson 40-pin header (I2C bus numbers `[U]`). |
| 4 | Which IMU chip is on the delivered driver board | Check on arrival (4.2). |
| 5 | Motor connector pin order, Yahboom cable vs Waveshare port | Verify before first power-up. |
| 6 | Track width, deck size, payload | Measure. |
| 7 | Camera wedge CAD (10–20° down, M2 holes) | To be modelled. Library printing is PLA-class filament; PETG not confirmed. |
| 8 | Barrel jack polarity | Meter the stock adapter. |
| 9 | Format of feedback messages T=1001 / T=1002 | Capture live from the board. |
| 10 | Amazon and Micro Center prices/stock for everything except B1 | Unreadable by the research tools; check manually. |
| 11 | Inline fuse holder, M2 camera screws | Not yet priced. |
| 12 | Power approach, storage picks, mount approach | Recommended `[R]`, awaiting Anshu's confirmation. |

---

## 9. Sources

**Jetson**
- Micro Center product page, SKU 812057, Westmont (PDF capture supplied by Anshu, 2026-09-17)
- https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html
- https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html
- https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/update_firmware.html
- https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/howto.html
- https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/supported_hardware.html
- https://forums.developer.nvidia.com/t/jetson-orin-nano-input-voltage/298625
- https://forums.developer.nvidia.com/t/powering-jetson-orin-nano-from-lipo-battery/294257
- https://forums.developer.nvidia.com/t/various-questions-on-the-jetson-orin-nano-super/346523
- https://forums.developer.nvidia.com/t/new-jetson-orin-nano-developer-kit-does-not-work-with-installed-aw-cb375nf-wifi-card/350957
- https://forums.developer.nvidia.com/t/slow-wi-fi-on-orin-nano-devkit-rtl8822ce-802-11ac/368697
- https://forums.developer.nvidia.com/t/unable-to-setup-intel-ax210-with-jetpack-6-2/336368
- https://forums.developer.nvidia.com/t/does-this-work-as-csi-camera-connector-for-orin-nano/285284
- https://forums.developer.nvidia.com/t/hdmi-on-devkit-broken-orin-nano-manual-link/249839
- https://hackaday.com/2023/03/21/hands-on-nvidia-jetson-orin-nano-developer-kit/
- https://jetsonhacks.com/2023/03/22/nvidia-jetson-orin-nano-developer-kit-the-perfect-solution-for-makers-and-developers-a-review/
- https://www.newegg.com/nvidia-jetson-orin-nano-super-developer-kit/p/N82E16813190033

**Driver board, servo, motors, lidar**
- https://www.waveshare.com/general-driver-for-robots.htm
- https://www.waveshare.com/wiki/General_Driver_for_Robots
- https://github.com/waveshareteam/ugv_base_general (`General_Driver/ugv_config.h`, `General_Driver/json_cmd.h`)
- https://www.waveshare.com/st3215-servo.htm
- https://www.waveshare.com/wiki/ST3215_Servo
- https://category.yahboom.net/products/smart-robot-car-chassis-kit
- https://www.yahboom.net/public/upload/upload-html/1740736196/0.%20Motor%20introduction%20and%20usage.html
- https://www.slamtec.com/en/c1/spec
- https://www.dfrobot.com/product-2803.html
- https://www.robotshop.com/products/slamtec-rplidar-c1-360-dtof-laser-scanner

**Camera, storage, cables**
- https://www.arducam.com/b0191-arducam-imx219-visible-light-fixed-focus-camera-module-nvidia-jetson-nano-raspberry-pi-compute-module.html
- https://www.sparkfun.com/arducam-imx219-visible-light-fixed-focus-camera-module-for-nvidiar-jetson-agx-orin-orin-nano-orin-nx.html
- https://www.adafruit.com/product/5818
- https://newegg.com/team-128gb-microsdxc/p/N82E16820985048
- https://www.newegg.com/team-group-256gb-mp33-nvme-1-3/p/N82E16820331415
- https://www.newegg.com/team-group-512gb-mp33-nvme-1-3/p/N82E16820331416
- https://plugable.com/products/dp-hdmi
- https://tensility.com/products/10-02220

**Power**
- https://us.ovonicshop.com/products/ovonic-50c-2200mah-3s1p-11-1v-xt60-2pcs-lipo-battery
- https://www.getfpv.com/1-8s-lipo-battery-voltage-tester-low-voltage-buzzer-alarm.html
- https://www.racedayquads.com/products/toolkitrc-c6-50w-5a-1-6s-compact-ac-charger-xt60
- https://chinahobbyline.com/products/cnhl-black-series-2200mah-14-8v-4s-40c-lipo-battery-with-xt60-plug
- https://www.pololu.com/product/3786
- https://www.waveshare.com/wiki/UPS_Module_3S
- https://www.waveshare.com/ups-power-module-c.htm
- https://blog.usedbytes.com/2019/03/battery-discharge-profile/
- https://www.kuruibms.com/blog/BMS-3S-Voltage-Guide.html

**Mount, sensors, misc**
- https://www.thingiverse.com/thing:7074577
- https://www.printables.com/model/653674
- https://www.scpld.org/use-your-library/3d-printing
- https://www.waveshare.com/2-axis-pan-tilt-camera-module.htm
- https://www.adafruit.com/product/3967
- https://www.waveshare.com/shopping_q_a
- https://www.salestaxhandbook.com/illinois/rates/saint-charles
