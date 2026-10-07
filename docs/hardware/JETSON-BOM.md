# Jetson build: what was bought, and what is left

**As of 2026-10-06:** the robot is a **Waveshare UGV Rover PT Jetson Orin
ROS2 Kit Acce** carrying an **NVIDIA Jetson Orin Nano Super Developer
Kit**.

* **Jetson:** bought, measured and kept (2026-10-05).
* **Rover:** ordered 2026-09-30, due Oct 19 - Nov 11, with a 30-day
  return.
* **Still conditional:**
  * a separate Jetson battery, decided by a stress test on arrival (9.5);
  * an adapter plate, if the devkit does not bolt onto the Rover's deck
    (section 7).

The board was chosen on measurement: OWLv2's 83% whole-pipeline recall
against the Pi tier's 50% (P19), and OWLv2 compiles to no Hailo (P17). The
Hailo / Pi path closed 2026-09-19. The shipped detector has since become
YOLOE, which the Jetson runs at 61 ms a frame. The Jetson's case now is that
it runs whatever detector wins, plus the ROS 2 stack.

The 2026-09-17 price plan, the kit-search shopping list and its research
brief are archived verbatim in `docs/archive/JETSON-BOM-2026-10-06.md`.

| for | read |
|---|---|
| what was bought, and what is left | **this file** |
| the concepts (encoders, firmware, vendor protocols, power) | `GUIDE-robot-base.md` |
| verified prices, Pi vs Jetson | `BOM-COMPARISON.md` |
| the ESP32 board's JSON protocol, udev, `dialout`, bring-up order | `HARDWARE-BOM.md` (read its editor's note first) |
| the Rover's deck and the devkit's fit | `UGV-ROVER-MOUNTING.md` |
| the board's measured results | `PLAN-ros-alignment.md` 3.33-3.37 |

---

## 1. What was bought

| Item | Part | Price | Ordered | State |
|---|---|---|---|---|
| Jetson Orin Nano Super Developer Kit | NVIDIA 945-13766-0000-000 | $399 | 2026-09-27, Amazon | Arrived 09-30, opened 10-02, **kept 10-05** |
| Robot base | Waveshare UGV Rover PT Jetson Orin ROS2 Kit Acce, SKU 29227 | ~$730 delivered | 2026-09-30, Amazon | Due Oct 19 - Nov 11; 30-day return |
| NVMe | SanDisk Optimus 5100 500 GB, SDSP51500GAN (M.2 2280, PCIe 4.0) | $110 + tax | 2026-10-03, Amazon | Installed; the board boots from it since 10-05 |
| microSD + USB-C reader | 128 GB A2 / U3 / V30 | -- | 2026-10-02 | The fallback boot device |
| Rover cells | 4x Molicel P26A 18650 (flat-top, unprotected, 2600 mAh, 35 A) | $34 shipped | 2026-10-03, IMR Batteries | Three go in the pack, one is a spare (9.5) |

**What the Rover kit brings**, replacing the 2026-09-17 plan's separate
parts:
* the ROS Driver board (an ESP32, closed-loop speed, encoder odometry
  `odl`/`odr` to the host, 660 pulses per wheel turn);
* a D500 lidar (replacing the RPLidar C1);
* an OAK-D Lite depth camera;
* a pan-tilt 5 MP camera (replacing the IMX219 CSI camera);
* the UPS power board and its 12.6 V charger.

Which camera feeds perception is still to be decided on recorded walks
(`GUIDE-robot-base.md` section 8).

**Conditional, not yet ordered:**
* a separate Jetson battery and its fused cable, about $100-110, only if
  9.5's stress test says so;
* an adapter plate and standoffs, about $15-40, only if section 7's
  caliper check says so.

---

## 2-6. The kit search's requirements (closed 2026-09-30)

Sections 2-6 held what a kit had to replace, its hard constraints, its
preferences, what kits leave out, and the totals to beat (~$944 all-in
piecemeal). The search is closed; they are archived verbatim. The
constraints still explain why the Rover was chosen:

1. **3S power (11.1 V) or higher.** The Jetson takes 9-20 V straight from
   the pack, so 2S is below its floor. A 3S pack's protection cuts off at
   or below that floor too, so the build needs a software cutoff anyway.
2. **Differential drive** (two wheels and a caster, or skid steer). No
   Ackermann: the simulator, the pivot guard, the search turns and nav2's
   fit all assume a robot that turns in place.
3. **Quadrature encoders whose data reaches the host.** Closed loop on the
   board is not the same claim (9.4).
4. **A motor controller on USB or UART serial with an open protocol.** No
   Pi HAT, and no closed firmware without a serial API.
5. **Deck space and a hole pattern that take the Developer Kit.** Space is
   not fit (section 7).
6. **An unobstructed 360-degree deck for the lidar.**
7. **For skid steer:** an encoder per side at least, a controller that
   drives all four motors, an IMU, a footprint near 25 x 22 cm, and the
   track and wheelbase published.

---

## 7. Open risks

**Closed on the board 2026-10-04** (`PLAN-ros-alignment.md` 3.33):

- **A working torch on JetPack 6.** JetPack 6.2.1 runs torch 2.8.0 on
  `cuda` from the Jetson AI Lab index, which needs `numpy<2` (pinned in
  `tools/jetson/setup.sh`). The pipeline agrees with the laptop on 63/63
  pinned frames. Staying on JetPack 6 keeps the Humble container. JetPack 7
  would mean Jazzy or Lyrical and re-proving the ROS stack.
- **Latency on the board.** The shipped pipeline at 15 W runs at a median
  60.6 ms and p90 109.9 ms a frame, against a 250 ms budget. 25 W bought
  nothing, because it caps the CPU lower. CPU image handling is about 19 ms,
  not P7b's projected 229 ms.

**Open: the Developer Kit's fit in the Rover.**
- **What the CAD shows.** Waveshare's STEP assembly models the chassis
  around its own JETSON-ORIN-IO-BASE carrier (90.5 x 103.0 mm) with a bare
  module, not NVIDIA's Developer Kit (100 x 79 mm), which is what we own.
- **Which holes could take it.** NVIDIA publishes no hole pattern. The
  devkit's 79 mm width rules out the deck's outer 86 x 81.61 mm rectangle,
  leaving the 86 x 58.00 and 86 x 23.61 sub-spans (`UGV-ROVER-MOUNTING.md`
  section 4).
- **What the video suggests.** Waveshare's Acce assembly video shows a
  Jetson the user judges to look like ours, which makes a direct fit
  likelier. It also shows the Jetson's holes carrying the top plate, so an
  adapter would carry the lidar and camera too (`UGV-ROVER-MOUNTING.md`
  section 7).
- **If it does not fit:**
  * an adapter plate (9.5);
  * a check that the kit's UART and power leads reach the devkit's 40-pin
    header and barrel jack;
  * a re-measure of the URDF's `[CAD]` heights (`laser_z`, `pan_z`,
    `camera_up`) if the upper deck rises.

  **It is not a reason to return the Rover.**
- **Owner and deadline.** The user, with calipers on the devkit already on
  the desk. Due before the Rover's earliest arrival (Oct 19); the hard stop
  is its return window, Nov 18 at the earliest. Waveshare support has the
  question (sent 2026-10-06 on the 257511 thread; support logged it as
  request 258472, unanswered as of 10-07).

---

## 8. The research brief (archived)

The self-contained prompt used for the kit search
(`docs/archive/JETSON-BOM-2026-10-06.md` section 8). Kept because a part
swap and the prompt that searched for it must not drift apart. Reuse it only
if the chassis is ever re-opened.

---

## 9. The chassis search, 2026-09-27 to 09-30, and since

The concepts behind this section are explained in `GUIDE-robot-base.md`;
this is the record.

### 9.1 Where it stands

| date | event |
|---|---|
| 10-07 | Waveshare's Amazon seller account re-answered the 25 W question: still "not verified"; at 25 W use cells of 4C or above. Nothing changes: the Jetson runs at 15 W, and the P26A cells are ~13C (9.3, 9.5). |
| 10-06 | Waveshare's CAD read: the kit is modelled around its own carrier, not the Developer Kit. Fit unconfirmed; caliper check owed before Oct 19 (section 7). Asked support: request 258472. |
| 10-05 | **Jetson KEPT.** Boots from the NVMe. G4 met on the fork firmware; headroom met with 3.37. |
| 10-04 | **Both board risks closed** (section 7). Offline suite 1728 passed, 0 failed, on the board. |
| 10-03 | Ordered the NVMe and the Rover's cells (section 1). |
| 10-02 | Jetson opened for bring-up before the Rover (3.33); microSD and card reader ordered. |
| 09-30 | **Ordered: the UGV Rover on Amazon, ~$730 delivered** (9.7). |
| 09-30 | Hiwonder ROSOrin order cancelled after Hiwonder confirmed its board reports no encoder data (9.3). |
| 09-30 | Waveshare answered every open question about the Rover (9.3). Cobra Flex firmware read (9.6). Yahboom ROSMASTER A1 rejected (Ackermann). |
| 09-29 | ROSOrin ordered; its driver source then showed no encoder data to the host. |
| 09-27 | The UGV Rover assumed as the chassis; the simulator, safety and nav2 switched to it and measured (`PLAN-ros-alignment.md` 3.21). |

### 9.2 Everything evaluated

| candidate | price, no computer | verdict |
|---|---|---|
| **Waveshare UGV Rover PT Jetson Orin ROS2 Kit Acce** | $539.99 direct; $675.99 Amazon | **Ordered.** Closed loop, measured odometry to the host, open firmware, a protocol our code already speaks. Power is tight (9.5). 253 x 231 mm. |
| Waveshare Cobra Flex | $319.99 direct | Runner-up: better drivetrain and power, but no IMU and DIY sensor mounts (9.6). |
| Hiwonder ROSOrin Advanced | $529.99 direct, $579.99 Amazon | Rejected: no encoder data to the host, proprietary firmware, Jetson port cannot sustain 25 W. |
| Waveshare UGV02 / UGV Beast / WAVE ROVER | $149.99 / $369.99+ / $89.99 | Every sensor to add / tracks slip on every turn / no encoders. |
| Yahboom ROSMASTER range, incl. A1 | -- | Mecanum or Ackermann: cannot pivot. |
| TurtleBot3 Burger; Husarion, TurtleBot 4, AgileX, myAGV | $681+; EUR 2,749+ | Battery too small for a 25 W Jetson; or 3-8x the budget. |

The full table, with ASINs and the retailers swept, is in the archive copy.

### 9.3 What the vendors said, verbatim where it decides something

**Waveshare support** (tickets 257272, 257427, 257511; 09-29 and 09-30):

* The kit's UPS **is** the standalone UPS Module 3S; "12.6V / 2A ... refers to
  the charger specification and does not mean that the UPS output is limited
  to 2A."
* No fixed continuous rating; overcurrent protection at ~7.5-12.5 A; "the
  normal continuous output current can reach up to **5 A**". Use 18650s of
  **4C or higher**, or "power the Jetson directly from an external battery
  pack".
* The Jetson is fed directly from the UPS through a **DC5525** connector.
  25 W MAXN SUPER with all peripherals is **untested, not guaranteed**.
* 2026-10-07, Waveshare's **Amazon seller** account (not support): "It comes with the ROS Driver board. We
  have not verified the situation you mentioned, but at **25W** power
  consumption, we recommend using a battery with a discharge rate of **4C or
  above**." This restates the 09-30 answers; 25 W is still unverified.
* SKU 29227 ships the **ROS Driver for Robots** board (`ugv_base_ros`),
  closed-loop speed control.
* **660 pulses per wheel revolution**; **two** encoder channels, one left and
  one right.

**Hiwonder support** (09-30):

* "The STM32 firmware currently **does not support reporting motor data or
  encoder feedback back to the host**."
* "The STM32 controller firmware is **proprietary**, and the source code is
  not open-source."
* Jetson power: "it can power the Jetson Orin Nano Super, but it **cannot
  supply enough current** to support the board running continuously at full
  load (25W mode)."

### 9.4 Three lessons the search taught

* **Closed loop is not "reports to the host".** The ROSOrin's four encoders
  hold wheel speed on the board and never reach the Jetson
  (`GUIDE-robot-base.md` section 9).
* **"Sold with an Orin Nano Super fitted" did not mean "powers it at 25 W".**
  Get power claims in writing.
* **"Jetson Orin kit" did not mean "takes the Developer Kit".** Measure the
  fit (section 7).

### 9.5 On the Rover's arrival

**Power: the separate Jetson battery is CONDITIONAL.**
- **Why it may not be needed.** The Jetson runs at 15 W (user,
  2026-10-01). At about 11 V that is ~1.4 A, or ~2.7 A with the motors'
  ~15 W peak, inside the 5 A Waveshare says the UPS sustains.
- **The test that decides.** Run the board at 15 W with the motors working
  hard, logging the Jetson's input voltage. Buy the battery only if it
  sags, or before ever moving to 25 W. (15 W costs perception nothing: 3.33
  measured 60.6 ms at 15 W against 64.2 ms at 25 W.)
- **If needed:**
  * a pack such as the Wheeltec E351S (3S 5100 mAh, protection board,
    charger, ~EUR 85);
  * a **fused male 5.5 x 2.1 to male 5.5 x 2.5 mm cable** (~5 A fuse);
  * **unplug the kit's own DC5525 Jetson lead** so the two supplies are
    never joined;
  * mount the pack on a Picatinny clamp or a printed tray.

**Cells.**
- **What the Rover takes.** It ships without cells: 3x 18650 in series,
  >= 2200 mAh, >= 4C, flat-top, unprotected.
- **Which to fit.** Install three Molicel P26A from the same order as a
  matched set; keep the fourth as an emergency spare.
- **Before the first drive.** Inspect the wraps (no tears near the + end,
  no dents), match the holder's polarity, and charge fully on the kit's
  12.6 V charger.
- **Waking the UPS.** It stays off after the cells go in until the charger
  is plugged in; then the power button works.
- **Reading the pack.** Once the Jetson is connected, the pack voltage is
  the `v` field of `T:1001` (~12.6 V full; charge below ~10.5 V).

**Checks within the 30-day return window:**
- [ ] The 15 W stress test above.
- [ ] `T:1001` carries `odl`/`odr`, and odometry is checked over a measured
      metre.
- [ ] The safety sweep against the real lidar.
- [ ] Read the ESP32 module's shield. Expected: ESP32-WROOM-32, the
      original ESP32, inferred from the firmware's pin map.
- [ ] Find which serial route the kit wires, for `ROBOT_SERIAL`. Expected:
      the 40-pin header's UART, `/dev/ttyTHS1`; the assembly video wires a
      double-row cable there, so check it reaches the Developer Kit's
      header. USB through a bridge chip would be `/dev/ttyUSB0`.
- [ ] **Disable Waveshare's own software**: the stock `ugv_jetson` app and
      any `ugv_bringup` / `ugv_driver` service. Only one program can hold
      the serial port, and theirs drive the motors without
      `robot/safety.py` (`PLAN-ros-alignment.md` 3.26).
- [ ] **Measure the `[CAD]` geometry** (3.27):
  * the lidar's offset ahead of the wheel centre (CAD 4.0 cm; safety
    depends on it), and its height;
  * the pan axis and the lens;
  * which way the D500's zero faces (CAD: turned 90 degrees, left), before
    the first scan is used.
- [ ] Dump the stock ESP32 image (`esptool.py read_flash`).
- [ ] Only after the checks above, flash our fork (`firmware/ugv_base_ros/`,
      `build.sh`; GPL-3.0 allows it).

**Already done in code:**
* 660 pulses per turn everywhere (3.25);
* `sim/fake_esp32.py` is the ROS Driver's firmware;
* `robot/hardware_robot.py` anchors odometry on `odl`/`odr`, or on the
  fork's 0.1 mm odometers when present (3.28-3.29).

### 9.6 The Cobra Flex, from its firmware source

Read 2026-09-30 from Waveshare's `Cobra_Flex0519.zip`; the runner-up.

* **No IMU.** The firmware's IMU functions are empty stubs, and the
  feedback frame's IMU fields are commented out. **Trap:** with the arm
  module fitted, `ax`/`ay`/`az` carry the arm's coordinates, not
  acceleration.
* **Odometry is measured** from its hub motors' absolute positions, but
  `odl`/`odr` go out in whole centimetres.
* Its speed command differs from the Rover's, so `robot/hardware_robot.py`
  would need a Cobra variant.
* **In its favour:** a 3S2P bay, 9-28 V input, and a battery-direct DC5525
  lead for the Jetson. One larger pack may run everything.

It is sold only direct from China. The full read is in the archive copy.

### 9.7 The buying route

**Decided 2026-09-30: Amazon, ~$730 delivered.** Waveshare direct came to
$656.45 delivered, but with a 15-day return to China and Illinois use tax
(~$67) still owed, so the real gap was under $10. Amazon's 30-day return to
a US address is long enough to run 9.5's checks.

The rule, set before the answer: buy direct only if duties were prepaid
**and** returns went to a US address. Direct failed on returns alone.
