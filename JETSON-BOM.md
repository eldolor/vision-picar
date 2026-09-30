# Jetson build — BOM, and a kit-search brief

**Written 2026-09-17.** The Jetson became the recommended build on
measurement, not preference: P19 scored OWLv2 at **83% whole-pipeline
recall against the Pi tier's 50%**, on identical frames and gate, and P17
showed OWLv2 compiles to **no** Hailo. The price gap is **~$86**
(`BOM-COMPARISON.md`, verified listings). Reasoning lives in
`PLAN-onboard-perception.md` P17–P19; part numbers and vendors live in
`HARDWARE-BOM.md`.

> **Chassis status 2026-09-30:** the robot base is **not yet bought**.
> The Hiwonder ROSOrin ordered 09-29 is being cancelled or returned (it sends
> no encoder data to the Jetson and cannot power it at 25 W); the
> recommendation is the **Waveshare UGV Rover PT Jetson Orin ROS2 Kit Acce
> plus a separate Jetson battery**. Section 9 has the record;
> `GUIDE-robot-base.md` explains the concepts.

> **Status 2026-09-28.** The board decision **closed 2026-09-19 for the
> Jetson**, and the Hailo path is not being pursued (`CLAUDE.md` section 3).
> OWLv2 is no longer the reason for the board: P22-P24 found YOLOE matches it
> at a fraction of the latency, and the shipped detector is
> **`yoloe-11s-seg.pt`** (`brain/perceive.py`'s `DEFAULT_YOLOE`, 139 ms on a
> laptop CPU). The Jetson's case is that it runs whatever detector wins,
> plus the ROS 2 stack. The totals in sections 1, 2 and 6 were recomputed
> from their own rows on 2026-09-28; nothing is bought.

**This file is the shopping list for the recommended build**, and it is
written to double as a brief for a research assistant searching for a
**ready-made robot kit** so that the only thing to buy and plug in is the
Jetson.

**Where the BOM documents live**, so the next reader is not hunting:

| for | read |
|---|---|
| **what to buy** | **this file** |
| verified prices, Pi vs Jetson | `BOM-COMPARISON.md` |
| part numbers, vendors, wiring, bring-up order | `HARDWARE-BOM.md` |
| the Pi build (history -- no longer a fallback: the Hailo path closed 2026-09-19) | `PLAN-onboard-perception.md` 3.6 |
| why the parts are what they are | `PLAN-onboard-perception.md` 1, P17-P19 |
| ~~`BOM.md`~~ | superseded 2026-09-12 build, estimates only |

Below: what must be bought regardless, what a kit would replace, and the
constraints that disqualify most kits.

> **Note for the search.** `HARDWARE-BOM.md` section 1 lists bundled ROS 2
> robot kits (Yahboom ROSMASTER M3/X3/M1, Hiwonder LanderPi) under
> *"Rejected — do not propose these."* That constraint is **lifted for this
> search** — those kits are now explicitly in scope. Say so, or the
> previous brief will filter them out again.

---

## 1. Buy regardless of which kit wins — $470.94 (+ ~$10 USB flash if needed) + options

*Corrected 2026-09-28: this heading said $464.94, which no combination of the
rows produces. The rows below without the USB flash and the NVMe sum to
$470.94 -- `BOM-COMPARISON.md`'s Part B exactly; with the flash, $480.94. The
flash is only needed for the JetPack 7 USB installer (`HARDWARE-BOM.md` B8,
"not in totals"), and the software assumes JetPack 6.*

Nothing here is kit-replaceable. Prices `[V]` from `HARDWARE-BOM.md`.

| Item | Exact part | $ | Source |
|---|---|---|---|
| **Jetson Orin Nano Super Dev Kit** | NVIDIA 945-13766-0000-000 | **399.00** | Micro Center Westmont, in stock |
| CSI camera | **Arducam B0191** (IMX219) | **19.95** | SparkFun SEN-28170 |
| microSD 128GB A2 | Team PRO+ TPPMSDX128GIA2V3003 | **34.99** | Newegg |
| Barrel pigtail 5.5×2.5mm, 18AWG, ≥8A | Tensility 10-02220 | ~7.00 | DigiKey |
| DisplayPort cable (first boot only) | any DP 1.2 | ~10.00 | — |
| USB flash ≥16GB (JetPack installer) | any | ~10.00 | — |
| *Optional* NVMe SSD, M.2 **2280** | Team MP33 256GB | 68.99 | Newegg |

**The camera is not negotiable.** It must be **IMX219**, not the Raspberry
Pi Camera Module 3 (IMX708) — JetPack driver support for IMX708 is poor.
Many kits bundle a Pi-oriented camera; assume it is the wrong one.

---

## 2. What a kit would replace — ~$339 if bought piecemeal

*Corrected 2026-09-28: this said ~$290, and the rows summed to $361.79. Two
rows were also wrong against `HARDWARE-BOM.md`: the battery line double-counted
the 2-pack (2 x $29.99 + $14.99 = $74.97, against A6's one 2-pack + charger +
buzzer = $50.47), and the wiring line priced a 10A fuse that no BOM has priced
(A9 is ~$22 without it). With those fixed the rows sum to $339.29 -- which is
`BOM-COMPARISON.md`'s Parts A + C less the jumper wires (C4, $11.85), a
cross-check that the rows are now the same parts.*

This is the shopping list a kit needs to cover. A kit covering **rows
marked ★** is the win; the rest are cheap to add.

| ★ | Item | Spec that matters | piecemeal $ |
|---|---|---|---|
| ★ | **Chassis, 2WD** | **Differential drive** (skid steer). Aluminium. | 69.00 |
| ★ | **Encoder motors ×2** | Quadrature encoders, 12V class | incl. |
| ★ | **Motor/IO controller** | Waveshare General Driver for Robots (ESP32) | 27.99 |
| ★ | **Battery ×2 + charger** | **3S LiPo**, XT60, ~2200mAh (one Ovonic 2-pack, iMAX B3, LiPo buzzer -- A6) | 50.47 |
| ★ | 360° lidar | Slamtec **RPLidar C1**, USB | 69.00 |
| | Pan servo | Waveshare **ST3215**, 12V variant, bus servo | 21.99 |
| | Camera pan mount | 3D-printed bracket + wedge | ~5.00 |
| | Powered USB hub | Waveshare USB3.2-Gen1-HUB-4U, **7–36V in** | 17.99 |
| | ToF sensors ×2 | VL53L1X, forward-down | 29.90 |
| | Bumper + microswitches | Omron SS-5GL ×4 + springs | ~11.00 |
| | Wiring, XT60, switch | 14AWG pigtails, rocker switch, 20AWG wire (A9). **10A inline fuse + holder: not yet priced** | ~22.00 |
| | M2.5 standoff kit | brass (no M3 kit found -- A10) | 12.95 |
| | Motor-rail capacitor | ≥470µF electrolytic, 25V | ~2.00 |

---

## 3. Hard constraints — these disqualify most kits

Give these to the search verbatim. **The first one eliminates the
majority of hobby robot kits on the market.**

1. **Power must be 3S (11.1V) or higher — 2S/7.4V is disqualifying.**
   The Jetson's barrel jack spec is **9–20V** and it is fed straight from
   the pack with no buck converter. A 2S pack at 7.4V nominal is *below
   the board's floor*. Most two-wheel hobby kits ship 2S.
   Related: a 3S BMS cuts off at 8.4–9.0V, **at or below the Jetson's own
   floor**, so battery protection does not protect the board — the build
   needs a software cutoff regardless of kit.
2. **Differential drive only.** Two driven wheels plus caster, or 4WD skid
   steer. **No Ackermann / steering-servo platforms** — the whole software
   stack assumes pivot-in-place.
3. **Motors must have quadrature encoders.** Odometry is load-bearing, not
   optional. Kits without encoders are out.
4. **The motor controller must be reachable over USB or UART serial**, and
   its protocol must be documented or open. **A Raspberry Pi HAT is not
   usable** — the Jetson's header is different. Kits whose controller is a
   Pi HAT, or whose firmware is closed with no serial API, are out.
5. **Deck space for the Jetson devkit** — roughly 100 × 90 mm footprint,
   ~35 mm tall with its cooler, plus clearance for a barrel plug and a
   fan. *Verify the exact devkit dimensions against the kit's deck before
   buying.*
6. **A flat upper deck for a 360° lidar** with unobstructed 360° view at
   its scan plane, or a lidar already mounted that way.

**Four-wheel skid steer (the user's choice, 2026-09-27) adds items 7-11 of
the brief below**: an encoder per side at least, a controller that drives
all four motors, an IMU, a footprint no larger than ~25 × 22 cm, and the
track width and wheelbase. Why, in terms of the code: the simulator models
no wheel slip (`PLAN-ros-alignment.md` section 4), so skid steer's slip on
every turn will throw off turns that run until the encoders say they are
done; the ROS controller then needs `wheel_separation_multiplier` for the
larger "effective" track, and an IMU fixes heading at the source.
`robot/hardware_robot.py` speaks only the ESP32 board's protocol; and every
safety measurement of 3.18-3.19 was made for a 22.8 × 19.8 cm chassis.
The table above still says "Chassis, 2WD" -- it is the build this file
priced, not a requirement.

---

## 4. Strongly preferred, not disqualifying

- **Kit includes the lidar** — RPLidar C1 or equivalent 360° USB scanner.
  Worth up to ~$70 of the kit's price.
- **Kit includes 3S packs and a charger.** If the pack spec is not
  published, treat it as absent and buy separately — that was the reason
  the standalone chassis was chosen at $69 over the $79 with-battery version.
- **Bus servo support** for camera pan, ideally ST3215-compatible.
- **Single vendor, single shipment.** Paying ~$5 more beats a second
  shipping charge.

---

## 5. What a kit will almost certainly NOT include

Budget for these on top of any kit price:

- The **IMX219 camera** (kits bundle Pi-oriented cameras — §1)
- **ToF sensors and a bumper** — rarely present, and both are
  safety-relevant here
- **NVMe SSD**, if wanted
- The **barrel pigtail** to feed the Jetson from the pack
- A **low-voltage buzzer** (set 3.4V/cell) — the only hardware
  over-discharge protection in this design

---

## 6. Totals to beat

From `BOM-COMPARISON.md`, verified listings, 8.75% tax and shipping
included:

| build | all-in |
|---|---|
| Jetson, piecemeal (Part A + B + C, no NVMe) | **~$944** |
| Jetson, piecemeal, with NVMe | ~$1,019 |

**A kit is worth buying if `$471 (§1) + kit + §5 extras` lands under
~$944.** (Was `$465`, corrected 2026-09-28 with §1. Note the $471 is before
tax and shipping, the ~$944 after.) Below ~$850 it is clearly better; above ~$1,000 the piecemeal
build wins on parts we have already verified.

---

## 7. Two open risks a kit does not remove

- **JetPack 6 vs 7.** The board's entire value rests on running OWLv2 in
  PyTorch, which needs a working `torch` + `transformers` wheel for the
  installed L4T. **Confirm a working torch wheel before committing an SD
  card**, and prefer the JetPack that has one today over the newer one.
  *(2026-09-27: the software already assumes **JetPack 6.x / Ubuntu 22.04 /
  ROS 2 Humble** -- `service/slam/Dockerfile` is `ros:humble-ros-base` (R3).
  JetPack 7 would mean moving that container to Jazzy. Treat 6.2.1 as the
  working choice unless the torch-wheel check fails on it. The detector is
  now `yoloe-11s-seg`, not OWLv2, but it still needs torch.)*
- **Latency is unmeasured.** OWLv2 is projected at roughly **5 Hz** on this
  board (fp16; INT8 destroys it — P7d), against the Hailo path's 92 FPS.
  That may mean a fast cheap detector for obstacle reaction alongside
  OWLv2 for deciding where to go. It is a design question for hardware
  day, not a purchase blocker — but it is not resolved.
  *(2026-09-28: superseded in part. The Hailo path closed 2026-09-19, so
  92 FPS is not an alternative on the table. The shipped detector is now
  `yoloe-11s-seg` -- 139 ms on a laptop CPU, against OWLv2's ~2.7 s there
  (P22-P24) -- so OWLv2's 5 Hz is no longer the number to worry about. What
  stays true is that latency ON THE BOARD is unmeasured; P7b found CPU
  preprocessing, not the model, was the projected bottleneck.)*

---

## 8. The research brief, for a web-research assistant

Kept here rather than in a chat log for the same reason the Alexa prompts
live in `PLAN-onboard-perception.md` 3.6.1: **a part swap and the prompt
that searches for it must not be able to drift apart.** If §3's
constraints change, change this too.

Self-contained on purpose — the assistant has no access to this repo.

> I'm building an indoor autonomous robot around an **NVIDIA Jetson Orin
> Nano Super Developer Kit**, which I'm buying separately. I want to find a
> **ready-made robot kit** that supplies everything else, so that ideally I
> only have to mount the Jetson and plug it in.
>
> **Please search for and compare candidate kits. Include bundled ROS 2
> robot kits — Yahboom ROSMASTER, Hiwonder LanderPi, and similar — they are
> explicitly in scope.**
>
> **The kit must satisfy all of these. Treat any one as disqualifying:**
>
> 1. **Battery is 3S (11.1V nominal) or higher.** The Jetson is powered
>    straight from the pack through a 9–20V barrel jack with no regulator,
>    so a 2S/7.4V pack is below the board's floor. This is the constraint
>    most kits fail — please check it first and say what pack each kit
>    ships.
> 2. **Differential drive** — two driven wheels plus a caster, or 4WD skid
>    steer. **No Ackermann or steering-servo platforms.**
> 3. **Motors have quadrature encoders.** Kits without encoders are out.
> 4. **The motor controller is reachable over USB or UART serial, with a
>    documented or open protocol.** A Raspberry Pi HAT is not usable — the
>    Jetson's header is different. Closed firmware with no serial API is
>    out.
> 5. **Deck space for the Jetson devkit** — roughly 100 × 90 mm, about
>    35 mm tall with its cooler, plus clearance for a barrel plug.
> 6. **A flat upper deck with an unobstructed 360° view** for a lidar, or a
>    360° lidar already mounted that way.
>
> **I want a FOUR-WHEEL skid-steer chassis. For those, also:**
>
> 7. **One encoder per side at minimum, all four preferred.** Say how many
>    of the motors have encoders.
> 8. **The motor controller must drive all four motors** -- four channels,
>    or left and right pairs wired together. Say how many motor channels
>    the controller has, and whether the Waveshare General Driver for
>    Robots (ESP32) can drive this chassis.
> 9. **An IMU on the board or the controller, strongly preferred.** Skid
>    steer slips sideways on every turn, so heading from the wheel
>    encoders alone is poor. Say whether one is included.
> 10. **Footprint no larger than about 25 × 22 cm, wheels included.** It
>     must turn in place between furniture, and the outer wheel corners
>     set the turning circle. Give the outer dimensions.
> 11. **Give the track width and the wheelbase** (left-right and
>     front-rear wheel spacing). Skid-steer handling depends on their
>     ratio.
>
> **Strongly preferred, not disqualifying:** a 360° USB lidar included
> (worth up to ~$70); 3S packs and charger included, with the pack spec
> actually published; bus-servo support for a camera pan axis; everything
> from one vendor in one shipment.
>
> **Please assume the kit does NOT include, and tell me if it does:** a
> camera (I need **IMX219** specifically — *not* the Raspberry Pi Camera
> Module 3 / IMX708, whose Jetson driver support is poor, so assume any
> bundled camera is the wrong one); VL53L1X time-of-flight sensors; a
> bumper with microswitches; an NVMe SSD.
>
> **Budget.** Buying every part separately comes to about **$944 all-in**
> (tax and shipping included), of which **about $471 (before tax) is the
> Jetson side I'm buying anyway**. So a kit is worth it if *$471 + kit + the missing items
> above* lands under ~$944; clearly worth it under ~$850.
>
> **For each candidate please give me:** exact product name and SKU, price,
> vendor, **current stock and lead time**, what it includes against the
> list above, what's missing, and an all-in total including the gaps. Tag
> each figure `[V]` if you read it from a vendor page, `[I]` if inferred or
> estimated, `[U]` if unverified. **Please flag anything on backorder** —
> I was recently caught by a part that was four-plus weeks out.
>
> If nothing clears the constraints, say so plainly and tell me which
> constraint each near-miss failed; buying the parts separately is a
> perfectly good outcome.

---

## 9. The chassis search, 2026-09-27 to 09-30 -- what was found and decided

The brief in section 8 was run, then widened to every vendor's full
catalogue and to third-party retailers. **The concepts behind this section --
encoders, firmware openness, vendor protocols, power budgets -- are explained
in `GUIDE-robot-base.md`**; this section is the record.

### 9.1 Where it stands

| date | event |
|---|---|
| 09-27 | UGV Rover assumed as the chassis; sim, safety and nav2 switched to it and measured (`PLAN-ros-alignment.md` 3.21). |
| 09-29 | **Hiwonder ROSOrin Advanced (no controller) ordered** on Amazon, $579.99, ASIN B0G2GPKZGZ, ships from Amazon, free 30-day return. |
| 09-29 | Reading Hiwonder's driver source: its board **reports no encoder data** to the host. |
| 09-30 | Hiwonder confirmed it, and more (9.3). **Under the rule set before asking, the ROSOrin is to be cancelled or returned.** |
| 09-30 | Waveshare answered every open question about the UGV Rover (9.3). |
| -- | **Recommended, not yet bought: Waveshare UGV Rover PT Jetson Orin ROS2 Kit Acce + a separate Jetson battery.** Route (Amazon or Waveshare direct) is the user's call. |

### 9.2 Everything evaluated

Prices without a computer, verified on the vendor's page unless tagged.

| candidate | price | verdict |
|---|---|---|
| **Waveshare UGV Rover PT Jetson Orin ROS2 Kit Acce** | $539.99 direct; $675.99 Amazon (third-party, no Prime) | **Recommended.** Closed loop, measured odometry to the host, open firmware, our code speaks its protocol. Power tight: add a Jetson pack. 253 x 231 mm. |
| **Waveshare Cobra Flex** (bare chassis) | $319.99 direct | **Runner-up.** 235 x 173 mm (inside the size target); battery DC output meant for a Jetson; reports 4 wheel speeds + odometry; open firmware; 12 kg payload. No IMU listed; sensors and brackets are DIY; not on Amazon. ~$510-555 as a lean build. |
| Hiwonder ROSOrin Advanced | $529.99 direct, $579.99 Amazon | **Rejected 09-30** (9.3): no encoder data to the host, proprietary firmware, Jetson port can't sustain 25 W. |
| Waveshare UGV02 (the Rover's bare chassis) | $149.99 | Same power board; 2 encoders; needs every sensor added. |
| Waveshare UGV Beast (tracked) | $369.99+ | Closed loop out of the box, but tracks slip on every turn. |
| Waveshare WAVE ROVER | $89.99 | No encoders. |
| Yahboom ROSMASTER M1/M3/X3/X3 Plus/A1/R2 | -- | Mecanum or Ackermann. |
| Yahboom Transbot SE | $279.99 | Tracked; Jetson Nano / Pi only; no lidar. |
| Hiwonder JetRover (tank), JetAuto, JetAcker, ROSOrin Pro | $769.99+ | Too big, bundled arm, or wrong drive type. |
| ROBOTIS TurtleBot3 Burger | $681-784 (includes a Pi) | ~1,800 mAh battery: too small for a 25 W Jetson. |
| Husarion ROSbot 3 / XL; TurtleBot 4; AgileX LIMO Pro; Elephant myAGV | EUR 2,749+ / ~$1,195+ / $2,799 / $4,765 | Excellent but 3-8x the budget, or bundle their own Jetson. |
| Micro Center "Hiwonder ROSOrin" | $299.99 | Part 21031708 = the **Starter** tier: mecanum only, no depth camera. |

Retailers swept: OpenELAB and ThinkRobotics (full catalogues; ThinkRobotics
prices in INR), Micro Center (blocked automated access; checked from a user
PDF), RobotShop, Generation Robots, DFRobot, Seeed (web search).

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
* SKU 29227 ships the **ROS Driver for Robots** board (`ugv_base_ros`),
  closed-loop speed control.
* **660 pulses per wheel revolution**; **two** encoder channels, one left and
  one right.

**Hiwonder support** (Zora, 09-30):

* "The STM32 firmware currently **does not support reporting motor data or
  encoder feedback back to the host**."
* "The STM32 controller firmware is **proprietary**, and the source code is
  not open-source."
* Encoder resolution: "we do not provide the specific encoder resolution
  data."
* Jetson power: "it can power the Jetson Orin Nano Super, but it **cannot
  supply enough current** to support the board running continuously at full
  load (25W mode)."
* Lidar: D500 from the China warehouse; Amazon stock mixes D500 and MS200.
  A charger is included.

### 9.4 Two things this section corrects

* **The ROSOrin's "four encoders" were never usable by the Jetson.** It was
  recommended on 09-29 partly for them, before its driver source was read.
  The lesson is in `GUIDE-robot-base.md` section 9: closed loop and
  "reports to the host" are different claims.
* **"Sold with an Orin Nano Super fitted" did not mean "powers it at 25 W".**
  Hiwonder says it doesn't. Get power claims in writing.

### 9.5 If the Rover is bought

* Add a **separate Jetson battery** (e.g. Wheeltec E351S, 3S 5100 mAh with a
  protection board and charger, ~EUR 85), an inline ~5 A fuse and a 5.5 x 2.1
  to 5.5 x 2.5 mm barrel adapter; mount it on a Picatinny rail clamp or a
  printed tray. ~$100-110 extra.
* Use **4C-rated 18650s** in the Rover's own holder.
* In code: the encoder constant becomes **660** (the sim and backends still
  carry 1650 from 3.21), and `robot/hardware_robot.py` should read the ROS
  Driver's `odl`/`odr` odometry rather than integrating wheel speeds.
