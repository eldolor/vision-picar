# BOM comparison: Raspberry Pi 5 + Hailo-8L vs Jetson Orin Nano Super

Prices read from retailer pages **2026-09-17**. Ship to / pick up near
St. Charles, IL 60174. Sales tax **8.75%**. **Nothing has been purchased.**

This file exists to answer one question -- *what does each option cost, like
for like* -- and nothing else. For part numbers, vendors, bring-up order and
the driver-board protocol, see **`HARDWARE-BOM.md`**. For why the parts are
what they are, see **`PLAN-onboard-perception.md`** sections 1, 3.6 and 3.6a.

Tags follow `HARDWARE-BOM.md`: `[V]` read from a vendor page, `[I]` inferred
or estimated, `[U]` unverified.

---

## 1. The answer

| Build | Pi 5 + Hailo-8L | Jetson Orin Nano Super | Delta |
|---|---|---|---|
| Essential (Part A + B) | **~$781** | **~$867** | +$86 |
| **+ recommended (A + B + C)** | **~$858** | **~$944** | **+$86** |
| + NVMe 256GB | **not possible** -- see 4.2 | ~$1,019 | -- |

All-in: subtotal + 8.75% tax + shipping ($58 Pi / $50 Jetson -- the Jetson
board is in-store pickup at Micro Center Westmont, the Pi ships).

**The Jetson costs about $86 more** for the build that can actually be bought
today, or **$59** more if the out-of-stock Pi parts are counted (4.6). Call the
honest range **$59-110**. A "$220-300" figure appears elsewhere in the repo and
is an artifact of mixing estimated and verified prices -- see 4.6. `PLAN-onboard-perception.md` 4.7 set
**~$170** as the delta at which the Jetson is worth re-opening; this is well
inside it.

---

## 2. Identical on both paths -- $351.14

Chassis, drive, sensing and power do not change with the compute choice.

| # | Item | $ | Source |
|---|---|---|---|
| A1 | Slamtec RPLidar C1 (C1M1-R2) | 69.00 `[V]` | DFRobot or Seeed |
| A2 | Yahboom 2WD encoder chassis (ASIN B0F3CZ3WYB, no battery) | 69.00 `[V]` | Yahboom store |
| A3 | Waveshare General Driver for Robots (ESP32) | 27.99 `[V]` | Waveshare |
| A4 | Waveshare ST3215 bus servo, **12V variant** | 21.99 `[V]` | Waveshare |
| A5 | Camera pan mount, 3D printed | ~5.00 `[I]` | St. Charles Public Library, $0.10/g |
| A6 | 2x Ovonic 3S 2200mAh + iMAX B3 charger + LiPo buzzer | 50.47 `[V]` | Ovonic, GetFPV |
| A9 | Wiring, rocker switch, XT60 pigtails | ~22.00 `[V]` | RaceDayQuads, Pololu, BNTECHGO |
| A10 | M2.5 standoff kit | 12.95 `[V]` | PiShop |
| A11 | Motor-rail electrolytic + TVS | ~2.00 `[V]` | DigiKey / Newark |
| | **Part A subtotal** | **280.40** | |
| C1 | 2x VL53L1X ToF (Adafruit #3967, exposes XSHUT) | 29.90 `[V]` | Adafruit |
| C2 | 4x Omron SS-5GL + spring assortment | ~11.00 `[V]` | Arrow, Harbor Freight |
| C3 | Waveshare USB3.2 4-port hub, **7-36V input** | 17.99 `[V]` | Waveshare |
| C4 | Jumper wire sets | 11.85 `[V]` | Adafruit |
| | **Part C subtotal** | **70.74** | |
| | **Common total** | **351.14** | |

---

## 3. Part B -- where they differ

### 3.1 Raspberry Pi 5 + Hailo-8L -- $384.04

| Item | $ | Source / note |
|---|---|---|
| Raspberry Pi 5, 8GB | 175.00 `[V]` | PiShop or CanaKit. **Was $80** -- three DRAM-driven rises since Dec 2025 |
| Active Cooler | 10.95 `[V]` | PiShop. Required; the Pi 5 throttles without it |
| **Hailo AI HAT+ 13T** (soldered) | 76.95 `[V]` | PiShop. **The only Hailo-8L in stock anywhere** -- see 4.1 |
| Camera Module 3 | 29.25 `[V]` | PiShop |
| Pi 5 camera cable | 3.95 `[V]` | PiShop. The Camera Module 3 box ships only the 15-pin |
| microSD 128GB A2 | 34.99 `[V]` | Newegg, Team PRO+ (same part as the Jetson column) |
| Buck converter 5V/5A | 39.95 `[V]` | Pololu D36V50F5. The Pi needs a regulated 5V rail off the 3S pack |
| USB-C power pigtail | ~5.00 `[I]` | Buck output to the Pi |
| micro-HDMI cable | ~8.00 `[I]` | Bring-up only |
| Wi-Fi / BT | 0.00 | On board |
| *NVMe SSD* | **unavailable** | See 4.2 |
| **Subtotal** | **384.04** | |

### 3.2 Jetson Orin Nano Super -- $470.94

| Item | $ | Source / note |
|---|---|---|
| **Jetson Orin Nano Super Developer Kit** (8GB) | 399.00 `[V]` | Micro Center Westmont, SKU **812057**, in-store pickup, 7 in stock at capture. NVIDIA list since July 2026 (was $249) |
| Wi-Fi + BT | **0.00** `[V]` | **RTL8822CE pre-installed**, two antennas in the base |
| Arducam B0191 (IMX219) | 19.95 `[V]` | SparkFun. Ships both 22-22 and 22-15 cables |
| microSD 128GB A2 | 34.99 `[V]` | Newegg, Team PRO+. Required even with an NVMe (firmware-update medium) |
| DisplayPort cable | ~10.00 `[I]` | The devkit has **no HDMI** |
| Barrel pigtail 5.5x2.5mm, 18AWG | ~7.00 `[I]` | USB-C is **debug/data only**. A 5.5x2.1mm plug will not seat |
| Cooling | 0.00 | Included |
| Accelerator / carrier | 0.00 | Not needed |
| 5V buck | 0.00 | Takes **9-20V** from the pack directly |
| *NVMe 256GB (optional)* | *68.99* `[V]` | *Newegg, Team MP33, M.2 2280* |
| **Subtotal** | **470.94** | |

---

## 4. What the $86 does not show

### 4.1 The Pi's accelerator line is a compromise, not a choice

`PLAN-onboard-perception.md` 3.6 specifies the **bare Hailo-8L M.2 module**,
explicitly *not* the soldered AI HAT+, because the module *"survives a Jetson
pivot and is the form the NVMe needs anyway."*

**That module cannot be bought.** The AI Kit that bundled it is discontinued
(CanaKit sold out, SparkFun retired, Seeed out of stock); the M.2 HAT+ at $12
fits only the module that does not exist; HatDrive! Dual is sold out at
Pineboards and discontinued at The Pi Hut; UP Shop's in-stock part is 2280 and
fits neither carrier. The $76.95 AI HAT+ is what is actually purchasable.

At the out-of-stock module + 2280 carrier ($89 + $13 = $102), the Pi's Part B
would be $409 and the delta would narrow to ~$61 -- but neither part ships.

### 4.2 The Pi path cannot have an NVMe

The AI HAT+ consumes the Pi 5's single PCIe lane, so the Pi boots from SD.
3.6 calls the NVMe row *"the one item that prevents losing work rather than an
annoyance,"* because **SD cards corrupt on brownout** -- the exact failure
section 1.3 is written about. The Jetson's M.2 Key M 2280 slot is free and
unaffected.

### 4.3 What the $86 buys -- stated carefully, because the obvious table is invalid

**CORRECTED 2026-09-17.** This section first carried a table putting OWLv2's
**82%** beside the Pi tier's **72%/45%** as though one could be subtracted from
the other. **It cannot.** OWLv2's 82% is a *detector* score at a 3-false-
positive budget; the tier's ~50% is the *whole pipeline* at the shipped
`P>=0.8` gate. Different measurements on different objects. The correction is
owed to the concurrent P16-P18 work in `PLAN-onboard-perception.md`, and it is
the single most important caveat in this file.

What can honestly be said:

| | Pi 5 + Hailo-8L | Jetson Orin Nano Super |
|---|---|---|
| Can it run OWLv2? | **No, at any price** | Yes, PyTorch fp16 |
| Tier as measured today | **49-50%** whole-pipeline (P16/P18) | not measured in tier form |
| OWLv2 as a detector | -- | 82% @3 FP (P7) |
| Reactive tier | 92 FPS on the 8L | GPU |

**OWLv2 does not compile to any Hailo** -- P6 for the 8L, P17 for four
configurations across two architectures. All translate, all optimise, none
compiles. So the Jetson is the only route to that model, and there is no
cheaper one.

**But the tier does not currently depend on it.** P16 found the floor mask
carries the pipeline (49% with it, 3% without) and P18 showed the mask compiles
to both Hailos and survives INT8 at IoU 0.988. **OWLv2 has never been run as a
crop source inside the tier**, only as a standalone detector -- so its advantage
*in the pipeline* is unmeasured. `brain/perceive_lab.py` can settle that off the
robot in an afternoon, and it should, before the money is spent.

### 4.4 Availability

The Jetson is in stock locally for same-day pickup. The Pi path's accelerator
is unobtainable in its specified form. **A part that can be bought beats a part
that is cheaper on paper and out of stock.**

### 4.5 Costs that fall on the Jetson side

- **Power is tighter.** 7-25W against the Pi's ~10-15W, and a 3S pack at 9.9V
  empty is only 0.9V above the Jetson's 9V floor. A 3S BMS cuts off at
  8.4-9.0V -- **at or below that floor** -- so a BMS protects the cells and not
  the board, and a **software cutoff off the driver board's INA219 is required
  work** this repo does not yet have.
- **JetPack 6 vs 7 is unresolved**, and the board's entire value rests on
  `torch` + `transformers` wheels existing for the installed L4T. Confirm
  before committing an SD card.
- **Firmware risk.** Some devkits ship with pre-36.0 firmware; one Micro Center
  reviewer reports a unit bricked during the required update. Do the firmware
  step on the stock 19V adapter, inside the 30-day return window.

---

### 4.6 Reconciling the $86 with the "$220-300" in `PLAN-onboard-perception.md`

A section dated the same day in `PLAN-onboard-perception.md` concludes **"The
Jetson is ~$220-300 more than either Pi build."** Both were written on
2026-09-17, in different sessions. **The difference is entirely one input**, and
it resolves cleanly:

> That section prices the Pi from **3.6's estimates** and the Jetson from
> **verified retailer pages**. 3.6 is marked superseded for pricing precisely
> because its Pi 5 line reads **$80** against a verified **$175** -- a $95 error
> on the single largest line, plus $23 on the microSD.

Priced consistently, every way of defining the build lands in the same band:

| comparison | Pi all-in | Jetson all-in | delta |
|---|---|---|---|
| Pi *specified* build @ **3.6 estimates** vs Jetson *verified* | ~$759 | ~$1,019 | **+$261** -- the disputed figure, apples to oranges |
| Pi *specified* build @ **verified** vs Jetson, both with NVMe | ~$960 | ~$1,019 | **+$59** |
| Pi *buyable* build vs Jetson, neither with NVMe | ~$858 | ~$944 | **+$86** |

**The honest range is $59-86.** The $220-300 figure is an artifact of mixing
estimated and verified prices and should not be quoted.

Note what the middle row costs, though: the *specified* Pi build (bare M.2
module + dual-slot carrier + NVMe) is the cheaper comparison **and neither of
its two key parts is in stock** (4.1). So the $59 is a price for a robot that
cannot currently be assembled, and the $86 is the price for one that can.

**The technical argument in that section is not affected by this and stands**
-- see 4.3. Its point that OWLv2's 82% is not comparable to the tier's ~50% is
correct, and this file has been corrected accordingly.

## 5. Open before ordering either

| # | Item |
|---|---|
| 1 | **Run P10's YOLO-World INT8 compile (~$1, one EC2 hour).** It is the only thing that can put the Pi path at 72% rather than 45%, and therefore the only thing that makes $86 a real debate |
| 2 | Micro Center Westmont stock and return window for SKU 812057 -- the 15-day list names *motherboards* and this SKU's component type is "Development Board / Mainboards" |
| 3 | Yahboom ASIN B0F3CYDQ21's bundled battery voltage (spec is image-only; probably 12.6V 3S) |
| 4 | Inline 10A fuse + holder -- **not priced on either path** |
| 5 | The camera wedge still has to be modelled (10-20 degrees down, M2 holes) |

---

## 6. Provenance

| Source | Date | Covers |
|---|---|---|
| Claude Cowork sourcing pass #1 | 2026-09-16 | Pi 5 pricing history, Hailo stock survey, Part A and C |
| Micro Center Westmont page (PDF capture) | 2026-09-16 | Jetson price, stock, full spec sheet |
| Claude Cowork sourcing pass #2 -> `HARDWARE-BOM.md` | 2026-09-17 | Full Jetson BOM, power budget, bring-up |
| Arithmetic re-checked in-repo | 2026-09-17 | Both columns; Cowork's A+B = $751.34 reproduced exactly |

Estimates marked `[I]` (A5, A9, A11, the cables and pigtails) are the weakest
lines and total under $60 across both columns -- they cannot move the $86.
