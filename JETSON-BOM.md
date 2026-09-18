# Jetson build — BOM, and a kit-search brief

**Written 2026-09-17.** The Jetson became the recommended build on
measurement, not preference: P19 scored OWLv2 at **83% whole-pipeline
recall against the Pi tier's 50%**, on identical frames and gate, and P17
showed OWLv2 compiles to **no** Hailo. The price gap is **~$86**
(`BOM-COMPARISON.md`, verified listings). Reasoning lives in
`PLAN-onboard-perception.md` P17–P19; part numbers and vendors live in
`HARDWARE-BOM.md`.

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
| the Pi build (cheapest, and the fallback) | `PLAN-onboard-perception.md` 3.6 |
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

## 1. Buy regardless of which kit wins — $464.94 + options

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

## 2. What a kit would replace — ~$290 if bought piecemeal

This is the shopping list a kit needs to cover. A kit covering **rows
marked ★** is the win; the rest are cheap to add.

| ★ | Item | Spec that matters | piecemeal $ |
|---|---|---|---|
| ★ | **Chassis, 2WD** | **Differential drive** (skid steer). Aluminium. | 69.00 |
| ★ | **Encoder motors ×2** | Quadrature encoders, 12V class | incl. |
| ★ | **Motor/IO controller** | Waveshare General Driver for Robots (ESP32) | 27.99 |
| ★ | **Battery ×2 + charger** | **3S LiPo**, XT60, ~2200mAh | 74.97 |
| ★ | 360° lidar | Slamtec **RPLidar C1**, USB | 69.00 |
| | Pan servo | Waveshare **ST3215**, 12V variant, bus servo | 21.99 |
| | Camera pan mount | 3D-printed bracket + wedge | ~5.00 |
| | Powered USB hub | Waveshare USB3.2-Gen1-HUB-4U, **7–36V in** | 17.99 |
| | ToF sensors ×2 | VL53L1X, forward-down | 29.90 |
| | Bumper + microswitches | Omron SS-5GL ×4 + springs | ~11.00 |
| | Wiring, XT60, switch, 10A fuse | 14AWG pigtails, rocker switch | ~20.00 |
| | M2.5/M3 standoff kit | brass | 12.95 |
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

**A kit is worth buying if `$465 (§1) + kit + §5 extras` lands under
~$944.** Below ~$850 it is clearly better; above ~$1,000 the piecemeal
build wins on parts we have already verified.

---

## 7. Two open risks a kit does not remove

- **JetPack 6 vs 7.** The board's entire value rests on running OWLv2 in
  PyTorch, which needs a working `torch` + `transformers` wheel for the
  installed L4T. **Confirm a working torch wheel before committing an SD
  card**, and prefer the JetPack that has one today over the newer one.
- **Latency is unmeasured.** OWLv2 is projected at roughly **5 Hz** on this
  board (fp16; INT8 destroys it — P7d), against the Hailo path's 92 FPS.
  That may mean a fast cheap detector for obstacle reaction alongside
  OWLv2 for deciding where to go. It is a design question for hardware
  day, not a purchase blocker — but it is not resolved.
