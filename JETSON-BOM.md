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
> (tax and shipping included), of which **$465 is the Jetson side I'm
> buying anyway**. So a kit is worth it if *$465 + kit + the missing items
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
