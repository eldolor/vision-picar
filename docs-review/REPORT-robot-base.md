# Docs review — robot-base documentation (2026-09-30)

Scoped review of the documents written or changed during the 2026-09-27..30
chassis search, run on the `robot-base-docs` worktree. Written beside
`REPORT.md` (the whole-repo review, merged to `dev`) rather than over it.
Read-only: no document was edited by this review.

## 1. Summary

The new material is sound on its central claim. The four-layer model, the
closed-loop-vs-reported distinction, the Hiwonder driver anatomy and the
Waveshare protocol facts all check out against the firmware and driver
source, and `robot/hardware_robot.py` really does speak the UGV Rover ROS
Driver's commands (`T:1`, `T:131`, `T:136`, and `T:1001` with `L`/`R`).
**The biggest problem is that the older hardware docs were not brought along:**
`CLAUDE.md`'s hardware line and "Then buy" section, and `HARDWARE-BOM.md`,
still describe the General Driver board, the RPLidar C1, a firmware change for
closed loop and 1650 pulses/rev. A reader who enters through `CLAUDE.md` gets
the old plan. A second, practical error: both new docs tell you to buy a barrel
**adapter** for the separate Jetson battery, but that pack's output is a
**female** socket, so an adapter won't connect it. Two `[V]` tags in the guide
also overclaim what was verified.

## 2. Scorecard

Weighted: criteria 3, 7 and 11 count ×2. N/A criteria are excluded from the
maximum.

| doc | type · reader | score | lowest criteria |
|---|---|---|---|
| `GUIDE-robot-base.md` | concept/reference · you in 6 months, a new contributor | **25 / 36** | 16 debuggability (0), 11 actionable (1), 15 safety (1), 10 current (1: mis-tags) |
| `JETSON-BOM.md` section 9 | decision record · you, buying | **23 / 32** | 11 actionable (1: wrong adapter), 10 current (1: sections 1-8 still headed "the recommended build"), 15 safety (1) |
| `CLAUDE.md` (hardware status + "Then buy") | orientation · every new session | **15 / 28** | 10 current (0: contradicts itself), 11 actionable (1), 8 gaps (1) |
| `PLAN-ros-alignment.md` 3.21 status note | decision record · you, a later session | **24 / 28** | 10 current (1: the body table still shows 1650 tagged `[V]`) |
| "ROS 2 for vision-picar" doc, section 13 + section 12 edits (Claude Docs) | concept · you | **24 / 34** | 10 current (1), 8 gaps (1: no evidence tags at all), 16 debuggability (0) |

Per-criterion scores for the two main docs:

| # | criterion | GUIDE | BOM §9 |
|---|---|---|---|
| 1 | audience / prerequisites | 1 | 1 |
| 2 | purpose up front | 2 | 2 |
| 3 | model before mechanics ×2 | 2 | 1 |
| 4 | progressive disclosure | 2 | 2 |
| 5 | precise terms | 2 | 2 |
| 6 | concrete examples | 1 | 1 |
| 7 | rationale / tradeoffs ×2 | 2 | 2 |
| 8 | honest gaps | 1 | 2 |
| 9 | navigable structure | 2 | 2 |
| 10 | verifiable and current | 1 | 1 |
| 11 | actionable outcome ×2 | 1 | 1 |
| 12 | units / conventions | 1 | N/A |
| 13 | reproducibility | N/A | N/A |
| 14 | hardware facts | 1 | 1 |
| 15 | operational safety | 1 | 1 |
| 16 | debuggability | 0 | N/A |

## 3. Verification failures

| # | doc claim | reality |
|---|---|---|
| V1 | `GUIDE-robot-base.md:350-351` "UGV Rover: geometric track 174.52 mm from its URDF `[V]`" | Not verified in this work: it came from a research assistant's report. The URDF path it cited (`waveshareteam/ugv_ws/.../ugv_rover.urdf`) returned nothing when fetched. Should be `[U]`. |
| V2 | `GUIDE-robot-base.md:88` "11 × 2 (half-quadrature) × 30 = 660 `[V: firmware + Waveshare support]`" | Only **660** is verified (`ugv_base_ros/ROS_Driver/ugv_config.h:344`; Waveshare ticket 257427). The 11-line encoder and the 1:30 gearbox are inferred from a research assistant's report. The breakdown should be `[I]`. |
| V3 | `GUIDE-robot-base.md:314` "inline fuse → **barrel adapter** → Jetson"; `JETSON-BOM.md:392-393` "a 5.5 x 2.1 to 5.5 x 2.5 mm barrel **adapter**" | The Wheeltec E351S output is a "**DC5.5-2.1 female**, shared charging/discharging port" (OpenELAB listing), and the Jetson's input is also a jack. What's needed is a **male 5.5×2.1 to male 5.5×2.5 cable** (with the fuse in line). An adapter plugs into neither. |
| V4 | `GUIDE-robot-base.md:272` "The Orin Nano Super offers 7 W, 15 W, 25 W and MAXN SUPER", untagged | Not checked against a board or NVIDIA's docs; it's from memory. It needs a tag, or a check of `/etc/nvpmodel.conf` on arrival. |
| V5 | `HARDWARE-BOM.md:334` "Firmware defaults are for Waveshare's own UGV and are wrong for this chassis `[V]`: wheel diameter 0.080 m, **1650** encoder pulses, track 0.172 m" | For the chassis now recommended, those defaults are **right** except 1650, which is **660** (`JETSON-BOM.md:363`; `GUIDE-robot-base.md:99-102`). Code still carries 1650: `sim/mock_robot.py:84`, `robot/hardware_robot.py:52`, `sim/fake_esp32.py:59`. This is flagged in `PLAN-ros-alignment.md:1796` but not fixed in `HARDWARE-BOM.md`. |
| V6 | `CLAUDE.md:189-190` "Camera IMX219, motor board Waveshare **ESP32 General Driver**, one ST3215 pan servo, **RPLidar C1**" | Eight lines further down, `CLAUDE.md:193-201` recommends the UGV Rover kit, which ships the **ROS Driver** board, a **D500** lidar, an OAK-D Lite and a 2-axis pan-tilt (`JETSON-BOM.md` §9.3). The same bullet contradicts itself. |
| V7 | `CLAUDE.md` "Then buy" (~line 992-997): "What to buy is `JETSON-BOM.md` (the recommended build, ~$944 ...)", and "closed-loop speed needs a firmware change" | `JETSON-BOM.md` §9 recommends the UGV Rover kit, whose ROS Driver board **ships** closed-loop firmware (Waveshare ticket 257427). The ~$944 figure is the 2WD piecemeal build. |
| V8 | `JETSON-BOM.md:339` "Hiwonder JetRover (tank), JetAuto, JetAcker, ROSOrin Pro \| **$769.99+**" | Hiwonder store data, no computer: ROSOrin Pro $769.99, **JetRover $779.99**, **JetAuto $439.99**, **JetAcker $399.99**. The row's price is only the ROSOrin Pro's. |
| V9 | Claude Docs page, section 12: "over 5,760 swept runs, none ended under 18 cm" (rewritten 2026-09-30) | `PLAN-ros-alignment.md:1603` measured 5,760 runs on the **228 × 198** 2WD footprint. On the Rover footprint (3.21) only the pinned sample was re-run. The sentence reads as current. |

No mismatches were found in the rest:
- **Protocol:** our backend's commands and parsing match `ugv_base_ros/ROS_Driver/json_cmd.h:3,80-81,130-132,163-164`.
- **Hiwonder function codes:** match `ros_robot_controller_sdk.py`'s `parsers`.
- **Chassis numbers:** the 0.040 m wheels and 0.172 m track match `sim/mock_robot.py:83,92`.
- **Geometry:** the 17.1 cm corner radius matches 253 × 231 mm.
- **Other references:** the Micro Center part number and `tests/chassis_fit.py` exist.

## 4. Per-doc findings

**`GUIDE-robot-base.md`**
- The V1, V2 and V4 tags (above).
- **No way to check any of it on the robot (criterion 16).** Section 2 says whether encoder data reaches the host is *the* question, but never shows how to check. Add both ends: listen on the serial port for `{"T":1001,...,"odl":...}` frames, and `ros2 topic echo /odom` against a measured metre.
- **Section 6 stops at "software must shut it down before the pack does".** It gives no threshold, no mechanism, and no pointer to where this project will implement it (the INA219 voltage in `T:1001` `v`).
- **Section 6's wiring (V3), plus a gap:** it never says to **disconnect the kit's own DC5525 lead** from the Jetson, which the separate-pack diagram implies.
- **Criterion 1:** there's no prerequisites line. Section 1 assumes the reader knows what a topic is; the Claude Docs page's section 3 teaches that, so link it.

**`JETSON-BOM.md` section 9**
- **§9.5's "adapter" (V3).**
- **§9.5's code changes aren't located.** "the encoder constant becomes 660" should name the three files in V5, plus `service/slam/.../controllers.yaml` if it gains `wheel_separation_multiplier`.
- **The file's framing contradicts §9.** It still opens "This file is the shopping list for the recommended build" (§1-6, the ~$944 2WD build), while §9 recommends a kit. The status box helps, but §6 "Totals to beat" now has a winner that isn't in it.
- **The 18650 cells aren't specified.** Name a cell that is 4C at the capacity chosen, since Waveshare's power answer depends on it.

**`CLAUDE.md`**
- **V6 and V7.** Both are one-line fixes, and they matter most, because `CLAUDE.md` is loaded into every session.

**`PLAN-ros-alignment.md` 3.21**
- The status note is correct.
- **The body table (line 1810) still reads "1650 ... `[V]`".** Strike it through, or annotate "stale (open-loop firmware); 660", so a reader who jumps to the table isn't misled.

**Claude Docs page ("ROS 2 for vision-picar")**
- **V9.**
- **No evidence tags anywhere,** unlike the repo docs. Section 13 states Waveshare's and Hiwonder's answers as fact without saying which were vendor emails. Add "(Waveshare support, 2026-09-30)"-style attributions where missing; most are present.
- **Section 13.5 repeats V3.**

## 5. Cross-doc issues and reading path

- **The same material lives in three places:** `GUIDE-robot-base.md`, the Claude Docs page's section 13, and `JETSON-BOM.md` §9. The three already differ slightly in wording. **Proposal:** make the **Claude Docs page the learning copy** (you said that's its purpose). Cut `GUIDE-robot-base.md` down to a pointer plus the tagged source facts, or keep it as the repo copy and state which one wins. `JETSON-BOM.md` §9 stays the decision record.
- **Contradictions:** V5, V6 and V7, all the same staleness, spanning `HARDWARE-BOM.md`, `CLAUDE.md`, `HARDWARE-READINESS.md` (not re-checked here; it was last re-bannered 2026-09-28, before the Rover recommendation) and the code constants.
- **Proposed reading path for anyone buying or bringing up the base:**
  1. `CLAUDE.md` §3b (status, one paragraph).
  2. The Claude Docs page, section 13 (concepts).
  3. `JETSON-BOM.md` §9 (what's decided and why).
  4. `HARDWARE-BOM.md` (part numbers, wiring, protocol; needs the Rover update first).
  5. `HARDWARE-READINESS.md` §5 (pre-flight).

## 6. Empirical spot check

**A. `GUIDE-robot-base.md` §6, "a separate battery for the computer"**, followed as a first-time reader wiring the Rover:
1. **The reader buys "a barrel adapter".** It won't mate: both the E351S output and the Jetson input are sockets (V3). **Stall.**
2. **"Inline fuse (~5 A)"**: no fuse type (blade or glass) and no holder. A reader can manage.
3. **Where does the new lead go?** The kit already runs a DC5525 lead from its UPS to the Jetson (`JETSON-BOM.md` §9.3). Nothing says to **unplug** it. A reader could plug the new pack into a splitter or leave both, back-feeding the UPS. **Misread risk.**
4. **"Grounds ... shared through the USB cable. That is normal."** Correct, and useful.
5. **"Software must shut it down before the pack does"**: there's no mechanism for the separate pack. Its protection board cuts off at 9 V, the Jetson's floor, so the Jetson will simply lose power. **Stall** on how to shut down safely.

**B. `JETSON-BOM.md` §9.5, "If the Rover is bought"**, followed as the next session making the code changes:
1. "The encoder constant becomes **660**": which files? The reader has to grep. The 3.21 table lists them, but §9.5 doesn't link it.
2. "`robot/hardware_robot.py` should read the ROS Driver's `odl`/`odr`": correct. But the fake board (`sim/fake_esp32.py`) doesn't emit `odl`/`odr`, so R7's tests can't cover the change until it does. That isn't mentioned. **Stall** at test time.
3. "Use 4C-rated 18650s": no cell named; see §4.

## 7. Prioritized fix list

| # | fix | why it matters | effort |
|---|---|---|---|
| 1 | V3: say **male-to-male 5.5×2.1 → 5.5×2.5 cable (fused)**, and "unplug the kit's DC5525 lead", in GUIDE §6, BOM §9.5 and the Docs page §13.5 | a wrong part, and a possible back-feed on the car | S |
| 2 | V6, V7: rewrite `CLAUDE.md:189-190` and "Then buy" to the Rover recommendation (ROS Driver, D500, no firmware change), pointing at `JETSON-BOM.md` §9 | every session reads `CLAUDE.md` first | S |
| 3 | V5: update `HARDWARE-BOM.md:334` (660, and the defaults are right for the Rover); annotate PLAN 3.21's table row | the parts doc disagrees with the decision | S |
| 4 | Add a low-voltage shutdown plan for both packs (threshold, reading `T:1001` `v`, a `systemd` or brain action) to GUIDE §6 and `HARDWARE-READINESS.md` | the Jetson browns out at the end of every charge | M |
| 5 | Add a "check encoder data reaches the host" procedure (serial listen + `/odom` against a measured metre) to GUIDE §2 | it's the test that decided the purchase, and it becomes the acceptance test | S |
| 6 | V1, V2, V4: re-tag to `[U]` / `[I]`, or verify | the guide's value rests on its tags | S |
| 7 | Make `sim/fake_esp32.py` emit `odl`/`odr` before switching the backend to read them (note in BOM §9.5) | otherwise the backend change is untested | M |
| 8 | V8, V9: fix the Hiwonder price row; qualify the Docs page's 5,760-run sentence as measured on the 2WD footprint | accuracy | S |
| 9 | Decide which copy of the concepts wins (Docs page vs GUIDE) and cut the other to a pointer | three copies are already drifting | M |
| 10 | Name the 18650 cells (4C at the chosen capacity) for the Rover | Waveshare's power answer depends on the cells | S |

## 8. Missing docs to write

- **Rover bring-up runbook:** unbox → fit 18650s → wire the separate Jetson pack → flash nothing, but verify firmware version → confirm `T:1001` carries `odl`/`odr` → measured-metre odometry check → set `wheel_separation_multiplier` → 25 W stress test with motors running → safety sweep against the real lidar.
- **Power and battery handling:** both packs, charging, storage voltage, low-voltage cutoffs, the fuse, what a brownout looks like in logs, and the latching e-stop the Docs page §12 asks for.
- **Vendor protocol reference for the ROS Driver board:** the `T` codes we use, the `T:1001` fields with units, and which fields `robot/hardware_robot.py` consumes. The protocol currently lives only in firmware comments and prose.
