# UGV Rover PT Jetson Orin — carrier board & deck mounting reference

Extracted 2026-10-06 from Waveshare's own published CAD, linked from the
UGV Rover Jetson Orin ROS2 wiki Resources section.

Source files:
- `UGV_Rover_Jetson_Orin_ROS2_Kit_2D.zip`
  → `UGV Rover PT Jetson Orin ROS2 Kit_DXF.dxf` (dimensioned drawing, 2024-10-25)
- `UGV_Rover_PT_Jetson_Orin_ROS2_Kit_STEP.zip`
  → `UGV Rover PT Jetson Orin ROS2 Kit v2.step` (101 MB assembly, 2024-10-25)

Both from `https://files.waveshare.com/wiki/UGV%20Rover%20Jetson%20Orin%20ROS2/`

---

## 1. FINDING: the kit is designed around Waveshare's own carrier, not NVIDIA's

The STEP assembly contains these product names:

    JETSON-ORIN-IO-BASE-PCB-240202
    jetson-orin-io-pcba_asm
    JETSON-XAVIER-ORIN-NX
    JETSON-ORIN-FAN-PWM

i.e. Waveshare's **JETSON-ORIN-IO-BASE** carrier (90.5 x 103.0 mm) populated with a
**bare Orin NX / Orin Nano SO-DIMM module**, plus Waveshare's own PWM fan.

An NVIDIA Jetson Orin Nano **Developer Kit** is a different object: the module is
already bolted to NVIDIA's carrier, outline 100 x 79 mm. The chassis was not
modelled around it.

Status: CONFIRMED from the manufacturer's engineering file, not inferred.

## 2. Deck mounting hole pattern (M3 clearance, 3.68 mm)

Twelve 3.68 mm circles exist in the DXF. Ten belong to the chassis deck view;
the final two (dx 392.16 / 412.16) sit in a separate view of the drawing and are
not part of this pattern.

Coordinates below are relative to the lower-left hole of the main array.

### Main array — 2 rows x 3 columns

    (  0.00,  0.00)   ( 23.61,  0.00)   ( 81.61,  0.00)
    (  0.00, 86.00)   ( 23.61, 86.00)   ( 81.61, 86.00)

Row pitch:     86.00 mm
Column stops:  0.00 / 23.61 / 81.61 mm
               (23.61 between cols 1-2, 58.00 between cols 2-3)

### Auxiliary pairs on the array centreline

    ( 12.28, 19.60)   ( 12.28, 66.40)      -> 46.80 mm apart
    (136.74, 23.00)   (136.74, 63.00)      -> 40.00 mm apart

CAVEAT: the DXF does not label which holes are the carrier mount. The 86.00 x
81.61 mm rectangle is the only candidate at the right scale, but this is
inference, not a labelled callout.

## 3. Why the comparison can't be completed from public data

NVIDIA does not publish the Orin Nano Developer Kit carrier's mounting hole
pattern. Spec SP-11324-001 gives the board outline (100.00 +/- 0.13 x 79.00
+/- 0.13 mm, max height 4.30 mm) and nothing more. The hole pattern exists only
inside the OrCAD reference design files. Multiple NVIDIA developer-forum threads
ask for it and go unanswered.

## 4. ACTION

Measure the Developer Kit's four mounting holes with calipers -- centre to
centre, and the hole diameter -- and compare against the sub-spans below.

**Corrected 2026-10-06: the outer 86.00 x 81.61 mm rectangle cannot be the
Developer Kit's.** The 86.00 mm pitch only fits along the kit's 100 mm side,
which leaves its 79 mm side to span the columns, and holes 81.61 mm apart do
not fit on a 79 mm board. That rectangle fits the IO-BASE (90.5 x 103.0 mm)
and is almost certainly its mount. A Developer Kit that bolts on directly does
so through a sub-span of the same array:

1. **86.00 x 58.00 mm** (columns 2-3) -- check this first. 86 x 58 is the
   pattern commonly quoted for the original Jetson Nano Developer Kit, whose
   outline the Orin Nano kit keeps. UNVERIFIED: recalled, not found in any
   NVIDIA document.
2. 86.00 x 23.61 mm (columns 1-2).

The deck holes are 3.68 mm (M3 clearance); if the kit's are M2.5, they still
mount through the deck with washers.

If neither lines up, the adapter plate target geometry is the array in
section 2. The Developer Kit is on the desk now, so do this before the Rover
arrives (earliest Oct 19); the hard stop is the Rover's Amazon return window,
30 days from delivery. Tracked in `JETSON-BOM.md` section 7.

## 5. Note on file provenance

The STEP file's internal product name is `UGV Rover PT Jetson Orin AI Kit`,
not ROS2 Kit. Waveshare appears to publish one chassis model across both tiers.
The mechanical deck should be identical, but this CAD is not strictly the
ROS2 variant.

## 6. Other dimensions read from the same drawing

    Overall:        264.13 / 283.09 (outer extents in the drawing's callouts)
    Body (spec):    253 x 231 x 289 mm
    Deck callouts:  252.4, 243.18, 159.96, 156.22, 132.59, 120.88,
                    93.74, 86, 58, 54, 45.61, 37.6, 37
