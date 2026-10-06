# Handoff 2026-10-03 -- the Jetson bring-up (PLAN-ros-alignment.md 3.33)

For a fresh session. **Section 1 is the user's list**: the steps only a
person at the desk can do. Section 2 is what the session does over SSH once
section 1's step 4 is done. The plan and its acceptance criteria are
`PLAN-ros-alignment.md` 3.33. The procedure, command by command, is
`tools/jetson/README.md`. Read both before acting.

**Ground rules.**

- **Keep the Jetson's box and all its packaging.** Change nothing that
  can't be undone. Amazon's return window closes about **Oct 30**, and the
  board must stay returnable until the user decides to keep it. Booting it
  and writing a microSD card don't count as modifying it.
- Every step is a stop point. If a step fails, record it in 3.33 and take
  it to the user. Never work around it.
- Run on the stock 19 V adapter throughout, never a battery. Run at
  **15 W** (user, 2026-10-01). Only the 25 W latency run uses MAXN SUPER.
- Other Claude sessions edit this repo at the same time. Work in your own
  git worktree, and commit only your own changes.

---

## 1. The user's list, in order

### Tonight / when the microSD and card reader arrive

1. **Write the card on the Mac.** Plug the card reader into the MacBook Air
   with the 128 GB microSD in it.
   1. Open `~/Downloads/balenaEtcher-2.1.7-arm64.dmg` and drag Etcher into
      Applications.
   2. In Etcher, choose **Flash from file** and pick
      `~/Downloads/jetson-orin-nano-devkit-super-SD-image_JP6.2.1.zip`.
      Don't unzip it.
   3. For **Select target**, choose the card. Check the size reads about
      128 GB, so you don't pick the wrong disk.
   4. Click **Flash**. Writing and verifying takes about 15-25 minutes.
      **Let the verify finish.** NVIDIA publishes no checksum, so the
      verify is the only check.
2. **Set up the board.** Put the card in the Jetson's slot, on the
   underside of the module. Connect a DisplayPort monitor, a USB keyboard
   and mouse, Ethernet, and the stock 19 V adapter last.
3. **Check the firmware at the first boot.** Press **Esc** repeatedly at
   the NVIDIA logo and read the UEFI version.
   - **36.0 or newer:** continue.
   - **Older:** stop and tell the session. NVIDIA's JetPack 6 firmware
     update comes first, on the stock adapter.
4. **Ubuntu's first-boot setup.** User **`picar`**, hostname
   **`picar-jetson`**. The Mac's SSH config expects both names exactly.
   Then, on the Mac:
   ```bash
   ssh-copy-id -i ~/.ssh/id_ed25519_jetson.pub picar-jetson
   ssh picar-jetson     # must log in without a password
   ```
   **Then tell the session.** From here it works over SSH (section 2).
5. **Keep the box.** It holds the 19 V adapter's spare cables and the
   guide. Don't throw anything away.

### When the NVMe arrives (SanDisk Optimus 5100 500 GB, Amazon)

6. **Check that the drive fits.** If there are chips on the back, it
   won't fit under the Jetson; if it's bare or only has a label, it fits.
   It goes in the long **2280** slot, on the underside of the carrier
   board.
7. **Ask the session for the move-to-NVMe steps** before you install the
   drive. Two ways to do it, decided at the time:
   - copy the running microSD system onto the NVMe from the board;
   - or re-flash JetPack onto the NVMe.

   Not before section 2's two risks have passed. The card then becomes the
   recovery card.

### When the cells arrive (4x Molicel P26A, IMR Batteries, expected Oct 7-9)

8. **Inspect the wraps.** No tears near the + end, no dents. Keep the
   cells in their plastic case until the Rover arrives. There is no
   multimeter check, by the user's choice.

### When the Rover arrives (Oct 19 - Nov 11, 30-day return)

9. **Fit three cells.** Match the holder's + and − markings, then charge
   fully on the kit's 12.6 V charger before the first drive. The fourth
   cell is a spare.
10. **Mounting -- DO THIS NOW, not on arrival (updated 2026-10-06).**
    Waveshare's own CAD shows the Rover is modelled around its
    JETSON-ORIN-IO-BASE carrier with a bare module, not the NVIDIA Developer
    Kit we own (`UGV-ROVER-MOUNTING.md`; risk in `JETSON-BOM.md` section 7).
    The devkit is on the desk, so measure it before the Rover comes:
    - **Calipers on the devkit's four corner holes**, centre to centre and
      the hole diameter. Compare against **86.00 x 58.00 mm** first, then
      86.00 x 23.61 mm (`UGV-ROVER-MOUNTING.md` section 4 says why the outer
      86 x 81.61 rectangle cannot be the devkit's). **Due before Oct 19**;
      hard stop, the Rover's Amazon return window.
    - Asked Waveshare support 2026-10-06 in ticket **257511** (not 257427,
      as this item first said): whether the plate takes the devkit, its hole
      pattern, height clearance and port access.
    - Fallback: an adapter plate (`JETSON-BOM.md` 9.5), and check the kit's
      UART and power leads reach the devkit's 40-pin header and barrel jack.
11. **The arrival checks** in `JETSON-BOM.md` 9.5. The session runs them;
    the user plugs things in:
    - a stress test at 15 W with the motors working;
    - read the ESP32 module's shield;
    - find the serial port (expected `/dev/ttyTHS1`);
    - disable Waveshare's own software on the Jetson;
    - measure the CAD geometry and the D500's mounting yaw;
    - dump the stock firmware before any flash.

### Housekeeping on the Mac

12. **Empty the Trash** by hand (Finder wouldn't let the session do it).
    After the card is written, the 11.7 GB image zip can go too. It is
    re-downloadable from NVIDIA's JetPack 6.2.1 page.

---

## 2. What the session does once SSH works (3.33 steps 2-7)

In order, each against 3.33's criteria:

1. **OS and power.** Record `nvpmodel -q`, then set 15 W. The `apt
   upgrade` to JetPack 6.2.2 is optional, and comes only after everything
   below works.
2. **The code.** The repo is private, so put **no GitHub credentials on
   the robot**. Push from the Mac over SSH instead, as
   `tools/jetson/README.md` section 2 describes, and `rsync` the
   recordings (about 450 MB).
3. **Risk 1, torch on the GPU.** Run `bash tools/jetson/setup.sh`:
   - it installs torch 2.8.0 / torchvision 0.23.0 from
     `pypi.jetson-ai-lab.io/jp6/cu126`, plus `libcusolver-12-6`;
   - it creates a Python 3.10 venv with torch pinned;
   - it checks the shipped pipeline is on `cuda`.

   Spec-review item 4a is **done** (2026-10-03): `setup.sh` now asserts the
   detector's device as well as CLIP's.
4. **Risk 2, latency.** Run
   `python -m tools.jetson.bench_perception --recordings recordings --out bench-15w.json`,
   then the same at 25 W.
   - **Budget:** 250 ms a frame at 15 W (user, 2026-10-02).
   - **Compare against the Mac** (`tools/jetson/README.md` section 3): a
     median of 119 ms on the defaults, 36 ms with both models on the
     Mac's GPU.
   - **If over budget,** P26 (`PLAN-onboard-perception.md`) is the first
     fix: decode once per frame, batch the crops, resize on the GPU.
   - **Never merge GPU and CPU rows.**
5. **The suite** from the board's `.venv`. Install Playwright's browser
   first, or the 3 browser tests error instead of skipping. List every
   skip.
6. **G4.** Build the ROS image natively, then run the live chain and nav
   suites **5 consecutive times**. A skip is not a pass.
   - **The start-up order matters** (`HANDOFF-2026-10-02-spec-review.md`
     2-pre and 2a):
     - a brain on :8001 with `ROUTE_PREFIX=/brain`;
     - `SIM_MAP=scaled_house` in pytest's own environment;
     - the container named `picar-ros`, started only after `GET /wheels`
       reports usable.
   - The full command set is in `tools/jetson/README.md` section 4
     (written 2026-10-03).
7. **Headroom.** Run a 10-minute nav2 run with the perception tier
   processing frames. Pass means all three:
   - 0 late ticks on the wheel loop (`/health` `wheel_loop`);
   - at least 1 GB of memory free;
   - no thermal throttling (`tegrastats`).

**Record the numbers in 3.33** and pin what can be pinned. If a risk fails
(no working torch wheel, or latency far over budget that P26 can't close),
that is the return-window decision. Take it to the user well before
Oct 30.

---

## 3. State at handoff

- **On the Mac:**
  - in `~/Downloads`: the JetPack 6.2.1 SD image zip (11,725,610,175 bytes,
    size checked against NVIDIA's server) and balenaEtcher 2.1.7;
  - the SSH key `~/.ssh/id_ed25519_jetson` and a `picar-jetson` host entry
    in `~/.ssh/config`;
  - about 27 GB free disk.
- **In the repo:** `tools/jetson/` holds:
  - `setup.sh`;
  - `bench_perception.py`;
  - `bench_frames.json` (63 frames pinned from 20 walks and 7 targets);
  - `README.md`.

  The offline suite passed under Python 3.10 on Arm Linux in Docker:
  1436 passed, 0 failed.
- **Orders** (`JETSON-BOM.md` 9.1):

  | Item | Status |
  |---|---|
  | Jetson | arrived 09-30, being opened |
  | 128 GB microSD and USB-C card reader | ordered 10-02 |
  | SanDisk Optimus 5100 500 GB NVMe (SDSP51500GAN) | ordered 10-03, $110 + tax |
  | 4x Molicel P26A | ordered 10-03, $34 with shipping |
  | Waveshare UGV Rover PT Jetson Orin ROS2 Kit Acce | ordered 09-30, ~$730 |
  | separate Jetson battery | **not bought**; conditional on the 15 W stress test |

- **The robot base's firmware fork** (`firmware/ugv_base_ros/`, 3.28-3.29)
  is built and not flashed. It is flashed on hardware day, after the
  arrival checks and a dump of the stock image.
