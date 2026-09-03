# Plan: perception on the car itself

Status: **PROPOSED, nothing built, nothing decided** · Date: 2026-09-03 · Phase IDs: none assigned yet

A holding pen, in the sense `microduck`'s `docs/ideas/` uses the term: written
down before it has a design so the thinking is not lost, and **deliberately not
a decision**. Nothing here has been measured on a Pi, nothing has been bought,
and no phase ID has been assigned -- section 1's recommendation is an order of
work, not a purchase.

It exists because reading Microduck raised a question this project had not
asked: *what could run on the car itself?* `PLAN-microduck-transplants.md`
answers the depth half of that (M2, M3, M10 -- a sensor, not a model). This
document is the other half: the vision model, the hardware that would run it,
and where it would sit.

Companion to [`HARDWARE-READINESS.md`](HARDWARE-READINESS.md) (what the kit
changes) and [`PLAN-microduck-transplants.md`](PLAN-microduck-transplants.md)
(§2's sensor-split argument, which this document assumes rather than repeats).

Section 3 is a glossary for all three documents.

---

## 0. The question, and what it is not

Microduck runs three model families on the robot, all local, all small:

| | What | Where | How |
|---|---|---|---|
| Locomotion | 9 ONNX policies, `obs[1,61] -> act[1,14]` | `robotd`, inside the 50Hz tick | ONNX Runtime, `dlopen`'d |
| `duck-detect` | `yolo11n` @ 320, **one class**, 3.9MB INT8, mAP50 0.976 | `mediad`, off the control loop | RK3566 NPU via `librknnrt.so`, `dlopen`'d; ONNX CPU fallback |
| `pet-detect` | ~20KB audio CNN over log-mel, sub-ms | `robotd`'s audio worker | ONNX |

Only the middle one transfers. A PiCar-X has no gait to learn and no head to
scratch.

**Two mismatches to hold onto before reading section 1.**

**`duck-detect` is single-class.** It detects Microduck's own duck and reduces
the box to a `bearing()` in -1..1 -- "turn towards it" needs a bearing, not a
box. This project's target is arbitrary text typed at mission start ("red
backpack", "blue bottle"), which is an open-vocabulary problem and a
substantially harder one on-device. Section 1.2 is why that turns out not to
matter for the targets actually in use.

**Recognition is not the failing half.** Stage 0 established that every cloud
model identifies a red backpack; four wordings failed on *depth*. So an
on-device detector does **not** close the Stage 0 gate, and no argument in this
document should be read as claiming it does. What it would buy is different,
and still real:

- **Rate** -- a bearing at tens of Hz in the ~1-3s gaps between `/navigate` calls
- **Cost** -- no Bedrock charge per frame
- **Offline** -- the collar and a target lock keep working when the link dies

That is Microduck's tiering (§2 below): fast local perception underneath slow
remote deliberation. It is a good argument. It is not the gate argument.

---

## 1. Accelerator options on a PiCar-X

### 1.1 The four options

**The Pi 5 has no NPU.** That is the single biggest difference from the RK3566,
and it cuts both ways: there is no free 0.8 TOPS sitting on the SoC, but the
four Cortex-A76 cores at 2.4GHz are a considerably stronger CPU host than the
Rockchip's A55s. CPU-only is a real option here in a way it is not on a duck.

| Option | Silicon | Inference runs | Rated | Chassis fit |
|---|---|---|---|---|
| **CPU only** | 4x Cortex-A76 @ 2.4GHz | on the Pi's cores | -- | Nothing to buy, nothing to mount, no power draw, no conflict |
| **AI Camera** | Sony IMX500 | **on the sensor** | ~3 TOPS class | CSI swap. Nothing on the GPIO header, no PCIe, negligible Pi CPU. Best physical fit |
| **AI HAT+** | Hailo-8L / Hailo-8 | on the module, over PCIe | 13 / 26 TOPS | **Conflicts with the Robot HAT** (§1.4). Real watts off the pack |
| **Coral USB** | Google Edge TPU | on the stick, over USB | 4 TOPS | Support on Pi 5 with recent kernels has been rough -- verify before buying |

All figures are vendor ratings at INT8. None has been measured on this project's
hardware, because this project has no hardware.

### 1.2 The finding that lowers the risk on all four paths

**`backpack` and `bottle` are both COCO classes.**

The Stage 0 targets are in the standard 80-class label set that essentially
every off-the-shelf detector predicts. So a stock, pre-compiled YOLO11n --
Hailo's Model Zoo ships HEFs for both Hailo-8 and Hailo-8L; the ONNX and TFLite
equivalents are equally available -- detects them with **zero training, zero
calibration set, zero distillation**. Download, run, filter to the class,
reduce the box to a bearing exactly as `duck-detect` does.

*(Confirm against the specific model's label file rather than taking this
paragraph's word for it.)*

This retires most of the "data is the project, not the model" concern that
Microduck's `autonomous_behavior.md` raises and that an earlier reading of this
question inherited. Distillation becomes the project only when the target
vocabulary goes past COCO's 80.

**What it does not give you is the colour.** COCO says `backpack`, not `red
backpack`. In a room with two backpacks you are back to either an HSV check on
the crop -- cheap, and probably sufficient -- or a genuinely open-vocabulary
model (YOLO-World, or CLIP re-ranking class-agnostic proposals), which is a much
larger commitment.

### 1.3 YOLO11n on the AI HAT+, specifically

Asked directly, so answered directly: **yes, and it is the most turnkey
combination on the list.**

- `sudo apt install hailo-all` gets the PCIe driver, HailoRT, the
  GStreamer/TAPPAS bits, and `rpicam-apps` with Hailo post-processing stages.
- Pre-compiled YOLO11n HEFs exist in Hailo's Model Zoo for both parts, so the
  COCO path needs no toolchain at all.
- A detection demo is a one-liner against `rpicam-hello`.

**The friction is custom classes.** You cannot hand the chip a `.pt` or an
`.onnx` -- it wants a HEF, produced by Hailo's Dataflow Compiler, which requires
**x86-64 Linux** (not the Pi, not an ARM Mac), plus a calibration set of a few
hundred representative frames for INT8 quantisation. That is a real setup cost
and it is the argument for staying inside the Model Zoo while COCO covers the
targets.

**It is also enormously overprovisioned.** YOLO11n at 640 on a Hailo-8L runs in
the high-tens-to-hundreds of FPS; at 320, more. The decision loop is one
`/navigate` call every 1-3 seconds. Throughput is therefore *not* the reason to
buy one, which means the decision rests entirely on physical fit -- and that is
where it gets difficult.

### 1.4 The Robot HAT conflict is the real blocker

**Physical, not software.** SunFounder's Robot HAT occupies the 40-pin GPIO
header -- battery input, motor drivers, servo headers, ADC. The AI HAT+ needs
*both* the PCIe FPC connector *and* 5V from that same header.

**Verify the pass-through on the actual kit before assuming either way.** As far
as this document knows the Robot HAT is a terminating board with no stacking
pins, but that is an assumption and it is cheap to check with the board in hand.

If there is no pass-through, the options are:

- A bare **M.2 -> PCIe FPC adapter** mounted off-header somewhere on the
  chassis, fed 5V separately. Doable, unsupported, and now you are fabricating a
  mount on a small car.
- The **AI Camera** instead, which avoids the header entirely.

Power is the second constraint. Hailo-8L is ~2W typical, Hailo-8 up to ~5W under
load, drawn from two 18650s that are also running two drive motors and two
servos. The AI Camera and CPU-only paths cost nothing extra.

### 1.5 The monocular-depth trap

**Do not reach for a depth model to fix the gate.** It is the obvious move and
it is the exact error `PLAN-microduck-transplants.md` §2 argues against.

Depth Anything V2 Small and MiDaS small both run on a Pi 5, slowly. They produce
**relative** inverse depth. The metric-finetuned variants are calibrated to
camera intrinsics and mounting heights nothing like a 10cm PiCar camera.
Adopting one replaces "ask a VLM how far" with "ask a smaller model how far" --
the same class of error with fewer parameters, and Stage 0 has already spent
five wordings establishing that this class of error is not a wording problem.

**One nuance worth keeping, though.** Relative depth plus a ground-plane
assumption yields a *traversability mask* -- "is the centre third of the floor
ahead clear relative to its surroundings" -- which is a genuinely different
question from "how many cm". That is `center-third-path`'s question answered by
geometry instead of by a prompt, and `center-third-path` failed as a prompt for
reasons (a threshold has no wording) that geometry does not share. It may be
worth something later. It is still not the metric answer M10 needs.

**And note what Microduck's own answer to distance is: not a model.** A
VL53L5CX/VL53L8CX 8x8 ToF, ~$20-30 on a breakout, publishing `Range` /
`NoTarget` / `Unusable`. That is M2 and M3 in real hardware, and it is the
cheapest item on this page by an order of magnitude.

### 1.6 Recommendation -- an order of work, not a purchase

1. **Benchmark stock YOLO11n at 320, single class, on the Pi 5 CPU.** Free, and
   it is the measurement every other option is judged against. Microduck builds
   `duck-bench` for exactly this reason. If it clears ~10 FPS, no accelerator
   was ever needed for a 1-3s loop.
2. **If CPU proves tight, prefer the AI Camera** over the AI HAT+. It sidesteps
   the header fight, the power draw and the mounting problem in one move.
3. **Treat the ToF as the depth answer**, independent of all of the above.
   Buying 13 TOPS does not move the gate; a $25 sensor does.
4. **Do not buy anything before step 1 reports a number.**

---

## 2. Where it would sit architecturally

### 2.1 The tiering this borrows

Microduck separates three rates, and the separation is the point:

```text
   ~50 Hz    reflex        ONNX policies, on-board, no network
              robotd's tick; safety owns the only motor write handle

  ~15-30 Hz  perception    NPU detector -> bearing
              tofd         -> distance          each question to the sensor
              BLE beacon   -> identity             that can answer it

   ~0.5 Hz   deliberation  the LLM, off-board, over WebSocket
              sends intents; never trusted to execute them
```

**vision-picar collapses the middle two tiers into the top one.** A single cloud
`/navigate` call is asked for bearing *and* distance, at 1-3s, and its answer
drives the robot. `bearing-only` (M1, measured 2026-09-02) is the first half of
un-collapsing that: `/navigate` keeps *what* and *which way*, and something else
owns *how far*. An on-device detector would be the second half -- the bearing
arriving at perception rate instead of deliberation rate.

### 2.2 Its own process. Not `robot/server.py`

This is the open **"Perception in its own process"** row in
`PLAN-microduck-transplants.md` §1, and it is the one architectural point this
document is confident about.

Microduck's rule (`architecture.md` §2.4) is **features, not frames**: put
perception next to the sensor, publish derived features -- "ball at (x,y)",
"person detected" -- tens of bytes at 10-30Hz, and let the control path read a
locally cached *latest* snapshot, non-blocking, last-value-wins. "Shipping
frames to `robotd` so it can run its own vision would waste most of the board's
memory bandwidth."

The failure mode it buys: **a stalled detector degrades perception rather than
adding jitter to motor control.** Applied here, that is the difference between a
wedged `picamera2` capture and a `/stop` that still answers -- which today is
undefined behaviour and is exactly what M9 exists to test.

So: a separate process, publishing features, that `robot/server.py` and the
brain read as a snapshot and never block on.

### 2.3 What must not change

Three constraints this project already holds, restated because a new perception
process is precisely the kind of thing that quietly violates them:

1. **`brain/` and `robot/server.py` talk only to `RobotInterface`.** A detector
   is not a reason to introduce a second path. Whatever it publishes has to
   arrive through the abstraction or alongside it, never around it.
2. **Safety is enforced server-side, always.** A bearing is an input to a
   decision, never a movement path of its own.
3. **`control/` may not import a backend or the simulator.** If the detector's
   output reaches the brain, it reaches it over HTTP like everything else.

### 2.4 Open questions -- all of them

Nothing below is decided. These are the questions a design doc would have to
answer before any of this is built.

| # | Question | Why it is not obvious |
|---|---|---|
| 1 | **What does the detector publish?** A bearing in -1..1 like `duck-detect`? A box? A class list with confidences? | A bearing is the smallest thing that is useful and the hardest to extend. A box defers the decision at the cost of making every consumer do the reduction |
| 2 | **Who consumes it?** The brain, as a new field on the frame? `robot/safety.py`, as a veto input? The twin, as an overlay? | Each answer implies a different transport and a different failure mode |
| 3 | **Does it change `RobotInterface`?** | S2 was careful to make `get_camera_frame()` uniform across backends. A `get_detections()` would have to be answerable by `MockRobot`, `ReplayRobot` and `TeleopRobot` too, or it splits the contract |
| 4 | **What does the sim do?** | `sim/renderer.py` renders flat-shaded walls. A COCO detector will find nothing in them, so a sim backend would have to synthesise detections from grid truth -- which makes the sim leg unable to test the detector, only its consumers |
| 5 | **Does `/navigate` still name the target?** | If the detector handles "where is the backpack", the cloud call's remaining job is smaller and possibly different in kind |
| 6 | **What happens when it disagrees with the cloud?** | Two things now answer "is the target visible". Microduck's answer to competing authorities is a decided priority order, not last-writer-wins (`architecture.md` §6) -- which is M4's subject |

### 2.5 What it would owe the twin

Per section 7 of `CLAUDE.md`: a phase is not done when its tests pass, it is
done when someone holding a phone can watch the thing it built do its job.

For this one that is unambiguous: **a bounding box drawn over the FPV canvas,
with the bearing as a number underneath, updating faster than the mission log
scrolls.** If the detector is running and the box is not on the phone, it is
not shipped.

---

## 3. Glossary

Acronyms used across this document, `PLAN-microduck-transplants.md`, and the
Microduck reading behind both.

### 3.1 On-device vision and accelerators

| | |
|---|---|
| **TOPS** | Tera-Operations Per Second. A throughput rating for inference accelerators, usually quoted at INT8. Marketing-adjacent: real speed depends on the model, not the number |
| **INT8** | 8-bit integer arithmetic. Quantising from 32-bit floats makes a model ~4x smaller and much faster, at some accuracy cost. What NPUs are built for |
| **NPU** | Neural Processing Unit. An on-chip inference accelerator. The RK3566 has one; the Pi 5 does not |
| **ONNX** | Open Neural Network Exchange. A portable model format, so a net trained in PyTorch can run under a different runtime |
| **HEF** | Hailo Executable Format. Hailo's compiled model file -- the chip will not take an ONNX |
| **DFC** | Dataflow Compiler. Hailo's toolchain that turns ONNX into a HEF. x86-64 Linux only |
| **YOLO** | "You Only Look Once". A family of single-pass object detectors. The `n` in YOLO11n is "nano", the smallest variant |
| **COCO** | Common Objects in Context. The standard detection dataset; its 80-class label set is what most off-the-shelf detectors predict, and it includes `backpack` and `bottle` |
| **mAP50** | mean Average Precision at 50% IoU -- the usual detection accuracy score |
| **IoU** | Intersection over Union. Box overlap, the thing mAP thresholds on |
| **XNNPACK** | Google's optimised CPU inference backend for ARM/x86, used under ONNX Runtime and TFLite |
| **ncnn** | Tencent's lightweight embedded inference engine; a common fast path on ARM CPUs |
| **TFLite** | TensorFlow Lite. Google's mobile/embedded runtime and model format |
| **CLIP** | Contrastive Language-Image Pre-training. Matches images to text, which is what makes open-vocabulary detection possible |
| **VLM / LLM** | Vision-Language Model / Large Language Model. The cloud tier -- what `/navigate` calls |
| **MiDaS / Depth Anything** | Monocular depth-estimation models: one image in, a depth map out. **Relative**, not metric -- §1.5 |
| **HSV** | Hue-Saturation-Value. The colour space for a "is that backpack red" check on a crop |

### 3.2 Hardware and buses

| | |
|---|---|
| **SoC** | System on Chip. The RK3566, or the Pi 5's BCM2712 |
| **HAT** | Hardware Attached on Top. The Pi's 40-pin add-on board spec. Two HATs wanting the same header is §1.4's conflict |
| **PCIe** | Peripheral Component Interconnect Express. The high-speed bus the AI HAT+ uses; the Pi 5 exposes one lane |
| **FPC** | Flexible Printed Circuit -- the flat ribbon cable. On a Pi 5 it is how PCIe leaves the board |
| **M.2** | The card form factor (as in NVMe SSDs) the Hailo module ships in |
| **CSI** | Camera Serial Interface. The Pi's ribbon camera port, distinct from USB |
| **ADC** | Analog-to-Digital Converter. On the Robot HAT, for battery voltage and analog sensors |
| **ToF** | Time of Flight. A depth sensor that times a light pulse; the VL53L5CX/L8CX give an 8x8 grid |
| **IMU** | Inertial Measurement Unit. Accelerometer + gyroscope, for orientation and fall detection |
| **I2C / UART** | Two low-speed serial buses. Microduck's ToF sits on I2C; its 15 servos and IMU share one UART |
| **HFOV** | Horizontal Field Of View, in degrees |
| **VPU / ISP** | Video Processing Unit (hardware codec) / Image Signal Processor (the sensor pipeline) |
| **RKNN / rknpu2** | Rockchip's NPU model format and runtime |

### 3.3 Microduck's system

| | |
|---|---|
| **RL** | Reinforcement Learning -- how the locomotion policies are trained |
| **PPO** | Proximal Policy Optimization, the specific RL algorithm |
| **MuJoCo** | The physics simulator the policies train in |
| **sim2real** | Getting a sim-trained policy to work on real hardware, usually via domain randomisation |
| **FK** | Forward Kinematics. Joint angles -> where a part is in space; needed to reproject ToF zones into the robot's frame |
| **SFLP** | Sensor Fusion Low Power. The IMU's on-chip fusion producing an orientation quaternion |
| **JSON-RPC** | A remote-procedure-call convention over JSON. Microduck runs 2.0 over unix sockets |
| **NDJSON** | Newline-Delimited JSON -- one object per line, so a stream is trivially framed |
| **UDS** | Unix Domain Socket. Local IPC via a filesystem path, which is where the free access control comes from |
| **IPC / RPC** | Inter-Process Communication / Remote Procedure Call |
| **SO_PEERCRED** | A socket option returning the caller's uid/gid/pid -- the basis of both the audit log and enforcement |
| **uid / gid** | User ID / Group ID |
| **BLE** | Bluetooth Low Energy. The phone's path in, via `btd` |
| **GATT** | Generic Attribute Profile. How BLE exposes readable/writable characteristics |
| **RSSI** | Received Signal Strength Indicator. Signal strength, used as coarse distance between ducks |
| **WebRTC** | Real-time media + data channels, browser-native. Carries telepresence |
| **SDP / ICE / STUN / TURN** | WebRTC's connection machinery: Session Description Protocol (what peers offer), Interactive Connectivity Establishment (finding a path), STUN (discovering your public address), TURN (a relay when NAT defeats you -- the one that costs real bandwidth) |
| **DTLS-SRTP** | The encryption on WebRTC media, end-to-end even through a TURN relay |
| **SCTP** | The transport under WebRTC data channels |
| **NAT** | Network Address Translation. Why a home robot is not directly reachable from the internet |
| **SSE** | Server-Sent Events. One-way server push over HTTP; considered and declined |
| **D-Bus** | Linux's system message bus; how BlueZ and NetworkManager are driven |
| **BlueZ / NetworkManager** | Linux's Bluetooth stack / network configuration daemon |
| **RT** | Real-Time. As in "RT-ish": tight timing, but not a hard-real-time kernel |
| **EMA** | Exponential Moving Average. The smoothing on battery voltage so a load sag cannot trip shutdown |
| **ULD** | Ultra Lite Driver. ST's vendored C driver for the ToF sensors |
| **shm / dmabuf** | Shared memory / a Linux buffer-sharing mechanism -- the zero-copy escape hatch if frames ever must cross a process boundary |
| **NV12 / UYVY / MJPEG / H.264** | Video formats: two raw YUV pixel layouts, a per-frame JPEG stream, and the compressed codec WebRTC carries |
| **V4L2 M2M** | Video4Linux2 Memory-to-Memory. The kernel API for hardware video encode |
| **flock / fsync / rename(2)** | Linux primitives for a safe file write: lock it, force it to disk, swap it in atomically |
| **inotify** | Linux filesystem change notification -- deliberately not used yet |
| **SHA-256 / minisign** | A cryptographic hash / a signature tool. Together, how a release is verified before install |
| **OTA** | Over-The-Air. Remote software updates |
| **RTT** | Round-Trip Time. Latency, watched by the deadman |
| **SDK** | Software Development Kit. The planned on-robot API for third-party code |

### 3.4 This project

| | |
|---|---|
| **ALB / NLB** | Application / Network Load Balancer. AWS's layer-7 and layer-4 balancers -- the pair all five services share |
| **ECS / Fargate** | Elastic Container Service / its serverless compute mode. Where twin, brain, vision, admin and teleop run |
| **ECR** | Elastic Container Registry. Where the ARM64 images go |
| **EFS** | Elastic File System. The network filesystem holding recorded walks, chosen because it survives a redeploy |
| **VPC** | Virtual Private Cloud. The network -- running VPC endpoints instead of a NAT gateway |
| **IAM** | Identity and Access Management. How the vision task authenticates to Bedrock without an API key |
| **CDN / CloudFront** | Content Delivery Network. The HTTPS front door |
| **IaC** | Infrastructure as Code. The CloudFormation templates |
| **FPV** | First-Person View. The twin's rendered camera canvas |
| **AR** | Augmented Reality. The Guide tab |
| **DOM / jsdom** | Document Object Model / a headless JS implementation of it -- why the UI tests are Playwright instead |
| **CORS** | Cross-Origin Resource Sharing. The browser rule the robot server had to allow |
| **WASD** | The keyboard drive keys, from the never-built manual control client |
| **CI** | Continuous Integration. The build/test pipeline |

---

## 4. Sources

- Microduck `docs/design/architecture.md` §2.4 (features not frames), §5.3
  (server-side agents over WebSocket), §6 (safety and authority)
- Microduck `docs/design/robotd-design.md` §1.4-§1.5 (the tick, the invariants),
  §2.4 (safety owns the only write handle)
- Microduck `docs/ideas/autonomous_behavior.md` ("Duck detector (camera + NPU)",
  and the camera/ToF/BLE sensor split)
- Microduck `docs/project/npu-bringup.md` (the `yolo11n` numbers, `dlopen` over
  linking, "the runtime dequantises")
- Microduck `tof/src/lib.rs` (`Range` / `NoTarget` / `Unusable`)
- Microduck `policies/README.md`, `pet-detect/README.md`
- This repo: `CLAUDE.md` Stage 0 notes, `PLAN-microduck-transplants.md` §1-§2
  and M1-M12, `HARDWARE-READINESS.md` §1 and §5

Hardware claims about the Pi 5, the AI HAT+, the AI Camera and Coral are from
general knowledge as of this document's date, **not** verified against a board.
Every one of them is cheap to check and should be checked before money moves.
