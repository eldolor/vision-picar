# vision-picar

An indoor robot car you can send to find something: "find the red
backpack". It explores the house, builds a map as it goes, recognises the
object with on-board models, and stops in front of it. It is built
simulation-first: every behaviour is proven with data in a simulator that
drives the same API the hardware will, before it goes on the car.

* For a non-technical introduction, read
  [`docs/guides/INTRODUCTION.md`](docs/guides/INTRODUCTION.md).
* For orientation in the code, read `CLAUDE.md`.

## Where it stands (2026-10-06)

* **Compute: in hand.** An NVIDIA Jetson Orin Nano Super (JetPack 6.2.1,
  15 W, on an NVMe drive). Everything that needs no wheels has been proven
  on it:
  * the on-board perception at 61 ms a frame (budget 250 ms);
  * the full test suite;
  * the ROS 2 stack driving a fake motor board, five runs in a row;
  * a 10-minute full-load run with no late safety-loop tick.
* **Chassis: ordered.** A Waveshare UGV Rover, due Oct 19 - Nov 11. It
  brings an ESP32 motor board with encoder odometry, a D500 lidar, an
  OAK-D Lite depth camera and a pan-tilt camera.
* **Nothing has driven real wheels yet.** The simulator already fakes the
  Rover's motor board on a serial line, so the car's motor code runs today.
* **Open work:** SLAM accuracy in large furnished rooms, an evaluation of
  NVIDIA's GPU packages (Isaac ROS), and the on-arrival checks for the
  Rover. `CLAUDE.md` section 3 has the list.

## How it fits together

```
 phone (web twin) ──HTTP──┐
                           ▼
 brain  :8001  ──HTTP──▶  robot server  :8000  ──▶  body: simulator today,
 (mission policy,          (safety veto, who is      ESP32 motor board on the car
  YOLOE + CLIP,             driving, watchdog)
  cloud calls)                  ▲   │
                                │   ▼ HTTP
                        ROS 2 container (service/slam/):
                        twist_mux, collision_monitor, diff_drive_controller,
                        slam_toolbox, nav2
```

* **The brain** decides what to do next. It reaches the robot only over
  HTTP, so it runs on a laptop or on the Jetson unchanged.
* **The robot server** is the only process that touches the body. Every
  command passes `robot/safety.py`, on every path.
* **ROS 2** provides mapping and route-planning from standard packages,
  inside one container. It is off by default, and nothing outside the
  container may import it.
* **The cloud** answers the vision model's questions (Claude on Amazon
  Bedrock) only when the on-board perception asks.

[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) is the whole system on one
page. The doc "ROS 2 for vision-picar" explains the ROS side for a reader
new to robotics.

## Quick start

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m playwright install chromium          # so the browser UI tests run, not skip
pip install -r requirements-perception.txt     # only for policy "tiered" (YOLOE + CLIP)

pytest tests/ -q                               # no API key needed; every model call is mocked
pytest service/vision_analyze/tests/ -q        # the cloud vision service's own suite
```

**Run the robot and the brain, and drive from the twin:**

```bash
uvicorn robot.server:app --port 8000
uvicorn control.brain_server:app --port 8001
# open http://127.0.0.1:8000/  -> Settings: connect both -> Sim tab
```

The Sim tab has:
* a D-pad, which drives the robot through the safety layer;
* a live map that fills in as the robot explores;
* a depth strip;
* the **Remote brain** panel, which starts a mission and then only
  observes it.

Restart the robot server to put the robot back at its start position.

**Start a mission without the page:**

```bash
curl -X POST localhost:8001/mission/start -H 'content-type: application/json' \
     -d '{"target_object": "red backpack"}'
curl localhost:8001/mission/status
curl -X POST localhost:8001/mission/stop        # stops the loop AND the car
```

**Other entry points:**

| To | Run / read |
|---|---|
| Turn on SLAM and nav2 | `service/slam/README.md` (Docker; `ROBOT_DRIVE=ros WORLD_MODE=ros`) |
| Drive the car's motor code against a fake board | `ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake WORLD_MODE=none bash service/tunnel/restart.sh` (`WORLD_MODE=ros` with the container) |
| Use another house | `SIM_MAP=scaled_house` or `home_first_floor` |
| Add people and pets that move | `SIM_MOVERS=<scenario>` |
| Replay a recorded phone walk through a policy | `python -m tests.demo_replay_mission recordings/<walk> "<target>" --policy tiered` |
| Check both servers' health | `python -m control.health` |
| Reach the local stack from a phone anywhere | `service/tunnel/` (an ngrok domain split by path) |
| Set up and benchmark the Jetson | `tools/jetson/README.md` |

## What is in the repo

| Path | What |
|---|---|
| `robot/` | The robot server, `RobotInterface`, the safety layer, the motor-board backend |
| `world/` | `WorldInterface`: pose and map, from the simulator or from SLAM |
| `brain/` | Mission policies (rule-based, vision, tiered), on-board perception, the arrival rule |
| `control/` | The brain as a service, failsafes, health, recorded-walk tools |
| `sim/` | The simulator: houses, physics, renderer, movers, the fake motor board |
| `service/` | The ROS 2 container, the cloud vision service, deployment and tunnel scripts |
| `firmware/` | Our patch to the Rover's ESP32 firmware (GPL-3.0, not yet flashed) |
| `web-twin/` | The phone UI |
| `docs/` | Specs (architecture and engineering per component), plans, guides, hardware, evaluations |

## Reading list

| Doc | For |
|---|---|
| `CLAUDE.md` | Orientation: the rules, the current state, the gotchas. Start here. |
| [`docs/README.md`](docs/README.md) | The index: a spec pair for each of 15 components, and the reading path |
| [`docs/plans/PLAN-ros-alignment.md`](docs/plans/PLAN-ros-alignment.md) | The governing plan: every phase since 2026-09-25, with criteria written first and results recorded |
| [`docs/guides/FEATURES.md`](docs/guides/FEATURES.md) | Every feature of the twin, end to end |
| [`docs/guides/AGENT-HARNESS.md`](docs/guides/AGENT-HARNESS.md) | How the brain service works: the tick, seams, failsafes |
| [`docs/hardware/JETSON-BOM.md`](docs/hardware/JETSON-BOM.md) | What was bought, and what is left before the Rover drives |
| [`docs/handoffs/`](docs/handoffs/) | Session handoffs, dated; the newest names the open work |

The build journal this file used to carry (Phase 0 onward, the deleted ECS
deployment, and why Lambda once failed on this account) is in
`docs/archive/README-build-journal-2026-10-06.md`, verbatim.
