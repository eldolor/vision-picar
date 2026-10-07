# Specifications

Every component of the robot has two specifications
([decision 0001](decisions/0001-architecture-vs-engineering-specs.md)):

- **Architecture** (`docs/<domain>/ARCHITECTURE.md`) covers what the
  component is and why it is shaped that way: its boundaries, its
  decisions and the alternatives rejected, its contracts with neighbours,
  and its failure modes. It stays true if the implementation is rewritten.
- **Engineering** (`docs/engineering/<domain>/ENGINEERING.md`) covers how
  it is built today: modules, signatures, routes, config keys, thresholds,
  procedures and the tests that pin them. It is true only until the code
  changes, and changes in the same commit as the code.

The heuristic: if a competent engineer could build it more than one
reasonable way and the document does not care which, it is architecture.
Once it commits to one way, it is engineering.

`tools/spec_lint.py` checks the rules a script can check, and
`tests/test_spec_lint.py` runs it in the suite. `SPEC_REVIEW_PROMPT.md`
covers the rules that need judgement. New specs start from
`templates/`.

The `PLAN-*.md` documents in [plans/](plans/) are the dated history: criteria
written first, then results, then corrections. Specs describe each
component as it is now and cite the plan sections that record how it got
there.

## Reading path

0. **[Architecture overview](ARCHITECTURE.md)** is the whole system on
   one page: the five processes, the four walls, the ROS container, and
   one command traced end to end. Start there, then read the specs below.
1. **[Decision 0001](decisions/0001-architecture-vs-engineering-specs.md)**
   explains why there are two documents per component.
2. **The two interfaces everything else is built around.** Read
   [body](body/ARCHITECTURE.md) (what the robot says about itself) and
   [world](world/ARCHITECTURE.md) (what is true about the house, and where
   the robot is in it).
3. **[Safety](safety/ARCHITECTURE.md)** covers what may move the wheels,
   and who outranks whom.
4. **The two processes:**
   - [control-api](control-api/ARCHITECTURE.md) is the robot server.
   - [mission](mission/ARCHITECTURE.md) is the brain service and its
     failsafes.
5. **What decides each move:** [policy](policy/ARCHITECTURE.md),
   [perception](perception/ARCHITECTURE.md) and
   [cloud-vision](cloud-vision/ARCHITECTURE.md).
6. **Below the body:** [ros](ros/ARCHITECTURE.md) (the container behind
   the wall), [motor-board](motor-board/ARCHITECTURE.md) and
   [platform](platform/ARCHITECTURE.md) (the hardware).
7. **Around it:** [simulator](simulator/ARCHITECTURE.md),
   [twin](twin/ARCHITECTURE.md), [recordings](recordings/ARCHITECTURE.md)
   and [operations](operations/ARCHITECTURE.md).

## Domains

| Domain | Architecture | Engineering | Covers |
|---|---|---|---|
| body | [body/](body/ARCHITECTURE.md) | [engineering/body/](engineering/body/ENGINEERING.md) | `RobotInterface`, the body contract, and its backends |
| world | [world/](world/ARCHITECTURE.md) | [engineering/world/](engineering/world/ENGINEERING.md) | `WorldInterface`: the map, the pose, goals, and the body/world line |
| safety | [safety/](safety/ARCHITECTURE.md) | [engineering/safety/](engineering/safety/ENGINEERING.md) | The local safety layer, driver arbitration and the watchdog |
| control-api | [control-api/](control-api/ARCHITECTURE.md) | [engineering/control-api/](engineering/control-api/ENGINEERING.md) | The robot server: the HTTP API the brain, the twin and ROS talk to |
| mission | [mission/](mission/ARCHITECTURE.md) | [engineering/mission/](engineering/mission/ENGINEERING.md) | The brain service: mission lifecycle, the tick, failsafes |
| policy | [policy/](policy/ARCHITECTURE.md) | [engineering/policy/](engineering/policy/ENGINEERING.md) | What decides each move: frontier, vision, tiered, and arrival |
| perception | [perception/](perception/ARCHITECTURE.md) | [engineering/perception/](engineering/perception/ENGINEERING.md) | The on-board perception tier: detector, encoder and gate |
| cloud-vision | [cloud-vision/](cloud-vision/ARCHITECTURE.md) | [engineering/cloud-vision/](engineering/cloud-vision/ENGINEERING.md) | The vision service on Bedrock: `/navigate` and its siblings |
| ros | [ros/](ros/ARCHITECTURE.md) | [engineering/ros/](engineering/ros/ENGINEERING.md) | The ROS 2 container behind the HTTP wall |
| motor-board | [motor-board/](motor-board/ARCHITECTURE.md) | [engineering/motor-board/](engineering/motor-board/ENGINEERING.md) | The ESP32 driver board, its firmware fork, and the serial backend |
| platform | [platform/](platform/ARCHITECTURE.md) | [engineering/platform/](engineering/platform/ENGINEERING.md) | The hardware: compute board, chassis, sensors, power |
| simulator | [simulator/](simulator/ARCHITECTURE.md) | [engineering/simulator/](engineering/simulator/ENGINEERING.md) | The grid-world sim, its renderer, maps and movers |
| twin | [twin/](twin/ARCHITECTURE.md) | [engineering/twin/](engineering/twin/ENGINEERING.md) | The web twin: the phone UI over both servers |
| recordings | [recordings/](recordings/ARCHITECTURE.md) | [engineering/recordings/](engineering/recordings/ENGINEERING.md) | Recorded walks, their storage, and the evaluation instruments |
| operations | [operations/](operations/ARCHITECTURE.md) | [engineering/operations/](engineering/operations/ENGINEERING.md) | Deployment, the tunnel, secrets, health and metrics |

## Beyond the specs

Everything else that used to sit at the repo root, grouped by what it is.
None of these are specs, and the linter skips them. They are cited by file
name, and every name is unique, so a search for a name still finds it.

| Folder | Holds |
|---|---|
| [plans/](plans/) | `PLAN-*.md`: each phase's criteria, results and corrections, dated. `PLAN-ros-alignment.md` is the governing plan; its 3.N sections are one file each in `plans/ros-alignment/` |
| [hardware/](hardware/) | What to buy and why: `JETSON-BOM.md` (the build), `HARDWARE-BOM.md` (parts, wiring, the board protocol), `HARDWARE-READINESS.md`, `BOM-COMPARISON.md` (verified prices), `PI-VS-JETSON.md`, `GUIDE-robot-base.md`, `UGV-ROVER-MOUNTING.md` (the Rover's deck hole pattern, from Waveshare's CAD), and the superseded `BOM.md` |
| [guides/](guides/) | Explainers: `INTRODUCTION.md`, `FEATURES.md` (every UI feature end to end), `AGENT-HARNESS.md` (how `control/` works), `PARALLEL-SESSIONS.md` (several sessions on the plan at once: one branch per section) |
| [evaluations/](evaluations/) | Write-ups of one-off measurements: the edge perception bench and the 2026-09-22 navigate-model evaluation |
| [handoffs/](handoffs/) | Session handoffs, dated; the newest says what is open |
| [archive/](archive/) | Older material moved out of `CLAUDE.md`, verbatim: `CLAUDE-history-2026-09.md` (the dated reversals) `CLAUDE-2026-10-06.md` (the whole file before its 2026-10-06 rewrite), and the 2026-10-06 copies of `JETSON-BOM.md` and the root README's build journal |

## Decisions

| # | Decision |
|---|---|
| [0001](decisions/0001-architecture-vs-engineering-specs.md) | Architecture specs and engineering specs are separate documents |
