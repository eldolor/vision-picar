# Web-based digital twin

A self-contained, mobile-first web page visualizing and controlling the
grid-world simulation -- open `index.html` directly in Safari on your
iPhone (or any browser). No server, no install, no App Store.

## What it is

- **Canvas view** of the starter house map -- rooms, doors, the sofa/
  fridge/backpack, the robot (with a "safety collar" ring showing the
  live collision-avoidance radius, flashing orange on a veto).
- **Manual D-pad control** and **autonomous "Explore"/"Find backpack"
  modes** -- both now call `robot/server.py` over HTTP for every move,
  sensor read, and safety check. No physics or safety logic is
  duplicated in JS anymore.
- **"Find the bag in a photo"** -- take a real photo with your phone
  camera, send it to the vision-analysis service
  (`service/vision_analyze/`, running on ECS Fargate), and see the same
  structured scene description the sim uses, plus a room guess.

## Architecture: this is now a real client of robot/server.py

Movement, sensing, and safety enforcement all happen server-side --
exactly mirroring the MacBook(brain)/Pi(robot) split in the main build
plan. The autonomous exploration *decision* logic (frontier-preference:
peek right/left/forward, prefer unvisited cells) intentionally still
lives in this file's JS, not on the server -- that's correct, not a
shortcut: deciding what to do next is the "brain" role, and the browser
is playing that role here the same way `brain/agent.py`'s `MissionAgent`
does in Python. Only the "robot runtime" (movement execution, distance
sensing, safety veto) moved server-side.

What's still hardcoded client-side, and why that's fine: the `LAYOUT`
and `OBJECTS` constants are for **drawing the map only** -- there's no
`GET /map` endpoint (real hardware has no grid to fetch), so this is
inherent to visualizing a simulation, not duplicated decision logic.

## Running it

1. Start the robot server: `uvicorn robot.server:app --host 0.0.0.0`
   (from the repo root). `--host 0.0.0.0` is required for your phone to
   reach it over Wi-Fi -- `127.0.0.1` only accepts connections from the
   same machine.
2. Find your Mac's LAN IP (System Settings -> Wi-Fi -> Details, or
   `ipconfig getifaddr en0` in Terminal).
3. Open `index.html` in Safari (on the same Wi-Fi network).
4. In "Robot server connection," enter `http://<your-mac-ip>:8000` and
   tap Connect.
5. D-pad, Explore, and Find backpack all now drive the real server.

## Known limitations

- No persistence -- reloading the page loses the server URL, the
  vision service URL/secret fields, and all mission bookkeeping (visited cells,
  search memory). The *robot's* actual position persists server-side
  since it's not reset on reconnect.
- "Reset mission" clears the twin's local bookkeeping only -- it does
  not (and, matching real hardware, cannot) teleport the robot back to
  a start position. Restart `uvicorn` to reset the sim's position.
- The robot-orientation triangle uses the server's `facing` field, which
  is camera-pan-adjusted (same ambiguity as `sim/grid_world.py`'s
  `frame_description()`) -- during a manual look-left/right tap the
  triangle reflects the camera pan, not strictly chassis heading. Purely
  cosmetic, doesn't affect movement or safety.
