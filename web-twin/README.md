# Web twin

*Rewritten 2026-09-28. The previous version described a page deleted on
2026-09-25 (a hardcoded map, Explore/Find, Vision Autopilot, the Camera
tab and the local brain); `git log -- web-twin/README.md` has it.*

The mobile-first web UI for the robot. It is a client of two servers --
`robot/server.py` (the body: moves, sensors, safety, map) and
`control/brain_server.py` (the autonomy loop) -- and holds no movement,
safety or decision logic of its own. For how each button works end to
end, read `FEATURES.md`; this file is how to run, reach, deploy and test
the page.

## Files

| File | What it is |
|---|---|
| `index.html` | markup and CSS |
| `app.js` | all behaviour |
| `manifest.json`, `icons/` | PWA manifest and home-screen icons |

Locally `robot/server.py` serves all of them (`/`, `/app.js`,
`/manifest.json`, the icons). Deployed, CloudFront serves them from the
serverless stack's static bucket.

## The page

Two tabs, plus Settings behind the gear in the header (the gear returns
you to the tab you came from):

- **Guide** -- the phone's real camera. Two modes: **Guide me** steers
  the person holding the phone toward an object (`/guidance`), and
  **Robot view** shows the move the robot would make from here
  (`/navigate`). Robot view has two switches: **Record this walk** saves
  each frame to the brain's walk store, and **Drive via brain** runs a
  real `MissionRunner` mission on the phone's frames (`mode: teleop`,
  `PLAN-teleop-robot.md`). Camera access needs `https://` or `localhost`.
- **Sim** -- the simulated robot: D-pad, turn step, look left/right, the
  server-rendered camera frame, the depth strip, the discovered map (tap
  it to send a nav2 goal under `WORLD_MODE=ros`), and the **Remote brain**
  panel -- policy picker (rule-based, vision, tiered), failsafe drills,
  mission readouts.
- **Settings** -- robot and brain URLs and secrets, cloud endpoint,
  health check, setup code.

Every D-pad `/action` carries `x-driver: twin-dpad`, which outranks the
brain (M4), so a tap during a mission ends it `preempted` and the car does
what the pad said.

## Run locally

From the repo root:

```bash
uvicorn robot.server:app --port 8000
uvicorn control.brain_server:app --port 8001
```

Open `http://127.0.0.1:8000/` and connect both in Settings
(`http://127.0.0.1:8001` for the brain). Add `--host 0.0.0.0` to reach it
from a phone on the same Wi-Fi, but the camera features will not work
over a plain `http://<lan-ip>`. Restart the robot server to put the robot
back at its start; there is no reset endpoint, as real hardware has none.

## Reach it from a phone

The deployed twin (HTTPS, CloudFront) reaches the robot and brain on your
laptop through one tunnel:

```bash
bash service/tunnel/run.sh            # grid world; ROBOT_MODE=teleop for the rig
ngrok start picar                     # in another terminal
```

In Settings: robot URL `https://<domain>`, brain URL
`https://<domain>/brain`, and `LOCAL_SECRET` from
`~/.vision-picar-local-secrets` for both. It is one ngrok domain because
the free plan allows one (a second endpoint is `ERR_NGROK_334`);
`service/tunnel/proxy.py` sends `/brain/*` to the brain and everything
else to the robot. `run.sh` sets `APP_SHARED_SECRET`, which matters:
the tunnel puts both servers on the public internet.

ngrok's free tier answers browser user-agents with an HTML interstitial,
so `app.js` sends `ngrok-skip-browser-warning` -- only to ngrok hostnames,
because it is a custom header and would otherwise force a CORS preflight
on every LAN poll.

## Deploy

```bash
bash service/static/sync.sh <static-bucket> <distribution-id>
```

Both values are outputs of the `vision-picar-serverless` stack.
`service/static/assets.json` is the list of files uploaded, with each
one's content type and cache policy; a file the page references but the
list omits is a silent 404 (`tests/test_static_assets.py` checks this).
Wait for the CloudFront invalidation to finish, then check parity rather
than assuming it:

```bash
curl -s https://<distribution>/app.js | diff - <(git show HEAD:web-twin/app.js)
```

## Test

```bash
python -m playwright install chromium     # once
pytest tests/test_ui.py tests/test_ui_admin.py tests/test_ui_pipeline.py -v
```

They drive the real page in a real browser at a 390px phone viewport,
and **skip, not fail, without a browser** -- so a green run without
Chromium installed has tested nothing here. Run them in module order, not
only with `-k`: `test_ui_pipeline.py` has timing tests that have passed
alone and failed alongside their neighbours.

## Sharing setup with another device

Settings -> **Show setup code** shows a QR of the `?secret=`/
`?robotSecret=` magic link, plus the service URLs when they differ from
what the receiving page would derive; scanning it opens the app fully
configured. The QR *is* the secrets, so it appears only on a tap, hides
itself after 60 seconds or when you leave the tab, and anyone who
photographs it has your access. The encoder is inline in `app.js`
(`qrEncode()`) rather than a library or a QR web service, because handing
these secrets to a third-party image API would be a leak; it was verified
module-for-module against a reference implementation and decoded with
zxing-cpp.

## Failsafe drills

The Remote brain panel's drill picker exists because the two guards that
matter most on hardware cannot be provoked by pressing anything. Each
drill (`control/drills.py`) breaks one thing and leaves every other guard
standing:

| Drill | Breaks | What you should see |
|---|---|---|
| Vision service errors (`vision_error`) | every vision call raises | three failures counted, then `failed`, robot stopped |
| Vision service hangs (`vision_hang`) | every vision call never returns | same, but the reason says "timed out" |
| Brain loop hangs (`tick_hang`) | the loop stops returning | one step, then `failed` -- "brain loop hung" |

The third guard, `robot/server.py`'s watchdog (1.0 s,
`watchdog_timeout_s`), needs no drill: drive with the D-pad, stop
touching it, and the **Robot watchdog** readout counts the silence past
the timeout and reports the motors stopped. `brain.allow_drills: false`
in `config/robot.yaml` removes the drills; `AGENT-HARNESS.md` section 6
is the reference for all three failsafes.
