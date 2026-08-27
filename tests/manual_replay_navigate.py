"""
manual_replay_navigate.py

Stage 0 of CLAUDE.md's build order -- the go/no-go gate before any of the
vision work in PLAN-sim-hardening.md.

Not an automated test: it makes real, paid calls to the deployed vision
service. Point it at photographs of your own rooms and check whether
/navigate returns sane moves. Everything the simulation has "proven"
about the vision policy so far was proven against flat-shaded raycaster
renders (PLAN-sim-hardening.md 3.5); this is the only check available,
without buying a robot, that speaks to real-world accuracy.

What to shoot: stand where the robot would stand, phone held low (the
PiCar-X camera sits ~10cm off the floor -- a chest-height photo is not
the robot's view), and cover a spread of cases:
  - target clearly visible ahead        -> expect FORWARD
  - target visible off to one side      -> expect LEFT / RIGHT
  - wall or furniture close ahead       -> expect STOP or a turn
  - empty room, target absent           -> expect exploration, not STOP
  - doorway ahead                       -> expect FORWARD

Usage:
    export VISION_URL="https://<your-cloudfront-domain>"
    export APP_SHARED_SECRET="..."        # if the deployment has one set
    python -m tests.manual_replay_navigate photos/ "red backpack"

Reads a single image or every image in a directory. Prints one line per
photo plus a summary, so the output can be pasted into a findings note.
"""

import base64
import json
import os
import sys
from pathlib import Path

import httpx

SUFFIXES = {".jpg", ".jpeg", ".png", ".webp", ".gif"}
MEDIA_TYPES = {".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".png": "image/png",
               ".webp": "image/webp", ".gif": "image/gif"}


def collect(path: Path) -> list:
    if path.is_file():
        return [path]
    images = sorted(p for p in path.iterdir() if p.suffix.lower() in SUFFIXES)
    if not images:
        sys.exit(f"No images found in {path} (looked for {', '.join(sorted(SUFFIXES))})")
    return images


def navigate(url: str, secret: str, image: Path, target: str) -> dict:
    headers = {"Content-Type": "application/json"}
    if secret:
        headers["x-app-secret"] = secret
    body = {
        "image_base64": base64.b64encode(image.read_bytes()).decode(),
        "media_type": MEDIA_TYPES.get(image.suffix.lower(), "image/jpeg"),
        "target_object": target,
    }
    resp = httpx.post(url.rstrip("/") + "/navigate", headers=headers, json=body, timeout=60.0)
    if resp.status_code != 200:
        raise RuntimeError(f"HTTP {resp.status_code}: {resp.text[:200]}")
    return resp.json()


def main():
    if len(sys.argv) != 3:
        sys.exit("Usage: python -m tests.manual_replay_navigate <image-or-dir> <target object>")

    url = os.environ.get("VISION_URL")
    if not url:
        sys.exit("Set VISION_URL to the deployed vision service's base URL.")
    secret = os.environ.get("APP_SHARED_SECRET", "")
    target = sys.argv[2]

    images = collect(Path(sys.argv[1]))
    print(f"Replaying {len(images)} photo(s) against {url}/navigate, target={target!r}\n")

    actions, failures = {}, 0
    for image in images:
        try:
            result = navigate(url, secret, image, target)
        except Exception as e:  # noqa: BLE001 -- this is a manual harness; report and continue
            print(f"{image.name:<28} ERROR  {e}")
            failures += 1
            continue

        action = result.get("action", "?")
        actions[action] = actions.get(action, 0) + 1
        seen = "visible/" + str(result.get("target_direction")) if result.get("target_visible") else "not visible"
        print(f"{image.name:<28} {action:<8} target={seen:<20} {result.get('reasoning', '')}")

    print(f"\n{len(images) - failures} call(s) ok, {failures} failed")
    if actions:
        print("Action spread: " + json.dumps(actions))
    print(
        "\nJudge it yourself: does each action match what you would have done\n"
        "standing there? A run that returns STOP or FORWARD for everything is\n"
        "a failure even with no errors -- it means the model is not reading the\n"
        "scene. Record the result before starting Stage 1."
    )


if __name__ == "__main__":
    main()
