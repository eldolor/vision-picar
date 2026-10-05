"""
body_server.py

The simulated body's PHYSICS, as its own program (PLAN-ros-alignment.md
3.36): the GridWorld house, the body's kinematics and collision, the movers,
furniture moves, the camera's pan, and the `FakeEsp32` turning the body's
wheels on a pseudo-terminal -- everything the car does not have and that
must stay in one place because it is the truth. Its sensors are cast by a
separate program, `sim/sensor_server.py`, from the state this one publishes
to shared memory (`sim/body_state.py`) every few milliseconds.

The robot server, started with `SIM_BODY_URL` (this) and `SIM_SENSORS_URL`
(the sensor program), opens the board's pty as a serial device and reads the
sensors over HTTP (`sim/body_client.py`), so the only code in its process is
the code the car runs.

Why three programs: on the Jetson, all of this inside the robot server sat
at 98% of one core and its 20 Hz wheel loop ran late on 15-20% of ticks; in
one separate body program it sat at 83% and every sensor read queued behind
the board loop (3.33, 3.36).

    SIM_MAP=scaled_house uvicorn sim.body_server:app --host 127.0.0.1 --port 8002

The body is built by `robot.factory.build_fake_body()` from the same env
vars as ever (`SIM_MAP`, `SIM_MOVERS`, `SIM_BOARD_FIRMWARE`,
`SIM_BOARD_SILENT_S`); `SIM_BODY_SHM` names the shared state block.
"""

import os
import threading
import time
from typing import List, Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from pydantic import BaseModel

from robot.factory import _DEFAULT_CONFIG, build_fake_body, load_config
from robot.identity import log_identity
from sim.body_state import DEFAULT_NAME, StateWriter

# The board loop runs at ~100 Hz; publishing at twice that means a sensor
# never casts from a state more than one board step old.
PUBLISH_S = 0.005


def require_secret(x_app_secret: str = Header(default="")):
    expected = os.environ.get("APP_SHARED_SECRET")
    if expected and x_app_secret != expected:
        raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")


class MoveObjectRequest(BaseModel):
    src: List[int]
    dst: List[int]


def create_app(config_path: Optional[str] = None) -> FastAPI:
    from sim.mock_world import MockWorld

    config_path = config_path or os.environ.get("ROBOT_CONFIG_PATH") or str(_DEFAULT_CONFIG)
    ident = log_identity("vision-picar sim body server", config_path)
    body, board = build_fake_body(load_config(config_path))
    grid = body.world
    truth = MockWorld(grid)
    shm_name = os.environ.get("SIM_BODY_SHM", DEFAULT_NAME)
    writer = StateWriter(shm_name, house=getattr(grid, "map_name", "") or "")
    gate = [Depends(require_secret)]
    stats = {"publishes": 0}

    def publish():
        # Under the board's lock: every mutation of the body (the board loop's
        # step, a pan, a furniture move) holds it, so the snapshot is one
        # instant, never a pose half-way through a sub-step.
        with board.lock:
            writer.publish(grid.x, grid.y, grid.theta, grid.pan, grid.sim_time, grid.objects)
        stats["publishes"] += 1

    def publisher():
        while True:
            publish()
            time.sleep(PUBLISH_S)

    publish()                                    # a state exists before anyone asks
    threading.Thread(target=publisher, daemon=True, name="body-state").start()

    app = FastAPI(title="vision-picar sim body")
    app.state.body, app.state.board, app.state.writer = body, board, writer

    @app.get("/health")
    def health():
        return {"ok": True, "identity": ident, "board_path": board.path,
                "sim_map": getattr(grid, "map_name", None), "state_shm": shm_name,
                "board_frames_out": board.frames_out, "state_publishes": stats["publishes"]}

    @app.post("/look/{side}", dependencies=gate)
    def look(side: str):
        turn = {"left": body.look_left, "right": body.look_right,
                "center": body.look_center}.get(side)
        if turn is None:
            raise HTTPException(status_code=404, detail="side is left, right or center")
        with board.lock:
            result = turn()
        publish()                                # the next sensor read sees the new pan
        return result

    @app.get("/truth", dependencies=gate)
    def get_truth():
        return truth.get_truth()

    @app.get("/sim/objects", dependencies=gate)
    def sim_objects():
        return grid.describe_objects()

    @app.post("/sim/objects/move", dependencies=gate)
    def sim_objects_move(req: MoveObjectRequest):
        if len(req.src) != 2 or len(req.dst) != 2:
            raise HTTPException(status_code=422, detail="src and dst are [x, y] cells")
        with board.lock:
            try:
                grid.move_object(req.src, req.dst)
            except ValueError as exc:
                raise HTTPException(status_code=409, detail=str(exc))
        publish()
        return grid.describe_objects()

    return app


app = create_app()
