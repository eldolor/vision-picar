"""
body_server.py

The fake motor board's simulated body, in a process of its own
(PLAN-ros-alignment.md 3.36). It owns the GridWorld house, the `MockRobot`
body with its ray-cast camera, depth grid and lidar, the movers, and the
`FakeEsp32` turning the body's wheels on a pseudo-terminal -- everything the
car does not have. The robot server, started with `SIM_BODY_URL` pointing
here, opens the board's pty as a serial device and reads the sensors over
HTTP (`sim/body_client.py`), so the only code in its process is the code the
car runs.

Why: on the Jetson, with all of this inside the robot server, one Python
process sat at 98% of a core and the 20 Hz wheel loop ran late on 15-20% of
its ticks (3.33's G4). That measured the simulator, not the robot server.

    SIM_MAP=scaled_house uvicorn sim.body_server:app --host 127.0.0.1 --port 8002

The body is built by `robot.factory.build_fake_body()`, the same function
the in-process path uses, from the same env vars (`SIM_MAP`, `SIM_MOVERS`,
`SIM_BOARD_FIRMWARE`, `SIM_BOARD_SILENT_S`). Bound to localhost; the
`x-app-secret` gate applies when `APP_SHARED_SECRET` is set, as on the
robot server. Every route but /health is ground truth or a simulated
sensor: nothing outside the robot server should read it.
"""

import os
from typing import List, Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from pydantic import BaseModel

from robot.factory import _DEFAULT_CONFIG, build_fake_body, load_config
from robot.identity import log_identity


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
    gate = [Depends(require_secret)]
    app = FastAPI(title="vision-picar sim body")
    app.state.body, app.state.board = body, board

    @app.get("/health")
    def health():
        return {"ok": True, "identity": ident, "board_path": board.path,
                "sim_map": getattr(grid, "map_name", None),
                "board_frames_out": board.frames_out}

    @app.get("/scan", dependencies=gate)
    def scan(max_range_m: Optional[float] = None):
        return body.get_scan(max_range_m=max_range_m)

    @app.get("/depth", dependencies=gate)
    def depth():
        return body.get_depth_grid()

    @app.get("/distance", dependencies=gate)
    def distance():
        return {"distance_cm": body.get_distance()}

    @app.get("/frame", dependencies=gate)
    def frame():
        return body.get_camera_frame()

    @app.post("/look/{side}", dependencies=gate)
    def look(side: str):
        turn = {"left": body.look_left, "right": body.look_right,
                "center": body.look_center}.get(side)
        if turn is None:
            raise HTTPException(status_code=404, detail="side is left, right or center")
        with board.lock:
            return turn()

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
        return grid.describe_objects()

    return app


app = create_app()
