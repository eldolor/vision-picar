"""
sensor_server.py

The simulated body's sensors, as their own program (PLAN-ros-alignment.md
3.36): the lidar scan, the depth grid, the scalar distance and the camera
frame, cast from the state `sim/body_server.py` (physics) publishes to shared
memory (`sim/body_state.py`). Stateless apart from that, so it runs as
SEVERAL worker processes and the ray casting and rendering spread over the
free cores -- on the Jetson, one process doing all of it was the bottleneck
(3.33's G4):

    SIM_BODY_SHM=picar_sim_body uvicorn sim.sensor_server:app \\
        --host 127.0.0.1 --port 8003 --workers 3

Each worker holds a replica of the house (static walls from `sim.maps`, the
objects from the snapshot) and casts with the same `MockRobot` methods the
in-process simulator always used, so a reading is the reading the body
would have produced at that pose. `GET /safety` returns the scan the safety
layer asks for, the depth grid and the distance cast from ONE snapshot, so
the robot server's vet never mixes two instants.
"""

import os
import threading
from typing import Optional

from fastapi import Depends, FastAPI, Header, HTTPException

from sim.body_state import DEFAULT_NAME, StateReader


def require_secret(x_app_secret: str = Header(default="")):
    expected = os.environ.get("APP_SHARED_SECRET")
    if expected and x_app_secret != expected:
        raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")


def create_app() -> FastAPI:
    from sim.maps import build_world
    from sim.mock_robot import MockRobot

    reader = StateReader(os.environ.get("SIM_BODY_SHM", DEFAULT_NAME))
    lock = threading.Lock()          # one replica per worker; casts are serialised within it
    replica = {"house": None, "body": None}
    gate = [Depends(require_secret)]
    app = FastAPI(title="vision-picar sim sensors")

    def synced():
        """The replica moved to the latest published state. Call under `lock`."""
        state = reader.read()
        if state is None:
            raise HTTPException(status_code=503, detail="no consistent body state")
        if replica["house"] != state["house"]:
            replica["body"] = MockRobot(build_world(state["house"]))
            replica["house"] = state["house"]
        world = replica["body"].world
        world.x, world.y, world.theta = state["x"], state["y"], state["theta"]
        world.pan, world.sim_time = state["pan"], state["sim_time"]
        world.objects = state["objects"]
        return replica["body"], state["seq"]

    # Build the replica now, not on the first request: the first scans after
    # start-up used to time out on the Jetson while it was built (3.36).
    try:
        with lock:
            synced()
    except HTTPException:
        pass

    @app.get("/health")
    def health():
        state = reader.read()
        return {"ok": state is not None, "pid": os.getpid(),
                "house": state and state["house"], "seq": state and state["seq"]}

    @app.get("/scan", dependencies=gate)
    def scan(max_range_m: Optional[float] = None):
        with lock:
            body, _ = synced()
            return body.get_scan(max_range_m=max_range_m)

    @app.get("/depth", dependencies=gate)
    def depth():
        with lock:
            body, _ = synced()
            return body.get_depth_grid()

    @app.get("/distance", dependencies=gate)
    def distance():
        with lock:
            body, _ = synced()
            return {"distance_cm": body.get_distance()}

    @app.get("/frame", dependencies=gate)
    def frame():
        with lock:
            body, _ = synced()
            return body.get_camera_frame()

    @app.get("/safety", dependencies=gate)
    def safety(max_range_m: Optional[float] = None):
        """Everything `robot/safety.py` reads for one decision, from one
        snapshot: the hinted scan, the depth grid, the scalar distance."""
        with lock:
            body, seq = synced()
            return {"scan": body.get_scan(max_range_m=max_range_m),
                    "depth": body.get_depth_grid(),
                    "distance_cm": body.get_distance(), "state_seq": seq,
                    "pid": os.getpid()}

    return app


app = create_app()
