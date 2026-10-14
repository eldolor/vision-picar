"""
service/local_vlm/server.py

3.80.B (`docs/plans/ros-alignment/3.80-tiered-identity.md`): the local VLM
tier as its own process, so an OOM kill takes it and nothing else.

    LOCAL_VLM_ENGINES=~/bench-378/ws91/q25VE/engines \\
    LOCAL_VLM_EDGELLM=~/bench-378/ell91 \\
        python -m service.local_vlm.server

`POST /identity` answers 3.48's arrival question with the raw score; the
brain applies 3.80's thresholds. A 503 means the VLM is absent. If the driver
dies, this process exits non-zero and systemd decides whether to start it
again (`tools/jetson/systemd/picar-local-vlm.service`).
"""

from __future__ import annotations

import base64
import contextlib
import binascii
import logging
import os
import threading
import time
from pathlib import Path
from typing import Callable, Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from pydantic import BaseModel, Field

from service.local_vlm.driver import DriverGone, EdgeDriver

logger = logging.getLogger("local_vlm")

DEFAULT_PORT = 8005
READY_TIMEOUT_S = 300.0  # VE loads in ~40 s idle; 3.78.F saw 70 s under load
MAX_DEADLINE_S = 30.0


# A 640x480 frame is ~60 KB; the cap keeps a bad caller from filling
# /dev/shm, which is RAM.
MAX_IMAGE_B64 = 4_000_000


class IdentityRequest(BaseModel):
    image_b64: str = Field(max_length=MAX_IMAGE_B64)
    target: str = Field(min_length=1, max_length=200)
    deadline_s: float = Field(default=3.0, gt=0, le=MAX_DEADLINE_S)


def require_secret(x_app_secret: str = Header(default="")):
    """The same gate as robot/server.py's and the brain's: inert when
    APP_SHARED_SECRET is unset. /health stays open."""
    expected = os.environ.get("APP_SHARED_SECRET")
    if expected and x_app_secret != expected:
        raise HTTPException(status_code=401, detail="Missing or invalid x-app-secret header.")


def _exit_on_death() -> None:  # pragma: no cover -- ends the process
    logger.error("the driver is gone; exiting so systemd decides on a restart")
    logging.shutdown()
    os._exit(3)


# Between seeing the driver gone and ending the process: long enough for
# the request that found it to get its 503 out.
DEATH_DELAY_S = 0.5


def create_app(make_driver: Callable[[], EdgeDriver], *, info: Optional[dict] = None,
               on_death: Callable[[], None] = _exit_on_death,
               ready_timeout_s: float = READY_TIMEOUT_S,
               watch_interval_s: float = 1.0,
               death_delay_s: float = DEATH_DELAY_S) -> FastAPI:
    state: dict = {"driver": None, "load_s": None, "dead": False}

    def died(why: str) -> None:
        if not state["dead"]:
            state["dead"] = True
            logger.error("local VLM absent: %s", why)
            threading.Timer(death_delay_s, on_death).start()

    def watch(drv: EdgeDriver) -> None:
        while not state["dead"]:
            if not drv.alive:
                died(f"driver exited ({drv.proc.poll()})")
                return
            time.sleep(watch_interval_s)

    def load() -> None:
        try:
            _load()
        except Exception as e:  # noqa: BLE001 -- a driver that cannot start is a death
            logger.exception("local VLM failed to start")
            died(f"failed to start: {e}")

    def _load() -> None:
        t0 = time.monotonic()
        drv = make_driver()
        state["driver"] = drv
        if state["dead"]:  # shut down while the process was starting
            drv.close()
            return
        try:
            drv.wait_ready(ready_timeout_s)
        except DriverGone as e:
            died(str(e))
            return
        state["load_s"] = time.monotonic() - t0
        logger.info("local VLM ready in %.1f s", state["load_s"])
        watch(drv)

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        # Loaded off the event loop, so /health answers `ready: false`
        # while the engines load instead of not answering at all.
        threading.Thread(target=load, daemon=True).start()
        yield
        state["dead"] = True  # an exit we asked for is not a death
        if state["driver"] is not None:
            state["driver"].close()

    app = FastAPI(title="vision-picar local VLM", lifespan=lifespan)

    @app.get("/health")
    def health() -> dict:
        drv = state["driver"]
        alive = drv is not None and drv.alive and not state["dead"]
        return {"service": "local_vlm", "ready": bool(alive and drv.ready),
                "alive": alive, "load_s": state["load_s"],
                "served": drv.served if drv else 0, **(info or {})}

    @app.post("/identity", dependencies=[Depends(require_secret)])
    def identity(req: IdentityRequest) -> dict:
        drv = state["driver"]
        if drv is None or not drv.ready or state["dead"]:
            raise HTTPException(status_code=503, detail="local VLM not ready")
        try:
            jpeg = base64.b64decode(req.image_b64, validate=True)
        except (binascii.Error, ValueError):
            raise HTTPException(status_code=400, detail="image_b64 is not base64")
        if not jpeg.startswith(b"\xff\xd8"):
            raise HTTPException(status_code=400, detail="image is not a JPEG")
        try:
            return drv.ask(jpeg, req.target, req.deadline_s)
        except DriverGone as e:
            died(str(e))
            raise HTTPException(status_code=503, detail="local VLM absent")

    return app


def _decoder(tokenizer_path: Path) -> Callable[[int], str]:  # pragma: no cover
    from tokenizers import Tokenizer
    tok = Tokenizer.from_file(str(tokenizer_path))
    cache: dict = {}

    def decode(i: int) -> str:
        if i not in cache:
            cache[i] = tok.decode([i], skip_special_tokens=False)
        return cache[i]
    return decode


def main() -> None:  # pragma: no cover -- needs the board
    import uvicorn

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    engines = Path(os.environ["LOCAL_VLM_ENGINES"]).expanduser()
    edgellm = Path(os.environ["LOCAL_VLM_EDGELLM"]).expanduser()
    driver = edgellm / "build/examples/llm/edgellm_identity"
    plugin = edgellm / "build/libNvInfer_edgellm_plugin.so"
    tokenizer = engines / "llm/tokenizer.json"
    decode = _decoder(tokenizer)
    log = open(os.environ.get("LOCAL_VLM_DRIVER_LOG", os.devnull), "a")

    def make() -> EdgeDriver:
        return EdgeDriver([str(driver), str(engines / "llm"), str(engines / "visual")],
                          dict(os.environ, EDGELLM_PLUGIN_PATH=str(plugin)), decode, stderr=log)

    app = create_app(make, info={"engines": str(engines), "pid": os.getpid()})
    uvicorn.run(app, host="127.0.0.1", port=int(os.environ.get("LOCAL_VLM_PORT", DEFAULT_PORT)),
                log_level="info")


if __name__ == "__main__":  # pragma: no cover
    main()
