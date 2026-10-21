"""
sim/fake_camera.py

3.90 A: a stand-in for the Rover's USB camera, for `robot/camera_stream.py`.
`SIM_CAMERA=fake` (read by `robot/factory.py` under `mode: hardware`) makes
the car's camera this source: JPEGs at `SIM_CAMERA_FPS` (30), 1280x720,
paced on the wall clock the way a UVC camera delivers MJPEG.

It exists for the stream's own measurements -- frame age, the reader's CPU,
the wheel loop beside it (3.90 criteria 1 and 3) -- and nothing else. Its
pictures are noise and a frame number, not a house: a mission run with it
sees nothing, by design. The camera's own JPEG is handed over as bytes, as
`OpenCvSource` does when the camera's MJPEG passes through, so this measures
the pass-through path; a camera that needs decoding costs more, which only
the Rover's camera can say (3.90 C).
"""

import io
import time

DEFAULT_FPS = 30.0
WIDTH, HEIGHT = 1280, 720
# Distinct pictures cycled through, so no two consecutive reads are the
# same bytes. Made once, at open: the source must not spend CPU per frame
# that a real camera would not.
PICTURES = 30


def _pictures(n: int) -> list:
    import numpy as np
    from PIL import Image, ImageDraw

    rng = np.random.default_rng(390)
    out = []
    for i in range(n):
        noise = rng.integers(0, 255, size=(HEIGHT // 8, WIDTH // 8, 3), dtype=np.uint8)
        img = Image.fromarray(noise).resize((WIDTH, HEIGHT))
        ImageDraw.Draw(img).text((40, 40), f"fake camera {i}", fill=(255, 255, 255))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        out.append(buf.getvalue())
    return out


class FakeCameraSource:
    def __init__(self, fps: float = DEFAULT_FPS):
        self.period = 1.0 / fps
        self._pictures = _pictures(PICTURES)
        self._next = time.monotonic()
        self._i = 0

    def read(self) -> bytes:
        self._next += self.period
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self._next = time.monotonic()    # fell behind: no burst to catch up
        self._i += 1
        return self._pictures[self._i % len(self._pictures)]

    def close(self) -> None:
        pass
