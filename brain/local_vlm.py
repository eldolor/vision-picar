"""
brain/local_vlm.py

3.80.C (`docs/plans/ros-alignment/3.80-tiered-identity.md`): the brain's
client for the local VLM service (`service/local_vlm/`, 3.80.B), the middle
tier between perception and the cloud.

`ask(frame)` returns the service's answer -- `status` and the raw `score`;
the thresholds are the policy's (`brain/tiered.py`). It raises
`LocalVLMAbsent` only when there is no service to answer: a refused
connection or a 503 (loading, or its driver gone). A hang is not absence:
the client's own timeout comes back as status `timeout`, which the policy
counts as unable, so a wedged service costs `tier_local_max_unable`
questions and then the cloud is asked.
"""

from __future__ import annotations

import base64
import io
import logging
from typing import Optional

import httpx

logger = logging.getLogger(__name__)

# B6 (2026-10-14): a deadline that falls inside the driver's image encode or
# prefill is felt by the NEXT request, which waits about 1 s for the
# cancelled one to end. The HTTP timeout allows the deadline plus that tail,
# so a healthy service is never cut off by its own client.
TAIL_S = 2.0


class LocalVLMAbsent(RuntimeError):
    """No local VLM to ask: not running, still loading, or its driver died."""


def _jpeg(frame: dict) -> Optional[bytes]:
    """The frame's pixels as JPEG bytes (the service takes JPEG only), or
    None if it has none."""
    b64 = frame.get("image_base64")
    if not b64:
        return None
    raw = base64.b64decode(b64)
    if raw.startswith(b"\xff\xd8"):
        return raw
    from PIL import Image  # a teleop phone may send PNG or HEIC-decoded frames
    out = io.BytesIO()
    Image.open(io.BytesIO(raw)).convert("RGB").save(out, format="JPEG", quality=90)
    return out.getvalue()


class LocalVLMClient:
    def __init__(self, url: str, target: str, *, secret: Optional[str] = None,
                 deadline_s: float = 3.0, client: Optional[httpx.Client] = None):
        self.url = url.rstrip("/")
        self.target = target
        self.secret = secret
        self.deadline_s = float(deadline_s)
        self.timeout_s = self.deadline_s + TAIL_S
        self._http = client or httpx.Client(timeout=self.timeout_s)

    def ask(self, frame: dict) -> dict:
        """`{"status", "score", ...}`; status `answered`, or an unable one
        (`timeout`, `unparsed`, `busy`, `error`). Raises LocalVLMAbsent."""
        try:
            jpeg = _jpeg(frame)
        except Exception as e:  # noqa: BLE001 -- a frame we cannot encode is unable, not absent
            return {"status": "error", "score": None, "reason": f"frame not encodable: {e}"}
        if jpeg is None:
            return {"status": "error", "score": None, "reason": "frame has no image"}
        headers = {"x-app-secret": self.secret} if self.secret else {}
        body = {"image_b64": base64.b64encode(jpeg).decode(), "target": self.target,
                "deadline_s": self.deadline_s}
        try:
            r = self._http.post(f"{self.url}/identity", json=body, headers=headers,
                                timeout=self.timeout_s)
        except httpx.TimeoutException:
            return {"status": "timeout", "score": None,
                    "reason": f"no reply in {self.timeout_s:g} s"}
        except httpx.HTTPError as e:
            # Refused, reset or dropped: the service is not there (an OOM
            # kill drops the connection mid-request).
            raise LocalVLMAbsent(f"local VLM unreachable: {e}") from e
        if r.status_code == 503:
            raise LocalVLMAbsent(f"local VLM not ready: {r.text[:200]}")
        if r.status_code != 200:
            # A wrong secret or a request it refuses is our fault; unable, so
            # the cloud is still reached after max_unable, and loud.
            logger.warning("local VLM answered %d: %s", r.status_code, r.text[:200])
            return {"status": "error", "score": None, "reason": f"HTTP {r.status_code}"}
        try:
            got = r.json()
        except ValueError:
            return {"status": "error", "score": None, "reason": "reply is not JSON"}
        return got if isinstance(got, dict) else {"status": "error", "score": None,
                                                  "reason": "reply is not an object"}

    __call__ = ask

    def close(self) -> None:
        self._http.close()
