"""
world/bridge_faults.py -- can waiting mend a ROS bridge error? (3.82)

One rule, shared by the places that decide it: `RosWorld` (the bridge's
own replies) and the robot server's goal routes, which pass the verdict on
as fields and headers -- never as prose a caller must match -- to
`RemoteNavigator` and `brain/explore.py`, which wait on a transient fault
and end the mission on any other. Before this each had its own rule.

A transport error or a 5xx may pass,
and so may 408, 409 and 429: the split `brain/navigate.py`'s
`CloudUnavailable` uses, plus 409, which the bridge answers while its SLAM
session changes. Any other 4xx -- a 401 for a mismatched secret, a 404
from a bridge without the route -- and a 200 that is not a JSON object (a
proxy's page; `httpx.DecodingError`), like a bridge URL with no scheme,
are faults in how the two are set up, which no wait mends; `hint()` says
what to check.
"""

from typing import Optional

import httpx

RETRYABLE_4XX = (408, 409, 429)
# The robot server's headers on a goal read or cancel it could not complete
# (its body stays the old `ros_unavailable: ...` detail).
RETRYABLE_HEADER = "x-bridge-retryable"
STATUS_HEADER = "x-bridge-status"


def retryable_status(status: Optional[int]) -> bool:
    """True for no status (a transport error), a 5xx, 408, 409 or 429."""
    return status is None or status >= 500 or status in RETRYABLE_4XX


def classify(exc: Exception) -> dict:
    """`{"bridge_status": int | None, "retryable": bool}` for an error from
    a bridge call."""
    if isinstance(exc, httpx.HTTPStatusError):
        status = exc.response.status_code
        return {"bridge_status": status, "retryable": retryable_status(status)}
    if isinstance(exc, httpx.TransportError) and not isinstance(exc, httpx.UnsupportedProtocol):
        return {"bridge_status": None, "retryable": True}
    # A 200 that is not JSON (DecodingError), a URL with no scheme
    # (UnsupportedProtocol), a redirect loop: how the two are set up.
    return {"bridge_status": None, "retryable": False}


def headers_for(fault: dict) -> dict:
    """The fault as the robot server's response headers."""
    out = {RETRYABLE_HEADER: "true" if fault["retryable"] else "false"}
    if fault["bridge_status"] is not None:
        out[STATUS_HEADER] = str(fault["bridge_status"])
    return out


def fault_from_headers(headers) -> Optional[dict]:
    """The robot server's verdict from its headers, or None when it sent
    none. A status header that is not a number is kept as None."""
    verdict = (headers or {}).get(RETRYABLE_HEADER)
    if verdict not in ("true", "false"):
        return None
    raw = (headers or {}).get(STATUS_HEADER)
    try:
        status = int(raw) if raw else None
    except ValueError:
        status = None
    return {"bridge_status": status, "retryable": verdict == "true"}


def hint(status: Optional[int]) -> str:
    """What an operator should check for a fault no wait mends."""
    if status in (401, 403):
        return "check that the robot server and the ROS container share APP_SHARED_SECRET"
    if status is None:
        return ("check ROS_BRIDGE_URL: it must be the bridge's own http:// address, "
                "and its reply must be JSON")
    return ("check ROS_BRIDGE_URL, and rebuild vision-picar-ros:latest from dev if the "
            "image is older than the robot server")


def setup_fault_message(what: str, fault: dict, detail: Optional[str] = None) -> str:
    """One wording for a fault no wait mends, wherever it is reported."""
    status = fault.get("bridge_status")
    why = f"HTTP {status}" if status is not None else "no usable reply"
    tail = f": {detail}" if detail else ""
    return (f"the ROS bridge refused {what} ({why}), a setup fault no wait mends -- "
            f"{hint(status)}{tail}")
