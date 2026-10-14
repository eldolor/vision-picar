"""One local port in front of the robot and the brain, so a single ngrok
free-tier domain can reach both.

ngrok's free plan gives ONE static domain per account -- two tunnels
contend for it and only one wins (ERR_NGROK_334). The project already
solved this shape once: robot/server.py and control/brain_server.py both
take a ROUTE_PREFIX so two instances can share one load balancer with
non-colliding paths. This is that arrangement with a 40-line proxy
standing in for the ALB.

    /brain/*  -> control/brain_server.py on :8001  (ROUTE_PREFIX=/brain)
    /*        -> robot/server.py on :8000          (no prefix)

**A proxy, not a merge.** Mounting both FastAPI apps in one uvicorn would
be less code and would put the agent loop on the same event loop the
watchdog polls -- exactly the failure PLAN-brain-relocation.md's "Why not
one process" describes. Two processes, one port.
"""
import os
from urllib.parse import quote

import httpx
from fastapi import FastAPI, Request, Response
from fastapi.responses import JSONResponse

ROBOT = os.environ.get("PROXY_ROBOT_URL", "http://127.0.0.1:8000")
BRAIN = os.environ.get("PROXY_BRAIN_URL", "http://127.0.0.1:8001")
# The DEPLOYED vision service, fanned in under /vision/* -- added 2026-09-22
# for a failure that only exists because of this tunnel.
#
# The serverless stack answers no OPTIONS at all (`OPTIONS /navigate` and
# `OPTIONS /health` both 404, with no access-control-allow-origin on any
# reply). That was invisible while the twin was SERVED from CloudFront,
# because the page and the service were then one origin and no preflight was
# ever issued. Serve the same page through ngrok and every vision call
# becomes cross-origin; `x-app-secret` makes it non-simple; the preflight
# 404s; and Safari reports the whole thing as a bare "Load failed".
#
# Driving via the brain hid it further, because the brain calls the vision
# service server-side where CORS does not apply -- so the bug appeared only
# with "Record this walk" ON and "Drive via brain" OFF, which is the one
# path where the PHONE talks to the vision service itself.
#
# Routing it through here makes those calls same-origin again. The proper
# fix is CORS on the serverless stack, which is a deploy and belongs in
# `tests/test_serverless_routes.py`'s remit -- this unblocks a rig session
# without one.
VISION = os.environ.get("PROXY_VISION_URL",
                        "https://d114x92g7i4syl.cloudfront.net")

app = FastAPI(title="vision-picar tunnel proxy")
# Long enough for a tiered tick: perception is local but the triggered
# /navigate call is a real round trip to Bedrock.
client = httpx.AsyncClient(timeout=120.0, follow_redirects=False)
# The largest legitimate body is a 4 MB frame (recording_routes
# MAX_FRAME_BYTES) as base64 in JSON, ~5.6 MB. The proxy reads a body
# before any secret is checked upstream, so without a cap one large POST
# from anyone who knows the domain is held whole in memory -- on the Jetson,
# 296 MB free under load (3.58) (3.76 H3-2).
MAX_BODY_BYTES = 16 * 1024 * 1024
# RFC 9110 7.6.1: these describe the phone's hop, not ours (3.76 H3-5).
ALLOW_HEADERS = "content-type,x-app-secret,ngrok-skip-browser-warning"
HOP_BY_HOP = {"connection", "keep-alive", "proxy-authenticate", "proxy-authorization",
              "te", "trailer", "transfer-encoding", "upgrade", "proxy-connection"}


# And across requests: 16 MB each does not bound twenty at once. Bodies in
# flight past this answer 503 (3.76 H3, the fixes' review).
MAX_INFLIGHT_BYTES = 48 * 1024 * 1024
_inflight = 0
# The proxy's OWN replies (413, 503, 502) reach the phone with no upstream
# CORS middleware in front; without this a cross-origin twin reads them as a
# bare "Load failed" instead of the detail (3.76 H3, the fixes' review).
_CORS = {"access-control-allow-origin": "*"}


def _error(status, detail):
    return JSONResponse({"detail": detail}, status_code=status, headers=_CORS)


def _too_large():
    return _error(413, f"body over the tunnel's {MAX_BODY_BYTES} bytes")


def _forward_path(raw: bytes, path: str, upstream: str) -> str:
    """The path as the phone sent it, still percent-encoded, as ASCII.

    `raw_path` keeps `%2F`/`%23` encoded (3.76 H3-4); a raw non-ASCII byte
    (httptools lets UTF-8 through) is percent-encoded here, since httpx
    would re-encode a latin-1 decoding of it as UTF-8 twice over."""
    text = quote(raw, safe="/%:@!$&'()*+,;=-._~") if raw else quote("/" + path)
    if upstream == VISION:
        if text == "/vision" or text.startswith("/vision/"):
            text = text[len("/vision"):]
        else:   # an encoded separator after the prefix: route as decoded
            text = quote(path[len("vision"):])
        text = "/" + text.lstrip("/")
    return text


@app.api_route("/{path:path}",
               methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "HEAD"])
async def proxy(path: str, request: Request):
    if path == "vision" or path.startswith("vision/"):
        upstream = VISION
        # Answer the preflight HERE rather than forwarding it. The upstream
        # 404s every OPTIONS, and a browser requires a 2xx for a preflight
        # regardless of the CORS headers on it -- so forwarding the 404
        # produced the identical "Load failed" with the headers now present,
        # which is a convincing near-miss and not a fix.
        if request.method == "OPTIONS":
            return Response(status_code=204, headers={
                "access-control-allow-origin": "*",
                # Echoed: the twin adds `ngrok-skip-browser-warning` on an
                # ngrok host, and a fixed list without it failed the
                # preflight the same way (3.76 H3-11).
                "access-control-allow-headers": request.headers.get(
                    "access-control-request-headers", ALLOW_HEADERS),
                "access-control-allow-methods": "GET,POST,PUT,DELETE,OPTIONS",
                "access-control-max-age": "600",
            })
    elif path == "brain" or path.startswith("brain/"):
        upstream = BRAIN
    else:
        upstream = ROBOT
    url = upstream + _forward_path(request.scope.get("raw_path") or b"", path, upstream)
    if request.url.query:
        url += "?" + request.url.query
    # Host must not be forwarded -- it is ngrok's, and passing it upstream
    # makes uvicorn's own redirects point back out through the tunnel.
    named = {t.strip().lower() for t in request.headers.get("connection", "").split(",")}
    headers = {k: v for k, v in request.headers.items()
               if k.lower() not in {"host", "content-length"} | HOP_BY_HOP | named}
    declared = request.headers.get("content-length", "")
    if declared.isdigit() and int(declared) > MAX_BODY_BYTES:
        return _too_large()
    global _inflight
    chunks, size = [], 0
    try:
        async for chunk in request.stream():
            size += len(chunk)
            _inflight += len(chunk)
            if size > MAX_BODY_BYTES:
                return _too_large()
            if _inflight > MAX_INFLIGHT_BYTES:
                return _error(503, "the tunnel is busy: too many large bodies in flight")
            chunks.append(chunk)
        body = b"".join(chunks)
        chunks = None
        try:
            r = await client.request(request.method, url, content=body, headers=headers)
        except (httpx.HTTPError, httpx.InvalidURL) as e:
            return _error(502, f"upstream {upstream} unreachable: {e}")
    finally:
        _inflight -= size
    drop = {"content-encoding", "content-length"} | HOP_BY_HOP
    out = {k: v for k, v in r.headers.items() if k.lower() not in drop}
    # Host is not forwarded, so an upstream's own redirect names
    # 127.0.0.1 -- unreachable from the phone. Keep it on the tunnel.
    loc = out.get("location", "")
    for origin in (ROBOT, BRAIN):
        if loc.startswith(origin + "/"):
            out["location"] = loc[len(origin):]
    # Same-origin is what removes the need for these, but the page may still
    # be on a different host from this proxy during split local dev -- and an
    # upstream that sends none of its own (the serverless stack sends none)
    # would then fail the same way through here. Answering them costs nothing
    # and is the whole reason /vision/* exists.
    # Only for the vision upstream: the robot and the brain answer CORS
    # themselves, and adding `*` here would override a narrowed
    # `server.allowed_origins` for everything through the tunnel (3.76 H3-16).
    # A robot or brain 500 comes from Starlette's ServerErrorMiddleware,
    # outside its CORS middleware, with no allow-origin: give it one, or a
    # cross-origin twin reads it as a bare "Load failed" (3.76 H3, round 3).
    if upstream != VISION and r.status_code >= 500:
        out.setdefault("access-control-allow-origin", "*")
    if upstream == VISION:
        out.setdefault("access-control-allow-origin", "*")
        out.setdefault("access-control-allow-headers", ALLOW_HEADERS)
        out.setdefault("access-control-allow-methods", "GET,POST,PUT,DELETE,OPTIONS")
    return Response(
        content=r.content, status_code=r.status_code, headers=out,
        media_type=r.headers.get("content-type"),
    )
