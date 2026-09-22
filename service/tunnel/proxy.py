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

import httpx
from fastapi import FastAPI, Request, Response

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


@app.api_route("/{path:path}",
               methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "HEAD"])
async def proxy(path: str, request: Request):
    if path == "vision" or path.startswith("vision/"):
        upstream, rest = VISION, path[len("vision"):].lstrip("/")
        # Answer the preflight HERE rather than forwarding it. The upstream
        # 404s every OPTIONS, and a browser requires a 2xx for a preflight
        # regardless of the CORS headers on it -- so forwarding the 404
        # produced the identical "Load failed" with the headers now present,
        # which is a convincing near-miss and not a fix.
        if request.method == "OPTIONS":
            return Response(status_code=204, headers={
                "access-control-allow-origin": "*",
                "access-control-allow-headers": "content-type,x-app-secret",
                "access-control-allow-methods": "GET,POST,PUT,DELETE,OPTIONS",
                "access-control-max-age": "600",
            })
    elif path == "brain" or path.startswith("brain/"):
        upstream, rest = BRAIN, path
    else:
        upstream, rest = ROBOT, path
    url = f"{upstream}/{rest}"
    if request.url.query:
        url += "?" + request.url.query
    # Host must not be forwarded -- it is ngrok's, and passing it upstream
    # makes uvicorn's own redirects point back out through the tunnel.
    headers = {k: v for k, v in request.headers.items()
               if k.lower() not in ("host", "content-length")}
    body = await request.body()
    try:
        r = await client.request(request.method, url, content=body, headers=headers)
    except httpx.HTTPError as e:
        return Response(content=f'{{"detail":"upstream {upstream} unreachable: {e}"}}',
                        status_code=502, media_type="application/json")
    drop = {"content-encoding", "content-length", "transfer-encoding", "connection"}
    out = {k: v for k, v in r.headers.items() if k.lower() not in drop}
    # Same-origin is what removes the need for these, but the page may still
    # be on a different host from this proxy during split local dev -- and an
    # upstream that sends none of its own (the serverless stack sends none)
    # would then fail the same way through here. Answering them costs nothing
    # and is the whole reason /vision/* exists.
    out.setdefault("access-control-allow-origin", "*")
    out.setdefault("access-control-allow-headers", "content-type,x-app-secret")
    out.setdefault("access-control-allow-methods", "GET,POST,PUT,DELETE,OPTIONS")
    return Response(
        content=r.content, status_code=r.status_code, headers=out,
        media_type=r.headers.get("content-type"),
    )
