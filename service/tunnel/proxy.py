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

app = FastAPI(title="vision-picar tunnel proxy")
# Long enough for a tiered tick: perception is local but the triggered
# /navigate call is a real round trip to Bedrock.
client = httpx.AsyncClient(timeout=120.0, follow_redirects=False)


@app.api_route("/{path:path}",
               methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "HEAD"])
async def proxy(path: str, request: Request):
    upstream = BRAIN if (path == "brain" or path.startswith("brain/")) else ROBOT
    url = f"{upstream}/{path}"
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
    return Response(
        content=r.content, status_code=r.status_code,
        headers={k: v for k, v in r.headers.items() if k.lower() not in drop},
        media_type=r.headers.get("content-type"),
    )
