"""AWS Lambda entry point for the vision service.

`service/vision_analyze/app.py` is an ordinary FastAPI app and stays that
way -- Mangum adapts ASGI to the Lambda event/response shape, so the same
code runs under `uvicorn` locally, in a container, or here. Nothing in the
app knows which.

## Why this exists rather than a Function URL

Read `PLAN-aws-cost-redesign.md` section 6 before changing how this is
invoked. Measured on 2026-09-04: **Lambda resource-based policies do not
grant invocation on this account.** Anonymous Function URLs, CloudFront
with Origin Access Control, and any principal holding only a
resource-policy grant all return 403 without the function ever running.
Identity-based auth works normally.

So this function is reached through an **API Gateway HTTP API whose
integration carries an explicit `credentials` role**. The gateway assumes
that role and invokes with identity-based auth, never consulting a
resource policy -- the one path that works here. Do not "simplify" this to
a Function URL; it will fail, and it will fail silently in the sense that
the function is never invoked and there is nothing in its logs to look at.

## The 6MB rule

A Lambda response is capped around 6MB. This app returns JSON derived from
model output and is nowhere near it. Its sibling (`walks_handler.py`) is
not, which is why the walk download redirects rather than returning bytes.
"""

from mangum import Mangum

# Flat import, not `service.vision_analyze.app`: this service keeps
# dependency-light copies of vision_core.py/rooms_core.py and imports them as
# top-level modules, exactly as its Dockerfile's one-WORKDIR layout does.
# build.sh therefore unpacks app.py/vision_core.py/rooms_core.py at the zip
# ROOT. Packaging them under a package directory makes `from vision_core
# import ...` fail at cold start with a ModuleNotFoundError.
from app import app

# lifespan="off": there is no ASGI startup/shutdown work in this app, and
# leaving it on makes every cold start run a lifespan protocol handshake
# that can only fail in ways that are hard to see from a Lambda log.
handler = Mangum(app, lifespan="off")
