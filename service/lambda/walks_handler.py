"""AWS Lambda entry point for the recorded-walk service.

Serves `control/admin_server.py`: the read/score/replay half of the
recording API, plus -- since the walks left EFS -- the write half too
(`control/recording_routes.py` mounts `/recording/frame` and
`/recording/finish` here).

## Why the write path is here and not on the brain

It used to live in `control/brain_server.py`, because the brain was the
process with the EFS volume attached. Storage is now a bucket, so the
question is credentials rather than mounts, and the brain is going back to
the Pi where `PLAN-brain-relocation.md` always said it belonged. A robot on
someone's floor should not be carrying AWS credentials in order to store a
JPEG. The write path follows the storage; the mission loop does not follow
either. See `control/recording_routes.py` for the full argument, which is
the same one `PLAN-teleop-robot.md` already made for the recording proxy.

## Configuration

Storage comes from the environment, not from a baked config file, so one
artifact serves every deployment:

    RECORDING_BACKEND=s3
    RECORDING_BUCKET=<the recordings bucket>
    RECORDING_PREFIX=recordings        (default)

`control/brain_config.py` reads those. With none of them set this falls
back to a local directory, which is what a developer running the same code
under `uvicorn` gets.

## Invocation, and the 6MB rule

Reached through an API Gateway HTTP API with a `credentials` role on the
integration -- NOT a Function URL. See `vision_handler.py`'s docstring and
`PLAN-aws-cost-redesign.md` section 6 for why that distinction is load
bearing on this account.

A Lambda response is capped around 6MB, and
`GET /recording/walks/{walk}/download` zips a whole walk: two of the 39
walks in the corpus are 8.64MB and 6.68MB. That route therefore writes the
zip to the bucket and returns a 307 to a presigned URL
(`WalkStore.download_url`), rather than returning bytes it cannot fit.
"""

import os
from pathlib import Path

from mangum import Mangum

from control.admin_server import create_app

# The app normally reads config/robot.yaml for its `brain:` block. In Lambda
# the only thing it needs from there is storage, and that arrives as
# RECORDING_* environment variables which brain_config layers on top of its
# defaults -- so point it at the repo's config if the file shipped, and let
# it use pure defaults if it did not. Either way the env wins.
_CONFIG = Path(__file__).resolve().parents[2] / "config" / "robot.yaml"

app = create_app(config_path=str(_CONFIG) if _CONFIG.is_file() else None)

handler = Mangum(app, lifespan="off")


def _assert_configured():
    """Fail loudly at import time if this was deployed without a bucket.

    The alternative is a function that starts cleanly, writes every recorded
    frame to the container's ephemeral disk, and loses them when it freezes
    -- which is the exact failure `ALLOW_RECORDING` was added to prevent on
    the teleop brain, one layer down.
    """
    if os.environ.get("RECORDING_BACKEND", "").lower() == "s3" and \
            not os.environ.get("RECORDING_BUCKET"):
        raise RuntimeError(
            "RECORDING_BACKEND=s3 but RECORDING_BUCKET is unset -- refusing to "
            "start rather than write recordings to an ephemeral disk."
        )


_assert_configured()
