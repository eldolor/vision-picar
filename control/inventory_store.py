"""Where a mission's object inventory goes when the mission ends
(PLAN-ros-alignment.md 3.46 D).

Always to a local file, and to S3 when a bucket is configured -- decided by
the user 2026-10-07. The bucket is the recordings bucket
(`cloudformation/recordings-s3.yaml`: private, encrypted, versioned), under
its own prefix:

    <local_dir>/<mission_id>.json
    s3://<bucket>/<prefix>/<robot>/<mission_id>.json

What is stored is `brain.inventory.Inventory.report()`: labels, map
positions, beliefs and votes -- **never an image**. It is still a list of
what is in someone's home, which is why it goes to the private bucket and
nowhere else.

**A failure here never fails a mission.** The mission is over when this
runs; `save()` records what went wrong in its return value and the log, and
the robot is already stopped either way.
"""
from __future__ import annotations

import contextlib
import json
import logging
import os
import re
import socket
import time
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_SAFE = re.compile(r"[^a-z0-9._-]+")


def _fsync(fd: int) -> None:
    """`os.fsync`, named here so a test can fail it for this module only."""
    os.fsync(fd)


def _fsync_dir(path: Path) -> None:
    """Make a rename in `path` durable. Best effort: not every platform can
    open a directory for fsync, and the file itself is already on disk."""
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        _fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)


STALE_TMP_S = 3600


def _sweep_stale_tmp(directory: Path) -> None:
    """A save cut off by a crash or a power loss leaves `.<name>.tmp`
    behind (no handler runs), and mission ids are unique, so nothing ever
    overwrites it. Remove those older than an hour -- never a fresh one,
    which may be another mission's save in progress on its own thread."""
    cutoff = time.time() - STALE_TMP_S
    for tmp in directory.glob(".*.json.tmp"):
        with contextlib.suppress(OSError):
            if tmp.stat().st_mtime < cutoff:
                tmp.unlink()


def _slug(text: str) -> str:
    return _SAFE.sub("-", str(text).lower()).strip("-")[:60] or "x"


class InventoryStore:
    def __init__(self, local_dir: str, bucket: str = "", prefix: str = "inventory",
                 robot: Optional[str] = None, client=None):
        self.local_dir = Path(local_dir)
        self.bucket = bucket or ""
        self.prefix = prefix.strip("/") or "inventory"
        self.robot = _slug(robot or socket.gethostname())
        self._client = client

    def _s3(self):
        if self._client is None:
            import boto3
            self._client = boto3.client("s3")
        return self._client

    def save(self, mission_id: str, report: dict) -> dict:
        name = f"{_slug(mission_id)}.json"
        body = json.dumps(report, indent=1, sort_keys=True)
        out = {"local": None, "s3": None, "errors": []}
        try:
            self.local_dir.mkdir(parents=True, exist_ok=True)
            _sweep_stale_tmp(self.local_dir)
            path = self.local_dir / name
            # 3.47: write then rename, so a brain that exits mid-save (the
            # save runs on a daemon thread) leaves no half-written file.
            tmp = path.with_name(f".{name}.tmp")
            try:
                with open(tmp, "w") as f:
                    f.write(body)
                    f.flush()
                    # Durable, not only atomic: the Jetson can lose power
                    # seconds after a mission, and a rename to a new name
                    # can then surface as an empty file.
                    _fsync(f.fileno())
                os.replace(tmp, path)
                _fsync_dir(self.local_dir)    # ...and the rename itself
            except BaseException:
                with contextlib.suppress(OSError):   # never hide the real error
                    tmp.unlink(missing_ok=True)
                raise
            out["local"] = str(path)
        except OSError as exc:
            out["errors"].append(f"local: {exc}")
        if self.bucket:
            key = f"{self.prefix}/{self.robot}/{name}"
            try:
                self._s3().put_object(Bucket=self.bucket, Key=key, Body=body.encode(),
                                      ContentType="application/json",
                                      ServerSideEncryption="AES256")
                out["s3"] = f"s3://{self.bucket}/{key}"
            except Exception as exc:  # noqa: BLE001 -- never fail a mission
                out["errors"].append(f"s3: {exc}")
        for err in out["errors"]:
            logger.warning("inventory %s not saved: %s", mission_id, err)
        return out


def inventory_store_from_config(config: dict) -> InventoryStore:
    return InventoryStore(
        local_dir=config.get("inventory_dir", "recordings/inventory"),
        bucket=config.get("inventory_bucket", ""),
        prefix=config.get("inventory_prefix", "inventory"),
    )
