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

import json
import logging
import os
import re
import socket
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_SAFE = re.compile(r"[^a-z0-9._-]+")


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
                    os.fsync(f.fileno())
                os.replace(tmp, path)
            except BaseException:
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
