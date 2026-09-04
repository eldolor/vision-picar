"""Where recorded walks live -- one abstraction, two backends.

This is the same shape as `robot/interface.py`'s `RobotInterface`, and it
exists for the same reason: the thing above it (`control/admin_server.py`,
the recording routes in `control/brain_server.py`) should not know whether
a walk is a directory on a mounted filesystem or a set of keys in a
bucket. `walk_store_from_config()` is this module's `robot/factory.py` --
the only place that picks.

Why it was written (2026-09-04): the walks used to live on an EFS volume,
and EFS is the only component in the deployment that genuinely *requires*
a VPC. A VPC forces private subnets, which force either interface
endpoints (~$72/month here) or a NAT gateway, and an internal ALB to
reach anything inside it. Moving 59MB of write-once blobs to S3 removes
that requirement, and with it the single largest line on the bill. The
storage change is small; what it unlocks is not.

## The layout is identical on both backends

    <root>/<walk>/frame-0000.jpg     the frames
    <root>/<walk>/walk.jsonl         one JSON object per frame, in seq order
    <root>/<walk>/meta.json          written by POST /recording/finish
    <root>/<walk>/tags.json          the operator's label
    <root>/<walk>/eval.json          the scorer's opinion, never the operator's
    <root>/<walk>/replay-*.json      one sidecar per (model, wording) replay

`<root>` is a directory for `LocalWalkStore` and a key prefix for
`S3WalkStore`. Nothing else differs, deliberately: `control/walk_eval.py`,
`control/walk_replay.py` and the download-zip route all read this layout,
and the recorded corpus is the project's only dataset. A migration that
also reshaped the format would have made every previously recorded walk
unreadable by the tooling that scores it.

## Two behaviours worth knowing before you use it

**Appends are read-modify-write on S3.** S3 objects are immutable, so
`append_text()` on that backend gets the object, concatenates, and puts it
back. That is safe here because a walk has exactly one writer -- one phone,
posting frames in sequence -- and unsafe the moment that stops being true.
It is not a general-purpose append. `LocalWalkStore` uses a real `O_APPEND`
open, so the two backends differ in concurrency behaviour even though they
agree on results; the conformance suite pins the results, not the timing.

**An empty walk exists locally and does not exist on S3.** `create_walk()`
makes a directory on one backend and is a no-op on the other, because S3
has no directories -- a prefix exists only while some object carries it.
Every real walk gets a frame written immediately after creation, so this
is invisible in practice, but it is why `create_walk()` is not enough on
its own to make `walk_exists()` true everywhere.

## Names are validated here, not only by callers

The routes already match `walk` against a regex before calling in. This
module checks again, because the check it replaces was a filesystem
accident rather than a rule: the old code compared `walk_dir.parent` to
the resolved root, which caught traversal only because `Path.resolve()`
collapsed `..` first. A key prefix has no such collapsing, so `..` in an
S3 key is just a character -- the guard has to be explicit, and it has to
live where both backends share it.
"""

from __future__ import annotations

import json
import os
import re
import shutil
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

# A walk or file name may not contain a path separator, may not be a
# relative-path element, and may not start with a dot. Deliberately
# stricter than either backend requires: the same string has to be a safe
# directory name AND a safe key segment, so this is the intersection.
SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class WalkStoreError(Exception):
    """A name the store refuses, or a backend that could not be reached."""


def _check(name: str, what: str) -> str:
    if not isinstance(name, str) or not SAFE_NAME.match(name) or ".." in name:
        raise WalkStoreError(f"Unsafe {what}: {name!r}")
    return name


class WalkStore(ABC):
    """The operations the recording routes actually perform.

    Kept deliberately small and file-shaped rather than walk-shaped: the
    callers already know the layout above, and a richer interface here
    would just move `admin_server`'s logic somewhere it is harder to test.
    """

    # -- reading ------------------------------------------------------

    @abstractmethod
    def list_walks(self) -> list[str]:
        """Walk names, sorted. Never raises for an absent root -- an empty
        store and a missing one are the same thing to every caller."""

    @abstractmethod
    def list_files(self, walk: str) -> list[tuple[str, int]]:
        """(filename, size_in_bytes) for one walk, sorted by name."""

    @abstractmethod
    def read_bytes(self, walk: str, name: str) -> bytes:
        """Raises FileNotFoundError if absent, like Path.read_bytes."""

    @abstractmethod
    def file_exists(self, walk: str, name: str) -> bool:
        ...

    @abstractmethod
    def walk_exists(self, walk: str) -> bool:
        ...

    @abstractmethod
    def available(self) -> bool:
        """Whether the backing store can be reached at all. Feeds /health
        and /stats, which reported `recording_dir_exists` before this
        existed."""

    # -- writing ------------------------------------------------------

    @abstractmethod
    def write_bytes(self, walk: str, name: str, data: bytes) -> None:
        ...

    @abstractmethod
    def append_text(self, walk: str, name: str, text: str) -> None:
        """Append to a text file, creating it if absent. Single-writer
        only on S3 -- see the module docstring."""

    @abstractmethod
    def create_walk(self, walk: str) -> None:
        ...

    @abstractmethod
    def delete_file(self, walk: str, name: str) -> None:
        ...

    @abstractmethod
    def delete_walk(self, walk: str) -> None:
        ...

    @abstractmethod
    def location(self, walk: str) -> str:
        """Where one walk's files are, as a string a consumer can act on.

        A real filesystem path on `LocalWalkStore` -- POST /recording/frame
        returns this as `dir`, and `sim/replay_robot.py` opens it -- and an
        `s3://` URI on `S3WalkStore`, where no local path exists and the
        caller has to be told so rather than handed something that looks
        openable and is not.
        """

    # -- conveniences shared by both backends -------------------------
    # Defined here rather than abstractly so the two backends cannot
    # drift on encoding, and so JSON handling has exactly one spelling.

    def read_text(self, walk: str, name: str) -> str:
        return self.read_bytes(walk, name).decode("utf-8")

    def write_text(self, walk: str, name: str, text: str) -> None:
        self.write_bytes(walk, name, text.encode("utf-8"))

    def read_json(self, walk: str, name: str) -> Optional[dict]:
        """None for absent OR malformed, matching what `_read_json_sidecar`
        did before this module existed: a half-written sidecar must not be
        able to take the console down."""
        try:
            return json.loads(self.read_text(walk, name))
        except (FileNotFoundError, json.JSONDecodeError, UnicodeDecodeError, OSError):
            return None

    def write_json(self, walk: str, name: str, obj) -> None:
        self.write_text(walk, name, json.dumps(obj, indent=1))

    def file_size(self, walk: str, name: str) -> int:
        for fname, size in self.list_files(walk):
            if fname == name:
                return size
        raise FileNotFoundError(f"{walk}/{name}")

    def list_names(self, walk: str, prefix: str = "", suffix: str = "") -> list[str]:
        """Sorted names filtered by prefix/suffix -- replaces the
        `startswith("frame-")` scans and the `glob("replay-*.json")` calls
        at the call sites, so neither backend needs a glob implementation."""
        return [n for n, _ in self.list_files(walk)
                if n.startswith(prefix) and n.endswith(suffix)]


class LocalWalkStore(WalkStore):
    """A directory on a filesystem -- what EFS looked like, and what a
    developer running the servers locally still gets."""

    def __init__(self, root):
        self.root = Path(root).resolve()

    def _dir(self, walk: str) -> Path:
        return self.root / _check(walk, "walk name")

    def _path(self, walk: str, name: str) -> Path:
        return self._dir(walk) / _check(name, "file name")

    def list_walks(self) -> list[str]:
        if not self.root.is_dir():
            return []
        return sorted(p.name for p in self.root.iterdir() if p.is_dir())

    def list_files(self, walk: str) -> list[tuple[str, int]]:
        d = self._dir(walk)
        if not d.is_dir():
            return []
        return sorted((p.name, p.stat().st_size) for p in d.iterdir() if p.is_file())

    def read_bytes(self, walk: str, name: str) -> bytes:
        return self._path(walk, name).read_bytes()

    def file_exists(self, walk: str, name: str) -> bool:
        return self._path(walk, name).is_file()

    def walk_exists(self, walk: str) -> bool:
        return self._dir(walk).is_dir()

    def available(self) -> bool:
        return self.root.is_dir()

    def write_bytes(self, walk: str, name: str, data: bytes) -> None:
        p = self._path(walk, name)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)

    def append_text(self, walk: str, name: str, text: str) -> None:
        p = self._path(walk, name)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("a") as f:
            f.write(text)

    def create_walk(self, walk: str) -> None:
        self._dir(walk).mkdir(parents=True, exist_ok=True)

    def delete_file(self, walk: str, name: str) -> None:
        self._path(walk, name).unlink()

    def delete_walk(self, walk: str) -> None:
        shutil.rmtree(self._dir(walk))

    def location(self, walk: str) -> str:
        return str(self._dir(walk))

    def describe(self) -> str:
        return f"local:{self.root}"


class S3WalkStore(WalkStore):
    """Walks as objects under one key prefix in one bucket.

    The client is injectable so the tests can drive an in-memory double
    rather than take a `moto` dependency -- the same reason
    `control/admin_server.py` takes a Bedrock client factory instead of
    constructing one inline.
    """

    def __init__(self, bucket: str, prefix: str = "recordings", client=None):
        if not bucket:
            raise WalkStoreError("S3WalkStore needs a bucket name.")
        self.bucket = bucket
        # Normalised to exactly one trailing slash internally, and stored
        # without one, so key building never doubles or drops a separator.
        self.prefix = prefix.strip("/")
        self._client = client

    @property
    def client(self):
        if self._client is None:  # pragma: no cover - needs real creds
            import boto3

            self._client = boto3.client("s3")
        return self._client

    def _walk_prefix(self, walk: str) -> str:
        return f"{self.prefix}/{_check(walk, 'walk name')}/" if self.prefix \
            else f"{_check(walk, 'walk name')}/"

    def _key(self, walk: str, name: str) -> str:
        return self._walk_prefix(walk) + _check(name, "file name")

    def _pages(self, **kwargs):
        token = None
        while True:
            if token:
                kwargs["ContinuationToken"] = token
            page = self.client.list_objects_v2(Bucket=self.bucket, **kwargs)
            yield page
            if not page.get("IsTruncated"):
                return
            token = page.get("NextContinuationToken")
            if not token:
                return

    def list_walks(self) -> list[str]:
        root = f"{self.prefix}/" if self.prefix else ""
        names = set()
        for page in self._pages(Prefix=root, Delimiter="/"):
            for cp in page.get("CommonPrefixes", []):
                name = cp["Prefix"][len(root):].strip("/")
                if name:
                    names.add(name)
        return sorted(names)

    def list_files(self, walk: str) -> list[tuple[str, int]]:
        wp = self._walk_prefix(walk)
        out = []
        for page in self._pages(Prefix=wp):
            for obj in page.get("Contents", []):
                name = obj["Key"][len(wp):]
                # Skip anything nested deeper than the walk: the layout is
                # flat, and a stray key must not be reported as a file.
                if name and "/" not in name:
                    out.append((name, obj["Size"]))
        return sorted(out)

    def read_bytes(self, walk: str, name: str) -> bytes:
        try:
            resp = self.client.get_object(Bucket=self.bucket, Key=self._key(walk, name))
        except Exception as exc:
            if _is_not_found(exc):
                raise FileNotFoundError(f"{walk}/{name}") from exc
            raise
        return resp["Body"].read()

    def file_exists(self, walk: str, name: str) -> bool:
        try:
            self.client.head_object(Bucket=self.bucket, Key=self._key(walk, name))
            return True
        except Exception as exc:
            if _is_not_found(exc):
                return False
            raise

    def walk_exists(self, walk: str) -> bool:
        for page in self._pages(Prefix=self._walk_prefix(walk), MaxKeys=1):
            if page.get("KeyCount") or page.get("Contents"):
                return True
        return False

    def available(self) -> bool:
        try:
            self.client.head_bucket(Bucket=self.bucket)
            return True
        except Exception:
            return False

    def write_bytes(self, walk: str, name: str, data: bytes) -> None:
        self.client.put_object(Bucket=self.bucket, Key=self._key(walk, name), Body=data)

    def append_text(self, walk: str, name: str, text: str) -> None:
        try:
            current = self.read_text(walk, name)
        except FileNotFoundError:
            current = ""
        self.write_text(walk, name, current + text)

    def create_walk(self, walk: str) -> None:
        # Intentionally nothing: S3 has no directories, and writing a
        # zero-byte placeholder would show up as a file in list_files()
        # and land in the download zip. See the module docstring.
        _check(walk, "walk name")

    def delete_file(self, walk: str, name: str) -> None:
        key = self._key(walk, name)
        if not self.file_exists(walk, name):
            raise FileNotFoundError(f"{walk}/{name}")
        self.client.delete_object(Bucket=self.bucket, Key=key)

    def delete_walk(self, walk: str) -> None:
        wp = self._walk_prefix(walk)
        keys = []
        for page in self._pages(Prefix=wp):
            keys.extend({"Key": o["Key"]} for o in page.get("Contents", []))
        for i in range(0, len(keys), 1000):  # delete_objects caps at 1000
            self.client.delete_objects(Bucket=self.bucket,
                                       Delete={"Objects": keys[i:i + 1000]})

    def location(self, walk: str) -> str:
        return f"s3://{self.bucket}/{self._walk_prefix(walk).rstrip('/')}"

    def describe(self) -> str:
        return f"s3://{self.bucket}/{self.prefix}"


def _is_not_found(exc: Exception) -> bool:
    """True for S3's several spellings of 'no such object'.

    botocore raises ClientError with NoSuchKey for get_object and a bare
    404 for head_object, and the exception class is generated at runtime,
    so this reads the response rather than catching a named type.
    """
    resp = getattr(exc, "response", None)
    if not isinstance(resp, dict):
        return False
    code = str(resp.get("Error", {}).get("Code", ""))
    status = resp.get("ResponseMetadata", {}).get("HTTPStatusCode")
    return code in ("NoSuchKey", "NotFound", "404") or status == 404


def walk_store_from_config(config: dict) -> WalkStore:
    """Pick a backend from the `brain:` config block.

    The one place that chooses, deliberately -- `robot/factory.py`'s role
    for this half of the system. `recording_backend: s3` needs
    `recording_bucket`; anything else is the local directory that has
    always been there, so an unconfigured checkout keeps working.
    """
    backend = str(config.get("recording_backend", "local")).lower()
    if backend == "s3":
        return S3WalkStore(
            bucket=config.get("recording_bucket", ""),
            prefix=config.get("recording_prefix", "recordings"),
        )
    if backend != "local":
        raise WalkStoreError(
            f"Unknown recording_backend {backend!r} -- expected 'local' or 's3'."
        )
    return LocalWalkStore(config["recording_dir"])
