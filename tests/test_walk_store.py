"""Backend-agnostic conformance suite for control/walk_store.py.

Same idea as tests/test_robot_contract.py, one layer down: every test here
runs against BOTH backends, and none of them may know which one it is
driving. That is the whole point -- the EFS-to-S3 migration is only safe
if `LocalWalkStore` and `S3WalkStore` are indistinguishable to
`control/admin_server.py`, and the way to believe that is to assert it
once and run it twice.

The S3 backend is driven by an in-memory double rather than `moto`, for
the reason `control/admin_server.py` already takes an injectable Bedrock
client: the project's automated suite does not reach AWS, and adding a
mocking dependency to pin a dozen calls is a worse trade than writing the
dozen calls. `FakeS3` below implements only what `S3WalkStore` uses, and
it paginates at 2 keys a page precisely so the pagination loop is
exercised by every listing test rather than by one test that remembers to.

The two documented divergences (an empty walk exists locally and not on
S3; appends are read-modify-write on S3) are asserted as such at the
bottom, so that if either backend changes, the test that fails says which
property was being relied on.
"""

import pytest

from control.walk_store import (
    LocalWalkStore,
    S3WalkStore,
    WalkStoreError,
    walk_store_from_config,
)


class FakeS3:
    """An in-memory stand-in for the handful of S3 calls S3WalkStore makes."""

    PAGE = 2  # tiny, so pagination is not a special case anybody has to opt into

    def __init__(self, bucket="test-bucket"):
        self.bucket = bucket
        self.objects: dict[str, bytes] = {}
        self.calls: list[str] = []

    # botocore raises a generated ClientError; what the code under test
    # actually reads is the `response` dict, so that is what we shape.
    def _missing(self, code="NoSuchKey", status=404):
        exc = Exception(code)
        exc.response = {"Error": {"Code": code},
                        "ResponseMetadata": {"HTTPStatusCode": status}}
        return exc

    def head_bucket(self, Bucket):
        self.calls.append("head_bucket")
        if Bucket != self.bucket:
            raise self._missing("NoSuchBucket")
        return {}

    def put_object(self, Bucket, Key, Body, ContentType=None):
        self.calls.append("put_object")
        assert Bucket == self.bucket
        self.objects[Key] = Body
        return {}

    def get_object(self, Bucket, Key):
        self.calls.append("get_object")
        if Key not in self.objects:
            raise self._missing()
        data = self.objects[Key]

        class _Body:
            def read(self_inner):
                return data

        return {"Body": _Body()}

    def head_object(self, Bucket, Key):
        self.calls.append("head_object")
        if Key not in self.objects:
            raise self._missing("NotFound")
        return {"ContentLength": len(self.objects[Key])}

    def delete_object(self, Bucket, Key):
        self.calls.append("delete_object")
        self.objects.pop(Key, None)
        return {}

    def delete_objects(self, Bucket, Delete):
        self.calls.append("delete_objects")
        assert len(Delete["Objects"]) <= 1000, "S3 caps a batch delete at 1000"
        for o in Delete["Objects"]:
            self.objects.pop(o["Key"], None)
        return {}

    def generate_presigned_url(self, op, Params, ExpiresIn):
        self.calls.append("generate_presigned_url")
        return f"https://presigned/{Params['Key']}?exp={ExpiresIn}"

    def list_objects_v2(self, Bucket, Prefix="", Delimiter=None,
                        MaxKeys=None, ContinuationToken=None):
        self.calls.append("list_objects_v2")
        keys = sorted(k for k in self.objects if k.startswith(Prefix))

        contents, common = [], []
        if Delimiter:
            seen = set()
            for k in keys:
                rest = k[len(Prefix):]
                if Delimiter in rest:
                    seen.add(Prefix + rest.split(Delimiter)[0] + Delimiter)
                else:
                    contents.append(k)
            common = sorted(seen)
        else:
            contents = keys

        items = [("p", c) for c in common] + [("c", k) for k in contents]
        start = int(ContinuationToken) if ContinuationToken else 0
        page_size = min(MaxKeys or self.PAGE, self.PAGE)
        window = items[start:start + page_size]
        nxt = start + page_size

        out = {
            "KeyCount": len(window),
            "Contents": [{"Key": k, "Size": len(self.objects[k])}
                         for kind, k in window if kind == "c"],
            "CommonPrefixes": [{"Prefix": p} for kind, p in window if kind == "p"],
            "IsTruncated": nxt < len(items),
        }
        if out["IsTruncated"]:
            out["NextContinuationToken"] = str(nxt)
        return out


@pytest.fixture(params=["local", "s3"])
def store(request, tmp_path):
    """Every test below gets each backend in turn and cannot tell which."""
    if request.param == "local":
        root = tmp_path / "recordings"
        root.mkdir()
        return LocalWalkStore(root)
    return S3WalkStore("test-bucket", "recordings", client=FakeS3())


def seed(store, walk, files):
    store.create_walk(walk)
    for name, data in files.items():
        store.write_bytes(walk, name, data)


# ---------- listing ----------


def test_an_untouched_store_lists_no_walks(store):
    assert store.list_walks() == []


def test_walks_come_back_sorted_and_deduplicated(store):
    for w in ["walk-c", "walk-a", "walk-b"]:
        seed(store, w, {"frame-0000.jpg": b"x"})
    assert store.list_walks() == ["walk-a", "walk-b", "walk-c"]


def test_listing_pages_past_the_backend_page_size(store):
    """FakeS3 pages at 2; a 7-walk store must still list 7. The local
    backend has no pagination at all, which is exactly why this assertion
    belongs in the shared suite rather than in an S3-only test."""
    names = [f"walk-{i:02d}" for i in range(7)]
    for w in names:
        seed(store, w, {"frame-0000.jpg": b"x"})
    assert store.list_walks() == names


def test_list_files_reports_names_and_true_sizes(store):
    seed(store, "w", {"frame-0000.jpg": b"abcd", "walk.jsonl": b"{}\n"})
    assert store.list_files("w") == [("frame-0000.jpg", 4), ("walk.jsonl", 3)]


def test_list_files_on_an_unknown_walk_is_empty_not_an_error(store):
    assert store.list_files("nope") == []


def test_list_names_filters_by_prefix_and_suffix(store):
    seed(store, "w", {"frame-0000.jpg": b"x", "frame-0001.jpg": b"x",
                      "walk.jsonl": b"x", "replay-a.json": b"x"})
    assert store.list_names("w", prefix="frame-") == ["frame-0000.jpg", "frame-0001.jpg"]
    assert store.list_names("w", prefix="replay-", suffix=".json") == ["replay-a.json"]


# ---------- reading and writing ----------


def test_bytes_survive_the_round_trip_unchanged(store):
    blob = bytes(range(256))
    seed(store, "w", {"frame-0000.jpg": blob})
    assert store.read_bytes("w", "frame-0000.jpg") == blob


def test_text_round_trips_as_utf8(store):
    store.create_walk("w")
    store.write_text("w", "meta.json", '{"note": "café ☕"}')
    assert store.read_text("w", "meta.json") == '{"note": "café ☕"}'


def test_reading_an_absent_file_raises_filenotfound_on_every_backend(store):
    seed(store, "w", {"frame-0000.jpg": b"x"})
    with pytest.raises(FileNotFoundError):
        store.read_bytes("w", "meta.json")


def test_writing_twice_replaces_rather_than_appends(store):
    store.create_walk("w")
    store.write_text("w", "meta.json", "first")
    store.write_text("w", "meta.json", "second")
    assert store.read_text("w", "meta.json") == "second"


def test_append_builds_a_jsonl_file_line_by_line(store):
    store.create_walk("w")
    for i in range(3):
        store.append_text("w", "walk.jsonl", '{"seq": %d}\n' % i)
    assert store.read_text("w", "walk.jsonl").splitlines() == [
        '{"seq": 0}', '{"seq": 1}', '{"seq": 2}']


def test_append_creates_the_file_when_it_is_the_first_write(store):
    store.create_walk("w")
    store.append_text("w", "walk.jsonl", "line\n")
    assert store.read_text("w", "walk.jsonl") == "line\n"


def test_write_bytes_creates_the_walk_without_a_prior_create_walk(store):
    """The recording route creates then writes, but nothing should depend
    on that order -- on S3 create_walk() is a no-op, so if the write did
    not imply the walk, every S3 recording would land nowhere."""
    store.write_bytes("fresh", "frame-0000.jpg", b"x")
    assert store.walk_exists("fresh")
    assert store.list_walks() == ["fresh"]


# ---------- existence ----------


def test_file_exists_tracks_writes_and_deletes(store):
    seed(store, "w", {"frame-0000.jpg": b"x"})
    assert store.file_exists("w", "frame-0000.jpg")
    assert not store.file_exists("w", "frame-0001.jpg")
    store.delete_file("w", "frame-0000.jpg")
    assert not store.file_exists("w", "frame-0000.jpg")


def test_a_walk_with_files_exists_and_an_unknown_one_does_not(store):
    seed(store, "w", {"frame-0000.jpg": b"x"})
    assert store.walk_exists("w")
    assert not store.walk_exists("other")


def test_file_size_matches_the_bytes_written(store):
    seed(store, "w", {"frame-0000.jpg": b"12345"})
    assert store.file_size("w", "frame-0000.jpg") == 5
    with pytest.raises(FileNotFoundError):
        store.file_size("w", "absent.json")


# ---------- json helpers ----------


def test_read_json_returns_none_for_absent_and_for_malformed(store):
    """Both spellings of "no usable sidecar" collapse to None, because the
    console must render a walk whose eval.json was half-written."""
    store.create_walk("w")
    assert store.read_json("w", "eval.json") is None
    store.write_text("w", "eval.json", "{not json")
    assert store.read_json("w", "eval.json") is None


def test_write_json_round_trips_through_read_json(store):
    store.create_walk("w")
    store.write_json("w", "eval.json", {"score": 42, "flags": ["stalled"]})
    assert store.read_json("w", "eval.json") == {"score": 42, "flags": ["stalled"]}


# ---------- deleting ----------


def test_deleting_an_absent_file_raises_filenotfound(store):
    store.create_walk("w")
    with pytest.raises(FileNotFoundError):
        store.delete_file("w", "frame-0000.jpg")


def test_deleting_a_walk_removes_every_file_in_it(store):
    seed(store, "w", {"frame-0000.jpg": b"x", "walk.jsonl": b"y", "meta.json": b"z"})
    seed(store, "keeper", {"frame-0000.jpg": b"x"})
    store.delete_walk("w")
    assert store.list_walks() == ["keeper"]
    assert store.list_files("w") == []


def test_deleting_a_walk_leaves_a_similarly_named_one_alone(store):
    """`walk` and `walk-2` share a key prefix on S3; a delete that used
    the bare prefix rather than a trailing separator would take both."""
    seed(store, "walk", {"frame-0000.jpg": b"x"})
    seed(store, "walk-2", {"frame-0000.jpg": b"x"})
    store.delete_walk("walk")
    assert store.list_walks() == ["walk-2"]


# ---------- name safety ----------


@pytest.mark.parametrize("bad", [
    "../escape", "a/b", "..", ".hidden", "", "with space", "x" * 129,
])
def test_unsafe_walk_names_are_refused_by_both_backends(store, bad):
    with pytest.raises(WalkStoreError):
        store.walk_exists(bad)


@pytest.mark.parametrize("bad", ["../../etc/passwd", "sub/dir.json", ".."])
def test_unsafe_file_names_are_refused_by_both_backends(store, bad):
    store.create_walk("w")
    with pytest.raises(WalkStoreError):
        store.read_bytes("w", bad)


def test_availability_is_reported(store):
    assert store.available() is True


def test_describe_names_the_backing_location(store):
    assert store.describe().startswith(("local:", "s3://"))


def test_location_ends_with_the_walk_name_and_carries_no_scheme_prefix_locally(store):
    """POST /recording/frame returns this as `dir`, and sim/replay_robot.py
    opens it. A store-level `describe()` string was returned here once and
    ReplayRobot could not open "local:/tmp/..." -- so the local backend must
    hand back a path, not a label."""
    seed(store, "w", {"frame-0000.jpg": b"x"})
    loc = store.location("w")
    assert loc.endswith("/w")
    if isinstance(store, LocalWalkStore):
        assert not loc.startswith("local:")
        from pathlib import Path
        assert Path(loc).is_dir()
    else:
        assert loc.startswith("s3://")


# ---------- the two documented divergences ----------


def test_local_offers_no_download_url_and_s3_does(tmp_path):
    """Decided per backend, never per size. A route that streamed small
    walks and redirected large ones would pass every test and fail only on
    the two walks in the corpus that are over the Lambda response cap."""
    assert LocalWalkStore(tmp_path).download_url("w.zip", b"x", "application/zip") is None

    fake = FakeS3()
    s3 = S3WalkStore("test-bucket", "recordings", client=fake)
    url = s3.download_url("w.zip", b"payload", "application/zip")
    assert url and url.startswith("https://presigned/")
    assert "put_object" in fake.calls


def test_an_export_is_keyed_outside_the_walk_prefix(tmp_path):
    """An export under `recordings/` would be listed by list_walks() as a
    walk named `exports`, because that reads S3 common prefixes and cannot
    tell one of them is not a walk."""
    fake = FakeS3()
    s3 = S3WalkStore("test-bucket", "recordings", client=fake)
    seed(s3, "real-walk", {"frame-0000.jpg": b"x"})
    s3.download_url("real-walk.zip", b"zipbytes", "application/zip")

    assert s3.list_walks() == ["real-walk"]
    assert any(k.startswith("exports/") for k in fake.objects)
    assert not any(k.startswith("recordings/exports") for k in fake.objects)


def test_an_empty_walk_exists_locally_and_not_on_s3(tmp_path):
    """Asserted rather than hidden: create_walk() alone is not enough to
    make walk_exists() true on S3, because S3 has no directories. Every
    real walk writes a frame immediately, which is why this never bites."""
    local = LocalWalkStore(tmp_path)
    local.create_walk("empty")
    assert local.walk_exists("empty")

    s3 = S3WalkStore("test-bucket", "recordings", client=FakeS3())
    s3.create_walk("empty")
    assert not s3.walk_exists("empty")


def test_s3_append_is_read_modify_write_and_local_is_not(tmp_path):
    """Pins the concurrency caveat in the module docstring. If S3 ever
    grows a real append this test should fail and the caveat be deleted."""
    fake = FakeS3()
    s3 = S3WalkStore("test-bucket", "recordings", client=fake)
    s3.write_text("w", "walk.jsonl", "one\n")
    fake.calls.clear()
    s3.append_text("w", "walk.jsonl", "two\n")
    assert "get_object" in fake.calls and "put_object" in fake.calls


# ---------- the factory ----------


def test_factory_defaults_to_the_local_directory(tmp_path):
    store = walk_store_from_config({"recording_dir": str(tmp_path)})
    assert isinstance(store, LocalWalkStore)


def test_factory_builds_an_s3_store_from_bucket_and_prefix():
    store = walk_store_from_config({
        "recording_backend": "s3", "recording_bucket": "b", "recording_prefix": "p",
    })
    assert isinstance(store, S3WalkStore)
    assert store.describe() == "s3://b/p"


def test_factory_refuses_s3_without_a_bucket():
    with pytest.raises(WalkStoreError):
        walk_store_from_config({"recording_backend": "s3", "recording_bucket": ""})


def test_factory_refuses_an_unknown_backend():
    with pytest.raises(WalkStoreError):
        walk_store_from_config({"recording_backend": "gcs", "recording_dir": "x"})
