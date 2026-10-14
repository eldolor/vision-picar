"""3.76 batch H2: the cloud vision service's confirmed review bugs, each red
on `dev` before its fix. No AWS call anywhere: Bedrock is a fake client.

Run with: pytest service/vision_analyze/tests/ -q
"""

import base64
import importlib
import io

import pytest
from fastapi.testclient import TestClient
from PIL import Image

import app as app_module
import vision_core
from rooms_core import identify_room

IMAGE_B64 = base64.b64encode(b"not-a-real-jpeg-but-bytes-are-bytes").decode()


@pytest.fixture
def client():
    return TestClient(app_module.app)


class FakeBedrock:
    def __init__(self, text):
        self.text = text
        self.calls = []

    def converse(self, **kwargs):
        self.calls.append(kwargs)
        return {"output": {"message": {"content": [{"text": self.text}]}},
                "usage": {"inputTokens": 1, "outputTokens": 1}}


def _fake(monkeypatch, text):
    fake = FakeBedrock(text)
    monkeypatch.setattr(vision_core, "_get_client", lambda *a, **kw: fake)
    return fake


def _png(size=(4, 4)) -> bytes:
    buf = io.BytesIO()
    Image.new("RGB", size, (200, 10, 10)).save(buf, format="PNG")
    return buf.getvalue()


# ---------- H2-1: valid JSON that is not an object ----------


@pytest.mark.parametrize("reply", ["[1, 2]", "null", "42", '"STOP"', "true"])
@pytest.mark.parametrize("parse", ["_parse_navigate_json", "_parse_guidance_json",
                                   "_parse_describe_json", "_parse_scene_json"])
def test_a_reply_that_is_json_but_not_an_object_is_a_parse_failure(parse, reply):
    """Was a TypeError in {**SCHEMA, **parsed}: app.py answered 502, and the
    brain reads any 5xx as the cloud being unreachable (an arrival ended
    `arrived_unconfirmed`). The model's own bad answer is the empty schema,
    like any other unparseable reply."""
    out = getattr(vision_core, parse)(reply)
    assert out["_raw"] == reply


def test_navigate_route_answers_200_not_502_on_a_list_reply(client, monkeypatch):
    _fake(monkeypatch, "[]")
    resp = client.post("/navigate", json={"image_base64": IMAGE_B64, "target_object": "bag"})
    assert resp.status_code == 200
    assert resp.json()["target_visible"] is False


# ---------- H2-2: a fenced reply followed by prose ----------


def test_a_fenced_reply_with_trailing_prose_still_parses():
    """`strip('`')` only strips backticks at the ends, so a closing fence
    followed by a sentence lost a real answer -- at arrival, a target the
    model saw read as 'not the target'."""
    text = ('```json\n{"target_visible": true, "target_direction": "center", '
            '"action": "FORWARD"}\n```\nThe bag is straight ahead.')
    out = vision_core._parse_navigate_json(text)
    assert "_raw" not in out
    assert out["target_visible"] is True


def test_a_plain_fenced_reply_still_parses():
    out = vision_core._parse_guidance_json('```json\n{"target_visible": true}\n```')
    assert out["target_visible"] is True and "_raw" not in out


# ---------- H2-3 / H2-4: image errors are the caller's, not an outage ----------


def test_an_undecodable_non_native_image_is_a_400_not_a_502(client, monkeypatch):
    """A HEIC-labelled body Pillow cannot read failed inside the vision call
    and came back 502 -- which the brain reads as the cloud being down."""
    fake = _fake(monkeypatch, "{}")
    resp = client.post("/navigate", json={
        "image_base64": IMAGE_B64, "media_type": "image/heic", "target_object": "bag"})
    assert resp.status_code == 400
    assert fake.calls == []


def test_the_size_limit_is_bedrocks_not_claudes(client, monkeypatch):
    """botocore's bedrock-runtime model (ImageSource): 'Each image's size ...
    must be no more than 3.75 MB'. A 4 MB image passed the old 5 MB check and
    came back 502 from Bedrock's ValidationException."""
    fake = _fake(monkeypatch, "{}")
    big = base64.b64encode(b"\0" * 4_000_000).decode()
    resp = client.post("/navigate", json={"image_base64": big, "target_object": "bag"})
    assert resp.status_code == 413
    assert fake.calls == []


def test_a_converted_image_is_size_checked_too(client, monkeypatch):
    """A HEIC converted to JPEG was never measured again."""
    fake = _fake(monkeypatch, "{}")
    monkeypatch.setattr(vision_core, "_convert_to_jpeg", lambda b: b"\0" * 4_000_000)
    resp = client.post("/navigate", json={
        "image_base64": IMAGE_B64, "media_type": "image/heic", "target_object": "bag"})
    assert resp.status_code == 413
    assert fake.calls == []


def test_a_readable_heic_style_image_still_converts(monkeypatch):
    fake = _fake(monkeypatch, '{"target_visible": false}')
    vision_core.describe_image_bytes_navigate(_png(), "bag", "image/bmp")
    assert fake.calls[0]["messages"][0]["content"][0]["image"]["format"] == "jpeg"


# ---------- H2-5: unvalidated model output and request bodies ----------


@pytest.mark.parametrize("objects", [[{"name": "sofa"}], [1, None], "sofa", ""])
def test_analyze_survives_any_important_objects(client, monkeypatch, objects):
    """Non-string items were an AttributeError (500), outside the try."""
    monkeypatch.setattr(app_module, "describe_image_bytes",
                        lambda b, mt: {"important_objects": objects})
    resp = client.post("/analyze", json={"image_base64": IMAGE_B64})
    assert resp.status_code == 200


def test_an_empty_object_name_does_not_match_every_room():
    """'' is a substring of every feature, so it scored every room and the
    first (living room) always won."""
    assert identify_room([""]) == "unknown"
    assert identify_room(["  "]) == "unknown"


def test_a_string_is_not_iterated_character_by_character():
    assert identify_room("bed") == "unknown"


def test_identify_room_still_matches_real_objects():
    assert identify_room(["Double bed", 3, "nightstand"]) == "bedroom"


@pytest.mark.parametrize("body", [[1, 2], "x", 3])
def test_a_json_body_that_is_not_an_object_is_a_400(client, body):
    resp = client.post("/analyze", json=body)
    assert resp.status_code == 400


def test_a_non_string_media_type_is_a_400(client):
    resp = client.post("/analyze", json={"image_base64": IMAGE_B64, "media_type": 5})
    assert resp.status_code == 400


# ---------- H2-6: the configured defaults are what is reported ----------


def _reload(monkeypatch, **env):
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    return importlib.reload(vision_core)


@pytest.fixture
def restore_vision_core(monkeypatch):
    yield
    monkeypatch.undo()
    importlib.reload(vision_core)


def test_an_unknown_default_prompt_variant_is_not_reported_as_run(monkeypatch, restore_vision_core):
    """NAVIGATE_PROMPT_VARIANT=typo ran the default wording but stamped
    'typo' on every reply -- a recorded walk attributed to a prompt that
    does not exist (the NavigateModelId lesson)."""
    vc = _reload(monkeypatch, NAVIGATE_PROMPT_VARIANT="no-such-variant")
    assert vc.DEFAULT_PROMPT_VARIANT in vc.NAVIGATE_PROMPT_VARIANTS


def test_the_default_model_is_always_in_the_picker(monkeypatch, restore_vision_core):
    vc = _reload(monkeypatch, BEDROCK_NAVIGATE_MODEL_CHOICES="amazon.nova-lite-v1:0",
                 BEDROCK_NAVIGATE_MODEL_ID="us.anthropic.claude-opus-4-5-20251101-v1:0")
    assert vc.NAVIGATE_MODEL_ID in vc.NAVIGATE_MODEL_CHOICES


# ---------- H2-8: caller-supplied text that goes into the prompt ----------


def test_an_overlong_target_object_is_a_400(client, monkeypatch):
    fake = _fake(monkeypatch, "{}")
    resp = client.post("/navigate", json={"image_base64": IMAGE_B64, "target_object": "x" * 10_000})
    assert resp.status_code == 400
    assert fake.calls == []


def test_a_non_string_target_object_is_a_400(client, monkeypatch):
    _fake(monkeypatch, "{}")
    resp = client.post("/navigate", json={"image_base64": IMAGE_B64, "target_object": {"a": 1}})
    assert resp.status_code == 400


def test_searched_rooms_are_bounded(client, monkeypatch):
    """Round 2 (S10): the brain sends the rooms sorted, so no slice keeps "the
    most recent"; at most 20 go, and a long name is cut, not dropped."""
    seen = {}

    def fake_nav(image_bytes, target, media_type, searched_rooms=None, **kw):
        seen["rooms"] = searched_rooms
        return {"action": "STOP"}

    monkeypatch.setattr(app_module, "describe_image_bytes_navigate", fake_nav)
    rooms = ["y" * 5000] + [f"room {i}" for i in range(500)]
    resp = client.post("/navigate", json={
        "image_base64": IMAGE_B64, "target_object": "bag", "searched_rooms": rooms})
    assert resp.status_code == 200
    assert len(seen["rooms"]) == app_module.MAX_SEARCHED_ROOMS
    assert seen["rooms"][0] == "y" * app_module.MAX_ROOM_CHARS


# ---------- H2-11: a hung Bedrock call ends before Lambda kills it ----------


def test_bedrock_read_timeout_is_inside_the_lambda_timeout(monkeypatch):
    """botocore's default read timeout is 60 s; the function is killed at
    30 s (serverless.yaml), so a hang was never raised, logged or answered."""
    monkeypatch.setattr(vision_core, "_clients", {})
    built = {}
    monkeypatch.setattr(vision_core.boto3, "client",
                        lambda svc, region_name=None, config=None: built.setdefault("config", config))
    vision_core._get_client("amazon.nova-lite-v1:0")
    cfg = built["config"]
    assert cfg is not None
    assert cfg.connect_timeout + cfg.read_timeout < 30


# ---------- round 2 ----------


def test_a_read_timeout_is_not_retried(monkeypatch):
    """S1: botocore's default retried a read timeout (5 attempts), running a
    hung call past Lambda's 30 s and paying for each retry. Proven against a
    local socket that accepts and never answers -- no AWS."""
    import socket
    import threading

    import boto3
    from botocore.config import Config

    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(16)
    conns = []

    def accept():
        while True:
            try:
                c, _ = srv.accept()
            except OSError:
                return
            conns.append(c)

    threading.Thread(target=accept, daemon=True).start()
    cfg = vision_core._BEDROCK_CONFIG.merge(Config(connect_timeout=0.5, read_timeout=0.2))
    bedrock = boto3.client("bedrock-runtime", region_name="us-east-2",
                           endpoint_url=f"http://127.0.0.1:{srv.getsockname()[1]}",
                           aws_access_key_id="x", aws_secret_access_key="y", config=cfg)
    try:
        with pytest.raises(Exception):
            bedrock.converse(modelId="m", messages=[{"role": "user", "content": [{"text": "hi"}]}])
    finally:
        srv.close()
    assert len(conns) == 1
    c = vision_core._BEDROCK_CONFIG
    assert c.retries["total_max_attempts"] * (c.connect_timeout + c.read_timeout) < 30


@pytest.mark.parametrize("field,value", [("model_id", ["a"]), ("model_id", {"a": 1}),
                                         ("prompt_variant", ["default"]), ("prompt_variant", {})])
def test_an_unhashable_model_id_or_variant_is_a_400(client, monkeypatch, field, value):
    """S2: `[...] in dict` is a TypeError -- a 500 the brain reads as an outage."""
    fake = _fake(monkeypatch, "{}")
    resp = client.post("/navigate", json={"image_base64": IMAGE_B64, "target_object": "bag",
                                          field: value})
    assert resp.status_code == 400
    assert fake.calls == []


def _raising(monkeypatch, code):
    from botocore.exceptions import ClientError

    class Refusing:
        def converse(self, **kw):
            raise ClientError({"Error": {"Code": code, "Message": "no"}}, "Converse")

    monkeypatch.setattr(vision_core, "_get_client", lambda *a, **kw: Refusing())


@pytest.mark.parametrize("route", ["/navigate", "/guidance", "/analyze", "/describe"])
@pytest.mark.parametrize("code", ["ValidationException", "AccessDeniedException"])
def test_bedrock_refusals_are_502(client, monkeypatch, route, code):
    """Round 3 (P1) reverts round 2's S3: a deployed-wrong model id or a
    model not enabled in the region is ALSO a ValidationException, and as a
    400 an arrived mission ended `failed` without 3.47's late re-ask. Every
    Bedrock refusal is 502 (known limit: a malformed frame reads as an
    outage too)."""
    _raising(monkeypatch, code)
    resp = client.post(route, json={"image_base64": IMAGE_B64, "target_object": "bag"})
    assert resp.status_code == 502


def test_a_big_converted_image_is_shrunk_not_refused(monkeypatch):
    """S4: a 48 MP iPhone HEIC re-encodes past 3.75 MB and was refused (413)."""
    import os
    monkeypatch.setattr(vision_core, "MAX_IMAGE_BYTES", 60_000)
    noise = Image.frombytes("RGB", (600, 600), os.urandom(600 * 600 * 3))
    buf = io.BytesIO()
    noise.save(buf, format="PNG")
    fake = _fake(monkeypatch, "{}")
    vision_core.describe_image_bytes_navigate(buf.getvalue(), "bag", "image/bmp")
    sent = fake.calls[0]["messages"][0]["content"][0]["image"]["source"]["bytes"]
    assert len(sent) <= 60_000


def test_a_converted_image_is_kept_within_8000_px(monkeypatch):
    fake = _fake(monkeypatch, "{}")
    vision_core.describe_image_bytes_navigate(_png((9000, 10)), "bag", "image/bmp")
    sent = fake.calls[0]["messages"][0]["content"][0]["image"]["source"]["bytes"]
    assert max(Image.open(io.BytesIO(sent)).size) <= 8000


@pytest.mark.parametrize("text", [
    '{"target_visible": true} The bag is ahead.',
    'Here it is: {"target_visible": true}',
    '```json\n{"target_visible": true, "reasoning": "a ``` mark"}\n```',
])
def test_json_with_prose_around_it_still_parses(text):
    """S7: plain JSON followed by prose read as not visible; a ``` inside a
    JSON string cut the fenced span short."""
    out = vision_core._parse_navigate_json(text)
    assert "_raw" not in out
    assert out["target_visible"] is True


def test_navigate_refuses_an_unknown_variant_itself(monkeypatch):
    """S8: the function ran the default wording under any unknown name and
    stamped that name on the reply."""
    fake = _fake(monkeypatch, "{}")
    with pytest.raises(ValueError):
        vision_core.describe_image_bytes_navigate(b"x", "bag", prompt_variant="nope")
    assert fake.calls == []


def test_one_image_limit(client, monkeypatch):
    """S9: app.py had its own MAX_IMAGE_BYTES; lowering vision_core's did not
    move the route's check."""
    monkeypatch.setattr(vision_core, "MAX_IMAGE_BYTES", 4)
    _fake(monkeypatch, "{}")
    resp = client.post("/analyze", json={"image_base64": IMAGE_B64})
    assert resp.status_code == 413
    assert not hasattr(app_module, "MAX_IMAGE_BYTES")


@pytest.mark.parametrize("route", ["/navigate", "/guidance"])
def test_a_blank_target_is_a_400(client, monkeypatch, route):
    """S10: a whitespace target bought a paid call on a blank noun."""
    fake = _fake(monkeypatch, "{}")
    resp = client.post(route, json={"image_base64": IMAGE_B64, "target_object": "   "})
    assert resp.status_code == 400
    assert fake.calls == []


# ---------- round 3 ----------


class Throttled:
    """Raises the given codes in turn, then answers."""

    def __init__(self, *codes):
        self.codes = list(codes)
        self.calls = 0

    def converse(self, **kw):
        from botocore.exceptions import ClientError
        self.calls += 1
        if self.codes:
            raise ClientError({"Error": {"Code": self.codes.pop(0), "Message": "x"}}, "Converse")
        return {"output": {"message": {"content": [{"text": '{"target_visible": true}'}]}}}


@pytest.fixture
def no_sleep(monkeypatch):
    monkeypatch.setattr(vision_core.time, "sleep", lambda s: None)


@pytest.mark.parametrize("code", ["ThrottlingException", "ServiceUnavailableException"])
def test_a_throttle_is_retried_once(no_sleep, code):
    """P2: one attempt (S1) also dropped botocore's retry of a throttle."""
    stub = Throttled(code)
    out = vision_core._converse(stub, modelId="m")
    assert stub.calls == 2 and out["output"]


def test_only_one_retry(no_sleep):
    from botocore.exceptions import ClientError
    stub = Throttled("ThrottlingException", "ThrottlingException")
    with pytest.raises(ClientError):
        vision_core._converse(stub, modelId="m")
    assert stub.calls == 2


def test_other_errors_are_not_retried(no_sleep):
    from botocore.exceptions import ClientError
    stub = Throttled("ValidationException")
    with pytest.raises(ClientError):
        vision_core._converse(stub, modelId="m")
    assert stub.calls == 1


def test_no_retry_when_the_budget_is_spent(no_sleep, monkeypatch):
    """A throttle that arrived late cannot fit another 25 s attempt in 30 s."""
    from botocore.exceptions import ClientError
    ticks = iter([0.0, 10.0])
    monkeypatch.setattr(vision_core.time, "monotonic", lambda: next(ticks))
    stub = Throttled("ThrottlingException")
    with pytest.raises(ClientError):
        vision_core._converse(stub, modelId="m")
    assert stub.calls == 1


def test_a_heic_over_the_bedrock_limit_is_shrunk_not_refused(client, monkeypatch):
    """P3: round 2's gate refused any upload over 3.75 MB before conversion,
    so the shrink-to-fit never ran for the photos it was written for."""
    fake = _fake(monkeypatch, "{}")
    buf = io.BytesIO()
    Image.new("RGB", (1300, 1100), (30, 90, 160)).save(buf, format="BMP")
    assert vision_core.MAX_IMAGE_BYTES < buf.tell() < app_module.MAX_REQUEST_IMAGE_BYTES
    resp = client.post("/navigate", json={
        "image_base64": base64.b64encode(buf.getvalue()).decode(),
        "media_type": "image/bmp", "target_object": "bag"})
    assert resp.status_code == 200
    assert len(fake.calls) == 1


def test_a_request_over_the_lambda_bound_is_a_413(client, monkeypatch):
    fake = _fake(monkeypatch, "{}")
    big = base64.b64encode(b"\0" * (app_module.MAX_REQUEST_IMAGE_BYTES + 1)).decode()
    resp = client.post("/navigate", json={"image_base64": big, "target_object": "bag"})
    assert resp.status_code == 413 and fake.calls == []


def test_the_request_bound_fits_a_lambda_payload():
    import math
    encoded = 4 * math.ceil(app_module.MAX_REQUEST_IMAGE_BYTES / 3)
    assert encoded + 1024 <= 6 * 1024 * 1024


def test_shrinking_takes_at_most_two_extra_encodes(monkeypatch):
    """P5: up to eight full re-encodes of a 48 MP image."""
    import os
    monkeypatch.setattr(vision_core, "MAX_IMAGE_BYTES", 60_000)
    noise = Image.frombytes("RGB", (600, 600), os.urandom(600 * 600 * 3))
    src = io.BytesIO()
    noise.save(src, format="PNG")
    saves = []
    orig_save = Image.Image.save

    def counting(self, fp, format=None, **kw):
        if format == "JPEG":
            saves.append(1)
        return orig_save(self, fp, format=format, **kw)

    monkeypatch.setattr(Image.Image, "save", counting)
    out = vision_core._convert_to_jpeg(src.getvalue())
    assert len(out) <= 60_000
    assert len(saves) <= 3


# ---------- round 4 ----------


def test_the_retry_budget_counts_the_conversion_before_the_call(client, monkeypatch):
    """G1: the budget ran from inside _converse, so seconds spent converting a
    HEIC were ignored and a retry could still pass Lambda's 30 s kill."""
    clock = [1000.0]
    monkeypatch.setattr(vision_core, "_now", lambda: clock[0], raising=False)

    def slow_convert(b):
        clock[0] += 5.0  # a 48 MP HEIC takes seconds
        return b"\xff\xd8small"

    monkeypatch.setattr(vision_core, "_convert_to_jpeg", slow_convert)
    monkeypatch.setattr(vision_core.time, "sleep", lambda s: None)
    stub = Throttled("ThrottlingException")
    monkeypatch.setattr(vision_core, "_get_client", lambda *a, **kw: stub)
    resp = client.post("/navigate", json={
        "image_base64": IMAGE_B64, "media_type": "image/heic", "target_object": "bag"})
    # 5 s converting + 1 s backoff + a 25 s attempt does not fit 28 s.
    assert stub.calls == 1
    assert resp.status_code == 502


def test_a_throttle_early_in_the_request_is_still_retried(client, monkeypatch):
    monkeypatch.setattr(vision_core.time, "sleep", lambda s: None)
    stub = Throttled("ThrottlingException")
    monkeypatch.setattr(vision_core, "_get_client", lambda *a, **kw: stub)
    resp = client.post("/navigate", json={"image_base64": IMAGE_B64, "target_object": "bag"})
    assert stub.calls == 2 and resp.status_code == 200


def test_a_native_image_over_8000_px_is_shrunk(monkeypatch):
    """G2: a 9000-px PNG went to Bedrock as is: ValidationException, 502,
    read by the brain as an outage."""
    fake = _fake(monkeypatch, "{}")
    vision_core.describe_image_bytes_navigate(_png((9000, 10)), "bag", "image/png")
    block = fake.calls[0]["messages"][0]["content"][0]["image"]
    assert max(Image.open(io.BytesIO(block["source"]["bytes"])).size) <= 8000
    assert block["format"] == "jpeg"


def test_a_native_image_over_the_byte_limit_is_shrunk_not_refused(monkeypatch):
    """G2: a JPEG over 3.75 MB got an avoidable 413."""
    import os
    monkeypatch.setattr(vision_core, "MAX_IMAGE_BYTES", 60_000)
    buf = io.BytesIO()
    Image.frombytes("RGB", (600, 600), os.urandom(600 * 600 * 3)).save(buf, format="JPEG", quality=95)
    assert buf.tell() > 60_000
    fake = _fake(monkeypatch, "{}")
    vision_core.describe_image_bytes_navigate(buf.getvalue(), "bag", "image/jpeg")
    assert len(fake.calls[0]["messages"][0]["content"][0]["image"]["source"]["bytes"]) <= 60_000


def test_a_small_native_image_is_sent_untouched(monkeypatch):
    fake = _fake(monkeypatch, "{}")
    png = _png((40, 30))
    vision_core.describe_image_bytes_navigate(png, "bag", "image/png")
    block = fake.calls[0]["messages"][0]["content"][0]["image"]
    assert block["source"]["bytes"] == png and block["format"] == "png"


def test_unreadable_native_bytes_are_sent_as_before(monkeypatch):
    fake = _fake(monkeypatch, "{}")
    vision_core.describe_image_bytes_navigate(b"not-a-jpeg", "bag", "image/jpeg")
    assert fake.calls[0]["messages"][0]["content"][0]["image"]["source"]["bytes"] == b"not-a-jpeg"


# ---------- round 5 ----------


def test_a_bomb_sized_native_image_is_still_shrunk(monkeypatch):
    """Pillow's own bomb check raised inside _needs_shrinking, which read it
    as 'not an image': a 14000 px flat PNG went to Bedrock as is (a
    ValidationException -> 502 -> CloudUnavailable). Scaled down: Pillow's
    limit and the side limit lowered so a 100 px PNG plays the part."""
    from PIL import Image as PILImage
    monkeypatch.setattr(PILImage, "MAX_IMAGE_PIXELS", 1000)
    monkeypatch.setattr(vision_core, "MAX_IMAGE_SIDE_PX", 50)
    fake = _fake(monkeypatch, "{}")
    vision_core.describe_image_bytes_navigate(_png((100, 100)), "bag", "image/png")
    block = fake.calls[0]["messages"][0]["content"][0]["image"]
    assert block["format"] == "jpeg"
    monkeypatch.undo()  # Pillow's limit back, to read the result
    assert max(Image.open(io.BytesIO(block["source"]["bytes"])).size) <= 50


@pytest.mark.parametrize("media_type", ["image/png", "image/bmp"])
def test_over_the_decode_ceiling_is_a_413_not_an_oom(client, monkeypatch, media_type):
    """An 89-178 MP PNG passed Image.open, then convert + thumbnail took over
    1 GB and the 1024 MB function was killed with nothing logged."""
    monkeypatch.setattr(vision_core, "MAX_IMAGE_SIDE_PX", 50)
    monkeypatch.setattr(vision_core, "MAX_DECODE_PIXELS", 5000)
    fake = _fake(monkeypatch, "{}")
    buf = io.BytesIO()
    Image.new("RGB", (100, 100), (1, 2, 3)).save(buf, format="PNG" if media_type == "image/png" else "BMP")
    resp = client.post("/navigate", json={
        "image_base64": base64.b64encode(buf.getvalue()).decode(),
        "media_type": media_type, "target_object": "bag"})
    assert resp.status_code == 413
    assert "pixels" in resp.json()["detail"]
    assert fake.calls == []


def test_a_big_jpeg_is_decoded_at_a_reduced_size(monkeypatch):
    """A JPEG is decoded through draft(), so one whose full decode is over
    the ceiling but whose reduced decode is not is still described."""
    monkeypatch.setattr(vision_core, "MAX_IMAGE_SIDE_PX", 100)
    monkeypatch.setattr(vision_core, "MAX_DECODE_PIXELS", 20_000)  # 400x300 = 120,000
    buf = io.BytesIO()
    Image.new("RGB", (400, 300), (9, 9, 9)).save(buf, format="JPEG")
    fake = _fake(monkeypatch, "{}")
    vision_core.describe_image_bytes_navigate(buf.getvalue(), "bag", "image/jpeg")
    sent = fake.calls[0]["messages"][0]["content"][0]["image"]["source"]["bytes"]
    assert max(Image.open(io.BytesIO(sent)).size) <= 100


def test_an_mpo_jpeg_is_decoded_at_a_reduced_size_too(monkeypatch):
    """Round 6: iPhone/Samsung photos with a second picture open as format
    "MPO" (a JpegImageFile); the `format == "JPEG"` test skipped their
    draft() and refused them 413 at the ceiling."""
    monkeypatch.setattr(vision_core, "MAX_IMAGE_SIDE_PX", 100)
    monkeypatch.setattr(vision_core, "MAX_DECODE_PIXELS", 20_000)
    buf = io.BytesIO()
    frames = [Image.new("RGB", (400, 300), (9, 9, 9)), Image.new("RGB", (400, 300), (7, 7, 7))]
    frames[0].save(buf, format="MPO", save_all=True, append_images=frames[1:])
    assert Image.open(io.BytesIO(buf.getvalue())).format == "MPO"
    fake = _fake(monkeypatch, "{}")
    vision_core.describe_image_bytes_navigate(buf.getvalue(), "bag", "image/jpeg")
    sent = fake.calls[0]["messages"][0]["content"][0]["image"]["source"]["bytes"]
    assert max(Image.open(io.BytesIO(sent)).size) <= 100
