"""Everything the pages reference must actually be published.

The static half of the same failure tests/test_serverless_routes.py covers
for the dynamic half, and the same failure that has shipped five times in
this project against the load balancer: the page loads, one file 404s, and
one feature is silently dead. `/app.js` was exactly this -- it used to be an
inline <script>, became its own file, and had no path until someone noticed
the twin was inert.

Moving to S3 changes the mechanism and not the failure. A file referenced by
index.html or admin.html but absent from service/static/assets.json simply
never gets uploaded, and CloudFront answers it with whatever the default
behaviour does -- which for a missing key is a 403 or 404, on a page that
otherwise rendered fine.

So: parse the real HTML for its relative references, and require each one to
be in the manifest that sync.sh uploads.
"""

import json
import re

import pytest

from tests.conftest import REPO_ROOT

MANIFEST = REPO_ROOT / "service" / "static" / "assets.json"


def manifest():
    return json.loads(MANIFEST.read_text())["assets"]


def referenced_paths(html_path):
    """Relative src=/href= targets in a page.

    Absolute URLs, anchors, data: URIs and template placeholders are not
    files this project publishes, so they are skipped rather than asserted
    about.
    """
    text = html_path.read_text()
    out = set()
    for m in re.finditer(r'(?:src|href)\s*=\s*"([^"]+)"', text):
        ref = m.group(1).strip()
        if (not ref or ref.startswith(("http://", "https://", "//", "#", "data:",
                                       "mailto:", "{", "$"))):
            continue
        out.add(ref.lstrip("./"))
    return out


def test_the_manifest_lists_only_files_that_exist():
    missing = [a["src"] for a in manifest() if not (REPO_ROOT / a["src"]).is_file()]
    assert not missing, f"assets.json lists files that do not exist: {missing}"


def test_every_file_the_twin_references_is_published():
    keys = {a["key"] for a in manifest()}
    refs = referenced_paths(REPO_ROOT / "web-twin" / "index.html")
    missing = sorted(r for r in refs if r not in keys)
    assert not missing, (
        f"web-twin/index.html references files not in service/static/assets.json: "
        f"{missing}. They will 404 from CloudFront while working locally.")


def test_every_file_the_admin_console_references_is_published():
    keys = {a["key"] for a in manifest()}
    refs = referenced_paths(REPO_ROOT / "control" / "admin.html")
    missing = sorted(r for r in refs if r not in keys)
    assert not missing, (
        f"control/admin.html references files not in service/static/assets.json: "
        f"{missing}.")


def test_every_icon_the_pwa_manifest_references_is_published():
    """manifest.json is a third referencing file and the easiest to forget:
    nothing on screen changes when an icon it names is missing, the install
    prompt just quietly does not offer the right icon. CLAUDE.md records that
    both shipped UI bugs in this project were found on a phone rather than by
    the suite; this is the cheap half of that lesson."""
    keys = {a["key"] for a in manifest()}
    icons = json.loads((REPO_ROOT / "web-twin" / "manifest.json").read_text())["icons"]
    missing = sorted(i["src"].lstrip("/") for i in icons
                     if i["src"].lstrip("/") not in keys)
    assert not missing, (
        f"web-twin/manifest.json names icons not in assets.json: {missing}")


def test_keys_mirror_the_reference_paths_exactly():
    """Every reference is relative, so the S3 key IS the URL path. A key that
    renames its file (app.js -> assets/app.js) silently breaks the page."""
    for a in manifest():
        assert not a["key"].startswith("/"), f"{a['key']}: keys are not rooted"
        assert ".." not in a["key"]


def test_the_admin_page_keeps_its_extensionless_key():
    """control/admin.html is served at /admin today and the console's own
    links assume it. An `admin.html` key would move the console."""
    entry = next(a for a in manifest() if a["src"] == "control/admin.html")
    assert entry["key"] == "admin"
    assert entry["type"].startswith("text/html"), (
        "an extensionless key cannot have its content type guessed -- without "
        "an explicit text/html the browser downloads the console instead of "
        "rendering it")


@pytest.mark.parametrize("src", ["web-twin/app.js", "control/admin.js", "web-twin/index.html"])
def test_the_pages_and_scripts_are_served_no_cache(src):
    """This project deliberately does not use content-hashed filenames (see
    web-twin/app.js's banner), so a cached script is a stale UI that nobody
    can reproduce and a redeploy does not fix."""
    entry = next(a for a in manifest() if a["src"] == src)
    assert entry["cache"] == "none"


def test_the_sync_script_reads_the_same_manifest():
    """Two lists of what the site consists of would drift; there is one."""
    text = (REPO_ROOT / "service" / "static" / "sync.sh").read_text()
    assert "assets.json" in text
    # Comments stripped first: the script explains at length why it does not
    # use `aws s3 sync`, and matching that prose is how this assertion failed
    # the first time it ran.
    code = "\n".join(ln for ln in text.splitlines()
                     if not ln.lstrip().startswith("#"))
    assert "s3 sync" not in code, (
        "aws s3 sync guesses content types from extensions, and the admin "
        "key deliberately has none")
    # macOS ships bash 3.2. `mapfile` does not exist there and the failure is
    # SILENT: the array comes back empty, the loop never runs, the script
    # uploads nothing and exits 0. It was written with mapfile first and did
    # exactly that.
    assert "mapfile" not in code and "readarray" not in code, (
        "mapfile/readarray need bash 4; macOS has 3.2 and fails silently")
