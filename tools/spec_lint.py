"""
tools/spec_lint.py

The deterministic half of decision 0001
(docs/decisions/0001-architecture-vs-engineering-specs.md): architecture
specs say WHAT and WHY, engineering specs say HOW, and the two live apart.
The judgement half -- does a sentence commit to one way of building it --
is docs/SPEC_REVIEW_PROMPT.md; this file checks only what a script can
check without being wrong.

ERRORS fail the build (tests/test_spec_lint.py runs this over docs/):

  E1  front matter: every spec under docs/ (archive and templates
      excepted) opens with
      `---` front matter naming kind, domain, status and verified (an ISO
      date); kind is architecture | engineering | decision; status is one
      of STATUSES.
  E2  location: an architecture spec is docs/<domain>/*.md, an engineering
      spec is docs/engineering/<domain>/*.md, a decision record is
      docs/decisions/*.md -- and the directory names the same domain the
      front matter does.
  E3  a TAGGED code fence (```python, ```bash, ```yaml ...) in an
      architecture spec or decision record. HOW, by construction. The
      diagram tags in DIAGRAM_TAGS are allowed.
  E4  parent: an engineering spec names `parent:`, a repo-root path to an
      architecture spec that exists and has the same domain.
  E5  pairing: every domain with one half has the other.
  E6  references: a repo path in backticks, and a relative markdown link,
      must exist. This is the drift docs-review/REPORT.md found most.
  E7  required sections, per kind (REQUIRED_SECTIONS), matched on the
      start of a `## ` heading, case-insensitive.
  E8  the index: every domain appears in docs/README.md.

WARNINGS are printed and never fail:

  W1  an UNTAGGED fence in an architecture spec that looks like commands
      or config (a shell prompt, pip/uvicorn/docker/ros2/curl, key: value
      lines). Untagged fences are where diagrams live, so this cannot be
      an error; it is the common way HOW leaks in untagged.
  W2  in an architecture spec: a config key from config/robot.yaml, an
      environment variable, a CLI flag, or a call signature with
      arguments -- each a HOW fact in backticks.
  W3  an engineering spec with fewer than MIN_ENGINEERING_FACTS concrete
      items (backticked identifiers + table rows): it is restating the
      decision rather than supplying the how.
  W4  verified more than STALE_DAYS ago.
  W5  an engineering spec whose body never links to its parent.

Run:  python tools/spec_lint.py            (exit 1 on any error)
      python tools/spec_lint.py --strict   (warnings fail too)
Stdlib plus PyYAML, which requirements.txt already carries.
"""

from __future__ import annotations

import argparse
import datetime as dt
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent

KINDS = ("architecture", "engineering", "decision")
STATUSES = ("current", "draft", "planned", "superseded", "accepted")
DIAGRAM_TAGS = {"text", "mermaid"}
STALE_DAYS = 120
MIN_ENGINEERING_FACTS = 15

# Files under docs/ that are not specs and carry no front matter.
# docs/ARCHITECTURE.md is the whole-system overview: it spans every domain,
# so it has no domain to sit under, but its references are still checked
# (tests/test_spec_lint.py).
NOT_SPECS = {"docs/README.md", "docs/SPEC_REVIEW_PROMPT.md", "docs/ARCHITECTURE.md"}
EXCLUDED_DIRS = ("docs/archive/", "docs/templates/")

REQUIRED_SECTIONS = {
    "architecture": (
        "Purpose",
        "Components and boundaries",
        "Decisions",
        "Contracts",
        "Failure modes",
    ),
    "engineering": (
        "Implementation",
        "Interfaces",
        "Parameters",
        "Procedures",
        "Verification",
    ),
    "decision": ("Context", "Decision", "Alternatives rejected", "Consequences"),
}

FENCE = re.compile(r"^(\s*)(`{3,}|~{3,})\s*([\w+-]*)")
BACKTICK = re.compile(r"(?<!`)`([^`\n]+)`(?!`)")
LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")
HEADING = re.compile(r"^##\s+(.+?)\s*$")
TABLE_ROW = re.compile(r"^\s*\|.*\|\s*$")
TABLE_RULE = re.compile(r"^\s*\|[\s:|-]+\|\s*$")

PATH_SUFFIXES = (
    ".py", ".md", ".yaml", ".yml", ".js", ".html", ".sh", ".cpp", ".hpp",
    ".xacro", ".json", ".txt", ".patch", ".toml", ".cfg", ".xml", ".urdf",
)
# What an untagged fence holding HOW looks like.
HOW_IN_FENCE = re.compile(
    r"^\s*(\$ |# |pip |python |uvicorn |docker |ros2 |colcon |curl |export |"
    r"bash |sudo |git |npm |aws |pytest |[A-Za-z_][\w.-]*:\s+\S)"
)
ENV_VAR = re.compile(r"^[A-Z][A-Z0-9]*(_[A-Z0-9]+)+(=.*)?$")
CLI_FLAG = re.compile(r"^--?[a-z][\w-]*(=.*)?$")
SIGNATURE = re.compile(r"^[\w.]+\([^)]+\)$")


@dataclass
class Finding:
    path: str
    line: int
    code: str
    message: str

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: {self.code} {self.message}"


@dataclass
class Spec:
    path: Path
    rel: str
    meta: dict
    body: list[str]          # lines after the front matter
    body_start: int          # 1-based line number of body[0]
    sections: list[str] = field(default_factory=list)

    @property
    def kind(self) -> str:
        return str(self.meta.get("kind", ""))

    @property
    def domain(self) -> str:
        return str(self.meta.get("domain", ""))


@dataclass
class Report:
    errors: list[Finding] = field(default_factory=list)
    warnings: list[Finding] = field(default_factory=list)

    def error(self, path, line, code, msg):
        self.errors.append(Finding(path, line, code, msg))

    def warn(self, path, line, code, msg):
        self.warnings.append(Finding(path, line, code, msg))


# ---------------------------------------------------------------- parsing

def parse(path: Path, repo: Path) -> tuple[dict | None, list[str], int]:
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0].strip() != "---":
        return None, lines, 1
    for i in range(1, len(lines)):
        if lines[i].strip() == "---":
            try:
                meta = yaml.safe_load("\n".join(lines[1:i])) or {}
            except yaml.YAMLError:
                meta = {}
            if not isinstance(meta, dict):
                meta = {}
            return meta, lines[i + 1:], i + 2
    return None, lines, 1


def iter_body(spec: Spec):
    """Yield (line_no, text, in_fence, fence_tag) for every body line."""
    in_fence, tag, marker = False, "", ""
    for i, text in enumerate(spec.body):
        n = spec.body_start + i
        m = FENCE.match(text)
        if m and not in_fence:
            in_fence, marker, tag = True, m.group(2)[0] * 3, m.group(3).lower()
            yield n, text, "open", tag
            continue
        if in_fence and text.strip().startswith(marker):
            in_fence = False
            yield n, text, "close", tag
            tag = ""
            continue
        yield n, text, in_fence, tag


def config_keys(repo: Path) -> set[str]:
    cfg = repo / "config" / "robot.yaml"
    keys: set[str] = set()
    if not cfg.exists():
        return keys

    def walk(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if isinstance(k, str) and "_" in k:
                    keys.add(k)
                walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)

    walk(yaml.safe_load(cfg.read_text()) or {})
    return keys


def looks_like_path(token: str) -> bool:
    t = token.strip()
    if not t or " " in t or any(c in t for c in "<>*{}$|"):
        return False
    if t.startswith(("/", "~", "http", "-", ".", "@")) or "://" in t:
        return False
    t = re.split(r"::|:(?=\d)", t)[0]   # file.py:120, file.py::test_x
    if "/" not in t.rstrip("/"):
        return False    # `exports/`: a bare name/ is as often an S3 prefix as a dir
    return t.endswith("/") or t.endswith(PATH_SUFFIXES)


def path_of(token: str) -> str:
    return re.split(r"::|:(?=\d)", token.strip())[0]


# ---------------------------------------------------------------- checks

def collect(repo: Path, rep: Report) -> list[Spec]:
    docs = repo / "docs"
    specs = []
    for path in sorted(docs.rglob("*.md")):
        rel = path.relative_to(repo).as_posix()
        if rel in NOT_SPECS or rel.startswith(EXCLUDED_DIRS):
            continue
        meta, body, start = parse(path, repo)
        if meta is None:
            rep.error(rel, 1, "E1", "no front matter: a spec opens with `---` "
                      "naming kind, domain, status and verified")
            continue
        spec = Spec(path, rel, meta, body, start)
        spec.sections = [m.group(1) for t in body if (m := HEADING.match(t))]
        specs.append(spec)
    return specs


def check_meta(spec: Spec, rep: Report) -> None:
    for key in ("kind", "domain", "status", "verified"):
        if key not in spec.meta:
            rep.error(spec.rel, 1, "E1", f"front matter is missing `{key}`")
    if spec.kind and spec.kind not in KINDS:
        rep.error(spec.rel, 1, "E1", f"kind `{spec.kind}` is not one of {KINDS}")
    status = spec.meta.get("status")
    if status is not None and status not in STATUSES:
        rep.error(spec.rel, 1, "E1", f"status `{status}` is not one of {STATUSES}")
    verified = spec.meta.get("verified")
    if verified is not None and not isinstance(verified, dt.date):
        try:
            dt.date.fromisoformat(str(verified))
        except ValueError:
            rep.error(spec.rel, 1, "E1", f"verified `{verified}` is not an ISO date")


def check_location(spec: Spec, rep: Report) -> None:
    parts = Path(spec.rel).parts       # ('docs', ..., 'X.md')
    if spec.kind == "architecture":
        ok = len(parts) == 3 and parts[1] not in ("engineering", "decisions")
        where = "docs/<domain>/"
        dom = parts[1] if len(parts) == 3 else None
    elif spec.kind == "engineering":
        ok = len(parts) == 4 and parts[1] == "engineering"
        where = "docs/engineering/<domain>/"
        dom = parts[2] if len(parts) == 4 else None
    elif spec.kind == "decision":
        ok = len(parts) == 3 and parts[1] == "decisions"
        where, dom = "docs/decisions/", None
    else:
        return
    if not ok:
        rep.error(spec.rel, 1, "E2", f"a {spec.kind} spec lives in {where}")
    elif dom is not None and dom != spec.domain:
        rep.error(spec.rel, 1, "E2",
                  f"directory says domain `{dom}`, front matter says `{spec.domain}`")


def check_fences(spec: Spec, rep: Report) -> None:
    if spec.kind not in ("architecture", "decision"):
        return
    block: list[tuple[int, str]] = []
    for n, text, state, tag in iter_body(spec):
        if state == "open":
            block = []
            if tag and tag not in DIAGRAM_TAGS:
                rep.error(spec.rel, n, "E3",
                          f"```{tag} fence in a {spec.kind} spec: code and config "
                          "are HOW and belong in the engineering spec")
        elif state is True:
            block.append((n, text))
        elif state == "close" and not tag and spec.kind == "architecture":
            hits = [n for n, t in block if HOW_IN_FENCE.match(t)]
            if hits:
                rep.warn(spec.rel, hits[0], "W1",
                         "untagged fence holds commands or config -- HOW, untagged")


def check_how_tokens(spec: Spec, rep: Report, cfg_keys: set[str]) -> None:
    if spec.kind != "architecture":
        return
    for n, text, state, _ in iter_body(spec):
        if state is not False:
            continue
        for tok in BACKTICK.findall(text):
            t = tok.strip()
            if t in cfg_keys or t.split(".")[-1] in cfg_keys and "." in t:
                rep.warn(spec.rel, n, "W2", f"config key `{t}` -- a HOW fact")
            elif ENV_VAR.match(t):
                rep.warn(spec.rel, n, "W2", f"environment variable `{t}` -- a HOW fact")
            elif CLI_FLAG.match(t):
                rep.warn(spec.rel, n, "W2", f"CLI flag `{t}` -- a HOW fact")
            elif SIGNATURE.match(t):
                rep.warn(spec.rel, n, "W2", f"signature `{t}` -- name the "
                         "contract here, the signature in the engineering spec")


def check_references(spec: Spec, rep: Report, repo: Path) -> None:
    for n, text, state, _ in iter_body(spec):
        if state is not False:
            continue
        for tok in BACKTICK.findall(text):
            if looks_like_path(tok) and not (repo / path_of(tok)).exists():
                rep.error(spec.rel, n, "E6", f"`{tok}` does not exist")
        for target in LINK.findall(text):
            if target.startswith(("http:", "https:", "mailto:", "#")):
                continue
            target = target.split("#")[0]
            if target and not (spec.path.parent / target).exists():
                rep.error(spec.rel, n, "E6", f"link target `{target}` does not exist")


def check_sections(spec: Spec, rep: Report) -> None:
    have = [s.lower() for s in spec.sections]
    for want in REQUIRED_SECTIONS.get(spec.kind, ()):
        if not any(h.startswith(want.lower()) for h in have):
            rep.error(spec.rel, spec.body_start, "E7",
                      f"missing required section `## {want}`")


def check_parent(spec: Spec, rep: Report, by_rel: dict[str, Spec]) -> None:
    if spec.kind != "engineering":
        return
    parent = spec.meta.get("parent")
    if not parent:
        rep.error(spec.rel, 1, "E4", "engineering spec names no `parent:` "
                  "architecture spec")
        return
    p = by_rel.get(str(parent))
    if p is None:
        rep.error(spec.rel, 1, "E4", f"parent `{parent}` is not an architecture spec under docs/")
        return
    if p.kind != "architecture":
        rep.error(spec.rel, 1, "E4", f"parent `{parent}` is a {p.kind} spec, not architecture")
    if p.domain != spec.domain:
        rep.error(spec.rel, 1, "E4",
                  f"parent's domain `{p.domain}` differs from `{spec.domain}`")
    name = Path(str(parent)).name
    if not any(name in t for t in LINK.findall("\n".join(spec.body))):
        rep.warn(spec.rel, spec.body_start, "W5",
                 "the body never links to its parent architecture spec")


def check_thin(spec: Spec, rep: Report) -> None:
    if spec.kind != "engineering":
        return
    facts = 0
    for _, text, state, _ in iter_body(spec):
        if state is not False:
            facts += 1 if state is True else 0
            continue
        facts += len(BACKTICK.findall(text))
        if TABLE_ROW.match(text) and not TABLE_RULE.match(text):
            facts += 1
    if facts < MIN_ENGINEERING_FACTS:
        rep.warn(spec.rel, spec.body_start, "W3",
                 f"only {facts} concrete items: an engineering spec supplies "
                 "parameters, not a restatement of the decision")


def check_stale(spec: Spec, rep: Report, today: dt.date) -> None:
    v = spec.meta.get("verified")
    try:
        d = v if isinstance(v, dt.date) else dt.date.fromisoformat(str(v))
    except ValueError:
        return
    if (today - d).days > STALE_DAYS:
        rep.warn(spec.rel, 1, "W4", f"verified {d}, more than {STALE_DAYS} days ago")


def check_domains(specs: list[Spec], rep: Report, repo: Path) -> None:
    arch = {s.domain for s in specs if s.kind == "architecture"}
    eng = {s.domain for s in specs if s.kind == "engineering"}
    for d in sorted(arch - eng):
        rep.error(f"docs/{d}/", 1, "E5", f"domain `{d}` has no engineering spec")
    for d in sorted(eng - arch):
        rep.error(f"docs/engineering/{d}/", 1, "E5", f"domain `{d}` has no architecture spec")
    index = repo / "docs" / "README.md"
    text = index.read_text(encoding="utf-8") if index.exists() else ""
    for d in sorted(arch | eng):
        if f"{d}/" not in text:
            rep.error("docs/README.md", 1, "E8", f"domain `{d}` is not in the index")


def lint(repo: Path = REPO, today: dt.date | None = None) -> Report:
    today = today or dt.date.today()
    rep = Report()
    specs = collect(repo, rep)
    by_rel = {s.rel: s for s in specs}
    keys = config_keys(repo)
    for s in specs:
        check_meta(s, rep)
        check_location(s, rep)
        check_fences(s, rep)
        check_how_tokens(s, rep, keys)
        check_references(s, rep, repo)
        check_sections(s, rep)
        check_parent(s, rep, by_rel)
        check_thin(s, rep)
        check_stale(s, rep, today)
    check_domains(specs, rep, repo)
    return rep


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    ap.add_argument("--strict", action="store_true", help="warnings fail too")
    ap.add_argument("--repo", type=Path, default=REPO)
    args = ap.parse_args(argv)
    rep = lint(args.repo)
    for f in rep.errors:
        print(f"ERROR   {f}")
    for f in rep.warnings:
        print(f"warning {f}")
    print(f"{len(rep.errors)} error(s), {len(rep.warnings)} warning(s)")
    return 1 if rep.errors or (args.strict and rep.warnings) else 0


if __name__ == "__main__":
    sys.exit(main())
