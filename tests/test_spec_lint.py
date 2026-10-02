"""
tests/test_spec_lint.py

Decision 0001 (docs/decisions/0001-architecture-vs-engineering-specs.md),
enforced: the repository's specs pass tools/spec_lint.py with no errors,
and every check the linter claims to make is shown to fire -- each one
against a minimal valid repo with exactly one thing broken. A linter that
has never been seen to fail has not been tested.

Static: no servers, no network. Run with: pytest tests/test_spec_lint.py -v
"""

import datetime as dt
import textwrap
from pathlib import Path

import pytest

from tools import spec_lint

TODAY = dt.date(2026, 10, 2)


# ---------------------------------------------------------------- the real docs

def test_repository_specs_have_no_errors():
    rep = spec_lint.lint(spec_lint.REPO, today=TODAY)
    assert not rep.errors, "\n" + "\n".join(map(str, rep.errors))


def test_every_domain_in_the_index_has_both_specs():
    # The index is the reading path; a row pointing at a missing spec is a
    # dead end E6 would also catch, but name it as the failure it is.
    index = (spec_lint.REPO / "docs" / "README.md").read_text()
    rows = [l for l in index.splitlines() if l.startswith("| ") and "ENGINEERING.md" in l]
    assert rows, "the index lists no domains"
    for row in rows:
        domain = row.split("|")[1].strip()
        assert (spec_lint.REPO / "docs" / domain / "ARCHITECTURE.md").exists(), domain
        assert (spec_lint.REPO / "docs" / "engineering" / domain / "ENGINEERING.md").exists(), domain


# ---------------------------------------------------------------- a fixture repo

ARCH = """\
---
kind: architecture
domain: brakes
status: current
verified: 2026-10-01
---

# Brakes -- architecture

See the [engineering spec](../engineering/brakes/ENGINEERING.md).

## Purpose

Stops the robot.

## Components and boundaries

```text
brain --> brakes --> wheels
```

## Decisions

Local, because a link can die. Rejected: remote braking.

## Contracts

The brain asks; the brakes decide.

## Failure modes and resilience targets

A dead link stops the wheels within one watchdog period.
"""

ENG = """\
---
kind: engineering
domain: brakes
status: current
verified: 2026-10-01
parent: docs/brakes/ARCHITECTURE.md
---

# Brakes -- engineering

How it is built; why is in the [architecture spec](../../brakes/ARCHITECTURE.md).

## Implementation

`robot/brakes.py` holds `Brakes`, `Brakes.apply`, `Brakes.release`.

## Interfaces

`POST /brake`, `GET /brake`, `x-driver`, `reason`, `stopped`.

## Parameters and configuration

| Key | Default | Unit |
|---|---|---|
| `brake_margin_cm` | 3 | cm |
| `brake_period_s` | 0.05 | s |
| `brake_retries` | 3 | count |

## Procedures

```bash
python -m robot.brakes --check
```

## Verification

`tests/test_brakes.py` pins `apply`, `release`, `margin`.

## Known gaps

None.
"""

DECISION = """\
---
kind: decision
domain: specs
status: accepted
verified: 2026-10-01
---

# 0001 -- a decision

## Context

Why.

## Decision

What.

## Alternatives rejected

Others.

## Consequences

Then.
"""


def write(root: Path, rel: str, text: str) -> None:
    p = root / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(textwrap.dedent(text))


@pytest.fixture
def repo(tmp_path):
    write(tmp_path, "config/robot.yaml", "safety:\n  min_distance_cm: 20\n")
    write(tmp_path, "robot/brakes.py", "")
    write(tmp_path, "tests/test_brakes.py", "")
    write(tmp_path, "docs/README.md", "| brakes | [brakes/](brakes/ARCHITECTURE.md) |\n")
    write(tmp_path, "docs/brakes/ARCHITECTURE.md", ARCH)
    write(tmp_path, "docs/engineering/brakes/ENGINEERING.md", ENG)
    write(tmp_path, "docs/decisions/0001-x.md", DECISION)
    return tmp_path


def codes(rep, kind="errors"):
    return sorted({f.code for f in getattr(rep, kind)})


def edit(root: Path, rel: str, old: str, new: str) -> None:
    p = root / rel
    text = p.read_text()
    assert old in text, f"fixture lacks {old!r}"
    p.write_text(text.replace(old, new, 1))


A = "docs/brakes/ARCHITECTURE.md"
E = "docs/engineering/brakes/ENGINEERING.md"


def test_the_fixture_is_clean(repo):
    rep = spec_lint.lint(repo, today=TODAY)
    assert rep.errors == [] and rep.warnings == [], rep.errors + rep.warnings


# ---------------------------------------------------------------- errors

def test_e1_missing_front_matter(repo):
    write(repo, A, ARCH.split("---\n", 2)[2])
    assert "E1" in codes(spec_lint.lint(repo, today=TODAY))


def test_e1_missing_field_and_bad_values(repo):
    edit(repo, A, "status: current\n", "")
    edit(repo, E, "status: current", "status: shipped")
    edit(repo, "docs/decisions/0001-x.md", "verified: 2026-10-01", "verified: last week")
    errs = [f for f in spec_lint.lint(repo, today=TODAY).errors if f.code == "E1"]
    assert len(errs) == 3


def test_e2_architecture_in_the_engineering_tree(repo):
    edit(repo, E, "kind: engineering", "kind: architecture")
    assert "E2" in codes(spec_lint.lint(repo, today=TODAY))


def test_e2_directory_and_front_matter_disagree_on_domain(repo):
    edit(repo, A, "domain: brakes", "domain: wheels")
    assert "E2" in codes(spec_lint.lint(repo, today=TODAY))


def test_e3_tagged_fence_in_architecture(repo):
    edit(repo, A, "```text", "```python")
    assert "E3" in codes(spec_lint.lint(repo, today=TODAY))


def test_e3_tagged_fence_in_a_decision(repo):
    edit(repo, "docs/decisions/0001-x.md", "What.", "```yaml\na: 1\n```")
    assert "E3" in codes(spec_lint.lint(repo, today=TODAY))


def test_e3_diagram_tags_and_engineering_fences_are_allowed(repo):
    edit(repo, A, "```text", "```mermaid")
    assert spec_lint.lint(repo, today=TODAY).errors == []   # ENG has ```bash


def test_e4_no_parent(repo):
    edit(repo, E, "parent: docs/brakes/ARCHITECTURE.md\n", "")
    assert "E4" in codes(spec_lint.lint(repo, today=TODAY))


def test_e4_parent_is_not_an_architecture_spec(repo):
    edit(repo, E, "parent: docs/brakes/ARCHITECTURE.md", "parent: docs/decisions/0001-x.md")
    assert "E4" in codes(spec_lint.lint(repo, today=TODAY))


def test_e5_a_domain_with_one_half(repo):
    (repo / E).unlink()
    assert "E5" in codes(spec_lint.lint(repo, today=TODAY))


def test_e6_backticked_path_that_does_not_exist(repo):
    edit(repo, E, "`robot/brakes.py`", "`robot/brake.py`")
    assert "E6" in codes(spec_lint.lint(repo, today=TODAY))


def test_e6_path_with_line_number_resolves_to_the_file(repo):
    edit(repo, E, "`robot/brakes.py`", "`robot/brakes.py:12`")
    assert spec_lint.lint(repo, today=TODAY).errors == []


def test_e6_broken_relative_link(repo):
    edit(repo, A, "(../engineering/brakes/ENGINEERING.md)", "(../engineering/brake/ENGINEERING.md)")
    assert "E6" in codes(spec_lint.lint(repo, today=TODAY))


def test_e6_placeholders_routes_and_devices_are_not_paths(repo):
    edit(repo, A, "Stops the robot.",
         "Stops it. `docs/<domain>/`, `/dev/ttyUSB0`, `/wheels`, `~/.secrets`, `a/*.py`, "
         "`s3://bucket/walks/`, `file://x/y.json`, `exports/`.")
    assert spec_lint.lint(repo, today=TODAY).errors == []


def test_e7_missing_required_section(repo):
    edit(repo, A, "## Contracts", "## Interfaces with neighbours")
    edit(repo, E, "## Verification", "## Tests")
    errs = [f for f in spec_lint.lint(repo, today=TODAY).errors if f.code == "E7"]
    assert {f.path for f in errs} == {A, E}


def test_e8_domain_missing_from_the_index(repo):
    write(repo, "docs/README.md", "nothing here\n")
    assert "E8" in codes(spec_lint.lint(repo, today=TODAY))


# ---------------------------------------------------------------- warnings

def test_w1_untagged_fence_holding_commands(repo):
    edit(repo, A, "```text\nbrain --> brakes --> wheels\n```",
         "```\n$ uvicorn robot.server:app\n```")
    rep = spec_lint.lint(repo, today=TODAY)
    assert rep.errors == [] and codes(rep, "warnings") == ["W1"]


def test_w1_untagged_diagram_is_fine(repo):
    edit(repo, A, "```text", "```")
    assert spec_lint.lint(repo, today=TODAY).warnings == []


@pytest.mark.parametrize("token", [
    "min_distance_cm", "safety.min_distance_cm", "ROBOT_DRIVE", "--policy", "turn_left(45)",
])
def test_w2_how_tokens_in_architecture(repo, token):
    edit(repo, A, "The brain asks; the brakes decide.", f"The brain asks via `{token}`.")
    assert codes(spec_lint.lint(repo, today=TODAY), "warnings") == ["W2"]


def test_w2_a_named_contract_is_not_a_signature(repo):
    edit(repo, A, "The brain asks; the brakes decide.", "The brain calls `stop()` on `RobotInterface`.")
    assert spec_lint.lint(repo, today=TODAY).warnings == []


def test_w3_engineering_spec_that_only_restates(repo):
    thin = ENG.split("## Implementation")[0] + "".join(
        f"## {s}\n\nIt stops the robot, locally.\n\n"
        for s in ("Implementation", "Interfaces", "Parameters", "Procedures", "Verification"))
    write(repo, E, thin)
    assert "W3" in codes(spec_lint.lint(repo, today=TODAY), "warnings")


def test_w4_stale_verification(repo):
    rep = spec_lint.lint(repo, today=TODAY + dt.timedelta(days=200))
    assert "W4" in codes(rep, "warnings")


def test_w5_engineering_body_never_links_its_parent(repo):
    edit(repo, E, "[architecture spec](../../brakes/ARCHITECTURE.md)", "architecture spec")
    assert codes(spec_lint.lint(repo, today=TODAY), "warnings") == ["W5"]


# ---------------------------------------------------------------- the CLI

def test_cli_exit_codes(repo, capsys):
    assert spec_lint.main(["--repo", str(repo)]) == 0
    edit(repo, A, "```text", "```python")
    assert spec_lint.main(["--repo", str(repo)]) == 1
    assert "E3" in capsys.readouterr().out


def test_cli_strict_fails_on_warnings(repo):
    edit(repo, A, "The brain asks; the brakes decide.", "Set `ROBOT_DRIVE`.")
    assert spec_lint.main(["--repo", str(repo)]) == 0
    assert spec_lint.main(["--repo", str(repo), "--strict"]) == 1
