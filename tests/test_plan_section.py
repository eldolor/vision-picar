"""tools/plan_section.py -- the parallel-sessions tooling
(docs/guides/PARALLEL-SESSIONS.md)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
import plan_section as ps  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_every_section_has_a_file_and_the_index_is_fresh():
    secs = ps.sections(ROOT)
    nums = [n for n, _, _ in secs]
    assert len(nums) == len(set(nums)), "duplicate section numbers"
    index = open(os.path.join(ROOT, ps.INDEX)).read()
    for n, _, f in secs:
        assert f"[{n}]({f})" in index, f"{f} missing from the index: run plan_section.py index"


def test_the_main_plan_no_longer_holds_sections():
    text = open(os.path.join(ROOT, ps.PLAN)).read()
    assert not any(ps.HEAD_RE.match(l) for l in text.split("\n"))


def test_resources_never_collide_between_sections():
    seen = {}
    for minor in range(1, 100):
        r = ps.resources(f"3.{minor}")
        for k in ("PICAR_ROBOT_PORT", "PICAR_BRIDGE_PORT", "ROS_DOMAIN_ID", "PICAR_ROS_CONTAINER"):
            key = (k, r[k])
            assert key not in seen, f"3.{minor} and {seen[key]} share {k}"
            seen[key] = f"3.{minor}"
        assert r["ROS_IMAGE"] != "vision-picar-ros:latest"
        assert r["ROS_DOMAIN_ID"] not in (0, 73, 74) and r["ROS_DOMAIN_ID"] <= 101
        assert r["PICAR_ROBOT_PORT"] >= 9000


def test_slugs_are_file_safe():
    assert ps.slugify("The stop distance follows `speed` (2026-10-07): less room") \
        == "the-stop-distance-follows-speed-less"
