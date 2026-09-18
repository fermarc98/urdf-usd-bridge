# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The release gate, run on every PR instead of on every tag push.

``.github/workflows/release.yml`` refuses a tag whose version has no
CHANGELOG.md section, and builds the GitHub release body out of that section.
Both use ``scripts/changelog_section.py``, and so does this file -- the point
being that a format mistake fails here, in a normal test run, rather than
after a tag has been pushed and has to be deleted from the remote.

It has cost two release runs already: the 0.1.1 entry was written as
``## 0.1.1 - ...`` where the workflow looks for ``## [0.1.1]``, and it was
placed inside the 0.1.0 section, which would have given 0.1.1 the whole of
0.1.0's notes. The first fault a grep would catch; the second needs a test
that looks at what comes *out*.
"""

from __future__ import annotations

import sys

import pytest

from tests.conftest import REPO_ROOT

sys.path.insert(0, str(REPO_ROOT / "scripts"))

from changelog_section import (
    MissingSectionError,
    packaged_version,
    section,
    versions,
)

CHANGELOG = REPO_ROOT / "CHANGELOG.md"


@pytest.fixture(scope="module")
def text() -> str:
    if not CHANGELOG.exists():
        pytest.skip("CHANGELOG.md is not present in this tree")
    return CHANGELOG.read_text()


def test_the_packaged_version_has_a_section(text):
    """The exact condition release.yml gates a tag on."""
    version = packaged_version()
    try:
        body = section(version, text)
    except MissingSectionError as exc:
        pytest.fail(
            f"{exc}\n\n"
            f"release.yml will refuse the v{version} tag. Add the section "
            "before tagging, not after."
        )
    assert body.strip(), f"the CHANGELOG section for {version} is empty"


def test_the_section_does_not_swallow_the_previous_release(text):
    """A section ends at the next version, not at the end of the file.

    This is the fault a format check misses: an entry written *underneath*
    another version's heading matches the grep and still produces the wrong
    release notes.
    """
    version = packaged_version()
    body = section(version, text)
    assert "## [" not in body, (
        f"the section for {version} contains another version heading, so it was "
        "written inside a previous release's section rather than above it"
    )


def test_versions_are_newest_first_and_unique(text):
    """Keep a Changelog order, and no version listed twice."""
    found = versions(text)
    assert found, "CHANGELOG.md has no version headings at all"
    assert found[0] == "Unreleased", f"the first section should be Unreleased, got {found[0]!r}"

    releases = found[1:]
    assert len(releases) == len(set(releases)), f"a version appears twice: {releases}"

    def key(v: str) -> tuple[int, ...]:
        return tuple(int(part) for part in v.split("."))

    assert releases == sorted(
        releases, key=key, reverse=True
    ), f"releases should be newest first, got {releases}"


def test_every_release_has_a_link_reference(text):
    """``[0.1.1]`` in a heading renders as literal text without one."""
    defined = {line.split("]:")[0].lstrip("[") for line in text.splitlines() if line.startswith("[")}
    for version in versions(text):
        assert version in defined, (
            f"no link-reference definition for [{version}] at the end of CHANGELOG.md, "
            f"so the heading will render as literal '[{version}]' on GitHub"
        )


def test_the_packaged_version_is_not_left_under_unreleased(text):
    """Shipping a version whose notes still sit under Unreleased."""
    unreleased = section("Unreleased", text)
    version = packaged_version()
    assert (
        version not in unreleased
    ), f"{version} is mentioned under Unreleased; move those notes into its own section"
