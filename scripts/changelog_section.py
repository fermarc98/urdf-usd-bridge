# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Extract one version's section from CHANGELOG.md, or fail saying why.

This is the single implementation of "which lines are release 0.1.1's notes".
``.github/workflows/release.yml`` calls it to gate a tag and to write the
GitHub release body, and ``tests/unit/test_changelog.py`` calls it on every
PR. One implementation, so the gate and the test cannot drift apart.

It exists because they did drift. The workflow looked for ``^## [<version>]``
while the 0.1.1 entry had been written as ``## 0.1.1 - ...``, without the
brackets, and the mismatch was only discovered by pushing a tag -- twice.
Nothing before the tag push had any opinion about the format.

The same entry had a second fault the format check would not have caught: it
was placed *inside* the 0.1.0 section, so the extraction would have handed
0.1.1 the whole of 0.1.0's release notes. :func:`section` is therefore checked
by the tests for what it returns, not only for whether it returns anything.

Format expected of CHANGELOG.md, following Keep a Changelog:

    ## [0.1.1] - 2026-09-18      <- the heading, version in square brackets
    ...the notes...
    ## [0.1.0] - 2026-09-18      <- ends at the next version heading
    ...
    [0.1.1]: https://...         <- or at the link-reference block
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CHANGELOG = REPO / "CHANGELOG.md"
VERSION_FILE = REPO / "src" / "urdf_usd_bridge" / "_version.py"

#: A version heading. The version sits in square brackets; the date and its
#: separator are free-form, because an em dash and a hyphen both read fine and
#: neither should fail a release.
HEADING = re.compile(r"^## \[(?P<version>[^\]]+)\]")
#: A Markdown link-reference definition, e.g. ``[0.1.1]: https://...``. These
#: live at the end of the file and render as nothing, so they end a section.
LINK_REF = re.compile(r"^\[[^\]]+\]: ")


class MissingSectionError(LookupError):
    """CHANGELOG.md has no section for the requested version."""


def packaged_version() -> str:
    """``__version__`` as the built package will report it."""
    match = re.search(r'^__version__ = "(.+)"$', VERSION_FILE.read_text(), re.M)
    if match is None:
        raise LookupError(f"no __version__ assignment in {VERSION_FILE}")
    return match.group(1)


def versions(text: str) -> list[str]:
    """Every version that has a section, in file order."""
    return [m.group("version") for m in (HEADING.match(line) for line in text.splitlines()) if m]


def section(version: str, text: str) -> str:
    """The body of ``version``'s section, without its heading.

    Raises :class:`MissingSectionError` if there is none, naming what *is* there --
    a release is not the moment to go looking for the spelling yourself.
    """
    lines = text.splitlines()
    start = None
    for i, line in enumerate(lines):
        match = HEADING.match(line)
        if match and match.group("version") == version:
            start = i + 1
            break

    if start is None:
        found = ", ".join(versions(text)) or "none"
        raise MissingSectionError(
            f"CHANGELOG.md has no section for {version}.\n"
            f"  expected a heading of the form: ## [{version}] - <date>\n"
            f"  sections found: {found}"
        )

    body = []
    for line in lines[start:]:
        if HEADING.match(line) or LINK_REF.match(line):
            break
        body.append(line)

    return "\n".join(body).strip("\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "version",
        nargs="?",
        help="version to extract, e.g. 0.1.1 or v0.1.1. Defaults to the packaged __version__.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate only: print nothing on success",
    )
    args = parser.parse_args(argv)

    version = (args.version or packaged_version()).lstrip("v")

    try:
        body = section(version, CHANGELOG.read_text())
    except MissingSectionError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if not body.strip():
        print(f"error: CHANGELOG.md section for {version} is empty", file=sys.stderr)
        return 1

    if not args.check:
        print(body)
    return 0


if __name__ == "__main__":
    sys.exit(main())
