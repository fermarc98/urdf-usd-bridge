# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The upstream issue document stays consistent with its filing table.

These reports are the project's public claims about other people's software, so
they get the same treatment as any other claim: a test that fails when the
document and the status table drift apart.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
DOC = REPO / "docs" / "UPSTREAM_ISSUES.md"


@pytest.fixture(scope="module")
def text() -> str:
    return DOC.read_text()


def _issue_numbers(text: str) -> list[int]:
    return [int(n) for n in re.findall(r"^## Issue (\d+) — ", text, flags=re.M)]


def _table_rows(text: str) -> list[tuple[str, str, str]]:
    section = text.split("## Filing status", 1)[1].split("##", 1)[0]
    rows = []
    for line in section.splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) == 5 and cells[0].isdigit():
            rows.append((cells[0], cells[3], cells[4]))
    return rows


def test_every_issue_appears_in_the_filing_table(text):
    numbers = _issue_numbers(text)
    assert numbers, "no issues found in the document"
    listed = [row[0] for row in _table_rows(text)]
    assert listed == [str(n) for n in numbers]


def test_the_filing_status_is_either_not_filed_or_a_url(text):
    """A row cannot claim to be filed without a URL to show for it."""
    for number, status, url in _table_rows(text):
        if "not filed" in status.lower():
            assert url in ("—", "-", ""), f"issue {number} says not filed but has a URL"
        else:
            assert url.startswith("http"), f"issue {number} claims {status!r} with no URL"


def test_the_splitter_produces_one_body_per_issue(text):
    """The filing script and the document agree on where an issue ends."""
    import sys

    sys.path.insert(0, str(REPO / "scripts"))
    from file_upstream_issues import TARGETS, split_issues

    issues = split_issues(text)
    assert [n for n, _, _ in issues] == _issue_numbers(text)
    for number, title, body in issues:
        assert number in TARGETS, f"issue {number} has no target repository"
        assert title
        # A body that lost its reproduction is not filable.
        assert "### Reproduce" in body or "### Reproduce" in body.replace("###", "###")
        assert len(body.splitlines()) > 30


def test_unreproduced_claims_are_labelled_as_such(text):
    """Issue 2b was never observed running, and must keep saying so."""
    assert "source reading only" in text.lower()
    assert "NOT reproduced" in text or "not reproduced" in text
