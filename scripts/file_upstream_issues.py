#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Split ``docs/UPSTREAM_ISSUES.md`` into filable issue bodies, and file them.

    python scripts/file_upstream_issues.py            # write the bodies, print the commands
    python scripts/file_upstream_issues.py --file     # actually file them, needs `gh`

Filing posts to a public tracker under your GitHub account, so ``--file`` is
opt-in and the script prints exactly what it will post first. Without it, the
bodies land in ``build/upstream/`` and the ``gh`` commands are printed for you
to run or paste.

Record the resulting URLs in the Filing status table at the top of
``docs/UPSTREAM_ISSUES.md``; ``tests/unit/test_upstream_issues.py`` checks the
table stays in step with the issues.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SOURCE = REPO / "docs" / "UPSTREAM_ISSUES.md"
OUT = REPO / "build" / "upstream"

#: Which repository each issue belongs to, by issue number in the document.
TARGETS = {
    1: "isaac-sim/IsaacSim",
    2: "isaac-sim/IsaacSim",
    3: "newton-physics/newton",
}


def split_issues(text: str) -> list[tuple[int, str, str]]:
    """``(number, title, body)`` for each ``## Issue N — ...`` section."""
    parts = re.split(r"^## Issue (\d+) — (.+)$", text, flags=re.M)
    issues = []
    for index in range(1, len(parts), 3):
        number = int(parts[index])
        title = parts[index + 1].strip()
        body = parts[index + 2].split("\n---\n")[0].strip()
        issues.append((number, title, body))
    return issues


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--file", action="store_true", help="actually create the issues (needs gh)")
    parser.add_argument("--only", type=int, action="append", help="restrict to these issue numbers")
    args = parser.parse_args(argv)

    if not SOURCE.exists():
        print(f"missing {SOURCE}", file=sys.stderr)
        return 2
    issues = split_issues(SOURCE.read_text())
    if args.only:
        issues = [i for i in issues if i[0] in set(args.only)]
    if not issues:
        print("no issues found", file=sys.stderr)
        return 2

    OUT.mkdir(parents=True, exist_ok=True)
    preamble = (
        "_Reported by the `urdf-usd-bridge` project. Every claim below is either "
        "reproduced on the stated build or explicitly marked as a source reading "
        "that was not observed running._\n\n"
    )

    for number, title, body in issues:
        target = TARGETS.get(number, "UNKNOWN/REPO")
        path = OUT / f"issue-{number}.md"
        path.write_text(preamble + body + "\n")
        print(f"issue {number} -> {target}")
        print(f"  title: {title}")
        print(f"  body:  {path}  ({len(body.splitlines())} lines)")
        print(f"  gh issue create --repo {target} \\")
        print(f"      --title {title!r} \\")
        print(f"      --body-file {path}")
        print()

    if not args.file:
        print("nothing was filed. Re-run with --file to create them, or paste the bodies by hand.")
        return 0

    if shutil.which("gh") is None:
        print(
            "gh is not installed, so this script cannot file anything.\n"
            "Install it (https://cli.github.com), run `gh auth login`, and re-run with --file,\n"
            "or create the issues by hand from the bodies above.",
            file=sys.stderr,
        )
        return 2

    failed = 0
    for number, title, _ in issues:
        target = TARGETS.get(number)
        path = OUT / f"issue-{number}.md"
        result = subprocess.run(
            ["gh", "issue", "create", "--repo", target, "--title", title, "--body-file", str(path)],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print(f"issue {number}: FAILED\n{result.stderr.strip()}", file=sys.stderr)
            failed += 1
            continue
        url = result.stdout.strip().splitlines()[-1]
        print(f"issue {number}: {url}")
        print(f"  -> record this in the Filing status table in {SOURCE.relative_to(REPO)}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
