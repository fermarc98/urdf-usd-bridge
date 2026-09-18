# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Fail if a Markdown link, or a path named in prose, does not resolve.

Two kinds of reference rot are possible in this repository and both have
happened:

1. A broken Markdown link -- one whose target file has moved or been renamed.
   GitHub renders these as dead links.
2. A stale path in *prose or a docstring* -- ``see `docs/<name>.md` `` inside a
   backticked span, or in a Python docstring. Nothing renders these, so nothing
   catches them; they simply mislead a reader who tries to follow them.

The second kind is why this checks source files too, not only Markdown.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

SKIP_DIRS = {
    ".git",
    ".venv",
    "venv",
    "references",
    "build",
    "dist",
    "_artifacts",
    "__pycache__",
    ".pytest_cache",
    ".ruff_cache",
    "node_modules",
    ".claude",
}

# A path that looks like one of ours: docs/..., examples/..., src/..., a
# top-level Markdown file. Anchored so "foo/docs/x.md" in someone else's tree
# is not mistaken for ours.
PROSE_PATH = re.compile(
    r"(?<![\w./-])((?:docs|examples|scripts|src|tests)/[\w./-]+?\.(?:md|py|usda|urdf)"
    r"|(?:README|CHANGELOG|CONTRIBUTING|THIRD_PARTY|LICENSE|NOTICE|CITATION)\.(?:md|cff))"
)
MD_LINK = re.compile(r"\]\(([^)\s]+)")

# Paths that live inside a *reference* repository, not this one. They appear in
# docs/history/ANALYSIS.md, which cites upstream files by their own repo-root
# path. They are correct as written and must not be "fixed" to point here.
EXTERNAL = {
    "docs/concept_mapping.md",  # newton-physics/mujoco-usd-converter
    "docs/CHANGELOG.md",  # newton-physics/urdf-usd-converter
    "tests/test_urdf_to_mjc_physx_conversion.py",  # isaac-sim/IsaacSim
    "LICENSE.md",  # newton-physics/urdf-usd-converter
}

# Directories whose contents are *generated*, gitignored, and therefore absent
# from a fresh checkout. A path under one of these is not stale when it is
# missing -- it just has not been produced yet.
#
# This is the exact failure that broke CI once: examples/01_inspect_report.py
# names the asset `scripts/run_converter_matrix.py` writes, the check passed on
# a developer machine that had run the matrix, and failed on a clean runner
# that had not. A check whose result depends on what you happen to have built
# locally is worse than no check.
GENERATED = (
    "tests/_artifacts/",
    "sim_artifacts/",
    "dist/",
    "build/",
)


def walk() -> list[Path]:
    out = []
    for path in REPO.rglob("*"):
        if not path.is_file():
            continue
        if any(part in SKIP_DIRS for part in path.relative_to(REPO).parts):
            continue
        if path.suffix in {".md", ".py", ".toml", ".cff", ".yml", ".yaml"}:
            out.append(path)
    return sorted(out)


def _is_generated(candidate: Path) -> bool:
    """True if ``candidate`` lives under a generated, gitignored directory."""
    try:
        rel = candidate.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return False
    return rel.startswith(GENERATED)


def check(path: Path, text: str) -> list[str]:
    bad = []
    rel = path.relative_to(REPO)

    for m in MD_LINK.finditer(text):
        target = m.group(1).split("#")[0].strip()
        if not target or target.startswith(("http://", "https://", "mailto:", "<")):
            continue
        resolved = path.parent / target
        if _is_generated(resolved):
            continue
        if not resolved.exists():
            bad.append(f"{rel}: broken link -> {target}")

    # Prose paths are always written from the repository root.
    for m in PROSE_PATH.finditer(text):
        target = m.group(1)
        if target in EXTERNAL or target.startswith(GENERATED):
            continue
        if not (REPO / target).exists():
            bad.append(f"{rel}: stale path -> {target}")

    return bad


def main() -> int:
    problems = []
    files = walk()
    for path in files:
        problems += check(path, path.read_text(encoding="utf-8", errors="replace"))
    for line in problems:
        print(line)
    print(f"\n{len(files)} files checked, {len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
