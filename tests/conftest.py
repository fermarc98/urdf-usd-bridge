# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared pytest fixtures and environment probes."""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures"
REFERENCES = REPO_ROOT / "references"
ARTIFACTS = REPO_ROOT / "tests" / "_artifacts"


def have_converter() -> bool:
    try:
        import urdf_usd_converter  # noqa: F401
    except ImportError:
        return False
    return True


def have_git_references() -> bool:
    return (
        shutil.which("git") is not None
        and (REFERENCES / "urdf-usd-converter" / ".git").exists()
        and (REFERENCES / "IsaacSim" / ".git").exists()
    )


@pytest.fixture(scope="session")
def fixtures_dir() -> Path:
    return FIXTURES


@pytest.fixture(scope="session")
def references_dir() -> Path:
    if not have_git_references():
        pytest.skip("references/ checkouts are not present")
    return REFERENCES


@pytest.fixture(scope="session")
def artifacts_dir() -> Path:
    return ARTIFACTS
