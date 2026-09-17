# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Console entry point for ``urdf-usd-bridge``."""

from __future__ import annotations

import sys


def main(argv: list[str] | None = None) -> int:
    from .cli import run

    return run(argv if argv is not None else sys.argv[1:])


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
