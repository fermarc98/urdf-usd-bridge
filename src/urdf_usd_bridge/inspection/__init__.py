# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Read-only inspection of a converted robot asset.

This package never authors, mutates, or saves anything.
"""

from .run import inspect_stage
from .schemas import SCHEMA_VERSION

__all__ = ["SCHEMA_VERSION", "inspect_stage"]
