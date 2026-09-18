# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Cross-backend simulation harness.

``metrics`` is pure and testable anywhere; everything else needs a simulator.
See ``docs/PHASE4_DESIGN.md`` for the scene contract and metric definitions.
"""

from . import metrics, scene

__all__ = ["metrics", "scene"]
