# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The stability layer: repairs authored as ``over`` prims, never in the input.

Scope in Phase 3 is exactly four gaps from ``docs/history/ANALYSIS.md``: G1 (drives),
G3 (armature), G2 (inertia) and G7 (joint limits). Collision filtering, physics
materials and scene defaults are deliberately not here.
"""

from .base import RULES, RULESET_VERSION, RepairOptions, RepairRecord, resolve_rules
from .run import REPORT_SCHEMA_VERSION, analyse, fix_asset, resolve_backends

__all__ = [
    "REPORT_SCHEMA_VERSION",
    "RULES",
    "RULESET_VERSION",
    "RepairOptions",
    "RepairRecord",
    "analyse",
    "fix_asset",
    "resolve_backends",
    "resolve_rules",
]
