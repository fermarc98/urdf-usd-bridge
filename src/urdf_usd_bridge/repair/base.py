# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Repair vocabulary: options, records, and the authored-value policy.

Two rules from ``docs/ANALYSIS.md`` §5 are enforced here rather than left to
each repair module:

1. **Never silently change dynamics.** Every rule evaluation produces a record,
   including the ones that decided to do nothing, and every record carries the
   old value, the new value, the rule that fired and why.
2. **Never overwrite what the user authored.** :func:`should_write` is the one
   place that decides, so no rule can accidentally disagree with the policy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

#: Version of the rule set. Bumped when a rule's behaviour changes, so a report
#: can be compared against another report and a golden layer can pin behaviour.
RULESET_VERSION = "1"

#: Version of the repair-report JSON schema.
REPORT_SCHEMA_VERSION = 1

# --- statuses --------------------------------------------------------------

APPLIED = "applied"
SKIPPED = "skipped"
REPORTED = "reported"
REFUSED = "refused"

# --- confidence ------------------------------------------------------------

#: The repaired value is the only defensible one (identity for an absent rotation).
HIGH = "high"
#: Derived from a documented model with a stated assumption (gains from f_n, zeta).
MEDIUM = "medium"
#: A judgement call on genuinely ambiguous input. Report-only unless opted in.
LOW = "low"

# --- severity, for report-only records -------------------------------------

INFORMATION = "information"
WARNING = "warning"
ERROR = "error"

# --- the three-way authored state, matching `inspect` ----------------------

ABSENT = "absent"
FALLBACK = "fallback"
AUTHORED = "authored"

# --- backends --------------------------------------------------------------

NEUTRAL = "neutral"
PHYSX = "physx"
MUJOCO = "mujoco"
NEWTON = "newton"

#: Backends that can be selected on the command line.
SELECTABLE_BACKENDS = (PHYSX, MUJOCO, NEWTON)
ALL_BACKENDS = "all"


@dataclass(frozen=True)
class Defaults:
    """Tuning defaults, every one of them **unmeasured**.

    ``docs/PHASE3_DESIGN.md`` §10 lists the Phase 4 experiments that have to
    justify or move these. Until then every report says so in its header, and
    the values are recorded as custom metadata on the authored layer so an
    asset carries the assumptions it was built with.
    """

    #: Target closed-loop natural frequency for derived drive gains, in Hz.
    target_frequency: float = 10.0
    #: Target damping ratio. 1.0 is critical damping: no overshoot.
    damping_ratio: float = 1.0
    #: Armature as a fraction of the joint's own equivalent inertia.
    armature_fraction: float = 0.01
    #: Armature floor as a fraction of the articulation's largest equivalent inertia.
    armature_floor: float = 1e-4
    #: Assumed control/simulation rate, in Hz. Checked, never used in a formula.
    control_rate: float = 60.0


DEFAULTS = Defaults()

#: Every rule id, with whether it is on by default.
RULES: dict[str, bool] = {
    "inertia.principal-axes-identity": True,
    "inertia.derive-from-geometry": True,
    "inertia.make-physical": True,
    "inertia.mass-floor": False,
    "limits.restore-missing": True,
    "limits.restore-missing-prismatic": False,
    "limits.report-ambiguous": True,
    "limits.compliance": True,
    "armature.default": True,
    "drives.mirror-passive": True,
    "drives.derive-gains": True,
    "drives.no-drive-for-fixed": True,
}


class OptionError(ValueError):
    """An option combination that cannot be honoured."""


@dataclass
class RepairOptions:
    """Everything that changes what ``fix`` does, resolved in one place."""

    backends: tuple[str, ...] = SELECTABLE_BACKENDS
    backends_requested: str = ALL_BACKENDS
    enabled: dict[str, bool] = field(default_factory=lambda: dict(RULES))
    target_frequency: float = DEFAULTS.target_frequency
    damping_ratio: float = DEFAULTS.damping_ratio
    armature_fraction: float = DEFAULTS.armature_fraction
    armature_floor: float = DEFAULTS.armature_floor
    control_rate: float = DEFAULTS.control_rate
    force: bool = False
    dry_run: bool = False
    variant_selections: dict[str, str] = field(default_factory=dict)

    def is_enabled(self, rule: str) -> bool:
        return self.enabled.get(rule, False)

    def tuning(self) -> dict[str, float]:
        """The tuning constants, for the report header and layer metadata."""
        return {
            "target_frequency_hz": self.target_frequency,
            "damping_ratio": self.damping_ratio,
            "armature_fraction": self.armature_fraction,
            "armature_floor": self.armature_floor,
            "control_rate_hz": self.control_rate,
        }


def resolve_rules(enable: list[str] | None, disable: list[str] | None) -> dict[str, bool]:
    """Apply ``--enable`` / ``--disable`` to the defaults.

    Raises:
        OptionError: if a rule name is not recognised, because a silently
            ignored ``--disable`` would be worse than a hard failure.
    """
    resolved = dict(RULES)
    for name in enable or []:
        if name not in resolved:
            raise OptionError(f"unknown rule {name!r}; known rules: {', '.join(sorted(resolved))}")
        resolved[name] = True
    for name in disable or []:
        if name not in resolved:
            raise OptionError(f"unknown rule {name!r}; known rules: {', '.join(sorted(resolved))}")
        resolved[name] = False
    return resolved


@dataclass
class RepairRecord:
    """One rule evaluation. Applied, skipped, reported or refused."""

    rule: str
    status: str
    prim: str
    attribute: str | None = None
    old: Any = None
    old_state: str = ABSENT
    new: Any = None
    units: str | None = None
    backend: str = NEUTRAL
    layer: str | None = None
    reason: str = ""
    confidence: str | None = None
    severity: str | None = None
    evidence: dict[str, Any] = field(default_factory=dict)
    forced: bool = False

    def as_dict(self) -> dict[str, Any]:
        out = {
            "rule": self.rule,
            "status": self.status,
            "prim": self.prim,
            "attribute": self.attribute,
            "old": self.old,
            "old_state": self.old_state,
            "new": self.new,
            "units": self.units,
            "backend": self.backend,
            "layer": self.layer,
            "reason": self.reason,
            "confidence": self.confidence,
            "forced": self.forced,
        }
        if self.severity:
            out["severity"] = self.severity
        if self.evidence:
            out["evidence"] = self.evidence
        return out


def attribute_state(prim, name: str) -> str:
    """Classify an attribute as absent, an unauthored fallback, or authored."""
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        return ABSENT
    return AUTHORED if attr.HasAuthoredValue() else FALLBACK


def should_write(state: str, *, valid: bool, force: bool) -> bool:
    """The authored-value policy, in one place.

    Writes when the value is absent, when it is only a schema fallback, or when
    it was authored but fails validation. Refuses to touch an authored value
    that validates, unless forced.
    """
    if force:
        return True
    if state in (ABSENT, FALLBACK):
        return True
    return not valid
