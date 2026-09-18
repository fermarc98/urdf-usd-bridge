# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Repair vocabulary: options, records, and the authored-value policy.

Two rules from ``docs/history/ANALYSIS.md`` §5 are enforced here rather than left to
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


#: Where each tuning constant's value comes from. Phase 3 shipped all five as
#: ``unmeasured`` engineering choices; Phase 4 measured them. A constant that
#: kept its Phase 3 value still says *why* it kept it.
#:
#: See ``docs/history/PHASE4_REPORT.md`` for the sweep behind each entry.
PROVENANCE: dict[str, str] = {
    "target_frequency": (
        "measured 2026-09-18: largest f_n where PhysX, Newton and MuJoCo all survive at the "
        "default 60 Hz control rate. Newton's Featherstone solver diverges at 10 Hz/60 Hz; "
        "PhysX and MuJoCo do not, so this value is set by the cross-backend guarantee"
    ),
    "damping_ratio": (
        "measured 2026-09-18: confirmed at the Phase 3 value. Minimises step settle time "
        "(0.175 s vs 0.263 s at 0.7 and 0.200 s at 1.4) with zero overshoot; 0.7 overshoots "
        "by 5.1%, past the 5% bound"
    ),
    "armature_fraction": (
        "measured 2026-09-18: **no measurable effect** on the stability margin. Sweeping alpha "
        "over 0, 0.01, 0.1 and 1.0 changed neither the divergence threshold nor the dt at which "
        "it occurs. Retained at the Phase 3 value; see docs/BENCHMARK.md"
    ),
    "armature_floor": (
        "unmeasured: the case it exists for -- a DOF whose own inertia is negligible -- is not "
        "represented in the tested corpus, so the sweep could not exercise it"
    ),
    "control_rate": (
        "measured 2026-09-18 as a *relation* rather than a value: all three backends are stable "
        "when f_n <= control_rate / 12 and Newton diverges at control_rate / 6. Phase 3 assumed "
        "control_rate / 4, which measurement contradicts. The 60 Hz assumption is unchanged"
    ),
}

#: What the constants were before Phase 4, and when they changed. Recorded in
#: every output layer so an asset built with the old values stays explicable
#: (decision N4).
PREVIOUS_DEFAULTS: dict[str, float] = {
    "target_frequency": 10.0,
    "damping_ratio": 1.0,
    "armature_fraction": 0.01,
    "armature_floor": 1e-4,
    "control_rate": 60.0,
}
DEFAULTS_CHANGED: str = "2026-09-18 (Phase 4): target_frequency 10.0 -> 5.0 Hz; others unchanged"


@dataclass(frozen=True)
class Defaults:
    """Tuning defaults, with the provenance of each recorded in :data:`PROVENANCE`.

    Phase 3 shipped these as documented engineering choices and said so in every
    report. Phase 4 ran the sweeps in ``docs/history/PHASE4_DESIGN.md`` section 8 and
    replaced or confirmed them; ``docs/history/PHASE4_REPORT.md`` carries the rows.
    """

    #: Target closed-loop natural frequency for derived drive gains, in Hz.
    #: Measured: 10 Hz diverges in Newton at a 60 Hz control rate.
    target_frequency: float = 5.0
    #: Target damping ratio. 1.0 is critical damping: no overshoot.
    damping_ratio: float = 1.0
    #: Armature as a fraction of the joint's own equivalent inertia.
    armature_fraction: float = 0.01
    #: Armature floor as a fraction of the articulation's largest equivalent inertia.
    armature_floor: float = 1e-4
    #: Assumed control/simulation rate, in Hz. Checked, never used in a formula.
    control_rate: float = 60.0


#: Measured ratio for a **cross-backend** asset: every backend survived
#: ``f_n <= control_rate / 12``. Phase 3 assumed 4, which the dt sweep
#: contradicts. See ``docs/history/PHASE4_REPORT.md`` section 5.1.
STABLE_RATE_RATIO = 12.0

#: Per-backend divisors, measured 2026-09-18 in the dt sweep.
#:
#: PhysX and MuJoCo were stable at ``control_rate / 6`` in every cell; Newton's
#: Featherstone solver diverged there and needed ``/12``. So an asset targeting
#: one backend can be driven twice as stiffly as one that has to work in all
#: three -- **except for Newton, where /6 is the value that diverged.**
#: Applying /6 to Newton would ship a default the measurement says fails.
BACKEND_RATE_DIVISOR: dict[str, float] = {
    PHYSX: 6.0,
    MUJOCO: 6.0,
    NEWTON: 12.0,
}


def default_target_frequency(backends, control_rate: float) -> tuple[float, str]:
    """``(f_n, basis)`` for a backend selection, from the measured divisors.

    One backend gets the stiffest gain that backend tolerated; a multi-backend
    asset gets the stiffest gain **all** of them tolerated, which is the weakest
    of the three. The basis string goes into the layer metadata so an asset says
    why its gains are what they are.
    """
    names = tuple(backends)
    if len(names) == 1 and names[0] in BACKEND_RATE_DIVISOR:
        divisor = BACKEND_RATE_DIVISOR[names[0]]
        note = " (Newton diverged at /6, so it keeps the cross-backend divisor)" if names[0] == NEWTON else ""
        return control_rate / divisor, (f"control_rate / {divisor:g}, measured for {names[0]} alone{note}")
    divisor = (
        max(BACKEND_RATE_DIVISOR.get(n, STABLE_RATE_RATIO) for n in names) if names else (STABLE_RATE_RATIO)
    )
    divisor = max(divisor, STABLE_RATE_RATIO)
    return control_rate / divisor, (
        f"control_rate / {divisor:g}, the weakest divisor among {', '.join(names) or 'all backends'}"
        " -- a cross-backend asset can only be as stiff as its least tolerant consumer"
    )


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
    #: ``None`` means "derive from the backend selection and control rate".
    target_frequency: float | None = None
    damping_ratio: float = DEFAULTS.damping_ratio
    armature_fraction: float = DEFAULTS.armature_fraction
    armature_floor: float = DEFAULTS.armature_floor
    control_rate: float = DEFAULTS.control_rate
    force: bool = False
    dry_run: bool = False
    #: Allow ``--backend all`` on an asset with no Physics variant set by
    #: writing one stabilized root per backend instead of refusing. ``convert``
    #: sets this, because everything it produces is variant-less; ``fix`` does
    #: not, so an asset handed to it directly still gets the refusal.
    multi_root: bool = False
    #: Unlock ``[0, 0]`` joints even when the asset shows a <limit> existed.
    #: Off by default: the value is genuinely ambiguous. On, because MuJoCo
    #: refuses to compile an asset that still contains one.
    force_unlock: bool = False
    #: Also author NewtonActuator + NewtonPDControlAPI on the Newton layer.
    #: Off by default: Newton 1.5.0 does not read it, and a later release that
    #: does would drive the joint twice alongside its UsdPhysics drive.
    newton_actuator: bool = False
    variant_selections: dict[str, str] = field(default_factory=dict)

    #: Filled in by :meth:`resolve_frequency` once the backends are known.
    target_frequency_basis: str = ""
    #: Cache for the derived value. Kept separate from ``target_frequency`` so
    #: resolving twice cannot make a derived value look user-supplied.
    _resolved_frequency: float | None = None

    def is_enabled(self, rule: str) -> bool:
        return self.enabled.get(rule, False)

    def resolve_frequency(self) -> float:
        """The target frequency to use, deriving it when none was given."""
        if self.target_frequency is not None:
            self.target_frequency_basis = "set explicitly on the command line"
            return float(self.target_frequency)
        if self._resolved_frequency is None:
            frequency, basis = default_target_frequency(self.backends, self.control_rate)
            self._resolved_frequency = frequency
            self.target_frequency_basis = basis
        return float(self._resolved_frequency)

    def tuning(self) -> dict[str, float]:
        """The tuning constants, for the report header and layer metadata."""
        return {
            "target_frequency_hz": self.resolve_frequency(),
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
