# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Joint-limit repairs -- ``docs/ANALYSIS.md`` G7.

``urdf-usd-converter`` turns a revolute joint whose ``<limit>`` is missing into
a joint with ``lower == upper == 0`` -- welded shut, with no warning
(``link.py:392-393``). Phase 2.5 confirmed this at runtime on fixture (c), on
0.3.2 and 0.3.3 alike.

Detecting ``[0, 0]`` is easy. Deciding whether it was *meant* is the actual
problem, because a URDF may legitimately say ``lower="0" upper="0"``. The asset
carries enough evidence to tell the two apart:

==========================================  =====================================
Evidence on a ``[0, 0]`` joint              Reading
==========================================  =====================================
no ``urdf:limit:effort``, no                ``<limit>`` was absent entirely --
``newton:velocityLimit``                    URDF requires ``effort`` and
                                            ``velocity`` on it, so neither
                                            could have been recorded
either one present                          ``<limit>`` existed and
                                            ``lower``/``upper`` were omitted or
                                            genuinely zero -- ambiguous
==========================================  =====================================

Fixture (c) shows both cases side by side: ``no_limit_joint`` has neither
attribute, ``partial_limit_joint`` has ``effort = 30.0`` and
``velocityLimit = 85.94``, and both came out ``[0, 0]``.
"""

from __future__ import annotations

import math

from pxr import Sdf

from .base import (
    APPLIED,
    AUTHORED,
    ERROR,
    LOW,
    MEDIUM,
    NEUTRAL,
    REFUSED,
    REPORTED,
    SKIPPED,
    WARNING,
    RepairRecord,
)
from .layer import PlannedWrite

_ANGULAR_JOINT = "PhysicsRevoluteJoint"
_LINEAR_JOINT = "PhysicsPrismaticJoint"

#: The attributes whose presence proves a `<limit>` element existed.
_LIMIT_EVIDENCE = ("urdf:limit:effort", "newton:velocityLimit")


def _authored(prim, name: str):
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid() or not attr.HasAuthoredValue():
        return None
    return attr.Get()


def _is_locked(lower, upper) -> bool:
    return lower is not None and upper is not None and abs(float(lower)) < 1e-12 and abs(float(upper)) < 1e-12


def apply_limit_rules(ctx) -> tuple[list[RepairRecord], list[PlannedWrite]]:
    """Run every joint-limit rule over every revolute and prismatic joint."""
    records: list[RepairRecord] = []
    writes: list[PlannedWrite] = []
    layer = ctx.layer_names[NEUTRAL]

    for joint in ctx.articulation.joints:
        prim = ctx.stage.GetPrimAtPath(joint.path)
        if not prim or not prim.IsValid():  # pragma: no cover - defensive
            continue
        type_name = str(prim.GetTypeName())
        if type_name not in (_ANGULAR_JOINT, _LINEAR_JOINT):
            continue

        lower = _authored(prim, "physics:lowerLimit")
        upper = _authored(prim, "physics:upperLimit")
        evidence = {name: _authored(prim, name) for name in _LIMIT_EVIDENCE}
        has_evidence = any(value is not None for value in evidence.values())

        if _is_locked(lower, upper):
            records.extend(
                _locked_joint(ctx, joint, type_name, lower, upper, evidence, has_evidence, layer, writes)
            )
            continue

        finite = (
            lower is not None
            and upper is not None
            and math.isfinite(float(lower))
            and math.isfinite(float(upper))
        )
        if finite and ctx.options.is_enabled("limits.compliance"):
            records.append(_compliance_report(joint.path, prim, lower, upper))

    return records, writes


def _locked_joint(
    ctx, joint, type_name, lower, upper, evidence, has_evidence, layer, writes
) -> list[RepairRecord]:
    path = joint.path
    if has_evidence:
        if ctx.options.force_unlock:
            records = _unlock(ctx, joint, type_name, lower, upper, layer, writes, forced=True)
            return records
        if not ctx.options.is_enabled("limits.report-ambiguous"):
            return []
        return [
            RepairRecord(
                rule="limits.report-ambiguous",
                status=REFUSED,
                prim=path,
                attribute="physics:lowerLimit",
                old=[float(lower), float(upper)],
                old_state=AUTHORED,
                reason=(
                    "joint is locked at [0, 0] and a <limit> element demonstrably existed, so the "
                    "value is genuinely ambiguous and this rule will not guess it. "
                    "**MuJoCo refuses to compile an asset containing a [0, 0] joint** "
                    '("range[0] should be smaller than range[1]"), so the asset is unusable '
                    "there until this is resolved. Either fix the <limit> in the URDF, or pass "
                    "--force-unlock to unlock every ambiguous joint and accept that the range is "
                    "a guess"
                ),
                severity=ERROR,
                backend=NEUTRAL,
                evidence={
                    "limit_evidence": {k: v for k, v in evidence.items() if v is not None},
                    "readings": [
                        "the URDF author wrote lower=0 upper=0 deliberately",
                        "the URDF had <limit effort=... velocity=...> with no lower/upper",
                    ],
                    "mujoco_compiles": False,
                    "resolve_with": "--force-unlock",
                },
            )
        ]

    is_angular = type_name == _ANGULAR_JOINT
    rule = "limits.restore-missing" if is_angular else "limits.restore-missing-prismatic"
    if not ctx.options.is_enabled(rule):
        return [
            RepairRecord(
                rule=rule,
                status=REPORTED,
                prim=path,
                attribute="physics:lowerLimit",
                old=[float(lower), float(upper)],
                old_state=AUTHORED,
                reason=(
                    "joint is welded at [0, 0] because <limit> was missing entirely. "
                    f"Repair is off by default for this joint type; --enable {rule} restores it"
                    if not is_angular
                    else "joint is welded at [0, 0] because <limit> was missing; rule disabled"
                ),
                severity=WARNING,
                confidence=MEDIUM,
                backend=NEUTRAL,
                evidence={"limit_evidence": "none"},
            )
        ]

    return _unlock(ctx, joint, type_name, lower, upper, layer, writes, forced=False)


def _unlock(ctx, joint, type_name, lower, upper, layer, writes, *, forced: bool):
    """Author an unlimited range on a welded joint."""
    is_angular = type_name == _ANGULAR_JOINT
    rule = (
        "limits.report-ambiguous"
        if forced
        else ("limits.restore-missing" if is_angular else "limits.restore-missing-prismatic")
    )
    reason = (
        (
            "unlocked under --force-unlock although a <limit> element existed, so the range is a "
            "guess. Done because MuJoCo will not compile an asset containing a [0, 0] joint"
        )
        if forced
        else (
            "joint is welded at [0, 0] and carries no evidence that <limit> ever existed "
            "(no urdf:limit:effort, no newton:velocityLimit), which means the URDF omitted it. "
            "A revolute joint with no <limit> is malformed URDF; 'continuous' is the nearest "
            "well-defined reading, and unlimited is strictly closer to any plausible intent "
            "than welded"
        )
    )
    for attribute, value in (("physics:lowerLimit", -math.inf), ("physics:upperLimit", math.inf)):
        writes.append(
            PlannedWrite(
                prim=joint.path,
                backend=NEUTRAL,
                attribute=attribute,
                value=value,
                type_name=Sdf.ValueTypeNames.Float,
            )
        )
    units = "degrees" if is_angular else "stage linear units"
    return [
        RepairRecord(
            rule=rule,
            status=APPLIED,
            prim=joint.path,
            attribute=attribute,
            old=float(old_value),
            old_state=AUTHORED,
            new=new_value,
            units=units,
            backend=NEUTRAL,
            layer=layer,
            reason=reason,
            confidence=LOW if forced else MEDIUM,
            evidence={
                "limit_evidence": "present" if forced else "none",
                "joint_type": type_name,
                "forced": forced,
            },
            forced=forced,
        )
        for attribute, old_value, new_value in (
            ("physics:lowerLimit", lower, "-inf"),
            ("physics:upperLimit", upper, "+inf"),
        )
    ]


def _compliance_report(path: str, prim, lower, upper) -> RepairRecord:
    """Limit compliance: diagnosed, and as of Phase 5 still not invented.

    Phase 4 measured the case for writing it -- driven joints overshoot their
    stops harder than undriven ones, in 6 of 20 cells. But the value still
    cannot be derived from the asset: ``newton:limitStiffness`` is an effort per
    unit penetration, and converting MuJoCo's ``solreflimit`` to it needs the
    effective inertia at ``qpos0``, which is a pose-dependent quantity that must
    not be baked into a static attribute. ``mujoco-usd-converter``
    (``joint.py:99-104``) declines it for the same reason, and they are right.

    What Phase 5 adds is the measured evidence in the record, so the reader sees
    the size of the problem rather than only the refusal.
    """
    stiffness = _authored(prim, "newton:limitStiffness")
    damping = _authored(prim, "newton:limitDamping")
    if stiffness is not None or damping is not None:
        return RepairRecord(
            rule="limits.compliance",
            status=SKIPPED,
            prim=path,
            attribute="newton:limitStiffness",
            old=stiffness,
            old_state=AUTHORED,
            reason="limit compliance is already authored",
            backend=NEUTRAL,
        )
    return RepairRecord(
        rule="limits.compliance",
        status=REPORTED,
        prim=path,
        attribute="newton:limitStiffness",
        old=None,
        reason=(
            "joint has finite limits and no limit compliance, so PhysX treats the stop as rigid "
            "while MuJoCo derives a soft one from solreflimit -- the same asset bounces in one "
            "backend and sticks in the other. Measured in Phase 4: a repaired, driven joint "
            "overshoots its stop harder than the undriven baseline in 6 of 20 cells (worst "
            "0.046 -> 0.328 rad). Still not repaired, because a limit stiffness cannot be "
            "derived from the asset alone: it needs the effective inertia at qpos0, which is "
            "pose-dependent and must not be baked into a static attribute "
            "(mujoco-usd-converter joint.py:99-104 declines it for the same reason)"
        ),
        severity=WARNING,
        backend=NEUTRAL,
        evidence={
            "limits": [float(lower), float(upper)],
            "phase4_overshoot_cells_worse": 6,
            "phase4_worst_overshoot_rad": 0.328,
            "why_not_derivable": "needs effective inertia at qpos0; pose-dependent",
        },
    )
