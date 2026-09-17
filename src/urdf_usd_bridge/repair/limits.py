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
    INFORMATION,
    MEDIUM,
    NEUTRAL,
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
        if not ctx.options.is_enabled("limits.report-ambiguous"):
            return []
        return [
            RepairRecord(
                rule="limits.report-ambiguous",
                status=REPORTED,
                prim=path,
                attribute="physics:lowerLimit",
                old=[float(lower), float(upper)],
                old_state=AUTHORED,
                reason=(
                    "joint is locked at [0, 0], but a <limit> element demonstrably existed, so "
                    "this may be intentional. Refusing to guess; pass --force to unlock anyway"
                ),
                severity=WARNING,
                backend=NEUTRAL,
                evidence={
                    "limit_evidence": {k: v for k, v in evidence.items() if v is not None},
                    "readings": [
                        "the URDF author wrote lower=0 upper=0 deliberately",
                        "the URDF had <limit effort=... velocity=...> with no lower/upper",
                    ],
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

    reason = (
        "joint is welded at [0, 0] and carries no evidence that <limit> ever existed "
        "(no urdf:limit:effort, no newton:velocityLimit), which means the URDF omitted it. "
        "A revolute joint with no <limit> is malformed URDF; 'continuous' is the nearest "
        "well-defined reading, and unlimited is strictly closer to any plausible intent than welded"
    )
    for attribute, value in (("physics:lowerLimit", -math.inf), ("physics:upperLimit", math.inf)):
        writes.append(
            PlannedWrite(
                prim=path,
                backend=NEUTRAL,
                attribute=attribute,
                value=value,
                type_name=Sdf.ValueTypeNames.Float,
            )
        )
    return [
        RepairRecord(
            rule=rule,
            status=APPLIED,
            prim=path,
            attribute="physics:lowerLimit",
            old=float(lower),
            old_state=AUTHORED,
            new="-inf",
            units="degrees" if type_name == _ANGULAR_JOINT else "stage linear units",
            backend=NEUTRAL,
            layer=layer,
            reason=reason,
            confidence=MEDIUM,
            evidence={"limit_evidence": "none", "joint_type": type_name},
        ),
        RepairRecord(
            rule=rule,
            status=APPLIED,
            prim=path,
            attribute="physics:upperLimit",
            old=float(upper),
            old_state=AUTHORED,
            new="+inf",
            units="degrees" if type_name == _ANGULAR_JOINT else "stage linear units",
            backend=NEUTRAL,
            layer=layer,
            reason=reason,
            confidence=MEDIUM,
            evidence={"limit_evidence": "none", "joint_type": type_name},
        ),
    ]


def _compliance_report(path: str, prim, lower, upper) -> RepairRecord:
    """Limit compliance is diagnosed, never invented. See the design doc."""
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
            "joint has finite limits but no limit compliance, so PhysX will treat the stop as "
            "rigid and MuJoCo will derive a soft one from solreflimit -- the same asset bounces "
            "in one backend and sticks in the other. Not repaired: deriving a limit stiffness "
            "needs the effective inertia at qpos0, which would bake a pose-dependent number into "
            "a static attribute (mujoco-usd-converter joint.py:99-104 declines it for the same "
            "reason). Phase 4 limit-sweep territory"
        ),
        severity=INFORMATION,
        backend=NEUTRAL,
        evidence={"limits": [float(lower), float(upper)]},
    )
