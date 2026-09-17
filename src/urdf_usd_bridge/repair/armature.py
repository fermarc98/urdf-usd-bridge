# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Armature / rotor inertia -- ``docs/ANALYSIS.md`` G3.

The URDF path authors no armature anywhere. Isaac Sim's ``convert_physx_to_mjc``
will copy ``physxJoint:armature`` into ``mjc:armature``, but nothing in the
import ever sets it, so it is always absent.

The rule::

    armature = max( alpha * I_eq ,  beta * I_eq_max )

Two terms, two jobs. ``alpha * I_eq`` is the scale-relative rotor term: a small
fraction of what the DOF already carries, so it does not change the robot's
gross behaviour. ``beta * I_eq_max`` is a **conditioning floor** relative to the
largest equivalent inertia in the same articulation, which bounds the ratio
between the largest and smallest joint-space mass-matrix diagonal at about
``1/beta``. The floor is what rescues a gripper fingertip, whose own 1% would be
1e-8 kg*m^2 and no regularisation at all.

Neither constant is measured. The *shape* is the principled part -- scale
relative, with a floor relative to the whole articulation, rather than one
absolute constant that is wrong for any robot that is not the size it was tuned
on. MuJoCo's own schema documentation is the closest thing to an authority:
*"positive armature significantly improves simulation stability, even for small
values, and is a recommended possible fix when encountering stability issues."*

Units: armature is **not** angle-scaled. ``newton:armature`` declares
``mass * distance * distance`` for angular DOFs, and ``mujoco-usd-converter``
writes the same number to ``mjc:armature`` and ``newton:armature``
(``joint.py:82,96``) while scaling damping between them. All three spellings get
the identical value.
"""

from __future__ import annotations

from pxr import Sdf

from ..model.units import ANGULAR, LINEAR
from .base import (
    ABSENT,
    APPLIED,
    MEDIUM,
    MUJOCO,
    NEUTRAL,
    NEWTON,
    PHYSX,
    SKIPPED,
    RepairRecord,
    attribute_state,
)
from .layer import PlannedWrite

#: Spelling per backend. The value is identical in all three.
ARMATURE_ATTRS = {
    NEUTRAL: ("newton:armature", Sdf.ValueTypeNames.Float, "NewtonJointAPI", False),
    NEWTON: ("newton:armature", Sdf.ValueTypeNames.Float, "NewtonJointAPI", False),
    PHYSX: ("physxJoint:armature", Sdf.ValueTypeNames.Float, "PhysxJointAPI", False),
    MUJOCO: ("mjc:armature", Sdf.ValueTypeNames.Double, "MjcJointAPI", True),
}

#: Any authored armature in any namespace means we leave the joint alone.
_ANY_ARMATURE = ("newton:armature", "physxJoint:armature", "mjc:armature")


def _existing_armature(prim) -> tuple[str, float] | None:
    for name in _ANY_ARMATURE:
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid() and attr.HasAuthoredValue():
            value = attr.Get()
            if value:
                return name, float(value)
    return None


def apply_armature_rules(ctx) -> tuple[list[RepairRecord], list[PlannedWrite]]:
    """Choose an armature per joint and author it in every namespace.

    Also records the chosen value on ``ctx.armature`` so ``drives.derive-gains``
    uses the same ``I_total`` the armature was computed against.
    """
    records: list[RepairRecord] = []
    writes: list[PlannedWrite] = []
    options = ctx.options

    angular_max = 0.0
    linear_max = 0.0
    inertias: dict[str, tuple[float, dict]] = {}
    for joint in ctx.articulation.joints:
        if joint.dof is None:
            continue
        value, evidence = ctx.articulation.equivalent_inertia(joint)
        inertias[joint.path] = (value, evidence)
        if joint.dof == ANGULAR:
            angular_max = max(angular_max, value)
        else:
            linear_max = max(linear_max, value)

    for joint in ctx.articulation.joints:
        if joint.dof is None:
            continue
        prim = ctx.stage.GetPrimAtPath(joint.path)
        if not prim or not prim.IsValid():  # pragma: no cover - defensive
            continue

        i_eq, evidence = inertias[joint.path]
        existing = _existing_armature(prim)
        if existing and not options.force:
            ctx.armature[joint.path] = existing[1]
            records.append(
                RepairRecord(
                    rule="armature.default",
                    status=SKIPPED,
                    prim=joint.path,
                    attribute=existing[0],
                    old=existing[1],
                    old_state="authored",
                    reason="armature is already authored; not overwriting without --force",
                    backend=NEUTRAL,
                )
            )
            continue

        if not options.is_enabled("armature.default"):
            ctx.armature[joint.path] = 0.0
            continue

        ceiling = angular_max if joint.dof == ANGULAR else linear_max
        rotor_term = options.armature_fraction * i_eq
        floor_term = options.armature_floor * ceiling
        value = max(rotor_term, floor_term)
        ctx.armature[joint.path] = value
        if value <= 0:
            records.append(
                RepairRecord(
                    rule="armature.default",
                    status=SKIPPED,
                    prim=joint.path,
                    attribute="newton:armature",
                    reason="equivalent inertia is zero, so both armature terms are zero too",
                    backend=NEUTRAL,
                    evidence={"I_eq": i_eq},
                )
            )
            continue

        units = "kg*m^2" if joint.dof == ANGULAR else "kg"
        chosen = (
            "rotor term (alpha * I_eq)"
            if rotor_term >= floor_term
            else "conditioning floor (beta * I_eq_max)"
        )
        shared_evidence = {
            "I_eq": i_eq,
            "I_eq_max": ceiling,
            "alpha": options.armature_fraction,
            "beta": options.armature_floor,
            "rotor_term": rotor_term,
            "floor_term": floor_term,
            "term_used": chosen,
            "dof": joint.dof,
            "formula": "armature = max(alpha * I_eq, beta * I_eq_max)",
            "unmeasured": True,
        }
        if joint.dof == LINEAR:
            shared_evidence["note"] = "linear DOF: I_eq and the floor are masses in kg, not kg*m^2"

        for backend in (NEUTRAL, *options.backends):
            if backend == NEWTON:
                # Newton reads the same newton:armature the neutral layer holds.
                continue
            attribute, type_name, schema, uniform = ARMATURE_ATTRS[backend]
            writes.append(
                PlannedWrite(
                    prim=joint.path,
                    backend=backend,
                    attribute=attribute,
                    value=value,
                    type_name=type_name,
                    apply_schema=schema,
                    uniform=uniform,
                )
            )
            records.append(
                RepairRecord(
                    rule="armature.default",
                    status=APPLIED,
                    prim=joint.path,
                    attribute=attribute,
                    old=None,
                    old_state=attribute_state(prim, attribute) if prim else ABSENT,
                    new=value,
                    units=units,
                    backend=backend,
                    layer=ctx.layer_names[backend],
                    reason=f"no armature in any namespace; {chosen} selected",
                    confidence=MEDIUM,
                    evidence=shared_evidence,
                    forced=bool(existing and options.force),
                )
            )

    return records, writes
