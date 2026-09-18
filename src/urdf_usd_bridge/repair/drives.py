# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Drive repairs -- ``docs/ANALYSIS.md`` G1.

Two halves that are easy to confuse, and keeping them apart is most of the
design.

**Mirror what the URDF said.** The URDF authored ``<dynamics damping="1.5"
friction="0.3">``; the converter put it in ``newton:damping`` /
``newton:friction``; Isaac Sim 6.1.0 reads ``urdf:dynamics:*``, a spelling
converter 0.3.0+ no longer writes, so PhysX and MuJoCo see nothing. Mirroring is
a lossless translation of a value the user authored, not an invention.

**Supply actuation the URDF never had.** Neither converter authors any drive
gains, so the robot cannot hold a pose. This half is a real addition: it is
opt-out-able, every value is logged with the arithmetic that produced it, and
the tuning constants behind it are unmeasured Phase 3 defaults.

Units, per ``docs/PHASE3_DESIGN.md`` section 3
---------------------------------------------
Everything is computed in SI and converted exactly once, on the way out:

* ``UsdPhysics.DriveAPI``      -- per **degree**  (multiply SI by pi/180)
* ``MjcActuator`` gain/bias    -- per **radian**  (SI unchanged)
* ``NewtonPDControlAPI`` kp/kd -- per **radian**  (SI unchanged)
* friction, armature, maxForce -- no angle unit, never scaled

We author the ``MjcActuator`` gains ourselves and never let Isaac Sim's
``create_mjc_actuator_from_physics`` derive them, because that function copies
the per-degree ``DriveAPI`` value straight into the per-radian MJCF gain slots
(``urdf_to_mjc_physx_conversion_utils.py:282-283``) -- a 57.3x error that only
becomes live once somebody populates the PhysX drive, which is exactly what this
module does. See ``docs/UPSTREAM_ISSUES.md``.

The PhysX asymmetry
-------------------
PhysX has no passive joint-damping attribute; its only velocity-proportional
term is the drive's damping. So on the PhysX path passive damping is folded into
``DriveAPI.damping``, while MuJoCo and Newton keep the passive term on the joint
and the drive term on the actuator. Same physics, different numbers in the same
conceptual slot -- which is why the backends get separate layers.
"""

from __future__ import annotations

import math

from pxr import Sdf

from ..model.units import ANGULAR, PER_RADIAN_TO_PER_DEGREE, gain_urdf_to_usd
from .base import (
    ABSENT,
    APPLIED,
    AUTHORED,
    HIGH,
    MEDIUM,
    MUJOCO,
    NEUTRAL,
    NEWTON,
    PHYSX,
    REPORTED,
    SKIPPED,
    STABLE_RATE_RATIO,
    WARNING,
    RepairRecord,
    attribute_state,
    should_write,
)
from .layer import PlannedWrite

#: MJCF gain/bias arrays are ten elements wide.
_MJC_PRM_WIDTH = 10

_FIXED_JOINT_TYPES = {"PhysicsFixedJoint", "PhysicsSphericalJoint", "PhysicsDistanceJoint", "PhysicsJoint"}


def _authored(prim, name: str):
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid() or not attr.HasAuthoredValue():
        return None
    return attr.Get()


def _prm(values: list[float]) -> list[float]:
    """Pad a gain/bias array out to MJCF's ten slots."""
    return [float(v) for v in values] + [0.0] * (_MJC_PRM_WIDTH - len(values))


def gains_from_frequency(inertia: float, frequency_hz: float, damping_ratio: float) -> tuple[float, float]:
    """``(K, D)`` in SI for a target natural frequency and damping ratio.

    ``K = I * (2 pi f_n)^2`` and ``D = 2 zeta sqrt(I K)``, the standard
    second-order relations. Adapted from Isaac Sim's
    ``gain_tuner_drive_math.stiffness_and_damping_from_natural_frequency_position_drive``
    (Apache-2.0; see ``THIRD_PARTY.md``), with the stored-gain conversion moved
    to our own units module so one table governs every backend.
    """
    omega = 2.0 * math.pi * frequency_hz
    stiffness = inertia * omega * omega
    damping = damping_ratio * 2.0 * math.sqrt(max(inertia, 0.0) * max(stiffness, 0.0))
    return stiffness, damping


def apply_drive_rules(ctx) -> tuple[list[RepairRecord], list[PlannedWrite]]:
    """Mirror passive dynamics and derive drive gains, per backend."""
    records: list[RepairRecord] = []
    writes: list[PlannedWrite] = []
    options = ctx.options

    frequency = options.resolve_frequency()
    # The warning uses the divisor that applies to *this* backend selection, so
    # a single-backend asset is not warned about a bound it does not have.
    from .base import BACKEND_RATE_DIVISOR

    divisor = (
        BACKEND_RATE_DIVISOR.get(options.backends[0], STABLE_RATE_RATIO)
        if len(options.backends) == 1
        else STABLE_RATE_RATIO
    )
    if frequency > options.control_rate / divisor:
        records.append(
            RepairRecord(
                rule="drives.derive-gains",
                status=REPORTED,
                prim="/",
                reason=(
                    f"target frequency {frequency:g} Hz is above control_rate / {divisor:g} = "
                    f"{options.control_rate / divisor:.4g} Hz for backends "
                    f"{', '.join(options.backends)}. Measured 2026-09-18: PhysX and MuJoCo are "
                    "stable at control_rate/6, Newton's Featherstone solver diverges there and "
                    "needs /12, and armature does not change either (docs/PHASE4_REPORT.md)"
                ),
                severity=WARNING,
                backend=NEUTRAL,
                evidence={
                    "target_frequency_hz": frequency,
                    "control_rate_hz": options.control_rate,
                    "divisor": divisor,
                },
            )
        )

    for joint in ctx.articulation.joints:
        prim = ctx.stage.GetPrimAtPath(joint.path)
        if not prim or not prim.IsValid():  # pragma: no cover - defensive
            continue
        dof = joint.dof
        if dof is None:
            if (
                ctx.options.is_enabled("drives.no-drive-for-fixed")
                and str(prim.GetTypeName()) in _FIXED_JOINT_TYPES
            ):
                records.append(
                    RepairRecord(
                        rule="drives.no-drive-for-fixed",
                        status=SKIPPED,
                        prim=joint.path,
                        reason=f"{prim.GetTypeName()} has no actuatable DOF; no drive or armature",
                        backend=NEUTRAL,
                    )
                )
            continue

        is_angular = dof == ANGULAR
        passive_damping_stored = _authored(prim, "newton:damping")
        passive_friction = _authored(prim, "newton:friction")
        # newton:damping is per degree; SI is per radian.
        passive_damping_si = (
            float(passive_damping_stored) * (180.0 / math.pi if is_angular else 1.0)
            if passive_damping_stored
            else 0.0
        )
        max_force = _authored(prim, "urdf:limit:effort")

        records.extend(
            _mirror_passive(ctx, prim, joint, is_angular, passive_damping_si, passive_friction, writes)
        )
        records.extend(_derive_gains(ctx, prim, joint, is_angular, passive_damping_si, max_force, writes))

    return records, writes


def _mirror_passive(
    ctx, prim, joint, is_angular: bool, passive_damping_si: float, passive_friction, writes
) -> list[RepairRecord]:
    """Translate ``newton:*`` passive dynamics into each backend's spelling."""
    if not ctx.options.is_enabled("drives.mirror-passive"):
        return []
    records: list[RepairRecord] = []
    options = ctx.options

    if passive_damping_si and MUJOCO in options.backends:
        state = attribute_state(prim, "mjc:damping")
        if should_write(state, valid=bool(_authored(prim, "mjc:damping")), force=options.force):
            writes.append(
                PlannedWrite(
                    prim=joint.path,
                    backend=MUJOCO,
                    attribute="mjc:damping",
                    value=passive_damping_si,
                    type_name=Sdf.ValueTypeNames.Double,
                    apply_schema="MjcJointAPI",
                    uniform=True,
                )
            )
            records.append(
                RepairRecord(
                    rule="drives.mirror-passive",
                    status=APPLIED,
                    prim=joint.path,
                    attribute="mjc:damping",
                    old=None,
                    old_state=state,
                    new=passive_damping_si,
                    units="N*m*s/rad" if is_angular else "N*s/m",
                    backend=MUJOCO,
                    layer=ctx.layer_names[MUJOCO],
                    reason=(
                        "newton:damping carries the URDF's <dynamics damping>, but MuJoCo reads "
                        "mjc:damping; converted from per-degree to per-radian"
                    ),
                    confidence=HIGH,
                    evidence={
                        "newton_damping_per_degree": passive_damping_si * PER_RADIAN_TO_PER_DEGREE,
                        "conversion": "x 180/pi" if is_angular else "none (linear DOF)",
                    },
                )
            )

    if passive_friction:
        friction = float(passive_friction)
        for backend, attribute, type_name, schema, uniform in (
            (MUJOCO, "mjc:frictionloss", Sdf.ValueTypeNames.Double, "MjcJointAPI", True),
            (PHYSX, "physxJoint:jointFriction", Sdf.ValueTypeNames.Float, "PhysxJointAPI", False),
        ):
            if backend not in options.backends:
                continue
            state = attribute_state(prim, attribute)
            if not should_write(state, valid=bool(_authored(prim, attribute)), force=options.force):
                continue
            writes.append(
                PlannedWrite(
                    prim=joint.path,
                    backend=backend,
                    attribute=attribute,
                    value=friction,
                    type_name=type_name,
                    apply_schema=schema,
                    uniform=uniform,
                )
            )
            records.append(
                RepairRecord(
                    rule="drives.mirror-passive",
                    status=APPLIED,
                    prim=joint.path,
                    attribute=attribute,
                    old=None,
                    old_state=state,
                    new=friction,
                    units="N*m" if is_angular else "N",
                    backend=backend,
                    layer=ctx.layer_names[backend],
                    reason=(
                        "newton:friction carries the URDF's <dynamics friction>; this backend "
                        "reads a different spelling. Coulomb friction is an effort, so no angle "
                        "conversion applies"
                    ),
                    confidence=HIGH,
                )
            )
    return records


def _derive_gains(
    ctx, prim, joint, is_angular: bool, passive_damping_si: float, max_force, writes
) -> list[RepairRecord]:
    """Derive and author PD gains in each selected backend's convention."""
    if not ctx.options.is_enabled("drives.derive-gains"):
        return []

    options = ctx.options
    instance = ANGULAR if is_angular else "linear"
    existing_stiffness = _authored(prim, f"drive:{instance}:physics:stiffness")
    already_tuned = bool(existing_stiffness)
    if already_tuned and not options.force:
        return [
            RepairRecord(
                rule="drives.derive-gains",
                status=SKIPPED,
                prim=joint.path,
                attribute=f"drive:{instance}:physics:stiffness",
                old=float(existing_stiffness),
                old_state=AUTHORED,
                reason="drive stiffness is already authored; not overwriting without --force",
                backend=PHYSX,
            )
        ]

    i_eq, evidence = ctx.articulation.equivalent_inertia(joint)
    armature = ctx.armature.get(joint.path, 0.0)
    i_total = i_eq + armature
    if i_total <= 0:
        return [
            RepairRecord(
                rule="drives.derive-gains",
                status=REPORTED,
                prim=joint.path,
                attribute=f"drive:{instance}:physics:stiffness",
                reason=(
                    "equivalent inertia is zero, so no gain can be derived. This usually means "
                    "the bodies below this joint have no mass or no inertia -- fix those first"
                ),
                severity=WARNING,
                backend=PHYSX,
                evidence={"I_eq": i_eq, "armature": armature},
            )
        ]

    stiffness_si, damping_si = gains_from_frequency(
        i_total, options.resolve_frequency(), options.damping_ratio
    )
    shared_evidence = {
        "I_eq": i_eq,
        "armature": armature,
        "I_total": i_total,
        "K_si": stiffness_si,
        "D_si": damping_si,
        "target_frequency_hz": options.resolve_frequency(),
        "target_frequency_basis": options.target_frequency_basis,
        "damping_ratio": options.damping_ratio,
        "formula": "K_si = I_total * (2*pi*f_n)**2 ; D_si = 2*zeta*sqrt(I_total*K_si)",
        "unmeasured": True,
        "subtree_bodies": evidence.get("subtree_bodies", []),
    }

    records: list[RepairRecord] = []
    if PHYSX in options.backends:
        records.extend(
            _author_physx(
                ctx,
                prim,
                joint,
                instance,
                is_angular,
                stiffness_si,
                damping_si,
                passive_damping_si,
                max_force,
                shared_evidence,
                writes,
            )
        )
    if MUJOCO in options.backends:
        records.extend(
            _author_mujoco(ctx, prim, joint, stiffness_si, damping_si, max_force, shared_evidence, writes)
        )
    if NEWTON in options.backends:
        records.extend(
            _author_newton(
                ctx,
                prim,
                joint,
                instance,
                is_angular,
                stiffness_si,
                damping_si,
                max_force,
                shared_evidence,
                writes,
            )
        )
    return records


def _author_physx(
    ctx,
    prim,
    joint,
    instance,
    is_angular,
    stiffness_si,
    damping_si,
    passive_damping_si,
    max_force,
    shared_evidence,
    writes,
) -> list[RepairRecord]:
    """``UsdPhysics.DriveAPI``, per degree, with passive damping folded in."""
    stored_stiffness = gain_urdf_to_usd(stiffness_si, angular=is_angular)
    total_damping_si = damping_si + passive_damping_si
    stored_damping = gain_urdf_to_usd(total_damping_si, angular=is_angular)
    layer = ctx.layer_names[PHYSX]
    prefix = f"drive:{instance}:physics"

    planned = [
        (f"{prefix}:type", "force", Sdf.ValueTypeNames.Token, True),
        (f"{prefix}:stiffness", stored_stiffness, Sdf.ValueTypeNames.Float, False),
        (f"{prefix}:damping", stored_damping, Sdf.ValueTypeNames.Float, False),
        (f"{prefix}:targetPosition", 0.0, Sdf.ValueTypeNames.Float, False),
    ]
    if max_force:
        planned.append((f"{prefix}:maxForce", float(max_force), Sdf.ValueTypeNames.Float, False))

    for attribute, value, type_name, uniform in planned:
        writes.append(
            PlannedWrite(
                prim=joint.path,
                backend=PHYSX,
                attribute=attribute,
                value=value,
                type_name=type_name,
                apply_schema="PhysicsDriveAPI",
                schema_instance=instance,
                uniform=uniform,
            )
        )

    units_k = "N*m/deg" if is_angular else "N/m"
    units_d = "N*m*s/deg" if is_angular else "N*s/m"
    records = [
        RepairRecord(
            rule="drives.derive-gains",
            status=APPLIED,
            prim=joint.path,
            attribute=f"{prefix}:stiffness",
            old=None,
            old_state=attribute_state(prim, f"{prefix}:stiffness"),
            new=stored_stiffness,
            units=units_k,
            backend=PHYSX,
            layer=layer,
            reason=(
                f"no drive stiffness authored; derived for f_n={ctx.options.resolve_frequency():g} Hz "
                f"({ctx.options.target_frequency_basis}), "
                f"zeta={ctx.options.damping_ratio}"
            ),
            confidence=MEDIUM,
            evidence={**shared_evidence, "stored_conversion": "K_si * pi/180" if is_angular else "none"},
        ),
        RepairRecord(
            rule="drives.derive-gains",
            status=APPLIED,
            prim=joint.path,
            attribute=f"{prefix}:damping",
            old=None,
            old_state=attribute_state(prim, f"{prefix}:damping"),
            new=stored_damping,
            units=units_d,
            backend=PHYSX,
            layer=layer,
            reason=(
                "derived drive damping plus the URDF's passive damping: PhysX has no passive "
                "joint-damping attribute, so the drive is the only place it can go"
            ),
            confidence=MEDIUM,
            evidence={
                **shared_evidence,
                "D_drive_si": damping_si,
                "D_passive_si": passive_damping_si,
                "D_total_si": damping_si + passive_damping_si,
            },
        ),
        RepairRecord(
            rule="drives.derive-gains",
            status=APPLIED,
            prim=joint.path,
            attribute=f"{prefix}:targetPosition",
            old=None,
            old_state=attribute_state(prim, f"{prefix}:targetPosition"),
            new=0.0,
            units="degrees" if is_angular else "stage linear units",
            backend=PHYSX,
            layer=layer,
            reason="hold the pose the asset was authored in, rather than driving it somewhere new",
            confidence=MEDIUM,
        ),
    ]
    if max_force:
        records.append(
            RepairRecord(
                rule="drives.derive-gains",
                status=APPLIED,
                prim=joint.path,
                attribute=f"{prefix}:maxForce",
                old=None,
                old_state=attribute_state(prim, f"{prefix}:maxForce"),
                new=float(max_force),
                units="N*m" if is_angular else "N",
                backend=PHYSX,
                layer=layer,
                reason="from urdf:limit:effort, the one URDF limit attribute whose spelling still connects",
                confidence=HIGH,
            )
        )
    else:
        records.append(
            RepairRecord(
                rule="drives.derive-gains",
                status=REPORTED,
                prim=joint.path,
                attribute=f"{prefix}:maxForce",
                reason=(
                    "no urdf:limit:effort on this joint, so the drive is torque-unbounded. "
                    "Refusing to invent a torque limit"
                ),
                severity=WARNING,
                backend=PHYSX,
            )
        )
    return records


def _author_mujoco(
    ctx, prim, joint, stiffness_si, damping_si, max_force, shared_evidence, writes
) -> list[RepairRecord]:
    """``MjcActuator`` in the position-control form, per radian.

    The encoding -- ``gainPrm = [kp, 0, ...]``, ``biasPrm = [0, -kp, -kd, ...]``,
    ``gainType = fixed``, ``biasType = affine`` -- is the one Isaac Sim's
    ``create_mjc_actuator_from_physics`` documents and writes. We author the
    values in radians, which is where that function is wrong.

    The actuator path matches Isaac's ``_resolve_actuator_path``
    (``<joint parent>/<joint name>_actuator``), so when Isaac has already
    created one our opinions land on it instead of adding a duplicate.
    """
    actuator_path = f"{joint.path.rsplit('/', 1)[0]}/{joint.path.rsplit('/', 1)[1]}_actuator"
    layer = ctx.layer_names[MUJOCO]
    gain_prm = _prm([stiffness_si])
    bias_prm = _prm([0.0, -stiffness_si, -damping_si])

    writes.append(
        PlannedWrite(
            prim=actuator_path,
            backend=MUJOCO,
            define_type="MjcActuator",
            relationship="mjc:target",
            rel_targets=[joint.path],
        )
    )
    for attribute, value, type_name in (
        ("mjc:gainType", "fixed", Sdf.ValueTypeNames.Token),
        ("mjc:biasType", "affine", Sdf.ValueTypeNames.Token),
        ("mjc:gainPrm", gain_prm, Sdf.ValueTypeNames.DoubleArray),
        ("mjc:biasPrm", bias_prm, Sdf.ValueTypeNames.DoubleArray),
    ):
        writes.append(
            PlannedWrite(
                prim=actuator_path,
                backend=MUJOCO,
                attribute=attribute,
                value=value,
                type_name=type_name,
                define_type="MjcActuator",
                uniform=True,
            )
        )
    if max_force:
        for attribute, value in (
            ("mjc:forceRange:max", float(max_force)),
            ("mjc:forceRange:min", -float(max_force)),
        ):
            writes.append(
                PlannedWrite(
                    prim=actuator_path,
                    backend=MUJOCO,
                    attribute=attribute,
                    value=value,
                    type_name=Sdf.ValueTypeNames.Double,
                    define_type="MjcActuator",
                    uniform=True,
                )
            )

    return [
        RepairRecord(
            rule="drives.derive-gains",
            status=APPLIED,
            prim=actuator_path,
            attribute="mjc:gainPrm",
            old=None,
            old_state=ABSENT,
            new=gain_prm,
            units="N*m/rad (slot 0)",
            backend=MUJOCO,
            layer=layer,
            reason=(
                "MjcActuator position-control gain, authored in radians. Isaac Sim's "
                "create_mjc_actuator_from_physics would copy the per-degree DriveAPI value here "
                "instead, which is 57.3x too soft -- see docs/UPSTREAM_ISSUES.md"
            ),
            confidence=MEDIUM,
            evidence={**shared_evidence, "target_joint": joint.path},
        ),
        RepairRecord(
            rule="drives.derive-gains",
            status=APPLIED,
            prim=actuator_path,
            attribute="mjc:biasPrm",
            old=None,
            old_state=ABSENT,
            new=bias_prm,
            units="[0, -N*m/rad, -N*m*s/rad, ...]",
            backend=MUJOCO,
            layer=layer,
            reason=(
                "affine bias giving effort = kp*(target - q) - kd*v. The joint's passive damping "
                "stays on mjc:damping, so the actuator carries the drive term alone"
            ),
            confidence=MEDIUM,
            evidence=shared_evidence,
        ),
    ]


def _author_newton(
    ctx, prim, joint, instance, is_angular, stiffness_si, damping_si, max_force, shared_evidence, writes
) -> list[RepairRecord]:
    """``UsdPhysics.DriveAPI``, per degree -- what Newton actually reads.

    Newton 1.5.0's USD importer takes drive gains from ``UsdPhysics.DriveAPI``
    and divides by ``DegreesToRadian`` (``import_usd.py:1663,1691``), so the
    per-degree value we author arrives as the SI gain we computed. Measured:
    authoring ``5.228089`` per degree yields ``joint_target_ke = 299.547``.

    It reads **nothing** from ``NewtonActuator`` / ``NewtonPDControlAPI``, which
    is consistent with that schema family documenting itself as EXPERIMENTAL.
    Authoring only those leaves the joint undriven, which is what Phase 3 did
    and Phase 4 measured; see ``docs/PHASE4_REPORT.md``.

    **The drive damping is the drive term alone**, unlike PhysX. Newton reads
    ``newton:damping`` as a separate passive term
    (``newton/_src/usd/schemas.py:131``) and adds it itself, so folding the
    passive value into the drive here would count it twice.
    """
    layer = ctx.layer_names[NEWTON]
    prefix = f"drive:{instance}:physics"
    stored_stiffness = gain_urdf_to_usd(stiffness_si, angular=is_angular)
    stored_damping = gain_urdf_to_usd(damping_si, angular=is_angular)

    planned = [
        (f"{prefix}:type", "force", Sdf.ValueTypeNames.Token, True),
        (f"{prefix}:stiffness", stored_stiffness, Sdf.ValueTypeNames.Float, False),
        (f"{prefix}:damping", stored_damping, Sdf.ValueTypeNames.Float, False),
        (f"{prefix}:targetPosition", 0.0, Sdf.ValueTypeNames.Float, False),
    ]
    if max_force:
        planned.append((f"{prefix}:maxForce", float(max_force), Sdf.ValueTypeNames.Float, False))

    for attribute, value, type_name, uniform in planned:
        writes.append(
            PlannedWrite(
                prim=joint.path,
                backend=NEWTON,
                attribute=attribute,
                value=value,
                type_name=type_name,
                apply_schema="PhysicsDriveAPI",
                schema_instance=instance,
                uniform=uniform,
            )
        )

    records = [
        RepairRecord(
            rule="drives.derive-gains",
            status=APPLIED,
            prim=joint.path,
            attribute=f"{prefix}:stiffness",
            old=None,
            old_state=attribute_state(prim, f"{prefix}:stiffness"),
            new=stored_stiffness,
            units="N*m/deg" if is_angular else "N/m",
            backend=NEWTON,
            layer=layer,
            reason=(
                "Newton reads drive gains from UsdPhysics.DriveAPI and converts per-degree to "
                "per-radian itself; NewtonActuator is not consumed by Newton 1.5.0"
            ),
            confidence=MEDIUM,
            evidence={**shared_evidence, "newton_reads": "UsdPhysics.DriveAPI"},
        ),
        RepairRecord(
            rule="drives.derive-gains",
            status=APPLIED,
            prim=joint.path,
            attribute=f"{prefix}:damping",
            old=None,
            old_state=attribute_state(prim, f"{prefix}:damping"),
            new=stored_damping,
            units="N*m*s/deg" if is_angular else "N*s/m",
            backend=NEWTON,
            layer=layer,
            reason=(
                "the drive term only: Newton adds newton:damping as a separate passive term, so "
                "folding it in here would count the URDF's damping twice"
            ),
            confidence=MEDIUM,
            evidence={**shared_evidence, "D_drive_si": damping_si, "passive_handled_by": "newton:damping"},
        ),
    ]

    if ctx.options.newton_actuator:
        records.extend(
            _author_newton_actuator(ctx, joint, stiffness_si, damping_si, max_force, shared_evidence, writes)
        )
    return records


def _author_newton_actuator(
    ctx, joint, stiffness_si, damping_si, max_force, shared_evidence, writes
) -> list[RepairRecord]:
    """``NewtonActuator`` + ``NewtonPDControlAPI``, per radian. Opt-in.

    Off by default, behind ``--newton-actuator``, for two measured reasons:

    * **Newton 1.5.0 does not read it.** Authoring it alone leaves the joint
      undriven, so it cannot be the driving mechanism.
    * **If a later Newton release does read it, the joint would be driven
      twice** -- once from ``UsdPhysics.DriveAPI`` above and once from here,
      with the same gain. The schema itself says attribute names, defaults and
      composition rules may change without notice, so that risk cannot be
      designed away from this side.

    Author it when you are targeting a Newton build you have checked, and pin
    that version.
    """
    actuator_path = f"{joint.path.rsplit('/', 1)[0]}/{joint.path.rsplit('/', 1)[1]}_newton_actuator"
    layer = ctx.layer_names[NEWTON]

    writes.append(
        PlannedWrite(
            prim=actuator_path,
            backend=NEWTON,
            define_type="NewtonActuator",
            relationship="newton:targets",
            rel_targets=[joint.path],
        )
    )
    for attribute, value in (("newton:kp", stiffness_si), ("newton:kd", damping_si)):
        writes.append(
            PlannedWrite(
                prim=actuator_path,
                backend=NEWTON,
                attribute=attribute,
                value=value,
                type_name=Sdf.ValueTypeNames.Float,
                define_type="NewtonActuator",
                apply_schema="NewtonPDControlAPI",
            )
        )
    if max_force:
        writes.append(
            PlannedWrite(
                prim=actuator_path,
                backend=NEWTON,
                attribute="newton:maxEffort",
                value=float(max_force),
                type_name=Sdf.ValueTypeNames.Float,
                define_type="NewtonActuator",
                apply_schema="NewtonMaxEffortClampingAPI",
            )
        )

    return [
        RepairRecord(
            rule="drives.derive-gains",
            status=APPLIED,
            prim=actuator_path,
            attribute="newton:kp",
            old=None,
            old_state=ABSENT,
            new=stiffness_si,
            units="N*m/rad",
            backend=NEWTON,
            layer=layer,
            reason=(
                "opt-in NewtonActuator gain, in radians. Not read by Newton 1.5.0; if a later "
                "release reads it, this joint is driven both here and by its UsdPhysics drive"
            ),
            confidence=MEDIUM,
            evidence={
                **shared_evidence,
                "target_joint": joint.path,
                "experimental_schema": True,
                "verified_unread_by": "newton 1.5.0",
                "double_drive_risk": True,
            },
        ),
    ]
