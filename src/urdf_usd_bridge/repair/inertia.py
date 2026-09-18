# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Inertia repairs -- ``docs/history/ANALYSIS.md`` G2.

Four rules, evaluated per body in a fixed order so their precedence is explicit
rather than emergent:

``inertia.derive-from-geometry``
    A body with mass but no usable tensor gets one from its collision geometry.
``inertia.principal-axes-identity``
    A zero ``physics:principalAxes`` quaternion becomes identity. This is the
    ``urdf-usd-converter`` 0.3.2 defect that Isaac Sim 6.1.0 ships, confirmed
    behaviourally in Phase 2.5.
``inertia.make-physical``
    Negative eigenvalues and triangle-inequality violations are projected back
    to the nearest physical tensor, always by *raising* inertia.
``inertia.mass-floor``
    Off by default: reports a massless body rather than inventing a mass.

Deriving runs before the quaternion fix, because a derived tensor authors a real
orientation and makes the identity fallback unnecessary.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from pxr import Sdf, UsdGeom

from .base import (
    ABSENT,
    APPLIED,
    AUTHORED,
    ERROR,
    HIGH,
    LOW,
    MEDIUM,
    NEUTRAL,
    REPORTED,
    SKIPPED,
    WARNING,
    RepairRecord,
    attribute_state,
    should_write,
)
from .geometry_inertia import canonical_eigendecomposition, inertia_from_geometry, newton_six_vector
from .layer import PlannedWrite

IDENTITY_QUAT = [1.0, 0.0, 0.0, 0.0]

#: Relative tolerance for the triangle inequality, matching `inspect`.
_TRIANGLE_TOL = 1e-6

#: Fallback body radius as a fraction of the articulation's bounding diagonal.
_FALLBACK_RADIUS_FRACTION = 0.01

#: Density used only by the opt-in mass floor, in kg/m^3 (roughly water).
_FALLBACK_DENSITY = 1000.0


def _quat_norm(value) -> float:
    if value is None:
        return 0.0
    try:
        imaginary = value.GetImaginary()
        return math.sqrt(value.GetReal() ** 2 + imaginary[0] ** 2 + imaginary[1] ** 2 + imaginary[2] ** 2)
    except AttributeError:  # pragma: no cover - defensive
        return 0.0


def _quat_list(value) -> list[float] | None:
    if value is None:
        return None
    try:
        imaginary = value.GetImaginary()
        return [float(value.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2])]
    except AttributeError:  # pragma: no cover
        return None


def _triangle_ok(diag: list[float]) -> bool:
    a, b, c = sorted(float(v) for v in diag)
    return a + b >= c * (1.0 - _TRIANGLE_TOL)


def _make_physical(diag: list[float]) -> tuple[list[float], list[str]]:
    """Smallest change that makes a diagonal tensor physical.

    Negatives are clamped to zero, then the two smaller moments are raised
    until the triangle inequality holds. The largest moment is never lowered:
    raising inertia is always the stability-increasing direction, and lowering
    it would make the repair itself a destabilising change.
    """
    notes: list[str] = []
    values = [float(v) for v in diag]
    if any(v < 0 for v in values):
        notes.append("clamped negative principal moments to zero")
        values = [max(0.0, v) for v in values]
    order = sorted(range(3), key=lambda i: values[i])
    low, mid, high = (values[i] for i in order)
    if low + mid < high:
        deficit = high - (low + mid)
        values[order[0]] = low + deficit / 2.0
        values[order[1]] = mid + deficit / 2.0
        notes.append("raised the two smaller moments to satisfy I1 + I2 >= I3")
    return values, notes


def _articulation_diagonal(stage) -> float:
    """Bounding-box diagonal of the whole asset, in stage units."""
    prim = stage.GetDefaultPrim()
    if not prim or not prim.IsValid():
        return 0.0
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide])
    try:
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    except Exception:  # pragma: no cover - defensive across USD versions
        return 0.0
    if box.IsEmpty():
        return 0.0
    size = box.GetSize()
    return float(math.sqrt(size[0] ** 2 + size[1] ** 2 + size[2] ** 2))


def apply_inertia_rules(ctx) -> tuple[list[RepairRecord], list[PlannedWrite]]:
    """Run every inertia rule over every rigid body."""
    records: list[RepairRecord] = []
    writes: list[PlannedWrite] = []
    scale = ctx.scale
    layer = ctx.layer_names[NEUTRAL]
    diagonal_span = _articulation_diagonal(ctx.stage) * scale

    for path in sorted(ctx.articulation.bodies):
        prim = ctx.stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():  # pragma: no cover - defensive
            continue
        body = ctx.articulation.bodies[path]

        mass_state = attribute_state(prim, "physics:mass")
        mass = body.mass
        diag_state = attribute_state(prim, "physics:diagonalInertia")
        diag_attr = prim.GetAttribute("physics:diagonalInertia")
        diag_value = list(diag_attr.Get()) if diag_attr and diag_attr.IsValid() else None
        diag_authored = diag_state == AUTHORED
        diag_is_zero = diag_value is not None and all(abs(float(v)) < 1e-15 for v in diag_value)

        axes_state = attribute_state(prim, "physics:principalAxes")
        axes_attr = prim.GetAttribute("physics:principalAxes")
        axes_value = axes_attr.Get() if axes_attr and axes_attr.IsValid() else None
        axes_zero = _quat_norm(axes_value) < 1e-9

        # --- mass ---------------------------------------------------------
        if mass is None or mass <= 0:
            records.extend(_mass_rule(ctx, prim, path, mass, mass_state, layer, writes, scale))
            continue

        # --- derive a tensor when there is none ---------------------------
        needs_tensor = (not diag_authored) or diag_is_zero or ctx.options.force
        derived = None
        if needs_tensor and ctx.options.is_enabled("inertia.derive-from-geometry"):
            com_attr = prim.GetAttribute("physics:centerOfMass")
            com = com_attr.Get() if com_attr and com_attr.IsValid() and com_attr.HasAuthoredValue() else None
            derived = inertia_from_geometry(prim, float(mass), scale=scale, center_of_mass=com)
            if derived is None and diagonal_span > 0:
                derived = _fallback_sphere(float(mass), diagonal_span)

        if needs_tensor and derived is not None:
            records.extend(
                _write_tensor(
                    ctx,
                    path,
                    derived,
                    layer,
                    writes,
                    scale,
                    old_diag=diag_value,
                    old_state=diag_state,
                    old_axes=_quat_list(axes_value),
                    axes_state=axes_state,
                )
            )
            continue

        if needs_tensor:
            records.append(
                RepairRecord(
                    rule="inertia.derive-from-geometry",
                    status=REPORTED,
                    prim=path,
                    attribute="physics:diagonalInertia",
                    old=diag_value,
                    old_state=diag_state,
                    reason=(
                        "body has mass but no inertia tensor, and no collision geometry to derive " "one from"
                        if ctx.options.is_enabled("inertia.derive-from-geometry")
                        else "rule disabled"
                    ),
                    severity=ERROR,
                    confidence=LOW,
                    backend=NEUTRAL,
                    evidence={"mass": float(mass)},
                )
            )

        # --- zero principal axes on an otherwise usable tensor -------------
        if axes_zero and axes_state == AUTHORED and ctx.options.is_enabled("inertia.principal-axes-identity"):
            writes.append(
                PlannedWrite(
                    prim=path,
                    backend=NEUTRAL,
                    attribute="physics:principalAxes",
                    value=IDENTITY_QUAT,
                    type_name=Sdf.ValueTypeNames.Quatf,
                    apply_schema="PhysicsMassAPI",
                )
            )
            records.append(
                RepairRecord(
                    rule="inertia.principal-axes-identity",
                    status=APPLIED,
                    prim=path,
                    attribute="physics:principalAxes",
                    old=_quat_list(axes_value),
                    old_state=axes_state,
                    new=IDENTITY_QUAT,
                    units="quaternion (w, x, y, z)",
                    backend=NEUTRAL,
                    layer=layer,
                    reason=(
                        "authored principalAxes is the zero quaternion, which has no meaning; "
                        "identity is the only reading of 'no rotation information'"
                    ),
                    confidence=HIGH,
                    evidence={
                        "norm": 0.0,
                        "known_cause": "urdf-usd-converter 0.3.2 apply_inertial, fixed in 0.3.3",
                    },
                )
            )

        # --- non-physical tensors -----------------------------------------
        if diag_authored and diag_value is not None and not diag_is_zero:
            negative = any(float(v) < 0 for v in diag_value)
            triangle_bad = not _triangle_ok(diag_value)
            if (negative or triangle_bad) and ctx.options.is_enabled("inertia.make-physical"):
                repaired, notes = _make_physical(diag_value)
                writes.append(
                    PlannedWrite(
                        prim=path,
                        backend=NEUTRAL,
                        attribute="physics:diagonalInertia",
                        value=repaired,
                        type_name=Sdf.ValueTypeNames.Float3,
                        apply_schema="PhysicsMassAPI",
                    )
                )
                records.append(
                    RepairRecord(
                        rule="inertia.make-physical",
                        status=APPLIED,
                        prim=path,
                        attribute="physics:diagonalInertia",
                        old=[float(v) for v in diag_value],
                        old_state=diag_state,
                        new=repaired,
                        units="kg*m^2",
                        backend=NEUTRAL,
                        layer=layer,
                        reason="; ".join(notes) or "projected onto the physical set",
                        confidence=MEDIUM,
                        evidence={
                            "negative_moment": negative,
                            "triangle_inequality_violated": triangle_bad,
                        },
                    )
                )
            elif negative or triangle_bad:
                records.append(
                    RepairRecord(
                        rule="inertia.make-physical",
                        status=SKIPPED,
                        prim=path,
                        attribute="physics:diagonalInertia",
                        old=[float(v) for v in diag_value],
                        old_state=diag_state,
                        reason="rule disabled",
                        severity=WARNING,
                        backend=NEUTRAL,
                    )
                )
            elif not should_write(diag_state, valid=True, force=ctx.options.force):
                records.append(
                    RepairRecord(
                        rule="inertia.derive-from-geometry",
                        status=SKIPPED,
                        prim=path,
                        attribute="physics:diagonalInertia",
                        old=[float(v) for v in diag_value],
                        old_state=diag_state,
                        reason="authored tensor is physical; not overwriting without --force",
                        backend=NEUTRAL,
                    )
                )

    return records, writes


def _fallback_sphere(mass: float, diagonal_span: float) -> dict[str, Any]:
    """A last-resort tensor for a body with mass and no geometry at all."""
    radius = max(_FALLBACK_RADIUS_FRACTION * diagonal_span, 1e-4)
    moment = 0.4 * mass * radius * radius
    return {
        "tensor": np.diag([moment, moment, moment]),
        "center_of_mass": None,
        "sources": [],
        "fallback": {
            "shape": "solid sphere",
            "radius_m": radius,
            "basis": "1% of the articulation's bounding-box diagonal",
        },
    }


def _write_tensor(
    ctx,
    path: str,
    derived: dict[str, Any],
    layer: str,
    writes: list[PlannedWrite],
    scale: float,
    *,
    old_diag,
    old_state,
    old_axes,
    axes_state,
) -> list[RepairRecord]:
    """Author diagonal inertia, principal axes and ``newton:inertia`` together.

    All three describe the same tensor. Authoring one without the others is how
    a stage ends up with a Newton solver and a PhysX solver disagreeing about
    the same body, so they are always written as a set.
    """
    tensor = derived["tensor"]
    diagonal, quaternion = canonical_eigendecomposition(tensor)
    fallback = derived.get("fallback")
    confidence = LOW if fallback else MEDIUM
    source_note = (
        f"no collision geometry; assumed a {fallback['shape']} of radius "
        f"{fallback['radius_m']:.6g} m ({fallback['basis']})"
        if fallback
        else "derived from collision geometry at uniform density, scaled to the authored mass"
    )
    evidence: dict[str, Any] = {
        "sources": derived.get("sources", []),
        "tensor": [[float(v) for v in row] for row in tensor],
    }
    if fallback:
        evidence["fallback"] = fallback
    if any(s.get("type") == "Mesh" for s in derived.get("sources", [])):
        evidence["mesh_approximation"] = "oriented bounding box, not the exact volume integral"
    if derived.get("com_offset"):
        evidence["com_offset_m"] = derived["com_offset"]
        evidence["com_note"] = (
            "the authored physics:centerOfMass is not where the collision geometry is; the "
            "tensor was moved to the declared point by the parallel-axis theorem, which adds "
            "inertia rather than removing it"
        )

    # A stage unit other than metres means the authored value is not kg*m^2.
    inverse = 1.0 / (scale * scale) if scale not in (0.0, 1.0) else 1.0
    stored_diagonal = [v * inverse for v in diagonal]

    writes.append(
        PlannedWrite(
            prim=path,
            backend=NEUTRAL,
            attribute="physics:diagonalInertia",
            value=stored_diagonal,
            type_name=Sdf.ValueTypeNames.Float3,
            apply_schema="PhysicsMassAPI",
        )
    )
    writes.append(
        PlannedWrite(
            prim=path,
            backend=NEUTRAL,
            attribute="physics:principalAxes",
            value=quaternion,
            type_name=Sdf.ValueTypeNames.Quatf,
            apply_schema="PhysicsMassAPI",
        )
    )
    writes.append(
        PlannedWrite(
            prim=path,
            backend=NEUTRAL,
            attribute="newton:inertia",
            value=[v * inverse for v in newton_six_vector(tensor)],
            type_name=Sdf.ValueTypeNames.DoubleArray,
            apply_schema="NewtonMassAPI",
        )
    )

    records = [
        RepairRecord(
            rule="inertia.derive-from-geometry",
            status=APPLIED,
            prim=path,
            attribute="physics:diagonalInertia",
            old=[float(v) for v in old_diag] if old_diag is not None else None,
            old_state=old_state,
            new=stored_diagonal,
            units="kg*m^2",
            backend=NEUTRAL,
            layer=layer,
            reason=source_note,
            confidence=confidence,
            evidence=evidence,
        ),
        RepairRecord(
            rule="inertia.derive-from-geometry",
            status=APPLIED,
            prim=path,
            attribute="physics:principalAxes",
            old=old_axes,
            old_state=axes_state,
            new=quaternion,
            units="quaternion (w, x, y, z)",
            backend=NEUTRAL,
            layer=layer,
            reason="eigenvectors of the derived tensor, canonicalised for determinism",
            confidence=confidence,
        ),
        RepairRecord(
            rule="inertia.derive-from-geometry",
            status=APPLIED,
            prim=path,
            attribute="newton:inertia",
            old=None,
            old_state=ABSENT,
            new=[v * inverse for v in newton_six_vector(tensor)],
            units="kg*m^2, [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]",
            backend=NEUTRAL,
            layer=layer,
            reason="the same tensor Newton reads, so Newton and PhysX agree on this body",
            confidence=confidence,
        ),
    ]
    return records


def _mass_rule(
    ctx, prim, path: str, mass, mass_state: str, layer: str, writes: list[PlannedWrite], scale: float
) -> list[RepairRecord]:
    """Massless bodies: report by default, estimate only when asked."""
    density_attr = prim.GetAttribute("physics:density")
    density = (
        float(density_attr.Get())
        if density_attr and density_attr.IsValid() and density_attr.HasAuthoredValue()
        else None
    )
    if density:
        return [
            RepairRecord(
                rule="inertia.mass-floor",
                status=SKIPPED,
                prim=path,
                attribute="physics:mass",
                old=mass,
                old_state=mass_state,
                reason=f"no mass, but physics:density = {density} is authored; the backend will derive one",
                severity=WARNING,
                backend=NEUTRAL,
            )
        ]

    if not ctx.options.is_enabled("inertia.mass-floor"):
        return [
            RepairRecord(
                rule="inertia.mass-floor",
                status=REPORTED,
                prim=path,
                attribute="physics:mass",
                old=mass,
                old_state=mass_state,
                reason=(
                    "rigid body has no mass and no density; refusing to invent one. "
                    "Fix the URDF, or pass --enable inertia.mass-floor to estimate from geometry"
                ),
                severity=ERROR,
                confidence=LOW,
                backend=NEUTRAL,
            )
        ]

    derived = inertia_from_geometry(prim, 1.0, scale=scale)
    if derived is None:
        return [
            RepairRecord(
                rule="inertia.mass-floor",
                status=REFUSED_NO_GEOMETRY,
                prim=path,
                attribute="physics:mass",
                old=mass,
                old_state=mass_state,
                reason="no mass, no density and no collision geometry to estimate a volume from",
                severity=ERROR,
                confidence=LOW,
                backend=NEUTRAL,
            )
        ]
    volume = sum(source["volume"] for source in derived["sources"])
    estimated = volume * _FALLBACK_DENSITY
    writes.append(
        PlannedWrite(
            prim=path,
            backend=NEUTRAL,
            attribute="physics:mass",
            value=estimated,
            type_name=Sdf.ValueTypeNames.Float,
            apply_schema="PhysicsMassAPI",
        )
    )
    return [
        RepairRecord(
            rule="inertia.mass-floor",
            status=APPLIED,
            prim=path,
            attribute="physics:mass",
            old=mass,
            old_state=mass_state,
            new=estimated,
            units="kg",
            backend=NEUTRAL,
            layer=layer,
            reason=f"estimated from {volume:.6g} m^3 of collision geometry at {_FALLBACK_DENSITY} kg/m^3",
            confidence=LOW,
            evidence={"volume_m3": volume, "density_kg_m3": _FALLBACK_DENSITY},
        )
    ]


#: Distinct from SKIPPED: we wanted to act and could not.
REFUSED_NO_GEOMETRY = "refused"
