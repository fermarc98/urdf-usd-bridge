# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The kinematic tree, and the equivalent inertia each joint has to move.

Drive-gain and armature selection both need one number per joint: how much
inertia that degree of freedom actually sees. This module derives it from the
composed stage, at the asset's default pose.

What ``equivalent_inertia`` is, and is not
-----------------------------------------
For a revolute joint it is the parallel-axis sum over every rigid body in the
subtree below the joint::

    I_eq = sum_i ( a . R_i I_i R_i^T . a  +  m_i * d_i**2 )

where ``a`` is the joint axis in world space, ``R_i I_i R_i^T`` is body *i*'s
inertia tensor rotated into world orientation, and ``d_i`` is the perpendicular
distance from the joint axis to that body's centre of mass. For a prismatic
joint it is the plain mass sum.

This is the inertia at the **default pose**, and it ignores the off-diagonal
coupling a full articulated-body mass matrix would carry. It is therefore a
lower bound on what the joint sees through a trajectory, which is the
conservative direction for gain selection: it yields softer gains. Isaac Sim's
own gain tuner takes a scalar ``m_eq`` the same way
(``gain_tuner_drive_math.py``, ``meq_for_drive_frequency``).

Everything here is SI: kilograms, metres, radians. Stage units are applied on
the way in, so an asset authored in centimetres still yields kg*m^2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from pxr import Gf, Usd, UsdGeom

from .units import ANGULAR, LINEAR

#: Joint prim types we derive an equivalent inertia for.
_ANGULAR_JOINTS = {"PhysicsRevoluteJoint"}
_LINEAR_JOINTS = {"PhysicsPrismaticJoint"}

#: Every joint type that connects two bodies. Fixed and multi-DOF joints carry
#: no drive of their own, but they still define parent/child, so a tool welded
#: to a link has to appear in the subtree of every joint above it -- otherwise
#: its mass is missing from that joint's equivalent inertia and the derived
#: gains are too soft.
_ALL_JOINTS = (
    _ANGULAR_JOINTS
    | _LINEAR_JOINTS
    | {
        "PhysicsFixedJoint",
        "PhysicsSphericalJoint",
        "PhysicsDistanceJoint",
        "PhysicsJoint",
    }
)

_AXIS_VECTORS = {
    "X": Gf.Vec3d(1.0, 0.0, 0.0),
    "Y": Gf.Vec3d(0.0, 1.0, 0.0),
    "Z": Gf.Vec3d(0.0, 0.0, 1.0),
}


@dataclass
class Body:
    """One rigid body, in SI units, with its world placement resolved."""

    path: str
    mass: float | None
    center_of_mass_local: Gf.Vec3d
    diagonal_inertia: Gf.Vec3d | None
    principal_axes: Gf.Quatd | None
    newton_inertia: list[float] | None
    #: Local-to-world in **stage units**. The metres-per-unit scale is applied
    #: to positions and tensors separately, never folded into this matrix:
    #: ``ExtractRotationMatrix`` on a scaled matrix returns the scale as well,
    #: which silently multiplies every inertia by the scale a second time.
    world_transform: Gf.Matrix4d
    #: Stage ``metersPerUnit``.
    scale: float = 1.0

    @property
    def center_of_mass_world(self) -> Gf.Vec3d:
        return self.world_transform.Transform(self.center_of_mass_local) * self.scale

    def inertia_tensor_world(self) -> Gf.Matrix3d | None:
        """The body's inertia tensor, rotated into world orientation.

        ``newton:inertia`` wins when authored, because it is the exact tensor
        the converter derived, while ``physics:diagonalInertia`` plus
        ``physics:principalAxes`` is its eigendecomposition and can carry the
        zero-quaternion defect this project exists to repair.
        """
        if self.newton_inertia and len(self.newton_inertia) == 6:
            ixx, iyy, izz, ixy, ixz, iyz = (float(v) for v in self.newton_inertia)
            body_tensor = Gf.Matrix3d(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz)
        elif self.diagonal_inertia is not None:
            body_tensor = Gf.Matrix3d(
                self.diagonal_inertia[0], 0.0, 0.0,
                0.0, self.diagonal_inertia[1], 0.0,
                0.0, 0.0, self.diagonal_inertia[2],
            )  # fmt: skip
            if self.principal_axes is not None and _quat_is_usable(self.principal_axes):
                principal = Gf.Matrix3d(self.principal_axes)
                body_tensor = principal * body_tensor * principal.GetTranspose()
        else:
            return None
        rotation = _rotation_of(self.world_transform)
        return rotation * body_tensor * rotation.GetTranspose()


@dataclass
class Joint:
    """One joint, with the bodies it connects and its world-space axis."""

    path: str
    type_name: str
    body0: str | None
    body1: str | None
    axis_token: str
    anchor_world: Gf.Vec3d
    axis_world: Gf.Vec3d

    @property
    def dof(self) -> str | None:
        if self.type_name in _ANGULAR_JOINTS:
            return ANGULAR
        if self.type_name in _LINEAR_JOINTS:
            return LINEAR
        return None


@dataclass
class Articulation:
    """The joint graph of one asset, plus the inertia bookkeeping."""

    bodies: dict[str, Body] = field(default_factory=dict)
    joints: list[Joint] = field(default_factory=list)
    #: parent body path -> child body paths, from each joint's body0/body1.
    children: dict[str, list[str]] = field(default_factory=dict)

    def subtree(self, root_body: str | None) -> list[str]:
        """Every body at or below ``root_body``, in sorted path order.

        Cycles cannot occur in a URDF-derived tree -- the converter refuses
        closed loops -- but the visited set makes that assumption safe rather
        than load-bearing.
        """
        if root_body is None or root_body not in self.bodies:
            return []
        seen: set[str] = set()
        stack = [root_body]
        while stack:
            current = stack.pop()
            if current in seen:
                continue
            seen.add(current)
            stack.extend(self.children.get(current, ()))
        return sorted(seen)

    def equivalent_inertia(self, joint: Joint) -> tuple[float, dict[str, Any]]:
        """``(I_eq, evidence)`` for one joint, in kg*m^2 or kg.

        The evidence dict goes into the repair record so a reader can redo the
        arithmetic without rerunning the tool.
        """
        dof = joint.dof
        members = self.subtree(joint.body1)
        evidence: dict[str, Any] = {
            "subtree_bodies": members,
            "dof": dof,
            "axis_world": [round(v, 12) for v in joint.axis_world],
        }
        if dof is None or not members:
            evidence["note"] = "no movable subtree below this joint"
            return 0.0, evidence

        total = 0.0
        contributions: list[dict[str, Any]] = []
        for path in members:
            body = self.bodies[path]
            mass = float(body.mass or 0.0)
            if dof == LINEAR:
                contribution = mass
            else:
                tensor = body.inertia_tensor_world()
                axis_term = 0.0
                if tensor is not None:
                    axis_term = float(Gf.Dot(joint.axis_world, tensor * joint.axis_world))
                offset = _perpendicular_distance(
                    body.center_of_mass_world, joint.anchor_world, joint.axis_world
                )
                contribution = axis_term + mass * offset * offset
            total += contribution
            contributions.append({"body": path, "mass": mass, "contribution": contribution})
        evidence["contributions"] = contributions
        return total, evidence


def _quat_is_usable(quat: Gf.Quatd) -> bool:
    """A quaternion is usable when it is not the degenerate zero quaternion.

    ``physics:principalAxes`` has a schema fallback of ``(0, 0, 0, 0)``, and
    ``urdf-usd-converter`` 0.3.2 authors exactly that for a link with an
    inertial origin but no tensor. Multiplying by it annihilates the rotation,
    so callers must treat it as "no rotation information" rather than a value.
    """
    imaginary = quat.GetImaginary()
    norm = math.sqrt(quat.GetReal() ** 2 + imaginary[0] ** 2 + imaginary[1] ** 2 + imaginary[2] ** 2)
    return norm > 1e-9


def _rotation_of(matrix: Gf.Matrix4d) -> Gf.Matrix3d:
    """The rotation part of a transform, with any scale divided out."""
    rows = []
    for i in range(3):
        row = Gf.Vec3d(matrix.GetRow3(i))
        length = row.GetLength()
        rows.append(row / length if length > 1e-12 else row)
    return Gf.Matrix3d(*rows[0], *rows[1], *rows[2])


def _perpendicular_distance(point: Gf.Vec3d, anchor: Gf.Vec3d, axis: Gf.Vec3d) -> float:
    """Distance from ``point`` to the line through ``anchor`` along ``axis``."""
    offset = point - anchor
    along = Gf.Dot(offset, axis)
    perpendicular = offset - axis * along
    return float(perpendicular.GetLength())


def _vec3d(value, default: Gf.Vec3d | None = None) -> Gf.Vec3d | None:
    if value is None:
        return default
    try:
        return Gf.Vec3d(float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, IndexError, ValueError):  # pragma: no cover - defensive
        return default


def _quatd(value) -> Gf.Quatd | None:
    if value is None:
        return None
    try:
        imaginary = value.GetImaginary()
        return Gf.Quatd(float(value.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2]))
    except AttributeError:  # pragma: no cover - defensive
        return None


def _authored_or_none(prim, name: str):
    """The attribute's value, but only when the input actually authored it.

    An unauthored attribute reports its schema fallback, which for
    ``physics:diagonalInertia`` is ``(0, 0, 0)`` and for
    ``physics:principalAxes`` is a zero quaternion. Treating either as data is
    how the defects in ``docs/history/ANALYSIS.md`` G2 propagate.
    """
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid() or not attr.HasAuthoredValue():
        return None
    return attr.Get()


def _first_target(prim, name: str) -> str | None:
    rel = prim.GetRelationship(name)
    if not rel or not rel.IsValid():
        return None
    targets = rel.GetTargets()
    return targets[0].pathString if targets else None


def _world_transform(prim, cache: UsdGeom.XformCache) -> Gf.Matrix4d:
    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return Gf.Matrix4d(1.0)
    return cache.GetLocalToWorldTransform(prim)


def build_articulation(stage, *, meters_per_unit: float | None = None) -> Articulation:
    """Read the kinematic tree and inertias off a composed stage.

    Args:
        stage: An open ``Usd.Stage``.
        meters_per_unit: Override for the stage's linear scale. Defaults to the
            stage metric, so lengths become metres and inertias kg*m^2.

    Returns:
        An :class:`Articulation`. Prims are visited in sorted path order so the
        result -- and therefore every value derived from it -- is deterministic.
    """
    scale = float(meters_per_unit if meters_per_unit is not None else UsdGeom.GetStageMetersPerUnit(stage))
    if scale <= 0:  # pragma: no cover - a stage with a zero metric is malformed
        scale = 1.0
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    articulation = Articulation()
    prims = sorted(stage.TraverseAll(), key=lambda p: p.GetPath().pathString)

    for prim in prims:
        applied = set(prim.GetAppliedSchemas())
        if "PhysicsRigidBodyAPI" not in applied:
            continue
        path = prim.GetPath().pathString
        transform = _world_transform(prim, cache)
        mass = _authored_or_none(prim, "physics:mass")
        diagonal = _vec3d(_authored_or_none(prim, "physics:diagonalInertia"))
        if diagonal is not None and scale != 1.0:
            diagonal = diagonal * (scale * scale)
        newton_inertia = _authored_or_none(prim, "newton:inertia")
        articulation.bodies[path] = Body(
            path=path,
            mass=float(mass) if mass is not None else None,
            center_of_mass_local=_vec3d(_authored_or_none(prim, "physics:centerOfMass"), Gf.Vec3d(0, 0, 0)),
            diagonal_inertia=diagonal,
            principal_axes=_quatd(_authored_or_none(prim, "physics:principalAxes")),
            newton_inertia=(
                [float(v) * scale * scale for v in newton_inertia] if newton_inertia is not None else None
            ),
            world_transform=transform,
            scale=scale,
        )

    for prim in prims:
        type_name = str(prim.GetTypeName())
        if type_name not in _ALL_JOINTS:
            continue
        body0 = _first_target(prim, "physics:body0")
        body1 = _first_target(prim, "physics:body1")
        axis_token = str(_authored_or_none(prim, "physics:axis") or "X")
        local_axis = _AXIS_VECTORS.get(axis_token, _AXIS_VECTORS["X"])

        parent_transform = Gf.Matrix4d(1.0)
        if body0 and body0 in articulation.bodies:
            parent_transform = articulation.bodies[body0].world_transform
        local_pos = _vec3d(_authored_or_none(prim, "physics:localPos0"), Gf.Vec3d(0, 0, 0))
        local_rot = _quatd(_authored_or_none(prim, "physics:localRot0")) or Gf.Quatd(1, 0, 0, 0)

        joint_frame = Gf.Matrix4d(Gf.Rotation(local_rot), local_pos) * parent_transform
        anchor_world = joint_frame.ExtractTranslation() * scale
        axis_world = _rotation_of(joint_frame) * local_axis
        length = axis_world.GetLength()
        axis_world = axis_world / length if length > 1e-12 else local_axis

        articulation.joints.append(
            Joint(
                path=prim.GetPath().pathString,
                type_name=type_name,
                body0=body0,
                body1=body1,
                axis_token=axis_token,
                anchor_world=anchor_world,
                axis_world=axis_world,
            )
        )
        if body0 and body1:
            articulation.children.setdefault(body0, []).append(body1)

    for parent in articulation.children:
        articulation.children[parent].sort()
    articulation.joints.sort(key=lambda j: j.path)
    return articulation


def max_equivalent_inertia(articulation: Articulation) -> float:
    """The largest angular ``I_eq`` in the articulation, for the armature floor.

    Only angular DOFs are considered: mixing kg with kg*m^2 to set a floor
    would be a unit error, and the armature floor exists to bound the
    *rotational* mass-matrix conditioning.
    """
    values = [
        articulation.equivalent_inertia(joint)[0] for joint in articulation.joints if joint.dof == ANGULAR
    ]
    return max(values) if values else 0.0
