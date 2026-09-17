# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Hand-authored USD stages that mimic known converter output shapes.

These exist to test *our inspector*, not to prove anything about upstream
behaviour. Evidence about what the converters actually author lives in
``tests/converter/``, which runs the real converters.

Attributes in the ``newton:``, ``mjc:`` and ``physx*:`` namespaces are authored
as plain custom attributes rather than through applied schemas, which exercises
the read-by-name path that must work without schema plugin registration.
"""

from __future__ import annotations

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def _custom(prim: Usd.Prim, name: str, type_name, value):
    attr = prim.CreateAttribute(name, type_name, custom=True)
    attr.Set(value)
    return attr


def new_stage(robot_name: str = "robot") -> Usd.Stage:
    """An empty asset with the scope layout both converters emit."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetMetadata(UsdPhysics.Tokens.kilogramsPerUnit, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{robot_name}").GetPrim()
    stage.SetDefaultPrim(root)
    UsdGeom.Scope.Define(stage, f"/{robot_name}/Geometry")
    UsdGeom.Scope.Define(stage, f"/{robot_name}/Physics")
    return stage


def add_body(
    stage: Usd.Stage,
    path: str,
    *,
    mass: float | None = None,
    diagonal_inertia: tuple[float, float, float] | None = None,
    principal_axes: tuple[float, float, float, float] | None = None,
    newton_inertia: list[float] | None = None,
) -> Usd.Prim:
    """A rigid body with whatever subset of mass properties is requested."""
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    if any(v is not None for v in (mass, diagonal_inertia, principal_axes)):
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        if mass is not None:
            mass_api.CreateMassAttr().Set(mass)
        if diagonal_inertia is not None:
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*diagonal_inertia))
        if principal_axes is not None:
            real, i, j, k = principal_axes
            mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(real, Gf.Vec3f(i, j, k)))
    if newton_inertia is not None:
        _custom(prim, "newton:inertia", Sdf.ValueTypeNames.DoubleArray, newton_inertia)
    return prim


def add_revolute_joint(
    stage: Usd.Stage,
    path: str,
    *,
    lower: float | None = None,
    upper: float | None = None,
    newton_damping: float | None = None,
    newton_friction: float | None = None,
    urdf_damping: float | None = None,
    urdf_friction: float | None = None,
    urdf_effort: float | None = None,
    drive: bool = False,
    drive_stiffness: float | None = None,
    drive_damping: float | None = None,
    drive_max_force: float | None = None,
    physx_armature: float | None = None,
) -> Usd.Prim:
    """A revolute joint shaped like a particular producer's output."""
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    prim = joint.GetPrim()
    if lower is not None:
        joint.CreateLowerLimitAttr().Set(lower)
    if upper is not None:
        joint.CreateUpperLimitAttr().Set(upper)
    if newton_damping is not None:
        _custom(prim, "newton:damping", Sdf.ValueTypeNames.Float, newton_damping)
    if newton_friction is not None:
        _custom(prim, "newton:friction", Sdf.ValueTypeNames.Float, newton_friction)
    if urdf_damping is not None:
        _custom(prim, "urdf:dynamics:damping", Sdf.ValueTypeNames.Float, urdf_damping)
    if urdf_friction is not None:
        _custom(prim, "urdf:dynamics:friction", Sdf.ValueTypeNames.Float, urdf_friction)
    if urdf_effort is not None:
        _custom(prim, "urdf:limit:effort", Sdf.ValueTypeNames.Float, urdf_effort)
    if physx_armature is not None:
        _custom(prim, "physxJoint:armature", Sdf.ValueTypeNames.Float, physx_armature)
    if drive:
        drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
        if drive_stiffness is not None:
            drive_api.CreateStiffnessAttr().Set(drive_stiffness)
        if drive_damping is not None:
            drive_api.CreateDampingAttr().Set(drive_damping)
        if drive_max_force is not None:
            drive_api.CreateMaxForceAttr().Set(drive_max_force)
    return prim


def add_mjc_actuator(
    stage: Usd.Stage,
    path: str,
    target: str,
    *,
    gain_prm: list[float] | None = None,
    bias_prm: list[float] | None = None,
) -> Usd.Prim:
    """An ``MjcActuator``, optionally without gain parameters (Isaac's fallback)."""
    prim = stage.DefinePrim(path, "MjcActuator")
    prim.CreateRelationship("mjc:target", custom=True).SetTargets([Sdf.Path(target)])
    if gain_prm is not None:
        _custom(prim, "mjc:gainPrm", Sdf.ValueTypeNames.FloatArray, gain_prm)
    if bias_prm is not None:
        _custom(prim, "mjc:biasPrm", Sdf.ValueTypeNames.FloatArray, bias_prm)
    return prim


def add_collider(stage: Usd.Stage, path: str, *, approximation: str | None = "convexHull"):
    """A mesh collider, defaulting to the only approximation the converter emits."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    prim = mesh.GetPrim()
    UsdGeom.Imageable(prim).CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(prim)
    if approximation is not None:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set(approximation)
    return prim


def save(stage: Usd.Stage, path) -> str:
    """Export an in-memory stage so it can be opened by identifier."""
    stage.GetRootLayer().Export(str(path))
    return str(path)
