# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Hand-authored articulations for the repair tests.

Separate from ``builders.py``, which shapes stages for the *inspector*. These
build a real kinematic tree -- bodies with world transforms, joints with
``physics:body0``/``body1`` and an axis -- because the repair rules compute an
equivalent inertia from the geometry and cannot be tested against a flat stage.

``variant_asset`` additionally reproduces the shape of an Isaac Sim 6.x package:
a ``Physics`` variant set with no authored selection. That is what makes the
variant-scoped authoring path testable without a 32 GB Isaac Sim install.
"""

from __future__ import annotations

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def _custom(prim, name, type_name, value, uniform=False):
    variability = Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying
    attr = prim.CreateAttribute(name, type_name, custom=True, variability=variability)
    attr.Set(value)
    return attr


def new_asset(name: str = "robot"):
    """An empty asset with the Geometry/Physics scopes both converters emit."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetMetadata(UsdPhysics.Tokens.kilogramsPerUnit, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}").GetPrim()
    stage.SetDefaultPrim(root)
    UsdGeom.Scope.Define(stage, f"/{name}/Geometry")
    UsdGeom.Scope.Define(stage, f"/{name}/Physics")
    return stage


def add_link(
    stage,
    path: str,
    *,
    translate=(0.0, 0.0, 0.0),
    rotate_x: float | None = None,
    mass: float | None = None,
    diagonal_inertia=None,
    principal_axes=None,
    center_of_mass=None,
    box: tuple[float, float, float] | None = None,
):
    """A rigid body, optionally with a box collider of the given dimensions.

    The collider is authored the way ``urdf-usd-converter`` does it: a unit
    ``Cube`` with a non-uniform ``xformOp:scale``. Building it any other way
    would test a shape no converter actually emits.
    """
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if rotate_x is not None:
        xform.AddRotateXOp().Set(rotate_x)
    prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    if any(v is not None for v in (mass, diagonal_inertia, principal_axes, center_of_mass)):
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        if mass is not None:
            mass_api.CreateMassAttr().Set(mass)
        if diagonal_inertia is not None:
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*diagonal_inertia))
        if principal_axes is not None:
            real, i, j, k = principal_axes
            mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(real, Gf.Vec3f(i, j, k)))
        if center_of_mass is not None:
            mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*center_of_mass))
    if box is not None:
        cube = UsdGeom.Cube.Define(stage, f"{path}/box")
        cube.CreateSizeAttr().Set(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(*box))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return prim


def add_joint(
    stage,
    path: str,
    body0: str,
    body1: str,
    *,
    kind: str = "revolute",
    axis: str = "Z",
    local_pos0=(0.0, 0.0, 0.0),
    lower: float | None = None,
    upper: float | None = None,
    newton_damping: float | None = None,
    newton_friction: float | None = None,
    urdf_effort: float | None = None,
    newton_velocity_limit: float | None = None,
):
    """A revolute or prismatic joint wired to two bodies."""
    if kind == "revolute":
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    elif kind == "prismatic":
        joint = UsdPhysics.PrismaticJoint.Define(stage, path)
    else:
        joint = UsdPhysics.FixedJoint.Define(stage, path)
    prim = joint.GetPrim()
    prim.CreateRelationship("physics:body0", custom=False).SetTargets([Sdf.Path(body0)])
    prim.CreateRelationship("physics:body1", custom=False).SetTargets([Sdf.Path(body1)])
    if kind in ("revolute", "prismatic"):
        joint.CreateAxisAttr().Set(axis)
        if lower is not None:
            joint.CreateLowerLimitAttr().Set(lower)
        if upper is not None:
            joint.CreateUpperLimitAttr().Set(upper)
    prim.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Point3f, custom=False).Set(
        Gf.Vec3f(*local_pos0)
    )
    if newton_damping is not None:
        _custom(prim, "newton:damping", Sdf.ValueTypeNames.Float, newton_damping)
    if newton_friction is not None:
        _custom(prim, "newton:friction", Sdf.ValueTypeNames.Float, newton_friction)
    if urdf_effort is not None:
        _custom(prim, "urdf:limit:effort", Sdf.ValueTypeNames.Float, urdf_effort)
    if newton_velocity_limit is not None:
        _custom(prim, "newton:velocityLimit", Sdf.ValueTypeNames.Float, newton_velocity_limit)
    return prim


def simple_arm(stage=None, name: str = "robot"):
    """A two-body arm: a fixed base and one revolute link, with real inertia.

    Chosen so the equivalent inertia is hand-computable:
    the link is 0.5 m from a Z axis, mass 2 kg, ``Izz = 0.3``, so
    ``I_eq = 0.3 + 2.0 * 0.5**2 = 0.8`` kg*m^2 exactly.
    """
    stage = stage or new_asset(name)
    add_link(
        stage,
        f"/{name}/Geometry/base_link",
        mass=5.0,
        diagonal_inertia=(0.05, 0.05, 0.05),
        principal_axes=(1, 0, 0, 0),
        box=(0.2, 0.2, 0.1),
    )
    add_link(
        stage,
        f"/{name}/Geometry/base_link/arm_link",
        translate=(0.5, 0.0, 0.0),
        mass=2.0,
        diagonal_inertia=(0.1, 0.1, 0.3),
        principal_axes=(1, 0, 0, 0),
        box=(0.05, 0.05, 0.3),
    )
    add_joint(
        stage,
        f"/{name}/Physics/shoulder",
        f"/{name}/Geometry/base_link",
        f"/{name}/Geometry/base_link/arm_link",
        axis="Z",
        lower=-90.0,
        upper=90.0,
        newton_damping=0.02617993877991494,  # 1.5 N*m*s/rad, per degree
        newton_friction=0.3,
        urdf_effort=87.0,
    )
    return stage


def variant_asset(name: str = "robot"):
    """An asset whose physics sits behind a ``Physics`` variant set.

    Mirrors Isaac Sim 6.1.0: the variants exist, and **no selection is
    authored**, so the asset composes with no physics until a consumer picks
    one. The ``mujoco`` variant deletes ``PhysicsDriveAPI`` the way Isaac's
    asset-transformer profile does, which is the behaviour the variant-scoped
    authoring has to respect.
    """
    stage = new_asset(name)
    root = stage.GetDefaultPrim()
    vset = root.GetVariantSets().AddVariantSet("Physics")
    for variant in ("none", "physics", "physx", "mujoco"):
        vset.AddVariant(variant)

    vset.SetVariantSelection("physics")
    with vset.GetVariantEditContext():
        simple_arm(stage, name)

    vset.SetVariantSelection("physx")
    with vset.GetVariantEditContext():
        simple_arm(stage, name)

    vset.SetVariantSelection("mujoco")
    with vset.GetVariantEditContext():
        simple_arm(stage, name)
        joint = stage.GetPrimAtPath(f"/{name}/Physics/shoulder")
        # Isaac's "Delete Physics Drive and Joint State APIs" rule, in miniature.
        joint.RemoveAPI(UsdPhysics.DriveAPI, "angular")

    vset.ClearVariantSelection()
    return stage


def export(stage, path) -> str:
    stage.GetRootLayer().Export(str(path))
    return str(path)
