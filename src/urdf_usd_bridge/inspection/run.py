# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Read-only inspection of a converted robot asset, using ``pxr`` only.

Design rules for this module:

* It never authors, mutates or saves anything.
* It reads attributes *by name*, so an asset inspects correctly even when the
  ``physx``, ``mjc`` or ``newton`` schema plugins are not registered.
* It reports each producer's spelling of a quantity separately and never
  collapses them, because the whole G1 finding in ``docs/ANALYSIS.md`` is that
  two spellings of "damping" failed to meet.
* It distinguishes *absent* (no such attribute) from *present but unauthored*
  (schema fallback) from *authored*. That distinction is the evidence.
"""

from __future__ import annotations

import math
from typing import Any

from pxr import Usd, UsdGeom, UsdPhysics

from .._version import __version__
from ..model.stage import (
    describe_variant_sets,
    detect_layout,
    layer_identifiers,
    stage_metrics,
)
from ..usd import usd_version
from . import schemas as schema_names

_ABSENT = None


# --------------------------------------------------------------------------
# value plumbing
# --------------------------------------------------------------------------


def _jsonable(value: Any) -> Any:
    """Convert a USD value into something ``json`` can serialise."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, "GetReal") and hasattr(value, "GetImaginary"):  # Gf.Quat*
        imaginary = value.GetImaginary()
        return [float(value.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2])]
    if hasattr(value, "pathString"):  # Sdf.Path
        return value.pathString
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    try:  # Gf.Vec*, Vt arrays, and anything else iterable of scalars
        return [_jsonable(item) for item in value]
    except TypeError:
        return str(value)


def _read_attr(prim, name: str) -> dict[str, Any] | None:
    """Read one attribute.

    Returns:
        ``None`` when the attribute does not exist on the prim at all;
        otherwise ``{"value": ..., "authored": bool}``.
    """
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        return _ABSENT
    return {"value": _jsonable(attr.Get()), "authored": bool(attr.HasAuthoredValue())}


def _read_attrs(prim, names: dict[str, str]) -> dict[str, Any]:
    """Read a ``{key: attribute_name}`` table into ``{key: reading | None}``."""
    return {key: _read_attr(prim, name) for key, name in names.items()}


def _authored_value(reading: dict[str, Any] | None) -> Any:
    """The value of a reading, but only when it was actually authored."""
    if reading and reading.get("authored"):
        return reading.get("value")
    return None


def _rel_targets(prim, name: str) -> list[str] | None:
    rel = prim.GetRelationship(name)
    if not rel or not rel.IsValid():
        return _ABSENT
    return [target.pathString for target in rel.GetTargets()]


def _applied_schemas(prim) -> list[str]:
    """Applied API schema tokens, whether or not the schema is registered."""
    try:
        applied = list(prim.GetAppliedSchemas())
        if applied:
            return applied
    except Exception:  # pragma: no cover - defensive across USD versions
        pass
    listop = prim.GetMetadata("apiSchemas")
    if listop is None:
        return []
    try:
        return list(listop.GetAddedOrExplicitItems())
    except Exception:  # pragma: no cover
        return []


#: Which applied-schema base name owns each multi-apply property prefix.
_PREFIX_OWNER = {
    schema_names.DRIVE_PREFIX: "PhysicsDriveAPI",
    schema_names.LIMIT_PREFIX: "PhysicsLimitAPI",
}


def _instances_with_prefix(prim, prefix: str) -> list[str]:
    """Multi-apply instance names, e.g. ``angular`` from ``drive:angular:...``."""
    owner = _PREFIX_OWNER.get(prefix)
    instances: list[str] = []
    for schema in _applied_schemas(prim):
        if ":" not in schema:
            continue
        base, instance = schema.split(":", 1)
        if base == owner:
            instances.append(instance)
    if instances:
        return sorted(set(instances))
    # Fall back to scanning authored property names, for stages whose
    # apiSchemas metadata was stripped or never authored.
    found = set()
    for prop in prim.GetPropertyNames():
        if prop.startswith(prefix):
            parts = prop.split(":")
            if len(parts) >= 2:
                found.add(parts[1])
    return sorted(found)


# --------------------------------------------------------------------------
# inertia validity
# --------------------------------------------------------------------------


def _tensor_from_newton6(values: list[float]):
    """Build the 3x3 tensor from ``newton:inertia`` = [Ixx,Iyy,Izz,Ixy,Ixz,Iyz]."""
    import numpy as np

    ixx, iyy, izz, ixy, ixz, iyz = (float(v) for v in values)
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)


def _triangle_inequality_ok(principal: list[float], rel_tol: float = 1e-6) -> bool:
    """Whether principal moments can belong to a real rigid body.

    For any rigid body, each principal moment is at most the sum of the other
    two. Violating this is not a tolerance question -- no mass distribution
    produces such a tensor -- but a small relative slack absorbs float noise
    from eigendecomposition.
    """
    a, b, c = sorted(float(v) for v in principal)
    scale = max(abs(a), abs(b), abs(c), 1.0)
    return c <= (a + b) + rel_tol * scale


def _inertia_checks(mass: float | None, diagonal: list[float] | None, newton6: list[float] | None):
    """Validity flags for a body's mass properties.

    Every flag is ``True`` only when the condition is definitely wrong; unknown
    or absent data yields ``False`` plus an entry in ``notes``.
    """
    import numpy as np

    checks: dict[str, Any] = {
        "zero_inertia_with_mass": False,
        "mass_without_authored_inertia": False,
        "negative_diagonal_inertia": False,
        "diagonal_triangle_inequality_violated": False,
        "newton_inertia_not_positive_semidefinite": False,
        "newton_inertia_triangle_inequality_violated": False,
        "newton_inertia_disagrees_with_diagonal": False,
        "notes": [],
    }
    has_mass = mass is not None and mass > 0.0

    # A body with mass but no inertia authored anywhere is not "zero inertia" --
    # it is *undefined* inertia, which each backend resolves differently (PhysX
    # auto-computes from colliders, MuJoCo demands a positive-definite tensor).
    # The two cases need different repairs, so they are reported separately.
    if has_mass and diagonal is None and newton6 is None:
        checks["mass_without_authored_inertia"] = True

    if diagonal is not None:
        values = [float(v) for v in diagonal]
        if has_mass and all(abs(v) <= 0.0 for v in values):
            checks["zero_inertia_with_mass"] = True
        if any(v < 0.0 for v in values):
            checks["negative_diagonal_inertia"] = True
        if any(v > 0.0 for v in values) and not _triangle_inequality_ok(values):
            checks["diagonal_triangle_inequality_violated"] = True
    else:
        checks["notes"].append("no physics:diagonalInertia authored")

    if newton6 is not None and len(newton6) == 6:
        tensor = _tensor_from_newton6(newton6)
        eigenvalues = sorted(float(v) for v in np.linalg.eigvalsh(tensor))
        scale = max(abs(v) for v in eigenvalues) or 1.0
        if eigenvalues[0] < -1e-9 * scale:
            checks["newton_inertia_not_positive_semidefinite"] = True
        if any(v > 0.0 for v in eigenvalues) and not _triangle_inequality_ok(eigenvalues):
            checks["newton_inertia_triangle_inequality_violated"] = True
        checks["newton_principal_moments"] = eigenvalues
        if diagonal is not None:
            authored = sorted(float(v) for v in diagonal)
            if any(
                not math.isclose(a, b, rel_tol=1e-4, abs_tol=1e-12) for a, b in zip(authored, eigenvalues)
            ):
                checks["newton_inertia_disagrees_with_diagonal"] = True
    elif newton6 is not None:
        checks["notes"].append(f"newton:inertia has {len(newton6)} components, expected 6")

    return checks


def _quaternion_validity(quat: list[float] | None) -> dict[str, Any]:
    """Whether an authored ``physics:principalAxes`` is a usable rotation.

    ``urdf-usd-converter`` 0.3.2 can author an all-zero quaternion when
    ``<inertial>`` carries an origin and a mass but no ``<inertia>``; 0.3.3
    fixed it. Isaac Sim 6.1.0 pins 0.3.2, so this check fires on its output.
    """
    if quat is None:
        return {"present": False, "is_zero": False, "is_normalized": None, "norm": None}
    norm = math.sqrt(sum(float(v) * float(v) for v in quat))
    return {
        "present": True,
        "is_zero": norm <= 1e-12,
        "is_normalized": bool(abs(norm - 1.0) <= 1e-4),
        "norm": norm,
    }


# --------------------------------------------------------------------------
# per-prim inspection
# --------------------------------------------------------------------------


def _inspect_body(prim) -> dict[str, Any]:
    mass_readings = _read_attrs(prim, schema_names.MASS_ATTRS)
    newton_inertia = _read_attr(prim, schema_names.NEWTON_INERTIA_ATTR)

    mass = _authored_value(mass_readings["mass"])
    diagonal = _authored_value(mass_readings["diagonal_inertia"])
    principal_axes = _authored_value(mass_readings["principal_axes"])
    newton6 = _authored_value(newton_inertia)

    return {
        "path": prim.GetPath().pathString,
        "name": prim.GetName(),
        "type_name": prim.GetTypeName(),
        "applied_schemas": _applied_schemas(prim),
        "mass": mass_readings,
        "newton_inertia": newton_inertia,
        "principal_axes_validity": _quaternion_validity(principal_axes),
        "inertia_checks": _inertia_checks(mass, diagonal, newton6),
    }


def _inspect_joint(prim) -> dict[str, Any]:
    type_name = str(prim.GetTypeName())
    is_angular = type_name in schema_names.ANGULAR_JOINT_TYPE_NAMES
    is_linear = type_name in schema_names.LINEAR_JOINT_TYPE_NAMES

    drives: dict[str, Any] = {}
    for instance in _instances_with_prefix(prim, schema_names.DRIVE_PREFIX):
        drives[instance] = {
            suffix.split(":")[-1]: _read_attr(prim, f"{schema_names.DRIVE_PREFIX}{instance}:{suffix}")
            for suffix in schema_names.DRIVE_SUFFIXES
        }

    limit_apis: dict[str, Any] = {}
    for instance in _instances_with_prefix(prim, schema_names.LIMIT_PREFIX):
        limit_apis[instance] = {
            "low": _read_attr(prim, f"{schema_names.LIMIT_PREFIX}{instance}:physics:low"),
            "high": _read_attr(prim, f"{schema_names.LIMIT_PREFIX}{instance}:physics:high"),
        }

    lower = _read_attr(prim, "physics:lowerLimit")
    upper = _read_attr(prim, "physics:upperLimit")

    joint: dict[str, Any] = {
        "path": prim.GetPath().pathString,
        "name": prim.GetName(),
        "type_name": type_name,
        "dof": "angular" if is_angular else ("linear" if is_linear else None),
        "applied_schemas": _applied_schemas(prim),
        "body0": _rel_targets(prim, "physics:body0"),
        "body1": _rel_targets(prim, "physics:body1"),
        "axis": _read_attr(prim, "physics:axis"),
        "joint_enabled": _read_attr(prim, "physics:jointEnabled"),
        "exclude_from_articulation": _read_attr(prim, "physics:excludeFromArticulation"),
        "limits": {"lower": lower, "upper": upper, "limit_apis": limit_apis},
        "drives": drives,
        "damping_by_namespace": _read_attrs(prim, schema_names.JOINT_DAMPING_ATTRS),
        "friction_by_namespace": _read_attrs(prim, schema_names.JOINT_FRICTION_ATTRS),
        "armature_by_namespace": _read_attrs(prim, schema_names.JOINT_ARMATURE_ATTRS),
        "limit_extras": _read_attrs(prim, schema_names.JOINT_LIMIT_EXTRA_ATTRS),
        "urdf_custom": {name: _read_attr(prim, name) for name in schema_names.URDF_CUSTOM_ATTRS},
        "mimic": {
            **_read_attrs(prim, schema_names.MIMIC_ATTRS),
            "newton_joint_targets": _rel_targets(prim, schema_names.MIMIC_RELS["newton_joint"]),
        },
    }

    # Derived flags: these are observations, not judgements. Each maps to a
    # gap identified in docs/ANALYSIS.md.
    lower_value = _authored_value(lower)
    upper_value = _authored_value(upper)
    locked = (
        (is_angular or is_linear)
        and lower_value is not None
        and upper_value is not None
        and lower_value == upper_value
    )

    drive_stiffness = [_authored_value(d.get("stiffness")) for d in drives.values() if d.get("stiffness")]
    drive_damping = [_authored_value(d.get("damping")) for d in drives.values() if d.get("damping")]
    any_stiffness = any(v for v in drive_stiffness if v)
    any_drive_damping = any(v for v in drive_damping if v)

    damping_namespaces = [
        key
        for key, reading in joint["damping_by_namespace"].items()
        if _authored_value(reading) not in (None, 0.0)
    ]
    friction_namespaces = [
        key
        for key, reading in joint["friction_by_namespace"].items()
        if _authored_value(reading) not in (None, 0.0)
    ]
    armature_namespaces = [
        key
        for key, reading in joint["armature_by_namespace"].items()
        if _authored_value(reading) not in (None, 0.0)
    ]

    joint["flags"] = {
        "actuatable": bool(is_angular or is_linear),
        "has_drive_api": bool(drives),
        "drive_has_gains": bool(any_stiffness or any_drive_damping),
        "drive_applied_without_gains": bool(drives) and not (any_stiffness or any_drive_damping),
        "damping_namespaces": damping_namespaces,
        "friction_namespaces": friction_namespaces,
        "armature_namespaces": armature_namespaces,
        # G1: the joint carries damping somewhere, but not where a UsdPhysics
        # drive consumer will look for it.
        "damping_stranded_outside_drive": bool(damping_namespaces) and not any_drive_damping,
        "no_armature_anywhere": not armature_namespaces,
        # G7: revolute/prismatic joint pinned shut by lower == upper.
        "locked_by_equal_limits": bool(locked),
        "effort_only_as_urdf_custom": bool(
            _authored_value(joint["limit_extras"]["urdf_effort"]) is not None
            and not any(_authored_value(d.get("maxForce")) for d in drives.values() if d.get("maxForce"))
        ),
    }
    return joint


def _inspect_collider(prim) -> dict[str, Any]:
    imageable = UsdGeom.Imageable(prim)
    purpose = imageable.GetPurposeAttr().Get() if imageable else None
    return {
        "path": prim.GetPath().pathString,
        "type_name": str(prim.GetTypeName()),
        "purpose": _jsonable(purpose),
        "applied_schemas": _applied_schemas(prim),
        "collision": _read_attrs(prim, schema_names.COLLISION_ATTRS),
        "filtered_pairs": _rel_targets(prim, schema_names.FILTERED_PAIRS_REL),
        "physics_material_binding": _rel_targets(prim, schema_names.PHYSICS_MATERIAL_BINDING_REL),
    }


def _inspect_actuator(prim) -> dict[str, Any]:
    attrs = {name: _read_attr(prim, name) for name in schema_names.MJC_ACTUATOR_ATTRS}
    gain_prm = _authored_value(attrs.get("mjc:gainPrm"))
    bias_prm = _authored_value(attrs.get("mjc:biasPrm"))
    return {
        "path": prim.GetPath().pathString,
        "type_name": str(prim.GetTypeName()),
        "target": _rel_targets(prim, schema_names.MJC_ACTUATOR_TARGET_REL),
        "attrs": attrs,
        "flags": {
            # G1: Isaac emits an MjcActuator with no gain parameters whenever
            # the PhysX drive it derives from has zero stiffness and damping.
            "has_gain_parameters": bool(gain_prm)
            or bool(bias_prm),
        },
    }


# --------------------------------------------------------------------------
# traversal
# --------------------------------------------------------------------------


def _schema_fallbacks() -> dict[str, Any]:
    """Record the schema fallbacks this USD build reports.

    Written into the report so a reader can tell an authored zero from a
    fallback without having to know the schema by heart.
    """
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/FallbackProbe", "Xform")
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    return {
        "physics:mass": _jsonable(mass_api.GetMassAttr().Get()),
        "physics:density": _jsonable(mass_api.GetDensityAttr().Get()),
        "physics:centerOfMass": _jsonable(mass_api.GetCenterOfMassAttr().Get()),
        "physics:diagonalInertia": _jsonable(mass_api.GetDiagonalInertiaAttr().Get()),
        "physics:principalAxes": _jsonable(mass_api.GetPrincipalAxesAttr().Get()),
    }


def inspect_stage(
    identifier: str,
    variant_selections: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Inspect a converted robot asset and return a JSON-serialisable report.

    Args:
        identifier: Path to the root layer of the asset.
        variant_selections: Variant selections to apply before reading, e.g.
            ``{"Physics": "physx"}`` for an Isaac Sim 6.x package.

    Returns:
        A report dict; see ``docs/`` and ``SCHEMA_VERSION`` for the contract.
    """
    from ..model.stage import open_stage

    stage = open_stage(identifier, variant_selections)
    default_prim = stage.GetDefaultPrim()

    bodies: list[dict[str, Any]] = []
    joints: list[dict[str, Any]] = []
    colliders: list[dict[str, Any]] = []
    actuators: list[dict[str, Any]] = []
    materials: list[dict[str, Any]] = []
    collision_groups: list[str] = []
    articulation_roots: list[dict[str, Any]] = []
    scenes: list[dict[str, Any]] = []

    for prim in stage.TraverseAll():
        applied = set(_applied_schemas(prim))
        type_name = str(prim.GetTypeName())

        if "PhysicsRigidBodyAPI" in applied:
            bodies.append(_inspect_body(prim))
        if "PhysicsArticulationRootAPI" in applied:
            articulation_roots.append(
                {
                    "path": prim.GetPath().pathString,
                    "applied_schemas": sorted(applied),
                    "self_collision": _read_attrs(prim, schema_names.SELF_COLLISION_ATTRS),
                }
            )
        if type_name in schema_names.JOINT_TYPE_NAMES:
            joints.append(_inspect_joint(prim))
        if "PhysicsCollisionAPI" in applied:
            colliders.append(_inspect_collider(prim))
        if type_name == schema_names.MJC_ACTUATOR_TYPE:
            actuators.append(_inspect_actuator(prim))
        if type_name == "PhysicsCollisionGroup":
            collision_groups.append(prim.GetPath().pathString)
        if "PhysicsMaterialAPI" in applied:
            materials.append(
                {
                    "path": prim.GetPath().pathString,
                    "attrs": _read_attrs(prim, schema_names.MATERIAL_ATTRS),
                }
            )
        if type_name == "PhysicsScene":
            scenes.append(
                {
                    "path": prim.GetPath().pathString,
                    "applied_schemas": sorted(applied),
                    "attrs": _read_attrs(prim, schema_names.SCENE_ATTRS),
                }
            )

    report = {
        "schema_version": schema_names.SCHEMA_VERSION,
        "tool": {
            "name": "urdf-usd-bridge",
            "version": __version__,
            "usd_version": usd_version(),
        },
        "input": {
            "identifier": identifier,
            "variant_selections": dict(variant_selections or {}),
        },
        "stage": {
            "default_prim": default_prim.GetPath().pathString if default_prim else None,
            "layout": detect_layout(stage),
            "metrics": stage_metrics(stage),
            "layers": layer_identifiers(stage),
        },
        "variant_sets": describe_variant_sets(default_prim),
        "schema_fallbacks": _schema_fallbacks(),
        "articulation_roots": articulation_roots,
        "physics_scenes": scenes,
        "bodies": bodies,
        "joints": joints,
        "colliders": colliders,
        "physics_materials": materials,
        "collision_groups": collision_groups,
        "mjc_actuators": actuators,
    }
    report["summary"] = summarize(report)
    return report


def summarize(report: dict[str, Any]) -> dict[str, Any]:
    """Counters that map one-to-one onto the gaps in ``docs/ANALYSIS.md``."""
    joints = report["joints"]
    bodies = report["bodies"]
    colliders = report["colliders"]
    actuators = report["mjc_actuators"]

    actuatable = [j for j in joints if j["flags"]["actuatable"]]
    masses = [
        _authored_value(b["mass"]["mass"])
        for b in bodies
        if _authored_value(b["mass"]["mass"]) not in (None, 0.0)
    ]

    approximations: dict[str, int] = {}
    for collider in colliders:
        reading = collider["collision"].get("approximation")
        key = str(_authored_value(reading)) if _authored_value(reading) else "(unauthored)"
        approximations[key] = approximations.get(key, 0) + 1

    damping_by_ns: dict[str, int] = {}
    friction_by_ns: dict[str, int] = {}
    armature_by_ns: dict[str, int] = {}
    for joint in actuatable:
        for key in joint["flags"]["damping_namespaces"]:
            damping_by_ns[key] = damping_by_ns.get(key, 0) + 1
        for key in joint["flags"]["friction_namespaces"]:
            friction_by_ns[key] = friction_by_ns.get(key, 0) + 1
        for key in joint["flags"]["armature_namespaces"]:
            armature_by_ns[key] = armature_by_ns.get(key, 0) + 1

    physics_variant = next((v for v in report["variant_sets"] if v["name"].lower() == "physics"), None)

    return {
        "layout": report["stage"]["layout"],
        "bodies_total": len(bodies),
        "bodies_with_authored_mass": len(masses),
        "bodies_zero_inertia_with_mass": sum(
            1 for b in bodies if b["inertia_checks"]["zero_inertia_with_mass"]
        ),
        "bodies_mass_without_authored_inertia": sum(
            1 for b in bodies if b["inertia_checks"]["mass_without_authored_inertia"]
        ),
        "bodies_negative_inertia": sum(1 for b in bodies if b["inertia_checks"]["negative_diagonal_inertia"]),
        "bodies_triangle_inequality_violated": sum(
            1
            for b in bodies
            if b["inertia_checks"]["diagonal_triangle_inequality_violated"]
            or b["inertia_checks"]["newton_inertia_triangle_inequality_violated"]
        ),
        "bodies_invalid_principal_axes": sum(
            1 for b in bodies if b["principal_axes_validity"].get("is_zero")
        ),
        "mass_ratio_max": (max(masses) / min(masses)) if len(masses) >= 2 else None,
        "joints_total": len(joints),
        "joints_actuatable": len(actuatable),
        "joints_with_drive_api": sum(1 for j in actuatable if j["flags"]["has_drive_api"]),
        "joints_with_drive_gains": sum(1 for j in actuatable if j["flags"]["drive_has_gains"]),
        "joints_drive_applied_without_gains": sum(
            1 for j in actuatable if j["flags"]["drive_applied_without_gains"]
        ),
        "joints_damping_stranded_outside_drive": sum(
            1 for j in actuatable if j["flags"]["damping_stranded_outside_drive"]
        ),
        "joints_without_armature": sum(1 for j in actuatable if j["flags"]["no_armature_anywhere"]),
        "joints_locked_by_equal_limits": sum(1 for j in actuatable if j["flags"]["locked_by_equal_limits"]),
        "joints_effort_only_as_urdf_custom": sum(
            1 for j in actuatable if j["flags"]["effort_only_as_urdf_custom"]
        ),
        "damping_authored_by_namespace": damping_by_ns,
        "friction_authored_by_namespace": friction_by_ns,
        "armature_authored_by_namespace": armature_by_ns,
        "colliders_total": len(colliders),
        "collider_approximations": approximations,
        "colliders_with_filtered_pairs": sum(1 for c in colliders if c["filtered_pairs"]),
        "collision_groups_total": len(report["collision_groups"]),
        "physics_materials_total": len(report["physics_materials"]),
        "colliders_with_physics_material": sum(1 for c in colliders if c["physics_material_binding"]),
        "physics_scenes_total": len(report["physics_scenes"]),
        "articulation_roots_total": len(report["articulation_roots"]),
        "mjc_actuators_total": len(actuators),
        "mjc_actuators_without_gains": sum(1 for a in actuators if not a["flags"]["has_gain_parameters"]),
        "physics_variant_set": physics_variant,
    }
