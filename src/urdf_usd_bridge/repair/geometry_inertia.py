# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Inertia tensors derived from collision geometry.

Used by ``inertia.derive-from-geometry`` when a body has a mass but no usable
inertia tensor. Each shape contributes its analytic tensor about its own
centroid; the contributions are then moved to the body's combined centre of
mass by the parallel-axis theorem and summed.

Mass is distributed across a body's colliders **by volume**, and the total is
rescaled to the mass the input authored. We never invent a mass here -- this
module only decides how an existing one is distributed in space.

Meshes use their oriented bounding box rather than the true volume integral.
That is a deliberate approximation: it is stable, cheap, needs no watertightness
assumption, and it errs towards *more* inertia, which is the direction that
makes a solver more stable rather than less. It is recorded as such in the
repair record so nobody mistakes it for an exact tensor.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from pxr import Gf, Usd, UsdGeom

#: Axis token -> index, for the shapes that have an axis.
_AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}


def _axis_aligned(axis: str, along: float, perpendicular_i: float, perpendicular_j: float) -> np.ndarray:
    """A diagonal tensor with ``along`` on the shape's axis."""
    index = _AXIS_INDEX.get(axis, 2)
    others = [i for i in range(3) if i != index]
    values = [0.0, 0.0, 0.0]
    values[index] = along
    values[others[0]] = perpendicular_i
    values[others[1]] = perpendicular_j
    return np.diag(values)


def unit_shape(prim, scale: np.ndarray) -> tuple[float, np.ndarray] | None:
    """``(volume, inertia_per_unit_mass)`` for a Gprim, in its own local frame.

    ``scale`` is the per-axis scale from the Gprim's transform, applied to the
    shape's dimensions rather than to the finished tensor. This matters because
    it is the *normal* case, not an edge case: ``urdf-usd-converter`` authors a
    unit ``Cube`` with ``xformOp:scale = (0.1, 0.1, 0.2)`` for
    ``<box size="0.1 0.1 0.2"/>``, so collapsing the scale to a single number
    would turn every box in every converted robot into a cube.

    Exact for cubes, ellipsoids and mesh bounding boxes. Cylinders, capsules and
    cones become their elliptic generalisations, which is exact for a uniform
    radial scale and a good approximation otherwise.

    Returns ``None`` for geometry we have no analytic form for.
    """
    type_name = str(prim.GetTypeName())
    sx, sy, sz = (abs(float(v)) for v in scale)

    if type_name == "Cube":
        size = float(UsdGeom.Cube(prim).GetSizeAttr().Get() or 2.0)
        dx, dy, dz = size * sx, size * sy, size * sz
        volume = dx * dy * dz
        return volume, np.diag(
            [(dy * dy + dz * dz) / 12.0, (dx * dx + dz * dz) / 12.0, (dx * dx + dy * dy) / 12.0]
        )

    if type_name == "Sphere":
        radius = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get() or 1.0)
        a, b, c = radius * sx, radius * sy, radius * sz
        volume = 4.0 / 3.0 * math.pi * a * b * c
        return volume, np.diag([(b * b + c * c) / 5.0, (a * a + c * c) / 5.0, (a * a + b * b) / 5.0])

    if type_name in ("Cylinder", "Capsule", "Cone"):
        return _round_shape(prim, type_name, (sx, sy, sz))

    if type_name == "Mesh":
        points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if points is None or len(points) == 0:
            return None
        array = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in points], dtype=np.float64)
        extent = (array.max(axis=0) - array.min(axis=0)) * np.array([sx, sy, sz])
        extent = np.maximum(extent, 1e-9)
        volume = float(extent[0] * extent[1] * extent[2])
        x, y, z = extent
        return volume, np.diag([(y * y + z * z) / 12.0, (x * x + z * z) / 12.0, (x * x + y * y) / 12.0])

    return None


def _round_shape(prim, type_name: str, scale: tuple[float, float, float]):
    """Cylinder, capsule and cone, generalised to elliptic cross-sections."""
    if type_name == "Cylinder":
        shape = UsdGeom.Cylinder(prim)
    elif type_name == "Capsule":
        shape = UsdGeom.Capsule(prim)
    else:
        shape = UsdGeom.Cone(prim)

    radius = float(shape.GetRadiusAttr().Get() or 1.0)
    height = float(shape.GetHeightAttr().Get() or 2.0)
    axis = str(shape.GetAxisAttr().Get() or "Z")
    index = _AXIS_INDEX.get(axis, 2)
    others = [i for i in range(3) if i != index]

    a = radius * scale[others[0]]
    b = radius * scale[others[1]]
    length = height * scale[index]

    if type_name == "Cylinder":
        volume = math.pi * a * b * length
        along = (a * a + b * b) / 4.0
        perp_i = (3.0 * b * b + length * length) / 12.0
        perp_j = (3.0 * a * a + length * length) / 12.0
        return volume, _axis_aligned(axis, along, perp_i, perp_j)

    if type_name == "Cone":
        volume = math.pi * a * b * length / 3.0
        along = 3.0 * (a * a + b * b) / 20.0
        perp_i = 3.0 * b * b / 20.0 + 3.0 * length * length / 80.0
        perp_j = 3.0 * a * a / 20.0 + 3.0 * length * length / 80.0
        return volume, _axis_aligned(axis, along, perp_i, perp_j)

    # Capsule: a cylinder plus two hemispherical caps, combined by mass fraction.
    radial = math.sqrt(max(a * b, 1e-18))
    cyl_volume = math.pi * a * b * length
    cap_volume = 4.0 / 3.0 * math.pi * a * b * radial
    volume = cyl_volume + cap_volume
    if volume <= 0:  # pragma: no cover - degenerate authoring
        return None
    cyl_fraction = cyl_volume / volume
    cap_fraction = cap_volume / volume
    along = cyl_fraction * (a * a + b * b) / 4.0 + cap_fraction * 0.4 * radial * radial
    offset = length / 2.0 + 3.0 * radial / 8.0
    perp = cyl_fraction * (3.0 * radial * radial + length * length) / 12.0 + cap_fraction * (
        0.4 * radial * radial + offset * offset
    )
    return volume, _axis_aligned(axis, along, perp, perp)


def _local_centroid(prim) -> Gf.Vec3d:
    """Centroid of a Gprim in its own local frame.

    Every analytic shape above is centred on its origin; a mesh is not, so its
    bounding-box centre is used.
    """
    if str(prim.GetTypeName()) != "Mesh":
        return Gf.Vec3d(0, 0, 0)
    points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
    if points is None or len(points) == 0:  # pragma: no cover - guarded upstream
        return Gf.Vec3d(0, 0, 0)
    array = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in points], dtype=np.float64)
    centre = (array.max(axis=0) + array.min(axis=0)) / 2.0
    return Gf.Vec3d(float(centre[0]), float(centre[1]), float(centre[2]))


def _parallel_axis(tensor: np.ndarray, mass: float, offset: np.ndarray) -> np.ndarray:
    """Move a tensor from a body's own centroid to a point ``offset`` away."""
    d2 = float(offset @ offset)
    return tensor + mass * (d2 * np.eye(3) - np.outer(offset, offset))


def collider_prims(body_prim) -> list:
    """Collision geometry belonging to a body, in sorted path order.

    A nested rigid body's colliders belong to that body, not to its ancestor,
    so the walk stops at any descendant that is itself a rigid body.
    """
    found = []
    # A manual walk, because `continue` inside a Usd.PrimRange loop skips one
    # prim but keeps descending into it -- which silently pulled a child body's
    # colliders into its parent's tensor.
    stack = [body_prim]
    while stack:
        prim = stack.pop()
        if prim != body_prim and "PhysicsRigidBodyAPI" in set(prim.GetAppliedSchemas()):
            continue  # a nested body owns its own colliders
        if "PhysicsCollisionAPI" in set(prim.GetAppliedSchemas()):
            found.append(prim)
        stack.extend(prim.GetChildren())
    return sorted(found, key=lambda p: p.GetPath().pathString)


def inertia_from_geometry(
    body_prim, mass: float, *, scale: float = 1.0, center_of_mass=None
) -> dict[str, Any] | None:
    """Derive a body-frame inertia tensor from a body's collision geometry.

    Args:
        body_prim: The prim carrying ``PhysicsRigidBodyAPI``.
        mass: The body's authored mass, in kilograms. The result is scaled so
            the tensor corresponds to exactly this mass.
        scale: Stage ``metersPerUnit``, so the tensor comes out in kg*m^2.
        center_of_mass: The body's authored ``physics:centerOfMass``, in stage
            units. ``physics:diagonalInertia`` is defined *about the centre of
            mass*, so when the input declares one that disagrees with where the
            geometry actually is, the tensor is moved to the declared point by
            the parallel-axis theorem. That adds inertia, which is the
            conservative direction, and the offset is recorded.

    Returns:
        ``{"tensor": 3x3 ndarray, "center_of_mass": [x, y, z], "sources": [...]}``
        in the body's local frame, or ``None`` when there is no usable geometry.
    """
    colliders = collider_prims(body_prim)
    if not colliders:
        return None

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    body_to_world = cache.GetLocalToWorldTransform(body_prim)
    world_to_body = body_to_world.GetInverse()

    entries: list[dict[str, Any]] = []
    total_volume = 0.0
    for collider in colliders:
        collider_to_body = cache.GetLocalToWorldTransform(collider) * world_to_body

        # Separate scale from rotation: the scale belongs to the shape's
        # dimensions, the rotation to the tensor.
        rows = [Gf.Vec3d(collider_to_body.GetRow3(i)) for i in range(3)]
        local_scale = np.array([row.GetLength() for row in rows], dtype=np.float64)
        shape = unit_shape(collider, local_scale)
        if shape is None:
            continue
        volume_local, tensor_local = shape

        rotation_rows = [
            (
                (rows[i] / local_scale[i])
                if local_scale[i] > 1e-12
                else Gf.Vec3d(*(1.0 if j == i else 0.0 for j in range(3)))
            )
            for i in range(3)
        ]
        # Gf matrices are row-vector (v' = v * M), so the column-vector rotation
        # is the transpose of the row stack.
        rotation = np.array([[rotation_rows[i][j] for j in range(3)] for i in range(3)], dtype=np.float64).T

        volume = volume_local * (scale**3)
        tensor_unit = rotation @ (tensor_local * (scale**2)) @ rotation.T
        centroid_body = collider_to_body.Transform(_local_centroid(collider))

        entries.append(
            {
                "prim": collider.GetPath().pathString,
                "type": str(collider.GetTypeName()),
                "volume": volume,
                "tensor_unit_mass": tensor_unit,
                "centroid": np.array(
                    [centroid_body[0] * scale, centroid_body[1] * scale, centroid_body[2] * scale],
                    dtype=np.float64,
                ),
            }
        )
        total_volume += volume

    if not entries or total_volume <= 0:
        return None

    centre = sum(entry["centroid"] * entry["volume"] for entry in entries) / total_volume
    tensor = np.zeros((3, 3), dtype=np.float64)
    for entry in entries:
        part_mass = mass * entry["volume"] / total_volume
        part = entry["tensor_unit_mass"] * part_mass
        tensor += _parallel_axis(part, part_mass, entry["centroid"] - centre)

    com_offset = None
    if center_of_mass is not None:
        declared = np.array(
            [
                float(center_of_mass[0]) * scale,
                float(center_of_mass[1]) * scale,
                float(center_of_mass[2]) * scale,
            ],
            dtype=np.float64,
        )
        shift = declared - centre
        if float(shift @ shift) > 1e-18:
            tensor = _parallel_axis(tensor, mass, shift)
            com_offset = [float(v) for v in shift]

    return {
        "tensor": tensor,
        "center_of_mass": [float(v) for v in centre],
        "com_offset": com_offset,
        "sources": [
            {"prim": entry["prim"], "type": entry["type"], "volume": entry["volume"]} for entry in entries
        ],
    }


def canonical_eigendecomposition(tensor: np.ndarray) -> tuple[list[float], list[float]]:
    """``(diagonal, quaternion)`` with ``I = R diag R^T``, deterministically.

    ``numpy.linalg.eigh`` fixes the eigenvalue order (ascending) but leaves each
    eigenvector's sign free, and for degenerate eigenvalues the basis within the
    degenerate subspace is arbitrary -- both can differ between BLAS builds. We
    therefore force each eigenvector's largest-magnitude component positive and
    make the basis right-handed, so the same tensor always produces the same
    quaternion.

    The ``I = R diag R^T`` convention and the row-vector transpose match
    ``urdf-usd-converter``'s ``_extract_inertia``, so a repaired tensor is
    directly comparable with a converter-authored one.
    """
    symmetric = (tensor + tensor.T) / 2.0
    eigenvalues, eigenvectors = np.linalg.eigh(symmetric)

    for column in range(3):
        vector = eigenvectors[:, column]
        dominant = int(np.argmax(np.abs(vector)))
        if vector[dominant] < 0:
            eigenvectors[:, column] = -vector
    if np.linalg.det(eigenvectors) < 0:
        eigenvectors[:, 2] = -eigenvectors[:, 2]

    matrix = Gf.Matrix3d(
        float(eigenvectors[0, 0]), float(eigenvectors[1, 0]), float(eigenvectors[2, 0]),
        float(eigenvectors[0, 1]), float(eigenvectors[1, 1]), float(eigenvectors[2, 1]),
        float(eigenvectors[0, 2]), float(eigenvectors[1, 2]), float(eigenvectors[2, 2]),
    )  # fmt: skip
    quat = matrix.ExtractRotation().GetQuat()
    imaginary = quat.GetImaginary()
    return (
        [float(v) for v in eigenvalues],
        [float(quat.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2])],
    )


def newton_six_vector(tensor: np.ndarray) -> list[float]:
    """``[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]``, the layout ``newton:inertia`` uses."""
    return [
        float(tensor[0, 0]),
        float(tensor[1, 1]),
        float(tensor[2, 2]),
        float(tensor[0, 1]),
        float(tensor[0, 2]),
        float(tensor[1, 2]),
    ]
