# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Inertia rules -- G2.

The derived tensors are checked against closed-form answers, not against
whatever the code happens to produce, so a regression in the shape maths cannot
be "fixed" by updating the expected value.
"""

from __future__ import annotations

import numpy as np
import pytest
from pxr import Gf, UsdGeom, UsdPhysics

from urdf_usd_bridge.repair import RepairOptions, analyse, fix_asset
from urdf_usd_bridge.repair.geometry_inertia import (
    canonical_eigendecomposition,
    collider_prims,
    inertia_from_geometry,
    unit_shape,
)
from urdf_usd_bridge.repair.inertia import _make_physical

from .repair_builders import add_joint, add_link, export, new_asset


def _records(report, rule, status="applied"):
    return [r for r in report["records"] if r["rule"] == rule and r["status"] == status]


def _record_for(report, prim_suffix, attribute):
    return next(
        r
        for r in report["records"]
        if r["prim"].endswith(prim_suffix) and r["attribute"] == attribute and r["status"] == "applied"
    )


# --- shape maths -----------------------------------------------------------


def test_box_inertia_matches_the_closed_form():
    """A unit Cube with a non-uniform scale is what the converter emits.

    Collapsing that scale to one number turns every box into a cube, which is
    what the first version of this code did.
    """
    stage = new_asset()
    cube = UsdGeom.Cube.Define(stage, "/robot/box")
    cube.CreateSizeAttr().Set(1.0)
    volume, tensor = unit_shape(cube.GetPrim(), np.array([0.1, 0.1, 0.2]))

    assert volume == pytest.approx(0.1 * 0.1 * 0.2)
    dx, dy, dz = 0.1, 0.1, 0.2
    assert tensor[0][0] == pytest.approx((dy**2 + dz**2) / 12.0)
    assert tensor[1][1] == pytest.approx((dx**2 + dz**2) / 12.0)
    assert tensor[2][2] == pytest.approx((dx**2 + dy**2) / 12.0)
    # The three moments must not be equal: that is the bug this guards.
    assert tensor[0][0] != pytest.approx(tensor[2][2])


def test_sphere_and_cylinder_match_the_closed_form():
    stage = new_asset()
    sphere = UsdGeom.Sphere.Define(stage, "/robot/s")
    sphere.CreateRadiusAttr().Set(0.5)
    volume, tensor = unit_shape(sphere.GetPrim(), np.array([1.0, 1.0, 1.0]))
    assert volume == pytest.approx(4.0 / 3.0 * np.pi * 0.125)
    assert tensor[0][0] == pytest.approx(0.4 * 0.25)

    cylinder = UsdGeom.Cylinder.Define(stage, "/robot/c")
    cylinder.CreateRadiusAttr().Set(0.2)
    cylinder.CreateHeightAttr().Set(1.0)
    cylinder.CreateAxisAttr().Set("Z")
    volume, tensor = unit_shape(cylinder.GetPrim(), np.array([1.0, 1.0, 1.0]))
    assert volume == pytest.approx(np.pi * 0.04 * 1.0)
    assert tensor[2][2] == pytest.approx(0.5 * 0.04)
    assert tensor[0][0] == pytest.approx((3 * 0.04 + 1.0) / 12.0)


def test_canonical_eigendecomposition_reconstructs_the_tensor():
    """``I = R diag R^T`` must hold, and must be stable run to run.

    ``Gf`` matrices are row-vector (``v' = v * M``), so the column-vector
    rotation is the transpose. Getting that backwards is the mistake the
    converter's own ``_extract_inertia`` comments warn about.
    """
    tensor = np.array([[0.5, 0.1, 0.02], [0.1, 0.4, 0.03], [0.02, 0.03, 0.7]])
    diagonal, quat = canonical_eigendecomposition(tensor)

    row_vector = Gf.Matrix3d(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
    rotation = np.array([[row_vector[i][j] for j in range(3)] for i in range(3)]).T
    assert np.allclose(rotation @ np.diag(diagonal) @ rotation.T, tensor, atol=1e-12)

    # Eigenvalues are the tensor's, and the result is reproducible.
    assert sorted(diagonal) == pytest.approx(sorted(np.linalg.eigvalsh(tensor)))
    assert canonical_eigendecomposition(tensor) == (diagonal, quat)


def test_eigendecomposition_is_stable_for_a_degenerate_tensor():
    """Equal moments leave the eigenvector basis free; we pin it."""
    isotropic = np.diag([0.25, 0.25, 0.25])
    first = canonical_eigendecomposition(isotropic)
    assert canonical_eigendecomposition(isotropic) == first
    assert first[0] == pytest.approx([0.25, 0.25, 0.25])


def test_collider_walk_stops_at_a_nested_rigid_body():
    """A child body owns its own colliders.

    `continue` inside a ``Usd.PrimRange`` loop skips one prim but keeps
    descending, which silently pulled a child's box into its parent's tensor.
    """
    stage = new_asset()
    add_link(stage, "/robot/Geometry/parent", mass=1.0, box=(0.1, 0.1, 0.1))
    add_link(stage, "/robot/Geometry/parent/child", mass=1.0, box=(0.2, 0.2, 0.2))
    parent = stage.GetPrimAtPath("/robot/Geometry/parent")
    found = [p.GetPath().pathString for p in collider_prims(parent)]
    assert found == ["/robot/Geometry/parent/box"]


def test_mass_is_distributed_by_volume_across_several_colliders():
    stage = new_asset()
    link = add_link(stage, "/robot/Geometry/l", mass=3.0, box=(0.2, 0.2, 0.2))
    second = UsdGeom.Cube.Define(stage, "/robot/Geometry/l/box2")
    second.CreateSizeAttr().Set(1.0)
    second.AddTranslateOp().Set(Gf.Vec3d(1.0, 0, 0))
    second.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.2))
    UsdPhysics.CollisionAPI.Apply(second.GetPrim())

    derived = inertia_from_geometry(link, 3.0)
    assert len(derived["sources"]) == 2
    # Equal volumes, so the combined centre of mass sits halfway between them.
    assert derived["center_of_mass"][0] == pytest.approx(0.5)


# --- rules -----------------------------------------------------------------


def test_zero_principal_axes_becomes_identity(tmp_path):
    """The converter 0.3.2 defect, repaired."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/link",
        mass=2.0,
        diagonal_inertia=(0.1, 0.2, 0.3),
        principal_axes=(0, 0, 0, 0),
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link")
    source = export(stage, tmp_path / "robot.usda")

    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    record = _record_for(report, "/link", "physics:principalAxes")
    assert record["rule"] == "inertia.principal-axes-identity"
    assert record["new"] == [1.0, 0.0, 0.0, 0.0]
    assert record["old"] == [0.0, 0.0, 0.0, 0.0]
    assert record["old_state"] == "authored"
    assert record["confidence"] == "high"


def test_an_authored_valid_tensor_is_left_alone(tmp_path):
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/link",
        mass=2.0,
        diagonal_inertia=(0.2, 0.2, 0.3),
        principal_axes=(1, 0, 0, 0),
        box=(0.1, 0.1, 0.2),
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link")
    source = export(stage, tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    written = [r for r in report["records"] if r["attribute"] == "physics:diagonalInertia"]
    assert all(r["status"] != "applied" for r in written)
    assert any(r["status"] == "skipped" and "not overwriting" in r["reason"] for r in written)


def test_force_overwrites_an_authored_tensor(tmp_path):
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/link",
        mass=2.0,
        diagonal_inertia=(9.9, 9.9, 9.9),
        principal_axes=(1, 0, 0, 0),
        box=(0.1, 0.1, 0.2),
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link")
    source = export(stage, tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx", force=True))
    record = _record_for(report, "/link", "physics:diagonalInertia")
    assert record["new"] != [9.9, 9.9, 9.9]


def test_tensor_is_derived_from_geometry_about_the_authored_com(tmp_path):
    """Box 0.1 x 0.1 x 0.2, 10 kg, COM declared 0.1 m above the geometry.

    ``physics:diagonalInertia`` is defined about the centre of mass, so the
    tensor is moved to the declared point by the parallel-axis theorem:
    ``Ixx = Iyy = 10*(0.01+0.04)/12 + 10*0.1**2``, ``Izz`` unchanged.
    """
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/link",
        mass=10.0,
        center_of_mass=(0.0, 0.0, 0.1),
        box=(0.1, 0.1, 0.2),
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link")
    source = export(stage, tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    record = _record_for(report, "/link", "physics:diagonalInertia")
    expected_perp = 10.0 * (0.1**2 + 0.2**2) / 12.0 + 10.0 * 0.1**2
    expected_axial = 10.0 * (0.1**2 + 0.1**2) / 12.0
    assert sorted(record["new"]) == pytest.approx(sorted([expected_axial, expected_perp, expected_perp]))
    assert record["evidence"]["com_offset_m"][2] == pytest.approx(0.1)

    # newton:inertia describes the same tensor, so Newton and PhysX agree.
    newton = _record_for(report, "/link", "newton:inertia")
    assert sorted(newton["new"][:3]) == pytest.approx(sorted([expected_axial, expected_perp, expected_perp]))
    assert newton["new"][3:] == pytest.approx([0.0, 0.0, 0.0], abs=1e-12)


def test_a_body_with_no_geometry_falls_back_and_says_so(tmp_path):
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1), box=(1.0, 1.0, 1.0))
    add_link(stage, "/robot/Geometry/base/ghost", mass=2.0)
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/ghost")
    source = export(stage, tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    record = _record_for(report, "/ghost", "physics:diagonalInertia")
    assert record["confidence"] == "low"
    assert record["evidence"]["fallback"]["shape"] == "solid sphere"
    assert all(v > 0 for v in record["new"])


def test_make_physical_raises_the_small_moments_and_never_lowers_the_large():
    repaired, notes = _make_physical([0.01, 0.01, 1.0])
    assert max(repaired) == pytest.approx(1.0)
    assert repaired[0] + repaired[1] >= repaired[2] * (1 - 1e-9)
    assert any("I1 + I2 >= I3" in n for n in notes)

    repaired, notes = _make_physical([-0.5, 0.4, 0.4])
    assert min(repaired) >= 0.0
    assert any("negative" in n for n in notes)


def test_non_physical_tensor_is_repaired(tmp_path):
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/link",
        mass=2.0,
        diagonal_inertia=(0.001, 0.001, 1.0),  # violates I1 + I2 >= I3
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link")
    source = export(stage, tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    record = _record_for(report, "/link", "physics:diagonalInertia")
    assert record["rule"] == "inertia.make-physical"
    assert record["evidence"]["triangle_inequality_violated"] is True
    assert record["new"][0] + record["new"][1] >= record["new"][2] * (1 - 1e-6)


def test_a_massless_body_is_reported_not_invented(tmp_path):
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(stage, "/robot/Geometry/base/link", box=(0.1, 0.1, 0.1))
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link")
    source = export(stage, tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    reported = _records(report, "inertia.mass-floor", status="reported")
    assert reported and reported[0]["severity"] == "error"
    assert not _records(report, "inertia.mass-floor")
    # An error-severity finding is what makes the CLI exit non-zero.
    assert report["summary"]["errors"] >= 1


def test_mass_floor_can_be_opted_into(tmp_path):
    from urdf_usd_bridge.repair.base import resolve_rules

    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(stage, "/robot/Geometry/base/link", box=(0.1, 0.1, 0.1))
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link")
    source = export(stage, tmp_path / "robot.usda")
    report = fix_asset(
        source,
        tmp_path / "out",
        RepairOptions(backends_requested="physx", enabled=resolve_rules(["inertia.mass-floor"], None)),
    )
    record = _record_for(report, "/link", "physics:mass")
    assert record["new"] == pytest.approx(0.001 * 1000.0)  # 0.1^3 m^3 at 1000 kg/m^3
    assert record["confidence"] == "low"


def test_inertia_rules_run_before_the_drive_rules(tmp_path):
    """Gains are derived from the repaired tensor, not the broken one."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(stage, "/robot/Geometry/base/link", mass=4.0, box=(0.2, 0.2, 0.2))
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link")
    source = export(stage, tmp_path / "robot.usda")
    plan = analyse(source, RepairOptions(backends_requested="physx"))

    order = [r.rule.split(".", 1)[0] for r in plan["records"] if r.status == "applied"]
    assert order.index("inertia") < order.index("drives")
