# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Equivalent inertia, against hand-worked examples.

Every drive gain and every armature value in this project is a function of
``I_eq``, so if this is wrong, everything downstream is wrong by the same
factor and no other test would notice. The cases here are deliberately ones
that can be computed on paper.
"""

from __future__ import annotations

import math

import pytest

from urdf_usd_bridge.model.articulation import build_articulation, max_equivalent_inertia

from .repair_builders import add_joint, add_link, new_asset, simple_arm


def _joint(articulation, name: str):
    return next(j for j in articulation.joints if j.path.endswith(name))


def test_point_mass_on_a_rod():
    """``I_eq = I_zz + m * d**2`` -- the parallel-axis theorem, one body."""
    articulation = build_articulation(simple_arm())
    value, evidence = articulation.equivalent_inertia(_joint(articulation, "shoulder"))
    # Izz = 0.3, m = 2.0 kg at d = 0.5 m from the Z axis through the origin.
    assert value == pytest.approx(0.3 + 2.0 * 0.5**2)
    assert evidence["dof"] == "angular"
    assert [p.rsplit("/", 1)[-1] for p in evidence["subtree_bodies"]] == ["arm_link"]


def test_two_link_chain_sums_the_whole_subtree():
    """A joint carries every body below it, not just its immediate child."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=10.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/l1",
        translate=(1.0, 0, 0),
        mass=3.0,
        diagonal_inertia=(0.1, 0.1, 0.2),
        principal_axes=(1, 0, 0, 0),
    )
    add_link(
        stage,
        "/robot/Geometry/base/l1/l2",
        translate=(1.0, 0, 0),
        mass=1.0,
        diagonal_inertia=(0.05, 0.05, 0.05),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(stage, "/robot/Physics/j1", "/robot/Geometry/base", "/robot/Geometry/base/l1", axis="Z")
    add_joint(stage, "/robot/Physics/j2", "/robot/Geometry/base/l1", "/robot/Geometry/base/l1/l2", axis="Z")

    articulation = build_articulation(stage)
    # j1 sees l1 at d=1 and l2 at d=2 (transforms compose).
    expected_j1 = (0.2 + 3.0 * 1.0**2) + (0.05 + 1.0 * 2.0**2)
    assert articulation.equivalent_inertia(_joint(articulation, "j1"))[0] == pytest.approx(expected_j1)
    # j2 sees only l2, and the axis passes through l1's origin at x=1.
    expected_j2 = 0.05 + 1.0 * 1.0**2
    assert articulation.equivalent_inertia(_joint(articulation, "j2"))[0] == pytest.approx(expected_j2)


def test_prismatic_joint_sums_mass_not_inertia():
    """A linear DOF sees kilograms; using kg*m^2 there would be a unit error."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=10.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/slider",
        translate=(0, 0, 0.4),
        mass=2.5,
        diagonal_inertia=(9.0, 9.0, 9.0),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(
        stage,
        "/robot/Physics/slide",
        "/robot/Geometry/base",
        "/robot/Geometry/base/slider",
        kind="prismatic",
        axis="Z",
    )
    articulation = build_articulation(stage)
    value, evidence = articulation.equivalent_inertia(_joint(articulation, "slide"))
    assert value == pytest.approx(2.5)
    assert evidence["dof"] == "linear"


def test_offset_along_the_axis_does_not_add_inertia():
    """Only the *perpendicular* distance counts."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/up",
        translate=(0, 0, 5.0),  # straight up the Z axis
        mass=4.0,
        diagonal_inertia=(0.1, 0.1, 0.25),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/up", axis="Z")
    articulation = build_articulation(stage)
    assert articulation.equivalent_inertia(_joint(articulation, "j"))[0] == pytest.approx(0.25)


def test_zero_quaternion_principal_axes_is_ignored_not_applied():
    """The 0.3.2 defect must not annihilate the tensor during analysis.

    Multiplying by a zero quaternion zeroes the rotation, so a naive reader
    would compute ``I_eq`` from a tensor of zeros and derive zero gains.
    """
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/link",
        mass=2.0,
        diagonal_inertia=(0.1, 0.1, 0.4),
        principal_axes=(0, 0, 0, 0),  # the converter 0.3.2 defect
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link", axis="Z")
    articulation = build_articulation(stage)
    assert articulation.equivalent_inertia(_joint(articulation, "j"))[0] == pytest.approx(0.4)


def test_newton_inertia_is_preferred_over_the_eigendecomposition():
    """``newton:inertia`` is the exact tensor; the diagonal pair is derived."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    link = add_link(
        stage,
        "/robot/Geometry/base/link",
        mass=2.0,
        diagonal_inertia=(0.1, 0.1, 0.1),
        principal_axes=(1, 0, 0, 0),
    )
    from pxr import Sdf

    link.CreateAttribute("newton:inertia", Sdf.ValueTypeNames.DoubleArray, custom=True).Set(
        [0.7, 0.7, 0.9, 0.0, 0.0, 0.0]
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link", axis="Z")
    articulation = build_articulation(stage)
    assert articulation.equivalent_inertia(_joint(articulation, "j"))[0] == pytest.approx(0.9)


def test_stage_units_are_converted_to_si():
    """A centimetre stage must still produce kg*m^2."""
    from pxr import UsdGeom

    stage = simple_arm()
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    articulation = build_articulation(stage)
    value = articulation.equivalent_inertia(_joint(articulation, "shoulder"))[0]
    # Both the tensor and the offset scale by (0.01)^2.
    assert value == pytest.approx((0.3 + 2.0 * 0.5**2) * 1e-4)


def test_max_equivalent_inertia_ignores_linear_dofs():
    """The armature floor is rotational; mixing kg into it would be a unit error."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/rot",
        mass=1.0,
        diagonal_inertia=(0.1, 0.1, 0.2),
        principal_axes=(1, 0, 0, 0),
    )
    add_link(stage, "/robot/Geometry/base/rot/sld", mass=500.0, diagonal_inertia=(1, 1, 1))
    add_joint(stage, "/robot/Physics/j_rot", "/robot/Geometry/base", "/robot/Geometry/base/rot", axis="Z")
    add_joint(
        stage,
        "/robot/Physics/j_sld",
        "/robot/Geometry/base/rot",
        "/robot/Geometry/base/rot/sld",
        kind="prismatic",
        axis="Z",
    )
    articulation = build_articulation(stage)
    # 500 kg on the prismatic DOF must not become the rotational ceiling.
    assert max_equivalent_inertia(articulation) < 10.0


def test_child_rotation_changes_which_moment_the_axis_sees():
    """The tensor is rotated into world space before the axis projects it.

    The child is turned 90 degrees about X relative to its parent, so its local
    Z (``Izz = 0.9``) points along world -Y and its local Y (``Iyy = 0.5``)
    points along world Z. A joint on world Z therefore sees 0.5, not 0.9.
    Skipping the rotation would silently return the wrong moment for every
    link a URDF places with a non-identity ``<origin rpy=...>``.
    """
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/link",
        rotate_x=90.0,
        mass=1.0,
        diagonal_inertia=(0.1, 0.5, 0.9),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link", axis="Z")
    articulation = build_articulation(stage)
    joint = _joint(articulation, "j")
    assert abs(joint.axis_world[2]) == pytest.approx(1.0, abs=1e-9)
    assert math.isclose(articulation.equivalent_inertia(joint)[0], 0.5, rel_tol=1e-6)


def test_rotating_the_whole_assembly_changes_nothing():
    """A joint sees the same inertia however the robot as a whole is oriented."""
    upright = build_articulation(simple_arm())
    baseline = upright.equivalent_inertia(_joint(upright, "shoulder"))[0]

    tipped = simple_arm()
    from pxr import UsdGeom as _UsdGeom

    _UsdGeom.Xform(tipped.GetPrimAtPath("/robot/Geometry/base_link")).AddRotateXOp().Set(90.0)
    tipped_articulation = build_articulation(tipped)
    assert tipped_articulation.equivalent_inertia(_joint(tipped_articulation, "shoulder"))[
        0
    ] == pytest.approx(baseline, rel=1e-6)


def test_a_fixed_joint_child_still_counts_towards_the_parent_joint():
    """A welded tool is part of what the joint above it has to move.

    Collecting only revolute and prismatic joints leaves fixed-joint children
    out of the kinematic tree entirely, so their mass silently vanishes from
    ``I_eq`` and every derived gain comes out too soft.
    """
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(
        stage,
        "/robot/Geometry/base/arm",
        mass=1.0,
        diagonal_inertia=(0.01, 0.01, 0.02),
        principal_axes=(1, 0, 0, 0),
    )
    add_link(
        stage,
        "/robot/Geometry/base/arm/tool",
        translate=(0.4, 0, 0),
        mass=3.0,
        diagonal_inertia=(0.001, 0.001, 0.001),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/arm", axis="Z")
    add_joint(
        stage,
        "/robot/Physics/weld",
        "/robot/Geometry/base/arm",
        "/robot/Geometry/base/arm/tool",
        kind="fixed",
    )

    articulation = build_articulation(stage)
    joint = _joint(articulation, "/j")
    expected = 0.02 + (0.001 + 3.0 * 0.4**2)
    assert articulation.equivalent_inertia(joint)[0] == pytest.approx(expected)
    assert len(articulation.subtree("/robot/Geometry/base/arm")) == 2
