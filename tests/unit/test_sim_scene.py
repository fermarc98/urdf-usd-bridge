# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The two scene guards, which exist so a null result cannot be faked.

Both guards were written because a probe produced a confident, meaningless
number: fixture (a) at its home pose has a vertical shoulder axis, so gravity
exerts no torque and repaired and unrepaired hold equally well; and a ground
plane intersecting the arm made a working repair look broken.

These tests build both situations deliberately and require the guards to catch
them. No simulator involved.
"""

from __future__ import annotations

import math

import pytest
from pxr import Gf, UsdGeom

from urdf_usd_bridge.model.articulation import build_articulation
from urdf_usd_bridge.sim.scene import (
    SceneSpec,
    check_gravity_loaded,
    check_no_interpenetration,
    collision_bounds_min_z,
    gravity_torque_about,
    loaded_pose,
    required_lift,
)

from .repair_builders import add_joint, add_link, new_asset


def _joint(articulation, suffix):
    return next(j for j in articulation.joints if j.path.endswith(suffix))


def _arm(axis: str, translate=(0.5, 0.0, 0.0), limits=(-90.0, 90.0)):
    """A one-joint arm with a chosen axis, for the loading cases."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=5.0, diagonal_inertia=(0.05, 0.05, 0.05))
    add_link(
        stage,
        "/robot/Geometry/base/arm",
        translate=translate,
        mass=2.0,
        diagonal_inertia=(0.1, 0.1, 0.3),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(
        stage,
        "/robot/Physics/shoulder",
        "/robot/Geometry/base",
        "/robot/Geometry/base/arm",
        axis=axis,
        lower=limits[0],
        upper=limits[1],
    )
    return stage


def test_a_turntable_cannot_be_gravity_loaded_at_any_angle():
    """A vertical axis with a horizontal arm is a turntable.

    Gravity produces no torque about it however far it turns, so no pose can
    test its drive. That is different from a joint that merely happens to be
    unloaded at the pose we picked, and the guard has to tell them apart.
    """
    from urdf_usd_bridge.sim.scene import best_loading_angle

    turntable = build_articulation(_arm("Z"))
    joint = _joint(turntable, "shoulder")
    _, torque = best_loading_angle(joint, turntable)
    assert torque == pytest.approx(0.0, abs=1e-9)


def test_a_horizontal_axis_loads_once_it_turns_off_the_axis():
    """Fixture (a)'s shape: the arm sits on the axis at home, and loads away from it."""
    stage = _arm("Y", translate=(0.0, 0.0, 0.5), limits=(-90.0, 90.0))
    articulation = build_articulation(stage)
    joint = _joint(articulation, "shoulder")

    # Directly above the axis: the lever arm is parallel to gravity, torque zero.
    assert gravity_torque_about(joint, articulation, angle=0.0) == pytest.approx(0.0, abs=1e-9)
    # A quarter turn puts the mass out sideways, and it loads up.
    assert gravity_torque_about(joint, articulation, angle=math.pi / 2) > 5.0

    pose = loaded_pose(stage)
    assert abs(pose["/robot/Physics/shoulder"]) > 1.0
    assert check_gravity_loaded(stage, pose=pose).passed is True


def test_the_gravity_guard_fails_when_no_joint_can_be_loaded():
    """A single-joint turntable leaves nothing to measure."""
    stage = _arm("Z")
    guard = check_gravity_loaded(stage)
    assert guard.passed is False
    assert "no actuated joint is gravity-loaded" in guard.reason
    assert guard.evidence["unloadable_joints"] == ["/robot/Physics/shoulder"]
    assert guard.evidence["loaded_joints"] == []


def test_the_gravity_guard_excludes_an_unloadable_joint_but_still_measures_the_rest():
    """Most real robots have one joint gravity cannot reach.

    Refusing the whole asset for that would throw away every other joint's
    measurement, so the unloadable joint is named and excluded instead -- and
    the metrics are computed over the joints that were actually tested.
    """
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=5.0, diagonal_inertia=(0.05, 0.05, 0.05))
    add_link(
        stage,
        "/robot/Geometry/base/turn",
        mass=2.0,
        diagonal_inertia=(0.1, 0.1, 0.3),
        principal_axes=(1, 0, 0, 0),
    )
    add_link(
        stage,
        "/robot/Geometry/base/turn/arm",
        translate=(0.4, 0.0, 0.0),
        mass=2.0,
        diagonal_inertia=(0.1, 0.1, 0.3),
        principal_axes=(1, 0, 0, 0),
    )
    # A turntable: its own mass sits on the axis.
    add_joint(stage, "/robot/Physics/turn", "/robot/Geometry/base", "/robot/Geometry/base/turn", axis="Z")
    # A loadable elbow.
    add_joint(
        stage,
        "/robot/Physics/elbow",
        "/robot/Geometry/base/turn",
        "/robot/Geometry/base/turn/arm",
        axis="Y",
        lower=-90.0,
        upper=90.0,
    )

    guard = check_gravity_loaded(stage)
    assert guard.passed is True
    assert "excluded from the measurement" in guard.reason
    assert guard.evidence["loaded_joints"] == ["/robot/Physics/elbow"]
    assert guard.evidence["unloadable_joints"] == ["/robot/Physics/turn"]


def test_the_gravity_guard_fails_on_an_unloaded_pose_that_could_have_been_loaded():
    """Distinguished from the turntable: a loading angle exists, we just missed it."""
    stage = _arm("Y", translate=(0.0, 0.0, 0.5))
    guard = check_gravity_loaded(stage, pose={"/robot/Physics/shoulder": 0.0})
    assert guard.passed is False
    assert "though a loading angle exists" in guard.reason
    assert guard.evidence["max_reachable_torque"]["/robot/Physics/shoulder"] > 5.0


def test_the_gravity_guard_passes_on_a_loaded_pose():
    stage = _arm("Y")
    guard = check_gravity_loaded(stage)
    assert guard.passed is True
    assert "gravity-loaded" in guard.reason
    assert all(v > 0 for v in guard.evidence["gravity_torque_per_joint"].values())


def test_the_gravity_guard_fails_when_there_is_nothing_to_load():
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    guard = check_gravity_loaded(stage)
    assert guard.passed is False
    assert "no actuated joints" in guard.reason


# --- interpenetration ------------------------------------------------------


def _arm_at_height(height: float):
    stage = new_asset()
    add_link(
        stage,
        "/robot/Geometry/base",
        translate=(0.0, 0.0, height),
        mass=5.0,
        diagonal_inertia=(0.05, 0.05, 0.05),
        box=(0.2, 0.2, 0.1),
    )
    add_link(
        stage,
        "/robot/Geometry/base/arm",
        translate=(0.5, 0.0, 0.0),
        mass=2.0,
        diagonal_inertia=(0.1, 0.1, 0.3),
        principal_axes=(1, 0, 0, 0),
        box=(0.05, 0.05, 0.3),
    )
    add_joint(stage, "/robot/Physics/shoulder", "/robot/Geometry/base", "/robot/Geometry/base/arm", axis="Z")
    return stage


def test_collision_bounds_find_the_lowest_collider():
    stage = _arm_at_height(1.0)
    lowest, culprit = collision_bounds_min_z(stage)
    assert math.isfinite(lowest)
    assert culprit is not None
    # The arm box is 0.3 tall, centred at z=1.0, so it reaches down to 0.85.
    assert lowest == pytest.approx(0.85, abs=1e-6)


def test_the_interpenetration_guard_fails_when_a_collider_starts_below_the_plane():
    """The artefact that made a working repair look like it was failing."""
    stage = _arm_at_height(0.0)  # colliders straddle z=0
    guard = check_no_interpenetration(stage)
    assert guard.passed is False
    assert "below the ground plane" in guard.reason
    assert guard.evidence["clearance"] < 0


def test_the_interpenetration_guard_passes_when_clear():
    guard = check_no_interpenetration(_arm_at_height(1.0))
    assert guard.passed is True
    assert guard.evidence["clearance"] == pytest.approx(0.85, abs=1e-6)


def test_required_lift_is_exactly_enough_to_clear_the_plane():
    stage = _arm_at_height(0.0)
    lift = required_lift(stage)
    assert lift > 0
    assert check_no_interpenetration(stage, lift=lift).passed is True
    # And a hair less is not enough, so the lift is not over-generous.
    assert check_no_interpenetration(stage, lift=lift - 0.01).passed is False


# --- the pose proposal -----------------------------------------------------


def test_loaded_pose_picks_the_angle_that_loads_the_joint():
    """Not the midpoint of the range -- that is the *unloaded* pose here."""
    stage = _arm("Y", translate=(0.0, 0.0, 0.5))
    articulation = build_articulation(stage)
    joint = _joint(articulation, "shoulder")
    chosen = loaded_pose(stage)["/robot/Physics/shoulder"]

    midpoint_torque = gravity_torque_about(joint, articulation, angle=0.0)
    chosen_torque = gravity_torque_about(joint, articulation, angle=chosen)
    assert midpoint_torque == pytest.approx(0.0, abs=1e-9)
    assert chosen_torque > 5.0


def test_loaded_pose_stays_near_the_authored_configuration():
    """When several angles load equally, the nearest to home wins."""
    stage = _arm("Y", translate=(0.5, 0.0, 0.0), limits=(-180.0, 180.0))
    chosen = loaded_pose(stage)["/robot/Physics/shoulder"]
    # 0 and +/-pi load this arm identically; the tie-break keeps it near home.
    assert abs(chosen) < 0.2


def test_loaded_pose_respects_joint_limits():
    stage = _arm("Y", translate=(0.0, 0.0, 0.5), limits=(-10.0, 10.0))
    chosen = loaded_pose(stage)["/robot/Physics/shoulder"]
    assert math.radians(-10.0) - 1e-9 <= chosen <= math.radians(10.0) + 1e-9


def test_scene_spec_records_what_is_comparable():
    scene = SceneSpec(dt=1 / 120, duration_s=2.0)
    assert scene.steps == 240
    payload = scene.as_dict()
    assert payload["gravity"] == (0.0, 0.0, -9.81)
    # Solver iteration counts are deliberately absent: three different solvers
    # cannot be equalised, so they are recorded per backend instead.
    assert "solver_iterations" not in payload


def test_a_rotated_base_rotates_the_joint_axis_with_it():
    """The guard reads world-space geometry, not the authored axis token."""
    stage = _arm("Y")
    UsdGeom.Xform(stage.GetPrimAtPath("/robot/Geometry/base")).AddRotateXOp().Set(90.0)
    articulation = build_articulation(stage)
    joint = _joint(articulation, "shoulder")
    # Local Y becomes world Z once the base is tipped a quarter turn about X.
    assert abs(float(joint.axis_world[2])) == pytest.approx(1.0, abs=1e-6)
    # And the arm, now vertical below a vertical axis, becomes a turntable.
    from urdf_usd_bridge.sim.scene import best_loading_angle

    assert best_loading_angle(joint, articulation)[1] == pytest.approx(0.0, abs=1e-9)


def test_guard_results_serialise_for_the_report():
    guard = check_gravity_loaded(_arm("Y"))
    payload = guard.as_dict()
    assert set(payload) == {"guard", "passed", "reason", "evidence"}
    assert payload["guard"] == "gravity_loaded"
    assert isinstance(payload["passed"], bool)
    _ = Gf.Vec3d(0, 0, 0)  # keep the pxr import meaningful for linters
