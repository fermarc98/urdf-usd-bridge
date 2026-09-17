# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Inspector behaviour on joint dynamics, drives and limits."""

from __future__ import annotations

import pytest

from urdf_usd_bridge.inspection import inspect_stage

from . import builders


def _joint(report, name):
    return next(j for j in report["joints"] if j["name"] == name)


@pytest.fixture()
def converter_like_stage(tmp_path):
    """Shaped like urdf-usd-converter >= 0.3.0 output: newton:*, no drives."""
    stage = builders.new_stage("conv")
    builders.add_revolute_joint(
        stage,
        "/conv/Physics/shoulder",
        lower=-90.0,
        upper=90.0,
        newton_damping=0.0261799,
        newton_friction=0.3,
        urdf_effort=87.0,
    )
    return builders.save(stage, tmp_path / "conv.usda")


@pytest.fixture()
def isaac_like_stage(tmp_path):
    """Shaped like Isaac Sim 6.1.0 output: DriveAPI applied but with no gains."""
    stage = builders.new_stage("isaac")
    builders.add_revolute_joint(
        stage,
        "/isaac/Physics/shoulder",
        lower=-90.0,
        upper=90.0,
        newton_damping=0.0261799,
        newton_friction=0.3,
        urdf_effort=87.0,
        drive=True,
        drive_max_force=87.0,
    )
    builders.add_mjc_actuator(stage, "/isaac/Physics/shoulder_actuator", "/isaac/Physics/shoulder")
    return builders.save(stage, tmp_path / "isaac.usda")


def test_damping_is_reported_per_namespace_not_collapsed(converter_like_stage):
    report = inspect_stage(converter_like_stage)
    joint = _joint(report, "shoulder")

    by_ns = joint["damping_by_namespace"]
    assert by_ns["newton"]["authored"] is True
    assert by_ns["newton"]["value"] == pytest.approx(0.0261799)
    # The spelling Isaac Sim 6.1.0 looks for is simply not there.
    assert by_ns["urdf_custom"] is None
    assert by_ns["mjc"] is None
    assert joint["flags"]["damping_namespaces"] == ["newton"]


def test_damping_outside_a_drive_is_flagged_as_stranded(converter_like_stage):
    report = inspect_stage(converter_like_stage)
    joint = _joint(report, "shoulder")
    assert joint["flags"]["has_drive_api"] is False
    assert joint["flags"]["damping_stranded_outside_drive"] is True
    assert report["summary"]["joints_damping_stranded_outside_drive"] == 1


def test_drive_applied_without_gains_is_distinguished_from_no_drive(isaac_like_stage):
    report = inspect_stage(isaac_like_stage)
    joint = _joint(report, "shoulder")
    assert joint["flags"]["has_drive_api"] is True
    assert joint["flags"]["drive_has_gains"] is False
    assert joint["flags"]["drive_applied_without_gains"] is True
    assert report["summary"]["joints_with_drive_api"] == 1
    assert report["summary"]["joints_with_drive_gains"] == 0


def test_unauthored_drive_gains_are_reported_as_fallback_not_authored(isaac_like_stage):
    report = inspect_stage(isaac_like_stage)
    drive = _joint(report, "shoulder")["drives"]["angular"]
    # The attribute exists (DriveAPI is applied) but carries only its fallback.
    assert drive["stiffness"] is not None
    assert drive["stiffness"]["authored"] is False
    assert drive["maxForce"]["authored"] is True
    assert drive["maxForce"]["value"] == pytest.approx(87.0)


def test_effort_stranded_as_urdf_custom_attribute(converter_like_stage):
    report = inspect_stage(converter_like_stage)
    joint = _joint(report, "shoulder")
    assert joint["limit_extras"]["urdf_effort"]["value"] == pytest.approx(87.0)
    assert joint["flags"]["effort_only_as_urdf_custom"] is True


def test_effort_not_stranded_once_it_reaches_drive_max_force(isaac_like_stage):
    report = inspect_stage(isaac_like_stage)
    assert _joint(report, "shoulder")["flags"]["effort_only_as_urdf_custom"] is False


def test_equal_limits_are_flagged_as_locked(tmp_path):
    stage = builders.new_stage("locked")
    builders.add_revolute_joint(stage, "/locked/Physics/no_limit_joint", lower=0.0, upper=0.0)
    builders.add_revolute_joint(stage, "/locked/Physics/ok_joint", lower=-45.0, upper=45.0)
    path = builders.save(stage, tmp_path / "locked.usda")

    report = inspect_stage(path)
    assert _joint(report, "no_limit_joint")["flags"]["locked_by_equal_limits"] is True
    assert _joint(report, "ok_joint")["flags"]["locked_by_equal_limits"] is False
    assert report["summary"]["joints_locked_by_equal_limits"] == 1


def test_armature_absence_is_detected_and_presence_is_attributed(tmp_path):
    stage = builders.new_stage("arm")
    builders.add_revolute_joint(stage, "/arm/Physics/bare")
    builders.add_revolute_joint(stage, "/arm/Physics/geared", physx_armature=0.01)
    path = builders.save(stage, tmp_path / "arm.usda")

    report = inspect_stage(path)
    assert _joint(report, "bare")["flags"]["no_armature_anywhere"] is True
    assert _joint(report, "geared")["flags"]["armature_namespaces"] == ["physx"]
    assert report["summary"]["joints_without_armature"] == 1
    assert report["summary"]["armature_authored_by_namespace"] == {"physx": 1}


def test_mjc_actuator_without_gain_parameters_is_flagged(isaac_like_stage):
    report = inspect_stage(isaac_like_stage)
    assert report["summary"]["mjc_actuators_total"] == 1
    assert report["summary"]["mjc_actuators_without_gains"] == 1
    actuator = report["mjc_actuators"][0]
    assert actuator["target"] == ["/isaac/Physics/shoulder"]


def test_mjc_actuator_with_gain_parameters_is_not_flagged(tmp_path):
    stage = builders.new_stage("mjc")
    builders.add_revolute_joint(stage, "/mjc/Physics/j", drive=True, drive_stiffness=800.0)
    builders.add_mjc_actuator(
        stage,
        "/mjc/Physics/j_actuator",
        "/mjc/Physics/j",
        gain_prm=[800.0] + [0.0] * 9,
        bias_prm=[0.0, -800.0, -40.0] + [0.0] * 7,
    )
    path = builders.save(stage, tmp_path / "mjc.usda")

    report = inspect_stage(path)
    assert report["summary"]["mjc_actuators_without_gains"] == 0
