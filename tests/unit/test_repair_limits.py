# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Joint-limit rules -- G7.

The interesting cases are the two that look identical in the USD and are told
apart only by what *else* the joint carries.
"""

from __future__ import annotations

import math

import pytest

from urdf_usd_bridge.repair import RepairOptions, fix_asset
from urdf_usd_bridge.repair.base import resolve_rules

from .repair_builders import add_joint, add_link, export, new_asset


def _chain(tmp_path, **joint_kwargs):
    """A base and one child joined by a joint with the given limit state."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=2.0, diagonal_inertia=(0.1, 0.1, 0.1))
    add_link(
        stage,
        "/robot/Geometry/base/link",
        translate=(0.3, 0, 0),
        mass=1.0,
        diagonal_inertia=(0.01, 0.01, 0.02),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(
        stage,
        "/robot/Physics/j",
        "/robot/Geometry/base",
        "/robot/Geometry/base/link",
        **joint_kwargs,
    )
    return export(stage, tmp_path / "robot.usda")


def _for(report, rule, status=None):
    return [r for r in report["records"] if r["rule"] == rule and (status is None or r["status"] == status)]


def test_a_welded_joint_with_no_limit_evidence_is_unlocked(tmp_path):
    """No ``urdf:limit:effort`` and no ``newton:velocityLimit`` means the URDF
    had no ``<limit>`` element at all, because those two are required on it."""
    source = _chain(tmp_path, lower=0.0, upper=0.0)
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    applied = _for(report, "limits.restore-missing", "applied")
    assert {r["attribute"] for r in applied} == {"physics:lowerLimit", "physics:upperLimit"}
    assert {r["new"] for r in applied} == {"-inf", "+inf"}
    assert all(r["confidence"] == "medium" for r in applied)

    from pxr import Usd

    stage = Usd.Stage.Open(str(tmp_path / "out" / "robot_stabilized.usda"), Usd.Stage.LoadAll)
    joint = stage.GetPrimAtPath("/robot/Physics/j")
    assert joint.GetAttribute("physics:lowerLimit").Get() == -math.inf
    assert joint.GetAttribute("physics:upperLimit").Get() == math.inf


def test_a_welded_joint_with_limit_evidence_is_reported_not_guessed(tmp_path):
    """``lower=0 upper=0`` may be exactly what the author meant."""
    source = _chain(tmp_path, lower=0.0, upper=0.0, urdf_effort=30.0, newton_velocity_limit=85.9)
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    assert not _for(report, "limits.restore-missing", "applied")
    reported = _for(report, "limits.report-ambiguous", "refused")
    assert len(reported) == 1
    record = reported[0]
    assert record["severity"] == "error"
    assert "MuJoCo refuses to compile" in record["reason"]
    assert "--force-unlock" in record["reason"]
    assert record["evidence"]["mujoco_compiles"] is False
    assert record["evidence"]["limit_evidence"]["urdf:limit:effort"] == pytest.approx(30.0)
    assert len(record["evidence"]["readings"]) == 2


def test_velocity_limit_alone_is_enough_evidence(tmp_path):
    """Either attribute proves ``<limit>`` existed; both are required on it."""
    source = _chain(tmp_path, lower=0.0, upper=0.0, newton_velocity_limit=57.3)
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    assert not _for(report, "limits.restore-missing", "applied")
    assert _for(report, "limits.report-ambiguous", "refused")


def test_a_normally_limited_joint_is_untouched(tmp_path):
    source = _chain(tmp_path, lower=-90.0, upper=90.0, urdf_effort=20.0)
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    assert not _for(report, "limits.restore-missing")
    assert not _for(report, "limits.report-ambiguous")


def test_a_continuous_joint_is_untouched(tmp_path):
    """+/-inf is how the converter spells an unbounded joint already."""
    source = _chain(tmp_path, lower=-math.inf, upper=math.inf, urdf_effort=10.0)
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    assert not _for(report, "limits.restore-missing")
    assert not _for(report, "limits.compliance")


def test_prismatic_joints_are_report_only_by_default(tmp_path):
    """Decision D4: an unlimited rail can be worse than a welded one."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=2.0, diagonal_inertia=(0.1, 0.1, 0.1))
    add_link(
        stage,
        "/robot/Geometry/base/slider",
        translate=(0, 0, 0.2),
        mass=1.0,
        diagonal_inertia=(0.01, 0.01, 0.01),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(
        stage,
        "/robot/Physics/slide",
        "/robot/Geometry/base",
        "/robot/Geometry/base/slider",
        kind="prismatic",
        lower=0.0,
        upper=0.0,
    )
    source = export(stage, tmp_path / "robot.usda")

    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    reported = _for(report, "limits.restore-missing-prismatic", "reported")
    assert len(reported) == 1
    assert "off by default" in reported[0]["reason"]

    opted_in = fix_asset(
        source,
        tmp_path / "out2",
        RepairOptions(
            backends_requested="physx",
            enabled=resolve_rules(["limits.restore-missing-prismatic"], None),
        ),
    )
    assert len(_for(opted_in, "limits.restore-missing-prismatic", "applied")) == 2


def test_limit_compliance_is_reported_never_invented(tmp_path):
    """Deriving a limit stiffness needs the effective inertia at qpos0."""
    source = _chain(tmp_path, lower=-45.0, upper=45.0, urdf_effort=10.0)
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    compliance = _for(report, "limits.compliance", "reported")
    assert len(compliance) == 1
    assert compliance[0]["severity"] == "warning"
    assert compliance[0]["evidence"]["phase4_overshoot_cells_worse"] == 6
    assert compliance[0]["new"] is None
    assert "mujoco-usd-converter" in compliance[0]["reason"]


def test_the_rule_can_be_disabled(tmp_path):
    source = _chain(tmp_path, lower=0.0, upper=0.0)
    report = fix_asset(
        source,
        tmp_path / "out",
        RepairOptions(backends_requested="physx", enabled=resolve_rules(None, ["limits.restore-missing"])),
    )
    assert not _for(report, "limits.restore-missing", "applied")
    assert _for(report, "limits.restore-missing", "reported")


def test_force_unlock_unlocks_an_ambiguous_joint_and_says_it_guessed(tmp_path):
    """Decision: keep refusing to guess, but give the user a named way out.

    MuJoCo will not compile an asset containing a [0, 0] joint, so "refuse and
    stop" leaves the user with an asset one backend cannot load at all.
    """
    source = _chain(tmp_path, lower=0.0, upper=0.0, urdf_effort=30.0)

    refused = fix_asset(source, tmp_path / "a", RepairOptions(backends_requested="physx"))
    assert _for(refused, "limits.report-ambiguous", "refused")
    assert refused["summary"]["errors"] >= 1

    forced = fix_asset(
        source,
        tmp_path / "b",
        RepairOptions(backends_requested="physx", force_unlock=True),
    )
    applied = _for(forced, "limits.report-ambiguous", "applied")
    assert len(applied) == 2
    assert all(r["forced"] is True for r in applied)
    assert all(r["confidence"] == "low" for r in applied)
    assert "the range is a guess" in applied[0]["reason"]
    assert forced["summary"]["errors"] == 0

    from pxr import Usd

    stage = Usd.Stage.Open(str(tmp_path / "b" / "robot_stabilized.usda"), Usd.Stage.LoadAll)
    joint = stage.GetPrimAtPath("/robot/Physics/j")
    assert joint.GetAttribute("physics:lowerLimit").Get() == -math.inf


def test_an_unambiguous_welded_joint_does_not_need_force_unlock(tmp_path):
    source = _chain(tmp_path, lower=0.0, upper=0.0)
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    applied = _for(report, "limits.restore-missing", "applied")
    assert len(applied) == 2
    assert all(r["forced"] is False for r in applied)
    assert report["summary"]["errors"] == 0
