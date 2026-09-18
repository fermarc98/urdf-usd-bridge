# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Drive and armature rules -- G1 and G3.

Gains are checked against the closed-form ``K = I (2 pi f)^2`` rather than
against recorded output, so the arithmetic cannot drift. The cross-backend unit
conventions have their own file, ``test_repair_units.py``.
"""

from __future__ import annotations

import math

import pytest

from urdf_usd_bridge.repair import RepairOptions, fix_asset
from urdf_usd_bridge.repair.base import DEFAULTS, resolve_rules
from urdf_usd_bridge.repair.drives import gains_from_frequency

from .repair_builders import add_joint, add_link, export, simple_arm

DEG_PER_RAD = 180.0 / math.pi

#: The hand-computed equivalent inertia of ``simple_arm``'s shoulder joint.
ARM_I_EQ = 0.3 + 2.0 * 0.5**2


def _applied(report, attribute, prim_suffix=None):
    return next(
        r
        for r in report["records"]
        if r["attribute"] == attribute
        and r["status"] == "applied"
        and (prim_suffix is None or r["prim"].endswith(prim_suffix))
    )


@pytest.fixture
def arm(tmp_path):
    return export(simple_arm(), tmp_path / "robot.usda")


def test_the_gain_formula_is_the_standard_second_order_one():
    stiffness, damping = gains_from_frequency(0.5, 10.0, 1.0)
    assert stiffness == pytest.approx(0.5 * (2 * math.pi * 10.0) ** 2)
    assert damping == pytest.approx(2 * 1.0 * math.sqrt(0.5 * stiffness))
    # Critical damping: D = 2 zeta omega_n I.
    assert damping == pytest.approx(2 * 1.0 * (2 * math.pi * 10.0) * 0.5)


def test_derived_stiffness_matches_the_closed_form(arm, tmp_path):
    report = fix_asset(arm, tmp_path / "out", RepairOptions(backends_requested="physx"))
    record = _applied(report, "drive:angular:physics:stiffness")

    armature = record["evidence"]["armature"]
    i_total = ARM_I_EQ + armature
    assert record["evidence"]["I_eq"] == pytest.approx(ARM_I_EQ)
    assert record["evidence"]["I_total"] == pytest.approx(i_total)
    frequency = record["evidence"]["target_frequency_hz"]
    # physx alone is measured stable at control_rate/6.
    assert frequency == pytest.approx(DEFAULTS.control_rate / 6.0)
    assert "physx" in record["evidence"]["target_frequency_basis"]
    expected_si = i_total * (2 * math.pi * frequency) ** 2
    assert record["evidence"]["K_si"] == pytest.approx(expected_si)
    # Stored per degree.
    assert record["new"] == pytest.approx(expected_si / DEG_PER_RAD, rel=1e-6)
    assert record["units"] == "N*m/deg"


def test_tuning_flags_change_the_gains(arm, tmp_path):
    slow = fix_asset(arm, tmp_path / "a", RepairOptions(backends_requested="physx", target_frequency=5.0))
    fast = fix_asset(arm, tmp_path / "b", RepairOptions(backends_requested="physx", target_frequency=20.0))
    slow_k = _applied(slow, "drive:angular:physics:stiffness")["new"]
    fast_k = _applied(fast, "drive:angular:physics:stiffness")["new"]
    # K scales with f^2, so quadrupling f multiplies K by 16.
    assert fast_k / slow_k == pytest.approx(16.0, rel=1e-4)


def test_every_derived_record_carries_its_tuning_provenance(arm, tmp_path):
    """Phase 3 marked these ``unmeasured``; Phase 4 measured them.

    This test inverted with that change, as ``docs/history/PHASE4_DESIGN.md`` section 8
    said it would, so the constants and the story told about them cannot drift
    apart.
    """
    from urdf_usd_bridge.repair.base import PROVENANCE

    report = fix_asset(arm, tmp_path / "out", RepairOptions(backends_requested="physx"))
    record = _applied(report, "drive:angular:physics:stiffness")

    assert report["options"]["tuning_provenance"] == dict(PROVENANCE)
    # The measured constants name a date and a measurement; the unmeasured ones
    # say plainly that they are unmeasured.
    assert PROVENANCE["target_frequency"].startswith("measured")
    assert PROVENANCE["damping_ratio"].startswith("measured")
    assert PROVENANCE["armature_floor"].startswith("unmeasured")
    assert "measured" in report["options"]["tuning_status"]
    # The per-record flag still marks the model as a model, not a measurement
    # of this particular robot.
    assert record["evidence"]["unmeasured"] is True


def test_passive_damping_is_folded_into_the_physx_drive(arm, tmp_path):
    """PhysX has nowhere else to put it."""
    report = fix_asset(arm, tmp_path / "out", RepairOptions(backends_requested="physx"))
    record = _applied(report, "drive:angular:physics:damping")
    evidence = record["evidence"]

    assert evidence["D_passive_si"] == pytest.approx(1.5, rel=1e-6)
    assert evidence["D_total_si"] == pytest.approx(evidence["D_drive_si"] + 1.5)
    assert record["new"] == pytest.approx(evidence["D_total_si"] / DEG_PER_RAD, rel=1e-6)


def test_max_force_comes_from_urdf_effort_and_is_never_invented(arm, tmp_path):
    report = fix_asset(arm, tmp_path / "out", RepairOptions(backends_requested="physx"))
    assert _applied(report, "drive:angular:physics:maxForce")["new"] == pytest.approx(87.0)

    stage = simple_arm(name="bare")
    joint = stage.GetPrimAtPath("/bare/Physics/shoulder")
    joint.GetAttribute("urdf:limit:effort").Clear()
    source = export(stage, tmp_path / "bare.usda")
    bare = fix_asset(source, tmp_path / "out2", RepairOptions(backends_requested="physx"))

    reported = [
        r
        for r in bare["records"]
        if r["attribute"] == "drive:angular:physics:maxForce" and r["status"] == "reported"
    ]
    assert reported and "Refusing to invent" in reported[0]["reason"]


def test_target_position_holds_the_authored_pose(arm, tmp_path):
    report = fix_asset(arm, tmp_path / "out", RepairOptions(backends_requested="physx"))
    assert _applied(report, "drive:angular:physics:targetPosition")["new"] == 0.0


def test_an_authored_drive_is_not_overwritten(tmp_path):
    from pxr import UsdPhysics

    stage = simple_arm()
    joint = stage.GetPrimAtPath("/robot/Physics/shoulder")
    UsdPhysics.DriveAPI.Apply(joint, "angular").CreateStiffnessAttr().Set(12.0)
    source = export(stage, tmp_path / "robot.usda")

    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    skipped = [
        r for r in report["records"] if r["rule"] == "drives.derive-gains" and r["status"] == "skipped"
    ]
    assert skipped and skipped[0]["old"] == pytest.approx(12.0)

    forced = fix_asset(source, tmp_path / "out2", RepairOptions(backends_requested="physx", force=True))
    assert _applied(forced, "drive:angular:physics:stiffness")["new"] != pytest.approx(12.0)


def test_fixed_joints_get_no_drive(tmp_path):
    stage = simple_arm()
    add_link(
        stage,
        "/robot/Geometry/base_link/arm_link/tool",
        mass=0.1,
        diagonal_inertia=(0.001, 0.001, 0.001),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(
        stage,
        "/robot/Physics/weld",
        "/robot/Geometry/base_link/arm_link",
        "/robot/Geometry/base_link/arm_link/tool",
        kind="fixed",
    )
    source = export(stage, tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    skipped = [r for r in report["records"] if r["rule"] == "drives.no-drive-for-fixed"]
    assert skipped and skipped[0]["prim"].endswith("/weld")
    assert not any(r["prim"].endswith("/weld") and r["status"] == "applied" for r in report["records"])


# --- armature --------------------------------------------------------------


def test_armature_uses_the_rotor_term_for_a_heavy_dof(arm, tmp_path):
    report = fix_asset(arm, tmp_path / "out", RepairOptions(backends_requested="physx"))
    record = _applied(report, "newton:armature")
    evidence = record["evidence"]

    assert evidence["formula"] == "armature = max(alpha * I_eq, beta * I_eq_max)"
    assert evidence["rotor_term"] == pytest.approx(0.01 * ARM_I_EQ)
    assert record["new"] == pytest.approx(0.01 * ARM_I_EQ)
    assert "rotor term" in evidence["term_used"]
    assert record["units"] == "kg*m^2"


def test_the_conditioning_floor_rescues_a_light_dof(tmp_path):
    """A fingertip's own 1% would be no regularisation at all."""
    stage = simple_arm()
    add_link(
        stage,
        "/robot/Geometry/base_link/arm_link/tip",
        translate=(0.05, 0, 0),
        mass=0.002,
        diagonal_inertia=(1e-8, 1e-8, 1e-8),
        principal_axes=(1, 0, 0, 0),
    )
    add_joint(
        stage,
        "/robot/Physics/tip_joint",
        "/robot/Geometry/base_link/arm_link",
        "/robot/Geometry/base_link/arm_link/tip",
        axis="Z",
        lower=-10.0,
        upper=10.0,
    )
    source = export(stage, tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))

    tip = _applied(report, "newton:armature", prim_suffix="/tip_joint")
    evidence = tip["evidence"]
    assert "conditioning floor" in evidence["term_used"]
    assert tip["new"] == pytest.approx(1e-4 * evidence["I_eq_max"])
    # The floor is what makes it meaningful: many times the DOF's own inertia.
    assert tip["new"] > evidence["I_eq"]


def test_armature_is_identical_in_every_namespace(tmp_path):
    source = export(simple_arm(), tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx,mujoco"))
    values = {
        r["attribute"]: r["new"]
        for r in report["records"]
        if r["status"] == "applied" and "armature" in (r["attribute"] or "")
    }
    assert set(values) == {"newton:armature", "physxJoint:armature", "mjc:armature"}
    assert len({round(v, 12) for v in values.values()}) == 1


def test_an_authored_armature_is_respected(tmp_path):
    from pxr import Sdf

    stage = simple_arm()
    joint = stage.GetPrimAtPath("/robot/Physics/shoulder")
    joint.CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float, custom=True).Set(0.05)
    source = export(stage, tmp_path / "robot.usda")

    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    skipped = [r for r in report["records"] if r["rule"] == "armature.default" and r["status"] == "skipped"]
    assert skipped and skipped[0]["old"] == pytest.approx(0.05)

    # And the drive gains must use the armature the asset already had.
    stiffness = _applied(report, "drive:angular:physics:stiffness")
    assert stiffness["evidence"]["armature"] == pytest.approx(0.05)


def test_armature_can_be_disabled(arm, tmp_path):
    report = fix_asset(
        arm,
        tmp_path / "out",
        RepairOptions(backends_requested="physx", enabled=resolve_rules(None, ["armature.default"])),
    )
    assert not [r for r in report["records"] if r["rule"] == "armature.default" and r["status"] == "applied"]
    assert _applied(report, "drive:angular:physics:stiffness")["evidence"]["armature"] == 0.0


def test_a_target_frequency_above_the_measured_ratio_warns(arm, tmp_path):
    """Measured: PhysX and MuJoCo survive rate/6; Newton needs rate/12."""
    report = fix_asset(
        arm,
        tmp_path / "out",
        RepairOptions(backends_requested="newton", target_frequency=10.0, control_rate=60.0),
    )
    warnings = [
        r
        for r in report["records"]
        if r["status"] == "reported" and r.get("severity") == "warning" and "control_rate" in r["reason"]
    ]
    assert warnings, "10 Hz at a 60 Hz rate diverged in Newton and must warn"
    assert "Newton" in warnings[0]["reason"]

    # The same 10 Hz is fine for PhysX alone, which was measured stable at /6.
    quiet = fix_asset(
        arm,
        tmp_path / "out2",
        RepairOptions(backends_requested="physx", target_frequency=10.0, control_rate=60.0),
    )
    assert not [
        r for r in quiet["records"] if r["status"] == "reported" and "control_rate" in r.get("reason", "")
    ]


def test_the_default_frequency_depends_on_the_backend_selection(arm, tmp_path):
    """One backend can be driven as stiffly as that backend tolerates.

    A cross-backend asset can only be as stiff as its least tolerant consumer,
    which the dt sweep measured to be Newton.
    """
    single = fix_asset(arm, tmp_path / "a", RepairOptions(backends_requested="physx"))
    assert single["options"]["tuning"]["target_frequency_hz"] == pytest.approx(DEFAULTS.control_rate / 6.0)

    newton_only = fix_asset(arm, tmp_path / "b", RepairOptions(backends_requested="newton"))
    assert newton_only["options"]["tuning"]["target_frequency_hz"] == pytest.approx(
        DEFAULTS.control_rate / 12.0
    )

    both = fix_asset(arm, tmp_path / "c", RepairOptions(backends_requested="physx,mujoco,newton"))
    assert both["options"]["tuning"]["target_frequency_hz"] == pytest.approx(DEFAULTS.control_rate / 12.0)
    assert "least tolerant consumer" in both["options"]["tuning_basis"]


def test_an_explicit_frequency_overrides_the_derived_one(arm, tmp_path):
    report = fix_asset(arm, tmp_path / "out", RepairOptions(backends_requested="physx", target_frequency=3.0))
    assert report["options"]["tuning"]["target_frequency_hz"] == pytest.approx(3.0)
    assert report["options"]["tuning_basis"] == "set explicitly on the command line"
