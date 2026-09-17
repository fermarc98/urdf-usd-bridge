# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""What each urdf-usd-converter version actually authors for our fixtures.

These are the behavioural counterpart to ``tests/unit/test_reference_sources.py``:
that module proves what the upstream *source* says, this one proves what the
upstream *code does*.

Artifacts come from ``scripts/run_converter_matrix.py``, which builds one
virtualenv per converter version. Without those artifacts every test here
skips, which is what happens on macOS, where ``usd-exchange`` -- and therefore
``urdf-usd-converter`` -- cannot be installed at all.

    python scripts/run_converter_matrix.py
    pytest tests/converter
"""

from __future__ import annotations

import json
import math

import pytest

from ..conftest import ARTIFACTS

pytestmark = pytest.mark.converter

VERSIONS = ("0.3.2", "0.3.3")

#: 1.5 N*m*s/rad expressed in NewtonJointAPI's per-degree convention.
REVOLUTE_DAMPING_PER_DEGREE = 1.5 * math.pi / 180.0


def _load(version: str, fixture: str) -> dict:
    matrix_path = ARTIFACTS / "matrix.json"
    if not matrix_path.exists():
        pytest.skip(
            "no converter artifacts; run `python scripts/run_converter_matrix.py` "
            "on a platform where usd-exchange installs (Linux or Windows)"
        )
    matrix = json.loads(matrix_path.read_text())
    entry = matrix.get("versions", {}).get(version)
    if not entry or not entry.get("env", {}).get("ok"):
        env = (entry or {}).get("env", {})
        reason = env.get("cause") or env.get("error") or "environment was not built"
        pytest.skip(f"converter {version} unavailable: {reason}")
    run = entry.get("runs", {}).get(fixture)
    if not run or not run.get("ok"):
        pytest.fail(f"converter {version} failed on {fixture}: {(run or {}).get('error')}")
    report_path = ARTIFACTS / version / f"{fixture}.json"
    return json.loads(report_path.read_text())


def _joint(report: dict, name: str) -> dict:
    matches = [j for j in report["joints"] if j["name"] == name]
    assert matches, f"joint {name!r} not in {[j['name'] for j in report['joints']]}"
    return matches[0]


def _body(report: dict, name: str) -> dict:
    matches = [b for b in report["bodies"] if b["name"] == name]
    assert matches, f"body {name!r} not in {[b['name'] for b in report['bodies']]}"
    return matches[0]


def _authored(reading):
    return reading["value"] if reading and reading["authored"] else None


# --------------------------------------------------------------------------
# fixture (a): <dynamics damping friction>
# --------------------------------------------------------------------------


@pytest.mark.parametrize("version", VERSIONS)
def test_a_revolute_damping_lands_in_newton_namespace_only(version):
    report = _load(version, "a_dynamics_damping")
    joint = _joint(report, "shoulder_joint")
    damping = joint["damping_by_namespace"]

    assert _authored(damping["newton"]) == pytest.approx(REVOLUTE_DAMPING_PER_DEGREE, rel=1e-6)
    assert damping["urdf_custom"] is None, (
        "urdf:dynamics:damping is the attribute Isaac Sim 6.1.0 reads; if it is "
        "present this version of the converter still writes it"
    )
    assert damping["mjc"] is None


@pytest.mark.parametrize("version", VERSIONS)
def test_a_revolute_friction_lands_in_newton_namespace_only(version):
    joint = _joint(_load(version, "a_dynamics_damping"), "shoulder_joint")
    friction = joint["friction_by_namespace"]

    assert _authored(friction["newton"]) == pytest.approx(0.3, rel=1e-6)
    assert friction["urdf_custom"] is None
    assert friction["physx"] is None
    assert friction["mjc"] is None


@pytest.mark.parametrize("version", VERSIONS)
def test_a_prismatic_damping_is_not_rescaled(version):
    """Linear DOFs need no per-degree conversion; 4.0 must stay 4.0."""
    joint = _joint(_load(version, "a_dynamics_damping"), "slide_joint")
    assert _authored(joint["damping_by_namespace"]["newton"]) == pytest.approx(4.0, rel=1e-6)
    assert _authored(joint["friction_by_namespace"]["newton"]) == pytest.approx(0.1, rel=1e-6)


@pytest.mark.parametrize("version", VERSIONS)
def test_a_effort_is_only_an_inert_custom_attribute(version):
    report = _load(version, "a_dynamics_damping")
    assert _authored(_joint(report, "shoulder_joint")["limit_extras"]["urdf_effort"]) == pytest.approx(87.0)
    assert _authored(_joint(report, "slide_joint")["limit_extras"]["urdf_effort"]) == pytest.approx(50.0)
    assert report["summary"]["joints_effort_only_as_urdf_custom"] == 2


@pytest.mark.parametrize("version", VERSIONS)
def test_a_velocity_limit_is_converted_for_angular_dofs_only(version):
    report = _load(version, "a_dynamics_damping")
    revolute = _joint(report, "shoulder_joint")["limit_extras"]["newton_velocity_limit"]
    prismatic = _joint(report, "slide_joint")["limit_extras"]["newton_velocity_limit"]
    assert _authored(revolute) == pytest.approx(math.degrees(2.5), rel=1e-6)
    assert _authored(prismatic) == pytest.approx(0.5, rel=1e-6)


@pytest.mark.parametrize("version", VERSIONS)
def test_a_no_actuation_is_authored_at_all(version):
    """G1, first half: no DriveAPI, no armature, no MjcActuator."""
    summary = _load(version, "a_dynamics_damping")["summary"]
    assert summary["joints_actuatable"] == 2
    assert summary["joints_with_drive_api"] == 0
    assert summary["joints_with_drive_gains"] == 0
    assert summary["joints_damping_stranded_outside_drive"] == 2
    assert summary["joints_without_armature"] == 2
    assert summary["mjc_actuators_total"] == 0


@pytest.mark.parametrize("version", VERSIONS)
def test_a_no_physics_material_or_collision_filtering(version):
    """G4 and G5 on a real conversion."""
    summary = _load(version, "a_dynamics_damping")["summary"]
    assert summary["physics_materials_total"] == 0
    assert summary["colliders_with_physics_material"] == 0
    assert summary["colliders_with_filtered_pairs"] == 0
    assert summary["collision_groups_total"] == 0


# --------------------------------------------------------------------------
# fixture (b): <inertial> with origin + mass, no <inertia>
# --------------------------------------------------------------------------


@pytest.mark.parametrize("version", VERSIONS)
def test_b_mass_is_authored_but_inertia_is_not(version):
    report = _load(version, "b_inertial_origin_mass_no_inertia")
    body = _body(report, "no_inertia_link")

    assert _authored(body["mass"]["mass"]) == pytest.approx(10.0)
    assert body["inertia_checks"]["mass_without_authored_inertia"] is True
    assert report["summary"]["bodies_mass_without_authored_inertia"] >= 2


def test_b_032_authors_a_zero_principal_axes_quaternion():
    """The defect Isaac Sim 6.1.0 ships, because it pins 0.3.2."""
    body = _body(_load("0.3.2", "b_inertial_origin_mass_no_inertia"), "no_inertia_link")
    axes = body["principal_axes_validity"]
    assert axes["present"] is True
    assert axes["is_zero"] is True


def test_b_033_leaves_principal_axes_unauthored():
    """0.3.3: 'Fixed an invalid zero physics:principalAxes ... no <inertia>'."""
    body = _body(_load("0.3.3", "b_inertial_origin_mass_no_inertia"), "no_inertia_link")
    assert body["principal_axes_validity"]["present"] is False


@pytest.mark.parametrize("version", VERSIONS)
def test_b_fully_specified_control_link_stays_clean(version):
    body = _body(_load(version, "b_inertial_origin_mass_no_inertia"), "control_link")
    checks = body["inertia_checks"]
    assert checks["mass_without_authored_inertia"] is False
    assert checks["zero_inertia_with_mass"] is False
    assert checks["negative_diagonal_inertia"] is False
    assert checks["diagonal_triangle_inequality_violated"] is False
    assert body["principal_axes_validity"]["is_zero"] is False


# --------------------------------------------------------------------------
# fixture (c): revolute joint with no <limit>
# --------------------------------------------------------------------------


@pytest.mark.parametrize("version", VERSIONS)
def test_c_missing_limit_becomes_a_locked_joint(version):
    """G7: a revolute joint with no <limit> is authored welded shut."""
    report = _load(version, "c_revolute_no_limit")
    for name in ("no_limit_joint", "partial_limit_joint"):
        joint = _joint(report, name)
        assert _authored(joint["limits"]["lower"]) == pytest.approx(0.0)
        assert _authored(joint["limits"]["upper"]) == pytest.approx(0.0)
        assert joint["flags"]["locked_by_equal_limits"] is True


@pytest.mark.parametrize("version", VERSIONS)
def test_c_continuous_joint_is_left_unbounded(version):
    """Negative control: a continuous joint must not be locked."""
    joint = _joint(_load(version, "c_revolute_no_limit"), "continuous_joint")
    assert joint["flags"]["locked_by_equal_limits"] is False
    assert _authored(joint["limits"]["lower"]) is None
    assert _authored(joint["limits"]["upper"]) is None


@pytest.mark.parametrize("version", VERSIONS)
def test_c_ordinary_limits_are_converted_to_degrees(version):
    joint = _joint(_load(version, "c_revolute_no_limit"), "limited_joint")
    assert _authored(joint["limits"]["lower"]) == pytest.approx(math.degrees(-0.785), rel=1e-5)
    assert _authored(joint["limits"]["upper"]) == pytest.approx(math.degrees(0.785), rel=1e-5)
    assert joint["flags"]["locked_by_equal_limits"] is False


@pytest.mark.parametrize("version", VERSIONS)
def test_c_locked_joint_count(version):
    summary = _load(version, "c_revolute_no_limit")["summary"]
    assert summary["joints_actuatable"] == 4
    assert summary["joints_locked_by_equal_limits"] == 2


# --------------------------------------------------------------------------
# structural expectations shared by all fixtures
# --------------------------------------------------------------------------


@pytest.mark.parametrize("version", VERSIONS)
@pytest.mark.parametrize(
    "fixture",
    ["a_dynamics_damping", "b_inertial_origin_mass_no_inertia", "c_revolute_no_limit"],
)
def test_every_mesh_collider_is_a_convex_hull(version, fixture):
    """G4: boxes stay boxes, but any mesh collider only ever gets convexHull."""
    summary = _load(version, fixture)["summary"]
    assert set(summary["collider_approximations"]) <= {"convexHull", "(unauthored)"}


@pytest.mark.parametrize("version", VERSIONS)
@pytest.mark.parametrize(
    "fixture",
    ["a_dynamics_damping", "b_inertial_origin_mass_no_inertia", "c_revolute_no_limit"],
)
def test_stage_metrics_are_explicit(version, fixture):
    metrics = _load(version, fixture)["stage"]["metrics"]
    assert metrics["up_axis"] == "Z"
    assert metrics["meters_per_unit"] == pytest.approx(1.0)
    assert metrics["kilograms_per_unit"] == pytest.approx(1.0)
