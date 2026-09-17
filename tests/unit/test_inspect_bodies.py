# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Inspector behaviour on mass, inertia and principal axes."""

from __future__ import annotations

import pytest

from urdf_usd_bridge.inspection import inspect_stage
from urdf_usd_bridge.inspection.run import _triangle_inequality_ok

from . import builders


def _body(report, name):
    return next(b for b in report["bodies"] if b["name"] == name)


def test_mass_without_inertia_is_flagged(tmp_path):
    stage = builders.new_stage("m")
    builders.add_body(stage, "/m/Geometry/heavy", mass=10.0, diagonal_inertia=(0.0, 0.0, 0.0))
    builders.add_body(stage, "/m/Geometry/ok", mass=1.0, diagonal_inertia=(0.004, 0.004, 0.001))
    path = builders.save(stage, tmp_path / "m.usda")

    report = inspect_stage(path)
    assert _body(report, "heavy")["inertia_checks"]["zero_inertia_with_mass"] is True
    assert _body(report, "ok")["inertia_checks"]["zero_inertia_with_mass"] is False
    assert report["summary"]["bodies_zero_inertia_with_mass"] == 1


def test_zero_principal_axes_quaternion_is_flagged(tmp_path):
    """The defect urdf-usd-converter 0.3.2 can author; 0.3.3 fixed it."""
    stage = builders.new_stage("q")
    builders.add_body(stage, "/q/Geometry/bad", mass=10.0, principal_axes=(0.0, 0.0, 0.0, 0.0))
    builders.add_body(stage, "/q/Geometry/good", mass=10.0, principal_axes=(1.0, 0.0, 0.0, 0.0))
    path = builders.save(stage, tmp_path / "q.usda")

    report = inspect_stage(path)
    bad = _body(report, "bad")["principal_axes_validity"]
    good = _body(report, "good")["principal_axes_validity"]
    assert bad["present"] is True and bad["is_zero"] is True
    assert good["is_zero"] is False and good["is_normalized"] is True
    assert report["summary"]["bodies_invalid_principal_axes"] == 1


def test_unauthored_principal_axes_is_not_a_finding(tmp_path):
    """0.3.3 leaves it unauthored; that must not be reported as a zero quaternion."""
    stage = builders.new_stage("q2")
    builders.add_body(stage, "/q2/Geometry/body", mass=10.0)
    path = builders.save(stage, tmp_path / "q2.usda")

    report = inspect_stage(path)
    assert _body(report, "body")["principal_axes_validity"]["present"] is False
    assert report["summary"]["bodies_invalid_principal_axes"] == 0


def test_negative_inertia_is_flagged(tmp_path):
    stage = builders.new_stage("n")
    builders.add_body(stage, "/n/Geometry/bad", mass=1.0, diagonal_inertia=(-0.1, 0.2, 0.2))
    path = builders.save(stage, tmp_path / "n.usda")

    report = inspect_stage(path)
    assert _body(report, "bad")["inertia_checks"]["negative_diagonal_inertia"] is True


@pytest.mark.parametrize(
    ("moments", "expected"),
    [
        ((1.0, 1.0, 1.0), True),
        ((1.0, 1.0, 2.0), True),
        ((1.0, 1.0, 2.0 + 1e-3), False),
        ((0.004, 0.004, 0.001), True),
        ((1.0, 2.0, 10.0), False),
    ],
)
def test_triangle_inequality_predicate(moments, expected):
    assert _triangle_inequality_ok(list(moments)) is expected


def test_triangle_inequality_violation_is_flagged(tmp_path):
    stage = builders.new_stage("t")
    builders.add_body(stage, "/t/Geometry/impossible", mass=1.0, diagonal_inertia=(1.0, 2.0, 10.0))
    path = builders.save(stage, tmp_path / "t.usda")

    report = inspect_stage(path)
    checks = _body(report, "impossible")["inertia_checks"]
    assert checks["diagonal_triangle_inequality_violated"] is True
    assert report["summary"]["bodies_triangle_inequality_violated"] == 1


def test_newton_inertia_is_checked_independently(tmp_path):
    stage = builders.new_stage("ni")
    # Indefinite tensor: large products of inertia force a negative eigenvalue.
    builders.add_body(
        stage,
        "/ni/Geometry/indefinite",
        mass=1.0,
        newton_inertia=[1.0, 1.0, 1.0, 5.0, 0.0, 0.0],
    )
    builders.add_body(
        stage,
        "/ni/Geometry/fine",
        mass=1.0,
        newton_inertia=[0.004, 0.004, 0.001, 0.0, 0.0, 0.0],
    )
    path = builders.save(stage, tmp_path / "ni.usda")

    report = inspect_stage(path)
    bad = _body(report, "indefinite")["inertia_checks"]
    good = _body(report, "fine")["inertia_checks"]
    assert bad["newton_inertia_not_positive_semidefinite"] is True
    assert good["newton_inertia_not_positive_semidefinite"] is False


def test_newton_and_usd_inertia_disagreement_is_flagged(tmp_path):
    stage = builders.new_stage("d")
    builders.add_body(
        stage,
        "/d/Geometry/mismatch",
        mass=1.0,
        diagonal_inertia=(0.004, 0.004, 0.001),
        newton_inertia=[0.5, 0.5, 0.5, 0.0, 0.0, 0.0],
    )
    builders.add_body(
        stage,
        "/d/Geometry/agree",
        mass=1.0,
        diagonal_inertia=(0.004, 0.004, 0.001),
        newton_inertia=[0.004, 0.004, 0.001, 0.0, 0.0, 0.0],
    )
    path = builders.save(stage, tmp_path / "d.usda")

    report = inspect_stage(path)
    assert _body(report, "mismatch")["inertia_checks"]["newton_inertia_disagrees_with_diagonal"] is True
    assert _body(report, "agree")["inertia_checks"]["newton_inertia_disagrees_with_diagonal"] is False


def test_mass_ratio_is_reported(tmp_path):
    stage = builders.new_stage("mr")
    builders.add_body(stage, "/mr/Geometry/base", mass=50.0)
    builders.add_body(stage, "/mr/Geometry/fingertip", mass=0.005)
    path = builders.save(stage, tmp_path / "mr.usda")

    report = inspect_stage(path)
    assert report["summary"]["mass_ratio_max"] == pytest.approx(10000.0)


def test_mass_with_no_inertia_authored_at_all_is_distinguished(tmp_path):
    """Undefined inertia and authored-zero inertia need different repairs."""
    stage = builders.new_stage("u")
    builders.add_body(stage, "/u/Geometry/undefined", mass=10.0)
    builders.add_body(stage, "/u/Geometry/authored_zero", mass=10.0, diagonal_inertia=(0.0, 0.0, 0.0))
    path = builders.save(stage, tmp_path / "u.usda")

    report = inspect_stage(path)
    undefined = _body(report, "undefined")["inertia_checks"]
    authored_zero = _body(report, "authored_zero")["inertia_checks"]

    assert undefined["mass_without_authored_inertia"] is True
    assert undefined["zero_inertia_with_mass"] is False
    assert authored_zero["mass_without_authored_inertia"] is False
    assert authored_zero["zero_inertia_with_mass"] is True

    assert report["summary"]["bodies_mass_without_authored_inertia"] == 1
    assert report["summary"]["bodies_zero_inertia_with_mass"] == 1


def test_principal_axes_schema_fallback_is_a_zero_quaternion(tmp_path):
    """Why 0.3.2's defect happens: the unauthored fallback is (0,0,0,0), so any
    code that Gets it and multiplies produces an invalid rotation."""
    stage = builders.new_stage("fb")
    path = builders.save(stage, tmp_path / "fb.usda")
    assert inspect_stage(path)["schema_fallbacks"]["physics:principalAxes"] == [0.0, 0.0, 0.0, 0.0]
