# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""``fix`` then ``inspect``, on real converter output.

``tests/unit`` proves each rule against hand-authored stages. This module runs
the repairs over assets a real ``urdf-usd-converter`` produced, and then asks
the inspector whether the gaps from ``docs/history/ANALYSIS.md`` are actually closed.
That round trip is the only test that can catch a repair which is individually
correct but does not compose -- authored in the wrong layer, on the wrong prim,
or under a variant nobody selects.

Artifacts come from ``scripts/run_converter_matrix.py``; every test skips
without them.
"""

from __future__ import annotations

import json
import math

import pytest

from urdf_usd_bridge.inspection import inspect_stage
from urdf_usd_bridge.repair import RepairOptions, fix_asset

from ..conftest import ARTIFACTS

pytestmark = pytest.mark.converter

#: Every matrix column, including the one pinned to Isaac Sim 6.1.0's stack.
COLUMNS = ("0.3.2", "0.3.3", "0.3.2-isaac")

DEG_PER_RAD = 180.0 / math.pi


def _asset(column: str, fixture: str) -> str:
    matrix_path = ARTIFACTS / "matrix.json"
    if not matrix_path.exists():
        pytest.skip(
            "no converter artifacts; run `python scripts/run_converter_matrix.py` "
            "on a platform where usd-exchange installs (Linux or Windows)"
        )
    matrix = json.loads(matrix_path.read_text())
    entry = matrix.get("versions", {}).get(column)
    if not entry or not entry.get("env", {}).get("ok"):
        env = (entry or {}).get("env", {})
        pytest.skip(f"converter {column} unavailable: {env.get('cause') or 'env not built'}")
    run = entry.get("runs", {}).get(fixture)
    if not run or not run.get("ok"):
        pytest.fail(f"converter {column} failed on {fixture}: {(run or {}).get('error')}")
    return run["asset"]


def _fix(column, fixture, tmp_path, **options):
    source = _asset(column, fixture)
    out = tmp_path / f"{column}-{fixture}"
    report = fix_asset(source, out, RepairOptions(**options))
    return report, str(out / f"{fixture}_stabilized.usda")


@pytest.mark.parametrize("column", COLUMNS)
def test_g1_every_actuatable_joint_gains_a_populated_drive(column, tmp_path):
    """The headline: no actuatable joint is left with an empty drive."""
    before = inspect_stage(_asset(column, "a_dynamics_damping"))
    assert before["summary"]["joints_with_drive_gains"] == 0
    assert before["summary"]["joints_damping_stranded_outside_drive"] == 2

    _, stabilized = _fix(column, "a_dynamics_damping", tmp_path, backends_requested="physx")
    after = inspect_stage(stabilized)

    actuatable = after["summary"]["joints_actuatable"]
    assert actuatable == 2
    assert after["summary"]["joints_with_drive_gains"] == actuatable
    assert after["summary"]["joints_drive_applied_without_gains"] == 0
    assert after["summary"]["joints_damping_stranded_outside_drive"] == 0


@pytest.mark.parametrize("column", COLUMNS)
def test_g1_the_urdf_damping_reaches_the_drive_with_the_right_units(column, tmp_path):
    """1.5 N*m*s/rad on the revolute joint, 4.0 N*s/m on the prismatic one."""
    _, stabilized = _fix(column, "a_dynamics_damping", tmp_path, backends_requested="physx")
    after = inspect_stage(stabilized)

    revolute = next(j for j in after["joints"] if j["name"] == "shoulder_joint")
    drive = revolute["drives"]["angular"]
    total_si = drive["damping"]["value"] * DEG_PER_RAD
    # The drive carries the derived term plus the URDF's 1.5; it cannot be less.
    assert total_si > 1.5
    assert revolute["damping_by_namespace"]["newton"]["value"] == pytest.approx(1.5 / DEG_PER_RAD, rel=1e-5)

    prismatic = next(j for j in after["joints"] if j["name"] == "slide_joint")
    linear = prismatic["drives"]["linear"]
    assert linear["damping"]["value"] > 4.0  # linear: no angle conversion at all
    assert linear["stiffness"]["value"] > 0


@pytest.mark.parametrize("column", COLUMNS)
def test_g3_armature_is_authored_everywhere_it_was_missing(column, tmp_path):
    before = inspect_stage(_asset(column, "a_dynamics_damping"))
    assert before["summary"]["armature_authored_by_namespace"] == {}
    assert before["summary"]["joints_without_armature"] == 2

    _, stabilized = _fix(column, "a_dynamics_damping", tmp_path, backends_requested="physx")
    after = inspect_stage(stabilized)

    namespaces = after["summary"]["armature_authored_by_namespace"]
    assert namespaces.get("newton") == 2
    assert namespaces.get("physx") == 2
    assert after["summary"]["joints_without_armature"] == 0


def test_g2_the_zero_principal_axes_defect_is_repaired_on_0_3_2(tmp_path):
    """0.3.2 authors ``(0,0,0,0)``; after the fix nothing is invalid."""
    before = inspect_stage(_asset("0.3.2", "b_inertial_origin_mass_no_inertia"))
    assert before["summary"]["bodies_invalid_principal_axes"] == 2
    assert before["summary"]["bodies_mass_without_authored_inertia"] == 2

    _, stabilized = _fix("0.3.2", "b_inertial_origin_mass_no_inertia", tmp_path, backends_requested="physx")
    after = inspect_stage(stabilized)

    assert after["summary"]["bodies_invalid_principal_axes"] == 0
    assert after["summary"]["bodies_mass_without_authored_inertia"] == 0
    assert after["summary"]["bodies_zero_inertia_with_mass"] == 0


def test_g2_0_3_3_needs_the_tensor_but_not_the_quaternion_fix(tmp_path):
    """0.3.3 leaves ``principalAxes`` unauthored rather than zero.

    Same end state, different route -- which is why the rule keys on the
    authored/fallback distinction rather than on the value alone.
    """
    before = inspect_stage(_asset("0.3.3", "b_inertial_origin_mass_no_inertia"))
    assert before["summary"]["bodies_invalid_principal_axes"] == 0
    assert before["summary"]["bodies_mass_without_authored_inertia"] == 2

    report, stabilized = _fix(
        "0.3.3", "b_inertial_origin_mass_no_inertia", tmp_path, backends_requested="physx"
    )
    after = inspect_stage(stabilized)
    assert after["summary"]["bodies_mass_without_authored_inertia"] == 0

    identity_fixes = [r for r in report["records"] if r["rule"] == "inertia.principal-axes-identity"]
    assert not identity_fixes, "0.3.3 authors no zero quaternion, so nothing to repair"


@pytest.mark.parametrize("column", COLUMNS)
def test_g7_the_welded_joint_is_unlocked_and_the_ambiguous_one_is_not(column, tmp_path):
    before = inspect_stage(_asset(column, "c_revolute_no_limit"))
    assert before["summary"]["joints_locked_by_equal_limits"] == 2

    report, stabilized = _fix(column, "c_revolute_no_limit", tmp_path, backends_requested="physx")
    after = inspect_stage(stabilized)

    # One is repaired; the one with limit evidence is deliberately left alone.
    assert after["summary"]["joints_locked_by_equal_limits"] == 1
    still_locked = [j["name"] for j in after["joints"] if j["flags"]["locked_by_equal_limits"]]
    assert still_locked == ["partial_limit_joint"]

    ambiguous = [r for r in report["records"] if r["rule"] == "limits.report-ambiguous"]
    assert len(ambiguous) == 1
    assert ambiguous[0]["prim"].endswith("partial_limit_joint")

    unaffected = next(j for j in after["joints"] if j["name"] == "continuous_joint")
    assert unaffected["limits"]["lower"]["value"] == -math.inf


@pytest.mark.parametrize("column", COLUMNS)
def test_the_mesh_fixture_keeps_its_convex_hull_and_gains_no_collision_repairs(column, tmp_path):
    """Fixture (d) closes the convexHull question, and G4 stays out of scope.

    Phase 3 authors nothing about collision, so the approximation the converter
    chose must survive untouched -- and the repair report must contain no
    collision records at all.
    """
    before = inspect_stage(_asset(column, "d_mesh_collision"))
    assert before["summary"]["collider_approximations"].get("convexHull") == 1

    report, stabilized = _fix(column, "d_mesh_collision", tmp_path, backends_requested="physx")
    after = inspect_stage(stabilized)

    assert after["summary"]["collider_approximations"] == before["summary"]["collider_approximations"]
    assert after["summary"]["colliders_with_filtered_pairs"] == 0
    assert after["summary"]["physics_materials_total"] == 0
    assert not [
        r
        for r in report["records"]
        if "collision" in (r["attribute"] or "") or "material" in (r["attribute"] or "")
    ]


@pytest.mark.parametrize("column", COLUMNS)
def test_a_mesh_collider_does_not_break_the_inertia_derivation(column, tmp_path):
    """The mesh link declares its own inertia, so nothing should be derived."""
    report, stabilized = _fix(column, "d_mesh_collision", tmp_path, backends_requested="physx")
    after = inspect_stage(stabilized)
    assert after["summary"]["bodies_zero_inertia_with_mass"] == 0
    assert report["summary"]["errors"] == 0


@pytest.mark.parametrize("column", COLUMNS)
def test_the_stabilized_asset_still_inspects_as_the_same_robot(column, tmp_path):
    """Repairs must not change the topology, only the dynamics."""
    before = inspect_stage(_asset(column, "a_dynamics_damping"))
    _, stabilized = _fix(column, "a_dynamics_damping", tmp_path, backends_requested="physx")
    after = inspect_stage(stabilized)

    assert [b["path"] for b in after["bodies"]] == [b["path"] for b in before["bodies"]]
    assert [j["path"] for j in after["joints"]] == [j["path"] for j in before["joints"]]
    assert after["stage"]["metrics"]["up_axis"] == before["stage"]["metrics"]["up_axis"]
    assert after["stage"]["metrics"]["meters_per_unit"] == pytest.approx(
        before["stage"]["metrics"]["meters_per_unit"]
    )
    assert after["stage"]["default_prim"] == before["stage"]["default_prim"]


def test_the_isaac_pinned_column_agrees_with_the_plain_one(tmp_path):
    """The repairs must not depend on which usd-exchange resolved.

    ``0.3.2`` and ``0.3.2-isaac`` run the same converter against different
    ``usd-exchange``/``newton-usd-schemas`` versions. If a repair value differs
    between them, something we author is version-sensitive and the tested
    matrix in README.md is understating the risk.
    """
    plain, _ = _fix("0.3.2", "a_dynamics_damping", tmp_path, backends_requested="physx")
    pinned, _ = _fix("0.3.2-isaac", "a_dynamics_damping", tmp_path, backends_requested="physx")

    def applied(report):
        return {
            (r["prim"].split("/Physics/")[-1], r["attribute"]): r["new"]
            for r in report["records"]
            if r["status"] == "applied" and isinstance(r["new"], (int, float))
        }

    plain_values, pinned_values = applied(plain), applied(pinned)
    assert set(plain_values) == set(pinned_values)
    for key, value in plain_values.items():
        assert pinned_values[key] == pytest.approx(value, rel=1e-6), key
