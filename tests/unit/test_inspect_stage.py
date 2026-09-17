# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Stage-level inspection: variants, collision, materials, units, CLI."""

from __future__ import annotations

import json
import subprocess
import sys

import pytest
from pxr import UsdPhysics

from urdf_usd_bridge.cli import run
from urdf_usd_bridge.inspection import inspect_stage
from urdf_usd_bridge.model import units
from urdf_usd_bridge.report import render_text

from . import builders


def test_stage_metrics_are_reported(tmp_path):
    stage = builders.new_stage("s")
    path = builders.save(stage, tmp_path / "s.usda")
    metrics = inspect_stage(path)["stage"]["metrics"]
    assert metrics["up_axis"] == "Z"
    assert metrics["meters_per_unit"] == pytest.approx(1.0)
    assert metrics["kilograms_per_unit"] == pytest.approx(1.0)


def test_collider_approximations_are_histogrammed(tmp_path):
    stage = builders.new_stage("c")
    builders.add_collider(stage, "/c/Geometry/link/mesh_a")
    builders.add_collider(stage, "/c/Geometry/link/mesh_b")
    builders.add_collider(stage, "/c/Geometry/link/mesh_c", approximation="convexDecomposition")
    path = builders.save(stage, tmp_path / "c.usda")

    summary = inspect_stage(path)["summary"]
    assert summary["colliders_total"] == 3
    assert summary["collider_approximations"] == {"convexHull": 2, "convexDecomposition": 1}
    assert summary["colliders_with_filtered_pairs"] == 0
    assert summary["colliders_with_physics_material"] == 0


def test_absence_of_physics_materials_and_scene_is_reported(tmp_path):
    stage = builders.new_stage("p")
    builders.add_collider(stage, "/p/Geometry/link/mesh")
    path = builders.save(stage, tmp_path / "p.usda")

    summary = inspect_stage(path)["summary"]
    assert summary["physics_materials_total"] == 0
    assert summary["physics_scenes_total"] == 0


def test_physics_material_is_found_when_present(tmp_path):
    stage = builders.new_stage("pm")
    material = stage.DefinePrim("/pm/Materials/rubber", "Material")
    UsdPhysics.MaterialAPI.Apply(material).CreateStaticFrictionAttr().Set(0.9)
    path = builders.save(stage, tmp_path / "pm.usda")

    report = inspect_stage(path)
    assert report["summary"]["physics_materials_total"] == 1
    attrs = report["physics_materials"][0]["attrs"]
    assert attrs["static_friction"]["value"] == pytest.approx(0.9)


def test_physics_variant_set_without_selection_is_reported(tmp_path):
    """Reproduces the Isaac Sim 6.1.0 interface-layer shape."""
    stage = builders.new_stage("v")
    root = stage.GetDefaultPrim()
    vset = root.GetVariantSets().AddVariantSet("Physics")
    for name in ("None", "mujoco", "physics", "physx"):
        vset.AddVariant(name)
    vset.ClearVariantSelection()
    path = builders.save(stage, tmp_path / "v.usda")

    report = inspect_stage(path)
    physics = report["summary"]["physics_variant_set"]
    assert physics is not None
    assert set(physics["variants"]) == {"None", "mujoco", "physics", "physx"}
    assert physics["has_authored_selection"] is False
    assert "no 'Physics' selection is authored" in render_text(report)


def test_variant_selection_is_applied_before_reading(tmp_path):
    """A joint that only exists inside the physx variant must appear once selected."""
    stage = builders.new_stage("vs")
    root = stage.GetDefaultPrim()
    vset = root.GetVariantSets().AddVariantSet("Physics")
    vset.AddVariant("None")
    vset.AddVariant("physx")
    vset.SetVariantSelection("physx")
    with vset.GetVariantEditContext():
        builders.add_revolute_joint(stage, "/vs/Physics/only_in_physx", lower=-10.0, upper=10.0)
    vset.SetVariantSelection("None")
    path = builders.save(stage, tmp_path / "vs.usda")

    assert inspect_stage(path)["summary"]["joints_total"] == 0
    selected = inspect_stage(path, {"Physics": "physx"})
    assert selected["summary"]["joints_total"] == 1
    assert selected["input"]["variant_selections"] == {"Physics": "physx"}


def test_schema_fallbacks_are_recorded(tmp_path):
    """The report states this USD build's fallbacks so readers need not guess."""
    stage = builders.new_stage("f")
    path = builders.save(stage, tmp_path / "f.usda")
    fallbacks = inspect_stage(path)["schema_fallbacks"]
    assert set(fallbacks) >= {"physics:mass", "physics:diagonalInertia", "physics:principalAxes"}
    assert fallbacks["physics:diagonalInertia"] == [0.0, 0.0, 0.0]


def test_report_is_json_serialisable(tmp_path):
    stage = builders.new_stage("j")
    builders.add_body(stage, "/j/Geometry/b", mass=1.0, diagonal_inertia=(1.0, 1.0, 1.0))
    builders.add_revolute_joint(stage, "/j/Physics/joint", lower=-1.0, upper=1.0, drive=True)
    builders.add_collider(stage, "/j/Geometry/b/mesh")
    path = builders.save(stage, tmp_path / "j.usda")

    payload = json.dumps(inspect_stage(path))
    assert json.loads(payload)["schema_version"] == 1


def test_text_render_runs_on_a_populated_report(tmp_path):
    stage = builders.new_stage("r")
    builders.add_body(stage, "/r/Geometry/b", mass=1.0, diagonal_inertia=(0.0, 0.0, 0.0))
    builders.add_revolute_joint(stage, "/r/Physics/joint", lower=0.0, upper=0.0, newton_damping=0.5)
    builders.add_collider(stage, "/r/Geometry/b/mesh")
    path = builders.save(stage, tmp_path / "r.usda")

    text = render_text(inspect_stage(path), verbose=True)
    assert "ZERO-INERTIA" in text
    assert "LOCKED" in text
    assert "damping-stranded" in text
    assert "Summary" in text


def test_cli_inspect_json_roundtrip(tmp_path, capsys):
    stage = builders.new_stage("cli")
    builders.add_revolute_joint(stage, "/cli/Physics/joint", lower=-1.0, upper=1.0)
    path = builders.save(stage, tmp_path / "cli.usda")

    assert run(["inspect", path, "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["summary"]["joints_total"] == 1


def test_cli_inspect_writes_output_file(tmp_path):
    stage = builders.new_stage("cli2")
    path = builders.save(stage, tmp_path / "cli2.usda")
    out = tmp_path / "report.json"

    assert run(["inspect", path, "--json", "-o", str(out)]) == 0
    assert json.loads(out.read_text())["schema_version"] == 1


def test_cli_rejects_malformed_variant():
    with pytest.raises(SystemExit):
        run(["inspect", "nope.usda", "--variant", "Physics"])


def test_cli_reports_missing_asset(tmp_path, capsys):
    assert run(["inspect", str(tmp_path / "missing.usda")]) == 1
    assert "error:" in capsys.readouterr().err


def test_console_script_is_installed():
    result = subprocess.run(
        [sys.executable, "-m", "urdf_usd_bridge", "--version"],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "urdf-usd-bridge" in result.stdout


def test_unit_conversions_round_trip():
    for value in (0.0, 1.5, 87.0, 1e-6):
        assert units.gain_usd_to_urdf(
            units.gain_urdf_to_usd(value, angular=True), angular=True
        ) == pytest.approx(value, rel=1e-12)
        assert units.gain_urdf_to_usd(value, angular=False) == value


def test_angular_gain_rescale_matches_the_documented_convention():
    """1.5 N*m*s/rad is 0.02617993877... N*m*s/deg; a 57.3x error is an explosion."""
    assert units.gain_urdf_to_usd(1.5, angular=True) == pytest.approx(0.026179938779914945)
