# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Properties of the output that are not about any one repair rule.

Three promises are tested here, because breaking any of them makes every other
result meaningless:

* the input is never modified,
* root-layer metadata survives, above all ``defaultPrim``,
* the same input produces byte-identical output.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
from pxr import Sdf, Usd, UsdGeom

from urdf_usd_bridge.repair import RepairOptions, fix_asset
from urdf_usd_bridge.repair.base import OptionError

from .repair_builders import export, simple_arm, variant_asset


def _digest_tree(root: Path) -> dict[str, str]:
    return {
        str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(root.rglob("*"))
        if p.is_file()
    }


@pytest.fixture
def flat_asset(tmp_path):
    source_dir = tmp_path / "src"
    source_dir.mkdir()
    return export(simple_arm(), source_dir / "robot.usda"), source_dir


def test_the_input_is_never_modified(flat_asset, tmp_path):
    source, source_dir = flat_asset
    before = _digest_tree(source_dir)
    fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    assert _digest_tree(source_dir) == before


def test_default_prim_survives(flat_asset, tmp_path):
    """The one that bit us first.

    ``defaultPrim`` is root-layer metadata and is **not** composed from
    sublayers. A new root layer that omits it opens with no default prim, which
    silently disables every variant selection and makes an Isaac Sim package
    compose as a geometry-only asset -- with no error anywhere.
    """
    source, _ = flat_asset
    out = tmp_path / "out"
    report = fix_asset(source, out, RepairOptions(backends_requested="physx"))

    root = out / "robot_stabilized.usda"
    assert Sdf.Layer.FindOrOpen(str(root)).defaultPrim == "robot"

    stage = Usd.Stage.Open(str(root), Usd.Stage.LoadAll)
    default_prim = stage.GetDefaultPrim()
    assert default_prim and default_prim.IsValid()
    assert default_prim.GetPath().pathString == "/robot"
    assert report["output"]["metadata_copied"]["defaultPrim"] == "robot"


def test_stage_metrics_survive(flat_asset, tmp_path):
    """``upAxis`` and the unit metrics travel with ``defaultPrim``.

    Losing ``metersPerUnit`` rescales the whole robot in any consumer that
    honours it, which is the "it exploded in the other app" class of bug.
    """
    source, _ = flat_asset
    out = tmp_path / "out"
    report = fix_asset(source, out, RepairOptions(backends_requested="physx"))
    stage = Usd.Stage.Open(str(out / "robot_stabilized.usda"), Usd.Stage.LoadAll)

    assert UsdGeom.GetStageUpAxis(stage) == "Z"
    assert UsdGeom.GetStageMetersPerUnit(stage) == pytest.approx(1.0)
    assert stage.GetMetadata("kilogramsPerUnit") == pytest.approx(1.0)
    assert set(report["output"]["metadata_copied"]) == {
        "defaultPrim",
        "upAxis",
        "metersPerUnit",
        "kilogramsPerUnit",
    }


def test_non_metre_stage_metrics_survive(tmp_path):
    """A centimetre asset must come back out as a centimetre asset."""
    stage = simple_arm()
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    source_dir = tmp_path / "src"
    source_dir.mkdir()
    source = export(stage, source_dir / "robot.usda")

    fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="physx"))
    out_stage = Usd.Stage.Open(str(tmp_path / "out" / "robot_stabilized.usda"), Usd.Stage.LoadAll)
    assert UsdGeom.GetStageMetersPerUnit(out_stage) == pytest.approx(0.01)


def test_output_is_byte_identical_across_runs(flat_asset, tmp_path):
    """Determinism, as promised in docs/PHASE3_DESIGN.md section 7.

    No timestamp, no hostname, no absolute path in any authored layer, sorted
    traversal, and a canonicalised eigendecomposition.
    """
    source, _ = flat_asset
    first = tmp_path / "first"
    second = tmp_path / "second"
    fix_asset(source, first, RepairOptions(backends_requested="physx,mujoco"))
    fix_asset(source, second, RepairOptions(backends_requested="physx,mujoco"))

    assert _digest_tree(first) == _digest_tree(second)


def test_no_timestamp_leaks_into_the_layers(flat_asset, tmp_path):
    """A clock is the easiest way to lose byte-identical output."""
    source, _ = flat_asset
    out = tmp_path / "out"
    fix_asset(source, out, RepairOptions(backends_requested="physx"))
    for layer in out.glob("*.usda"):
        text = layer.read_text()
        assert "202" not in text.replace("2026 urdf-usd-bridge", ""), f"{layer.name} looks timestamped"


def test_tuning_is_recorded_in_the_layer_metadata(flat_asset, tmp_path):
    """An asset must carry the assumptions it was built with."""
    source, _ = flat_asset
    out = tmp_path / "out"
    fix_asset(
        source,
        out,
        RepairOptions(backends_requested="physx", target_frequency=7.5, damping_ratio=0.8),
    )
    data = Sdf.Layer.FindOrOpen(str(out / "robot_stabilized.usda")).customLayerData
    recorded = data["urdf_usd_bridge"]
    assert recorded["tuning"]["target_frequency_hz"] == pytest.approx(7.5)
    assert recorded["tuning"]["damping_ratio"] == pytest.approx(0.8)
    assert "unmeasured" in recorded["tuning_status"]
    assert recorded["input_sha256"] == hashlib.sha256(Path(source).read_bytes()).hexdigest()


def test_muting_the_stability_layers_restores_the_input(flat_asset, tmp_path):
    """The repairs are an overlay, not a rewrite."""
    source, _ = flat_asset
    out = tmp_path / "out"
    fix_asset(source, out, RepairOptions(backends_requested="physx"))

    stage = Usd.Stage.Open(str(out / "robot_stabilized.usda"), Usd.Stage.LoadAll)
    joint = stage.GetPrimAtPath("/robot/Physics/shoulder")
    assert joint.GetAttribute("drive:angular:physics:stiffness").HasAuthoredValue()

    for name in ("Stability.usda", "Stability_physx.usda"):
        stage.MuteLayer(str(out / name))
    joint = stage.GetPrimAtPath("/robot/Physics/shoulder")
    attr = joint.GetAttribute("drive:angular:physics:stiffness")
    assert not attr or not attr.HasAuthoredValue()


def test_backend_all_is_refused_without_a_variant_set(flat_asset, tmp_path):
    """Decision D2: refuse rather than author conflicting gain conventions."""
    source, _ = flat_asset
    with pytest.raises(OptionError) as excinfo:
        fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="all"))
    message = str(excinfo.value)
    assert "Physics" in message and "--backend physx" in message


def test_backend_all_is_accepted_with_a_variant_set(tmp_path):
    source = export(variant_asset(), tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="all"))
    assert report["output"]["variant_scoped"] is True
    assert set(report["options"]["backends"]) == {"physx", "mujoco", "newton"}


def test_the_output_never_pins_a_backend(tmp_path):
    """Which physics a consumer loads stays their decision."""
    source = export(variant_asset(), tmp_path / "robot.usda")
    out = tmp_path / "out"
    fix_asset(source, out, RepairOptions(backends_requested="all"))

    stage = Usd.Stage.Open(str(out / "robot_stabilized.usda"))
    selection = stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").GetVariantSelection()
    assert not selection, f"the stabilized asset pins Physics={selection!r}"


def test_dry_run_writes_nothing_but_reports_the_same_plan(flat_asset, tmp_path):
    source, _ = flat_asset
    out = tmp_path / "out"
    dry = fix_asset(source, None, RepairOptions(backends_requested="physx", dry_run=True))
    wet = fix_asset(source, out, RepairOptions(backends_requested="physx"))

    assert dry["output"]["written"] is False
    assert not out.exists() or not list(out.glob("Stability*.usda")) or wet["output"]["written"]

    def applied(report):
        return [
            (r["rule"], r["prim"], r["attribute"], r["new"])
            for r in report["records"]
            if r["status"] == "applied"
        ]

    assert applied(dry) == applied(wet)


def test_dry_run_requires_no_output_directory(flat_asset):
    source, _ = flat_asset
    report = fix_asset(source, None, RepairOptions(backends_requested="physx", dry_run=True))
    assert report["summary"]["records_total"] > 0
    assert report["output"]["root"] is None
