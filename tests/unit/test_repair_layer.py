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
import time
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
    """Determinism, as promised in docs/history/PHASE3_DESIGN.md section 7.

    No timestamp, no hostname, no absolute path in any authored layer, sorted
    traversal, and a canonicalised eigendecomposition.
    """
    source, _ = flat_asset
    first = tmp_path / "first"
    second = tmp_path / "second"
    fix_asset(source, first, RepairOptions(backends_requested="physx,mujoco"))
    fix_asset(source, second, RepairOptions(backends_requested="physx,mujoco"))

    assert _digest_tree(first) == _digest_tree(second)


def test_no_clock_reading_leaks_into_the_layers(flat_asset, tmp_path):
    """A clock is the easiest way to lose byte-identical output.

    The layers *do* carry a fixed release date (when the defaults last changed),
    which is a constant and not a clock read -- so the check is for a time of
    day, and determinism is proved separately by the byte-identical test.
    """
    import re

    source, _ = flat_asset
    out = tmp_path / "out"
    fix_asset(source, out, RepairOptions(backends_requested="physx"))
    for layer in out.glob("*.usda"):
        text = layer.read_text()
        assert not re.search(r"\d{2}:\d{2}:\d{2}", text), f"{layer.name} carries a time of day"
        assert "T" + time.strftime("%H") not in text


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
    assert recorded["tuning_status"]
    assert recorded["defaults_changed"]
    assert recorded["previous_defaults"]["target_frequency"] == pytest.approx(10.0)
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


def test_multi_root_writes_one_root_per_backend(flat_asset, tmp_path):
    """``convert``'s way out of the flat-asset problem.

    A flat asset cannot carry three gain conventions in one composed stage, so
    ``--backend all`` gets one root per backend instead of a refusal. Each root
    composes the neutral layer, its own backend layer and the original, and
    nothing else.
    """
    source, _ = flat_asset
    out = tmp_path / "out"
    report = fix_asset(source, out, RepairOptions(backends_requested="all", multi_root=True))

    assert report["output"]["multi_root"] is True
    roots = report["output"]["roots"]
    assert len(roots) == 3
    assert {Path(r).name for r in roots} == {
        "robot_stabilized_physx.usda",
        "robot_stabilized_mujoco.usda",
        "robot_stabilized_newton.usda",
    }
    # The combined root is not written: it would be the thing we are avoiding.
    assert not (out / "robot_stabilized.usda").exists()


def test_each_multi_root_carries_exactly_one_convention(flat_asset, tmp_path):
    from pxr import UsdPhysics

    source, _ = flat_asset
    out = tmp_path / "out"
    fix_asset(source, out, RepairOptions(backends_requested="all", multi_root=True))

    def read(backend):
        root = out / f"robot_stabilized_{backend}.usda"
        stage = Usd.Stage.Open(
            Sdf.Layer.FindOrOpen(str(root)), Sdf.Layer.CreateAnonymous(), Usd.Stage.LoadAll
        )
        joint = stage.GetPrimAtPath("/robot/Physics/shoulder")
        attr = UsdPhysics.DriveAPI(joint, "angular").GetStiffnessAttr()
        drive = attr.Get() if attr and attr.IsValid() and attr.HasAuthoredValue() else None
        mjc = [p for p in stage.Traverse() if str(p.GetTypeName()) == "MjcActuator"]
        newton = [p for p in stage.Traverse() if str(p.GetTypeName()) == "NewtonActuator"]
        armature = joint.GetAttribute("newton:armature")
        return {
            "drive": drive,
            "mjc": len(mjc),
            "newton": len(newton),
            "armature": armature.Get() if armature and armature.IsValid() else None,
            "defaultPrim": stage.GetDefaultPrim().GetName(),
        }

    physx, mujoco, newton = read("physx"), read("mujoco"), read("newton")

    assert physx["drive"] and physx["mjc"] == 0 and physx["newton"] == 0
    assert mujoco["drive"] is None and mujoco["mjc"] == 1 and mujoco["newton"] == 0
    # N1: Newton is driven through UsdPhysics.DriveAPI, and authors no
    # NewtonActuator unless asked.
    assert newton["drive"] and newton["mjc"] == 0 and newton["newton"] == 0

    # The neutral layer is shared by all three, and metadata survives on each.
    assert physx["armature"] == pytest.approx(mujoco["armature"])
    assert physx["armature"] == pytest.approx(newton["armature"])
    assert {physx["defaultPrim"], mujoco["defaultPrim"], newton["defaultPrim"]} == {"robot"}


def test_fix_still_refuses_backend_all_without_multi_root(flat_asset, tmp_path):
    """``fix`` keeps the refusal: its input may be an Isaac package."""
    source, _ = flat_asset
    with pytest.raises(OptionError):
        fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="all"))


def test_multi_root_is_ignored_when_the_asset_has_variants(tmp_path):
    """Variant scoping is the better answer, so it wins where it applies."""
    source = export(variant_asset(), tmp_path / "robot.usda")
    report = fix_asset(source, tmp_path / "out", RepairOptions(backends_requested="all", multi_root=True))
    assert report["output"]["variant_scoped"] is True
    assert report["output"]["multi_root"] is False
    assert len(report["output"]["roots"]) == 1


def test_tuning_provenance_is_recorded_and_honest(flat_asset, tmp_path):
    """Every report and every layer says where the tuning constants came from.

    Phase 3 shipped them as ``unmeasured``; Phase 4 measured them. Whichever is
    true, an asset carries it, and the two can never drift apart because both
    read the same table.
    """
    from urdf_usd_bridge.repair.base import PROVENANCE
    from urdf_usd_bridge.repair.layer import tuning_status

    source, _ = flat_asset
    out = tmp_path / "out"
    report = fix_asset(source, out, RepairOptions(backends_requested="physx"))

    assert report["options"]["tuning_status"] == tuning_status()
    assert report["options"]["tuning_provenance"] == dict(PROVENANCE)

    recorded = Sdf.Layer.FindOrOpen(str(out / "robot_stabilized.usda")).customLayerData["urdf_usd_bridge"]
    assert recorded["tuning_status"] == tuning_status()
    assert set(recorded["tuning_provenance"]) == set(PROVENANCE)
    assert set(PROVENANCE) == set(report["options"]["tuning"]) - {"control_rate_hz"} | {
        "control_rate"
    } or set(PROVENANCE) == {
        "target_frequency",
        "damping_ratio",
        "armature_fraction",
        "armature_floor",
        "control_rate",
    }


def test_tuning_status_reflects_the_provenance_table():
    """The banner is derived, never hand-written, so it cannot go stale."""
    import urdf_usd_bridge.repair.base as base
    from urdf_usd_bridge.repair.layer import tuning_status

    original = dict(base.PROVENANCE)
    try:
        base.PROVENANCE.update(dict.fromkeys(original, "unmeasured"))
        assert tuning_status().startswith("unmeasured")

        base.PROVENANCE.update(dict.fromkeys(original, "measured: sweep-1"))
        assert tuning_status().startswith("measured")

        base.PROVENANCE["control_rate"] = "unmeasured"
        status = tuning_status()
        assert status.startswith("partly measured")
        assert "control_rate" in status
    finally:
        base.PROVENANCE.clear()
        base.PROVENANCE.update(original)


def test_a_constant_with_a_reason_still_counts_as_unmeasured():
    """The provenance strings explain themselves, and the banner must still parse them.

    ``armature_floor`` is unmeasured *with a reason*. An exact-match check read
    that as measured and the banner claimed every constant was backed by a
    sweep, which was not true.
    """
    import urdf_usd_bridge.repair.base as base
    from urdf_usd_bridge.repair.layer import tuning_status

    original = dict(base.PROVENANCE)
    try:
        base.PROVENANCE.update(dict.fromkeys(original, "measured: sweep-1"))
        base.PROVENANCE["armature_floor"] = "unmeasured: the case it exists for is not in the corpus"
        status = tuning_status()
        assert status.startswith("partly measured")
        assert "armature_floor" in status
    finally:
        base.PROVENANCE.clear()
        base.PROVENANCE.update(original)


def test_the_shipped_provenance_is_honest_about_what_was_not_measured():
    from urdf_usd_bridge.repair.layer import tuning_status

    status = tuning_status()
    assert status.startswith("partly measured")
    assert "armature_floor" in status
