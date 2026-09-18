# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Orchestration: analyse, then author.

The pass is deliberately split in two.

**Analysis** opens the input read-only, builds the articulation, and runs every
rule to produce a list of :class:`~.layer.PlannedWrite` objects plus one record
per rule evaluation. It authors nothing.

**Authoring** creates the output layers and applies the planned writes.

``--dry-run`` runs analysis and stops. Because the rules cannot author directly,
a dry run and a real run cannot disagree about what would happen -- the only
difference is whether phase two executes.

Rule order is fixed and documented: inertia, then limits, then armature, then
drives. Drives need the armature value, and armature needs the repaired inertia,
so the order is a real dependency rather than a convention.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from pxr import UsdGeom

from .._version import __version__
from ..model.articulation import Articulation, build_articulation
from ..model.stage import detect_layout, open_stage
from ..usd import usd_version
from .armature import apply_armature_rules
from .base import (
    ALL_BACKENDS,
    APPLIED,
    ERROR,
    NEUTRAL,
    PROVENANCE,
    REPORT_SCHEMA_VERSION,
    REPORTED,
    RULESET_VERSION,
    WARNING,
    OptionError,
    RepairOptions,
    RepairRecord,
)
from .drives import apply_drive_rules
from .inertia import apply_inertia_rules
from .layer import BACKEND_LAYER_NAME, PHYSICS_VARIANT_SET, PlannedWrite, StabilityLayers, input_digest
from .limits import apply_limit_rules


def tuning_status() -> str:
    """The provenance line every report carries, measured or not."""
    from .layer import tuning_status as _status

    return _status()


@dataclass
class RepairContext:
    """Everything the rules read. Rules never touch anything outside this."""

    stage: Any
    articulation: Articulation
    options: RepairOptions
    scale: float
    layer_names: dict[str, str]
    variant_scoped: bool
    #: Filled in by the armature rule, consumed by the drive rule.
    armature: dict[str, float] = field(default_factory=dict)


def _has_physics_variant(stage) -> bool:
    prim = stage.GetDefaultPrim()
    if not prim or not prim.IsValid():
        return False
    return PHYSICS_VARIANT_SET in prim.GetVariantSets().GetNames()


#: Preferred variant to *read* through when none is selected. The neutral
#: ``physics`` layer carries every body, joint, mass and limit without a backend
#: overlay, which is exactly what the rules need to see.
_ANALYSIS_VARIANT_ORDER = ("physics", "physx", "mujoco")


def ensure_readable_variant(stage, requested: dict[str, str]) -> str | None:
    """Select a Physics variant for analysis when the asset authors none.

    Isaac Sim 6.1.0 ships URDF packages with a ``Physics`` variant set and **no
    default selection** (``clear_default_variant_sets``), so an asset opened as
    authored has no rigid bodies, no joints and no masses at all -- every rule
    would find nothing and report success. Reading through the neutral
    ``physics`` variant is what makes the asset analysable; the choice is
    recorded in the report and never written into the output.
    """
    if requested.get(PHYSICS_VARIANT_SET):
        return None
    prim = stage.GetDefaultPrim()
    if not prim or not prim.IsValid():
        return None
    sets = prim.GetVariantSets()
    if PHYSICS_VARIANT_SET not in sets.GetNames():
        return None
    vset = sets.GetVariantSet(PHYSICS_VARIANT_SET)
    if vset.GetVariantSelection():
        return None
    available = vset.GetVariantNames()
    for candidate in _ANALYSIS_VARIANT_ORDER:
        if candidate in available:
            vset.SetVariantSelection(candidate)
            return candidate
    return None


def _replay_into_session(stage, writes: list[PlannedWrite]) -> None:
    """Apply planned writes to the stage's session layer, for analysis only."""
    from pxr import Usd as _Usd

    previous = stage.GetEditTarget()
    stage.SetEditTarget(_Usd.EditTarget(stage.GetSessionLayer()))
    try:
        for write in sorted(writes, key=PlannedWrite.sort_key):
            StabilityLayers._author(stage, write)
    finally:
        stage.SetEditTarget(previous)


def resolve_backends(requested: str, *, variant_scoped: bool, multi_root: bool = False) -> tuple[str, ...]:
    """Turn the ``--backend`` option into a concrete tuple.

    ``all`` on an asset with no ``Physics`` variant set is refused rather than
    guessed at. Without variants every backend's opinions land in one composed
    stage, so the same joint would carry a per-degree ``DriveAPI`` gain and a
    per-radian ``MjcActuator`` gain at once, and whichever a consumer reads, the
    other is wrong by 57.3x.

    ``multi_root`` is the way out: keep the conventions apart by writing one
    stabilized root per backend instead of one root carrying all three. That is
    what ``convert`` does, since everything it produces is variant-less.
    """
    from .base import SELECTABLE_BACKENDS

    if requested == ALL_BACKENDS:
        if not variant_scoped and multi_root:
            return SELECTABLE_BACKENDS
        if not variant_scoped:
            raise OptionError(
                "--backend all needs a 'Physics' variant set to keep each backend's gains apart, "
                "and this asset has none (a flattened or newton-atomic layout).\n"
                "Authoring all three into one composed stage would give the same joint a "
                "per-degree UsdPhysics drive gain and a per-radian MjcActuator gain.\n"
                "Choose one: --backend physx | --backend mujoco | --backend newton"
            )
        return SELECTABLE_BACKENDS
    names = tuple(part.strip() for part in requested.split(",") if part.strip())
    unknown = [n for n in names if n not in SELECTABLE_BACKENDS]
    if unknown:
        raise OptionError(
            f"unknown backend(s) {', '.join(unknown)}; choose from "
            f"{', '.join(SELECTABLE_BACKENDS)} or 'all'"
        )
    return names


def analyse(identifier: str, options: RepairOptions) -> dict[str, Any]:
    """Run every rule and return the plan plus the records. Authors nothing."""
    stage = open_stage(identifier, options.variant_selections)
    variant_scoped = _has_physics_variant(stage)
    analysis_variant = ensure_readable_variant(stage, options.variant_selections)
    backends = resolve_backends(
        options.backends_requested, variant_scoped=variant_scoped, multi_root=options.multi_root
    )
    options.backends = backends

    scale = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    layer_names = {NEUTRAL: BACKEND_LAYER_NAME[NEUTRAL]}
    layer_names.update({backend: BACKEND_LAYER_NAME[backend] for backend in backends})

    ctx = RepairContext(
        stage=stage,
        articulation=build_articulation(stage, meters_per_unit=scale),
        options=options,
        scale=scale,
        layer_names=layer_names,
        variant_scoped=variant_scoped,
    )

    records: list[RepairRecord] = []
    writes: list[PlannedWrite] = []
    if analysis_variant:
        records.append(
            RepairRecord(
                rule="layer.variant-scoping",
                status=REPORTED,
                prim="/",
                reason=(
                    f"no Physics variant selection is authored, so the asset composes with no "
                    f"physics at all; analysed through the '{analysis_variant}' variant. "
                    "This selection is not written into the output -- consumers still choose"
                ),
                severity=WARNING,
                backend=NEUTRAL,
                evidence={"analysis_variant": analysis_variant},
            )
        )
    if not variant_scoped:
        records.append(
            RepairRecord(
                rule="layer.variant-scoping",
                status=REPORTED,
                prim="/",
                reason=(
                    "this asset has no 'Physics' variant set, so the stability layers are plain "
                    "root sublayers and apply under every composition. That is correct for a "
                    "newton-atomic or flattened layout; it is why --backend all is refused here"
                ),
                severity=WARNING,
                backend=NEUTRAL,
                evidence={"layout": detect_layout(stage)},
            )
        )

    inertia_records, inertia_writes = apply_inertia_rules(ctx)
    records.extend(inertia_records)
    writes.extend(inertia_writes)

    # Armature and drive gains are functions of I_eq, and I_eq must reflect the
    # inertia we just repaired -- otherwise a body whose tensor was missing
    # contributes nothing, I_eq comes out zero, and no gain is derived at all.
    # The repairs are replayed into the stage's *session* layer, which is
    # in-memory and never saved, and the articulation is rebuilt from the
    # result. The same writes are still applied to the real output layers later.
    if inertia_writes:
        _replay_into_session(stage, inertia_writes)
        ctx.articulation = build_articulation(stage, meters_per_unit=scale)

    for rule_set in (apply_limit_rules, apply_armature_rules, apply_drive_rules):
        rule_records, rule_writes = rule_set(ctx)
        records.extend(rule_records)
        writes.extend(rule_writes)

    return {
        "stage": stage,
        "context": ctx,
        "records": records,
        "writes": writes,
        "variant_scoped": variant_scoped,
        "backends": backends,
        "scale": scale,
        "analysis_variant": analysis_variant,
        "multi_root": bool(options.multi_root and not variant_scoped and len(backends) > 1),
    }


def fix_asset(
    identifier: str,
    out_dir: str | Path | None,
    options: RepairOptions,
) -> dict[str, Any]:
    """Analyse an asset and, unless ``--dry-run``, write the stability layers.

    Returns the repair report as a JSON-serialisable dict.
    """
    plan = analyse(identifier, options)
    stage = plan["stage"]
    records: list[RepairRecord] = plan["records"]
    writes: list[PlannedWrite] = plan["writes"]

    asset_name = Path(identifier).stem
    output: dict[str, Any] = {
        "written": False,
        "root": None,
        "roots": [],
        "layers": {},
        "variant_scoped": plan["variant_scoped"],
        "multi_root": plan["multi_root"],
        "metadata_copied": {},
    }

    if not options.dry_run:
        if out_dir is None:
            raise OptionError("an output directory is required unless --dry-run is given")
        layers = StabilityLayers(
            source_stage=stage,
            source_identifier=identifier,
            out_dir=Path(out_dir),
            options=options,
            variant_scoped=plan["variant_scoped"],
            asset_name=asset_name,
        )
        layers.create(plan["backends"], multi_root=plan["multi_root"])
        layers.apply(writes)
        output.update(
            {
                "written": True,
                "root": str(layers.root_paths[0]),
                "roots": [str(path) for path in layers.root_paths],
                "layers": {
                    backend: str(Path(out_dir) / BACKEND_LAYER_NAME[backend])
                    for backend in (NEUTRAL, *plan["backends"])
                },
                "metadata_copied": layers.metadata_copied,
            }
        )

    report = {
        "schema_version": REPORT_SCHEMA_VERSION,
        "tool": {
            "name": "urdf-usd-bridge",
            "version": __version__,
            "usd_version": usd_version(),
            "ruleset_version": RULESET_VERSION,
        },
        "input": {
            "identifier": identifier,
            "sha256": input_digest(identifier),
            "layout": detect_layout(stage),
            "variant_selections": dict(options.variant_selections),
            "analysis_variant": plan["analysis_variant"],
            "meters_per_unit": plan["scale"],
        },
        "output": output,
        "options": {
            "backends": list(plan["backends"]),
            "backends_requested": options.backends_requested,
            "force": options.force,
            "dry_run": options.dry_run,
            "tuning": options.tuning(),
            "tuning_basis": options.target_frequency_basis,
            "tuning_provenance": dict(PROVENANCE),
            "tuning_status": tuning_status(),
            "rules": dict(sorted(options.enabled.items())),
        },
        "records": [record.as_dict() for record in records],
    }
    report["summary"] = summarize(report)
    return report


def summarize(report: dict[str, Any]) -> dict[str, Any]:
    """Counters over the records, plus the exit-code-relevant error count."""
    records = report["records"]
    by_status: dict[str, int] = {}
    by_rule: dict[str, int] = {}
    by_backend: dict[str, int] = {}
    by_severity: dict[str, int] = {}
    for record in records:
        by_status[record["status"]] = by_status.get(record["status"], 0) + 1
        if record["status"] == APPLIED:
            by_rule[record["rule"]] = by_rule.get(record["rule"], 0) + 1
            by_backend[record["backend"]] = by_backend.get(record["backend"], 0) + 1
        severity = record.get("severity")
        if severity:
            by_severity[severity] = by_severity.get(severity, 0) + 1
    return {
        "records_total": len(records),
        "by_status": dict(sorted(by_status.items())),
        "applied_by_rule": dict(sorted(by_rule.items())),
        "applied_by_backend": dict(sorted(by_backend.items())),
        "by_severity": dict(sorted(by_severity.items())),
        "errors": by_severity.get(ERROR, 0),
        "prims_touched": len({r["prim"] for r in records if r["status"] == APPLIED}),
    }
