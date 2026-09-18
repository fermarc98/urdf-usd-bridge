# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Writing the stability layers, without ever touching the input.

The output is a new root layer whose sublayers are our repairs followed by the
untouched original::

    <name>_stabilized.usda     subLayers = [Stability.usda, ..., <original root>]

Earlier sublayers are stronger, so our ``over`` opinions win and the original is
referenced by relative path and never opened for write.

Two things this module exists to get right
------------------------------------------
**Root-layer metadata is not composed from sublayers.** ``defaultPrim``,
``upAxis``, ``metersPerUnit`` and ``kilogramsPerUnit`` live on the root layer
alone. A new root layer that omits them opens with ``defaultPrim = None``, which
silently disables every variant selection and makes an Isaac Sim package compose
as a geometry-only asset. :func:`_copy_root_metadata` copies all four, and
``tests/unit/test_repair_layer.py`` fails if ``defaultPrim`` is ever lost again.

**Per-backend opinions are variant-scoped when the asset has a Physics variant
set.** Isaac Sim's ``mujoco.usda`` deliberately deletes ``PhysicsDriveAPI`` so
MuJoCo actuation comes only from ``MjcActuator``. A plain root sublayer is
stronger than that deletion and would resurrect a per-degree PhysX drive inside
the MuJoCo variant, next to a per-radian ``MjcActuator`` gain for the same
joint. Authoring each backend inside its own variant keeps them apart. Verified:
with the physx variant selected the drive is ours, with mujoco selected the
joint has no drive at all.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from pxr import Gf, Sdf, Usd, UsdGeom

from .._version import __version__
from .base import (
    DEFAULTS_CHANGED,
    MUJOCO,
    NEUTRAL,
    NEWTON,
    PHYSX,
    PREVIOUS_DEFAULTS,
    PROVENANCE,
    RULESET_VERSION,
    STABLE_RATE_RATIO,
    RepairOptions,
)

#: The variant set both Isaac Sim 6.x and any variant-bearing asset use.
PHYSICS_VARIANT_SET = "Physics"

#: Which variant each backend's opinions belong in. Newton consumes the
#: backend-neutral layer -- there is no ``newton`` variant -- so it maps to
#: ``physics``.
BACKEND_VARIANT = {
    PHYSX: "physx",
    MUJOCO: "mujoco",
    NEWTON: "physics",
}

#: File name per backend.
BACKEND_LAYER_NAME = {
    NEUTRAL: "Stability.usda",
    PHYSX: "Stability_physx.usda",
    MUJOCO: "Stability_mujoco.usda",
    NEWTON: "Stability_newton.usda",
}


@dataclass
class PlannedWrite:
    """One authoring operation, decided during analysis and applied later.

    Rules never author directly. They return planned writes, which means
    ``--dry-run`` runs exactly the same analysis and simply does not apply
    them -- a dry run cannot diverge from a real one.
    """

    prim: str
    backend: str
    attribute: str | None = None
    value: Any = None
    type_name: Any = None
    apply_schema: str | None = None
    schema_instance: str | None = None
    define_type: str | None = None
    relationship: str | None = None
    rel_targets: list[str] = field(default_factory=list)
    #: Every ``mjc:*`` attribute is declared ``uniform``; creating it varying
    #: would author an attribute the MJC schema does not recognise.
    uniform: bool = False

    def sort_key(self) -> tuple:
        return (self.prim, self.attribute or "", self.relationship or "", self.backend)


def input_digest(identifier: str) -> str:
    """SHA-256 of the input root layer, for provenance without a timestamp."""
    try:
        return hashlib.sha256(Path(identifier).read_bytes()).hexdigest()
    except OSError:  # pragma: no cover - in-memory or remote identifiers
        return ""


def _copy_root_metadata(source_stage, target_layer: Sdf.Layer, target_stage) -> dict[str, Any]:
    """Carry root-layer metadata onto our new root layer.

    Returns what was copied, so the report can show it and a test can assert it.
    """
    copied: dict[str, Any] = {}
    source_root = source_stage.GetRootLayer()
    if source_root.defaultPrim:
        target_layer.defaultPrim = source_root.defaultPrim
        copied["defaultPrim"] = source_root.defaultPrim

    up_axis = UsdGeom.GetStageUpAxis(source_stage)
    if up_axis:
        UsdGeom.SetStageUpAxis(target_stage, up_axis)
        copied["upAxis"] = str(up_axis)

    meters = UsdGeom.GetStageMetersPerUnit(source_stage)
    if meters:
        UsdGeom.SetStageMetersPerUnit(target_stage, meters)
        copied["metersPerUnit"] = float(meters)

    kilograms = source_stage.GetMetadata("kilogramsPerUnit")
    if kilograms is not None:
        target_stage.SetMetadata("kilogramsPerUnit", kilograms)
        copied["kilogramsPerUnit"] = float(kilograms)
    return copied


def tuning_status() -> str:
    """One line saying whether the tuning constants were measured."""
    # startswith, not equality: a provenance string explains *why* something is
    # unmeasured, and an exact-match check quietly reported a constant with a
    # reason attached as if it had been measured.
    unmeasured = sorted(k for k, v in PROVENANCE.items() if v.startswith("unmeasured"))
    if not unmeasured:
        return "measured: every tuning constant is backed by a sweep in docs/BENCHMARK.md"
    if len(unmeasured) == len(PROVENANCE):
        return (
            "unmeasured: pre-measurement defaults, see docs/BENCHMARK.md for the experiments "
            "that would justify them"
        )
    verb = "remains" if len(unmeasured) == 1 else "remain"
    return "partly measured: " + ", ".join(unmeasured) + f" {verb} unmeasured; see docs/BENCHMARK.md"


def _stability_metadata(options: RepairOptions, digest: str) -> dict[str, Any]:
    """Custom layer data recording the assumptions the asset was built with.

    Deliberately carries no timestamp, hostname or absolute path: the output has
    to be byte-identical for the same input, and a clock is the easiest way to
    lose that. ``defaults_changed`` is a fixed release date, not a clock read.
    """
    payload: dict[str, Any] = {
        "version": __version__,
        "ruleset_version": RULESET_VERSION,
        "input_sha256": digest,
        "tuning": dict(options.tuning()),
        "target_frequency_basis": options.target_frequency_basis,
        # Both numbers, always: an asset tuned for one backend records what a
        # cross-backend asset would have used, so a reader can see the cost of
        # the choice without re-deriving it.
        "target_frequency_cross_backend_hz": options.control_rate / STABLE_RATE_RATIO,
        "tuning_status": tuning_status(),
        "tuning_provenance": dict(PROVENANCE),
        # A plain Python list lands in customLayerData as an unregistered
        # vector<VtValue> and makes USD warn on every read, so the backend
        # set is stored as text.
        "backends": ", ".join(options.backends),
    }
    if PREVIOUS_DEFAULTS:
        payload["previous_defaults"] = dict(PREVIOUS_DEFAULTS)
    if DEFAULTS_CHANGED:
        payload["defaults_changed"] = DEFAULTS_CHANGED
    return {"urdf_usd_bridge": payload}


class StabilityLayers:
    """Creates, populates and saves the output layers for one asset."""

    def __init__(
        self,
        source_stage,
        source_identifier: str,
        out_dir: Path,
        options: RepairOptions,
        *,
        variant_scoped: bool,
        asset_name: str,
    ):
        self.source_stage = source_stage
        self.source_identifier = source_identifier
        self.out_dir = out_dir
        self.options = options
        self.variant_scoped = variant_scoped
        self.asset_name = asset_name
        self.root_path = out_dir / f"{asset_name}_stabilized.usda"
        #: Every root written. One in the normal case; one per backend when a
        #: flat asset is stabilized for several backends at once.
        self.root_paths: list[Path] = []
        self.layers: dict[str, Sdf.Layer] = {}
        self.metadata_copied: dict[str, Any] = {}
        self.missing_variants: list[tuple[str, str]] = []
        self._stage = None

    def layer_name(self, backend: str) -> str:
        return BACKEND_LAYER_NAME[backend]

    def create(self, backends: tuple[str, ...], *, multi_root: bool = False) -> None:
        """Create the stability layers and the root(s) that compose them.

        Normally one root sublayers every stability layer plus the original.
        With ``multi_root`` -- a flat asset stabilized for several backends at
        once -- that single root would put a per-degree ``UsdPhysics`` drive
        gain and a per-radian ``MjcActuator`` gain on the same joint, so one
        root is written **per backend** instead, each composing only the
        neutral layer, its own backend layer and the original. The conventions
        never meet, and the caller picks a file rather than a flag.
        """
        import os

        self.out_dir.mkdir(parents=True, exist_ok=True)
        wanted = (NEUTRAL, *backends)
        digest = input_digest(self.source_identifier)
        relative_source = os.path.relpath(self.source_identifier, self.out_dir)

        for backend in wanted:
            path = self.out_dir / BACKEND_LAYER_NAME[backend]
            if path.exists():
                path.unlink()
            self.layers[backend] = Sdf.Layer.CreateNew(str(path))

        def write_root(path: Path, sublayers: list[str]) -> Sdf.Layer:
            if path.exists():
                path.unlink()
            layer = Sdf.Layer.CreateNew(str(path))
            layer.subLayerPaths = [*sublayers, relative_source]
            layer.customLayerData = _stability_metadata(self.options, digest)
            layer.Save()
            return layer

        if multi_root:
            roots = {
                backend: write_root(
                    self.out_dir / f"{self.asset_name}_stabilized_{backend}.usda",
                    [
                        f"./{BACKEND_LAYER_NAME[backend]}",
                        f"./{BACKEND_LAYER_NAME[NEUTRAL]}",
                    ],
                )
                for backend in backends
            }
            # Authoring needs one stage whose layer stack contains *every*
            # stability layer. It is anonymous and never written: the files the
            # caller gets are the per-backend roots above.
            authoring = Sdf.Layer.CreateAnonymous(f"{self.asset_name}_authoring.usda")
            authoring.subLayerPaths = [
                str((self.out_dir / BACKEND_LAYER_NAME[backend]).resolve()) for backend in wanted
            ] + [str(Path(self.source_identifier).resolve())]
            self._stage = Usd.Stage.Open(authoring, Usd.Stage.LoadAll)
            self.metadata_copied = _copy_root_metadata(self.source_stage, authoring, self._stage)
            for backend, layer in roots.items():
                layer.defaultPrim = authoring.defaultPrim
                inner = Usd.Stage.Open(layer, Usd.Stage.LoadAll)
                _copy_root_metadata(self.source_stage, layer, inner)
                layer.Save()
                self.root_paths.append(self.out_dir / f"{self.asset_name}_stabilized_{backend}.usda")
            self._stage = Usd.Stage.Open(authoring, Usd.Stage.LoadAll)
            return

        root_layer = write_root(self.root_path, [f"./{BACKEND_LAYER_NAME[backend]}" for backend in wanted])
        self._stage = Usd.Stage.Open(str(self.root_path), Usd.Stage.LoadAll)
        self.metadata_copied = _copy_root_metadata(self.source_stage, root_layer, self._stage)
        root_layer.Save()
        # Reopen so the metadata we just wrote is what composition sees.
        self._stage = Usd.Stage.Open(str(self.root_path), Usd.Stage.LoadAll)
        self.root_paths.append(self.root_path)

    @property
    def stage(self):
        if self._stage is None:  # pragma: no cover - misuse
            raise RuntimeError("StabilityLayers.create() must be called first")
        return self._stage

    def available_variants(self) -> list[str]:
        prim = self.stage.GetDefaultPrim()
        if not prim or not prim.IsValid():
            return []
        sets = prim.GetVariantSets()
        if PHYSICS_VARIANT_SET not in sets.GetNames():
            return []
        return list(sets.GetVariantSet(PHYSICS_VARIANT_SET).GetVariantNames())

    def apply(self, writes: list[PlannedWrite]) -> list[PlannedWrite]:
        """Author every planned write. Returns them in the order applied."""
        ordered = sorted(writes, key=PlannedWrite.sort_key)
        by_backend: dict[str, list[PlannedWrite]] = {}
        for write in ordered:
            by_backend.setdefault(write.backend, []).append(write)

        for backend in sorted(by_backend):
            self._apply_backend(backend, by_backend[backend])

        self._clear_selections()
        for layer in self.layers.values():
            layer.Save()
        if not self.stage.GetRootLayer().anonymous:
            self.stage.GetRootLayer().Save()
        return ordered

    def _apply_backend(self, backend: str, writes: list[PlannedWrite]) -> None:
        """Author one backend's writes, variant-scoped when the asset allows it.

        ``SetVariantSelection`` authors into whatever layer is the current edit
        target, so the edit target is returned to the root layer before any
        selection is touched. Getting this wrong puts a variant *selection*
        inside a stability layer and drops every backend's opinions into
        whichever variant happened to be selected -- which is exactly what the
        first version of this method did.
        """
        layer = self.layers.get(backend)
        if layer is None:  # pragma: no cover - guarded by the caller
            return
        stage = self.stage
        root_target = Usd.EditTarget(stage.GetRootLayer())
        variant = BACKEND_VARIANT.get(backend) if self.variant_scoped else None

        if backend == NEUTRAL or variant is None:
            stage.SetEditTarget(Usd.EditTarget(layer))
            for write in writes:
                self._author(stage, write)
            stage.SetEditTarget(root_target)
            return

        stage.SetEditTarget(root_target)
        prim = stage.GetDefaultPrim()
        vset = prim.GetVariantSets().GetVariantSet(PHYSICS_VARIANT_SET)
        if variant not in vset.GetVariantNames():
            # Fall back to a flat opinion rather than dropping the backend.
            stage.SetEditTarget(Usd.EditTarget(layer))
            for write in writes:
                self._author(stage, write)
            stage.SetEditTarget(root_target)
            self.missing_variants.append((backend, variant))
            return

        vset.SetVariantSelection(variant)
        stage.SetEditTarget(Usd.EditTarget(layer))
        with vset.GetVariantEditContext():
            for write in writes:
                self._author(stage, write)
        stage.SetEditTarget(root_target)
        vset.ClearVariantSelection()

    def _clear_selections(self) -> None:
        """Make sure we never pin the asset to one backend.

        Which physics a consumer gets is their decision; a stability layer that
        authored a selection would take it away from them.
        """
        prim_path = self.stage.GetRootLayer().defaultPrim
        if not prim_path:
            return
        for layer in [self.stage.GetRootLayer(), *self.layers.values()]:
            spec = layer.GetPrimAtPath(f"/{prim_path}")
            if spec is not None and PHYSICS_VARIANT_SET in spec.variantSelections:
                del spec.variantSelections[PHYSICS_VARIANT_SET]

    @staticmethod
    def _author(stage, write: PlannedWrite) -> None:
        if write.define_type:
            prim = stage.DefinePrim(write.prim, write.define_type)
        else:
            prim = stage.OverridePrim(write.prim)
        if not prim or not prim.IsValid():  # pragma: no cover - defensive
            return

        if write.apply_schema:
            if write.schema_instance:
                prim.AddAppliedSchema(f"{write.apply_schema}:{write.schema_instance}")
            else:
                prim.AddAppliedSchema(write.apply_schema)

        if write.relationship:
            rel = prim.CreateRelationship(write.relationship, custom=False)
            rel.SetTargets([Sdf.Path(t) for t in write.rel_targets])
            return

        if write.attribute is None:
            return
        attr = prim.GetAttribute(write.attribute)
        if not attr or not attr.IsValid():
            variability = Sdf.VariabilityUniform if write.uniform else Sdf.VariabilityVarying
            attr = prim.CreateAttribute(
                write.attribute, write.type_name, custom=False, variability=variability
            )
        attr.Set(_coerce(write.value, write.type_name))


def _coerce(value: Any, type_name) -> Any:
    """Match the value to the attribute's declared type.

    Letting Python's default float land in a ``float`` (single-precision)
    attribute is a source of non-reproducible text output, so the conversion is
    explicit.
    """
    if type_name is None:
        return value
    if type_name == Sdf.ValueTypeNames.Quatf and isinstance(value, (list, tuple)):
        return Gf.Quatf(float(value[0]), float(value[1]), float(value[2]), float(value[3]))
    if type_name == Sdf.ValueTypeNames.Quatd and isinstance(value, (list, tuple)):
        return Gf.Quatd(float(value[0]), float(value[1]), float(value[2]), float(value[3]))
    if type_name == Sdf.ValueTypeNames.Float3 and isinstance(value, (list, tuple)):
        return Gf.Vec3f(float(value[0]), float(value[1]), float(value[2]))
    if type_name == Sdf.ValueTypeNames.DoubleArray and isinstance(value, (list, tuple)):
        return [float(v) for v in value]
    if type_name == Sdf.ValueTypeNames.FloatArray and isinstance(value, (list, tuple)):
        return [float(v) for v in value]
    return value
