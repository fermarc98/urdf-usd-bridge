# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Opening a converted asset and working out how it is laid out."""

from __future__ import annotations

from typing import Any

from pxr import Usd, UsdGeom, UsdPhysics

#: Layouts we know how to read.
LAYOUT_NEWTON_ATOMIC = "newton-atomic"
LAYOUT_ISAAC_PACKAGE = "isaac-package"
LAYOUT_FLAT = "flat"
LAYOUT_UNKNOWN = "unknown"


def open_stage(identifier: str, variant_selections: dict[str, str] | None = None):
    """Open a USD stage with all payloads loaded, applying variant selections.

    Args:
        identifier: Path to the root layer.
        variant_selections: ``{variant_set: variant}`` applied to the default
            prim before anything is read. Needed for Isaac Sim 6.1.0 packages,
            whose ``Physics`` variant set has no default selection authored.

    Returns:
        The opened ``Usd.Stage``.

    Raises:
        ValueError: if the stage cannot be opened.
    """
    try:
        stage = Usd.Stage.Open(identifier, Usd.Stage.LoadAll)
    except Exception as exc:  # pxr raises Tf.ErrorException, which is not an OSError
        raise ValueError(f"could not open USD stage {identifier!r}: {exc}") from exc
    if not stage:
        raise ValueError(f"could not open USD stage {identifier!r}")
    if variant_selections:
        prim = stage.GetDefaultPrim()
        if prim and prim.IsValid():
            sets = prim.GetVariantSets()
            for name, selection in variant_selections.items():
                if name in sets.GetNames():
                    sets.GetVariantSet(name).SetVariantSelection(selection)
    return stage


def describe_variant_sets(prim) -> list[dict[str, Any]]:
    """Report every variant set on ``prim``, its options and its selection.

    A selection of ``None`` means no selection is authored *and* no fallback
    applies, so the variant contributes nothing to composition. Isaac Sim
    6.1.0 ships URDF assets in exactly that state for the ``Physics`` set.
    """
    if not prim or not prim.IsValid():
        return []
    out: list[dict[str, Any]] = []
    sets = prim.GetVariantSets()
    for name in sets.GetNames():
        vset = sets.GetVariantSet(name)
        selection = vset.GetVariantSelection()
        out.append(
            {
                "name": name,
                "variants": list(vset.GetVariantNames()),
                "selection": selection or None,
                "has_authored_selection": bool(selection),
            }
        )
    return out


def layer_identifiers(stage) -> list[str]:
    """Every layer identifier used by the stage, root first."""
    return [layer.identifier for layer in stage.GetUsedLayers()]


def detect_layout(stage) -> str:
    """Classify the asset layout.

    Distinguishes the Newton ``urdf-usd-converter`` atomic-component layout
    from the Isaac Sim 6.x transformer package, and flags a flattened or
    unrecognised stage. Detection reads layer identifiers and prim structure,
    never file paths on disk, so it also works for in-memory stages.
    """
    identifiers = " ".join(layer_identifiers(stage)).replace("\\", "/")

    isaac_markers = ("payloads/base", "payloads/physics/", "payloads/robot", "payloads/geometries")
    if any(marker in identifiers.lower() for marker in isaac_markers):
        return LAYOUT_ISAAC_PACKAGE

    newton_markers = ("payload/contents", "payload/physics.usda", "payload/geometry.usda")
    if any(marker in identifiers.lower() for marker in newton_markers):
        return LAYOUT_NEWTON_ATOMIC

    prim = stage.GetDefaultPrim()
    if prim and prim.IsValid():
        children = {child.GetName() for child in prim.GetChildren()}
        if {"Geometry", "Physics"} <= children:
            # Same prim topology both producers emit; the layer split is what
            # tells them apart, and this stage has been flattened.
            return LAYOUT_FLAT
        if UsdGeom.Xform(prim):
            return LAYOUT_FLAT
    return LAYOUT_UNKNOWN


def stage_metrics(stage) -> dict[str, Any]:
    """Stage-level unit metadata, which every backend relies on."""
    root = stage.GetRootLayer()
    kg_per_unit = None
    if root.HasCustomLayerData() or True:
        kg_per_unit = stage.GetMetadata(UsdPhysics.Tokens.kilogramsPerUnit)
    return {
        "up_axis": UsdGeom.GetStageUpAxis(stage),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "kilograms_per_unit": kg_per_unit,
        "root_layer": root.identifier,
    }
