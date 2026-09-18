# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The matrix: assets x variants x backends x suites, and the comparison.

Every asset is prepared twice -- **baseline**, the converter's output, and
**repaired**, that same output with our stability layers over it -- so the pair
differs only by the repairs. Both are produced here rather than committed, so a
result can never be compared against a stale artefact.
"""

from __future__ import annotations

import json
import platform
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .._version import __version__
from . import metrics as sim_metrics
from .backends import get_backend
from .corpus import RobotSpec, fetch
from .suites import SUITES

BASELINE = "baseline"
REPAIRED = "repaired"


@dataclass
class PreparedAsset:
    """One robot, converted once and repaired once."""

    label: str
    role: str
    exploratory: bool
    baseline: str | None = None
    repaired: dict[str, str] = field(default_factory=dict)
    skipped: str = ""
    provenance: dict[str, Any] = field(default_factory=dict)
    repair_summary: dict[str, Any] = field(default_factory=dict)


def _environment() -> dict[str, Any]:
    """Everything a reader needs to know which machine produced a number."""
    env: dict[str, Any] = {
        "tool_version": __version__,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": platform.python_version(),
    }
    try:
        smi = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            timeout=20,
        )
        env["gpu"] = smi.stdout.strip() or None
    except Exception:  # pragma: no cover - no nvidia-smi
        env["gpu"] = None
    for module, key in (("newton", "newton"), ("warp", "warp"), ("mujoco", "mujoco"), ("pxr", "usd")):
        try:
            imported = __import__(module)
            env[key] = getattr(imported, "__version__", None) or str(
                getattr(imported, "config", None) and imported.config.version
            )
        except Exception:
            env[key] = None
    isaac_version = Path.home() / "isaacsim" / "VERSION"
    env["isaac_sim"] = isaac_version.read_text().strip() if isaac_version.exists() else None
    return env


def prepare(
    spec: RobotSpec,
    out_dir: Path,
    *,
    backends: tuple[str, ...],
    converter_python: str | None = None,
    tuning: dict[str, float] | None = None,
) -> PreparedAsset:
    """Convert a URDF and produce the repaired counterpart, per backend."""
    prepared = PreparedAsset(label=spec.label, role=spec.role, exploratory=spec.exploratory)
    spec = fetch(spec)
    if "skipped" in spec.resolved:
        prepared.skipped = spec.resolved["skipped"]
        return prepared
    prepared.provenance = dict(spec.resolved)

    work = out_dir / spec.label
    work.mkdir(parents=True, exist_ok=True)
    converted_dir = work / "converted"

    python = converter_python or _find_converter_python()
    if python is None:
        prepared.skipped = (
            "no interpreter with urdf-usd-converter installed; run "
            "`python scripts/run_converter_matrix.py` or pass --converter-python"
        )
        return prepared

    convert = subprocess.run(
        [python, "-m", "urdf_usd_bridge", "convert", spec.resolved["urdf"], str(converted_dir), "--no-fix"],
        capture_output=True,
        text=True,
        env=_clean_env(),
    )
    if convert.returncode != 0:
        prepared.skipped = f"conversion failed: {(convert.stderr or convert.stdout).strip()[-400:]}"
        return prepared
    prepared.baseline = convert.stdout.strip().splitlines()[-1]

    from ..repair import RepairOptions, fix_asset

    options = RepairOptions(
        backends_requested=",".join(backends),
        multi_root=True,
        **(tuning or {}),
    )
    report = fix_asset(prepared.baseline, work / "stability", options)
    prepared.repair_summary = report["summary"]
    for root in report["output"].get("roots", []):
        for backend in backends:
            if root.endswith(f"_stabilized_{backend}.usda"):
                prepared.repaired[backend] = root
        if len(backends) == 1 and root.endswith("_stabilized.usda"):
            prepared.repaired[backends[0]] = root
    return prepared


def _clean_env() -> dict[str, str]:
    """Environment for a subprocess that is *not* Isaac Sim's interpreter.

    ``python.sh`` exports ``PYTHONHOME`` and ``PYTHONPATH`` pointing at Kit's
    own Python 3.12. Any other interpreter launched from inside it then loads
    Kit's standard library and dies on import, which presents as "conversion
    failed" with a traceback from a completely different Python.
    """
    import os

    env = dict(os.environ)
    for variable in ("PYTHONHOME", "PYTHONPATH", "PYTHONSTARTUP"):
        env.pop(variable, None)
    return env


def _find_converter_python() -> str | None:
    """An interpreter that has ``urdf_usd_converter``, if one is lying around.

    The converter matrix already builds these, so the common case needs no
    flag; anything else is explicit.
    """
    repo = Path(__file__).resolve().parents[3]
    candidates = [
        repo / "tests/_artifacts/venv-0.3.2/bin/python",
        repo / "tests/_artifacts/venv-0.3.2-isaac/bin/python",
        repo / "tests/_artifacts/venv-0.3.3/bin/python",
    ]
    for candidate in candidates:
        if candidate.exists():
            # Deliberately not resolve(): a venv's bin/python is a symlink to
            # the system interpreter, and following it throws away the venv --
            # and with it urdf-usd-converter.
            return str(candidate)
    return None


def run_matrix(
    *,
    out_dir: Path,
    robots: list[RobotSpec],
    backend_names: tuple[str, ...],
    suite_names: tuple[str, ...],
    converter_python: str | None = None,
    dt: float | None = None,
) -> dict[str, Any]:
    """Run every cell and return the results document."""
    out_dir.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    prepared_all: list[PreparedAsset] = []

    backends = {}
    for name in backend_names:
        backend = get_backend(name)
        usable, reason = backend.available()
        backends[name] = (backend, usable, reason)

    for spec in robots:
        prepared = prepare(spec, out_dir, backends=backend_names, converter_python=converter_python)
        prepared_all.append(prepared)
        if prepared.skipped:
            continue

        for backend_name, (backend, usable, reason) in backends.items():
            if not usable:
                results.append(
                    {
                        "suite": "*",
                        "asset_label": prepared.label,
                        "backend": backend_name,
                        "variant": "*",
                        "status": "unavailable",
                        "reason": reason,
                    }
                )
                continue
            variants = {BASELINE: prepared.baseline, REPAIRED: prepared.repaired.get(backend_name)}
            for variant, asset in variants.items():
                if not asset:
                    continue
                for suite_name in suite_names:
                    suite = SUITES[suite_name]
                    result = (
                        suite(asset, prepared.label, backend, dt=dt)
                        if _takes_dt(suite)
                        else suite(asset, prepared.label, backend)
                    )
                    result.variant = variant
                    results.append(result.as_dict())

    document = {
        "schema_version": 1,
        "environment": _environment(),
        "backends": {
            name: {"usable": usable, "reason": reason} for name, (_, usable, reason) in backends.items()
        },
        "assets": [
            {
                "label": p.label,
                "role": p.role,
                "exploratory": p.exploratory,
                "skipped": p.skipped,
                "provenance": p.provenance,
                "baseline": p.baseline,
                "repaired": p.repaired,
                "repair_summary": p.repair_summary,
            }
            for p in prepared_all
        ],
        "results": results,
    }
    document["comparisons"] = compare_results(results)
    document["agreement"] = agreement_results(results)
    return document


def run_sweep(
    *,
    out_dir: Path,
    robots: list[RobotSpec],
    backend_names: tuple[str, ...],
    frequencies: list[float],
    damping_ratios: list[float],
    armature_fractions: list[float] | None = None,
    timesteps: list[float] | None = None,
    converter_python: str | None = None,
) -> dict[str, Any]:
    """Re-repair each asset per tuning cell and measure hold-pose and a step.

    This is what turns the Phase 3 defaults from engineering choices into
    measurements, or shows that they cannot be justified -- which is also a
    result (``docs/PHASE4_DESIGN.md`` section 8).
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    backends = {}
    for name in backend_names:
        backend = get_backend(name)
        usable, reason = backend.available()
        backends[name] = (backend, usable, reason)

    rows: list[dict[str, Any]] = []
    armature_fractions = armature_fractions or [None]
    timesteps = timesteps or [None]
    for spec in robots:
        for frequency in frequencies:
            for zeta in damping_ratios:
                for alpha in armature_fractions:
                    tuning: dict[str, float] = {
                        "target_frequency": frequency,
                        "damping_ratio": zeta,
                        # 40 Hz at a 60 Hz control rate trips the sanity check;
                        # raise the assumed rate so the sweep can explore it.
                        "control_rate": max(240.0, frequency * 4.0),
                    }
                    if alpha is not None:
                        tuning["armature_fraction"] = alpha
                    label = f"{spec.label}-f{frequency:g}-z{zeta:g}" + (
                        f"-a{alpha:g}" if alpha is not None else ""
                    )
                    cell_dir = out_dir / "cells" / label
                    prepared = prepare(
                        RobotSpec(**{**spec.__dict__, "label": label}),
                        cell_dir,
                        backends=backend_names,
                        converter_python=converter_python,
                        tuning=tuning,
                    )
                    if prepared.skipped:
                        rows.append(
                            {
                                "asset": spec.label,
                                "frequency": frequency,
                                "zeta": zeta,
                                "armature_fraction": alpha,
                                "status": "skipped",
                                "reason": prepared.skipped,
                            }
                        )
                        continue
                    for backend_name, (backend, usable, _) in backends.items():
                        if not usable:
                            continue
                        asset = prepared.repaired.get(backend_name)
                        if not asset:
                            continue
                        for dt in timesteps:
                            for suite_name in ("hold_pose", "gain_step"):
                                suite = SUITES[suite_name]
                                result = (
                                    suite(asset, label, backend, dt=dt)
                                    if dt and _takes_dt(suite)
                                    else suite(asset, label, backend)
                                )
                                rows.append(
                                    {
                                        "asset": spec.label,
                                        "frequency": frequency,
                                        "zeta": zeta,
                                        "armature_fraction": alpha,
                                        "dt": dt,
                                        "backend": backend_name,
                                        "suite": suite_name,
                                        "status": result.status,
                                        "reason": result.reason,
                                        "metrics": result.metrics,
                                    }
                                )
    return {
        "schema_version": 1,
        "environment": _environment(),
        "sweep": {
            "frequencies": frequencies,
            "damping_ratios": damping_ratios,
            "armature_fractions": armature_fractions,
            "timesteps": timesteps,
        },
        "rows": rows,
    }


def _takes_dt(function) -> bool:
    import inspect

    return "dt" in inspect.signature(function).parameters


def compare_results(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Baseline vs repaired, per (asset, backend, suite, metric)."""
    index: dict[tuple, dict[str, dict[str, Any]]] = {}
    for row in results:
        if row.get("status") != "ok":
            continue
        key = (row["asset_label"], row["backend"], row["suite"])
        index.setdefault(key, {})[row["variant"]] = row

    out: list[dict[str, Any]] = []
    for (asset, backend, suite), variants in sorted(index.items()):
        baseline = variants.get(BASELINE)
        repaired = variants.get(REPAIRED)
        if not baseline or not repaired:
            continue
        for metric in sorted(set(baseline["metrics"]) | set(repaired["metrics"])):
            a, b = baseline["metrics"].get(metric), repaired["metrics"].get(metric)
            if isinstance(a, bool) or isinstance(b, bool):
                out.append(
                    {
                        "asset": asset,
                        "backend": backend,
                        "suite": suite,
                        "metric": metric,
                        "baseline": a,
                        "repaired": b,
                        "verdict": (
                            "improved"
                            if a and not b
                            else "worse" if b and not a else "no measurable difference"
                        ),
                    }
                )
                continue
            comparison = sim_metrics.compare(a, b, metric)
            comparison.update({"asset": asset, "backend": backend, "suite": suite})
            out.append(comparison)
    return out


def agreement_results(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Cross-backend agreement, per (asset, suite, variant)."""
    index: dict[tuple, dict[str, dict[str, Any]]] = {}
    for row in results:
        if row.get("status") != "ok":
            continue
        key = (row["asset_label"], row["suite"], row["variant"])
        index.setdefault(key, {})[row["backend"]] = row

    out = []
    for (asset, suite, variant), per_backend in sorted(index.items()):
        payload = {
            name: {
                "final_q": row["info"].get("final_q"),
                "settle_time": row["metrics"].get("settle_time"),
                "base_height_final": row["metrics"].get("base_height_final"),
                "dt_max_stable": row["metrics"].get("dt_max_stable"),
                "diverged": row["metrics"].get("diverged"),
                "pose_drift_final": row["metrics"].get("pose_drift_final"),
            }
            for name, row in per_backend.items()
        }
        # final_q is not recorded per backend yet; fall back to the drift metric,
        # which is the same quantity reduced to a scalar.
        for value in payload.values():
            if value["final_q"] is None:
                drift = value["pose_drift_final"]
                value["final_q"] = [drift] if drift is not None else None
        record = sim_metrics.agreement(payload)
        record.update({"asset": asset, "suite": suite, "variant": variant})
        out.append(record)
    return out


def write(document: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, default=str) + "\n")
