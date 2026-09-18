# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The five suites, and the guards each one has to clear first.

A suite prepares a scene, asks a backend to run it, and turns the returned
arrays into metrics. It never computes a metric itself -- ``metrics.py`` does
that for every backend identically -- and it never reports a number whose guard
failed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from . import metrics as sim_metrics
from .backends.base import BackendUnavailableError, RunRequest
from .scene import (
    SceneSpec,
    check_gravity_loaded,
    check_no_interpenetration,
    loaded_pose,
    required_lift,
)


@dataclass
class SuiteResult:
    """One (suite, asset, backend, variant) cell of the matrix."""

    suite: str
    asset: str
    asset_label: str
    backend: str
    variant: str
    status: str = "ok"
    reason: str = ""
    metrics: dict[str, Any] = field(default_factory=dict)
    guards: list[dict[str, Any]] = field(default_factory=list)
    info: dict[str, Any] = field(default_factory=dict)
    scene: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "suite": self.suite,
            "asset": self.asset,
            "asset_label": self.asset_label,
            "backend": self.backend,
            "variant": self.variant,
            "status": self.status,
            "reason": self.reason,
            "metrics": self.metrics,
            "guards": self.guards,
            "info": self.info,
            "scene": self.scene,
        }


def open_for_reading(asset: str, variant: str | None = "physics"):
    """Open an asset on its own session layer, with a physics variant chosen.

    ``Usd.Stage.Open`` caches by identifier, so every caller gets its own
    session layer; otherwise selecting a variant here would change what another
    open stage resolves to.
    """
    from pxr import Sdf, Usd

    stage = Usd.Stage.Open(Sdf.Layer.FindOrOpen(asset), Sdf.Layer.CreateAnonymous(), Usd.Stage.LoadAll)
    prim = stage.GetDefaultPrim()
    if prim and prim.IsValid() and variant:
        sets = prim.GetVariantSets()
        if "Physics" in sets.GetNames():
            vset = sets.GetVariantSet("Physics")
            if not vset.GetVariantSelection() and variant in vset.GetVariantNames():
                vset.SetVariantSelection(variant)
    return stage


def _leaf(path: str) -> str:
    return path.rsplit("/", 1)[-1]


def targets_for(dof_names: list[str], pose: dict[str, float], default: float = 0.0) -> np.ndarray:
    """Map a ``{joint path: value}`` pose onto a backend's DOF ordering.

    Matched by joint leaf name, because Newton and Isaac Sim order and name
    their DOFs differently and neither exposes our prim paths.
    """
    by_leaf = {_leaf(path): value for path, value in pose.items()}
    return np.asarray([by_leaf.get(_leaf(name), default) for name in dof_names], dtype=np.float64)


def joint_limits_for(asset: str, dof_names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    """Per-DOF limits in SI, in the backend's DOF order."""
    from ..model.articulation import build_articulation
    from ..model.units import ANGULAR
    from .scene import _authored_float

    stage = open_for_reading(asset)
    articulation = build_articulation(stage)
    lower: dict[str, float] = {}
    upper: dict[str, float] = {}
    for joint in articulation.joints:
        if joint.dof is None:
            continue
        prim = stage.GetPrimAtPath(joint.path)
        low = _authored_float(prim, "physics:lowerLimit")
        high = _authored_float(prim, "physics:upperLimit")
        scale = math.radians if joint.dof == ANGULAR else (lambda x: x)
        lower[_leaf(joint.path)] = scale(low) if low is not None and math.isfinite(low) else -math.inf
        upper[_leaf(joint.path)] = scale(high) if high is not None and math.isfinite(high) else math.inf
    return (
        np.asarray([lower.get(_leaf(n), -math.inf) for n in dof_names]),
        np.asarray([upper.get(_leaf(n), math.inf) for n in dof_names]),
    )


def _slice_after_settle(traj, scene: SceneSpec):
    mask = sim_metrics.after_settle(traj.t, scene.settle_s)
    if not np.any(mask):  # pragma: no cover - a suite shorter than its settle window
        mask = np.ones_like(traj.t, dtype=bool)
    return traj.t[mask], traj.q[mask], traj.v[mask]


def measured_dofs(dof_names: list[str], guard_evidence: dict) -> list[int]:
    """Indices of the DOFs the guard certified as actually loaded.

    Metrics are computed over these columns only. A joint gravity cannot reach
    contributes nothing but noise to a drift figure, and including it would let
    an untested joint dilute -- or fake -- the result.
    """
    loaded = {_leaf(path) for path in guard_evidence.get("loaded_joints", [])}
    if not loaded:
        return list(range(len(dof_names)))
    return [i for i, name in enumerate(dof_names) if _leaf(name) in loaded] or list(range(len(dof_names)))


def _base_result(suite, asset, label, backend, variant, scene) -> SuiteResult:
    return SuiteResult(
        suite=suite,
        asset=asset,
        asset_label=label,
        backend=backend.name,
        variant=variant,
        scene=scene.as_dict(),
    )


# --- hold pose -------------------------------------------------------------


def hold_pose(asset: str, label: str, backend, *, dt: float | None = None, duration: float = 5.0):
    """Fixed base, joints commanded to a gravity-loaded pose, drift measured."""
    scene = SceneSpec(dt=dt or SceneSpec.dt, duration_s=duration, ground_plane=False)
    result = _base_result("hold_pose", asset, label, backend, "", scene)

    stage = open_for_reading(asset)
    pose = loaded_pose(stage)
    gravity_guard = check_gravity_loaded(stage, pose=pose)
    result.guards.append(gravity_guard.as_dict())
    if not gravity_guard.passed:
        result.status = "guard_failed"
        result.reason = gravity_guard.reason
        return result

    try:
        traj = backend.run(RunRequest(asset=asset, scene=scene, label=label, targets=None, initial_q=None))
    except BackendUnavailableError as exc:
        result.status = "unavailable"
        result.reason = str(exc)
        return result
    except Exception as exc:  # a backend blowing up is a result, not a crash
        result.status = "error"
        result.reason = f"{type(exc).__name__}: {exc}"
        return result

    targets = targets_for(traj.dof_names, pose)
    # The backend started from the asset's own pose; command the loaded one.
    try:
        traj = backend.run(
            RunRequest(asset=asset, scene=scene, label=label, targets=targets, initial_q=targets)
        )
    except Exception as exc:  # pragma: no cover - second call rarely differs
        result.status = "error"
        result.reason = f"{type(exc).__name__}: {exc}"
        return result

    t, q, v = _slice_after_settle(traj, scene)
    columns = measured_dofs(traj.dof_names, gravity_guard.evidence)
    result.metrics = {
        **sim_metrics.pose_drift(q[:, columns], targets[columns]),
        **sim_metrics.velocity_stats(v[:, columns]),
        "jitter": sim_metrics.jitter(v[:, columns], scene.dt),
        "settle_time": sim_metrics.settle_time(t, v[:, columns]),
        "diverged": sim_metrics.diverged(traj.q, traj.v, target=targets),
    }
    result.info = {
        **traj.info,
        "targets": targets.tolist(),
        "measured_dofs": [traj.dof_names[i] for i in columns],
        "excluded_dofs": [n for i, n in enumerate(traj.dof_names) if i not in columns],
        "final_q": traj.q[-1][columns].tolist(),
        "dof_names": traj.dof_names,
        "applied_stiffness": traj.applied_stiffness,
        "applied_damping": traj.applied_damping,
        "applied_armature": traj.applied_armature,
        "gravity_torque": gravity_guard.evidence.get("gravity_torque_per_joint"),
    }
    return result


# --- drop ------------------------------------------------------------------


def drop(asset: str, label: str, backend, *, dt: float | None = None, duration: float = 5.0):
    """Floating base released above the plane; contact and settling measured."""
    scene = SceneSpec(dt=dt or SceneSpec.dt, duration_s=duration, ground_plane=True, fixed_base=False)
    result = _base_result("drop", asset, label, backend, "", scene)

    stage = open_for_reading(asset)
    lift = required_lift(stage) + scene.drop_height
    penetration_guard = check_no_interpenetration(stage, lift=lift)
    result.guards.append(penetration_guard.as_dict())
    if not penetration_guard.passed:
        result.status = "guard_failed"
        result.reason = penetration_guard.reason
        return result

    pose = loaded_pose(stage)
    try:
        probe = backend.run(RunRequest(asset=asset, scene=scene, label=label))
    except BackendUnavailableError as exc:
        result.status = "unavailable"
        result.reason = str(exc)
        return result
    except Exception as exc:
        result.status = "error"
        result.reason = f"{type(exc).__name__}: {exc}"
        return result

    targets = targets_for(probe.dof_names, pose)
    try:
        traj = backend.run(
            RunRequest(asset=asset, scene=scene, label=label, targets=targets, initial_q=targets)
        )
    except Exception as exc:  # pragma: no cover
        result.status = "error"
        result.reason = f"{type(exc).__name__}: {exc}"
        return result

    _, q, v = _slice_after_settle(traj, scene)
    result.metrics = {
        **sim_metrics.velocity_stats(v),
        "settle_time": sim_metrics.settle_time(traj.t, traj.v),
        "diverged": sim_metrics.diverged(traj.q, traj.v),
        "pose_drift_final": sim_metrics.pose_drift(q, targets)["pose_drift_final"],
    }
    if traj.base_height is not None and np.any(np.isfinite(traj.base_height)):
        final = float(traj.base_height[-1])
        result.metrics["base_height_final"] = final
        result.metrics["base_height_drop"] = float(traj.base_height[0] - final)
    result.info = {**traj.info, "lift": lift, "dof_names": traj.dof_names}
    return result


# --- limit sweep -----------------------------------------------------------


def limit_sweep(asset: str, label: str, backend, *, dt: float | None = None, duration: float = 3.0):
    """Command each limited joint past its stop and measure what happens."""
    scene = SceneSpec(dt=dt or SceneSpec.dt, duration_s=duration, ground_plane=False)
    result = _base_result("limit_sweep", asset, label, backend, "", scene)

    stage = open_for_reading(asset)
    pose = loaded_pose(stage)
    try:
        probe = backend.run(RunRequest(asset=asset, scene=scene, label=label))
    except BackendUnavailableError as exc:
        result.status = "unavailable"
        result.reason = str(exc)
        return result
    except Exception as exc:
        result.status = "error"
        result.reason = f"{type(exc).__name__}: {exc}"
        return result

    lower, upper = joint_limits_for(asset, probe.dof_names)
    base_targets = targets_for(probe.dof_names, pose)
    finite = [
        i
        for i in range(len(probe.dof_names))
        if math.isfinite(lower[i]) and math.isfinite(upper[i]) and (upper[i] - lower[i]) > 1e-6
    ]
    if not finite:
        result.status = "not_applicable"
        result.reason = "no joint has a finite, non-degenerate limit to sweep"
        return result

    worst: dict[str, Any] = {"limit_overshoot": 0.0, "limit_escape": False, "chatter_freq": float("nan")}
    per_joint = []
    for index in finite:
        span = upper[index] - lower[index]
        targets = base_targets.copy()
        targets[index] = upper[index] + 0.2 * span  # 20% past the stop
        try:
            traj = backend.run(
                RunRequest(asset=asset, scene=scene, label=label, targets=targets, initial_q=base_targets)
            )
        except Exception as exc:  # pragma: no cover
            per_joint.append({"dof": probe.dof_names[index], "error": str(exc)})
            continue
        t, q, v = _slice_after_settle(traj, scene)
        limits = sim_metrics.limit_metrics(t, q, lower, upper)
        chatter = sim_metrics.chatter_frequency(v[:, [index]], scene.dt)
        per_joint.append(
            {
                "dof": probe.dof_names[index],
                **limits,
                "chatter_freq": chatter,
                "commanded": float(targets[index]),
            }
        )
        worst["limit_overshoot"] = max(worst["limit_overshoot"], limits["limit_overshoot"])
        worst["limit_escape"] = worst["limit_escape"] or limits["limit_escape"]
        if math.isfinite(chatter):
            worst["chatter_freq"] = (
                chatter if not math.isfinite(worst["chatter_freq"]) else max(worst["chatter_freq"], chatter)
            )

    result.metrics = worst
    result.info = {"per_joint": per_joint, "dof_names": probe.dof_names}
    return result


# --- gain sweep ------------------------------------------------------------


def gain_step(asset: str, label: str, backend, *, dof_index: int = 0, step_rad: float = math.radians(10.0)):
    """Hold-pose plus a step command, for the gain sweep's step metrics."""
    scene = SceneSpec(duration_s=3.0, ground_plane=False, settle_s=0.25)
    result = _base_result("gain_step", asset, label, backend, "", scene)

    stage = open_for_reading(asset)
    pose = loaded_pose(stage)
    guard = check_gravity_loaded(stage, pose=pose)
    result.guards.append(guard.as_dict())
    if not guard.passed:
        result.status = "guard_failed"
        result.reason = guard.reason
        return result

    try:
        probe = backend.run(RunRequest(asset=asset, scene=scene, label=label))
    except BackendUnavailableError as exc:
        result.status = "unavailable"
        result.reason = str(exc)
        return result
    except Exception as exc:
        result.status = "error"
        result.reason = f"{type(exc).__name__}: {exc}"
        return result

    start = targets_for(probe.dof_names, pose)
    if dof_index >= len(start):
        result.status = "not_applicable"
        result.reason = "no such DOF"
        return result
    targets = start.copy()
    targets[dof_index] = start[dof_index] + step_rad

    try:
        traj = backend.run(
            RunRequest(asset=asset, scene=scene, label=label, targets=targets, initial_q=start)
        )
    except Exception as exc:  # pragma: no cover
        result.status = "error"
        result.reason = f"{type(exc).__name__}: {exc}"
        return result

    response = sim_metrics.step_response(
        traj.t, traj.q[:, dof_index], start=float(start[dof_index]), target=float(targets[dof_index])
    )
    _, q, v = _slice_after_settle(traj, scene)
    result.metrics = {
        **response,
        "jitter": sim_metrics.jitter(v, scene.dt),
        "pose_drift_final": sim_metrics.pose_drift(q, targets)["pose_drift_final"],
        "diverged": sim_metrics.diverged(traj.q, traj.v, target=targets),
    }
    result.info = {
        **traj.info,
        "dof": probe.dof_names[dof_index],
        "step_rad": step_rad,
        "applied_stiffness": traj.applied_stiffness,
    }
    return result


SUITES = {
    "hold_pose": hold_pose,
    "drop": drop,
    "limit_sweep": limit_sweep,
    "gain_step": gain_step,
}
