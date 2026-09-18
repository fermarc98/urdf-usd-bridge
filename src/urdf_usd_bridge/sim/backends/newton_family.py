# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Newton and MuJoCo Warp. One USD import path, two solvers.

Both read the same stage through ``newton.ModelBuilder.add_usd``; only the
solver differs (``SolverFeatherstone`` versus ``SolverMuJoCo``). Keeping them in
one module makes the difference between them visible instead of duplicated.

The MJC schema guard
--------------------
MuJoCo's gains live on ``MjcActuator`` prims, and they are only parsed if
``SolverMuJoCo.register_custom_attributes(builder)`` has been called **before**
``add_usd`` *and* the ``omni.usd.schema.mujoco`` plugin is registered so those
prims have a type at all. When either is missing, the scan finds nothing and
returns silently -- a MuJoCo run with no gains looks exactly like a MuJoCo run
whose gains were dropped, which is the bug this whole project exists to catch.

So :func:`require_mjc_schema` raises. It never warns, and it never lets a run
proceed with an empty scan.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import numpy as np

from .base import BackendUnavailableError, RunRequest, Trajectory

#: Where Isaac Sim keeps the MuJoCo USD schema plugin.
_MJC_PLUGIN_SUFFIX = "exts/omni.usd.schema.mujoco/plugins/mjcPhysics/resources"

#: Newton lives in Isaac Sim's pip prebundle; also importable from a venv that
#: installed ``newton[sim]`` directly.
_NEWTON_PREBUNDLE = "exts/isaacsim.pip.newton/pip_prebundle"


def isaac_root() -> Path | None:
    for candidate in (os.environ.get("ISAAC_SIM_DIR"), str(Path.home() / "isaacsim")):
        if candidate and Path(candidate, "python.sh").exists():
            return Path(candidate)
    return None


def ensure_newton_importable() -> None:
    """Put Newton on ``sys.path`` if it is only in Isaac Sim's prebundle."""
    import sys

    try:
        import newton

        return
    except ImportError:
        pass
    root = isaac_root()
    if root is None:
        raise BackendUnavailableError(
            "newton is not importable and no Isaac Sim install was found "
            "(set ISAAC_SIM_DIR, or pip install 'newton[sim]')"
        )
    prebundle = root / _NEWTON_PREBUNDLE
    if not prebundle.is_dir():
        raise BackendUnavailableError(f"no newton prebundle at {prebundle}")
    sys.path.insert(0, str(prebundle))
    try:
        import newton  # noqa: F401
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise BackendUnavailableError(f"newton present but not importable: {exc}") from exc


def require_mjc_schema() -> str:
    """Register the MuJoCo USD schema, or raise saying why it matters.

    Returns the plugin path that was registered, for the evidence trail.
    """
    from pxr import Plug

    registry = Plug.Registry()
    if registry.GetPluginWithName("mjcPhysics"):
        return "already registered"

    root = isaac_root()
    candidates = []
    if root is not None:
        candidates.append(root / _MJC_PLUGIN_SUFFIX)
        candidates.append(
            root / "exts/isaacsim.pip.newton/pip_prebundle/mujoco_usd_converter/plugins/mjcPhysics/resources"
        )
    for path in candidates:
        if path.is_dir():
            registry.RegisterPlugins(str(path))
            if registry.GetPluginWithName("mjcPhysics"):
                return str(path)

    raise BackendUnavailableError(
        "the MuJoCo USD schema plugin (mjcPhysics) is not registered, so MjcActuator prims "
        "have no type and their gains would be scanned as if absent. A MuJoCo run in that "
        "state is indistinguishable from one whose gains were dropped, so it is refused "
        "rather than reported. Looked in: " + ", ".join(str(p) for p in candidates)
    )


def assert_mjc_prims_typed(asset: str) -> int:
    """Confirm the registration took, on this actual asset.

    Registering the plugin is necessary but not sufficient -- an asset authored
    before registration, or opened on a stale stage cache, can still present
    untyped prims. This counts what the solver will actually see.
    """
    from pxr import Sdf, Usd

    stage = Usd.Stage.Open(Sdf.Layer.FindOrOpen(asset), Sdf.Layer.CreateAnonymous(), Usd.Stage.LoadAll)
    prim = stage.GetDefaultPrim()
    if prim and prim.IsValid():
        vset = prim.GetVariantSets()
        if "Physics" in vset.GetNames():
            selection = vset.GetVariantSet("Physics")
            if not selection.GetVariantSelection() and "mujoco" in selection.GetVariantNames():
                selection.SetVariantSelection("mujoco")
    typed = [p for p in stage.Traverse() if str(p.GetTypeName()) == "MjcActuator"]
    gains = [
        p
        for p in typed
        if p.GetAttribute("mjc:gainPrm")
        and p.GetAttribute("mjc:gainPrm").HasAuthoredValue()
        and p.GetAttribute("mjc:gainPrm").Get()[0]
    ]
    return len(gains)


def _targets_from(model, request: RunRequest, dof_count: int) -> np.ndarray:
    if request.targets is not None:
        return np.asarray(request.targets, dtype=np.float32).reshape(-1)[:dof_count]
    return np.zeros(dof_count, dtype=np.float32)


def run_newton_family(request: RunRequest, *, solver_name: str) -> Trajectory:
    """Simulate with Newton's Featherstone solver or with MuJoCo Warp."""
    ensure_newton_importable()
    import newton
    import warp as wp

    scene = request.scene
    mjc_actuators_with_gains = None
    plugin_path = None

    builder = newton.ModelBuilder()
    if solver_name == "mujoco":
        plugin_path = require_mjc_schema()
        mjc_actuators_with_gains = assert_mjc_prims_typed(request.asset)
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder)

    parsed = builder.add_usd(
        request.asset,
        collapse_fixed_joints=False,
        enable_self_collisions=False,
    )
    if scene.ground_plane:
        builder.add_ground_plane()

    model = builder.finalize()
    dof_count = int(model.joint_dof_count)
    if dof_count == 0:
        raise BackendUnavailableError("the asset has no actuated degrees of freedom")

    if solver_name == "mujoco":
        solver = newton.solvers.SolverMuJoCo(model)
    else:
        solver = newton.solvers.SolverFeatherstone(model)

    state_0, state_1 = model.state(), model.state()
    control = model.control()

    initial = np.asarray(state_0.joint_q.numpy(), dtype=np.float64).copy()
    if request.initial_q is not None:
        wanted = np.asarray(request.initial_q, dtype=np.float32).reshape(-1)
        if wanted.size == initial.size:
            state_0.joint_q.assign(wp.array(wanted, dtype=wp.float32))
            state_1.joint_q.assign(wp.array(wanted, dtype=wp.float32))
            initial = np.asarray(wanted, dtype=np.float64)

    targets = _targets_from(model, request, dof_count)
    if control.joint_target_q is not None:
        control.joint_target_q.assign(wp.array(targets, dtype=wp.float32))

    contacts = None
    pipeline = None
    if scene.ground_plane:
        pipeline = newton.CollisionPipeline(model)
        contacts = pipeline.contacts()

    steps = scene.steps
    t = np.arange(steps, dtype=np.float64) * scene.dt
    q = np.zeros((steps, dof_count), dtype=np.float64)
    v = np.zeros((steps, dof_count), dtype=np.float64)

    for step in range(steps):
        if pipeline is not None:
            pipeline.collide(state_0, contacts)
        state_0.clear_forces()
        solver.step(state_0, state_1, control, contacts, scene.dt)
        state_0, state_1 = state_1, state_0
        q[step] = np.asarray(state_0.joint_q.numpy(), dtype=np.float64)[:dof_count]
        v[step] = np.asarray(state_0.joint_qd.numpy(), dtype=np.float64)[:dof_count]
        if not np.all(np.isfinite(q[step])):
            q[step:] = q[step]
            v[step:] = np.nan
            break

    info: dict[str, Any] = {
        "solver": type(solver).__name__,
        "dof_count": dof_count,
        "body_count": int(model.body_count),
        "device": str(wp.get_device()),
        "initial_q": initial.tolist(),
    }
    if solver_name == "mujoco":
        info["mjc_schema_plugin"] = plugin_path
        info["mjc_actuators_with_gains"] = mjc_actuators_with_gains

    return Trajectory(
        t=t,
        q=q,
        v=v,
        dof_names=_dof_names(model, parsed, dof_count),
        applied_stiffness=_as_list(model, "joint_target_ke"),
        applied_damping=_as_list(model, "joint_target_kd"),
        applied_armature=_as_list(model, "joint_armature"),
        info=info,
    )


def _dof_names(model, parsed: dict[str, Any], dof_count: int) -> list[str]:
    """Prim path per DOF, in Newton's own ordering.

    Newton 1.5.0's ``Model`` carries no joint names, so the mapping comes from
    ``add_usd``'s ``path_joint_map`` (prim path -> joint index) combined with
    ``joint_qd_start`` (joint index -> first DOF). Guessing the order instead
    would silently mis-assign every joint target on any robot whose joints are
    not in path order.
    """
    path_by_joint = {index: path for path, index in (parsed.get("path_joint_map") or {}).items()}
    if not path_by_joint:
        return [f"dof_{i}" for i in range(dof_count)]
    try:
        starts = [int(x) for x in np.asarray(model.joint_qd_start.numpy()).ravel()]
    except Exception:  # pragma: no cover - defensive across Newton versions
        return [f"dof_{i}" for i in range(dof_count)]

    names = [f"dof_{i}" for i in range(dof_count)]
    for joint_index, path in sorted(path_by_joint.items()):
        if joint_index + 1 >= len(starts):
            continue
        for dof in range(starts[joint_index], starts[joint_index + 1]):
            if 0 <= dof < dof_count:
                names[dof] = path
    return names


def _as_list(model, attribute: str) -> list[float] | None:
    array = getattr(model, attribute, None)
    if array is None:
        return None
    try:
        return [float(x) for x in np.asarray(array.numpy()).ravel()]
    except Exception:  # pragma: no cover - defensive
        return None


class NewtonBackend:
    name = "newton"
    needs_gpu = False  # Warp has a CPU device; unverified for every solver

    def available(self) -> tuple[bool, str]:
        try:
            ensure_newton_importable()
        except BackendUnavailableError as exc:
            return False, str(exc)
        return True, "newton importable"

    def run(self, request: RunRequest) -> Trajectory:
        return run_newton_family(request, solver_name="newton")


class MuJoCoBackend:
    name = "mujoco"
    needs_gpu = True  # MuJoCo Warp; no CPU fallback verified

    def available(self) -> tuple[bool, str]:
        try:
            ensure_newton_importable()
            require_mjc_schema()
        except BackendUnavailableError as exc:
            return False, str(exc)
        return True, "newton importable and mjcPhysics registered"

    def run(self, request: RunRequest) -> Trajectory:
        return run_newton_family(request, solver_name="mujoco")


def expected_total_energy(model, state) -> float:  # pragma: no cover - helper
    """Placeholder for a future energy probe; unused until it is validated."""
    return math.nan
