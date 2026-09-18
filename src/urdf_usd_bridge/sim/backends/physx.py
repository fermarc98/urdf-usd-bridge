# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""PhysX, through Isaac Sim.

There is no PhysX on PyPI, so this backend needs an Isaac Sim install and the
process to be running under its ``python.sh`` with a ``SimulationApp`` already
started. The runner boots Kit once and shares it across every PhysX run, because
a cold start costs about 150 s and a warm one about 10 s.

Nothing is rendered. ``World.step(render=False)`` advances physics only.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .base import BackendUnavailableError, RunRequest, Trajectory

#: Set by the runner once Kit is up, so backends do not each boot their own.
_SIMULATION_APP: Any = None


def set_simulation_app(app) -> None:
    global _SIMULATION_APP
    _SIMULATION_APP = app


def simulation_app_running() -> bool:
    return _SIMULATION_APP is not None


class PhysXBackend:
    name = "physx"
    needs_gpu = True

    def available(self) -> tuple[bool, str]:
        if not simulation_app_running():
            return False, (
                "PhysX needs a running Isaac Sim SimulationApp; run the matrix under "
                "<isaac>/python.sh so the runner can start one"
            )
        return True, "SimulationApp running"

    def run(self, request: RunRequest) -> Trajectory:
        if not simulation_app_running():
            raise BackendUnavailableError(self.available()[1])

        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import add_reference_to_stage

        scene = request.scene
        world = World(stage_units_in_meters=1.0, physics_dt=scene.dt, rendering_dt=scene.dt)
        world.clear()
        if scene.ground_plane:
            world.scene.add_default_ground_plane(
                static_friction=scene.ground_friction,
                dynamic_friction=scene.ground_friction,
                restitution=scene.ground_restitution,
            )
        add_reference_to_stage(usd_path=request.asset, prim_path="/World/robot")
        world.reset()

        articulation = Articulation("/World/robot")
        articulation.initialize()
        dof_names = list(articulation.dof_names or [])
        dof_count = len(dof_names)
        if dof_count == 0:
            raise BackendUnavailableError("Isaac Sim found no articulated degrees of freedom")

        if request.initial_q is not None:
            wanted = np.asarray(request.initial_q, dtype=np.float32).reshape(1, -1)
            if wanted.shape[1] == dof_count:
                articulation.set_joint_positions(wanted)
                world.step(render=False)

        targets = (
            np.asarray(request.targets, dtype=np.float32).reshape(1, -1)
            if request.targets is not None
            else np.asarray(articulation.get_joint_positions(), dtype=np.float32).reshape(1, -1)
        )
        articulation.set_joint_position_targets(targets)

        stiffness, damping = articulation.get_gains()
        steps = scene.steps
        t = np.arange(steps, dtype=np.float64) * scene.dt
        q = np.zeros((steps, dof_count), dtype=np.float64)
        v = np.zeros((steps, dof_count), dtype=np.float64)
        base_height = np.zeros(steps, dtype=np.float64)

        for step in range(steps):
            world.step(render=False)
            q[step] = np.asarray(articulation.get_joint_positions(), dtype=np.float64).ravel()[:dof_count]
            v[step] = np.asarray(articulation.get_joint_velocities(), dtype=np.float64).ravel()[:dof_count]
            try:
                positions, _ = articulation.get_world_poses()
                base_height[step] = float(np.asarray(positions).ravel()[2])
            except Exception:  # pragma: no cover - not every asset exposes a base pose
                base_height[step] = np.nan
            if not np.all(np.isfinite(q[step])):
                q[step:] = q[step]
                v[step:] = np.nan
                break

        return Trajectory(
            t=t,
            q=q,
            v=v,
            dof_names=dof_names,
            base_height=base_height,
            applied_stiffness=[float(x) for x in np.asarray(stiffness).ravel()],
            applied_damping=[float(x) for x in np.asarray(damping).ravel()],
            info={"solver": "PhysX (Isaac Sim)", "dof_count": dof_count},
        )
