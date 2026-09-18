# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Backend adapters. Each one builds the declared scene and returns arrays."""

from .base import Backend, BackendUnavailableError, RunRequest, Trajectory
from .newton_family import MuJoCoBackend, NewtonBackend
from .physx import PhysXBackend

#: Order matters only for readable reports.
BACKENDS = {"physx": PhysXBackend, "newton": NewtonBackend, "mujoco": MuJoCoBackend}


def get_backend(name: str):
    if name not in BACKENDS:
        raise KeyError(f"unknown backend {name!r}; known: {', '.join(BACKENDS)}")
    return BACKENDS[name]()


__all__ = [
    "BACKENDS",
    "Backend",
    "BackendUnavailableError",
    "MuJoCoBackend",
    "NewtonBackend",
    "PhysXBackend",
    "RunRequest",
    "Trajectory",
    "get_backend",
]
