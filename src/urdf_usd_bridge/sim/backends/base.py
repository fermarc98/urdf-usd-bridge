# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""What every backend has to provide, and what it has to hand back.

A backend's job is narrow: build the declared scene, run it, and return
trajectory arrays. It computes no metrics -- ``metrics.py`` does that, from the
arrays, identically for all three -- so a difference between backends can never
come from a difference in how a metric was calculated.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np


class BackendUnavailableError(RuntimeError):
    """The backend cannot run here. Skipped with a reason, never silently."""


@dataclass
class Trajectory:
    """Recorded state, plus whatever the backend could say about itself."""

    t: np.ndarray
    q: np.ndarray
    v: np.ndarray
    dof_names: list[str] = field(default_factory=list)
    base_height: np.ndarray | None = None
    energy: np.ndarray | None = None
    penetration: np.ndarray | None = None
    #: Gains the solver actually ended up with, for the evidence trail.
    applied_stiffness: list[float] | None = None
    applied_damping: list[float] | None = None
    applied_armature: list[float] | None = None
    info: dict[str, Any] = field(default_factory=dict)

    def as_arrays(self) -> dict[str, np.ndarray]:
        out = {"t": self.t, "q": self.q, "v": self.v}
        for name in ("base_height", "energy", "penetration"):
            value = getattr(self, name)
            if value is not None:
                out[name] = value
        return out


@dataclass
class RunRequest:
    """One simulation: an asset, a scene, and what to command."""

    asset: str
    scene: Any
    #: Per-DOF position targets in SI, or ``None`` to hold the initial pose.
    targets: np.ndarray | None = None
    #: Initial joint positions in SI, or ``None`` for the asset's own pose.
    initial_q: np.ndarray | None = None
    label: str = ""


class Backend(Protocol):
    name: str
    needs_gpu: bool

    def available(self) -> tuple[bool, str]:
        """``(usable, reason)``. A reason is required either way."""

    def run(self, request: RunRequest) -> Trajectory:
        """Simulate and return the trajectory. Raises BackendUnavailable."""
