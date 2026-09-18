#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The same asset in three backends, before and after repair.

    <isaac>/python.sh examples/03_cross_backend.py [robot.urdf]

Needs Isaac Sim and an NVIDIA GPU: PhysX has no PyPI distribution, and Newton
and MuJoCo Warp come from Isaac Sim's bundled ``newton``.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))


def main() -> int:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        from urdf_usd_bridge.sim.backends import get_backend
        from urdf_usd_bridge.sim.backends import physx as physx_backend
        from urdf_usd_bridge.sim.corpus import RobotSpec
        from urdf_usd_bridge.sim.runner import prepare
        from urdf_usd_bridge.sim.suites import hold_pose

        physx_backend.set_simulation_app(app)

        urdf = sys.argv[1] if len(sys.argv) > 1 else str(REPO / "tests/fixtures/a_dynamics_damping.urdf")
        out = Path("cross_backend_example")
        spec = RobotSpec(label=Path(urdf).stem, urdf=urdf)
        backends = ("physx", "newton", "mujoco")
        prepared = prepare(spec, out, backends=backends)
        if prepared.skipped:
            print(f"could not prepare: {prepared.skipped}", file=sys.stderr)
            return 2

        print(f"{'backend':9} {'variant':9} {'drift (rad)':>13}  diverged")
        print("-" * 46)
        for name in backends:
            backend = get_backend(name)
            usable, why = backend.available()
            if not usable:
                print(f"{name:9} {'-':9} {'unavailable':>13}  {why[:40]}")
                continue
            for variant, asset in (
                ("baseline", prepared.baseline),
                ("repaired", prepared.repaired.get(name)),
            ):
                if not asset:
                    continue
                result = hold_pose(asset, spec.label, backend)
                if result.status != "ok":
                    print(f"{name:9} {variant:9} {result.status:>13}  {result.reason[:40]}")
                    continue
                m = result.metrics
                print(f"{name:9} {variant:9} {m['pose_drift_max']:>13.3e}  {m['diverged']}")

        print(
            "\nThe repaired rows should agree with each other far more closely than the\n"
            "baseline rows do. That agreement is the claim; see docs/history/PHASE4_REPORT.md."
        )
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
