#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Run the cross-backend simulation matrix. One command, JSON out.

    <isaac>/python.sh scripts/run_sim_matrix.py --out sim_artifacts --json results.json

PhysX needs Isaac Sim, so the script boots one headless ``SimulationApp`` for
the whole run when it can -- a cold Kit start costs ~150 s and a warm one ~10 s,
so booting per run would dominate the matrix. Newton and MuJoCo Warp come from
Isaac Sim's bundled ``newton``; both also work in a plain venv with
``newton[sim]``, in which case ``--backend newton,mujoco`` skips PhysX cleanly.

Artifacts land under ``--out`` and are gitignored. Every result carries the
environment block that produced it, and every backend records whether it needed
a GPU.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="sim_artifacts", help="artifact directory")
    parser.add_argument("--json", default=None, help="write the results document here")
    parser.add_argument(
        "--backend",
        default="physx,newton,mujoco",
        help="comma-separated subset of physx, newton, mujoco",
    )
    parser.add_argument(
        "--suite",
        default="hold_pose,drop,limit_sweep",
        help="comma-separated subset; known: hold_pose, drop, limit_sweep, gain_step",
    )
    parser.add_argument("--asset", action="append", help="restrict to these corpus labels, repeatable")
    parser.add_argument("--dt", type=float, default=None, help="override the scene timestep")
    parser.add_argument("--no-public", action="store_true", help="fixtures only, no downloads")
    parser.add_argument(
        "--exploratory", action="store_true", help="include exploratory robots (reported separately)"
    )
    parser.add_argument("--converter-python", default=None, help="interpreter with urdf-usd-converter")
    parser.add_argument("--no-simulation-app", action="store_true", help="do not boot Kit (skips PhysX)")
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="run the tuning sweep instead of the suite matrix (produces the measured defaults)",
    )
    parser.add_argument("--frequencies", default="2,5,10,20,40", help="sweep: target frequencies in Hz")
    parser.add_argument("--zetas", default="0.5,0.7,1.0,1.4", help="sweep: damping ratios")
    parser.add_argument("--alphas", default=None, help="sweep: armature fractions, e.g. 0,0.01,0.1")
    parser.add_argument("--dts", default=None, help="sweep: timesteps, e.g. 0.016667,0.008333,0.004167")
    args = parser.parse_args(argv)

    backend_names = tuple(b.strip() for b in args.backend.split(",") if b.strip())
    suite_names = tuple(s.strip() for s in args.suite.split(",") if s.strip())

    app = None
    if "physx" in backend_names and not args.no_simulation_app:
        try:
            from isaacsim import SimulationApp

            print("booting headless Kit for PhysX ...", file=sys.stderr)
            app = SimulationApp({"headless": True})
            from urdf_usd_bridge.sim.backends import physx as physx_backend

            physx_backend.set_simulation_app(app)
        except ImportError as exc:
            print(f"PhysX unavailable ({exc}); continuing without it", file=sys.stderr)

    try:
        from urdf_usd_bridge.sim.corpus import corpus
        from urdf_usd_bridge.sim.runner import run_matrix, write

        robots = corpus(include_public=not args.no_public, include_exploratory=args.exploratory)
        if args.asset:
            wanted = set(args.asset)
            robots = [r for r in robots if r.label in wanted]
        if not robots:
            print("no assets selected", file=sys.stderr)
            return 2

        if args.sweep:
            from urdf_usd_bridge.sim.runner import run_sweep

            document = run_sweep(
                out_dir=Path(args.out),
                robots=robots,
                backend_names=backend_names,
                frequencies=[float(x) for x in args.frequencies.split(",") if x],
                damping_ratios=[float(x) for x in args.zetas.split(",") if x],
                armature_fractions=([float(x) for x in args.alphas.split(",") if x] if args.alphas else None),
                timesteps=[float(x) for x in args.dts.split(",") if x] if args.dts else None,
                converter_python=args.converter_python,
            )
            target = Path(args.json) if args.json else Path(args.out) / "sweep.json"
            write(document, target)
            print(f"wrote {target}")
            ok = sum(1 for r in document["rows"] if r.get("status") == "ok")
            print(f"{ok} of {len(document['rows'])} sweep rows produced metrics")
            return 0

        document = run_matrix(
            out_dir=Path(args.out),
            robots=robots,
            backend_names=backend_names,
            suite_names=suite_names,
            converter_python=args.converter_python,
            dt=args.dt,
        )
        target = Path(args.json) if args.json else Path(args.out) / "results.json"
        write(document, target)
        print(f"wrote {target}")

        ok = sum(1 for r in document["results"] if r.get("status") == "ok")
        print(f"{ok} of {len(document['results'])} cells produced metrics")
        for row in document["results"]:
            if row.get("status") not in ("ok", None):
                print(
                    f"  {row.get('status'):14} {row.get('asset_label')}/{row.get('backend')}"
                    f"/{row.get('suite')}: {str(row.get('reason'))[:100]}"
                )
        return 0
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    raise SystemExit(main())
