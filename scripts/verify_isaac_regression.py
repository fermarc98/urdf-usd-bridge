#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Check, on a real Isaac Sim install, whether URDF joint damping survives import.

**This script does not run on the development machine.** It needs an Isaac Sim
installation, which is Linux + NVIDIA GPU only. See ``docs/VERIFY.md``.

Background
----------
``docs/ANALYSIS.md`` G1 claims that Isaac Sim 6.1.0 silently drops
``<dynamics damping>`` and ``<dynamics friction>``:

* ``urdf-usd-converter`` 0.3.0+ writes ``newton:damping`` / ``newton:friction``.
* Isaac Sim's ``urdf_to_mjc_physx_conversion_utils.convert_urdf_to_physx()``
  reads ``urdf:dynamics:damping`` / ``urdf:dynamics:friction``, the spelling
  used by 0.2.0 and earlier.
* Isaac Sim 6.1.0 pins 0.3.2, so the two never meet; 6.0.1 pinned 0.1.3, where
  they did.

``tests/unit/test_reference_sources.py`` already proves this from upstream
source. This script proves it from upstream *behaviour*, which is what an
upstream bug report needs.

What it does
------------
1. Imports ``tests/fixtures/a_dynamics_damping.urdf`` with ``URDFImporter`` and
   stock config -- exactly what a user gets by default.
2. Inspects the result with ``urdf_usd_bridge`` (added to ``sys.path`` from this
   repo; nothing needs installing into Isaac Sim's interpreter).
3. Repeats with explicit drive overrides, to confirm the documented workaround
   still populates the drives -- which distinguishes "damping was dropped" from
   "our inspector cannot see drives".
4. Writes a JSON report and prints a verdict.

Run it once under Isaac Sim 6.1.0 and once under 6.0.1 and diff the verdicts.

Kit is required
---------------
On a real Isaac Sim install the importer cannot be imported from a bare
``python.sh``: ``isaacsim`` is a *regular* package rooted at
``python_packages/isaacsim`` with a fixed ``__path__``, and each extension ships
its implementation under its own ``exts/<ext>/pip_prebundle/isaacsim/...``.
Only Kit's extension manager stitches those trees together, so this script boots
a headless ``SimulationApp`` and enables ``isaacsim.asset.importer.urdf`` before
importing it. No rendering happens; ``--no-simulation-app`` opts out.
"""

from __future__ import annotations

import argparse
import json
import platform
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE = REPO_ROOT / "tests" / "fixtures" / "a_dynamics_damping.urdf"

# URDF input values, for reference in the report.
URDF_REVOLUTE_DAMPING = 1.5  # N*m*s/rad on shoulder_joint
URDF_REVOLUTE_FRICTION = 0.3  # N*m on shoulder_joint
URDF_REVOLUTE_EFFORT = 87.0  # N*m on shoulder_joint


def _bootstrap_bridge() -> None:
    """Make ``urdf_usd_bridge`` importable without installing it."""
    src = REPO_ROOT / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))


IMPORTER_EXTENSION = "isaacsim.asset.importer.urdf"


def _boot_simulation_app():
    """Start a headless Kit app and enable the URDF importer extension.

    Returns the ``SimulationApp`` (to be closed by the caller), or ``None`` if
    booting was skipped.
    """
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(IMPORTER_EXTENSION):
        manager.set_extension_enabled_immediate(IMPORTER_EXTENSION, True)
    return app


def _isaac_version() -> str:
    try:
        from isaacsim.core.version import get_version

        return str(get_version())
    except Exception:
        return "unknown"


def _import_urdf(urdf: Path, out_dir: Path, **overrides) -> str:
    """Run the official importer and return the path to the resulting asset."""
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    config = URDFImporterConfig(urdf_path=str(urdf), usd_path=str(out_dir), **overrides)
    return URDFImporter(config).import_urdf()


def _inspect(asset: str, variant: str | None) -> dict:
    from urdf_usd_bridge.inspection import inspect_stage

    selections = {"Physics": variant} if variant else None
    return inspect_stage(asset, selections)


def _joint(report: dict, name: str) -> dict | None:
    return next((j for j in report["joints"] if j["name"] == name), None)


def _authored(reading):
    return reading["value"] if reading and reading.get("authored") else None


def _observe(report: dict) -> dict:
    """Pull the handful of values the claim turns on."""
    joint = _joint(report, "shoulder_joint")
    if joint is None:
        return {"error": "shoulder_joint not found", "joints": [j["name"] for j in report["joints"]]}

    drive = joint["drives"].get("angular", {})
    actuators = [a for a in report["mjc_actuators"] if any("shoulder" in t for t in (a["target"] or []))]
    return {
        "drive_api_applied": bool(joint["drives"]),
        "drive_damping": _authored(drive.get("damping")),
        "drive_stiffness": _authored(drive.get("stiffness")),
        "drive_max_force": _authored(drive.get("maxForce")),
        "newton_damping": _authored(joint["damping_by_namespace"]["newton"]),
        "urdf_dynamics_damping": _authored(joint["damping_by_namespace"]["urdf_custom"]),
        "mjc_damping": _authored(joint["damping_by_namespace"]["mjc"]),
        "newton_friction": _authored(joint["friction_by_namespace"]["newton"]),
        "physx_joint_friction": _authored(joint["friction_by_namespace"]["physx"]),
        "mjc_frictionloss": _authored(joint["friction_by_namespace"]["mjc"]),
        "urdf_limit_effort": _authored(joint["limit_extras"]["urdf_effort"]),
        "armature_namespaces": joint["flags"]["armature_namespaces"],
        "mjc_actuator_count": len(actuators),
        "mjc_actuator_has_gains": [a["flags"]["has_gain_parameters"] for a in actuators],
    }


def _verdict(default: dict, overridden: dict | None) -> tuple[str, list[str]]:
    """Decide what the observations say about the G1 claim."""
    reasons: list[str] = []
    if "error" in default:
        return "INCONCLUSIVE", [default["error"]]

    damping_reached_drive = bool(default["drive_damping"])
    friction_reached_physx = default["physx_joint_friction"] is not None
    actuators_have_gains = any(default["mjc_actuator_has_gains"] or [])

    if damping_reached_drive:
        reasons.append(
            f"drive:angular:physics:damping is authored ({default['drive_damping']}), "
            "so URDF damping did reach the PhysX drive"
        )
        return "REGRESSION_ABSENT", reasons

    reasons.append("drive:angular:physics:damping is not authored")
    if default["newton_damping"] is not None:
        reasons.append(
            f"newton:damping is authored ({default['newton_damping']}), so the converter "
            "did carry the value -- it is the Isaac-side reader that misses it"
        )
    if default["urdf_dynamics_damping"] is None:
        reasons.append("urdf:dynamics:damping is absent, which is the attribute Isaac reads")
    if not friction_reached_physx:
        reasons.append("physxJoint:jointFriction is not authored either")
    if not actuators_have_gains:
        reasons.append("MjcActuator prims carry no gainPrm/biasPrm")

    if overridden is not None:
        if overridden.get("drive_damping"):
            reasons.append(
                "with explicit override_joint_damping the drive IS populated "
                f"({overridden['drive_damping']}), so drives are readable and the "
                "default path is genuinely dropping the URDF value"
            )
        else:
            return "INCONCLUSIVE", [
                *reasons,
                "override_joint_damping also produced no drive damping; the inspection "
                "path may be wrong rather than the importer",
            ]

    return "REGRESSION_CONFIRMED", reasons


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="isaac_regression", help="output directory")
    parser.add_argument(
        "--variant",
        default="physx",
        help="Physics variant to select before inspecting (6.1.0 authors no default)",
    )
    parser.add_argument(
        "--skip-override-run",
        action="store_true",
        help="skip the control run that passes explicit drive gains",
    )
    parser.add_argument("--json", default=None, help="write the JSON report here")
    parser.add_argument(
        "--no-simulation-app",
        action="store_true",
        help="do not boot Kit first (only useful if the importer is importable standalone)",
    )
    args = parser.parse_args(argv)

    _bootstrap_bridge()
    out_root = Path(args.out).resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    app = None
    if not args.no_simulation_app:
        print("booting headless Kit (needed to resolve the importer extension) ...")
        app = _boot_simulation_app()
    try:
        return _run(args, report_root=out_root)
    finally:
        if app is not None:
            app.close()


def _run(args, report_root: Path) -> int:
    out_root = report_root

    report: dict = {
        "fixture": str(FIXTURE),
        "urdf_input": {
            "shoulder_joint.dynamics.damping": URDF_REVOLUTE_DAMPING,
            "shoulder_joint.dynamics.friction": URDF_REVOLUTE_FRICTION,
            "shoulder_joint.limit.effort": URDF_REVOLUTE_EFFORT,
        },
        "host": {"platform": platform.platform(), "python": sys.version.split()[0]},
        "isaac_sim_version": _isaac_version(),
        "variant_selected": args.variant,
        "runs": {},
    }

    print(f"Isaac Sim version: {report['isaac_sim_version']}")
    print(f"importing {FIXTURE.name} with stock URDFImporterConfig ...")
    default_asset = _import_urdf(FIXTURE, out_root / "default")
    report["runs"]["default"] = {
        "asset": default_asset,
        "observed": _observe(_inspect(default_asset, args.variant)),
    }

    overridden = None
    if not args.skip_override_run:
        print("importing again with explicit drive overrides (control) ...")
        override_asset = _import_urdf(
            FIXTURE,
            out_root / "overridden",
            joint_drive_type="force",
            joint_target_type="position",
            override_joint_stiffness=800.0,
            override_joint_damping=40.0,
        )
        overridden = _observe(_inspect(override_asset, args.variant))
        report["runs"]["overridden"] = {"asset": override_asset, "observed": overridden}

    verdict, reasons = _verdict(report["runs"]["default"]["observed"], overridden)
    report["verdict"] = verdict
    report["reasons"] = reasons

    payload = json.dumps(report, indent=2)
    if args.json:
        Path(args.json).write_text(payload + "\n")
        print(f"wrote {args.json}")

    print("\n" + "=" * 72)
    print(f"VERDICT: {verdict}")
    for reason in reasons:
        print(f"  - {reason}")
    print("=" * 72)
    print("\nObserved (default import):")
    for key, value in report["runs"]["default"]["observed"].items():
        print(f"  {key:26s} {value}")

    return 0 if verdict != "INCONCLUSIVE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
