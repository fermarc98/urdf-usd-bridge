#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""What a converted asset is actually missing.

Needs nothing but ``usd-core``. Works on macOS, where the converter cannot be
installed at all.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from urdf_usd_bridge.inspection import inspect_stage  # noqa: E402


def main() -> int:
    asset = sys.argv[1] if len(sys.argv) > 1 else None
    if asset is None:
        candidate = REPO / "tests/_artifacts/0.3.2/a_dynamics_damping/a_dynamics_damping.usda"
        if not candidate.exists():
            print(
                "pass an asset path, or build the fixtures first:\n"
                "    python scripts/run_converter_matrix.py",
                file=sys.stderr,
            )
            return 2
        asset = str(candidate)

    report = inspect_stage(asset)
    summary = report["summary"]

    print(f"asset:  {asset}")
    print(f"layout: {summary['layout']}\n")

    print("What the converter authored:")
    print(f"  actuatable joints          {summary['joints_actuatable']}")
    print(f"  ... with drive gains       {summary['joints_with_drive_gains']}")
    print(f"  ... with no armature       {summary['joints_without_armature']}")
    print(f"  joints welded at [0, 0]    {summary['joints_locked_by_equal_limits']}")
    print(f"  bodies with mass, no inertia {summary['bodies_mass_without_authored_inertia']}")
    print(f"  bodies with a zero quaternion {summary['bodies_invalid_principal_axes']}")

    print("\nWhere the damping ended up (the G1 finding, one joint at a time):")
    for joint in report["joints"]:
        damping = joint["damping_by_namespace"]
        authored = {name: r["value"] for name, r in damping.items() if r and r["authored"]}
        if not authored:
            continue
        drive = joint["drives"].get("angular") or joint["drives"].get("linear") or {}
        reading = drive.get("damping")
        in_drive = reading["value"] if reading and reading["authored"] else None
        print(f"  {joint['name']:22} authored as {authored}, in the drive: {in_drive}")

    print(
        "\nA value present under `newton:` and absent from the drive is the asset "
        "carrying the URDF's damping\nwhere the PhysX and MuJoCo paths do not look for it."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
