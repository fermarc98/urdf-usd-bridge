#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Convert a URDF, repair it, and prove the gaps closed.

The proof is the point: rather than trusting the repair report, this reruns
``inspect`` on the result and compares the counters.

Needs the ``[convert]`` extra, so Linux or Windows.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from urdf_usd_bridge.inspection import inspect_stage  # noqa: E402
from urdf_usd_bridge.repair import RepairOptions, fix_asset  # noqa: E402

COUNTERS = [
    ("joints_with_drive_gains", "joints with drive gains", "up"),
    ("joints_drive_applied_without_gains", "drives applied but empty", "down"),
    ("joints_damping_stranded_outside_drive", "damping stranded outside the drive", "down"),
    ("joints_without_armature", "joints with no armature", "down"),
    ("bodies_invalid_principal_axes", "bodies with a zero quaternion", "down"),
    ("bodies_mass_without_authored_inertia", "bodies with mass but no inertia", "down"),
    ("joints_locked_by_equal_limits", "joints welded at [0, 0]", "down"),
]


def main() -> int:
    urdf = sys.argv[1] if len(sys.argv) > 1 else str(REPO / "tests/fixtures/a_dynamics_damping.urdf")
    try:
        import urdf_usd_converter
    except ImportError:
        print(
            "this example needs the converter:\n"
            "    pip install 'urdf-usd-bridge[convert]'   # Linux/Windows only",
            file=sys.stderr,
        )
        return 2

    work = Path(tempfile.mkdtemp(prefix="urdf-usd-bridge-example-"))
    asset = urdf_usd_converter.Converter().convert(urdf, str(work / "converted"))
    before = inspect_stage(str(asset.path))

    report = fix_asset(str(asset.path), work / "stability", RepairOptions(backends_requested="physx"))
    after = inspect_stage(report["output"]["roots"][0])

    print(f"urdf:      {urdf}")
    print(f"converted: {asset.path}")
    print(f"repaired:  {report['output']['roots'][0]}\n")
    print(f"{'counter':38} {'before':>8} {'after':>8}   want")
    print("-" * 68)
    for key, label, direction in COUNTERS:
        b, a = before["summary"][key], after["summary"][key]
        mark = "ok" if (a > b if direction == "up" else a <= b) else "REGRESSED"
        print(f"{label:38} {b:>8} {a:>8}   {direction:4} {mark}")

    tuning = report["options"]["tuning"]
    print(
        f"\ngains derived for f_n = {tuning['target_frequency_hz']:g} Hz, "
        f"zeta = {tuning['damping_ratio']:g}"
    )
    print(f"  basis: {report['options']['tuning_basis']}")
    print(f"  {report['options']['tuning_status']}")
    print(
        f"\nthe input is untouched; mute {Path(report['output']['layers']['neutral']).name} "
        "and friends to get it back"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
