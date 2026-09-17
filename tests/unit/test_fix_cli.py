# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The ``fix`` command line: arguments, output and exit codes."""

from __future__ import annotations

import json

import pytest

from urdf_usd_bridge.cli import run

from .repair_builders import add_joint, add_link, export, new_asset, simple_arm


@pytest.fixture
def arm(tmp_path):
    return export(simple_arm(), tmp_path / "robot.usda")


def test_fix_writes_layers_and_prints_a_table(arm, tmp_path, capsys):
    out = tmp_path / "out"
    assert run(["fix", arm, "--out", str(out), "--backend", "physx"]) == 0

    assert (out / "robot_stabilized.usda").exists()
    assert (out / "Stability.usda").exists()
    assert (out / "Stability_physx.usda").exists()

    printed = capsys.readouterr().out
    assert "Repairs applied" in printed
    assert "drives.derive-gains" in printed
    assert "unmeasured" in printed


def test_dry_run_writes_nothing(arm, tmp_path, capsys):
    out = tmp_path / "out"
    assert run(["fix", arm, "--out", str(out), "--backend", "physx", "--dry-run"]) == 0
    assert not out.exists()
    assert "Repair plan (dry run)" in capsys.readouterr().out


def test_dry_run_needs_no_out_directory(arm, capsys):
    assert run(["fix", arm, "--backend", "physx", "--dry-run"]) == 0
    assert "Repair plan (dry run)" in capsys.readouterr().out


def test_out_is_required_without_dry_run(arm):
    with pytest.raises(SystemExit):
        run(["fix", arm, "--backend", "physx"])


def test_json_report_round_trips(arm, tmp_path, capsys):
    out = tmp_path / "out"
    assert run(["fix", arm, "--out", str(out), "--backend", "physx", "--json"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["schema_version"] == 1
    assert report["options"]["backends"] == ["physx"]
    assert "unmeasured" in report["options"]["tuning_status"]
    assert any(r["rule"] == "drives.derive-gains" for r in report["records"])


def test_report_can_be_written_to_a_file(arm, tmp_path):
    out = tmp_path / "out"
    report_path = tmp_path / "report.json"
    assert run(["fix", arm, "--out", str(out), "--backend", "physx", "--json", "-o", str(report_path)]) == 0
    assert json.loads(report_path.read_text())["summary"]["records_total"] > 0


def test_backend_all_on_a_flat_asset_exits_two(arm, tmp_path, capsys):
    """Decision D2, surfaced through the CLI with an actionable message."""
    assert run(["fix", arm, "--out", str(tmp_path / "out"), "--backend", "all"]) == 2
    assert "--backend physx" in capsys.readouterr().err


def test_an_unknown_backend_is_rejected(arm, tmp_path, capsys):
    assert run(["fix", arm, "--out", str(tmp_path / "out"), "--backend", "bullet"]) == 2
    assert "unknown backend" in capsys.readouterr().err


def test_an_unknown_rule_name_is_rejected(arm, tmp_path, capsys):
    assert run(["fix", arm, "--out", str(tmp_path / "out"), "--disable", "drives.typo"]) == 2
    assert "unknown rule" in capsys.readouterr().err


def test_tuning_flags_reach_the_report(arm, tmp_path, capsys):
    out = tmp_path / "out"
    assert (
        run(
            [
                "fix",
                arm,
                "--out",
                str(out),
                "--backend",
                "physx",
                "--target-frequency",
                "4.0",
                "--damping-ratio",
                "0.7",
                "--armature-fraction",
                "0.05",
                "--json",
            ]
        )
        == 0
    )
    tuning = json.loads(capsys.readouterr().out)["options"]["tuning"]
    assert tuning["target_frequency_hz"] == pytest.approx(4.0)
    assert tuning["damping_ratio"] == pytest.approx(0.7)
    assert tuning["armature_fraction"] == pytest.approx(0.05)


def test_an_error_severity_finding_exits_one(tmp_path, capsys):
    """A body with no mass and no density still needs a human."""
    stage = new_asset()
    add_link(stage, "/robot/Geometry/base", mass=1.0, diagonal_inertia=(1, 1, 1))
    add_link(stage, "/robot/Geometry/base/link", box=(0.1, 0.1, 0.1))
    add_joint(stage, "/robot/Physics/j", "/robot/Geometry/base", "/robot/Geometry/base/link")
    source = export(stage, tmp_path / "robot.usda")

    assert run(["fix", source, "--out", str(tmp_path / "out"), "--backend", "physx"]) == 1
    assert "error-severity" in capsys.readouterr().out


def test_verbose_prints_the_reasons(arm, tmp_path, capsys):
    out = tmp_path / "out"
    assert run(["fix", arm, "--out", str(out), "--backend", "physx", "-v"]) == 0
    printed = capsys.readouterr().out
    # Reasons are wrapped, so assert on fragments rather than whole sentences.
    assert "drive:angular:physics:damping:" in printed
    assert "derived drive damping plus the URDF's passive damping" in printed

    quiet = tmp_path / "quiet"
    assert run(["fix", arm, "--out", str(quiet), "--backend", "physx"]) == 0
    assert "derived drive damping plus" not in capsys.readouterr().out
