# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Plain-text rendering of an inspection report. No third-party dependencies."""

from __future__ import annotations

from typing import Any

_DASH = "-"


def _fmt(value: Any, places: int = 6) -> str:
    if value is None:
        return _DASH
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        if value == int(value) and abs(value) < 1e15:
            return str(int(value))
        return f"{value:.{places}g}"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_fmt(v, places) for v in value) + "]"
    return str(value)


def _authored(reading: dict[str, Any] | None) -> Any:
    if reading and reading.get("authored"):
        return reading.get("value")
    return None


def _reading_cell(reading: dict[str, Any] | None) -> str:
    """Render absent / fallback / authored distinctly -- the distinction is the point."""
    if reading is None:
        return _DASH
    if not reading.get("authored"):
        return f"({_fmt(reading.get('value'))})"
    return _fmt(reading.get("value"))


def _table(headers: list[str], rows: list[list[str]], indent: str = "  ") -> list[str]:
    if not rows:
        return [f"{indent}(none)"]
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))
    out = [
        indent + "  ".join(h.ljust(widths[i]) for i, h in enumerate(headers)).rstrip(),
        indent + "  ".join("-" * widths[i] for i in range(len(headers))),
    ]
    for row in rows:
        out.append(indent + "  ".join(row[i].ljust(widths[i]) for i in range(len(headers))).rstrip())
    return out


def _section(title: str) -> list[str]:
    return ["", title, "=" * len(title)]


def render_text(report: dict[str, Any], verbose: bool = False) -> str:
    """Render an inspection report as a human-readable table."""
    lines: list[str] = []
    tool = report["tool"]
    stage = report["stage"]
    summary = report["summary"]

    lines.append(f"{tool['name']} {tool['version']}  (OpenUSD {tool['usd_version']})")
    lines.append(f"asset   : {report['input']['identifier']}")
    lines.append(f"layout  : {stage['layout']}")
    lines.append(f"root    : {stage['default_prim']}")
    metrics = stage["metrics"]
    lines.append(
        "metrics : upAxis={up}  metersPerUnit={mpu}  kilogramsPerUnit={kpu}".format(
            up=_fmt(metrics["up_axis"]),
            mpu=_fmt(metrics["meters_per_unit"]),
            kpu=_fmt(metrics["kilograms_per_unit"]),
        )
    )
    if report["input"]["variant_selections"]:
        lines.append(f"variants: {report['input']['variant_selections']}")

    # -- variant sets --------------------------------------------------------
    lines += _section("Variant sets")
    rows = [
        [
            vs["name"],
            ", ".join(vs["variants"]) or _DASH,
            vs["selection"] or "(none authored)",
        ]
        for vs in report["variant_sets"]
    ]
    lines += _table(["set", "variants", "selection"], rows)
    physics_vs = summary.get("physics_variant_set")
    if physics_vs and not physics_vs["has_authored_selection"]:
        lines.append("  note: no 'Physics' selection is authored, so no physics layer composes")
        lines.append("        until a consumer selects one (Isaac Sim 6.1.0 behaviour).")

    # -- joints --------------------------------------------------------------
    lines += _section(f"Joints ({summary['joints_total']}, {summary['joints_actuatable']} actuatable)")
    rows = []
    for joint in report["joints"]:
        drive_cells = []
        for instance, drive in joint["drives"].items():
            k = _authored(drive.get("stiffness"))
            d = _authored(drive.get("damping"))
            f = _authored(drive.get("maxForce"))
            drive_cells.append(f"{instance}:k={_fmt(k)},d={_fmt(d)},F={_fmt(f)}")
        notes = []
        flags = joint["flags"]
        if flags["locked_by_equal_limits"]:
            notes.append("LOCKED")
        if flags["drive_applied_without_gains"]:
            notes.append("no-gains")
        if flags["damping_stranded_outside_drive"]:
            notes.append("damping-stranded")
        if flags["no_armature_anywhere"] and flags["actuatable"]:
            notes.append("no-armature")
        if flags["effort_only_as_urdf_custom"]:
            notes.append("effort-unused")
        rows.append(
            [
                joint["name"],
                joint["type_name"].replace("Physics", ""),
                f"{_fmt(_authored(joint['limits']['lower']))}..{_fmt(_authored(joint['limits']['upper']))}",
                "; ".join(drive_cells) or _DASH,
                ",".join(flags["damping_namespaces"]) or _DASH,
                ",".join(flags["friction_namespaces"]) or _DASH,
                ",".join(flags["armature_namespaces"]) or _DASH,
                " ".join(notes) or "",
            ]
        )
    lines += _table(["joint", "type", "limits", "drive", "damping@", "friction@", "armature@", "notes"], rows)

    if verbose:
        lines.append("")
        lines.append("  Per-namespace joint dynamics  (value, (fallback), - absent)")
        rows = []
        for joint in report["joints"]:
            if not joint["flags"]["actuatable"]:
                continue
            row = [joint["name"]]
            for group in ("damping_by_namespace", "friction_by_namespace", "armature_by_namespace"):
                for reading in joint[group].values():
                    row.append(_reading_cell(reading))
            row.append(_reading_cell(joint["limit_extras"]["urdf_effort"]))
            row.append(_reading_cell(joint["limit_extras"]["newton_velocity_limit"]))
            rows.append(row)
        headers = ["joint"]
        example = next((j for j in report["joints"] if j["flags"]["actuatable"]), None)
        if example:
            headers += [f"d:{k}" for k in example["damping_by_namespace"]]
            headers += [f"f:{k}" for k in example["friction_by_namespace"]]
            headers += [f"a:{k}" for k in example["armature_by_namespace"]]
            headers += ["urdf:effort", "newton:velLimit"]
            lines += _table(headers, rows)

    # -- bodies --------------------------------------------------------------
    lines += _section(f"Bodies ({summary['bodies_total']})")
    rows = []
    for body in report["bodies"]:
        checks = body["inertia_checks"]
        axes = body["principal_axes_validity"]
        notes = []
        if checks["zero_inertia_with_mass"]:
            notes.append("ZERO-INERTIA")
        if checks["mass_without_authored_inertia"]:
            notes.append("UNDEFINED-INERTIA")
        if checks["negative_diagonal_inertia"]:
            notes.append("NEGATIVE-INERTIA")
        if checks["diagonal_triangle_inequality_violated"]:
            notes.append("TRIANGLE-VIOLATION")
        if checks["newton_inertia_not_positive_semidefinite"]:
            notes.append("NEWTON-NOT-PSD")
        if checks["newton_inertia_triangle_inequality_violated"]:
            notes.append("NEWTON-TRIANGLE-VIOLATION")
        if checks["newton_inertia_disagrees_with_diagonal"]:
            notes.append("NEWTON/USD-MISMATCH")
        if axes["present"] and axes["is_zero"]:
            notes.append("ZERO-QUATERNION")
        rows.append(
            [
                body["name"],
                _reading_cell(body["mass"]["mass"]),
                _reading_cell(body["mass"]["diagonal_inertia"]),
                _reading_cell(body["mass"]["principal_axes"]),
                _reading_cell(body["newton_inertia"]),
                " ".join(notes) or "",
            ]
        )
    lines += _table(
        ["body", "mass", "physics:diagonalInertia", "physics:principalAxes", "newton:inertia", "notes"],
        rows,
    )

    # -- collision -----------------------------------------------------------
    lines += _section(f"Collision ({summary['colliders_total']} colliders)")
    lines.append("  approximations:")
    for key, count in sorted(summary["collider_approximations"].items()):
        lines.append(f"    {key}: {count}")
    lines.append(f"  colliders with physics:filteredPairs : {summary['colliders_with_filtered_pairs']}")
    lines.append(f"  PhysicsCollisionGroup prims          : {summary['collision_groups_total']}")
    lines.append(f"  colliders bound to a physics material: {summary['colliders_with_physics_material']}")
    lines.append(f"  PhysicsMaterialAPI prims             : {summary['physics_materials_total']}")

    # -- articulation / scene ------------------------------------------------
    lines += _section("Articulation and scene")
    rows = []
    for root in report["articulation_roots"]:
        rows.append(
            [
                root["path"],
                _reading_cell(root["self_collision"]["newton"]),
                _reading_cell(root["self_collision"]["physx"]),
            ]
        )
    lines += _table(["articulation root", "newton:selfCollision", "physx:selfCollisions"], rows)
    lines.append(f"  PhysicsScene prims: {summary['physics_scenes_total']}")

    # -- mujoco actuators ----------------------------------------------------
    lines += _section(f"MuJoCo actuators ({summary['mjc_actuators_total']})")
    rows = []
    for actuator in report["mjc_actuators"]:
        rows.append(
            [
                actuator["path"].rsplit("/", 1)[-1],
                ", ".join(actuator["target"] or []) or _DASH,
                _reading_cell(actuator["attrs"].get("mjc:gainType")),
                _reading_cell(actuator["attrs"].get("mjc:gainPrm")),
                _reading_cell(actuator["attrs"].get("mjc:biasPrm")),
                "" if actuator["flags"]["has_gain_parameters"] else "NO-GAINS",
            ]
        )
    lines += _table(["actuator", "target", "gainType", "gainPrm", "biasPrm", "notes"], rows)

    # -- summary -------------------------------------------------------------
    lines += _section("Summary")
    ordered = [
        ("bodies", "bodies_total"),
        ("  with authored mass", "bodies_with_authored_mass"),
        ("  zero inertia despite mass", "bodies_zero_inertia_with_mass"),
        ("  mass but no inertia authored", "bodies_mass_without_authored_inertia"),
        ("  negative inertia", "bodies_negative_inertia"),
        ("  triangle-inequality violations", "bodies_triangle_inequality_violated"),
        ("  zero principalAxes quaternion", "bodies_invalid_principal_axes"),
        ("  max mass ratio", "mass_ratio_max"),
        ("joints (actuatable)", "joints_actuatable"),
        ("  with DriveAPI applied", "joints_with_drive_api"),
        ("  with non-zero drive gains", "joints_with_drive_gains"),
        ("  DriveAPI applied but no gains", "joints_drive_applied_without_gains"),
        ("  damping authored outside the drive", "joints_damping_stranded_outside_drive"),
        ("  without armature in any namespace", "joints_without_armature"),
        ("  locked by lower == upper", "joints_locked_by_equal_limits"),
        ("  effort only as urdf: custom attr", "joints_effort_only_as_urdf_custom"),
        ("MjcActuators", "mjc_actuators_total"),
        ("  without gain parameters", "mjc_actuators_without_gains"),
    ]
    rows = [[label, _fmt(summary.get(key))] for label, key in ordered]
    lines += _table(["metric", "value"], rows)

    for label, key in (
        ("damping authored by namespace", "damping_authored_by_namespace"),
        ("friction authored by namespace", "friction_authored_by_namespace"),
        ("armature authored by namespace", "armature_authored_by_namespace"),
    ):
        value = summary.get(key) or {}
        lines.append(f"  {label}: " + (", ".join(f"{k}={v}" for k, v in sorted(value.items())) or "(none)"))

    lines.append("")
    return "\n".join(lines)
