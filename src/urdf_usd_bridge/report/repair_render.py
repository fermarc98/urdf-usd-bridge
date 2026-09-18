# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Plain-text rendering of a repair report.

Grouped by rule, because the question a reader actually has is "what did each
rule do", not "what happened to each prim". Every table shows the old value next
to the new one, and the ``unmeasured`` banner is printed whether or not anything
was repaired.
"""

from __future__ import annotations

from typing import Any

from .render import _fmt, _section, _table

_STATUS_ORDER = {"applied": 0, "reported": 1, "refused": 2, "skipped": 3}


def _short(value: Any, width: int = 34) -> str:
    text = _fmt(value)
    return text if len(text) <= width else text[: width - 1] + "…"


def _leaf(path: str) -> str:
    return path.rsplit("/", 1)[-1] or path


def render_repair_text(report: dict[str, Any], verbose: bool = False) -> str:
    """Render a repair report for a terminal."""
    lines: list[str] = []
    options = report["options"]
    output = report["output"]
    summary = report["summary"]

    title = "Repair plan (dry run)" if options["dry_run"] else "Repairs applied"
    lines += [title, "=" * len(title)]
    lines.append(f"  input    {report['input']['identifier']}")
    lines.append(f"  layout   {report['input']['layout']}")
    lines.append(f"  backends {', '.join(options['backends'])}")
    lines.append(
        "  layers   "
        + (
            "variant-scoped (per-backend opinions live inside the Physics variant)"
            if output["variant_scoped"]
            else "flat root sublayers (asset has no Physics variant set)"
        )
    )
    if output["written"]:
        roots = output.get("roots") or [output["root"]]
        lines.append(f"  root     {roots[0]}")
        for extra in roots[1:]:
            lines.append(f"           {extra}")
        if output.get("multi_root"):
            lines.append("           (one root per backend: a flat asset cannot carry all three at once)")
        copied = output.get("metadata_copied") or {}
        lines.append(
            "  metadata " + (", ".join(f"{k}={v}" for k, v in sorted(copied.items())) or "(none copied)")
        )

    lines += _section("Tuning")
    for key, value in options["tuning"].items():
        lines.append(f"  {key:24} {_fmt(value)}")
    lines.append("")
    for chunk in _wrap(options["tuning_status"], 78):
        lines.append(f"  ! {chunk}")

    records = report["records"]
    by_rule: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        by_rule.setdefault(record["rule"], []).append(record)

    for rule in sorted(by_rule):
        entries = sorted(
            by_rule[rule],
            key=lambda r: (_STATUS_ORDER.get(r["status"], 9), r["prim"], r["attribute"] or ""),
        )
        applied = sum(1 for e in entries if e["status"] == "applied")
        heading = f"{rule}  ({applied} applied, {len(entries) - applied} other)"
        lines += _section(heading)
        rows = []
        for entry in entries:
            rows.append(
                [
                    entry["status"],
                    _leaf(entry["prim"]),
                    entry["attribute"] or "-",
                    _short(entry["old"]),
                    _short(entry["new"]),
                    entry["units"] or "-",
                    entry["backend"],
                    entry["confidence"] or entry.get("severity") or "-",
                ]
            )
        lines += _table(["status", "prim", "attribute", "old", "new", "units", "backend", "conf"], rows)
        if verbose:
            for entry in entries:
                if not entry.get("reason"):
                    continue
                lines.append(f"    {_leaf(entry['prim'])}.{entry['attribute'] or ''}:")
                for chunk in _wrap(entry["reason"], 74):
                    lines.append(f"      {chunk}")

    lines += _section("Summary")
    lines.append(f"  records          {summary['records_total']}")
    for key in ("by_status", "applied_by_rule", "applied_by_backend", "by_severity"):
        value = summary.get(key) or {}
        if value:
            lines.append(f"  {key:16} " + ", ".join(f"{k}={v}" for k, v in value.items()))
    lines.append(f"  prims touched    {summary['prims_touched']}")
    if summary["errors"]:
        lines.append("")
        lines.append(f"  {summary['errors']} error-severity finding(s) remain; see the records above.")
    return "\n".join(lines)


def _wrap(text: str, width: int) -> list[str]:
    words = text.split()
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = f"{current} {word}".strip()
        if len(candidate) > width and current:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    return lines
