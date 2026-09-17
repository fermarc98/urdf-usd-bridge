# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Command line interface for ``urdf-usd-bridge``.

``inspect``
    Read-only report on a converted asset. Pure ``pxr``; runs anywhere OpenUSD
    Python bindings are installed, including macOS.
``fix``
    Author the stability layer: drives, armature, inertia and joint limits, as
    ``over`` prims in new layers above an untouched input.
``convert``
    Thin front-end over ``urdf-usd-converter``, followed by ``fix`` unless
    ``--no-fix`` is given. Requires the ``[convert]`` extra, which pulls
    ``usd-exchange`` and is therefore Linux/Windows only.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from ._version import __version__
from .repair.base import OptionError
from .usd import UsdUnavailableError, require_pxr, try_register_newton_schemas


class UsageError(ValueError):
    """A bad command line, reported through argparse so the exit code is 2."""


def _parse_variant(values: list[str] | None) -> dict[str, str]:
    """Parse ``--variant Set=Selection`` options."""
    out: dict[str, str] = {}
    for item in values or []:
        if "=" not in item:
            raise UsageError(f"--variant expects SET=SELECTION, got {item!r}")
        name, _, selection = item.partition("=")
        out[name.strip()] = selection.strip()
    return out


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="urdf-usd-bridge",
        description="Stability layer and cross-backend validator for URDF-derived OpenUSD assets.",
    )
    parser.add_argument("--version", action="version", version=f"urdf-usd-bridge {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    inspect_parser = sub.add_parser(
        "inspect",
        help="report what physics data a converted USD asset actually carries",
        description=(
            "Read-only inspection. Reports drives, damping/friction/armature in every "
            "namespace, mass and inertia validity, joint limits, collision filtering, "
            "physics materials, and the Physics variant set."
        ),
    )
    inspect_parser.add_argument("asset", help="path to the asset's root USD layer")
    inspect_parser.add_argument(
        "--variant",
        action="append",
        metavar="SET=SELECTION",
        help="variant selection to apply before reading, e.g. --variant Physics=physx",
    )
    inspect_parser.add_argument("--json", action="store_true", help="emit the raw JSON report")
    inspect_parser.add_argument(
        "-o", "--output", metavar="PATH", help="write the report to PATH instead of stdout"
    )
    inspect_parser.add_argument(
        "-v", "--verbose", action="store_true", help="include the per-namespace joint table"
    )

    fix_parser = sub.add_parser(
        "fix",
        help="author the stability layer over a converted asset",
        description=(
            "Derive drive gains, armature, inertia and joint limits, and author them as "
            "`over` prims in new layers above the input. The input is never modified."
        ),
    )
    fix_parser.add_argument("asset", help="path to the asset's root USD layer")
    fix_parser.add_argument(
        "--out",
        metavar="DIR",
        help="directory for the stability layers (required unless --dry-run)",
    )
    _add_fix_options(fix_parser)

    convert_parser = sub.add_parser(
        "convert",
        help="run urdf-usd-converter, then fix (requires the [convert] extra; Linux/Windows only)",
    )
    convert_parser.add_argument("urdf", help="path to the input .urdf file")
    convert_parser.add_argument("output_dir", help="directory to write the USD asset into")
    convert_parser.add_argument(
        "--flatten",
        action="store_true",
        help="emit a single flattened layer instead of the atomic-component layout",
    )
    convert_parser.add_argument(
        "--no-scene",
        action="store_true",
        help="do not author a UsdPhysics.Scene (what Isaac Sim's importer does)",
    )
    convert_parser.add_argument(
        "--package",
        action="append",
        metavar="NAME=PATH",
        help="ROS package name to path mapping, repeatable",
    )
    convert_parser.add_argument(
        "--no-fix",
        dest="fix",
        action="store_false",
        help="stop after conversion instead of authoring the stability layer",
    )
    convert_parser.set_defaults(fix=True)
    convert_parser.add_argument(
        "--out",
        metavar="DIR",
        help="directory for the stability layers (default: <output_dir>/stability)",
    )
    _add_fix_options(convert_parser)
    return parser


def _add_fix_options(parser: argparse.ArgumentParser) -> None:
    """Options shared by ``fix`` and ``convert``.

    The tuning defaults are **unmeasured** Phase 3 engineering choices. They are
    exposed here, recorded in every report, and written into the output layer's
    custom data, so an asset always carries the assumptions it was built with.
    """
    from .repair.base import ALL_BACKENDS, DEFAULTS

    group = parser.add_argument_group("repair")
    group.add_argument(
        "--backend",
        default=ALL_BACKENDS,
        metavar="NAME",
        help=(
            "physx, mujoco, newton, a comma-separated subset, or all (default). "
            "'all' requires a Physics variant set to keep the per-backend gain "
            "conventions apart, and is refused without one"
        ),
    )
    group.add_argument(
        "--variant",
        action="append",
        metavar="SET=SELECTION",
        help="variant selection to apply before reading, e.g. --variant Physics=physx",
    )
    group.add_argument("--enable", action="append", metavar="RULE", help="turn a rule on, repeatable")
    group.add_argument("--disable", action="append", metavar="RULE", help="turn a rule off, repeatable")
    group.add_argument(
        "--target-frequency",
        type=float,
        default=DEFAULTS.target_frequency,
        metavar="HZ",
        help=f"target drive natural frequency (default {DEFAULTS.target_frequency}, unmeasured)",
    )
    group.add_argument(
        "--damping-ratio",
        type=float,
        default=DEFAULTS.damping_ratio,
        metavar="Z",
        help=f"target damping ratio (default {DEFAULTS.damping_ratio}, unmeasured)",
    )
    group.add_argument(
        "--armature-fraction",
        type=float,
        default=DEFAULTS.armature_fraction,
        metavar="A",
        help=f"armature as a fraction of I_eq (default {DEFAULTS.armature_fraction}, unmeasured)",
    )
    group.add_argument(
        "--armature-floor",
        type=float,
        default=DEFAULTS.armature_floor,
        metavar="B",
        help=f"armature floor as a fraction of max I_eq (default {DEFAULTS.armature_floor}, unmeasured)",
    )
    group.add_argument(
        "--control-rate",
        type=float,
        default=DEFAULTS.control_rate,
        metavar="HZ",
        help=f"assumed control rate, checked but never used in a formula (default {DEFAULTS.control_rate})",
    )
    group.add_argument(
        "--force",
        action="store_true",
        help="overwrite values the input authored and that validate",
    )
    group.add_argument("--dry-run", action="store_true", help="report what would change, write nothing")
    group.add_argument("--json", action="store_true", help="emit the raw JSON report")
    group.add_argument("-o", "--output", metavar="PATH", help="write the report to PATH instead of stdout")
    group.add_argument("-v", "--verbose", action="store_true", help="include each record's full reason")


def _cmd_inspect(args: argparse.Namespace) -> int:
    require_pxr()  # turn a missing OpenUSD into advice before we import anything
    from .inspection import inspect_stage
    from .report import render_text

    try_register_newton_schemas()
    selections = _parse_variant(args.variant)
    report = inspect_stage(args.asset, selections)

    if args.json:
        text = json.dumps(report, indent=2, sort_keys=False)
    else:
        text = render_text(report, verbose=args.verbose)

    if args.output:
        Path(args.output).write_text(text + ("\n" if not text.endswith("\n") else ""))
        print(f"wrote {args.output}", file=sys.stderr)
    else:
        print(text)
    return 0


def _repair_options(args: argparse.Namespace):
    """Build :class:`RepairOptions` from parsed arguments."""
    from .repair.base import RepairOptions, resolve_rules

    return RepairOptions(
        backends_requested=args.backend,
        enabled=resolve_rules(args.enable, args.disable),
        target_frequency=args.target_frequency,
        damping_ratio=args.damping_ratio,
        armature_fraction=args.armature_fraction,
        armature_floor=args.armature_floor,
        control_rate=args.control_rate,
        force=args.force,
        dry_run=args.dry_run,
        variant_selections=_parse_variant(args.variant),
    )


def _emit(text: str, output: str | None) -> None:
    if output:
        Path(output).write_text(text + ("\n" if not text.endswith("\n") else ""))
        print(f"wrote {output}", file=sys.stderr)
    else:
        print(text)


def _run_fix(asset: str, args: argparse.Namespace) -> int:
    """Shared body of ``fix`` and the tail of ``convert``."""
    require_pxr()
    from .repair import fix_asset
    from .report.repair_render import render_repair_text

    try_register_newton_schemas()
    options = _repair_options(args)
    if not options.dry_run and not args.out:
        raise UsageError("--out DIR is required unless --dry-run is given")

    report = fix_asset(asset, args.out, options)
    text = json.dumps(report, indent=2) if args.json else render_repair_text(report, args.verbose)
    _emit(text, args.output)
    # A remaining error-severity finding means the asset still needs a human.
    return 1 if report["summary"]["errors"] else 0


def _cmd_fix(args: argparse.Namespace) -> int:
    return _run_fix(args.asset, args)


def _cmd_convert(args: argparse.Namespace) -> int:
    try:
        import urdf_usd_converter
    except ImportError:
        print(
            "urdf-usd-converter is not installed.\n"
            "    pip install 'urdf-usd-bridge[convert]'\n"
            "It depends on usd-exchange, which publishes no macOS wheels, so this\n"
            "subcommand is Linux/Windows only. See docs/VERIFY.md.",
            file=sys.stderr,
        )
        return 2

    packages = []
    for item in args.package or []:
        name, _, path = item.partition("=")
        if not path:
            print(f"--package expects NAME=PATH, got {item!r}", file=sys.stderr)
            return 2
        packages.append({"name": name.strip(), "path": path.strip()})

    converter = urdf_usd_converter.Converter(
        layer_structure=not args.flatten,
        scene=not args.no_scene,
        ros_packages=packages,
    )
    asset = converter.convert(args.urdf, args.output_dir)
    if not args.fix:
        print(asset.path)
        return 0
    print(f"converted: {asset.path}", file=sys.stderr)

    # The stability layers go next to the converted asset, not inside it, so
    # the converter's own output stays a self-contained, untouched unit.
    if not args.out:
        args.out = str(Path(args.output_dir) / "stability")
    try:
        status = _run_fix(str(asset.path), args)
    except OptionError as exc:
        # The conversion itself succeeded; only the repair pass could not
        # proceed. Say so, and leave the converted asset in place.
        print(f"error: {exc}", file=sys.stderr)
        print(asset.path)
        return 2
    print(str(Path(args.out) / f"{Path(str(asset.path)).stem}_stabilized.usda"))
    return status


def run(argv: list[str]) -> int:
    """Entry point used by the console script."""
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "inspect":
            return _cmd_inspect(args)
        if args.command == "fix":
            return _cmd_fix(args)
        if args.command == "convert":
            return _cmd_convert(args)
    except UsdUnavailableError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except UsageError as exc:
        parser.error(str(exc))
    except OptionError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except (ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    parser.error(f"unknown command {args.command!r}")
    return 2
