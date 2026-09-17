# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Command line interface for ``urdf-usd-bridge``.

Phase 2 provides:

``inspect``
    Read-only report on a converted asset. Pure ``pxr``; runs anywhere OpenUSD
    Python bindings are installed, including macOS.
``convert``
    Thin front-end over ``urdf-usd-converter``. Requires the ``[convert]``
    extra, which pulls ``usd-exchange`` and is therefore Linux/Windows only.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from ._version import __version__
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

    convert_parser = sub.add_parser(
        "convert",
        help="run urdf-usd-converter (requires the [convert] extra; Linux/Windows only)",
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
    return parser


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
    print(asset.path)
    return 0


def run(argv: list[str]) -> int:
    """Entry point used by the console script."""
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "inspect":
            return _cmd_inspect(args)
        if args.command == "convert":
            return _cmd_convert(args)
    except UsdUnavailableError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except UsageError as exc:
        parser.error(str(exc))
    except (ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    parser.error(f"unknown command {args.command!r}")
    return 2
